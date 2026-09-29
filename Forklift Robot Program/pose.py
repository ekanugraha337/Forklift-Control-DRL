"""
pose.py - sumber pose robot dalam cm.

Dua sumber dengan antarmuka sama, jadi navigasi.py tidak perlu tahu bedanya:
    SumberKamera    - AprilTag lewat kamera ONVIF (perangkat nyata)
    SumberSimulasi  - model sepeda yang diintegrasikan dari perintah rc

TITIK ACUAN. Kamera melihat PUSAT TAG, sedangkan perencana dan pelacak bekerja
pada POROS RODA DEPAN. Roda depan tidak berbelok, sehingga robot selalu bergerak
searah heading di titik itu - itulah yang membuat model sepeda berlaku untuk
kendaraan berkemudi belakang. Konversinya ada di ke_poros_depan().

Uji mandiri:
    python pose.py                  -> pose live dari kamera
    python pose.py --simulasi       -> robot digerakkan tombol panah
"""
import argparse
import math
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

import konfig

import state_space as SS

konfig.pasang_path_kamera()
import area_constraint as AC          # noqa: E402
import cctv_lurus as C                # noqa: E402


@dataclass
class Pose:
    x: float            # cm, POROS DEPAN
    y: float            # cm
    yaw: float          # derajat
    t: float            # detik (time.time) saat frame diambil
    terlihat: bool      # pose masih layak dipakai
    # True bila pose ini hasil RAMALAN model, bukan pengukuran kamera. Navigasi
    # boleh memakainya sebentar untuk menjembatani tag yang kabur; identifikasi
    # TIDAK boleh - pose ramalan bukan pengukuran, dan merekamnya sebagai data
    # berarti mengukur model sendiri, bukan robot.
    diramalkan: bool = False

    @property
    def xy(self):
        return (self.x, self.y)

    def tuple(self):
        return (self.x, self.y, self.yaw)


def ke_poros_depan(x_tag, y_tag, yaw_deg):
    """Pusat tag -> poros roda depan."""
    a = math.radians(yaw_deg)
    d = konfig.TAG_TO_FRONT_AXLE_CM
    return (x_tag + d * math.cos(a), y_tag + d * math.sin(a), yaw_deg)


def ke_pusat_tag(x_poros, y_poros, yaw_deg):
    """Poros roda depan -> pusat tag (kebalikan, untuk menggambar)."""
    a = math.radians(yaw_deg)
    d = konfig.TAG_TO_FRONT_AXLE_CM
    return (x_poros - d * math.cos(a), y_poros - d * math.sin(a), yaw_deg)


class KameraBersama:
    """Satu aliran RTSP + satu detektor untuk SEMUA robot di arena.

    Dua robot dengan SumberKamera masing-masing berarti dua aliran RTSP: dekode
    dua kali lipat, kamera ONVIF bisa menolak sesi kedua, dan pose kedua robot
    berasal dari frame yang BERBEDA - padahal wasit anti-tabrakan membandingkan
    keduanya seolah diambil bersamaan. Di sini satu frame dideteksi sekali, dan
    setiap SumberKamera mengambil tag miliknya dari hasil yang sama.
    """

    def __init__(self, ctx, url=None, family=None):
        self.det = AC.Detektor(ctx, family or konfig.TAG_FAMILY)
        self.reader = C.StreamReader(url or C.rtsp_url())
        self.frame = None
        self.fid = 0                 # nomor frame yang sudah dideteksi
        self.deteksi = []            # [(tag_id, center, yaw), ...] frame itu
        self.waktu_frame = time.time()
        self._mulai = False

    def mulai(self):
        if self._mulai:
            return True
        f = self.reader.wait_first_frame()
        if f is None:
            raise SystemExit("[!] Tidak ada frame dari kamera. Cek RTSP dan kartu jaringan.")
        self.frame = f
        self.waktu_frame = time.time()
        self._mulai = True
        return True

    def perbarui(self):
        """Deteksi frame terbaru bila ada yang baru. Aman dipanggil berkali-kali."""
        frame, fid = self.reader.read_new(self.fid)
        if frame is None:
            return
        self.frame, self.fid = frame, fid
        self.waktu_frame = time.time()
        self.deteksi = self.det.deteksi(frame)

    def berhenti(self):
        self.reader.stop()


class SumberKamera:
    """AprilTag -> pose cm. Membungkus StreamReader + Detektor dari alur cctv_lurus.

    profil: konfig.profil() robot ini (tag, cm/langkah, akselerasi untuk
    meramal). None = robot yang sedang dipilih.
    kamera: KameraBersama bila beberapa robot berbagi satu kamera. None =
    sumber ini membuka aliran RTSP-nya sendiri, seperti sebelumnya.
    """

    def __init__(self, ctx, url=None, family=None, tag_id=None, profil=None, kamera=None):
        self.ctx = ctx
        self.p = profil or konfig.profil()
        self.tag_id = self.p.tag if tag_id is None else tag_id
        self.kamera = kamera
        if kamera is None:
            self.det = AC.Detektor(ctx, family or konfig.TAG_FAMILY)
            self.reader = C.StreamReader(url or C.rtsp_url())
        self.frame = None
        self._last_id = 0

        # Diinisialisasi time.time(), BUKAN 0.0. Di v1 nilai awal 0.0 membuat uji
        # "frame basi" selalu benar pada iterasi pertama - sebelum frame pertama
        # sempat tiba - sehingga estop terkirim tiap kali program start dan rute
        # pertama tiap sesi selalu berjalan di atas stepper yang sudah dimatikan.
        self.waktu_frame = time.time()
        self.waktu_tag = 0.0
        self.pose = None

        # Keadaan untuk meramal pose saat tag tidak terdeteksi:
        # [x, y, theta(rad), delta(rad), v(cm/detik)]
        self._xi = None
        self._t_ramal = 0.0
        self._cmd = (0.0, 0.0)      # (v_cm_s, steer_deg) yang sedang dikirim
        # Kecepatan yang ditaksir BENAR-BENAR dicapai motor - bukan yang
        # diperintahkan. Dilacak di perintah(), dipakai di _mulai_ramal().
        self._v = 0.0
        self._t_cmd = None

    def perintah(self, speed_sps, steer_deg):
        """Beri tahu perintah yang sedang dikirim ke robot.

        Diperlukan untuk meramal pose saat tag hilang. Dilewatkan dari pemanggil,
        bukan dibaca diam-diam dari robot.py, supaya jelas bahwa ramalan ini
        memakai PERINTAH - bukan pengukuran.

        Sekaligus melacak kecepatan yang benar-benar dicapai motor. Stepper
        MENANJAK mengikuti ramp akselerasi, tidak melompat ke perintah, dan robot
        ini tidak punya encoder yang bisa memberi tahu sudah sampai mana - jadi
        satu-satunya cara mengetahuinya adalah mengintegrasikan ramp itu sendiri
        dari aliran perintah. navigasi.py memanggil ini tiap siklus kendali, di
        kedua cabang (melacak maupun diam), jadi pelacaknya tidak pernah lapar.

        Diukur terhadap cusp.csv (validasi_nyata.py): tanpa ini galat ramalan
        dalam jendela RAMAL_MAKS_S mencapai 9,41 cm, melebihi SAFETY_MARGIN_CM,
        karena model mengira motor sudah cepat padahal masih menanjak. Dengan
        ini turun ke 6,60 cm dan tidak ada lagi cuplikan yang melewati margin.

        Satu hal yang TIDAK dimodelkan: setelah estop, arus motor diputus dan
        robot meluncur bebas (SIM_LUNCUR_DESEL_CM_S2), bukan melambat mengikuti
        ramp. Pada setelan sekarang keduanya nyaris sama (30 vs 29,9 cm/detik^2)
        - kebetulan, bukan rancangan. Kalau ACCEL_SPS2 diubah, selisihnya muncul.
        """
        now = time.time()
        if self._t_cmd is not None:
            # Selama selang yang baru lewat, motor mengejar perintah SEBELUMNYA;
            # yang baru ini baru saja dikirim. Menanjak ke perintah baru di sini
            # berarti mendahului motor satu siklus (zero-order hold terbalik).
            #
            # dt dibatasi: kalau loop kendali sempat tersendat, satu langkah
            # raksasa akan melompatkan taksiran langsung ke nilai perintah -
            # dan itu persis kekeliruan yang sedang diperbaiki di sini.
            dt = min(now - self._t_cmd, 0.2)
            a = self.p.accel_sps2 * self.p.cm_per_step
            self._v += max(-a * dt, min(a * dt, self._cmd[0] - self._v))
        self._t_cmd = now
        self._cmd = (float(speed_sps) * self.p.cm_per_step, float(steer_deg))

    def _mulai_ramal(self, pose):
        a = math.radians(pose.yaw)
        # Sudut kemudi tidak terukur di mana pun, jadi ramalan dimulai dari sudut
        # yang terakhir diperintahkan. Jeda kemudi diurus SS.turunan().
        #
        # Kecepatan TIDAK diambil dari perintah (self._cmd[0]) seperti dulu,
        # melainkan dari taksiran ramp di perintah(). Menganggap motor sudah
        # sampai di kecepatan perintah membuat ramalan melesat terlalu jauh
        # setiap kali perintah baru berubah - lihat docstring perintah().
        self._xi = np.array([pose.x, pose.y, a, math.radians(self._cmd[1]), self._v])
        self._t_ramal = time.time()

    def _ramal(self):
        """Majukan pose ramalan sampai sekarang. None bila belum ada acuan."""
        if self._xi is None:
            return None
        now = time.time()
        sisa = now - self._t_ramal
        self._t_ramal = now
        u = np.array([math.radians(self._cmd[1]), self._cmd[0]])
        h = 0.01
        while sisa > 1e-9:
            langkah = min(h, sisa)
            self._xi = self._xi + SS.turunan(
                self._xi, u, accel_sps2=self.p.accel_sps2,
                cm_per_step=self.p.cm_per_step, max_steer=self.p.max_steer_rad) * langkah
            sisa -= langkah
        return Pose(float(self._xi[0]), float(self._xi[1]),
                    math.degrees(float(self._xi[2])), now, True, diramalkan=True)

    def mulai(self, timeout=10.0):
        if self.kamera is not None:
            self.kamera.mulai()
            self.frame = self.kamera.frame
            self.waktu_frame = time.time()
            return True
        f = self.reader.wait_first_frame()
        if f is None:
            raise SystemExit("[!] Tidak ada frame dari kamera. Cek RTSP dan kartu jaringan.")
        self.frame = f
        self.waktu_frame = time.time()
        return True

    def _frame_baru(self):
        """(frame, deteksi) bila ada frame yang belum dibaca sumber ini, atau (None, None)."""
        if self.kamera is not None:
            self.kamera.perbarui()
            if self.kamera.fid == self._last_id:
                return None, None
            self._last_id = self.kamera.fid
            return self.kamera.frame, self.kamera.deteksi
        frame, fid = self.reader.read_new(self._last_id)
        if frame is None:
            return None, None
        self._last_id = fid
        return frame, self.det.deteksi(frame)

    def baca(self):
        """Pose terkini: hasil ukur bila tag terlihat, ramalan bila tidak.

        Pose lama TIDAK pernah dikembalikan apa adanya. Kamera satu-satunya
        umpan balik posisi di robot ini, jadi pose yang membeku berarti
        pengendali mengemudi berdasarkan tempat robot beberapa ratus milidetik
        yang lalu - pada 31 cm/detik itu puluhan sentimeter.
        """
        frame, deteksi = self._frame_baru()
        terukur = False
        if frame is not None:
            self.frame = frame
            self.waktu_frame = time.time()
            for tid, center, yaw in deteksi:
                if tid != self.tag_id:
                    continue
                x, y, yw = ke_poros_depan(float(center[0]), float(center[1]), yaw)
                self.pose = Pose(x, y, yw, self.waktu_frame, True)
                self.waktu_tag = self.waktu_frame
                self._mulai_ramal(self.pose)
                terukur = True
                break

        if terukur or self.pose is None:
            return self.pose

        umur = time.time() - self.waktu_tag
        if umur > konfig.RAMAL_MAKS_S:
            # Terlalu lama untuk diramal. Pose ditandai tidak sah supaya
            # pemanggil menghentikan robot - bukan diteruskan diam-diam.
            self.pose.terlihat = False
            self.pose.diramalkan = True
            return self.pose

        ramalan = self._ramal()
        if ramalan is not None:
            self.pose = ramalan
        return self.pose

    @property
    def frame_basi(self):
        return time.time() - self.waktu_frame > konfig.FRAME_BASI_S

    def berhenti(self):
        if self.kamera is not None:
            self.kamera.berhenti()
        else:
            self.reader.stop()


class SumberSimulasi:
    """Model sepeda yang diintegrasikan dari perintah rc. Tanpa kamera, tanpa robot.

    Gerak dasarnya sama persis dengan yang dipakai perencana
    (_integrate_segment di forklift_planner.py):
        x' = v cos(yaw),  y' = v sin(yaw),  yaw' = v tan(delta) / L

    Sudut kemudi memakai konvensi LOGIS (seperti kendaraan berkemudi depan).
    Pembalikan untuk roda belakang dilakukan firmware lewat str_inv, jadi sisi
    PC tidak perlu mengurusnya.

    TIGA JEDA DIMODELKAN, karena tanpanya simulasi terlalu bersahabat dan
    pertanyaan soal kompensasi jeda tidak bisa dijawab sama sekali:

      kemudi  tunda murni -> orde-1 -> batas laju servo   (total ~0,65 detik)
      kamera  fps terbatas + latensi pemrosesan           (usia frame berayun)
      gas     ramp akselerasi stepper                     (tidak melompat)

    Matikan semuanya dengan jeda=False untuk membandingkan.

    profil: konfig.profil() robot yang disimulasikan (cm/langkah, akselerasi,
    batas kemudi). None = robot yang sedang dipilih.
    """

    def __init__(self, x, y, yaw, derau_cm=0.0, derau_deg=0.0, dt_tetap=None,
                 jeda=True, profil=None):
        self.p = profil or konfig.profil()
        self.x, self.y, self.yaw = x, y, yaw
        self.derau_cm, self.derau_deg = derau_cm, derau_deg
        # dt_tetap membuat simulasi berjalan lebih cepat dari waktu nyata,
        # dipakai pengujian otomatis. None = ikut jam dinding.
        self.dt_tetap = dt_tetap
        self.t = time.time()
        self.waktu = 0.0                 # jam simulasi, lepas dari jam dinding
        self.frame_basi = False

        self.jeda = jeda
        self.tunda = konfig.SIM_KEMUDI_TUNDA_S if jeda else 0.0
        self.tau = konfig.SIM_KEMUDI_TAU_S if jeda else 0.0
        self.slew = konfig.SIM_SERVO_SLEW_DEG_S if jeda else float("inf")
        self.fps = konfig.SIM_KAMERA_FPS if jeda else 0.0
        self.kamera_delay = konfig.SIM_KAMERA_DELAY_S if jeda else 0.0
        self.kamera_jitter = konfig.SIM_KAMERA_JITTER_S if jeda else 0.0
        self.accel = self.p.accel_sps2 if jeda else float("inf")

        # yang DIPERINTAHKAN vs yang BENAR-BENAR terjadi
        self.speed_cmd = 0.0
        self.speed_sps = 0.0
        self.steer_cmd = 0.0
        self.steer_deg = 0.0
        # True setelah estop: driver tanpa arus, robot meluncur bebas
        self.bebas = False

        self._antre = deque()            # (waktu_berlaku, sudut) untuk tunda murni
        self._frames = deque(maxlen=128)  # (waktu_ambil, x, y, yaw)
        self._kamera_t = -1e9

    def mulai(self):
        self.t = time.time()
        return True

    def perintah(self, speed_sps, steer_deg):
        self.speed_cmd = float(speed_sps)
        self.steer_cmd = float(steer_deg)
        # Perintah rc bukan-nol menyalakan driver lagi - persis seperti
        # enableOutputs() di driveRc() firmware. Perintah nol TIDAK
        # menyalakannya, jadi luncuran bebas berlanjut selama jeda titik ganti
        # arah, saat pelacak memang mengirim kecepatan nol.
        if self.speed_cmd:
            self.bebas = False

    def henti_mendadak(self):
        """Padanan estop firmware: forceStop() LALU disableOutputs().

        Yang penting justru disableOutputs(): arus ke motor diputus, dan
        stepper tanpa arus tidak punya torsi penahan. Robot MELUNCUR BEBAS -
        bukan berhenti seketika.

        Versi sebelumnya menolkan kecepatan nyata di sini, memodelkan rem
        sempurna. Itu kebalikan dari yang terjadi, dan itulah sebabnya simulasi
        dulu menyimpulkan estop di titik ganti arah memperbaiki keadaan.
        """
        self.speed_cmd = 0.0
        self.bebas = True

    # ------------------------------------------------------------------ jeda
    def _kemudi(self, dt):
        """Sudut kemudi nyata: tunda murni -> orde-1 -> batas laju servo."""
        if not self.jeda:
            self.steer_deg = self.steer_cmd
            return
        self._antre.append((self.waktu + self.tunda, self.steer_cmd))
        target = self.steer_deg
        while self._antre and self._antre[0][0] <= self.waktu:
            target = self._antre.popleft()[1]
        selisih = target - self.steer_deg
        gerak = selisih * min(1.0, dt / self.tau) if self.tau > 0 else selisih
        batas = self.slew * dt
        self.steer_deg += max(-batas, min(batas, gerak))

    def _gas(self, dt):
        """Kecepatan nyata mengikuti ramp akselerasi, tidak melompat."""
        if self.bebas:
            # Tanpa arus: tidak ada torsi motor sama sekali. Yang memperlambat
            # hanya gesekan gelinding dan detent, jauh lebih lemah daripada
            # pengereman motor - jadi robot terus meluncur.
            desel = konfig.SIM_LUNCUR_DESEL_CM_S2 / self.p.cm_per_step * dt
            if self.speed_sps > 0:
                self.speed_sps = max(0.0, self.speed_sps - desel)
            else:
                self.speed_sps = min(0.0, self.speed_sps + desel)
            return
        selisih = self.speed_cmd - self.speed_sps
        batas = self.accel * dt
        self.speed_sps += max(-batas, min(batas, selisih))

    # ----------------------------------------------------------------- gerak
    def langkah(self, dt=None):
        """Maju satu periode loop kendali, dipecah menjadi sub-langkah halus.

        Sub-langkah wajib: loop kendali berjalan 20 Hz (50 ms) sedangkan kamera
        25 fps (40 ms). Kalau fisika diintegrasikan langsung dengan dt 50 ms,
        setiap langkah pasti melewati satu batas frame dan usia frame menjadi
        konstan - laju kamera tidak terwakili sama sekali, dan justru ragam
        usianya yang sedang diteliti.
        """
        now = time.time()
        if dt is None:
            dt = self.dt_tetap if self.dt_tetap is not None else (now - self.t)
        self.t = now
        dt = min(dt, 0.2)                      # jangan melompat jauh setelah jeda

        sisa = dt
        while sisa > 1e-9:
            self._substep(min(konfig.SIM_SUBSTEP_S, sisa))
            sisa -= min(konfig.SIM_SUBSTEP_S, sisa)

    def _substep(self, dt):
        self.waktu += dt

        self._kemudi(dt)
        self._gas(dt)

        v = self.speed_sps * self.p.cm_per_step
        delta = math.radians(max(-self.p.max_steer_deg,
                                 min(self.p.max_steer_deg, self.steer_deg)))
        yaw_rad = math.radians(self.yaw)
        yaw_rad += v * math.tan(delta) / konfig.WHEELBASE_CM * dt
        self.x += v * math.cos(yaw_rad) * dt
        self.y += v * math.sin(yaw_rad) * dt
        self.yaw = math.degrees(yaw_rad)

        # Kamera menangkap pose hanya pada laju fps-nya sendiri. Loop kendali
        # berjalan di laju lain, jadi usia frame yang terbaca ikut berayun -
        # itulah delay yang berubah-ubah, tanpa perlu bilangan acak.
        if self.fps > 0 and self.waktu - self._kamera_t >= 1.0 / self.fps:
            self._kamera_t = self.waktu
            # t_siap = kapan frame ini selesai diproses dan bisa dipakai loop
            # kendali. Latensi tetap + ragam dekode/jaringan.
            siap = self.waktu + self.kamera_delay
            if self.kamera_jitter:
                siap += float(np.random.uniform(0.0, self.kamera_jitter))
            self._frames.append((self.waktu, siap, self.x, self.y, self.yaw))

    def baca(self):
        self.langkah()

        if not self.jeda:
            x, y, yaw = self.x, self.y, self.yaw
        else:
            # Frame terbaru yang sudah selesai diproses; yang lebih baru belum
            # "sampai" ke loop kendali.
            pilih = None
            for f in self._frames:
                if f[1] <= self.waktu:
                    pilih = f
            x, y, yaw = (pilih[2], pilih[3], pilih[4]) if pilih else (self.x, self.y, self.yaw)

        if self.derau_cm or self.derau_deg:
            # Derau ditambahkan pada yang DIBACA, bukan pada keadaan sebenarnya -
            # persis seperti kamera: robotnya tidak bergetar, pengukurannya yang
            # berderau.
            x += np.random.normal(0, self.derau_cm)
            y += np.random.normal(0, self.derau_cm)
            yaw += np.random.normal(0, self.derau_deg)
        return Pose(x, y, yaw, time.time(), True)

    def berhenti(self):
        pass


# ------------------------------------------------------------------ mandiri
def main():
    ap = argparse.ArgumentParser(description="Lihat pose robot dalam cm")
    ap.add_argument("--peta", default=None)
    ap.add_argument("--simulasi", action="store_true")
    ap.add_argument("--url", default=None)
    args = ap.parse_args()

    for tingkat, teks in konfig.periksa():
        print(f"[{tingkat}] {teks}")

    import arena as A
    arena = A.Arena(A.pilih(args.peta))
    print("[i]", arena.ringkas())

    if args.simulasi:
        x0, y0, x1, y1 = arena.ctx.ext
        sumber = SumberSimulasi((x0 + x1) / 2, (y0 + y1) / 2, 0.0)
        print("[i] SIMULASI: panah kiri/kanan = kemudi, atas/bawah = gas, spasi = berhenti")
    else:
        sumber = SumberKamera(arena.ctx, url=args.url)
        sumber.mulai()
        print(f"[i] Kamera siap, mencari tag id {konfig.TAG_ID_ROBOT}")

    latar = arena.kanvas()
    arena.gambar_grid(latar)
    arena.gambar_zona(latar)

    judul = "Pose robot"
    cv2.namedWindow(judul, cv2.WINDOW_AUTOSIZE)
    gas, kemudi = 0.0, 0.0
    try:
        while True:
            if args.simulasi:
                sumber.perintah(gas, kemudi)
            p = sumber.baca()
            img = latar.copy()

            if p is not None:
                warna = (0, 255, 0) if p.terlihat else (140, 140, 140)
                A.gambar_robot(img, arena.ctx, p.x, p.y, p.yaw, warna, isi=True)
                tag = ke_pusat_tag(p.x, p.y, p.yaw)
                cv2.drawMarker(img, tuple(arena.ctx.px([tag[:2]])[0]), (255, 0, 255),
                               cv2.MARKER_TILTED_CROSS, 12, 2)
                bebas = arena.bebas_cm(p.x, p.y)
                AC.label(img, f"poros depan X={p.x:.1f} Y={p.y:.1f} yaw={p.yaw:.1f} deg | "
                              f"bebas {bebas:.1f} cm | ungu = pusat tag",
                         (10, 22), warna, 0.5)
            else:
                AC.label(img, "tag belum terlihat", (10, 22), (0, 0, 255), 0.5)

            if args.simulasi:
                AC.label(img, f"gas {gas:+.0f} sps  kemudi {kemudi:+.1f} deg",
                         (10, 44), (255, 255, 255), 0.5)
            elif sumber.frame_basi:
                AC.label(img, "FRAME BASI - kamera berhenti mengirim", (10, 44), (0, 0, 255), 0.6)

            cv2.imshow(judul, img)
            k = cv2.waitKeyEx(30)
            key = k & 0xFF
            if key in (ord("q"), 27):
                break
            if args.simulasi:
                if k in (2424832, 65361) or key == ord("a"):
                    kemudi = max(-konfig.MAX_STEER_DEG, kemudi - 2)
                elif k in (2555904, 65363) or key == ord("d"):
                    kemudi = min(konfig.MAX_STEER_DEG, kemudi + 2)
                elif k in (2490368, 65362) or key == ord("w"):
                    gas = min(konfig.FORWARD_SPEED, gas + 500)
                elif k in (2621440, 65364) or key == ord("s"):
                    gas = max(-konfig.REVERSE_SPEED, gas - 500)
                elif key == ord(" "):
                    gas, kemudi = 0.0, 0.0
    finally:
        sumber.berhenti()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
