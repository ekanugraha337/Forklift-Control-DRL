"""
identifikasi.py - ukur parameter ruang-keadaan robot DI PERANGKAT NYATA.

Empat percobaan, masing-masing menjawab satu parameter:

  lurus   kemudi 0, beberapa kecepatan
          -> CM_PER_STEP (skala langkah) dan titik lurus kemudi sebenarnya
  busur   kemudi tetap, beberapa sudut
          -> penguatan kemudi tan(delta_nyata) vs tan(delta_diminta), dan
             di sudut berapa roda mulai jenuh
  tangga  kemudi melompat saat berjalan
          -> tunda murni dan tetapan waktu orde-1 (SIM_KEMUDI_TUNDA_S, _TAU_S)
  gas     kecepatan melompat naik lalu turun, kemudi 0, empat laju target
          -> akselerasi motor yang BENAR-BENAR terjadi, tunda perintah->gerak,
             dan apakah akselerasinya tetap atau melemah di laju tinggi.
             TIDAK ikut --mode semua: jalankan sendiri, dengan robot menghadap
             lintasan lurus bebas ~1,2 m (empat kali maju, tiap kali pulang).

PENGAMAN. Program ini MENGGERAKKAN ROBOT. Sebelum tiap manuver, lintasannya
diramalkan dengan model tak-linear di state_space.py lalu diperiksa terhadap
peta constraint; manuver yang tidak muat dilewati, bukan dicoba lalu ditabrakkan.
Selama berjalan, badan robot dipantau tiap frame dan estop dikirim begitu
menyentuh zona, tag hilang, frame kamera basi, atau MQTT putus.

RUANG YANG DIBUTUHKAN. Busur pada kemudi penuh berjari-jari ~32 cm, jadi
perlu lingkaran bebas ~110 cm. Peta yang rapat (mis. Rumah) tidak akan muat -
pakai area lapang, dan program akan mengatakan sudut mana saja yang muat.

    python identifikasi.py --simulasi            uji programnya sendiri, tanpa robot
    python identifikasi.py --peta Kalibrasi      di perangkat
    python identifikasi.py --peta Kalibrasi --mode busur
    python identifikasi.py --peta Kalibrasi --mode gas
    python identifikasi.py --peta Kalibrasi --id 2     robot lain (lihat konfig.ROBOT)
"""
import argparse
import csv
import glob
import math
import os
import re
import time

import numpy as np
from scipy.optimize import least_squares

import arena as A
import konfig
import pose as P
import robot as RB
import state_space as SS

konfig.pasang_path_kamera()
import area_constraint as AC          # noqa: E402

# ---------------------------------------------------------------------------
# UJI TANGGA GAS
# ---------------------------------------------------------------------------
# Tiap lintasan: diam -> lompat ke target -> turun ke sepertiganya, kemudi 0.
#
# Fase NAIK harus cukup panjang untuk mencapai laju tunak bahkan kalau
# akselerasi nyata serendah yang disarankan cusp.csv (~2000 sps/s^2): 7000 sps
# butuh 3,5 detik pada 2000, ditambah ~0,5 detik tunak agar skala cm/langkah
# ikut terukur. Kalau fasenya pendek, laju tunak tidak pernah tercapai dan
# akselerasi tidak bisa dipisahkan dari skala.
#
# Fase TURUN sengaja berhenti di sepertiga, BUKAN nol. Di firmware keduanya
# jalur kode yang berbeda (driveRc di drive.cpp): perintah di bawah RC_MIN_SPS
# memanggil stopMove(), sedangkan perubahan laju di tengah rute memanggil
# applySpeedAcceleration(). Yang perlu diukur yang kedua - perlambatan di
# tengah rute itulah yang gagal diikuti robot pada tabrakan Z8.
#
# Start dari diam juga jalurnya sendiri: enableOutputs() lalu runForward(),
# karena estop sebelumnya mematikan driver. Karena itu tunda NAIK dan tunda
# TURUN dicocokkan terpisah - tidak ada alasan keduanya sama.
GAS_DIAM_S = 0.5             # acuan posisi awal + lantai derau kamera
GAS_NAIK_S = 4.0
GAS_TURUN_S = 2.5
GAS_PULANG_SPS = 4000        # laju leg pulang; mundur selalu pelan

# Kolom telemetri firmware yang disimpan selama uji gas (net.cpp).
#   tgt_sps  driveSpeedHz - berubah saat driveRc() MENERIMA perintah rc
#   sps      kecepatan stepper saat ini - berubah saat ramp benar-benar MULAI
#   pos, rc, dir, drv  pelengkap untuk membaca ulang kejadiannya
KOLOM_TELEMETRI = ("tgt_sps", "sps", "pos", "rc", "dir", "drv")


# ===========================================================================
# PEREKAM
# ===========================================================================
class Rekaman:
    """Deret waktu satu manuver: perintah yang dikirim dan pose yang terukur."""

    def __init__(self, nama, sps, steer_deg, kolom_cmd="cmd_deg"):
        self.nama, self.sps, self.steer_deg = nama, sps, steer_deg
        # Yang dicatat di "cmd" adalah besaran yang DILOMPATKAN: sudut kemudi
        # untuk uji tangga, laju langkah untuk uji gas. Nama kolomnya ikut,
        # supaya CSV-nya tidak menyebut sps sebagai derajat.
        self.kolom_cmd = kolom_cmd
        self.t, self.x, self.y, self.yaw, self.cmd = [], [], [], [], []

    def tambah(self, t, pose, cmd_deg):
        self.t.append(t); self.x.append(pose.x); self.y.append(pose.y)
        self.yaw.append(pose.yaw); self.cmd.append(cmd_deg)

    def arr(self):
        return (np.array(self.t), np.array(self.x), np.array(self.y),
                np.unwrap(np.radians(np.array(self.yaw))), np.array(self.cmd))

    def transien_s(self):
        """Lama transien yang harus dibuang sebelum mengukur keadaan tunak.

        Dua hal harus selesai dulu: ramp akselerasi stepper (sps/accel) dan
        kemudi mencapai sudutnya. Memakai satu angka tetap membuat pengukuran
        pada 12000 sps ikut menghitung masa ramp, dan CM_PER_STEP terbaca
        terlalu kecil.

        Untuk kemudi dipakai tunda + 3 tau (95% tuntas), bukan 2 tau (86%).
        Dengan 2 tau, penguatan kemudi terbaca ~4% lebih kecil dari yang
        sebenarnya - bukan karena robotnya, melainkan karena diukur sebelum
        kemudinya selesai bergerak. Uji-mandiri terhadap simulator yang
        penguatannya JELAS 1,000 membaca 0,963; itu biasnya.
        """
        ramp = abs(self.sps) / max(konfig.SIM_ACCEL_SPS2, 1.0)
        if not np.isfinite(self.steer_deg) or not self.steer_deg:
            return ramp
        return max(ramp, konfig.SIM_KEMUDI_TUNDA_S + 3.0 * konfig.SIM_KEMUDI_TAU_S)

    def jendela_ukur_s(self):
        """Lama jendela yang benar-benar dipakai mengukur, setelah transien."""
        return (self.t[-1] - self.t[0] - self.transien_s()) if len(self.t) > 1 else 0.0

    def laju_dan_kelengkungan(self, buang_awal_s=None):
        """(v cm/detik, kappa 1/cm) keadaan tunak, tahan derau kamera.

        JANGAN memakai beda-hingga antar cuplikan. Kecepatan dari
        |selisih posisi| BIAS KE ATAS oleh derau, dan biasnya tidak hilang
        dengan dirata-rata karena setiap sukunya positif. Pada 10 cm/detik dan
        derau 1,2 cm, gerak nyata per cuplikan 0,52 cm sementara derau
        menyumbang 1,7 cm - yang terukur deraunya, bukan robotnya. Diukur di
        simulator: CM_PER_STEP terbaca +199% dengan cara itu.

        Yang dipakai di sini:
          - laju yaw dari REGRESI LINEAR yaw terhadap waktu. Deraunya nol-rata,
            jadi kemiringannya tak bias dan seluruh cuplikan ikut terpakai.
          - kecepatan dari TALI BUSUR ujung-ke-ujung, dikoreksi menjadi panjang
            busur memakai laju yaw di atas. Derau menyumbang beberapa cm pada
            perpindahan puluhan cm, dan tidak bias.
        """
        t, x, y, yaw, _ = self.arr()
        if len(t) < 8:
            return float("nan"), float("nan")
        if buang_awal_s is None:
            buang_awal_s = self.transien_s()
        mulai = np.searchsorted(t, t[0] + buang_awal_s)
        if len(t) - mulai < 5:
            mulai = max(0, len(t) - 5)
        t, x, y, yaw = t[mulai:], x[mulai:], y[mulai:], yaw[mulai:]

        dT = float(t[-1] - t[0])
        if dT <= 1e-6:
            return float("nan"), float("nan")

        w = float(np.polyfit(t, yaw, 1)[0])              # rad/detik, tak bias
        tali = float(np.hypot(x[-1] - x[0], y[-1] - y[0]))

        # tali = 2 R sin(w dT / 2) dan busur = R w dT, jadi
        # busur = tali * (w dT / 2) / sin(w dT / 2). Untuk w -> 0 faktornya 1.
        sudut = abs(w) * dT / 2.0
        faktor = 1.0 if sudut < 1e-6 else sudut / math.sin(sudut) if sudut < math.pi else float("nan")
        busur = tali * faktor
        v = busur / dT

        k = w / v if v > 0.5 else float("nan")
        return v, k

    def laju_yaw_licin(self, jendela=9):
        """Laju yaw per cuplikan lewat regresi bergerak - untuk uji tangga.

        Uji tangga memerlukan kelengkungan yang BERUBAH terhadap waktu, jadi
        satu angka tunak tidak cukup. Beda-hingga mentah terlalu berderau untuk
        itu, jadi tiap titik dipasangi regresi linear pada jendela di sekitarnya.
        """
        t, x, y, yaw, _ = self.arr()
        n = len(t)
        if n < jendela + 2:
            return np.array([]), np.array([]), np.array([])
        h = jendela // 2
        w = np.full(n, np.nan)
        v = np.full(n, np.nan)
        for i in range(h, n - h):
            sl = slice(i - h, i + h + 1)
            w[i] = np.polyfit(t[sl], yaw[sl], 1)[0]
            dT = t[sl][-1] - t[sl][0]
            if dT > 1e-6:
                v[i] = np.hypot(x[sl][-1] - x[sl][0], y[sl][-1] - y[sl][0]) / dT
        return t, v, w

    def simpan(self, folder):
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, f"{self.nama}.csv")
        with open(path, "w") as fh:
            fh.write(f"t,x_cm,y_cm,yaw_deg,{self.kolom_cmd},sps\n")
            for i in range(len(self.t)):
                fh.write(f"{self.t[i]:.3f},{self.x[i]:.2f},{self.y[i]:.2f},"
                         f"{self.yaw[i]:.2f},{self.cmd[i]:.2f},{self.sps}\n")
        return path


# ===========================================================================
# PELAKSANA
# ===========================================================================
class Pelaksana:
    def __init__(self, args):
        self.args = args
        self.arena = A.Arena(A.pilih(args.peta))
        self.per = __import__("rencana").Perencana(self.arena)
        # Akhiran id: parameter yang diukur milik SATU robot, dan log ID 01 dan
        # ID 02 tidak boleh tertukar saat dianalisis ulang dengan --dari.
        self.folder = os.path.join(konfig.DIR, "logs_identifikasi",
                                   time.strftime("%Y%m%d_%H%M%S") + f"_id{konfig.ROBOT_ID:02d}")

        if args.simulasi:
            x0, y0, x1, y1 = self.arena.ctx.ext
            d = self.arena.omap.dist
            iy, ix = np.unravel_index(np.argmax(d), d.shape)
            xx, yy = self.arena.ke_cm(float(ix), float(iy))
            self.sim = P.SumberSimulasi(xx, yy, 0.0, dt_tetap=konfig.RC_PERIODE_S,
                                        derau_cm=args.derau, derau_deg=args.derau_deg)
            self.sumber = self.sim
        else:
            self.sim = None
            self.sumber = P.SumberKamera(self.arena.ctx, url=args.url)
            self.sumber.mulai()

        self.robot = RB.Robot(host=args.host, simulasi=self.sim)
        if not self.robot.sambung() and not args.simulasi:
            raise SystemExit("[!] MQTT tidak tersambung.")
        self.batal = None          # alasan seluruh sisa percobaan dibatalkan

    def jam(self):
        """Waktu yang seirama dengan gerak robot.

        Di simulasi, jam dinding TIDAK boleh dipakai: simulator maju dt_tetap
        per panggilan sementara loop memakan waktu nyata yang sedikit berbeda,
        dan selisih beberapa persen itu langsung muncul sebagai CM_PER_STEP yang
        salah. Di perangkat nyata keduanya memang jam yang sama.
        """
        return self.sim.waktu if self.sim is not None else time.time()

    # ------------------------------------------------------------ pengaman
    def pose_sekarang(self, timeout=5.0):
        batas = time.time() + timeout
        while time.time() < batas:
            p = self.sumber.baca()
            if p is not None and p.terlihat:
                return p
            time.sleep(0.05)
        return None

    def bahaya(self, pose, izinkan_ramalan=False):
        if pose is None or not pose.terlihat:
            return "tag robot hilang"
        # Pose RAMALAN boleh dipakai navigasi untuk menjembatani tag yang kabur,
        # tapi TIDAK boleh masuk ke sini: ia bukan pengukuran. Merekamnya sebagai
        # data berarti mengukur model sendiri, bukan robot - dan parameter yang
        # dihasilkan akan sekadar memantulkan balik apa yang sudah diasumsikan.
        #
        # izinkan_ramalan=True dipakai uji gas: pose ramalan tidak MEMBATALKAN
        # percobaan, hanya dilewati pemanggil saat merekam. Sebabnya,
        # SumberKamera.baca() meramal setiap kali satu siklus loop tidak kebagian
        # frame baru (StreamReader.read_new tidak menunggu) atau tag tidak
        # terdeteksi di frame itu. Membatalkan karenanya membuat uji di laju
        # tinggi - persis yang ingin diukur, dan paling sering kabur - gugur
        # karena satu frame. Pemeriksaan bahaya lainnya tetap berjalan.
        if getattr(pose, "diramalkan", False) and not izinkan_ramalan:
            return "pose sedang diramalkan (tag tidak terdeteksi)"
        if not self.args.simulasi:
            if self.sumber.frame_basi:
                return "frame kamera basi"
            if not self.robot.tersambung:
                return "MQTT terputus"
            # Stall atau gangguan driver membuat motor TIDAK bergerak sesuai
            # perintah. Tanpa pemeriksaan ini, CM_PER_STEP dan penguatan kemudi
            # terukur terlalu kecil tanpa satu pun tanda - hasilnya salah tapi
            # terlihat rapi, lalu dipakai mengubah konfig.py.
            gangguan = self.robot.gangguan_driver()
            if gangguan:
                return f"driver bermasalah: {gangguan}"
            if self.robot.ambil_stall():
                return "motor STALL - pengukuran tidak sah"
        st, zona, _ = AC.evaluasi(A.jejak_badan(pose.x, pose.y, pose.yaw),
                                  self.arena.zona, 0.0)
        return f"badan robot menyentuh zona {zona}" if st == AC.PELANGGARAN else None

    def muat(self, pose, sps, steer_deg, detik):
        """Ramalkan manuver dengan model tak-linear, periksa ke peta constraint.

        Inilah gunanya state_space.turunan(): manuver diuji di atas kertas dulu.
        Kalau tidak muat, dilewati - bukan dicoba lalu ditabrakkan.

        Keadaan awal SENGAJA memakai kemudi 0 dan kecepatan 0, bukan sudut yang
        diminta. Kemudi robot tertinggal 0,2-0,7 detik, jadi di awal manuver ia
        melaju jauh lebih LURUS daripada busur yang diminta - dan lurus berarti
        lebih jauh menjangkau ke depan. Memulai ramalan dari sudut penuh membuat
        pemeriksaan ini terlalu optimistis; uji-mandiri pertama menabrak zona
        justru karena itu.
        """
        # steer_deg boleh berupa fungsi waktu, supaya manuver yang sudutnya
        # BERUBAH (uji tangga) diperiksa dengan profil yang benar-benar akan
        # dijalankan. Memeriksanya sebagai busur tetap pernah meloloskan uji
        # tangga yang lintasannya dua kali lebih panjang.
        #
        # Sebab kegagalan disimpan di self.alasan_muat. Tanpa itu "tidak muat"
        # tidak bisa ditindaklanjuti: pada 2026-09-27 uji tangga gugur karena
        # robot masih di titik akhir navigasi menghadap tepi (menabrak Z10
        # setelah 41 cm), dan itu baru ketahuan lewat analisis terpisah.
        sudut = steer_deg if callable(steer_deg) else (lambda _t: steer_deg)
        xi = np.array([pose.x, pose.y, math.radians(pose.yaw), 0.0, 0.0])
        h = 0.02
        jarak = 0.0
        self.alasan_muat = None
        for i in range(int(detik / h)):
            u = np.array([math.radians(sudut(i * h)), sps * konfig.CM_PER_STEP])
            x_lalu, y_lalu = xi[0], xi[1]
            xi = xi + SS.turunan(xi, u) * h
            jarak += math.hypot(xi[0] - x_lalu, xi[1] - y_lalu)
            fp = A.jejak_badan(xi[0], xi[1], math.degrees(xi[2]))
            st, zona, _ = AC.evaluasi(fp, self.arena.zona, 0.0)
            if st == AC.PELANGGARAN or not self.arena.di_dalam(xi[0], xi[1]):
                sebab = (f"menabrak zona {zona}" if st == AC.PELANGGARAN
                         else "keluar batas arena")
                self.alasan_muat = (f"{sebab} setelah ~{jarak:.0f} cm, di ({xi[0]:.0f}, "
                                    f"{xi[1]:.0f}); robot di ({pose.x:.0f}, {pose.y:.0f}) "
                                    f"menghadap {pose.yaw:.0f} deg")
                return False
        return True

    def arah_muat(self, pose, sps, steer_deg, detik):
        """Arah hadap (per 15 deg) yang MUAT dari titik yang sama -> daftar derajat.

        Untuk uji sekali-jalan (tangga, gas) yang gugur: operator bisa memutar
        robot di tempat alih-alih mencari titik baru secara coba-coba.
        """
        simpan = getattr(self, "alasan_muat", None)
        hasil = [d for d in range(0, 360, 15)
                 if self.muat(P.Pose(pose.x, pose.y, float(d), pose.t, True),
                              sps, steer_deg, detik)]
        self.alasan_muat = simpan
        return hasil

    def _lapor_tidak_muat(self, nama, pose, sps, steer_deg, detik, arah=False):
        print(f"  [-] {nama}: tidak muat - {self.alasan_muat}")
        if arah:
            d = self.arah_muat(pose, sps, steer_deg, detik)
            print(f"      arah hadap yang muat dari titik ini: "
                  + (", ".join(f"{x}" for x in d) + " deg" if d else
                     "TIDAK ADA - pindahkan robot ke area yang lebih lapang"))

    # ------------------------------------------------------------- manuver
    def _jalan(self, nama, sps, steer_deg, detik):
        """Jalankan gerakan tanpa merekam (leg pulang, putar arah)."""
        if self.batal:
            return False
        pose = self.pose_sekarang()
        if pose is None or not self.muat(pose, sps, steer_deg, detik):
            sebab = "tag tidak terlihat" if pose is None else self.alasan_muat
            print(f"       ({nama} tidak muat: {sebab} - manuver berikutnya mungkin terganggu)")
            return False
        t0 = self.jam()
        while self.jam() - t0 < detik:
            p = self.sumber.baca()
            alasan = self.bahaya(p)
            if alasan:
                self.robot.estop(alasan)
                self.batal = alasan
                print(f"       [!] {nama} DIHENTIKAN - {alasan}")
                return False
            self.robot.kirim_rc(sps, steer_deg, paksa=True)
            if self.sim is None:
                time.sleep(konfig.RC_PERIODE_S)
        self.robot.estop(f"selesai {nama}")
        self._diam(1.0)
        return True

    def pulang(self, sps, steer_deg, detik):
        """Ulangi manuver ke arah sebaliknya untuk kembali ke dekat titik awal.

        Tanpa ini percobaan tidak bisa jalan sama sekali: tiap manuver memindah
        robot 50-100 cm dan tidak pernah balik, jadi setelah dua atau tiga
        manuver ruang di depannya habis dan sisanya gugur "tidak muat".
        """
        return self._jalan("pulang", -abs(sps), steer_deg, detik)

    def putar(self):
        """Ubah arah hadap robot sekitar 90 derajat.

        Dipakai DI ANTARA pengulangan lintasan lurus, dan bukan sekadar variasi:
        mengulang di koridor yang sama membuat galatnya BERKORELASI. Distorsi
        homografi yang tersisa di suatu tempat akan terulang identik setiap kali
        robot melewatinya, sehingga merata-ratakan tidak menghapusnya sama
        sekali - sebaran antar pengulangan terlihat kecil padahal angkanya tetap
        meleset. Titik lurus kemudi adalah sifat ROBOT, jadi ia harus sama di
        segala arah; distorsi kamera tidak. Menghadap arah berbeda memisahkan
        keduanya.
        """
        sps = self.args.sps_busur
        v = sps * konfig.CM_PER_STEP
        delta = math.radians(30.0)
        omega = v * math.tan(delta) / konfig.WHEELBASE_CM      # rad/detik
        detik = (math.pi / 2) / omega + konfig.SIM_KEMUDI_TUNDA_S + konfig.SIM_KEMUDI_TAU_S

        pose = self.pose_sekarang()
        if pose is None:
            return False

        # Arah putar dipilih yang MENJAUH dari zona, bukan selalu ke kanan.
        # Memutar ke arah yang sama terus membuat robot mengembara: pada
        # pengulangan ke-7 ia sudah menepi dan menabrak zona. Kedua arah
        # diramalkan dulu, lalu diambil yang berakhir paling lapang.
        pilihan = []
        for sudut in (30.0, -30.0):
            akhir = self._ramal(pose, sps, sudut, detik)
            if akhir is None:
                continue
            pilihan.append((self.arena.bebas_cm(akhir[0], akhir[1]), sudut))
        pilihan.sort(reverse=True)
        for _, sudut in pilihan:
            if self._jalan("putar", sps, sudut, detik):
                return True
        return False

    def _ramal(self, pose, sps, steer_deg, detik):
        """Pose akhir menurut model, atau None bila lintasannya menabrak."""
        sudut = steer_deg if callable(steer_deg) else (lambda _t: steer_deg)
        xi = np.array([pose.x, pose.y, math.radians(pose.yaw), 0.0, 0.0])
        h = 0.02
        for i in range(int(detik / h)):
            u = np.array([math.radians(sudut(i * h)), sps * konfig.CM_PER_STEP])
            xi = xi + SS.turunan(xi, u) * h
            st, _, _ = AC.evaluasi(A.jejak_badan(xi[0], xi[1], math.degrees(xi[2])),
                                   self.arena.zona, 0.0)
            if st == AC.PELANGGARAN or not self.arena.di_dalam(xi[0], xi[1]):
                return None
        return (xi[0], xi[1], math.degrees(xi[2]))

    def manuver(self, nama, sps, steer_deg, detik):
        if self.batal:
            return None
        pose = self.pose_sekarang()
        if pose is None:
            print(f"  [-] {nama}: tag tidak terlihat, dilewati")
            return None
        if not self.muat(pose, sps, steer_deg, detik):
            self._lapor_tidak_muat(nama, pose, sps, steer_deg, detik)
            return None

        rek = Rekaman(nama, sps, steer_deg)
        t0 = self.jam()
        alasan = None
        while self.jam() - t0 < detik:
            p = self.sumber.baca()
            alasan = self.bahaya(p)
            if alasan:
                break
            self.robot.kirim_rc(sps, steer_deg, paksa=True)
            rek.tambah(self.jam() - t0, p, steer_deg)
            if self.sim is None:
                time.sleep(konfig.RC_PERIODE_S)

        self.robot.estop(f"selesai {nama}" if not alasan else alasan)
        self._diam(1.2)
        if alasan:
            print(f"  [!] {nama}: DIHENTIKAN - {alasan}")
            # Robot berhenti di tempat yang menyentuh zona. Manuver berikutnya
            # akan gugur satu per satu tanpa ada yang bisa menolong, jadi
            # seluruh sisa percobaan dibatalkan dengan sebab yang jelas -
            # bukan dibiarkan mencetak belasan baris "tidak muat".
            self.batal = alasan
            return None
        v, k = rek.laju_dan_kelengkungan()
        if not np.isfinite(v):
            print(f"  [-] {nama}: data terlalu sedikit")
            return None
        print(f"  [OK] {nama}: {len(rek.t)} cuplikan, v {v:6.2f} cm/s, "
              f"kappa {k * 100:7.4f} /m")
        rek.simpan(self.folder)
        return rek

    def _diam(self, detik):
        t0 = self.jam()
        while self.jam() - t0 < detik:
            self.robot.kirim_rc(0, 0, paksa=True)
            self.sumber.baca()
            if self.sim is None:
                time.sleep(konfig.RC_PERIODE_S)

    # ---------------------------------------------------------- percobaan
    def lurus(self):
        ulang = max(1, int(self.args.ulang))
        print(f"\n--- LURUS: skala langkah & titik lurus kemudi "
              f"({ulang} pengulangan) ---")
        hasil = []
        for putaran in range(ulang):
            if self.batal:
                break
            if putaran:
                print(f"  putaran {putaran + 1}/{ulang} - memutar arah dulu")
                self.putar()
            for sps in (4000, 8000, 12000):
                r = self.manuver(f"lurus_{sps}_p{putaran}", sps, 0.0,
                                 self.args.detik_lurus)
                if r:
                    hasil.append(r)
                self.pulang(sps, 0.0, self.args.detik_lurus)
        return hasil

    def busur(self):
        print("\n--- BUSUR: penguatan kemudi & kejenuhan ---")
        hasil = []
        for deg in (5, 10, 15, 20, 25, 30, 35):
            for tanda in (1, -1):
                r = self.manuver(f"busur_{tanda * deg:+03d}", self.args.sps_busur,
                                 tanda * deg, self.args.detik_busur)
                if r:
                    hasil.append(r)
                self.pulang(self.args.sps_busur, tanda * deg, self.args.detik_busur)
        return hasil

    def tangga(self):
        print("\n--- TANGGA: tunda dan tetapan waktu kemudi ---")
        # Tiap fase harus cukup lama agar responsnya sempat terlihat, tapi
        # jumlah fase dibatasi supaya robot tidak berjalan terlalu jauh.
        fase_s = max(2.5, konfig.SIM_KEMUDI_TUNDA_S + 3.0 * konfig.SIM_KEMUDI_TAU_S)
        pola = (0.0, 20.0, 0.0, -20.0)
        total = fase_s * len(pola)

        def sudut(t):
            return pola[int(t / fase_s) % len(pola)]

        pose = self.pose_sekarang()
        if pose is None:
            print("  [-] tangga: tag tidak terlihat, dilewati")
            return []
        if not self.muat(pose, self.args.sps_busur, sudut, total):
            self._lapor_tidak_muat("tangga", pose, self.args.sps_busur, sudut, total,
                                   arah=True)
            return []
        rek = Rekaman("tangga", self.args.sps_busur, float("nan"))
        t0 = self.jam()
        alasan = None
        while self.jam() - t0 < total:
            t = self.jam() - t0
            cmd = sudut(t)
            p = self.sumber.baca()
            alasan = self.bahaya(p)
            if alasan:
                break
            self.robot.kirim_rc(self.args.sps_busur, cmd, paksa=True)
            rek.tambah(t, p, cmd)
            if self.sim is None:
                time.sleep(konfig.RC_PERIODE_S)
        self.robot.estop("selesai tangga" if not alasan else alasan)
        self._diam(1.2)
        if alasan:
            print(f"  [!] tangga DIHENTIKAN - {alasan}")
            return []
        print(f"  [OK] tangga: {len(rek.t)} cuplikan")
        rek.simpan(self.folder)
        return [rek]

    def gas(self):
        """Tangga GAS pada beberapa laju target -> akselerasi motor sebenarnya.

        EMPAT target, bukan satu, dan itu inti rancangannya - bukan variasi.
        Dalam SATU lintasan, tunda dan akselerasi tidak bisa dipisahkan: ramp
        12000 sps/s^2 hanya berlangsung 0,2-0,6 detik (4-12 frame kamera), dan
        "terlambat mulai" terlihat hampir sama dengan "lambat menanjak". Uji-
        mandiri di simulator mengukur korelasi keduanya +0,98 sampai +0,99.

        Yang terukur baik dalam satu lintasan adalah KETERLAMBATAN TOTAL
        L = tunda + target / (2 A) - pergeseran garis laju tunak, yang
        ditentukan puluhan cuplikan. Antar-lintasan, L linear terhadap target:
        intersepnya tunda, kemiringannya 1/(2A). Beberapa target memisahkan
        keduanya, dan target keempat memberi derajat bebas untuk menguji apakah
        garisnya memang lurus - yaitu apakah akselerasi memang TETAP.
        """
        print("\n--- GAS: akselerasi motor yang sebenarnya ---")
        print(f"  setelan: ACCEL_SPS2 = {konfig.ACCEL_SPS2:.0f} sps/s^2 "
              f"(harus sama dengan NVS)")
        hasil = []
        f = konfig.FORWARD_SPEED
        for target in sorted({int(round(f * r, -2)) for r in (0.3, 0.55, 0.8, 1.0)}):
            if self.batal:
                break
            r = self._gas_satu(target)
            if r is None:
                continue
            hasil.append(r)
            # Pulang sejauh yang benar-benar ditempuh, bukan sepanjang durasi
            # yang sama: profil gas tidak berlaju tetap, jadi mengulang durasinya
            # pada laju pulang akan mendarat di tempat yang salah.
            jarak = math.hypot(r.x[-1] - r.x[0], r.y[-1] - r.y[0])
            self.pulang(GAS_PULANG_SPS, 0.0,
                        jarak / (GAS_PULANG_SPS * konfig.CM_PER_STEP))
        return hasil

    def _gas_satu(self, target):
        rendah = max(konfig.SPS_MINIMUM, target // 3)
        total = GAS_DIAM_S + GAS_NAIK_S + GAS_TURUN_S

        def profil(t):
            if t < GAS_DIAM_S:
                return 0
            return target if t < GAS_DIAM_S + GAS_NAIK_S else rendah

        nama = f"gas_{target}"
        pose = self.pose_sekarang()
        if pose is None:
            print(f"  [-] {nama}: tag tidak terlihat, dilewati")
            return None
        # Diperiksa seolah seluruh lintasan berjalan pada target - lebih jauh
        # dari yang sebenarnya karena fase turun lebih pelan. Sengaja: kalau
        # salah, salahnya ke arah aman.
        if not self.muat(pose, target, 0.0, total - GAS_DIAM_S):
            self._lapor_tidak_muat(nama, pose, target, 0.0, total - GAS_DIAM_S, arah=True)
            return None

        rek = Rekaman(nama, target, 0.0, kolom_cmd="cmd_sps")
        # Waktu KIRIM tiap perubahan perintah, dicatat langsung di loop. Tidak
        # bisa diturunkan dari cuplikan yang terekam: cuplikan pose ramalan
        # dilewati, jadi cuplikan pertama dengan perintah baru bisa terlambat
        # satu siklus - dan satu siklus (50 ms) itu seukuran tunda yang dicari.
        rek.peristiwa = []
        dibuang = 0
        terakhir = 0
        # Telemetri direkam sepanjang lintasan PLUS masa diam sesudahnya: tunda
        # yang terukur ~1 detik berarti efek perintah turun bisa jatuh dekat
        # ujung lintasan, dan tunda yang lebih panjang lagi tidak boleh hilang.
        if self.sim is None:
            self.robot.mulai_rekam_telemetri()
        t0 = self.jam()
        alasan = None
        while self.jam() - t0 < total:
            t = self.jam() - t0
            cmd = profil(t)
            p = self.sumber.baca()
            alasan = self.bahaya(p, izinkan_ramalan=True)
            if alasan:
                break
            self.robot.kirim_rc(cmd, 0.0, paksa=True)
            if cmd != terakhir:
                rek.peristiwa.append((t, cmd))
                terakhir = cmd
            if getattr(p, "diramalkan", False):
                dibuang += 1
            else:
                rek.tambah(t, p, cmd)
            if self.sim is None:
                time.sleep(konfig.RC_PERIODE_S)

        self.robot.estop(f"selesai {nama}" if not alasan else alasan)
        self._diam(1.2)
        # jam() di perangkat = time.time(), jadi waktu terima telemetri bisa
        # langsung dijadikan relatif terhadap awal lintasan.
        rek.telemetri = ([(tw - t0, d) for tw, d in self.robot.selesai_rekam_telemetri()]
                         if self.sim is None else [])
        if alasan:
            print(f"  [!] {nama}: DIHENTIKAN - {alasan}")
            self.batal = alasan
            return None
        if len(rek.peristiwa) < 2 or len(rek.t) < 30:
            print(f"  [-] {nama}: data terlalu sedikit ({len(rek.t)} cuplikan)")
            return None
        print(f"  [OK] {nama}: {len(rek.t)} cuplikan"
              + (f", {dibuang} pose ramalan dilewati" if dibuang else "")
              + (f", {len(rek.telemetri)} pesan telemetri" if self.sim is None else ""))
        if self.sim is None and len(rek.telemetri) < 10:
            print(f"  [!] {nama}: telemetri hampir tidak masuk - cek topik "
                  f"{konfig.TOPIK_TELEMETRI}")
        rek.simpan(self.folder)
        # Waktu kirim perintah disimpan terpisah, supaya --dari bisa
        # menganalisis ulang dengan waktu yang TEPAT. Dari cuplikan saja ia
        # hanya bisa diperkirakan, bisa terlambat satu siklus loop.
        with open(os.path.join(self.folder, f"{nama}_perintah.csv"), "w") as fh:
            fh.write("t,sps\n")
            for t_kirim, sps in rek.peristiwa:
                fh.write(f"{t_kirim:.3f},{sps}\n")
        if rek.telemetri:
            with open(os.path.join(self.folder, f"{nama}_telemetri.csv"), "w") as fh:
                fh.write(",".join(("t",) + KOLOM_TELEMETRI) + "\n")
                for t_terima, d in rek.telemetri:
                    fh.write(f"{t_terima:.3f},"
                             + ",".join(str(d.get(k, "")) for k in KOLOM_TELEMETRI) + "\n")
        return rek

    def tutup(self):
        try:
            self.robot.berhenti()
            time.sleep(0.2)
            self.robot.putus()
        finally:
            self.sumber.berhenti()


# ===========================================================================
# PENCOCOKAN PARAMETER
# ===========================================================================
def cocokkan_lurus(rekaman):
    print("\n=== HASIL: skala langkah & titik lurus ===")
    if not rekaman:
        print("  tidak ada data.")
        return
    skala, offset = [], []
    per_laju = {}
    for r in rekaman:
        vv, kk = r.laju_dan_kelengkungan()
        s = vv / r.sps if r.sps else float("nan")
        # kelengkungan sisa pada kemudi 0 = titik lurus yang meleset
        d_off = (math.degrees(math.atan(kk * konfig.WHEELBASE_CM))
                 if np.isfinite(kk) else float("nan"))
        skala.append(s); offset.append(d_off)
        per_laju.setdefault(r.sps, []).append((vv, s, d_off))

    for sps in sorted(per_laju):
        a = np.array(per_laju[sps], dtype=float)
        print(f"  {sps:6d} sps x{len(a):<3d} -> {np.nanmean(a[:, 0]):6.2f} cm/detik | "
              f"{np.nanmean(a[:, 1]):.6f} cm/langkah | "
              f"sisa belok {np.nanmean(a[:, 2]):+6.2f} deg")

    skala = np.array(skala, dtype=float)
    offset = np.array(offset, dtype=float)
    n = int(np.isfinite(offset).sum())
    s = float(np.nanmean(skala))
    print(f"  CM_PER_STEP terukur {s:.6f}  (konfig {konfig.CM_PER_STEP:.6f}, "
          f"selisih {100.0 * (s - konfig.CM_PER_STEP) / konfig.CM_PER_STEP:+.1f}%)")

    # Titik lurus diturunkan dari kelengkungan SISA pada lintasan yang seharusnya
    # lurus - besaran nyaris nol, jadi derau kamera sangat terasa di sini.
    #
    # Yang dilaporkan adalah GALAT BAKU RATA-RATA (sebaran / akar n), bukan
    # sebarannya. Sebaran menjawab "seberapa berbeda satu lintasan dari lintasan
    # lain", sedangkan yang ditanyakan adalah "seberapa yakin kita pada
    # rata-ratanya" - dan itu mengecil seiring jumlah pengulangan. Memakai
    # sebaran akan menolak angka yang sebenarnya sudah cukup baik.
    o = float(np.nanmean(offset))
    sd = float(np.nanstd(offset, ddof=1)) if n > 1 else float("nan")
    se = sd / math.sqrt(n) if n > 1 else float("nan")
    print(f"  Titik lurus meleset {o:+.2f} deg, galat baku {se:.2f} deg "
          f"(dari {n} lintasan, sebaran {sd:.2f})")

    BATAS = 0.25          # ketelitian yang dianggap memadai, derajat
    if not np.isfinite(se):
        print("  Data terlalu sedikit untuk menilai.")
    elif abs(o) > 2 * se:
        # Arah koreksinya bergantung pada pembalikan firmware. robot.py mengirim
        # center + kemudi, dan firmware v2 dengan str_inv = 1 memutar servo ke
        # 2 x str_center - itu = center - kemudi. Karena str_center dan
        # STEER_SERVO_CENTER harus sama, titik lurus yang baru = center + o.
        # Rumus lama (center - o) hanya benar tanpa pembalikan; dengan
        # pembalikan ia menyarankan arah yang SALAH (terjadi di ID 02,
        # 2026-09-30: menyarankan 96,36 padahal yang benar ~95,94).
        baru = konfig.STEER_SERVO_CENTER + (o if konfig.STEER_INVERT else -o)
        print(f"  ADA pergeseran nyata (|{o:+.2f}| > 2 x {se:.2f}).")
        print(f"  -> titik lurus sebaiknya {baru:.2f} (sekarang {konfig.STEER_SERVO_CENTER:.2f}).")
        print(f"     Pasang di DUA tempat sekaligus: konfig.ROBOT[{konfig.ROBOT_ID}] dan robot")
        print(f"     (\"set str_center {baru:.2f}\" + \"save\").")
    elif se <= BATAS:
        print(f"  TIDAK ADA pergeseran yang terdeteksi dalam +-{2 * se:.2f} deg.")
        print(f"  STEER_SERVO_CENTER = {konfig.STEER_SERVO_CENTER:.2f} sudah baik.")
    else:
        perlu = math.ceil((sd / BATAS) ** 2 / max(1, len(per_laju)))
        print(f"  BELUM CUKUP TELITI: galat baku {se:.2f} deg masih di atas "
              f"{BATAS:.2f} deg,")
        print(f"  jadi pergeseran sekecil apa pun belum bisa dibedakan dari nol.")
        print(f"  Ulangi dengan --ulang {perlu} (sekarang "
              f"{len(rekaman) // max(1, len(per_laju))}), atau pakai")
        print(f"  tools/kalibrasi_kemudi.py yang memang dibuat untuk ini.")
        print(f"  JANGAN mengubah STEER_SERVO_CENTER berdasarkan angka di atas.")


def cocokkan_busur(rekaman):
    print("\n=== HASIL: penguatan kemudi ===")
    if len(rekaman) < 3:
        print("  data kurang untuk dicocokkan.")
        return
    dm, dn = [], []
    print(f"  {'diminta':>9s} {'nyata':>9s} {'radius':>9s}")
    for r in rekaman:
        _, kk = r.laju_dan_kelengkungan()
        if not np.isfinite(kk) or abs(kk) < 1e-6:
            continue
        d_nyata = math.degrees(math.atan(kk * konfig.WHEELBASE_CM))
        dm.append(r.steer_deg); dn.append(d_nyata)
        print(f"  {r.steer_deg:9.1f} {d_nyata:9.2f} {1.0 / abs(kk):9.1f} cm")
    if len(dm) < 3:
        print("  data kurang.")
        return
    dm, dn = np.array(dm), np.array(dn)
    # penguatan dicocokkan hanya pada sudut kecil, sebelum roda jenuh
    kecil = np.abs(dm) <= 20.0
    if kecil.sum() >= 2:
        g = float(np.polyfit(np.tan(np.radians(dm[kecil])),
                             np.tan(np.radians(dn[kecil])), 1)[0])
        jendela = min(r.jendela_ukur_s() for r in rekaman)
        print(f"  Penguatan kemudi (sudut <= 20 deg): {g:.3f}")
        if jendela < 2.0 * konfig.SIM_KEMUDI_TAU_S:
            print(f"  PERINGATAN: jendela ukur hanya {jendela:.1f} detik, kurang dari "
                  f"2 tau ({2 * konfig.SIM_KEMUDI_TAU_S:.1f} detik).")
            print(f"  Penguatan akan terbaca TERLALU KECIL karena kemudi belum")
            print(f"  selesai bergerak. Ulangi dengan --detik {math.ceil(konfig.SIM_KEMUDI_TUNDA_S + 5 * konfig.SIM_KEMUDI_TAU_S):.0f} atau lebih.")
        else:
            print(f"  -> STEER_SERVO_PER_DEG sebaiknya "
                  f"{konfig.STEER_SERVO_PER_DEG / g:.3f} (sekarang {konfig.STEER_SERVO_PER_DEG:.3f})")
    jenuh = np.nanmax(np.abs(dn))
    print(f"  Sudut nyata terbesar yang tercapai: {jenuh:.1f} deg "
          f"-> MAX_STEER_DEG sebaiknya <= {jenuh:.0f} (sekarang {konfig.MAX_STEER_DEG:.0f})")


def cocokkan_tangga(rekaman):
    print("\n=== HASIL: tunda dan tetapan waktu kemudi ===")
    if not rekaman:
        print("  tidak ada data.")
        return
    r = rekaman[0]
    t_all, _, _, _, cmd = r.arr()
    if len(t_all) < 40:
        print("  data kurang.")
        return
    dt = float(np.median(np.diff(t_all)))
    # Regresi bergerak, bukan beda-hingga: beda-hingga pada pose berderau
    # menghasilkan kelengkungan yang hampir seluruhnya derau, dan puncak
    # korelasi silangnya jadi acak.
    t, v, w = r.laju_yaw_licin()
    if len(t) == 0:
        print("  data kurang.")
        return
    with np.errstate(divide="ignore", invalid="ignore"):
        k = np.where(v > 0.5, w / v, np.nan)
    d_nyata = np.degrees(np.arctan(k * konfig.WHEELBASE_CM))
    c = cmd
    sah = np.isfinite(d_nyata)
    if sah.sum() < 30:
        print("  data sah terlalu sedikit (robot mungkin terlalu lambat).")
        return
    a = c[sah] - np.mean(c[sah])
    b = d_nyata[sah] - np.mean(d_nyata[sah])
    n = min(len(a), int(2.0 / dt))
    kor = [np.corrcoef(a[:len(a) - i], b[i:])[0, 1] if len(a) - i > 10 else -1
           for i in range(n)]
    puncak = int(np.argmax(kor))
    print(f"  Korelasi silang memuncak di {puncak} cuplikan = {puncak * dt:.2f} detik "
          f"(r = {kor[puncak]:.3f})")
    # Pembandingnya HARUS angka yang diukur dengan cara yang sama. v1 mengukur
    # perintah-kemudi vs KELENGKUNGAN NYATA dan mendapat 0,65 detik - itu
    # metode yang persis dipakai di sini. Jangan dibandingkan dengan korelasi
    # perintah-kemudi vs SUDUT KEMUDI (0,60 detik di model): yang terakhir tidak
    # memuat waktu yang dibutuhkan badan robot untuk mulai berbelok, jadi
    # angkanya memang lebih kecil dan membandingkannya menyesatkan.
    print(f"  Acuan v1 di perangkat, metode sama: 0,65 detik (r = 0,94-0,96)")
    print(f"  Model simulasi sekarang: tunda {konfig.SIM_KEMUDI_TUNDA_S:.2f} + "
          f"tau {konfig.SIM_KEMUDI_TAU_S:.2f} detik")
    # Saran "bagi kedua parameter dengan (puncak / 0,65)" yang dulu dicetak di
    # sini TERBALIK arah: puncak yang lebih kecil dari acuan membuatnya
    # membesarkan parameter. Ia juga tidak bisa memisahkan tunda dari tau.
    # Diganti pencocokan yaw di bawah, yang memberi keduanya langsung.
    k = pasang_kemudi(r)
    if k is None:
        return
    print(f"\n  PENCOCOKAN YAW  (yaw terukur vs model kemudi, bukan turunannya):")
    print(f"    tunda total terlihat kamera d = {k['d']:.3f} s,  tau = "
          + (f"{k['tau']:.3f} s" if k["tau"] >= 0.05 else
             f"< 0,05 s (tak terbedakan dari tunda murni)")
          + f"   sisa yaw {k['rms']:.2f} deg")
    print(f"    penguatan kemudi {k['g']:.3f}, titik lurus {k['off']:+.2f} deg "
          f"(dicocokkan agar tidak bocor ke d dan tau; ukur teliti dengan --mode busur/lurus)")
    print(f"    model simulator sekarang (tunda {konfig.SIM_KEMUDI_TUNDA_S:.2f} + tau "
          f"{konfig.SIM_KEMUDI_TAU_S:.2f}) di data ini: sisa yaw {k['rms_konfig']:.2f} deg")
    kam = (konfig.SIM_KAMERA_DELAY_S + 0.5 / konfig.SIM_KAMERA_FPS
           + 0.5 * konfig.SIM_KAMERA_JITTER_S)
    print(f"    d memuat latensi kamera. Simulator memodelkan kamera terpisah (~{kam:.2f} s),")
    print(f"    jadi yang setara untuk simulator: SIM_KEMUDI_TUNDA_S ~ {max(k['d'] - kam, 0):.2f}, "
          f"SIM_KEMUDI_TAU_S ~ {max(k['tau'], 0.02):.2f}")
    # Dua uji 2026-09-27 memberi d + tau 0,41 dan 0,24 s - rincian satu uji
    # tidak bisa langsung jadi nilai konfig. Yang konsisten antar-uji adalah
    # jumlahnya dan kecilnya tau.
    print(f"    SATU uji tidak cukup untuk mengubah konfig: dua uji 2026-09-27 berbeda")
    print(f"    ~0,17 s (jalur perintah berayun). Ulangi, lalu pilih model yang baik")
    print(f"    untuk SEMUA uji - bukan optimum satu uji.")


def pasang_kemudi(rek):
    """Tunda murni dan tetapan waktu kemudi dari YAW terukur uji tangga -> dict/None.

    Model:  delta(t) = orde-1 (tau) dari perintah(t - d)
            yaw'     = v * g * tan(delta + delta0) / L
    d adalah tunda total yang TERLIHAT KAMERA: latensi kamera + jalur perintah
    + tunda murni servo. g dan delta0 ikut dicocokkan supaya penguatan dan titik
    lurus kemudi tidak bocor ke d dan tau.

    Yaw yang dicocokkan, bukan kelengkungan: kelengkungan harus diturunkan dari
    posisi berderau, yaw terukur langsung. Tanggapan orde-1 dihitung analitik
    supaya kesalahan kecil dalam d tetap memberi turunan yang halus.

    Laju diambil dari KAMERA (panjang busur tunak), bukan dari perintah: selisih
    laju akan terserap ke penguatan g.
    """
    t, x, y, yaw, cmd = rek.arr()
    if len(t) < 40:
        return None
    langkah = [(t[i], math.radians(cmd[i] - cmd[i - 1]))
               for i in range(1, len(t)) if cmd[i] != cmd[i - 1]]
    if not langkah:
        return None
    xy = np.column_stack([x, y])
    a = konfig.ACCEL_SPS2 * konfig.CM_PER_STEP
    geser = np.hypot(*(xy - xy[0]).T)
    if not (geser > 0.3).any():
        return None
    awal = t[np.argmax(geser > 0.3)] - math.sqrt(2 * 0.3 / a)
    busur = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    w = t > awal + 1.0
    if w.sum() < 10:
        return None
    v_ss = float(np.polyfit(t[w], busur[w], 1)[0])
    h = 0.005
    tt = np.arange(0.0, t[-1] + h, h)
    v = np.clip((tt - awal) * a, 0.0, v_ss)

    def yaw_model(d, tau, g, off, yaw0):
        delta = np.zeros_like(tt)
        for tk, dk in langkah:
            s = tt - tk - d
            m = s > 0
            delta[m] += dk * (1.0 - np.exp(-s[m] / max(tau, 1e-4)))
        laju = v * g * np.tan(delta + math.radians(off)) / konfig.WHEELBASE_CM
        yt = yaw0 + np.concatenate([[0.0], np.cumsum(0.5 * (laju[1:] + laju[:-1]) * h)])
        return np.interp(t, tt, yt)

    terbaik = None
    for d0 in (0.1, 0.4, 0.8):
        for tau0 in (0.05, 0.3, 0.8):
            r = least_squares(lambda th: yaw_model(*th) - yaw, [d0, tau0, 1.0, 0.0, yaw[0]],
                              bounds=([0.0, 1e-4, 0.5, -8.0, -np.inf],
                                      [2.0, 2.0, 1.5, 8.0, np.inf]))
            if terbaik is None or r.cost < terbaik.cost:
                terbaik = r
    d, tau, g, off, _ = terbaik.x
    rms = float(np.degrees(np.sqrt(np.mean((yaw_model(*terbaik.x) - yaw) ** 2))))

    # model konfig sekarang, d & tau dikunci, g & titik lurus tetap dicocokkan
    rk = least_squares(lambda th: yaw_model(konfig.SIM_KEMUDI_TUNDA_S, konfig.SIM_KEMUDI_TAU_S,
                                            *th) - yaw,
                       [1.0, 0.0, yaw[0]], bounds=([0.5, -8.0, -np.inf], [1.5, 8.0, np.inf]))
    rms_k = float(np.degrees(np.sqrt(np.mean(rk.fun ** 2))))
    return dict(d=float(d), tau=float(tau), g=float(g), off=float(off), rms=rms,
                rms_konfig=rms_k, v=v_ss)


def _s_ramp(t, peristiwa, a_naik, a_turun, cm_per_step, s0):
    """Posisi sepanjang lintasan (cm) pada waktu t, menurut model ramp.

    peristiwa = [(waktu, laju_sps), ...]: laju target berlaku SEJAK waktunya,
    sebelum yang pertama robot diam. Laju langkah menanjak ke target dengan
    a_naik bila naik, a_turun bila turun (sps/detik^2).

    ANALITIK, bukan integrasi Euler. Dengan Euler, posisi menjadi fungsi TANGGA
    dari tunda pada skala langkah integrasi, sehingga turunan numerik terhadap
    tunda bernilai nol di hampir semua titik dan pencocok berhenti di tempat.
    Laju ramp linear sepotong-sepotong, jadi posisinya kuadrat sepotong-sepotong
    dan bisa dihitung tepat.
    """
    t_lo, t_hi = float(t.min()) - 1.0, float(t.max()) + 1.0
    tn, vn = [t_lo], [0.0]

    def simpul(tt, vv):
        if tt > tn[-1] + 1e-12:
            tn.append(tt)
            vn.append(vv)

    v = 0.0
    for j, (tj, L) in enumerate(peristiwa):
        tj = min(max(tj, t_lo), t_hi)
        simpul(tj, v)                                   # datar sampai peristiwa
        t_next = peristiwa[j + 1][0] if j + 1 < len(peristiwa) else t_hi
        t_next = min(max(t_next, tj), t_hi)
        if L == v:
            continue
        r = a_naik if L > v else a_turun
        tau = abs(L - v) / r
        if tj + tau <= t_next:
            v = float(L)
            simpul(tj + tau, v)
        else:                                           # disela peristiwa berikut
            v = v + math.copysign(r * (t_next - tj), L - v)
            simpul(t_next, v)
    simpul(t_hi, v)

    tn, vn = np.array(tn), np.array(vn)
    luas = np.concatenate([[0.0], np.cumsum(0.5 * (vn[1:] + vn[:-1]) * np.diff(tn))])
    k = np.clip(np.searchsorted(tn, t, side="right") - 1, 0, len(tn) - 2)
    vq = np.interp(t, tn, vn)
    return s0 + cm_per_step * (luas[k] + 0.5 * (vn[k] + vq) * (t - tn[k]))


def pasang_gas(rek):
    """Cocokkan model ramp ke posisi terukur satu lintasan gas.

    Yang dicocokkan POSISI, bukan kecepatan. Kecepatan dari kamera harus
    diturunkan dari posisi yang berderau, dan terbukti bias: pada robot yang
    DIAM, laju dari busur terbaca p95 5,75 cm/detik. Posisi terukur langsung.

    Enam parameter:
      s0            posisi awal
      rho           skala cm/langkah efektif / CM_PER_STEP (selip bila < 1)
      a_naik        akselerasi saat laju naik, sps/detik^2
      a_turun       akselerasi saat laju turun
      tunda_naik    perintah naik dikirim -> gerak terlihat kamera
      tunda_turun   perintah turun dikirim -> perlambatan terlihat kamera

    Tunda dan akselerasi WAJIB dicocokkan bersama. Keduanya membuat robot
    "terlambat"; kalau tunda tidak punya parameter sendiri, ia terserap menjadi
    akselerasi yang terlalu rendah. Tunda di sini TOTAL: MQTT + firmware +
    kamera. Latensi kamera saja sudah 25-65 ms, jadi tunda nol tidak mungkin.
    """
    t = np.array(rek.t, dtype=float)
    xy = np.column_stack([rek.x, rek.y]).astype(float)
    if len(t) < 30 or len(rek.peristiwa) < 2:
        return None

    # Arah lintasan dari komponen utama posisi, BUKAN dari yaw awal: yaw satu
    # frame berderau ~0,7 derajat, sedangkan komponen utama memakai seluruh
    # lintasan. Kemudi 0 berarti lurus, jadi proyeksi = jarak tempuh.
    pusat = xy.mean(axis=0)
    _, _, vt = np.linalg.svd(xy - pusat, full_matrices=False)
    s = (xy - pusat) @ vt[0]
    if s[-1] < s[0]:
        s = -s
    lateral = (xy - pusat) @ vt[1]

    (t1, lv1), (t2, lv2) = rek.peristiwa[:2]
    diam = t < t1
    derau = float(np.std(s[diam], ddof=1)) if diam.sum() >= 3 else float("nan")

    # Parameter diskalakan agar semuanya berorde satuan: akselerasi dalam
    # ribuan sps/s^2, skala sebagai rasio terhadap konfig.
    def model(th):
        s0, rho, an, at, tdn, tdt = th
        return _s_ramp(t, [(t1 + tdn, lv1), (t2 + tdt, lv2)],
                       an * 1000.0, at * 1000.0, rho * konfig.CM_PER_STEP, s0)

    # Batas atas tunda 3 detik. Semula 1 detik, dengan anggapan tunda paling-
    # paling puluhan milidetik - dan jalan nyata pertama (2026-09-27) langsung
    # melampauinya: robot baru bergerak 0,9-1,2 detik setelah perintah. Tunda
    # yang mentok di batas memaksa sisa keterlambatannya diserap akselerasi,
    # dan program lalu menyimpulkan "akselerasi melemah" yang keliru.
    bawah = [-np.inf, 0.5, 0.2, 0.2, 0.0, 0.0]
    atas = [np.inf, 1.5, 100.0, 100.0, 3.0, 3.0]
    terbaik = None
    # Beberapa titik awal: kalau akselerasi nyata jauh dari setelan, satu
    # titik awal di setelan bisa terjebak minimum lokal tunda<->akselerasi.
    for an0 in (3.0, konfig.ACCEL_SPS2 / 1000.0, 30.0):
        for td0 in (0.05, 0.5, 1.2):
            x0 = [float(s[0]), 1.0, an0, an0, td0, td0]
            r = least_squares(lambda th: model(th) - s, x0, bounds=(bawah, atas))
            if terbaik is None or r.cost < terbaik.cost:
                terbaik = r

    th = terbaik.x
    sisa = model(th) - s
    n, p = len(s), len(th)
    s2 = float(np.sum(sisa ** 2)) / max(n - p, 1)
    J = terbaik.jac
    kov = np.linalg.pinv(J.T @ J) * s2
    galat = np.sqrt(np.clip(np.diag(kov), 0.0, None))

    def kor(i, j):
        d = math.sqrt(kov[i, i] * kov[j, j])
        return float(kov[i, j] / d) if d > 0 else float("nan")

    def lambat(i_a, i_td, lompat):
        """Keterlambatan total L = tunda + lompat / (2 A), dengan galat bakunya.

        Galat baku lewat metode delta, MEMAKAI kovarians a<->tunda. Justru
        karena keduanya berkorelasi hampir sempurna, galat L jauh lebih kecil
        daripada galat a atau tunda sendiri-sendiri: kesalahan yang satu
        dikompensasi yang lain, dan jumlahnya tetap terkunci oleh data.
        """
        a = th[i_a] * 1000.0
        L = th[i_td] + lompat / (2.0 * a)
        g = np.zeros(p)
        g[i_td] = 1.0
        g[i_a] = -lompat / (2.0 * 1000.0 * th[i_a] ** 2)
        return float(L), float(math.sqrt(max(g @ kov @ g, 0.0)))

    L_naik, L_naik_se = lambat(2, 4, lv1)
    L_turun, L_turun_se = lambat(3, 5, lv1 - lv2)

    # Autokorelasi sisa lag-1: pembeda galat SISTEMATIS dari derau. Derau
    # kamera dan jitter waktu frame tidak berurutan; model yang salah bentuk
    # menghasilkan deret sisa bertanda sama. Membandingkan besar sisa dengan
    # derau saat DIAM tidak bisa dipakai - saat bergerak, jitter waktu frame
    # menjadi derau posisi sebesar laju x jitter, yang tidak ada saat diam.
    autokor = (float(np.corrcoef(sisa[:-1], sisa[1:])[0, 1])
               if np.std(sisa) > 1e-9 else 0.0)

    # AWAL GERAK TANPA MODEL. Robot yang menanjak lambat tetap mulai bergerak
    # segera - hanya pelan. Jadi awal gerak yang terlambat adalah tunda murni,
    # apa pun akselerasinya. Angka ini tetap sah ketika model ramp gagal, dan
    # pada jalan nyata pertama justru angka inilah yang paling jelas.
    # Ambang minimal 0,3 cm: pada ramp 12000 itu tercapai ~0,14 s setelah gerak
    # sebenarnya dimulai, jadi angka ini sedikit MELEBIHKAN tunda.
    ambang = max(4.0 * derau, 0.3) if np.isfinite(derau) else 0.3
    turun_idx = int(np.searchsorted(t, t2))
    naik_ok = (s - s[diam].mean()) > ambang if diam.any() else np.zeros_like(s, bool)
    gerak_naik = next((t[k] - t1 for k in range(len(s))
                       if t[k] >= t1 and naik_ok[k:turun_idx].all() and k < turun_idx),
                      float("nan"))
    w = (t > t2 - 1.0) & (t < t2)
    gerak_turun = float("nan")
    if w.sum() >= 5:
        kem, icp = np.polyfit(t[w], s[w], 1)
        kurang = (kem * t + icp - s) > ambang
        gerak_turun = next((t[k] - t2 for k in range(turun_idx, len(s))
                            if kurang[k:].all()), float("nan"))

    return dict(
        target=int(lv1), rendah=int(lv2),
        rho=th[1], rho_se=galat[1],
        L_naik=L_naik, L_naik_se=L_naik_se,
        L_turun=L_turun, L_turun_se=L_turun_se,
        # Dicatat untuk diagnosis; tidak dipakai menyimpulkan - lihat kor_naik.
        a_naik=th[2] * 1000.0, tunda_naik=th[4],
        a_turun=th[3] * 1000.0, tunda_turun=th[5],
        kor_naik=kor(2, 4), kor_turun=kor(3, 5),
        # Laju tunak harus tercapai sebelum fase berikutnya, kalau tidak rumus
        # L tidak berlaku dan skala cm/langkah ikut kacau.
        tunak_naik=(th[4] + lv1 / (th[2] * 1000.0)) < (t2 - t1),
        di_batas=[i for i in range(1, p)
                  if abs(th[i] - bawah[i]) < 1e-6 or abs(th[i] - atas[i]) < 1e-6],
        # Batas yang benar-benar MERUSAK L: skala (garis tunak salah kemiringan)
        # atau tunda di batas ATAS (keterlambatan terpotong). Akselerasi yang
        # mentok TIDAK: itu hanya berarti ramp lintasan ini terlalu pendek untuk
        # dipisahkan dari tunda - persis keadaan yang membuat L dirancang, dan
        # L tetap terkunci oleh garis tunak. Versi sebelumnya menolak garis
        # yang sah karena a_naik lintasan 2100 mentok (ramp 0,175 s).
        merusak_L=[n for i, n in ((1, "rho"), (4, "tunda_naik"), (5, "tunda_turun"))
                   if abs(th[i] - atas[i]) < 1e-6
                   or (i == 1 and abs(th[i] - bawah[i]) < 1e-6)],
        sisa_rms=float(np.sqrt(np.mean(sisa ** 2))),
        autokor=autokor,
        gerak_naik=gerak_naik, gerak_turun=gerak_turun, ambang=ambang,
        derau=derau,
        lateral_maks=float(np.max(np.abs(lateral - lateral.mean()))),
        n=n,
    )


def muat_gas(folder):
    """Muat ulang lintasan gas tersimpan -> daftar Rekaman. Tanpa robot, tanpa kamera.

    Waktu kirim perintah diambil dari gas_XXXX_perintah.csv bila ada. Rekaman
    yang lebih tua tidak punya berkas itu; waktunya lalu diperkirakan dari
    cuplikan TEREKAM pertama dengan perintah baru, yang bisa terlambat satu
    siklus loop (~60 ms) karena cuplikan pose ramalan tidak direkam. Terlambat
    berarti tunda terbaca sedikit LEBIH KECIL dari sebenarnya.
    """
    hasil, perkiraan = [], []
    # Hanya gas_<angka>.csv - berkas samping (_perintah, _telemetri) ikut
    # cocok dengan "gas_*.csv" dan bukan lintasan.
    berkas = [p for p in glob.glob(os.path.join(folder, "gas_*.csv"))
              if re.fullmatch(r"gas_\d+\.csv", os.path.basename(p))]
    for path in sorted(berkas, key=lambda p: int(os.path.basename(p)[4:-4])):
        with open(path) as fh:
            baris = list(csv.DictReader(fh))
        if not baris or "cmd_sps" not in baris[0]:
            continue
        nama = os.path.basename(path)[:-4]
        rek = Rekaman(nama, int(float(baris[0]["sps"])), 0.0, kolom_cmd="cmd_sps")
        for r in baris:
            rek.t.append(float(r["t"]))
            rek.x.append(float(r["x_cm"]))
            rek.y.append(float(r["y_cm"]))
            rek.yaw.append(float(r["yaw_deg"]))
            rek.cmd.append(float(r["cmd_sps"]))
        samping = path[:-4] + "_perintah.csv"
        if os.path.exists(samping):
            with open(samping) as fh:
                rek.peristiwa = [(float(r["t"]), int(float(r["sps"])))
                                 for r in csv.DictReader(fh)]
        else:
            rek.peristiwa, lalu = [], 0
            for t, c in zip(rek.t, rek.cmd):
                if c != lalu:
                    rek.peristiwa.append((t, int(c)))
                    lalu = c
            perkiraan.append(nama)
        rek.telemetri = []
        tel = path[:-4] + "_telemetri.csv"
        if os.path.exists(tel):
            with open(tel) as fh:
                for r in csv.DictReader(fh):
                    rek.telemetri.append((float(r["t"]), {k: r.get(k, "") for k in KOLOM_TELEMETRI}))
        hasil.append(rek)
    return hasil, perkiraan


def _angka(d, kunci):
    try:
        return float(d.get(kunci, ""))
    except (TypeError, ValueError):
        return float("nan")


def urai_telemetri(rek):
    """Kapan firmware MENERIMA tiap perintah dan kapan stepper MULAI -> dict atau None.

    Semua waktu relatif terhadap perintah DIKIRIM, dan semuanya waktu TERIMA
    telemetri di PC. Dua akibatnya:
      - telemetri 4 Hz, jadi tiap angka bisa terlambat sampai ~0,25 s dari
        kejadian sebenarnya (batas ATAS, bukan nilai tepat);
      - "diterima" adalah bolak-balik: perintah ke firmware, lalu telemetri
        kembali ke PC lewat broker yang sama.
    """
    tl = getattr(rek, "telemetri", None)
    if not tl or len(rek.peristiwa) < 2:
        return None
    (t1, lv1), (t2, lv2) = rek.peristiwa[:2]

    def pertama(t_mulai, syarat, prasyarat):
        """Selang sampai telemetri pertama yang memenuhi syarat.

        prasyarat diperiksa pada telemetri TERAKHIR sebelum t_mulai. Kalau
        keadaan sesudahnya sudah terpenuhi sejak sebelum perintah dikirim -
        mis. tgt_sps sisa perintah lama kebetulan sama dengan target - maka
        tidak ada perubahan yang bisa diukur, dan hasilnya NaN, bukan 0.
        """
        sebelum = [d for t, d in tl if t < t_mulai]
        if sebelum and not prasyarat(sebelum[-1]):
            return float("nan")
        return next((t - t_mulai for t, d in tl if t >= t_mulai and syarat(d)), float("nan"))

    sps = lambda d: abs(_angka(d, "sps"))
    tgt = lambda d: _angka(d, "tgt_sps")
    t_tl = np.array([t for t, _ in tl])
    return dict(
        terima_naik=pertama(t1, lambda d: tgt(d) == lv1, lambda d: tgt(d) != lv1),
        mulai_naik=pertama(t1, lambda d: sps(d) > 0, lambda d: sps(d) == 0),
        terima_turun=pertama(t2, lambda d: tgt(d) == lv2, lambda d: tgt(d) != lv2),
        mulai_turun=pertama(t2, lambda d: sps(d) < 0.95 * lv1, lambda d: sps(d) >= 0.95 * lv1),
        selang=float(np.median(np.diff(t_tl))) if len(t_tl) > 1 else float("nan"),
        n=len(tl),
    )


def _garis(x, y, se):
    """Regresi linear berbobot y = b0 + b1 x -> (b, galat_b, chi2, dof)."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    w = 1.0 / np.maximum(np.asarray(se, float), 1e-4) ** 2
    X = np.column_stack([np.ones_like(x), x])
    kov = np.linalg.inv(X.T @ (X * w[:, None]))
    b = kov @ X.T @ (w * y)
    chi2 = float(np.sum(w * (y - X @ b) ** 2))
    return b, np.sqrt(np.diag(kov)), chi2, len(x) - 2


def waktu_stepper(rek):
    """Kapan stepper BENAR-BENAR mulai untuk tiap perintah, dari kurva pos -> dict/None.

    pos di telemetri = langkah yang sudah dikeluarkan stepper, tepat saat
    diterbitkan. Stepper mengikuti ramp setelan dengan persis (FastAccelStepper
    menghitung tiap langkah dari ACCEL_SPS2), jadi kurva pos hanya punya satu
    yang tidak diketahui per perintah: kapan ia mulai. Itu dicocokkan pada
    SELURUH kurva - jauh lebih teliti daripada membaca "cuplikan pertama yang
    berubah", yang terikat selang telemetri 0,25 s. Versi pertama laporan ini
    melakukan yang kedua, dan membaca 0,57 s untuk stepper yang sebenarnya
    mulai ~0,33 s.

    Waktunya relatif terhadap perintah DIKIRIM, dan termasuk latensi telemetri
    kembali ke PC - karena pos dicap waktu saat DITERIMA di PC.
    """
    tl = getattr(rek, "telemetri", None)
    if not tl or len(rek.peristiwa) < 2 or not rek.t:
        return None
    # Hanya selama lintasan: sesudahnya estop menghentikan stepper mendadak,
    # dan itu bukan bagian dari profil yang dimodelkan.
    tl = [(t, d) for t, d in tl if t <= rek.t[-1]]
    tt = np.array([t for t, _ in tl])
    pos = np.array([_angka(d, "pos") for _, d in tl])
    ok = np.isfinite(pos)
    tt, pos = tt[ok], pos[ok]
    if len(tt) < 8:
        return None
    P = pos - pos[0]
    if P[-1] < 0:
        P = -P
    (t1, lv1), (t2, lv2) = rek.peristiwa[:2]
    A = konfig.ACCEL_SPS2

    def model(th, t=tt):
        return _s_ramp(t, [(t1 + th[1], lv1), (t2 + th[2], lv2)], A, A, 1.0, th[0])

    terbaik = None
    for d0 in (0.05, 0.5, 1.2):
        r = least_squares(lambda th: model(th) - P, [float(P[0]), d0, d0],
                          bounds=([-np.inf, -0.5, -0.5], [np.inf, 3.0, 3.0]))
        if terbaik is None or r.cost < terbaik.cost:
            terbaik = r
    th = terbaik.x
    sisa = model(th) - P
    # Pesan yang menumpuk lalu tiba sekaligus: tanda aliran MQTT atau PC sempat
    # tersendat. Selang normal 0,25 s; di jalan nyata pertama 15 pesan tiba
    # dalam 0,12 s tepat sebelum perintah naik yang terlambat ~0,3 s.
    dt = np.diff(np.array([t for t, _ in getattr(rek, "telemetri")]))
    rapat = int(np.sum(dt < 0.02))
    return dict(mulai_naik=float(th[1]), mulai_turun=float(th[2]),
                sisa_langkah=float(np.sqrt(np.mean(sisa ** 2))),
                langkah=lambda t: model([0.0, th[1], th[2]], np.asarray(t, float)),
                menumpuk=rapat,
                t_menumpuk=(float(np.array([t for t, _ in rek.telemetri])[1:][dt < 0.02][0])
                            if rapat else float("nan")))


def latensi_kamera(rek, ws):
    """Geser kurva kamera sampai berimpit dengan kurva langkah -> (geser, cm/langkah, sisa).

    Kalau tidak ada langkah hilang, posisi robot = langkah x CM_PER_STEP; kamera
    melihat posisi yang sama, hanya terlambat. Geseran terbaik = latensi kamera
    DIKURANGI latensi telemetri kembali ke PC (karena kurva langkah dicap waktu
    terima), jadi ia batas BAWAH latensi kamera.

    Geseran NEGATIF berarti kamera melihat gerak sebelum langkahnya tercatat -
    hanya mungkin bila telemetrinya yang terlambat tiba di PC.

    Hasilnya sekaligus menguji selip: pergeseran WAKTU murni menghasilkan
    cm/langkah = CM_PER_STEP dan sisa kecil. Langkah yang hilang (selip, stall)
    membuat kamera menempuh jarak lebih pendek, dan tidak ada geseran waktu yang
    bisa menutupinya.
    """
    t = np.array(rek.t, float)
    xy = np.column_stack([rek.x, rek.y]).astype(float)
    c = xy.mean(axis=0)
    _, _, vt = np.linalg.svd(xy - c, full_matrices=False)
    s = (xy - c) @ vt[0]
    if s[-1] < s[0]:
        s = -s
    terbaik = None
    for geser in np.arange(-1.5, 2.0, 0.005):
        L = ws["langkah"](t - geser)
        X = np.column_stack([np.ones_like(L), L])
        b, *_ = np.linalg.lstsq(X, s, rcond=None)
        rms = float(np.sqrt(np.mean((s - X @ b) ** 2)))
        if terbaik is None or rms < terbaik[2]:
            terbaik = (float(geser), float(b[1]), rms)
    return terbaik


def _lapor_telemetri(pasangan):
    """Urai tunda per tahap: dikirim -> stepper mulai -> terlihat kamera.

    Dua pengukuran presisi dari kolom pos, bukan pembacaan cuplikan 4 Hz:
      stepper mulai  = kapan langkah benar-benar keluar (jaringan + firmware)
      latensi kamera = seberapa jauh kamera tertinggal dari langkah itu
    Keduanya memuat latensi telemetri kembali ke PC dengan tanda berlawanan,
    jadi jumlahnya bebas dari latensi itu - dan harus kira-kira sama dengan
    AWAL GERAK TANPA MODEL di atas.

    Pemeriksaan pertama sengaja soal ARAH: kalau kamera "mendahului" langkah
    (geseran negatif), telemetrinyalah yang terlambat, dan angka stepper tidak
    boleh dibaca sebagai waktu firmware.

    Untuk memisahkan jaringan dari firmware dipakai kapan tgt_sps berubah:
    firmware yang lambat tetap MENERIMA perintah cepat (tgt_sps berubah dini)
    tapi baru melangkah kemudian.
    """
    baris = []
    for r, h in pasangan:
        ws = waktu_stepper(r)
        if ws is None:
            continue
        geser, cpl, sisa = latensi_kamera(r, ws)
        baris.append((h, ws, urai_telemetri(r), geser, cpl, sisa))
    if not baris:
        return

    print(f"\n  TELEMETRI FIRMWARE  (dicocokkan dari hitungan langkah 'pos' seluruh lintasan)")
    print(f"  {'target':>7} {'stepper mulai':>21} {'latensi':>9} {'cm/langkah':>18} {'sisa':>6}")
    print(f"  {'':>7} {'naik':>10} {'turun':>10} {'kamera':>9}")
    for h, ws, u, geser, cpl, sisa in baris:
        print(f"  {h['target']:>7d} {ws['mulai_naik']:>9.3f}s {ws['mulai_turun']:>9.3f}s "
              f"{geser:>8.3f}s {cpl:.6f} ({100 * (cpl / konfig.CM_PER_STEP - 1):+5.1f}%) "
              f"{sisa:>5.2f}")
    for h, ws, *_ in baris:
        if ws["menumpuk"] >= 3:
            print(f"  [!] target {h['target']}: {ws['menumpuk']} pesan telemetri tiba hampir "
                  f"bersamaan sekitar t={ws['t_menumpuk']:.2f} s - aliran MQTT atau PC")
            print(f"      sempat tersendat, pesan menumpuk lalu tiba sekaligus.")

    geser_m = float(np.median([b[3] for b in baris]))
    murni = all(abs(b[4] / konfig.CM_PER_STEP - 1) < 0.03 and b[5] < 0.5 for b in baris)
    print()
    if geser_m < -0.2:
        print(f"  Kamera melihat gerak SEBELUM langkahnya tercatat ({geser_m:+.2f} s).")
        print(f"  Itu hanya mungkin bila TELEMETRI terlambat tiba di PC: jalur ESP32 ->")
        print(f"  broker -> PC lambat. Waktu 'stepper mulai' ikut tergeser, jadi tidak")
        print(f"  bisa dibaca sebagai waktu firmware.")
        return

    ada = False
    if geser_m > 0.2:
        ada = True
        print(f"  KAMERA terlambat ~{geser_m:.2f} s dari langkah yang sebenarnya "
              f"(lintasan: {', '.join('%.2f' % b[3] for b in baris)} s).")
        if murni:
            print(f"  Pergeseran WAKTU murni: cm/langkah cocok dan sisanya kecil, jadi tidak")
            print(f"  ada langkah hilang - robotnya bergerak tepat waktu, kameranya yang")
            print(f"  menayangkan terlambat.")
        v = konfig.FORWARD_SPEED * konfig.CM_PER_STEP
        print(f"  Setiap pose yang dipakai navigasi setua itu: pada {v:.1f} cm/detik robot")
        print(f"  sudah {v * geser_m:.1f} cm lebih jauh dari yang dilihat pengendali. Ini")
        print(f"  batas BAWAH - latensi telemetri kembali ke PC belum ditambahkan.")

    for h, ws, u, *_ in baris:
        for arah in ("naik", "turun"):
            S = ws[f"mulai_{arah}"]
            if S <= 0.25:
                continue
            ada = True
            terima = u[f"terima_{arah}"] if u else float("nan")
            if np.isfinite(terima) and S - terima > 0.1:
                print(f"  Target {h['target']} {arah}: firmware MENERIMA perintah dalam "
                      f"<= {terima:.2f} s, tapi stepper baru MULAI {S:.2f} s -")
                print(f"    tundanya di firmware atau driver (driveRc, FastAccelStepper, "
                      f"TMC2209), bukan di jaringan.")
            else:
                print(f"  Target {h['target']} {arah}: Firmware baru MENERIMA perintah "
                      f"~{S:.2f} s setelah dikirim, dan stepper langsung mulai -")
                print(f"    tundanya di jalur PC -> broker -> ESP32 atau di loop MQTT "
                      f"firmware.")
    if not ada:
        print(f"  Tidak ada tunda besar: stepper mulai dalam 0,25 s dan kamera tertinggal "
              f"{geser_m:.2f} s.")


def cocokkan_gas(rekaman):
    print("\n=== HASIL: akselerasi motor (tangga gas) ===")
    pasangan = [(r, h) for r in rekaman for h in [pasang_gas(r)] if h is not None]
    hasil = [h for _, h in pasangan]
    if not hasil:
        print("  tidak ada data.")
        return
    setel = konfig.ACCEL_SPS2

    print(f"  setelan NVS = konfig.ACCEL_SPS2 = {setel:.0f} sps/s^2")

    # ---- per lintasan: yang terukur baik adalah keterlambatan TOTAL
    print(f"\n  KETERLAMBATAN TOTAL per lintasan  (L = tunda + lompatan / 2A):")
    print(f"  {'target':>7} {'L naik':>16} {'L turun':>16} {'cm/langkah':>19} "
          f"{'sisa':>6} {'autokor':>8}")
    for h in hasil:
        cpl = h["rho"] * konfig.CM_PER_STEP
        print(f"  {h['target']:>7d} {h['L_naik']:>7.3f} +-{h['L_naik_se']:<5.3f} s "
              f"{h['L_turun']:>7.3f} +-{h['L_turun_se']:<5.3f} s "
              f"{cpl:.6f} ({100 * (h['rho'] - 1):+5.1f}%) {h['sisa_rms']:>5.2f} "
              f"{h['autokor']:>+8.2f}")

    # Kecocokan model itu sendiri dulu: angka di atas tidak berarti apa pun
    # kalau model ramp tidak cocok dengan lintasannya.
    nama_param = ["", "rho", "a_naik", "a_turun", "tunda_naik", "tunda_turun"]
    for h in hasil:
        cacat = []
        # Pemeriksaan KASAR - untuk kegagalan besar di tengah lintasan (stall,
        # selip mendadak). Ia TIDAK menangkap akselerasi yang melemah dengan
        # laju: dalam satu lintasan ramp seperti itu masih cukup mirip ramp
        # tetap. Di uji-mandiri, lintasan dengan akselerasi melemah hanya
        # berautokorelasi +0,42, sama dengan lintasan yang benar (+0,39).
        # Kelemahan itu ditangkap uji chi2 antar-target di bawah.
        if h["autokor"] > 0.5:
            cacat.append(f"sisa berautokorelasi {h['autokor']:+.2f} - galat SISTEMATIS, "
                         f"model ramp tetap tidak cocok dengan lintasan ini")
        if not h["tunak_naik"]:
            cacat.append("laju tunak tidak tercapai sebelum fase turun - "
                         "L naik dan skala tidak sah")
        if h["merusak_L"]:
            cacat.append("parameter menyentuh batas pencocokan: "
                         + ", ".join(h["merusak_L"]) + " - L lintasan ini tidak sah")
        elif h["di_batas"]:
            cacat.append("akselerasi menyentuh batas pencocokan ("
                         + ", ".join(nama_param[i] for i in h["di_batas"])
                         + ") - ramp terlalu pendek untuk dipisahkan dari tunda di "
                           "lintasan ini; L tetap sah")
        if h["lateral_maks"] > 5.0:
            cacat.append(f"lintasan menyimpang {h['lateral_maks']:.1f} cm dari garis "
                         f"lurus - proyeksi jarak kurang tepat")
        for c in cacat:
            print(f"  [!] target {h['target']}: {c}")

    # ---- awal gerak TANPA MODEL: tetap sah walau model ramp gagal
    # Tanpa tunda, ramp setelan melewati ambang deteksi dalam tau_amb detik
    # (posisi = 1/2 a t^2). Awal gerak yang jauh melampaui itu = tunda murni.
    a_cm = setel * konfig.CM_PER_STEP
    print(f"\n  AWAL GERAK TANPA MODEL  (perintah dikirim -> pertama terlihat di kamera):")
    print(f"  {'target':>7} {'gerak':>9} {'perlambatan':>12}   tanpa tunda mestinya")
    murni_n, murni_t = [], []
    for h in hasil:
        tau = math.sqrt(2.0 * h["ambang"] / a_cm)
        murni_n.append(h["gerak_naik"] - tau)
        murni_t.append(h["gerak_turun"] - tau)
        print(f"  {h['target']:>7d} {h['gerak_naik']:>7.3f} s {h['gerak_turun']:>10.3f} s"
              f"   ~{tau:.2f} s (ramp {setel:.0f} melewati ambang {h['ambang']:.1f} cm)")
    murni_n, murni_t = np.array(murni_n), np.array(murni_t)
    sah_n = murni_n[np.isfinite(murni_n)]
    sah_t = murni_t[np.isfinite(murni_t)]
    if len(sah_n):
        print(f"  -> tunda murni naik {sah_n.min():.2f}-{sah_n.max():.2f} s"
              + (f", turun {sah_t.min():.2f}-{sah_t.max():.2f} s" if len(sah_t) else ""))
    # Tunda dianggap TETAP bila sebarannya tidak lebih dari ~2 frame. Garis L di
    # bawah MENGANDAIKAN tunda tetap; kalau tidak, intersep dan kemiringannya
    # tidak bermakna - itulah yang terjadi pada jalan nyata pertama.
    tunda_tetap = all(len(x) < 2 or np.ptp(x) <= 0.15 for x in (sah_n, sah_t))
    # Tanpa telemetri, data posisi saja tidak bisa memilah tundanya. Dengan
    # telemetri, bagian TELEMETRI FIRMWARE di bawah memilahnya - paragraf ini
    # lalu hanya menakut-nakuti: tunda wajar sesudah perbaikan kamera
    # (~0,13 s perintah + ~0,09 s kamera) sudah melewati ambang 0,15 s.
    ada_telemetri = any(getattr(r, "telemetri", None) for r, _ in pasangan)
    if (not ada_telemetri and len(sah_n)
            and max(sah_n.max(), sah_t.max() if len(sah_t) else 0) > 0.15):
        print(f"  Tunda ini jauh melebihi yang bisa dijelaskan akselerasi berapa pun.")
        print(f"  Sumbernya jalur perintah (PC -> MQTT -> firmware -> motor) ATAU kamera;")
        print(f"  data posisi saja tidak bisa memisahkan keduanya. Telemetri firmware bisa:")
        print(f"  tgt_sps berubah saat perintah DITERIMA, sps saat stepper MULAI.")
    if not tunda_tetap:
        print(f"  [!] tunda BERBEDA antar-lintasan (sebaran > 0,15 s) - tunda tidak tetap.")

    _lapor_telemetri(pasangan)

    if len(hasil) < 3:
        print(f"\n  Hanya {len(hasil)} lintasan sah - tunda dan akselerasi butuh >= 3")
        print(f"  target untuk dipisahkan (dan >= 4 untuk menguji apakah akselerasi tetap).")
        return

    # ---- antar-lintasan: L linear terhadap lompatan -> tunda & akselerasi
    print(f"\n  PEMISAHAN dari variasi antar-target  (L = tunda + (1/2A) x lompatan):")
    kesimpulan = {}
    for arah, kunci, lompat in (
            ("naik", "L_naik", [h["target"] for h in hasil]),
            ("turun", "L_turun", [h["target"] - h["rendah"] for h in hasil])):
        L = [h[kunci] for h in hasil]
        se = [h[kunci + "_se"] for h in hasil]
        (td, kemir), (td_se, kemir_se), chi2, dof = _garis(lompat, L, se)
        if kemir <= 0:
            print(f"    {arah:<6}: kemiringan {kemir:+.2e} tidak positif - L tidak naik "
                  f"seiring lompatan, model ramp tidak berlaku")
            continue
        A = 1.0 / (2.0 * kemir)
        A_se = A * kemir_se / kemir
        uji = f"chi2/dof {chi2 / dof:.1f}" if dof > 0 else "chi2 (dof 0)"
        print(f"    {arah:<6}: tunda {td:.3f} +- {td_se:.3f} s | "
              f"A {A:6.0f} +- {A_se:<5.0f} sps/s^2 ({A / setel:.2f} x setelan) | {uji}")
        # Tunda negatif mustahil secara fisik - robot tidak bergerak sebelum
        # diperintah. Muncul bila L tidak linear terhadap lompatan: garisnya
        # dipaksa lurus lewat titik-titik yang melengkung, dan intersepnya
        # terdorong ke bawah nol. Uji-mandiri dengan akselerasi yang melemah
        # di laju tinggi menghasilkan persis ini (-0,106 s arah turun).
        if td < -2.0 * td_se:
            print(f"    [!] {arah}: tunda NEGATIF - tidak fisik. Garis L tidak berlaku;")
            print(f"        angka tunda dan A arah ini jangan dipakai.")
        # Setiap arah diperiksa sendiri. Versi sebelumnya hanya menafsirkan
        # arah naik, sehingga garis turun yang bengkok (chi2/dof 49 pada jalan
        # 2026-09-27, karena tunda perintah turun 0,05-0,16 s berbeda antar-
        # lintasan) tercetak "A 4934 = 0,41 x setelan" tanpa peringatan - padahal
        # kurva kamera berimpit dengan ramp 12000 sampai 1,4 mm, termasuk saat
        # melambat.
        elif dof > 0 and chi2 > {1: 6.63, 2: 9.21}.get(dof, 3.0 * dof):
            print(f"    [!] {arah}: garis tidak lurus (chi2 {chi2:.1f}) - tunda arah ini "
                  f"berbeda antar-lintasan;")
            print(f"        angka tunda dan A arah ini jangan dipakai. Lihat TELEMETRI "
                  f"FIRMWARE.")
        kesimpulan[arah] = dict(td=td, td_se=td_se, A=A, A_se=A_se, chi2=chi2, dof=dof,
                                sisa_puncak=L[-1] - (td + kemir * lompat[-1]),
                                sisa_puncak_se=se[-1])

    # ---- tafsiran
    # Kesimpulan tentang tunda dan A HANYA dari garis yang sah. Versi pertama
    # program ini mencetak "akselerasi MELEMAH" dari garis yang sudah ia tolak
    # sendiri, dengan tunda yang mentok di batas pencocokan - dan kesimpulan
    # itu keliru (data mentahnya menunjukkan tunda ~1 s, bukan akselerasi).
    print()
    k = kesimpulan.get("naik")
    if k is None:
        print(f"  Garis naik tidak berlaku, jadi garis mana pun di atas tidak dipakai untuk")
        print(f"  menyimpulkan tunda atau A. Pakai AWAL GERAK dan TELEMETRI FIRMWARE.")
    if k is not None:
        # chi2 persentil 99 untuk dof 1 dan 2; di atas itu garisnya tidak lurus
        batas = {1: 6.63, 2: 9.21}.get(k["dof"], 3.0 * max(k["dof"], 1))
        lurus = not (k["dof"] > 0 and k["chi2"] > batas)
        mentok = sorted({n for h in hasil for n in h["merusak_L"]})
        belum_tunak = [h["target"] for h in hasil if not h["tunak_naik"]]
        if not lurus:
            print(f"  L naik TIDAK linear terhadap target (chi2 {k['chi2']:.1f} > {batas:.1f}).")

        alasan = []
        if not lurus and not tunda_tetap:
            alasan.append("garis tidak lurus, dan tunda BERBEDA antar-lintasan - "
                          "tunda yang berubah-ubah sudah cukup membengkokkan garis")
        if mentok:
            alasan.append(f"parameter mentok di batas pencocokan ({', '.join(mentok)})")
        if belum_tunak:
            alasan.append(f"laju tunak tidak tercapai pada target {belum_tunak}")
        if k["td"] < -2.0 * k["td_se"]:
            alasan.append("tunda negatif")

        if alasan:
            print(f"  Tunda dan A dari garis di atas TIDAK BOLEH dipakai:")
            for a in alasan:
                print(f"    - {a}")
            print(f"  Pakai AWAL GERAK TANPA MODEL di atas sebagai ukuran tunda.")
        elif not lurus:
            # Garis bengkok padahal tunda tetap: yang berubah dengan laju pasti
            # waktu menanjaknya - itulah akselerasi yang tidak tetap.
            arah = "LEBIH" if k["sisa_puncak"] > 0 else "KURANG"
            print(f"  Tunda TETAP antar-lintasan, jadi garis yang bengkok berasal dari")
            print(f"  waktu menanjak. Pada target tertinggi robot terlambat {arah} dari garis "
                  f"({k['sisa_puncak']:+.3f} +- {k['sisa_puncak_se']:.3f} s).")
            if k["sisa_puncak"] > 0:
                print(f"  Akselerasi MELEMAH di laju tinggi - ramp motor tidak linear,")
                print(f"  sesuai torsi stepper yang turun seiring laju. Pelacak ramp di")
                print(f"  pose.py mengandaikan akselerasi tetap {setel:.0f}.")
        elif abs(k["A"] - setel) > max(2.0 * k["A_se"], 0.10 * setel):
            print(f"  Akselerasi naik TETAP tapi BERBEDA dari setelan: "
                  f"{k['A']:.0f} vs {setel:.0f} sps/s^2.")
        else:
            print(f"  Akselerasi naik TETAP dan SESUAI setelan "
                  f"({k['A']:.0f} +- {k['A_se']:.0f} vs {setel:.0f}).")

        # Pertanyaan asalnya: dari mana angka ~2000 di cusp.csv?
        h = hasil[-1]
        print(f"\n  Pada target {h['target']} sps robot terlambat {h['L_naik']:.2f} s "
              f"(gerak pertama {h['gerak_naik']:.2f} s).")
        print(f"  Pelacak di pose.py (tunda 0, A {setel:.0f}) mengandaikan "
              f"{h['target'] / (2 * setel):.2f} s;")
        print(f"  kalau akselerasinya 2000 seperti saran cusp.csv, mestinya "
              f"{h['target'] / (2 * 2000):.2f} s - dan robot tetap mulai bergerak SEGERA.")

    print(f"\n  JANGAN ubah ACCEL_SPS2 berdasarkan angka di atas. Nilai itu SETELAN dan")
    print(f"  harus sama dengan NVS. Kalau akselerasi nyata berbeda dari setelan, yang")
    print(f"  perlu diubah adalah model pelacak di pose.py - bukan setelannya.")


# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description="Identifikasi parameter robot")
    ap.add_argument("--id", type=int, default=konfig.ROBOT_ID,
                    help=f"robot yang diukur, lihat konfig.ROBOT (bawaan {konfig.ROBOT_ID})")
    ap.add_argument("--peta", default=None)
    ap.add_argument("--mode", default="semua",
                    choices=["semua", "lurus", "busur", "tangga", "gas"],
                    help="'semua' = lurus + busur + tangga. 'gas' harus dijalankan "
                         "sendiri: butuh lintasan lurus bebas ~1,2 m di depan robot")
    ap.add_argument("--simulasi", action="store_true")
    # Dua durasi berbeda, karena tuntutannya bertentangan. Lintasan LURUS
    # memakan ruang sebanding durasi (12000 sps selama 9 detik = 280 cm), jadi
    # harus pendek. BUSUR sebaliknya: kemudi butuh tunda + 3 tau baru menetap,
    # dan mengukur sebelum itu membuat penguatan terbaca terlalu kecil - terukur
    # 0,963 pada 5 detik versus 0,993 pada 9 detik, untuk robot yang penguatan
    # sebenarnya 1,000.
    ap.add_argument("--detik-lurus", type=float, default=4.0,
                    help="durasi manuver lurus (dibatasi ruang)")
    ap.add_argument("--detik-busur", type=float, default=9.0,
                    help="durasi manuver busur & tangga (dibatasi waktu menetap kemudi)")
    # Busur perlu ruang: pada kemudi penuh jari-jarinya ~32 cm, dan makin cepat
    # makin jauh robot melaju sebelum kemudinya sempat berbelok. 4000 sps
    # (10 cm/detik) membuat sebagian besar sudut muat di area yang wajar.
    ap.add_argument("--sps-busur", type=int, default=4000,
                    help="kecepatan untuk busur & tangga (langkah/detik)")
    # Derau pose untuk uji-mandiri. Estimator yang bagus di data bersih belum
    # tentu bagus di data kamera: beda-hingga per cuplikan melipatgandakan derau
    # posisi menjadi derau kecepatan sebesar derau/dt.
    # Titik lurus kemudi diturunkan dari kelengkungan sisa - besaran nyaris nol
    # yang tenggelam di derau kamera pada satu lintasan. Satu-satunya obatnya
    # pengulangan, dan di antara pengulangan robot memutar arah supaya galat
    # distorsi kamera tidak ikut terulang identik.
    ap.add_argument("--ulang", type=int, default=1,
                    help="berapa kali percobaan LURUS diulang (untuk titik lurus kemudi)")
    ap.add_argument("--derau", type=float, default=0.0,
                    help="simulasi: derau posisi kamera (cm)")
    ap.add_argument("--derau-deg", type=float, default=0.0,
                    help="simulasi: derau sudut kamera (derajat)")
    ap.add_argument("--url", default=None)
    ap.add_argument("--host", default=None)
    ap.add_argument("--dari", default=None, metavar="FOLDER",
                    help="analisis ulang lintasan gas yang sudah terekam di FOLDER "
                         "(logs_identifikasi/...), tanpa robot dan tanpa kamera")
    args = ap.parse_args()

    # Sebelum --dari juga: analisis ulang memakai CM_PER_STEP dan ACCEL_SPS2,
    # yang berbeda per robot.
    konfig.pilih_robot(args.id)
    print(f"[i] Robot ID {konfig.ROBOT_ID:02d}: topik {konfig._PREFIX}/#, "
          f"tag {konfig.TAG_ID_ROBOT}, str_center {konfig.STEER_SERVO_CENTER}")

    if args.dari:
        rek, perkiraan = muat_gas(args.dari)
        print(f"[i] {len(rek)} lintasan gas dari {args.dari}")
        if perkiraan:
            print(f"[!] {', '.join(perkiraan)}: tidak ada berkas _perintah.csv (rekaman lama).")
            print(f"    Waktu kirim diperkirakan dari cuplikan - bisa terlambat satu siklus")
            print(f"    loop, jadi tunda terbaca sedikit LEBIH KECIL dari sebenarnya.")
        if rek:
            cocokkan_gas(rek)
        return

    for tingkat, teks in konfig.periksa():
        print(f"[{tingkat}] {teks}")
        if tingkat == "x" and not args.simulasi:
            raise SystemExit("[!] Perbaiki dulu yang bertanda [x].")

    p = Pelaksana(args)
    print(f"[i] {p.arena.ringkas()}")
    print(f"[i] log -> {p.folder}")
    if not args.simulasi:
        print("[i] ROBOT AKAN BERGERAK. Pastikan area bebas. Ctrl-C untuk batal.")
        time.sleep(3.0)

    lurus = busur = tangga = gas = []
    try:
        if args.mode in ("semua", "lurus"):
            lurus = p.lurus()
        if args.mode in ("semua", "busur"):
            busur = p.busur()
        if args.mode in ("semua", "tangga"):
            tangga = p.tangga()
        # Gas SENGAJA tidak ikut "semua". Ia butuh empat lintasan lurus
        # masing-masing sampai ~1,2 m, dan hasilnya tidak berguna kalau
        # kurang dari tiga yang muat - tunda dan akselerasi baru terpisah dari
        # variasi antar-target. Setelah lurus + busur + tangga ruangnya sudah
        # habis: di simulasi peta Kalibrasi hanya 2 dari 4 target yang muat.
        if args.mode == "gas":
            gas = p.gas()
    except KeyboardInterrupt:
        print("\n[!] dibatalkan pengguna")
    finally:
        p.tutup()

    if lurus:
        cocokkan_lurus(lurus)
    if busur:
        cocokkan_busur(busur)
    if tangga:
        cocokkan_tangga(tangga)
    if gas:
        cocokkan_gas(gas)

    v = konfig.FORWARD_SPEED * konfig.CM_PER_STEP
    print()
    SS.Model(v).cetak()


if __name__ == "__main__":
    main()

