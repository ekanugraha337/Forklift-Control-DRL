"""
navigasi.py - program utama: klik tujuan dengan mouse, robot berjalan sendiri.

    python navigasi.py --simulasi            uji penuh tanpa kamera & tanpa robot
    python navigasi.py --peta Rumah          perangkat nyata
    python navigasi.py --peta Rumah --log jalan.csv
    python navigasi.py --peta Rumah --id 2   robot lain (lihat konfig.ROBOT)

MOUSE
    klik-tarik kiri : tetapkan tujuan. Titik klik = posisi, arah tarikan = arah
                      hadap. Tarikan pendek = arah hadap bebas (perencana memilih).
    klik kanan      : batalkan tujuan, robot berhenti.

TOMBOL
    spasi  berhenti / lanjut      e  estop      r  hitung ulang jalur
    g  grid     z  zona     s  screenshot     q  keluar

PERENCANAAN DI THREAD TERPISAH. Firmware menghentikan motor bila aliran "rc"
putus lebih lama dari RC_TIMEOUT_S (0,5 detik), sedangkan Hybrid A* di peta
nyata terukur sampai 1,3 detik. Kalau perencanaan ikut di loop kendali, robot
akan tersendat persis saat paling butuh jalur baru. Jadi loop kendali berjalan
terus pada 20 Hz, dan perencanaan dikerjakan pekerja di belakang.
"""
import argparse
import math
import os
import threading
import time

import cv2
import numpy as np

import arena as A
import jejak
import konfig
import pose as P
import rencana as R
import robot as RB

konfig.pasang_path_kamera()
import area_constraint as AC          # noqa: E402

SIAP, MERENCANA, MELACAK, SELESAI, BAHAYA = "SIAP", "MERENCANA", "MELACAK", "SELESAI", "BAHAYA"
WARNA = {SIAP: (200, 200, 200), MERENCANA: (0, 255, 255), MELACAK: (0, 255, 0),
         SELESAI: (255, 200, 0), BAHAYA: (0, 0, 255)}

# Kolom "diramalkan" ditambahkan 2026-09-27. Tanpanya validasi_nyata.py tidak
# bisa membedakan pose UKUR dari pose RAMALAN: "terlihat" bernilai 1 untuk
# keduanya, jadi sebagian baris acuan bisa saja keluaran model itu sendiri - dan
# memvalidasi model terhadap dirinya sendiri membuat angkanya terlalu bagus.
KEPALA_LOG = ("t,state,x_cm,y_cm,yaw_deg,speed_sps,steer_deg,"
              "deviasi_cm,margin_cm,terlihat,diramalkan\n")


def buka_log(path):
    """Buka CSV log untuk ditambahi. Tulis kepala bila berkasnya baru atau kosong.

    Log dibuka dalam mode tambah, jadi satu berkas bisa menampung beberapa sesi.
    Karena itu berkas lama dengan kepala BERBEDA ditolak, bukan ditambahi:
    baris 11 kolom di bawah kepala 10 kolom tetap terbaca oleh csv.DictReader,
    tapi kolom "diramalkan"-nya hilang diam-diam - persis kekeliruan yang
    sedang ditutup kolom itu.
    """
    if os.path.exists(path) and os.path.getsize(path) > 0:
        with open(path) as fh:
            lama = fh.readline()
        if lama != KEPALA_LOG:
            raise SystemExit(
                f"[!] {path} ditulis dengan format log lama:\n    {lama.strip()}\n"
                f"    Pakai nama berkas baru - jangan campur dua format dalam satu CSV.")
        return open(path, "a")
    fh = open(path, "a")
    fh.write(KEPALA_LOG)
    return fh


class Navigasi:
    def __init__(self, args):
        self.args = args
        self.arena = A.Arena(A.pilih(args.peta), cm_per_cell=args.sel)
        self.per = R.Perencana(self.arena)
        self.margin_cm = args.margin

        # --- sumber pose ---
        if args.simulasi:
            x0, y0, x1, y1 = self.arena.ctx.ext
            awal = self._pose_bebas_pertama((x0 + x1) / 2, (y0 + y1) / 2)
            self.sim = P.SumberSimulasi(*awal, derau_cm=args.derau, derau_deg=args.derau_deg)
            self.sumber = self.sim
        else:
            self.sim = None
            self.sumber = P.SumberKamera(self.arena.ctx, url=args.url)
            self.sumber.mulai()

        # --- robot ---
        self.robot = RB.Robot(host=args.host, simulasi=self.sim)
        if not self.robot.sambung():
            if not args.simulasi:
                raise SystemExit("[!] MQTT tidak tersambung. Cek broker dan Wi-Fi robot.")

        # --- keadaan ---
        self.state = SIAP
        self.tujuan = None
        self.jalur = None
        self.tracker = None
        self.pose = None
        self.perintah = None
        self.alasan = ""
        self.ditahan = False
        self.info = {}
        self.waktu_rencana = 0.0
        self.perlu_rencana = False      # permintaan tertunda, menunggu pose pertama
        # Segmen & arah pada frame sebelumnya, untuk mengenali akhir gerakan maju.
        self._seg_lalu = None
        self._arah_lalu = 0
        # Nama zona yang badan robot sentuh saat robot DIAM (SIAP/SELESAI).
        # Bukan keadaan bahaya - robot tidak bergerak - tapi harus terlihat.
        self.langgar_diam = None

        # --- pekerja perencana ---
        self._hasil = None
        self._kunci = threading.Lock()
        self._sedang_merencana = False

        # --- tampilan ---
        self.show_grid, self.show_zona = True, True
        self.latar_statis = self.arena.kanvas()
        self.arena.gambar_grid(self.latar_statis)
        self.arena.gambar_zona(self.latar_statis)

        self.log = buka_log(args.log) if args.log else None

    # ------------------------------------------------------------- bantuan
    def _pose_bebas_pertama(self, cx, cy):
        if self.per.pose_bebas(cx, cy, 0.0):
            return (cx, cy, 0.0)
        d = self.arena.omap.dist
        iy, ix = np.unravel_index(np.argmax(d), d.shape)
        x, y = self.arena.ke_cm(float(ix), float(iy))
        return (x, y, 0.0)

    def _akhiri_maju(self, alasan, di_tujuan):
        """Hentikan gerakan MAJU yang baru saja berakhir.

        Di TUJUAN dan di TITIK GANTI ARAH diperlakukan berbeda, karena estop
        bukan sekadar "berhenti lebih keras": ia memutus arus motor. Di tujuan
        itu tepat - robot memang harus diam di sana. Di titik ganti arah itu
        melepas pengereman dan torsi penahan justru saat robot paling
        membutuhkannya, dan dua tabrakan di perangkat terjadi persis di situ.

        Kalau estop tidak dipakai, robot tetap berhenti: pelacak mengirim
        kecepatan nol selama jeda, dan firmware menanganinya dengan stopMove() -
        melambat terkendali dengan motor tetap berarus.
        """
        if self._arah_lalu <= 0:
            return                      # mundur pelan, tidak perlu diperlakukan khusus
        pakai = konfig.ESTOP_TUJUAN if di_tujuan else konfig.ESTOP_TITIK_GANTI_ARAH
        if pakai:
            self.robot.estop(alasan)

    def _terima_tujuan(self, baru):
        """Terima atau tolak tujuan yang baru diklik.

        Ditolak DI SINI, sebelum robot berangkat. Sebelumnya tujuan yang hanya
        muat dengan jarak aman tipis tetap diterima: robot menempuh dua meter
        untuk sampai ke sana, lalu berdiri di dalam zona - dan barulah ketahuan.
        """
        if not self.arena.di_dalam(baru[0], baru[1]):
            self.alasan = "tujuan di luar arena"
            print(f"[!] {self.alasan}")
            return

        margin = self.per.margin_untuk_pose(baru[0], baru[1], baru[2])
        if margin is None:
            self.alasan = ("tujuan terlalu rapat ke zona - badan robot tidak muat "
                           f"walau dengan jarak aman {min(self.per.margin_cadangan()):.0f} cm")
            print(f"[!] {self.alasan}. Pilih tujuan yang lebih lapang.")
            return

        self.tujuan = baru
        arah = "bebas" if baru[2] is None else f"{baru[2]:.0f} deg"
        catatan = "" if margin >= konfig.SAFETY_MARGIN_CM - 0.01 else \
            f" - hanya muat dengan jarak aman {margin:.0f} cm"
        self.alasan = ""
        print(f"[i] Tujuan X={baru[0]:.0f} Y={baru[1]:.0f} arah {arah}{catatan}")
        self.minta_rencana()

    def _beri_tahu_pose(self, speed, steer_deg):
        """Teruskan perintah ke sumber pose, agar ia bisa meramal saat tag hilang."""
        if hasattr(self.sumber, "perintah") and self.sim is None:
            self.sumber.perintah(speed, steer_deg)

    def _status_zona(self, pose):
        """(status, nama_zona, jarak_cm) badan robot terhadap zona."""
        fp = A.jejak_badan(pose.x, pose.y, pose.yaw)
        return AC.evaluasi(fp, self.arena.zona, self.margin_cm)

    # ---------------------------------------------------------- perencanaan
    def minta_rencana(self):
        if self.tujuan is None or self._sedang_merencana:
            return
        if self.pose is None:
            # Pose belum pernah terbaca - frame pertama belum tiba. Permintaan
            # DITUNDA, bukan dibuang: kalau dibuang, tujuan pertama tiap sesi
            # tidak pernah dijalankan dan tidak ada satu pesan pun yang
            # menjelaskan mengapa. Gejala yang sama pernah muncul di v1 dengan
            # sebab berbeda (lihat PERBAIKAN_2026-09-15.md bab 1).
            self.perlu_rencana = True
            return
        self.perlu_rencana = False

        # Robot yang sudah terlanjur berada di dalam zona tidak bisa ditolong
        # perencana: setiap jalur dari sana berawal dari tabrakan, dan hasilnya
        # hanya "tidak ada jalur" berulang-ulang tanpa petunjuk.
        #
        # Yang diuji di sini adalah BADAN SEBENARNYA (sama seperti pemantau
        # keselamatan), bukan model perencana yang sudah digelembungkan
        # SAFETY_MARGIN_CM. Memakai model perencana akan menolak pose yang
        # sebenarnya aman - misalnya robot yang baru saja selesai di tujuan
        # yang memang rapat - padahal dari sana ia masih bisa keluar dengan
        # selamat. Margin itu untuk merencanakan jalur baru, bukan untuk
        # menghakimi tempat robot sekarang berdiri.
        status, zona, _ = self._status_zona(self.pose)
        if status == AC.PELANGGARAN:
            self.alasan = f"robot berada di dalam zona {zona} - pindahkan dulu ke tempat bebas"
            self.state = BAHAYA
            print(f"[!] {self.alasan}")
            return

        awal = self.pose.tuple()
        tujuan = self.tujuan
        self._sedang_merencana = True
        self.waktu_rencana = time.time()
        if self.state != MELACAK:
            self.state = MERENCANA

        def kerja():
            try:
                jalur, info = self.per.rencanakan(awal, tujuan)
            except Exception as e:                       # pekerja tidak boleh mematikan program
                jalur, info = None, {"error": f"{type(e).__name__}: {e}"}
            with self._kunci:
                self._hasil = (jalur, info)

        threading.Thread(target=kerja, daemon=True).start()

    def ambil_hasil(self):
        with self._kunci:
            hasil, self._hasil = self._hasil, None
        if hasil is None:
            return
        self._sedang_merencana = False
        jalur, info = hasil
        self.info = info
        if not jalur:
            self.jalur, self.tracker = None, None
            self.state = SIAP
            self.alasan = info.get("error") or self.per.stats.get("reason", "tidak ada jalur")
            # Sebab yang paling sering tapi paling tidak kelihatan: robot berdiri
            # terlalu rapat ke zona, sehingga langkah pertama pun sudah tertolak.
            if self.pose is not None and not self.per.pose_bebas(*self.pose.tuple()):
                self.alasan += " (robot terlalu rapat ke zona untuk memulai)"
            print(f"[!] Gagal merencanakan: {self.alasan} "
                  f"({info.get('waktu_s', 0) * 1000:.0f} ms)")
            return
        self.jalur = jalur
        # Kendaraan yang dipakai pelacak HARUS yang sama dengan yang dipakai
        # merencanakan: kalau perencana terpaksa mundur ke margin lebih kecil,
        # pelacak yang memakai margin penuh akan mengira jalurnya sendiri sempit
        # dan melambat sepanjang rute tanpa sebab.
        self.tracker = jejak.buat(self.args.tracker, jalur, self.per.kendaraan_dipakai,
                                  self.arena.omap, self.arena)
        # Jalur baru = penomoran segmen baru. Tanpa dinolkan, segmen 0 jalur ini
        # akan dibandingkan dengan segmen terakhir jalur sebelumnya dan memicu
        # estop palsu tepat saat robot mulai bergerak.
        self._seg_lalu, self._arah_lalu = None, 0
        self.state = MELACAK
        self.alasan = ("margin keselamatan diturunkan ke "
                       f"{info['margin_cm']:.0f} cm") if info.get("margin_turun") else ""
        print(f"[OK] jalur {info['panjang_cm']:.0f} cm, {info['titik']} titik, "
              f"{info['waktu_s'] * 1000:.0f} ms, margin {info['margin_cm']:.0f} cm"
              f"{' (DITURUNKAN)' if info.get('margin_turun') else ''}"
              f" -> mulai melacak dengan '{self.args.tracker}'")

    # ------------------------------------------------------------ keamanan
    def periksa_keamanan(self):
        """Kembalikan alasan estop, atau None bila aman."""
        if self.pose is None:
            return "pose robot belum pernah terlihat"
        if not self.args.simulasi:
            if self.sumber.frame_basi:
                return "frame kamera berhenti diperbarui"
            if not self.pose.terlihat:
                return "tag robot hilang dari pandangan"
            # Firmware mematikan driver sendiri saat overtemp atau short, jadi
            # robot memang berhenti. Tanpa pemeriksaan ini layar tetap menulis
            # MELACAK sementara robot diam - sebabnya tidak muncul di mana pun.
            gangguan = self.robot.gangguan_driver()
            if gangguan:
                return f"driver bermasalah: {gangguan}"
            if not self.robot.tersambung:
                return "MQTT terputus"
        status, zona, _ = self._status_zona(self.pose)
        if status == AC.PELANGGARAN:
            return f"badan robot melanggar zona {zona}"
        return None

    # ---------------------------------------------------------------- loop
    def satu_langkah(self, now):
        self.pose = self.sumber.baca()
        if self.perlu_rencana and self.pose is not None:
            self.minta_rencana()
        self.ambil_hasil()

        # Zona diperiksa di SETIAP state, bukan hanya saat melacak. Sebelumnya
        # pemeriksaan berhenti begitu state menjadi SELESAI, sehingga layar
        # menulis "SELESAI" dengan tenang sementara badan robot berdiri di dalam
        # zona - persis yang terjadi di Z9. Estop-nya memang tidak diperlukan
        # saat robot sudah diam, tapi operator tetap harus diberi tahu.
        self.langgar_diam = None
        if self.pose is not None and self.state in (SIAP, SELESAI):
            s, zona, _ = self._status_zona(self.pose)
            if s == AC.PELANGGARAN:
                self.langgar_diam = zona

        bahaya = self.periksa_keamanan()
        if bahaya and self.state in (MELACAK, MERENCANA):
            self.robot.estop(bahaya)
            self.state, self.alasan = BAHAYA, bahaya
            self.jalur, self.tracker = None, None
            self.tujuan = None
        elif not bahaya and self.state == BAHAYA:
            self.state, self.alasan = SIAP, ""

        # Peta rintangan bersifat STATIS (zona virtual, tidak bergerak), jadi
        # jalur tidak mungkin tiba-tiba terhalang. Pemicu hitung ulang hanya
        # dua: robot menyimpang jauh, dan driver melaporkan stall - keduanya
        # berarti posisi robot tidak lagi sesuai rencana.
        if self.state == MELACAK and self.tracker is not None:
            if self.robot.ambil_stall():
                print("[!] Firmware melaporkan STALL - posisi perintah tidak lagi "
                      "mewakili posisi roda. Jalur dihitung ulang dari pose AprilTag.")
                self.minta_rencana()
            dev_cm = self.arena.panjang_ke_cm(self.tracker.deviation)
            if dev_cm > konfig.REPLAN_DEVIATION_CM and \
                    now - self.waktu_rencana > konfig.REPLAN_MIN_INTERVAL_S:
                print(f"[!] Simpangan {dev_cm:.0f} cm - jalur dihitung ulang.")
                self.minta_rencana()

        # --- perintah ke robot ---
        if self.state == MELACAK and self.tracker is not None and not self.ditahan:
            gx, gy, yaw = self.arena.pose_ke_sel(*self.pose.tuple())
            self.perintah = self.tracker.update(gx, gy, yaw, now)
            seg = self.perintah.get("seg")
            arah = self.perintah.get("dir", 0)

            if self.perintah.get("done") or self.tracker.done:
                # Akhir gerakan maju yang terakhir. estop menghentikan motor
                # seketika (forceStop) alih-alih melandai seperti "stop", jadi
                # robot tidak meluncur melewati tujuan. Sisa luncuran di sini
                # langsung menjadi galat posisi akhir - tidak ada jalur tersisa
                # untuk mengoreksinya.
                self._akhiri_maju("tujuan tercapai", di_tujuan=True)
                self.state = SELESAI
                self.robot.tengah()
                print("[OK] Tujuan tercapai.")
            else:
                # Pergantian segmen = titik ganti arah. Kalau segmen yang BARU
                # SAJA selesai adalah gerakan maju, hentikan seketika dulu:
                # gerakan maju berjalan sampai 31 cm/detik, dan meluncur sedikit
                # saja sebelum mundur akan menggeser seluruh manuver berikutnya.
                if self._seg_lalu is not None and seg != self._seg_lalu:
                    self._akhiri_maju("akhir gerakan maju sebelum ganti arah",
                                      di_tujuan=False)
                self.robot.kirim_rc(self.perintah["speed"], self.perintah["steer_deg"])
                self._beri_tahu_pose(self.perintah["speed"], self.perintah["steer_deg"])

            self._seg_lalu, self._arah_lalu = seg, arah
        else:
            # Aliran rc tetap dikirim walau diam: firmware memakai aliran ini
            # sebagai tanda bahwa pengendali masih hidup.
            self.perintah = None
            self.robot.kirim_rc(0, 0)
            self._beri_tahu_pose(0, 0)

        if self.log and self.pose is not None:
            dev = self.arena.panjang_ke_cm(self.tracker.deviation) if self.tracker else 0.0
            _, _, jarak = self._status_zona(self.pose)
            self.log.write(f"{now:.3f},{self.state},{self.pose.x:.2f},{self.pose.y:.2f},"
                           f"{self.pose.yaw:.2f},"
                           f"{self.perintah['speed'] if self.perintah else 0},"
                           f"{self.perintah['steer_deg'] if self.perintah else 0:.2f},"
                           f"{dev:.2f},{jarak if np.isfinite(jarak) else ''},"
                           f"{int(self.pose.terlihat)},{int(self.pose.diramalkan)}\n")

    # --------------------------------------------------------------- gambar
    def gambar(self):
        if self.args.simulasi or self.sumber.frame is None:
            img = self.latar_statis.copy()
        else:
            img = self.arena.ctx.top_view(self.sumber.frame)
            if self.show_grid:
                self.arena.gambar_grid(img)
            if self.show_zona:
                self.arena.gambar_zona(img)

        if self.jalur:
            R.gambar_jalur(img, self.arena, self.jalur)

        if self.tujuan is not None:
            yaw = self.tujuan[2] if self.tujuan[2] is not None else 0.0
            A.gambar_robot(img, self.arena.ctx, self.tujuan[0], self.tujuan[1], yaw,
                           (0, 255, 255), tebal=1)

        if self.pose is not None:
            status, zona, jarak = self._status_zona(self.pose)
            warna = AC.WARNA_STATUS.get(status, (255, 255, 255))
            if not self.pose.terlihat:
                warna = (140, 140, 140)
            elif self.pose.diramalkan:
                warna = (0, 200, 255)     # kuning: pose ramalan, tag tak terlihat
            A.gambar_robot(img, self.arena.ctx, self.pose.x, self.pose.y, self.pose.yaw,
                           warna, isi=True)

        if self.perintah and self.perintah.get("target"):
            t = self.arena.ke_cm(*self.perintah["target"][:2])
            cv2.drawMarker(img, tuple(self.arena.ctx.px([t])[0]), (255, 0, 255),
                           cv2.MARKER_CROSS, 14, 2)

        self.tunjuk.gambar(img, (0, 255, 255))

        # --- baris status ---
        w = WARNA[self.state]
        baris = f"{self.state}"
        if self.langgar_diam:
            # Ditulis lebih dulu dan mewarnai seluruh baris: robot berdiri di
            # dalam zona, dan itu tidak boleh tersamar oleh kata "SELESAI".
            baris += f" - BADAN DI DALAM ZONA {self.langgar_diam}, PINDAHKAN ROBOT"
            w = WARNA[BAHAYA]
        if self.alasan:
            baris += f": {self.alasan}"
        if self.ditahan:
            baris += "  [DITAHAN - spasi untuk lanjut]"
        if self.pose is not None:
            baris += f" | X={self.pose.x:.0f} Y={self.pose.y:.0f} yaw={self.pose.yaw:.0f}"
            if self.pose.terlihat and self.pose.diramalkan:
                baris += " [POSE DIRAMALKAN - tag tak terlihat]"
        if self.tracker is not None:
            baris += f" | simpang {self.arena.panjang_ke_cm(self.tracker.deviation):.1f} cm"
        if self.perintah:
            baris += (f" | {self.perintah['speed']:+d} sps "
                      f"{self.perintah['steer_deg']:+.1f} deg")
            sempit = self.perintah.get("sempit", 1.0)
            if sempit < 0.99:
                baris += f" | CELAH SEMPIT {sempit * 100:.0f}%"
        AC.label(img, baris, (10, 22), w, 0.5)
        AC.label(img, f"peta {self.arena.nama} | pelacak {self.args.tracker} | "
                      f"{self.robot.ringkas()}", (10, 44), (220, 220, 220), 0.45)
        AC.label(img, "klik-tarik kiri: tujuan (tarik = arah hadap) | kanan: batal | "
                      "spasi: tahan | e: estop | q: keluar",
                 (10, img.shape[0] - 10), (230, 230, 230), 0.45)
        if self.state == BAHAYA or self.langgar_diam:
            cv2.rectangle(img, (0, 0), (img.shape[1] - 1, img.shape[0] - 1), (0, 0, 255), 8)
        return img

    # ----------------------------------------------------------------- main
    def jalankan(self):
        judul = f"Navigasi Forklift {konfig.ROBOT_ID:02d} - {self.arena.nama}"
        cv2.namedWindow(judul, cv2.WINDOW_AUTOSIZE)
        self.tunjuk = A.PenunjukPose(self.arena.ctx)

        def on_mouse(event, x, y, flags, param):
            self.tunjuk.on_mouse(event, x, y, flags, param)
            if event == cv2.EVENT_RBUTTONDOWN:
                self.tujuan, self.jalur, self.tracker = None, None, None
                self.state = SIAP
                self.robot.berhenti()
                print("[i] Tujuan dibatalkan.")

        cv2.setMouseCallback(judul, on_mouse)
        print(f"[i] {self.arena.ringkas()}")
        print(f"[i] pelacak tersedia: {', '.join(jejak.tersedia())}")
        print("[i] Klik-tarik di jendela untuk menetapkan tujuan.")

        try:
            while True:
                now = time.time()
                baru = self.tunjuk.ambil()
                if baru is not None:
                    self._terima_tujuan(baru)

                self.satu_langkah(now)
                cv2.imshow(judul, self.gambar())

                sisa = konfig.RC_PERIODE_S - (time.time() - now)
                k = cv2.waitKey(max(1, int(sisa * 1000))) & 0xFF
                if k in (ord("q"), 27):
                    break
                elif k == ord("e"):
                    self.robot.estop("tombol e")
                    self.state, self.alasan = BAHAYA, "estop manual"
                    self.jalur, self.tracker, self.tujuan = None, None, None
                elif k == ord(" "):
                    self.ditahan = not self.ditahan
                    if self.ditahan:
                        self.robot.berhenti()
                elif k == ord("r"):
                    self.minta_rencana()
                elif k == ord("g"):
                    self.show_grid = not self.show_grid
                elif k == ord("z"):
                    self.show_zona = not self.show_zona
                elif k == ord("s"):
                    nama = time.strftime("nav_%Y%m%d_%H%M%S.jpg")
                    cv2.imwrite(nama, self.gambar())
                    print(f"[OK] {nama}")
        finally:
            self.robot.berhenti()
            time.sleep(0.2)
            self.robot.putus()
            self.sumber.berhenti()
            if self.log:
                self.log.close()
            cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="Navigasi otomatis forklift (Hybrid A* + pure pursuit)")
    ap.add_argument("--id", type=int, default=konfig.ROBOT_ID,
                    help=f"robot yang dikendalikan, lihat konfig.ROBOT (bawaan {konfig.ROBOT_ID})")
    ap.add_argument("--peta", default=None)
    ap.add_argument("--sel", type=float, default=None, help="cm per sel grid")
    ap.add_argument("--tracker", default=konfig.PELACAK_BAWAAN, choices=jejak.tersedia())
    ap.add_argument("--simulasi", action="store_true", help="tanpa kamera & tanpa robot")
    ap.add_argument("--derau", type=float, default=0.0, help="simulasi: derau posisi (cm)")
    ap.add_argument("--derau-deg", type=float, default=0.0, help="simulasi: derau sudut (derajat)")
    ap.add_argument("--margin", type=float, default=10.0, help="jarak PERINGATAN dari zona (cm)")
    ap.add_argument("--url", default=None, help="override URL RTSP")
    ap.add_argument("--host", default=None, help="override alamat broker MQTT")
    ap.add_argument("--log", default=None, help="tulis CSV")
    args = ap.parse_args()

    konfig.pilih_robot(args.id)
    print(f"[i] Robot ID {konfig.ROBOT_ID:02d}: topik {konfig._PREFIX}/#, "
          f"tag {konfig.TAG_ID_ROBOT}, str_center {konfig.STEER_SERVO_CENTER}")
    for tingkat, teks in konfig.periksa():
        print(f"[{tingkat}] {teks}")
        if tingkat == "x" and not args.simulasi:
            raise SystemExit("[!] Perbaiki dulu yang bertanda [x].")

    Navigasi(args).jalankan()


if __name__ == "__main__":
    main()
