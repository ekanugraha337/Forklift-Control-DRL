"""
navigasi.py - program utama: klik tujuan dengan mouse, robot berjalan sendiri.

    python navigasi.py --simulasi            uji penuh tanpa kamera & tanpa robot
    python navigasi.py --peta Rumah          perangkat nyata
    python navigasi.py --peta Rumah --log jalan.csv
    python navigasi.py --peta Rumah --id 2   robot lain (lihat konfig.ROBOT)
    python navigasi.py --peta Rumah --id 1,2 DUA robot sekaligus (lihat wasit.py)
    python navigasi.py --simulasi --id 1,2   dua robot di simulasi
    python navigasi.py --simulasi --id 1,2 --perencana dqfd
                                             perencana DQfD (drl/), A* sebagai cadangan

MOUSE
    klik-tarik kiri : tetapkan tujuan ROBOT AKTIF. Titik klik = posisi, arah
                      tarikan = arah hadap. Tarikan pendek = arah hadap bebas
                      (perencana memilih).
    klik kanan      : batalkan tujuan robot aktif, robot itu berhenti.

TOMBOL
    1..9   pilih robot aktif (dengan --id 1,2)
    spasi  berhenti / lanjut (semua)   e  estop (semua)   r  hitung ulang jalur robot aktif
    g  grid     z  zona     b  bayangan     s  screenshot     q  keluar

PERENCANAAN DI PROSES TERPISAH. Firmware menghentikan motor bila aliran "rc"
putus lebih lama dari RC_TIMEOUT_S (0,5 detik), sedangkan Hybrid A* di peta
nyata terukur sampai 1,3 detik. Kalau perencanaan ikut di loop kendali, robot
akan tersendat persis saat paling butuh jalur baru. Jadi loop kendali berjalan
terus pada 20 Hz, dan setiap robot merencana di proses perencananya sendiri
(perencana_proses.py). Dulu perencanaan dikerjakan thread; thread berbagi GIL,
sehingga loop terlambat 7-28 ms per detak selama rencana A* yang panjang dan
waktu rencana yang tercatat ikut memuat loop dan robot lain (catatan Obsidian
18, bagian 5). Sesi dimulai +-1 s lebih lama karena setiap proses memuat peta
dan model sendiri.

DUA ROBOT. Satu proses memegang semua robot: satu aliran kamera (pose semua
robot dari frame yang sama) dan satu wasit anti-tabrakan (wasit.py). Setiap
siklus dibagi tiga:
    1. setiap robot membaca pose, mengurus perencanaan & keselamatannya, dan
       menghitung perintah pelacak - BELUM dikirim;
    2. wasit memutuskan robot mana yang harus menunggu;
    3. setiap robot mengirim perintahnya, atau nol bila menunggu.
Dengan satu robot langkah 2 tidak memutuskan apa pun, jadi perilakunya sama
seperti sebelum ada wasit.

State tambahan untuk dua robot:
    MENUNGGU  jalur dipegang, robot diam karena bayangannya akan mengenai robot
              lain. Lanjut setelah bebas MULTI_LANJUT_S; setelah menunggu
              MULTI_TUNGGU_MAKS_S jalur dihitung ulang memutari robot lain.
    MINGGIR   jalur memutar tidak ada (mis. dua robot bertukar tempat), jadi
              robot menepi ke pose di luar jalur lawan dan menunggu lawan lewat.
Garis tipis di sekitar robot yang berjalan adalah bayangannya (tombol b).
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
import perencana_proses as PP
import pose as P
import rencana as R
import robot as RB
import wasit as W

konfig.pasang_path_kamera()
import area_constraint as AC          # noqa: E402

SIAP, MERENCANA, MELACAK, SELESAI, BAHAYA = "SIAP", "MERENCANA", "MELACAK", "SELESAI", "BAHAYA"
# Jalur tetap dipegang, robot diam karena bayangannya akan mengenai robot lain.
MENUNGGU = "MENUNGGU"
# Robot sudah menepi ke pose sementara dan menunggu lawannya lewat.
MINGGIR = "MINGGIR"
WARNA = {SIAP: (200, 200, 200), MERENCANA: (0, 255, 255), MELACAK: (0, 255, 0),
         SELESAI: (255, 200, 0), BAHAYA: (0, 0, 255), MENUNGGU: (0, 140, 255),
         MINGGIR: (255, 120, 120)}
# Warna penanda tiap robot (BGR) di tampilan dua robot.
WARNA_ID = {1: (255, 170, 0), 2: (255, 0, 200)}

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


def path_log(path, robot_id, banyak):
    """jalan.csv -> jalan_id02.csv bila ada lebih dari satu robot.

    Satu berkas per robot, dengan kepala yang sama persis seperti log satu
    robot - validasi_nyata.py tetap bisa membacanya tanpa diubah.
    """
    if not path or not banyak:
        return path
    dasar, ekst = os.path.splitext(path)
    return f"{dasar}_id{robot_id:02d}{ekst or '.csv'}"


class Unit:
    """Satu robot: sambungan, sumber pose, perencana, pelacak, dan keadaannya."""

    def __init__(self, nav, profil, sumber, sim, log_path):
        self.nav = nav
        self.args = nav.args
        self.arena = nav.arena
        self.p = profil
        self.id = profil.id
        self.per = R.Perencana(self.arena, profil, jenis=self.args.perencana)
        self.margin_cm = self.args.margin
        self.sim = sim
        self.sumber = sumber

        # --- robot ---
        self.robot = RB.Robot(host=self.args.host, simulasi=sim, profil=profil)
        if not self.robot.sambung():
            if not self.args.simulasi:
                raise SystemExit(f"[!] MQTT robot {self.id:02d} tidak tersambung. "
                                 "Cek broker dan Wi-Fi robot.")

        # --- keadaan ---
        self.state = SIAP
        self.tujuan = None
        self.jalur = None
        # Jumlah titik di ujung self.jalur yang berasal dari tembakan
        # Reeds-Shepp, atau dari model DQfD dekat; sisanya primitif Hybrid A*
        # (atau model DQfD jauh). Hanya bermakna selama self.jalur ada - diisi
        # bersamanya di ambil_hasil().
        self.jalur_rs = 0
        self.jalur_label = "RS"
        self.tracker = None
        self.pose = None
        self.perintah = None
        self.alasan = ""
        self.info = {}
        self.waktu_rencana = 0.0
        self.perlu_rencana = False      # permintaan tertunda, menunggu pose pertama
        # Segmen & arah pada frame sebelumnya, untuk mengenali akhir gerakan maju.
        self._seg_lalu = None
        self._arah_lalu = 0
        # Nama zona yang badan robot sentuh saat robot DIAM (SIAP/SELESAI).
        # Bukan keadaan bahaya - robot tidak bergerak - tapi harus terlihat.
        self.langgar_diam = None
        self._now = time.time()

        # --- multi-robot ---
        self.keputusan = None           # keputusan wasit siklus ini
        self._tunggu_sejak = None       # kapan mulai MENUNGGU
        self._bebas_sejak = None        # sejak kapan bayangan bebas (untuk lanjut)
        self.bayangan = []              # poligon cm, untuk digambar
        # (speed_sps, steer_deg) yang akan dikirim siklus ini; None = kirim nol
        self._kirim = None
        self._kirim_lalu = (0, 0.0)
        # Laju yang diperkirakan BENAR-BENAR dicapai motor (cm/detik). Stepper
        # menanjak mengikuti ramp, tidak melompat ke perintah - dan bayangan
        # robot yang sedang mengerem harus memakai laju ini, bukan perintah nol.
        self.v_taksir = 0.0
        self._t_v = None
        self.a_rem = profil.accel_sps2 * profil.cm_per_step

        # --- pekerja perencana ---
        self._hasil = None
        self._kunci = threading.Lock()
        self._sedang_merencana = False
        # Perencanaan yang sedang berjalan adalah jalan memutar robot lain.
        self._rencana_hindar = False
        # Bila perencanaan yang sedang berjalan gagal, jalur lama tetap dipakai.
        self._pertahankan = False
        # Jalur yang sedang dilacak menuju pose MINGGIR, bukan ke tujuan:
        # dict(pose=(x, y, yaw), lawan=id), atau None.
        self._minggir = None
        self._minggir_sejak = None

        self.log = buka_log(log_path) if log_path else None

    # ------------------------------------------------------------- bantuan
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

    def terima_tujuan(self, baru, tunda=False):
        """Terima atau tolak tujuan yang baru diklik.

        Ditolak DI SINI, sebelum robot berangkat. Sebelumnya tujuan yang hanya
        muat dengan jarak aman tipis tetap diterima: robot menempuh dua meter
        untuk sampai ke sana, lalu berdiri di dalam zona - dan barulah ketahuan.

        tunda=True: tujuan diterima tetapi perencanaan belum dimulai
        (perlu_rencana). Dipakai saat beberapa robot diberi tujuan bersamaan:
        tetapkan semua tujuan dulu, lalu panggil minta_rencana() masing-masing.
        Tanpa itu robot pertama merencana dengan robot berikutnya - yang belum
        menerima tujuan - sebagai rintangan diam, robot berikutnya tidak.
        """
        if not self.arena.di_dalam(baru[0], baru[1]):
            self.alasan = "tujuan di luar arena"
            print(f"[!] {self.id:02d}: {self.alasan}")
            return

        margin = self.per.margin_untuk_pose(baru[0], baru[1], baru[2])
        if margin is None:
            self.alasan = ("tujuan terlalu rapat ke zona - badan robot tidak muat "
                           f"walau dengan jarak aman {min(self.per.margin_cadangan()):.0f} cm")
            print(f"[!] {self.id:02d}: {self.alasan}. Pilih tujuan yang lebih lapang.")
            return

        # Tujuan di dekat robot lain yang sedang DIAM: robot itu tidak akan
        # pergi, jadi tujuan ini tidak akan pernah bisa dicapai. Diuji dengan
        # perencana itu sendiri, supaya tidak ada tujuan yang lolos di sini lalu
        # gagal di perencana dengan pesan "tujuan menabrak rintangan".
        rintangan = self.nav.rintangan_untuk(self)
        if rintangan and self.per.margin_untuk_pose(baru[0], baru[1], baru[2],
                                                    rintangan) is None:
            # Tanpa angka jarak: perencana memodelkan badan dengan deretan
            # lingkaran yang melebihkan panjang badan ~6 cm di depan dan di
            # belakang, jadi "butuh 6 cm" akan menyesatkan - di depan/belakang
            # robot lain yang dibutuhkan sebenarnya ~12 cm.
            self.alasan = "tujuan terlalu dekat dengan robot lain yang sedang diam"
            print(f"[!] {self.id:02d}: {self.alasan}. Pilih tujuan lain.")
            return

        # Tujuan bertumpuk dengan tujuan robot lain: kedua robot akan saling
        # menunggu di sana selamanya.
        bentrok = self.nav.tujuan_bentrok(self, baru)
        if bentrok:
            self.alasan = bentrok
            print(f"[!] {self.id:02d}: {self.alasan}. Pilih tujuan lain.")
            return

        self.tujuan = baru
        arah = "bebas" if baru[2] is None else f"{baru[2]:.0f} deg"
        catatan = "" if margin >= konfig.SAFETY_MARGIN_CM - 0.01 else \
            f" - hanya muat dengan jarak aman {margin:.0f} cm"
        self.alasan = ""
        print(f"[i] {self.id:02d}: tujuan X={baru[0]:.0f} Y={baru[1]:.0f} arah {arah}{catatan}")
        if tunda:
            self.perlu_rencana = True
        else:
            self.minta_rencana()

    def batalkan(self):
        self.tujuan, self.jalur, self.tracker = None, None, None
        self.perlu_rencana = False
        self._minggir = None
        self.state = SIAP
        self.alasan = ""
        self.robot.berhenti()
        print(f"[i] {self.id:02d}: tujuan dibatalkan.")

    def _beri_tahu_pose(self, speed, steer_deg):
        """Teruskan perintah ke sumber pose, agar ia bisa meramal saat tag hilang."""
        if hasattr(self.sumber, "perintah") and self.sim is None:
            self.sumber.perintah(speed, steer_deg)

    def _status_zona(self, pose):
        """(status, nama_zona, jarak_cm) badan robot terhadap zona."""
        fp = A.jejak_badan(pose.x, pose.y, pose.yaw)
        return AC.evaluasi(fp, self.arena.zona, self.margin_cm)

    @property
    def berjalan(self):
        """Punya jalur aktif dan boleh bergerak - dilihat wasit sebagai robot berjalan."""
        return (self.state in (MELACAK, MENUNGGU) and self.tracker is not None
                and not self.nav.ditahan)

    @property
    def diam(self):
        """Tidak punya tujuan yang sedang dikejar: robot ini tidak akan pergi.

        Robot yang sedang MERENCANA bukan robot diam - sebentar lagi ia
        berangkat. Menjadikannya rintangan membuat robot lain yang diberi tujuan
        bersamaan menolak tujuan di dekat posisi AWAL-nya, yang toh akan kosong.
        Robot yang MINGGIR justru dihitung diam: ia sengaja berdiri di situ
        sampai lawannya lewat, dan lawan itu harus merencanakan di sekitarnya.

        Robot yang punya tujuan dengan perencanaan TERTUNDA (perlu_rencana,
        lihat terima_tujuan(tunda=True)) juga bukan robot diam: ia akan
        merencana pada saat yang sama dengan robot lain.
        """
        if self.tujuan is None:
            return True
        return self.state not in (MERENCANA, MELACAK, MENUNGGU) and not self.perlu_rencana

    # ---------------------------------------------------------- perencanaan
    def minta_rencana(self, hindari_semua=False, lawan=None, pertahankan=False):
        """Mulai perencanaan di belakang.

        hindari_semua: SEMUA robot lain dijadikan rintangan, termasuk yang
        sedang berjalan. Dipakai setelah menunggu terlalu lama. Tanpa itu hanya
        robot yang DIAM yang dihindari - robot yang berjalan akan pergi, dan
        wasit yang menjaga agar keduanya tidak bertemu.

        lawan: robot (Unit) yang sedang ditunggu. Bila jalur memutar tidak ada
        dan lawan itu masih punya jalur, pekerja mencari pose MINGGIR di luar
        jalur lawan (lihat konfig MULTI_MINGGIR_*).

        pertahankan: bila perencanaan gagal, jalur yang sedang dipegang tetap
        dipakai (robot kembali MENUNGGU) alih-alih dibuang. Selalu berlaku
        untuk hindari_semua.
        """
        # Hanya perencana di PROSES yang boleh merencana (lihat docstring
        # modul). Perencana lokal di thread ini akan berbagi GIL dengan loop
        # kendali - ditolak keras, bukan diam-diam dipakai.
        if not isinstance(self.per, PP.PerencanaDiProses):
            raise RuntimeError(f"robot {self.id:02d}: perencana {type(self.per).__name__} bukan "
                               "PerencanaDiProses - perencanaan di thread sudah dihapus")
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
            print(f"[!] {self.id:02d}: {self.alasan}")
            return

        awal = self.pose.tuple()
        tujuan = self.tujuan
        rintangan = self.nav.rintangan_untuk(self, semua=hindari_semua)
        # Sisa jalur lawan diambil DI SINI, di thread loop kendali - pelacak
        # lawan terus diperbarui oleh loop, dan pekerja tidak boleh membacanya
        # sambil jalan.
        # "Masih punya jalur" - bukan "sedang berjalan": lawan yang kebetulan
        # sedang MERENCANA pada siklus yang sama tetap akan melanjutkan jalurnya,
        # dan tanpa jalur itu pencarian minggir dilewati sama sekali (terukur:
        # dua robot berhadapan saling menunggu 190 detik).
        jalur_lawan = lawan.jejak_sisa() if hindari_semua and lawan is not None \
            and lawan.tracker is not None and not lawan.diam else None
        # Rintangan SETELAH lawan sampai: lawan berdiri di ujung jalurnya,
        # robot lain di tempatnya sekarang. Dipakai memeriksa bahwa dari pose
        # minggir tujuan sendiri masih bisa dicapai.
        rintangan_lanjut = (self.nav.rintangan_untuk(self, semua=True, kecuali=lawan)
                            + jalur_lawan[-1:]) if jalur_lawan else None
        self._sedang_merencana = True
        self._rencana_hindar = hindari_semua
        self._pertahankan = hindari_semua or pertahankan
        self.waktu_rencana = self._now
        if self.state != MELACAK:
            self.state = MERENCANA

        # Jalan memutar ke TUJUAN yang opsional - memutari robot lain, atau
        # mengganti jalur minggir yang masih bisa diteruskan - hanya diterima
        # dengan margin penuh. Terukur di peta Rumah dengan derau 1,2 cm: jalan
        # memutar 4-7 m dengan margin cadangan 6 cm membuat pose TERUKUR
        # menyentuh zona (estop) di 3 dari 4 percobaan, padahal badan
        # sebenarnya tidak pernah masuk. Lebih baik tetap menunggu. Jalur ke
        # pose MINGGIR tetap boleh memakai margin cadangan: manuvernya pendek,
        # dan tanpa itu koridor 38 cm di peta Rumah tidak punya pose minggir
        # sama sekali - dua robot menunggu selamanya.
        cadangan = not self._pertahankan

        def kerja():
            # Perencanaannya sendiri dikerjakan PROSES perencana robot ini
            # (self.per = PerencanaDiProses, lihat Navigasi.__init__). Thread
            # ini hanya menunggu jawaban di pipa - tanpa memegang GIL - jadi loop
            # kendali dan perencana robot lain tidak saling memperlambat, dan
            # waktu rencana yang tercatat adalah waktu perencana itu saja.
            try:
                jalur, info = self.per.rencanakan(awal, tujuan, rintangan, cadangan)
                if not jalur and jalur_lawan:
                    terbaik = self._cari_minggir(awal, tujuan, rintangan, rintangan_lanjut,
                                                 jalur_lawan, lawan.id)
                    if terbaik:
                        jalur, info = terbaik
            except Exception as e:                       # pekerja tidak boleh mematikan program
                jalur, info = None, {"error": f"{type(e).__name__}: {e}"}
            with self._kunci:
                self._hasil = (jalur, info, tujuan)

        threading.Thread(target=kerja, daemon=True).start()

    def jejak_sisa(self, jarak_cm=4.0):
        """Jejak badan di sepanjang SELURUH sisa jalur, dijarangkan tiap jarak_cm."""
        if self.tracker is None:
            return []
        poli, lalu = [], None
        for p in self.tracker.sisa_jalur():
            x, y, yaw = self.arena.pose_ke_cm(p[0], p[1], p[2])
            if lalu is None or math.hypot(x - lalu[0], y - lalu[1]) >= jarak_cm:
                poli.append(A.jejak_badan(x, y, yaw))
                lalu = (x, y)
        return poli

    def _cari_minggir(self, awal, tujuan, rintangan, rintangan_lanjut, jalur_lawan, lawan_id):
        """(jalur, info) ke pose minggir terbaik, atau None. Jalan di pekerja.

        Dari semua calon diambil jalur TERPENDEK - bukan calon terdekat. Radius
        belok ID 02 ~71 cm: menggeser badan 35 cm ke samping bisa berarti
        manuver 2,7 m, sementara calon 50 cm di belakangnya cukup dicapai
        dengan mundur lurus.

        Calon hanya diterima kalau dari sana TUJUAN SENDIRI masih bisa
        direncanakan setelah lawan parkir di ujung jalurnya. Terukur di koridor
        Rumah: tanpa pemeriksaan ini robot minggir tepat di depan tujuan lawan,
        lawan parkir di sana, dan koridornya tertutup - jalan buntu permanen.

        Setelah satu calon lolos, calon lain hanya dicoba sebentar lagi; tanpa
        satu pun yang lolos, anggarannya lebih panjang tapi tetap terbatas.
        """
        t_mulai, terbaik = time.time(), None
        for calon in self._calon_minggir(awal, rintangan, jalur_lawan):
            if time.time() - t_mulai > (1.0 if terbaik else 6.0):
                break
            j, inf = self.per.rencanakan(awal, calon, rintangan)
            if not j or (terbaik and inf["panjang_cm"] >= terbaik[1]["panjang_cm"]):
                continue
            lanjut, _ = self.per.rencanakan(calon, tujuan, rintangan_lanjut)
            if not lanjut:
                continue
            terbaik = (j, dict(inf, minggir=calon, lawan=lawan_id))
        return terbaik

    def _calon_minggir(self, awal, rintangan, jalur_lawan):
        """Pose sementara di luar jalur lawan, dari yang terdekat. Jalan di pekerja.

        Setiap calon harus bebas dari zona dan robot lain menurut perencana,
        dan badannya cukup jauh dari sisa jalur lawan supaya lawan bisa lewat
        tanpa dihentikan wasit.
        """
        omap = self.per.peta_dengan(rintangan)
        # Margin terkecil yang boleh dipakai perencana untuk jalur minggir.
        kendaraan, _ = self.per._untuk(min(self.per.margin_cadangan()))
        bebas = konfig.MULTI_MINGGIR_BEBAS_CM
        x0, y0, yaw0 = awal
        # Arah hadap yang searah atau berlawanan dengan sekarang dicoba dulu:
        # itu yang bisa dicapai dengan maju atau mundur tanpa manuver panjang.
        urut_yaw = sorted(range(0, 360, 30),
                          key=lambda d: min(abs((d - yaw0 + 180) % 360 - 180),
                                            abs((d - yaw0) % 360 - 180)))
        calon = []
        for r in konfig.MULTI_MINGGIR_JARAK_CM:
            for arah in range(0, 360, 30):
                x = x0 + r * math.cos(math.radians(arah))
                y = y0 + r * math.sin(math.radians(arah))
                if not self.arena.di_dalam(x, y):
                    continue
                for yaw in urut_yaw:
                    if not omap.pose_free(kendaraan, *self.arena.pose_ke_sel(x, y, yaw)):
                        continue
                    if W.jarak_himpunan(jalur_lawan, [A.jejak_badan(x, y, yaw)], bebas) < bebas:
                        continue
                    calon.append((x, y, float(yaw)))
                    break
                if len(calon) >= konfig.MULTI_MINGGIR_CALON:
                    return calon
        return calon

    def ambil_hasil(self):
        with self._kunci:
            hasil, self._hasil = self._hasil, None
        if hasil is None:
            return
        self._sedang_merencana = False
        hindar, self._rencana_hindar = self._rencana_hindar, False
        pertahankan, self._pertahankan = self._pertahankan, False
        jalur, info, tujuan_dipakai = hasil

        # Tujuan berubah atau dibatalkan selagi pekerja berjalan (klik kanan,
        # estop, BAHAYA). Tanpa pemeriksaan ini jalur lama tetap dipasang dan
        # robot berangkat ke tujuan yang sudah dibatalkan.
        if self.tujuan != tujuan_dipakai:
            if self.tujuan is not None:
                self.minta_rencana()
            return

        self.info = info
        if not jalur:
            if pertahankan and self.tracker is not None:
                # Jalan lain tidak ada. Jalur lama tetap dipegang: robot yang
                # sedang berjalan meneruskannya, yang sedang diam kembali
                # menunggu - robot lain mungkin masih akan pergi.
                if self.state == MELACAK:
                    return              # jalur sekarang diteruskan, coba lagi nanti
                self.state = MENUNGGU
                self._tunggu_sejak, self._bebas_sejak = self._now, None
                self.alasan = "tidak ada jalan memutar - tetap menunggu"
                print(f"[!] {self.id:02d}: {self.alasan}")
                return
            self.jalur, self.tracker = None, None
            self.state = SIAP
            self.alasan = info.get("error") or info.get("alasan") or "tidak ada jalur"
            # Sebab yang paling sering tapi paling tidak kelihatan: robot berdiri
            # terlalu rapat ke zona, sehingga langkah pertama pun sudah tertolak.
            if self.pose is not None and not self.per.pose_bebas(*self.pose.tuple()):
                self.alasan += " (robot terlalu rapat ke zona untuk memulai)"
            print(f"[!] {self.id:02d}: gagal merencanakan: {self.alasan} "
                  f"({info.get('waktu_s', 0) * 1000:.0f} ms)")
            return
        self.jalur = jalur
        self.jalur_rs = info.get("rs_titik", 0)
        self.jalur_label = "serah" if str(info.get("perencana", "")).startswith("dqfd") else "RS"
        # Kendaraan yang dipakai pelacak HARUS yang sama dengan yang dipakai
        # merencanakan: kalau perencana terpaksa mundur ke margin lebih kecil,
        # pelacak yang memakai margin penuh akan mengira jalurnya sendiri sempit
        # dan melambat sepanjang rute tanpa sebab.
        self.tracker = jejak.buat(self.args.tracker, jalur,
                                  info.get("kendaraan", self.per.kendaraan_dipakai),
                                  self.arena.omap, self.arena)
        # Jalur baru = penomoran segmen baru. Tanpa dinolkan, segmen 0 jalur ini
        # akan dibandingkan dengan segmen terakhir jalur sebelumnya dan memicu
        # estop palsu tepat saat robot mulai bergerak.
        self._seg_lalu, self._arah_lalu = None, 0
        self.state = MELACAK
        self._tunggu_sejak, self._bebas_sejak = None, None
        self.alasan = ("margin keselamatan diturunkan ke "
                       f"{info['margin_cm']:.0f} cm") if info.get("margin_turun") else ""
        self._minggir = None
        if info.get("minggir"):
            self._minggir = dict(pose=info["minggir"], lawan=info["lawan"])
            self.alasan = f"minggir memberi jalan ke robot {info['lawan']:02d}"
        catatan = " (MINGGIR)" if self._minggir else " (MEMUTARI ROBOT LAIN)" if hindar else ""
        print(f"[OK] {self.id:02d}: jalur {info['panjang_cm']:.0f} cm, {info['titik']} titik "
              f"({'model dekat' if self.jalur_label == 'serah' else 'Reeds-Shepp'} {self.jalur_rs}), "
              f"{info['waktu_s'] * 1000:.0f} ms oleh {info.get('perencana', 'A*')}, "
              f"margin {info['margin_cm']:.0f} cm"
              f"{' (DITURUNKAN)' if info.get('margin_turun') else ''}{catatan}"
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
    def persiapan(self, now):
        """Langkah 1 siklus: pose, perencanaan, keselamatan, perintah pelacak.

        Perintah gerak BELUM dikirim - disimpan di self._kirim sampai wasit
        memutuskan. Perintah berhenti (estop di tujuan / titik ganti arah)
        tetap dikirim langsung: berhenti tidak pernah perlu izin wasit.
        """
        self._now = now
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
        if self.pose is not None and self.state in (SIAP, SELESAI, MINGGIR):
            s, zona, _ = self._status_zona(self.pose)
            if s == AC.PELANGGARAN:
                self.langgar_diam = zona

        bahaya = self.periksa_keamanan()
        if bahaya and self.state in (MELACAK, MERENCANA, MENUNGGU):
            self.robot.estop(bahaya)
            self.state, self.alasan = BAHAYA, bahaya
            self.jalur, self.tracker = None, None
            self.tujuan, self._minggir = None, None
        elif not bahaya and self.state == BAHAYA:
            self.state, self.alasan = SIAP, ""

        # Masih dalam perjalanan ke pose minggir, tapi lawan sudah diam: tidak
        # ada lagi yang perlu diberi jalan. Jalur langsung ke tujuan DICOBA,
        # tapi jalur minggir tidak dibuang sebelum yang baru didapat - terukur
        # di koridor Rumah: dari pose setengah jadi di tengah manuver dekat
        # zona, perencana sering tidak menemukan apa-apa, sedangkan dari pose
        # minggir ia bisa. Tanpa percobaan ini, robot yang sama menghabiskan
        # 50 detik menyelesaikan manuver minggir 5,5 m yang sudah tidak perlu.
        if self._minggir and self.state in (MELACAK, MENUNGGU) and \
                not self._sedang_merencana and \
                now - self.waktu_rencana > 2 * konfig.REPLAN_MIN_INTERVAL_S:
            lawan = self.nav.unit(self._minggir["lawan"])
            if lawan is None or lawan.diam:
                if not self._minggir.get("dilapor"):
                    print(f"[i] {self.id:02d}: robot {self._minggir['lawan']:02d} sudah diam - "
                          "mencoba langsung ke tujuan (diulang tiap "
                          f"{2 * konfig.REPLAN_MIN_INTERVAL_S:.0f} s).")
                    self._minggir["dilapor"] = True
                self.minta_rencana(pertahankan=True)

        # Sudah menepi: lanjut ke tujuan begitu lawan tidak lagi mengejar
        # tujuannya (sampai, dibatalkan, atau gagal), atau setelah batas waktu.
        if self.state == MINGGIR:
            lawan = self.nav.unit(self._minggir["lawan"]) if self._minggir else None
            lama = now - self._minggir_sejak
            if lawan is None or lawan.diam or lama > konfig.MULTI_MINGGIR_MAKS_S:
                print(f"[i] {self.id:02d}: selesai minggir ({lama:.1f} s) - lanjut ke tujuan.")
                self._minggir = None
                self.state, self.alasan = SIAP, ""
                self.minta_rencana()

        # Peta rintangan bersifat STATIS (zona virtual, tidak bergerak), jadi
        # jalur tidak mungkin tiba-tiba terhalang. Pemicu hitung ulang hanya
        # dua: robot menyimpang jauh, dan driver melaporkan stall - keduanya
        # berarti posisi robot tidak lagi sesuai rencana. Robot LAIN yang
        # menghalangi diurus wasit, bukan di sini.
        if self.state == MELACAK and self.tracker is not None:
            if self.robot.ambil_stall():
                print(f"[!] {self.id:02d}: firmware melaporkan STALL - posisi perintah tidak lagi "
                      "mewakili posisi roda. Jalur dihitung ulang dari pose AprilTag.")
                self.minta_rencana()
            dev_cm = self.arena.panjang_ke_cm(self.tracker.deviation)
            if dev_cm > konfig.REPLAN_DEVIATION_CM and \
                    now - self.waktu_rencana > konfig.REPLAN_MIN_INTERVAL_S:
                print(f"[!] {self.id:02d}: simpangan {dev_cm:.0f} cm - jalur dihitung ulang.")
                self.minta_rencana()

        # --- perintah pelacak ---
        self._kirim = None
        if self.berjalan:
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
                if self._minggir:
                    self._akhiri_maju("sampai di pose minggir", di_tujuan=True)
                    self.state = MINGGIR
                    self._minggir_sejak = now
                    self.alasan = f"menepi, menunggu robot {self._minggir['lawan']:02d} lewat"
                    self.robot.tengah()
                    print(f"[i] {self.id:02d}: {self.alasan}.")
                else:
                    self._akhiri_maju("tujuan tercapai", di_tujuan=True)
                    self.state = SELESAI
                    self.robot.tengah()
                    print(f"[OK] {self.id:02d}: tujuan tercapai.")
            else:
                # Pergantian segmen = titik ganti arah. Kalau segmen yang BARU
                # SAJA selesai adalah gerakan maju, hentikan seketika dulu:
                # gerakan maju berjalan sampai 25 cm/detik, dan meluncur sedikit
                # saja sebelum mundur akan menggeser seluruh manuver berikutnya.
                if self._seg_lalu is not None and seg != self._seg_lalu:
                    self._akhiri_maju("akhir gerakan maju sebelum ganti arah",
                                      di_tujuan=False)
                self._kirim = (self.perintah["speed"], self.perintah["steer_deg"])

            self._seg_lalu, self._arah_lalu = seg, arah
        else:
            self.perintah = None

    def _laju_bayangan(self, arah):
        """Laju (cm/detik) yang dipakai menghitung panjang bayangan.

        Robot yang MENUNGGU memakai laju nominal segmennya, bukan laju sekarang
        (nol). Yang ditanyakan untuk robot yang menunggu adalah "kalau aku
        berangkat sekarang, apakah jalanku bebas?" - dan jawabannya harus
        dihitung pada laju yang akan dicapainya, supaya ia tidak berangkat,
        memanjangkan bayangan, lalu berhenti lagi.
        """
        cps = self.p.cm_per_step
        if self.state == MENUNGGU:
            return (konfig.FORWARD_SPEED if arah >= 0 else konfig.REVERSE_SPEED) * cps
        perintah = abs(self.perintah["speed"]) * cps if self.perintah else 0.0
        return max(abs(self.v_taksir), perintah)

    def peserta(self):
        """Keadaan robot ini untuk wasit."""
        self.bayangan = []
        if self.pose is None:
            return W.Peserta(self.id, None, False)
        ps = W.Peserta(self.id, A.jejak_badan(*self.pose.tuple()), self.pose.terlihat)
        if self.berjalan:
            titik = self.tracker.sisa_jalur()
            if titik:
                arah = self.perintah.get("dir", 0) if self.perintah else 0
                arah = arah or titik[0][3]
                L = W.panjang_bayangan(self._laju_bayangan(arah), self.a_rem)
                cm = [self.arena.pose_ke_cm(p[0], p[1], p[2]) for p in titik]
                self.bayangan = W.sapuan(cm, L)
                ps.berjalan, ps.bayangan, ps.arah = True, self.bayangan, arah
                ps.sisa_cm = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                 for a, b in zip(cm[:-1], cm[1:]))
                ps.minggir = self._minggir is not None
        return ps

    def terapkan(self, now, keputusan):
        """Langkah 3 siklus: jalankan keputusan wasit, kirim perintah, tulis log."""
        self.keputusan = keputusan
        if keputusan is not None and keputusan.jenis == W.KONTAK:
            if self.state in (MELACAK, MERENCANA, MENUNGGU):
                self.robot.estop(keputusan.alasan)
                self.jalur, self.tracker, self.tujuan = None, None, None
                self._minggir = None
            self.state, self.alasan = BAHAYA, keputusan.alasan
        elif self.state in (MELACAK, MENUNGGU) and self._kirim is not None:
            if keputusan is not None:
                self._menunggu(now, keputusan)
            elif self.state == MENUNGGU:
                if self._bebas_sejak is None:
                    self._bebas_sejak = now
                if now - self._bebas_sejak >= konfig.MULTI_LANJUT_S:
                    print(f"[i] {self.id:02d}: jalur bebas - melanjutkan.")
                    self.state, self.alasan = MELACAK, ""
                    self._tunggu_sejak, self._bebas_sejak = None, None

        if self.state == MENUNGGU and self._kirim is not None:
            # Kemudi tetap diarahkan: servo boleh bergerak saat roda diam, dan
            # begitu jalan bebas robot tidak perlu menunggu jeda kemudi lagi.
            self._kirim = (0, self._kirim[1])
        elif self.state != MELACAK:
            self._kirim = None

        # Aliran rc tetap dikirim walau diam: firmware memakai aliran ini
        # sebagai tanda bahwa pengendali masih hidup.
        speed, steer = self._kirim if self._kirim is not None else (0, 0)
        self.robot.kirim_rc(speed, steer)
        self._beri_tahu_pose(speed, steer)
        self._kirim_lalu = (speed, steer)
        self._perbarui_laju(now, speed)

        if self.log and self.pose is not None:
            dev = self.arena.panjang_ke_cm(self.tracker.deviation) if self.tracker else 0.0
            _, _, jarak = self._status_zona(self.pose)
            self.log.write(f"{now:.3f},{self.state},{self.pose.x:.2f},{self.pose.y:.2f},"
                           f"{self.pose.yaw:.2f},{speed},{steer:.2f},"
                           f"{dev:.2f},{jarak if np.isfinite(jarak) else ''},"
                           f"{int(self.pose.terlihat)},{int(self.pose.diramalkan)}\n")

    def _menunggu(self, now, keputusan):
        if self.state != MENUNGGU:
            print(f"[i] {self.id:02d}: menunggu - {keputusan.alasan}")
            self.state = MENUNGGU
            self._tunggu_sejak = now
        self._bebas_sejak = None
        self.alasan = keputusan.alasan
        lama = now - self._tunggu_sejak
        lawan = self.nav.unit(keputusan.lawan)
        # Lawan yang sedang merencanakan akan segera punya jalur baru. Kalau
        # keduanya merencanakan pada siklus yang sama, masing-masing memutari
        # posisi lawan yang sesaat lagi berubah - terukur: dua robot gagal
        # bersamaan tiap 5 detik, selamanya. Jadi yang satu menunggu hasilnya.
        if lawan is not None and lawan._sedang_merencana:
            return
        if lama > keputusan.tunggu_maks and not self._sedang_merencana:
            print(f"[i] {self.id:02d}: sudah menunggu {lama:.1f} s - jalur dihitung ulang "
                  "memutari robot lain.")
            self._tunggu_sejak = now
            self.minta_rencana(hindari_semua=True, lawan=lawan)

    def _perbarui_laju(self, now, speed_sps):
        """Integrasikan ramp akselerasi firmware dari perintah yang dikirim."""
        if self._t_v is not None:
            dt = min(now - self._t_v, 0.2)
            target = speed_sps * self.p.cm_per_step
            batas = self.a_rem * dt
            self.v_taksir += max(-batas, min(batas, target - self.v_taksir))
        self._t_v = now

    def henti_darurat(self, alasan):
        self.robot.estop(alasan)
        self.state, self.alasan = BAHAYA, alasan
        self.jalur, self.tracker, self.tujuan = None, None, None
        self._minggir = None

    def tutup(self):
        self.robot.berhenti()
        proses = getattr(self.per, "proses", None)
        if proses is not None:
            proses.tutup()

    # --------------------------------------------------------------- gambar
    def gambar(self, img, multi, aktif):
        ctx = self.arena.ctx
        warna_id = WARNA_ID.get(self.id, (255, 255, 255))
        if self.jalur:
            # Warna membedakan bagian Hybrid A* dan Reeds-Shepp - R.LEGENDA_JALUR.
            R.gambar_jalur(img, self.arena, self.jalur, rs_titik=self.jalur_rs, label=self.jalur_label)

        if multi and self.nav.show_bayangan and self.bayangan:
            w = WARNA[MENUNGGU] if self.state == MENUNGGU else warna_id
            for poli in self.bayangan[1:]:
                cv2.polylines(img, [ctx.px(poli)], True, w, 1, cv2.LINE_AA)

        if self.tujuan is not None:
            yaw = self.tujuan[2] if self.tujuan[2] is not None else 0.0
            A.gambar_robot(img, ctx, self.tujuan[0], self.tujuan[1], yaw,
                           warna_id if multi else (0, 255, 255), tebal=1)
            if multi:
                u, v = ctx.px([self.tujuan[:2]])[0]
                AC.label(img, f"{self.id:02d}", (int(u) + 8, int(v) + 16), warna_id, 0.45)

        if self.pose is not None:
            status, zona, jarak = self._status_zona(self.pose)
            warna = AC.WARNA_STATUS.get(status, (255, 255, 255))
            if not self.pose.terlihat:
                warna = (140, 140, 140)
            elif self.pose.diramalkan:
                warna = (0, 200, 255)     # kuning: pose ramalan, tag tak terlihat
            A.gambar_robot(img, ctx, self.pose.x, self.pose.y, self.pose.yaw,
                           warna, tebal=3 if aktif and multi else 2, isi=True)
            if multi:
                u, v = ctx.px([self.pose.xy])[0]
                AC.label(img, f"{self.id:02d}{' *' if aktif else ''}",
                         (int(u) + 10, int(v) - 10), warna_id, 0.55)

        if self.perintah and self.perintah.get("target"):
            t = self.arena.ke_cm(*self.perintah["target"][:2])
            cv2.drawMarker(img, tuple(ctx.px([t])[0]), (255, 0, 255),
                           cv2.MARKER_CROSS, 14, 2)

    def baris_status(self, multi):
        """(teks, warna) baris status robot ini."""
        w = WARNA[self.state]
        baris = f"[{self.id:02d}] {self.state}" if multi else f"{self.state}"
        if self.langgar_diam:
            # Ditulis lebih dulu dan mewarnai seluruh baris: robot berdiri di
            # dalam zona, dan itu tidak boleh tersamar oleh kata "SELESAI".
            baris += f" - BADAN DI DALAM ZONA {self.langgar_diam}, PINDAHKAN ROBOT"
            w = WARNA[BAHAYA]
        if self.alasan:
            baris += f": {self.alasan}"
        if self.nav.ditahan:
            baris += "  [DITAHAN - spasi untuk lanjut]"
        if self.pose is not None:
            baris += f" | X={self.pose.x:.0f} Y={self.pose.y:.0f} yaw={self.pose.yaw:.0f}"
            if self.pose.terlihat and self.pose.diramalkan:
                baris += " [POSE DIRAMALKAN - tag tak terlihat]"
        elif multi:
            baris += " | tag belum terlihat"
        if self.tracker is not None:
            baris += f" | simpang {self.arena.panjang_ke_cm(self.tracker.deviation):.1f} cm"
        if self.perintah:
            speed, steer = self._kirim_lalu
            baris += f" | {int(speed):+d} sps {steer:+.1f} deg"
            sempit = self.perintah.get("sempit", 1.0)
            if sempit < 0.99:
                baris += f" | CELAH SEMPIT {sempit * 100:.0f}%"
        return baris, w


class Navigasi:
    def __init__(self, args, ids):
        self.args = args
        self.arena = A.Arena(A.pilih(args.peta), cm_per_cell=args.sel)
        self.ditahan = False
        self.wasit = W.Wasit()
        self.kamera = None
        multi = len(ids) > 1

        self.units = []
        if args.simulasi:
            for rid, awal in zip(ids, self._pose_awal_sim(ids)):
                p = konfig.profil(rid)
                sim = P.SumberSimulasi(*awal, derau_cm=args.derau, derau_deg=args.derau_deg,
                                       profil=p)
                self.units.append(Unit(self, p, sim, sim, path_log(args.log, rid, multi)))
        else:
            # Satu kamera untuk semua robot, bahkan bila robotnya hanya satu:
            # jalurnya sama, jadi yang teruji di satu robot juga yang berjalan
            # di dua robot.
            self.kamera = P.KameraBersama(self.arena.ctx, url=args.url)
            self.kamera.mulai()
            for rid in ids:
                p = konfig.profil(rid)
                sumber = P.SumberKamera(self.arena.ctx, profil=p, kamera=self.kamera)
                sumber.mulai()
                self.units.append(Unit(self, p, sumber, None, path_log(args.log, rid, multi)))
        self.aktif = self.units[0]
        # Setiap robot merencana di PROSES sendiri (perencana_proses.py). Thread
        # tidak dipakai lagi: thread berbagi GIL dengan loop kendali dan dengan
        # perencana robot lain, sehingga loop terlambat dan waktu rencana yang
        # tercatat ikut memuat pekerjaan lain (catatan Obsidian 18, bagian 5).
        # args.jenis_proses: jenis yang dimuat proses (bawaan: args.perencana);
        # uji_robot_nyata memuat semua perencana uji sekaligus.
        jenis = tuple(getattr(args, "jenis_proses", None) or (args.perencana,))
        self.proses = self.pasang_perencana_proses({u.id: jenis for u in self.units})

        # --- tampilan ---
        self.show_grid, self.show_zona, self.show_bayangan = True, True, True
        self.latar_statis = self.arena.kanvas()
        self.arena.gambar_grid(self.latar_statis)
        self.arena.gambar_zona(self.latar_statis)

    @property
    def multi(self):
        return len(self.units) > 1

    def unit(self, robot_id):
        for u in self.units:
            if u.id == robot_id:
                return u
        return None

    # ------------------------------------------------------------- bantuan
    def _pose_awal_sim(self, ids):
        """Pose awal simulasi: robot pertama di tengah (bila bebas), robot
        berikutnya di tempat paling lapang yang cukup jauh dari robot lain."""
        per = R.Perencana(self.arena, konfig.profil(ids[0]))
        x0, y0, x1, y1 = self.arena.ctx.ext
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        if per.pose_bebas(cx, cy, 0.0):
            hasil = [(cx, cy, 0.0)]
        else:
            d = self.arena.omap.dist
            iy, ix = np.unravel_index(np.argmax(d), d.shape)
            hasil = [(*self.arena.ke_cm(float(ix), float(iy)), 0.0)]

        d = self.arena.omap.dist
        urut = np.argsort(d, axis=None)[::-1]
        for _ in ids[1:]:
            for flat in urut:
                iy, ix = np.unravel_index(flat, d.shape)
                x, y = self.arena.ke_cm(float(ix), float(iy))
                if all(math.hypot(x - h[0], y - h[1]) >= konfig.MULTI_JARAK_AWAL_SIM_CM
                       for h in hasil) and per.pose_bebas(x, y, 0.0):
                    hasil.append((x, y, 0.0))
                    break
            else:
                raise SystemExit("[!] Arena terlalu sempit untuk menaruh semua robot simulasi.")
        return hasil

    def pasang_perencana_proses(self, jenis_per_robot):
        """Satu proses perencana per robot (perencana_proses.py).

        jenis_per_robot: {rid: (jenis, ...)} yang dimuat proses itu. Semua proses
        dimulai dulu lalu ditunggu bersama, jadi memuat peta dan model berjalan
        paralel. Unit.per diganti PerencanaDiProses untuk jenis navigasi ini
        (bila dimuat). Kembalian: {rid: PerencanaProses}.
        """
        t0 = time.perf_counter()
        proses = {u.id: PP.PerencanaProses(u.id, self.arena.nama, jenis_per_robot[u.id],
                                           cm_per_cell=self.arena.cm_per_cell, tunggu=False)
                  for u in self.units}
        try:
            for p in proses.values():
                p.tunggu_siap()
        except Exception:
            for p in proses.values():
                p.tutup()
            raise
        for u in self.units:
            if self.args.perencana in proses[u.id].jenis:
                u.per = PP.PerencanaDiProses(u.per, proses[u.id], self.args.perencana)
        print(f"[OK] perencana di proses terpisah: {len(proses)} proses siap dalam "
              f"{time.perf_counter() - t0:.2f} s")
        return proses

    def rintangan_untuk(self, unit, semua=False, kecuali=None):
        """Badan robot LAIN yang harus dihindari saat merencanakan unit ini.

        Robot yang diam selalu dihindari - ia tidak akan pergi, dan tanpa ini
        perencana dengan tenang membuat jalur menembus robot yang sedang
        diparkir. Robot yang berjalan hanya dihindari bila semua=True.
        kecuali: satu robot (Unit) yang tidak diikutkan.
        """
        out = []
        for u in self.units:
            if u is unit or u is kecuali or u.pose is None:
                continue
            if semua or u.diam:
                out.append(A.jejak_badan(*u.pose.tuple()))
        return out

    def tujuan_bentrok(self, unit, baru):
        """Alasan tujuan ini bertumpuk dengan tujuan robot lain, atau None.

        Robot lain yang DIAM tidak diuji di sini - itu urusan perencana, lewat
        rintangan_untuk(), supaya ukurannya sama dengan yang dipakai merencanakan.
        """
        aman = konfig.MULTI_JARAK_AMAN_CM
        x, y, yaw = baru
        for u in self.units:
            if u is unit or u.diam:
                continue
            if u.tujuan is not None:
                tx, ty, tyaw = u.tujuan
                if yaw is not None and tyaw is not None:
                    dekat = W.jarak_poligon(A.jejak_badan(x, y, yaw),
                                            A.jejak_badan(tx, ty, tyaw)) < aman
                else:
                    # Arah hadap salah satunya belum diketahui: anggap badan
                    # bisa menghadap ke mana saja, jadi pakai panjang badan.
                    dekat = math.hypot(tx - x, ty - y) < konfig.PANJANG_CM + aman
                if dekat:
                    return f"tujuan bertumpuk dengan tujuan robot {u.id:02d}"
        return None

    # ---------------------------------------------------------------- loop
    def satu_langkah(self, now):
        for u in self.units:
            u.persiapan(now)
        keputusan = self.wasit.putuskan([u.peserta() for u in self.units]) if self.multi else {}
        for u in self.units:
            u.terapkan(now, keputusan.get(u.id))

    # --------------------------------------------------------------- gambar
    def gambar(self):
        if self.args.simulasi or self.kamera is None or self.kamera.frame is None:
            img = self.latar_statis.copy()
        else:
            img = self.arena.ctx.top_view(self.kamera.frame)
            if self.show_grid:
                self.arena.gambar_grid(img)
            if self.show_zona:
                self.arena.gambar_zona(img)

        for u in self.units:
            u.gambar(img, self.multi, u is self.aktif)

        self.tunjuk.gambar(img, WARNA_ID.get(self.aktif.id, (0, 255, 255))
                           if self.multi else (0, 255, 255))

        # --- baris status: satu per robot ---
        y = 22
        for u in self.units:
            teks, w = u.baris_status(self.multi)
            AC.label(img, teks, (10, y), w, 0.5)
            y += 20
        info = f"peta {self.arena.nama} | perencana {self.args.perencana} | pelacak {self.args.tracker} | "
        if self.multi:
            info += f"aktif {self.aktif.id:02d}: "
        AC.label(img, info + self.aktif.robot.ringkas(), (10, y), (220, 220, 220), 0.45)

        bantu = ("klik-tarik kiri: tujuan (tarik = arah hadap) | kanan: batal | "
                 "spasi: tahan | e: estop | q: keluar")
        if self.multi:
            bantu = "1/2: pilih robot | " + bantu + " | b: bayangan"
        AC.label(img, bantu, (10, img.shape[0] - 10), (230, 230, 230), 0.45)
        AC.label(img, R.LEGENDA_JALUR, (10, img.shape[0] - 30), (230, 230, 230), 0.45)
        if any(u.state == BAHAYA or u.langgar_diam for u in self.units):
            cv2.rectangle(img, (0, 0), (img.shape[1] - 1, img.shape[0] - 1), (0, 0, 255), 8)
        return img

    # ----------------------------------------------------------------- main
    def jalankan(self):
        nama_id = "+".join(f"{u.id:02d}" for u in self.units)
        judul = f"Navigasi Forklift {nama_id} - {self.arena.nama}"
        cv2.namedWindow(judul, cv2.WINDOW_AUTOSIZE)
        self.tunjuk = A.PenunjukPose(self.arena.ctx)

        def on_mouse(event, x, y, flags, param):
            self.tunjuk.on_mouse(event, x, y, flags, param)
            if event == cv2.EVENT_RBUTTONDOWN:
                self.aktif.batalkan()

        cv2.setMouseCallback(judul, on_mouse)
        print(f"[i] {self.arena.ringkas()}")
        print(f"[i] pelacak tersedia: {', '.join(jejak.tersedia())}")
        if self.multi:
            print(f"[i] {len(self.units)} robot. Tombol 1/2 memilih robot yang diberi tujuan.")
        print("[i] Klik-tarik di jendela untuk menetapkan tujuan.")

        try:
            while True:
                now = time.time()
                baru = self.tunjuk.ambil()
                if baru is not None:
                    self.aktif.terima_tujuan(baru)

                self.satu_langkah(now)
                cv2.imshow(judul, self.gambar())

                sisa = konfig.RC_PERIODE_S - (time.time() - now)
                k = cv2.waitKey(max(1, int(sisa * 1000))) & 0xFF
                if k in (ord("q"), 27):
                    break
                elif k == ord("e"):
                    for u in self.units:
                        u.henti_darurat("estop manual")
                elif k == ord(" "):
                    self.ditahan = not self.ditahan
                    if self.ditahan:
                        for u in self.units:
                            u.robot.berhenti()
                elif k == ord("r"):
                    self.aktif.minta_rencana()
                elif k == ord("g"):
                    self.show_grid = not self.show_grid
                elif k == ord("z"):
                    self.show_zona = not self.show_zona
                elif k == ord("b"):
                    self.show_bayangan = not self.show_bayangan
                elif k == ord("s"):
                    nama = time.strftime("nav_%Y%m%d_%H%M%S.jpg")
                    cv2.imwrite(nama, self.gambar())
                    print(f"[OK] {nama}")
                elif ord("1") <= k <= ord("9"):
                    for u in self.units:
                        if u.id == k - ord("0"):
                            self.aktif = u
                            print(f"[i] robot aktif: {u.id:02d}")
        finally:
            for u in self.units:
                u.tutup()
            time.sleep(0.2)
            for u in self.units:
                u.robot.putus()
                if u.log:
                    u.log.close()
            if self.kamera is not None:
                self.kamera.berhenti()
            cv2.destroyAllWindows()


def urai_id(teks):
    """"1" -> [1], "1,2" -> [1, 2]. Urutan dipertahankan, duplikat dibuang."""
    ids = []
    for bagian in str(teks).replace(" ", "").split(","):
        if not bagian:
            continue
        try:
            rid = int(bagian)
        except ValueError:
            raise SystemExit(f"[!] --id harus angka, dipisah koma (mis. 1,2): '{teks}'")
        if rid not in ids:
            ids.append(rid)
    if not ids:
        raise SystemExit("[!] --id kosong.")
    return ids


def main():
    ap = argparse.ArgumentParser(description="Navigasi otomatis forklift (Hybrid A* atau DQfD + pure pursuit)")
    ap.add_argument("--id", default=str(konfig.ROBOT_ID),
                    help="robot yang dikendalikan, lihat konfig.ROBOT. Beberapa robot: "
                         f"--id 1,2 (bawaan {konfig.ROBOT_ID})")
    ap.add_argument("--peta", default=None)
    ap.add_argument("--sel", type=float, default=None, help="cm per sel grid")
    ap.add_argument("--tracker", default=konfig.PELACAK_BAWAAN, choices=jejak.tersedia())
    ap.add_argument("--perencana", default="astar", choices=R.JENIS_PERENCANA,
                    help="astar (bawaan) | dqfd: DQfD dengan A* sebagai cadangan | dqfd-murni")
    ap.add_argument("--simulasi", action="store_true", help="tanpa kamera & tanpa robot")
    ap.add_argument("--derau", type=float, default=0.0, help="simulasi: derau posisi (cm)")
    ap.add_argument("--derau-deg", type=float, default=0.0, help="simulasi: derau sudut (derajat)")
    ap.add_argument("--margin", type=float, default=10.0, help="jarak PERINGATAN dari zona (cm)")
    ap.add_argument("--url", default=None, help="override URL RTSP")
    ap.add_argument("--host", default=None, help="override alamat broker MQTT")
    ap.add_argument("--log", default=None,
                    help="tulis CSV (dua robot: satu berkas per robot, akhiran _id01/_id02)")
    args = ap.parse_args()
    ids = urai_id(args.id)

    # Pemeriksaan per robot, lalu pesan yang sama (kalibrasi kamera, geometri)
    # hanya dicetak sekali.
    pesan = []
    for rid in ids:
        konfig.pilih_robot(rid)
        print(f"[i] Robot ID {konfig.ROBOT_ID:02d}: topik {konfig._PREFIX}/#, "
              f"tag {konfig.TAG_ID_ROBOT}, str_center {konfig.STEER_SERVO_CENTER}, "
              f"kemudi maks {konfig.MAX_STEER_DEG:.1f} deg")
        for p in konfig.periksa():
            if p not in pesan:
                pesan.append(p)
    konfig.pilih_robot(ids[0])
    fatal = False
    for tingkat, teks in pesan:
        print(f"[{tingkat}] {teks}")
        fatal |= tingkat == "x"
    if fatal and not args.simulasi:
        raise SystemExit("[!] Perbaiki dulu yang bertanda [x].")

    Navigasi(args, ids).jalankan()


if __name__ == "__main__":
    main()
