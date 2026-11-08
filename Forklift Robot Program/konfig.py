"""
konfig.py - satu sumber kebenaran untuk geometri robot, kalibrasi, dan jaringan.

SEMUA PANJANG DALAM SENTIMETER. Tidak ada piksel di berkas ini.

Alasannya: peta constraint (area_constraint.py) menyimpan zona dalam cm lewat
homografi terkalibrasi, dan "Model Robot ID 01.md" juga dalam cm. Versi lama
(MQTT_1tag.py) bekerja dalam piksel dengan faktor PX_PER_CM yang harus
dikalibrasi ulang setiap kamera bergeser - dan faktor itu diturunkan dari ukuran
AprilTag, yang ternyata tidak cocok dengan dokumen model (14,1 vs 10 cm).
Dengan bekerja dalam cm, seluruh persoalan itu hilang.

Jalankan berkas ini untuk memeriksa kewarasan seluruh nilai:
    python konfig.py
"""
import bisect
import json
import math
import os
import sys
from types import SimpleNamespace

DIR = os.path.dirname(os.path.abspath(__file__))

# Lokasi cctv_lurus.py + area_constraint.py + data kalibrasi + peta constraint:
# folder Camera/ di repo ini, satu tingkat di atas folder program. Dengan begitu
# peta, kalibrasi lensa, dan ukuran robot yang DIPAKAI robot adalah yang
# ter-commit. Sampai 2026-10-20 di branch ini nilainya menunjuk ke salinan di
# luar repo (Documents\Master ITB\Semester 3\Camera), sehingga kalibrasi dan
# peta yang disimpan di repo tidak terbaca (sama dengan ea1f759 di RS-optimize).
CAMERA_DIR = os.path.normpath(os.path.join(DIR, "..", "Camera"))


def pasang_path_kamera():
    """Buat area_constraint & cctv_lurus bisa diimpor. Panggil sebelum mengimpornya."""
    if CAMERA_DIR not in sys.path:
        sys.path.insert(0, CAMERA_DIR)


# ===========================================================================
# IDENTITAS ROBOT & JARINGAN (firmware v2)
# ===========================================================================
# Hal yang BERBEDA antar robot. Geometri, kecepatan, dan model gerak sama untuk
# keduanya ("Model Robot ID 01.md" = "Model Robot ID 02.md"), jadi tidak ikut
# di tabel ini.
#
#   tag           id AprilTag (TAG_FAMILY) yang terpasang di robot itu.
#   steer_center  HARUS sama dengan "str_center" di NVS robot itu. Bukan hanya
#                 soal titik lurus: firmware membalik kemudi roda belakang di
#                 sekitar str_center MILIKNYA, sedangkan robot.py mengirim sudut
#                 mutlak = steer_center + kemudi. Selisih d derajat antara
#                 keduanya membuat perintah "lurus" menjadi belok d derajat.
#   cm_per_step   skala langkah, diukur dari jalan nyata per robot. Sejauh ini
#                 keduanya sama dalam 0,1%, tapi itu hasil ukur, bukan anggapan.
#   accel_sps2    HARUS sama dengan setelan "accel" di NVS robot itu - pose.py
#                 memakainya untuk melacak laju motor saat meramal pose. Lihat
#                 bagian KECEPATAN soal kenapa 12000, bukan bawaan firmware 4000.
#   max_steer_deg sudut kemudi NYATA terbesar yang sanggup dicapai, di KEDUA
#                 sisi (perencana menganggapnya simetris, jadi yang dipakai sisi
#                 terlemah). Menentukan R_MIN/R_PLAN/R_SAPU - lihat bagian KEMUDI.
#   kemudi_kiri / kemudi_kanan
#                 kompensasi kemudi per sisi: daftar (perintah, nyata) dalam
#                 derajat, nilai mutlak, naik, mulai (0, 0) - hasil uji busur
#                 TANPA kompensasi. robot.steer_ke_servo membaliknya: untuk
#                 mendapat sudut nyata d, diperintahkan sudut yang menurut tabel
#                 menghasilkan d. None = tanpa kompensasi (perintah = nyata).
ROBOT = {
    # max_steer: uji busur 2026-10-20 setelah perbaikan mekanisme,
    # logs_identifikasi/20261020_112030_id01 (5..35), 20261020_120248_id01
    # (30..60) dan 20261020_120718_id01 (60..70); 4000 sps. Sudut nyata:
    #   kiri  (+): 30/40/50/55/60/65/70 -> 31,0/41,0/51,3/56,5/61,6/67,0/70,1
    #   kanan (-): 30/40/50/55/60/65/70 -> 29,2/38,7/47,9/51,8/57,3/62,7/54,3
    # Beloknya rata sampai 65 (sisa yaw ~1 deg); -70 TIDAK STABIL (sisa yaw
    # 2,8, putar hanya 90 deg dalam 3 s) - sama dengan ID 02. Di sudut kecil
    # kedua sisi seimbang (+-5 -> +5,02/-5,03), jadi bukan titik lurus: sisi
    # kanan tertinggal makin jauh di sudut besar (linkage tak simetris), dan
    # sangat berulang (-60 -> -57,34 / -57,21 di dua uji). Dipilih 55,0 seperti
    # ID 02; sisi kanan dikompensasi lewat kemudi_kanan. Sisi kiri melebihi
    # perintah 1-1,5 deg - belok sedikit lebih rapat, tidak dikompensasi.
    # Rata-rata semua uji di sudut yang sama; 10 deg dianggap tepat (10,14 /
    # 9,79 - derau). Perintah terbesar untuk nyata 55: ~57,9.
    1: dict(tag=1, steer_center=89.45, cm_per_step=0.002491, accel_sps2=12000.0,
            max_steer_deg=55.0,
            kemudi_kanan=[(0, 0.0), (10, 10.0), (15, 14.2), (20, 19.3), (25, 24.5),
                          (30, 29.2), (35, 34.1), (40, 38.7), (45, 43.1), (50, 47.9),
                          (55, 51.8), (60, 57.3), (65, 62.7)]),
    # Dari kemudi_id02.csv (2026-09-30, 5 lintasan 70 cm). kalibrasi_kemudi.py
    # melaporkan pusat 106,1 - tapi skrip itu menganggap pivot firmware 89,
    # padahal str_center robot saat uji 84. Sudut fisik sebenarnya =
    # label - 2 x (89 - 84) = 106,15 - 10 = 96,15. Robot HARUS diberi angka
    # yang sama: "set str_center 96.15" lalu "save".
    #
    # cm_per_step: 0,002490 +- 0,000007 dari 5 lintasan kalibrasi (tali busur),
    # 0,002486 dari 6 lintasan identifikasi.py --mode lurus - praktis SAMA dengan
    # ID 01. Angka 0,002540 yang sempat dipakai salah: jaraknya dijumlah dari
    # |selisih posisi| antar-cuplikan, yang bias +2% oleh derau kamera (lihat
    # Rekaman.laju_dan_kelengkungan di identifikasi.py).
    #
    # accel: NVS ID 02 semula 4000 (bawaan firmware, belum pernah disetel;
    # ramp di kalibrasi cocok dengannya). Disetel 12000 lewat MQTT 2026-09-30
    # bersama str_center 96.15, lalu "save" - dipastikan dengan "show config".
    #
    # max_steer: uji busur 2026-09-30 (logs_identifikasi/20260930_030300_id02,
    # 4000 sps, 9 s per sudut). Kemudi ID 02 jenuh TIDAK SIMETRIS:
    #   kiri  (+): +30 dan +35 sama-sama 29,1 / 28,8 deg nyata (R ~40 cm)
    #   kanan (-): -30 dan -35 sama-sama 23,6 / 23,8 deg nyata (R ~51 cm),
    #              -25 sudah tertahan di 21,7
    # Sampai 20 deg kedua sisi simetris dan penguatannya 0,92 - sama dengan
    # ID 01 (lihat KEMUDI), jadi STEER_SERVO_PER_DEG tidak dibedakan.
    # Sisi kanan membatasi: 23,5. Dugaan kuat mekanis (servo di ~120 deg fisik
    # tertahan linkage/badan) - kalau diperbaiki, ulangi --mode busur lalu naikkan.
    #
    # SETELAH PERBAIKAN MEKANISME (uji busur ulang 2026-09-30 20:46,
    # logs_identifikasi/20260930_204639_id02, kondisi sama: 4000 sps, 9 s):
    # kemudi kini linear dan SIMETRIS - penguatan 0,993 (semula 0,924) - tapi
    # jenuh LEBIH AWAL di kedua sisi:
    #   kiri  (+): +25/+30/+35 -> 22,5 / 22,2 / 22,4 deg nyata (semula 29)
    #   kanan (-): -25/-30     -> 23,7 / 22,5 deg nyata
    # Sisi terlemah 22,2 -> 22,0. Uji lurus 20:57: titik lurus -0,36 +- 0,23 deg,
    # tak berbeda dari nol, jadi steer_center tetap. Uji tangga 20:58/20:58:
    # tunda total 0,42 / 0,28 s, rata-rata sama dengan model simulator.
    #
    # !! "JENUH" 20:46 DI ATAS KELIRU - itu batas PROGRAM, bukan roda. Saat uji
    # itu max_steer_deg masih 23,5, dan robot.steer_ke_servo memotong setiap
    # perintah ke angka itu sebelum dikirim. Uji busur 2026-10-20 11:31 dengan
    # batas 22,0 mengulang hal yang sama (datar di 21,8 / 23,2). identifikasi.py
    # kini melepas batas itu selama uji (--kemudi-maks, bawaan = sudut terbesar).
    # Uji busur ulang 2026-10-20 11:42 (logs_identifikasi/20261020_114218_id02,
    # 4000 sps, 9 s, batas dilepas), setelah perbaikan mekanisme kedua robot:
    #   kiri  (+): +25/+30/+35 -> 25,1 / 29,5 / 33,9 deg nyata
    #   kanan (-): -25/-30/-35 -> 25,5 / 30,5 / 35,3 deg nyata
    # Penguatan 1,004, tidak jenuh sampai 35.
    #
    # Lalu 30..70 deg (--busur-dari 30 --busur-sampai 70; durasi dipendekkan
    # ke setengah putaran di sudut besar), logs_identifikasi/20261020_115322_id02
    # (30..65) dan 20261020_115857_id02 (65, 70). Sudut nyata, kiri / kanan:
    #   40 -> 39,5 / 39,6     50 -> 49,2 / 49,5     55 -> 53,1 / 54,3
    #   60 -> 57,8 / 59,6     65 -> 62,3 / 63,8 (uji 2: 60,8 / 61,5)
    #   70 -> 65,2 / 48,9 - kanan TIDAK STABIL: laju yaw berayun 5..58 deg/s
    #         (roda tertahan lalu lepas), sisa yaw 5,6 deg; kiri rata.
    # Tidak ada jenuh tegas, tapi mulai 55 roda tertinggal 1-3 deg dan berbeda
    # antar-uji. Dipilih 55,0: sudut terbesar yang masih konsisten; rencana
    # (x PLAN_STEER_RATIO) ~41 deg, yang tertinggal < 1 deg. R_MIN 15,8 cm,
    # R_PLAN 25,7 cm (pada 35: 32,1 / 45,6). ID 01 juga 55,0 (lihat di atas).
    # Model DQfD model_dqn/id02_jauh.pt HARUS dilatih ulang setiap nilai ini berubah
    # (kelengkungan aksinya diturunkan dari sini); perencana_dqfd menolak model
    # yang profilnya tidak cocok.
    2: dict(tag=2, steer_center=96.15, cm_per_step=0.002490, accel_sps2=12000.0,
            max_steer_deg=55.0),
}

# Robot yang dikendalikan. Pilih lewat "--id" di navigasi.py, atau lewat
# environment variable untuk skrip lain:  $env:FORKLIFT_ID = 2
ROBOT_ID = int(os.environ.get("FORKLIFT_ID", "1"))

MQTT_HOST = "10.210.1.1"
MQTT_PORT = 1883
MQTT_USER = "forklift01"
MQTT_PASS = "testing88"


def kemudi_perintah(tabel, nyata_deg):
    """Sudut nyata (nilai mutlak) -> sudut yang harus diperintahkan, menurut tabel
    kompensasi (perintah, nyata) satu sisi. Tabel kosong/None = tanpa kompensasi.
    Di luar ujung tabel perintahnya tertahan di perintah terbesar yang diuji."""
    if not tabel:
        return nyata_deg
    perintah, nyata = zip(*tabel)
    i = bisect.bisect_right(nyata, nyata_deg)
    if i >= len(nyata):
        return float(perintah[-1])
    f = (nyata_deg - nyata[i - 1]) / (nyata[i] - nyata[i - 1])
    return perintah[i - 1] + f * (perintah[i] - perintah[i - 1])


def profil(robot_id=None):
    """Seluruh nilai yang BERBEDA antar-robot, sebagai satu objek.

    pilih_robot() di bawah menimpa global modul ini, dan itu cukup selama satu
    proses hanya memegang satu robot. Begitu satu proses memegang dua robot
    (navigasi.py --id 1,2), global tidak bisa dipakai lagi: nilainya milik
    robot yang TERAKHIR dipilih, dan robot lain diam-diam ikut memakainya -
    str_center ID 02 terkirim ke ID 01, topik MQTT tertukar. Jadi setiap bagian
    yang memegang satu robot tertentu (Robot, SumberKamera, SumberSimulasi,
    Perencana) membaca dari objek ini, bukan dari global.

    robot_id None = robot yang sedang dipilih lewat pilih_robot().
    """
    rid = ROBOT_ID if robot_id is None else robot_id
    if rid not in ROBOT:
        raise SystemExit(f"[!] Robot ID {rid} tidak dikenal. "
                         f"Yang terdaftar di konfig.ROBOT: {sorted(ROBOT)}")
    r = ROBOT[rid]
    prefix = f"forklift{rid:02d}"
    max_rad = math.radians(r["max_steer_deg"])
    r_plan = WHEELBASE_CM / math.tan(max_rad * PLAN_STEER_RATIO)
    return SimpleNamespace(
        id=rid,
        tag=r["tag"],
        prefix=prefix,
        topik_rc=f"{prefix}/in/rc",
        topik_cmd=f"{prefix}/in/cmd",
        topik_telemetri=f"{prefix}/out/telemetry",
        topik_state=f"{prefix}/out/state",
        topik_log=f"{prefix}/out/log",
        steer_center=r["steer_center"],
        cm_per_step=r["cm_per_step"],
        accel_sps2=r["accel_sps2"],
        max_steer_deg=r["max_steer_deg"],
        kemudi_kiri=r.get("kemudi_kiri"),
        kemudi_kanan=r.get("kemudi_kanan"),
        max_steer_rad=max_rad,
        r_min_cm=WHEELBASE_CM / math.tan(max_rad),
        r_plan_cm=r_plan,
        # Radius sapuan sudut terluar badan saat berbelok penuh pada radius rencana.
        r_sapu_cm=math.hypot(r_plan + HALF_WIDTH_CM, BODY_FRONT_CM),
    )


def pilih_robot(robot_id):
    """Pasang identitas robot: topik MQTT, id tag, dan titik lurus kemudi.

    Harus dipanggil SEBELUM Robot dan SumberKamera dibuat - keduanya membaca
    nilai di sini saat dibangun (kecuali diberi profil sendiri).
    """
    global ROBOT_ID, _PREFIX, TOPIK_RC, TOPIK_CMD, TOPIK_TELEMETRI, TOPIK_STATE, \
        TOPIK_LOG, TAG_ID_ROBOT, STEER_SERVO_CENTER, CM_PER_STEP, ACCEL_SPS2, \
        SIM_ACCEL_SPS2, MAX_STEER_DEG, MAX_STEER_RAD, R_MIN_CM, R_PLAN_CM, R_SAPU_CM
    p = profil(robot_id)
    ROBOT_ID = p.id
    _PREFIX = p.prefix
    TOPIK_RC = p.topik_rc
    TOPIK_CMD = p.topik_cmd
    TOPIK_TELEMETRI = p.topik_telemetri
    TOPIK_STATE = p.topik_state
    TOPIK_LOG = p.topik_log
    TAG_ID_ROBOT = p.tag
    STEER_SERVO_CENTER = p.steer_center
    CM_PER_STEP = p.cm_per_step
    ACCEL_SPS2 = p.accel_sps2
    SIM_ACCEL_SPS2 = ACCEL_SPS2

    MAX_STEER_DEG = p.max_steer_deg
    MAX_STEER_RAD = p.max_steer_rad
    R_MIN_CM = p.r_min_cm
    R_PLAN_CM = p.r_plan_cm
    R_SAPU_CM = p.r_sapu_cm


# Firmware menghentikan motor bila aliran "rc" putus lebih lama dari ini
# (RC_TIMEOUT_MS di drive.cpp). Loop kendali HARUS mengirim lebih cepat dari ini.
RC_TIMEOUT_S = 0.5
RC_PERIODE_S = 0.05          # 20 Hz

# ===========================================================================
# GEOMETRI ROBOT - dari "Model Robot ID 01.md"
# ===========================================================================
WHEELBASE_CM = 22.5          # poros roda depan (penggerak) -> poros roda belakang (kemudi)
BODY_WIDTH_CM = 17.5         # lebar robot
ROBOT_HEIGHT_CM = 16.5       # tinggi robot = tinggi AprilTag di atas lantai
TAG_SIZE_CM = 10.0           # dimensi AprilTag
TAG_TO_FRONT_CM = 9.5        # pusat tag -> ujung DEPAN robot

# Tidak dipakai: jarak terluar roda depan 13 cm dan lebar roda 2,5 cm. Keduanya
# lebih kecil dari lebar badan 17,5 cm, jadi roda berada di dalam badan dan
# tidak menentukan jejak robot.

# Diukur 2026-09-26, tercatat di "Model Robot ID 01.md". Badan menjulur 0,5 cm
# di belakang poros roda belakang (23 - 22,5), jadi roda kemudi tidak berada
# tepat di ujung.
BODY_FRONT_CM = 4.0          # poros roda depan -> ujung depan robot
BODY_REAR_CM = 23.0          # poros roda depan -> ujung belakang robot
GEOMETRI_DIUKUR = True

# --- turunan ---------------------------------------------------------------
PANJANG_CM = BODY_FRONT_CM + BODY_REAR_CM
HALF_WIDTH_CM = BODY_WIDTH_CM / 2.0

# Pusat tag terhadap poros depan. Positif = poros depan DI DEPAN pusat tag.
# Pose yang dipakai perencana & pelacak adalah poros DEPAN, bukan pusat tag:
# roda depan tidak berbelok, sehingga robot selalu bergerak searah heading di
# titik itu - itulah yang membuat model sepeda berlaku untuk kemudi belakang.
TAG_TO_FRONT_AXLE_CM = TAG_TO_FRONT_CM - BODY_FRONT_CM

# Pusat badan terhadap pusat tag (dipakai area_constraint sebagai "geser_cm").
GESER_CM = TAG_TO_FRONT_CM - PANJANG_CM / 2.0

# Jarak aman tambahan di luar badan saat memeriksa tabrakan.
#
# Angka ini HARUS lebih besar dari simpangan pelacakan yang sebenarnya terjadi,
# karena perencana menjamin jalurnya bebas - bukan jalur yang benar-benar
# ditempuh robot. Diukur lewat simulasi di peta nyata (honeywell & Rumah):
#
#   margin   tercapai   melanggar zona   simpangan maks
#     2 cm      9/10           1            7,8 cm
#     4 cm      8/10           2            8,1 cm
#     6 cm     10/10           0            8,4 cm
#    10 cm     10/10           0           10,5 cm
#
# Uji ulang DENGAN derau pose 1,2 cm di peta Rumah (yang tersempit) menunjukkan
# 6 cm ternyata belum cukup - derau menambah simpangan sampai 12 cm:
#
#   margin   tercapai   melanggar zona
#     6 cm      3/6            3
#     8 cm      6/6            0        <- dipakai
#
# Uji tanpa derau di atas terlalu bersahabat: tanpa derau, simpangan hanya
# berasal dari geometri jalur. Kamera nyata selalu berderau.
#
# Padanan v1 hanya 5 px = 2,0 cm, dan itu memang cukup di arena yang lapang.
# Peta Rumah jauh lebih rapat (31,8% terisi), sehingga simpangan 8 cm cukup
# untuk menyenggol zona. Menaikkannya lebih jauh tidak gratis: makin besar
# margin, makin sedikit celah yang bisa dilewati.
#
# Tabel di atas diukur saat BODY_FRONT/BODY_REAR masih asumsi (6,0 / 22,5).
# Setelah diukur menjadi 4,0 / 23,0 badan robot sedikit lebih pendek, dan
# margin 6,0 tetap lolos 11/11 rute - jadi angkanya tidak diubah. Kalau nanti
# geometri berubah lagi, ulangi sapuan ini daripada menebak.
# Sempat diturunkan ke 4,0 sebagai jalan keluar dari kegagalan perencanaan.
# MARGIN_CADANGAN_CM di bawah menyelesaikan persoalan itu dengan benar, jadi
# nilai penuhnya dikembalikan. Diukur di peta Rumah dengan derau pose 1,2 cm,
# 20 rute - inilah yang menyebabkan robot menabrak zona, BUKAN kecepatannya:
#
#   setelan                        tercapai   melanggar zona
#   margin 4 rata                    6/20          13
#   margin 6 + cadangan [4]         11/20           8
#   margin 8 + cadangan [6,4]       19/20           1     <- dipakai
#   margin 8, tapi dipelankan       16/20           4
#
# Baris terakhir penting: MEMPERLAMBAT robot justru memperburuk. Luncuran di
# titik ganti arah tetap ~4,6 cm pada kecepatan tiba 3,6 maupun 7,1 cm/detik,
# karena yang menentukannya seg_end_tol (4,1 cm) - jarak yang sudah terlanjur
# ditempuh sebelum pelacak sadar segmennya habis - bukan jarak pengereman.
# Dengan margin 4, luncuran itu menghabiskan seluruh margin; dengan margin 8
# ia terserap.
SAFETY_MARGIN_CM = 8.0

# Margin cadangan, dicoba berurutan kalau margin penuh tidak menghasilkan jalur.
#
# Ini BUKAN pelonggaran diam-diam. Margin keselamatan ada untuk menjaga JALUR
# menjauh dari zona, tapi pose AWAL bukan pilihan - robot sudah berada di situ.
# Menolaknya membuat robot terkunci di tempat: setiap tujuan gagal dengan
# "tidak ada jalur", dan tidak ada tujuan mana pun yang bisa menolong.
#
# Persis itu yang terjadi saat SAFETY_MARGIN_CM dinaikkan 6 -> 8 cm: robot yang
# sedang diparkir tertolak SELISIH 0,38 cm, dan seluruh perencanaan mati. Ruang
# arena yang bisa ditempati di peta Rumah juga turun dari 21,1% ke 16,2%.
#
# Margin yang benar-benar dipakai selalu dilaporkan ke layar dan ke log, jadi
# operator tahu kapan robot berjalan dengan jarak aman yang dikurangi.
# 4,0 DIBUANG dari daftar pada 2026-09-27. Jalan nyata menunjukkan tujuan yang
# hanya bisa dicapai dengan margin 4 cm memang berakhir dengan badan robot di
# dalam zona: simpangan akhir 7,8 cm melebihi 4 cm yang tersisa. Itu sejalan
# dengan pengukuran simulasi - margin 4 menghasilkan 11-13 pelanggaran per
# 20-40 rute di peta Rumah, margin 8 menghasilkan nol.
#
# Kalau margin 6 pun tidak bisa, tujuannya memang tidak layak dan lebih baik
# ditolak dengan jelas daripada dijalani dengan jarak aman yang sudah terbukti
# tidak cukup.
MARGIN_CADANGAN_CM = [6.0]

# ===========================================================================
# KEMUDI
# ===========================================================================
# Roda jenuh, dan meminta lebih dari yang sanggup diberikannya hanya membuat
# perencana menyusun tikungan yang tidak bisa diikuti.
#
# v1 mengukur perintah 38,6 derajat menghasilkan 33,6 derajat nyata. Diukur
# ulang 2026-09-27 dari jalan nyata (cusp.csv, hanya saat kemudi sudah menetap
# >= 1,5 detik):
#
#   diminta 15-25 deg -> nyata 21,7 (rasio 0,92)
#   diminta 25-30 deg -> nyata 26,3 (rasio 0,93)
#   diminta 34-36 deg -> nyata 23,7 (rasio 0,68)   <- responsnya rusak di sini
#   sudut nyata TERBESAR yang pernah tercapai: 34,0 deg
#
# Sempat diturunkan ke 33 karena rasio 0,68 itu - lalu DIBATALKAN setelah tiap
# episode kemudi mentok diperiksa satu per satu, bukan dirata-rata:
#
#   episode  lama    sudut nyata: awal -> akhir (maks)
#      0     0,31 s   11,8 -> 10,4  (11,8)
#      2     1,86 s   13,3 -> 21,0  (31,1)
#      4     1,37 s    9,8 -> 35,0  (36,0)   <- episode tabrakan itu sendiri
#
# Episode 4 mencapai 35,0 derajat dan memuncak di 36,0. Roda SANGGUP. Rasio
# rendah tadi hanya transien menuju ke sana - rata-rata yang mencampur transien
# dengan keadaan tunak menyembunyikannya.
#
# Menurunkannya ke 33 juga tidak gratis: radius rencana melebar 45,6 -> 48,8 cm,
# dan di peta Rumah itu memaksa perencanaan turun dari margin 6 cm ke 4 cm -
# margin yang sudah terbukti menghasilkan jauh lebih banyak pelanggaran.
#
# Yang terungkap justru hal lain: mencapai kemudi mentok butuh ~1,4 detik,
# sedangkan CUSP_PAUSE_S hanya 0,6 detik.
#
# Semua di atas adalah ID 01 (35,0). MAX_STEER_DEG kini per robot - tabel ROBOT;
# sejak perbaikan mekanisme 2026-10-20 keduanya 55,0. MAX_STEER_RAD, R_MIN_CM,
# R_PLAN_CM, dan R_SAPU_CM ikut dihitung di pilih_robot().

# Perencana hanya memakai sebagian sudut kemudi; sisanya cadangan untuk koreksi
# pelacak. Tanpa cadangan ini, jalur dibuat persis di batas kemampuan kendaraan
# dan setiap simpangan kecil tidak bisa dikoreksi lagi.
PLAN_STEER_RATIO = 0.75

# STEER_SERVO_CENTER per robot, lihat tabel ROBOT di atas.
STEER_SERVO_PER_DEG = 1.0
# = str_inv firmware (kedua robot 1: kemudi roda belakang). Menentukan ARAH
# koreksi titik lurus: dengan str_inv = 1 dan str_center = STEER_SERVO_CENTER,
# sudut servo fisik = center - kemudi, jadi robot yang melenceng ke kanan butuh
# center yang LEBIH KECIL.
STEER_INVERT = True
STEER_SERVO_MIN = 10.0       # = str_min firmware
STEER_SERVO_MAX = 170.0      # = str_max firmware

# ===========================================================================
# KECEPATAN (langkah/detik - satuan yang dimengerti firmware)
# ===========================================================================
# Diukur 2026-09-27 dari jalan nyata (cusp.csv, 337 cuplikan TUNAK):
#   0,002491 +- 0,000009 cm/langkah
# Nilai v1 (0,0026) meleset -4,2%. Hanya cuplikan saat perintah kecepatan sudah
# tetap >= 1,5 detik yang dipakai; sisanya tercemar episode ketika motor tidak
# sanggup mengikuti perubahan perintah.
#
# Catatan yang belum bisa disimpulkan: pada 6000-9000 sps angkanya turun ke
# 0,002204 (-12%), sedangkan pada 1000-3000 sps 0,002505. Itu tanda SELIP di
# kecepatan tinggi - torsi stepper turun seiring laju - tapi n hanya 15, terlalu
# sedikit untuk ditindaklanjuti. Kalau terulang di log berikutnya, artinya
# kecepatan maju harus diturunkan, bukan skalanya yang diubah.
#
# Semua di atas adalah ID 01. CM_PER_STEP kini per robot - tabel ROBOT.

# Disetel 2026-09-26 setelah robot terasa lambat di perangkat. Pengukuran
# simulasi (10 rute, peta Rumah + Kalibrasi) menunjukkan sebab yang tidak
# terduga: robot hampir TIDAK PERNAH memakai kecepatan puncaknya.
#
#   setelan                              waktu   laju    di kecepatan penuh
#   9000 / curve 0.55 / final 73 cm     225 s   6,7 cm/s      0,4%
#   + zona belokan akhir 73 -> 35 cm    206 s   7,3 cm/s      0,5%
#   + curve 0.55 -> 0.70                187 s   8,0 cm/s      0,5%
#   + maju 9000 -> 12000                167 s   9,0 cm/s      0,5%
#   + min ratio 0.35 -> 0.50            153 s   9,8 cm/s      0,5%   <- dipakai
#   + maju 15000                        141 s  10,6 cm/s      0,5%
#
# Diuji ULANG setelah simulator dilengkapi model jeda (kemudi 0,65 detik,
# kamera 25 fps + latensi berayun), 30 rute per baris, derau pose 1,2 cm:
#
#   FORWARD_SPEED   waktu   simpangan   galat akhir   jarak terdekat ke zona
#        9000      161,9 s    2,80 cm     4,78 cm          5,74 cm
#       10500      142,5 s    2,63 cm     4,74 cm          6,11 cm
#       12000      143,0 s    2,86 cm     4,68 cm          5,79 cm   <- dipakai
#       15000      138,7 s    2,74 cm     4,09 cm          5,72 cm
#
# Ketelitian praktis DATAR di seluruh rentang, bahkan dengan jeda aktif, dan
# tidak ada pelanggaran zona di keempatnya. 12000 dipilih sebagai lutut kurva:
# 15000 hanya memangkas 3% waktu lagi sambil menambah risiko stall dan tag
# kabur yang tidak terlihat di simulasi.
#
# Yang membatasi ternyata rasio-rasio perlambatan, bukan kecepatan puncak:
# jalur Hybrid A* di peta rapat melengkung hampir di mana-mana, sehingga
# curve_speed_ratio berlaku sepanjang rute. Menaikkan FORWARD_SPEED saja hanya
# menaikkan langit-langit yang tidak pernah tersentuh.
#
# Ketelitian TIDAK memburuk: galat akhir 4,1 -> 3,9 cm, simpangan 2,36 -> 2,25
# cm. Ini sejalan dengan temuan v1 bahwa mempercepat lintasan lurus tidak
# menukar akurasi dengan waktu.
FORWARD_SPEED = 10000        # ~31 cm/detik
REVERSE_SPEED = 4000         # ~10 cm/detik; mundur selalu manuver rapat
CURVE_SPEED_RATIO = 0.70     # kecepatan di tikungan tertajam yang direncanakan
MIN_SPEED_RATIO = 0.50       # batas bawah saat melambat mendekati ujung segmen

# Belokan TERAKHIR sebelum tujuan sengaja TETAP LAMBAT: di situ galat tidak
# sempat dikoreksi lagi karena jalurnya habis, dan sisa simpangan langsung
# menjadi galat posisi akhir. Rasionya tidak diubah - yang diperpendek hanya
# zonanya (lihat final_turn_dist di PARAM_PELACAK), karena 73 cm membuat
# seluruh rute pendek 60-100 cm berjalan pelan, bukan cuma belokan akhirnya.
FINAL_TURN_RATIO = 0.30      # 3600 sps = 9,4 cm/detik

CUSP_PAUSE_S = 0.6           # jeda di titik ganti arah, memberi waktu servo berbelok

# Ramp akselerasi stepper. HARUS SAMA dengan setelan "accel" di NVS firmware -
# periksa dengan "show config" di MMI.
#
# Dipindah ke sini dari bagian MODEL JEDA pada 2026-09-27. Ia dulu bernama
# SIM_ACCEL_SPS2 dan bagian itu menyatakan "tidak mempengaruhi robot nyata sama
# sekali" - dua hal yang tidak pernah cocok, karena komentarnya sendiri menuntut
# nilainya sama dengan NVS. Sekarang pose.py memakainya di jalur perangkat nyata
# (melacak kecepatan yang benar-benar dicapai motor), jadi menaruhnya di bagian
# khusus simulator akan menyesatkan.
#
# Dinaikkan 3000 -> 12000 pada 2026-09-27 setelah jalan nyata menunjukkan inilah
# penyebab tabrakan zona Z8. Pelacak menurunkan perintah 5600 sps dalam 0,6
# detik; pada 3000 sps/detik^2 motor butuh 1,9 detik untuk itu, jadi robot
# memasuki tikungan terakhir pada 17 cm/detik sementara pengendali mengira 4
# cm/detik. Terukur di cusp.csv: rasio laju nyata/perintah mencapai 3,98.
#
# Jarak berhenti berkuasa dari laju penuh, dihitung ulang untuk FORWARD_SPEED
# 7000 (angka lama 62/23/16 cm berasal dari 12000 sps dan tidak berlaku lagi):
#
#   accel    jarak berhenti
#    3000        20,3 cm
#    8000         7,6 cm
#   12000         5,1 cm
#
# DIPASTIKAN 2026-09-27 lewat "show config" di MMI: NVS = 12000, sama dengan
# nilai di bawah. (Bawaan firmware DEF_DRIVE_ACCEL = 4000, jadi robot yang NVS-nya
# dihapus akan kembali ke 4000 - periksa lagi setelah flash ulang.)
#
# Sapuan terhadap cusp.csv sempat menunjukkan kelompok transien paling cocok
# dengan ~2000. Uji tangga gas dengan telemetri firmware (2026-09-27,
# logs_identifikasi/20260927_091649) menunjukkan itu BUKAN akselerasi: stepper
# mengikuti ramp 12000 dengan tepat. Yang terlambat adalah KAMERA - lihat
# SIM_KAMERA_DELAY_S. Nilai di bawah benar, baik sebagai setelan maupun sebagai
# perilaku motor.
#
# Semua di atas adalah ID 01. ACCEL_SPS2 kini per robot - tabel ROBOT - karena
# yang harus diikutinya adalah NVS masing-masing robot.
AKSEL_MINIMUM_SPS2 = 12000.0     # di bawah ini periksa() memberi peringatan

# estop di akhir gerakan MAJU. Dipisah menjadi dua tempat, karena keduanya
# ternyata tidak sama sekali.
#
# estop = forceStop() LALU disableOutputs(): berhenti seketika, tapi kemudian
# arus motor DIPUTUS dan robot meluncur bebas tanpa torsi penahan.
# stop  = stopMove(): melambat terkendali dengan motor tetap berarus.
#
# DI TUJUAN estop tepat: robot memang harus berhenti dan diam di sana, dan
# melepas arus tidak merugikan.
#
# DI TITIK GANTI ARAH sempat diduga merugikan - dua tabrakan di perangkat (Z4
# lalu Z6) terjadi persis setelahnya - tapi setelah luncuran bebas dimodelkan,
# simulasi justru menyanggahnya. Pada accel firmware 3000 sps/s^2:
#
#   estop (arus diputus, meluncur bebas) : berhenti dalam 16 cm
#   stop  (melambat, motor berarus)      : berhenti dalam 62 cm
#
# Ramp bertenaga JUSTRU LEBIH LEMAH daripada luncuran bebas, karena accel-nya
# rendah. Jadi estop tetap dipakai. Yang mencurigakan sekarang bukan estop-nya,
# melainkan accel yang terlalu kecil - 62 cm untuk berhenti berkuasa adalah
# jarak yang mustahil di peta serapat Rumah.
#
# Perbandingan di atas bergantung pada SIM_LUNCUR_DESEL_CM_S2 yang BELUM
# DIUKUR. Estop menang selama luncuran bebas melambat lebih cepat dari
# accel * CM_PER_STEP (7,8 cm/detik^2 pada accel 3000). Ukur dengan
# navigasi.py --log sebelum menganggap ini pasti.
ESTOP_TUJUAN = True
ESTOP_TITIK_GANTI_ARAH = True

# --- Melambat di celah sempit ----------------------------------------------
# Kecepatan diturunkan saat badan robot mendekat ke zona. Yang diukur adalah
# jarak bebas DI LUAR margin keselamatan (pose_margin), jadi nilai 0 berarti
# badan tepat menyentuh batas margin - masih SAFETY_MARGIN_CM dari zona
# sebenarnya, belum menabrak.
#
# Ambangnya tidak bisa dibuat longgar. Sebaran jarak bebas di sepanjang jalur
# yang benar-benar direncanakan:
#
#            peta Rumah        peta Kalibrasi
#   median     5,6 cm             34,6 cm
#   < 4 cm    30,1% titik          0,7% titik
#   < 8 cm    77,3% titik          5,8% titik
#   < 15 cm  100,0% titik         15,7% titik
#
# Ambang 15 cm berarti SELURUH peta Rumah dianggap sempit. Ambang 8 cm pun masih
# aktif 78% waktu dan menambah waktu tempuh 58%, jadi dipakai 4 cm: di bawah itu
# simpangan pelacakan biasa (2,3 cm rata, 6,6 cm terburuk) memang sanggup
# menghabiskan seluruh sisa jarak.
#
# Biaya terukurnya sangat bergantung peta - dan itu wajar, karena yang diukur
# memang kerapatan petanya:
#
#              perlambatan mati   perlambatan aktif   aktif berapa lama
#   Kalibrasi       57,0 s             57,0 s               0,0%
#   Rumah           95,4 s            147,0 s              95,4%
#
# CATATAN JUJUR: simulasi TIDAK menunjukkan manfaat keselamatan - jarak terdekat
# ke zona tetap 2,19 cm dengan maupun tanpa perlambatan. Sebabnya simulator ini
# tidak memodelkan jeda kemudi 0,65 detik, padahal di situlah manfaatnya berada:
# pada kecepatan rendah, jarak yang ditempuh sebelum koreksi kemudi bekerja jauh
# lebih pendek. Manfaatnya nyata di perangkat, bukan di sini - yang terukur di
# sini hanya ongkosnya.
SEMPIT_MULAI_CM = 4.0        # mulai melambat di bawah jarak ini
SEMPIT_PENUH_CM = 0.0        # perlambatan penuh saat menyentuh batas margin
SEMPIT_RATIO = 0.45          # kecepatan saat perlambatan penuh

# Jarak jalur ke depan yang ikut diperiksa, supaya robot mengerem SEBELUM masuk
# celah. Bukan angka sembarang: mengerem dari 31 ke 14 cm/detik pada akselerasi
# firmware 8000 sps/s^2 memakan ~18 cm, jadi melihat lebih pendek dari itu
# membuat perlambatannya tidak sempat terjadi.
SEMPIT_LIHAT_CM = 20.0

# Firmware menganggap kecepatan di bawah RC_MIN_SPS (100) sebagai netral dan
# menghentikan motor. Beberapa faktor perlambatan yang bertumpuk bisa menembus
# batas itu, jadi hasil akhirnya dijaga di atas nilai ini.
SPS_MINIMUM = 600

# ===========================================================================
# MODEL JEDA - HANYA DIPAKAI SIMULATOR
# ===========================================================================
# Angka-angka ini tidak mempengaruhi robot nyata sama sekali. Gunanya membuat
# simulasi cukup jujur untuk menjawab pertanyaan yang tidak bisa dijawab
# simulator tanpa jeda: apakah kompensasi jeda layak, dan apakah kecepatan
# 31 cm/detik masih terkendali.
#
# JEDA KEMUDI. Diukur di v1 (PERBAIKAN_2026-09-15.md): korelasi silang antara
# sudut kemudi yang diperintahkan dan kelengkungan yang benar-benar terjadi
# memuncak pada 13 frame = 0,65 detik, r = 0,94-0,96. Dipecah menjadi tunda
# murni (transport) dan tetapan waktu orde-1, karena responsnya bukan lompatan
# melainkan naik bertahap.
# Nilainya disetel sampai KORELASI SILANG model ini memuncak di 0,65 detik -
# metrik yang sama persis dengan cara v1 mengukurnya di robot nyata, bukan
# sekadar menjumlahkan tunda + tau di atas kertas.
#
# !! TERUKUR ULANG 2026-09-27, SETELAH PERBAIKAN KAMERA - dua uji tangga
# (identifikasi.py --mode tangga, logs_identifikasi/20260927_100034 pada 4000
# sps dan 20260927_101224 pada 3000 sps), dalam kerangka waktu KAMERA:
#                                 uji 1           uji 2
#   korelasi silang               0,42 s          0,30 s        (v1: 0,65 s)
#   pencocokan yaw: tunda d       0,41 s          0,12 s
#                   tau           ~0              0,12 s
#                   d + tau       0,41 s          0,24 s
# Angka pastinya BERBEDA antar-uji (selisih ~0,17 s, sebesar variasi jalur
# perintah yang terlihat di uji gas), tapi dua hal konsisten: tau KECIL, jauh
# dari 0,72, dan keterlambatan efektif d + tau ~0,35 s. Pencocoknya terbukti
# sanggup melihat tau bila ada: di simulator ia membaca 0,714 untuk tau
# tertanam 0,72 (uji_gas.py).
#
# Sapuan satu model untuk KEDUA uji (sisa yaw RMS): tunda 0,25 + tau 0,10
# memberi 0,59 / 0,57 deg, hanya ~0,1 deg dari optimum masing-masing
# (0,49 / 0,44). Model di bawah (0,22 + 0,72) memberi 2,13 / 2,06 deg. Jadi
# tunda lama (0,22) kurang lebih benar; tau-nya (0,72) yang salah besar.
# Setara untuk simulator (kamera dimodelkan terpisah ~0,06 s): tunda ~0,19,
# tau ~0,10.
#
# DITERAPKAN 2026-09-27 (semula 0,22 / 0,72). Ini menggeser simulator, matriks
# state_space.py, dan ramalan pose di pose.py sekaligus. Setiap tabel "diukur
# di simulasi" di berkas ini dibuat dengan kemudi lama yang ~7x lebih lamban -
# anggap tabel itu sejarah, bukan ukuran robot sekarang. Log navigasi
# jalan_setelah_kamera.csv tidak bisa membedakan kedua model (pada ~5 cm/detik
# selisihnya ~0,02 cm dalam 0,4 s); yang bisa hanya uji tangga.
SIM_KEMUDI_TUNDA_S = 0.19    # tunda murni (semula 0,22)
SIM_KEMUDI_TAU_S = 0.10      # tetapan waktu orde-1 (semula 0,72)

# Batas laju servo dari firmware: STEER_STEP_MS = 2 ms per derajat.
SIM_SERVO_SLEW_DEG_S = 500.0

# JEDA KAMERA. Latensi pemrosesan 0,025 detik diukur di v1. Yang sering
# terlupakan justru fps-nya: pose hanya diperbarui 25x per detik sementara loop
# kendali berjalan 20 Hz, sehingga usia frame berayun 0-40 ms tergantung fase.
# Itulah "delay yang berubah-ubah" yang sebenarnya.
#
# !! TERUKUR 2026-09-27: LATENSI KAMERA UJUNG-KE-UJUNG ~0,68 detik, bukan 0,025.
# 0,025 adalah latensi PEMROSESAN saja; latensi aliran RTSP (enkode kamera,
# jaringan, dekode) tidak pernah diukur sebelumnya. Diukur dengan menggeser
# posisi kamera terhadap hitungan langkah firmware (pos di telemetri) sampai
# berimpit: 0,725 / 0,685 / 0,675 s di tiga lintasan, sisa 0,08-0,17 cm,
# cm/langkah cocok dalam 0,3% - pergeseran WAKTU murni, bukan selip
# (identifikasi.py --dari logs_identifikasi/20260927_091649). Itu batas bawah:
# latensi telemetri kembali ke PC belum ditambahkan.
#
# PENYEBABNYA DITEMUKAN DAN DIPERBAIKI di hari yang sama: dekoder FFmpeg di
# OpenCV memakai thread sebanyak inti CPU (16), dan dekode multithread per-frame
# menahan ~15 frame. cctv_lurus.StreamReader sekarang membuka aliran dengan 1
# thread (cctv_lurus.buka_rtsp): frame keluar ~0,59-0,61 s lebih dulu
# (uji_kamera.py). DIPASTIKAN DI ROBOT 2026-09-27 (identifikasi.py --mode gas,
# logs_identifikasi/20260927_094202): latensi kamera 0,085 / 0,095 / 0,095 s,
# turun dari 0,675-0,725 s - cocok dengan perkiraan. Nilai simulator di bawah
# (0,025 + umur frame + jitter) terukur ~0,04-0,05 s dengan cara yang sama,
# jadi simulator kini masih ~0,05 s LEBIH CEPAT dari robot - jauh lebih dekat
# daripada selisih 0,63 s sebelumnya, tapi belum sama.
#
# Semua rekaman robot SEBELUM perbaikan (cusp.csv, uji gas 085850 & 091649,
# logs_identifikasi lama) dibuat dengan kamera ~0,68 s. Tabel "diukur di
# simulasi" di berkas ini memakai anggapan 0,025 s, yang tidak cocok dengan
# robot waktu itu tapi kemungkinan cocok dengan robot sekarang.
SIM_KAMERA_FPS = 25.0
SIM_KAMERA_DELAY_S = 0.025
# Ragam waktu dekode + jaringan per frame. Tanpa ini usia frame di simulasi
# menjadi konstan (fase terkunci oleh dt tetap), padahal justru ragamnya yang
# ditanyakan: RTSP nyata tidak pernah datang serapi itu.
SIM_KAMERA_JITTER_S = 0.020

# Ramp akselerasi stepper. Bukan nilai khusus simulator lagi - lihat
# ACCEL_SPS2 di bagian KECEPATAN. Nama lama ini dipertahankan agar simulator,
# state_space.py, dan identifikasi.py tidak perlu diubah, dan agar mengubah
# ACCEL_SPS2 di berkas ini otomatis mengubah keduanya: simulator yang memakai
# accel lain dari robotnya hanya menghasilkan kesimpulan yang tidak berlaku di
# perangkat. Keduanya diisi pilih_robot(); program yang mengubah
# konfig.ACCEL_SPS2 sendiri saat berjalan harus mengubah alias ini juga.

# Perlambatan saat motor TANPA ARUS, yaitu setelah estop mematikan driver.
#
# estop firmware memanggil forceStop() LALU disableOutputs() - arus ke motor
# diputus. Stepper tanpa arus tidak punya torsi penahan, jadi robot MELUNCUR
# BEBAS; yang memperlambatnya hanya gesekan gelinding dan torsi detent motor.
# Itu KEBALIKAN dari "stop" biasa, yang mengerem dengan motor tetap berarus.
#
# Versi simulator sebelumnya memodelkan estop sebagai berhenti seketika - rem
# sempurna. Justru itu yang membuat simulasi dulu menyimpulkan estop di titik
# ganti arah memperbaiki keadaan, padahal di perangkat ia melepas rem.
#
# BELUM DIUKUR. Perkiraan kasar dari torsi detent NEMA17 (2-4 mNm) lewat roda
# berjari-jari ~1,3 cm pada robot ~0,5-1 kg memberi 20-60 cm/detik^2, yaitu
# luncuran 8-24 cm dari kecepatan penuh. Angka di bawah di tengah rentang itu.
# Ukur dengan "navigasi.py --log" lalu lihat jendela sekitar estop; lantai
# keramik akan berada di ujung yang lebih licin.
SIM_LUNCUR_DESEL_CM_S2 = 30.0

# Langkah integrasi internal simulator. Harus jauh lebih kecil dari periode
# frame kamera (40 ms) agar laju kamera benar-benar terwakili.
SIM_SUBSTEP_S = 0.005

# ===========================================================================
# GRID PERENCANAAN
# ===========================================================================
# Occupancy grid dibuat 1 sel = GRID_CM cm. Dengan 1,0 cm per sel, jarak yang
# dikembalikan cv2.distanceTransform langsung bersatuan cm.
GRID_CM = 1.0

# ===========================================================================
# PARAMETER PERENCANA (cm) - dikonversi ke sel oleh rencana.py
# ===========================================================================
# Angka-angka ini hasil konversi dari nilai piksel v1 yang sudah tersetel
# (PX_PER_CM = 34,8/14,1 = 2,468), jadi tuning lama tidak hilang.
PARAM_RENCANA = dict(
    xy_reso=4.9,             # 12 px
    yaw_reso_deg=10.0,
    step=7.3,                # 18 px
    sample_ds=2.4,           # 6 px
    n_steer=7,
    reverse_cost=2.0,
    switch_cost=32.4,        # 80 px
    steer_cost=0.3,
    steer_change_cost=6.0,
    clearance_pref=6.1,      # 15 px
    clearance_cost=0.8,
    # f = g + w*h, h = max(h_RS, h_D): biaya Reeds-Shepp tanpa rintangan
    # (heuristik_rs.py) dan Dijkstra 2D dengan rintangan. w = 2,0 untuk SEMUA
    # Hybrid A* (pembanding dan demonstrator DQfD) dan untuk pencarian DQfD.
    h_weight=2.0,
    h_reso=4.9,              # 12 px
    approach_len=32.4,       # 80 px
    goal_pos_tol=3.2,        # 8 px
    goal_yaw_tol_deg=12.0,
    time_limit=2.0,
    max_expansions=30000,
)
# Aturan tembakan Reeds-Shepp per peran Hybrid A* (rencana.buat_perencana):
#   "astar"         perencana pembanding (--perencana astar), sejak 2026-11-08
#                   SAMA dengan aturan serah DQfD murni supaya perbandingan adil:
#                   RS dicoba di setiap node < 60 cm dari tujuan, hanya bila peta
#                   Dijkstra tidak memaksa memutar atau sudah < RADIUS_SERAH_PAKSA_CM
#                   (saring_serah, = drl/lingkungan._boleh_coba_serah).
#   "astar-standar" Hybrid A* standar (aturan "astar" sampai 2026-11-08): RS dicoba
#                   di setiap ekspansi ke-4, berapa pun jaraknya ke tujuan.
#   "demonstrator"  demonstrator model jauh DQfD dan pustaka geraknya: RS dicoba
#                   di setiap node yang titik acuannya < 60 cm dari tujuan. Batas
#                   ini juga batas serah model jauh (drl/lingkungan.py).
ATURAN_RS = {
    "astar": dict(analytic_every=None, analytic_dist=60.0, saring_serah=True),
    "astar-standar": dict(analytic_every=4, analytic_dist=0.0),
    "demonstrator": dict(analytic_every=None, analytic_dist=60.0),
}
# Saringan serah (cm): di dalam jarak ini peta Dijkstra tidak lagi disyaratkan
# bebas - untuk tujuan yang mepet rintangan. Tembakan RS tetap harus bebas.
# Dipakai model jauh DQfD (drl/lingkungan.py) dan Hybrid A* peran "astar".
RADIUS_SERAH_PAKSA_CM = 25.0
ALLOW_REVERSE = True

# ===========================================================================
# PARAMETER PELACAK (cm), dipisah per jenis
# ===========================================================================
# Sengaja per jenis, bukan satu daftar datar: pelacak lain (MPC, Stanley) tidak
# memerlukan "lookahead", dan meratakannya akan mengotori konfigurasi tetangga.
PARAM_PELACAK = {
    "pursuit": dict(
        # v1 memakai 100 px = 40,5 cm dan terbukti MEMOTONG tikungan: pada busur
        # radius 44 cm galatnya 2,41 px, sementara 80 px (32 cm) menurunkannya
        # ke 1,02 px. Angka 100 dulu dipilih untuk melawan zig-zag saat gain
        # kemudi masih salah; gain itu sudah benar sekarang, jadi kita mulai
        # langsung dari nilai yang lebih baik.
        # Diturunkan lagi 32 -> 26 setelah forensik menunjukkan bahwa yang
        # membedakan frame "melanggar zona" dari frame normal hanyalah SIMPANGAN
        # (7,22 cm vs 3,56 cm) - bukan titik ganti arah, bukan jeda kemudi,
        # bukan kecepatan. Jadi yang perlu diperbaiki memang simpangannya.
        #
        # Diukur di peta Rumah, derau pose 1,2 cm, 20 rute per baris:
        #
        #   lookahead   melanggar   simpangan rata/maks   gerak servo
        #     40 cm         2          3,06 / 9,57         2,02 deg/frame
        #     32 cm         0          2,44 / 7,99         2,89
        #     26 cm         0          2,18 / 7,20         4,12      <- dipakai
        #     20 cm         0          2,07 / 7,37         6,54
        #
        # 26 cm adalah lututnya: di bawah itu simpangan praktis berhenti membaik
        # sementara gerak servo terus naik - dan gerak servo bukan sekadar angka,
        # itu keausan MG995 yang sebenarnya. Pola yang sama terukur di v1
        # (lookahead 100/80/60 px -> galat 2,41/1,02/0,55 px).
        lookahead=26.0,
        seg_end_tol=4.1,     # 10 px
        slowdown_dist=32.4,  # 80 px
        # Panjang zona "belokan terakhir". Diperpendek dari 73 cm (180 px v1):
        # pada rute pendek 60-100 cm, 73 cm berarti hampir SELURUH rute berjalan
        # di kecepatan belokan akhir. Diuji 20/27/35/50/73 cm - galat posisi
        # akhir rata tetap 3,9-4,0 cm di semuanya, jadi memperpendeknya tidak
        # menukar ketelitian dengan kecepatan.
        #
        # 35 cm dipilih, bukan 20 cm yang sedikit lebih cepat, karena jarak ini
        # harus cukup untuk BENAR-BENAR melambat: dari 12000 ke 3600 sps dengan
        # akselerasi firmware 8000 sps/s^2 butuh ~21 cm. Dengan akselerasi
        # bawaan 4000 butuh ~43 cm dan perlambatannya tidak akan selesai -
        # karena itu firmware harus diset "accel 8000" (lihat README).
        final_turn_dist=35.0,
        # Kompensasi jeda: pose diproyeksikan maju sekian detik sebelum sudut
        # kemudi dihitung, supaya perintah cocok dengan posisi robot saat
        # perintah itu benar-benar bekerja.
        #
        # TETAP NOL, dan kali ini dengan bukti sendiri. v1 menonaktifkannya lalu
        # menulis "siap dicoba kalau nanti lookahead diperkecil" - lookahead
        # memang sudah diperkecil (40,5 -> 32 cm) dan kecepatan dinaikkan, jadi
        # pertanyaannya diuji ulang dengan model jeda lengkap di simulator,
        # 30 rute per baris, derau pose 1,2 cm:
        #
        #   lag_comp   simpangan   servo        jarak terdekat   pelanggaran
        #              rata (cm)   (deg/frame)  ke zona (cm)
        #     0,00       2,86        2,80          5,63              0
        #     0,15       2,80        5,92          5,10              0
        #     0,30       3,01        9,42          4,66              0
        #     0,45       3,05       12,67          3,97              0
        #     0,60       3,15       15,61          3,31              2
        #
        # Simpangan tidak membaik sama sekali, sementara gerak servo melonjak
        # 5,6 kali dan jarak terdekat ke zona memburuk terus. Sebabnya tetap
        # sama seperti di v1: lead dari lookahead (1,03 detik) masih LEBIH
        # BESAR daripada jeda kemudi (0,65 detik), jadi memprediksi maju di
        # atasnya hanya melebihkan bidikan sambil melipatgandakan derau pose
        # menjadi gerakan servo.
        #
        # Kodenya tetap ada di PathTracker._predict() - layak dicoba lagi hanya
        # kalau lookahead nanti turun di bawah ~20 cm.
        lag_comp=0.0,
    ),
}
PELACAK_BAWAAN = "pursuit"

# ===========================================================================
# HITUNG ULANG JALUR
# ===========================================================================
REPLAN_DEVIATION_CM = 16.2   # 40 px
REPLAN_BLOCKED_FRAMES = 3
REPLAN_MIN_INTERVAL_S = 1.0

# ===========================================================================
# MULTI-ROBOT (wasit.py) - lihat "Multi Robot Navigation and Control.md"
# ===========================================================================
# Setiap robot yang sedang berjalan membawa BAYANGAN: jejak badannya sendiri
# di sepanjang jalur ke depan. Robot yang bayangannya akan mengenai robot lain
# berhenti (MENUNGGU) dan melanjutkan setelah bayangannya bebas.
#
# Panjang bayangan = jarak yang PASTI ditempuh sebelum robot benar-benar diam:
#
#     L = MULTI_LIHAT_MIN_CM + v * MULTI_T_REAKSI_S + v^2 / (2 * a_rem)
#
# Rancangan awal memakai waktu tetap 500 ms (L = v * 0,5). Itu linear terhadap
# v, sedangkan jarak berhenti kuadrat. Pada FORWARD_SPEED x CM_PER_STEP = 24,9
# cm/detik bayangannya 12,5 cm, padahal robot butuh ~15 cm untuk diam (reaksi
# 5,0 + pengereman 10,4) - ia berhenti SESUDAH ujung bayangannya sendiri. Dan
# selisihnya membesar kuadratik bila kecepatan dinaikkan. Rumus di atas
# memberi 20,4 cm.
#
# a_rem tidak disetel di sini: MENUNGGU mengirim rc bernilai nol, dan firmware
# melambatkannya dengan ramp "accel" di NVS - jadi a_rem = accel_sps2 *
# cm_per_step robot itu (29,9 cm/detik^2 pada 12000). Itu setelan yang
# tercatat, bukan tebakan seperti SIM_LUNCUR_DESEL_CM_S2.
#
# T_REAKSI: kamera 0,085-0,095 s (terukur 2026-09-27) + satu periode loop
# 0,05 s + MQTT. 0,2 s dibulatkan ke atas. BELUM DIUKUR sebagai satu rantai -
# ukur dari log dua robot sebelum menurunkannya.
MULTI_T_REAKSI_S = 0.2

# Bayangan minimum, bahkan saat diam. Tanpa ini robot yang baru mulai bergerak
# (v ~ 0) punya bayangan nol, dianggap bebas, lalu melaju ke robot lain.
MULTI_LIHAT_MIN_CM = 5.0

# Jarak bebas yang harus tersisa antara bayangan satu robot dan badan robot
# lain. HARUS lebih kecil dari margin cadangan perencana (MARGIN_CADANGAN_CM,
# terkecil 6 cm): jalur memutar yang direncanakan dengan robot lain sebagai
# rintangan lewat sedekat margin itu, dan kalau wasit menuntut lebih, jalur
# memutar itu langsung dianggap bertabrakan lagi dan robot menunggu selamanya.
MULTI_JARAK_AMAN_CM = 5.0

# Wasit, aturan ZONA (2026-11-08): robot yang bayangannya (mengikuti badan, panjang L di
# atas) MENDEKAT ke zona sampai jarak bebas <= ini berhenti dan jalurnya dihitung ulang.
# Jalur perencana menjaga SAFETY_MARGIN_CM (atau margin cadangan, >= 6 cm) dari zona, jadi
# bayangan baru mendekati 1 cm bila badan menyimpang jauh dari jalurnya.
ZONA_JARAK_AMAN_CM = 1.0
# Aturan zona wasit menyala. False hanya untuk pembanding (drl/uji_navigasi.py --tanpa-wasit-zona).
WASIT_ZONA = True

# Lanjut dari MENUNGGU hanya setelah bayangan bebas selama ini, BERTURUT-TURUT.
# Bayangan robot yang menunggu dihitung pada kecepatan TUJUAN-nya, bukan
# kecepatan sekarang (nol) - kalau tidak, robot diam selalu tampak bebas,
# berangkat, bayangannya memanjang, kena lagi, dan berhenti lagi berulang-ulang.
MULTI_LANJUT_S = 0.5

# Menunggu lebih lama dari ini -> jalur dihitung ulang dengan robot lain
# sebagai rintangan. Menangani robot lawan yang PARKIR di jalur (tidak akan
# pernah "lewat") dan dua robot yang berhadapan di koridor.
MULTI_TUNGGU_MAKS_S = 5.0
# Pada konflik berhadapan (keduanya saling menghalangi), robot yang KALAH
# prioritas menghitung ulang lebih cepat dari ini, supaya yang menang tidak
# ikut menunggu 5 detik penuh.
MULTI_TUNGGU_KALAH_S = 1.0

# Selisih sisa jalur yang dianggap "sama dekat ke tujuan" saat menentukan
# prioritas. Di bawah ini ID yang lebih kecil didahulukan.
MULTI_SISA_SETARA_CM = 5.0

# MINGGIR. Jalur memutar tidak selalu ada: dua robot yang BERTUKAR tempat
# (tujuan masing-masing di dekat posisi lawan sekarang) tidak akan pernah
# mendapat jalur memutar selama keduanya berdiri di situ - tujuannya sendiri
# tertutup badan lawan. Terukur di simulasi: keduanya menunggu 85 detik penuh.
# Maka robot yang gagal memutar mencari pose sementara di LUAR sisa jalur
# lawan, pergi ke sana, dan menunggu lawan lewat sebelum melanjutkan.
#
# Jarak calon pose minggir dari posisi robot, dicoba dari yang terdekat.
MULTI_MINGGIR_JARAK_CM = (35.0, 50.0, 70.0, 90.0)
# Badan di pose minggir harus sejauh ini dari sisa jalur lawan. Dua kali jarak
# aman wasit: lawan lewat dengan simpangan pelacakan, bukan tepat di garis.
MULTI_MINGGIR_BEBAS_CM = 10.0
# Calon yang dicoba direncanakan; yang jalurnya terpendek dipakai. Tiap
# percobaan bisa makan sampai time_limit perencana, jadi dibatasi (dan
# navigasi.py memberi anggaran waktu total).
MULTI_MINGGIR_CALON = 10
# Setelah minggir, lanjut ke tujuan begitu lawan diam (sampai tujuannya),
# atau paling lambat setelah selama ini.
MULTI_MINGGIR_MAKS_S = 30.0

# Simulasi: robot kedua dan seterusnya ditaruh sejauh minimal ini dari robot
# lain saat program mulai.
MULTI_JARAK_AWAL_SIM_CM = 60.0

# ===========================================================================
# PENGLIHATAN
# ===========================================================================
TAG_FAMILY = "36h11"
# TAG_ID_ROBOT per robot, lihat tabel ROBOT di atas.
# Tag tidak terlihat selama ini -> pose dianggap hilang sama sekali.
TAG_HILANG_S = 1.0

# Selama tag tidak terdeteksi, pose DIRAMALKAN dengan model ruang-keadaan dari
# perintah yang sedang dikirim, bukan dibekukan di nilai terakhir.
#
# Membekukannya berbahaya justru karena AprilTag satu-satunya umpan balik posisi
# di robot ini: tidak ada encoder, dan "pos" dari firmware hanya hitungan langkah
# yang diperintahkan. Pose beku selama 1 detik pada 31 cm/detik berarti robot
# menempuh 31 cm sementara pengendali mengira ia masih di tempat lama - dan terus
# mengemudi berdasarkan posisi usang itu.
#
# Ramalan hanya sahih sebentar: galatnya tumbuh karena jeda kemudi membuat sudut
# roda yang sebenarnya tidak diketahui persis. 0,4 detik = 12 cm pada kecepatan
# penuh, cukup untuk menjembatani tag yang kabur beberapa frame, dan tidak cukup
# lama untuk menyesatkan.
RAMAL_MAKS_S = 0.4
FRAME_BASI_S = 0.5           # frame kamera tidak diperbarui selama ini -> estop
VIEW_OFFSET_CM = 30.0        # perluasan tampilan di luar area homografi

# Tarik mouse minimal sejauh ini untuk menetapkan arah hadap tujuan.
GOAL_DRAG_MIN_CM = 6.0

FILE_ROBOT_CONSTRAINT = os.path.join(CAMERA_DIR, "robot_constraint.json")
FILE_KALIBRASI = os.path.join(CAMERA_DIR, "kalibrasi_lensa.json")

# Robot bawaan dipasang di SINI, setelah seluruh geometri dan kemudi terdefinisi:
# pilih_robot() menghitung R_MIN/R_PLAN/R_SAPU dari WHEELBASE_CM,
# PLAN_STEER_RATIO, dan badan robot.
pilih_robot(ROBOT_ID)


# ===========================================================================
# PEMERIKSAAN
# ===========================================================================
def _baca_json(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except Exception:
        return None


def periksa():
    """Kembalikan daftar (tingkat, pesan). tingkat: 'x' = fatal, '!' = peringatan."""
    pesan = []

    if not os.path.isdir(CAMERA_DIR):
        pesan.append(("x", f"CAMERA_DIR tidak ditemukan: {CAMERA_DIR}"))
        return pesan

    kal = _baca_json(FILE_KALIBRASI)
    if kal is None:
        pesan.append(("x", "kalibrasi_lensa.json tidak terbaca. "
                           "Jalankan: python cctv_lurus.py kalibrasi"))
    else:
        if not kal.get("homografi"):
            pesan.append(("x", "Belum ada homografi. Jalankan: python cctv_lurus.py homografi"))
        # Tanpa tinggi tag, sudut tag diproyeksikan seolah menempel di lantai.
        # Tag Anda ada 16,5 cm di atas lantai, jadi setiap posisi akan meleset
        # makin jauh dari titik tepat di bawah kamera - dan gejalanya menyesatkan,
        # terlihat seperti kalibrasi kemudi yang salah.
        if not kal.get("tinggi_tag_cm"):
            pesan.append(("!", f"tinggi_tag_cm belum diisi di kalibrasi_lensa.json. "
                               f"Tag berada {ROBOT_HEIGHT_CM:.1f} cm di atas lantai; tanpa nilai ini "
                               f"posisi cm meleset secara sistematis. Isi lewat cctv_lurus.py."))

    # Dimensi robot hidup di dua tempat: di sini (dipakai perencana) dan di
    # robot_constraint.json (dipakai pemantau keselamatan area_constraint).
    # Kalau berbeda, keduanya akan berselisih soal seberapa besar robotnya.
    rc = _baca_json(FILE_ROBOT_CONSTRAINT)
    if rc is None:
        pesan.append(("!", "robot_constraint.json tidak terbaca; area_constraint akan "
                           "memakai bawaan 40x30 cm - jauh lebih besar dari robot ini."))
    else:
        for kunci, milik_kita in (("panjang_cm", PANJANG_CM), ("lebar_cm", BODY_WIDTH_CM),
                                  ("geser_cm", GESER_CM)):
            milik_dia = rc.get(kunci)
            if milik_dia is None or abs(float(milik_dia) - milik_kita) > 0.5:
                pesan.append(("!", f"robot_constraint.json {kunci}={milik_dia} tidak cocok "
                                   f"dengan konfig ({milik_kita:.2f}). "
                                   f"Samakan dengan: python konfig.py --tulis-robot"))

    if not GEOMETRI_DIUKUR:
        pesan.append(("!", "BODY_FRONT_CM / BODY_REAR_CM masih asumsi (belum diukur). "
                           "Jejak badan robot bisa meleset beberapa cm."))

    if STEER_SERVO_CENTER is None:
        pesan.append(("x", f"steer_center robot ID {ROBOT_ID:02d} belum diisi di konfig.ROBOT. "
                           f"Isi dengan str_center yang tersimpan di NVS robot itu."))
    else:
        p = profil()
        for sisi, tabel in (("kemudi_kiri", p.kemudi_kiri), ("kemudi_kanan", p.kemudi_kanan)):
            if tabel and (tuple(tabel[0]) != (0, 0) or
                          any(b[0] <= a[0] or b[1] <= a[1] for a, b in zip(tabel, tabel[1:]))):
                pesan.append(("x", f"{sisi} robot ID {ROBOT_ID:02d} harus mulai (0, 0) dan "
                                   f"kedua kolomnya naik."))
        # Perintah terbesar ke servo = sudut nyata maksimum SETELAH kompensasi.
        kiri = kemudi_perintah(p.kemudi_kiri, MAX_STEER_DEG)
        kanan = kemudi_perintah(p.kemudi_kanan, MAX_STEER_DEG)
        if STEER_SERVO_CENTER + kiri * STEER_SERVO_PER_DEG > STEER_SERVO_MAX or \
                STEER_SERVO_CENTER - kanan * STEER_SERVO_PER_DEG < STEER_SERVO_MIN:
            pesan.append(("x", "Rentang kemudi melampaui batas servo firmware."))

    if ACCEL_SPS2 < AKSEL_MINIMUM_SPS2:
        pesan.append(("!", f"accel robot ID {ROBOT_ID:02d} = {ACCEL_SPS2:.0f} sps/s^2, di bawah "
                           f"{AKSEL_MINIMUM_SPS2:.0f} yang dipakai menyetel pelacak. Motor lambat "
                           f"mengikuti perlambatan - penyebab tabrakan Z8 di ID 01. Setel robot: "
                           f"\"set accel {AKSEL_MINIMUM_SPS2:.0f}\" + \"save\", lalu samakan konfig.ROBOT."))

    return pesan


def tulis_robot_constraint():
    """Samakan robot_constraint.json dengan geometri di berkas ini."""
    rc = _baca_json(FILE_ROBOT_CONSTRAINT) or {}
    rc.update(panjang_cm=round(PANJANG_CM, 2), lebar_cm=round(BODY_WIDTH_CM, 2),
              geser_cm=round(GESER_CM, 2))
    rc.setdefault("margin_cm", 20.0)
    rc.setdefault("prediksi_s", 0.5)
    with open(FILE_ROBOT_CONSTRAINT, "w") as fh:
        json.dump(rc, fh, indent=2)
    print(f"[OK] {FILE_ROBOT_CONSTRAINT} disamakan: "
          f"panjang {rc['panjang_cm']} lebar {rc['lebar_cm']} geser {rc['geser_cm']} cm")


def ringkas():
    print("=== GEOMETRI ROBOT (cm) ===")
    print(f"  wheelbase          {WHEELBASE_CM:7.2f}   (poros depan -> poros belakang)")
    print(f"  badan              {PANJANG_CM:7.2f} x {BODY_WIDTH_CM:.2f}"
          f"{'' if GEOMETRI_DIUKUR else '   [panjang masih ASUMSI]'}")
    print(f"  poros depan->depan {BODY_FRONT_CM:7.2f}")
    print(f"  poros depan->belakang {BODY_REAR_CM:4.2f}")
    print(f"  tag -> poros depan {TAG_TO_FRONT_AXLE_CM:7.2f}   (positif = poros di depan tag)")
    print(f"  geser pusat badan  {GESER_CM:7.2f}   (untuk robot_constraint.json)")
    print(f"  tinggi tag         {ROBOT_HEIGHT_CM:7.2f}")
    print("=== KEMUDI ===")
    print(f"  maksimum           {MAX_STEER_DEG:7.2f} derajat")
    print(f"  radius minimum     {R_MIN_CM:7.2f} cm   (kemudi mentok)")
    print(f"  radius rencana     {R_PLAN_CM:7.2f} cm   (ratio {PLAN_STEER_RATIO})")
    print(f"  sapuan terluar     {R_SAPU_CM:7.2f} cm")
    print(f"  -> robot butuh lingkaran bebas ~{2 * R_SAPU_CM:.0f} cm untuk berputar penuh.")
    print(f"     Koridor lebih sempit dari itu memaksa manuver maju-mundur.")
    print("=== KECEPATAN ===")
    print(f"  maju    {FORWARD_SPEED:6d} sps = {FORWARD_SPEED * CM_PER_STEP:5.1f} cm/detik")
    print(f"  mundur  {REVERSE_SPEED:6d} sps = {REVERSE_SPEED * CM_PER_STEP:5.1f} cm/detik")
    print(f"  tikungan ratio {CURVE_SPEED_RATIO} = {FORWARD_SPEED * CURVE_SPEED_RATIO * CM_PER_STEP:.1f} cm/detik")
    print("=== ROBOT ===")
    print(f"  id {ROBOT_ID:02d} | tag {TAG_ID_ROBOT} | str_center {STEER_SERVO_CENTER}")
    print("=== JARINGAN ===")
    print(f"  broker  {MQTT_HOST}:{MQTT_PORT}")
    print(f"  topik   {TOPIK_RC} | {TOPIK_CMD} | {TOPIK_TELEMETRI}")


def main():
    if "--tulis-robot" in sys.argv:
        tulis_robot_constraint()
        return
    ringkas()
    print("=== PEMERIKSAAN ===")
    pesan = periksa()
    if not pesan:
        print("  semua beres.")
    for tingkat, teks in pesan:
        print(f"  [{tingkat}] {teks}")
    if any(t == "x" for t, _ in pesan):
        sys.exit(1)


if __name__ == "__main__":
    main()
