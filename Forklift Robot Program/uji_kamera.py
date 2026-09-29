"""uji_kamera.py - bandingkan latensi dekode aliran kamera antar-pengaturan, TANPA robot.

Latensi kamera ujung-ke-ujung terukur ~0,68 s (identifikasi.py --mode gas,
2026-09-27). Tersangka utamanya multithreading dekoder FFmpeg: OpenCV membuka
dekoder dengan thread sebanyak inti CPU (16 di PC ini), dan dekode per-frame
multithread MENAHAN keluaran sekitar (thread - 1) frame. Pada 25 fps itu
15 x 40 ms = 0,6 s.

CARA UKUR. Aliran yang sama dibuka beberapa kali sekaligus, masing-masing
dengan pengaturan berbeda. Semua sesi menerima bitstream yang SAMA dari
enkoder kamera, dan dekode H.264 deterministik - jadi frame ke-k keluar
identik bit per bit dari dekoder mana pun. Tiap frame diberi sidik (hash);
selisih waktu munculnya sidik yang sama di dua sesi = selisih latensi kedua
pengaturan, dengan presisi milidetik. Tidak butuh gerakan, robot, maupun
pemasangan apa pun.

Yang terukur adalah SELISIH terhadap pengaturan "lama" (bawaan OpenCV, yang
dipakai StreamReader sampai 2026-09-27), bukan latensi mutlak. Latensi mutlak
diperkirakan = 0,68 s + selisih, lalu dipastikan ulang dengan uji gas di robot.

Hasil 2026-09-27: 1 thread -611 ms, jadi cctv_lurus.StreamReader sekarang
membuka aliran dengan 1 thread (cctv_lurus.buka_rtsp). Baris terakhir laporan,
"StreamReader terpasang", menguji kelas yang benar-benar dipakai program -
bukan tiruannya - dan harus sama dengan "thread 1".

    python uji_kamera.py              20 detik
    python uji_kamera.py --detik 60   lebih lama, untuk melihat apakah latensi MELAYANG
"""
import argparse
import hashlib
import os
import threading
import time

import cv2
import numpy as np

import konfig

konfig.pasang_path_kamera()
import cctv_lurus as C                # noqa: E402

# Latensi ujung-ke-ujung pengaturan "lama", terukur di robot 2026-09-27
# (batas bawah; latensi telemetri kembali ke PC belum ditambahkan).
LATENSI_LAMA_S = 0.68

# (nama, OPENCV_FFMPEG_CAPTURE_OPTIONS, thread dekoder atau None = bawaan OpenCV)
#
# fflags;nobuffer + flags;low_delay sempat diuji 2026-09-27 dan TIDAK menambah
# manfaat di atas 1 thread (-600 vs -611 ms), jadi dikeluarkan: setiap sesi
# tambahan membebani kamera, dan empat sesi sudah memunculkan galat dekode.
PENGATURAN = [
    ("lama (thread bawaan)", "rtsp_transport;tcp", None),
    ("thread 1", "rtsp_transport;tcp", 1),
    ("thread 2", "rtsp_transport;tcp", 2),
]


def buka(url, opsi, thread):
    """Buka satu sesi. Opsi FFmpeg dibaca OpenCV dari environment SAAT membuka,
    jadi sesi dibuka berurutan, bukan bersamaan."""
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = opsi
    if thread is None:
        # persis seperti cctv_lurus.StreamReader - pembanding harus jujur
        return cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    return cv2.VideoCapture(url, cv2.CAP_FFMPEG, [cv2.CAP_PROP_N_THREADS, thread])


class Perekam(threading.Thread):
    """Baca secepat mungkin, catat (waktu, sidik) tiap frame."""

    def __init__(self, cap):
        super().__init__(daemon=True)
        self.cap = cap
        self.catat = []
        self.berhenti = False

    def run(self):
        while not self.berhenti:
            ok, f = self.cap.read()
            t = time.perf_counter()
            if not ok:
                time.sleep(0.01)
                continue
            # Sub-sampel cukup: dua frame berbeda hampir pasti berbeda di sini,
            # dan hash atas 1/16 piksel jauh lebih murah dari seluruh frame.
            sidik = hashlib.blake2b(np.ascontiguousarray(f[::4, ::4]).tobytes(),
                                    digest_size=8).digest()
            self.catat.append((t, sidik))


class PerekamReader(threading.Thread):
    """Catat frame dari cctv_lurus.StreamReader yang TERPASANG, lewat read_new().

    Dipoll tiap 1 ms, jadi waktu tercatat paling lambat 1 ms setelah frame
    tersedia - jauh di bawah selisih yang sedang dicari.
    """

    def __init__(self, reader):
        super().__init__(daemon=True)
        self.reader = reader
        self.catat = []
        self.berhenti = False

    def run(self):
        terakhir = 0
        while not self.berhenti:
            f, fid = self.reader.read_new(terakhir)
            if f is None:
                time.sleep(0.001)
                continue
            terakhir = fid
            t = time.perf_counter()
            sidik = hashlib.blake2b(np.ascontiguousarray(f[::4, ::4]).tobytes(),
                                    digest_size=8).digest()
            self.catat.append((t, sidik))


def sidik_unik(catat):
    """sidik -> waktu, hanya sidik yang muncul SEKALI.

    Pada adegan yang benar-benar diam, dua frame berturut-turut bisa identik;
    sidik seperti itu tidak bisa dipasangkan dengan pasti, jadi dibuang.
    """
    hitung, waktu = {}, {}
    for t, s in catat:
        hitung[s] = hitung.get(s, 0) + 1
        waktu.setdefault(s, t)
    return {s: waktu[s] for s, n in hitung.items() if n == 1}


def main():
    ap = argparse.ArgumentParser(description="Bandingkan latensi dekode kamera antar-pengaturan")
    ap.add_argument("--detik", type=float, default=20.0)
    args = ap.parse_args()

    url = C.rtsp_url()
    asli = os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS")
    print(f"[i] OpenCV {cv2.__version__}, {os.cpu_count()} inti CPU "
          f"(= thread dekoder bawaan)")
    sesi = []
    try:
        for nama, opsi, thread in PENGATURAN:
            cap = buka(url, opsi, thread)
            if not cap.isOpened():
                print(f"[!] {nama}: gagal dibuka, dilewati")
                continue
            sesi.append((nama, cap, Perekam(cap)))
            print(f"[i] {nama}: terbuka")
    finally:
        if asli is None:
            os.environ.pop("OPENCV_FFMPEG_CAPTURE_OPTIONS", None)
        else:
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = asli
    if len(sesi) < 2 or not sesi[0][0].startswith("lama"):
        raise SystemExit("[!] Butuh sesi 'lama' dan minimal satu pembanding.")

    # Perekam dimulai SEGERA. Sesi yang dibuka tapi tidak dibaca membiarkan
    # data menumpuk di soket; dekoder bawaan 16 thread terukur tidak pulih dari
    # tumpukan itu (read() lalu mengulang frame yang sama ratusan kali per
    # detik). Frame sebelum semua sesi berjalan tidak mengganggu: yang dipakai
    # hanya frame yang tertangkap di KEDUA sesi yang dibandingkan.
    for _, _, p in sesi:
        p.start()

    # StreamReader dibuat SETELAH environment dipulihkan: ia membuka alirannya
    # di thread sendiri, dan harus membaca opsi yang sama seperti di program.
    reader = C.StreamReader(url)
    reader.wait_first_frame()
    pr = PerekamReader(reader)
    pr.start()
    sesi.append(("StreamReader terpasang", None, pr))
    print(f"[i] StreamReader terpasang: terbuka")
    print(f"[i] merekam {args.detik:.0f} detik ...")
    time.sleep(args.detik)
    for _, cap, p in sesi:
        p.berhenti = True
    for _, cap, p in sesi:
        p.join(timeout=3)
        if cap is not None:
            cap.release()
    reader.stop()

    # Sesi sehat: fps tidak jauh di atas kamera, dan sebagian besar frame unik.
    # Sesi yang macet mengulang frame yang sama ratusan kali per detik - dipakai
    # sebagai acuan, ia membuat hampir tidak ada frame yang bisa dipasangkan.
    def sehat(p):
        if len(p.catat) < 20:
            return False
        durasi = p.catat[-1][0] - p.catat[0][0]
        fps = (len(p.catat) - 1) / durasi if durasi > 0 else float("inf")
        return fps < 40 and len(sidik_unik(p.catat)) > 0.5 * len(p.catat)

    for nama, _, p in sesi:
        if not sehat(p):
            print(f"[!] {nama}: sesi TIDAK sehat (frame berulang / fps tidak wajar) - "
                  f"barisnya diabaikan")
    acuan_i = 0 if sehat(sesi[0][2]) else next(
        (i for i, (n, _, p) in enumerate(sesi) if n == "thread 1" and sehat(p)), None)
    if acuan_i is None:
        raise SystemExit("[!] Tidak ada sesi acuan yang sehat. Ulangi uji.")
    nama_acuan = sesi[acuan_i][0]
    geser_acuan = 0.0
    if acuan_i != 0:
        # 'thread 1' terukur -611 ms terhadap 'lama' pada uji yang sehat
        # (2026-09-27); dipakai untuk mengembalikan angka ke skala 'lama'.
        geser_acuan = -0.611
        print(f"[!] acuan diganti ke '{nama_acuan}', digeser {geser_acuan * 1000:+.0f} ms "
              f"(selisihnya terhadap 'lama' dari uji sebelumnya)")
    acuan = sidik_unik(sesi[acuan_i][2].catat)
    print(f"\n=== SELISIH LATENSI terhadap 'lama' ({len(acuan)} frame unik di acuan "
          f"'{nama_acuan}') ===")
    print(f"  {'pengaturan':<22} {'fps':>5} {'frame':>6} {'selisih median':>15} "
          f"{'p10..p90':>16} {'melayang':>9}   perkiraan latensi")
    for nama, _, p in sesi:
        if not sehat(p):
            print(f"  {nama:<22}   (sesi tidak sehat, diabaikan)")
            continue
        sini = sidik_unik(p.catat)
        pasang = sorted((acuan[s], sini[s] - acuan[s] + geser_acuan)
                        for s in sini if s in acuan)
        durasi = (p.catat[-1][0] - p.catat[0][0]) if len(p.catat) > 1 else float("nan")
        fps = (len(p.catat) - 1) / durasi if durasi and durasi > 0 else float("nan")
        if len(pasang) < 20:
            print(f"  {nama:<22} {fps:5.1f} {len(pasang):>6}   frame yang cocok terlalu sedikit")
            continue
        d = np.array([x[1] for x in pasang])
        # Melayang: selisih di sepertiga akhir dikurangi sepertiga awal. Dekoder
        # yang tidak sanggup mengikuti 25 fps membuat antrean tumbuh, dan
        # latensinya naik terus - itu lebih buruk daripada latensi tetap.
        k = len(d) // 3
        layang = float(np.median(d[-k:]) - np.median(d[:k])) if k >= 5 else float("nan")
        print(f"  {nama:<22} {fps:5.1f} {len(d):>6} {np.median(d) * 1000:>12.0f} ms "
              f"{np.percentile(d, 10) * 1000:>6.0f}..{np.percentile(d, 90) * 1000:<6.0f}ms "
              f"{layang * 1000:>+6.0f} ms   ~{LATENSI_LAMA_S + np.median(d):.2f} s")

    print(f"\n  Selisih NEGATIF = frame keluar lebih DULU daripada pengaturan lama.")
    print(f"  'perkiraan latensi' = {LATENSI_LAMA_S:.2f} s (terukur di robot untuk "
          f"'lama') + selisih.")
    print(f"  Beberapa sesi sekaligus membebani kamera; galat dekode FFmpeg di stderr")
    print(f"  selama uji ini wajar - satu sesi sendirian terukur tanpa galat.")
    print(f"  Pastikan ulang dengan: python identifikasi.py --peta Kalibrasi --mode gas")


if __name__ == "__main__":
    main()
