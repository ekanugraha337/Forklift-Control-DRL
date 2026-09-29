"""
CCTV RTSP viewer + koreksi distorsi lensa (barrel / cembung).

Cara pakai:
    python cctv_lurus.py kalibrasi   -> cari parameter lensa memakai garis nat ubin 30x30 cm
    python cctv_lurus.py lihat       -> tampilkan video yang sudah diluruskan
    python cctv_lurus.py homografi   -> klik sudut ubin untuk memetakan piksel -> cm lantai
    python cctv_lurus.py tag --tinggi-tag 50 --tinggi-kamera 280
                                     -> deteksi AprilTag dan tampilkan posisi X,Y (cm) + sudut robot
    python cctv_lurus.py             -> otomatis: lihat (jika sudah ada kalibrasi), jika belum -> kalibrasi

Prinsip: garis nat ubin di dunia nyata LURUS. Di kalibrasi, klik beberapa titik di
sepanjang beberapa garis nat (yang terlihat melengkung). Program mencari parameter
distorsi yang membuat semua garis itu kembali lurus.
"""
import os

# RTSP lewat TCP lebih stabil (tidak ada frame rusak/abu-abu) dibanding UDP
os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp")

import argparse
import itertools
import json
import threading
import time
from urllib.parse import quote

import cv2
import numpy as np

# ------------------------------------------------------------------ konfigurasi
CAMERA_IP = "192.168.100.250"
# CAMERA_IP = "192.168.1.64"
CAMERA_PORT = 554
USERNAME = "onvifuser"
PASSWORD = "Realme7pro"
STREAM_PATH = "/Streaming/Channels/101?transportmode=unicast&profile=Profile_1"  # 101 = main stream 1280x720

FOLDER = os.path.dirname(os.path.abspath(__file__))
FILE_KALIBRASI = os.path.join(FOLDER, "kalibrasi_lensa.json")

MODEL_STANDAR, MODEL_FISHEYE = 0, 1
PARAM_DEFAULT = {"model": MODEL_FISHEYE, "f": 0.4, "k1": 0.0, "k2": 0.0, "zoom": 1.0}


def rtsp_url():
    user = quote(USERNAME, safe="")
    pwd = quote(PASSWORD, safe="")
    return f"rtsp://{user}:{pwd}@{CAMERA_IP}:{CAMERA_PORT}{STREAM_PATH}"


# ------------------------------------------------------------------ pembaca stream
def buka_rtsp(url):
    """Buka aliran RTSP dengan dekoder SATU thread.

    Bawaan OpenCV membuka dekoder FFmpeg dengan thread sebanyak inti CPU, dan
    dekode multithread per-frame MENAHAN keluaran sekitar (thread - 1) frame.
    Pada PC 16 inti dan kamera ~26 fps itu ~0,6 detik - frame yang diterima
    program sudah setua itu sebelum sempat diproses.

    Terukur 2026-09-27 (Forklift Robot Program/uji_kamera.py, sidik frame dari
    beberapa sesi sekaligus): 1 thread mengeluarkan frame yang sama 611 ms
    LEBIH DULU daripada bawaan (p10..p90 -613..-606 ms), tanpa melayang, dan
    tetap sanggup 26 fps pada aliran utama 1280x720. Latensi ujung-ke-ujung
    yang terukur di robot turun dari ~0,68 s ke perkiraan ~0,07 s.
    fflags;nobuffer dan flags;low_delay tidak menambah manfaat; 2 thread
    menambah satu frame (~50 ms).

    Kalau aliran diganti ke resolusi yang jauh lebih besar dan dekoder satu
    thread tidak lagi sanggup mengikuti fps kamera, frame akan MENUMPUK dan
    latensinya naik terus - ukur ulang dengan uji_kamera.py, yang melaporkan
    pelayangan itu.
    """
    n_thread = getattr(cv2, "CAP_PROP_N_THREADS", None)
    if n_thread is None:            # OpenCV lama: tidak bisa diatur
        return cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    return cv2.VideoCapture(url, cv2.CAP_FFMPEG, [n_thread, 1])


class StreamReader:
    """Membaca RTSP di thread terpisah agar selalu dapat frame terbaru (tanpa delay menumpuk)."""

    def __init__(self, url):
        self.url = url
        self.frame = None
        self.frame_id = 0
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        cap = None
        while self.running:
            if cap is None or not cap.isOpened():
                cap = buka_rtsp(self.url)
                if not cap.isOpened():
                    print("[!] Gagal terhubung ke kamera, mencoba lagi...")
                    time.sleep(2)
                    continue
            ok, frame = cap.read()
            if not ok:
                print("[!] Stream terputus, menyambung ulang...")
                cap.release()
                cap = None
                time.sleep(1)
                continue
            with self.lock:
                self.frame = frame
                self.frame_id += 1
        if cap is not None:
            cap.release()

    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def read_new(self, last_id):
        """Kembalikan (frame, id) hanya jika ada frame BARU sejak last_id; jika tidak (None, last_id).
        Frame tidak disalin: thread pembaca selalu membuat array baru, jadi aman dipakai langsung."""
        with self.lock:
            if self.frame is None or self.frame_id == last_id:
                return None, last_id
            return self.frame, self.frame_id

    def wait_first_frame(self, timeout=20):
        t0 = time.time()
        while time.time() - t0 < timeout:
            frame = self.read()
            if frame is not None:
                return frame
            time.sleep(0.1)
        raise RuntimeError("Tidak ada frame dari kamera. Cek IP, username/password, dan jaringan.")

    def stop(self):
        self.running = False


# ------------------------------------------------------------------ model lensa
def camera_matrix(p, w, h, zoom=1.0):
    f = p["f"] * w * zoom
    return np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1]], dtype=np.float64)


def dist_coeffs(p):
    if p["model"] == MODEL_FISHEYE:
        return np.array([p["k1"], p["k2"], 0.0, 0.0], dtype=np.float64).reshape(4, 1)
    return np.array([p["k1"], p["k2"], 0.0, 0.0, 0.0], dtype=np.float64)


def fold_radius(p):
    """Radius ternormalisasi (tan sudut) tempat model polinomial mulai 'terlipat' (tidak monoton lagi)."""
    t = np.linspace(1e-4, 20.0, 20000)
    if p["model"] == MODEL_FISHEYE:
        t = np.minimum(t, np.pi / 2 - 1e-3)  # theta
        rd = t * (1 + p["k1"] * t**2 + p["k2"] * t**4)
        radius = np.tan(t)
    else:
        rd = t * (1 + p["k1"] * t**2 + p["k2"] * t**4)
        radius = t
    bad = np.nonzero(np.diff(rd) <= 0)[0]
    return radius[bad[0]] if len(bad) else radius[-1]


def build_maps(p, w, h):
    K = camera_matrix(p, w, h)
    D = dist_coeffs(p)
    newK = camera_matrix(p, w, h, p["zoom"])
    if p["model"] == MODEL_FISHEYE:
        mx, my = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), newK, (w, h), cv2.CV_32FC1)
    else:
        mx, my = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_32FC1)
    # hitamkan area di luar batas valid model (mencegah gambar tepi terlipat/berulang)
    u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    r = np.hypot((u - newK[0, 2]) / newK[0, 0], (v - newK[1, 2]) / newK[1, 1])
    invalid = r >= fold_radius(p)
    mx[invalid] = -1
    my[invalid] = -1
    return cv2.convertMaps(mx, my, cv2.CV_16SC2)


def undistort_points(pts, p, w, h, to_pixel=False):
    """Titik piksel (terdistorsi) -> titik lurus. Mengembalikan None jika model tidak valid."""
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
    K = camera_matrix(p, w, h)
    D = dist_coeffs(p)
    crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-10)
    if p["model"] == MODEL_FISHEYE:
        und = cv2.fisheye.undistortPoints(pts, K, D, R=np.eye(3), P=np.eye(3), criteria=crit)
    else:
        und = cv2.undistortPoints(pts, K, D, R=np.eye(3), P=np.eye(3), criteria=crit)
    und = und.reshape(-1, 2)
    if not np.all(np.isfinite(und)):
        return None

    # validasi: proyeksikan kembali, harus jatuh di titik semula (menolak solusi "terlipat")
    obj = np.hstack([und, np.ones((len(und), 1))]).reshape(-1, 1, 3)
    zero = np.zeros((3, 1))
    if p["model"] == MODEL_FISHEYE:
        back, _ = cv2.fisheye.projectPoints(obj, zero, zero, K, D)
    else:
        back, _ = cv2.projectPoints(obj, zero, zero, K, D)
    if np.max(np.abs(back.reshape(-1, 2) - pts.reshape(-1, 2))) > 1.0:
        return None

    if to_pixel:
        newK = camera_matrix(p, w, h, p["zoom"])
        und = und * newK[0, 0] + newK[:2, 2]
    return und


def straightness_cost(lines, p, w, h):
    """Jumlah (lebar/panjang)^2 tiap garis setelah diluruskan. 0 = lurus sempurna."""
    cost = 0.0
    for line in lines:
        und = undistort_points(line, p, w, h)
        if und is None:
            return float("inf")
        c = und - und.mean(axis=0)
        s = np.linalg.svd(c, compute_uv=False)
        if s[0] < 1e-9:
            return float("inf")
        cost += (s[1] / s[0]) ** 2
    return cost


def optimize(lines, p, w, h):
    """Grid search kasar -> halus. Standar: cari k1,k2 (f tetap). Fisheye: cari f,k1 (k2=0).

    Banyak kombinasi parameter memberi hasil hampir sama lurus, jadi diberi penalti kecil
    agar dipilih koefisien k yang kecil (model sederhana, tepi gambar tidak aneh).
    """
    reg = 2e-4
    if p["model"] == MODEL_FISHEYE:
        names, lo, hi = ["f", "k1"], np.array([0.15, -1.0]), np.array([2.0, 1.0])
        base = dict(p, k2=0.0)
        penalty = lambda t: reg * t["k1"] ** 2
    else:
        names, lo, hi = ["k1", "k2"], np.array([-1.0, -1.0]), np.array([1.0, 1.0])
        base = dict(p)
        penalty = lambda t: reg * t["k2"] ** 2
    lo_b, hi_b = lo.copy(), hi.copy()
    n = 21
    best, best_cost = None, float("inf")
    for _ in range(7):
        axes = [np.linspace(lo[i], hi[i], n) for i in range(len(names))]
        for combo in itertools.product(*axes):
            trial = dict(base, **dict(zip(names, map(float, combo))))
            c = straightness_cost(lines, trial, w, h) + penalty(trial)
            if c < best_cost:
                best, best_cost = trial, c
        if best is None:
            break
        center = np.array([best[k] for k in names])
        half = (hi - lo) / (n - 1) * 2
        lo, hi = np.maximum(center - half, lo_b), np.minimum(center + half, hi_b)
    if best is None:
        return None, float("inf")
    return best, straightness_cost(lines, best, w, h)


# ------------------------------------------------------------------ simpan / muat
def save_params(p):
    # gabung dengan isi file lama agar data lain (mis. ROI kotak area) tidak hilang
    data = {}
    if os.path.exists(FILE_KALIBRASI):
        with open(FILE_KALIBRASI) as fh:
            data = json.load(fh)
    data.update(p)
    with open(FILE_KALIBRASI, "w") as fh:
        json.dump(data, fh, indent=2)
    print(f"[OK] Kalibrasi disimpan ke {FILE_KALIBRASI}")


def load_params():
    if not os.path.exists(FILE_KALIBRASI):
        return None
    with open(FILE_KALIBRASI) as fh:
        return dict(PARAM_DEFAULT, **json.load(fh))


# ------------------------------------------------------------------ mode kalibrasi
WIN_ASLI = "Asli - klik titik di garis nat ubin"
WIN_HASIL = "Hasil koreksi"
WIN_PANEL = "Parameter"
COLORS = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255), (255, 255, 0), (0, 128, 255)]

BANTUAN = """
=== MODE KALIBRASI ===
Klik kiri pada jendela 'Asli' di sepanjang SATU garis nat ubin (4-8 titik, dari ujung ke ujung).
  n      : mulai garis baru
  u      : hapus titik terakhir
  r      : hapus semua garis
  a      : AUTO - cari parameter yang membuat garis lurus (min. 3 garis, sebaiknya 5+)
  m      : ganti model lensa (standar / fisheye)
  spasi  : ambil frame baru dari kamera
  g      : tampilkan/sembunyikan grid bantu
  s      : simpan kalibrasi
  q/Esc  : keluar
Tips: pilih garis yang dekat tepi gambar (distorsinya paling besar), horizontal dan vertikal,
      termasuk garis pertemuan lantai-dinding.
"""


def _make_trackbars(p):
    cv2.namedWindow(WIN_PANEL, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_PANEL, 500, 220)
    cv2.createTrackbar("model 0=std 1=fish", WIN_PANEL, int(p["model"]), 1, lambda v: None)
    cv2.createTrackbar("f (% lebar)", WIN_PANEL, int(round(p["f"] * 100)), 300, lambda v: None)
    cv2.setTrackbarMin("f (% lebar)", WIN_PANEL, 10)
    cv2.createTrackbar("k1 (x1000+1000)", WIN_PANEL, int(round(p["k1"] * 1000)) + 1000, 2000, lambda v: None)
    cv2.createTrackbar("k2 (x1000+1000)", WIN_PANEL, int(round(p["k2"] * 1000)) + 1000, 2000, lambda v: None)
    cv2.createTrackbar("zoom (%)", WIN_PANEL, int(round(p["zoom"] * 100)), 200, lambda v: None)
    cv2.setTrackbarMin("zoom (%)", WIN_PANEL, 20)


def _read_trackbars():
    return {
        "model": cv2.getTrackbarPos("model 0=std 1=fish", WIN_PANEL),
        "f": max(cv2.getTrackbarPos("f (% lebar)", WIN_PANEL), 10) / 100.0,
        "k1": (cv2.getTrackbarPos("k1 (x1000+1000)", WIN_PANEL) - 1000) / 1000.0,
        "k2": (cv2.getTrackbarPos("k2 (x1000+1000)", WIN_PANEL) - 1000) / 1000.0,
        "zoom": max(cv2.getTrackbarPos("zoom (%)", WIN_PANEL), 20) / 100.0,
    }


def _set_trackbars(p):
    cv2.setTrackbarPos("model 0=std 1=fish", WIN_PANEL, int(p["model"]))
    cv2.setTrackbarPos("f (% lebar)", WIN_PANEL, int(np.clip(round(p["f"] * 100), 10, 300)))
    cv2.setTrackbarPos("k1 (x1000+1000)", WIN_PANEL, int(np.clip(round(p["k1"] * 1000) + 1000, 0, 2000)))
    cv2.setTrackbarPos("k2 (x1000+1000)", WIN_PANEL, int(np.clip(round(p["k2"] * 1000) + 1000, 0, 2000)))
    cv2.setTrackbarPos("zoom (%)", WIN_PANEL, int(np.clip(round(p["zoom"] * 100), 20, 200)))


def _draw_grid(img, step=40):
    h, w = img.shape[:2]
    for x in range(0, w, step):
        cv2.line(img, (x, 0), (x, h), (0, 200, 255), 1)
    for y in range(0, h, step):
        cv2.line(img, (0, y), (w, y), (0, 200, 255), 1)


def mode_kalibrasi(url):
    print(BANTUAN)
    reader = StreamReader(url)
    frame = reader.wait_first_frame()
    h, w = frame.shape[:2]
    scale = 2.0 if w <= 800 else 1.0  # perbesar tampilan stream kecil agar mudah diklik
    print(f"[i] Resolusi stream: {w}x{h}")

    p = load_params() or dict(PARAM_DEFAULT)
    lines = [[]]
    show_grid = True

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            lines[-1].append((x / scale, y / scale))

    cv2.namedWindow(WIN_ASLI, cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow(WIN_HASIL, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WIN_ASLI, on_mouse)
    _make_trackbars(p)

    maps, maps_key = None, None
    status = {"text": "Klik titik di sepanjang SATU garis nat, lalu tekan n untuk garis berikutnya", "until": time.time() + 8,
              "color": (0, 255, 255)}

    def info(text, color=(0, 255, 255), seconds=6):
        print(text)
        status.update(text=text, color=color, until=time.time() + seconds)

    def draw_text(img, text, org, color, size=0.6):
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, size, (0, 0, 0), 4)
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, size, color, 1 if size < 0.7 else 2)

    def is_bent(line):
        """True jika titik-titik dalam satu garis membentuk belokan (mis. sudut 90 derajat)."""
        if len(line) < 3:
            return False
        c = np.asarray(line) - np.mean(line, axis=0)
        s = np.linalg.svd(c, compute_uv=False)
        return s[1] / max(s[0], 1e-9) > 0.12

    try:
        while True:
            p = _read_trackbars()
            key_p = tuple(sorted(p.items()))
            if key_p != maps_key:
                maps, maps_key = build_maps(p, w, h), key_p

            valid_lines = [l for l in lines if len(l) >= 3]
            cost = straightness_cost(valid_lines, p, w, h) if valid_lines else None

            # --- jendela asli
            asli = cv2.resize(frame, None, fx=scale, fy=scale)
            for i, line in enumerate(lines):
                col = COLORS[i % len(COLORS)]
                pts = [(int(x * scale), int(y * scale)) for x, y in line]
                for a, b in zip(pts, pts[1:]):
                    cv2.line(asli, a, b, col, 1)
                for pt in pts:
                    cv2.circle(asli, pt, 4, col, -1)
            n_valid = len([l for l in lines if len(l) >= 3])
            draw_text(asli, f"garis ke-{len(lines)} | titik: {len(lines[-1])} | garis siap: {n_valid}/3 (min)",
                      (10, 25), (0, 255, 255), 0.7)
            if is_bent(lines[-1]):
                draw_text(asli, "PERINGATAN: garis ini berbelok! Satu garis = satu nat lurus. Tekan u untuk hapus titik.",
                          (10, 55), (0, 0, 255))
            elif len(lines[-1]) >= 4:
                draw_text(asli, "Garis cukup. Tekan n untuk mulai garis baru.", (10, 55), (0, 255, 0))
            if time.time() < status["until"]:
                draw_text(asli, status["text"], (10, asli.shape[0] - 15), status["color"])
            cv2.imshow(WIN_ASLI, asli)

            # --- jendela hasil
            hasil = cv2.remap(frame, maps[0], maps[1], cv2.INTER_LINEAR)
            hasil = cv2.resize(hasil, None, fx=scale, fy=scale)
            if show_grid:
                _draw_grid(hasil)
            for i, line in enumerate(valid_lines):
                und = undistort_points(line, p, w, h, to_pixel=True)
                if und is None:
                    continue
                col = COLORS[i % len(COLORS)]
                pts = (und * scale).astype(int)
                for a, b in zip(pts, pts[1:]):
                    cv2.line(hasil, tuple(a), tuple(b), col, 2)
            model_name = "fisheye" if p["model"] == MODEL_FISHEYE else "standar"
            txt = f"{model_name} f={p['f']:.2f} k1={p['k1']:.3f} k2={p['k2']:.3f} zoom={p['zoom']:.2f}"
            cv2.putText(hasil, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(hasil, txt, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            if cost is not None:
                ctxt = f"error kelurusan: {cost * 1e4:.2f}  (makin kecil makin lurus)"
                cv2.putText(hasil, ctxt, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                cv2.putText(hasil, ctxt, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.imshow(WIN_HASIL, hasil)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("n"):
                if len(lines[-1]) < 3:
                    info("Garis ini belum cukup titik (min. 3). Tambah titik dulu.", (0, 0, 255))
                else:
                    lines.append([])
                    info(f"Mulai garis ke-{len(lines)}", (0, 255, 0), 3)
            elif key == ord("u"):
                if lines[-1]:
                    lines[-1].pop()
                elif len(lines) > 1:
                    lines.pop()
            elif key == ord("r"):
                lines = [[]]
            elif key == ord("g"):
                show_grid = not show_grid
            elif key == ord("m"):
                p["model"] = 1 - p["model"]
                p.update(f=0.5 if p["model"] == MODEL_STANDAR else 0.4, k1=0.0, k2=0.0)
                _set_trackbars(p)
            elif key == ord(" "):
                new = reader.read()
                if new is not None:
                    frame = new
                    info("Frame baru diambil", (0, 255, 0), 3)
            elif key == ord("a"):
                valid_lines = [l for l in lines if len(l) >= 3]
                if len(valid_lines) < 3:
                    info(f"AUTO GAGAL: baru {len(valid_lines)} garis. Butuh min. 3 garis terpisah (tekan n antar garis).",
                         (0, 0, 255), 8)
                    continue
                bent = [i + 1 for i, l in enumerate(valid_lines) if is_bent(l)]
                if bent:
                    info(f"AUTO GAGAL: garis {bent} berbelok. Hapus (u) dan klik ulang tiap nat lurus terpisah.",
                         (0, 0, 255), 8)
                    continue
                draw_text(asli, "Mencari parameter terbaik... tunggu", (10, asli.shape[0] - 15), (0, 255, 255))
                cv2.imshow(WIN_ASLI, asli)
                cv2.waitKey(1)
                best, best_cost = optimize(valid_lines, p, w, h)
                if best is None:
                    info("AUTO GAGAL: tidak ada solusi valid. Coba model lain (tekan m) atau klik ulang titik.",
                         (0, 0, 255), 8)
                else:
                    before = straightness_cost(valid_lines, dict(p, k1=0.0, k2=0.0), w, h)
                    info(f"AUTO OK: error {before * 1e4:.1f} -> {best_cost * 1e4:.1f} | "
                         f"f={best['f']:.3f} k1={best['k1']:.3f} k2={best['k2']:.3f}. Tekan s untuk simpan.",
                         (0, 255, 0), 10)
                    _set_trackbars(best)
                    p = best
            elif key == ord("s"):
                save_params(p)
                info("Kalibrasi tersimpan ke kalibrasi_lensa.json", (0, 255, 0), 4)
    finally:
        reader.stop()
        cv2.destroyAllWindows()


# ------------------------------------------------------------------ mode lihat
TAMPIL_POTONG, TAMPIL_LURUS, TAMPIL_PENUH = 0, 1, 2
NAMA_TAMPILAN = ["potong (luar kotak dihitamkan)", "kotak diluruskan jadi persegi panjang", "penuh + garis kotak"]

BANTUAN_LIHAT = """
=== MODE LIHAT ===
  w      : ganti tampilan: potong / kotak diluruskan / penuh
  r      : gambar ulang kotak area (klik 4 sudut)
  b      : bandingkan dengan video asli
  s      : screenshot
  q/Esc  : keluar
"""


def order_corners(pts):
    """Urutkan 4 titik menjadi kiri-atas, kanan-atas, kanan-bawah, kiri-bawah."""
    pts = np.float32(pts)
    s, d = pts.sum(axis=1), np.diff(pts, axis=1).ravel()
    return np.float32([pts[np.argmin(s)], pts[np.argmin(d)], pts[np.argmax(s)], pts[np.argmax(d)]])


def crop_roi(img, roi_px):
    """Hitamkan semua di luar kotak, lalu potong sebatas kotak."""
    mask = np.zeros(img.shape[:2], np.uint8)
    cv2.fillPoly(mask, [roi_px.astype(np.int32)], 255)
    out = cv2.bitwise_and(img, img, mask=mask)
    x, y, bw, bh = cv2.boundingRect(roi_px.astype(np.int32))
    return out[y:y + bh, x:x + bw]


def warp_roi(img, roi_px):
    """Tarik isi kotak (boleh miring/trapesium) menjadi persegi panjang tegak."""
    tl, tr, br, bl = roi_px
    out_w = int(max(np.linalg.norm(tr - tl), np.linalg.norm(br - bl)))
    out_h = int(max(np.linalg.norm(bl - tl), np.linalg.norm(br - tr)))
    dst = np.float32([[0, 0], [out_w - 1, 0], [out_w - 1, out_h - 1], [0, out_h - 1]])
    M = cv2.getPerspectiveTransform(roi_px, dst)
    return cv2.warpPerspective(img, M, (out_w, out_h), flags=cv2.INTER_LINEAR)


def mode_lihat(url):
    p = load_params()
    if p is None:
        print("[!] Belum ada kalibrasi. Jalankan dulu: python cctv_lurus.py kalibrasi")
        return
    print(BANTUAN_LIHAT)

    reader = StreamReader(url)
    reader.wait_first_frame()
    maps, size = None, None
    compare = False
    tampilan = p.get("tampilan", TAMPIL_POTONG)
    roi = p.get("roi")  # 4 titik (TL, TR, BR, BL) relatif 0..1 terhadap frame terkoreksi
    picking = []  # titik saat memilih kotak baru (mode 'r')
    picking_on = False
    win = "CCTV - terkoreksi"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def on_mouse(event, x, y, flags, param):
        if picking_on and event == cv2.EVENT_LBUTTONDOWN and len(picking) < 4:
            picking.append((x, y))

    cv2.setMouseCallback(win, on_mouse)
    try:
        while True:
            frame = reader.read()
            if frame is None:
                time.sleep(0.05)
                continue
            h, w = frame.shape[:2]
            if size != (w, h):  # parameter disimpan relatif terhadap lebar -> aman jika resolusi berubah
                maps, size = build_maps(p, w, h), (w, h)
            hasil = cv2.remap(frame, maps[0], maps[1], cv2.INTER_LINEAR)

            if picking_on:
                show = hasil.copy()
                for pt in picking:
                    cv2.circle(show, pt, 3, (0, 0, 255), -1)
                if len(picking) > 1:
                    cv2.polylines(show, [np.int32(picking)], len(picking) == 4, (0, 0, 255), 1)
                cv2.putText(show, f"Klik 4 sudut kotak ({len(picking)}/4) | Esc: batal", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                if len(picking) == 4:
                    pts = order_corners(picking)
                    roi = [[x / w, y / h] for x, y in pts]
                    save_params({"roi": roi})
                    picking_on = False
                    print("[OK] Kotak area baru disimpan")
            else:
                roi_px = np.float32([[x * w, y * h] for x, y in roi]) if roi else None
                if roi_px is None or tampilan == TAMPIL_PENUH:
                    show = hasil.copy()
                    if roi_px is not None:
                        cv2.polylines(show, [roi_px.astype(np.int32)], True, (0, 0, 255), 1)
                elif tampilan == TAMPIL_POTONG:
                    show = crop_roi(hasil, roi_px)
                else:
                    show = warp_roi(hasil, roi_px)
                if compare:
                    show = np.hstack([frame, cv2.resize(show, (int(show.shape[1] * h / show.shape[0]), h))])
            cv2.imshow(win, show)

            key = cv2.waitKey(1) & 0xFF
            if key == 27 and picking_on:
                picking_on = False
            elif key in (ord("q"), 27):
                break
            elif key == ord("b"):
                compare = not compare
            elif key == ord("w"):
                tampilan = (tampilan + 1) % 3
                save_params({"tampilan": tampilan})
                print(f"[i] Tampilan: {NAMA_TAMPILAN[tampilan]}")
            elif key == ord("r"):
                picking.clear()
                picking_on = True
                print("[i] Klik 4 sudut kotak area di jendela video")
            elif key == ord("s"):
                name = os.path.join(FOLDER, time.strftime("capture_%Y%m%d_%H%M%S.jpg"))
                cv2.imwrite(name, show)
                print(f"[OK] Screenshot: {name}")
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        reader.stop()
        cv2.destroyAllWindows()


# ------------------------------------------------------------------ homografi (piksel -> cm lantai)
UBIN_CM = 30.0
LENSA_KEYS = ("model", "f", "k1", "k2")


def pixel_to_norm(pts, p, w, h):
    """Piksel gambar terkoreksi -> koordinat ternormalisasi (bebas dari zoom & resolusi)."""
    newK = camera_matrix(p, w, h, p["zoom"])
    return (np.asarray(pts, dtype=np.float64).reshape(-1, 2) - newK[:2, 2]) / newK[0, 0]


def norm_to_pixel(pts, p, w, h):
    newK = camera_matrix(p, w, h, p["zoom"])
    return np.asarray(pts, dtype=np.float64).reshape(-1, 2) * newK[0, 0] + newK[:2, 2]


def apply_h(H, pts):
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H).reshape(-1, 2)


def estimasi_kamera(H):
    """Perkirakan posisi kamera (Cx, Cy, tinggi) dalam cm dari homografi lantai.

    Homografi memetakan koordinat ternormalisasi -> cm. Kebalikannya = K'[r1 r2 t] dengan
    K' = diag(s, s, 1) (s = koreksi kecil fokus). s diperkirakan dari syarat r1 tegak lurus r2
    dan |r1| = |r2|; jika tidak stabil dipakai s = 1.
    """
    G = np.linalg.inv(H)
    g1, g2, g3 = G[:, 0], G[:, 1], G[:, 2]
    cands = []
    den = g1[2] * g2[2]
    if abs(den) > 1e-12:
        cands.append(-(g1[0] * g2[0] + g1[1] * g2[1]) / den)
    den = g2[2] ** 2 - g1[2] ** 2
    if abs(den) > 1e-12:
        cands.append((g1[0] ** 2 + g1[1] ** 2 - g2[0] ** 2 - g2[1] ** 2) / den)
    cands = [np.sqrt(c) for c in cands if c > 0 and 0.5 < np.sqrt(c) < 2.0]
    s = float(np.median(cands)) if cands else 1.0

    kinv = np.diag([1 / s, 1 / s, 1.0])
    a1, a2, a3 = kinv @ g1, kinv @ g2, kinv @ g3
    lam = 2.0 / (np.linalg.norm(a1) + np.linalg.norm(a2))
    r1, r2, t = lam * a1, lam * a2, lam * a3
    if t[2] < 0:  # lantai harus di depan kamera
        r1, r2, t = -r1, -r2, -t
    U, _, Vt = np.linalg.svd(np.column_stack([r1, r2, np.cross(r1, r2)]))
    R = U @ Vt
    C = -R.T @ t
    return np.array([C[0], C[1], abs(C[2])])


def load_homografi(p):
    hom = p.get("homografi")
    if not hom:
        return None
    if any(abs(hom["lensa"][k] - p[k]) > 1e-9 for k in LENSA_KEYS):
        print("[!] PERINGATAN: kalibrasi lensa berubah sejak homografi dibuat. Ulangi mode homografi.")
    return hom


def floor_points(H, norm_pts, cam=None, tinggi_tag=0.0):
    """Koordinat ternormalisasi -> cm lantai, dengan koreksi tinggi (paralaks) jika tinggi_tag > 0."""
    P = apply_h(H, norm_pts)
    if cam is not None and tinggi_tag:
        c_xy, hc = cam[:2], cam[2]
        P = c_xy + (P - c_xy) * (hc - tinggi_tag) / hc
    return P


def _text(img, text, org, color=(255, 255, 255), size=0.55, thick=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, size, (0, 0, 0), thick + 3)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, size, color, thick)


def _draw_floor_grid(img, H, p, w, h, i_range, j_range, ubin, color=(255, 255, 0)):
    Hinv = np.linalg.inv(H)
    for i in range(i_range[0], i_range[1] + 1):
        a = norm_to_pixel(apply_h(Hinv, [(i * ubin, j_range[0] * ubin), (i * ubin, j_range[1] * ubin)]), p, w, h)
        cv2.line(img, tuple(a[0].astype(int)), tuple(a[1].astype(int)), color, 1)
    for j in range(j_range[0], j_range[1] + 1):
        a = norm_to_pixel(apply_h(Hinv, [(i_range[0] * ubin, j * ubin), (i_range[1] * ubin, j * ubin)]), p, w, h)
        cv2.line(img, tuple(a[0].astype(int)), tuple(a[1].astype(int)), color, 1)


BANTUAN_HOMOGRAFI = """
=== MODE HOMOGRAFI (piksel -> cm lantai) ===
Langkah 1: klik 4 sudut nat ubin yang membentuk persegi panjang BESAR, berurutan:
   1 = titik asal (0,0)   2 = sudut searah sumbu X   3 = sudut diagonal   4 = sudut searah sumbu Y
Langkah 2: ketik jarak titik 1->2 dalam CM lalu Enter, kemudian jarak titik 1->4 dalam CM lalu Enter.
Langkah 3: klik sebanyak mungkin sudut ubin lain (20-40 titik tersebar). Setiap klik otomatis
           dicocokkan ke sudut ubin terdekat. Titik hijau = cocok (<1 cm), kuning < 2.5 cm, merah = cek ulang.
  u : hapus titik terakhir    r : ulang dari awal    spasi : frame baru
  s : simpan                  q/Esc : keluar
Gunakan lup (kaca pembesar) di pojok kanan atas agar klik tepat di tengah persilangan nat.
"""


def mode_homografi(url):
    p = load_params()
    if p is None:
        print("[!] Belum ada kalibrasi lensa. Jalankan dulu: python cctv_lurus.py kalibrasi")
        return
    print(BANTUAN_HOMOGRAFI)
    reader = StreamReader(url)
    frame = reader.wait_first_frame()
    h, w = frame.shape[:2]
    maps = build_maps(p, w, h)
    win = "Homografi - klik sudut ubin"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    st = {"stage": "klik4", "corners": [], "typed": "", "cmx": 0.0, "cmy": 0.0, "pairs": [], "H": None,
          "mouse": (0, 0), "ubin": UBIN_CM}

    hom = load_homografi(p)
    if hom and all(abs(hom["lensa"][k] - p[k]) < 1e-9 for k in LENSA_KEYS):
        st.update(stage="tambah", pairs=[tuple(t) for t in hom["titik"]], H=np.array(hom["H"]), ubin=hom["ubin_cm"])
        print(f"[i] Homografi lama dimuat ({len(st['pairs'])} titik). Tekan r untuk mulai ulang.")

    def recompute():
        if len(st["pairs"]) >= 4:
            src = np.array([q[:2] for q in st["pairs"]])
            dst = np.array([q[2:] for q in st["pairs"]])
            H, _ = cv2.findHomography(src, dst, 0)
            if H is not None:
                st["H"] = H

    def on_mouse(event, x, y, flags, param):
        st["mouse"] = (x, y)
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        n = pixel_to_norm([(x, y)], p, w, h)[0]
        if st["stage"] == "klik4" and len(st["corners"]) < 4:
            st["corners"].append(n)
            if len(st["corners"]) == 4:
                st["stage"] = "cmx"
        elif st["stage"] == "tambah":
            u = st["ubin"]
            X, Y = np.round(apply_h(st["H"], [n])[0] / u) * u
            st["pairs"] = [q for q in st["pairs"] if (q[2], q[3]) != (X, Y)]
            st["pairs"].append((float(n[0]), float(n[1]), float(X), float(Y)))
            recompute()

    cv2.setMouseCallback(win, on_mouse)
    try:
        while True:
            clean = cv2.remap(frame, maps[0], maps[1], cv2.INTER_LINEAR)
            img = clean.copy()
            H = st["H"]

            if H is not None:
                idx = np.array([q[2:] for q in st["pairs"]]) / st["ubin"]
                lo, hi = idx.min(axis=0).astype(int) - 1, idx.max(axis=0).astype(int) + 1
                _draw_floor_grid(img, H, p, w, h, (lo[0], hi[0]), (lo[1], hi[1]), st["ubin"])
                errs = []
                for q in st["pairs"]:
                    e = float(np.linalg.norm(apply_h(H, [q[:2]])[0] - q[2:]))
                    errs.append(e)
                    col = (0, 255, 0) if e < 1 else (0, 255, 255) if e < 2.5 else (0, 0, 255)
                    pt = tuple(norm_to_pixel([q[:2]], p, w, h)[0].astype(int))
                    cv2.circle(img, pt, 4, col, -1)
                    cv2.putText(img, f"{int(q[2] / st['ubin'])},{int(q[3] / st['ubin'])}", (pt[0] + 5, pt[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
                rms = float(np.sqrt(np.mean(np.square(errs))))
                cam = estimasi_kamera(H)
                _text(img, f"{len(st['pairs'])} titik | error rata2 (RMS): {rms:.2f} cm | maks: {max(errs):.2f} cm",
                      (10, 50), (0, 255, 0))
                _text(img, f"perkiraan kamera: X={cam[0]:.0f} Y={cam[1]:.0f} tinggi={cam[2]:.0f} cm",
                      (10, 75), (255, 255, 0))
            for i, n in enumerate(st["corners"] if st["stage"] != "tambah" else []):
                pt = tuple(norm_to_pixel([n], p, w, h)[0].astype(int))
                cv2.circle(img, pt, 5, (0, 0, 255), -1)
                _text(img, str(i + 1), (pt[0] + 6, pt[1] - 6), (0, 0, 255))

            msg = {
                "klik4": f"Klik sudut ke-{len(st['corners']) + 1} dari 4 (1=asal, 2=arah X, 3=diagonal, 4=arah Y)",
                "cmx": f"Ketik jarak titik 1->2 dalam cm lalu Enter: {st['typed']}_",
                "cmy": f"Ketik jarak titik 1->4 dalam cm lalu Enter: {st['typed']}_",
                "tambah": "Klik sudut ubin lain sebanyak mungkin | u: hapus | s: simpan | q: keluar",
            }[st["stage"]]
            _text(img, msg, (10, 25), (0, 255, 255), 0.6)

            # lup / kaca pembesar
            mx, my = st["mouse"]
            r, z = 25, 5
            pad = cv2.copyMakeBorder(clean, r, r, r, r, cv2.BORDER_CONSTANT)
            crop = pad[my:my + 2 * r, mx:mx + 2 * r]
            if crop.shape[:2] == (2 * r, 2 * r):
                lup = cv2.resize(crop, None, fx=z, fy=z, interpolation=cv2.INTER_CUBIC)
                c = r * z
                cv2.line(lup, (c, 0), (c, 2 * c), (0, 0, 255), 1)
                cv2.line(lup, (0, c), (2 * c, c), (0, 0, 255), 1)
                cv2.rectangle(lup, (0, 0), (2 * c - 1, 2 * c - 1), (255, 255, 255), 2)
                img[5:5 + 2 * c, w - 5 - 2 * c:w - 5] = lup

            cv2.imshow(win, img)
            key = cv2.waitKey(30) & 0xFF
            if st["stage"] in ("cmx", "cmy"):
                if ord("0") <= key <= ord("9"):
                    st["typed"] += chr(key)
                elif key == 8:
                    st["typed"] = st["typed"][:-1]
                elif key in (13, 10) and st["typed"] and int(st["typed"]) > 0:
                    st[st["stage"]] = float(st["typed"])
                    st["typed"] = ""
                    if st["stage"] == "cmx":
                        st["stage"] = "cmy"
                    else:
                        lx, ly = st["cmx"], st["cmy"]
                        dst = [(0, 0), (lx, 0), (lx, ly), (0, ly)]
                        st["pairs"] = [(float(a[0]), float(a[1]), float(b[0]), float(b[1]))
                                       for a, b in zip(st["corners"], dst)]
                        recompute()
                        st["corners"] = []
                        st["stage"] = "tambah"
                elif key == 27:
                    st.update(stage="klik4", corners=[], typed="")
                continue

            if key in (ord("q"), 27):
                break
            elif key == ord("u"):
                if st["stage"] == "klik4" and st["corners"]:
                    st["corners"].pop()
                elif st["stage"] == "tambah" and len(st["pairs"]) > 4:
                    st["pairs"].pop()
                    recompute()
            elif key == ord("r"):
                st.update(stage="klik4", corners=[], typed="", pairs=[], H=None)
            elif key == ord(" "):
                new = reader.read()
                if new is not None:
                    frame = new
            elif key == ord("s"):
                if st["H"] is None:
                    print("[!] Belum ada homografi untuk disimpan.")
                    continue
                save_params({"homografi": {
                    "H": st["H"].tolist(),
                    "titik": [list(q) for q in st["pairs"]],
                    "ubin_cm": st["ubin"],
                    "lensa": {k: p[k] for k in LENSA_KEYS},
                }})
                cam = estimasi_kamera(st["H"])
                print(f"[OK] Homografi disimpan ({len(st['pairs'])} titik). "
                      f"Perkiraan kamera: X={cam[0]:.0f} Y={cam[1]:.0f} tinggi={cam[2]:.0f} cm")
    finally:
        reader.stop()
        cv2.destroyAllWindows()


# ------------------------------------------------------------------ deteksi AprilTag
TAG_FAMILIES = {
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "16h5": cv2.aruco.DICT_APRILTAG_16h5,
}

BANTUAN_TAG = """
=== MODE TAG (posisi robot) ===
  g : tampilkan/sembunyikan grid ubin      p : tampilkan/sembunyikan peta
  o : tampilkan/sembunyikan teks di video (default: sembunyi, posisi tetap terlihat di peta)
  v : ganti tampilan video: ATAS (hanya area homografi, dalam cm) / kamera penuh
  s : screenshot                           q/Esc : keluar
Koordinat: X,Y dalam cm dari titik asal homografi; sudut 0 derajat = sumbu +X,
positif berputar dari +X ke +Y. Arah depan robot = sisi ATAS tag.
"""


def detection_roi(H, p, w, h, extent_cm, cam, tinggi_tag, margin_px=40):
    """Kotak piksel pada frame ASLI yang mencakup seluruh area lantai terkalibrasi (untuk tag setinggi
    tinggi_tag). Deteksi hanya di kotak ini -> lebih cepat & tidak tertipu benda di dinding."""
    x0, y0, x1, y1 = extent_cm
    xs, ys = np.linspace(x0, x1, 12), np.linspace(y0, y1, 12)
    border = [(x, y0) for x in xs] + [(x, y1) for x in xs] + [(x0, y) for y in ys] + [(x1, y) for y in ys]
    P = np.array(border, dtype=np.float64)
    if tinggi_tag and cam[2] > tinggi_tag:
        # tag yang lebih tinggi terlihat bergeser menjauhi titik di bawah kamera
        P = cam[:2] + (P - cam[:2]) * cam[2] / (cam[2] - tinggi_tag)
    n = apply_h(np.linalg.inv(H), P)
    obj = np.hstack([n, np.ones((len(n), 1))]).reshape(-1, 1, 3)
    K, D, zero = camera_matrix(p, w, h), dist_coeffs(p), np.zeros((3, 1))
    if p["model"] == MODEL_FISHEYE:
        raw, _ = cv2.fisheye.projectPoints(obj, zero, zero, K, D)
    else:
        raw, _ = cv2.projectPoints(obj, zero, zero, K, D)
    raw = raw.reshape(-1, 2)
    rx0, ry0 = np.maximum(raw.min(axis=0) - margin_px, 0).astype(int)
    rx1, ry1 = np.minimum(raw.max(axis=0) + margin_px, [w, h]).astype(int)
    return int(rx0), int(ry0), int(rx1), int(ry1)


def build_topdown_maps(H, p, w, h, extent_cm, px_per_cm, cam, plane_h=0.0):
    """Peta remap langsung dari frame ASLI ke tampilan atas (bird's-eye) dalam cm.

    Setiap piksel keluaran = satu titik (X, Y) pada bidang setinggi plane_h. Titik itu diproyeksikan
    ke lantai sepanjang sinar kamera (koreksi tinggi), lalu ke koordinat ternormalisasi (homografi),
    lalu ke piksel terdistorsi (model lensa). Hasil: satu remap saja per frame.
    """
    x0, y0, x1, y1 = extent_cm
    out_w, out_h = int(round((x1 - x0) * px_per_cm)), int(round((y1 - y0) * px_per_cm))
    u, v = np.meshgrid(np.arange(out_w), np.arange(out_h))
    P = np.stack([x0 + (u.ravel() + 0.5) / px_per_cm, y0 + (v.ravel() + 0.5) / px_per_cm], axis=1)
    if plane_h and cam[2] > plane_h:
        P = cam[:2] + (P - cam[:2]) * cam[2] / (cam[2] - plane_h)
    n = apply_h(np.linalg.inv(H), P)
    obj = np.hstack([n, np.ones((len(n), 1))]).reshape(-1, 1, 3)
    K, D, zero = camera_matrix(p, w, h), dist_coeffs(p), np.zeros((3, 1))
    if p["model"] == MODEL_FISHEYE:
        raw, _ = cv2.fisheye.projectPoints(obj, zero, zero, K, D)
    else:
        raw, _ = cv2.projectPoints(obj, zero, zero, K, D)
    raw = raw.reshape(out_h, out_w, 2).astype(np.float32)
    return cv2.convertMaps(raw[..., 0], raw[..., 1], cv2.CV_16SC2)


def mode_tag(url, args):
    p = load_params()
    hom = load_homografi(p) if p else None
    if hom is None:
        print("[!] Belum ada homografi. Jalankan dulu: python cctv_lurus.py homografi")
        return

    changes = {}
    if args.tinggi_tag is not None:
        changes["tinggi_tag_cm"] = args.tinggi_tag
    if args.tinggi_kamera is not None:
        changes["tinggi_kamera_cm"] = args.tinggi_kamera
    if changes:
        save_params(changes)
        p.update(changes)

    H = np.array(hom["H"])
    ubin = hom["ubin_cm"]
    cam = estimasi_kamera(H)
    if p.get("tinggi_kamera_cm"):
        cam[2] = p["tinggi_kamera_cm"]
    tinggi_tag = float(p.get("tinggi_tag_cm", 0.0))
    print(BANTUAN_TAG)
    print(f"[i] Kamera (X,Y perkiraan; tinggi {'diukur' if p.get('tinggi_kamera_cm') else 'perkiraan'}): "
          f"X={cam[0]:.0f} Y={cam[1]:.0f} tinggi={cam[2]:.0f} cm | tinggi tag = {tinggi_tag:.1f} cm")
    if tinggi_tag and not p.get("tinggi_kamera_cm"):
        print("[!] Disarankan ukur tinggi kamera dari lantai dan jalankan dengan --tinggi-kamera <cm>")

    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(TAG_FAMILIES[args.family]), params)

    # area peta & deteksi = seluruh titik homografi + max(1 ubin, offset tampilan)
    dst = np.array([q[2:] for q in hom["titik"]])
    tepi = max(ubin, float(args.offset))
    x0, y0 = dst.min(axis=0) - tepi
    x1, y1 = dst.max(axis=0) + tepi
    skala = min(700 / (x1 - x0), 700 / (y1 - y0))  # piksel per cm di peta
    idx_lo = np.floor(np.array([x0, y0]) / ubin).astype(int)
    idx_hi = np.ceil(np.array([x1, y1]) / ubin).astype(int)

    def to_map(P):
        return (int((P[0] - x0) * skala), int((P[1] - y0) * skala))

    # tampilan atas: area yang dicakup titik-titik homografi + offset di setiap sisi
    hx0, hy0 = dst.min(axis=0)
    hx1, hy1 = dst.max(axis=0)
    off = max(float(args.offset), 0.0)
    tx0, ty0, tx1, ty1 = hx0 - off, hy0 - off, hx1 + off, hy1 + off
    s_top = min(1000 / (tx1 - tx0), 800 / (ty1 - ty0))  # piksel per cm

    def to_top(P):
        return (int(round((P[0] - tx0) * s_top)), int(round((P[1] - ty0) * s_top)))

    # latar peta (grid statis) digambar sekali saja
    peta_bg = np.full((int((y1 - y0) * skala) + 1, int((x1 - x0) * skala) + 1, 3), 40, np.uint8)
    for i in range(idx_lo[0], idx_hi[0] + 1):
        cv2.line(peta_bg, to_map((i * ubin, y0)), to_map((i * ubin, y1)), (90, 90, 90), 1)
    for j in range(idx_lo[1], idx_hi[1] + 1):
        cv2.line(peta_bg, to_map((x0, j * ubin)), to_map((x1, j * ubin)), (90, 90, 90), 1)
    cv2.drawMarker(peta_bg, to_map((0, 0)), (0, 255, 255), cv2.MARKER_CROSS, 14, 2)
    cv2.putText(peta_bg, "(0,0)", to_map((0, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    cv2.drawMarker(peta_bg, to_map(cam[:2]), (255, 128, 0), cv2.MARKER_TILTED_CROSS, 14, 2)
    cv2.putText(peta_bg, f"1 kotak = {ubin:.0f} cm | biru = kamera", (5, peta_bg.shape[0] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

    log = None
    if args.log:
        new_file = not os.path.exists(args.log)
        log = open(args.log, "a")
        if new_file:
            log.write("waktu,id,x_cm,y_cm,sudut_deg\n")

    reader = StreamReader(url)
    reader.wait_first_frame()
    maps, size, roi = None, None, None
    grid_layer = None
    show_grid, show_map, show_osd = True, True, False
    view_top = True
    trails = {}
    last_id, pending_key = 0, 255
    fps, t_proc, t_prev = 0.0, 0.0, time.perf_counter()
    win = "Tag - posisi robot"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while True:
            frame, new_id = reader.read_new(last_id)
            if frame is None:
                # belum ada frame baru dari kamera: jangan proses ulang frame yang sama
                key = cv2.waitKey(3) & 0xFF
                if key != 255:
                    pending_key = key  # simpan tombol, diproses setelah frame berikutnya
                continue
            dropped = new_id - last_id - 1 if last_id else 0
            last_id = new_id
            t_start = time.perf_counter()

            h, w = frame.shape[:2]
            if size != (w, h):
                maps, size = build_maps(p, w, h), (w, h)
                roi = detection_roi(H, p, w, h, (x0, y0, x1, y1), cam, tinggi_tag)
                # grid ubin digambar sekali ke lapisan terpisah, lalu ditempel tiap frame
                grid_layer = np.zeros((h, w, 3), np.uint8)
                _draw_floor_grid(grid_layer, H, p, w, h, idx_lo, idx_hi, ubin, (200, 200, 0))
                grid_mask = grid_layer.any(axis=2)
                # tampilan atas dibuat pada bidang setinggi tag -> tag tampil tepat di posisi X,Y-nya
                top_maps = build_topdown_maps(H, p, w, h, (tx0, ty0, tx1, ty1), s_top, cam, tinggi_tag)
                top_grid = np.zeros(top_maps[0].shape[:2] + (3,), np.uint8)
                for i in range(int(np.ceil(tx0 / ubin)), int(np.floor(tx1 / ubin)) + 1):
                    cv2.line(top_grid, to_top((i * ubin, ty0)), to_top((i * ubin, ty1)), (200, 200, 0), 1)
                for j in range(int(np.ceil(ty0 / ubin)), int(np.floor(ty1 / ubin)) + 1):
                    cv2.line(top_grid, to_top((tx0, j * ubin)), to_top((tx1, j * ubin)), (200, 200, 0), 1)
                if off:  # batas area homografi (di luar garis kuning = area offset)
                    cv2.rectangle(top_grid, to_top((hx0, hy0)), to_top((hx1, hy1)), (0, 255, 255), 2)
                top_mask = top_grid.any(axis=2)
                print(f"[i] Area deteksi tag: x {roi[0]}-{roi[2]}, y {roi[1]}-{roi[3]} "
                      f"({100 * (roi[2] - roi[0]) * (roi[3] - roi[1]) / (w * h):.0f}% frame)")
                print(f"[i] Tampilan atas (offset {off:.0f} cm): X {tx0:.0f}-{tx1:.0f} cm, Y {ty0:.0f}-{ty1:.0f} cm "
                      f"({top_grid.shape[1]}x{top_grid.shape[0]} px, {s_top:.2f} px/cm)")
            if view_top:
                img = cv2.remap(frame, top_maps[0], top_maps[1], cv2.INTER_LINEAR)
                if show_grid:
                    img[top_mask] = top_grid[top_mask]
            else:
                img = cv2.remap(frame, maps[0], maps[1], cv2.INTER_LINEAR)
                if show_grid:
                    img[grid_mask] = grid_layer[grid_mask]

            # deteksi hanya di area lantai pada frame ASLI (tanpa interpolasi),
            # lalu sudut tag diluruskan secara matematis
            rx0, ry0, rx1, ry1 = roi
            gray = cv2.cvtColor(frame[ry0:ry1, rx0:rx1], cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            robots = []
            if ids is not None:
                for c, tag_id in zip(corners, ids.ravel()):
                    n = undistort_points(c.reshape(4, 2) + (rx0, ry0), p, w, h)
                    if n is None:
                        continue
                    F = floor_points(H, n, cam, tinggi_tag)
                    center = F.mean(axis=0)
                    front = (F[0] + F[1]) / 2
                    yaw = float(np.degrees(np.arctan2(front[1] - center[1], front[0] - center[0])))
                    robots.append((int(tag_id), center, yaw))

                    if view_top:
                        px = np.int32([to_top(q) for q in F])
                    else:
                        px = norm_to_pixel(n, p, w, h).astype(np.int32)
                    cv2.polylines(img, [px], True, (0, 255, 0), 2)
                    cv2.line(img, tuple(px[0]), tuple(px[1]), (0, 0, 255), 2)  # sisi depan
                    org = (int(px[:, 0].min()), int(px[:, 1].min()) - 8)
                    if show_osd:
                        cv2.putText(img, f"ID {tag_id}: X={center[0]:.1f} Y={center[1]:.1f} cm  {yaw:.1f} deg", org,
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)
                    if log:
                        log.write(f"{time.time():.3f},{tag_id},{center[0]:.2f},{center[1]:.2f},{yaw:.2f}\n")
            if show_osd:
                cv2.putText(img, f"tag terdeteksi: {len(robots)} | tinggi tag {tinggi_tag:.0f} cm", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(win, img)

            now = time.perf_counter()
            fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6) if fps else 1.0 / max(now - t_prev, 1e-6)
            t_proc = 0.9 * t_proc + 0.1 * (now - t_start) * 1000 if t_proc else (now - t_start) * 1000
            t_prev = now

            if show_map:
                peta = peta_bg.copy()
                cv2.putText(peta, f"{fps:.1f} fps | proses {t_proc:.1f} ms" + (f" | frame terlewat {dropped}" if dropped else ""),
                            (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
                for tag_id, center, yaw in robots:
                    trail = trails.setdefault(tag_id, [])
                    trail.append(to_map(center))
                    del trail[:-300]
                for tag_id, trail in trails.items():
                    if len(trail) > 1:
                        cv2.polylines(peta, [np.int32(trail)], False, (0, 120, 0), 1)
                for tag_id, center, yaw in robots:
                    c = to_map(center)
                    tip = (int(c[0] + 25 * np.cos(np.radians(yaw))), int(c[1] + 25 * np.sin(np.radians(yaw))))
                    cv2.circle(peta, c, 8, (0, 255, 0), -1)
                    cv2.arrowedLine(peta, c, tip, (0, 0, 255), 2, tipLength=0.3)
                    cv2.putText(peta, f"{tag_id} ({center[0]:.0f},{center[1]:.0f})", (c[0] + 10, c[1] + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
                cv2.imshow("Peta lantai (cm)", peta)

            key = cv2.waitKey(1) & 0xFF
            if key == 255:
                key, pending_key = pending_key, 255
            if key in (ord("q"), 27):
                break
            elif key == ord("g"):
                show_grid = not show_grid
            elif key == ord("o"):
                show_osd = not show_osd
            elif key == ord("v"):
                view_top = not view_top
                cv2.destroyWindow(win)
                cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            elif key == ord("p"):
                show_map = not show_map
                if not show_map:
                    cv2.destroyWindow("Peta lantai (cm)")
            elif key == ord("s"):
                name = os.path.join(FOLDER, time.strftime("tag_%Y%m%d_%H%M%S.jpg"))
                cv2.imwrite(name, img)
                print(f"[OK] Screenshot: {name}")
    finally:
        reader.stop()
        if log:
            log.close()
        cv2.destroyAllWindows()


# ------------------------------------------------------------------ main
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="CCTV RTSP dengan koreksi distorsi lensa")
    ap.add_argument("mode", nargs="?", choices=["kalibrasi", "lihat", "homografi", "tag"],
                    help="kosongkan untuk otomatis")
    ap.add_argument("--url", default=None, help="override URL RTSP")
    ap.add_argument("--tinggi-tag", type=float, default=None, help="tinggi AprilTag dari lantai (cm), disimpan")
    ap.add_argument("--tinggi-kamera", type=float, default=None, help="tinggi kamera dari lantai (cm), disimpan")
    ap.add_argument("--family", default="36h11", choices=sorted(TAG_FAMILIES), help="keluarga AprilTag")
    ap.add_argument("--log", default=None, help="simpan posisi ke file CSV")
    ap.add_argument("--offset", type=float, default=30.0,
                    help="mode tag: perluas tampilan atas sejauh ini (cm) dari tiap sisi area homografi")
    args = ap.parse_args()

    url = args.url or rtsp_url()
    mode = args.mode or ("lihat" if os.path.exists(FILE_KALIBRASI) else "kalibrasi")
    if mode == "kalibrasi":
        mode_kalibrasi(url)
    elif mode == "homografi":
        mode_homografi(url)
    elif mode == "tag":
        mode_tag(url, args)
    else:
        mode_lihat(url)
