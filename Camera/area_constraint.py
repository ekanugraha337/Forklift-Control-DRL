"""
area_constraint.py - zona constraint (larangan / batas kerja) untuk robot yang dilacak dengan AprilTag.

Dibangun di atas cctv_lurus.py (kalibrasi lensa + homografi + koreksi tinggi + deteksi AprilTag).
Jalankan dulu cctv_lurus.py kalibrasi & homografi sampai selesai.

Cara pakai:
    python area_constraint.py daftar                  -> lihat semua peta constraint
    python area_constraint.py setup                   -> pilih / buat peta, lalu gambar zona (cm)
    python area_constraint.py setup --peta gudang_A   -> langsung edit peta "gudang_A"
    python area_constraint.py setup --peta gudang_B --dari gudang_A
                                                      -> peta baru, zona awal disalin dari gudang_A
    python area_constraint.py pantau                  -> pilih peta, pantau robot
    python area_constraint.py pantau --peta gudang_A --robot-panjang 60 --robot-lebar 40
    python area_constraint.py pantau --simulasi       -> uji zona dengan "robot" yang mengikuti mouse

Setiap peta constraint disimpan sebagai file terpisah di folder peta_constraint/.
Ukuran robot disimpan terpisah (robot_constraint.json) karena sama untuk semua peta.
Tinggi tag & tinggi kamera diatur lewat cctv_lurus.py.

Jenis zona:
    larangan : robot TIDAK BOLEH masuk (menyentuh) area ini
    batas    : robot HARUS tetap di DALAM area ini (batas area kerja)

Yang dicek adalah SELURUH badan robot (persegi panjang panjang x lebar yang ikut berputar sesuai arah
robot), bukan hanya titik tengah tag.
"""
import argparse
import json
import os
import re
import socket
import threading
import time

import cv2
import numpy as np

import cctv_lurus as C

FOLDER_PETA = os.path.join(C.FOLDER, "peta_constraint")
FILE_ROBOT = os.path.join(C.FOLDER, "robot_constraint.json")

TIPE_LARANGAN, TIPE_BATAS = "larangan", "batas"
ROBOT_DEFAULT = {"panjang_cm": 40.0, "lebar_cm": 30.0, "geser_cm": 0.0, "margin_cm": 20.0, "prediksi_s": 0.5}

AMAN, PERINGATAN, PELANGGARAN, HILANG = "AMAN", "PERINGATAN", "PELANGGARAN", "HILANG"
WARNA_STATUS = {AMAN: (0, 200, 0), PERINGATAN: (0, 200, 255), PELANGGARAN: (0, 0, 255), HILANG: (160, 160, 160)}
WARNA_ZONA = {TIPE_LARANGAN: (0, 0, 255), TIPE_BATAS: (0, 220, 0)}
SNAPS = [0, 5, 15, 30]  # langkah snap koordinat saat menggambar zona (cm)
TAG_HILANG_S = 1.0      # tag tidak terlihat selama ini -> status HILANG


# ------------------------------------------------------------------ penyimpanan peta & robot
def nama_valid(nama):
    """Nama peta aman untuk nama file: huruf, angka, _ dan -."""
    return re.sub(r"[^\w\-]+", "_", nama.strip()).strip("_")


def path_peta(nama):
    return os.path.join(FOLDER_PETA, nama + ".json")


def daftar_peta():
    if not os.path.isdir(FOLDER_PETA):
        return []
    return sorted(f[:-5] for f in os.listdir(FOLDER_PETA) if f.lower().endswith(".json"))


def cari_peta(nama):
    """Nama peta yang sudah ada (tidak peka huruf besar/kecil), atau None."""
    target = nama_valid(nama).lower()
    return next((n for n in daftar_peta() if n.lower() == target), None)


def load_peta(nama):
    peta = {"nama": nama, "zona": []}
    if os.path.exists(path_peta(nama)):
        with open(path_peta(nama)) as fh:
            peta.update(json.load(fh))
    peta["nama"] = nama
    return peta


def save_peta(peta):
    os.makedirs(FOLDER_PETA, exist_ok=True)
    peta["diubah"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(path_peta(peta["nama"]), "w") as fh:
        json.dump(peta, fh, indent=2)
    print(f"[OK] Peta '{peta['nama']}' disimpan ke {path_peta(peta['nama'])}")


def ringkas_peta(nama):
    zona = load_peta(nama)["zona"]
    n_lar = sum(z["tipe"] == TIPE_LARANGAN for z in zona)
    return f"{len(zona)} zona ({n_lar} larangan, {len(zona) - n_lar} batas)"


def load_robot():
    data = {}
    if os.path.exists(FILE_ROBOT):
        with open(FILE_ROBOT) as fh:
            data = json.load(fh)
    return dict(ROBOT_DEFAULT, **data)


def save_robot(robot):
    with open(FILE_ROBOT, "w") as fh:
        json.dump(robot, fh, indent=2)


def pilih_peta(boleh_baru):
    """Menu pilih peta di terminal. Mengembalikan nama peta."""
    semua = daftar_peta()
    if not semua and not boleh_baru:
        raise SystemExit("[!] Belum ada peta constraint. Buat dulu: python area_constraint.py setup")
    if len(semua) == 1 and not boleh_baru:
        print(f"[i] Hanya ada satu peta: '{semua[0]}'")
        return semua[0]
    print("\n=== PILIH PETA CONSTRAINT ===")
    for i, nama in enumerate(semua, 1):
        print(f"  {i:2d}. {nama:25s} {ringkas_peta(nama)}")
    if not semua:
        print("  (belum ada peta)")
    while True:
        prompt = "Ketik nomor peta" + (" atau nama peta BARU" if boleh_baru else "") + ": "
        try:
            jawab = input(prompt).strip()
        except EOFError:
            raise SystemExit("[!] Tidak ada input. Gunakan --peta <nama>.")
        if jawab.isdigit() and 1 <= int(jawab) <= len(semua):
            return semua[int(jawab) - 1]
        if jawab and cari_peta(jawab):
            return cari_peta(jawab)
        if boleh_baru and nama_valid(jawab):
            nama = nama_valid(jawab)
            print(f"[i] Membuat peta baru: '{nama}'")
            return nama
        print("[!] Pilihan tidak valid.")


def mode_daftar(args):
    semua = daftar_peta()
    if not semua:
        print("Belum ada peta constraint. Buat dengan: python area_constraint.py setup")
        return
    print(f"Peta constraint di {FOLDER_PETA}:")
    for nama in semua:
        diubah = load_peta(nama).get("diubah", "-")
        print(f"  - {nama:25s} {ringkas_peta(nama):35s} diubah {diubah}")
    r = load_robot()
    print(f"Robot: {r['panjang_cm']:.0f} x {r['lebar_cm']:.0f} cm, geser {r['geser_cm']:.0f} cm, "
          f"margin {r['margin_cm']:.0f} cm, prediksi {r['prediksi_s']:.1f} s")


# ------------------------------------------------------------------ konteks kamera + homografi
class Konteks:
    """Semua data kalibrasi dari cctv_lurus.py + tampilan atas (cm <-> piksel)."""

    def __init__(self, offset_cm):
        self.p = C.load_params()
        if self.p is None:
            raise SystemExit("[!] Belum ada kalibrasi lensa. Jalankan: python cctv_lurus.py kalibrasi")
        hom = C.load_homografi(self.p)
        if hom is None:
            raise SystemExit("[!] Belum ada homografi. Jalankan: python cctv_lurus.py homografi")
        self.H = np.array(hom["H"])
        self.ubin = hom["ubin_cm"]
        self.cam = C.estimasi_kamera(self.H)
        if self.p.get("tinggi_kamera_cm"):
            self.cam[2] = self.p["tinggi_kamera_cm"]
        self.tinggi_tag = float(self.p.get("tinggi_tag_cm", 0.0))

        dst = np.array([q[2:] for q in hom["titik"]])
        self.hom_box = (*dst.min(axis=0), *dst.max(axis=0))
        off = max(float(offset_cm), 0.0)
        hx0, hy0, hx1, hy1 = self.hom_box
        self.ext = (hx0 - off, hy0 - off, hx1 + off, hy1 + off)
        x0, y0, x1, y1 = self.ext
        self.s = min(1000 / (x1 - x0), 800 / (y1 - y0))  # piksel per cm
        self.maps = None
        self.size = None

    def px(self, P):
        """cm -> piksel tampilan atas (array Nx2 int32)."""
        P = np.asarray(P, dtype=np.float64).reshape(-1, 2)
        return np.round((P - self.ext[:2]) * self.s).astype(np.int32)

    def cm(self, u, v):
        return np.array([self.ext[0] + u / self.s, self.ext[1] + v / self.s])

    def top_view(self, frame):
        h, w = frame.shape[:2]
        if self.size != (w, h):
            # bidang lantai (tinggi 0): zona digambar mengacu pada lantai
            self.maps = C.build_topdown_maps(self.H, self.p, w, h, self.ext, self.s, self.cam, 0.0)
            self.size = (w, h)
        return cv2.remap(frame, self.maps[0], self.maps[1], cv2.INTER_LINEAR)

    def grid_layer(self, shape):
        layer = np.zeros(shape, np.uint8)
        x0, y0, x1, y1 = self.ext
        u = self.ubin
        for i in range(int(np.ceil(x0 / u)), int(np.floor(x1 / u)) + 1):
            a, b = self.px([(i * u, y0), (i * u, y1)])
            cv2.line(layer, tuple(a), tuple(b), (200, 200, 0), 1)
        for j in range(int(np.ceil(y0 / u)), int(np.floor(y1 / u)) + 1):
            a, b = self.px([(x0, j * u), (x1, j * u)])
            cv2.line(layer, tuple(a), tuple(b), (200, 200, 0), 1)
        hx0, hy0, hx1, hy1 = self.hom_box
        a, b = self.px([(hx0, hy0), (hx1, hy1)])
        cv2.rectangle(layer, tuple(a), tuple(b), (0, 255, 255), 1)
        return layer


# ------------------------------------------------------------------ geometri
def jejak_robot(center, yaw_deg, robot):
    """4 sudut badan robot (cm). geser_cm = jarak pusat robot di DEPAN pusat tag (negatif = di belakang)."""
    a = np.radians(yaw_deg)
    f = np.array([np.cos(a), np.sin(a)])
    l = np.array([-np.sin(a), np.cos(a)])
    c = np.asarray(center) + f * robot["geser_cm"]
    L, W = robot["panjang_cm"] / 2, robot["lebar_cm"] / 2
    return np.array([c + f * L + l * W, c + f * L - l * W, c - f * L - l * W, c - f * L + l * W])


def _ppt(poly, pts):
    """Jarak bertanda titik ke poligon: + di dalam, - di luar, 0 di garis."""
    contour = np.asarray(poly, dtype=np.float32).reshape(-1, 1, 2)
    return np.array([cv2.pointPolygonTest(contour, (float(x), float(y)), True) for x, y in pts])


def _edges_cross(A, B):
    """True jika ada sisi poligon A yang memotong sisi poligon B."""
    A, B = np.asarray(A, float), np.asarray(B, float)
    a1, a2 = A[:, None, :], np.roll(A, -1, axis=0)[:, None, :]
    b1, b2 = B[None, :, :], np.roll(B, -1, axis=0)[None, :, :]
    r, s, qp = a2 - a1, b2 - b1, b1 - a1

    def cross(u, v):
        return u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]

    den = cross(r, s)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = cross(qp, s) / den
        u = cross(qp, r) / den
    ok = (np.abs(den) > 1e-12) & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1)
    return bool(ok.any())


def jarak_zona(fp, zona):
    """Jarak bebas (cm) badan robot terhadap zona. Negatif = melanggar."""
    Z = np.asarray(zona["titik"], dtype=np.float64)
    in_zone = _ppt(Z, fp)     # sudut robot terhadap zona
    in_robot = _ppt(fp, Z)    # sudut zona terhadap robot
    cross = _edges_cross(fp, Z)
    if zona["tipe"] == TIPE_LARANGAN:
        if cross or (in_zone >= 0).any() or (in_robot >= 0).any():
            return -1.0
        return float(min((-in_zone).min(), (-in_robot).min()))
    # batas: seluruh badan robot harus di dalam
    if cross or (in_zone <= 0).any() or (in_robot >= 0).any():
        return -1.0
    return float(min(in_zone.min(), (-in_robot).min()))


def evaluasi(fp, zones, margin):
    """(status, nama_zona_terdekat, jarak_cm)."""
    if not zones:
        return AMAN, None, float("inf")
    d, nama = min((jarak_zona(fp, z), z["nama"]) for z in zones)
    if d < 0:
        return PELANGGARAN, nama, 0.0
    return (PERINGATAN if d < margin else AMAN), nama, d


# ------------------------------------------------------------------ tampilan
def label(img, text, org, color, size=0.5):
    """Teks dengan latar kotak gelap (tanpa garis tepi hitam di huruf)."""
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, 1)
    x, y = int(org[0]), int(org[1])
    cv2.rectangle(img, (x - 3, y - th - 4), (x + tw + 3, y + base + 2), (30, 30, 30), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)


def zona_layer(ctx, zones, shape):
    """Lapisan warna zona + mask (dihitung sekali, ditempel setiap frame)."""
    color = np.zeros(shape, np.uint8)
    mask = np.zeros(shape[:2], np.uint8)
    for z in zones:
        pts = ctx.px(z["titik"])
        if z["tipe"] == TIPE_LARANGAN:
            cv2.fillPoly(color, [pts], WARNA_ZONA[TIPE_LARANGAN])
            cv2.fillPoly(mask, [pts], 255)
        else:  # arsir area DI LUAR batas
            outside = np.full(shape[:2], 255, np.uint8)
            cv2.fillPoly(outside, [pts], 0)
            color[outside > 0] = (0, 0, 180)
            mask[outside > 0] = 255
    return color, mask > 0


def tempel_zona(img, ctx, zones, layer):
    color, mask = layer
    if mask.any():
        img[mask] = cv2.addWeighted(img[mask], 0.7, color[mask], 0.3, 0)
    for z in zones:
        pts = ctx.px(z["titik"])
        cv2.polylines(img, [pts], True, WARNA_ZONA[z["tipe"]], 2, cv2.LINE_AA)
        # label di sudut pertama zona (bergeser ke dalam), agar tidak menutupi robot di tengah area
        c = pts.mean(axis=0)
        org = pts[0] + np.sign(c - pts[0]) * (8, 20)
        label(img, f"{z['nama']} ({z['tipe']})", (org[0], org[1]), WARNA_ZONA[z["tipe"]], 0.45)


# ------------------------------------------------------------------ mode setup
BANTUAN_SETUP = """
=== SETUP ZONA CONSTRAINT ===
  klik kiri     : tambah titik sudut zona
  klik kanan / Enter : tutup zona (min. 3 titik)
  t             : ganti jenis zona yang sedang digambar: LARANGAN <-> BATAS
  u             : hapus titik terakhir (jika kosong: hapus zona terakhir)
  d             : hapus zona di bawah kursor mouse
  x             : ganti snap koordinat: bebas / 5 / 15 / 30 cm
  spasi         : ambil frame baru dari kamera
  s             : simpan              q/Esc : keluar
Koordinat sama dengan cctv_lurus.py tag (cm, titik asal = titik 1 homografi).
"""


def mode_setup(args):
    ctx = Konteks(args.offset)
    if args.peta:
        nama = cari_peta(args.peta) or nama_valid(args.peta)
    else:
        nama = pilih_peta(boleh_baru=True)
    if not nama:
        raise SystemExit("[!] Nama peta tidak valid.")
    data = load_peta(nama)
    baru = cari_peta(nama) is None
    if args.dari:
        sumber = cari_peta(args.dari)
        if sumber is None:
            raise SystemExit(f"[!] Peta sumber '{args.dari}' tidak ditemukan.")
        if data["zona"]:
            raise SystemExit(f"[!] Peta '{nama}' sudah berisi zona; --dari hanya untuk peta kosong/baru.")
        data["zona"] = [dict(z) for z in load_peta(sumber)["zona"]]
        print(f"[i] {len(data['zona'])} zona disalin dari '{sumber}' (belum disimpan)")
    print(f"[i] {'Peta BARU' if baru else 'Edit peta'}: '{nama}' - {len(data['zona'])} zona")
    print(BANTUAN_SETUP)
    reader = C.StreamReader(args.url or C.rtsp_url())
    frame = reader.wait_first_frame()

    st = {"cur": [], "tipe": TIPE_LARANGAN, "mouse": (0, 0), "snap": 0, "dirty": bool(args.dari) or baru}
    win = f"Setup peta constraint - {nama}"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    def snap_cm(u, v):
        P = ctx.cm(u, v)
        step = SNAPS[st["snap"]]
        return np.round(P / step) * step if step else np.round(P, 1)

    def nama_baru():
        used = {z["nama"] for z in data["zona"]}
        k = 1
        while f"Z{k}" in used:
            k += 1
        return f"Z{k}"

    def tutup_zona():
        if len(st["cur"]) < 3:
            print("[!] Zona butuh minimal 3 titik.")
            return
        z = {"nama": nama_baru(), "tipe": st["tipe"], "titik": [[float(x), float(y)] for x, y in st["cur"]]}
        data["zona"].append(z)
        st["cur"] = []
        st["dirty"] = True
        print(f"[i] Zona {z['nama']} ({z['tipe']}) ditambahkan, {len(z['titik'])} titik")

    def on_mouse(event, x, y, flags, param):
        st["mouse"] = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            st["cur"].append(snap_cm(x, y))
        elif event == cv2.EVENT_RBUTTONDOWN:
            tutup_zona()

    cv2.setMouseCallback(win, on_mouse)
    try:
        while True:
            img = ctx.top_view(frame)
            grid = ctx.grid_layer(img.shape)
            gm = grid.any(axis=2)
            img[gm] = grid[gm]
            tempel_zona(img, ctx, data["zona"], zona_layer(ctx, data["zona"], img.shape))

            # zona yang sedang digambar
            mouse_cm = snap_cm(*st["mouse"])
            col = WARNA_ZONA[st["tipe"]]
            if st["cur"]:
                pts = ctx.px(st["cur"] + [mouse_cm])
                cv2.polylines(img, [pts], False, col, 2, cv2.LINE_AA)
                cv2.line(img, tuple(pts[-1]), tuple(pts[0]), col, 1, cv2.LINE_AA)
                for q in pts[:-1]:
                    cv2.circle(img, tuple(q), 4, col, -1)
            mp = ctx.px([mouse_cm])[0]
            cv2.drawMarker(img, tuple(mp), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
            label(img, f"X={mouse_cm[0]:.1f}  Y={mouse_cm[1]:.1f} cm", (mp[0] + 12, mp[1] - 10), (255, 255, 255), 0.45)

            snap_txt = f"{SNAPS[st['snap']]} cm" if SNAPS[st["snap"]] else "bebas"
            label(img, f"Peta: {nama} | Jenis: {st['tipe'].upper()} (t) | titik: {len(st['cur'])} | snap: {snap_txt} (x) | "
                       f"zona: {len(data['zona'])}" + (" | BELUM DISIMPAN (s)" if st["dirty"] else ""),
                  (10, 22), col, 0.5)
            label(img, "klik kiri: titik | klik kanan/Enter: tutup zona | u: undo | d: hapus zona | q: keluar",
                  (10, img.shape[0] - 10), (230, 230, 230), 0.45)
            cv2.imshow(win, img)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                if st["dirty"]:
                    print("[!] Ada perubahan yang BELUM disimpan (tekan s sebelum keluar). Tekan q lagi untuk keluar.")
                    st["dirty"] = False
                    continue
                break
            elif key in (13, 10):
                tutup_zona()
            elif key == ord("t"):
                st["tipe"] = TIPE_BATAS if st["tipe"] == TIPE_LARANGAN else TIPE_LARANGAN
            elif key == ord("u"):
                if st["cur"]:
                    st["cur"].pop()
                elif data["zona"]:
                    z = data["zona"].pop()
                    st["dirty"] = True
                    print(f"[i] Zona {z['nama']} dihapus")
            elif key == ord("d"):
                P = ctx.cm(*st["mouse"])
                for z in reversed(data["zona"]):
                    if _ppt(z["titik"], [P])[0] >= 0:
                        data["zona"].remove(z)
                        st["dirty"] = True
                        print(f"[i] Zona {z['nama']} dihapus")
                        break
            elif key == ord("x"):
                st["snap"] = (st["snap"] + 1) % len(SNAPS)
            elif key == ord(" "):
                new = reader.read()
                if new is not None:
                    frame = new
            elif key == ord("s"):
                save_peta(data)
                st["dirty"] = False
    finally:
        reader.stop()
        cv2.destroyAllWindows()


# ------------------------------------------------------------------ deteksi & pelacakan
class Detektor:
    def __init__(self, ctx, family):
        self.ctx = ctx
        prm = cv2.aruco.DetectorParameters()
        prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(C.TAG_FAMILIES[family]), prm)
        self.roi = None

    def deteksi(self, frame):
        ctx = self.ctx
        h, w = frame.shape[:2]
        if self.roi is None:
            x0, y0, x1, y1 = ctx.ext
            m = ctx.ubin
            self.roi = C.detection_roi(ctx.H, ctx.p, w, h, (x0 - m, y0 - m, x1 + m, y1 + m), ctx.cam, ctx.tinggi_tag)
        rx0, ry0, rx1, ry1 = self.roi
        corners, ids, _ = self.det.detectMarkers(cv2.cvtColor(frame[ry0:ry1, rx0:rx1], cv2.COLOR_BGR2GRAY))
        out = []
        if ids is None:
            return out
        for c, tag_id in zip(corners, ids.ravel()):
            n = C.undistort_points(c.reshape(4, 2) + (rx0, ry0), ctx.p, w, h)
            if n is None:
                continue
            F = C.floor_points(ctx.H, n, ctx.cam, ctx.tinggi_tag)
            center = F.mean(axis=0)
            front = (F[0] + F[1]) / 2
            yaw = float(np.degrees(np.arctan2(front[1] - center[1], front[0] - center[0])))
            out.append((int(tag_id), center, yaw))
        return out


class Pelacak:
    """Menyimpan posisi terakhir, kecepatan (untuk prediksi), dan status tiap robot."""

    def __init__(self):
        self.tracks = {}

    def update(self, detections, now):
        for tag_id, center, yaw in detections:
            tr = self.tracks.get(tag_id)
            if tr is None:
                self.tracks[tag_id] = {"center": center, "yaw": yaw, "t": now, "v": np.zeros(2), "status": None}
                continue
            dt = now - tr["t"]
            if 0 < dt < 0.5:
                v_new = (center - tr["center"]) / dt
                tr["v"] = 0.6 * tr["v"] + 0.4 * v_new
            else:
                tr["v"] = np.zeros(2)
            tr.update(center=center, yaw=yaw, t=now)


class Alarm:
    def __init__(self, enabled):
        self.enabled = enabled
        self.last = 0.0
        try:
            import winsound
            self.winsound = winsound
        except ImportError:
            self.winsound = None

    def bunyi(self):
        now = time.time()
        if not self.enabled or now - self.last < 1.0:
            return
        self.last = now
        if self.winsound:
            threading.Thread(target=self.winsound.Beep, args=(1800, 250), daemon=True).start()
        else:
            print("\a", end="", flush=True)


# ------------------------------------------------------------------ mode pantau
BANTUAN_PANTAU = """
=== PANTAU ZONA CONSTRAINT ===
  [ / ] : peta constraint sebelumnya / berikutnya
  r : muat ulang peta dari file (setelah diedit di setup)
  g : tampilkan/sembunyikan grid       z : tampilkan/sembunyikan zona
  m : alarm suara on/off               s : screenshot
  q/Esc : keluar
Mode --simulasi: robot mengikuti mouse, a/d = putar robot.
Status: AMAN (hijau) | PERINGATAN (kuning: < margin dari zona, atau diprediksi akan melanggar)
        PELANGGARAN (merah) | HILANG (abu-abu: tag tidak terlihat > 1 detik)
"""


def mode_pantau(args):
    ctx = Konteks(args.offset)
    robot = load_robot()
    ubah = {k: v for k, v in {"panjang_cm": args.robot_panjang, "lebar_cm": args.robot_lebar,
                              "geser_cm": args.geser, "margin_cm": args.margin,
                              "prediksi_s": args.prediksi}.items() if v is not None}
    if ubah:
        robot.update(ubah)
        save_robot(robot)

    if args.peta:
        nama_peta = cari_peta(args.peta)
        if nama_peta is None:
            raise SystemExit(f"[!] Peta '{args.peta}' tidak ditemukan. Peta yang ada: {', '.join(daftar_peta()) or '-'}")
    else:
        nama_peta = pilih_peta(boleh_baru=False)
    zones = load_peta(nama_peta)["zona"]

    print(BANTUAN_PANTAU)
    print(f"[i] Peta '{nama_peta}': {ringkas_peta(nama_peta)}")
    print(f"[i] Robot {robot['panjang_cm']:.0f} x {robot['lebar_cm']:.0f} cm, geser {robot['geser_cm']:.0f} cm, "
          f"margin {robot['margin_cm']:.0f} cm, prediksi {robot['prediksi_s']:.1f} s "
          f"| tinggi tag {ctx.tinggi_tag:.0f} cm (diatur di cctv_lurus.py)")
    if not zones:
        print(f"[!] Peta '{nama_peta}' belum berisi zona.")

    udp = None
    if args.udp:
        host, port = args.udp.rsplit(":", 1)
        udp = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (host, int(port)))
        udp[0].setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        print(f"[i] Status dikirim via UDP ke {host}:{port}")

    log = None
    if args.log:
        baru = not os.path.exists(args.log)
        log = open(args.log, "a")
        if baru:
            log.write("waktu,peta,id,status,zona,jarak_cm,x_cm,y_cm,sudut_deg\n")

    reader = C.StreamReader(args.url or C.rtsp_url())
    reader.wait_first_frame()
    detektor = Detektor(ctx, args.family)
    pelacak = Pelacak()
    alarm = Alarm(not args.tanpa_suara)
    show_grid, show_zona = True, True
    layers = None
    sim = {"pos": None, "yaw": 0.0}
    last_id, pending_key = 0, 255

    win = "Pantau zona constraint"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    if args.simulasi:
        def on_mouse(event, x, y, flags, param):
            sim["pos"] = ctx.cm(x, y)
        cv2.setMouseCallback(win, on_mouse)
        print("[i] MODE SIMULASI: gerakkan mouse di jendela, a/d untuk memutar robot")

    try:
        while True:
            frame, new_id = reader.read_new(last_id)
            if frame is None:
                key = cv2.waitKey(3) & 0xFF
                if key != 255:
                    pending_key = key
                continue
            last_id = new_id
            now = time.time()

            img = ctx.top_view(frame)
            if layers is None:
                grid = ctx.grid_layer(img.shape)
                layers = (grid, grid.any(axis=2), zona_layer(ctx, zones, img.shape))
            if show_grid:
                img[layers[1]] = layers[0][layers[1]]
            if show_zona:
                tempel_zona(img, ctx, zones, layers[2])

            if args.simulasi:
                dets = [(0, sim["pos"], sim["yaw"])] if sim["pos"] is not None else []
            else:
                dets = detektor.deteksi(frame)
            pelacak.update(dets, now)

            laporan = []
            for tag_id, tr in sorted(pelacak.tracks.items()):
                fp = jejak_robot(tr["center"], tr["yaw"], robot)
                if now - tr["t"] > TAG_HILANG_S:
                    status, zona, jarak, prediksi = HILANG, None, float("nan"), False
                else:
                    status, zona, jarak = evaluasi(fp, zones, robot["margin_cm"])
                    prediksi = False
                    if status != PELANGGARAN and robot["prediksi_s"] > 0 and np.linalg.norm(tr["v"]) > 2:
                        fp_pred = jejak_robot(tr["center"] + tr["v"] * robot["prediksi_s"], tr["yaw"], robot)
                        st_pred, zona_pred, _ = evaluasi(fp_pred, zones, 0.0)
                        if st_pred == PELANGGARAN:
                            status, zona, prediksi = PERINGATAN, zona_pred, True
                        pts = ctx.px(fp_pred)
                        cv2.polylines(img, [pts], True, WARNA_STATUS[status], 1, cv2.LINE_AA)

                # gambar badan robot
                col = WARNA_STATUS[status]
                pts = ctx.px(fp)
                over = img.copy()
                cv2.fillPoly(over, [pts], col)
                cv2.addWeighted(over, 0.35, img, 0.65, 0, img)
                cv2.polylines(img, [pts], True, col, 2, cv2.LINE_AA)
                c = ctx.px([tr["center"]])[0]
                a = np.radians(tr["yaw"])
                tip = ctx.px([tr["center"] + 0.6 * robot["panjang_cm"] * np.array([np.cos(a), np.sin(a)])])[0]
                cv2.arrowedLine(img, tuple(c), tuple(tip), col, 2, cv2.LINE_AA, tipLength=0.25)
                teks = f"ID {tag_id} {status}"
                if status in (AMAN, PERINGATAN) and zona and np.isfinite(jarak):
                    teks += f" {jarak:.0f} cm ke {zona}"
                elif status == PELANGGARAN:
                    teks += f" di {zona}"
                if prediksi:
                    teks += " (prediksi)"
                label(img, teks, (c[0] + 12, c[1] - 12), col, 0.5)

                # event: perubahan status
                if status != tr["status"]:
                    print(f"[{time.strftime('%H:%M:%S')}] ID {tag_id}: {tr['status']} -> {status}"
                          + (f" ({zona})" if zona else "")
                          + f" | X={tr['center'][0]:.1f} Y={tr['center'][1]:.1f} cm")
                    if log:
                        log.write(f"{now:.3f},{nama_peta},{tag_id},{status},{zona or ''},"
                                  f"{'' if not np.isfinite(jarak) else f'{jarak:.1f}'},"
                                  f"{tr['center'][0]:.2f},{tr['center'][1]:.2f},{tr['yaw']:.2f}\n")
                        log.flush()
                    tr["status"] = status
                if status == PELANGGARAN:
                    alarm.bunyi()

                laporan.append({"id": tag_id, "status": status, "zona": zona, "prediksi": prediksi,
                                "jarak_cm": None if not np.isfinite(jarak) else round(jarak, 1),
                                "x_cm": round(float(tr["center"][0]), 1), "y_cm": round(float(tr["center"][1]), 1),
                                "sudut_deg": round(tr["yaw"], 1)})

            # bingkai merah jika ada pelanggaran
            if any(r["status"] == PELANGGARAN for r in laporan):
                cv2.rectangle(img, (0, 0), (img.shape[1] - 1, img.shape[0] - 1), (0, 0, 255), 8)
            ringkas = " | ".join(f"ID {r['id']}: {r['status']}" for r in laporan) or "tidak ada robot"
            label(img, f"Peta: {nama_peta} ([ ])  |  " + ringkas + ("" if alarm.enabled else " | alarm OFF"),
                  (10, 22), (255, 255, 255), 0.5)
            cv2.imshow(win, img)

            if udp:
                pesan = json.dumps({"waktu": round(now, 3), "peta": nama_peta, "robot": laporan})
                try:
                    udp[0].sendto(pesan.encode(), udp[1])
                except OSError as e:
                    print(f"[!] UDP gagal: {e}")

            key = cv2.waitKey(1) & 0xFF
            if key == 255:
                key, pending_key = pending_key, 255
            if key in (ord("q"), 27):
                break
            elif key == ord("g"):
                show_grid = not show_grid
            elif key == ord("z"):
                show_zona = not show_zona
            elif key in (ord("["), ord("]"), ord("r")):
                semua = daftar_peta()
                if key != ord("r") and semua:
                    i = semua.index(nama_peta) if nama_peta in semua else 0
                    nama_peta = semua[(i + (1 if key == ord("]") else -1)) % len(semua)]
                zones = load_peta(nama_peta)["zona"]
                layers = None  # gambar ulang lapisan zona
                for tr in pelacak.tracks.values():
                    tr["status"] = None  # status dievaluasi ulang & tercatat di log
                print(f"[i] {'Muat ulang' if key == ord('r') else 'Ganti ke'} peta '{nama_peta}': "
                      f"{ringkas_peta(nama_peta)}")
            elif key == ord("m"):
                alarm.enabled = not alarm.enabled
            elif key == ord("a") and args.simulasi:
                sim["yaw"] -= 10
            elif key == ord("d") and args.simulasi:
                sim["yaw"] += 10
            elif key == ord("s"):
                name = os.path.join(C.FOLDER, time.strftime("zona_%Y%m%d_%H%M%S.jpg"))
                cv2.imwrite(name, img)
                print(f"[OK] Screenshot: {name}")
    finally:
        reader.stop()
        if log:
            log.close()
        cv2.destroyAllWindows()


# ------------------------------------------------------------------ main
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Zona constraint robot berbasis cctv_lurus.py + AprilTag")
    ap.add_argument("mode", choices=["daftar", "setup", "pantau"])
    ap.add_argument("--peta", default=None, help="nama peta constraint (tanpa ini: pilih dari daftar)")
    ap.add_argument("--dari", default=None, help="setup: salin zona dari peta lain sebagai awal (untuk peta baru)")
    ap.add_argument("--url", default=None, help="override URL RTSP")
    ap.add_argument("--offset", type=float, default=30.0, help="perluasan tampilan dari area homografi (cm)")
    ap.add_argument("--family", default="36h11", choices=sorted(C.TAG_FAMILIES), help="keluarga AprilTag")
    ap.add_argument("--robot-panjang", type=float, default=None, help="panjang robot, arah maju (cm), disimpan")
    ap.add_argument("--robot-lebar", type=float, default=None, help="lebar robot (cm), disimpan")
    ap.add_argument("--geser", type=float, default=None,
                    help="jarak pusat robot di depan pusat tag (cm), negatif = di belakang, disimpan")
    ap.add_argument("--margin", type=float, default=None, help="jarak PERINGATAN sebelum menyentuh zona (cm), disimpan")
    ap.add_argument("--prediksi", type=float, default=None,
                    help="lihat ke depan (detik) untuk prediksi pelanggaran, 0 = mati, disimpan")
    ap.add_argument("--udp", default=None, help="kirim status JSON ke host:port (mis. 192.168.100.50:5005)")
    ap.add_argument("--log", default=None, help="simpan perubahan status ke file CSV")
    ap.add_argument("--simulasi", action="store_true", help="robot mengikuti mouse (uji zona tanpa robot)")
    ap.add_argument("--tanpa-suara", action="store_true", help="matikan alarm suara")
    args = ap.parse_args()

    if args.mode == "daftar":
        mode_daftar(args)
    elif args.mode == "setup":
        mode_setup(args)
    else:
        mode_pantau(args)
