"""
arena.py - peta constraint (area_constraint.py) -> occupancy grid -> ObstacleMap.

Rintangan robot ini VIRTUAL: zona di peta constraint tidak punya wujud fisik,
lantainya kosong. Jadi tidak ada yang bisa dideteksi mata kamera, dan petalah
satu-satunya sumber kebenaran soal ke mana robot boleh pergi. Berbeda dengan
MQTT_1tag.py v1 yang mencari rintangan di dalam citra.

Pemetaan zona ke sel:
    "larangan" -> sel DI DALAM poligon terisi
    "batas"    -> sel DI LUAR poligon terisi (robot harus tetap di dalam)
    tepi arena -> otomatis dinding, diurus ObstacleMap

Satuan: 1 sel = konfig.GRID_CM cm. Dengan 1,0 cm per sel, jarak yang
dikembalikan cv2.distanceTransform langsung bersatuan cm.

Uji mandiri (tanpa kamera, tanpa robot):
    python arena.py                 -> pilih peta, tampilkan grid
    python arena.py --peta Rumah
"""
import argparse
import math

import cv2
import numpy as np

import konfig

konfig.pasang_path_kamera()
import area_constraint as AC          # noqa: E402
from forklift_planner import ObstacleMap  # noqa: E402


class Arena:
    """Peta constraint yang sudah siap dipakai perencana."""

    def __init__(self, nama_peta, cm_per_cell=None, offset_cm=None):
        self.cm_per_cell = float(cm_per_cell or konfig.GRID_CM)
        self.ctx = AC.Konteks(konfig.VIEW_OFFSET_CM if offset_cm is None else offset_cm)
        self.nama = nama_peta
        peta = AC.load_peta(nama_peta)
        self.zona = peta["zona"]
        # Peta simulasi (mis. Gudang3) membawa ukurannya sendiri: "ukuran_cm" =
        # [lebar, tinggi] area dalam. Tanpa kunci itu ukuran arena tetap dari
        # homografi kamera, seperti semua peta nyata.
        if peta.get("ukuran_cm"):
            lebar, tinggi = (float(v) for v in peta["ukuran_cm"])
            off = max(float(konfig.VIEW_OFFSET_CM if offset_cm is None else offset_cm), 0.0)
            self.ctx.hom_box = (0.0, 0.0, lebar, tinggi)
            self.ctx.ext = (-off, -off, lebar + off, tinggi + off)
            self.ctx.s = min(1000 / (lebar + 2 * off), 800 / (tinggi + 2 * off))
            self.ctx.maps = self.ctx.size = None

        x0, y0, x1, y1 = self.ctx.ext
        self.asal = (x0, y0)
        self.lebar_sel = max(2, int(math.ceil((x1 - x0) / self.cm_per_cell)))
        self.tinggi_sel = max(2, int(math.ceil((y1 - y0) / self.cm_per_cell)))

        self.occ = self._rasterkan()
        self.omap = ObstacleMap(self.occ)

    # ---------------------------------------------------------------- grid
    def _rasterkan(self):
        occ = np.zeros((self.tinggi_sel, self.lebar_sel), np.uint8)
        for z in self.zona:
            pts = self.sel_poligon(z["titik"])
            if z["tipe"] == AC.TIPE_LARANGAN:
                cv2.fillPoly(occ, [pts], 255)
            else:
                # Robot harus di dalam SETIAP zona batas, jadi daerah terlarang
                # adalah gabungan dari "di luar masing-masing" - bukan di luar
                # gabungannya. Dua zona batas berarti irisan keduanya.
                luar = np.full_like(occ, 255)
                cv2.fillPoly(luar, [pts], 0)
                occ |= luar
        return occ

    def sel_poligon(self, titik_cm):
        P = np.asarray(titik_cm, dtype=np.float64).reshape(-1, 2)
        return np.round((P - self.asal) / self.cm_per_cell).astype(np.int32)

    # ------------------------------------------------------- konversi satuan
    def ke_sel(self, x_cm, y_cm):
        return ((x_cm - self.asal[0]) / self.cm_per_cell,
                (y_cm - self.asal[1]) / self.cm_per_cell)

    def ke_cm(self, gx, gy):
        return (self.asal[0] + gx * self.cm_per_cell,
                self.asal[1] + gy * self.cm_per_cell)

    def panjang_ke_sel(self, cm):
        return cm / self.cm_per_cell

    def panjang_ke_cm(self, sel):
        return sel * self.cm_per_cell

    def pose_ke_sel(self, x_cm, y_cm, yaw_deg):
        """(x,y) cm + yaw derajat -> (gx, gy, yaw radian) untuk perencana.

        Sudut tidak ikut diskalakan: sel berbentuk bujur sangkar, jadi rotasi
        tidak berubah.
        """
        gx, gy = self.ke_sel(x_cm, y_cm)
        return (gx, gy, math.radians(yaw_deg))

    def pose_ke_cm(self, gx, gy, yaw_rad):
        x, y = self.ke_cm(gx, gy)
        return (x, y, math.degrees(yaw_rad))

    def jalur_ke_cm(self, jalur):
        """Jalur perencana (sel) -> [(x_cm, y_cm, yaw_deg), ...]."""
        return [self.pose_ke_cm(p[0], p[1], p[2]) for p in jalur]

    # ------------------------------------------------------------- bantuan
    def di_dalam(self, x_cm, y_cm):
        gx, gy = self.ke_sel(x_cm, y_cm)
        return 0 <= gx < self.lebar_sel and 0 <= gy < self.tinggi_sel

    def bebas_cm(self, x_cm, y_cm):
        """Jarak bebas (cm) dari titik ke rintangan terdekat. Negatif = di luar arena."""
        gx, gy = self.ke_sel(x_cm, y_cm)
        return self.panjang_ke_cm(self.omap.clearance(gx, gy))

    def ukuran_tampilan(self):
        x1, y1 = self.ctx.ext[2], self.ctx.ext[3]
        u, v = self.ctx.px([(x1, y1)])[0]
        return int(v) + 1, int(u) + 1     # (tinggi, lebar) untuk np.zeros

    def kanvas(self):
        """Kanvas tampilan atas kosong, ukuran sama dengan ctx.px()."""
        h, w = self.ukuran_tampilan()
        return np.zeros((h, w, 3), np.uint8)

    def gambar_zona(self, img, nama=True):
        AC.tempel_zona(img, self.ctx, self.zona,
                       AC.zona_layer(self.ctx, self.zona, img.shape), nama=nama)

    def gambar_grid(self, img):
        lapis = self.ctx.grid_layer(img.shape)
        m = lapis.any(axis=2)
        img[m] = lapis[m]

    def ringkas(self):
        n_lar = sum(z["tipe"] == AC.TIPE_LARANGAN for z in self.zona)
        terisi = int((self.occ > 0).sum())
        total = self.occ.size
        return (f"peta '{self.nama}': {len(self.zona)} zona "
                f"({n_lar} larangan, {len(self.zona) - n_lar} batas) | "
                f"grid {self.lebar_sel}x{self.tinggi_sel} sel @ {self.cm_per_cell} cm | "
                f"terisi {100.0 * terisi / total:.1f}%")


class PenunjukPose:
    """Klik-dan-tarik di tampilan atas -> pose (x_cm, y_cm, yaw_deg).

    Klik menentukan POSISI, arah tarikan menentukan ARAH HADAP. Tarikan yang
    lebih pendek dari drag_min_cm dianggap klik biasa: posisinya dipakai, arah
    hadapnya dibiarkan bebas (None) supaya perencana boleh memilih sendiri.
    """

    def __init__(self, ctx, drag_min_cm=None, tombol=cv2.EVENT_LBUTTONDOWN):
        self.ctx = ctx
        self.drag_min = konfig.GOAL_DRAG_MIN_CM if drag_min_cm is None else drag_min_cm
        self._turun = {cv2.EVENT_LBUTTONDOWN: cv2.EVENT_LBUTTONDOWN,
                       cv2.EVENT_RBUTTONDOWN: cv2.EVENT_RBUTTONDOWN}[tombol]
        self._naik = (cv2.EVENT_LBUTTONUP if tombol == cv2.EVENT_LBUTTONDOWN
                      else cv2.EVENT_RBUTTONUP)
        self.awal = None          # cm, selama menarik
        self.kini = None          # cm, posisi mouse sekarang
        self.pose = None          # hasil terakhir (x_cm, y_cm, yaw_deg|None)
        self.baru = False         # True satu kali setelah pose baru selesai

    def on_mouse(self, event, x, y, flags, param=None):
        P = self.ctx.cm(x, y)
        if event == cv2.EVENT_MOUSEMOVE:
            self.kini = P
        elif event == self._turun:
            self.awal, self.kini = P, P
        elif event == self._naik and self.awal is not None:
            d = P - self.awal
            jarak = float(np.hypot(*d))
            yaw = float(np.degrees(np.arctan2(d[1], d[0]))) if jarak >= self.drag_min else None
            self.pose = (float(self.awal[0]), float(self.awal[1]), yaw)
            self.awal = None
            self.baru = True

    def ambil(self):
        """Pose baru sekali saja; None bila belum ada yang baru."""
        if not self.baru:
            return None
        self.baru = False
        return self.pose

    def gambar(self, img, warna=(0, 255, 255)):
        """Panah sementara selagi menarik."""
        if self.awal is None or self.kini is None:
            return
        a = self.ctx.px([self.awal])[0]
        b = self.ctx.px([self.kini])[0]
        cv2.arrowedLine(img, tuple(a), tuple(b), warna, 2, cv2.LINE_AA, tipLength=0.3)
        jarak = float(np.hypot(*(self.kini - self.awal)))
        AC.label(img, f"{jarak:.0f} cm" + ("" if jarak >= self.drag_min else "  (arah bebas)"),
                 (b[0] + 10, b[1] - 8), warna, 0.45)


def jejak_badan(x_cm, y_cm, yaw_deg):
    """4 sudut badan robot (cm), titik acuan POROS DEPAN.

    Sengaja dihitung dari konfig.py, bukan dari robot_constraint.json seperti
    area_constraint.jejak_robot(). Dengan begitu perencana dan pemantau
    keselamatan memakai badan yang persis sama - tidak ada peluang keduanya
    berselisih soal seberapa besar robotnya.
    """
    a = math.radians(yaw_deg)
    f = np.array([math.cos(a), math.sin(a)])
    l = np.array([-math.sin(a), math.cos(a)])
    c = np.array([x_cm, y_cm])
    fr, re, w = konfig.BODY_FRONT_CM, konfig.BODY_REAR_CM, konfig.HALF_WIDTH_CM
    return np.array([c + f * fr + l * w, c + f * fr - l * w,
                     c - f * re - l * w, c - f * re + l * w])


def gambar_robot(img, ctx, x_cm, y_cm, yaw_deg, warna, tebal=2, isi=False):
    """Jejak badan robot di tampilan atas, memakai geometri konfig.py."""
    a = math.radians(yaw_deg)
    f = np.array([math.cos(a), math.sin(a)])
    c = np.array([x_cm, y_cm])          # titik acuan = poros DEPAN
    fr = konfig.BODY_FRONT_CM
    pts = ctx.px(jejak_badan(x_cm, y_cm, yaw_deg))
    if isi:
        over = img.copy()
        cv2.fillPoly(over, [pts], warna)
        cv2.addWeighted(over, 0.3, img, 0.7, 0, img)
    cv2.polylines(img, [pts], True, warna, tebal, cv2.LINE_AA)
    ujung = ctx.px([c + f * (fr + 8)])[0]
    cv2.arrowedLine(img, tuple(ctx.px([c])[0]), tuple(ujung), warna, tebal, cv2.LINE_AA, tipLength=0.3)


def pilih(nama=None, boleh_baru=False):
    """Nama peta dari argumen, atau menu pilihan di terminal."""
    if nama:
        cocok = AC.cari_peta(nama)
        if cocok is None:
            raise SystemExit(f"[!] Peta '{nama}' tidak ada. Yang tersedia: "
                             f"{', '.join(AC.daftar_peta()) or '-'}")
        return cocok
    return AC.pilih_peta(boleh_baru=boleh_baru)


# ------------------------------------------------------------------ mandiri
def main():
    ap = argparse.ArgumentParser(description="Lihat peta constraint sebagai grid perencanaan")
    ap.add_argument("--peta", default=None)
    ap.add_argument("--sel", type=float, default=None, help="cm per sel (bawaan konfig.GRID_CM)")
    args = ap.parse_args()

    for tingkat, teks in konfig.periksa():
        print(f"[{tingkat}] {teks}")

    arena = Arena(pilih(args.peta), cm_per_cell=args.sel)
    print("[i]", arena.ringkas())

    x0, y0, x1, y1 = arena.ctx.ext
    print(f"[i] area {x1 - x0:.0f} x {y1 - y0:.0f} cm, asal ({x0:.1f}, {y0:.1f}) cm")
    print(f"[i] robot butuh lingkaran bebas ~{2 * konfig.R_SAPU_CM:.0f} cm untuk berputar penuh")

    # Peta jarak bebas: makin terang makin jauh dari rintangan. Ini persis angka
    # yang dipakai perencana untuk menolak pose yang menabrak, jadi kalau
    # gambarnya aneh, perencanaannya juga akan aneh.
    d = arena.omap.dist
    maks = float(d.max()) or 1.0
    panas = cv2.applyColorMap(np.clip(d / maks * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
    h, w = arena.ukuran_tampilan()
    panas = cv2.resize(panas, (w, h), interpolation=cv2.INTER_NEAREST)

    img = arena.kanvas()
    img[:] = panas
    arena.gambar_grid(img)
    arena.gambar_zona(img)

    # Lingkaran radius belok, ditaruh di titik paling lapang - supaya terlihat
    # apakah arena ini memang muat untuk memutar.
    iy, ix = np.unravel_index(np.argmax(d), d.shape)
    lapang_cm = arena.ke_cm(float(ix), float(iy))
    pusat = arena.ctx.px([lapang_cm])[0]
    for r_cm, warna, nama in ((konfig.R_PLAN_CM, (0, 255, 255), "R rencana"),
                              (konfig.R_SAPU_CM, (255, 200, 0), "R sapuan")):
        cv2.circle(img, tuple(pusat), int(round(r_cm * arena.ctx.s)), warna, 1, cv2.LINE_AA)
    AC.label(img, f"titik terlapang: {arena.panjang_ke_cm(d[iy, ix]):.0f} cm bebas",
             (pusat[0] + 8, pusat[1] - 8), (255, 255, 255), 0.45)

    AC.label(img, arena.ringkas(), (10, 22), (255, 255, 255), 0.5)
    AC.label(img, "kuning: radius belok rencana | biru: sapuan badan | q: keluar",
             (10, img.shape[0] - 10), (230, 230, 230), 0.45)

    judul = f"Arena - {arena.nama}"
    cv2.namedWindow(judul, cv2.WINDOW_AUTOSIZE)

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            P = arena.ctx.cm(x, y)
            print(f"\r  X={P[0]:7.1f} Y={P[1]:7.1f} cm | bebas {arena.bebas_cm(*P):6.1f} cm   ",
                  end="", flush=True)

    cv2.setMouseCallback(judul, on_mouse)
    while True:
        cv2.imshow(judul, img)
        if (cv2.waitKey(30) & 0xFF) in (ord("q"), 27):
            break
    print()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
