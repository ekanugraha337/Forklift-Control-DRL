"""
rencana.py - perencanaan jalur Hybrid A* di atas peta constraint.

Membungkus HybridAStarPlanner dari forklift_planner.py (disalin apa adanya dari
v1, tidak diubah - algoritmanya bebas satuan). Tugas berkas ini hanya satu:
menerjemahkan parameter cm di konfig.py menjadi sel grid, dan menerjemahkan
jalur yang dihasilkan kembali ke cm.

Titik acuan pose adalah POROS RODA DEPAN, bukan pusat AprilTag. Roda depan
tidak berbelok, jadi robot selalu bergerak searah heading di titik itu - itulah
yang membuat model sepeda berlaku untuk kendaraan berkemudi belakang.

Uji mandiri (tanpa kamera, tanpa robot):
    python rencana.py --peta Rumah
        klik-tarik kiri  : tetapkan TUJUAN (klik = posisi, tarik = arah hadap)
        klik-tarik kanan : tetapkan AWAL
        r: hitung ulang   c: hapus jalur   q: keluar
"""
import argparse
import math
import time

import cv2
import numpy as np

import arena as A
import konfig
from forklift_planner import HybridAStarPlanner, VehicleModel

konfig.pasang_path_kamera()
import area_constraint as AC          # noqa: E402


def buat_kendaraan(arena, margin_cm=None):
    """VehicleModel dengan seluruh panjang dinyatakan dalam SEL grid."""
    s = arena.panjang_ke_sel
    return VehicleModel(
        wheelbase=s(konfig.WHEELBASE_CM),
        max_steer=konfig.MAX_STEER_RAD,
        body_front=s(konfig.BODY_FRONT_CM),
        body_rear=s(konfig.BODY_REAR_CM),
        half_width=s(konfig.HALF_WIDTH_CM),
        safety_margin=s(konfig.SAFETY_MARGIN_CM if margin_cm is None else margin_cm),
    )


def buat_perencana(arena, kendaraan):
    p = konfig.PARAM_RENCANA
    s = arena.panjang_ke_sel
    return HybridAStarPlanner(
        kendaraan,
        plan_steer_ratio=konfig.PLAN_STEER_RATIO,
        allow_reverse=konfig.ALLOW_REVERSE,
        xy_reso=s(p["xy_reso"]),
        yaw_reso=math.radians(p["yaw_reso_deg"]),
        step=s(p["step"]),
        n_steer=p["n_steer"],
        sample_ds=s(p["sample_ds"]),
        reverse_cost=p["reverse_cost"],
        switch_cost=s(p["switch_cost"]),
        steer_cost=p["steer_cost"],
        steer_change_cost=p["steer_change_cost"],
        clearance_pref=s(p["clearance_pref"]),
        clearance_cost=p["clearance_cost"],
        h_weight=p["h_weight"],
        # h_reso adalah langkah indeks array, jadi harus bilangan bulat >= 1.
        h_reso=max(1, int(round(s(p["h_reso"])))),
        approach_len=s(p["approach_len"]),
        goal_pos_tol=s(p["goal_pos_tol"]),
        goal_yaw_tol=math.radians(p["goal_yaw_tol_deg"]),
        analytic_every=p["analytic_every"],
        analytic_dist=s(p["analytic_dist"]),
        time_limit=p["time_limit"],
        max_expansions=p["max_expansions"],
    )


class Perencana:
    """Hybrid A* untuk satu arena. Pose masuk & keluar dalam cm/derajat."""

    def __init__(self, arena):
        self.arena = arena
        self._cache = {}
        self.kendaraan, self.planner = self._untuk(konfig.SAFETY_MARGIN_CM)
        # Margin & kendaraan yang BENAR-BENAR dipakai jalur terakhir. Bisa lebih
        # kecil dari yang diinginkan kalau terpaksa mundur ke cadangan.
        self.margin_dipakai = konfig.SAFETY_MARGIN_CM
        self.kendaraan_dipakai = self.kendaraan
        self.stats = {}

    def _untuk(self, margin_cm):
        """(VehicleModel, HybridAStarPlanner) untuk satu nilai margin, di-cache."""
        kunci = round(float(margin_cm), 3)
        if kunci not in self._cache:
            k = buat_kendaraan(self.arena, kunci)
            self._cache[kunci] = (k, buat_perencana(self.arena, k))
        return self._cache[kunci]

    def margin_cadangan(self):
        """Urutan margin yang dicoba: yang diinginkan dulu, lalu mundur bertahap.

        Margin keselamatan ada untuk menjaga JALUR menjauh dari zona. Tapi pose
        AWAL bukan pilihan - robot sudah berada di situ, dan menolaknya hanya
        membuat robot terkunci di tempat tanpa jalan keluar. Begitu juga koridor
        yang memang lebih sempit daripada margin: lebih baik lewat dengan margin
        kecil sambil mengatakannya, daripada diam dan melapor "tidak ada jalur".
        """
        penuh = konfig.SAFETY_MARGIN_CM
        urut = [penuh]
        for m in konfig.MARGIN_CADANGAN_CM:
            if m < penuh - 0.01:
                urut.append(m)
        return urut

    # ------------------------------------------------------------ bantuan
    def pose_bebas(self, x_cm, y_cm, yaw_deg):
        """Apakah badan robot pada pose ini bebas dari zona?"""
        if not self.arena.di_dalam(x_cm, y_cm):
            return False
        gx, gy, yaw = self.arena.pose_ke_sel(x_cm, y_cm, yaw_deg)
        return self.arena.omap.pose_free(self.kendaraan, gx, gy, yaw)

    def margin_untuk_pose(self, x_cm, y_cm, yaw_deg=None):
        """Margin TERBESAR yang masih membuat pose ini bebas, atau None.

        Dipakai menolak tujuan sebelum robot berangkat. Tanpa ini, tujuan yang
        hanya muat dengan jarak aman tipis tetap diterima, robot menempuh dua
        meter untuk sampai ke sana, lalu berdiri di dalam zona - dan barulah
        ketahuan.

        yaw_deg None berarti arah hadap bebas: dicoba beberapa arah, diambil
        yang terbaik, karena perencana nanti juga akan memilih sendiri.
        """
        sudut = [yaw_deg] if yaw_deg is not None else list(range(0, 360, 30))
        for margin in self.margin_cadangan():
            kendaraan, _ = self._untuk(margin)
            for d in sudut:
                gx, gy, yaw = self.arena.pose_ke_sel(x_cm, y_cm, d)
                if self.arena.di_dalam(x_cm, y_cm) and \
                        self.arena.omap.pose_free(kendaraan, gx, gy, yaw):
                    return margin
        return None

    def margin_cm(self, x_cm, y_cm, yaw_deg):
        gx, gy, yaw = self.arena.pose_ke_sel(x_cm, y_cm, yaw_deg)
        return self.arena.panjang_ke_cm(
            self.arena.omap.pose_margin(self.kendaraan, gx, gy, yaw))

    # ---------------------------------------------------------- perencanaan
    def rencanakan(self, awal_cm, tujuan_cm):
        """awal/tujuan = (x_cm, y_cm, yaw_deg). yaw tujuan boleh None (bebas).

        Kembalian: (jalur_sel, info). jalur_sel adalah daftar (gx, gy, yaw_rad)
        dalam SEL - format asli perencana, langsung bisa dipakai pelacak.
        """
        t0 = time.time()
        jalur = None
        for margin in self.margin_cadangan():
            self.kendaraan_dipakai, self.planner = self._untuk(margin)
            self.margin_dipakai = margin
            jalur = self._sekali(awal_cm, tujuan_cm)
            if jalur:
                break

        dt = time.time() - t0
        self.stats = dict(self.planner.last_stats or {})
        self.stats["waktu_s"] = dt
        self.stats["margin_cm"] = self.margin_dipakai
        return jalur, {"waktu_s": dt, "titik": len(jalur) if jalur else 0,
                       "panjang_cm": self.panjang_cm(jalur) if jalur else 0.0,
                       "margin_cm": self.margin_dipakai,
                       "margin_turun": bool(jalur) and self.margin_dipakai < konfig.SAFETY_MARGIN_CM - 0.01}

    def _sekali(self, awal_cm, tujuan_cm):
        """Satu percobaan perencanaan dengan margin yang sedang aktif."""
        arena = self.arena
        awal = arena.pose_ke_sel(*awal_cm)
        yaw_tujuan = tujuan_cm[2]

        if yaw_tujuan is not None:
            tujuan = arena.pose_ke_sel(tujuan_cm[0], tujuan_cm[1], yaw_tujuan)
            jalur = self.planner.plan(arena.omap, awal, tujuan)
        else:
            # Arah hadap bebas: perencana selalu menuntut SATU heading tujuan,
            # jadi pilihannya dicoba di sini - bukan dibiarkan jadi tebakan diam-diam.
            #
            # Urutannya tidak acak: arah yang searah garis awal->tujuan dicoba
            # lebih dulu, lalu menyimpang makin jauh. Itu arah yang paling mungkin
            # berhasil sekaligus menghasilkan jalur terpendek, sehingga biasanya
            # cukup satu percobaan. Mencoba 0..330 berurutan akan menghabiskan
            # waktu di arah yang jelas tidak masuk akal lebih dulu.
            bearing = math.degrees(math.atan2(tujuan_cm[1] - awal_cm[1],
                                              tujuan_cm[0] - awal_cm[0]))
            urutan = [bearing]
            for d in range(30, 181, 30):
                urutan += [bearing + d, bearing - d]

            # Batas waktu per percobaan diperkecil: yang dicari jawaban cepat,
            # bukan jawaban sempurna untuk arah yang memang tidak cocok.
            simpan = self.planner.time_limit
            self.planner.time_limit = min(simpan, 0.4)
            try:
                for deg in urutan:
                    t = arena.pose_ke_sel(tujuan_cm[0], tujuan_cm[1], deg)
                    jalur = self.planner.plan(arena.omap, awal, t)
                    if jalur:
                        break
            finally:
                self.planner.time_limit = simpan

        return jalur

    def panjang_cm(self, jalur_sel):
        if not jalur_sel or len(jalur_sel) < 2:
            return 0.0
        total = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(jalur_sel[:-1], jalur_sel[1:]))
        return self.arena.panjang_ke_cm(total)


def gambar_jalur(img, arena, jalur_sel, warna_maju=(0, 255, 0), warna_mundur=(0, 165, 255)):
    """Jalur digambar per arah gerak: maju dan mundur diberi warna berbeda."""
    if not jalur_sel or len(jalur_sel) < 2:
        return
    titik_cm = [arena.ke_cm(p[0], p[1]) for p in jalur_sel]
    px = arena.ctx.px(titik_cm)
    for i in range(len(px) - 1):
        a, b = jalur_sel[i], jalur_sel[i + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        maju = (dx * math.cos(a[2]) + dy * math.sin(a[2])) >= 0
        cv2.line(img, tuple(px[i]), tuple(px[i + 1]),
                 warna_maju if maju else warna_mundur, 2, cv2.LINE_AA)


# ------------------------------------------------------------------ mandiri
def main():
    ap = argparse.ArgumentParser(description="Taman uji Hybrid A* di atas peta constraint")
    ap.add_argument("--peta", default=None)
    ap.add_argument("--sel", type=float, default=None)
    args = ap.parse_args()

    for tingkat, teks in konfig.periksa():
        print(f"[{tingkat}] {teks}")

    arena = A.Arena(A.pilih(args.peta), cm_per_cell=args.sel)
    per = Perencana(arena)
    print("[i]", arena.ringkas())
    print(f"[i] kendaraan: {len(per.kendaraan.circle_offsets)} lingkaran, jari-jari "
          f"{arena.panjang_ke_cm(per.kendaraan.circle_radius):.2f} cm "
          f"-> butuh jarak bebas segitu untuk lewat")
    print("[i] klik-tarik KIRI = tujuan, KANAN = awal, r = ulang, c = hapus, q = keluar")

    # Titik awal: tempat paling lapang, menghadap ke kanan.
    iy, ix = np.unravel_index(np.argmax(arena.omap.dist), arena.omap.dist.shape)
    awal = (*arena.ke_cm(float(ix), float(iy)), 0.0)
    tujuan, jalur, info = None, None, {}

    judul = f"Rencana Hybrid A* - {arena.nama}"
    cv2.namedWindow(judul, cv2.WINDOW_AUTOSIZE)
    tunjuk_tujuan = A.PenunjukPose(arena.ctx, tombol=cv2.EVENT_LBUTTONDOWN)
    tunjuk_awal = A.PenunjukPose(arena.ctx, tombol=cv2.EVENT_RBUTTONDOWN)

    def on_mouse(event, x, y, flags, param):
        tunjuk_tujuan.on_mouse(event, x, y, flags, param)
        tunjuk_awal.on_mouse(event, x, y, flags, param)

    cv2.setMouseCallback(judul, on_mouse)

    latar = arena.kanvas()
    arena.gambar_grid(latar)
    arena.gambar_zona(latar)

    def hitung():
        nonlocal jalur, info
        if tujuan is None:
            return
        if not per.pose_bebas(*awal):
            print("[!] Pose AWAL menabrak zona - jalur tidak dihitung.")
            jalur = None
            return
        yaw_uji = tujuan[2] if tujuan[2] is not None else 0.0
        if tujuan[2] is not None and not per.pose_bebas(tujuan[0], tujuan[1], yaw_uji):
            print("[!] Pose TUJUAN menabrak zona - jalur tidak dihitung.")
            jalur = None
            return
        jalur, info = per.rencanakan(awal, tujuan)
        if jalur:
            print(f"[OK] {info['titik']} titik, {info['panjang_cm']:.0f} cm, "
                  f"{info['waktu_s'] * 1000:.0f} ms")
        else:
            print(f"[!] Tidak ada jalur ({info['waktu_s'] * 1000:.0f} ms). "
                  f"Coba arah hadap lain, atau tujuan yang lebih lapang.")

    while True:
        baru = tunjuk_awal.ambil()
        if baru:
            awal = (baru[0], baru[1], baru[2] if baru[2] is not None else awal[2])
            hitung()
        baru = tunjuk_tujuan.ambil()
        if baru:
            tujuan = baru
            hitung()

        img = latar.copy()
        gambar_jalur(img, arena, jalur)
        A.gambar_robot(img, arena.ctx, *awal, (255, 255, 255), isi=True)
        if tujuan is not None:
            yaw = tujuan[2] if tujuan[2] is not None else 0.0
            A.gambar_robot(img, arena.ctx, tujuan[0], tujuan[1], yaw,
                           (0, 255, 255), tebal=1)
        tunjuk_tujuan.gambar(img, (0, 255, 255))
        tunjuk_awal.gambar(img, (255, 255, 255))

        teks = arena.ringkas()
        if jalur:
            teks += f" | jalur {info['panjang_cm']:.0f} cm, {info['waktu_s'] * 1000:.0f} ms"
        elif tujuan is not None:
            teks += " | TIDAK ADA JALUR"
        AC.label(img, teks, (10, 22), (255, 255, 255), 0.5)
        AC.label(img, "kiri: tujuan | kanan: awal | hijau: maju, oranye: mundur",
                 (10, img.shape[0] - 10), (230, 230, 230), 0.45)
        cv2.imshow(judul, img)

        k = cv2.waitKey(30) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("r"):
            hitung()
        if k == ord("c"):
            jalur, tujuan = None, None

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
