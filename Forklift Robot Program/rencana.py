"""
rencana.py - perencanaan jalur Hybrid A* di atas peta constraint.

Membungkus HybridAStarPlanner dari forklift_planner.py (disalin apa adanya dari
v1, tidak diubah - algoritmanya bebas satuan). Tugas berkas ini hanya satu:
menerjemahkan parameter cm di konfig.py menjadi sel grid, dan menerjemahkan
jalur yang dihasilkan kembali ke cm.

PERENCANA BISA DITUKAR (Perencana(..., jenis=...)):
    "astar"       Hybrid A* (bawaan)
    "astar-rs"    Hybrid A* dengan heuristik max(RS, Dijkstra) dan w = 2,0 (A3,
                  Tahap 1.1) - hanya untuk uji, bukan bawaan robot
    "dqfd"        jaringan DQfD robot itu (model_dqn/id0N.pt), Hybrid A* sebagai
                  cadangan bila DQfD gagal
    "dqfd-murni"  DQfD saja, tanpa cadangan - untuk menilai DQfD sendirian
Keduanya mengembalikan jalur berformat sama, jadi pelacak & navigasi tidak
tahu bedanya. Lihat drl/perencana_dqfd.py dan catatan Obsidian "Deep
Reinforcement Learning".

Titik acuan pose adalah POROS RODA DEPAN, bukan pusat AprilTag. Roda depan
tidak berbelok, jadi robot selalu bergerak searah heading di titik itu - itulah
yang membuat model sepeda berlaku untuk kendaraan berkemudi belakang.

Uji mandiri (tanpa kamera, tanpa robot):
    python rencana.py --peta Rumah
    python rencana.py --peta Rumah --id 2 --perencana dqfd
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
from forklift_planner import HybridAStarPlanner, ObstacleMap, VehicleModel

konfig.pasang_path_kamera()
import area_constraint as AC          # noqa: E402

JENIS_PERENCANA = ("astar", "astar-rs", "dqfd", "dqfd-murni")
# Varian Hybrid A* yang dipilih lewat nama jenis (uji closed-loop Tahap 1.1):
# "astar-rs" = A3, heuristik max(RS tanpa rintangan, Dijkstra) dengan w = 2,0.
OPSI_JENIS = {"astar-rs": {"heuristik": "rs", "h_weight": 2.0}}


def _buat_jam_cpu_thread():
    """Fungsi -> detik CPU yang sudah dipakai thread PEMANGGIL.

    time.thread_time() di Windows memakai GetThreadTimes, yang hanya naik per
    detak penjadwal (15,625 ms) - lebih kasar daripada satu rencana (5-30 ms).
    Di Windows dipakai QueryThreadCycleTime (siklus CPU thread, resolusi tinggi),
    dikonversi ke detik dengan frekuensi yang dikalibrasi sekali terhadap
    perf_counter (20 ms, saat modul dimuat). Di sistem lain thread_time sudah
    berresolusi nanodetik.
    """
    try:
        import ctypes
        k32 = ctypes.windll.kernel32
        k32.GetCurrentThread.restype = ctypes.c_void_p

        def siklus():
            # Penampung BARU setiap panggilan: satu penampung bersama dibaca-tulis
            # oleh beberapa thread sekaligus, dan thread bisa membaca siklus
            # milik thread lain (terukur: CPU 11 ms untuk rencana 26 ms).
            nilai = ctypes.c_ulonglong()
            k32.QueryThreadCycleTime(ctypes.c_void_p(k32.GetCurrentThread()), ctypes.byref(nilai))
            return nilai.value

        c0, t0 = siklus(), time.perf_counter()
        while time.perf_counter() - t0 < 0.02:
            pass
        hz = (siklus() - c0) / (time.perf_counter() - t0)
        if hz <= 0:
            raise OSError("frekuensi siklus tidak valid")
        return lambda: siklus() / hz
    except (AttributeError, OSError):
        return time.thread_time


jam_cpu_thread = _buat_jam_cpu_thread()


def buat_kendaraan(arena, margin_cm=None, max_steer_rad=None):
    """VehicleModel dengan seluruh panjang dinyatakan dalam SEL grid.

    max_steer_rad: batas kemudi robot yang direncanakan. ID 01 dan ID 02
    berbeda (35 vs 23,5 derajat), jadi program dua robot WAJIB mengisinya.
    """
    s = arena.panjang_ke_sel
    return VehicleModel(
        wheelbase=s(konfig.WHEELBASE_CM),
        max_steer=konfig.MAX_STEER_RAD if max_steer_rad is None else max_steer_rad,
        body_front=s(konfig.BODY_FRONT_CM),
        body_rear=s(konfig.BODY_REAR_CM),
        half_width=s(konfig.HALF_WIDTH_CM),
        safety_margin=s(konfig.SAFETY_MARGIN_CM if margin_cm is None else margin_cm),
    )


class HybridAStarBertanda(HybridAStarPlanner):
    """HybridAStarPlanner yang mencatat bagian jalur mana yang Reeds-Shepp.

    Jalur Hybrid A* terdiri dari dua bagian: primitif gerak hasil pencarian,
    lalu - bila pencarian berakhir lewat ekspansi analitik - satu tembakan
    Reeds-Shepp ke tujuan. plan() hanya mengembalikan gabungannya, dan
    forklift_planner.py tidak diubah, jadi batasnya dicatat di sini:
    last_stats["rs_titik"] = jumlah titik di UJUNG jalur yang berasal dari
    Reeds-Shepp (0 = tujuan dicapai primitif saja).

    _analytic dipanggil berkali-kali selama pencarian, tapi pencarian langsung
    berhenti begitu satu panggilan berhasil - jadi nilai dari panggilan
    terakhirlah yang benar.

    last_stats["rincian_s"] = {"astar": ..., "rs": ...}: waktu di dalam SEMUA
    panggilan _analytic (Reeds-Shepp + pemeriksaan tabrakannya, termasuk yang
    gagal) dan sisanya (heuristik Dijkstra + ekspansi primitif).
    """

    # Nilai awal: _analytic juga dipanggil LANGSUNG di luar plan() (mis.
    # lingkungan.rs_bebas() saat memutar ulang demo), sebelum plan() sempat
    # mengisi penghitung ini.
    _t_rs = 0.0
    _rs_titik = 0

    def _analytic(self, omap, node, goal):
        t = time.perf_counter()
        rs = super()._analytic(omap, node, goal)
        self._t_rs += time.perf_counter() - t
        self._rs_titik = len(rs[0]) if rs is not None else 0
        return rs

    def plan(self, omap, start, goal):
        self._rs_titik = 0
        self._t_rs = 0.0
        t0 = time.perf_counter()
        jalur = super().plan(omap, start, goal)
        total = time.perf_counter() - t0
        self.last_stats["rs_titik"] = self._rs_titik if jalur else 0
        self.last_stats["rincian_s"] = {"astar": total - self._t_rs, "rs": self._t_rs}
        return jalur


def buat_perencana(arena, kendaraan, opsi=None):
    """HybridAStarBertanda dari konfig.PARAM_RENCANA.

    opsi (varian baseline, benchmark Tahap 1.1; None = konfigurasi robot):
      h_weight      bobot heuristik (bawaan PARAM_RENCANA, 1,3)
      heuristik     "dijkstra" (bawaan) | "rs": max(biaya RS tanpa rintangan, Dijkstra)
      saring_serah  True = tembakan RS hanya bila peta Dijkstra tidak memaksa memutar
                    (atau < 25 cm), saringan handover yang sama dengan model jauh DQfD
    """
    p = konfig.PARAM_RENCANA
    s = arena.panjang_ke_sel
    opsi = opsi or {}
    planner = HybridAStarBertanda(
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
        # None = Reeds-Shepp hanya di dalam analytic_dist. Perencana memeriksa
        # "ekspansi % analytic_every == 0", jadi dimatikan dengan angka yang
        # tidak pernah tercapai (ekspansi berhenti di max_expansions).
        analytic_every=p["analytic_every"] or p["max_expansions"] + 1,
        analytic_dist=s(p["analytic_dist"]),
        time_limit=p["time_limit"],
        max_expansions=p["max_expansions"],
    )
    if "h_weight" in opsi:
        planner.h_weight = float(opsi["h_weight"])
    if opsi.get("heuristik", "dijkstra") == "rs":
        import heuristik_rs
        planner.h_rs = heuristik_rs.tabel_untuk(planner)
    if opsi.get("saring_serah"):
        from drl.lingkungan import RADIUS_SERAH_PAKSA_CM
        planner.serah_saring = True
        planner.serah_paksa = s(RADIUS_SERAH_PAKSA_CM)
    return planner


class Perencana:
    """Hybrid A* untuk satu arena. Pose masuk & keluar dalam cm/derajat.

    profil: konfig.profil() robot yang direncanakan (batas kemudinya). None =
    robot yang sedang dipilih.
    jenis: salah satu JENIS_PERENCANA, lihat docstring modul.
    """

    def __init__(self, arena, profil=None, jenis="astar", opsi=None):
        """opsi: varian perencana untuk benchmark (None = konfigurasi robot).
        jenis "astar": lihat buat_perencana(). jenis DQfD: bobot, saring_serah,
        metode ("gabungan" = dengan backtracking, "terbaik" = tanpa) - lihat DQfDPlanner."""
        if jenis not in JENIS_PERENCANA:
            raise SystemExit(f"[!] Perencana '{jenis}' tidak dikenal. Yang ada: {', '.join(JENIS_PERENCANA)}")
        self.arena = arena
        self.p = profil or konfig.profil()
        self.jenis = jenis
        self.opsi = {**OPSI_JENIS.get(jenis, {}), **(opsi or {})}
        self._cache = {}
        self.kendaraan, self.planner = self._untuk(konfig.SAFETY_MARGIN_CM)
        # Margin & kendaraan yang BENAR-BENAR dipakai jalur terakhir. Bisa lebih
        # kecil dari yang diinginkan kalau terpaksa mundur ke cadangan.
        self.margin_dipakai = konfig.SAFETY_MARGIN_CM
        self.kendaraan_dipakai = self.kendaraan
        self.stats = {}
        self._rincian = {}

    def _untuk(self, margin_cm):
        """(VehicleModel, HybridAStarPlanner) untuk satu nilai margin, di-cache."""
        kunci = round(float(margin_cm), 3)
        if kunci not in self._cache:
            k = buat_kendaraan(self.arena, kunci, self.p.max_steer_rad)
            if not self.jenis.startswith("dqfd"):
                planner = buat_perencana(self.arena, k, self.opsi)
            else:
                # Diimpor di sini, bukan di atas: tanpa DQfD, rencana.py (dan
                # navigasi.py) tidak perlu PyTorch sama sekali.
                from drl.perencana_dqfd import DQfDPlanner
                # Hybrid A* di dalam DQfD (pustaka gerak + cadangan) selalu konfigurasi robot.
                planner = DQfDPlanner(buat_perencana(self.arena, k), self.arena, self.p, kunci,
                                      cadangan=self.jenis == "dqfd", **self.opsi)
            self._cache[kunci] = (k, planner)
        return self._cache[kunci]

    def peta_dengan(self, rintangan_cm):
        """ObstacleMap arena ditambah poligon rintangan sementara (cm).

        Dipakai untuk robot LAIN: badannya dijadikan rintangan hanya untuk satu
        kali perencanaan, tanpa mengubah peta arena yang dipakai bersama.
        Margin keselamatan tidak ditambahkan di sini - perencana sudah
        menggembungkan badan robot SENDIRI dengan margin itu.
        """
        if not rintangan_cm:
            return self.arena.omap
        occ = self.arena.occ.copy()
        for poli in rintangan_cm:
            cv2.fillPoly(occ, [self.arena.sel_poligon(poli)], 255)
        return ObstacleMap(occ)

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

    def margin_untuk_pose(self, x_cm, y_cm, yaw_deg=None, rintangan_cm=None):
        """Margin TERBESAR yang masih membuat pose ini bebas, atau None.

        Dipakai menolak tujuan sebelum robot berangkat. Tanpa ini, tujuan yang
        hanya muat dengan jarak aman tipis tetap diterima, robot menempuh dua
        meter untuk sampai ke sana, lalu berdiri di dalam zona - dan barulah
        ketahuan.

        yaw_deg None berarti arah hadap bebas: dicoba beberapa arah, diambil
        yang terbaik, karena perencana nanti juga akan memilih sendiri.

        rintangan_cm: poligon tambahan (badan robot lain), sama seperti di
        rencanakan() - supaya penolakan di sini memakai ukuran yang persis sama
        dengan yang nanti dipakai perencana.
        """
        omap = self.peta_dengan(rintangan_cm)
        sudut = [yaw_deg] if yaw_deg is not None else list(range(0, 360, 30))
        for margin in self.margin_cadangan():
            kendaraan, _ = self._untuk(margin)
            for d in sudut:
                gx, gy, yaw = self.arena.pose_ke_sel(x_cm, y_cm, d)
                if self.arena.di_dalam(x_cm, y_cm) and omap.pose_free(kendaraan, gx, gy, yaw):
                    return margin
        return None

    def margin_cm(self, x_cm, y_cm, yaw_deg):
        gx, gy, yaw = self.arena.pose_ke_sel(x_cm, y_cm, yaw_deg)
        return self.arena.panjang_ke_cm(
            self.arena.omap.pose_margin(self.kendaraan, gx, gy, yaw))

    # ---------------------------------------------------------- perencanaan
    def rencanakan(self, awal_cm, tujuan_cm, rintangan_cm=None, cadangan=True):
        """awal/tujuan = (x_cm, y_cm, yaw_deg). yaw tujuan boleh None (bebas).

        rintangan_cm: poligon tambahan (cm) yang harus dihindari kali ini saja,
        misalnya badan robot lain. None = hanya zona peta.

        cadangan: False = hanya margin penuh, tanpa mundur ke MARGIN_CADANGAN_CM.
        Untuk jalur yang OPSIONAL (memutari atau minggir dari robot lain):
        menunggu lebih baik daripada jalan memutar panjang yang mepet zona.

        Kembalian: (jalur_sel, info). jalur_sel adalah daftar (gx, gy, yaw_rad)
        dalam SEL - format asli perencana, langsung bisa dipakai pelacak.
        """
        t0 = time.time()
        # Waktu CPU thread ini saja: tidak memuat thread lain (loop kendali,
        # perencana robot lain) yang berebut GIL selama rencana berjalan.
        c0 = jam_cpu_thread()
        self._rincian = {}
        omap = self.peta_dengan(rintangan_cm)
        jalur = None
        for margin in self.margin_cadangan() if cadangan else [konfig.SAFETY_MARGIN_CM]:
            self.kendaraan_dipakai, self.planner = self._untuk(margin)
            self.margin_dipakai = margin
            jalur = self._sekali(awal_cm, tujuan_cm, omap)
            if jalur:
                break

        dt = time.time() - t0
        cpu = jam_cpu_thread() - c0
        # Sisa di luar perencana (peta rintangan sementara, pemeriksaan awal).
        rincian = dict(self._rincian, lain=max(0.0, dt - sum(self._rincian.values())))
        self.stats = dict(self.planner.last_stats or {})
        self.stats["waktu_s"] = dt
        self.stats["margin_cm"] = self.margin_dipakai
        # "kendaraan" ikut dikembalikan: pemanggil yang merencanakan beberapa
        # kali lalu memilih salah satu (navigasi.py, pencarian minggir) harus
        # memakai kendaraan milik jalur yang dipilih, bukan kendaraan_dipakai
        # dari percobaan TERAKHIR.
        return jalur, {"waktu_s": dt, "cpu_s": cpu,"titik": len(jalur) if jalur else 0,
                       "panjang_cm": self.panjang_cm(jalur) if jalur else 0.0,
                       "margin_cm": self.margin_dipakai,
                       "margin_turun": bool(jalur) and self.margin_dipakai < konfig.SAFETY_MARGIN_CM - 0.01,
                       "kendaraan": self.kendaraan_dipakai,
                       "perencana": self.stats.get("perencana", "A*"),
                       "rs_titik": self.stats.get("rs_titik", 0) if jalur else 0,
                       "rincian_s": rincian,
                       "alasan": self.stats.get("reason", "")}

    def _sekali(self, awal_cm, tujuan_cm, omap=None):
        """Satu percobaan perencanaan dengan margin yang sedang aktif."""
        arena = self.arena
        omap = arena.omap if omap is None else omap
        awal = arena.pose_ke_sel(*awal_cm)
        yaw_tujuan = tujuan_cm[2]

        if yaw_tujuan is not None:
            tujuan = arena.pose_ke_sel(tujuan_cm[0], tujuan_cm[1], yaw_tujuan)
            jalur = self._plan(omap, awal, tujuan)
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
                    jalur = self._plan(omap, awal, t)
                    if jalur:
                        break
            finally:
                self.planner.time_limit = simpan

        return jalur

    def _plan(self, omap, awal, tujuan):
        """planner.plan() yang menjumlahkan rincian waktunya ke self._rincian.

        Satu rencanakan() bisa memanggil perencana berkali-kali (margin cadangan,
        arah hadap bebas) - rincian yang dilaporkan harus mencakup semuanya.
        """
        jalur = self.planner.plan(omap, awal, tujuan)
        for k, v in (self.planner.last_stats or {}).get("rincian_s", {}).items():
            self._rincian[k] = self._rincian.get(k, 0.0) + v
        return jalur

    def panjang_cm(self, jalur_sel):
        if not jalur_sel or len(jalur_sel) < 2:
            return 0.0
        total = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(jalur_sel[:-1], jalur_sel[1:]))
        return self.arena.panjang_ke_cm(total)


# Warna jalur (BGR). Bagian Hybrid A* = primitif hasil pencarian; bagian
# Reeds-Shepp = tembakan analitik penutup ke tujuan (info["rs_titik"]).
WARNA_HA_MAJU, WARNA_HA_MUNDUR = (0, 255, 0), (0, 165, 255)      # hijau, oranye
WARNA_RS_MAJU, WARNA_RS_MUNDUR = (255, 255, 0), (255, 0, 255)    # biru muda, ungu
LEGENDA_JALUR = ("Hybrid A* / DQfD jauh: hijau maju, oranye mundur | "
                 "Reeds-Shepp / DQfD dekat: biru muda maju, ungu mundur")


def gambar_jalur(img, arena, jalur_sel, rs_titik=0, label="RS",
                 warna_maju=WARNA_HA_MAJU, warna_mundur=WARNA_HA_MUNDUR):
    """Jalur digambar per arah gerak dan per asal.

    Maju dan mundur diberi warna berbeda. rs_titik = jumlah titik di ujung
    jalur yang berasal dari Reeds-Shepp, atau dari model DQfD dekat
    (Perencana.rencanakan -> info): bagian itu digambar dengan WARNA_RS_*, dan
    titik sambungnya ditandai label ("RS" untuk Hybrid A*, "serah" untuk DQfD).
    """
    if not jalur_sel or len(jalur_sel) < 2:
        return
    titik_cm = [arena.ke_cm(p[0], p[1]) for p in jalur_sel]
    px = arena.ctx.px(titik_cm)
    # Segmen i (titik i -> i+1) termasuk Reeds-Shepp bila titik i+1 bagian dari
    # ekor RS; segmen pertama RS berawal di titik terakhir primitif.
    awal_rs = len(jalur_sel) - rs_titik if 0 < rs_titik < len(jalur_sel) else None
    for i in range(len(px) - 1):
        a, b = jalur_sel[i], jalur_sel[i + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        maju = (dx * math.cos(a[2]) + dy * math.sin(a[2])) >= 0
        if awal_rs is not None and i + 1 >= awal_rs:
            warna = WARNA_RS_MAJU if maju else WARNA_RS_MUNDUR
        else:
            warna = warna_maju if maju else warna_mundur
        cv2.line(img, tuple(px[i]), tuple(px[i + 1]), warna, 2, cv2.LINE_AA)
    if awal_rs is not None:
        u, v = (int(c) for c in px[awal_rs - 1])
        cv2.circle(img, (u, v), 5, (255, 255, 255), 2, cv2.LINE_AA)
        AC.label(img, label, (u + 7, v - 7), (255, 255, 255), 0.45)


# ------------------------------------------------------------------ mandiri
def main():
    ap = argparse.ArgumentParser(description="Taman uji Hybrid A* di atas peta constraint")
    ap.add_argument("--peta", default=None)
    ap.add_argument("--sel", type=float, default=None)
    ap.add_argument("--id", type=int, default=None, help="robot (batas kemudi & model DQfD-nya)")
    ap.add_argument("--perencana", default="astar", choices=JENIS_PERENCANA)
    args = ap.parse_args()
    if args.id is not None:
        konfig.pilih_robot(args.id)

    for tingkat, teks in konfig.periksa():
        print(f"[{tingkat}] {teks}")

    arena = A.Arena(A.pilih(args.peta), cm_per_cell=args.sel)
    per = Perencana(arena, jenis=args.perencana)
    print("[i]", arena.ringkas(), f"| robot {konfig.ROBOT_ID:02d} | perencana {args.perencana}")
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
                  f"{info['waktu_s'] * 1000:.0f} ms, oleh {info['perencana']}")
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
        gambar_jalur(img, arena, jalur, rs_titik=info.get("rs_titik", 0) if jalur else 0,
                     label="serah" if str(info.get("perencana", "")).startswith("dqfd") else "RS")
        A.gambar_robot(img, arena.ctx, *awal, (255, 255, 255), isi=True)
        if tujuan is not None:
            yaw = tujuan[2] if tujuan[2] is not None else 0.0
            A.gambar_robot(img, arena.ctx, tujuan[0], tujuan[1], yaw,
                           (0, 255, 255), tebal=1)
        tunjuk_tujuan.gambar(img, (0, 255, 255))
        tunjuk_awal.gambar(img, (255, 255, 255))

        teks = arena.ringkas()
        if jalur:
            teks += (f" | jalur {info['panjang_cm']:.0f} cm, {info['waktu_s'] * 1000:.0f} ms"
                     f" ({info['perencana']}), Reeds-Shepp {info.get('rs_titik', 0)} titik")
        elif tujuan is not None:
            teks += " | TIDAK ADA JALUR"
        AC.label(img, teks, (10, 22), (255, 255, 255), 0.5)
        AC.label(img, "kiri: tujuan | kanan: awal | r: hitung ulang | c: hapus | q: keluar",
                 (10, img.shape[0] - 10), (230, 230, 230), 0.45)
        AC.label(img, LEGENDA_JALUR, (10, img.shape[0] - 30), (230, 230, 230), 0.45)
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
