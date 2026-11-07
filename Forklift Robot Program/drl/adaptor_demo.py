"""
adaptor_demo.py - jalur Hybrid A* -> episode demonstrasi (s, a, r, s') untuk model jauh DQfD.

Demonstratornya Hybrid A* + Reeds-Shepp peran "demonstrator" (rencana.buat_perencana:
w = 2,0, h = max(h_RS, h_D), RS dicoba di setiap node < 60 cm dari tujuan).
Karena lingkungan perencanaan deterministik dan memakai aritmetika A* sendiri,
aksi demonstrator tidak perlu ditebak - ia DIREKONSTRUKSI tepat dari jalur:

    jalur = [awal] + (n_sub pose per primitif) x N + (pose ekor Reeds-Shepp)
    tiap kelompok n_sub pose -> satu primitif: arah dari kolom arah,
                                kappa = delta-yaw / (arah x step), dibulatkan
    ekor Reeds-Shepp          -> BUKAN aksi: lingkungan berhenti (SERAH) tepat
                                 setelah primitif terakhir dan memasang tembakan
                                 RS yang sama (_analytic), PERSIS ekor A*

plan() tidak mengembalikan g-cost, jadi biaya A* direkam dari luar selama
plan() berjalan: _h_cari dan _analytic dibungkus di tingkat OBJEK, dan heapq
modul forklift_planner ditukar sementara dengan perekam.

UJI PENERIMAAN:
    python -m drl.adaptor_demo --uji                      # ID 01 & 02, semua peta
    python -m drl.adaptor_demo --uji --id 2 --peta Kalibrasi --n 30

    Lulus bila, untuk SETIAP jalur yang ditemukan A*:
      1. episode jauh lolos mask, SERAH tepat setelah primitif A* terakhir, dan
         jalurnya (primitif + ekor RS) IDENTIK dengan jalur A*;
      2. biaya episode = g-cost A* seluruh jalur;
      3. atau demo ditolak dengan salah satu alasan sah di TOLAK;
      4. observasi berukuran 47 dan berhingga.
"""
import argparse
import heapq
import math
import os
import sys
from types import SimpleNamespace

import numpy as np

if __package__ in (None, ""):
    # Dijalankan sebagai berkas (python drl/adaptor_demo.py), bukan modul.
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import arena as A                     # noqa: E402
import forklift_planner as FP         # noqa: E402
import konfig                         # noqa: E402
from drl import lingkungan as L       # noqa: E402

konfig.pasang_path_kamera()
import area_constraint as AC          # noqa: E402

TOL_JALUR_SEL = 0.01
TOL_BIAYA = 1e-6


# ----------------------------------------------------------------- perekam
def rencanakan_terekam(planner, omap, awal, tujuan):
    """planner.plan() + g-cost simpul akhir dan ekor Reeds-Shepp-nya.

    Kembalian (jalur, rekaman). rekaman.biaya = g-cost total jalur yang
    dikembalikan; rekaman.n_ekor = banyak pose ekor RS di ujung jalur (0 bila
    tiba lewat primitif).

    g simpul diambil dua cara: untuk ekor RS langsung dari simpulnya (node[5],
    eksak); untuk tiba lewat primitif dari f - h_weight*h saat simpul didorong
    ke heap, dengan h = _h_cari (heuristik yang dipakai heap; galat pembulatan
    ~1e-13, jauh di bawah toleransi uji).
    """
    rek = SimpleNamespace(h=0.0, g={0: 0.0}, pop=None, rs=None)
    h_asli, analitik_asli = planner._h_cari, planner._analytic

    def h(x, y, yaw, goal):
        rek.h = h_asli(x, y, yaw, goal)
        return rek.h

    def analitik(omap_, node, goal):
        r = analitik_asli(omap_, node, goal)
        if r is not None:
            rek.rs = (node[5], len(r[0]), r[1])
        return r

    def dorong(heap, item):
        rek.g[item[1]] = item[0] - planner.h_weight * rek.h
        heapq.heappush(heap, item)

    def ambil(heap):
        item = heapq.heappop(heap)
        rek.pop = item[1]
        return item

    planner._h_cari, planner._analytic = h, analitik
    FP.heapq = SimpleNamespace(heappush=dorong, heappop=ambil)
    try:
        jalur = planner.plan(omap, awal, tujuan)
    finally:
        FP.heapq = heapq
        del planner._h_cari, planner._analytic

    hasil = SimpleNamespace(biaya=None, n_ekor=0, g_sambung=None, stats=dict(planner.last_stats))
    if jalur:
        if rek.rs is not None:
            g, hasil.n_ekor, biaya_rs = rek.rs
            hasil.g_sambung = g
            hasil.biaya = g + biaya_rs
        else:
            hasil.biaya = rek.g[rek.pop]
    return jalur, hasil


def urai_aksi(planner, jalur, n_ekor):
    """Jalur A* + ekor Reeds-Shepp -> (aksi model jauh, pose ekor RS).

    Bagian A* diurai PERSIS menjadi primitif (satu kelompok n_sub pose = satu
    primitif). Ekor RS dikembalikan apa adanya, untuk dibandingkan dengan
    tembakan RS lingkungan. ValueError bila strukturnya tak terduga.
    """
    n = planner.n_sub
    tubuh = jalur[1:len(jalur) - n_ekor]
    if len(tubuh) % n:
        raise ValueError(f"{len(tubuh)} pose primitif bukan kelipatan n_sub={n}")
    aksi = []
    yaw = jalur[0][2]
    for i in range(0, len(tubuh), n):
        kel = tubuh[i:i + n]
        arah = kel[0][3]
        if any(p[3] != arah for p in kel):
            raise ValueError(f"arah berubah di dalam primitif ke-{i // n}")
        kappa = (kel[-1][2] - yaw) / (arah * planner.step)
        a = L.aksi_dari(planner, arah, kappa)
        if abs(L.arah_kappa(planner, a)[1] - kappa) > 1e-6 * planner.kappa_max + 1e-12:
            raise ValueError(f"kappa {kappa:.6g} bukan tingkat kemudi perencana")
        aksi.append(a)
        yaw = kel[-1][2]
    ekor = list(jalur[len(jalur) - n_ekor:]) if n_ekor else []
    return aksi, ekor


def putar_ulang(env, awal, tujuan, aksi, omap=None):
    """Mainkan aksi di lingkungan. Kembalian (episode dict, pesan galat atau None)."""
    obs, info = env.reset(awal, tujuan, omap)
    if obs is None:
        return None, f"reset ditolak: {info['alasan']}"
    ep = {"obs": [obs], "aksi": [], "reward": [], "mask": [env.mask()], "selesai": []}
    for i, a in enumerate(aksi):
        if not env.mask()[a]:
            return None, f"aksi demo ke-{i} ({a}) ter-mask"
        obs, r, selesai, info = env.step(a)
        ep["obs"].append(obs)
        ep["aksi"].append(a)
        ep["reward"].append(r)
        ep["mask"].append(env.mask() if not selesai else np.zeros(env.n_aksi, bool))
        ep["selesai"].append(selesai)
        if selesai and i != len(aksi) - 1:
            return None, f"episode berakhir di aksi ke-{i} dari {len(aksi)} ({info['alasan']})"
    if not env.sukses:
        return None, "aksi habis tanpa serah"
    return {k: np.asarray(v) for k, v in ep.items()}, None


# Kode alasan demo ditolak (demo_jauh -> "tolak"). Semua SAH, bukan galat:
TOLAK = {
    "jauh_margin": "bagian A* menyentuh margin (peta dengan robot lain)",
    "belum_bebas": "RS mulai sebelum peta Dijkstra bebas (RS memutari rintangan)",
    "tanpa_rs": "A* tiba tanpa ekor RS - tidak ada titik serah",
    "rs_di_awal": "A* langsung menembak RS dari pose awal - tidak ada langkah model jauh",
}


def demo_jauh(env, awal, tujuan, aksi_jauh, ekor, omap=None):
    """Satu demo A* + RS -> episode model jauh.

    Model jauh memainkan primitif A*; lingkungannya harus SERAH tepat setelah
    primitif terakhir - titik yang sama dengan saat Hybrid A* beralih ke RS -
    dengan tembakan RS yang identik dengan ekor A*.

    Kembalian dict: ep (episode atau None), tolak (kode TOLAK atau None),
    galat (pesan galat struktur atau None).
    """
    h = {"ep": None, "tolak": None, "galat": None}
    if not ekor:
        h["tolak"] = "tanpa_rs"
        return h
    if not aksi_jauh:
        h["tolak"] = "rs_di_awal"
        return h
    ep, galat = putar_ulang(env, awal, tujuan, aksi_jauh, omap)
    if galat:
        if galat_margin(galat):
            h["tolak"] = "jauh_margin"
        elif galat.startswith("aksi habis tanpa serah"):
            h["tolak"] = "belum_bebas"
        else:
            h["galat"] = galat
        return h
    if selisih_jalur(env.ekor_rs, ekor) > TOL_JALUR_SEL:
        h["galat"] = "tembakan RS lingkungan berbeda dari ekor RS A*"
        return h
    h["ep"] = ep
    return h


def galat_margin(galat):
    """True bila galat adalah aksi yang ter-mask (menyentuh margin), bukan galat struktur."""
    return bool(galat) and galat.startswith("aksi demo ke-")


def selisih_jalur(j1, j2):
    """Selisih terbesar x/y (sel) dan yaw (rad) antara dua jalur, inf bila panjang beda."""
    j1, j2 = list(j1 or []), list(j2 or [])
    if len(j1) != len(j2):
        return float("inf")
    if not j1:
        return 0.0
    a, b = np.asarray(j1, dtype=np.float64), np.asarray(j2, dtype=np.float64)
    return float(np.max(np.abs(a - b)))


# -------------------------------------------------------------- skenario uji
def acak_skenario(env, rng, n, peluang_bebas=0.3, jarak_cm=(40.0, 250.0)):
    """n pasangan (awal, tujuan) dalam sel yang badannya bebas dengan margin penuh."""
    omap, v, arena = env.arena.omap, env.v, env.arena
    lo, hi = arena.panjang_ke_sel(jarak_cm[0]), arena.panjang_ke_sel(jarak_cm[1])

    def pose():
        while True:
            x, y = rng.uniform(0, omap.w), rng.uniform(0, omap.h)
            yaw = rng.uniform(-math.pi, math.pi)
            if omap.pose_free(v, x, y, yaw):
                return x, y, yaw

    out = []
    while len(out) < n:
        a, t = pose(), pose()
        if not lo <= math.hypot(t[0] - a[0], t[1] - a[1]) <= hi:
            continue
        if rng.random() < peluang_bebas:
            t = (t[0], t[1], None)
        out.append((a, t))
    return out


def uji(ids, peta, n, seed):
    """Uji penerimaan model jauh: putar ulang tiap demo A* + RS di lingkungan.

    Lulus bila untuk setiap jalur A*: episode jauh lolos mask, SERAH tepat
    setelah primitif A* terakhir, jalurnya (termasuk ekor RS) IDENTIK dengan
    jalur A*, biayanya = g-cost A*, dan observasinya 47 angka berhingga - atau
    demo ditolak dengan salah satu alasan sah di TOLAK.
    """
    aturan = konfig.ATURAN_RS["demonstrator"]
    print(f"[i] uji penerimaan model jauh: {n} skenario per robot per peta, seed {seed}")
    print(f"[i] demonstrator: w {konfig.PARAM_RENCANA['h_weight']}, h = max(h_RS, h_D), RS di setiap node "
          f"< {aturan['analytic_dist']:.0f} cm; serah = < {aturan['analytic_dist']:.0f} cm + peta Dijkstra bebas "
          f"(atau < {L.RADIUS_SERAH_PAKSA_CM} cm) + tembakan RS bebas; toleransi jalur {TOL_JALUR_SEL} sel")
    ringkas = []
    semua_lulus = True
    for rid in ids:
        prof = konfig.profil(rid)
        for nama in peta:
            arena = A.Arena(nama)
            env = L.LingkunganRencana(arena, prof)
            pl = env.planner
            # Bukan hash(nama): hash string diacak per proses, jadi skenarionya
            # tidak akan sama di setiap jalan.
            rng = np.random.default_rng([seed, rid, *nama.encode()])
            s = SimpleNamespace(n=0, ditemukan=0, lulus=0, maks_jalur=0.0, maks_biaya=0.0, galat=[],
                                n_jauh=[], n_rs=[], tolak={})
            for awal, tujuan in acak_skenario(env, rng, n):
                s.n += 1
                jalur, rek = rencanakan_terekam(pl, arena.omap, awal, tujuan)
                if not jalur:
                    continue
                s.ditemukan += 1
                try:
                    aksi_jauh, ekor = urai_aksi(pl, jalur, rek.n_ekor)
                except ValueError as e:
                    s.galat.append(f"urai: {e}")
                    continue
                h = demo_jauh(env, awal, tujuan, aksi_jauh, ekor)
                if h["galat"]:
                    s.galat.append(h["galat"])
                    continue
                if h["tolak"]:
                    s.tolak[h["tolak"]] = s.tolak.get(h["tolak"], 0) + 1
                    continue
                ep = h["ep"]
                dj = selisih_jalur(jalur, env.jalur())
                db = abs(env.biaya - rek.biaya) / max(1.0, rek.biaya)
                s.n_jauh.append(len(ep["aksi"]))
                s.n_rs.append(env.n_titik_rs)
                obs_ok = ep["obs"].shape[1] == L.N_OBS and np.isfinite(ep["obs"]).all()
                s.maks_jalur = max(s.maks_jalur, dj)
                s.maks_biaya = max(s.maks_biaya, db)
                if dj < TOL_JALUR_SEL and db < TOL_BIAYA and obs_ok:
                    s.lulus += 1
                else:
                    s.galat.append(f"selisih jalur {dj:.3g} sel, selisih biaya {db:.3g}, "
                                   f"obs {'ok' if obs_ok else 'RUSAK'}")
            ok = not s.galat
            semua_lulus &= ok
            ringkas.append((rid, nama, s, ok))
            print(f"    ID {rid:02d} {nama:<9} lulus {s.lulus}/{s.ditemukan} (A* menemukan {s.ditemukan}/{s.n}; "
                  f"ditolak {s.tolak or '-'})  {'OK' if ok else 'GAGAL'}")
            for g in s.galat[:5]:
                print(f"        [!] {g}")

    print()
    print("  robot peta       lulus     aksi jauh (rata)  titik RS (rata)  maks selisih jalur   maks selisih biaya")
    for rid, nama, s, ok in ringkas:
        print(f"  ID {rid:02d} {nama:<9} {s.lulus:>3}/{s.ditemukan:<3}   {np.mean(s.n_jauh or [0]):10.1f}"
              f"   {np.mean(s.n_rs or [0]):12.1f}   {s.maks_jalur:14.2e} sel   {s.maks_biaya:12.2e}")
    print()
    print("[OK] SEMUA LULUS" if semua_lulus else "[!] ADA YANG GAGAL - lihat di atas")
    return semua_lulus


def main():
    ap = argparse.ArgumentParser(description="Adaptor demonstrasi Hybrid A* -> model jauh DQfD")
    ap.add_argument("--uji", action="store_true", help="jalankan uji penerimaan")
    ap.add_argument("--id", default="1,2", help="robot, mis. 1 atau 1,2")
    ap.add_argument("--peta", default=None,
                    help="dipisah koma; bawaan semua peta di Camera/peta_constraint/")
    ap.add_argument("--n", type=int, default=15, help="skenario per robot per peta")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    if not args.uji:
        ap.print_help()
        return
    ids = [int(t) for t in args.id.split(",") if t.strip()]
    peta = ([t.strip() for t in args.peta.split(",") if t.strip()] if args.peta
            else AC.daftar_peta())
    sys.exit(0 if uji(ids, peta, args.n, args.seed) else 1)


if __name__ == "__main__":
    main()
