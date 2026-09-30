"""
adaptor_demo.py - jalur Hybrid A* -> episode demonstrasi (s, a, r, s') untuk DQfD.

Karena lingkungan perencanaan deterministik dan memakai aritmetika A* sendiri,
aksi demonstrator tidak perlu ditebak - ia DIREKONSTRUKSI tepat dari jalur:

    jalur = [awal] + (n_sub pose per primitif) x N + (pose ekor Reeds-Shepp)
    tiap kelompok n_sub pose -> satu primitif: arah dari kolom arah,
                                kappa = delta-yaw / (arah x step), dibulatkan
    ekor Reeds-Shepp          -> satu aksi SAMBUNG

plan() tidak mengembalikan g-cost, jadi biaya A* direkam dari luar selama
plan() berjalan: _h dan _analytic dibungkus di tingkat OBJEK, dan heapq modul
forklift_planner ditukar sementara dengan perekam. forklift_planner.py sendiri
tidak disentuh.

UJI PENERIMAAN (tahap 1-2):
    python -m drl.adaptor_demo --uji                      # ID 01 & 02, tiga peta
    python -m drl.adaptor_demo --uji --id 2 --peta Rumah --n 30

    Lulus bila, untuk SETIAP jalur yang ditemukan A*:
      1. semua aksi demo lolos mask lingkungan, dan episodenya berakhir sukses
         tepat pada aksi terakhir;
      2. jalur putar-ulang = jalur A* (selisih maks < 0,01 sel);
      3. jumlah biaya lingkungan = g-cost A* (selisih < 1e-6);
      4. pure pursuit memberi perintah yang sama pada kedua jalur;
      5. observasi berukuran 47 dan berhingga di setiap langkah.
"""
import argparse
import heapq
import math
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

if __package__ in (None, ""):
    # Dijalankan sebagai berkas (python drl/adaptor_demo.py), bukan modul.
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import arena as A                     # noqa: E402
import forklift_planner as FP         # noqa: E402
import jejak                          # noqa: E402
import konfig                         # noqa: E402
from drl import lingkungan as L       # noqa: E402

PETA_UJI = ("Kalibrasi", "Rumah2", "Rumah")
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
    ke heap (galat pembulatan ~1e-13, jauh di bawah toleransi uji).
    """
    rek = SimpleNamespace(h=0.0, g={0: 0.0}, pop=None, rs=None)
    h_asli, analitik_asli = planner._h, planner._analytic

    def h(x, y, gx, gy):
        rek.h = h_asli(x, y, gx, gy)
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

    planner._h, planner._analytic = h, analitik
    FP.heapq = SimpleNamespace(heappush=dorong, heappop=ambil)
    try:
        jalur = planner.plan(omap, awal, tujuan)
    finally:
        FP.heapq = heapq
        del planner._h, planner._analytic

    hasil = SimpleNamespace(biaya=None, n_ekor=0, stats=dict(planner.last_stats))
    if jalur:
        if rek.rs is not None:
            g, hasil.n_ekor, biaya_rs = rek.rs
            hasil.biaya = g + biaya_rs
        else:
            hasil.biaya = rek.g[rek.pop]
    return jalur, hasil


def urai_aksi(planner, jalur, n_ekor):
    """Jalur A* -> daftar aksi 0..14. Gagal (ValueError) bila strukturnya tak terduga."""
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
    if n_ekor:
        aksi.append(L.SAMBUNG)
    return aksi


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
        ep["mask"].append(env.mask() if not selesai else np.zeros(L.N_AKSI, bool))
        ep["selesai"].append(selesai)
        if selesai and i != len(aksi) - 1:
            return None, f"episode berakhir di aksi ke-{i} dari {len(aksi)} ({info['alasan']})"
    if not env.sukses:
        return None, "aksi habis tanpa tiba di tujuan"
    return {k: np.asarray(v) for k, v in ep.items()}, None


def selisih_jalur(j1, j2):
    """Selisih terbesar x/y (sel) dan yaw (rad) antara dua jalur, inf bila panjang beda."""
    if len(j1) != len(j2):
        return float("inf")
    if not j1:
        return 0.0
    a, b = np.asarray(j1, dtype=np.float64), np.asarray(j2, dtype=np.float64)
    return float(np.max(np.abs(a - b)))


def bandingkan_pelacak(arena, env, j_astar, j_ulang, langkah=6):
    """Umpankan pose yang sama ke pure pursuit pada kedua jalur. True = perintah identik."""
    t1 = jejak.buat("pursuit", j_astar, env.v, env.omap, arena)
    t2 = jejak.buat("pursuit", j_ulang, env.v, env.omap, arena)
    now = 0.0
    kolom = ("speed", "steer_deg", "dir", "seg", "done", "paused")
    for p in j_astar[::langkah]:
        now += konfig.RC_PERIODE_S
        c1, c2 = t1.update(p[0], p[1], p[2], now), t2.update(p[0], p[1], p[2], now)
        if any(c1.get(k) != c2.get(k) for k in kolom):
            return False
    return True


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
    print(f"[i] uji penerimaan adaptor + lingkungan: {n} skenario per robot per peta, seed {seed}")
    print(f"[i] toleransi: jalur < {TOL_JALUR_SEL} sel, biaya < {TOL_BIAYA:g}")
    ringkas = []
    semua_lulus = True
    for rid in ids:
        prof = konfig.profil(rid)
        for nama in peta:
            arena = A.Arena(nama)
            env = L.LingkunganRencana(arena, prof)
            # Bukan hash(nama): hash string diacak per proses, jadi skenarionya
            # tidak akan sama di setiap jalan.
            rng = np.random.default_rng([seed, rid, *nama.encode()])
            s = SimpleNamespace(n=0, ditemukan=0, lulus=0, maks_jalur=0.0, maks_biaya=0.0,
                                langkah=[], sambung=0, tiba=0, galat=[],
                                t_astar=[], t_ulang=[], t_step=[])
            for awal, tujuan in acak_skenario(env, rng, n):
                s.n += 1
                t0 = time.perf_counter()
                jalur, rek = rencanakan_terekam(env.planner, arena.omap, awal, tujuan)
                s.t_astar.append(time.perf_counter() - t0)
                if not jalur:
                    continue
                s.ditemukan += 1
                try:
                    aksi = urai_aksi(env.planner, jalur, rek.n_ekor)
                except ValueError as e:
                    s.galat.append(f"urai: {e}")
                    continue
                t0 = time.perf_counter()
                ep, galat = putar_ulang(env, awal, tujuan, aksi)
                dt = time.perf_counter() - t0
                if galat:
                    s.galat.append(galat)
                    continue
                s.t_ulang.append(dt)
                s.t_step.append(dt / len(aksi))

                dj = selisih_jalur(jalur, env.jalur())
                db = abs(env.biaya - rek.biaya)
                obs_ok = ep["obs"].shape[1] == L.N_OBS and np.isfinite(ep["obs"]).all()
                pp_ok = bandingkan_pelacak(arena, env, jalur, env.jalur())
                s.maks_jalur = max(s.maks_jalur, dj)
                s.maks_biaya = max(s.maks_biaya, db)
                s.langkah.append(len(aksi))
                s.sambung += aksi[-1] == L.SAMBUNG
                s.tiba += aksi[-1] != L.SAMBUNG
                if dj < TOL_JALUR_SEL and db < TOL_BIAYA and obs_ok and pp_ok:
                    s.lulus += 1
                else:
                    s.galat.append(f"selisih jalur {dj:.3g}, biaya {db:.3g}, "
                                   f"obs {'ok' if obs_ok else 'RUSAK'}, "
                                   f"pelacak {'sama' if pp_ok else 'BEDA'}")
            ok = s.lulus == s.ditemukan and not s.galat
            semua_lulus &= ok
            ringkas.append((rid, nama, s, ok))
            print(f"    ID {rid:02d} {nama:<9} lulus {s.lulus}/{s.ditemukan} "
                  f"(A* menemukan {s.ditemukan}/{s.n})  {'OK' if ok else 'GAGAL'}")
            for g in s.galat[:5]:
                print(f"        [!] {g}")

    print()
    print("  robot peta       lulus  A*   aksi rata/maks  akhir SAMBUNG/tiba  "
          "maks selisih jalur  maks selisih biaya  A* rata  ulang rata  per langkah")
    for rid, nama, s, ok in ringkas:
        lk = s.langkah or [0]
        print(f"  ID {rid:02d} {nama:<9} {s.lulus:>3}/{s.ditemukan:<3} {s.ditemukan:>2}/{s.n:<2} "
              f"{np.mean(lk):6.1f} / {max(lk):<4}  {s.sambung:>6} / {s.tiba:<6}      "
              f"{s.maks_jalur:10.2e} sel      {s.maks_biaya:10.2e}      "
              f"{1000 * np.mean(s.t_astar):6.0f} ms "
              f"{1000 * np.mean(s.t_ulang or [0]):7.1f} ms {1000 * np.mean(s.t_step or [0]):7.2f} ms")
    print()
    print("[OK] SEMUA LULUS" if semua_lulus else "[!] ADA YANG GAGAL - lihat di atas")
    return semua_lulus


def main():
    ap = argparse.ArgumentParser(description="Adaptor demonstrasi Hybrid A* -> DQfD")
    ap.add_argument("--uji", action="store_true", help="jalankan uji penerimaan tahap 1-2")
    ap.add_argument("--id", default="1,2", help="robot, mis. 1 atau 1,2")
    ap.add_argument("--peta", default=",".join(PETA_UJI))
    ap.add_argument("--n", type=int, default=15, help="skenario per robot per peta")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    if not args.uji:
        ap.print_help()
        return
    ids = [int(t) for t in args.id.split(",") if t.strip()]
    peta = [t.strip() for t in args.peta.split(",") if t.strip()]
    sys.exit(0 if uji(ids, peta, args.n, args.seed) else 1)


if __name__ == "__main__":
    main()
