"""
banding_baseline.py - Hybrid A* + RS standar vs DQfD (model jauh + tembakan RS), satu run offline.

    python -m drl.banding_baseline                       kedua perencana, semua rute uji, ID 01 & 02
    python -m drl.banding_baseline --varian astar --maks 20      uji singkat

Rute: semua episode UJI demo_jauh_id0N (peta asli dengan rute yang belum
dilihat, dan peta sintetis yang tidak pernah dilihat saat latih), lewat
Perencana.rencanakan() - jalur kode yang sama dengan robot (sapuan heading
bebas, margin cadangan), batas waktu 2 s per percobaan untuk kedua perencana.

Varian (aturan ver-1-6, w = 2,0 untuk keduanya):
    astar  Hybrid A* + RS standar: h = max(h_RS, h_D), RS setiap ekspansi ke-4
    dqfd   DQfD murni: model jauh (terbaik-dulu + backtracking) + tembakan RS
           di dalam 60 cm bila tidak ada halangan, tanpa cadangan Hybrid A*

Per rute dicatat: sukses, ms (median ULANG ulangan), node (jumlah ekspansi /
langkah DQfD SEMUA panggilan perencana di rencanakan() terakhir, termasuk
sapuan heading bebas), biaya (Eq. 2 dihitung ulang dari jalur dengan SATU
fungsi untuk semua varian), ganti arah (seluruh / di luar 60 cm), galat akhir,
panjang, dan geometri awal (jarak, sudut heading awal terhadap arah ke tujuan).

Keluaran: model_dqn/banding_baseline_<tanggal>.json
"""
import argparse
import datetime
import json
import math
import os
import sys
import time

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                                   # noqa: E402
import rencana as R                             # noqa: E402
from drl import rollout as RO                   # noqa: E402
from drl import skenario as S                   # noqa: E402
from drl.perencana_dqfd import hitung_ganti_arah  # noqa: E402

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ULANG = 3

VARIAN = {
    "astar": "astar",
    "dqfd": "dqfd-murni",
}


def daftar_peta(rid):
    d = RO.muat_npz(os.path.join(AKAR, "drl", "data", f"demo_jauh_id{rid:02d}.npz"))
    return sorted({str(d["ep_peta"][e]) for e in np.flatnonzero(d["ep_uji"])})


def rute_uji(rid, peta):
    d = RO.muat_npz(os.path.join(AKAR, "drl", "data", f"demo_jauh_id{rid:02d}.npz"))
    ar = S._arena(peta)
    out = []
    for e in np.flatnonzero(d["ep_uji"]):
        if str(d["ep_peta"][e]) != peta:
            continue
        a = ar.pose_ke_cm(*d["ep_awal"][e])
        t = d["ep_tujuan"][e]
        tc = ar.pose_ke_cm(t[0], t[1], 0.0 if np.isnan(t[2]) else t[2])
        poli = d["ep_rintangan"][e]
        out.append({"e": int(e), "awal": tuple(float(v) for v in a),
                    "tujuan": (float(tc[0]), float(tc[1]), None if np.isnan(t[2]) else float(tc[2])),
                    "rintangan": [] if np.isnan(poli).any() else [poli]})
    return ar, out


def biaya_jalur(pl, omap, kendaraan, jalur):
    """Biaya Eq. (2) dari daftar pose (x, y, yaw, arah) dalam sel - satu rumus untuk semua planner.

    Per pasangan titik berurutan: panjang x (1 maju / reverse_cost mundur), kelengkungan
    (dari selisih yaw, dibulatkan ke tingkat kemudi planner), ganti kelengkungan, ganti
    arah (switch_cost), dan penalti clearance per panjang seperti _clearance_penalty."""
    if not jalur or len(jalur) < 2:
        return 0.0
    km = pl.kappa_max
    tingkat = list(pl.kappas)
    biaya, d_prev, k_prev = 0.0, 0, 0.0
    for a, b in zip(jalur[:-1], jalur[1:]):
        lng = math.hypot(b[0] - a[0], b[1] - a[1])
        if lng < 1e-9:
            continue
        d = 1 if b[3] > 0 else -1
        k_est = math.remainder(b[2] - a[2], 2.0 * math.pi) / (lng * d)
        k = min(tingkat, key=lambda q: abs(q - k_est))
        biaya += lng * (1.0 if d > 0 else pl.reverse_cost)
        biaya += pl.steer_cost * lng * abs(k) / km
        biaya += pl.steer_change_cost * abs(k - k_prev) / km
        if d_prev != 0 and d != d_prev:
            biaya += pl.switch_cost
        m = omap.pose_margin(kendaraan, b[0], b[1], b[2])
        biaya += pl.clearance_cost * max(0.0, pl.clearance_pref - m) * lng / pl.step
        d_prev, k_prev = d, k
    return biaya


class PencatatNode:
    """Jumlah ekspansi (A*) / langkah (DQfD) semua panggilan planner dalam satu rencanakan()."""

    def __init__(self, per):
        self.total = 0
        asli = per._plan

        def _plan(omap, awal, tujuan):
            jalur = asli(omap, awal, tujuan)
            self.total += int((per.planner.last_stats or {}).get("expansions", 0) or 0)
            return jalur
        per._plan = _plan


def jalankan(args):
    hasil = []
    t_mulai = time.time()
    varian = [v.strip() for v in args.varian.split(",")]
    for rid in [int(x) for x in args.id.split(",")]:
        prof = konfig.profil(rid)
        for peta in daftar_peta(rid):
            ar, rute = rute_uji(rid, peta)
            if args.maks:
                rute = rute[:args.maks]
            # Planner per varian dibuat sekali per peta (seperti robot: satu planner per arena)
            per = {}
            for v in varian:
                p = R.Perencana(ar, prof, jenis=VARIAN[v])
                p._untuk(konfig.SAFETY_MARGIN_CM)
                p.rencanakan(rute[0]["awal"], rute[0]["tujuan"], rute[0]["rintangan"])   # pemanasan
                per[v] = (p, PencatatNode(p))
            # Parameter biaya Eq. (2) untuk SEMUA varian (sama untuk setiap peran Hybrid A*)
            pl_biaya = R.buat_perencana(ar, R.buat_kendaraan(ar, konfig.SAFETY_MARGIN_CM,
                                                             prof.max_steer_rad), "astar")
            for r in rute:
                ax, ay, ayaw = r["awal"]
                gx, gy = r["tujuan"][0], r["tujuan"][1]
                arah = math.degrees(math.atan2(gy - ay, gx - ax))
                geo = {"jarak_cm": math.hypot(gx - ax, gy - ay),
                       "sudut_awal_deg": abs(math.remainder(arah - ayaw, 360.0)),
                       "heading_bebas": r["tujuan"][2] is None}
                for v in varian:
                    p, cat = per[v]
                    jam = []
                    for _ in range(args.ulang):
                        cat.total = 0
                        t0 = time.perf_counter()
                        jalur, info = p.rencanakan(r["awal"], r["tujuan"], r["rintangan"])
                        jam.append(1000.0 * (time.perf_counter() - t0))
                    baris = {"robot": rid, "peta": peta, "e": r["e"], "varian": v, **geo,
                             "sukses": bool(jalur), "ms": float(np.median(jam)), "ms_semua": jam,
                             "node": cat.total, "oleh": info.get("perencana", "") if jalur else "gagal",
                             "alasan": info.get("alasan", "") if not jalur else "",
                             "margin_cm": info.get("margin_cm")}
                    if jalur:
                        omap = p.peta_dengan(r["rintangan"])
                        kend = info["kendaraan"]
                        if r["tujuan"][2] is not None:
                            tj = ar.pose_ke_sel(gx, gy, r["tujuan"][2])
                        else:
                            tj = (*ar.pose_ke_sel(gx, gy, 0.0)[:2], jalur[-1][2])
                        g_all, g_dekat = hitung_ganti_arah(jalur, tj, ar.panjang_ke_sel(60.0))
                        x, y, yaw = jalur[-1][:3]
                        baris.update(
                            biaya=biaya_jalur(pl_biaya, omap, kend, jalur),
                            panjang_cm=float(info.get("panjang_cm", 0.0) or 0.0),
                            ganti_arah=g_all, ganti_arah_luar60=g_all - g_dekat,
                            galat_pos_cm=ar.panjang_ke_cm(math.hypot(tj[0] - x, tj[1] - y)),
                            galat_yaw_deg=(abs(math.degrees(math.remainder(yaw - tj[2], 2.0 * math.pi)))
                                           if r["tujuan"][2] is not None else 0.0))
                    hasil.append(baris)
            print(f"ID{rid:02d} {peta:<16} {len(rute):3d} rute x {len(varian)} varian selesai "
                  f"({time.time() - t_mulai:.0f} s)", flush=True)
    return hasil


def ringkas(hasil, varian):
    print("\n=== Ringkasan (median / p90 ms, median node, tiba)")
    for rid in sorted({h["robot"] for h in hasil}):
        for v in varian:
            xs = [h for h in hasil if h["robot"] == rid and h["varian"] == v]
            if not xs:
                continue
            ms = np.array([h["ms"] for h in xs])
            nd = np.array([h["node"] for h in xs])
            print(f"ID{rid:02d} {v}: tiba {100 * np.mean([h['sukses'] for h in xs]):5.1f}%  "
                  f"ms {np.median(ms):6.1f} / {np.percentile(ms, 90):6.0f}  node {np.median(nd):5.0f}  "
                  f"(n={len(xs)})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--varian", default=",".join(VARIAN))
    ap.add_argument("--ulang", type=int, default=ULANG)
    ap.add_argument("--maks", type=int, default=0, help="maks rute per peta (0 = semua)")
    ap.add_argument("--keluar", default=None)
    args = ap.parse_args()
    hasil = jalankan(args)
    keluar = args.keluar or os.path.join(
        AKAR, "model_dqn", f"banding_baseline_{datetime.date.today().isoformat()}.json")
    meta = {"tanggal": datetime.datetime.now().isoformat(timespec="seconds"), "ulang": args.ulang,
            "varian": {v: VARIAN[v] for v in args.varian.split(",")},
            "python": sys.version.split()[0], "platform": sys.platform}
    try:
        import platform
        meta["cpu"] = platform.processor()
    except Exception:
        pass
    with open(keluar, "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "rute": hasil}, f,
                  default=lambda o: o.item() if isinstance(o, np.generic) else str(o))
    ringkas(hasil, args.varian.split(","))
    print(f"[OK] {keluar}")


if __name__ == "__main__":
    main()
