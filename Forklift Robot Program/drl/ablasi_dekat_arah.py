"""
Kapan aturan demonstrator (Hybrid A* + RS) gagal mendarat, padahal model dekat berhasil?

Pelengkap drl.ablasi_dekat. Pose dan penilaiannya SAMA (pose_serah, nilai_pose):
setiap pose pendaratan dinilai dengan urutan Q model dekat dan aturan
demonstrator (<= 6 kandidat termurah). Di sini setiap pose juga dicatat
geometrinya, lalu kegagalan demonstrator dikelompokkan menurut:

  beda_hadap   |arah hadap pose - arah hadap tujuan| [deg]   (None bila tujuan tanpa arah)
  sudut_tuju   |arah ke titik tujuan - arah hadap pose| [deg]
  jarak        jarak titik acuan ke tujuan [cm]
  n_sah/bebas  jumlah kata sah (mask) dan yang bebas tabrakan

Hanya pose yang punya setidaknya satu kata bebas yang dihitung.
Hasil (ver-1-4, 2026-11-03): demonstrator gagal di 5/378 (ID 01) dan 5/357
(ID 02) pose serah; semua pose bertujuan-berarah di antaranya punya beda hadap
> 90 deg, dengan median 2 kata bebas dari 14-15 kata sah. Rute pendek: 0 gagal.

    python -m drl.ablasi_dekat_arah --id 1,2   (hasil: model_dqn/ablasi_dekat_arah.json)
"""
import argparse
import json
import math
import os
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import konfig  # noqa: E402
from drl import lingkungan as L  # noqa: E402
from drl import rollout as RO  # noqa: E402
from drl import skenario as S  # noqa: E402
from drl.ablasi_dekat import nilai_pose, pose_serah  # noqa: E402
from drl.dqfd import muat_jaringan, periksa_profil  # noqa: E402
from drl.perencana_dqfd import jalur_model  # noqa: E402

KELAS_BEDA = ((0, 45), (45, 90), (90, 135), (135, 181))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def baris_pose(env, sk, pose, net_d, rng):
    """Satu pose -> dict geometri + hasil Q dan demonstrator, atau None bila tanpa kata bebas."""
    r = nilai_pose(env, net_d, rng)
    if r["n_bebas"] == 0:
        return None
    x, y, yaw = (float(v) for v in pose[:3])
    gx, gy, gyaw = sk.tujuan
    return {
        "peta": sk.peta,
        "pose": [x, y, yaw],
        "tujuan": [float(gx), float(gy), None if gyaw is None else float(gyaw)],
        "beda_hadap": None if gyaw is None else abs(math.degrees(_wrap(yaw - gyaw))),
        "sudut_tuju": abs(math.degrees(_wrap(math.atan2(gy - y, gx - x) - yaw))),
        "jarak": math.hypot(gx - x, gy - y),
        "n_sah": r["n_sah"],
        "n_bebas": r["n_bebas"],
        "q_tiba": r["q"]["aksi"] is not None,
        "demo_tiba": r["demo"]["aksi"] is not None,
        "q_biaya": r["q"]["biaya"],
        "demo_biaya": r["demo"]["biaya"],
    }


def _stat(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return {"n": 0}
    a = np.asarray(xs, float)
    return {"n": len(a), "median": float(np.median(a)),
            "p25": float(np.percentile(a, 25)), "p75": float(np.percentile(a, 75))}


def ringkas(rows):
    gagal = [r for r in rows if r["q_tiba"] and not r["demo_tiba"]]
    tiba = [r for r in rows if r["demo_tiba"]]
    out = {"n_pose_bebas": len(rows), "n_q_tiba": sum(r["q_tiba"] for r in rows),
           "n_demo_tiba": len(tiba), "n_q_ya_demo_tidak": len(gagal)}
    for nama, g in (("demo_gagal", gagal), ("demo_tiba", tiba)):
        out[nama] = {k: _stat([r[k] for r in g]) for k in ("beda_hadap", "sudut_tuju", "jarak", "n_bebas", "n_sah")}
    out["per_beda_hadap"] = []
    for lo, hi in KELAS_BEDA:
        g = [r for r in rows if r["beda_hadap"] is not None and lo <= r["beda_hadap"] < hi]
        out["per_beda_hadap"].append({
            "kelas": [lo, min(hi, 180)], "n": len(g),
            "demo_gagal_pct": 100.0 * sum(not r["demo_tiba"] for r in g) / len(g) if g else None,
            "q_gagal_pct": 100.0 * sum(not r["q_tiba"] for r in g) / len(g) if g else None})
    return out


def cetak(label, r):
    print(f"  [{label}] pose ber-kata-bebas {r['n_pose_bebas']}, Q tiba {r['n_q_tiba']}, "
          f"demo tiba {r['n_demo_tiba']}, Q ya / demo tidak {r['n_q_ya_demo_tidak']}", flush=True)
    for nama in ("demo_gagal", "demo_tiba"):
        s = r[nama]
        teks = "  ".join(f"{k} {s[k]['median']:.1f}" if s[k]["n"] else f"{k} -"
                         for k in ("beda_hadap", "sudut_tuju", "jarak", "n_bebas", "n_sah"))
        print(f"    {nama:<10} median: {teks}", flush=True)
    for k in r["per_beda_hadap"]:
        if k["n"]:
            print(f"    beda hadap {k['kelas'][0]:3d}-{k['kelas'][1]:3d} deg: n {k['n']:4d}  "
                  f"demo gagal {k['demo_gagal_pct']:5.1f}%  Q gagal {k['q_gagal_pct']:4.1f}%", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--keluar", default=os.path.join(AKAR, "model_dqn"))
    ap.add_argument("--batas-rute", type=float, default=2.0,
                    help="batas waktu rencanakan_dua (s), seperti di robot dan drl.ablasi_dekat")
    args = ap.parse_args()
    os.makedirs(args.keluar, exist_ok=True)
    semua = {}
    for rid in [int(t) for t in args.id.split(",")]:
        prof = konfig.profil(rid)
        jj, jd = jalur_model(rid, "jauh"), jalur_model(rid, "dekat")
        net_j, meta_j = muat_jaringan(jj, jenis="jauh")
        net_d, meta_d = muat_jaringan(jd, jenis="dekat")
        periksa_profil(meta_j, prof, jj)
        periksa_profil(meta_d, prof, jd)
        pabrik = RO.PabrikEnv(prof, "jauh")
        rng = np.random.default_rng(rid)

        d = RO.muat_npz(os.path.join(S.DIR_DATA, f"demo_jauh_id{rid:02d}.npz"))
        uji_penuh = [RO.dari_demo(d, e) for e in np.flatnonzero(d["ep_uji"])]
        d = RO.muat_npz(os.path.join(S.DIR_DATA, f"demo_dekat_tambahan_id{rid:02d}.npz"))
        uji_dekat = [RO.dari_demo(d, e) for e in np.flatnonzero(d["ep_uji"])]
        print(f"=== ID {rid:02d}: {len(uji_penuh)} rute penuh, {len(uji_dekat)} rute dekat", flush=True)

        sumber = {"serah": [(sk, p, dd, kk) for sk, p, dd, kk, _ in
                            pose_serah(rid, prof, net_j, net_d, uji_penuh, pabrik, args.batas_rute)],
                  "dekat": [(sk, sk.awal, sk.d0, sk.k0) for sk in uji_dekat]}
        hasil_robot = {}
        for nama, daftar in sumber.items():
            t0 = time.time()
            rows = []
            for sk, pose, d0, k0 in daftar:
                env = L.LingkunganRencana(pabrik.arena(sk.peta), prof, margin_cm=sk.margin, jenis="dekat")
                obs, _ = env.reset(tuple(pose), sk.tujuan, pabrik.omap(sk), d0=d0, k0=k0)
                if obs is None:
                    continue
                b = baris_pose(env, sk, pose, net_d, rng)
                if b is not None:
                    rows.append(b)
            r = ringkas(rows)
            r["waktu_s"] = time.time() - t0
            r["pose_demo_gagal"] = [b for b in rows if b["q_tiba"] and not b["demo_tiba"]]
            hasil_robot[nama] = r
            cetak(nama, r)
        semua[f"id{rid:02d}"] = hasil_robot
    keluar = os.path.join(args.keluar, "ablasi_dekat_arah.json")
    with open(keluar, "w") as f:
        json.dump(semua, f, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print("->", keluar)


if __name__ == "__main__":
    main()
