"""Ringkas run uji_robot_nyata (sim/nyata) per perencana: makespan, rencana pertama,
replan, galat akhir, jarak badan minimum, dan mundur di tengah rute.

    cd "Forklift Robot Program" && python drl/ringkas_uji_varian.py uji_nyata/<run> [uji_nyata/<run> ...]
"""
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.getcwd())
import arena as A        # noqa: E402
import wasit as W        # noqa: E402

URUT = ("astar", "astar-rs", "dqfd-murni")
NAMA = {"astar": "A0 Hybrid A*", "astar-rs": "A3 Hybrid A* RS w2", "dqfd-murni": "D0 DQfD"}


def mundur_tengah(jalur_cm, tujuan, r=60.0):
    n = 0
    for a, b in zip(jalur_cm[:-1], jalur_cm[1:]):
        if a[3] != b[3] and math.hypot(tujuan[0] - a[0], tujuan[1] - a[1]) >= r:
            n += 1
    return n


def jarak_min(dir_run):
    """{(skenario, ulang, perencana): jarak badan terkecil (cm) selama fase UJI}."""
    jalur = os.path.join(dir_run, "lintasan.csv")
    if not os.path.exists(jalur):
        return {}
    frame = {}
    with open(jalur, encoding="utf-8") as f:
        for b in csv.DictReader(f):
            if b.get("fase") != "UJI":
                continue
            try:
                x, y, yaw = float(b["x_cm"]), float(b["y_cm"]), float(b["yaw_deg"])
            except (TypeError, ValueError):
                continue
            k = (b["skenario"], int(b["ulang"]), b["perencana"])
            frame.setdefault(k, {}).setdefault(b["t"], {})[b["robot"]] = (x, y, yaw)
    out = {}
    for k, fr in frame.items():
        dmin = float("inf")
        for pose in fr.values():
            if len(pose) == 2:
                p1, p2 = list(pose.values())
                dmin = min(dmin, W.jarak_poligon(A.jejak_badan(*p1), A.jejak_badan(*p2)))
        out[k] = dmin
    return out


def ringkas(dir_run):
    hasil = [json.loads(b) for b in open(os.path.join(dir_run, "hasil.jsonl"), encoding="utf-8") if b.strip()]
    hasil = [h for h in hasil if h.get("status") != "dilewati"]
    jm = jarak_min(dir_run)
    print(f"\n=== {os.path.basename(dir_run)} ({len(hasil)} percobaan)")
    out = {}
    for p in URUT:
        hs = [h for h in hasil if h["perencana"] == p]
        if not hs:
            continue
        mk = [h["makespan_s"] for h in hs if h.get("makespan_s") is not None]
        rr = [r for h in hs for r in h["robot"].values()]
        first = [r["rencana"][0] for r in rr if r.get("rencana")]
        ms1 = [c["ms"] for c in first]
        # mundur di tengah rute pada rencana pertama, tanpa S2 (berbalik di lajur memang wajib)
        first_non_s2 = [r["rencana"][0] for h in hs if h["skenario"] != "S2"
                        for r in h["robot"].values() if r.get("rencana")]
        mt = sum(mundur_tengah(c["jalur_cm"], c["tujuan"]) > 0 for c in first_non_s2 if c.get("jalur_cm"))
        first = first_non_s2
        replan = sum(max(0, len(r.get("rencana", [])) - 1) for r in rr)
        gagal = sum(1 for r in rr for c in r.get("rencana", [])[1:] if not c.get("ok", True))
        gc = [r["galat_cm"] for r in rr if r.get("galat_cm") is not None]
        gd = [r["galat_deg"] for r in rr if r.get("galat_deg") is not None]
        tunggu = [r.get("t_menunggu_s") or 0.0 for r in rr]
        estop = sum(1 for r in rr if r.get("estop"))
        tiba = sum(1 for h in hs if h.get("semua_tiba"))
        gap = [jm[(h["skenario"], h["ulang"], p)] for h in hs if (h["skenario"], h["ulang"], p) in jm]
        per_sk = {}
        for h in hs:
            per_sk.setdefault(h["skenario"], []).append(h["makespan_s"])
        s = {"n": len(hs), "tiba": tiba, "estop": estop, "makespan_mean": float(np.mean(mk)),
             "makespan_sd": float(np.std(mk, ddof=1)) if len(mk) > 1 else 0.0, "makespan_sum": float(np.sum(mk)),
             "rencana1_ms": float(np.mean(ms1)), "replan": replan, "replan_gagal": gagal,
             "galat_cm": float(np.mean(gc)) if gc else None, "galat_deg": float(np.mean(gd)) if gd else None,
             "gap_min": float(min(gap)) if gap else None, "gap_mean": float(np.mean(gap)) if gap else None,
             "tunggu_s": float(np.mean(tunggu)), "rencana1_mundur_tengah": f"{mt}/{len(first)}",
             "per_skenario": {k: float(np.mean(v)) for k, v in sorted(per_sk.items())}}
        out[p] = s
        print(f"{NAMA[p]:<20} n {s['n']:2d} tiba {tiba}/{len(hs)} estop {estop}  makespan {s['makespan_mean']:5.1f} ± "
              f"{s['makespan_sd']:4.1f} s (jumlah {s['makespan_sum']:6.1f})  rencana-1 {s['rencana1_ms']:5.1f} ms  "
              f"replan {replan} ({gagal})  galat {s['galat_cm'] or 0:.1f} cm/{s['galat_deg'] or 0:.1f} deg  "
              f"gap min {s['gap_min'] if s['gap_min'] is not None else float('nan'):.1f} cm  tunggu {s['tunggu_s']:.1f} s  "
              f"mundur-tengah {s['rencana1_mundur_tengah']}")
        print(" " * 20 + " per skenario: " + "  ".join(f"{k} {v:.1f}" for k, v in s["per_skenario"].items()))
    return out


if __name__ == "__main__":
    semua = {os.path.basename(d.rstrip("/")): ringkas(d) for d in sys.argv[1:]}
    keluar = os.environ.get("KELUAR")
    if keluar:
        json.dump(semua, open(keluar, "w", encoding="utf-8"), indent=1)
        print(f"[OK] {keluar}")
