"""
model_di_lab.py - model mana yang dipakai di robot nyata (peta lab)? Model lab (tahap 1, model_dqn) vs model
akhir (tahap 2, model_dqn_gudang3), DQfD tanpa P_col, pada rute uji dataset kelima peta lab, kedua robot,
dibandingkan dengan Hybrid A* standar: tiba, waktu, rasio panjang, dan jalur yang jauh lebih panjang.

    cd "Forklift Robot Program" && ../.venv/bin/python uji_nyata/paper_ver9/model_di_lab.py
"""
import json
import os
import sys

os.environ["FORKLIFT_TANPA_P"] = "1"
sys.path.insert(0, os.getcwd())
import numpy as np                              # noqa: E402
import konfig                                   # noqa: E402
import rencana as R                             # noqa: E402
from drl import perencana_dqfd as PD            # noqa: E402
from drl import banding_baseline as BB          # noqa: E402

MODEL = {"lab": os.path.abspath("model_dqn"), "akhir": os.path.abspath("model_dqn_gudang3")}
hasil = []
for rid in (1, 2):
    konfig.pilih_robot(rid)
    for peta in [p for p in BB.daftar_peta(rid) if not p.startswith("sintetis")]:
        ar, rute = BB.rute_uji(rid, peta)
        per = {"astar-standar": R.Perencana(ar, konfig.profil(rid), jenis="astar-standar")}
        for m, d in MODEL.items():
            PD.DIR_MODEL = d
            per[m] = R.Perencana(ar, konfig.profil(rid), jenis="dqfd-murni")
        for j, p in per.items():
            if j in MODEL:
                PD.DIR_MODEL = MODEL[j]
            p.rencanakan(rute[0]["awal"], rute[0]["tujuan"], rute[0]["rintangan"] or None, None)
        for r in rute:
            b = dict(rid=rid, peta=peta)
            for j, p in per.items():
                if j in MODEL:
                    PD.DIR_MODEL = MODEL[j]
                jl, info = p.rencanakan(r["awal"], r["tujuan"], r["rintangan"] or None, None)
                b[j] = dict(ok=bool(jl), ms=info["cpu_s"] * 1000.0, panjang=info.get("panjang_cm"),
                            perencana=info.get("perencana"))
            hasil.append(b)
        print(f"ID {rid:02d} {peta}: {len(rute)} rute", flush=True)
json.dump(hasil, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "model_di_lab.json"), "w"))


def ringkas(rows, judul):
    print(f"== {judul}: {len(rows)} rute")
    okB = np.array([b["astar-standar"]["ok"] for b in rows])
    for j in ("astar-standar", "lab", "akhir"):
        ok = np.array([b[j]["ok"] for b in rows]); t = np.array([b[j]["ms"] for b in rows])
        m = ok & okB
        q = np.array([b[j]["panjang"] / b["astar-standar"]["panjang"] for b, k in zip(rows, m) if k])
        mb = sum("mundur" in str(b[j]["perencana"]) for b in rows)
        print(f"   {j:<14} tiba {100 * ok.mean():5.1f}%  waktu med {np.median(t):5.1f} p90 {np.percentile(t, 90):6.1f} "
              f"maks {t.max():6.0f} ms | panjang/A* med {np.median(q):.2f} p90 {np.percentile(q, 90):.2f} "
              f">1.5x {int((q > 1.5).sum())} | backtracking {mb}")


ringkas(hasil, "semua peta lab")
for peta in sorted({b["peta"] for b in hasil}):
    ringkas([b for b in hasil if b["peta"] == peta], peta)
