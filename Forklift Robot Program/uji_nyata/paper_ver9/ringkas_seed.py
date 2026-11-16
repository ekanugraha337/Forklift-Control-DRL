"""
ringkas_seed.py - sebaran antar-seed pelatihan tahap 2 (review pembimbing ver8: "latih minimal 3 seed").
Model ID 01: seed 0 = model paper (model_dqn_gudang3), seed 1 dan 2 = model_dqn_gudang3_seed1/2, argumen
latih lain identik. Data per seed dari hitung_primitif.py (sama dengan Tabel I):
    w_<peta>_s<seed>.json  --tanpa-hitung (waktu)      p_<peta>_s<seed>.json  dengan pencatat (primitif dicek)
Panjang jalur dibandingkan dengan Hybrid A* standar dari b4_<peta>.json (tidak bergantung seed).

    cd "Forklift Robot Program"
    for s in 0 1 2; do M=model_dqn_gudang3; [ $s -gt 0 ] && M=${M}_seed$s
      for P in Gudang3 Gudang4; do
        ../.venv/bin/python uji_nyata/paper_ver9/hitung_primitif.py --peta $P --dir-model $M --jenis dqfd-murni --tanpa-hitung --keluar uji_nyata/paper_ver9/seed/w_${P}_s$s.json
        ../.venv/bin/python uji_nyata/paper_ver9/hitung_primitif.py --peta $P --dir-model $M --jenis dqfd-murni --keluar uji_nyata/paper_ver9/seed/p_${P}_s$s.json
      done; done
    ../.venv/bin/python uji_nyata/paper_ver9/ringkas_seed.py
"""
import json
import os

import numpy as np

S = os.path.dirname(os.path.abspath(__file__))
D = "dqfd-murni"


def muat(f):
    return json.load(open(os.path.join(S, f)))["rute"]


for peta in ("Gudang3", "Gudang4"):
    b = muat(f"b4_{peta}.json")
    LB = np.array([r["astar-standar"]["panjang_cm"] if r["astar-standar"]["ok"] else np.nan for r in b])
    print(f"===== {peta}: {len(b)} rute")
    baris = {}
    for s in (0, 1, 2):
        try:
            w, p = muat(f"seed/w_{peta}_s{s}.json"), muat(f"seed/p_{peta}_s{s}.json")
        except FileNotFoundError:
            continue
        assert [(r["dari"], r["ke"]) for r in w] == [(r["dari"], r["ke"]) for r in b] == [(r["dari"], r["ke"]) for r in p]
        ok = np.array([r[D]["ok"] for r in w])
        t = np.array([r[D]["cpu_ms"] for r in w])
        prim = np.array([r[D]["primitif"] for r in p], dtype=float)
        L = np.array([r[D]["panjang_cm"] if r[D]["ok"] else np.nan for r in w])
        m = ok & np.isfinite(LB)
        q = L[m] / LB[m]
        balik = np.array([r[D]["ganti_arah"] for r in w if r[D]["ok"]], dtype=float)
        mundur = sum("mundur" in str(r[D]["perencana"]) for r in w)
        baris[s] = dict(tiba=100 * ok.mean(), prim_med=np.median(prim), prim_p90=np.percentile(prim, 90),
                        t_med=np.median(t), t_p90=np.percentile(t, 90), t_rata=t.mean(), lewat1s=int((t > 1000).sum()),
                        q_med=np.median(q), q_p90=np.percentile(q, 90), balik=balik.mean(), backtracking=mundur)
        x = baris[s]
        print(f"  seed {s}: tiba {x['tiba']:.1f}% | primitif med {x['prim_med']:.0f} p90 {x['prim_p90']:.0f} | "
              f"waktu med {x['t_med']:.1f} p90 {x['t_p90']:.0f} rata {x['t_rata']:.1f} ms >1s {x['lewat1s']} | "
              f"panjang/A* med {x['q_med']:.3f} p90 {x['q_p90']:.3f} | ganti arah {x['balik']:.2f} | backtracking {x['backtracking']}")
    if len(baris) > 1:
        print("  kisaran antar-seed: " + ", ".join(
            f"{k} {min(v[k] for v in baris.values()):.3g}-{max(v[k] for v in baris.values()):.3g}"
            for k in ("tiba", "prim_med", "prim_p90", "t_rata", "q_med", "q_p90")))
