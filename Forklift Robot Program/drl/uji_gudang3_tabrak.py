"""
uji_gudang3_tabrak.py - prediksi tabrakan di Gudang3: apakah Q tahu aksi mana yang menabrak,
dan seberapa baik jaringan tabrakan (P_col) memprediksinya, pada keadaan yang DIBUKA pencarian
DQfD di rute operasional (drl/uji_gudang3.py). Juga: primitif yang disimulasi per node.

    FORKLIFT_DIR_MODEL=model_dqn_gudang3 python -m drl.uji_gudang3_tabrak --id 1

Label = simulasi lengkap ke-14 primitif di keadaan itu (kelayakan.label_keadaan), aturan yang
sama dengan lingkungan._tabrak. Hanya mengukur; tidak melatih apa pun.
"""
import argparse
import json
import os
import sys

import numpy as np

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if AKAR not in sys.path:
    sys.path.insert(0, AKAR)

import konfig                                   # noqa: E402
import arena as A                               # noqa: E402
import rencana as R                             # noqa: E402
from drl import kelayakan as K                  # noqa: E402
from drl import uji_gudang3 as U                # noqa: E402
from drl.banding_baseline import PencatatNode   # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--per-alur", type=int, default=150)
    ap.add_argument("--benih", type=int, default=3)
    ap.add_argument("--keluar", default=None)
    a = ap.parse_args()
    konfig.pilih_robot(a.id)
    ar = A.Arena(U.PETA)
    st = U.stasiun(R.Perencana(ar, konfig.profil(a.id), jenis="astar"))
    rute = U.susun_rute(st, a.per_alur, a.benih)
    per = R.Perencana(ar, konfig.profil(a.id), jenis="dqfd-murni")
    catat = PencatatNode(per)
    pl = per.planner
    env, net = pl.env, pl.net
    X, Y, hitung = [], [], {"simulasi": 0}
    asli_calon, asli_siap = env.calon_malas, env.siapkan_aksi

    def rekam():
        X.append(env.observasi())
        Y.append(K.label_keadaan(env))
        return asli_calon()

    def siap(*args, **kw):
        if not kw.get("tanpa_cek", False):
            hitung["simulasi"] += 1
        return asli_siap(*args, **kw)

    env.calon_malas, env.siapkan_aksi = rekam, siap
    node = 0
    for i, (_, na, nb) in enumerate(rute):
        catat.total = 0
        per.rencanakan(st[na], st[nb], None, None)
        node += catat.total
        if i % 100 == 0:
            print(f"  {i}/{len(rute)} rute, {len(X)} keadaan", flush=True)
    X, Y = np.asarray(X, np.float32), np.asarray(Y, bool)
    Q = np.array([net.q_tabrak(x)[0] for x in X])
    P = np.array([net.q_tabrak(x)[1] for x in X])
    hasil = {"robot": a.id, "rute": len(rute), "keadaan": int(len(X)), "node": int(node),
             "aksi_menabrak_persen": 100 * float(Y.mean()),
             "keadaan_dengan_tabrakan_persen": 100 * float(Y.any(1).mean()),
             "auc_q": K.auc(-Q, Y), "p_col": K.metrik(P, Y),
             "primitif_disimulasi_lazy": hitung["simulasi"]}
    print(json.dumps({k: v for k, v in hasil.items() if k != "p_col"}, indent=1))
    pa = hasil["p_col"]["per_ambang"]["0.5"]
    print(f"P_col: AUC {hasil['p_col']['auc']:.4f}, @0,5 terdeteksi {pa['terdeteksi']:.1f}% alarm palsu {pa['alarm_palsu']:.1f}%")
    keluar = a.keluar or os.path.join(AKAR, "model_dqn_gudang3", f"uji_gudang3_tabrak_id{a.id:02d}.json")
    with open(keluar, "w") as f:
        json.dump(hasil, f, indent=1)
    print("tersimpan:", keluar)


if __name__ == "__main__":
    main()
