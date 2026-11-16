"""
buat_skenario_lab.py - skenario dua robot untuk uji robot nyata paper ver9 (validasi kelayakan di peta lab).

Kriteria pemilihan (ditulis di paper): dari rute uji dataset di peta lab (tidak dipakai melatih), per robot
diambil K rute yang paling lama direncanakan Hybrid A* standar dan diselesaikan semua perencana uji. Pasangan
ID 01 + ID 02 diperiksa seperti mode --buat drl/uji_robot_nyata.py (periksa_skenario): setiap robot
direncanakan dari awalnya ke tujuannya dengan setiap perencana, robot lain di pose AWAL-nya; tujuan tidak
bertumpuk. Dipilih 2 pasangan terberat yang jalurnya tidak bersilangan dan 1 yang bersilangan, tanpa memakai
rute yang sama dua kali. Skor = jumlah waktu Hybrid A* standar kedua robot.

    cd "Forklift Robot Program"
    ../.venv/bin/python uji_nyata/paper_ver9/buat_skenario_lab.py --peta Gudang2 --keluar uji_nyata/lab_Gudang2.json

Model bawaan = model akhir (model_dqn_gudang3), sama dengan "DQfD" di paper; di peta lab ia setara model lab
(model_di_lab.py). Robot nyata harus memakai model yang sama: FORKLIFT_DIR_MODEL=model_dqn_gudang3.
"""
import argparse
import json
import os
import sys

os.environ["FORKLIFT_TANPA_P"] = "1"           # paper ver9: DQfD tanpa P_col
sys.path.insert(0, os.getcwd())

ap = argparse.ArgumentParser()
ap.add_argument("--peta", default="Gudang2")
ap.add_argument("--keluar", required=True)
ap.add_argument("--k", type=int, default=10, help="rute terberat per robot yang dipasangkan")
ap.add_argument("--jarak-awal-cm", type=float, default=10.0, help="jarak badan minimum antar pose awal")
ap.add_argument("--dir-model", default="model_dqn_gudang3", help="model DQfD (bawaan: model akhir paper, tahap 2)")
a = ap.parse_args()
os.environ["FORKLIFT_DIR_MODEL"] = a.dir_model

import konfig                                   # noqa: E402
import arena as A                               # noqa: E402
import rencana as R                             # noqa: E402
import wasit as W                               # noqa: E402
from drl import perencana_dqfd as PD            # noqa: E402
from drl import banding_baseline as BB          # noqa: E402

assert PD.TANPA_P and os.path.basename(PD.DIR_MODEL) == os.path.basename(a.dir_model), PD.DIR_MODEL
J = ("astar-standar", "dqfd-murni")
ar = A.Arena(a.peta)
per = {rid: {j: R.Perencana(ar, konfig.profil(rid), jenis=j) for j in J} for rid in (1, 2)}


def rencana(rid, j, awal, tujuan, lain=None):
    jl, info = per[rid][j].rencanakan(tuple(awal), tuple(tujuan), lain or None)
    return jl, info


def jalur_cm(jl):
    return [ar.pose_ke_cm(p[0], p[1], p[2])[:2] for p in jl]


def bersilang(p, q):
    def ccw(a, b, c):
        return (c[1] - a[1]) * (b[0] - a[0]) - (b[1] - a[1]) * (c[0] - a[0])
    for a1, a2 in zip(p, p[1:]):
        for b1, b2 in zip(q, q[1:]):
            if ccw(a1, b1, b2) * ccw(a2, b1, b2) < 0 and ccw(a1, a2, b1) * ccw(a1, a2, b2) < 0:
                return True
    return False


# 1. Rute uji per robot, diurutkan menurut waktu Hybrid A* standar (tanpa robot lain).
calon = {}
for rid in (1, 2):
    _, rute = BB.rute_uji(rid, a.peta)
    rute = [r for r in rute if r["tujuan"][2] is not None]
    for j in J:                                                    # pemanasan
        rencana(rid, j, rute[0]["awal"], rute[0]["tujuan"])
    nilai = []
    for r in rute:
        jl, info = rencana(rid, "astar-standar", r["awal"], r["tujuan"])
        jd, _ = rencana(rid, "dqfd-murni", r["awal"], r["tujuan"])
        if jl and jd:
            nilai.append((info["cpu_s"] * 1000.0, r))
    nilai.sort(key=lambda x: -x[0])
    calon[rid] = [r for _, r in nilai[:a.k]]
    print(f"ID {rid:02d}: {len(rute)} rute uji berarah, {len(nilai)} diselesaikan keduanya; "
          f"Hybrid A* standar terberat: {', '.join(f'{t:.0f}' for t, _ in nilai[:a.k])} ms", flush=True)

# 2. Pasangan ID 01 + ID 02 yang layak, seperti periksa_skenario.
pasangan = []
for r1 in calon[1]:
    for r2 in calon[2]:
        if W.jarak_poligon(A.jejak_badan(*r1["awal"]), A.jejak_badan(*r2["awal"])) < a.jarak_awal_cm:
            continue
        if W.jarak_poligon(A.jejak_badan(*r1["tujuan"]), A.jejak_badan(*r2["tujuan"])) < konfig.MULTI_JARAK_AMAN_CM:
            continue
        ok, ms, jalur = True, {}, {}
        for rid, r, lain in ((1, r1, r2), (2, r2, r1)):
            hal = [A.jejak_badan(*lain["awal"])]
            for j in J:
                if per[rid][j].margin_untuk_pose(*r["tujuan"], hal) is None:
                    ok = False
                    break
                jl, info = rencana(rid, j, r["awal"], r["tujuan"], hal)
                if not jl:
                    ok = False
                    break
                ms[(rid, j)] = info["cpu_s"] * 1000.0
                jalur[(rid, j)] = (jalur_cm(jl), info["panjang_cm"])
            if not ok:
                break
        if ok:
            silang = bersilang(jalur[(1, "astar-standar")][0], jalur[(2, "astar-standar")][0])
            pasangan.append(dict(r1=r1, r2=r2, ms=ms, silang=silang,
                                 skor=ms[(1, "astar-standar")] + ms[(2, "astar-standar")],
                                 panjang={k: v[1] for k, v in jalur.items()}))
print(f"{len(pasangan)} pasangan layak, {sum(p['silang'] for p in pasangan)} bersilangan", flush=True)

# 3. Pilih 2 terberat tanpa silang + 1 terberat bersilang, rute tidak dipakai ulang.
pasangan.sort(key=lambda p: -p["skor"])
pilih, dipakai = [], set()
for mau_silang, n in ((False, 2), (True, 1)):
    for p in pasangan:
        if sum(q["silang"] == mau_silang for q in pilih) >= n:
            break
        if p["silang"] != mau_silang or p["r1"]["e"] in dipakai or ("2", p["r2"]["e"]) in dipakai:
            continue
        pilih.append(p)
        dipakai |= {p["r1"]["e"], ("2", p["r2"]["e"])}

skenario, catatan = [], []
for i, p in enumerate(pilih, 1):
    nama = f"L{i}" if not p["silang"] else f"LX{i}"
    bulat = lambda v: [round(float(x), 2) for x in v]   # noqa: E731
    skenario.append({"nama": nama, "robot": {
        "1": {"awal": bulat(p["r1"]["awal"]), "tujuan": bulat(p["r1"]["tujuan"])},
        "2": {"awal": bulat(p["r2"]["awal"]), "tujuan": bulat(p["r2"]["tujuan"])}}})
    catatan.append({"nama": nama, "bersilang": p["silang"], "episode_uji": [p["r1"]["e"], p["r2"]["e"]],
                    "ms_mac": {f"{rid:02d} {j}": round(v, 1) for (rid, j), v in p["ms"].items()},
                    "panjang_cm": {f"{rid:02d} {j}": round(v) for (rid, j), v in p["panjang"].items()}})
    print(f"  {nama}: silang={p['silang']}  " + "  ".join(f"{k}: {v} ms" for k, v in catatan[-1]["ms_mac"].items()))
    print(f"       panjang: " + "  ".join(f"{k}: {v} cm" for k, v in catatan[-1]["panjang_cm"].items()))
with open(a.keluar, "w", encoding="utf-8") as f:
    json.dump({"peta": a.peta, "robot": [1, 2], "skenario": skenario}, f, indent=1)
with open(os.path.splitext(a.keluar)[0] + "_pemilihan.json", "w", encoding="utf-8") as f:
    json.dump({"peta": a.peta, "model": a.dir_model, "kriteria": __doc__.split("Kriteria pemilihan")[1].split("\n\n")[0], "skenario": catatan}, f, indent=1)
print("tersimpan:", a.keluar)
