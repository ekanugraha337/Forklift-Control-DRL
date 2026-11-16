"""lab_v4.py - peta lab (5 peta nyata, rute uji dataset, robot ID 01): Hybrid A* standar, Hybrid A* lazy,
DQfD model lab tanpa P_col. Lintasan 1: waktu (tanpa pencatat). Lintasan 2: primitif dicek DQfD."""
import json, os, sys
os.environ["FORKLIFT_TANPA_P"] = "1"
sys.path.insert(0, os.getcwd()); sys.path.insert(0, os.path.join(os.getcwd(), "uji_nyata", "paper_ver9"))
import numpy as np
from drl import perencana_dqfd as PD
PD.DIR_MODEL = os.path.abspath("model_dqn")
import konfig, rencana as R
from drl import lingkungan as L, banding_baseline as BB
from drl.banding_baseline import PencatatNode
import astar_lazy as AL
assert PD.TANPA_P
RID = 1; konfig.pilih_robot(RID)
J = ("astar-standar", "astar-lazy", "dqfd")
asli = R.buat_perencana
mode = {"j": None}
def buat(arena, k, peran="astar"):
    if mode["j"] == "astar-lazy" and peran == "astar":
        return AL.jadikan_lazy(asli(arena, k, "astar-standar"))
    if mode["j"] == "astar-standar" and peran == "astar":
        return asli(arena, k, "astar-standar")
    return asli(arena, k, peran)
R.buat_perencana = buat
hit = {"n": 0, "on": False}
_e = L.LingkunganRencana._ekspansi
def eks(self, nd, nk, cek=True):
    if cek: hit["n"] += 1
    return _e(self, nd, nk, cek)
out = []
for peta in [p for p in BB.daftar_peta(RID) if not p.startswith("sintetis")]:
    ar, rute = BB.rute_uji(RID, peta)
    per = {}
    for j in J:
        mode["j"] = j
        per[j] = R.Perencana(ar, konfig.profil(RID), jenis="dqfd-murni" if j == "dqfd" else "astar")
        per[j].cat = PencatatNode(per[j])
        per[j].rencanakan(rute[0]["awal"], rute[0]["tujuan"], rute[0]["rintangan"] or None, None)
    baris = []
    for r in rute:
        b = {"peta": peta}
        for j in J:
            mode["j"] = j; per[j].cat.total = 0; AL.HIT["n"] = 0
            jl, info = per[j].rencanakan(r["awal"], r["tujuan"], r["rintangan"] or None, None)
            b[j] = dict(ok=bool(jl), node=per[j].cat.total, cpu_ms=info.get("cpu_s", 0) * 1000, panjang_cm=info.get("panjang_cm"),
                        primitif=AL.HIT["n"] if j == "astar-lazy" else None)
        baris.append(b)
    L.LingkunganRencana._ekspansi = eks
    for r, b in zip(rute, baris):
        mode["j"] = "dqfd"; hit["n"] = 0
        per["dqfd"].rencanakan(r["awal"], r["tujuan"], r["rintangan"] or None, None)
        b["dqfd"]["primitif"] = hit["n"]
    L.LingkunganRencana._ekspansi = _e
    out += baris
    print(peta, len(baris), flush=True)
json.dump(out, open(sys.argv[1], "w"))
for j in J:
    t = np.array([b[j]["cpu_ms"] for b in out]); n = np.array([b[j]["node"] for b in out])
    p = np.array([14 * b[j]["node"] if j == "astar-standar" else b[j]["primitif"] for b in out])
    print(f"{j}: tiba {100*np.mean([b[j]['ok'] for b in out]):.1f}% waktu med {np.median(t):.1f} p90 {np.percentile(t,90):.1f} mean {t.mean():.1f} maks {t.max():.0f} | primitif med {np.median(p):.0f} p90 {np.percentile(p,90):.0f} | node med {np.median(n):.0f}")
