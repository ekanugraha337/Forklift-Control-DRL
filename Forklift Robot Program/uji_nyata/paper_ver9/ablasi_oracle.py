"""ablasi_oracle.py - batas atas manfaat P_col: P diganti label tabrakan SEBENARNYA (prediktor sempurna)."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.getcwd())
from drl import perencana_dqfd as PD
import konfig, arena as A, rencana as R
from drl import lingkungan as L, uji_gudang3 as U, kelayakan as K, jaringan_numpy as JN
from drl.banding_baseline import PencatatNode

S = {"n": 0, "jeda": False, "oracle": False, "env": None}
_asli = L.LingkunganRencana._ekspansi
def _eks(self, nd, nk, cek=True):
    if cek and not S["jeda"]: S["n"] += 1
    return _asli(self, nd, nk, cek)
L.LingkunganRencana._ekspansi = _eks
_plan = PD.DQfDPlanner.plan
def plan(self, omap, start, goal):
    S["env"] = self.env
    return _plan(self, omap, start, goal)
PD.DQfDPlanner.plan = plan
_qt = JN.NetDenganP.q_tabrak
def q_tabrak(self, obs):
    q, p = _qt(self, obs)
    if S["oracle"]:
        S["jeda"] = True
        p = K.label_keadaan(S["env"]).astype(np.float32)      # label sebenarnya, tidak dihitung sebagai cek
        S["jeda"] = False
    return q, p
JN.NetDenganP.q_tabrak = q_tabrak

RID = 1; konfig.pilih_robot(RID)
ar = A.Arena("Gudang3")
st = U.stasiun(R.Perencana(ar, konfig.profil(RID), jenis="astar")); rr = U.susun_rute(st, 150, 3)
VAR = (("tanpa_P", 2.0, False), ("dengan_P", 0.5, False), ("P_sempurna", 0.5, True))
hasil = {}
for nama, dirm in (("model akhir", "model_dqn_gudang3"), ("model lab", "model_dqn")):
    PD.DIR_MODEL = os.path.abspath(dirm)
    per = {}
    for v, amb, orc in VAR:
        per[v] = R.Perencana(ar, konfig.profil(RID), jenis="dqfd-murni"); per[v].cat = PencatatNode(per[v])
    out = []
    for _, a, b in rr:
        r = {}
        for v, amb, orc in VAR:
            PD.AMBANG_TABRAK = amb; S["oracle"] = orc; S["n"] = 0; per[v].cat.total = 0
            jl, info = per[v].rencanakan(st[a], st[b], None, None)
            r[v] = dict(ok=bool(jl), node=per[v].cat.total, primitif=S["n"], oleh=info.get("perencana"))
        out.append(r)
    hasil[nama] = out
    print("==", nama, flush=True)
    for v, _, _ in VAR:
        p = np.array([r[v]["primitif"] for r in out]); n = np.array([r[v]["node"] for r in out])
        print(f"   {v}: tiba {100*np.mean([r[v]['ok'] for r in out]):.1f}% primitif med {np.median(p):.0f} p90 {np.percentile(p,90):.0f} total {p.sum()} per node {p.sum()/n.sum():.3f} node med {np.median(n):.0f} total {n.sum()} mundur-balik {sum('mundur' in str(r[v]['oleh']) for r in out)}", flush=True)
json.dump(hasil, open(sys.argv[1], "w"))
