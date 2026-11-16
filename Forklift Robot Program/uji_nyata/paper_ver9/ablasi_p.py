"""ablasi_p.py - di kondisi mana jaringan tabrakan (P_col) berguna? DQfD lazy dengan dan tanpa P_col:
peta lab (model lab), Gudang3 dengan model lab, Gudang3 dengan badan robot lain di tengah jalur.
Hanya mengukur (pencatat dipasang dari luar). Jalankan dari 'Forklift Robot Program'."""
import json, math, os, sys, time
import numpy as np
sys.path.insert(0, os.getcwd())
from drl import perencana_dqfd as PD
import konfig, arena as A, rencana as R
from drl import lingkungan as L, uji_gudang3 as U, banding_baseline as BB
from drl.banding_baseline import PencatatNode

hit = {"n": 0}
_asli = L.LingkunganRencana._ekspansi
def _eks(self, nd, nk, cek=True):
    if cek: hit["n"] += 1
    return _asli(self, nd, nk, cek)
L.LingkunganRencana._ekspansi = _eks

RID = 1
konfig.pilih_robot(RID)
VAR = (("dengan_P", 0.5), ("tanpa_P", 2.0))

def uji(nama, ar, rute, dir_model):
    PD.DIR_MODEL = os.path.abspath(dir_model)
    per = {}
    for v, amb in VAR:
        PD.AMBANG_TABRAK = amb
        per[v] = R.Perencana(ar, konfig.profil(RID), jenis="dqfd-murni"); per[v].cat = PencatatNode(per[v])
        per[v].rencanakan(rute[0][0], rute[0][1], rute[0][2])
    out = []
    for awal, tujuan, rint in rute:
        r = {}
        for v, amb in VAR:
            PD.AMBANG_TABRAK = amb; PD.DIR_MODEL = os.path.abspath(dir_model)
            per[v].cat.total = 0; hit["n"] = 0
            jl, info = per[v].rencanakan(awal, tujuan, rint)
            r[v] = dict(ok=bool(jl), node=per[v].cat.total, primitif=hit["n"], cpu_ms=info.get("cpu_s", 0) * 1000,
                        oleh=info.get("perencana"), panjang_cm=info.get("panjang_cm"))
        out.append(r)
    ring = {}
    for v, _ in VAR:
        p = np.array([r[v]["primitif"] for r in out]); n = np.array([r[v]["node"] for r in out]); t = np.array([r[v]["cpu_ms"] for r in out])
        ring[v] = dict(tiba=100 * np.mean([r[v]["ok"] for r in out]), prim_med=float(np.median(p)), prim_p90=float(np.percentile(p, 90)),
                       prim_mean=float(p.mean()), per_node=float(p.sum() / max(1, n.sum())), node_med=float(np.median(n)),
                       ms_med=float(np.median(t)), ms_mean=float(t.mean()), mundur=sum("mundur" in str(r[v]["oleh"]) for r in out))
    print(f"== {nama}: {len(out)} rute", flush=True)
    for v in ring: print("  ", v, {k: round(x, 2) for k, x in ring[v].items()}, flush=True)
    return dict(nama=nama, ringkas=ring, rute=out)

hasil = []
# A. peta lab, model lab
semua = []
for peta in BB.daftar_peta(RID):
    ar, rute = BB.rute_uji(RID, peta)
    semua.append((peta, ar, [(r["awal"], r["tujuan"], r["rintangan"] or None) for r in rute]))
for peta, ar, rute in semua:
    hasil.append(uji(f"lab:{peta} (model lab)", ar, rute, "model_dqn"))
# B dan C: Gudang3
ar = A.Arena("Gudang3")
pa = R.Perencana(ar, konfig.profil(RID), jenis="astar")
st = U.stasiun(pa); rr = U.susun_rute(st, 150, 3)
polos = [(st[a], st[b], None) for _, a, b in rr]
hasil.append(uji("Gudang3 (model lab)", ar, polos, "model_dqn"))
rint = []
for awal, tujuan, _ in polos:
    dasar, _i = pa.rencanakan(awal, tujuan, None, None)
    if not dasar: continue
    q = dasar[len(dasar) // 2]; xc, yc, yd = ar.pose_ke_cm(q[0], q[1], q[2])
    if min(math.hypot(xc - awal[0], yc - awal[1]), math.hypot(xc - tujuan[0], yc - tujuan[1])) > 60.0:
        rint.append((awal, tujuan, [np.asarray(A.jejak_badan(xc, yc, yd), np.float64)]))
hasil.append(uji("Gudang3 + badan robot lain di tengah jalur (model Gudang3)", ar, rint, "model_dqn_gudang3"))
json.dump(hasil, open(sys.argv[1], "w"))
