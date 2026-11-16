"""
hitung_primitif.py - jumlah primitif yang disimulasi DENGAN cek tabrakan per rute,
Hybrid A* + RS vs DQfD, di rute operasional drl/uji_gudang3.py. Hanya mengukur:
pencatat dipasang dari luar (monkeypatch), program tidak diubah.

    cd "Forklift Robot Program"
    ../.venv/bin/python <skrip ini> --peta Gudang3 --dir-model model_dqn_gudang3 --jenis astar,dqfd-murni --keluar x.json
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.getcwd())

ap = argparse.ArgumentParser()
ap.add_argument("--id", type=int, default=1)
ap.add_argument("--peta", default="Gudang3")
ap.add_argument("--dir-model", default=None)
ap.add_argument("--jenis", default="astar,dqfd-murni")
ap.add_argument("--keluar", required=True)
ap.add_argument("--tanpa-hitung", action="store_true", help="tanpa pencatat primitif/RS: untuk waktu")
ap.add_argument("--dir-lab", default="model_dqn")
a = ap.parse_args()

from drl import perencana_dqfd as PD            # noqa: E402
if a.dir_model:
    PD.DIR_MODEL = os.path.abspath(a.dir_model)
import konfig                                   # noqa: E402
import arena as A                               # noqa: E402
import rencana as R                             # noqa: E402
from drl import uji_gudang3 as U                # noqa: E402
from drl.banding_baseline import PencatatNode   # noqa: E402

JENIS = a.jenis.split(",")
konfig.pilih_robot(a.id)
ar = A.Arena(a.peta)
hit = {j: 0 for j in JENIS}
aktif = {"j": None}          # perencana yang sedang dibuat / dijalankan


class KappaHitung(list):
    """Daftar tingkat kemudi; tiap nilai yang diambil loop ekspansi A* = satu primitif disimulasi."""
    def __iter__(self):
        j = aktif["j"]
        for k in list.__iter__(self):
            hit[j] += 1
            yield k


# Semua Hybrid A* (juga planner margin cadangan, yang dibuat belakangan) lewat buat_perencana.
_buat_asli = R.buat_perencana


DIRM = {"dqfd-murni": os.path.abspath(a.dir_model) if a.dir_model else PD.DIR_MODEL, "dqfd-lab": os.path.abspath(a.dir_lab)}


DIRM["dqfd-tanpa-p"] = DIRM["dqfd-eager"] = DIRM["dqfd-murni"]


def pilih(j):
    aktif["j"] = j
    if j in DIRM:
        PD.DIR_MODEL = DIRM[j]
    PD.AMBANG_TABRAK = 2.0 if j == "dqfd-tanpa-p" else 0.5      # P tidak pernah menggeser urutan
    PD.PENCARIAN_ROBOT = "eager" if j == "dqfd-eager" else "lazy"


def _buat(arena, kendaraan, peran="astar"):
    j = aktif["j"]
    if j == "astar-standar" and peran == "astar":
        peran = "astar-standar"          # Hybrid A* standar: RS tiap ekspansi ke-4 (konfig.ATURAN_RS)
    pl = _buat_asli(arena, kendaraan, peran)
    if j.startswith("astar"):
        pl.kappas = KappaHitung(pl.kappas)
    return pl


if not a.tanpa_hitung:
    R.buat_perencana = _buat

from drl import lingkungan as L                 # noqa: E402
_eks_asli = L.LingkunganRencana._ekspansi


def _ekspansi(self, nd, nk, cek=True):
    if cek:
        hit[aktif["j"]] += 1
    return _eks_asli(self, nd, nk, cek)


if not a.tanpa_hitung:
    L.LingkunganRencana._ekspansi = _ekspansi

import cepat as _C                              # noqa: E402
assert _C.ADA_NUMBA
rs = {"n": 0}
_rs_asli = _C.rs_periksa


def _rs_hitung(*x, **k):
    rs["n"] += 1
    return _rs_asli(*x, **k)


if not a.tanpa_hitung:
    _C.rs_periksa = _rs_hitung

per, catat = {}, {}
for j in JENIS:
    pilih(j)
    per[j] = R.Perencana(ar, konfig.profil(a.id), jenis="dqfd-murni" if j.startswith("dqfd") else
                         (j if a.tanpa_hitung else "astar"))
    catat[j] = PencatatNode(per[j])
    print(j, "peran:", getattr(per[j].planner, "peran", "-"),
          "analytic_every:", getattr(per[j].planner, "analytic_every", "-"), flush=True)

st = U.STASIUN[a.peta](per[JENIS[0]])
rute = U.susun_rute(st, 150, 3)
print(f"{a.peta}: {len(st)} stasiun, {len(rute)} rute", flush=True)
for j in JENIS:
    pilih(j)
    per[j].rencanakan(st[rute[0][1]], st[rute[0][2]], None, None)
hasil = []
for i, (alur, na, nb) in enumerate(rute):
    r = dict(alur=alur, dari=na, ke=nb)
    for j in JENIS:
        catat[j].total = 0
        hit[j] = 0
        rs["n"] = 0
        pilih(j)
        jalur, info = per[j].rencanakan(st[na], st[nb], None, None)
        mutu = {}
        if jalur:
            kd = per[j].kendaraan_dipakai
            ds = [ar.panjang_ke_cm(math.hypot(q[0] - p[0], q[1] - p[1])) for p, q in zip(jalur, jalur[1:])]
            mg = [ar.panjang_ke_cm(float(ar.omap.pose_margin(kd, p[0], p[1], p[2]))) for p in jalur[:-1]]
            mutu = dict(mundur_cm=sum(x for x, p in zip(ds, jalur) if len(p) > 3 and p[3] < 0),
                        sempit_cm=sum(x for x, m in zip(ds, mg) if m < 4.0),
                        bebas_rata_cm=sum(mg) / max(1, len(mg)))
        r[j] = dict(**mutu, ok=bool(jalur), node=catat[j].total, margin_cm=info.get('margin_cm'), primitif=hit[j], rs_kurva=rs["n"],
                    panjang_cm=info.get("panjang_cm"), alasan=info.get("alasan"),
                    ganti_arah=(sum(1 for p, q in zip(jalur, jalur[1:]) if len(p) > 3 and p[3] != q[3])
                                if jalur else None),
                    cpu_ms=info.get("cpu_s", 0.0) * 1000.0, perencana=info.get("perencana"))
    hasil.append(r)
    if i % 100 == 0:
        print(f"  {i}/{len(rute)}", flush=True)
with open(a.keluar, "w") as f:
    json.dump(dict(peta=a.peta, dir_model=a.dir_model, rute=hasil), f)
print("tersimpan:", a.keluar)
