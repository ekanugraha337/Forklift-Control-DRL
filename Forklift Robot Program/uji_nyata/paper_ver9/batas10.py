"""
batas10.py - review pembimbing ver8 (3.1, 3.2 "Arrival rate"): apakah kegagalan Hybrid A* di
Warehouse3 melekat pada perencananya atau hanya akibat batas waktu 2 s? Rute yang gagal pada
b4_Gudang3.json direncanakan ulang dengan batas waktu 10 s (batas ekspansi praktis dibuang,
supaya hanya waktu yang membatasi). Program tidak diubah: batas dipasang dari luar.

    cd "Forklift Robot Program"
    caffeinate -i -s ../.venv/bin/python uji_nyata/paper_ver9/batas10.py --keluar uji_nyata/paper_ver9/batas10_Gudang3.json
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.getcwd())

ap = argparse.ArgumentParser()
ap.add_argument("--id", type=int, default=1)
ap.add_argument("--peta", default="Gudang3")
ap.add_argument("--batas", type=float, default=10.0)
ap.add_argument("--jenis", default="astar-standar,astar-lazy")
ap.add_argument("--keluar", required=True)
a = ap.parse_args()

import konfig                                   # noqa: E402
import arena as A                               # noqa: E402
import rencana as R                             # noqa: E402
from drl import uji_gudang3 as U                # noqa: E402

_buat_asli = R.buat_perencana


def _buat(arena, kendaraan, peran="astar"):
    pl = _buat_asli(arena, kendaraan, peran)
    pl.time_limit = a.batas
    pl.max_expansions = 10 ** 8
    return pl


R.buat_perencana = _buat

asal = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), f"b4_{a.peta}.json")))["rute"]
konfig.pilih_robot(a.id)
ar = A.Arena(a.peta)
JENIS = a.jenis.split(",")
per = {j: R.Perencana(ar, konfig.profil(a.id), jenis=j) for j in JENIS}
st = U.STASIUN[a.peta](per[JENIS[0]])
rute = U.susun_rute(st, 150, 3)
assert [(x, y, z) for x, y, z in rute] == [(r["alur"], r["dari"], r["ke"]) for r in asal]
for j in JENIS:
    per[j].rencanakan(st[rute[0][1]], st[rute[0][2]], None, None)       # pemanasan
    print(j, "peran:", per[j].planner.peran, "time_limit:", per[j].planner.time_limit, flush=True)

hasil = []
for j in JENIS:
    for i, r in enumerate(asal):
        if r[j]["ok"]:
            continue
        jalur, info = per[j].rencanakan(st[r["dari"]], st[r["ke"]], None, None)
        x = dict(i=i, perencana=j, alur=r["alur"], dari=r["dari"], ke=r["ke"], ok=bool(jalur),
                 cpu_ms=info["cpu_s"] * 1000.0, dinding_ms=info["waktu_s"] * 1000.0,
                 ekspansi=(per[j].planner.last_stats or {}).get("expansions"),
                 panjang_cm=info.get("panjang_cm"), alasan=info.get("alasan"),
                 dqfd_panjang_cm=r["dqfd-murni"]["panjang_cm"], dqfd_cpu_ms=r["dqfd-murni"]["cpu_ms"])
        hasil.append(x)
        print(f"  {j} rute {i} {r['dari']}->{r['ke']}: {'tiba' if jalur else 'gagal'} "
              f"{x['cpu_ms']:.0f} ms CPU ({x['dinding_ms']:.0f} ms dinding), "
              f"panjang {x['panjang_cm']:.0f} vs DQfD {x['dqfd_panjang_cm']:.0f} cm  {x['alasan']}", flush=True)
with open(a.keluar, "w") as f:
    json.dump(dict(peta=a.peta, batas_s=a.batas, rute=hasil), f)
print("tersimpan:", a.keluar)
