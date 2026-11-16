"""sim5.py - ringkasan closed-loop berulang (Tabel II paper ver9): makespan per skenario dan perencana
(rata-rata dan simpangan baku atas ulangan), tiba, jarak terdekat ke zona, dan bedah waktu per robot
(panjang jalur rencana, laju saat bergerak = panjang rencana / waktu gerak, jarak mundur dan jarak dekat
zona pada jalur pertama, waktu rencana). Jarak tempuh terukur tidak dipakai: dengan derau pose ia
ikut menjumlah derau. Percobaan dengan jeda > JEDA_MAKS s di lintasan.csv (simulasi berhenti,
mis. komputer tidur) tidak sah: dikeluarkan dan dilaporkan.

    cd "Forklift Robot Program" && ../.venv/bin/python uji_nyata/paper_ver9/sim5.py <folder hasil> ...
"""
import collections
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.getcwd())
import konfig                      # noqa: E402
import arena as A                  # noqa: E402
import rencana as R                # noqa: E402

konfig.pilih_robot(1)
ar = A.Arena("Gudang3")
kend = {rid: R.Perencana(ar, konfig.profil(rid), jenis="astar").kendaraan for rid in (1, 2)}

JEDA_MAKS = 10.0
import csv  # noqa: E402

mk = collections.defaultdict(lambda: collections.defaultdict(list))     # skenario -> perencana -> [makespan|None]
rob = collections.defaultdict(lambda: collections.defaultdict(list))    # perencana -> ukuran -> [nilai]
zona_min = collections.defaultdict(lambda: float("inf"))
for d in sys.argv[1:]:
    jeda = collections.defaultdict(float)
    t_akhir = {}
    for r in csv.DictReader(open(os.path.join(d, "lintasan.csv"))):
        if r["fase"] == "PULANG":
            continue
        kunci = (r["skenario"], r["ulang"], r["perencana"])
        t = float(r["t"])
        if kunci in t_akhir:
            jeda[kunci] = max(jeda[kunci], t - t_akhir[kunci])
        t_akhir[kunci] = t
    for line in open(os.path.join(d, "hasil.jsonl")):
        h = json.loads(line)
        per = h["perencana"]
        if jeda[(h["skenario"], str(h["ulang"]), per)] > JEDA_MAKS:
            print(f"[!] tidak sah (jeda {jeda[(h['skenario'], str(h['ulang']), per)]:.0f} s): "
                  f"{os.path.basename(d)} {h['skenario']} ulangan {h['ulang'] + 1} {per}")
            continue
        mk[h["skenario"]][per].append(h["makespan_s"] if h["semua_tiba"] else None)
        for rid, r in h["robot"].items():
            rob[per]["tiba"].append(bool(r["tiba"]))
            if not r["tiba"] or not r.get("rencana") or not r["rencana"][0].get("jalur_cm"):
                continue
            zona_min[per] = min(zona_min[per], r["zona_min_cm"])
            p = r["rencana"][0]["jalur_cm"]
            k = kend[int(rid)]
            sempit = mundur = 0.0
            for a, b in zip(p, p[1:]):
                ds = math.hypot(b[0] - a[0], b[1] - a[1])
                g = ar.pose_ke_sel(a[0], a[1], a[2])
                if ar.panjang_ke_cm(float(ar.omap.pose_margin(k, *g))) < 4.0:
                    sempit += ds
                if a[3] < 0:
                    mundur += ds
            gerak = r["t_tempuh_s"] - r["t_menunggu_s"] - r["t_tahan_s"]
            rob[per]["jarak"].append(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(p, p[1:])))
            rob[per]["gerak_s"].append(gerak)
            rob[per]["mundur"].append(mundur)
            rob[per]["sempit"].append(sempit)
            rob[per]["ms"].append(r["ms_rencana_total"])
            rob[per]["tunggu"].append(r["t_menunggu_s"] + r["t_tahan_s"])

pers = sorted({p for v in mk.values() for p in v})
print("makespan [s]: rata-rata +- sd (selesai/ulangan)")
for sk in sorted(mk):
    sel = []
    for p in pers:
        v = mk[sk].get(p, [])
        ok = [x for x in v if x is not None]
        sel.append(f"{p}: {np.mean(ok):5.1f} +- {np.std(ok, ddof=1) if len(ok) > 1 else 0:4.1f} ({len(ok)}/{len(v)})"
                   if ok else f"{p}: --  (0/{len(v)})")
    print(" ", sk, " | ".join(sel))
lengkap = [sk for sk in mk if all(mk[sk].get(p) and None not in mk[sk][p] for p in pers)]
print("skenario yang diselesaikan semua perencana di semua ulangan:", sorted(lengkap))
for p in pers:
    m = [np.mean(mk[sk][p]) for sk in lengkap]
    print(f"  {p}: rata-rata makespan {np.mean(m):.1f} s")
print("per robot (rata-rata atas robot yang tiba)")
for p in pers:
    r = rob[p]
    print(f"  {p}: tiba {sum(r['tiba'])}/{len(r['tiba'])}, jarak {np.mean(r['jarak']):.0f} cm, "
          f"laju {np.sum(r['jarak']) / np.sum(r['gerak_s']):.1f} cm/s, mundur {np.mean(r['mundur']):.0f} cm, "
          f"dekat zona {np.mean(r['sempit']):.0f} cm, rencana {np.mean(r['ms']):.0f} ms, tunggu {np.mean(r['tunggu']):.1f} s, "
          f"jarak terdekat ke zona {zona_min[p]:.1f} cm")
