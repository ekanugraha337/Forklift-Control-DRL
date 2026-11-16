"""
statistik.py - review pembimbing ver8 (Tabel I): uji Wilcoxon signed-rank berpasangan dan CI bootstrap
95% untuk angka kunci Tabel I paper ver9 (Warehouse3, 632 rute). Sumber data sama dengan tabel4.py:
b4 (waktu, panjang, tiba), h4 (primitif DQfD), v2 (primitif Hybrid A* standar), primitif lazy A* di b4.

Primitif dan waktu: semua rute (rute yang gagal tercatat sampai batas waktu, jadi angka Hybrid A*-nya
batas bawah). Panjang: hanya rute yang diselesaikan kedua perencana.

    cd "Forklift Robot Program" && ../.venv/bin/python uji_nyata/paper_ver9/statistik.py [Gudang3]
"""
import json
import os
import sys

import numpy as np
from scipy import stats

S = os.path.dirname(os.path.abspath(__file__))
peta = sys.argv[1] if len(sys.argv) > 1 else "Gudang3"
g = {"Gudang3": "g3", "Gudang4": "g4"}[peta]


def muat(f):
    return json.load(open(os.path.join(S, f)))["rute"]


def arr(d, j, k):
    return np.array([np.nan if r[j].get(k) is None else float(r[j][k]) for r in d])


d, h, v = muat(f"b4_{peta}.json"), muat(f"h4_{peta}.json"), muat(f"v2_{g}.json")
assert len(d) == len(h) == len(v)
assert all((a["dari"], a["ke"]) == (b["dari"], b["ke"]) == (c["dari"], c["ke"]) for a, b, c in zip(d, h, v))
D, B, Z = "dqfd-murni", "astar-standar", "astar-lazy"
prim = {D: arr(h, D, "primitif"), B: arr(v, B, "primitif"), Z: arr(d, Z, "primitif")}
waktu = {j: arr(d, j, "cpu_ms") for j in (D, B, Z)}
ok = {j: arr(d, j, "ok") > 0 for j in (D, B, Z)}
pjg = {j: arr(d, j, "panjang_cm") for j in (D, B, Z)}
n = len(d)
rng = np.random.default_rng(20261116)
NB = 10000
IDX = rng.integers(0, n, size=(NB, n))


def ci(f, *xs):
    """CI bootstrap 95% (persentil) untuk statistik f atas rute yang diambil ulang berpasangan."""
    nilai = np.array([f(*(x[i] for x in xs)) for i in IDX])
    nilai = nilai[np.isfinite(nilai)]
    return f(*xs), np.percentile(nilai, 2.5), np.percentile(nilai, 97.5)


def rasio(q):
    return lambda a, b: np.percentile(a, q) / np.percentile(b, q)


def tulis(nama, t):
    print(f"   {nama}: {t[0]:.3f}  [95% CI {t[1]:.3f}, {t[2]:.3f}]")


def wil(nama, a, b):
    r = stats.wilcoxon(a, b)
    lebih = np.mean(a > b)
    print(f"   Wilcoxon {nama}: n={len(a)}, p={r.pvalue:.2e}, a>b pada {100 * lebih:.1f}% pasangan")
    return r.pvalue


print(f"===== {peta}: {n} rute")
for X in (B, Z):
    print(f"-- {X} vs {D}")
    wil("primitif", prim[X], prim[D])
    wil("waktu", waktu[X], waktu[D])
    tulis("primitif median X/D", ci(rasio(50), prim[X], prim[D]))
    tulis("primitif p90 X/D", ci(rasio(90), prim[X], prim[D]))
    tulis("waktu median X/D", ci(rasio(50), waktu[X], waktu[D]))
    tulis("waktu p90 X/D", ci(rasio(90), waktu[X], waktu[D]))
    tulis("waktu rata-rata X/D", ci(lambda a, b: a.mean() / b.mean(), waktu[X], waktu[D]))
    m = ok[X] & ok[D]
    q = pjg[D][m] / pjg[X][m]
    wil(f"panjang D vs X (keduanya tiba, {m.sum()})", pjg[D][m], pjg[X][m])
    nq = len(q)
    iq = rng.integers(0, nq, size=(NB, nq))
    for p in (50, 90):
        bs = np.percentile(q[iq], p, axis=1)
        print(f"   panjang D/X p{p}: {np.percentile(q, p):.3f}  [95% CI {np.percentile(bs, 2.5):.3f}, {np.percentile(bs, 97.5):.3f}]")
print(f"-- {B} vs {Z}")
wil("primitif", prim[B], prim[Z])
wil("waktu", waktu[B], waktu[Z])
tulis("primitif median B/Z", ci(rasio(50), prim[B], prim[Z]))
tulis("waktu rata-rata B/Z", ci(lambda a, b: a.mean() / b.mean(), waktu[B], waktu[Z]))
