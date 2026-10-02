"""
rincian_komputasi.py - rincian waktu PER TAHAP perencanaan: Hybrid A* vs DQfD.

    python -m drl.rincian_komputasi                                   # tiga peta, ID 01 & 02
    python -m drl.rincian_komputasi --peta Rumah --id 2 --perencana astar,dqfd

Pelengkap drl.banding_komputasi (yang hanya mengukur waktu total). Rutenya
sama (rute uji dataset di peta itu, lengkap dengan margin dan badan robot lain)
dan jalur kodenya sama dengan navigasi.py: Perencana.rencanakan(), termasuk
mundur ke margin cadangan dan percobaan arah hadap bila tujuan tanpa arah.

Tiap rute direncanakan DUA kali setelah pemanasan:
  1. polos      tanpa pengukur  -> waktu jam yang sebenarnya (median, p90)
  2. terukur    fungsi-fungsi tahap dibungkus pengukur waktu
Pengukur menambah ongkos per panggilan, jadi jumlah tahap pada (2) sedikit di
atas (1); selisihnya dicetak sebagai "overhead pengukur". forklift_planner.py
dan drl/ tidak diubah - pembungkus dipasang di objek/kelas selama pengukuran.

Waktu tiap tahap EKSKLUSIF (tanpa tahap lain yang dipanggil di dalamnya), jadi
jumlahnya = total. Tahap:
    peta rintangan        Perencana.peta_dengan: badan robot lain -> ObstacleMap
    heuristik Dijkstra    _holonomic_heuristic (sekali per plan)
    arah pendekatan       _feasible_approach_dirs (sekali per plan)
    Reeds-Shepp           _analytic -> _best_free_rs (kandidat + cek tabrakan)
    primitif              DQfD: LingkunganRencana._ekspansi (14 calon per keadaan)
    sinar / observasi     DQfD: 32 sinar, dan sisa vektor observasi
    jaringan              DQfD: forward DQN (Q per keadaan + V di awal)
    pencarian (sisa)      A*: loop plan() = ekspansi primitif + heap + closed set
                          (ekspansi A* ada di dalam loop, tidak bisa dibungkus);
                          DQfD: heap, pulihkan simpul, mask, reset di luar tahap di atas
    lain-lain             Perencana: loop margin, arah hadap, panjang jalur

Jangan jalankan bersamaan dengan pelatihan/uji lain: yang diukur waktu.
Keluaran: model_dqn/rincian_komputasi.json
"""
import argparse
import json
import os
import sys
import time
from collections import defaultdict

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import forklift_planner as FP       # noqa: E402
import konfig                       # noqa: E402
import rencana as R                 # noqa: E402
from drl import lingkungan as L     # noqa: E402
from drl.banding_komputasi import rute_uji  # noqa: E402

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# (label, kelas, atribut). Urutan = urutan baris di tabel.
TAHAP = (
    ("peta rintangan", R.Perencana, "peta_dengan"),
    ("heuristik Dijkstra", FP.HybridAStarPlanner, "_holonomic_heuristic"),
    ("arah pendekatan", FP.HybridAStarPlanner, "_feasible_approach_dirs"),
    ("Reeds-Shepp", FP.HybridAStarPlanner, "_analytic"),
    ("primitif", L.LingkunganRencana, "_ekspansi"),
    ("sinar", L.LingkunganRencana, "_sinar"),
    ("observasi lain", L.LingkunganRencana, "observasi"),
    ("jaringan", None, "forward"),
    ("pencarian A* (sisa)", FP.HybridAStarPlanner, "plan"),
    ("pencarian DQfD (sisa)", None, "plan"),
    ("lain-lain", R.Perencana, "rencanakan"),
)
URUT = [t[0] for t in TAHAP]


class Pengukur:
    """Waktu eksklusif per label, lewat tumpukan: waktu anak dikurangkan dari induk."""

    def __init__(self):
        self.t, self.n = defaultdict(float), defaultdict(int)
        self._tumpuk, self._pulih = [], []

    def nol(self):
        self.t.clear()
        self.n.clear()

    def bungkus(self, obj, attr, label):
        asli = obj.__dict__[attr] if isinstance(obj, type) else getattr(obj, attr)
        tumpuk, t, n = self._tumpuk, self.t, self.n

        def w(*a, **k):
            tumpuk.append(0.0)
            t0 = time.perf_counter()
            try:
                return asli(*a, **k)
            finally:
                dt = time.perf_counter() - t0
                anak = tumpuk.pop()
                t[label] += dt - anak
                n[label] += 1
                if tumpuk:
                    tumpuk[-1] += dt
        setattr(obj, attr, w)
        self._pulih.append((obj, attr, asli, isinstance(obj, type)))

    def lepas(self):
        for obj, attr, asli, kelas in reversed(self._pulih):
            if kelas:
                setattr(obj, attr, asli)
            else:
                delattr(obj, attr)          # kembali ke metode kelas
        self._pulih.clear()


def _pasang(ukur, per, rid):
    for label, kelas, attr in TAHAP:
        if kelas is not None:
            ukur.bungkus(kelas, attr, label)
    if per.jenis != "astar":
        from drl.perencana_dqfd import DQfDPlanner, muat_model
        ukur.bungkus(DQfDPlanner, "plan", "pencarian DQfD (sisa)")
        ukur.bungkus(muat_model(rid), "forward", "jaringan")


def ukur(rid, peta, n, perencana):
    ar, rute = rute_uji(rid, peta, n)
    prof = konfig.profil(rid)
    hasil = {}
    for jenis in perencana:
        per = R.Perencana(ar, prof, jenis=jenis)
        per.rencanakan(rute[0]["awal"], rute[0]["tujuan"], rute[0]["rintangan"])   # pemanasan
        baris = []
        for r in rute:
            t0 = time.perf_counter()
            jalur, info = per.rencanakan(r["awal"], r["tujuan"], r["rintangan"])
            polos = time.perf_counter() - t0

            p = Pengukur()
            _pasang(p, per, rid)
            try:
                t0 = time.perf_counter()
                jalur2, _ = per.rencanakan(r["awal"], r["tujuan"], r["rintangan"])
                terukur = time.perf_counter() - t0
            finally:
                p.lepas()
            baris.append({"e": r["e"], "sukses": bool(jalur), "sama": bool(jalur) == bool(jalur2),
                          "oleh": info.get("perencana", "A*") if jalur else "gagal",
                          "margin": info.get("margin_cm"),
                          "ms": 1000 * polos, "ms_terukur": 1000 * terukur,
                          "tahap_ms": {k: 1000 * v for k, v in p.t.items()},
                          "panggil": dict(p.n)})
        hasil[jenis] = baris
    return {"robot": rid, "peta": peta, "n": len(rute), "hasil": hasil}


def cetak(r):
    print(f"\n=== ID {r['robot']:02d} di {r['peta']}: {r['n']} rute uji "
          f"(ms RATA-RATA per rencana; % = porsi waktu terukur)")
    jenis = list(r["hasil"])
    kepala = "    tahap                  " + "".join(f"{j:>27}" for j in jenis)
    print(kepala)
    rata = {}
    for j in jenis:
        b = r["hasil"][j]
        tot = np.mean([x["ms_terukur"] for x in b])
        rata[j] = (tot, {k: np.mean([x["tahap_ms"].get(k, 0.0) for x in b]) for k in URUT},
                   {k: np.mean([x["panggil"].get(k, 0) for x in b]) for k in URUT})
    for k in URUT:
        if all(rata[j][1][k] == 0 for j in jenis):
            continue
        sel = ""
        for j in jenis:
            tot, ms, n = rata[j]
            if ms[k] == 0:
                sel += f"{'-':>27}"
            else:
                kali = f"{n[k]:7.1f}x" if k not in ("lain-lain",) and "sisa" not in k else " " * 8
                sel += f"{ms[k]:9.2f} ms {100 * ms[k] / tot:5.1f}% {kali}"
        print(f"    {k:<22} {sel}")
    print("    " + "-" * (23 + 27 * len(jenis)))
    for label, f in (("total terukur (rata)", lambda b: np.mean([x["ms_terukur"] for x in b])),
                     ("total polos (rata)", lambda b: np.mean([x["ms"] for x in b])),
                     ("total polos (median)", lambda b: np.median([x["ms"] for x in b])),
                     ("total polos (p90)", lambda b: np.percentile([x["ms"] for x in b], 90))):
        print(f"    {label:<22} " + "".join(f"{f(r['hasil'][j]):9.2f} ms{'':>15}" for j in jenis))
    for j in jenis:
        b = r["hasil"][j]
        oleh = {}
        for x in b:
            oleh[x["oleh"]] = oleh.get(x["oleh"], 0) + 1
        tp, tu = np.mean([x["ms"] for x in b]), np.mean([x["ms_terukur"] for x in b])
        beda = sum(not x["sama"] for x in b)
        print(f"    {j:<11} tiba {100 * np.mean([x['sukses'] for x in b]):5.1f}%  {oleh}  "
              f"overhead pengukur {100 * (tu / tp - 1):+.0f}%"
              + (f"  [{beda} rute hasil polos/terukur berbeda - batas waktu]" if beda else ""))


def main():
    ap = argparse.ArgumentParser(description="Rincian waktu per tahap: Hybrid A* vs DQfD")
    ap.add_argument("--peta", default="Kalibrasi,Rumah2,Rumah", help="dipisah koma")
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--n", type=int, default=1000, help="rute maksimum per robot per peta")
    ap.add_argument("--perencana", default="astar,dqfd-murni")
    args = ap.parse_args()
    perencana = [j.strip() for j in args.perencana.split(",") if j.strip()]
    semua = []
    for peta in [p.strip() for p in args.peta.split(",") if p.strip()]:
        for rid in [int(t) for t in args.id.split(",") if t.strip()]:
            r = ukur(rid, peta, args.n, perencana)
            cetak(r)
            semua.append(r)
            sys.stdout.flush()
    keluar = os.path.join(AKAR, "model_dqn", "rincian_komputasi.json")
    with open(keluar, "w", encoding="utf-8") as f:
        json.dump(semua, f, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print(f"\n    -> {keluar}")


if __name__ == "__main__":
    main()
