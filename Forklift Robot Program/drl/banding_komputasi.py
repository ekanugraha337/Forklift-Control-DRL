"""
banding_komputasi.py - performa KOMPUTASI perencana: Hybrid A* vs DQfD.

    python -m drl.banding_komputasi                          # semua peta, ID 01 & 02
    python -m drl.banding_komputasi --peta Kalibrasi --ulang 5
    python -m drl.banding_komputasi --perencana astar,dqfd-murni

Adil karena:
  - rute SAMA PERSIS untuk semua perencana: rute uji dataset di peta itu,
    lengkap dengan margin dan badan robot lain sebagai rintangan bila ada;
  - lewat jalur kode yang SAMA dengan navigasi.py: Perencana.rencanakan(),
    termasuk mundur ke margin cadangan;
  - tiap rute direncanakan --ulang kali, yang dilaporkan MEDIANNYA (derau
    penjadwal Windows), setelah pemanasan. Memuat model PyTorch dan
    perencanaan pertama dilaporkan terpisah - itu ongkos sekali saat
    navigasi.py dimulai, bukan ongkos tiap rencana.

Yang diukur per rute: waktu jam (ms), berhasil,
perencana yang akhirnya menghasilkan jalur, dan rinciannya:
    Hybrid A*   jumlah ekspansi simpul
    DQfD        langkah pencarian, forward jaringan, dan porsi waktu di jaringan
                vs sisanya (pemeriksaan tabrakan, Reeds-Shepp, observasi)

Perencana yang dibandingkan:
    astar       Hybrid A* (bawaan navigasi.py)
    dqfd        DQfD terbaik-dulu + gerbang mutu + Hybrid A* cadangan (operasional)
    dqfd-murni  DQfD saja - komputasi DQfD sendirian

Jangan jalankan bersamaan dengan pelatihan/uji lain: yang diukur waktu.
Keluaran: model_dqn/banding_komputasi_<peta>.json dan .png
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import arena as A                   # noqa: E402
import konfig                       # noqa: E402
import rencana as R                 # noqa: E402
from drl import rollout as RO       # noqa: E402

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PERENCANA = ("astar", "dqfd", "dqfd-murni")


def rute_uji(rid, peta, n):
    """Rute uji dataset di peta itu, dalam cm seperti yang diterima navigasi.py."""
    d = RO.muat_npz(os.path.join(AKAR, "drl", "data", f"demo_jauh_id{rid:02d}.npz"))
    ar = A.Arena(peta)
    idx = [e for e in np.flatnonzero(d["ep_uji"]) if d["ep_peta"][e] == peta][:n]
    out = []
    for e in idx:
        a = ar.pose_ke_cm(*d["ep_awal"][e])
        t = d["ep_tujuan"][e]
        tc = ar.pose_ke_cm(t[0], t[1], 0.0 if np.isnan(t[2]) else t[2])
        poli = d["ep_rintangan"][e]
        out.append({"e": int(e), "awal": a, "tujuan": (tc[0], tc[1], None if np.isnan(t[2]) else tc[2]),
                    "rintangan": [] if np.isnan(poli).any() else [poli],
                    "margin": float(d["ep_margin"][e]), "biaya_astar": float(d["ep_biaya_astar"][e])})
    return ar, out


class PenghitungForward:
    """Membungkus forward jaringan untuk menghitung jumlah & waktunya."""

    def __init__(self, net):
        self.net, self.n, self.t = net, 0, 0.0
        self._asli = net.forward

        def forward(*a, **k):
            t0 = time.perf_counter()
            r = self._asli(*a, **k)
            self.t += time.perf_counter() - t0
            self.n += 1
            return r
        net.forward = forward

    def nol(self):
        self.n, self.t = 0, 0.0


def ukur(rid, peta, n, ulang, perencana=PERENCANA):
    ar, rute = rute_uji(rid, peta, n)
    prof = konfig.profil(rid)
    hasil = {}
    awal_mula = {}
    for jenis in perencana:
        t0 = time.perf_counter()
        per = R.Perencana(ar, prof, jenis=jenis)
        per._untuk(konfig.SAFETY_MARGIN_CM)          # bangun perencana (+ muat model)
        t_bangun = time.perf_counter() - t0
        hit = None
        if jenis != "astar":
            from drl.perencana_dqfd import muat_model
            hit = PenghitungForward(muat_model(rid))
        # pemanasan: rute pertama, tidak dihitung
        t0 = time.perf_counter()
        per.rencanakan(rute[0]["awal"], rute[0]["tujuan"], rute[0]["rintangan"])
        awal_mula[jenis] = {"bangun_ms": 1000 * t_bangun, "rencana_pertama_ms": 1000 * (time.perf_counter() - t0)}

        baris = []
        for r in rute:
            jam, cpu, simpan = [], [], None
            for _ in range(ulang):
                if hit:
                    hit.nol()
                c0, t0 = time.process_time(), time.perf_counter()
                jalur, info = per.rencanakan(r["awal"], r["tujuan"], r["rintangan"])
                jam.append(time.perf_counter() - t0)
                cpu.append(time.process_time() - c0)
                simpan = (jalur, info, dict(per.stats), (hit.n, hit.t) if hit else (0, 0.0))
            jalur, info, st, (n_fwd, t_fwd) = simpan
            baris.append({
                "e": r["e"], "sukses": bool(jalur), "ms": 1000 * float(np.median(jam)),
                "cpu_ms": 1000 * float(np.median(cpu)), "oleh": info.get("perencana", "A*") if jalur else "gagal",
                "panjang_cm": info.get("panjang_cm", 0.0), "margin": info.get("margin_cm"),
                "ekspansi": int(st.get("expansions", 0)), "forward": n_fwd,
                "porsi_jaringan": (t_fwd / max(1e-9, jam[-1])) if n_fwd else 0.0,
            })
        hasil[jenis] = baris
        per = None
    return {"robot": rid, "peta": peta, "n": len(rute), "ulang": ulang, "awal": awal_mula, "hasil": hasil}


def _p(x, q):
    return float(np.percentile(x, q)) if len(x) else float("nan")


def cetak(r):
    print(f"\n=== ID {r['robot']:02d} di {r['peta']}: {r['n']} rute uji, tiap rute {r['ulang']}x (median)")
    # Waktu CPU proses tidak dicetak: di Windows jamnya berdetak 15,6 ms, jadi
    # untuk rencana 5-20 ms angkanya hanya 0 atau 15,6. Tetap tersimpan di JSON.
    print("    perencana    tiba    ms median   rata    p90     maks   | kerja median"
          "                 | oleh")
    for jenis, b in r["hasil"].items():
        ms = np.array([x["ms"] for x in b])
        if jenis == "astar":
            kerja = f"{np.median([x['ekspansi'] for x in b]):6.0f} ekspansi           "
        else:
            dq = [x for x in b if x["oleh"] == "dqfd"]
            kerja = (f"{np.median([x['ekspansi'] for x in dq]) if dq else 0:5.0f} langkah, "
                     f"{np.median([x['forward'] for x in dq]) if dq else 0:4.0f} fwd, "
                     f"jaringan {100 * np.median([x['porsi_jaringan'] for x in dq]) if dq else 0:3.0f}%")
        oleh = {}
        for x in b:
            oleh[x["oleh"]] = oleh.get(x["oleh"], 0) + 1
        print(f"    {jenis:<11} {100 * np.mean([x['sukses'] for x in b]):5.1f}%  {np.median(ms):8.1f} {ms.mean():7.1f}"
              f" {_p(ms, 90):7.1f} {ms.max():8.1f}  | {kerja} | {oleh}")
    a = np.array([x["ms"] for x in r["hasil"]["astar"]])
    for jenis in [j for j in r["hasil"] if j != "astar"]:
        d = np.array([x["ms"] for x in r["hasil"][jenis]])
        rasio = a / np.maximum(d, 1e-3)
        print(f"    A* / {jenis:<10}: median {np.median(rasio):5.2f}x  (>1 = DQfD lebih cepat), "
              f"DQfD lebih cepat di {100 * np.mean(d < a):4.1f}% rute, "
              f"total waktu {d.sum() / 1000:.2f} s vs A* {a.sum() / 1000:.2f} s")
    print("    ongkos awal (sekali saat navigasi.py mulai): " + ", ".join(
        f"{j} bangun {v['bangun_ms']:.0f} ms + rencana pertama {v['rencana_pertama_ms']:.0f} ms"
        for j, v in r["awal"].items()))


def gambar(semua, keluar, pembanding):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, len(semua), figsize=(5.2 * len(semua), 4.6), squeeze=False)
    for k, r in enumerate(semua):
        x = np.array([v["ms"] for v in r["hasil"]["astar"]])
        y = np.array([v["ms"] for v in r["hasil"][pembanding]])
        ok = np.array([v["oleh"] == "dqfd" for v in r["hasil"][pembanding]])
        g = ax[0, k]
        g.scatter(x[ok], y[ok], s=14, alpha=0.7, label="jalur oleh DQfD")
        g.scatter(x[~ok], y[~ok], s=18, marker="x", color="C3", label="Hybrid A* cadangan / gagal")
        lo, hi = max(0.3, min(x.min(), y.min()) * 0.8), max(x.max(), y.max()) * 1.2
        g.plot([lo, hi], [lo, hi], color="0.5", lw=1, ls="--", label="sama cepat")
        g.set_xscale("log")
        g.set_yscale("log")
        g.set_xlim(lo, hi)
        g.set_ylim(lo, hi)
        g.set_xlabel("Hybrid A* (ms)")
        g.set_ylabel(f"DQfD - {pembanding} (ms)")
        g.set_title(f"ID {r['robot']:02d} - {r['peta']} ({r['n']} rute)\ndi bawah garis = DQfD lebih cepat")
        g.legend(fontsize=8, loc="upper left")
        g.grid(True, which="both", alpha=0.2)
    fig.tight_layout()
    fig.savefig(keluar, dpi=120)
    print(f"\n    -> {keluar}")


def main():
    ap = argparse.ArgumentParser(description="Performa komputasi Hybrid A* vs DQfD")
    ap.add_argument("--peta", default=None,
                    help="satu atau beberapa, dipisah koma; bawaan semua di Camera/peta_constraint/")
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--n", type=int, default=1000, help="rute maksimum per robot per peta")
    ap.add_argument("--ulang", type=int, default=3, help="pengulangan per rute (diambil median)")
    ap.add_argument("--perencana", default=",".join(PERENCANA),
                    help="yang dibandingkan; astar selalu ikut sebagai acuan")
    args = ap.parse_args()
    perencana = ["astar"] + [j for j in args.perencana.split(",") if j.strip() and j != "astar"]
    pembanding = perencana[1]
    ringkas = []
    if args.peta:
        daftar = [p.strip() for p in args.peta.split(",") if p.strip()]
    else:
        konfig.pasang_path_kamera()
        import area_constraint as AC
        daftar = AC.daftar_peta()
    for peta in daftar:
        semua = []
        for rid in [int(t) for t in args.id.split(",") if t.strip()]:
            r = ukur(rid, peta, args.n, args.ulang, perencana)
            cetak(r)
            semua.append(r)
            ringkas.append(r)
        akhiran = "" if len(perencana) == len(PERENCANA) else "_" + "_".join(perencana[1:])
        dasar = os.path.join(AKAR, "model_dqn", f"banding_komputasi_{peta}{akhiran}")
        with open(dasar + ".json", "w", encoding="utf-8") as f:
            json.dump(semua, f, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
        gambar(semua, dasar + ".png", pembanding)

    print(f"\n=== RINGKASAN: Hybrid A* vs {pembanding} (waktu median / p90 / maks, ms)")
    print("    robot peta        rute  tiba A* / DQfD     A* med/p90/maks          DQfD med/p90/maks"
          "        DQfD lebih cepat  total A* / DQfD")
    for r in ringkas:
        a = np.array([x["ms"] for x in r["hasil"]["astar"]])
        d = np.array([x["ms"] for x in r["hasil"][pembanding]])
        ta = 100 * np.mean([x["sukses"] for x in r["hasil"]["astar"]])
        td = 100 * np.mean([x["sukses"] for x in r["hasil"][pembanding]])
        print(f"    ID {r['robot']:02d} {r['peta']:<10} {r['n']:>4}  {ta:5.1f} / {td:5.1f}%   "
              f"{np.median(a):6.1f} /{_p(a, 90):7.1f} /{a.max():7.0f}    {np.median(d):6.1f} /{_p(d, 90):7.1f} /{d.max():7.0f}"
              f"      {100 * np.mean(d < a):5.1f}%       {a.sum() / 1000:6.1f} / {d.sum() / 1000:6.1f} s")


if __name__ == "__main__":
    main()
