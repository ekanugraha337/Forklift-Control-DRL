"""
ulang_rencana_pertama.py - Tahap 1.1: putar ulang RENCANA PERTAMA uji Gudang/Gudang2
dengan varian baseline, dari pose start yang benar-benar diukur kamera.

    python -m drl.ulang_rencana_pertama
    python -m drl.ulang_rencana_pertama --varian A0,A1,A3,A4,D0

Untuk tiap trial dan tiap robot: pose start = titik pertama jalur rencana
pertama yang tercatat (pose robot setelah pulang), tujuan = tujuan tercatat,
tanpa badan robot lain sebagai rintangan (rencana pertama memang dibuat tanpa
robot lain). Varian sama dengan drl.banding_baseline.

Yang dicatat: sukses, ms (median 3 ulangan), node, panjang, ganti arah seluruh
jalur dan di LUAR 60 cm dari tujuan ("manuver mundur di tengah rute"), dan
apakah jalur varian perencana yang dipakai di trial itu sama dengan rekaman
(jumlah titik sama) - pemeriksaan bahwa putar ulang setia.

Keluaran: model_dqn/ulang_rencana_pertama_<tanggal>.json
"""
import argparse
import datetime
import json
import os
import sys
import time

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import arena as A                               # noqa: E402
import konfig                                   # noqa: E402
import rencana as R                             # noqa: E402
from drl.banding_baseline import VARIAN         # noqa: E402
from drl.perencana_dqfd import hitung_ganti_arah  # noqa: E402

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

RUNS = [
    ("Gudang", "sim", "20261105_002833_Gudang_sim"),
    ("Gudang", "nyata", "20261105_010053_Gudang"),
    ("Gudang2 wasit", "sim", "20261105_061315_Gudang2_sim"),
    ("Gudang2 wasit", "nyata", "20261105_070646_Gudang2"),
    ("Gudang2 jadwal", "sim", "20261105_081257_Gudang2_sim"),
    ("Gudang2 jadwal", "nyata", "20261105_082220_Gudang2"),
]
PERENCANA_KE_VARIAN = {"astar": "A0", "dqfd-murni": "D0"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--varian", default="A0,A1,A2,A3,A4,D0")
    ap.add_argument("--ulang", type=int, default=3)
    ap.add_argument("--keluar", default=None)
    args = ap.parse_args()
    varian = args.varian.split(",")
    hasil = []
    for kelompok, mode, run in RUNS:
        d = os.path.join(AKAR, "uji_nyata", run)
        with open(os.path.join(d, "skenario.json"), encoding="utf-8") as f:
            peta = json.load(f)["peta"]
        ar = A.Arena(A.pilih(peta))
        per = {}
        with open(os.path.join(d, "hasil.jsonl"), encoding="utf-8") as f:
            catatan = [json.loads(b) for b in f if b.strip()]
        for h in catatan:
            if h.get("status") == "dilewati":
                continue
            for rid_s, r in h["robot"].items():
                rid = int(rid_s)
                calls = r.get("rencana", [])
                if not calls or not calls[0].get("jalur_cm"):
                    continue
                c = calls[0]
                awal, tujuan = tuple(c["jalur_cm"][0][:3]), tuple(c["tujuan"])
                for v in varian:
                    if (rid, v) not in per:
                        jenis, opsi = VARIAN[v]
                        p = R.Perencana(ar, konfig.profil(rid), jenis=jenis, opsi=opsi)
                        p.rencanakan(awal, tujuan, None)            # pemanasan
                        per[(rid, v)] = p
                    p = per[(rid, v)]
                    jam = []
                    for _ in range(args.ulang):
                        t0 = time.perf_counter()
                        jalur, info = p.rencanakan(awal, tujuan, None)
                        jam.append(1000.0 * (time.perf_counter() - t0))
                    baris = {"kelompok": kelompok, "mode": mode, "run": run, "skenario": h["skenario"],
                             "ulang": h["ulang"], "perencana_trial": h["perencana"], "robot": rid,
                             "varian": v, "sukses": bool(jalur), "ms": float(np.median(jam)),
                             "node": int(p.stats.get("expansions", 0) or 0),
                             "galat_awal_cm": r.get("galat_awal_cm"), "galat_awal_deg": r.get("galat_awal_deg")}
                    if jalur:
                        tj = ar.pose_ke_sel(*tujuan)
                        g_all, g_dekat = hitung_ganti_arah(jalur, tj, ar.panjang_ke_sel(60.0))
                        baris.update(panjang_cm=float(info["panjang_cm"]), ganti_arah=g_all,
                                     ganti_arah_luar60=g_all - g_dekat)
                        if PERENCANA_KE_VARIAN.get(h["perencana"]) == v:
                            baris["sama_rekaman"] = len(jalur) == len(c["jalur_cm"])
                    hasil.append(baris)
                print(f"{kelompok:<15} {mode:<5} {h['skenario']} u{h['ulang'] + 1} {h['perencana']:<10} "
                      f"ID{rid:02d}: " + "  ".join(
                          f"{b['varian']} {b.get('ganti_arah_luar60', '-')}/{b['node']}"
                          for b in hasil[-len(varian):]), flush=True)
    keluar = args.keluar or os.path.join(
        AKAR, "model_dqn", f"ulang_rencana_pertama_{datetime.date.today().isoformat()}.json")
    with open(keluar, "w", encoding="utf-8") as f:
        json.dump(hasil, f, indent=1, default=lambda o: o.item() if isinstance(o, np.generic) else str(o))
    print(f"[OK] {keluar}")


if __name__ == "__main__":
    main()
