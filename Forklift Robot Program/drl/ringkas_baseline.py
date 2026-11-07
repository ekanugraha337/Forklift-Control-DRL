"""
ringkas_baseline.py - ringkasan hasil drl.banding_baseline (Tahap 1.1).

    python -m drl.ringkas_baseline model_dqn/banding_baseline_<tanggal>.json [--json keluar.json]

Per robot dan varian, pada semua rute uji dan pada tiga peta (Kalibrasi,
Rumah2, Rumah): tiba, waktu (median / p90 / rata-rata), node (median / p90),
rasio biaya terhadap A0 (rute yang dua-duanya tiba), ganti arah, dan seberapa
sering D0 lebih cepat. Lalu D0 vs varian Hybrid A* per sudut heading awal.
"""
import argparse
import json
import sys

import numpy as np

TIGA = ("Kalibrasi", "Rumah2", "Rumah")
LABEL = {
    "A0": "Hybrid A*: Dijkstra, w 1.3 (robot)",
    "A1": "Hybrid A*: Dijkstra, w 2.0",
    "A2": "Hybrid A*: max(RS, Dijkstra), w 1.3",
    "A3": "Hybrid A*: max(RS, Dijkstra), w 2.0",
    "A4": "Hybrid A*: max(RS, Dijkstra), w 2.0, filter",
    "D0": "DQfD: w 2.0, filter, backtracking (robot)",
    "D1": "DQfD: w 1.3",
    "D2": "DQfD: tanpa filter",
    "D3": "DQfD: tanpa backtracking",
}


def _p(x, q):
    return float(np.percentile(x, q)) if len(x) else float("nan")


def statistik(rute, varian, rid, pilih):
    idx = {(h["peta"], h["e"]): h for h in rute if h["robot"] == rid and h["varian"] == varian and pilih(h)}
    ref = {(h["peta"], h["e"]): h for h in rute if h["robot"] == rid and h["varian"] == "A0" and pilih(h)}
    d0 = {(h["peta"], h["e"]): h for h in rute if h["robot"] == rid and h["varian"] == "D0" and pilih(h)}
    xs = list(idx.values())
    if not xs:
        return None
    ms = np.array([h["ms"] for h in xs])
    nd = np.array([h["node"] for h in xs])
    ok = [h for h in xs if h["sukses"]]
    rasio = [h["biaya"] / ref[k]["biaya"] for k, h in idx.items()
             if h["sukses"] and k in ref and ref[k]["sukses"] and ref[k]["biaya"] > 0]
    kunci = [k for k in idx if k in d0]
    lebih_cepat = [d0[k]["ms"] < idx[k]["ms"] for k in kunci]
    rasio_ms = [d0[k]["ms"] / idx[k]["ms"] for k in kunci if d0[k]["sukses"] and idx[k]["sukses"]]
    return {
        "n": len(xs), "tiba": 100.0 * len(ok) / len(xs),
        "ms_median": float(np.median(ms)), "ms_p90": _p(ms, 90), "ms_mean": float(ms.mean()),
        "node_median": float(np.median(nd)), "node_p90": _p(nd, 90), "node_mean": float(nd.mean()),
        "rasio_biaya_median": float(np.median(rasio)) if rasio else None,
        "rasio_biaya_p90": _p(rasio, 90) if rasio else None,
        "ganti_arah": float(np.mean([h["ganti_arah"] for h in ok])) if ok else None,
        "ganti_arah_luar60": float(np.mean([h["ganti_arah_luar60"] for h in ok])) if ok else None,
        "galat_pos_maks_cm": max((h["galat_pos_cm"] for h in ok), default=None),
        "ms_per_node_median": float(np.median([h["ms"] / h["node"] for h in ok if h["node"] > 0]))
        if ok else None,
        "gagal": sorted(f"{h['peta']} e{h['e']}" for h in xs if not h["sukses"]),
        "d0_lebih_cepat_pct": 100.0 * float(np.mean(lebih_cepat)) if kunci else None,
        "d0_rasio_ms_median": float(np.median(rasio_ms)) if rasio_ms else None,
        "d0_rasio_ms_p90": _p([d0[k]["ms"] for k in kunci], 90) / _p([idx[k]["ms"] for k in kunci], 90)
        if kunci else None,
    }


def per_sudut(rute, varian, rid):
    """D0 vs varian: rute yang dua-duanya tiba, per kelompok sudut heading awal ke arah tujuan."""
    a = {(h["peta"], h["e"]): h for h in rute if h["robot"] == rid and h["varian"] == varian}
    d = {(h["peta"], h["e"]): h for h in rute if h["robot"] == rid and h["varian"] == "D0"}
    kunci = [k for k in a if k in d and a[k]["sukses"] and d[k]["sukses"]]
    kel = {
        "135-180 & >=170 cm": lambda h: h["sudut_awal_deg"] >= 135 and h["jarak_cm"] >= 170,
        "45-135": lambda h: 45 <= h["sudut_awal_deg"] < 135,
        "0-45": lambda h: h["sudut_awal_deg"] < 45,
        "semua": lambda h: True,
    }
    out = {}
    for nama, f in kel.items():
        ks = [k for k in kunci if f(a[k])]
        if not ks:
            continue
        r = [d[k]["ms"] / a[k]["ms"] for k in ks]
        out[nama] = {"n": len(ks), "d0_lebih_cepat_pct": 100.0 * float(np.mean([x < 1 for x in r])),
                     "rasio_ms_median": float(np.median(r)),
                     "node_a_median": float(np.median([a[k]["node"] for k in ks])),
                     "node_d0_median": float(np.median([d[k]["node"] for k in ks]))}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("berkas")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()
    data = json.load(open(args.berkas, encoding="utf-8"))
    rute = data["rute"]
    varian = [v for v in LABEL if any(h["varian"] == v for h in rute)]
    robot = sorted({h["robot"] for h in rute})
    hasil = {"meta": data.get("meta", {}), "semua": {}, "tiga": {}, "sudut": {}}
    for judul, kunci, pilih in (("SEMUA RUTE UJI", "semua", lambda h: True),
                                ("TIGA PETA", "tiga", lambda h: h["peta"] in TIGA)):
        print(f"\n=== {judul}")
        print(f"{'':4}{'varian':<44}{'n':>4}{'tiba':>7}{'ms med':>8}{'p90':>7}{'mean':>7}"
              f"{'node med':>9}{'p90':>6}{'biaya med':>10}{'p90':>6}{'g.arah':>7}{'tengah':>7}"
              f"{'D0 lebih cepat':>15}{'rasio med':>10}{'rasio p90':>10}")
        for rid in robot:
            for v in varian:
                s = statistik(rute, v, rid, pilih)
                if s is None:
                    continue
                hasil[kunci][f"ID{rid:02d} {v}"] = s
                f = lambda x, fmt: (fmt % x) if x is not None else "-"   # noqa: E731
                print(f"ID{rid:02d} {LABEL[v]:<44}{s['n']:>4}{s['tiba']:>7.1f}{s['ms_median']:>8.1f}"
                      f"{s['ms_p90']:>7.0f}{s['ms_mean']:>7.0f}{s['node_median']:>9.0f}{s['node_p90']:>6.0f}"
                      f"{f(s['rasio_biaya_median'], '%.3f'):>10}{f(s['rasio_biaya_p90'], '%.2f'):>6}"
                      f"{f(s['ganti_arah'], '%.2f'):>7}{f(s['ganti_arah_luar60'], '%.2f'):>7}"
                      f"{f(s['d0_lebih_cepat_pct'], '%.0f%%'):>15}{f(s['d0_rasio_ms_median'], '%.2f'):>10}"
                      f"{f(s['d0_rasio_ms_p90'], '%.2f'):>10}")
    print("\n=== ms per node (median per rute) dan rute gagal, semua rute uji")
    for k, s_ in hasil["semua"].items():
        print(f"{k}: {s_['ms_per_node_median']:.3f} ms/node; gagal {len(s_['gagal'])}: {', '.join(s_['gagal'])}")
    print("\n=== D0 vs Hybrid A* per sudut heading awal (rute yang dua-duanya tiba)")
    for rid in robot:
        for v in [x for x in varian if x.startswith("A")]:
            ps = per_sudut(rute, v, rid)
            hasil["sudut"][f"ID{rid:02d} {v}"] = ps
            print(f"ID{rid:02d} vs {v}: " + "  |  ".join(
                f"{k}: n {x['n']}, D0 lebih cepat {x['d0_lebih_cepat_pct']:.0f}%, rasio {x['rasio_ms_median']:.2f}, "
                f"node {x['node_a_median']:.0f}/{x['node_d0_median']:.0f}" for k, x in ps.items()))
    if args.json:
        json.dump(hasil, open(args.json, "w", encoding="utf-8"), indent=1)
        print(f"[OK] {args.json}")


if __name__ == "__main__":
    sys.exit(main())
