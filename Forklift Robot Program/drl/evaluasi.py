"""
evaluasi.py - DQfD dua model (jauh + dekat) vs Hybrid A* + Reeds-Shepp pada SELURUH rute uji.

    python -m drl.evaluasi --id 1,2
    python -m drl.evaluasi --id 1 --model-jauh model_dqn/id01_jauh/ckpt_0050000.pt

Rute uji:
    penuh       episode UJI demo_jauh: start di luar radius serah, rute lengkap
                model jauh -> serah -> model dekat
    dekat saja  episode UJI demo_dekat_tambahan (bila ada): start di dalam
                radius, hanya model dekat

Tiap rute direncanakan dengan tiga metode (dipakai untuk KEDUA model), satu
per satu supaya waktunya sebanding, tanpa batas waktu:
    mundur      greedy berurutan Q, mundur saat buntu
    terbaik     pencarian terbaik-dulu f = g + c + 2 x h
    gabungan    terbaik-dulu, mundur bila gagal (metode operasional DQfDPlanner)
    A* + RS     HybridAStarPlanner dengan batas waktu 2 s seperti di robot

Rasio biaya = biaya jalur DQfD (jauh + dekat) / biaya demo A* + RS rute itu.
Galat pose akhir: A* + RS = 0 (Reeds-Shepp eksak); model dekat <= 1 cm & 2 deg.

Skenario NEGATIF (A* "tidak ada jalur"): DQfD seharusnya buntu. Bila ternyata
menemukan jalur, jalurnya diperiksa ulang secara independen.

Keluaran: model_dqn/evaluasi_id0N.json (+ png).
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                                   # noqa: E402
from drl import lingkungan as L                # noqa: E402
from drl import rollout as RO                   # noqa: E402
from drl import skenario as S                   # noqa: E402
from drl.adaptor_demo import rencanakan_terekam  # noqa: E402
from drl.dqfd import muat_jaringan, periksa_profil  # noqa: E402
from drl.perencana_dqfd import jalur_model, mutu_jalur, rencanakan_dua  # noqa: E402

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BATAS_WAKTU_ASTAR = 2.0
MODE_DQFD = ("mundur", "terbaik", "gabungan")


def _jenis(p):
    return p.split(":")[0]


def periksa_jalur(env, jalur, sk):
    """Periksa jalur secara independen dari mask lingkungan. '' = sah."""
    om, v, pl = env.omap, env.v, env.planner
    if len(jalur) < 2:
        return "jalur kosong"
    for i, p in enumerate(jalur[1:], 1):
        if om.pose_margin(v, p[0], p[1], p[2]) <= 0:
            return f"pose {i} menyentuh zona"
    langkah_maks = max(pl.step / pl.n_sub, pl.sample_ds) + 1e-6
    for a, b in zip(jalur[:-1], jalur[1:]):
        if math.hypot(b[0] - a[0], b[1] - a[1]) > langkah_maks:
            return "pose tidak tersambung"
    x, y, yaw = jalur[-1][:3]
    if not env._tiba(x, y, yaw):
        return "tidak tiba di tujuan (toleransi model dekat)"
    return ""


def _stat(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return {"n": 0}
    a = np.asarray(xs, float)
    return {"n": len(a), "rata": float(a.mean()), "median": float(np.median(a)),
            "p90": float(np.percentile(a, 90)), "maks": float(a.max())}


def rute_uji(rid, data):
    """[(Skenario, 'penuh'/'dekat')] dari episode uji dataset."""
    out = []
    p = os.path.join(data, f"demo_jauh_id{rid:02d}.npz")
    if os.path.exists(p):
        d = RO.muat_npz(p)
        out += [(RO.dari_demo(d, e), "penuh") for e in np.flatnonzero(d["ep_uji"])]
    p = os.path.join(data, f"demo_dekat_tambahan_id{rid:02d}.npz")
    if os.path.exists(p):
        d = RO.muat_npz(p)
        out += [(RO.dari_demo(d, e), "dekat") for e in np.flatnonzero(d["ep_uji"])]
    return out


def evaluasi_robot(rid, args):
    prof = konfig.profil(rid)
    jj = args.model_jauh or jalur_model(rid, "jauh", args.keluar)
    jd = args.model_dekat or jalur_model(rid, "dekat", args.keluar)
    net_j, meta_j = muat_jaringan(jj, jenis="jauh")
    net_d, meta_d = muat_jaringan(jd, jenis="dekat")
    periksa_profil(meta_j, prof, jj)
    periksa_profil(meta_d, prof, jd)
    pabrik = RO.PabrikEnv(prof, "jauh")
    uji = rute_uji(rid, args.data)
    if not uji:
        raise SystemExit(f"[!] Tidak ada rute uji ID {rid:02d} di {args.data}")
    neg = RO.muat_npz(os.path.join(args.data, f"negatif_id{rid:02d}.npz")) \
        if os.path.exists(os.path.join(args.data, f"negatif_id{rid:02d}.npz")) else None
    negatif = []
    if neg is not None:
        ok = np.flatnonzero(neg["alasan"] == "tidak ada jalur")
        pilih = ok[np.linspace(0, len(ok) - 1, min(args.n_neg, len(ok))).astype(int)] if len(ok) else []
        negatif = [RO.dari_negatif(neg, i) for i in pilih]
    print(f"\n=== ID {rid:02d}: jauh {os.path.relpath(jj, AKAR)} (langkah {meta_j['langkah']}), dekat "
          f"{os.path.relpath(jd, AKAR)} (langkah {meta_d['langkah']}), {len(uji)} rute uji "
          f"({sum(j == 'dekat' for _, j in uji)} dekat saja), {len(negatif)} negatif")

    def lingkungan(sk):
        a = pabrik.arena(sk.peta)
        return (L.LingkunganRencana(a, prof, margin_cm=sk.margin, jenis="jauh"),
                L.LingkunganRencana(a, prof, margin_cm=sk.margin, jenis="dekat"))

    baris = []
    t0 = time.time()
    for k, (sk, asal) in enumerate(uji):
        cm = pabrik.arena(sk.peta).panjang_ke_cm(1.0)
        om = pabrik.omap(sk)
        acuan = sk.biaya_astar_total
        r = {"peta": sk.peta, "jenis": _jenis(sk.peta), "asal": asal, "biaya_astar": acuan}
        for mode in MODE_DQFD:
            ej, ed = lingkungan(sk)
            # reset di dalam rencanakan_dua ikut diukur: ia menghitung heuristik
            # Dijkstra (~3 ms), yang pada Hybrid A* juga terhitung di dalam plan().
            h = rencanakan_dua(net_j, net_d, ej, ed, sk.awal, sk.tujuan, om, metode=mode)
            x = {"sukses": h["sukses"], "fase": h["fase"], "alasan": h.get("alasan", ""),
                 "ms": 1000 * h["waktu_s"], "n_serah": h["n_serah"],
                 "biaya": h["biaya"] if h["sukses"] else None,
                 "rasio": h["biaya"] / acuan if h["sukses"] and acuan else None,
                 "metode_jauh": h["metode_jauh"], "metode_dekat": h["metode_dekat"]}
            if h["sukses"] and h["jalur"]:
                x.update(mutu_jalur(h["jalur"], sk.tujuan, ej.radius_serah, cm))
                x["periksa"] = periksa_jalur(ed, h["jalur"], sk)
            r[mode] = x
        ej, _ = lingkungan(sk)
        pl = ej.planner
        pl.time_limit = BATAS_WAKTU_ASTAR
        t1 = time.perf_counter()
        jalur, rek = rencanakan_terekam(pl, om, sk.awal, sk.tujuan)
        r["astar"] = {"sukses": bool(jalur), "ms": 1000 * (time.perf_counter() - t1),
                      "biaya": rek.biaya, "alasan": rek.stats.get("reason", "")}
        if jalur:
            r["astar"].update(mutu_jalur(jalur, sk.tujuan, ej.radius_serah, cm))
        baris.append(r)
        if (k + 1) % 50 == 0:
            print(f"    {k + 1}/{len(uji)} rute ({time.time() - t0:.0f} s)")

    hasil_neg, contoh = [], []
    for sk in negatif:
        ej, ed = lingkungan(sk)
        h = rencanakan_dua(net_j, net_d, ej, ed, sk.awal, sk.tujuan, pabrik.omap(sk), metode="gabungan",
                           batas_waktu=args.batas_negatif)
        x = {"jenis": _jenis(sk.peta), "sukses": h["sukses"], "alasan": h.get("alasan", ""),
             "ms": 1000 * h["waktu_s"]}
        if h["sukses"] and h["jalur"]:
            x["periksa"] = periksa_jalur(ed, h["jalur"], sk)
            if len(contoh) < 6:
                contoh.append((sk, h["jalur"], ed))
        hasil_neg.append(x)

    ring = ringkas(baris, hasil_neg)
    ring.update(robot=rid, model_jauh=os.path.relpath(jj, AKAR), model_dekat=os.path.relpath(jd, AKAR),
                langkah_jauh=meta_j["langkah"], langkah_dekat=meta_d["langkah"], waktu_eval_s=time.time() - t0)
    cetak(ring)
    keluar = os.path.join(args.keluar, f"evaluasi_id{rid:02d}.json")
    with open(keluar, "w", encoding="utf-8") as f:
        # default: angka numpy (float32 dari distance transform ikut masuk ke biaya)
        json.dump({"ringkasan": ring, "rute": baris, "negatif": hasil_neg}, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print(f"    -> {keluar}")
    if contoh:
        gambar_contoh(contoh, os.path.join(args.keluar, f"negatif_ditemukan_id{rid:02d}.png"))
    return ring


def ringkas(baris, neg):
    out = {"per_jenis": {}, "per_asal": {}}
    kelompok = [("semua", None, None)] + [("per_jenis", j, "jenis") for j in sorted({r["jenis"] for r in baris})] \
        + [("per_asal", a, "asal") for a in sorted({r["asal"] for r in baris})]
    for tempat, j, kunci in kelompok:
        b = [r for r in baris if kunci is None or r[kunci] == j]
        if not b:
            continue
        blok = {"n": len(b)}
        for mode in (*MODE_DQFD, "astar"):
            tiba = [r for r in b if r[mode]["sukses"]]
            m = {"tiba_pct": 100.0 * len(tiba) / len(b), "ms": _stat([r[mode]["ms"] for r in b])}
            for k in ("ganti_arah", "ganti_arah_dekat", "galat_posisi_cm", "galat_arah_deg"):
                m[k] = _stat([r[mode].get(k) for r in tiba])
            if mode != "astar":
                m["gagal_jauh_pct"] = 100.0 * sum(r[mode]["fase"] == "gagal_jauh" for r in b) / len(b)
                m["gagal_dekat_pct"] = 100.0 * sum(r[mode]["fase"] == "gagal_dekat" for r in b) / len(b)
                m["rasio"] = _stat([r[mode]["rasio"] for r in b])
                m["serah_dicoba"] = _stat([r[mode]["n_serah"] for r in b])
                m["tidak_sah"] = sum(1 for r in tiba if r[mode].get("periksa"))
            blok[mode] = m
        if tempat == "semua":
            out["semua"] = blok
        else:
            out[tempat][j] = blok
    ditemukan = [r for r in neg if r["sukses"]]
    out["negatif"] = {
        "n": len(neg), "buntu_pct": 100.0 * sum(not r["sukses"] for r in neg) / max(1, len(neg)),
        "ditemukan": len(ditemukan),
        "ditemukan_sah": sum(1 for r in ditemukan if r.get("periksa") == ""),
        "galat_periksa": [r["periksa"] for r in ditemukan if r.get("periksa")],
        "ms_menyerah": _stat([r["ms"] for r in neg if not r["sukses"]]),
    }
    return out


def cetak(r):
    g = lambda s, k="rata": s.get(k, float("nan"))  # noqa: E731
    print("    peta          n    TIBA mundur/terbaik/GABUNGAN/A*   gagal jauh/dekat (gab.)   rasio p90 "
          "mun|terb|GAB   balik total/<60cm GAB | A*   galat akhir GAB cm/deg (p90)   ms med/p90 GAB | A*")
    daftar = [("SEMUA", r["semua"]), *r["per_jenis"].items(), *[(f"[{a}]", b) for a, b in r["per_asal"].items()]]
    for j, b in daftar:
        m, t, c, a = b["mundur"], b["terbaik"], b["gabungan"], b["astar"]
        print(f"    {j:<11} {b['n']:>4}   {m['tiba_pct']:5.1f} / {t['tiba_pct']:5.1f} / {c['tiba_pct']:5.1f} / "
              f"{a['tiba_pct']:5.1f}       {c['gagal_jauh_pct']:4.1f} / {c['gagal_dekat_pct']:4.1f}            "
              f"{g(m['rasio'], 'p90'):5.2f}|{g(t['rasio'], 'p90'):5.2f}|{g(c['rasio'], 'p90'):5.2f}     "
              f"{g(c['ganti_arah']):4.2f} / {g(c['ganti_arah_dekat']):4.2f} | {g(a['ganti_arah']):4.2f} / "
              f"{g(a['ganti_arah_dekat']):4.2f}     {g(c['galat_posisi_cm']):4.2f} / {g(c['galat_arah_deg']):4.2f} "
              f"({g(c['galat_arah_deg'], 'p90'):4.2f})       {g(c['ms'], 'median'):5.0f} / {g(c['ms'], 'p90'):5.0f} | "
              f"{g(a['ms'], 'median'):4.0f} / {g(a['ms'], 'p90'):5.0f}")
    tidak = {m: r["semua"][m]["tidak_sah"] for m in MODE_DQFD}
    if any(tidak.values()):
        print(f"    [!] jalur DQfD yang tiba tapi TIDAK sah menurut pemeriksaan independen: {tidak}")
    n = r["negatif"]
    print(f"    negatif: {n['n']} skenario, buntu {n['buntu_pct']:.0f}%, jalur ditemukan {n['ditemukan']} "
          f"(sah menurut pemeriksaan independen: {n['ditemukan_sah']}), "
          f"menyerah median {n['ms_menyerah'].get('median', 0):.0f} ms")
    if n["galat_periksa"]:
        print(f"    [!] jalur negatif TIDAK sah: {n['galat_periksa'][:5]}")


def gambar_contoh(contoh, keluar):
    img = []
    for sk, jalur, env in contoh:
        a = env.arena
        im = cv2.cvtColor(np.where(env.omap.dist < 1, 60, 235).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        for p, q in zip(jalur[:-1], jalur[1:]):
            cv2.line(im, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])),
                     (0, 170, 0) if q[3] > 0 else (0, 140, 255), 2, cv2.LINE_AA)
        cv2.polylines(im, [env.v.footprint(*sk.awal)], True, (255, 90, 0), 2)
        gyaw = sk.tujuan[2] if sk.tujuan[2] is not None else jalur[-1][2]
        cv2.polylines(im, [env.v.footprint(sk.tujuan[0], sk.tujuan[1], gyaw)], True, (0, 0, 220), 2)
        cv2.putText(im, f"{sk.peta} m{sk.margin:.0f} {sk.info.get('alasan', '')}", (20, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
        img.append(cv2.resize(im, (a.lebar_sel, a.tinggi_sel)))
    while len(img) % 3:
        img.append(np.full_like(img[0], 255))
    cv2.imwrite(keluar, np.vstack([np.hstack(img[i:i + 3]) for i in range(0, len(img), 3)]))
    print(f"    -> {keluar}")


def main():
    ap = argparse.ArgumentParser(description="Evaluasi DQfD dua model vs Hybrid A* + RS pada seluruh rute uji")
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--model-jauh", default=None, help="checkpoint; bawaan model_dqn/id0N_jauh.pt")
    ap.add_argument("--model-dekat", default=None, help="checkpoint; bawaan model_dqn/id0N_dekat.pt")
    ap.add_argument("--n-neg", type=int, default=150)
    ap.add_argument("--batas-negatif", type=float, default=5.0,
                    help="batas waktu (s) per skenario negatif - tanpa batas, pencarian buntu bisa lama")
    ap.add_argument("--data", default=S.DIR_DATA)
    ap.add_argument("--keluar", default=os.path.join(AKAR, "model_dqn"))
    args = ap.parse_args()
    for rid in [int(t) for t in args.id.split(",") if t.strip()]:
        evaluasi_robot(rid, args)


if __name__ == "__main__":
    main()
