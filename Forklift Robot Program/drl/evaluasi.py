"""
evaluasi.py - DQfD vs Hybrid A* pada SELURUH rute uji (tahap 8).

    python -m drl.evaluasi --id 1,2
    python -m drl.evaluasi --id 1 --model model_dqn/id01/ckpt_0055000.pt

Tiap rute uji direncanakan empat cara, satu per satu (bukan serempak), supaya
waktunya sebanding:
    DQfD greedy     rollout argmax-Q, berhenti di buntu pertama
    DQfD mundur     sama, tetapi saat buntu mundur ke aksi terbaik berikutnya
    DQfD terbaik    pencarian terbaik-dulu f = g + c + 2 x h (metode operasional)
    A* operasional  HybridAStarPlanner dengan batas waktu 2 s seperti di robot
                    (waktu A* di dataset diukur saat 15 proses berebut CPU)

Rasio biaya = biaya DQfD / g-cost A* dataset (tanpa batas waktu).

Skenario NEGATIF (A* "tidak ada jalur"): DQfD seharusnya buntu. Bila ternyata
menemukan jalur, jalurnya diperiksa ulang secara independen - setiap pose
bebas zona dengan margin skenario, pose berurutan tersambung, dan pose akhir
dalam toleransi tujuan - lalu digambar ke negatif_ditemukan_id0N.png.

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
from drl.perencana_dqfd import (ANGGARAN_TERBAIK, BOBOT_TERBAIK, rencanakan,  # noqa: E402
                                 rencanakan_terbaik)

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BATAS_WAKTU_ASTAR = 2.0
MODE_DQFD = ("greedy", "mundur", "terbaik")


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
    if not pl._at_goal(x, y, yaw, sk.tujuan):
        return "tidak tiba di tujuan"
    return ""


def evaluasi_robot(rid, jalur_model, args):
    prof = konfig.profil(rid)
    net, meta = muat_jaringan(jalur_model)
    periksa_profil(meta, prof, jalur_model)
    d = RO.muat_npz(os.path.join(args.data, f"demo_id{rid:02d}.npz"))
    neg = RO.muat_npz(os.path.join(args.data, f"negatif_id{rid:02d}.npz"))
    pabrik = RO.PabrikEnv(prof)
    uji = [RO.dari_demo(d, e) for e in np.flatnonzero(d["ep_uji"])]
    ok = np.flatnonzero(neg["alasan"] == "tidak ada jalur")
    pilih = ok[np.linspace(0, len(ok) - 1, min(args.n_neg, len(ok))).astype(int)] if len(ok) else []
    negatif = [RO.dari_negatif(neg, i) for i in pilih]
    print(f"\n=== ID {rid:02d}: {os.path.relpath(jalur_model, AKAR)} (langkah {meta['langkah']}), "
          f"{len(uji)} rute uji, {len(negatif)} negatif")

    baris = []
    t0 = time.time()
    for k, sk in enumerate(uji):
        r = {"peta": sk.peta, "jenis": _jenis(sk.peta), "biaya_astar": sk.biaya_astar,
             "langkah_astar": len(sk.aksi_demo)}
        for mode in MODE_DQFD:
            # reset() IKUT diukur: ia menghitung heuristik Dijkstra (~3 ms), yang
            # pada Hybrid A* juga terhitung di dalam plan(). Sampai 2026-09-30
            # reset ada di luar pengukuran, sehingga median waktu DQfD di
            # laporan sebelumnya ~3 ms terlalu kecil. Pembanding komputasi
            # yang lengkap (lewat Perencana, seperti navigasi.py): drl.banding_komputasi.
            env = L.LingkunganRencana(pabrik.arena(sk.peta), pabrik.p, margin_cm=sk.margin)
            om = pabrik.omap(sk)
            t_reset = time.perf_counter()
            env.reset(sk.awal, sk.tujuan, om)
            t_reset = time.perf_counter() - t_reset
            if mode == "terbaik":
                h = rencanakan_terbaik(net, env, anggaran=ANGGARAN_TERBAIK, bobot=BOBOT_TERBAIK)
            else:
                h = rencanakan(net, env, mundur=mode == "mundur", anggaran=args.anggaran)
            r[mode] = {"sukses": h["sukses"], "alasan": h["alasan"], "ms": 1000 * (h["waktu_s"] + t_reset),
                       "biaya": env.biaya if h["sukses"] else None,
                       "rasio": env.biaya / sk.biaya_astar if h["sukses"] else None,
                       "langkah": len(env.jalur()), "dicoba": h["dicoba"], "mundur": h["mundur"]}
        env, _, _ = pabrik.buat(sk)
        env.planner.time_limit = BATAS_WAKTU_ASTAR
        t1 = time.perf_counter()
        jalur, rek = rencanakan_terekam(env.planner, env.omap, sk.awal, sk.tujuan)
        r["astar"] = {"sukses": bool(jalur), "ms": 1000 * (time.perf_counter() - t1),
                      "biaya": rek.biaya, "alasan": rek.stats.get("reason", "")}
        baris.append(r)
        if (k + 1) % 50 == 0:
            print(f"    {k + 1}/{len(uji)} rute ({time.time() - t0:.0f} s)")

    hasil_neg, contoh = [], []
    for sk in negatif:
        env, obs, info = pabrik.buat(sk)
        if obs is None:
            hasil_neg.append({"jenis": _jenis(sk.peta), "sukses": False, "alasan": info["alasan"], "ms": 0.0})
            continue
        h = rencanakan_terbaik(net, env, anggaran=ANGGARAN_TERBAIK, bobot=BOBOT_TERBAIK)
        r = {"jenis": _jenis(sk.peta), "sukses": h["sukses"], "alasan": h["alasan"], "ms": 1000 * h["waktu_s"]}
        if h["sukses"]:
            r["periksa"] = periksa_jalur(env, env.jalur(), sk)
            if len(contoh) < 6:
                contoh.append((sk, env.jalur(), env))
        hasil_neg.append(r)

    ring = ringkas(baris, hasil_neg)
    ring.update(robot=rid, model=os.path.relpath(jalur_model, AKAR), langkah_model=meta["langkah"],
                anggaran=args.anggaran, waktu_eval_s=time.time() - t0)
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


def _stat(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return {"n": 0}
    a = np.asarray(xs, float)
    return {"n": len(a), "rata": float(a.mean()), "median": float(np.median(a)),
            "p90": float(np.percentile(a, 90)), "maks": float(a.max())}


def ringkas(baris, neg):
    out = {"per_jenis": {}}
    for j in [None, *S.KOMPOSISI]:
        b = [r for r in baris if j is None or r["jenis"] == j]
        if not b:
            continue
        blok = {"n": len(b)}
        for mode in (*MODE_DQFD, "astar"):
            blok[mode] = {
                "tiba_pct": 100.0 * sum(r[mode]["sukses"] for r in b) / len(b),
                "ms": _stat([r[mode]["ms"] for r in b]),
            }
            if mode != "astar":
                blok[mode]["rasio"] = _stat([r[mode]["rasio"] for r in b])
                blok[mode]["alasan_gagal"] = {a: sum(1 for r in b if not r[mode]["sukses"] and r[mode]["alasan"] == a)
                                              for a in {r[mode]["alasan"] for r in b if not r[mode]["sukses"]}}
        if j is None:
            out["semua"] = blok
        else:
            out["per_jenis"][j] = blok
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
    print("    peta       n    tiba greedy/mundur/TERBAIK/A*(2s)   rasio p90 mundur | TERBAIK (med)"
          "   ms med/p90: TERBAIK | A*")
    for j, b in [("SEMUA", r["semua"]), *r["per_jenis"].items()]:
        g, m, t, a = b["greedy"], b["mundur"], b["terbaik"], b["astar"]
        print(f"    {j:<9} {b['n']:>4}   {g['tiba_pct']:5.1f} / {m['tiba_pct']:5.1f} / {t['tiba_pct']:5.1f}"
              f" / {a['tiba_pct']:5.1f}      {m['rasio'].get('p90', float('nan')):5.2f} | "
              f"{t['rasio'].get('p90', float('nan')):5.2f} ({t['rasio'].get('median', float('nan')):.3f})"
              f"      {t['ms'].get('median', 0):5.0f} / {t['ms'].get('p90', 0):5.0f} | "
              f"{a['ms'].get('median', 0):5.0f} / {a['ms'].get('p90', 0):5.0f}")
    print(f"    gagal (terbaik): {r['semua']['terbaik']['alasan_gagal']}")
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
    ap = argparse.ArgumentParser(description="Evaluasi DQfD vs Hybrid A* pada seluruh rute uji")
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--model", default=None, help="checkpoint; bawaan model_dqn/id0N.pt")
    ap.add_argument("--anggaran", type=int, default=600, help="langkah maksimum pencarian mundur")
    ap.add_argument("--n-neg", type=int, default=150)
    ap.add_argument("--data", default=S.DIR_DATA)
    ap.add_argument("--keluar", default=os.path.join(AKAR, "model_dqn"))
    args = ap.parse_args()
    for rid in [int(t) for t in args.id.split(",") if t.strip()]:
        evaluasi_robot(rid, args.model or os.path.join(AKAR, "model_dqn", f"id{rid:02d}.pt"), args)


if __name__ == "__main__":
    main()
