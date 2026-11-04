"""
Ablasi model dekat: apakah urutan Q hasil belajar lebih baik daripada
mengurutkan kata RS menurut biaya rumusnya?

Di setiap pose pendaratan, semua metode memakai kandidat yang SAMA
(env._calon dari rumus RS, mask = punya solusi geometri) dan cek tabrakan
yang SAMA (env._cek_kurva). Yang berbeda hanya URUTAN pengecekan:

  q          model dekat: urutan Q menurun, cek sampai kata pertama bebas (sistem sekarang)
  biaya      urutan biaya rumus RS menaik, cek sampai kata pertama bebas (semua kata sah)
  demo       aturan Hybrid A* + RS (aksi_demonstrator): <= 6 termurah, berhenti dini
  acak       urutan acak (rata-rata 5 permutasi), cek sampai kata pertama bebas
  oracle     cek SEMUA kata sah; biaya total termurah yang bebas = acuan

Sumber pose:
  serah      setiap percobaan pendaratan saat merencanakan rute penuh uji
             (rencanakan_dua, metode gabungan) - pose yang benar-benar dihadapi sistem
  dekat      start episode uji demo_dekat_tambahan (rute pendek di dalam 60 cm)

    python -m drl.ablasi_dekat --id 1,2      (hasil: model_dqn/ablasi_dekat.json)
"""
import argparse
import json
import math
import os
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import torch  # noqa: E402

import konfig  # noqa: E402
from drl import lingkungan as L  # noqa: E402
from drl import rollout as RO  # noqa: E402
from drl import skenario as S  # noqa: E402
from drl.dqfd import muat_jaringan, periksa_profil  # noqa: E402
from drl.perencana_dqfd import jalur_model, rencanakan_dua  # noqa: E402

METODE = ("q", "biaya", "demo", "acak")
N_ACAK = 5


def cek_urut(env, urutan):
    """Cek tabrakan menurut urutan sampai kata pertama bebas -> (aksi, biaya total, n_cek, ms_cek)."""
    t0 = time.perf_counter()
    n = 0
    for a in urutan:
        segs, b = env._calon[a]
        _, tot, tabrak = env._cek_kurva(segs, b)
        n += 1
        if tabrak is None:
            return int(a), tot, n, 1000 * (time.perf_counter() - t0)
    return None, None, n, 1000 * (time.perf_counter() - t0)


def demo_hitung(env, sah):
    """Salinan aksi_demonstrator() dengan penghitung cek."""
    t0 = time.perf_counter()
    urut = sorted(sah, key=lambda a: env._calon[a][1])
    best, n = None, 0
    for a in urut[:6]:
        segs, biaya = env._calon[a]
        if best is not None and biaya >= best[1]:
            break
        _, total, tabrak = env._cek_kurva(segs, biaya)
        n += 1
        if tabrak is None and (best is None or total < best[1]):
            best = (int(a), total)
    ms = 1000 * (time.perf_counter() - t0)
    return (None, None, n, ms) if best is None else (best[0], best[1], n, ms)


def nilai_pose(env, net, rng):
    """Satu pose (env sudah di-reset, obs tidak None) -> dict per metode + oracle."""
    sah = [int(a) for a in np.flatnonzero(env.mask())]
    r = {"n_sah": len(sah)}
    # oracle
    bebas = []
    for a in sah:
        segs, b = env._calon[a]
        _, tot, tabrak = env._cek_kurva(segs, b)
        if tabrak is None:
            bebas.append((tot, a))
    r["n_bebas"] = len(bebas)
    r["oracle_biaya"] = min(bebas)[0] if bebas else None
    r["oracle_aksi"] = min(bebas)[1] if bebas else None

    # q
    t0 = time.perf_counter()
    with torch.no_grad():
        q = net(torch.as_tensor(env.observasi()[None]))[0].numpy()
    urut = [sah[i] for i in np.argsort(-q[sah], kind="stable")]
    ms_urut = 1000 * (time.perf_counter() - t0)
    a, tot, n, ms = cek_urut(env, urut)
    r["q"] = {"aksi": a, "biaya": tot, "n_cek": n, "ms": ms_urut + ms, "ms_urut": ms_urut}

    # biaya rumus
    t0 = time.perf_counter()
    urut = sorted(sah, key=lambda x: env._calon[x][1])
    ms_urut = 1000 * (time.perf_counter() - t0)
    a, tot, n, ms = cek_urut(env, urut)
    r["biaya"] = {"aksi": a, "biaya": tot, "n_cek": n, "ms": ms_urut + ms, "ms_urut": ms_urut}

    # demonstrator
    a, tot, n, ms = demo_hitung(env, sah)
    r["demo"] = {"aksi": a, "biaya": tot, "n_cek": n, "ms": ms, "ms_urut": 0.0}

    # acak
    ns, mss, bs = [], [], []
    for _ in range(N_ACAK):
        urut = list(rng.permutation(sah))
        a, tot, n, ms = cek_urut(env, urut)
        ns.append(n), mss.append(ms), bs.append(tot)
    r["acak"] = {"aksi": a, "biaya": None if bs[0] is None else float(np.mean(bs)),
                 "n_cek": float(np.mean(ns)), "ms": float(np.mean(mss)), "ms_urut": 0.0}
    return r


def stat(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return {"n": 0}
    a = np.asarray(xs, float)
    return {"n": len(a), "rata": float(a.mean()), "median": float(np.median(a)),
            "p90": float(np.percentile(a, 90)), "maks": float(a.max())}


def ringkas(baris):
    out = {"n_pose": len(baris), "n_ada_bebas": sum(r["n_bebas"] > 0 for r in baris),
           "n_sah": stat([r["n_sah"] for r in baris]), "n_bebas": stat([r["n_bebas"] for r in baris])}
    bisa = [r for r in baris if r["n_bebas"] > 0]
    tak = [r for r in baris if r["n_bebas"] == 0]
    for m in METODE:
        x = {
            "tiba_pct_dari_yang_bisa": 100.0 * sum(r[m]["aksi"] is not None for r in bisa) / max(1, len(bisa)),
            "n_cek_bisa": stat([r[m]["n_cek"] for r in bisa]),
            "n_cek_tak_bisa": stat([r[m]["n_cek"] for r in tak]),
            "n_cek_semua": stat([r[m]["n_cek"] for r in baris]),
            "ms_bisa": stat([r[m]["ms"] for r in bisa]),
            "ms_semua": stat([r[m]["ms"] for r in baris]),
            "rasio_biaya_vs_oracle": stat([r[m]["biaya"] / r["oracle_biaya"] for r in bisa
                                           if r[m]["biaya"] is not None and r["oracle_biaya"]]),
            "pilih_optimal_pct": 100.0 * sum(r[m]["aksi"] == r["oracle_aksi"] for r in bisa
                                             if r[m]["aksi"] is not None) / max(1, len(bisa)),
            "cek_pertama_bebas_pct": 100.0 * sum(r[m]["n_cek"] == 1 and r[m]["aksi"] is not None
                                                 for r in bisa) / max(1, len(bisa)),
        }
        if m == "q":
            x["ms_urut"] = stat([r[m]["ms_urut"] for r in baris])
        out[m] = x
    return out


def pose_serah(rid, prof, net_j, net_d, uji, pabrik, batas=None):
    """Jalankan rencanakan_dua di rute uji penuh; kumpulkan setiap percobaan pendaratan."""
    hasil = []
    t0 = time.time()
    for k, sk in enumerate(uji):
        a = pabrik.arena(sk.peta)
        om = pabrik.omap(sk)
        ej = L.LingkunganRencana(a, prof, margin_cm=sk.margin, jenis="jauh")
        ed = L.LingkunganRencana(a, prof, margin_cm=sk.margin, jenis="dekat")
        catat = []
        rencanakan_dua(net_j, net_d, ej, ed, sk.awal, sk.tujuan, om, metode="gabungan",
                       batas_waktu=batas, catat=catat)
        for c in catat:
            hasil.append((sk, c["pose"], c["d"], c["k"], c["sukses"]))
        if (k + 1) % 100 == 0:
            print(f"    ID {rid:02d}: {k + 1}/{len(uji)} rute, {len(hasil)} pose serah ({time.time() - t0:.0f} s)",
                  flush=True)
    return hasil


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--keluar", default=os.path.join(AKAR, "model_dqn"))
    ap.add_argument("--batas-rute", type=float, default=2.0, help="batas waktu rencanakan_dua (s), seperti di robot")
    args = ap.parse_args()
    os.makedirs(args.keluar, exist_ok=True)
    semua = {}
    for rid in [int(t) for t in args.id.split(",")]:
        prof = konfig.profil(rid)
        jj, jd = jalur_model(rid, "jauh"), jalur_model(rid, "dekat")
        net_j, meta_j = muat_jaringan(jj, jenis="jauh")
        net_d, meta_d = muat_jaringan(jd, jenis="dekat")
        periksa_profil(meta_j, prof, jj)
        periksa_profil(meta_d, prof, jd)
        pabrik = RO.PabrikEnv(prof, "jauh")
        rng = np.random.default_rng(rid)

        d = RO.muat_npz(os.path.join(S.DIR_DATA, f"demo_jauh_id{rid:02d}.npz"))
        uji_penuh = [RO.dari_demo(d, e) for e in np.flatnonzero(d["ep_uji"])]
        d = RO.muat_npz(os.path.join(S.DIR_DATA, f"demo_dekat_tambahan_id{rid:02d}.npz"))
        uji_dekat = [RO.dari_demo(d, e) for e in np.flatnonzero(d["ep_uji"])]
        print(f"=== ID {rid:02d}: dekat {os.path.basename(jd)} (langkah {meta_d['langkah']}), "
              f"{len(uji_penuh)} rute penuh, {len(uji_dekat)} rute dekat", flush=True)

        sumber = {"serah": [(sk, p, dd, kk) for sk, p, dd, kk, _ in
                            pose_serah(rid, prof, net_j, net_d, uji_penuh, pabrik, args.batas_rute)],
                  "dekat": [(sk, sk.awal, sk.d0, sk.k0) for sk in uji_dekat]}
        hasil_robot = {}
        for nama, daftar in sumber.items():
            baris = []
            t0 = time.time()
            for sk, pose, d0, k0 in daftar:
                env = L.LingkunganRencana(pabrik.arena(sk.peta), prof, margin_cm=sk.margin, jenis="dekat")
                obs, info = env.reset(tuple(pose), sk.tujuan, pabrik.omap(sk), d0=d0, k0=k0)
                if obs is None:
                    continue
                baris.append(nilai_pose(env, net_d, rng))
            r = ringkas(baris)
            r["waktu_s"] = time.time() - t0
            hasil_robot[nama] = r
            print(f"  [{nama}] {r['n_pose']} pose, {r['n_ada_bebas']} punya kata bebas", flush=True)
            for m in METODE:
                x = r[m]
                g = lambda s, k: s.get(k, float('nan'))  # noqa: E731
                print(f"    {m:<6} tiba {x['tiba_pct_dari_yang_bisa']:5.1f}%  "
                      f"cek(bisa) rata {g(x['n_cek_bisa'], 'rata'):5.2f} med {g(x['n_cek_bisa'], 'median'):4.0f} "
                      f"p90 {g(x['n_cek_bisa'], 'p90'):5.1f}  cek(semua) rata {g(x['n_cek_semua'], 'rata'):5.2f}  "
                      f"ms(bisa) rata {g(x['ms_bisa'], 'rata'):.3f} p90 {g(x['ms_bisa'], 'p90'):.3f}  "
                      f"biaya/oracle med {g(x['rasio_biaya_vs_oracle'], 'median'):.3f} "
                      f"p90 {g(x['rasio_biaya_vs_oracle'], 'p90'):.3f}  optimal {x['pilih_optimal_pct']:5.1f}%  "
                      f"cek-1 {x['cek_pertama_bebas_pct']:5.1f}%", flush=True)
        semua[f"id{rid:02d}"] = hasil_robot
    with open(os.path.join(args.keluar, "ablasi_dekat.json"), "w") as f:
        json.dump(semua, f, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print("->", os.path.join(args.keluar, "ablasi_dekat.json"))


if __name__ == "__main__":
    main()
