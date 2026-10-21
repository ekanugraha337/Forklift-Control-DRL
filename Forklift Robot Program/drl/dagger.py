"""
dagger.py - DAgger untuk model dekat: demo Reeds-Shepp dari pose serah yang
BENAR-BENAR dicapai model jauh.

Masalahnya: demo_dekat dimulai di pose tempat Hybrid A* beralih ke RS. Saat
dipakai, model dekat mulai di pose serah MODEL JAUH - sebaran yang lain
(arah datang, jarak, kemudi yang sedang berjalan). Sebagian kegagalan
"gagal dekat" terjadi di pose yang tidak pernah dilihatnya saat latih.

Caranya (Ross dkk. 2011, DAgger - dataset aggregation):
  1. skenario acak dengan sebaran yang sama dengan dataset (skenario.acak_satu,
     seed lain supaya tidak mengulang dataset utama);
  2. rute dijalankan dengan perencana operasional DUA model terlatih
     (perencana_dqfd.rencanakan_dua, metode bawaan) - setiap pose tempat model
     jauh mencoba serah dicatat, berhasil atau gagal didaratkan;
  3. DEMONSTRATOR dari tiap pose itu: tembakan Reeds-Shepp terbaik yang bebas
     (planner._analytic - sama dengan ekor RS Hybrid A*), diurai menjadi
     aksi model dekat (adaptor_demo.urai_ekor_rs) dan diputar ulang dengan
     arah & kemudi yang sedang berjalan. Pose yang tembakan RS-nya terhalang
     tidak menghasilkan demo (dihitung "rs_terhalang").
Hasilnya demo_dekat_dagger_id0N.npz, ikut dibaca latih.py untuk model dekat
(skenario.berkas_demo). Semua episode bukan-uji: set uji tetap set uji lama.
Kolom tambahan ep_dekat_berhasil = model dekat (lama) berhasil mendarat dari
pose itu.

Jalankan dari folder "Forklift Robot Program" (butuh id0N_jauh.pt & id0N_dekat.pt):
    python -m drl.dagger --id 1,2 --n 3000          # 3000 skenario per robot
    python -m drl.dagger --id 1 --n 60 --pekerja 2  # uji cepat
Sesudahnya latih ulang model dekat, lalu model jauh (nilai akhirnya bergantung
model dekat):
    python -m drl.latih --id 1,2 --model dekat,jauh --perangkat cuda
"""
import argparse
import json
import os
import sys
import time
from collections import Counter

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                                               # noqa: E402
import rencana as R                                         # noqa: E402
from drl import lingkungan as L                             # noqa: E402
from drl import skenario as S                               # noqa: E402
from drl.adaptor_demo import galat_margin, putar_ulang, urai_ekor_rs  # noqa: E402

SKENARIO_PER_TUGAS = 20
# Seed peta sintetis & skenario: jauh dari dataset utama (seed 1, --hanya-dekat 501)
# dan dari kumpulan sintetis latihan sendiri (90_000_000+).
SEED_BAWAAN = 7


def _model(rid, dir_model):
    from drl.perencana_dqfd import muat_model
    return muat_model(rid, "jauh", dir_model), muat_model(rid, "dekat", dir_model)


def kerja(tugas):
    """Satu tugas = satu peta, SKENARIO_PER_TUGAS skenario. Di proses pekerja."""
    from drl.perencana_dqfd import rencanakan_dua

    rid, spek = tugas["id"], tugas["peta"]
    net_j, net_d = _model(rid, tugas["dir_model"])
    arena = S._arena(spek)
    prof = konfig.profil(rid)
    per = R.Perencana(arena, prof)
    rng = np.random.default_rng([tugas["seed"], rid])
    env = {m: L.LingkunganRencana(arena, prof, margin_cm=m, jenis="jauh")
           for m in (konfig.SAFETY_MARGIN_CM, *konfig.MARGIN_CADANGAN_CM)}
    dekat = {m: L.LingkunganRencana(arena, prof, margin_cm=m, jenis="dekat", planner=e.planner)
             for m, e in env.items()}
    # lingkungan pemutar ulang demo: tidak boleh sama dengan milik rencanakan_dua
    ulang = {m: L.LingkunganRencana(arena, prof, margin_cm=m, jenis="dekat", planner=e.planner)
             for m, e in env.items()}

    hasil = {"tugas": tugas, "episode": [], "hitung": Counter(), "rute": Counter(), "waktu_s": 0.0}
    t0 = time.perf_counter()
    for i in range(SKENARIO_PER_TUGAS):
        margin, omap, poli, pasangan = S.acak_satu(arena, per, env, rng, tugas["peluang_dekat"])
        if pasangan is None:
            continue
        awal, tujuan = pasangan
        catat = []
        h = rencanakan_dua(net_j, net_d, env[margin], dekat[margin], awal, tujuan, omap, catat=catat)
        hasil["rute"]["tiba" if h["sukses"] else h["fase"]] += 1
        pl, ed = env[margin].planner, ulang[margin]
        for j, c in enumerate(catat):
            hasil["hitung"]["pose"] += 1
            hasil["hitung"]["pose_dekat_gagal"] += not c["sukses"]
            pose, d0, k0 = c["pose"], c["d"], c["k"]
            obs, info = ed.reset(pose, tujuan, omap, d0=d0, k0=k0)
            if obs is None:
                hasil["hitung"]["sudah_tiba"] += 1
                continue
            # ed.reset sudah memasang _hcost & _approach_dirs tujuan ini di perencana
            rs = pl._analytic(omap, (pose[0], pose[1], pose[2], d0, k0), tujuan)
            if rs is None:
                hasil["hitung"]["rs_terhalang"] += 1
                continue
            aksi = urai_ekor_rs(pl, pose, rs[0])
            ep, galat = putar_ulang(ed, pose, tujuan, aksi, omap, d0=d0, k0=k0)
            if galat:
                hasil["hitung"]["dekat_margin" if galat_margin(galat) else "dekat_presisi"] += 1
                continue
            hasil["hitung"]["demo"] += 1
            hasil["hitung"]["demo_dari_gagal"] += not c["sukses"]
            hasil["episode"].append({
                "peta": spek, "margin": margin, "awal": awal, "tujuan": tujuan, "rintangan": poli,
                "tugas": tugas["no"], "i": i * 100 + j, "uji": False, "waktu_astar": np.nan,
                "biaya_astar": ed.biaya, "berhasil": bool(c["sukses"]),
                "dekat": {"ep": ep, "biaya": ed.biaya, "awal": pose, "d0": d0, "k0": k0},
            })
    hasil["waktu_s"] = time.perf_counter() - t0
    return hasil


def daftar_tugas(rid, n_skenario, seed, dir_model):
    """Tugas berurutan; jenis peta mengikuti skenario.KOMPOSISI (deterministik dari seed)."""
    rng = np.random.default_rng([seed, rid, 0xDA66E2])
    jenis = list(S.KOMPOSISI)
    p = np.array([S.KOMPOSISI[j] for j in jenis], float)
    out = []
    for k in range(int(np.ceil(n_skenario / SKENARIO_PER_TUGAS))):
        j = jenis[int(rng.choice(len(jenis), p=p / p.sum()))]
        spek = f"sintetis:{seed * 100000 + rid * 10000 + k}" if j == "sintetis" else j
        out.append({"id": rid, "peta": spek, "seed": seed * 1000003 + k, "no": k,
                    "peluang_dekat": S.PELUANG_DEKAT, "dir_model": dir_model})
    return out


def bangkitkan(rid, n_skenario, seed, pekerja, dir_keluar, dir_model):
    import multiprocessing as mp

    tugas = daftar_tugas(rid, n_skenario, seed, dir_model)
    t0 = time.time()
    hasil = []
    if pekerja <= 1:
        for k, t in enumerate(tugas):
            hasil.append(kerja(t))
            print(f"\r[{time.time() - t0:6.0f} s] ID {rid:02d} tugas {k + 1}/{len(tugas)}", end="", flush=True)
    else:
        with mp.get_context("spawn").Pool(pekerja) as pool:
            for k, r in enumerate(pool.imap_unordered(kerja, tugas)):
                hasil.append(r)
                n_demo = sum(x["hitung"]["demo"] for x in hasil)
                print(f"\r[{time.time() - t0:6.0f} s] ID {rid:02d} tugas {k + 1}/{len(tugas)}, "
                      f"demo {n_demo}", end="", flush=True)
    print()
    hasil.sort(key=lambda r: r["tugas"]["no"])
    episode = [m for r in hasil for m in r["episode"]]
    hitung, rute = Counter(), Counter()
    for r in hasil:
        hitung.update(r["hitung"])
        rute.update(r["rute"])
    ring = {"robot": rid, "skenario": n_skenario, "seed": seed, "episode": len(episode),
            "rute": dict(rute), "pose_serah": dict(hitung),
            "waktu_dinding_s": time.time() - t0, "waktu_cpu_s": sum(r["waktu_s"] for r in hasil)}
    if episode:
        p = os.path.join(dir_keluar, f"demo_dekat_dagger_id{rid:02d}.npz")
        ring["transisi"] = S._simpan_episode(
            p, episode, "dekat",
            ep_d0=np.array([m["dekat"]["d0"] for m in episode], np.int8),
            ep_k0=np.array([m["dekat"]["k0"] for m in episode], np.float64),
            ep_dekat_berhasil=np.array([m["berhasil"] for m in episode], bool))
        ring["berkas"] = p
    with open(os.path.join(dir_keluar, f"ringkasan_dagger_id{rid:02d}.json"), "w", encoding="utf-8") as f:
        json.dump(ring, f, indent=2, ensure_ascii=False)
    n_rute = sum(rute.values())
    print(f"=== ID {rid:02d}: {n_rute} rute - " + ", ".join(f"{k} {v}" for k, v in sorted(rute.items())))
    print(f"    pose serah dicoba {hitung['pose']} (model dekat gagal {hitung['pose_dekat_gagal']}) -> "
          f"demo RS {hitung['demo']} (dari pose gagal {hitung['demo_dari_gagal']}); "
          f"RS terhalang {hitung['rs_terhalang']}, menyentuh margin {hitung['dekat_margin']}, "
          f"tidak presisi {hitung['dekat_presisi']}")
    if episode:
        print(f"    -> {p}  ({ring['transisi']} transisi, {ring['waktu_dinding_s'] / 60:.1f} menit)")
    return ring


def main():
    ap = argparse.ArgumentParser(description="DAgger model dekat: demo RS dari pose serah model jauh")
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--n", type=int, default=3000, help="skenario per robot")
    ap.add_argument("--seed", type=int, default=SEED_BAWAAN)
    ap.add_argument("--pekerja", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--keluar", default=S.DIR_DATA)
    ap.add_argument("--model", default=None, help="folder model (bawaan model_dqn/)")
    args = ap.parse_args()
    for rid in [int(t) for t in args.id.split(",") if t.strip()]:
        bangkitkan(rid, args.n, args.seed, args.pekerja, args.keluar, args.model)


if __name__ == "__main__":
    main()
