"""
demo_rs.py - dataset model dekat versi 6 (kata Reeds-Shepp) dari metadata dataset model dekat yang ada.

Model dekat versi 6 (ver-1-4) punya SATU aksi per episode: kata RS pilihan
demonstrator di pose serah (lingkungan.aksi_demonstrator, persis
_best_free_rs Hybrid A*). Dataset model dekat versi lain (demo_dekat,
demo_dekat_tambahan, demo_dekat_dagger) sudah menyimpan semua yang dibutuhkan
untuk membangunnya ulang tanpa menjalankan Hybrid A* lagi: pose serah
(ep_awal), arah & kemudi berjalan (ep_d0 / ep_k0), tujuan, peta, margin, dan
badan robot lain. Tiap episode diputar ulang dengan satu aksi demonstrator.
Episode yang dari posenya tidak ada kata RS bebas dibuang dan dihitung.

    python -m drl.demo_rs --id 1,2 --sumber drl/data/arsip_v5_dekat_sinar --keluar drl/data

Keluaran: berkas dengan nama yang sama di --keluar (demo_dekat_id0N.npz, ...),
format skenario._simpan_episode, kolom tambahan disalin (ep_dekat_berhasil).
Demo model jauh tidak berubah aksinya; kolom ep_obs_serah / ep_mask_serah-nya
dihitung ulang dengan drl.hitung_ulang_obs.
"""
import argparse
import os
import sys
import time
from collections import Counter

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                                   # noqa: E402
from drl import lingkungan as L                 # noqa: E402
from drl import rollout as RO                   # noqa: E402
from drl import skenario as S                   # noqa: E402
from drl.adaptor_demo import putar_ulang        # noqa: E402

BERKAS = ("demo_dekat_id{:02d}.npz", "demo_dekat_tambahan_id{:02d}.npz", "demo_dekat_dagger_id{:02d}.npz")


def ubah(rid, masuk, keluar):
    d = RO.muat_npz(masuk)
    prof = konfig.profil(rid)
    pab = RO.PabrikEnv(prof, "dekat")
    env = {}
    daftar, hitung, kata = [], Counter(), Counter()
    tambahan = {k: [] for k in ("ep_dekat_berhasil",) if k in d}
    selisih = []
    t0 = time.time()
    for e in range(len(d["ep_mulai"])):
        sk = RO.dari_demo(d, e)
        kunci = (sk.peta, sk.margin)
        if kunci not in env:
            env[kunci] = L.LingkunganRencana(pab.arena(sk.peta), prof, margin_cm=sk.margin, jenis="dekat")
        en, om = env[kunci], pab.omap(sk)
        obs, info = en.reset(sk.awal, sk.tujuan, om, d0=sk.d0, k0=sk.k0)
        if obs is None:
            hitung["reset ditolak: " + info["alasan"]] += 1
            continue
        a = en.aksi_demonstrator()
        if a is None:
            hitung["tanpa kata RS bebas"] += 1
            continue
        ep, galat = putar_ulang(en, sk.awal, sk.tujuan, [a], om, d0=sk.d0, k0=sk.k0)
        if galat:
            hitung["galat: " + galat] += 1
            continue
        hitung["episode"] += 1
        kata[a] += 1
        if sk.biaya_astar:
            selisih.append(en.biaya / float(d["ep_biaya"][e]) - 1.0)
        poli = sk.rintangan
        daftar.append({"peta": sk.peta, "margin": sk.margin, "awal": sk.awal, "tujuan": sk.tujuan,
                       "rintangan": poli, "uji": bool(d["ep_uji"][e]), "waktu_astar": float(d["ep_waktu_astar"][e]),
                       "biaya_astar": float(d["ep_biaya_astar"][e]) if "ep_biaya_astar" in d else en.biaya,
                       "dekat": {"ep": ep, "biaya": en.biaya, "awal": sk.awal, "d0": sk.d0, "k0": sk.k0}})
        for k in tambahan:
            tambahan[k].append(d[k][e])
        if (e + 1) % 1000 == 0:
            print(f"\r    {e + 1}/{len(d['ep_mulai'])} ({time.time() - t0:.0f} s)", end="", flush=True)
    print()
    n_tr = S._simpan_episode(keluar, daftar, "dekat",
                             ep_d0=np.array([m["dekat"]["d0"] for m in daftar], np.int8),
                             ep_k0=np.array([m["dekat"]["k0"] for m in daftar], np.float64),
                             **{k: np.array(v) for k, v in tambahan.items()})
    top = ", ".join(f"{L.nama_kata(a)} {100 * n / max(1, hitung['episode']):.1f}%" for a, n in kata.most_common(6))
    sel = np.asarray(selisih)
    print(f"[OK] {os.path.basename(masuk)}: {len(d['ep_mulai'])} episode sumber -> {hitung['episode']} episode "
          f"({n_tr} transisi), {int(sum(d['ep_uji'][:]))} uji di sumber, {sum(m['uji'] for m in daftar)} uji di hasil")
    lain = {k: v for k, v in hitung.items() if k != "episode"}
    if lain:
        print(f"     dibuang: {lain}")
    print(f"     biaya kata RS vs episode sumber (segmen yang diurai): median {100 * np.median(sel):+.2f}%, "
          f"p10 {100 * np.percentile(sel, 10):+.2f}%, p90 {100 * np.percentile(sel, 90):+.2f}%")
    print(f"     kata terbanyak: {top}; {len(kata)} dari {L.n_aksi('dekat')} aksi pernah dipilih "
          f"({time.time() - t0:.0f} s)\n     -> {keluar}")


def main():
    ap = argparse.ArgumentParser(description="Dataset model dekat versi 6 (kata RS) dari dataset model dekat lain")
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--sumber", required=True, help="folder dataset model dekat lama")
    ap.add_argument("--keluar", default=S.DIR_DATA)
    args = ap.parse_args()
    if os.path.abspath(args.sumber) == os.path.abspath(args.keluar):
        ap.error("--keluar harus berbeda dari --sumber (arsipkan dataset lama dulu)")
    os.makedirs(args.keluar, exist_ok=True)
    for rid in [int(t) for t in args.id.split(",") if t.strip()]:
        for pola in BERKAS:
            p = os.path.join(args.sumber, pola.format(rid))
            if os.path.exists(p):
                ubah(rid, p, os.path.join(args.keluar, pola.format(rid)))


if __name__ == "__main__":
    main()
