"""
hitung_ulang_obs.py - hitung ulang OBSERVASI dataset demo dengan lingkungan sekarang.

Dipakai bila hanya masukan jaringan yang berubah, sedangkan aksi, biaya, dan
mask demonstratornya tetap - mis. versi 5 (ver-1-3): model dekat 9 -> 47
masukan. Tiap episode diputar ulang dari metadatanya (pose awal, d0/k0, tujuan,
peta, margin, rintangan, aksi), persis seperti skenario --periksa. Berhenti
dengan galat bila aksi, reward, mask, atau tanda selesai berbeda dari dataset:
yang boleh berubah HANYA obs / obs_next (dan ep_obs_serah untuk demo jauh).

Contoh - data DAgger versi 4 dipakai lagi untuk model dekat versi 5:
    python -m drl.hitung_ulang_obs --id 1 --masuk drl/data/arsip_v4_dekat_buta/demo_dekat_dagger_id01.npz --keluar drl/data/demo_dekat_dagger_id01.npz
"""
import argparse
import os
import sys
import time

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                                    # noqa: E402
from drl import lingkungan as L                  # noqa: E402
from drl import rollout as RO                    # noqa: E402
from drl.adaptor_demo import putar_ulang         # noqa: E402


def hitung_ulang(rid, masuk, keluar):
    d = RO.muat_npz(masuk)
    jenis = str(d["jenis"])
    prof = konfig.profil(rid)
    pab = RO.PabrikEnv(prof, jenis)
    n_tr, n_ep = len(d["aksi"]), len(d["ep_mulai"])
    obs = np.zeros((n_tr, L.n_obs(jenis)), np.float32)
    obs2 = np.zeros_like(obs)
    serah, mask_serah = [], []
    env, dekat = {}, {}
    t0 = time.time()
    for e in range(n_ep):
        sk = RO.dari_demo(d, e)
        kunci = (sk.peta, sk.margin)
        if kunci not in env:
            env[kunci] = L.LingkunganRencana(pab.arena(sk.peta), prof, margin_cm=sk.margin, jenis=jenis)
            if jenis == "jauh":
                dekat[kunci] = L.LingkunganRencana(pab.arena(sk.peta), prof, margin_cm=sk.margin,
                                                   jenis="dekat", planner=env[kunci].planner)
        en, om = env[kunci], pab.omap(sk)
        ep, galat = putar_ulang(en, sk.awal, sk.tujuan, sk.aksi_demo, om, d0=sk.d0, k0=sk.k0)
        s0, T = int(d["ep_mulai"][e]), int(d["ep_langkah"][e])
        if galat:
            raise SystemExit(f"[!] episode {e}: putar ulang gagal ({galat})")
        if len(ep["aksi"]) != T:
            raise SystemExit(f"[!] episode {e}: putar ulang berhenti setelah {len(ep['aksi'])} dari {T} aksi")
        sl = slice(s0, s0 + T)
        sama = (np.array_equal(ep["aksi"], d["aksi"][sl])
                and np.allclose(ep["reward"], d["reward"][sl], rtol=0, atol=1e-4)
                and np.array_equal(ep["mask"][:-1], d["mask"][sl])
                and np.array_equal(ep["mask"][1:], d["mask_next"][sl])
                and np.array_equal(ep["selesai"], d["selesai"][sl]))
        if not sama:
            raise SystemExit(f"[!] episode {e}: aksi/reward/mask berbeda dari dataset - bukan hanya "
                             f"observasi yang berubah, bangkitkan ulang datasetnya")
        obs[sl], obs2[sl] = ep["obs"][:-1], ep["obs"][1:]
        if jenis == "jauh":
            o, _ = dekat[kunci].reset((en.x, en.y, en.yaw), sk.tujuan, om, d0=en.d, k0=en.k)
            serah.append(o if o is not None else np.zeros(L.n_obs("dekat"), np.float32))
            mask_serah.append(dekat[kunci].mask() if o is not None else np.zeros(L.n_aksi("dekat"), bool))
        if (e + 1) % 500 == 0:
            print(f"\r    {e + 1}/{n_ep} episode ({time.time() - t0:.0f} s)", end="", flush=True)
    print()
    lama = d["obs"].shape[1]
    awalan = lama <= obs.shape[1] and np.allclose(d["obs"], obs[:, :lama], rtol=0, atol=1e-6)
    out = dict(d)
    out["obs"], out["obs_next"] = obs, obs2
    if jenis == "jauh":
        out["ep_obs_serah"] = np.array(serah, np.float32)
        out["ep_mask_serah"] = np.array(mask_serah, bool)
    np.savez_compressed(keluar, **out)
    print(f"[OK] {os.path.basename(masuk)} ({jenis}): {n_ep} episode, {n_tr} transisi, observasi "
          f"{lama} -> {obs.shape[1]} kolom; aksi, reward, mask identik"
          + (f"; {lama} kolom pertama sama dengan observasi lama" if awalan else "")
          + f" ({time.time() - t0:.0f} s)\n    -> {keluar}")


def main():
    ap = argparse.ArgumentParser(description="Hitung ulang observasi dataset demo dengan lingkungan sekarang")
    ap.add_argument("--id", type=int, required=True, help="robot pemilik dataset (profil kemudi)")
    ap.add_argument("--masuk", required=True)
    ap.add_argument("--keluar", required=True)
    args = ap.parse_args()
    if os.path.abspath(args.masuk) == os.path.abspath(args.keluar):
        ap.error("--keluar harus berbeda dari --masuk")
    hitung_ulang(args.id, args.masuk, args.keluar)


if __name__ == "__main__":
    main()
