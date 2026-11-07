"""
label_tabrak.py - label kepala kelayakan untuk dataset demo: aksi mana yang menabrak zona.

    python -m drl.label_tabrak --id 1        # tambah kolom "tabrak" ke drl/data/demo_jauh_id01.npz

Demo Hybrid A* TIDAK dibangkitkan ulang. Tiap episode diputar ulang di lingkungan
(reset, lalu step dengan aksi demo), dan di setiap keadaan dicatat env.tabrak():
ke-14 aksi, True bila badannya menabrak zona (aturan cek tabrakan A*, termasuk
kelonggaran "lepas diri"). Aksi ke sel yang sudah dikunjungi BUKAN tabrakan.

Pemutaran ulang diperiksa: mask di setiap keadaan harus sama dengan yang tersimpan.
Di mesin lain (mis. data dibuat di Windows, diputar di Mac) observasi bisa berbeda
1 ulp karena libm, jadi selisih observasi hanya dilaporkan; selisih mask di atas
BATAS_SELISIH_MASK menghentikan proses.

drl/latih.py memanggil tambahkan() sendiri bila kepala_tabrak dipakai dan kolomnya
belum ada.
"""
import argparse
import os
import sys
import time

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                                   # noqa: E402
from drl import rollout as RO                   # noqa: E402
from drl import skenario as S                   # noqa: E402

BATAS_SELISIH_MASK = 0.001


def hitung(d, prof):
    """(label (N, 14) bool, ringkasan) untuk dataset demo d (dict larik)."""
    pabrik = RO.PabrikEnv(prof)
    lab = np.zeros(d["mask"].shape, bool)
    n_obs_beda = n_mask_beda = n_ditolak = 0
    for e in range(len(d["ep_mulai"])):
        s0, T = int(d["ep_mulai"][e]), int(d["ep_langkah"][e])
        env, obs, _ = pabrik.buat(RO.dari_demo(d, e))
        if obs is None:
            n_ditolak += 1
            continue
        for t in range(T):
            i = s0 + t
            n_obs_beda += not np.array_equal(obs, d["obs"][i])
            n_mask_beda += not np.array_equal(env.mask(), d["mask"][i])
            lab[i] = env.tabrak()
            if env.selesai:
                break
            obs, _, _, _ = env.step(int(d["aksi"][i]))
    n = len(lab)
    ring = {"keadaan": n, "episode": len(d["ep_mulai"]), "episode_ditolak_reset": n_ditolak,
            "obs_beda": n_obs_beda, "mask_beda": n_mask_beda,
            "aksi_menabrak_persen": 100.0 * float(lab.mean()),
            "keadaan_ada_tabrakan_persen": 100.0 * float(lab.any(1).mean())}
    return lab, ring


def tambahkan(jalur, prof):
    """Tambah kolom "tabrak" ke berkas demo .npz (ditulis ulang). Kembalian: dict larik."""
    t0 = time.time()
    d = RO.muat_npz(jalur)
    print(f"[i] label tabrakan: memutar ulang {len(d['ep_mulai'])} episode demo {os.path.basename(jalur)} ...",
          flush=True)
    lab, ring = hitung(d, prof)
    print(f"    {ring['keadaan']} keadaan, aksi menabrak {ring['aksi_menabrak_persen']:.1f}%, keadaan dengan aksi "
          f"menabrak {ring['keadaan_ada_tabrakan_persen']:.1f}% | mask beda {ring['mask_beda']}, observasi beda "
          f"{ring['obs_beda']}, episode ditolak reset {ring['episode_ditolak_reset']} "
          f"({time.time() - t0:.0f} s)", flush=True)
    if ring["mask_beda"] > BATAS_SELISIH_MASK * ring["keadaan"] or ring["episode_ditolak_reset"]:
        raise SystemExit(f"[!] pemutaran ulang demo tidak cocok dengan dataset ({ring['mask_beda']} mask beda, "
                         f"{ring['episode_ditolak_reset']} episode ditolak). Label tidak ditulis.")
    d["tabrak"] = lab
    sementara = jalur + ".tmp.npz"
    np.savez_compressed(sementara, **d)
    os.replace(sementara, jalur)
    print(f"[OK] kolom tabrak ditambahkan ke {jalur}", flush=True)
    return d


def main():
    ap = argparse.ArgumentParser(description="Tambah label tabrakan (kepala kelayakan) ke dataset demo")
    ap.add_argument("--id", default="1")
    ap.add_argument("--data", default=S.DIR_DATA)
    args = ap.parse_args()
    for rid in [int(t) for t in args.id.split(",") if t.strip()]:
        konfig.pilih_robot(rid)
        berkas = S.berkas_demo(args.data, rid)
        if not berkas:
            raise SystemExit(f"[!] dataset demo ID {rid:02d} tidak ada di {args.data}")
        tambahkan(berkas[0], konfig.profil(rid))


if __name__ == "__main__":
    main()
