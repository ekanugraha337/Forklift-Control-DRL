"""
jaringan_numpy.py - jaringan DQfD untuk INFERENSI tanpa PyTorch (ver-1-5, perubahan 2).

    JaringanNumpy        forward Dueling DQN (drl/jaringan.py) dengan numpy, float32
    muat_dari_pt(jalur)  JaringanNumpy + meta dari checkpoint .pt, lewat cache .npz
    periksa_kontrak / periksa_profil   pemeriksaan model, tanpa PyTorch

Kenapa: jaringannya kecil (47 -> 256 -> 256 -> 128 -> 14 / 88). Satu forward
PyTorch untuk satu keadaan didominasi overhead dispatch PyTorch (Windows ~89 us,
Mac ~35 us), bukan hitungannya. Empat perkalian matriks numpy jauh lebih
ringan. Bobotnya bobot checkpoint .pt yang SAMA - tidak ada latih ulang.

Hasil Q tidak identik sampai bit terakhir dengan PyTorch (urutan penjumlahan
BLAS berbeda, selisih ~1e-6 pada float32). Di keadaan yang Q-nya nyaris seri,
urutan calon bisa berbeda; itu sebabnya pelatihan (drl/latih.py, dagger.py)
tetap memakai PyTorch, dan hanya perencana operasional (DQfDPlanner) yang
memakai berkas ini.

CACHE .npz: bobot disalin sekali ke <model>.npz di samping <model>.pt,
bersama md5 berkas .pt dan meta-nya. Selama md5 sama, model dimuat tanpa
mengimpor torch sama sekali (navigasi.py mulai 1-3 s lebih cepat). Bila .pt
berubah (dilatih ulang), .npz dibangun ulang otomatis - itu satu-satunya saat
torch masih diimpor.
"""
import hashlib
import json
import os

import numpy as np

# Reward = -biaya / SKALA_REWARD. Biaya per langkah median 17 (primitif penuh;
# primitif pendek lebih murah), jadi Q ada di kisaran satuan sampai puluhan.
# Didefinisikan di sini (tanpa torch); drl/jaringan.py mengimpornya.
SKALA_REWARD = 100.0

_LAPIS = ("fc1", "fc2", "v1", "v2", "a1", "a2")
_META_DISIMPAN = ("jenis", "robot", "versi_aksi", "n_obs", "n_aksi", "max_steer_deg", "r_plan_cm", "langkah")


class JaringanNumpy:
    """Forward JaringanDQfD (drl/jaringan.py) dengan numpy float32.

    q(obs)  -> Q semua aksi, larik (n_aksi,) float32
    qv(obs) -> (Q, V) ; V dipakai gerbang mutu (perkiraan biaya total)
    Bobot disimpan TERTRANSPOSISI (masukan x keluaran) supaya x @ W sama
    bentuknya dengan x @ W.T di nn.Linear.

    Cabang V dan A digabung supaya jumlah operasi numpy sedikit (yang mahal
    adalah overhead per operasi, bukan hitungannya): v1|a1 menjadi satu matriks
    256 x 256, v2 dan a2 satu matriks blok 256 x (1 + n_aksi) yang nol di luar
    bloknya. Matematikanya sama; selisih dengan PyTorch tetap ~1e-6.
    Mac: PyTorch 37 us, numpy terpisah 22 us, numpy gabung 16 us per forward.
    """

    def __init__(self, bobot):
        w = {k: np.asarray(bobot[f"{k}.weight"], dtype=np.float32).T for k in _LAPIS}
        b = {k: np.asarray(bobot[f"{k}.bias"], dtype=np.float32) for k in _LAPIS}
        nh, self.n_aksi = w["v1"].shape[1], w["a2"].shape[1]
        self.n_obs = w["fc1"].shape[0]
        self.w1, self.b1 = np.ascontiguousarray(w["fc1"]), b["fc1"]
        self.w2, self.b2 = np.ascontiguousarray(w["fc2"]), b["fc2"]
        self.w_va = np.ascontiguousarray(np.concatenate([w["v1"], w["a1"]], axis=1))
        self.b_va = np.concatenate([b["v1"], b["a1"]])
        self.w_out = np.zeros((2 * nh, 1 + self.n_aksi), np.float32)
        self.w_out[:nh, :1] = w["v2"]
        self.w_out[nh:, 1:] = w["a2"]
        self.b_out = np.concatenate([b["v2"], b["a2"]])

    def qv(self, obs):
        x = np.asarray(obs, dtype=np.float32)
        h = x @ self.w1; h += self.b1; np.maximum(h, 0.0, out=h)
        h2 = h @ self.w2; h2 += self.b2; np.maximum(h2, 0.0, out=h2)
        hva = h2 @ self.w_va; hva += self.b_va; np.maximum(hva, 0.0, out=hva)
        o = hva @ self.w_out; o += self.b_out          # o[0] = V, o[1:] = A
        a = o[1:]
        return (a - a.mean()) + o[0], float(o[0])

    def q(self, obs):
        return self.qv(obs)[0]


def periksa_kontrak(meta, jalur, jenis=None):
    """Tolak model dengan ruang aksi / masukan lain dari program sekarang.

    Diperiksa SEBELUM bobot dimuat: model ruang aksi lama (15 keluaran, dengan
    SAMBUNG) akan gagal di load_state_dict dengan pesan bentuk tensor yang
    tidak menjelaskan apa-apa. Versi diperiksa PER JENIS (lingkungan.VERSI_MODEL):
    model jauh versi 4 masih cocok dengan program versi 5, model dekat versi 4 tidak.
    """
    from drl.lingkungan import VERSI_AKSI, VERSI_MODEL, n_aksi, n_obs, versi_cocok
    versi, jm = meta.get("versi_aksi", 1), meta.get("jenis")
    if not versi_cocok(jm, versi) or meta.get("n_obs") != n_obs(jm) or meta.get("n_aksi") != n_aksi(jm):
        syarat = (f"butuh versi {VERSI_MODEL[jm]}-{VERSI_AKSI}" if jm in VERSI_MODEL
                  else "model sebelum dua-model (versi < 4)")
        raise SystemExit(f"[!] {jalur}: model '{jm}' ruang aksi versi {versi}, {syarat}. Program "
                         f"sekarang versi {VERSI_AKSI} (jauh meniru Hybrid A* - {n_obs('jauh')} "
                         f"masukan / {n_aksi('jauh')} aksi, dekat meniru Reeds-Shepp dan sadar "
                         f"rintangan - {n_obs('dekat')} masukan / {n_aksi('dekat')} aksi).\n"
                         f"    Bangkitkan ulang demo dan latih ulang: python -m drl.skenario lalu "
                         f"python -m drl.latih")
    if jenis is not None and jm != jenis:
        raise SystemExit(f"[!] {jalur} adalah model '{jm}', bukan '{jenis}'")


def periksa_profil(meta, prof, jalur):
    """Tolak model yang dilatih untuk kemudi lain.

    Ketujuh tingkat kelengkungan aksi diturunkan dari R_PLAN robot. Kalau
    max_steer_deg di konfig berubah (kemudi diperbaiki / diukur ulang), indeks
    aksi yang sama berarti belokan lain, dan nilai Q model lama tidak lagi
    berlaku - padahal jaringannya tetap jalan tanpa galat. Model sebelum
    pengaman ini tidak menyimpan profil, jadi juga ditolak.
    """
    if "r_plan_cm" not in meta:
        raise SystemExit(f"[!] {jalur} tidak mencatat profil kemudi (dilatih sebelum 2026-09-30).\n"
                         f"    Latih ulang: python -m drl.latih --id {prof.id}")
    if abs(meta["r_plan_cm"] - prof.r_plan_cm) > 1e-3:
        raise SystemExit(f"[!] {jalur} dilatih untuk max_steer {meta['max_steer_deg']:.1f} deg "
                         f"(R_PLAN {meta['r_plan_cm']:.1f} cm), konfig sekarang {prof.max_steer_deg:.1f} deg "
                         f"(R_PLAN {prof.r_plan_cm:.1f} cm).\n"
                         f"    Bangkitkan ulang demo dan latih ulang: python -m drl.skenario --id {prof.id} "
                         f"lalu python -m drl.latih --id {prof.id}. Sementara itu pakai --perencana astar.")


def _md5(jalur):
    h = hashlib.md5()
    with open(jalur, "rb") as f:
        for blok in iter(lambda: f.read(1 << 20), b""):
            h.update(blok)
    return h.hexdigest()


def jalur_cache(jalur_pt):
    return os.path.splitext(jalur_pt)[0] + ".npz"


def muat_dari_pt(jalur_pt, jenis=None):
    """(JaringanNumpy, meta) dari checkpoint .pt, lewat cache .npz di sampingnya.

    Cache dipakai bila md5 .pt sama dengan yang tercatat; kontrak versi tetap
    diperiksa ulang setiap kali (program bisa berubah walau model tidak).
    Bila cache tidak ada / basi, .pt dibuka dengan PyTorch (drl.dqfd.muat_jaringan,
    termasuk pemeriksaannya) lalu cache ditulis.
    """
    md5 = _md5(jalur_pt)
    jc = jalur_cache(jalur_pt)
    if os.path.exists(jc):
        with np.load(jc) as f:
            if str(f["sumber_md5"]) == md5:
                meta = json.loads(str(f["meta"]))
                periksa_kontrak(meta, jalur_pt, jenis)
                return JaringanNumpy({k: f[k] for k in f.files if k.endswith((".weight", ".bias"))}), meta
    from drl.dqfd import muat_jaringan          # torch hanya di sini, sekali per model baru
    net, ck = muat_jaringan(jalur_pt, jenis=jenis)
    bobot = {k: v.detach().cpu().numpy() for k, v in net.state_dict().items()}
    meta = {k: (ck[k].item() if hasattr(ck[k], "item") else ck[k]) for k in _META_DISIMPAN if k in ck}
    sementara = jc + ".tmp.npz"
    np.savez(sementara, sumber_md5=np.array(md5), meta=np.array(json.dumps(meta)), **bobot)
    os.replace(sementara, jc)
    return JaringanNumpy(bobot), meta
