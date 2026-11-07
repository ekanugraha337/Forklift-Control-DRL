"""
jaringan_numpy.py - forward jaringan DQfD dengan numpy, tanpa PyTorch.

    JaringanNumpy        forward Dueling DQN (drl/jaringan.py) dengan numpy, float32
    muat_dari_pt(jalur)  JaringanNumpy + meta dari checkpoint .pt, lewat cache .npz
    periksa_kontrak / periksa_profil   pemeriksaan model, tanpa PyTorch

ATURAN ver-1-6: SEMUA forward jaringan memakai kelas ini - perencana operasional
(DQfDPlanner) maupun pelatihan (rollout latihan sendiri, evaluasi, dasbor).
PyTorch hanya dipakai untuk gradien (drl/dqfd.py); bobotnya disalin ke sini
dengan JaringanNumpy.dari_state_dict setelah langkah gradien.

Kenapa: jaringannya kecil (47 -> 256 -> 256 -> 128 -> 14). Satu forward
PyTorch untuk satu keadaan didominasi overhead dispatch PyTorch (Windows ~89 us,
Mac ~35 us), bukan hitungannya. Empat perkalian matriks numpy jauh lebih
ringan. Hasil Q tidak identik sampai bit terakhir dengan PyTorch (urutan
penjumlahan BLAS berbeda, selisih ~1e-6 pada float32).

CACHE .npz: bobot disalin sekali ke <model>.npz di samping <model>.pt,
bersama md5 berkas .pt dan meta-nya. Selama md5 sama, model dimuat tanpa
mengimpor torch sama sekali. Bila .pt berubah (dilatih ulang), .npz dibangun
ulang otomatis - itu satu-satunya saat torch masih diimpor di jalur perencana.
"""
import hashlib
import json
import os

import numpy as np

# Reward = -biaya / SKALA_REWARD. Biaya satu primitif belasan sel, jadi Q ada di
# kisaran satuan sampai puluhan. Didefinisikan di sini (tanpa torch);
# drl/jaringan.py mengimpornya.
SKALA_REWARD = 100.0

_LAPIS = ("fc1", "fc2", "v1", "v2", "a1", "a2")
_META_DISIMPAN = ("jenis", "robot", "versi_aksi", "n_obs", "n_aksi", "max_steer_deg", "r_plan_cm", "langkah",
                  "kepala_tabrak")


class JaringanNumpy:
    """Forward JaringanDQfD (drl/jaringan.py) dengan numpy float32.

    q(obs)        -> Q semua aksi, larik (n_aksi,) float32
    qv(obs)       -> (Q, V) ; V dipakai gerbang mutu (perkiraan biaya total)
    q_batch(X)    -> Q untuk B keadaan sekaligus, (B, n_aksi) - rollout latihan
    aktivasi(obs) -> keluaran tiap lapis, untuk dasbor (drl/visual_neuron.py)
    Bobot disimpan TERTRANSPOSISI (masukan x keluaran) supaya x @ W sama
    bentuknya dengan x @ W.T di nn.Linear.

    Cabang V dan A digabung supaya jumlah operasi numpy sedikit (yang mahal
    adalah overhead per operasi, bukan hitungannya): v1|a1 menjadi satu matriks
    256 x 256, v2 dan a2 satu matriks blok 256 x (1 + n_aksi) yang nol di luar
    bloknya. Matematikanya sama; selisih dengan PyTorch tetap ~1e-6.
    """

    def __init__(self, bobot):
        w = {k: np.asarray(bobot[f"{k}.weight"], dtype=np.float32).T for k in _LAPIS}
        b = {k: np.asarray(bobot[f"{k}.bias"], dtype=np.float32) for k in _LAPIS}
        self.nh, self.n_aksi = w["v1"].shape[1], w["a2"].shape[1]
        nh, n = self.nh, self.n_aksi
        self.n_obs = w["fc1"].shape[0]
        # Kepala kelayakan opsional (t2: A-hidden -> logit tabrak) ikut digabung ke matriks
        # keluaran: o = [V | A (14) | logit tabrak (14)]. Model tanpa t2 tetap jalan.
        self.kepala_tabrak = "t2.weight" in bobot
        self.w1, self.b1 = np.ascontiguousarray(w["fc1"]), b["fc1"]
        self.w2, self.b2 = np.ascontiguousarray(w["fc2"]), b["fc2"]
        self.w_va = np.ascontiguousarray(np.concatenate([w["v1"], w["a1"]], axis=1))
        self.b_va = np.concatenate([b["v1"], b["a1"]])
        self.w_out = np.zeros((2 * nh, 1 + n + (n if self.kepala_tabrak else 0)), np.float32)
        self.w_out[:nh, :1] = w["v2"]
        self.w_out[nh:, 1:1 + n] = w["a2"]
        b_out = [b["v2"], b["a2"]]
        if self.kepala_tabrak:
            self.w_out[nh:, 1 + n:] = np.asarray(bobot["t2.weight"], dtype=np.float32).T
            b_out.append(np.asarray(bobot["t2.bias"], dtype=np.float32))
        self.b_out = np.concatenate(b_out)

    @classmethod
    def dari_state_dict(cls, sd):
        """Dari state_dict jaringan PyTorch (tensor apa pun yang punya .detach())."""
        return cls({k: (v.detach().cpu().numpy() if hasattr(v, "detach") else v) for k, v in sd.items()})

    def _lapis(self, x):
        h = x @ self.w1; h += self.b1; np.maximum(h, 0.0, out=h)
        h2 = h @ self.w2; h2 += self.b2; np.maximum(h2, 0.0, out=h2)
        hva = h2 @ self.w_va; hva += self.b_va; np.maximum(hva, 0.0, out=hva)
        o = hva @ self.w_out; o += self.b_out          # o[..., 0] = V, o[..., 1:1+n] = A, sisanya logit tabrak
        return h, h2, hva, o

    def qv(self, obs):
        o = self._lapis(np.asarray(obs, dtype=np.float32))[3]
        a = o[1:1 + self.n_aksi]
        return (a - a.mean()) + o[0], float(o[0])

    def q(self, obs):
        return self.qv(obs)[0]

    def q_tabrak(self, obs):
        """(Q, P aksi menabrak) untuk satu keadaan, satu forward. P None bila model tanpa kepala_tabrak."""
        o = self._lapis(np.asarray(obs, dtype=np.float32))[3]
        n = self.n_aksi
        a = o[1:1 + n]
        p = 1.0 / (1.0 + np.exp(-o[1 + n:])) if self.kepala_tabrak else None
        return (a - a.mean()) + o[0], p

    def q_batch(self, X):
        o = self._lapis(np.asarray(X, dtype=np.float32).reshape(-1, self.n_obs))[3]
        a = o[:, 1:1 + self.n_aksi]
        return (a - a.mean(axis=1, keepdims=True)) + o[:, :1]

    def tabrak_batch(self, X):
        """P aksi menabrak untuk B keadaan, (B, n_aksi). Hanya model dengan kepala_tabrak."""
        o = self._lapis(np.asarray(X, dtype=np.float32).reshape(-1, self.n_obs))[3]
        return 1.0 / (1.0 + np.exp(-o[:, 1 + self.n_aksi:]))

    def v_batch(self, X):
        """V(s) untuk B keadaan sekaligus, (B,) - peta nilai dasbor."""
        return self._lapis(np.asarray(X, dtype=np.float32).reshape(-1, self.n_obs))[3][:, 0]

    def aktivasi(self, obs):
        """(Q, dict fc1, fc2, v_hidden, a_hidden, v, a) untuk satu keadaan - nama sama dengan PyTorch lama."""
        h, h2, hva, o = self._lapis(np.asarray(obs, dtype=np.float32))
        a = o[1:1 + self.n_aksi]
        return (a - a.mean()) + o[0], {"fc1": h, "fc2": h2, "v_hidden": hva[:self.nh],
                                       "a_hidden": hva[self.nh:], "v": o[:1], "a": a}


def bermask(q, mask, terlarang=-1e9):
    """Q (numpy) dengan aksi terlarang dibuat sangat kecil, supaya tidak pernah jadi argmax/max."""
    return np.where(mask, q, terlarang)


def periksa_kontrak(meta, jalur):
    """Tolak model dengan ruang aksi / masukan / aturan lain dari program sekarang.

    Diperiksa SEBELUM bobot dimuat. Model sebelum ver-1-6 (dua model, demonstrator
    w = 1,3, versi aksi <= 6) ditolak: masukan & keluarannya mungkin sama, tetapi
    demonstrator dan aturan serahnya berbeda.
    """
    from drl.lingkungan import JENIS, N_AKSI, N_OBS, VERSI_AKSI
    versi, jm = meta.get("versi_aksi", 1), meta.get("jenis")
    if versi != VERSI_AKSI or jm != JENIS or meta.get("n_obs") != N_OBS or meta.get("n_aksi") != N_AKSI:
        raise SystemExit(f"[!] {jalur}: model '{jm}' versi aksi {versi}, {meta.get('n_obs')} masukan / "
                         f"{meta.get('n_aksi')} aksi. Program ver-1-6 butuh model '{JENIS}' versi {VERSI_AKSI} "
                         f"({N_OBS} masukan / {N_AKSI} aksi, demonstrator Hybrid A* w = 2,0).\n"
                         f"    Bangkitkan ulang demo dan latih ulang: python -m drl.skenario lalu python -m drl.latih")


def periksa_profil(meta, prof, jalur):
    """Tolak model yang dilatih untuk kemudi lain.

    Ketujuh tingkat kelengkungan aksi diturunkan dari R_PLAN robot. Kalau
    max_steer_deg di konfig berubah (kemudi diperbaiki / diukur ulang), indeks
    aksi yang sama berarti belokan lain, dan nilai Q model lama tidak lagi
    berlaku - padahal jaringannya tetap jalan tanpa galat.
    """
    if "r_plan_cm" not in meta:
        raise SystemExit(f"[!] {jalur} tidak mencatat profil kemudi.\n"
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


def muat_dari_pt(jalur_pt):
    """(JaringanNumpy, meta) dari checkpoint .pt, lewat cache .npz di sampingnya.

    Cache dipakai bila md5 .pt sama dengan yang tercatat; kontrak versi tetap
    diperiksa ulang setiap kali (program bisa berubah walau model tidak).
    Bila cache tidak ada / basi, .pt dibuka dengan PyTorch (drl.dqfd.muat_bobot,
    termasuk pemeriksaannya) lalu cache ditulis.
    """
    md5 = _md5(jalur_pt)
    jc = jalur_cache(jalur_pt)
    if os.path.exists(jc):
        with np.load(jc) as f:
            if str(f["sumber_md5"]) == md5:
                meta = json.loads(str(f["meta"]))
                periksa_kontrak(meta, jalur_pt)
                return JaringanNumpy({k: f[k] for k in f.files if k.endswith((".weight", ".bias"))}), meta
    from drl.dqfd import muat_bobot             # torch hanya di sini, sekali per model baru
    bobot, ck = muat_bobot(jalur_pt)
    meta = {k: (ck[k].item() if hasattr(ck[k], "item") else ck[k]) for k in _META_DISIMPAN if k in ck}
    sementara = jc + ".tmp.npz"
    np.savez(sementara, sumber_md5=np.array(md5), meta=np.array(json.dumps(meta)), **bobot)
    os.replace(sementara, jc)
    return JaringanNumpy(bobot), meta
