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


class ModelPNumpy:
    """Forward model P terpisah (drl.kelayakan --latih: 47 -> lebar -> lebar -> 14, ReLU, sigmoid).
    p(obs) -> P aksi menabrak (14,), p_batch(X) -> (B, 14)."""

    def __init__(self, bobot):
        self.w = [np.ascontiguousarray(np.asarray(bobot[f"l{i}.weight"], np.float32).T) for i in (1, 2, 3)]
        self.b = [np.asarray(bobot[f"l{i}.bias"], np.float32) for i in (1, 2, 3)]

    def logit_batch(self, X):
        h = np.asarray(X, np.float32).reshape(-1, self.w[0].shape[0])
        for i in range(3):
            h = h @ self.w[i]
            h += self.b[i]
            if i < 2:
                np.maximum(h, 0.0, out=h)
        return h

    def p(self, obs):
        h = np.asarray(obs, np.float32)
        for i in range(3):
            h = h @ self.w[i]
            h += self.b[i]
            if i < 2:
                np.maximum(h, 0.0, out=h)
        return 1.0 / (1.0 + np.exp(-h))

    def p_batch(self, X):
        return 1.0 / (1.0 + np.exp(-self.logit_batch(X)))


# q_tabrak NetDenganP: satu lintasan numpy untuk model jauh dan model P (lihat NetDenganP).
# False = dua forward terpisah seperti sebelum 2026-11-08 (pembanding, drl.ukur_node --forward-terpisah).
GABUNG_FORWARD = True


class NetDenganP:
    """Jaringan DQfD (JaringanNumpy) dengan P aksi menabrak dari model P terpisah. Antarmuka sama
    dengan JaringanNumpy berkepala_tabrak (q, qv, q_batch, v_batch, q_tabrak, tabrak_batch):
    Q dan V dari model jauh, P dari model P. Dipasang DQfDPlanner bila id0N_kepala_p.pt ada.

    FORWARD GABUNGAN (GABUNG_FORWARD, sejak 2026-11-08). q_tabrak - satu panggilan per simpul
    pencarian - menghitung kedua jaringan dalam satu lintasan, karena waktunya didominasi
    overhead per operasi numpy, bukan hitungannya:
        lapis 1   satu matriks 47 x (256 + 256)
        lapis 2   satu matmul batch (2, 256, 256)
        lapis 3   V|A-hidden model jauh dan keluaran model P (dua matmul, tanpa blok nol)
        keluaran  dueling Q = V + A - rata(A) dilipat ke satu matriks 256 x 14
    P sama persis dengan ModelPNumpy.p. Q berbeda < 1e-5 dari JaringanNumpy.q (|Q| sampai
    ~13): urutan penjumlahan float32 berubah karena V dan rata(A) sudah dilipat ke bobot.
    Mac M1 Pro: 18.8 us per panggilan, terpisah 26.5 us. q, qv, q_batch tetap lewat model jauh.
    """

    def __init__(self, net, model_p, gabung=None):
        self.net, self.model_p = net, model_p
        self.kepala_tabrak = True
        self.n_aksi, self.n_obs, self.nh = net.n_aksi, net.n_obs, net.nh
        self.gabung = GABUNG_FORWARD if gabung is None else bool(gabung)
        if self.gabung:
            self.gabung = self._siapkan_gabung()

    def _siapkan_gabung(self):
        """Bobot gabungan; False bila bentuk kedua jaringan tidak cocok (forward terpisah dipakai)."""
        net, mp = self.net, self.model_p
        h = net.w1.shape[1]
        if (len(mp.w) != 3 or mp.w[0].shape != net.w1.shape or net.w2.shape != (h, h)
                or mp.w[1].shape != (h, h) or net.w_va.shape[0] != h):
            return False
        n = self.n_aksi
        self._h = h
        self._w1 = np.ascontiguousarray(np.concatenate([net.w1, mp.w[0]], axis=1))
        self._b1 = np.concatenate([net.b1, mp.b[0]])
        self._w2 = np.ascontiguousarray(np.stack([net.w2, mp.w[1]]))
        self._b2 = np.stack([net.b2, mp.b[1]])
        self._w_va, self._b_va = net.w_va, net.b_va
        self._w_p, self._b_p = mp.w[2], mp.b[2]
        # Q_a = V + A_a - rata(A): baris V (nol di kolom A) dan baris A (nol di kolom V) satu matriks.
        wv = net.w_out[:, :1].astype(np.float64)
        wa = net.w_out[:, 1:1 + n].astype(np.float64)
        self._w_q = np.ascontiguousarray((wv + wa - wa.mean(axis=1, keepdims=True)).astype(np.float32))
        bv = net.b_out[:1].astype(np.float64)
        ba = net.b_out[1:1 + n].astype(np.float64)
        self._b_q = (bv + ba - ba.mean()).astype(np.float32)
        return True

    def _q_tabrak_gabung(self, obs):
        h = np.asarray(obs, dtype=np.float32) @ self._w1
        h += self._b1
        np.maximum(h, 0.0, out=h)
        h = np.matmul(h.reshape(2, 1, self._h), self._w2).reshape(2, self._h)
        h += self._b2
        np.maximum(h, 0.0, out=h)
        hva = h[0] @ self._w_va
        hva += self._b_va
        np.maximum(hva, 0.0, out=hva)
        q = hva @ self._w_q
        q += self._b_q
        p = h[1] @ self._w_p
        p += self._b_p
        np.negative(p, out=p)
        np.exp(p, out=p)
        p += 1.0
        np.reciprocal(p, out=p)
        return q, p

    def q(self, obs):
        return self.net.q(obs)

    def qv(self, obs):
        return self.net.qv(obs)

    def q_batch(self, X):
        return self.net.q_batch(X)

    def v_batch(self, X):
        return self.net.v_batch(X)

    def q_tabrak(self, obs):
        if self.gabung:
            return self._q_tabrak_gabung(obs)
        return self.net.q(obs), self.model_p.p(obs)

    def tabrak_batch(self, X):
        return self.model_p.p_batch(X)


def muat_model_p(jalur_pt):
    """(ModelPNumpy, meta) dari model P terpisah .pt, lewat cache .npz di sampingnya (md5 .pt).
    torch hanya diimpor bila cache belum ada / basi - sama dengan muat_dari_pt."""
    md5 = _md5(jalur_pt)
    jc = jalur_cache(jalur_pt)
    if os.path.exists(jc):
        with np.load(jc) as f:
            if str(f["sumber_md5"]) == md5:
                return (ModelPNumpy({k: f[k] for k in f.files if k.endswith((".weight", ".bias"))}),
                        json.loads(str(f["meta"])))
    import torch
    ck = torch.load(jalur_pt, map_location="cpu", weights_only=False)
    bobot = {k: v.detach().cpu().numpy() for k, v in ck["sd"].items()}
    sementara = jc + ".tmp.npz"
    np.savez(sementara, sumber_md5=np.array(md5), meta=np.array(json.dumps(ck["meta"])), **bobot)
    os.replace(sementara, jc)
    return ModelPNumpy(bobot), ck["meta"]


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
