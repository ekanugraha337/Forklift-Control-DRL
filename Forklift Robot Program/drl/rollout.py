"""
rollout.py - menjalankan jaringan DQfD di lingkungan perencanaan.

Satu mesin untuk semua pemakai: evaluasi, latihan sendiri, dasbor, dan nanti
perencana operasional (tahap 9). Beberapa episode dijalankan serempak supaya
jaringan cukup dipanggil sekali per langkah untuk semuanya.

    Skenario       satu pasangan awal-tujuan + peta, margin, rintangan
    PabrikEnv      Skenario -> LingkunganRencana yang sudah di-reset
    KumpulanRollout  N episode serempak; langkah() maju satu aksi untuk semua
    jalankan()     sekumpulan skenario sampai selesai semua
"""
import math
from dataclasses import dataclass, field

import numpy as np
import torch

import rencana as R
from drl import lingkungan as L
from drl import skenario as S
from drl.jaringan import SKALA_REWARD, bermask


@dataclass
class Skenario:
    peta: str
    margin: float
    awal: tuple                       # (gx, gy, yaw) sel
    tujuan: tuple                     # (gx, gy, yaw atau None) sel
    rintangan: np.ndarray = None      # (4, 2) cm, badan robot lain; None = tidak ada
    biaya_astar: float = None         # g-cost A*; None = tidak diketahui / A* gagal
    aksi_demo: list = None            # aksi A*; None = bukan dari demo
    nama: str = ""
    info: dict = field(default_factory=dict)


def muat_npz(jalur):
    """Dataset .npz ke dict larik di memori. np.load terkompresi mendekompres
    ulang SETIAP kali d["obs"] diakses (~0,02 s); di loop per episode itu
    menjadi menit."""
    with np.load(jalur) as f:
        return {k: f[k] for k in f.files}


def _tujuan(t):
    return (float(t[0]), float(t[1]), None if np.isnan(t[2]) else float(t[2]))


def dari_demo(d, e):
    """Episode ke-e dataset demo_id0N.npz -> Skenario (dengan aksi & biaya A*)."""
    poli = d["ep_rintangan"][e]
    s0, T = int(d["ep_mulai"][e]), int(d["ep_langkah"][e])
    return Skenario(peta=str(d["ep_peta"][e]), margin=float(d["ep_margin"][e]),
                    awal=tuple(float(v) for v in d["ep_awal"][e]), tujuan=_tujuan(d["ep_tujuan"][e]),
                    rintangan=None if np.isnan(poli).any() else poli,
                    biaya_astar=float(d["ep_biaya"][e]),
                    aksi_demo=[int(a) for a in d["aksi"][s0:s0 + T]],
                    nama=f"demo#{e}")


def dari_negatif(n, i):
    poli = n["rintangan"][i]
    return Skenario(peta=str(n["peta"][i]), margin=float(n["margin"][i]),
                    awal=tuple(float(v) for v in n["awal"][i]), tujuan=_tujuan(n["tujuan"][i]),
                    rintangan=None if np.isnan(poli).any() else poli,
                    nama=f"negatif#{i}", info={"alasan": str(n["alasan"][i])})


class PabrikEnv:
    """Membangun lingkungan untuk skenario. Arena & Perencana di-cache per peta."""

    def __init__(self, profil):
        self.p = profil
        self._per = {}

    def arena(self, peta):
        return S._arena(peta)

    def omap(self, sk):
        arena = self.arena(sk.peta)
        if sk.rintangan is None:
            return arena.omap
        if sk.peta not in self._per:
            self._per[sk.peta] = R.Perencana(arena, self.p)
        return self._per[sk.peta].peta_dengan([sk.rintangan])

    def buat(self, sk):
        """(env, obs, info). Lingkungan BARU tiap kali: keadaan perencana
        (_hcost, _approach_dirs) tersimpan di objeknya, jadi tidak boleh dipakai
        bersama dua episode yang berjalan serempak."""
        env = L.LingkunganRencana(self.arena(sk.peta), self.p, margin_cm=sk.margin)
        obs, info = env.reset(sk.awal, sk.tujuan, self.omap(sk))
        return env, obs, info

    def acak(self, rng, peta):
        """Skenario acak baru di peta itu, dengan sebaran yang sama dengan dataset."""
        arena = self.arena(peta)
        if peta not in self._per:
            self._per[peta] = R.Perencana(arena, self.p)
        env = {m: L.LingkunganRencana(arena, self.p, margin_cm=m)
               for m in (S.konfig.SAFETY_MARGIN_CM, *S.konfig.MARGIN_CADANGAN_CM)}
        for _ in range(20):
            margin, _, poli, pasangan = S.acak_satu(arena, self._per[peta], env, rng)
            if pasangan is not None:
                return Skenario(peta=peta, margin=margin, awal=pasangan[0], tujuan=pasangan[1],
                                rintangan=poli, nama=f"acak:{peta}")
        return None


class KumpulanRollout:
    """Beberapa episode yang maju serempak, satu aksi per langkah().

    net: jaringan di CPU (forward satu keadaan 0,03 ms; lewat GPU justru lebih
    lambat karena ongkos transfer). epsilon: peluang aksi acak di antara aksi
    yang SAH - aksi terlarang tidak pernah dipilih, termasuk saat eksplorasi.
    """

    def __init__(self, pabrik, n_slot):
        self.pabrik = pabrik
        self.slot = [None] * n_slot

    def isi(self, i, sk):
        """Pasang skenario ke slot i. Kembalian (ok, alasan): ok False bila reset
        menolak - pasangan tidak sah, atau buntu sejak pose awal."""
        env, obs, info = self.pabrik.buat(sk)
        if obs is None:
            self.slot[i] = None
            return False, info["alasan"]
        self.slot[i] = {"sk": sk, "env": env, "obs": [obs], "aksi": [], "reward": [],
                        "mask": [env.mask()], "q_pilih": []}
        return True, ""

    def aktif(self):
        return [i for i, s in enumerate(self.slot) if s is not None]

    def langkah(self, net, epsilon=0.0, rng=None):
        """Maju satu aksi di semua slot aktif. Kembalian: [(slot, episode)] yang selesai."""
        idx = self.aktif()
        if not idx:
            return []
        X = torch.as_tensor(np.stack([self.slot[i]["obs"][-1] for i in idx]))
        M = torch.as_tensor(np.stack([self.slot[i]["mask"][-1] for i in idx]))
        with torch.no_grad():
            q = bermask(net(X), M)
        pilih = q.argmax(1).numpy()
        selesai = []
        for j, i in enumerate(idx):
            s = self.slot[i]
            a = int(pilih[j])
            if epsilon and rng is not None and rng.random() < epsilon:
                a = int(rng.choice(np.flatnonzero(s["mask"][-1])))
            obs, r, akhir, info = s["env"].step(a)
            s["obs"].append(obs)
            s["aksi"].append(a)
            s["reward"].append(r)
            s["q_pilih"].append(float(q[j, a]))
            s["mask"].append(s["env"].mask() if not akhir else np.zeros(L.N_AKSI, bool))
            if akhir:
                selesai.append((i, ringkas_episode(s, info)))
                self.slot[i] = None
        return selesai


def ringkas_episode(s, info):
    env, sk = s["env"], s["sk"]
    biaya = env.biaya
    return {
        "sk": sk, "sukses": bool(info["sukses"]), "alasan": info["alasan"],
        "terpotong": bool(info["terpotong"]), "biaya": biaya, "langkah": len(s["aksi"]),
        "rasio": biaya / sk.biaya_astar if info["sukses"] and sk.biaya_astar else None,
        "obs": np.asarray(s["obs"], np.float32), "aksi": np.asarray(s["aksi"], np.int64),
        # reward sudah diskalakan untuk jaringan
        "reward": np.asarray(s["reward"], np.float32) / SKALA_REWARD,
        "mask": np.asarray(s["mask"], bool), "jalur": env.jalur(),
    }


def jalankan(net, pabrik, daftar_sk, n_slot=32, epsilon=0.0, rng=None):
    """Jalankan semua skenario sampai selesai. Urutan hasil = urutan daftar_sk
    (skenario yang ditolak reset diberi alasan 'tidak sah')."""
    kumpulan = KumpulanRollout(pabrik, n_slot)
    hasil = [None] * len(daftar_sk)
    antre = list(range(len(daftar_sk)))
    milik = {}
    while antre or kumpulan.aktif():
        for i in range(n_slot):
            while kumpulan.slot[i] is None and antre:
                k = antre.pop(0)
                ok, alasan = kumpulan.isi(i, daftar_sk[k])
                if ok:
                    milik[i] = k
                else:
                    hasil[k] = {"sk": daftar_sk[k], "sukses": False, "alasan": alasan,
                                "langkah": 0, "biaya": 0.0, "rasio": None}
        for i, ep in kumpulan.langkah(net, epsilon, rng):
            hasil[milik.pop(i)] = ep
    return hasil


def transisi_episode(obs, aksi, reward, mask, terminal, gamma, n):
    """Episode -> larik transisi 1-langkah dan n-langkah untuk buffer.

    obs (T+1, 47), aksi (T,), reward (T,) SUDAH diskalakan, mask (T+1, 15).
    terminal: episode berakhir di keadaan terminal (tiba/tersambung/buntu).
    False = terpotong MAKS_LANGKAH - keadaan terakhir tetap di-bootstrap.
    """
    T = len(aksi)
    akhir1 = np.zeros(T, bool)
    akhir1[-1] = terminal
    rn = np.zeros(T, np.float32)
    obsn = np.empty((T, obs.shape[1]), np.float32)
    maskn = np.empty((T, mask.shape[1]), bool)
    akhirn = np.zeros(T, bool)
    gn = np.empty(T, np.float32)
    pangkat = gamma ** np.arange(n)
    for t in range(T):
        k = min(n, T - t)
        rn[t] = float(np.dot(pangkat[:k], reward[t:t + k]))
        obsn[t] = obs[t + k]
        maskn[t] = mask[t + k]
        akhirn[t] = terminal and t + k == T
        gn[t] = gamma ** k
    return {"obs": obs[:T], "aksi": aksi, "mask": mask[:T],
            "r1": reward.astype(np.float32), "obs1": obs[1:], "mask1": mask[1:], "akhir1": akhir1,
            "g1": np.full(T, gamma, np.float32),
            "rn": rn, "obsn": obsn, "maskn": maskn, "akhirn": akhirn, "gn": gn}


def episode_demo(d, e):
    """Episode ke-e dataset -> (obs, aksi, reward diskalakan, mask), bentuk transisi_episode."""
    s0, T = int(d["ep_mulai"][e]), int(d["ep_langkah"][e])
    obs = np.concatenate([d["obs"][s0:s0 + T], d["obs_next"][s0 + T - 1:s0 + T]])
    mask = np.concatenate([d["mask"][s0:s0 + T], d["mask_next"][s0 + T - 1:s0 + T]])
    return obs, d["aksi"][s0:s0 + T].astype(np.int64), d["reward"][s0:s0 + T] / SKALA_REWARD, mask


def jarak_lurus_cm(pabrik, sk):
    a = pabrik.arena(sk.peta)
    return a.panjang_ke_cm(math.hypot(sk.tujuan[0] - sk.awal[0], sk.tujuan[1] - sk.awal[1]))
