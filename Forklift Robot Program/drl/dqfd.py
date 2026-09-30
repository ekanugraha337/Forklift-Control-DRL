"""
dqfd.py - Deep Q-learning from Demonstrations (Hester dkk. 2018).

    J = J_DQ + lam_n * J_n + lam_e * J_E  (+ L2 lewat weight_decay)

    J_DQ  TD 1-langkah, Double DQN (argmax jaringan online, nilai jaringan target)
    J_n   TD n-langkah (n = 10), sama
    J_E   large-margin, HANYA sampel demo:  max_a[Q(s,a) + l(a_E,a)] - Q(s,a_E)
          l = margin untuk a != a_E, 0 untuk a_E

Semua max/argmax hanya atas aksi SAH (mask). Tanpa itu jaringan akan dihukum
atau dibujuk oleh aksi yang toh tidak pernah boleh dipilih.

Buffer: prioritized replay proporsional. Data demo di depan dan PERMANEN;
data agen di belakangnya, melingkar. Prioritas demo mendapat bonus eps_demo
supaya tetap sering terambil walau TD-nya sudah kecil.
"""
import copy

import numpy as np
import torch
import torch.nn.functional as F

from drl.jaringan import JaringanDQfD, bermask
from drl.lingkungan import N_AKSI, N_OBS

KOLOM = {
    "obs": ((N_OBS,), torch.float32), "aksi": ((), torch.int64), "mask": ((N_AKSI,), torch.bool),
    "r1": ((), torch.float32), "obs1": ((N_OBS,), torch.float32), "mask1": ((N_AKSI,), torch.bool),
    "akhir1": ((), torch.bool), "g1": ((), torch.float32),
    "rn": ((), torch.float32), "obsn": ((N_OBS,), torch.float32), "maskn": ((N_AKSI,), torch.bool),
    "akhirn": ((), torch.bool), "gn": ((), torch.float32),
}


class Buffer:
    def __init__(self, kapasitas, device, alpha=0.4, eps_agen=1e-3, eps_demo=1.0):
        self.N, self.device = kapasitas, device
        self.alpha, self.eps_agen, self.eps_demo = alpha, eps_agen, eps_demo
        self.t = {k: torch.zeros((kapasitas, *bentuk), dtype=dt, device=device)
                  for k, (bentuk, dt) in KOLOM.items()}
        self.demo = np.zeros(kapasitas, bool)
        self.pa = np.zeros(kapasitas)          # prioritas^alpha
        self.pa_maks = 1.0
        self.n_demo = 0
        self.isi = 0
        self.ptr = 0

    def tambah(self, tr, demo=False):
        T = len(tr["aksi"])
        if demo:
            if self.isi != self.n_demo:
                raise RuntimeError("data demo harus ditambahkan sebelum data agen")
            if self.n_demo + T > self.N:
                raise RuntimeError("buffer tidak cukup untuk data demo")
            idx = np.arange(self.n_demo, self.n_demo + T)
            self.n_demo += T
            self.isi = self.ptr = self.n_demo
        else:
            ruang = self.N - self.n_demo
            if ruang <= 0:
                raise RuntimeError("buffer tidak menyisakan ruang untuk data agen")
            idx = self.n_demo + (self.ptr - self.n_demo + np.arange(T)) % ruang
            self.ptr = int(self.n_demo + (self.ptr - self.n_demo + T) % ruang)
            self.isi = min(self.N, self.isi + T)
        it = torch.as_tensor(idx, device=self.device)
        for k in KOLOM:
            self.t[k][it] = torch.as_tensor(np.asarray(tr[k]), device=self.device).to(self.t[k].dtype)
        self.demo[idx] = demo
        self.pa[idx] = self.pa_maks

    def ambil(self, B, beta, rng):
        pa = self.pa[:self.isi]
        cum = np.cumsum(pa)
        idx = np.minimum(np.searchsorted(cum, rng.uniform(0.0, cum[-1], B)), self.isi - 1)
        w = (self.isi * pa[idx] / cum[-1]) ** (-beta)
        w /= w.max()
        it = torch.as_tensor(idx, device=self.device)
        batch = {k: v[it] for k, v in self.t.items()}
        batch["demo"] = torch.as_tensor(self.demo[idx], device=self.device)
        batch["w"] = torch.as_tensor(w, dtype=torch.float32, device=self.device)
        return idx, batch

    def perbarui_prioritas(self, idx, td_abs):
        p = td_abs + self.eps_agen + self.eps_demo * self.demo[idx]
        self.pa[idx] = p ** self.alpha
        self.pa_maks = max(self.pa_maks, float(self.pa[idx].max()))


class AgenDQfD:
    def __init__(self, device, lr=1e-4, gamma=0.995, n=10, lam_n=1.0, lam_e=1.0,
                 l2=1e-5, margin=0.8, target_setiap=2000, seed=0):
        torch.manual_seed(seed)
        self.device = device
        self.gamma, self.n = gamma, n
        self.lam_n, self.lam_e, self.margin = lam_n, lam_e, margin
        self.target_setiap = target_setiap
        self.net = JaringanDQfD().to(device)
        self.target = copy.deepcopy(self.net)
        self.opt = torch.optim.Adam(self.net.parameters(), lr=lr, weight_decay=l2)
        # Salinan CPU untuk rollout & dasbor, disinkronkan sesekali.
        self.cpu = JaringanDQfD()
        self.sinkron_cpu()
        self.langkah = 0

    def sinkron_cpu(self):
        self.cpu.load_state_dict({k: v.detach().cpu() for k, v in self.net.state_dict().items()})
        self.cpu.eval()

    def _target(self, obs, mask, akhir, g, r):
        with torch.no_grad():
            a = bermask(self.net(obs), mask).argmax(1, keepdim=True)
            qt = self.target(obs).gather(1, a).squeeze(1)
            return r + torch.where(akhir, torch.zeros_like(qt), g * qt)

    def latih_satu(self, buf, B, beta, rng):
        idx, b = buf.ambil(B, beta, rng)
        q = self.net(b["obs"])
        qa = q.gather(1, b["aksi"][:, None]).squeeze(1)
        y1 = self._target(b["obs1"], b["mask1"], b["akhir1"], b["g1"], b["r1"])
        yn = self._target(b["obsn"], b["maskn"], b["akhirn"], b["gn"], b["rn"])
        w = b["w"]
        j_dq = (w * F.smooth_l1_loss(qa, y1, reduction="none")).mean()
        j_n = (w * F.smooth_l1_loss(qa, yn, reduction="none")).mean()

        demo = b["demo"]
        if demo.any():
            l = torch.full_like(q, self.margin)
            l.scatter_(1, b["aksi"][:, None], 0.0)
            maks = bermask(q + l, b["mask"]).max(1).values
            j_e = (w[demo] * (maks - qa)[demo]).mean()
        else:
            j_e = torch.zeros((), device=self.device)

        loss = j_dq + self.lam_n * j_n + self.lam_e * j_e
        self.opt.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.net.parameters(), 10.0)
        self.opt.step()

        td = (y1 - qa).detach().abs().cpu().numpy()
        buf.perbarui_prioritas(idx, td)
        self.langkah += 1
        if self.langkah % self.target_setiap == 0:
            self.target.load_state_dict(self.net.state_dict())

        with torch.no_grad():
            setuju = (bermask(q, b["mask"]).argmax(1) == b["aksi"])[demo].float().mean() \
                if demo.any() else torch.tensor(float("nan"))
        return {"j_dq": j_dq.item(), "j_n": j_n.item(), "j_e": j_e.item(),
                "q": qa.mean().item(), "td": float(td.mean()),
                "setuju_batch": float(setuju), "porsi_demo": float(demo.float().mean())}

    def simpan(self, jalur, **meta):
        torch.save({"net": self.net.state_dict(), "target": self.target.state_dict(),
                    "opt": self.opt.state_dict(), "langkah": self.langkah, **meta}, jalur)


def muat_jaringan(jalur, device="cpu"):
    """Jaringan dari checkpoint, siap dipakai (eval)."""
    ck = torch.load(jalur, map_location=device, weights_only=False)
    net = JaringanDQfD().to(device)
    net.load_state_dict(ck["net"])
    net.eval()
    return net, ck
