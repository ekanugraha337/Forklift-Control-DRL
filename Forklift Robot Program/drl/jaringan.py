"""
jaringan.py - Dueling DQN untuk DQfD (PyTorch, HANYA untuk gradien saat latih).

    input 47 -> FC1 256 -> FC2 256 -+-> V-hidden 128 -> V(s)    1 -+
                                    +-> A-hidden 128 -> A(s,a) 14  +-> Q = V + A - rata(A)

Satu model per robot: model jauh, meniru Hybrid A* (lingkungan.N_OBS masukan,
lingkungan.N_AKSI primitif), 145.807 parameter. Susunannya sama untuk ID 01
dan ID 02; bobotnya terpisah.

ATURAN ver-1-6: forward untuk rollout, evaluasi, dasbor, dan perencana memakai
drl/jaringan_numpy.JaringanNumpy dengan bobot yang disalin dari jaringan ini.

Q dalam satuan -(biaya-sisa ke tujuan) / SKALA_REWARD, jadi -Q x SKALA_REWARD
adalah perkiraan biaya A* (satuan sel) dari keadaan itu, termasuk tembakan RS.
"""
import torch.nn as nn
import torch.nn.functional as F

# Reward = -biaya / SKALA_REWARD (100). Didefinisikan di jaringan_numpy (tanpa
# torch) supaya perencana operasional bisa memakainya tanpa mengimpor torch.
from drl.jaringan_numpy import SKALA_REWARD  # noqa: E402,F401

Q_TERLARANG = -1e9


class JaringanDQfD(nn.Module):
    def __init__(self, n_aksi, n_obs, lebar=256, cabang=128):
        super().__init__()
        self.fc1 = nn.Linear(n_obs, lebar)
        self.fc2 = nn.Linear(lebar, lebar)
        self.v1 = nn.Linear(lebar, cabang)
        self.v2 = nn.Linear(cabang, 1)
        self.a1 = nn.Linear(lebar, cabang)
        self.a2 = nn.Linear(cabang, n_aksi)

    def forward(self, x):
        h2 = F.relu(self.fc2(F.relu(self.fc1(x))))
        v = self.v2(F.relu(self.v1(h2)))
        a = self.a2(F.relu(self.a1(h2)))
        return v + a - a.mean(dim=1, keepdim=True)


def bermask(q, mask):
    """Q dengan aksi terlarang dibuat Q_TERLARANG, supaya tidak pernah jadi argmax/max."""
    return q.masked_fill(~mask, Q_TERLARANG)


def jumlah_parameter(net):
    return sum(p.numel() for p in net.parameters())
