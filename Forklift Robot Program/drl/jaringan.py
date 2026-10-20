"""
jaringan.py - Dueling DQN untuk DQfD. Susunannya di catatan Obsidian bagian 7.

    input 47 -> FC1 256 -> FC2 256 -+-> V-hidden 128 -> V(s)   1 -+
                                    +-> A-hidden 128 -> A(s,a) 32 +-> Q = V + A - rata(A)

148.129 parameter (ruang aksi versi 2, 32 aksi; versi 1 dengan 15 aksi:
145.936). Sama untuk ID 01 dan ID 02; bobotnya terpisah.

Q dalam satuan -(biaya-sisa ke tujuan) / SKALA_REWARD, jadi -Q x SKALA_REWARD
adalah perkiraan biaya A* (satuan sel) dari keadaan itu.
"""
import torch
import torch.nn as nn
import torch.nn.functional as F

from drl.lingkungan import N_AKSI, N_OBS

# Reward = -biaya / SKALA_REWARD. Biaya per langkah median 17 (primitif penuh;
# primitif pendek lebih murah), jadi Q ada di kisaran satuan sampai puluhan.
SKALA_REWARD = 100.0

Q_TERLARANG = -1e9


class JaringanDQfD(nn.Module):
    def __init__(self, n_obs=N_OBS, n_aksi=N_AKSI, lebar=256, cabang=128):
        super().__init__()
        self.fc1 = nn.Linear(n_obs, lebar)
        self.fc2 = nn.Linear(lebar, lebar)
        self.v1 = nn.Linear(lebar, cabang)
        self.v2 = nn.Linear(cabang, 1)
        self.a1 = nn.Linear(lebar, cabang)
        self.a2 = nn.Linear(cabang, n_aksi)

    def forward(self, x, aktivasi=False):
        h1 = F.relu(self.fc1(x))
        h2 = F.relu(self.fc2(h1))
        hv = F.relu(self.v1(h2))
        ha = F.relu(self.a1(h2))
        v = self.v2(hv)
        a = self.a2(ha)
        q = v + a - a.mean(dim=1, keepdim=True)
        if aktivasi:
            return q, {"fc1": h1, "fc2": h2, "v_hidden": hv, "a_hidden": ha, "v": v, "a": a}
        return q


def bermask(q, mask):
    """Q dengan aksi terlarang dibuat Q_TERLARANG, supaya tidak pernah jadi argmax/max."""
    return q.masked_fill(~mask, Q_TERLARANG)


def jumlah_parameter(net):
    return sum(p.numel() for p in net.parameters())
