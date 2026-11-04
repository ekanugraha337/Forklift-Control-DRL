"""
jaringan.py - Dueling DQN untuk DQfD. Susunannya di catatan Obsidian bagian 7.

    input m -> FC1 256 -> FC2 256 -+-> V-hidden 128 -> V(s)   1 -+
                                   +-> A-hidden 128 -> A(s,a) n  +-> Q = V + A - rata(A)

m, n = jumlah masukan dan aksi jenis modelnya (lingkungan.n_obs / n_aksi).
Versi aksi 6 punya DUA model per robot:
    jauh   meniru Hybrid A*:   m = 47, n = 14  -> 145.807 parameter
    dekat  memilih kata Reeds-Shepp:
                               m =  9, n = 88  -> 145.625 parameter
           (9 masukan RS; 44 kata x 2 titik sambung. Versi 4: 9 / 30, 138.143;
            versi 5: 47 / 30, 147.871)
Lapis tersembunyi sama; yang meniru algoritmanya adalah masukan & keluarannya.
Susunannya sama untuk ID 01 dan ID 02; bobotnya terpisah.

Q dalam satuan -(biaya-sisa ke tujuan) / SKALA_REWARD, jadi -Q x SKALA_REWARD
adalah perkiraan biaya A* (satuan sel) dari keadaan itu.
"""
import torch
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
