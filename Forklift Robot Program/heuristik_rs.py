"""
heuristik_rs.py - heuristik non-holonomik Hybrid A*: biaya Reeds-Shepp tanpa rintangan.

Dolgov dkk. (2010) memakai DUA heuristik dan mengambil maksimumnya:
  1. non-holonomik tanpa rintangan: kurva Reeds-Shepp terpendek ke tujuan.
     Memperhitungkan arah hadap dan radius belok, mengabaikan rintangan.
  2. holonomik dengan rintangan: jarak Dijkstra 2D yang memutari rintangan
     (HybridAStarPlanner._holonomic_heuristic).
Semua Hybrid A* di program ini (pembanding `astar` dan demonstrator DQfD)
memakai keduanya: h = max(h_RS, h_D). Modul ini menyediakan h_RS sebagai TABEL
(lookup table), dihitung sekali per profil kemudi, supaya satu pembacaan cukup
beberapa mikrodetik.

Isi tabel = biaya Eq. (2) jalur RS termurah (rumus _rs_cost Hybrid A*: maju 1x,
mundur reverse_cost x, ganti arah, kelengkungan, ganti kelengkungan; TANPA
penalti clearance) dari pose node ke pose tujuan, sebagai fungsi pose node di
KERANGKA TUJUAN (u, v, psi):
    u, v  posisi node relatif tujuan, diputar -yaw_tujuan (sel)
    psi   yaw node - yaw tujuan
Grid: u, v di [-jangkau, jangkau] berjarak reso sel, psi n_yaw sudut.
Simetri cermin (v -> -v, psi -> -psi, L <-> R) dipakai saat membangun.
Di luar jangkau: jarak lurus (batas bawah panjang RS).

    python heuristik_rs.py            bangun tabel profil robot terpilih (cache di model_dqn/)
"""
import hashlib
import json
import math
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

from forklift_planner import reeds_shepp_paths, wrap_angle

DIR_CACHE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model_dqn")
JANGKAU_CM = 340.0          # cukup untuk peta 2,5 x 2,2 m (diagonal +-330 cm)
RESO_CM = 5.0               # sama orde dengan lattice Hybrid A* (4,9 cm)
N_YAW = 72                  # 5 derajat


def _param(planner):
    """Parameter yang menentukan isi tabel (untuk nama cache)."""
    return {"radius": round(planner.radius, 6), "kappa_max": round(planner.kappa_max, 9),
            "reverse_cost": planner.reverse_cost, "switch_cost": round(planner.switch_cost, 6),
            "steer_cost": planner.steer_cost, "steer_change_cost": planner.steer_change_cost}


class _BiayaRS:
    """Salinan rumus HybridAStarPlanner._rs_cost tanpa objek perencana (untuk proses anak)."""

    def __init__(self, p):
        self.p = p

    def __call__(self, u, v, psi):
        p = self.p
        best = None
        for segs, _ in reeds_shepp_paths((u, v, psi), (0.0, 0.0, 0.0), p["radius"]):
            cost, d_prev, k_prev = 0.0, 0, 0.0
            for kind, length in segs:
                d = 1 if length > 0 else -1
                k = 0.0 if kind == 'S' else (p["kappa_max"] if kind == 'L' else -p["kappa_max"])
                cost += abs(length) * (1.0 if d > 0 else p["reverse_cost"])
                cost += p["steer_cost"] * abs(length) * abs(k) / p["kappa_max"]
                cost += p["steer_change_cost"] * abs(k - k_prev) / p["kappa_max"]
                if d_prev != 0 and d != d_prev:
                    cost += p["switch_cost"]
                d_prev, k_prev = d, k
            if best is None or cost < best:
                best = cost
        return best


def _baris(args):
    """Satu nilai u: biaya untuk semua v >= 0 dan semua psi."""
    p, u, vs, psis = args
    f = _BiayaRS(p)
    out = np.empty((len(vs), len(psis)), dtype=np.float32)
    for j, v in enumerate(vs):
        for k, psi in enumerate(psis):
            c = f(u, v, psi)
            out[j, k] = c if c is not None else math.hypot(u, v)
    return out


def bangun(planner, jangkau=None, reso=None, n_yaw=N_YAW, proses=None, cetak=True):
    """Hitung tabel (float32, [nu, nv, n_yaw]) untuk parameter kemudi & biaya planner ini."""
    jangkau = jangkau if jangkau is not None else JANGKAU_CM
    reso = reso if reso is not None else RESO_CM
    p = _param(planner)
    n = int(round(jangkau / reso))
    us = [(i - n) * reso for i in range(2 * n + 1)]
    vs_pos = [i * reso for i in range(n + 1)]                    # v >= 0 saja, sisanya cermin
    psis = [k * 2.0 * math.pi / n_yaw - math.pi for k in range(n_yaw)]
    t0 = time.time()
    tugas = [(p, u, vs_pos, psis) for u in us]
    with Pool(proses or os.cpu_count()) as pool:
        baris = pool.map(_baris, tugas, chunksize=1)
    T = np.empty((2 * n + 1, 2 * n + 1, n_yaw), dtype=np.float32)
    for i, b in enumerate(baris):
        T[i, n:, :] = b                                          # v >= 0
        for j in range(1, n + 1):                                # v < 0: cermin
            T[i, n - j, :] = b[j, [(n_yaw - k) % n_yaw for k in range(n_yaw)]]
    if cetak:
        print(f"[tabel RS] {T.shape} dibangun dalam {time.time() - t0:.0f} s", flush=True)
    return T, {"jangkau": jangkau, "reso": reso, "n_yaw": n_yaw, **p}


class TabelRS:
    """Heuristik RS tanpa rintangan dari tabel. h(x, y, yaw, gx, gy, gyaw) dalam satuan biaya planner."""

    def __init__(self, T, meta):
        self.meta = meta
        self.n = (T.shape[0] - 1) // 2
        self.ny = T.shape[2]
        self.reso = float(meta["reso"])
        self.batas = self.n * self.reso
        self.T = T
        # Daftar datar: pembacaan elemen ~10x lebih cepat daripada indeks numpy skalar.
        self._datar = T.reshape(-1).tolist()
        self._s1 = T.shape[1] * T.shape[2]
        self._s2 = T.shape[2]
        self._k_yaw = self.ny / (2.0 * math.pi)

    def h(self, x, y, yaw, gx, gy, gyaw):
        dx, dy = x - gx, y - gy
        c, s = math.cos(gyaw), math.sin(gyaw)
        u = c * dx + s * dy
        v = -s * dx + c * dy
        b = self.batas
        if u <= -b or u >= b or v <= -b or v >= b:
            return math.hypot(dx, dy)
        n, r = self.n, self.reso
        iu = int(u / r + n + 0.5)
        iv = int(v / r + n + 0.5)
        psi = (yaw - gyaw + math.pi) % (2.0 * math.pi)
        k = int(psi * self._k_yaw + 0.5) % self.ny
        return self._datar[iu * self._s1 + iv * self._s2 + k]


_CACHE = {}


def tabel_untuk(planner, proses=None):
    """TabelRS untuk parameter planner ini: dari memori, dari file cache, atau dibangun."""
    p = _param(planner)
    meta0 = {"jangkau": JANGKAU_CM, "reso": RESO_CM, "n_yaw": N_YAW, **p}
    kunci = hashlib.sha1(json.dumps(meta0, sort_keys=True).encode()).hexdigest()[:10]
    if kunci in _CACHE:
        return _CACHE[kunci]
    jalur = os.path.join(DIR_CACHE, f"tabel_rs_{kunci}.npz")
    if os.path.exists(jalur):
        z = np.load(jalur, allow_pickle=False)
        T, meta = z["T"], json.loads(str(z["meta"]))
    else:
        T, meta = bangun(planner, proses=proses)
        os.makedirs(DIR_CACHE, exist_ok=True)
        np.savez_compressed(jalur, T=T, meta=json.dumps(meta))
        print(f"[tabel RS] disimpan: {jalur}", flush=True)
    _CACHE[kunci] = TabelRS(T, meta)
    return _CACHE[kunci]


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import arena as A
    import konfig
    import rencana as R
    ar = A.Arena(A.pilih("Kalibrasi"))
    per = R.Perencana(ar, konfig.profil(), jenis="astar")
    tab = tabel_untuk(per.planner)
    print("meta:", tab.meta)
    # Uji singkat: tabel vs hitung langsung di beberapa pose acak dalam jangkau.
    rng = np.random.default_rng(0)
    f = _BiayaRS(_param(per.planner))
    gal = []
    for _ in range(300):
        u, v = rng.uniform(-300, 300, 2)
        psi = rng.uniform(-math.pi, math.pi)
        langsung = f(u, v, psi)
        dari_tabel = tab.h(u, v, psi, 0.0, 0.0, 0.0)
        gal.append(abs(dari_tabel - langsung) / max(langsung, 1.0))
    print(f"galat relatif tabel (pembulatan grid {tab.reso} cm / {360 / tab.ny:.0f} deg): "
          f"median {np.median(gal):.3f}, p90 {np.percentile(gal, 90):.3f}")
    t0 = time.perf_counter()
    for _ in range(100000):
        tab.h(120.0, -40.0, 1.0, 10.0, 20.0, 0.3)
    print(f"waktu satu pembacaan: {(time.perf_counter() - t0) * 10:.2f} us")
