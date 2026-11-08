"""
cepat.py - kernel numba untuk bagian perencana yang mahal di Python murni.

Aturan: setiap kernel menghasilkan angka yang SAMA PERSIS (sampai bit terakhir)
dengan kode Python yang digantikannya - urutan operasi, tipe (float64 / float32
seperti numpy 2), dan pemecah seri heap sama. Jalur kedua perencana tidak boleh
berubah; diperiksa dengan membandingkan jalur semua rute uji sebelum dan sesudah.

    dijkstra_grid     peta Dijkstra 8-tetangga h_D (HybridAStarPlanner._holonomic_heuristic)
                      - Hybrid A* (heuristik) dan DQfD (masukan, saringan serah)
    rs_periksa        satu kandidat tembakan RS: sampling + cek tabrakan + penalti
                      (HybridAStarPlanner._best_free_rs) - Hybrid A* dan DQfD
    calon_aritmetika  pose ujung, sel, biaya 14 primitif tanpa cek tabrakan
                      (LingkunganRencana.calon_malas) - DQfD, pencarian lazy
    obs_isi           47 masukan DQfD (LingkunganRencana._obs_jauh)

Tanpa numba, ADA_NUMBA = False dan pemanggil memakai versi Python-nya sendiri.
"""
import ctypes
import ctypes.util
import heapq
import math
import os
import random
import sys

import numpy as np

try:
    from numba import njit
    ADA_NUMBA = True
except ImportError:                      # pragma: no cover
    ADA_NUMBA = False


def _fungsi_libm():
    """sin, cos, atan2 dari pustaka C yang dipakai modul math Python, lewat ctypes.

    Kenapa tidak math.sin di dalam numba: LLVM menggabungkan sin(a) dan cos(a) dengan argumen
    sama menjadi satu panggilan sincos (macOS __sincos_stret) yang hasil sin-nya kadang beda
    1 ulp dari math.sin Python - pose jalur lalu berbeda di digit terakhir. Panggilan lewat
    penunjuk fungsi ctypes tidak bisa digabung LLVM, jadi hasilnya sama persis dengan Python."""
    lib = ctypes.CDLL("ucrtbase") if sys.platform == "win32" else ctypes.CDLL(ctypes.util.find_library("m"))
    fungsi = []
    for nama, n in (("sin", 1), ("cos", 1), ("atan2", 2)):
        f = getattr(lib, nama)
        f.restype = ctypes.c_double
        f.argtypes = [ctypes.c_double] * n
        fungsi.append(f)
    return fungsi


# CEPAT_MATI=1: paksa versi Python (mis. untuk membuat jalur acuan sebelum/sesudah).
if os.environ.get("CEPAT_MATI") == "1":
    ADA_NUMBA = False

_sin, _cos, _atan2 = _fungsi_libm()
_acak = random.Random(12345)
for _ in range(2000):
    _a, _b = _acak.uniform(-60.0, 60.0), _acak.uniform(-60.0, 60.0)
    if _sin(_a) != math.sin(_a) or _cos(_a) != math.cos(_a) or _atan2(_a, _b) != math.atan2(_a, _b):
        ADA_NUMBA = False                # libm ctypes bukan libm modul math: pakai Python saja
        break


def _dijkstra_inti(blocked, cx, cy, r, diag):
    """Sama dengan loop HybridAStarPlanner._holonomic_heuristic: heap (c, x, y) dengan pemecah seri
    x lalu y, urutan tetangga sama, c + w dalam float64 (r bilangan bulat -> eksak)."""
    ny, nx = blocked.shape
    cost = np.full((ny, nx), np.inf)
    cost[cy, cx] = 0.0
    dxs = (1, -1, 0, 0, 1, 1, -1, -1)
    dys = (0, 0, 1, -1, 1, -1, 1, -1)
    ws = (r, r, r, r, diag, diag, diag, diag)
    heap = [(0.0, cx, cy)]
    while len(heap) > 0:
        c, x, y = heapq.heappop(heap)
        if c > cost[y, x]:
            continue
        for i in range(8):
            xx = x + dxs[i]
            yy = y + dys[i]
            if 0 <= xx < nx and 0 <= yy < ny and not blocked[yy, xx]:
                nc = c + ws[i]
                if nc < cost[yy, xx]:
                    cost[yy, xx] = nc
                    heapq.heappush(heap, (nc, xx, yy))
    return cost


if ADA_NUMBA:
    _dijkstra_nb = njit(nogil=True)(_dijkstra_inti)
    _dijkstra_nb(np.zeros((3, 3), np.bool_), 1, 1, 1.0, 1.4142135623730951)   # kompilasi saat impor


def dijkstra_grid(blocked, cx, cy, r, diag):
    """Peta biaya (ny, nx) float64, inf = tak terjangkau. blocked: larik bool numpy."""
    return _dijkstra_nb(np.ascontiguousarray(blocked, dtype=np.bool_), int(cx), int(cy), float(r), float(diag))


_TWO_PI = 2.0 * math.pi


def _calon_inti(x, y, yaw, nds, nks, panjang, n_sub, xy_reso, yaw_reso, reverse_cost, steer_cost,
                steer_change_cost, kappa_max, switch_cost, k_now, d_now):
    """Pose ujung, sel (HybridAStarPlanner._key), dan biaya tanpa penalti jarak-bebas untuk tiap
    primitif - salinan aritmetika LingkunganRencana._ekspansi(cek=False) dengan urutan operasi sama."""
    n = nds.shape[0]
    ujung = np.empty((n, 3))
    kunci = np.empty((n, 3), np.int64)
    biaya = np.empty(n)
    for a in range(n):
        nd = nds[a]
        nk = nks[a]
        ds = nd * panjang / n_sub
        px, py, pyaw = x, y, yaw
        for _ in range(n_sub):
            if nk == 0.0:
                px += ds * _cos(pyaw)
                py += ds * _sin(pyaw)
            else:
                nyaw = pyaw + nk * ds
                px += (_sin(nyaw) - _sin(pyaw)) / nk
                py -= (_cos(nyaw) - _cos(pyaw)) / nk
                pyaw = nyaw
        ujung[a, 0] = px
        ujung[a, 1] = py
        ujung[a, 2] = pyaw
        kunci[a, 0] = int(px // xy_reso)
        kunci[a, 1] = int(py // xy_reso)
        kunci[a, 2] = int((pyaw % _TWO_PI) // yaw_reso)
        cost = panjang * (1.0 if nd > 0 else reverse_cost)
        cost += steer_cost * panjang * abs(nk) / kappa_max
        cost += steer_change_cost * abs(nk - k_now) / kappa_max
        if d_now != 0 and nd != d_now:
            cost += switch_cost
        biaya[a] = cost
    return ujung, kunci, biaya


if ADA_NUMBA:
    _calon_nb = njit(nogil=True)(_calon_inti)
    _calon_nb(0.0, 0.0, 0.0, np.ones(2, np.int64), np.zeros(2), 1.0, 4, 1.0, 0.1, 2.0, 0.3, 6.0, 0.04, 30.0,
              0.0, 0)


def calon_aritmetika(x, y, yaw, nds, nks, panjang, n_sub, xy_reso, yaw_reso, reverse_cost, steer_cost,
                     steer_change_cost, kappa_max, switch_cost, k_now, d_now):
    """(ujung (n, 3), kunci (n, 3) int64, biaya (n,)) untuk aksi dengan arah nds dan kelengkungan nks."""
    return _calon_nb(float(x), float(y), float(yaw), nds, nks, float(panjang), int(n_sub), float(xy_reso),
                     float(yaw_reso), float(reverse_cost), float(steer_cost), float(steer_change_cost),
                     float(kappa_max), float(switch_cost), float(k_now), int(d_now))


_TETANGGA_DX = (1, -1, 0, 0, 1, 1, -1, -1)
_TETANGGA_DY = (0, 0, 1, -1, 1, -1, 1, -1)


def _obs_inti(o, x, y, yaw, gx, gy, gyaw, ada_gyaw, maju_muat, mundur_muat, d, k, kappa_max, J, hyp,
              H, h_reso, dist, cx, cy, ux, uy, tepi, batas, jangkau, iterasi, offs, circle_radius,
              rasio_margin, bebas_norm):
    """Isi 47 masukan DQfD ke o (float32) - salinan LingkunganRencana._obs_jauh. cos/sin sudut sinar
    (ux, uy) dan hyp = math.hypot(gx - x, gy - y) dihitung pemanggil dengan numpy/Python, supaya
    pembulatannya sama persis dengan kode aslinya."""
    c, s = _cos(yaw), _sin(yaw)
    dx, dy = gx - x, gy - y
    o[0] = (c * dx + s * dy) / J
    o[1] = (-s * dx + c * dy) / J
    o[2] = hyp / J
    if not ada_gyaw:
        o[3] = 1.0
        o[4] = 0.0
        o[5] = 1.0
    else:
        e = gyaw - yaw
        o[3] = _cos(e)
        o[4] = _sin(e)
        o[5] = 0.0
    o[6] = 1.0 if maju_muat else 0.0
    o[7] = 1.0 if mundur_muat else 0.0
    o[8] = d
    o[9] = k / kappa_max
    # 32 sinar (_sinar_loop + pasca-proses _sinar: clip((min(t, batas) - tepi) / jangkau, 0, 1))
    h, w = dist.shape
    for i in range(ux.shape[0]):
        ti = 0.0
        for _ in range(iterasi):
            ix = int(math.floor(cx + ti * ux[i]))
            iy = int(math.floor(cy + ti * uy[i]))
            if 0 <= ix < w and 0 <= iy < h:
                dd = float(dist[iy, ix])
            else:
                dd = 0.0
            if not (dd >= 1.0 and ti < batas[i]):
                break
            ti = ti + max(dd - 0.5, 0.5)
        v = (min(ti, batas[i]) - tepi[i]) / jangkau
        o[10 + i] = 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)
    # h_D (HybridAStarPlanner._h)
    ny, nx = H.shape
    iy = min(ny - 1, max(0, int(y // h_reso)))
    ix = min(nx - 1, max(0, int(x // h_reso)))
    hv = H[iy, ix]
    o[42] = (hv if hv != math.inf else hyp) / J
    # arah turun h_D (_arah_heuristik)
    terbaik = H[iy, ix]
    adx, ady, ada = 0, 0, False
    for j in range(8):
        xx, yy = ix + _TETANGGA_DX[j], iy + _TETANGGA_DY[j]
        if 0 <= xx < nx and 0 <= yy < ny and H[yy, xx] < terbaik:
            terbaik, adx, ady, ada = H[yy, xx], _TETANGGA_DX[j], _TETANGGA_DY[j], True
    if ada:
        sudut = _atan2(float(ady), float(adx)) - yaw
        o[43] = _cos(sudut)
        o[44] = _sin(sudut)
    elif dx == 0.0 and dy == 0.0:
        o[43] = 0.0
        o[44] = 0.0
    else:
        sudut = _atan2(dy, dx) - yaw
        o[43] = _cos(sudut)
        o[44] = _sin(sudut)
    o[45] = rasio_margin
    # margin badan (ObstacleMap.pose_margin): float32 seperti numpy 2 (float32 - float Python -> float32)
    luar = False
    dmin = np.float32(np.inf)
    for j in range(offs.shape[0]):
        ix2 = int(x + offs[j] * c)
        iy2 = int(y + offs[j] * s)
        if 0 <= ix2 < w and 0 <= iy2 < h:
            if dist[iy2, ix2] < dmin:
                dmin = dist[iy2, ix2]
        else:
            luar = True
    if luar:
        q = (-1.0 - circle_radius) / bebas_norm
        o[46] = max(-1.0, min(1.0, q))
    else:
        q32 = (dmin - np.float32(circle_radius)) / np.float32(bebas_norm)
        o[46] = max(np.float32(-1.0), min(np.float32(1.0), q32))


if ADA_NUMBA:
    _obs_nb = njit(nogil=True)(_obs_inti)
    _obs_nb(np.zeros(47, np.float32), 1.0, 1.0, 0.0, 5.0, 5.0, 0.0, True, True, False, 1, 0.0, 0.04, 300.0, 5.0,
            np.zeros((3, 3)), 5, np.ones((8, 8), np.float32), 2.0, 2.0, np.ones(32), np.zeros(32), np.ones(32),
            np.ones(32) * 2.0, 100.0, 4, np.zeros(4), 10.0, 1.0, 10.0)          # kompilasi saat impor


def obs_isi(o, x, y, yaw, gx, gy, gyaw, ada_gyaw, maju_muat, mundur_muat, d, k, kappa_max, J, hyp, H, h_reso,
            dist, cx, cy, ux, uy, tepi, batas, jangkau, iterasi, offs, circle_radius, rasio_margin, bebas_norm):
    _obs_nb(o, float(x), float(y), float(yaw), float(gx), float(gy), float(gyaw), bool(ada_gyaw), bool(maju_muat),
            bool(mundur_muat), int(d), float(k), float(kappa_max), float(J), float(hyp), H, int(h_reso), dist,
            float(cx), float(cy), ux, uy, tepi, batas, float(jangkau), int(iterasi), offs, float(circle_radius),
            float(rasio_margin), float(bebas_norm))


def _rs_periksa_inti(x, y, yaw, kinds, lengths, radius, ds, dist, offs, circle_radius, clearance_cost,
                     clearance_pref, step, cost0, poses):
    """Satu kandidat tembakan RS dari HybridAStarPlanner._best_free_rs: sample_rs_path lalu
    pose_margin + _clearance_penalty per titik, berhenti di titik pertama yang menabrak.

    Tipe meniru numpy 2: pose_margin float32 (float32 - float Python -> float32), penalti
    float32 bila positif dan 0.0 Python bila tidak, dan biaya berubah menjadi float32 sejak
    penalti positif pertama. Kembalian (bebas, biaya, biaya_float32, jumlah_titik); poses (N, 4)
    diisi x, y, yaw, arah. kinds: 0 = S, 1 = L, 2 = R."""
    k = 1.0 / radius
    h, w = dist.shape
    r32 = np.float32(circle_radius)
    pref32 = np.float32(clearance_pref)
    cc32 = np.float32(clearance_cost)
    ds32 = np.float32(ds)
    step32 = np.float32(step)
    cost = cost0
    is32 = False
    cost32 = np.float32(0.0)
    n_p = 0
    for si in range(kinds.shape[0]):
        kind = kinds[si]
        length = lengths[si]
        arah = 1 if length > 0 else -1
        n = max(1, math.ceil(abs(length) / ds))
        st = length / n
        for _ in range(n):
            if kind == 0:
                x, y = x + st * _cos(yaw), y + st * _sin(yaw)
            else:
                kk = k if kind == 1 else -k
                nyaw = yaw + kk * st
                x, y, yaw = (x + (_sin(nyaw) - _sin(yaw)) / kk,
                             y - (_cos(nyaw) - _cos(yaw)) / kk,
                             nyaw)
            poses[n_p, 0] = x
            poses[n_p, 1] = y
            poses[n_p, 2] = yaw
            poses[n_p, 3] = arah
            n_p += 1
            c, s = _cos(yaw), _sin(yaw)
            luar = False
            dmin = np.float32(np.inf)
            for j in range(offs.shape[0]):
                ix = int(x + offs[j] * c)
                iy = int(y + offs[j] * s)
                if 0 <= ix < w and 0 <= iy < h:
                    if dist[iy, ix] < dmin:
                        dmin = dist[iy, ix]
                else:
                    luar = True
            if luar:
                return False, cost, is32, n_p                 # m = -1 - r <= 0
            m = dmin - r32
            if m <= np.float32(0.0):
                return False, cost, is32, n_p
            sel = pref32 - m
            if sel > np.float32(0.0):
                pen = cc32 * sel * ds32 / step32
                if is32:
                    cost32 = cost32 + pen
                else:
                    cost32 = np.float32(cost) + pen
                    is32 = True
            if is32:
                cost = float(cost32)
    return True, cost, is32, n_p


if ADA_NUMBA:
    _rs_periksa_nb = njit(nogil=True)(_rs_periksa_inti)
    _rs_periksa_nb(1.0, 1.0, 0.0, np.zeros(2, np.int64), np.ones(2), 25.0, 2.4, np.ones((8, 8), np.float32),
                   np.zeros(4), 10.0, 0.8, 6.1, 7.3, 0.0, np.empty((2, 4)))        # kompilasi saat impor

_KODE_SEG = {'S': 0, 'L': 1, 'R': 2}


def rs_periksa(x, y, yaw, segs, radius, ds, dist, offs, circle_radius, clearance_cost, clearance_pref, step,
               cost0):
    """(poses atau None, biaya) untuk satu kandidat RS - poses list (x, y, yaw, arah) seperti
    sample_rs_path; biaya float Python, atau np.float32 bila penalti jarak-bebas pernah positif."""
    kinds = np.array([_KODE_SEG[kd] for kd, _ in segs], np.int64)
    lengths = np.array([ln for _, ln in segs], np.float64)
    n_maks = int(sum(max(1, math.ceil(abs(ln) / ds)) for ln in lengths))
    buf = np.empty((n_maks, 4))
    bebas, cost, is32, n_p = _rs_periksa_nb(float(x), float(y), float(yaw), kinds, lengths, float(radius),
                                            float(ds), dist, offs, float(circle_radius), float(clearance_cost),
                                            float(clearance_pref), float(step), float(cost0), buf)
    if not bebas:
        return None, cost
    cost = np.float32(cost) if is32 else cost
    return [(px, py, pyaw, int(d)) for px, py, pyaw, d in buf[:n_p].tolist()], cost
