"""
astar_lazy.py - Hybrid A* + RS dengan cek tabrakan LAZY (Lazy Weighted A*), pembanding paper ver9.

Sama dengan Hybrid A* standar (heuristik, bobot, primitif, biaya, aturan RS, batas waktu),
kecuali kapan primitif dicek:

  standar  setiap ekspansi mensimulasi DAN mengecek ke-14 primitif.
  lazy     ekspansi hanya menghitung pose ujung dan biaya ke-14 primitif dengan aritmetika
           gerak (kernel cepat.calon_aritmetika, kernel yang sama dengan DQfD), tanpa cek
           tabrakan dan tanpa penalti jarak-bebas. Primitif baru disimulasi dan dicek saat
           diambil dari open list: menabrak atau selnya sudah ditutup -> dibuang; biaya tepat
           menaikkan f di atas puncak open list -> dimasukkan ulang dengan f tepat.

Ini persis cara pencarian DQfD (perencana_dqfd.rencanakan_terbaik_malas), dengan heuristik
Hybrid A* h = max(h_RS, h_D) sebagai ganti Q. Jadi selisih DQfD terhadap perencana ini adalah
sumbangan model Q, bukan sumbangan "lazy".

Pemakaian: jadikan_lazy(planner) mengubah satu HybridAStarBertanda di tempat.
HIT["n"] = jumlah primitif yang dicek tabrakannya (dinolkan pemanggil).
"""
import heapq
import itertools
import math
import time

import numpy as np

import cepat as _cepat

HIT = {"n": 0}
_EST, _TEPAT = 0, 1


def _plan_lazy(self, omap, start, goal):
    t0 = time.time()
    sx, sy, syaw = start
    gx, gy, gyaw = goal
    self.last_stats = {"expansions": 0, "time": 0.0, "reason": ""}

    if gyaw is not None and not omap.pose_free(self.v, gx, gy, gyaw):
        self.last_stats["reason"] = "tujuan menabrak rintangan"
        return None
    if gyaw is None and omap.clearance(gx, gy) < self.v.half_width:
        self.last_stats["reason"] = "tujuan terlalu dekat rintangan"
        return None
    if self._at_goal(sx, sy, syaw, goal):
        self.last_stats["reason"] = "sudah di tujuan"
        return []

    self._hcost = self._holonomic_heuristic(omap, gx, gy)
    self._approach_dirs = self._feasible_approach_dirs(omap, goal)
    start_in_col = omap.pose_margin(self.v, sx, sy, syaw) <= 0

    # Urutan aksi sama dengan loop ekspansi standar: for nd in dirs: for nk in kappas.
    aksi = [(nd, nk) for nd in self.dirs for nk in self.kappas]
    nds = np.array([a[0] for a in aksi], dtype=np.int64)
    nks = np.array([a[1] for a in aksi], dtype=np.float64)

    # node = (x, y, yaw, arah, kappa, g, parent, poses, masih_menempel)
    nodes = [(sx, sy, syaw, 0, 0.0, 0.0, -1, [], start_in_col)]
    closed = {self._key(sx, sy, syaw)}
    heap, urut = [], itertools.count()
    w = self.h_weight
    state = {"expansions": 0}

    def buka(idx):
        """Ekspansi node idx. Kembalian: (idx, ekor RS) bila pencarian selesai, else None."""
        node = nodes[idx]
        x, y, yaw, d, k, g = node[:6]
        state["expansions"] += 1
        if idx != 0 and self._at_goal(x, y, yaw, goal):
            return (idx, [])
        if not node[8] and (math.hypot(gx - x, gy - y) < self.analytic_dist
                            or state["expansions"] % self.analytic_every == 0):
            rs = self._analytic(omap, node, goal)
            if rs is not None:
                return (idx, rs[0])
        ujung, kunci, biaya = _cepat.calon_aritmetika(
            x, y, yaw, nds, nks, self.step, self.n_sub, self.xy_reso, self.yaw_reso,
            self.reverse_cost, self.steer_cost, self.steer_change_cost, self.kappa_max, self.switch_cost, k, d)
        ul, kl, bl = ujung.tolist(), kunci.tolist(), biaya.tolist()
        for i in range(len(aksi)):
            if (kl[i][0], kl[i][1], kl[i][2]) in closed:
                continue
            f = g + bl[i] + w * self._h_cari(ul[i][0], ul[i][1], ul[i][2], goal)
            heapq.heappush(heap, (f, next(urut), idx, i, _EST))
        return None

    result = buka(0)
    while heap and result is None:
        if state["expansions"] >= self.max_expansions or time.time() - t0 > self.time_limit:
            self.last_stats["reason"] = "batas waktu/iterasi habis"
            break
        f, _, idx, i, jenis = heapq.heappop(heap)
        x, y, yaw, d, k, g = nodes[idx][:6]
        in_col = nodes[idx][8]
        nd, nk = aksi[i]
        # Simulasi + cek tabrakan primitif ini: salinan loop ekspansi HybridAStarPlanner.plan().
        HIT["n"] += 1
        ds = nd * self.step / self.n_sub
        m0 = omap.pose_margin(self.v, x, y, yaw) if in_col else 0.0
        px, py, pyaw = x, y, yaw
        poses = []
        ok = True
        col, last_m, min_m = in_col, m0, float('inf')
        for _ in range(self.n_sub):
            if nk == 0.0:
                px += ds * math.cos(pyaw)
                py += ds * math.sin(pyaw)
            else:
                nyaw = pyaw + nk * ds
                px += (math.sin(nyaw) - math.sin(pyaw)) / nk
                py -= (math.cos(nyaw) - math.cos(pyaw)) / nk
                pyaw = nyaw
            m = omap.pose_margin(self.v, px, py, pyaw)
            if m > 0:
                col = False
                min_m = min(min_m, m)
            elif not (col and m >= last_m - 0.5):
                ok = False
                break
            last_m = m
            poses.append((px, py, pyaw, nd))
        if not ok:
            continue
        nkey = self._key(px, py, pyaw)
        if nkey in closed:
            continue
        cost = self.step * (1.0 if nd > 0 else self.reverse_cost)
        cost += self.steer_cost * self.step * abs(nk) / self.kappa_max
        cost += self.steer_change_cost * abs(nk - k) / self.kappa_max
        if d != 0 and nd != d:
            cost += self.switch_cost
        if min_m != float('inf'):
            cost += self._clearance_penalty(min_m, self.step)
        ng = g + cost
        if jenis == _EST:
            fe = ng + w * self._h_cari(px, py, pyaw, goal)
            if heap and fe > f and fe > heap[0][0]:
                heapq.heappush(heap, (fe, next(urut), idx, i, _TEPAT))
                continue
        closed.add(nkey)
        nodes.append((px, py, pyaw, nd, nk, ng, idx, poses, col))
        result = buka(len(nodes) - 1)

    self.last_stats["expansions"] = state["expansions"]
    self.last_stats["time"] = time.time() - t0
    if result is None:
        if not self.last_stats["reason"]:
            self.last_stats["reason"] = "tidak ada jalur"
        return None
    idx, tail = result
    chain = []
    while idx > 0:
        chain.append(nodes[idx][7])
        idx = nodes[idx][6]
    path = [p for poses in reversed(chain) for p in poses] + list(tail)
    if not path:
        return []
    return [(sx, sy, syaw, path[0][3])] + path


def jadikan_lazy(planner):
    """Ubah satu HybridAStarBertanda (rencana.buat_perencana) menjadi versi lazy, di tempat.
    plan() meniru HybridAStarBertanda.plan() (pencatat waktu RS), dengan _plan_lazy sebagai inti."""
    asal = type(planner)

    def plan(self, omap, start, goal):
        self._rs_titik = 0
        self._t_rs = 0.0
        t0 = time.perf_counter()
        jalur = _plan_lazy(self, omap, start, goal)
        total = time.perf_counter() - t0
        self.last_stats["rs_titik"] = self._rs_titik if jalur else 0
        self.last_stats["rincian_s"] = {"astar": total - self._t_rs, "rs": self._t_rs}
        return jalur

    planner.__class__ = type("HybridAStarLazy", (asal,), {"plan": plan})
    planner.peran = "astar-lazy"
    return planner
