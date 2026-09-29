"""
Perencana jalur global + pelacak jalur untuk forklift KEMUDI RODA BELAKANG.

Model kinematik (koordinat piksel arena, sumbu Y ke bawah):
    Titik acuan = tengah poros roda DEPAN (poros tetap, tidak ikut berbelok).
    x'   = v * cos(yaw)
    y'   = v * sin(yaw)
    yaw' = v * kappa,   kappa = tan(delta) / L

    delta = sudut kemudi LOGIS (sama dengan yang dikirim ke firmware):
    positif = kendaraan belok ke kanan layar saat maju. Pembalikan arah servo
    karena kemudinya di roda belakang sudah ditangani firmware (steerInvert).

Kenapa titik acuan di poros depan: pada kemudi roda belakang hanya titik di
poros TETAP yang geraknya selalu searah heading, jadi model di atas eksak di
titik itu. Poros belakang justru mengayun keluar saat belok (tail swing); efek
ini tertangkap karena tabrakan dicek dengan footprint badan penuh yang
sebagian besar berada DI BELAKANG titik acuan.

Isi modul:
    VehicleModel        geometri & footprint kendaraan
    ObstacleMap         peta jarak ke rintangan (distance transform)
    reeds_shepp_paths   semua jalur Reeds-Shepp (maju+mundur, radius belok minimum)
    HybridAStarPlanner  Hybrid A* + ekspansi analitik Reeds-Shepp
    PathTracker         pure pursuit per segmen maju/mundur, dengan jeda di titik ganti arah
"""
import heapq
import math
import time

import cv2
import numpy as np

TWO_PI = 2.0 * math.pi


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


# ==========================================================================
# KENDARAAN
# ==========================================================================
class VehicleModel:
    def __init__(self, wheelbase, max_steer, body_front, body_rear, half_width, safety_margin=5.0):
        """
        wheelbase   : jarak poros depan (tetap) ke poros belakang (kemudi), px
        max_steer   : sudut kemudi maksimum, radian
        body_front  : jarak poros depan ke ujung depan badan (termasuk garpu), px
        body_rear   : jarak poros depan ke ujung belakang badan, px
        half_width  : setengah lebar badan, px
        """
        self.L = wheelbase
        self.max_steer = max_steer
        self.kappa_max = math.tan(max_steer) / wheelbase
        self.body_front = body_front
        self.body_rear = body_rear
        self.half_width = half_width

        # Footprint persegi panjang didekati beberapa lingkaran di sumbu badan
        length = body_front + body_rear
        n = max(2, math.ceil(length / half_width))
        seg = length / n
        self.circle_offsets = [body_front - (i + 0.5) * seg for i in range(n)]
        self.circle_radius = math.hypot(seg / 2.0, half_width) + safety_margin

    def footprint(self, x, y, yaw, inflate=0.0):
        """Titik sudut badan (np.int32, 4x2) untuk digambar / dikosongkan dari peta."""
        c, s = math.cos(yaw), math.sin(yaw)
        f, r, w = self.body_front + inflate, self.body_rear + inflate, self.half_width + inflate
        pts = []
        for lon, lat in ((f, -w), (f, w), (-r, w), (-r, -w)):
            pts.append((x + lon * c - lat * s, y + lon * s + lat * c))
        return np.array(pts, dtype=np.int32)


# ==========================================================================
# PETA RINTANGAN
# ==========================================================================
class ObstacleMap:
    def __init__(self, occupancy):
        """occupancy: uint8 HxW, nilai > 0 = rintangan. Tepi arena otomatis dianggap dinding."""
        free = np.where(occupancy > 0, 0, 255).astype(np.uint8)
        free[0, :] = 0
        free[-1, :] = 0
        free[:, 0] = 0
        free[:, -1] = 0
        self.dist = cv2.distanceTransform(free, cv2.DIST_L2, 5)
        self.h, self.w = free.shape

    def clearance(self, x, y):
        ix, iy = int(x), int(y)
        if 0 <= ix < self.w and 0 <= iy < self.h:
            return self.dist[iy, ix]
        return -1.0

    def pose_margin(self, vehicle, x, y, yaw):
        """Jarak bebas terkecil antara badan (termasuk safety margin) dan rintangan. <= 0 berarti tabrakan."""
        c, s = math.cos(yaw), math.sin(yaw)
        r = vehicle.circle_radius
        return min(self.clearance(x + off * c, y + off * s) for off in vehicle.circle_offsets) - r

    def pose_free(self, vehicle, x, y, yaw):
        return self.pose_margin(vehicle, x, y, yaw) > 0


# ==========================================================================
# REEDS-SHEPP (radius belok 1, mengikuti implementasi OMPL / paper Reeds-Shepp)
# Segmen: ('L'|'R'|'S', panjang bertanda). L = yaw naik, panjang negatif = mundur.
# ==========================================================================
_RS_EPS = 1e-9


def _mod2pi(x):
    v = math.fmod(x, TWO_PI)
    if v < -math.pi:
        v += TWO_PI
    elif v > math.pi:
        v -= TWO_PI
    return v


def _polar(x, y):
    return math.hypot(x, y), math.atan2(y, x)


def _tau_omega(u, v, xi, eta, phi):
    delta = _mod2pi(u - v)
    a = math.sin(u) - math.sin(delta)
    b = math.cos(u) - math.cos(delta) - 1.0
    t1 = math.atan2(eta * a - xi * b, xi * a + eta * b)
    t2 = 2.0 * (math.cos(delta) - math.cos(v) - math.cos(u)) + 3.0
    tau = _mod2pi(t1 + math.pi) if t2 < 0 else _mod2pi(t1)
    omega = _mod2pi(tau - u + v - phi)
    return tau, omega


def _LpSpLp(x, y, phi):
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if t >= -_RS_EPS:
        v = _mod2pi(phi - t)
        if v >= -_RS_EPS:
            return t, u, v
    return None


def _LpSpRp(x, y, phi):
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1 = u1 * u1
    if u1 >= 4.0:
        u = math.sqrt(u1 - 4.0)
        t = _mod2pi(t1 + math.atan2(2.0, u))
        v = _mod2pi(t - phi)
        if t >= -_RS_EPS and v >= -_RS_EPS:
            return t, u, v
    return None


def _LpRmL(x, y, phi):
    u1, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u1 <= 4.0:
        u = -2.0 * math.asin(0.25 * u1)
        t = _mod2pi(theta + 0.5 * u + math.pi)
        v = _mod2pi(phi - t + u)
        if t >= -_RS_EPS and u <= _RS_EPS:
            return t, u, v
    return None


def _LpRupLumRm(x, y, phi):
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho = 0.25 * (2.0 + math.hypot(xi, eta))
    if rho <= 1.0:
        u = math.acos(rho)
        t, v = _tau_omega(u, -u, xi, eta, phi)
        if t >= -_RS_EPS and v <= _RS_EPS:
            return t, u, v
    return None


def _LpRumLumRp(x, y, phi):
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho = (20.0 - xi * xi - eta * eta) / 16.0
    if 0.0 <= rho <= 1.0:
        u = -math.acos(rho)
        if u >= -0.5 * math.pi:
            t, v = _tau_omega(u, u, xi, eta, phi)
            if t >= -_RS_EPS and v >= -_RS_EPS:
                return t, u, v
    return None


def _LpRmSmLm(x, y, phi):
    rho, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if rho >= 2.0:
        r = math.sqrt(rho * rho - 4.0)
        u = 2.0 - r
        t = _mod2pi(theta + math.atan2(r, -2.0))
        v = _mod2pi(phi - 0.5 * math.pi - t)
        if t >= -_RS_EPS and u <= _RS_EPS and v <= _RS_EPS:
            return t, u, v
    return None


def _LpRmSmRm(x, y, phi):
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho, theta = _polar(-eta, xi)
    if rho >= 2.0:
        t = theta
        u = 2.0 - rho
        v = _mod2pi(t + 0.5 * math.pi - phi)
        if t >= -_RS_EPS and u <= _RS_EPS and v <= _RS_EPS:
            return t, u, v
    return None


def _LpRmSLmRp(x, y, phi):
    xi, eta = x + math.sin(phi), y - 1.0 - math.cos(phi)
    rho, _ = _polar(xi, eta)
    if rho >= 2.0:
        u = 4.0 - math.sqrt(rho * rho - 4.0)
        if u <= _RS_EPS:
            t = _mod2pi(math.atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta))
            v = _mod2pi(t - phi)
            if t >= -_RS_EPS and v >= -_RS_EPS:
                return t, u, v
    return None


def _swap_lr(word):
    return tuple({'L': 'R', 'R': 'L'}.get(c, c) for c in word)


def _rs_words(x, y, phi):
    """Semua kandidat (kata, panjang) untuk tujuan ternormalisasi (x, y, phi)."""
    out = []
    hp = 0.5 * math.pi

    def four(fn, word, build, xx=x, yy=y):
        # Varian asli, time-flip, reflect, dan keduanya
        for sx, sy, sphi, sign, refl in ((1, 1, 1, 1, False), (-1, 1, -1, -1, False),
                                         (1, -1, -1, 1, True), (-1, -1, 1, -1, True)):
            r = fn(sx * xx, sy * yy, sphi * phi)
            if r is not None:
                lengths = [sign * l for l in build(*r)]
                out.append((_swap_lr(word) if refl else word, lengths))

    xb = x * math.cos(phi) + y * math.sin(phi)
    yb = x * math.sin(phi) - y * math.cos(phi)

    # CSC
    four(_LpSpLp, ('L', 'S', 'L'), lambda t, u, v: (t, u, v))
    four(_LpSpRp, ('L', 'S', 'R'), lambda t, u, v: (t, u, v))
    # CCC (+ versi mundur)
    four(_LpRmL, ('L', 'R', 'L'), lambda t, u, v: (t, u, v))
    four(_LpRmL, ('L', 'R', 'L'), lambda t, u, v: (v, u, t), xb, yb)
    # CCCC
    four(_LpRupLumRm, ('L', 'R', 'L', 'R'), lambda t, u, v: (t, u, -u, v))
    four(_LpRumLumRp, ('L', 'R', 'L', 'R'), lambda t, u, v: (t, u, u, v))
    # CCSC (+ versi mundur)
    four(_LpRmSmLm, ('L', 'R', 'S', 'L'), lambda t, u, v: (t, -hp, u, v))
    four(_LpRmSmRm, ('L', 'R', 'S', 'R'), lambda t, u, v: (t, -hp, u, v))
    four(_LpRmSmLm, ('L', 'S', 'R', 'L'), lambda t, u, v: (v, u, -hp, t), xb, yb)
    four(_LpRmSmRm, ('R', 'S', 'R', 'L'), lambda t, u, v: (v, u, -hp, t), xb, yb)
    # CCSCC
    four(_LpRmSLmRp, ('L', 'R', 'S', 'L', 'R'), lambda t, u, v: (t, -hp, u, -hp, v))
    return out


def _integrate_segment(x, y, yaw, kind, length, curvature):
    """Posisi setelah menempuh segmen (panjang bertanda) dengan kelengkungan tertentu."""
    if kind == 'S':
        return x + length * math.cos(yaw), y + length * math.sin(yaw), yaw
    k = curvature if kind == 'L' else -curvature
    nyaw = yaw + k * length
    return (x + (math.sin(nyaw) - math.sin(yaw)) / k,
            y - (math.cos(nyaw) - math.cos(yaw)) / k,
            nyaw)


def reeds_shepp_paths(start, goal, radius):
    """Semua jalur Reeds-Shepp yang valid dari start ke goal (x, y, yaw) dengan radius belok tertentu.
    Mengembalikan list [(segmen, panjang_total)], segmen = [(kind, panjang_bertanda_px)].
    Setiap kandidat diverifikasi dengan integrasi numerik, jadi yang salah pasti terbuang."""
    sx, sy, syaw = start
    gx, gy, gyaw = goal
    dx, dy = gx - sx, gy - sy
    c, s = math.cos(syaw), math.sin(syaw)
    x = (c * dx + s * dy) / radius
    y = (-s * dx + c * dy) / radius
    phi = wrap_angle(gyaw - syaw)

    results = []
    for word, lengths in _rs_words(x, y, phi):
        segs = [(k, l) for k, l in zip(word, lengths) if abs(l) > 1e-9]
        px, py, pyaw = 0.0, 0.0, 0.0
        for k, l in segs:
            px, py, pyaw = _integrate_segment(px, py, pyaw, k, l, 1.0)
        if abs(px - x) > 1e-4 or abs(py - y) > 1e-4 or abs(wrap_angle(pyaw - phi)) > 1e-4:
            continue
        scaled = [(k, l * radius) for k, l in segs]
        results.append((scaled, sum(abs(l) for _, l in scaled)))
    return results


def sample_rs_path(start, segments, radius, ds):
    """Titik-titik (x, y, yaw, arah) sepanjang jalur Reeds-Shepp, tidak termasuk titik start."""
    x, y, yaw = start
    k = 1.0 / radius
    poses = []
    for kind, length in segments:
        d = 1 if length > 0 else -1
        n = max(1, math.ceil(abs(length) / ds))
        step = length / n
        for _ in range(n):
            x, y, yaw = _integrate_segment(x, y, yaw, kind, step, k)
            poses.append((x, y, yaw, d))
    return poses


# ==========================================================================
# HYBRID A*
# ==========================================================================
class HybridAStarPlanner:
    def __init__(self, vehicle, plan_steer_ratio=0.85, allow_reverse=True,
                 xy_reso=12.0, yaw_reso=math.radians(10.0), step=18.0, n_steer=7, sample_ds=6.0,
                 reverse_cost=2.0, switch_cost=80.0, steer_cost=0.3, steer_change_cost=6.0,
                 clearance_pref=15.0, clearance_cost=0.8,
                 h_weight=1.3, h_reso=12, approach_len=80.0,
                 goal_pos_tol=8.0, goal_yaw_tol=math.radians(12.0),
                 analytic_every=4, analytic_dist=250.0,
                 time_limit=2.0, max_expansions=30000):
        self.v = vehicle
        # Perencana memakai sebagian sudut kemudi saja; sisanya cadangan untuk koreksi pelacak
        self.kappa_max = math.tan(vehicle.max_steer * plan_steer_ratio) / vehicle.L
        self.radius = 1.0 / self.kappa_max
        self.dirs = (1, -1) if allow_reverse else (1,)
        self.kappas = [self.kappa_max * (2.0 * i / (n_steer - 1) - 1.0) for i in range(n_steer)]
        self.xy_reso, self.yaw_reso = xy_reso, yaw_reso
        self.step, self.sample_ds = step, sample_ds
        self.n_sub = max(1, math.ceil(step / sample_ds))
        self.reverse_cost, self.switch_cost = reverse_cost, switch_cost
        self.steer_cost, self.steer_change_cost = steer_cost, steer_change_cost
        # Biaya tambahan bila badan lebih dekat dari clearance_pref px ke rintangan (pilih lewat tengah celah)
        self.clearance_pref, self.clearance_cost = clearance_pref, clearance_cost
        self.h_weight, self.h_reso = h_weight, h_reso
        # Panjang bagian lurus terakhir saat tujuan punya arah hadap, supaya pelacak sempat meluruskan heading
        self.approach_len = approach_len
        self.goal_pos_tol, self.goal_yaw_tol = goal_pos_tol, goal_yaw_tol
        self.analytic_every, self.analytic_dist = analytic_every, analytic_dist
        self.time_limit, self.max_expansions = time_limit, max_expansions
        self.last_stats = {}

    # ---- heuristik 2D sadar-rintangan (Dijkstra dari tujuan) ----
    def _holonomic_heuristic(self, omap, gx, gy):
        r = self.h_reso
        centers = omap.dist[r // 2::r, r // 2::r]
        blocked = (centers < self.v.half_width).tolist()
        ny, nx = centers.shape
        inf = float('inf')
        cost = [[inf] * nx for _ in range(ny)]
        cx, cy = min(nx - 1, max(0, int(gx // r))), min(ny - 1, max(0, int(gy // r)))
        cost[cy][cx] = 0.0
        heap = [(0.0, cx, cy)]
        diag = r * math.sqrt(2.0)
        nbrs = [(1, 0, r), (-1, 0, r), (0, 1, r), (0, -1, r),
                (1, 1, diag), (1, -1, diag), (-1, 1, diag), (-1, -1, diag)]
        while heap:
            c, x, y = heapq.heappop(heap)
            if c > cost[y][x]:
                continue
            for ddx, ddy, w in nbrs:
                xx, yy = x + ddx, y + ddy
                if 0 <= xx < nx and 0 <= yy < ny and not blocked[yy][xx]:
                    nc = c + w
                    if nc < cost[yy][xx]:
                        cost[yy][xx] = nc
                        heapq.heappush(heap, (nc, xx, yy))
        return cost

    def _h(self, x, y, gx, gy):
        r = self.h_reso
        row = self._hcost[min(len(self._hcost) - 1, max(0, int(y // r)))]
        v = row[min(len(row) - 1, max(0, int(x // r)))]
        return v if v != float('inf') else math.hypot(gx - x, gy - y)

    def _key(self, x, y, yaw):
        return (int(x // self.xy_reso), int(y // self.xy_reso), int((yaw % TWO_PI) // self.yaw_reso))

    def _at_goal(self, x, y, yaw, goal):
        gx, gy, gyaw = goal
        if math.hypot(gx - x, gy - y) > self.goal_pos_tol:
            return False
        return gyaw is None or abs(wrap_angle(yaw - gyaw)) <= self.goal_yaw_tol

    # ---- ekspansi analitik: coba sambung langsung ke tujuan dengan Reeds-Shepp ----
    def _rs_cost(self, segments, prev_d, prev_k):
        cost, d_prev, k_prev = 0.0, prev_d, prev_k
        for kind, length in segments:
            d = 1 if length > 0 else -1
            k = 0.0 if kind == 'S' else (self.kappa_max if kind == 'L' else -self.kappa_max)
            cost += abs(length) * (1.0 if d > 0 else self.reverse_cost)
            cost += self.steer_cost * abs(length) * abs(k) / self.kappa_max
            cost += self.steer_change_cost * abs(k - k_prev) / self.kappa_max
            if d_prev != 0 and d != d_prev:
                cost += self.switch_cost
            d_prev, k_prev = d, k
        return cost

    def _clearance_penalty(self, margin, length):
        return self.clearance_cost * max(0.0, self.clearance_pref - margin) * length / self.step

    def _analytic(self, omap, node, goal):
        x, y, yaw, d, k = node[0], node[1], node[2], node[3], node[4]
        gx, gy, gyaw = goal
        # Target = (pose sambungan Reeds-Shepp, segmen lurus penutup atau None)
        if gyaw is None:
            bearing = math.atan2(gy - y, gx - x)
            targets = [((gx, gy, bearing), None), ((gx, gy, bearing + math.pi), None)]
        elif self._approach_dirs:
            # Sambung ke titik sebelum tujuan, lalu masuk lurus: maju (ad=+1) atau mundur (ad=-1)
            c, s, la = math.cos(gyaw), math.sin(gyaw), self.approach_len
            targets = [((gx - ad * la * c, gy - ad * la * s, gyaw), ('S', ad * la)) for ad in self._approach_dirs]
        else:
            targets = [((gx, gy, gyaw), None)]   # pendekatan lurus tidak muat: sambung langsung

        return self._best_free_rs(omap, x, y, yaw, d, k, targets)

    def _feasible_approach_dirs(self, omap, goal):
        """Arah pendekatan lurus (maju/mundur) yang badannya bebas rintangan sepanjang approach_len."""
        gx, gy, gyaw = goal
        if gyaw is None or self.approach_len <= 0:
            return []
        c, s = math.cos(gyaw), math.sin(gyaw)
        n = max(1, math.ceil(self.approach_len / self.sample_ds))
        dirs = []
        for ad in self.dirs:
            if all(omap.pose_free(self.v, gx - ad * self.approach_len * c * i / n,
                                  gy - ad * self.approach_len * s * i / n, gyaw) for i in range(n + 1)):
                dirs.append(ad)
        return dirs

    def _best_free_rs(self, omap, x, y, yaw, d, k, targets):
        cands = []
        for pose, tail in targets:
            for segs, _ in reeds_shepp_paths((x, y, yaw), pose, self.radius):
                if tail is not None:
                    segs = segs + [tail]
                cands.append((self._rs_cost(segs, d, k), segs))
        cands.sort(key=lambda c: c[0])

        best = None
        for cost, segs in cands[:6]:
            if best is not None and cost >= best[1]:
                break
            poses = sample_rs_path((x, y, yaw), segs, self.radius, self.sample_ds)
            for p in poses:
                m = omap.pose_margin(self.v, p[0], p[1], p[2])
                if m <= 0:
                    break
                cost += self._clearance_penalty(m, self.sample_ds)
            else:
                if best is None or cost < best[1]:
                    best = (poses, cost)
        return best

    # ---- pencarian utama ----
    def plan(self, omap, start, goal):
        """start = (x, y, yaw); goal = (x, y, yaw atau None = heading bebas).
        Mengembalikan list pose (x, y, yaw, arah) dari start ke goal, atau None bila gagal."""
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

        # Jika posisi awal sudah menempel rintangan (mis. mepet tepi arena), izinkan gerakan
        # "melepaskan diri" selama badan tidak makin mepet, sampai pose bebas lagi.
        start_in_col = omap.pose_margin(self.v, sx, sy, syaw) <= 0

        # node = (x, y, yaw, arah, kappa, g, parent, poses, masih_menempel)
        nodes = [(sx, sy, syaw, 0, 0.0, 0.0, -1, [], start_in_col)]
        heap = [(self.h_weight * self._h(sx, sy, gx, gy), 0)]
        best_g = {self._key(sx, sy, syaw): 0.0}
        closed = set()
        expansions = 0
        result = None

        while heap:
            if expansions >= self.max_expansions or time.time() - t0 > self.time_limit:
                self.last_stats["reason"] = "batas waktu/iterasi habis"
                break
            _, idx = heapq.heappop(heap)
            node = nodes[idx]
            x, y, yaw, d, k, g = node[:6]
            key = self._key(x, y, yaw)
            if key in closed:
                continue
            closed.add(key)
            expansions += 1

            if idx != 0 and self._at_goal(x, y, yaw, goal):
                result = (idx, [])
                break
            in_col = node[8]
            if not in_col and (math.hypot(gx - x, gy - y) < self.analytic_dist
                               or expansions % self.analytic_every == 0):
                rs = self._analytic(omap, node, goal)
                if rs is not None:
                    result = (idx, rs[0])
                    break

            m0 = omap.pose_margin(self.v, x, y, yaw) if in_col else 0.0
            for nd in self.dirs:
                ds = nd * self.step / self.n_sub
                for nk in self.kappas:
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
                    if ng >= best_g.get(nkey, float('inf')):
                        continue
                    best_g[nkey] = ng
                    nodes.append((px, py, pyaw, nd, nk, ng, idx, poses, col))
                    heapq.heappush(heap, (ng + self.h_weight * self._h(px, py, gx, gy), len(nodes) - 1))

        self.last_stats["expansions"] = expansions
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


# ==========================================================================
# PELACAK JALUR (PURE PURSUIT MAJU/MUNDUR)
# ==========================================================================
def split_segments(path):
    """Pecah jalur menjadi segmen dengan arah (maju/mundur) yang sama. Titik ganti arah ikut di kedua segmen."""
    segments = []
    cur, cur_d = [], None
    for p in path:
        d = p[3]
        if cur_d is not None and d != cur_d:
            segments.append((cur, cur_d))
            cur = [cur[-1]]
        cur.append(p)
        cur_d = d
    if len(cur) > 1:
        segments.append((cur, cur_d))
    return segments


def _circle_hit(x1, y1, x2, y2, px, py, r):
    """Perpotongan terjauh (t di [0,1]) antara ruas garis dan lingkaran, atau None."""
    dx, dy = x2 - x1, y2 - y1
    fx, fy = x1 - px, y1 - py
    a = dx * dx + dy * dy
    if a == 0:
        return None
    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    t = (-b + math.sqrt(disc)) / (2 * a)
    if 0.0 <= t <= 1.0:
        return x1 + t * dx, y1 + t * dy
    return None


class PathTracker:
    def __init__(self, path, vehicle, omap=None, lookahead=60.0, seg_end_tol=10.0,
                 forward_speed=6000, reverse_speed=4000, slowdown_dist=80.0, min_speed_ratio=0.35,
                 cusp_pause=0.6, servo_center=89, servo_min=30, servo_max=150, servo_per_deg=1.0,
                 lag_comp=0.0, curve_speed_ratio=1.0, curve_k_ref=None,
                 final_turn_ratio=1.0, final_turn_dist=180.0):
        self.v = vehicle
        self.segments = split_segments(path)
        # Pose yang bebas saat rencana dibuat; hanya pose ini yang dipantau untuk deteksi jalur terhalang
        self.watch = [[omap is None or omap.pose_free(vehicle, p[0], p[1], p[2]) for p in pts]
                      for pts, _ in self.segments]
        self.cum = []
        for pts, _ in self.segments:
            acc = [0.0]
            for a, b in zip(pts[:-1], pts[1:]):
                acc.append(acc[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
            self.cum.append(acc)
        self.lookahead, self.seg_end_tol = lookahead, seg_end_tol
        self.forward_speed, self.reverse_speed = forward_speed, reverse_speed
        self.slowdown_dist, self.min_speed_ratio = slowdown_dist, min_speed_ratio
        self.cusp_pause = cusp_pause
        self.servo_center, self.servo_min, self.servo_max = servo_center, servo_min, servo_max
        self.servo_per_deg = servo_per_deg
        self.seg_i, self.idx = 0, 0
        self.pause_until = 0.0
        self.deviation = 0.0
        self.done = len(self.segments) == 0
        # Kompensasi jeda kemudi: kemudi baru sampai di sudut yang diminta
        # beberapa ratus milidetik setelah perintah dikirim, jadi sudut dihitung
        # untuk pose yang diproyeksikan maju sejauh itu, bukan pose sekarang.
        self.lag_comp = lag_comp
        self.prev = None        # (x, y, yaw, t) frame sebelumnya
        self.vel = (0.0, 0.0, 0.0)   # (vx, vy, yaw_rate) hasil pelicinan

        # --- Kecepatan mengikuti kelengkungan ---------------------------------
        # Kelengkungan diambil dari jalur RENCANA, bukan dari sudut kemudi yang
        # sedang dipakai: memakai sudut kemudi berarti koreksi kecil sesaat ikut
        # memperlambat robot, dan perlambatan itu mengubah koreksi berikutnya
        # (umpan balik yang membuat kecepatan berdenyut). Kelengkungan rencana
        # sudah diketahui jauh sebelum robot sampai, jadi robot bisa mengerem
        # SEBELUM masuk tikungan, bukan setelah telanjur melebar.
        self.curve_speed_ratio = curve_speed_ratio
        # Kelengkungan acuan: di sinilah curve_speed_ratio dipakai penuh, di
        # bawahnya diinterpolasi linear. Bawaannya batas fisik kendaraan, tapi
        # pemanggil sebaiknya mengisi kelengkungan tertajam yang benar-benar
        # DIRENCANAKAN (max_steer * plan_steer_ratio). Kalau tidak, tikungan
        # tertajam yang mungkin muncul di jalur pun masih jauh di bawah acuan,
        # sehingga ratio tidak pernah terpakai penuh.
        self.k_ref = curve_k_ref if curve_k_ref else math.tan(vehicle.max_steer) / vehicle.L

        # Belokan TERAKHIR sebelum tujuan diperlakukan khusus: di situ galat
        # tidak sempat dikoreksi lagi karena jalurnya habis, dan sisa simpangan
        # langsung jadi galat posisi akhir. final_turn_ratio menggantikan
        # curve_speed_ratio (dipilih yang lebih pelan) begitu sisa jalur pada
        # segmen TERAKHIR tinggal final_turn_dist px.
        self.final_turn_ratio = final_turn_ratio
        self.final_turn_dist = final_turn_dist
        # Sekali melambat di zona akhir, kecepatan TIDAK BOLEH naik lagi. Tanpa
        # kancing ini robot melambat di belokan terakhir lalu ngebut lagi di
        # ekor lurus menjelang tujuan (kelengkungannya nol), baru mengerem
        # mendadak - persis kebalikan dari yang diinginkan.
        self._final_hold = None
        self.curv = []
        for pts, _ in self.segments:
            k = [0.0] * len(pts)
            for j in range(1, len(pts) - 1):
                dyaw = wrap_angle(pts[j + 1][2] - pts[j - 1][2])
                ds = (math.hypot(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1]) +
                      math.hypot(pts[j][0] - pts[j - 1][0], pts[j][1] - pts[j - 1][1]))
                k[j] = abs(dyaw) / ds if ds > 1e-6 else 0.0
            if len(k) > 2:
                k[0], k[-1] = k[1], k[-2]
            self.curv.append(k)

    def _curve_ratio(self):
        """Faktor kecepatan 1.0 (lurus) .. curve_speed_ratio (tikungan tertajam).

        Yang dilihat adalah tikungan TERTAJAM pada jarak sejauh lookahead di
        depan, supaya perlambatan sudah selesai saat robot tiba di tikungan:
        firmware meramp kecepatan dengan akselerasi terbatas, jadi keputusan
        yang diambil tepat di mulut tikungan datang terlambat.

        Pada belokan terakhir menjelang tujuan, final_turn_ratio dipakai sebagai
        ganti curve_speed_ratio (yang lebih pelan yang menang). Bagian lurus di
        ujung rute tidak ikut melambat karena kelengkungannya nol - yang
        mengurus bagian itu tetap slowdown_dist.
        """
        cum, k = self.cum[self.seg_i], self.curv[self.seg_i]
        ratio = self.curve_speed_ratio
        zona_akhir = (self.final_turn_ratio < 1.0 and self.seg_i == len(self.segments) - 1
                      and cum[-1] - cum[self.idx] <= self.final_turn_dist)
        if zona_akhir:
            ratio = min(ratio, self.final_turn_ratio)
        if ratio >= 1.0 and not zona_akhir:
            return 1.0
        batas = cum[self.idx] + self.lookahead
        j = self.idx
        k_max = 0.0
        while j < len(k) and cum[j] <= batas:
            k_max = max(k_max, k[j])
            j += 1
        f = min(1.0, k_max / self.k_ref) if self.k_ref > 0 else 0.0
        hasil = 1.0 - (1.0 - ratio) * f
        if zona_akhir:
            hasil = min(hasil, self._final_hold if self._final_hold is not None else hasil)
            self._final_hold = hasil
        return hasil

    def _predict(self, x, y, yaw, now):
        """Pose robot diproyeksikan maju sejauh self.lag_comp detik.

        Kecepatan diturunkan dari selisih pose antar frame lalu dilicinkan
        (EMA): menurunkan turunan dari pose kamera mentah menghasilkan derau
        yang, kalau langsung dikalikan lag_comp, justru membuat kemudi gemetar.
        """
        p, self.prev = self.prev, (x, y, yaw, now)
        if self.lag_comp <= 0.0 or p is None:
            return x, y, yaw
        dt = now - p[3]
        if dt <= 0.0 or dt > 0.5:       # frame lompat / kamera sempat putus
            return x, y, yaw
        a = 0.3
        vx = (1 - a) * self.vel[0] + a * (x - p[0]) / dt
        vy = (1 - a) * self.vel[1] + a * (y - p[1]) / dt
        vr = (1 - a) * self.vel[2] + a * wrap_angle(yaw - p[2]) / dt
        self.vel = (vx, vy, vr)
        tau = self.lag_comp
        return x + vx * tau, y + vy * tau, yaw + vr * tau

    def path_blocked(self, omap):
        """True bila ada pose di sisa jalur yang tadinya bebas kini tertutup rintangan baru."""
        if self.done:
            return False
        for si in range(self.seg_i, len(self.segments)):
            pts, _ = self.segments[si]
            start = self.idx + 2 if si == self.seg_i else 0
            for j in range(start, len(pts)):
                if self.watch[si][j] and not omap.pose_free(self.v, pts[j][0], pts[j][1], pts[j][2]):
                    return True
        return False

    def _stop(self):
        return {"speed": 0, "servo": self.servo_center, "steer_deg": 0.0, "target": None,
                "dir": 0, "done": self.done, "paused": False}

    # Bagian di bawah ini sengaja dipecah dari update(): semuanya TIDAK
    # bergantung pada algoritma kemudi, jadi pelacak lain (mis. MPCTracker di
    # mpc_tracker.py) bisa memakainya ulang dan hanya mengganti cara menghitung
    # sudut kemudi. Dengan begitu perbandingan antar pelacak benar-benar adil:
    # pemilihan segmen, profil kecepatan, dan konversi servo persis sama.

    def _advance(self, x, y, now):
        """Pindah segmen bila ujungnya tercapai, lalu perbarui titik terdekat.

        Mengisi self.seg_i, self.idx, dan self.deviation. Mengembalikan True
        bila seluruh rute sudah selesai.
        """
        while not self.done:
            pts, d = self.segments[self.seg_i]
            ex, ey, eyaw = pts[-1][:3]
            tx, ty = d * math.cos(eyaw), d * math.sin(eyaw)
            dist_end = math.hypot(ex - x, ey - y)
            passed = (x - ex) * tx + (y - ey) * ty > 0 and dist_end < self.lookahead
            if dist_end >= self.seg_end_tol and not passed:
                break
            self.seg_i += 1
            self.idx = 0
            if self.seg_i >= len(self.segments):
                self.done = True
            elif self.segments[self.seg_i][1] != d:
                self.pause_until = now + self.cusp_pause   # beri waktu servo & motor ganti arah
        if self.done:
            return True

        # Kemajuan di jalur: titik terdekat, hanya boleh maju
        pts, _ = self.segments[self.seg_i]
        best_j, best_dd = self.idx, float('inf')
        for j in range(self.idx, min(len(pts), self.idx + 40)):
            dd = math.hypot(pts[j][0] - x, pts[j][1] - y)
            if dd < best_dd:
                best_j, best_dd = j, dd
        self.idx, self.deviation = best_j, best_dd
        return False

    def _ratio(self):
        """Faktor kecepatan gabungan: melambat di ujung segmen DAN di tikungan.

        Dua-duanya alasan yang berdiri sendiri, jadi yang dipakai yang paling pelan.
        """
        cum = self.cum[self.seg_i]
        remaining = cum[-1] - cum[self.idx]
        ratio = max(self.min_speed_ratio, min(1.0, remaining / self.slowdown_dist))
        return min(ratio, self._curve_ratio())

    def _lookahead_target(self, x, y):
        """Titik lookahead; lewat ujung segmen diperpanjang lurus searah heading akhir."""
        pts, d = self.segments[self.seg_i]
        ld = self.lookahead
        for j in range(self.idx + 1, len(pts)):
            if math.hypot(pts[j][0] - x, pts[j][1] - y) >= ld:
                return _circle_hit(pts[j - 1][0], pts[j - 1][1], pts[j][0], pts[j][1], x, y, ld) \
                    or (pts[j][0], pts[j][1])
        ex, ey, eyaw = pts[-1][:3]
        ext = max(ld - math.hypot(ex - x, ey - y), 0.3 * ld)
        return (ex + d * math.cos(eyaw) * ext, ey + d * math.sin(eyaw) * ext)

    def _command(self, delta, target, now):
        """Bungkus sudut kemudi (radian) menjadi perintah untuk firmware."""
        d = self.segments[self.seg_i][1]
        delta = max(-self.v.max_steer, min(self.v.max_steer, delta))
        steer_deg = math.degrees(delta)
        servo = int(round(self.servo_center + steer_deg * self.servo_per_deg))
        servo = max(self.servo_min, min(self.servo_max, servo))

        speed = d * (self.forward_speed if d > 0 else self.reverse_speed) * self._ratio()
        paused = now < self.pause_until
        if paused:
            speed = 0

        return {"speed": int(speed), "servo": servo, "steer_deg": steer_deg, "target": target,
                "dir": d, "done": False, "paused": paused}

    def update(self, x, y, yaw, now):
        if self._advance(x, y, now):
            return self._stop()

        d = self.segments[self.seg_i][1]
        target = self._lookahead_target(x, y)

        # Pure pursuit pada titik poros tetap. Saat mundur, arah gerak = heading + 180 derajat.
        # Titik lookahead dipilih dari pose SEBENARNYA (di atas), tapi sudut
        # kemudinya dihitung dari pose PREDIKSI: perintah yang dikirim sekarang
        # baru terwujud saat robot sudah sampai di sana.
        px, py, pyaw = self._predict(x, y, yaw, now)
        motion_yaw = pyaw if d > 0 else pyaw + math.pi
        alpha = wrap_angle(math.atan2(target[1] - py, target[0] - px) - motion_yaw)
        dist_t = max(math.hypot(target[0] - px, target[1] - py), 1e-6)
        kappa = d * 2.0 * math.sin(alpha) / dist_t
        return self._command(math.atan(self.v.L * kappa), target, now)
