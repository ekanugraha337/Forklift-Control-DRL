"""
lingkungan.py - MDP perencanaan untuk DQfD, pengganti ruang pencarian Hybrid A*.

Satu episode = satu panggilan plan(): dari pose awal, agen memilih primitif
gerak satu per satu sampai tersambung ke tujuan. Tidak ada dinamika (jeda
kemudi, ramp gas, kamera) - itu urusan pelacak, yang tidak berubah. Yang
dimodelkan hanya yang juga dimodelkan Hybrid A*, dan dengan aritmetika yang
SAMA PERSIS, supaya jalur A* adalah episode yang sah di sini dan jumlah
biayanya sama dengan g-cost A*. Diperiksa oleh "python -m drl.adaptor_demo --uji".

Pustaka geraknya adalah objek HybridAStarPlanner itu sendiri: kappas, step,
n_sub, bobot biaya, heuristik, _analytic, _at_goal, _key. Hanya ekspansi SATU
primitif yang ditulis ulang (_ekspansi), karena di forklift_planner.py ia
berada di dalam loop plan(), bukan fungsi tersendiri - dan berkas itu tidak
diubah. Kalau ekspansi di sana berubah, _ekspansi harus ikut, dan uji di atas
yang akan menangkapnya.

SATUAN. Seperti perencana: pose dan biaya dalam SEL grid, yaw radian. Hanya
observasi yang dinormalisasi dari cm.

AKSI (15)
    0..6    maju,   kappa = planner.kappas[0..6]   (-kmax .. +kmax)
    7..13   mundur, kappa = planner.kappas[0..6]
    14      SAMBUNG: jalur Reeds-Shepp bebas ke tujuan (planner._analytic),
            episode selesai

MASK - aksi terlarang, Q-nya dibuat -inf sebelum argmax:
    primitif yang badannya menyentuh zona di salah satu sub-titiknya
        (aturan A*, termasuk kelonggaran "lepas diri" bila awalnya menempel);
    primitif yang mendarat di sel yang sudah dikunjungi (planner._key) -
        padanan closed set, supaya rollout tidak berputar-putar;
    SAMBUNG bila tidak ada Reeds-Shepp bebas, atau bila badan masih menempel
        rintangan. TIDAK dibatasi jarak: A* juga mencoba Reeds-Shepp pada setiap
        ekspansi ke-analytic_every berapa pun jaraknya, dan demonya memang
        tersambung dari > analytic_dist. Biayanya terukur 0,3 ms rata-rata
        (maks 0,7 ms) di jarak mana pun, jadi pembatasan tidak perlu.
        Pengecualian: perencana "dqfd-murni" memasang jarak_sambung = 100 cm
        (perencana_dqfd.JARAK_SAMBUNG_MURNI_CM), RS hanya di akhir jalur.

BIAYA per langkah = biaya ekspansi A* (reward = -biaya). Episode berakhir bila
tersambung (SAMBUNG, atau primitif mendarat dalam toleransi tujuan), buntu
(semua aksi ter-mask, + BIAYA_BUNTU_CM), atau melewati MAKS_LANGKAH (terpotong,
bukan gagal - untuk DQN itu bukan keadaan terminal).
"""
import math

import numpy as np

import konfig
import rencana as R

N_KEMUDI = 7
N_PRIMITIF = 2 * N_KEMUDI
SAMBUNG = N_PRIMITIF
N_AKSI = N_PRIMITIF + 1
N_SINAR = 32
N_OBS = 47

# Skala normalisasi observasi (cm). Arena 300 x 270 cm, jadi jarak ternormalisasi
# tetap di sekitar [-1, 1].
JARAK_NORM_CM = 300.0
SINAR_JANGKAU_CM = 100.0
BEBAS_NORM_CM = 20.0

MAKS_LANGKAH = 300           # x 7,3 cm = 22 m, jauh di atas rute terpanjang di arena
# Harus lebih mahal dari jalur mana pun, kalau tidak agen yang menghadapi rute
# mahal "memilih" buntu. Biaya episode demo terbesar di dataset tahap 3:
# 2592 (ID 01) dan 2922 (ID 02); 2000 yang semula dipakai ternyata di bawahnya.
BIAYA_BUNTU_CM = 5000.0

# Iterasi maksimum sphere tracing per sinar. Langkah terkecil 0,5 sel, jadi
# sinar yang menyusur dinding pun sampai ke ujung jangkauannya.
_SINAR_ITERASI = 256

_TETANGGA = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def arah_kappa(planner, a):
    """Aksi primitif 0..13 -> (arah +1/-1, kappa dalam 1/sel)."""
    return (1 if a < N_KEMUDI else -1), planner.kappas[a % N_KEMUDI]


def aksi_dari(planner, arah, kappa):
    """Kebalikan arah_kappa: kappa dibulatkan ke tingkat terdekat."""
    i = int(np.argmin([abs(kappa - k) for k in planner.kappas]))
    return i + (0 if arah > 0 else N_KEMUDI)


def tabel_aksi(planner):
    """Keterangan tiap aksi: (nama, arah, kappa, sudut kemudi derajat, radius sel)."""
    baris = []
    for a in range(N_PRIMITIF):
        d, k = arah_kappa(planner, a)
        sudut = math.degrees(math.atan(k * planner.v.L))
        radius = abs(1.0 / k) if k else float("inf")
        baris.append((f"{'maju' if d > 0 else 'mundur'} {sudut:+.1f}", d, k, sudut, radius))
    baris.append(("SAMBUNG", 0, 0.0, 0.0, float("inf")))
    return baris


class LingkunganRencana:
    """Satu arena, satu robot, satu margin. reset() untuk tiap pasangan awal-tujuan.

    planner: HybridAStarPlanner yang dijadikan pustaka gerak. None = dibangun
    dari konfig untuk profil & margin ini, dengan fungsi yang sama yang dipakai
    Perencana (rencana.buat_kendaraan / buat_perencana).
    """

    def __init__(self, arena, profil=None, margin_cm=None, planner=None):
        self.arena = arena
        self.p = profil or konfig.profil()
        self.margin_cm = konfig.SAFETY_MARGIN_CM if margin_cm is None else float(margin_cm)
        if planner is None:
            kendaraan = R.buat_kendaraan(arena, self.margin_cm, self.p.max_steer_rad)
            planner = R.buat_perencana(arena, kendaraan)
        if len(planner.kappas) != N_KEMUDI:
            raise ValueError(f"perencana punya {len(planner.kappas)} tingkat kemudi, "
                             f"ruang aksi dirancang untuk {N_KEMUDI}")
        self.planner = planner
        self.v = planner.v
        self.boleh_mundur = -1 in planner.dirs

        s = arena.panjang_ke_sel
        self._jarak_norm = s(JARAK_NORM_CM)
        self._bebas_norm = s(BEBAS_NORM_CM)
        self._jangkau = s(SINAR_JANGKAU_CM)
        # Batas jarak SAMBUNG - tak hingga, lihat MASK di atas. Hanya untuk
        # percobaan; nilai terbatas membuat demo A* ada yang ter-mask.
        self.jarak_sambung = float("inf")
        self.biaya_buntu = s(BIAYA_BUNTU_CM)
        # False: step() mengembalikan obs None. Perencana (perencana_dqfd)
        # menghitung observasi sendiri hanya untuk simpul yang dibuka, jadi
        # observasi di step() - 32 sinar - terbuang. Pelatihan butuh True.
        self.obs_step = True

        # Sinar berpangkal di pusat badan, bukan di poros depan (titik acuan
        # pose): pusat badan 9,5 cm di belakangnya. Jarak yang dilaporkan
        # dihitung dari TEPI badan (tanpa margin), jadi 0 = menempel.
        v = self.v
        self._pusat_off = (v.body_front - v.body_rear) / 2.0
        a, b = (v.body_front + v.body_rear) / 2.0, v.half_width
        self._sudut = np.arange(N_SINAR) * (2.0 * math.pi / N_SINAR)
        c, sn = np.abs(np.cos(self._sudut)), np.abs(np.sin(self._sudut))
        with np.errstate(divide="ignore"):
            self._tepi = np.minimum(np.where(c > 1e-9, a / c, np.inf),
                                    np.where(sn > 1e-9, b / sn, np.inf))

        self.selesai = True

    # ------------------------------------------------------------ episode
    def reset(self, awal, tujuan, omap=None):
        """awal = (gx, gy, yaw); tujuan = (gx, gy, yaw atau None). Dalam SEL.

        omap: peta dengan rintangan tambahan (Perencana.peta_dengan), None = arena.
        Kembalian (obs, info). obs None bila pasangan ini tidak sah - dengan
        pemeriksaan yang sama dengan awal plan().
        """
        self.omap = self.arena.omap if omap is None else omap
        pl, v = self.planner, self.v
        sx, sy, syaw = awal
        gx, gy, gyaw = tujuan
        self.awal = (sx, sy, syaw)
        self.tujuan = (gx, gy, gyaw)
        self._pose = []
        self.biaya = 0.0
        self.langkah_ke = 0
        self.selesai = True
        self.sukses = False

        if gyaw is not None and not self.omap.pose_free(v, gx, gy, gyaw):
            return None, {"sah": False, "alasan": "tujuan menabrak rintangan"}
        if gyaw is None and self.omap.clearance(gx, gy) < v.half_width:
            return None, {"sah": False, "alasan": "tujuan terlalu dekat rintangan"}
        if pl._at_goal(sx, sy, syaw, self.tujuan):
            return None, {"sah": False, "alasan": "sudah di tujuan"}

        # Keadaan bersama di objek perencana, diisi persis seperti plan().
        pl._hcost = pl._holonomic_heuristic(self.omap, gx, gy)
        pl._approach_dirs = pl._feasible_approach_dirs(self.omap, self.tujuan)
        self._hgrid = np.array(pl._hcost, dtype=np.float64)

        self.x, self.y, self.yaw = sx, sy, syaw
        self.d, self.k = 0, 0.0
        self.in_col = self.omap.pose_margin(v, sx, sy, syaw) <= 0
        self.dikunjungi = {pl._key(sx, sy, syaw)}
        self.selesai = False
        self._hitung_calon()
        if not self._mask.any():
            # Tidak ada satu aksi pun yang sah dari pose awal: jawaban yang benar
            # adalah "tidak ada jalur", bukan memaksa jaringan memilih.
            self.selesai = True
            return None, {"sah": False, "alasan": "buntu"}
        return self.observasi(), {"sah": True, "alasan": ""}

    def reset_cm(self, awal_cm, tujuan_cm, omap=None):
        """Sama dengan reset(), pose dalam (x_cm, y_cm, yaw_deg); yaw tujuan boleh None."""
        a = self.arena.pose_ke_sel(*awal_cm)
        if tujuan_cm[2] is None:
            gx, gy = self.arena.ke_sel(tujuan_cm[0], tujuan_cm[1])
            t = (gx, gy, None)
        else:
            t = self.arena.pose_ke_sel(*tujuan_cm)
        return self.reset(a, t, omap)

    def mask(self):
        return self._mask.copy()

    def step(self, a):
        """Kembalian (obs, reward, selesai, info). reward = -biaya langkah ini."""
        if self.selesai:
            raise RuntimeError("episode sudah selesai - panggil reset()")
        a = int(a)
        if not self._mask[a]:
            raise ValueError(f"aksi {a} ter-mask")

        pl = self.planner
        info = {"alasan": "", "terpotong": False}
        if a == SAMBUNG:
            poses, biaya = self._calon[SAMBUNG]
            self.x, self.y, self.yaw = poses[-1][0], poses[-1][1], poses[-1][2]
            self.selesai = self.sukses = True
            info["alasan"] = "tersambung"
        else:
            poses, biaya, col = self._calon[a]
            nd, nk = arah_kappa(pl, a)
            self.x, self.y, self.yaw = poses[-1][0], poses[-1][1], poses[-1][2]
            self.d, self.k, self.in_col = nd, nk, col
            self.dikunjungi.add(pl._key(self.x, self.y, self.yaw))
            if pl._at_goal(self.x, self.y, self.yaw, self.tujuan):
                self.selesai = self.sukses = True
                info["alasan"] = "tiba"

        self._pose.extend(poses)
        self.biaya += biaya
        self.langkah_ke += 1
        reward = -biaya

        if not self.selesai:
            self._hitung_calon()
            if not self._mask.any():
                self.selesai = True
                reward -= self.biaya_buntu
                info["alasan"] = "buntu"
            elif self.langkah_ke >= MAKS_LANGKAH:
                self.selesai = True
                info["alasan"] = "batas langkah"
                info["terpotong"] = True

        info["sukses"] = self.sukses
        info["biaya"] = self.biaya
        return self.observasi() if self.obs_step else None, reward, self.selesai, info

    def simpan_keadaan(self):
        """Keadaan untuk pencarian mundur (backtracking). Himpunan sel yang
        sudah dikunjungi SENGAJA tidak ikut: setelah mundur, sel yang pernah
        dicoba tetap tertutup - padanan closed set A*, supaya cabang yang sama
        tidak dijelajahi dua kali."""
        return (self.x, self.y, self.yaw, self.d, self.k, self.in_col,
                len(self._pose), self.biaya, self.langkah_ke)

    def pulihkan_keadaan(self, s, calon=None):
        """calon: simpan_calon() yang diambil di keadaan s. Hasil primitif dan
        Reeds-Shepp hanya bergantung pada keadaan itu, jadi tidak perlu
        dihitung ulang - hanya mask sel yang dikunjungi sejak itu. Hasilnya
        sama persis dengan tanpa calon."""
        (self.x, self.y, self.yaw, self.d, self.k, self.in_col,
         n, self.biaya, self.langkah_ke) = s
        del self._pose[n:]
        self.selesai = self.sukses = False
        if calon is None:
            self._hitung_calon()      # mask baru: sel yang dikunjungi sejak itu ikut tertutup
            return
        hasil, mask, kunci = calon
        self._calon, self._mask, self._kunci = list(hasil), mask.copy(), kunci
        for a in range(N_PRIMITIF):
            if self._mask[a] and kunci[a] in self.dikunjungi:
                self._calon[a] = None
                self._mask[a] = False

    def simpan_calon(self):
        """Calon aksi keadaan sekarang, untuk pulihkan_keadaan(s, calon) dan calon_sah()."""
        return self._calon, self._mask.copy(), self._kunci

    def calon_sah(self, calon, a):
        """Apakah aksi a dari simpan_calon() suatu keadaan masih sah SEKARANG,
        tanpa memulihkan keadaan itu - sama dengan mask()[a] sesudah pulihkan."""
        hasil, mask, kunci = calon
        return bool(mask[a]) and (a == SAMBUNG or kunci[a] not in self.dikunjungi)

    def jalur(self):
        """Jalur yang sudah ditempuh, format plan(): [(x, y, yaw, arah), ...] dalam sel."""
        if not self._pose:
            return []
        return [(*self.awal, self._pose[0][3])] + self._pose

    # -------------------------------------------------------------- calon
    def _hitung_calon(self):
        """Hitung ke-15 aksi dari keadaan sekarang: hasil + mask. Dipakai ulang step()."""
        pl = self.planner
        self._calon = [None] * N_AKSI
        self._mask = np.zeros(N_AKSI, dtype=bool)
        self._kunci = [None] * N_PRIMITIF       # sel pendaratan tiap primitif, untuk calon_sah()
        for a in range(N_PRIMITIF):
            nd, nk = arah_kappa(pl, a)
            if nd < 0 and not self.boleh_mundur:
                continue
            hasil = self._ekspansi(nd, nk)
            if hasil is None:
                continue
            px, py, pyaw = hasil[0][-1][:3]
            kunci = pl._key(px, py, pyaw)
            if kunci in self.dikunjungi:
                continue
            self._calon[a] = hasil
            self._mask[a] = True
            self._kunci[a] = kunci

        gx, gy = self.tujuan[0], self.tujuan[1]
        if not self.in_col and math.hypot(gx - self.x, gy - self.y) < self.jarak_sambung:
            rs = pl._analytic(self.omap, (self.x, self.y, self.yaw, self.d, self.k), self.tujuan)
            if rs is not None:
                self._calon[SAMBUNG] = rs
                self._mask[SAMBUNG] = True

    def _ekspansi(self, nd, nk):
        """Satu primitif dari keadaan sekarang -> (poses, biaya, masih_menempel) atau None.

        SALINAN aritmetika loop ekspansi HybridAStarPlanner.plan(), dengan
        urutan operasi yang sama - supaya jalur dan biaya sama sampai bit
        terakhir, bukan hanya mirip.
        """
        pl, omap, v = self.planner, self.omap, self.v
        x, y, yaw = self.x, self.y, self.yaw
        in_col = self.in_col
        m0 = omap.pose_margin(v, x, y, yaw) if in_col else 0.0
        ds = nd * pl.step / pl.n_sub
        px, py, pyaw = x, y, yaw
        poses = []
        col, last_m, min_m = in_col, m0, float("inf")
        for _ in range(pl.n_sub):
            if nk == 0.0:
                px += ds * math.cos(pyaw)
                py += ds * math.sin(pyaw)
            else:
                nyaw = pyaw + nk * ds
                px += (math.sin(nyaw) - math.sin(pyaw)) / nk
                py -= (math.cos(nyaw) - math.cos(pyaw)) / nk
                pyaw = nyaw
            m = omap.pose_margin(v, px, py, pyaw)
            if m > 0:
                col = False
                min_m = min(min_m, m)
            elif not (col and m >= last_m - 0.5):
                return None
            last_m = m
            poses.append((px, py, pyaw, nd))
        cost = pl.step * (1.0 if nd > 0 else pl.reverse_cost)
        cost += pl.steer_cost * pl.step * abs(nk) / pl.kappa_max
        cost += pl.steer_change_cost * abs(nk - self.k) / pl.kappa_max
        if self.d != 0 and nd != self.d:
            cost += pl.switch_cost
        if min_m != float("inf"):
            cost += pl._clearance_penalty(min_m, pl.step)
        return poses, cost, col

    # ---------------------------------------------------------- observasi
    def observasi(self):
        """Vektor 47 angka. Tata letaknya di catatan Obsidian, bagian 7.1."""
        o = np.zeros(N_OBS, dtype=np.float32)
        pl = self.planner
        x, y, yaw = self.x, self.y, self.yaw
        gx, gy, gyaw = self.tujuan
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = gx - x, gy - y
        J = self._jarak_norm

        o[0] = (c * dx + s * dy) / J
        o[1] = (-s * dx + c * dy) / J
        o[2] = math.hypot(dx, dy) / J
        if gyaw is None:
            o[3], o[4], o[5] = 1.0, 0.0, 1.0
        else:
            e = gyaw - yaw
            o[3], o[4] = math.cos(e), math.sin(e)
        o[6] = 1.0 in pl._approach_dirs
        o[7] = -1.0 in pl._approach_dirs
        o[8] = self.d
        o[9] = self.k / pl.kappa_max
        o[10:10 + N_SINAR] = self._sinar()
        o[42] = pl._h(x, y, gx, gy) / J
        o[43], o[44] = self._arah_heuristik()
        o[45] = self.margin_cm / konfig.SAFETY_MARGIN_CM
        m = self.omap.pose_margin(self.v, x, y, yaw)
        o[46] = max(-1.0, min(1.0, m / self._bebas_norm))
        return o

    def _sinar(self):
        """Jarak bebas dari tepi badan ke zona, 32 arah, 0..1 (1 = >= jangkauan).

        Sphere tracing di atas distance transform: dari titik mana pun, lingkaran
        berjari-jari dist[titik] pasti kosong, jadi sinar boleh melompat sejauh
        itu tanpa melewatkan rintangan.
        """
        dist = self.omap.dist
        h, w = dist.shape
        off = self._pusat_off
        cx = self.x + off * math.cos(self.yaw)
        cy = self.y + off * math.sin(self.yaw)
        ang = self.yaw + self._sudut
        ux, uy = np.cos(ang), np.sin(ang)
        batas = self._tepi + self._jangkau
        t = np.zeros(N_SINAR)
        aktif = np.ones(N_SINAR, dtype=bool)
        for _ in range(_SINAR_ITERASI):
            ix = np.floor(cx + t * ux).astype(np.int64)
            iy = np.floor(cy + t * uy).astype(np.int64)
            dalam = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
            d = np.where(dalam, dist[np.clip(iy, 0, h - 1), np.clip(ix, 0, w - 1)], 0.0)
            aktif &= (d >= 1.0) & (t < batas)
            if not aktif.any():
                break
            t = np.where(aktif, t + np.maximum(d - 0.5, 0.5), t)
        return np.clip((np.minimum(t, batas) - self._tepi) / self._jangkau, 0.0, 1.0)

    def _arah_heuristik(self):
        """(cos, sin) arah turun heuristik Dijkstra, dalam kerangka robot.

        Arah ke tetangga 8 yang biayanya terkecil - bukan gradien beda-hingga,
        karena sel tertutup bernilai tak hingga dan gradien di tepinya tidak
        terdefinisi. Tanpa tetangga yang lebih murah: arah garis lurus ke tujuan.
        """
        r = self.planner.h_reso
        H = self._hgrid
        ny, nx = H.shape
        ix = min(nx - 1, max(0, int(self.x // r)))
        iy = min(ny - 1, max(0, int(self.y // r)))
        terbaik, arah = H[iy, ix], None
        for ddx, ddy in _TETANGGA:
            xx, yy = ix + ddx, iy + ddy
            if 0 <= xx < nx and 0 <= yy < ny and H[yy, xx] < terbaik:
                terbaik, arah = H[yy, xx], (ddx, ddy)
        if arah is None:
            arah = (self.tujuan[0] - self.x, self.tujuan[1] - self.y)
            if arah == (0.0, 0.0):
                return 0.0, 0.0
        sudut = math.atan2(arah[1], arah[0]) - self.yaw
        return math.cos(sudut), math.sin(sudut)
