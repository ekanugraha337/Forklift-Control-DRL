"""
lingkungan.py - MDP perencanaan untuk DQfD, pengganti ruang pencarian Hybrid A*.

Satu episode = satu panggilan plan(): dari pose awal, agen memilih primitif
gerak satu per satu sampai TIBA di toleransi tujuan. Tidak ada dinamika (jeda
kemudi, ramp gas, kamera) - itu urusan pelacak, yang tidak berubah.

FULL DQfD (versi 2, 2026-10-20). Demonstratornya Hybrid A* + Reeds-Shepp,
tapi saat dipakai TIDAK ADA algoritma Reeds-Shepp maupun A*: seluruh jalur
dipilih jaringan. Dulu ekor Reeds-Shepp adalah satu aksi SAMBUNG yang
memanggil planner._analytic; sekarang ekor itu diurai (adaptor_demo) menjadi
primitif biasa ditambah primitif PENDEK, dan jaringan menirunya langkah demi
langkah. Busur Reeds-Shepp memakai kelengkungan yang sama persis dengan
primitif kemudi penuh/lurus (radius = 1/kappa_max); yang berbeda hanya
panjangnya, dan sisa yang bukan kelipatan langkah ditutup primitif pendek.
Diukur pada 210 ekor RS: dengan 1/2, 1/4, 1/8 langkah semuanya mendarat di
toleransi tujuan (galat maks 1,9 cm / 2,3 deg).

Bagian A* tetap direkonstruksi PERSIS: aritmetika ekspansi primitif penuh
sama dengan loop HybridAStarPlanner.plan() sampai bit terakhir, jadi biaya
bagian itu sama dengan g-cost A*. Diperiksa oleh "python -m drl.adaptor_demo --uji".

Pustaka geraknya adalah objek HybridAStarPlanner itu sendiri: kappas, step,
n_sub, bobot biaya, heuristik, _at_goal, _key. Hanya ekspansi satu primitif
yang ditulis ulang (_ekspansi), karena di forklift_planner.py ia berada di
dalam loop plan(), bukan fungsi tersendiri - dan berkas itu tidak diubah.

SATUAN. Seperti perencana: pose dan biaya dalam SEL grid, yaw radian. Hanya
observasi yang dinormalisasi dari cm.

AKSI (32)
    0..6     maju,   langkah penuh, kappa = planner.kappas[0..6] (-kmax .. +kmax)
    7..13    mundur, langkah penuh, sama
    14..31   PENDEK: panjang {1/2, 1/4, 1/8} langkah x kappa {-kmax, 0, +kmax}
             x {maju, mundur} - lihat aksi_info(). Hanya kemudi penuh dan
             lurus, karena hanya itu yang dipakai Reeds-Shepp.

MASK - aksi terlarang, Q-nya dibuat -inf sebelum argmax:
    aksi yang badannya menyentuh zona di salah satu sub-titiknya
        (aturan A*, termasuk kelonggaran "lepas diri" bila awalnya menempel);
    primitif PENUH yang mendarat di sel yang sudah dikunjungi DENGAN ARAH
        GERAK YANG SAMA (planner._key + arah) - padanan closed set supaya
        rollout tidak berputar-putar. Arah ikut dihitung karena manuver
        maju-mundur Reeds-Shepp memang melewati sel yang sama dua kali;
    primitif PENDEK yang mendarat di sel HALUS yang sudah dikunjungi dengan
        arah yang sama - sel 1/8 xy_reso (0,6 cm) dan 1/8 yaw_reso. Sel kasar
        tidak bisa dipakai: primitif pendek hampir selalu mendarat di sel
        kasar yang sama. Sel halus lebih kecil dari gerak terpendek (1/8
        langkah = 0,91 cm > 0,6 x akar 2), jadi satu gerak pendek selalu
        pindah sel, sedangkan bolak-balik ke tempat yang sama tertutup -
        tanpa ini pencarian terbaik-dulu membuka cabang pendek yang nyaris
        sama berulang-ulang;
    primitif PENDEK setelah MAKS_PENDEK dipakai dalam satu episode.

BIAYA per langkah = rumus biaya ekspansi A* dengan panjang aksi itu
(reward = -biaya). Untuk ekor RS jumlahnya mendekati biaya Reeds-Shepp
(_rs_cost), tidak persis: sisa panjang dibulatkan ke 1/8 langkah. Episode
berakhir bila TIBA (planner._at_goal), buntu (semua aksi ter-mask,
+ BIAYA_BUNTU_CM), atau melewati MAKS_LANGKAH (terpotong, bukan gagal - untuk
DQN itu bukan keadaan terminal).
"""
import math

import numpy as np

import konfig
import rencana as R

N_KEMUDI = 7
N_PRIMITIF = 2 * N_KEMUDI
# Primitif pendek: pecahan langkah x indeks planner.kappas (kanan penuh, lurus,
# kiri penuh) x arah. Urutannya bagian dari kontrak model - jangan diubah tanpa
# menaikkan VERSI_AKSI.
PECAHAN_PENDEK = (0.5, 0.25, 0.125)
KEMUDI_PENDEK = (0, N_KEMUDI // 2, N_KEMUDI - 1)
N_PENDEK = len(PECAHAN_PENDEK) * len(KEMUDI_PENDEK) * 2
N_AKSI = N_PRIMITIF + N_PENDEK
N_SINAR = 32
N_OBS = 47
# Dicatat di checkpoint (dqfd.meta_profil): model dengan ruang aksi lain ditolak.
# 1 = 14 primitif + SAMBUNG (sampai 2026-10-20), 2 = full DQfD, 32 aksi.
VERSI_AKSI = 2
# Primitif pendek per episode. Satu segmen RS butuh <= 3 (1/2 + 1/4 + 1/8),
# ekor RS terpanjang yang terukur 6 segmen -> 18. Sisanya cadangan.
MAKS_PENDEK = 24
# Sel halus untuk primitif pendek, pecahan dari xy_reso / yaw_reso perencana.
HALUS = 8

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


def aksi_info(planner, a):
    """Aksi 0..N_AKSI-1 -> (arah +1/-1, kappa 1/sel, pecahan langkah 0..1)."""
    if a < N_PRIMITIF:
        return (1 if a < N_KEMUDI else -1), planner.kappas[a % N_KEMUDI], 1.0
    i = a - N_PRIMITIF
    i_pecah, sisa = divmod(i, len(KEMUDI_PENDEK) * 2)
    i_kemudi, i_arah = divmod(sisa, 2)
    return ((1, -1)[i_arah], planner.kappas[KEMUDI_PENDEK[i_kemudi]],
            PECAHAN_PENDEK[i_pecah])


def arah_kappa(planner, a):
    """Aksi -> (arah +1/-1, kappa dalam 1/sel). Panjangnya lihat aksi_info()."""
    d, k, _ = aksi_info(planner, a)
    return d, k


def aksi_dari(planner, arah, kappa, pecahan=1.0):
    """Kebalikan aksi_info: kappa dibulatkan ke tingkat terdekat.

    Untuk pecahan < 1 hanya kemudi penuh dan lurus yang ada; ValueError bila
    pecahan bukan salah satu PECAHAN_PENDEK.
    """
    i = int(np.argmin([abs(kappa - k) for k in planner.kappas]))
    if pecahan == 1.0:
        return i + (0 if arah > 0 else N_KEMUDI)
    if pecahan not in PECAHAN_PENDEK or i not in KEMUDI_PENDEK:
        raise ValueError(f"tidak ada primitif pendek {pecahan} langkah, kappa {kappa:.4g}")
    return (N_PRIMITIF + PECAHAN_PENDEK.index(pecahan) * len(KEMUDI_PENDEK) * 2
            + KEMUDI_PENDEK.index(i) * 2 + (0 if arah > 0 else 1))


def tabel_aksi(planner):
    """Keterangan tiap aksi: (nama, arah, kappa, sudut kemudi derajat, radius sel)."""
    baris = []
    for a in range(N_AKSI):
        d, k, f = aksi_info(planner, a)
        sudut = math.degrees(math.atan(k * planner.v.L))
        radius = abs(1.0 / k) if k else float("inf")
        nama = f"{'maju' if d > 0 else 'mundur'} {sudut:+.1f}"
        if f < 1.0:
            nama += f" x{f:g}"
        baris.append((nama, d, k, sudut, radius))
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
        self.biaya_buntu = s(BIAYA_BUNTU_CM)

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
        self.n_pendek = 0
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
        # Sel awal tertutup untuk kedua arah: kembali ke pose awal tidak berguna.
        k0 = pl._key(sx, sy, syaw)
        self.dikunjungi = {(k0, 1), (k0, -1)}
        h0 = self._key_halus(sx, sy, syaw)
        self.dikunjungi_halus = {(h0, 1), (h0, -1)}
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
        poses, biaya, col = self._calon[a]
        nd, nk, pecahan = aksi_info(pl, a)
        self.x, self.y, self.yaw = poses[-1][0], poses[-1][1], poses[-1][2]
        self.d, self.k, self.in_col = nd, nk, col
        if pecahan < 1.0:
            self.n_pendek += 1
            self.dikunjungi_halus.add((self._key_halus(self.x, self.y, self.yaw), nd))
        else:
            self.dikunjungi.add((pl._key(self.x, self.y, self.yaw), nd))
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
        return self.observasi(), reward, self.selesai, info

    def simpan_keadaan(self):
        """Keadaan untuk pencarian mundur (backtracking). Himpunan sel yang
        sudah dikunjungi SENGAJA tidak ikut: setelah mundur, sel yang pernah
        dicoba tetap tertutup - padanan closed set A*, supaya cabang yang sama
        tidak dijelajahi dua kali."""
        return (self.x, self.y, self.yaw, self.d, self.k, self.in_col,
                len(self._pose), self.biaya, self.langkah_ke, self.n_pendek)

    def pulihkan_keadaan(self, s):
        (self.x, self.y, self.yaw, self.d, self.k, self.in_col,
         n, self.biaya, self.langkah_ke, self.n_pendek) = s
        del self._pose[n:]
        self.selesai = self.sukses = False
        self._hitung_calon()          # mask baru: sel yang dikunjungi sejak itu ikut tertutup

    def jalur(self):
        """Jalur yang sudah ditempuh, format plan(): [(x, y, yaw, arah), ...] dalam sel."""
        if not self._pose:
            return []
        return [(*self.awal, self._pose[0][3])] + self._pose

    # -------------------------------------------------------------- calon
    def _hitung_calon(self):
        """Hitung semua aksi dari keadaan sekarang: hasil + mask. Dipakai ulang step()."""
        pl = self.planner
        self._calon = [None] * N_AKSI
        self._mask = np.zeros(N_AKSI, dtype=bool)
        pendek_habis = self.n_pendek >= MAKS_PENDEK
        for a in range(N_AKSI):
            nd, nk, pecahan = aksi_info(pl, a)
            if nd < 0 and not self.boleh_mundur:
                continue
            if pecahan < 1.0 and pendek_habis:
                continue
            hasil = self._ekspansi(nd, nk, pecahan)
            if hasil is None:
                continue
            px, py, pyaw = hasil[0][-1][:3]
            if pecahan == 1.0 and (pl._key(px, py, pyaw), nd) in self.dikunjungi:
                continue
            if pecahan < 1.0 and (self._key_halus(px, py, pyaw), nd) in self.dikunjungi_halus:
                continue
            self._calon[a] = hasil
            self._mask[a] = True

    def _key_halus(self, x, y, yaw):
        """Sel halus (1/HALUS sel A*) untuk aturan sel tertutup primitif pendek."""
        pl = self.planner
        r, ry = pl.xy_reso / HALUS, pl.yaw_reso / HALUS
        return int(x // r), int(y // r), int((yaw % (2.0 * math.pi)) // ry)

    def _ekspansi(self, nd, nk, pecahan=1.0):
        """Satu aksi dari keadaan sekarang -> (poses, biaya, masih_menempel) atau None.

        Untuk primitif PENUH ini SALINAN aritmetika loop ekspansi
        HybridAStarPlanner.plan(), dengan urutan operasi yang sama - supaya
        jalur dan biaya sama sampai bit terakhir, bukan hanya mirip. Primitif
        pendek memakai rumus yang sama dengan panjang pecahan x step, dibagi
        sub-titik selebar-lebarnya sample_ds seperti sample_rs_path.
        """
        pl, omap, v = self.planner, self.omap, self.v
        x, y, yaw = self.x, self.y, self.yaw
        in_col = self.in_col
        m0 = omap.pose_margin(v, x, y, yaw) if in_col else 0.0
        if pecahan == 1.0:
            panjang, n_sub = pl.step, pl.n_sub
        else:
            panjang = pecahan * pl.step
            n_sub = max(1, math.ceil(panjang / pl.sample_ds))
        ds = nd * panjang / n_sub
        px, py, pyaw = x, y, yaw
        poses = []
        col, last_m, min_m = in_col, m0, float("inf")
        for _ in range(n_sub):
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
        cost = panjang * (1.0 if nd > 0 else pl.reverse_cost)
        cost += pl.steer_cost * panjang * abs(nk) / pl.kappa_max
        cost += pl.steer_change_cost * abs(nk - self.k) / pl.kappa_max
        if self.d != 0 and nd != self.d:
            cost += pl.switch_cost
        if min_m != float("inf"):
            cost += pl._clearance_penalty(min_m, panjang)
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
