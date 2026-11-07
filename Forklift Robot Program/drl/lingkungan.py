"""
lingkungan.py - MDP perencanaan untuk DQfD (model jauh), pengganti ruang pencarian Hybrid A*.

ATURAN ver-1-6. Satu model per robot, MODEL JAUH, yang meniru masukan & keluaran
Hybrid A* (demonstrator: Hybrid A* + Reeds-Shepp peran "demonstrator", w = 2,0,
h = max(h_RS, h_D), lihat rencana.buat_perencana):

    masukan  pose awal & tujuan + PETA RINTANGAN: 47 angka (tujuan relatif,
             32 sinar, heuristik Dijkstra h_D dan arah turunnya, margin, gerak
             terakhir). Demonstratornya sendiri mencari dengan h = max(h_D, h_RS);
             masukan model sengaja h_D saja: h_RS ditentukan pose tujuan relatif
             yang sudah jadi masukan, sedangkan h_D membawa informasi rintangan
             global yang tidak terlihat oleh sinar.
    keluaran 14 primitif (7 tingkat kemudi x maju/mundur), 7,3 cm

SERAH (akhir episode). Model jauh berhenti begitu titik acuan di dalam 60 cm
(analytic_dist demonstrator) DAN TIDAK ADA HALANGAN, lalu TEMBAKAN REEDS-SHEPP
menutup jalur ke tujuan. "Tidak ada halangan" diperiksa dua tahap:
    1. saringan cepat (_boleh_coba_serah): peta Dijkstra tidak memaksa memutar
       (biaya Dijkstra = jarak oktil grid), atau sudah < RADIUS_SERAH_PAKSA_CM;
    2. tembakan Reeds-Shepp dari pose itu bebas tabrakan (rs_bebas): fungsi
       _analytic Hybrid A* yang sama, jadi kurvanya PERSIS ekor RS demonstrator.
Selama masih ada halangan, model jauh MELANJUTKAN. Aturan ini sama saat membuat
dataset, saat latihan sendiri, dan saat dipakai robot.

Bagian A* direkonstruksi PERSIS: aritmetika ekspansi primitif sama dengan loop
HybridAStarPlanner.plan() sampai bit terakhir. Diperiksa oleh
"python -m drl.adaptor_demo --uji". Pustaka geraknya objek HybridAStarPlanner
itu sendiri (kappas, step, n_sub, bobot biaya, heuristik, _analytic, _key);
hanya ekspansi satu primitif yang ditulis ulang (_ekspansi), karena di
forklift_planner.py ia berada di dalam loop plan().

SATUAN. Seperti perencana: pose dan biaya dalam SEL grid, yaw radian. Hanya
observasi yang dinormalisasi.

MASK - aksi terlarang, Q-nya dibuat -inf sebelum argmax:
    aksi yang badannya menyentuh zona di salah satu sub-titiknya
        (aturan A*, termasuk kelonggaran "lepas diri" bila awalnya menempel);
    aksi yang mendarat di sel yang sudah dikunjungi DENGAN ARAH GERAK YANG SAMA
        (planner._key + arah) - padanan closed set supaya rollout tidak
        berputar-putar.

BIAYA per langkah = rumus biaya ekspansi A* (reward = -biaya). Langkah yang
berakhir SERAH ditambah biaya tembakan RS (_rs_cost + penalti jarak bebas, sama
dengan A*), jadi biaya episode = g-cost A* seluruh jalur. Episode berakhir bila
SERAH (sukses), buntu (semua aksi ter-mask, + BIAYA_BUNTU_CM), atau MAKS_LANGKAH
(terpotong, bukan gagal - untuk DQN itu bukan keadaan terminal).

JALAN PINTAS RUANG TERBUKA (pintas_terbuka, mati bawaan). Bila margin pose sekarang
melebihi ambang_terbuka, ke-14 primitif PASTI bebas tabrakan dan penalti jarak-bebasnya
PASTI nol (lihat _terbuka), jadi _ekspansi dijalankan tanpa cek tabrakan - hasilnya
sama persis dengan yang dicek. calon_malas() dan siapkan_aksi() melayani pencarian
lazy (perencana_dqfd.rencanakan_terbaik_malas): 14 primitif tidak disimulasi di muka,
hanya aksi yang diambil dari heap yang dicek tabrakannya.

SINAR (32 arah) dihitung dengan numba - wajib (ver-1-6), tanpa cadangan numpy.
"""
import math

import numpy as np

import konfig
import rencana as R

JENIS = "jauh"                # nama model (berkas model_dqn/id0N_jauh.pt, meta checkpoint)
N_KEMUDI = 7
N_AKSI = 2 * N_KEMUDI         # 14 primitif
N_OBS = 47
N_SINAR = 32
# Dicatat di checkpoint (dqfd.meta_model): model dengan versi lain ditolak.
# 9 = ver-1-6: satu model (jauh), demonstrator Hybrid A* w = 2,0, h = max(h_D, h_RS)
#     (tujuan tanpa arah hadap diselesaikan dengan sapuan arah hadap, jadi h_RS
#     selalu ikut), serah = < 60 cm + saringan Dijkstra + tembakan RS bebas,
#     imbalan akhir = biaya RS; masukan heuristik model = h_D (indeks 42) dan
#     arah turun h_D (indeks 43-44).
VERSI_AKSI = 9

# Saringan serah: di dalam jarak ini (cm) peta Dijkstra tidak lagi disyaratkan
# bebas - untuk tujuan yang mepet rintangan. Tembakan RS tetap harus bebas.
RADIUS_SERAH_PAKSA_CM = 25.0

# Normalisasi observasi (cm). Arena 300 x 270 cm -> sekitar [-1, 1].
JARAK_NORM_CM = 300.0
SINAR_JANGKAU_CM = 100.0
BEBAS_NORM_CM = 20.0

MAKS_LANGKAH = 300           # x 7,3 cm = 22 m, jauh di atas rute terpanjang di arena
# Harus lebih mahal dari jalur mana pun, kalau tidak agen yang menghadapi rute
# mahal "memilih" buntu.
BIAYA_BUNTU_CM = 5000.0

# Jalan pintas ruang terbuka (_terbuka). Mati bawaan; DQfDPlanner bisa menyalakannya.
# Rasio jarak chamfer 5x5 OpenCV (bobot 1; 1,4; 2,1969) terhadap jarak Euclid
# antar-piksel paling besar ~1,02; dipakai 1,05 supaya ada sisa aman.
PINTAS_TERBUKA_BAWAAN = False
_FAKTOR_CHAMFER = 1.05

# Iterasi maksimum sphere tracing per sinar. Langkah terkecil 0,5 sel, jadi
# sinar yang menyusur dinding pun sampai ke ujung jangkauannya.
_SINAR_ITERASI = 256


def _sinar_loop(dist, cx, cy, ux, uy, batas, iterasi):
    """Sphere tracing satu sinar demi satu sinar (dikompilasi numba): jarak tempuh t tiap sinar.

    Dari titik mana pun, lingkaran berjari-jari dist[titik] pasti kosong, jadi
    sinar boleh melompat sejauh itu tanpa melewatkan rintangan. cos/sin TIDAK
    dihitung di sini - tetap numpy di _sinar - karena libm numba bisa berbeda
    satu ulp dari numpy, dan floor() akan memperbesar beda itu menjadi sel yang
    berbeda. Sinar yang berhenti langsung selesai.
    """
    h, w = dist.shape
    n = ux.shape[0]
    t = np.zeros(n)
    for i in range(n):
        ti = 0.0
        for _ in range(iterasi):
            ix = int(math.floor(cx + ti * ux[i]))
            iy = int(math.floor(cy + ti * uy[i]))
            if 0 <= ix < w and 0 <= iy < h:
                d = float(dist[iy, ix])
            else:
                d = 0.0
            if not (d >= 1.0 and ti < batas[i]):
                break
            ti = ti + max(d - 0.5, 0.5)
        t[i] = ti
    return t


# numba WAJIB: 32 sinar adalah porsi terbesar waktu model jauh, dan aturan
# ver-1-6 menetapkan sinar dihitung dengan numba. cache=True: hasil kompilasi
# disimpan di __pycache__, jadi kompilasi (~1-2 s) hanya sekali per mesin;
# nogil=True: thread perencana tidak menahan GIL selama sinar dihitung.
# Dikompilasi saat impor (pemanasan di bawah), bukan saat rencana pertama robot.
try:
    from numba import njit as _njit
except ImportError as _e:                    # pragma: no cover - lingkungan tanpa numba
    raise ImportError("drl/lingkungan.py butuh numba untuk 32 sinar (aturan ver-1-6). "
                      "Pasang dengan: pip install numba") from _e
_sinar_inti = _njit(cache=True, nogil=True)(_sinar_loop)
_sinar_inti(np.ones((2, 2), np.float32), 0.5, 0.5, np.ones(1), np.ones(1), np.ones(1), 1)

_TETANGGA = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def n_aksi():
    """Jumlah aksi (keluaran jaringan): 14 primitif."""
    return N_AKSI


def n_obs():
    """Jumlah masukan jaringan: 47."""
    return N_OBS


def aksi_info(planner, a):
    """Aksi -> (arah +1/-1, kappa 1/sel, pecahan langkah = 1)."""
    return (1 if a < N_KEMUDI else -1), planner.kappas[a % N_KEMUDI], 1.0


def arah_kappa(planner, a):
    """Aksi -> (arah +1/-1, kappa dalam 1/sel)."""
    d, k, _ = aksi_info(planner, a)
    return d, k


def aksi_dari(planner, arah, kappa):
    """Kebalikan aksi_info: kappa dibulatkan ke tingkat terdekat."""
    i = int(np.argmin([abs(kappa - k) for k in planner.kappas]))
    return i + (0 if arah > 0 else N_KEMUDI)


def tabel_aksi(planner):
    """Keterangan tiap aksi: (nama, arah, kappa, sudut kemudi derajat, radius sel)."""
    baris = []
    for a in range(N_AKSI):
        d, k, f = aksi_info(planner, a)
        sudut = math.degrees(math.atan(k * planner.v.L))
        radius = abs(1.0 / k) if k else float("inf")
        baris.append((f"{'maju' if d > 0 else 'mundur'} {sudut:+.1f}", d, k, sudut, radius))
    return baris


class LingkunganRencana:
    """Satu arena, satu robot, satu margin. reset() untuk tiap pasangan awal-tujuan.

    planner: HybridAStarPlanner yang dijadikan pustaka gerak. None = dibangun
    dari konfig untuk profil & margin ini dengan peran "demonstrator"
    (rencana.buat_perencana). Batas serah = planner.analytic_dist.
    """

    def __init__(self, arena, profil=None, margin_cm=None, planner=None):
        self.arena = arena
        self.p = profil or konfig.profil()
        self.margin_cm = konfig.SAFETY_MARGIN_CM if margin_cm is None else float(margin_cm)
        if planner is None:
            kendaraan = R.buat_kendaraan(arena, self.margin_cm, self.p.max_steer_rad)
            planner = R.buat_perencana(arena, kendaraan, "demonstrator")
        if len(planner.kappas) != N_KEMUDI:
            raise ValueError(f"perencana punya {len(planner.kappas)} tingkat kemudi, "
                             f"ruang aksi dirancang untuk {N_KEMUDI}")
        if not planner.analytic_dist > 0:
            raise ValueError("pustaka gerak DQfD harus Hybrid A* peran 'demonstrator' "
                             "(analytic_dist = batas serah > 0)")
        self.planner = planner
        self.v = planner.v
        self.boleh_mundur = -1 in planner.dirs
        self.n_aksi = N_AKSI
        self.n_obs = N_OBS

        s = arena.panjang_ke_sel
        self._jarak_norm = s(JARAK_NORM_CM)
        self.radius_serah = planner.analytic_dist
        self.radius_paksa = s(RADIUS_SERAH_PAKSA_CM)
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
        self.sukses = False
        self.ekor_rs = None
        self._rs_calon = None
        self._obs_c = None          # cache observasi(), lihat di sana

        # Ambang jalan pintas ruang terbuka (lihat _terbuka). Titik badan mana pun
        # bergeser paling jauh step x sqrt(1 + (offset x kappa)^2) dalam satu primitif.
        p = self.planner
        off_maks = max(abs(o) for o in v.circle_offsets)
        self.gerak_maks = p.step * math.sqrt(1.0 + (off_maks * p.kappa_max) ** 2)
        self.ambang_terbuka = p.clearance_pref + _FAKTOR_CHAMFER * (self.gerak_maks + math.sqrt(2.0))
        self.pintas_terbuka = PINTAS_TERBUKA_BAWAAN

    # ------------------------------------------------------------ episode
    def reset(self, awal, tujuan, omap=None):
        """awal = (gx, gy, yaw); tujuan = (gx, gy, yaw atau None). Dalam SEL.

        omap: peta dengan rintangan tambahan (Perencana.peta_dengan), None = arena.
        Kembalian (obs, info). obs None bila pasangan ini tidak sah atau sudah
        di tujuan - dengan pemeriksaan yang sama dengan awal plan() - atau bila
        sudah boleh serah di pose awal (alasan "serah di pose awal": jalurnya
        tembakan RS saja, env.sukses True, env.jalur() siap dipakai).
        """
        self._obs_c = None
        self.omap = self.arena.omap if omap is None else omap
        pl, v = self.planner, self.v
        sx, sy, syaw = awal
        gx, gy, gyaw = tujuan
        self.awal = (sx, sy, syaw)
        self.tujuan = (gx, gy, gyaw)
        self._pose = []
        self.ekor_rs = None
        self.biaya = 0.0
        self.biaya_rs = 0.0
        self.langkah_ke = 0
        self.selesai = True
        self.sukses = False
        self._mask = np.zeros(N_AKSI, dtype=bool)

        if gyaw is not None and not self.omap.pose_free(v, gx, gy, gyaw):
            return None, {"sah": False, "alasan": "tujuan menabrak rintangan"}
        if gyaw is None and self.omap.clearance(gx, gy) < v.half_width:
            return None, {"sah": False, "alasan": "tujuan terlalu dekat rintangan"}
        if pl._at_goal(sx, sy, syaw, self.tujuan):
            return None, {"sah": False, "alasan": "sudah di tujuan"}

        # Keadaan bersama di objek perencana, diisi persis seperti plan().
        pl._hcost = pl._holonomic_heuristic(self.omap, gx, gy)
        self._hgrid = np.array(pl._hcost, dtype=np.float64)
        pl._approach_dirs = pl._feasible_approach_dirs(self.omap, self.tujuan)

        self.x, self.y, self.yaw = sx, sy, syaw
        self.d, self.k = 0, 0.0
        self.in_col = self.omap.pose_margin(v, sx, sy, syaw) <= 0
        if self._boleh_serah():
            self._serah()
            return None, {"sah": False, "alasan": "serah di pose awal"}
        # Sel awal tertutup untuk kedua arah: kembali ke pose awal tidak berguna.
        kunci = pl._key(sx, sy, syaw)
        self.dikunjungi = {(kunci, 1), (kunci, -1)}
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

    def step(self, a, hitung_calon=True):
        """Kembalian (obs, reward, selesai, info). reward = -biaya langkah ini
        (+ biaya tembakan RS bila langkah ini berakhir serah).

        hitung_calon=False (pencarian lazy): calon keadaan baru TIDAK dihitung -
        pemanggil memakai calon_malas() / siapkan_aksi(); buntu tidak dideteksi di sini."""
        if self.selesai:
            raise RuntimeError("episode sudah selesai - panggil reset()")
        a = int(a)
        if not self._mask[a]:
            raise ValueError(f"aksi {a} ter-mask")
        self._obs_c = None
        pl = self.planner
        info = {"alasan": "", "terpotong": False}
        poses, biaya, col = self._calon[a]
        nd, nk, _ = aksi_info(pl, a)
        self.x, self.y, self.yaw = poses[-1][0], poses[-1][1], poses[-1][2]
        self.d, self.k, self.in_col = nd, nk, col
        self.dikunjungi.add((pl._key(self.x, self.y, self.yaw), nd))
        self._pose.extend(poses)
        self.biaya += biaya
        self.langkah_ke += 1
        reward = -biaya

        if self._boleh_serah():
            self._serah()
            reward -= self.biaya_rs
            info["alasan"] = "serah"
        else:
            if hitung_calon:
                self._hitung_calon()
                buntu = not self._mask.any()
            else:
                self._kosongkan_calon()
                buntu = False
            if buntu:
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
                len(self._pose), self.biaya, self.langkah_ke)

    def pulihkan_keadaan(self, s, hitung_calon=True):
        """hitung_calon=False (pencarian lazy): calon tidak dihitung ulang."""
        self._obs_c = None
        (self.x, self.y, self.yaw, self.d, self.k, self.in_col,
         n, self.biaya, self.langkah_ke) = s
        del self._pose[n:]
        self.selesai = self.sukses = False
        self.ekor_rs = None
        self.biaya_rs = 0.0
        if hitung_calon:
            self._hitung_calon()      # mask baru: sel yang dikunjungi sejak itu ikut tertutup
        else:
            self._kosongkan_calon()

    def jalur(self):
        """Jalur yang sudah ditempuh, format plan(): [(x, y, yaw, arah), ...] dalam sel.
        Setelah serah termasuk ekor tembakan RS (n_titik_rs titik terakhir)."""
        pose = self._pose + (self.ekor_rs or [])
        if not pose:
            return []
        return [(*self.awal, pose[0][3])] + pose

    @property
    def n_titik_rs(self):
        """Jumlah titik ekor tembakan RS di ujung jalur() (0 bila belum serah)."""
        return len(self.ekor_rs) if self.ekor_rs else 0

    # -------------------------------------------------------------- serah
    def _di_radius(self, x, y):
        """Sudah di radius serah? Sama dengan syarat Reeds-Shepp demonstrator (titik acuan, '<')."""
        return math.hypot(self.tujuan[0] - x, self.tujuan[1] - y) < self.radius_serah

    def bebas_ke_tujuan(self, x, y):
        """Peta Dijkstra tidak memaksa memutar: biaya Dijkstra holonomik = jarak oktil grid.

        Heuristik Hybrid A* (_holonomic_heuristic) adalah Dijkstra 8-tetangga di
        grid h_reso, dengan sel yang terlalu sempit untuk badan sebagai
        rintangan. Di ruang bebas biayanya PERSIS jarak oktil antar-sel; lebih
        besar berarti jalannya harus memutari sesuatu.
        """
        pl = self.planner
        r = pl.h_reso
        H = self._hgrid
        ny, nx = H.shape
        ix, iy = min(nx - 1, max(0, int(x // r))), min(ny - 1, max(0, int(y // r)))
        tx = min(nx - 1, max(0, int(self.tujuan[0] // r)))
        ty = min(ny - 1, max(0, int(self.tujuan[1] // r)))
        dx, dy = abs(ix - tx), abs(iy - ty)
        oktil = r * (max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy))
        return bool(H[iy, ix] <= oktil + 1e-6)

    def _boleh_coba_serah(self, x, y):
        """Saringan cepat: di dalam 60 cm, dan peta Dijkstra bebas (atau < radius paksa)."""
        if not self._di_radius(x, y):
            return False
        if math.hypot(self.tujuan[0] - x, self.tujuan[1] - y) < self.radius_paksa:
            return True
        return self.bebas_ke_tujuan(x, y)

    def rs_bebas(self):
        """Tembakan Reeds-Shepp dari pose sekarang: (poses, biaya) bila bebas tabrakan, else None.

        Sama dengan syarat Hybrid A* beralih ke RS (plan(): badan tidak menempel,
        _analytic tidak None) - kurva dan biayanya PERSIS milik demonstrator."""
        if self.in_col:
            return None
        return self.planner._analytic(self.omap, (self.x, self.y, self.yaw, self.d, self.k), self.tujuan)

    def _boleh_serah(self):
        """Model jauh berhenti di sini? Saringan cepat + tembakan RS bebas (disimpan untuk _serah)."""
        self._rs_calon = None
        if not self._boleh_coba_serah(self.x, self.y):
            return False
        self._rs_calon = self.rs_bebas()
        return self._rs_calon is not None

    def _serah(self):
        """Tutup episode dengan tembakan RS yang baru lolos _boleh_serah."""
        poses, biaya = self._rs_calon
        self.ekor_rs = list(poses)
        self.biaya_rs = float(biaya)
        self.biaya += self.biaya_rs
        self.selesai = self.sukses = True
        self._mask = np.zeros(N_AKSI, dtype=bool)

    # -------------------------------------------------------------- calon
    def _hitung_calon(self):
        """Hitung semua aksi dari keadaan sekarang: hasil + mask. Dipakai ulang step().
        Di ruang terbuka (_terbuka) tanpa cek tabrakan - hasilnya sama persis."""
        pl = self.planner
        cek = not self._terbuka()
        self._calon = [None] * N_AKSI
        self._mask = np.zeros(N_AKSI, dtype=bool)
        for a in range(N_AKSI):
            nd, nk, _ = aksi_info(pl, a)
            if nd < 0 and not self.boleh_mundur:
                continue
            hasil = self._ekspansi(nd, nk, cek)
            if hasil is None:
                continue
            px, py, pyaw = hasil[0][-1][:3]
            if (pl._key(px, py, pyaw), nd) in self.dikunjungi:
                continue
            self._calon[a] = hasil
            self._mask[a] = True

    def _ekspansi(self, nd, nk, cek=True):
        """Satu primitif dari keadaan sekarang -> (poses, biaya, masih_menempel) atau None.

        SALINAN aritmetika loop ekspansi HybridAStarPlanner.plan(), dengan urutan
        operasi yang sama - supaya jalur dan biaya sama sampai bit terakhir,
        bukan hanya mirip.

        cek=False: tanpa cek tabrakan dan tanpa penalti jarak-bebas, tidak pernah
        None. Sama persis dengan cek=True bila _terbuka(); selain itu perkiraan
        (pencarian lazy).
        """
        pl, omap, v = self.planner, self.omap, self.v
        x, y, yaw = self.x, self.y, self.yaw
        in_col = self.in_col
        m0 = omap.pose_margin(v, x, y, yaw) if (cek and in_col) else 0.0
        panjang, n_sub = pl.step, pl.n_sub
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
            if cek:
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

    def _kosongkan_calon(self):
        self._calon = [None] * N_AKSI
        self._mask = np.zeros(N_AKSI, dtype=bool)

    def _terbuka(self):
        """Ruang terbuka: ke-14 primitif dari pose sekarang PASTI bebas tabrakan dan
        penalti jarak-bebasnya PASTI nol, tanpa mensimulasi satu pun.

        Dalam satu primitif, pusat lingkaran badan mana pun bergeser paling jauh
        gerak_maks. Nilai peta jarak (chamfer OpenCV) di titik yang bergeser d
        turun paling banyak FAKTOR x (d + sqrt 2); sqrt 2 dari pembulatan ke piksel.
        Jadi bila margin sekarang >= clearance_pref + FAKTOR x (gerak_maks + sqrt 2),
        setiap sub-titik bermargin >= clearance_pref: tidak menabrak, dan penalti
        jarak-bebasnya tepat 0. Badan yang sedang menempel tidak pernah terbuka.
        """
        if not self.pintas_terbuka or self.in_col:
            return False
        return self.omap.pose_margin(self.v, self.x, self.y, self.yaw) >= self.ambang_terbuka

    def calon_malas(self):
        """Pencarian lazy: [(aksi, biaya, terbuka), ...] dari keadaan sekarang, untuk
        aksi yang sel ujungnya belum dikunjungi. Pose ujung dari aritmetika gerak saja.

        terbuka=True (_terbuka): pasti bebas tabrakan dan biayanya tepat; aksi langsung
        siap untuk step(). terbuka=False: tabrakan belum dicek dan biayanya tanpa
        penalti jarak-bebas - siapkan_aksi() memeriksanya saat aksi diambil dari heap.
        """
        pl = self.planner
        terbuka = self._terbuka()
        self._kosongkan_calon()
        out = []
        for a in range(N_AKSI):
            nd, nk, _ = aksi_info(pl, a)
            if nd < 0 and not self.boleh_mundur:
                continue
            hasil = self._ekspansi(nd, nk, cek=False)
            px, py, pyaw = hasil[0][-1][:3]
            if (pl._key(px, py, pyaw), nd) in self.dikunjungi:
                continue
            if terbuka:
                self._calon[a] = hasil
                self._mask[a] = True
            out.append((a, hasil[1], terbuka))
        return out

    def siapkan_aksi(self, a, tanpa_cek=False):
        """Pencarian lazy: siapkan aksi a dari keadaan sekarang untuk step(a).

        tanpa_cek=True hanya untuk calon dari keadaan terbuka (aritmetika gerak saja);
        selain itu simulasi penuh dengan cek tabrakan. Kembalian (poses, biaya,
        menempel), atau None bila menabrak atau sel ujungnya sudah dikunjungi
        (cabang lain bisa memasukinya sejak calon dibuat).
        """
        pl = self.planner
        nd, nk, _ = aksi_info(pl, a)
        hasil = self._ekspansi(nd, nk, cek=False)
        px, py, pyaw = hasil[0][-1][:3]
        if (pl._key(px, py, pyaw), nd) in self.dikunjungi:
            return None
        if not tanpa_cek:
            hasil = self._ekspansi(nd, nk, cek=True)
            if hasil is None:
                return None
        self._calon[a] = hasil
        self._mask[a] = True
        return hasil

    # ---------------------------------------------------------- observasi
    def observasi(self):
        """Masukan jaringan: 47 angka (_obs_jauh).

        Dihitung SEKALI per keadaan. Pencarian perencana_dqfd memanggilnya dua
        kali untuk keadaan yang sama: step() mengembalikannya (dibuang
        pemanggil), lalu buka()/_urutan() memintanya lagi untuk forward
        jaringan. Cache dikosongkan oleh reset(), step(), dan pulihkan_keadaan().
        Kode yang MENGUBAH x/y/yaw/d/k langsung (mis. drl/visual_neuron.py)
        harus mengosongkan self._obs_c sendiri. Yang dikembalikan salinan:
        pemanggil boleh menyimpan larik itu (buffer latih).
        """
        if self._obs_c is None:
            self._obs_c = self._obs_jauh()
        return self._obs_c.copy()

    def _obs_jauh(self):
        """47 angka: tujuan, peta rintangan (sinar + heuristik Dijkstra h_D), margin."""
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
        o[42] = pl._h(x, y, gx, gy) / J                      # heuristik Dijkstra h_D
        o[43], o[44] = self._arah_heuristik()
        o[45] = self.margin_cm / konfig.SAFETY_MARGIN_CM
        m = self.omap.pose_margin(self.v, x, y, yaw)
        o[46] = max(-1.0, min(1.0, m / self._bebas_norm))
        return o

    def _sinar(self):
        """Jarak bebas dari tepi badan ke zona, 32 arah, 0..1 (1 = >= jangkauan). Loop numba."""
        dist = self.omap.dist
        off = self._pusat_off
        cx = self.x + off * math.cos(self.yaw)
        cy = self.y + off * math.sin(self.yaw)
        ang = self.yaw + self._sudut
        ux, uy = np.cos(ang), np.sin(ang)
        batas = self._tepi + self._jangkau
        t = _sinar_inti(dist, cx, cy, ux, uy, batas, _SINAR_ITERASI)
        return np.clip((np.minimum(t, batas) - self._tepi) / self._jangkau, 0.0, 1.0)

    def _arah_heuristik(self):
        """(cos, sin) arah turun heuristik Dijkstra h_D, dalam kerangka robot.

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
