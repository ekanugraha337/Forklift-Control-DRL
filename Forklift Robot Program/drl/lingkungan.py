"""
lingkungan.py - MDP perencanaan untuk DQfD, pengganti ruang pencarian Hybrid A*.

DUA MODEL PER ROBOT (versi aksi 4, branch ver-1-2). Demonstratornya Hybrid A* +
Reeds-Shepp; saat dipakai TIDAK ADA algoritma Reeds-Shepp maupun A* - seluruh
jalur dipilih dua jaringan, masing-masing meniru MASUKAN dan KELUARAN algoritma
yang ditirunya:

    jauh   meniru HYBRID A*
           masukan  pose awal & tujuan + PETA RINTANGAN: 47 angka (tujuan
                    relatif, 32 sinar, peta Dijkstra, margin, gerak terakhir)
           keluaran 14 primitif (7 tingkat kemudi x maju/mundur), 7,3 cm
    dekat  meniru REEDS-SHEPP
           masukan  HANYA pose tujuan relatif terhadap robot, dibagi radius
                    belok R (seperti reeds_shepp_paths), + arah & kemudi yang
                    sedang berjalan (seperti _rs_cost) + target pendekatan
                    lurus (_approach_dirs): 9 angka. TIDAK melihat rintangan -
                    sama dengan RS, yang tabrakannya dicek terpisah oleh A*.
                    Di sini tabrakan dicegah mask, dan pencarian mencoba aksi
                    berikutnya menurut Q (seperti _best_free_rs mencoba
                    kandidat dari yang termurah).
           keluaran 30 = segmen RS: {kanan penuh, lurus, kiri penuh} x {maju,
                    mundur} x panjang {1, 1/2, 1/4, 1/8, 1/16 langkah}.
                    Busur RS memakai kelengkungan yang sama persis dengan
                    kemudi penuh (radius = 1/kappa_max); panjang sisanya
                    dibulatkan ke 1/16 langkah (200/200 ekor RS mendarat
                    <= 1,0 cm & 1,2 deg).

SERAH (akhir episode jauh). Model jauh berhenti begitu titik acuan di dalam 60 cm
(analytic_dist, batas TERJAUH) DAN TIDAK ADA HALANGAN untuk mendarat. Selama
masih ada halangan, model jauh MELANJUTKAN - memperkecil jarak - sampai bebas,
lalu model dekat mengambil alih. "Tidak ada halangan" diperiksa dua tahap:
    1. saringan cepat (_boleh_coba_serah): peta Dijkstra tidak memaksa memutar
       (biaya Dijkstra = jarak oktil grid), atau sudah < RADIUS_SERAH_PAKSA_CM;
    2. cek_serah(env): jalur pendaratan benar-benar bebas.
         - saat LATIH (bawaan): tembakan Reeds-Shepp dari pose itu bebas -
           syarat yang SAMA PERSIS dengan saat Hybrid A* beralih ke RS. Jadi
           demo jauh berhenti tepat di titik demonstrator beralih, dan demo
           dekat 100% Reeds-Shepp. RS di sini peran demonstrator, bukan
           perencana yang dipakai robot.
         - saat DIPAKAI (perencana_dqfd): model DEKAT mencoba mendarat dari
           pose itu; berhasil tanpa tabrakan = tidak ada halangan.

Bagian A* direkonstruksi PERSIS: aritmetika ekspansi primitif penuh sama dengan
loop HybridAStarPlanner.plan() sampai bit terakhir. Diperiksa oleh
"python -m drl.adaptor_demo --uji". Pustaka geraknya objek HybridAStarPlanner
itu sendiri (kappas, step, n_sub, bobot biaya, heuristik, _analytic, _key);
hanya ekspansi satu primitif yang ditulis ulang (_ekspansi), karena di
forklift_planner.py ia berada di dalam loop plan() - dan berkas itu tidak diubah.

SATUAN. Seperti perencana: pose dan biaya dalam SEL grid, yaw radian. Hanya
observasi yang dinormalisasi.

MASK - aksi terlarang, Q-nya dibuat -inf sebelum argmax:
    aksi yang badannya menyentuh zona di salah satu sub-titiknya
        (aturan A*, termasuk kelonggaran "lepas diri" bila awalnya menempel);
    aksi panjang PENUH yang mendarat di sel yang sudah dikunjungi DENGAN ARAH
        GERAK YANG SAMA (planner._key + arah) - padanan closed set supaya
        rollout tidak berputar-putar. Arah ikut dihitung karena manuver
        maju-mundur RS memang melewati sel yang sama dua kali;
    aksi PENDEK (dekat) yang mendarat di sel HALUS yang sudah dikunjungi
        dengan arah yang sama - 1/16 sel A* (0,3 cm, 0,625 deg), lebih kecil
        dari gerak terpendek (0,46 cm > 0,3 x akar 2);
    aksi PENDEK setelah MAKS_PENDEK dipakai dalam satu episode.

BIAYA per langkah = rumus biaya ekspansi A* dengan panjang aksi itu (reward =
-biaya). Episode berakhir bila SERAH (jauh; reward - nilai_serah, biaya mendarat
model dekat), TIBA (dekat; <= TOL_DEKAT_CM & TOL_DEKAT_DEG), buntu (semua aksi
ter-mask, + BIAYA_BUNTU_CM), SERAH HABIS (jauh; cek_serah mengembalikan None =
jatah percobaan pendaratan habis, gagal, + BIAYA_BUNTU_CM), atau MAKS_LANGKAH
(terpotong, bukan gagal - untuk DQN itu bukan keadaan terminal).
"""
import math

import numpy as np

import konfig
import rencana as R

N_KEMUDI = 7
N_PRIMITIF = 2 * N_KEMUDI
JENIS = ("jauh", "dekat")
# Aksi model dekat = segmen Reeds-Shepp. Urutannya bagian dari kontrak model -
# jangan diubah tanpa menaikkan VERSI_AKSI.
PANJANG_DEKAT = (1.0, 0.5, 0.25, 0.125, 0.0625)
KEMUDI_DEKAT = (0, N_KEMUDI // 2, N_KEMUDI - 1)     # indeks planner.kappas: kanan penuh, lurus, kiri penuh
NAMA_KEMUDI_DEKAT = ("R", "S", "L")
PECAHAN_PENDEK = PANJANG_DEKAT[1:]
_N_AKSI = {"jauh": N_PRIMITIF, "dekat": len(PANJANG_DEKAT) * len(KEMUDI_DEKAT) * 2}
_N_OBS = {"jauh": 47, "dekat": 9}
N_SINAR = 32
# Dicatat di checkpoint (dqfd.meta_profil): model dengan ruang aksi lain ditolak.
# 1 = 14 primitif + SAMBUNG; 2 = satu model, 32 aksi (ver-1-1); 3 = dua model,
# jauh 14 / dekat 38 aksi, masukan sama (ver-1-2, awal); 4 = dua model meniru
# masukan & keluaran A* (47 / 14) dan RS (9 / 30).
VERSI_AKSI = 4
# Aksi pendek per episode. Satu segmen RS butuh <= 4 (1/2 + 1/4 + 1/8 + 1/16),
# ekor RS terpanjang yang terukur 6 segmen -> 24. Sisanya cadangan.
MAKS_PENDEK = 32
# Sel halus untuk aksi pendek, pecahan dari xy_reso / yaw_reso perencana.
HALUS = 16
# Toleransi TIBA model dekat (model jauh tidak tiba, ia menyerahkan).
TOL_DEKAT_CM = 1.0
TOL_DEKAT_DEG = 2.0
# Saringan serah: di dalam jarak ini (cm) peta Dijkstra tidak lagi disyaratkan
# bebas - untuk tujuan yang mepet rintangan. cek_serah tetap harus lolos.
RADIUS_SERAH_PAKSA_CM = 25.0

# Normalisasi observasi model jauh (cm). Arena 300 x 270 cm -> sekitar [-1, 1].
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


def _cek_jenis(jenis):
    if jenis not in JENIS:
        raise ValueError(f"jenis lingkungan '{jenis}' tidak dikenal, pilih {JENIS}")


def n_aksi(jenis):
    """Jumlah aksi (keluaran jaringan): jauh 14, dekat 30."""
    _cek_jenis(jenis)
    return _N_AKSI[jenis]


def n_obs(jenis):
    """Jumlah masukan jaringan: jauh 47, dekat 9."""
    _cek_jenis(jenis)
    return _N_OBS[jenis]


def aksi_info(planner, a, jenis):
    """Aksi -> (arah +1/-1, kappa 1/sel, pecahan langkah 0..1)."""
    if jenis == "jauh":
        return (1 if a < N_KEMUDI else -1), planner.kappas[a % N_KEMUDI], 1.0
    i_panjang, sisa = divmod(a, len(KEMUDI_DEKAT) * 2)
    i_kemudi, i_arah = divmod(sisa, 2)
    return ((1, -1)[i_arah], planner.kappas[KEMUDI_DEKAT[i_kemudi]], PANJANG_DEKAT[i_panjang])


def arah_kappa(planner, a, jenis):
    """Aksi -> (arah +1/-1, kappa dalam 1/sel). Panjangnya lihat aksi_info()."""
    d, k, _ = aksi_info(planner, a, jenis)
    return d, k


def aksi_dari(planner, arah, kappa, pecahan=1.0, jenis="jauh"):
    """Kebalikan aksi_info: kappa dibulatkan ke tingkat terdekat.

    jenis dekat hanya punya kemudi penuh dan lurus, dan panjang PANJANG_DEKAT;
    ValueError bila (kappa, pecahan) bukan segmen RS.
    """
    i = int(np.argmin([abs(kappa - k) for k in planner.kappas]))
    if jenis == "jauh":
        if pecahan != 1.0:
            raise ValueError("model jauh hanya punya primitif penuh")
        return i + (0 if arah > 0 else N_KEMUDI)
    if pecahan not in PANJANG_DEKAT or i not in KEMUDI_DEKAT:
        raise ValueError(f"bukan segmen Reeds-Shepp: {pecahan} langkah, kappa {kappa:.4g}")
    return PANJANG_DEKAT.index(pecahan) * len(KEMUDI_DEKAT) * 2 + KEMUDI_DEKAT.index(i) * 2 \
        + (0 if arah > 0 else 1)


def tabel_aksi(planner, jenis):
    """Keterangan tiap aksi: (nama, arah, kappa, sudut kemudi derajat, radius sel)."""
    baris = []
    for a in range(n_aksi(jenis)):
        d, k, f = aksi_info(planner, a, jenis)
        sudut = math.degrees(math.atan(k * planner.v.L))
        radius = abs(1.0 / k) if k else float("inf")
        if jenis == "dekat":
            nama = f"{NAMA_KEMUDI_DEKAT[KEMUDI_DEKAT.index(planner.kappas.index(k))]}{'+' if d > 0 else '-'} x{f:g}"
        else:
            nama = f"{'maju' if d > 0 else 'mundur'} {sudut:+.1f}"
        baris.append((nama, d, k, sudut, radius))
    return baris


class LingkunganRencana:
    """Satu arena, satu robot, satu margin. reset() untuk tiap pasangan awal-tujuan.

    planner: HybridAStarPlanner yang dijadikan pustaka gerak. None = dibangun
    dari konfig untuk profil & margin ini, dengan fungsi yang sama yang dipakai
    Perencana (rencana.buat_kendaraan / buat_perencana).
    jenis: 'jauh' atau 'dekat', lihat docstring modul.
    nilai_serah: hanya jauh - fungsi(env) -> biaya (sel) mendarat dari pose
    serah, ditambahkan ke biaya langkah terakhir. None = 0.
    cek_serah: hanya jauh - fungsi(env) -> bool, "jalur pendaratan bebas" di
    pose sekarang. None = tembakan Reeds-Shepp bebas (peran demonstrator,
    dipakai saat membuat dataset). perencana_dqfd memasang model dekat di sini
    (rencanakan_dua, buat_cek_pendaratan). Fungsi boleh mengembalikan None:
    jatah percobaan habis, episode berakhir gagal. env.n_coba_serah = jumlah
    pemanggilan dalam episode ini.
    """

    def __init__(self, arena, profil=None, margin_cm=None, planner=None, jenis="dekat",
                 nilai_serah=None, cek_serah=None):
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
        self.jenis = jenis
        self.n_aksi = n_aksi(jenis)
        self.n_obs = n_obs(jenis)
        self.nilai_serah = nilai_serah
        self.cek_serah = cek_serah

        s = arena.panjang_ke_sel
        self._jarak_norm = s(JARAK_NORM_CM)
        self.radius_serah = planner.analytic_dist
        self.radius_paksa = s(RADIUS_SERAH_PAKSA_CM) if RADIUS_SERAH_PAKSA_CM is not None else 0.0
        self.tol_pos = s(TOL_DEKAT_CM)
        self.tol_yaw = math.radians(TOL_DEKAT_DEG)
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
    def reset(self, awal, tujuan, omap=None, d0=0, k0=0.0):
        """awal = (gx, gy, yaw); tujuan = (gx, gy, yaw atau None). Dalam SEL.

        omap: peta dengan rintangan tambahan (Perencana.peta_dengan), None = arena.
        d0, k0: arah gerak dan kelengkungan yang sedang berjalan - untuk model
        dekat yang melanjutkan model jauh di pose serah (biaya ganti arah /
        ganti kemudi langkah pertama ikut benar). 0 = berangkat dari diam.
        Kembalian (obs, info). obs None bila pasangan ini tidak sah - dengan
        pemeriksaan yang sama dengan awal plan(). Jenis jauh yang sudah boleh
        serah di pose awal: alasan "sudah di radius dekat".
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
        self.biaya_serah = 0.0
        self.n_coba_serah = 0
        self.langkah_ke = 0
        self.selesai = True
        self.sukses = False

        if gyaw is not None and not self.omap.pose_free(v, gx, gy, gyaw):
            return None, {"sah": False, "alasan": "tujuan menabrak rintangan"}
        if gyaw is None and self.omap.clearance(gx, gy) < v.half_width:
            return None, {"sah": False, "alasan": "tujuan terlalu dekat rintangan"}
        if self.jenis == "dekat" and self._tiba(sx, sy, syaw):
            return None, {"sah": False, "alasan": "sudah di tujuan"}

        # Keadaan bersama di objek perencana, diisi persis seperti plan().
        pl._hcost = pl._holonomic_heuristic(self.omap, gx, gy)
        pl._approach_dirs = pl._feasible_approach_dirs(self.omap, self.tujuan)
        self._hgrid = np.array(pl._hcost, dtype=np.float64)

        self.x, self.y, self.yaw = sx, sy, syaw
        self.d, self.k = d0, k0
        self.in_col = self.omap.pose_margin(v, sx, sy, syaw) <= 0
        if self.jenis == "jauh" and self._boleh_serah():
            return None, {"sah": False, "alasan": "sudah di radius dekat"}
        # Sel awal tertutup untuk kedua arah: kembali ke pose awal tidak berguna.
        kunci = pl._key(sx, sy, syaw)
        self.dikunjungi = {(kunci, 1), (kunci, -1)}
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
        nd, nk, pecahan = aksi_info(pl, a, self.jenis)
        self.x, self.y, self.yaw = poses[-1][0], poses[-1][1], poses[-1][2]
        self.d, self.k, self.in_col = nd, nk, col
        if pecahan < 1.0:
            self.n_pendek += 1
            self.dikunjungi_halus.add((self._key_halus(self.x, self.y, self.yaw), nd))
        else:
            self.dikunjungi.add((pl._key(self.x, self.y, self.yaw), nd))
        self._pose.extend(poses)
        self.biaya += biaya
        self.langkah_ke += 1
        reward = -biaya

        if self.jenis == "dekat" and self._tiba(self.x, self.y, self.yaw):
            self.selesai = self.sukses = True
            info["alasan"] = "tiba"
        elif self.jenis == "jauh":
            serah = self._boleh_serah()
            if serah:
                self.selesai = self.sukses = True
                info["alasan"] = "serah"
                if self.nilai_serah is not None:
                    self.biaya_serah = float(self.nilai_serah(self))
                    reward -= self.biaya_serah
            elif serah is None:
                self.selesai = True
                reward -= self.biaya_buntu
                info["alasan"] = "serah habis"

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

    # -------------------------------------------------------------- serah
    def _di_radius(self, x, y):
        """Sudah di radius serah? Sama dengan syarat Reeds-Shepp Hybrid A* (titik acuan, '<')."""
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
        """Tembakan Reeds-Shepp dari pose sekarang bebas tabrakan - syarat Hybrid A*
        beralih ke RS (plan(): titik acuan < analytic_dist, badan tidak menempel,
        _analytic tidak None). Peran demonstrator: dipakai saat LATIH saja."""
        if self.in_col:
            return False
        return self.planner._analytic(self.omap, (self.x, self.y, self.yaw, self.d, self.k),
                                      self.tujuan) is not None

    def _boleh_serah(self):
        """Model jauh berhenti di sini? Saringan cepat + jalur pendaratan bebas.
        True / False, atau None dari cek_serah (jatah habis: berhenti, gagal)."""
        if not self._boleh_coba_serah(self.x, self.y):
            return False
        if self.cek_serah is None:
            return self.rs_bebas()
        self.n_coba_serah += 1
        return self.cek_serah(self)

    # -------------------------------------------------------------- calon
    def _hitung_calon(self):
        """Hitung semua aksi dari keadaan sekarang: hasil + mask. Dipakai ulang step()."""
        pl = self.planner
        self._calon = [None] * self.n_aksi
        self._mask = np.zeros(self.n_aksi, dtype=bool)
        pendek_habis = self.n_pendek >= MAKS_PENDEK
        for a in range(self.n_aksi):
            nd, nk, pecahan = aksi_info(pl, a, self.jenis)
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

    def _tiba(self, x, y, yaw):
        """Tiba versi model dekat: TOL_DEKAT_CM dan TOL_DEKAT_DEG (arah bebas: posisi saja)."""
        gx, gy, gyaw = self.tujuan
        if math.hypot(gx - x, gy - y) > self.tol_pos:
            return False
        return gyaw is None or abs(math.remainder(yaw - gyaw, 2.0 * math.pi)) <= self.tol_yaw

    def _key_halus(self, x, y, yaw):
        """Sel halus (1/HALUS sel A*) untuk aturan sel tertutup aksi pendek."""
        pl = self.planner
        r, ry = pl.xy_reso / HALUS, pl.yaw_reso / HALUS
        return int(x // r), int(y // r), int((yaw % (2.0 * math.pi)) // ry)

    def _ekspansi(self, nd, nk, pecahan=1.0):
        """Satu aksi dari keadaan sekarang -> (poses, biaya, masih_menempel) atau None.

        Untuk primitif PENUH ini SALINAN aritmetika loop ekspansi
        HybridAStarPlanner.plan(), dengan urutan operasi yang sama - supaya
        jalur dan biaya sama sampai bit terakhir, bukan hanya mirip. Aksi
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
        """Masukan jaringan: jauh 47 angka (_obs_jauh), dekat 9 angka (_obs_dekat)."""
        return self._obs_jauh() if self.jenis == "jauh" else self._obs_dekat()

    def _obs_dekat(self):
        """9 angka = masukan Reeds-Shepp. Tanpa sinar, tanpa peta.

            0-1  tujuan relatif (maju, samping) / R   R = radius belok rencana (1/kappa_max)
            2-3  selisih arah hadap tujuan (cos, sin)  - (1, 0) bila arah bebas
            4    penanda arah tujuan bebas
            5-6  pendekatan lurus muat dari depan / dari belakang (_approach_dirs)
            7    arah gerak yang sedang berjalan       +1 / -1 / 0
            8    kelengkungan yang sedang berjalan     kappa / kappa_max
        Persis yang dipakai A* untuk menyusun dan menilai tembakan RS:
        reeds_shepp_paths((x, y, yaw), target, radius), target dari
        _approach_dirs, _rs_cost(segmen, d, k).
        """
        o = np.zeros(_N_OBS["dekat"], dtype=np.float32)
        pl = self.planner
        gx, gy, gyaw = self.tujuan
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = gx - self.x, gy - self.y
        R = pl.radius
        o[0] = (c * dx + s * dy) / R
        o[1] = (-s * dx + c * dy) / R
        if gyaw is None:
            o[2], o[3], o[4] = 1.0, 0.0, 1.0
        else:
            e = gyaw - self.yaw
            o[2], o[3] = math.cos(e), math.sin(e)
        o[5] = 1.0 in pl._approach_dirs
        o[6] = -1.0 in pl._approach_dirs
        o[7] = self.d
        o[8] = self.k / pl.kappa_max
        return o

    def _obs_jauh(self):
        """47 angka = masukan Hybrid A*: tujuan, peta rintangan (sinar + Dijkstra), margin."""
        o = np.zeros(_N_OBS["jauh"], dtype=np.float32)
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
