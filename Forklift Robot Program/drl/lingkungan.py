"""
lingkungan.py - MDP perencanaan untuk DQfD, pengganti ruang pencarian Hybrid A*.

DUA MODEL PER ROBOT (versi aksi 6, branch ver-1-4). Demonstratornya Hybrid A* +
Reeds-Shepp. Model jauh meniru masukan & keluaran Hybrid A*. Model dekat
MEMILIH KATA Reeds-Shepp: jaringan mengurutkan kandidat kurva RS, panjang
segmennya dihitung rumus RS, lalu kandidat dicek tabrakan berurutan menurut Q:

    jauh   meniru HYBRID A*
           masukan  pose awal & tujuan + PETA RINTANGAN: 47 angka (tujuan
                    relatif, 32 sinar, peta Dijkstra, margin, gerak terakhir)
           keluaran 14 primitif (7 tingkat kemudi x maju/mundur), 7,3 cm
    dekat  MEMILIH KATA REEDS-SHEPP (ver-1-4)
           masukan  9 angka = masukan rumus RS: pose tujuan relatif dibagi
                    radius belok R (seperti reeds_shepp_paths), arah & kemudi
                    yang sedang berjalan (seperti _rs_cost), titik sambung
                    pendekatan lurus (_approach_dirs). Tidak melihat rintangan.
           keluaran 88 = 44 jenis kata RS (11 keluarga x 4 varian: asli, balik
                    waktu, cermin, keduanya - urutan _rs_words) x 2 titik
                    sambung (slot 0: masuk maju, atau langsung ke tujuan bila
                    pendekatan lurus tidak muat; slot 1: masuk mundur. Tujuan
                    tanpa arah: slot 0/1 = menghadap tujuan / membelakanginya).
                    Mask = kata itu punya solusi untuk pose ini. Panjang
                    segmennya dari rumus RS, jadi jalurnya EKSAK.
           satu episode = SATU aksi: kurva dicek tabrakan (badan + margin di
                    tiap sampel, seperti _best_free_rs); bebas = TIBA tepat di
                    tujuan, menabrak = gagal (+ BIAYA_BUNTU_CM). Pencarian
                    mundur (perencana_dqfd) lalu mencoba kata berikutnya
                    menurut Q - semua kandidat sah, sampai ada yang bebas.
           demonstrator: aksi_demonstrator() = PERSIS _best_free_rs (urut
                    biaya rumus, cek <= 6, ambil yang bebas termurah).

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
    model dekat: kata RS yang tidak punya solusi untuk pose ini (tabrakan
        tidak masuk mask - dicek saat kata dijalankan).

BIAYA per langkah = rumus biaya ekspansi A* dengan panjang aksi itu (reward =
-biaya; kata RS: biaya rumus _rs_cost + penalti jarak bebas, sama dengan A*).
Episode berakhir bila SERAH (jauh; reward - nilai_serah, biaya mendarat model
dekat), TIBA (dekat; kata RS bebas tabrakan), TABRAKAN (dekat; + BIAYA_BUNTU_CM), buntu (semua aksi
ter-mask, + BIAYA_BUNTU_CM), SERAH HABIS (jauh; cek_serah mengembalikan None =
jatah percobaan pendaratan habis, gagal, + BIAYA_BUNTU_CM), atau MAKS_LANGKAH
(terpotong, bukan gagal - untuk DQN itu bukan keadaan terminal).
"""
import math

import numpy as np

import forklift_planner as FP
import konfig
import rencana as R

N_KEMUDI = 7
N_PRIMITIF = 2 * N_KEMUDI
JENIS = ("jauh", "dekat")
# Aksi model dekat = KATA Reeds-Shepp. Keluarga dan variannya PERSIS urutan
# forklift_planner._rs_words (diuji sama dengan reeds_shepp_paths). Urutannya
# bagian dari kontrak model - jangan diubah tanpa menaikkan VERSI_AKSI.
_HP = 0.5 * math.pi
RS_KELUARGA = (   # (nama, fungsi, kata, susun panjang (t, u, v), pakai (xb, yb))
    ("CSC", FP._LpSpLp, ("L", "S", "L"), lambda t, u, v: (t, u, v), False),
    ("CSC", FP._LpSpRp, ("L", "S", "R"), lambda t, u, v: (t, u, v), False),
    ("CCC", FP._LpRmL, ("L", "R", "L"), lambda t, u, v: (t, u, v), False),
    ("CCC balik", FP._LpRmL, ("L", "R", "L"), lambda t, u, v: (v, u, t), True),
    ("CCCC", FP._LpRupLumRm, ("L", "R", "L", "R"), lambda t, u, v: (t, u, -u, v), False),
    ("CCCC", FP._LpRumLumRp, ("L", "R", "L", "R"), lambda t, u, v: (t, u, u, v), False),
    ("CCSC", FP._LpRmSmLm, ("L", "R", "S", "L"), lambda t, u, v: (t, -_HP, u, v), False),
    ("CCSC", FP._LpRmSmRm, ("L", "R", "S", "R"), lambda t, u, v: (t, -_HP, u, v), False),
    ("CSCC", FP._LpRmSmLm, ("L", "S", "R", "L"), lambda t, u, v: (v, u, -_HP, t), True),
    ("CSCC", FP._LpRmSmRm, ("R", "S", "R", "L"), lambda t, u, v: (v, u, -_HP, t), True),
    ("CCSCC", FP._LpRmSLmRp, ("L", "R", "S", "L", "R"), lambda t, u, v: (t, -_HP, u, -_HP, v), False),
)
# (tanda x, tanda y, tanda phi, tanda panjang, cermin L<->R): asli, balik waktu, cermin, keduanya
_RS_VARIAN = ((1, 1, 1, 1, False), (-1, 1, -1, -1, False), (1, -1, -1, 1, True), (-1, -1, 1, -1, True))
NAMA_VARIAN = ("asli", "balik waktu", "cermin", "balik waktu + cermin")
N_KATA_RS = len(RS_KELUARGA) * len(_RS_VARIAN)          # 44
SLOT_SAMBUNG = ("masuk maju", "masuk mundur")
_N_AKSI = {"jauh": N_PRIMITIF, "dekat": N_KATA_RS * len(SLOT_SAMBUNG)}
_N_OBS = {"jauh": 47, "dekat": 9}
N_SINAR = 32
# Dicatat di checkpoint (dqfd.meta_profil): model dengan ruang aksi lain ditolak.
# 1 = 14 primitif + SAMBUNG; 2 = satu model, 32 aksi (ver-1-1); 3 = dua model,
# jauh 14 / dekat 38 aksi, masukan sama (ver-1-2, awal); 4 = dua model meniru
# masukan & keluaran A* (47 / 14) dan RS (9 / 30); 5 = model dekat sadar
# rintangan, 47 masukan (9 RS + 32 sinar + 6 ruang manuver) / 30 aksi (ver-1-3);
# 6 = model dekat memilih kata RS: 9 masukan / 88 aksi (ver-1-4).
VERSI_AKSI = 6
# Versi terakhir saat kontrak masukan/keluaran TIAP JENIS model berubah. Model
# jenis itu dengan VERSI_MODEL[jenis] <= versi_aksi <= VERSI_AKSI masih cocok:
# versi 5 dan 6 hanya mengubah model dekat, jadi model jauh versi 4 tetap bisa
# dipakai (masukan 47 dan 14 aksinya identik, demo jauhnya sama persis).
VERSI_MODEL = {"jauh": 4, "dekat": 6}


def versi_cocok(jenis, versi):
    """Model jenis ini dengan versi_aksi itu cocok dengan program sekarang?"""
    return jenis in VERSI_MODEL and VERSI_MODEL[jenis] <= versi <= VERSI_AKSI


# Toleransi tiba (untuk pemeriksaan jalur; kata RS sendiri tiba eksak).
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


def _sinar_loop_numpy(dist, cx, cy, ux, uy, batas, iterasi):
    """Sphere tracing semua sinar sekaligus dengan numpy: jarak tempuh t tiap sinar.

    Versi asli (sampai ver-1-4). Dipakai bila numba tidak tersedia, dan sebagai
    acuan verifikasi _sinar_loop. Lambat bukan karena hitungannya: tiap iterasi
    ~28 operasi numpy pada larik 32 elemen, masing-masing ~0,3-0,5 us overhead
    pemanggilan, dan sinar yang sudah berhenti tetap ikut dihitung sampai sinar
    TERPANJANG selesai.
    """
    h, w = dist.shape
    t = np.zeros(ux.shape[0])
    aktif = np.ones(ux.shape[0], dtype=bool)
    for _ in range(iterasi):
        ix = np.floor(cx + t * ux).astype(np.int64)
        iy = np.floor(cy + t * uy).astype(np.int64)
        dalam = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        d = np.where(dalam, dist[np.clip(iy, 0, h - 1), np.clip(ix, 0, w - 1)], 0.0)
        aktif &= (d >= 1.0) & (t < batas)
        if not aktif.any():
            break
        t = np.where(aktif, t + np.maximum(d - 0.5, 0.5), t)
    return t


def _sinar_loop(dist, cx, cy, ux, uy, batas, iterasi):
    """Sama dengan _sinar_loop_numpy, satu sinar demi satu sinar (dikompilasi numba).

    Hasilnya IDENTIK bit demi bit, karena operasinya sama persis dan urutannya
    sama: t*u lalu + c, floor, perbandingan, d - 0,5 (dist float32: selisih
    dengan 0,5 eksak untuk jarak < 2^22 sel), lalu + ke t (float64). cos/sin
    TIDAK dihitung di sini - tetap numpy di _sinar - karena libm numba bisa
    berbeda satu ulp dari numpy, dan floor() akan memperbesar beda itu menjadi
    sel yang berbeda. Sinar yang berhenti langsung selesai, tidak menunggu
    sinar terpanjang seperti versi numpy.
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


# numba (ver-1-5): _sinar_loop dikompilasi ke kode mesin - sinar adalah porsi
# terbesar waktu model jauh. Tanpa numba (atau bila kompilasi gagal) _sinar
# kembali ke _sinar_loop_numpy dengan hasil yang sama, hanya lebih lambat.
# cache=True: hasil kompilasi disimpan di __pycache__, jadi kompilasi (~1-2 s)
# hanya sekali per mesin; nogil=True: thread perencana tidak menahan GIL selama
# sinar dihitung, sehingga tidak berebut dengan loop kamera/kendali navigasi.py.
# Dikompilasi saat impor (pemanasan di bawah), bukan saat rencana pertama robot.
try:
    from numba import njit as _njit
    _sinar_inti = _njit(cache=True, nogil=True)(_sinar_loop)
    _sinar_inti(np.ones((2, 2), np.float32), 0.5, 0.5, np.ones(1), np.ones(1), np.ones(1), 1)
except Exception as _e:                      # numba tidak ada / gagal kompilasi
    if not isinstance(_e, ImportError):
        print(f"[!] lingkungan.py: numba gagal ({type(_e).__name__}: {_e}), sinar memakai numpy")
    _sinar_inti = None
SINAR_NUMBA = _sinar_inti is not None

_TETANGGA = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))


def _cek_jenis(jenis):
    if jenis not in JENIS:
        raise ValueError(f"jenis lingkungan '{jenis}' tidak dikenal, pilih {JENIS}")


def n_aksi(jenis):
    """Jumlah aksi (keluaran jaringan): jauh 14, dekat 88 (44 kata RS x 2 titik sambung)."""
    _cek_jenis(jenis)
    return _N_AKSI[jenis]


def n_obs(jenis):
    """Jumlah masukan jaringan: jauh 47, dekat 9 (masukan rumus RS)."""
    _cek_jenis(jenis)
    return _N_OBS[jenis]


def aksi_info(planner, a, jenis):
    """Aksi model jauh -> (arah +1/-1, kappa 1/sel, pecahan langkah = 1)."""
    if jenis != "jauh":
        raise ValueError("aksi model dekat versi 6 adalah kata RS - lihat kata_rs()")
    return (1 if a < N_KEMUDI else -1), planner.kappas[a % N_KEMUDI], 1.0


def kata_rs(a):
    """Aksi model dekat -> (indeks keluarga, indeks varian, slot sambung)."""
    slot, k = divmod(int(a), N_KATA_RS)
    i, j = divmod(k, len(_RS_VARIAN))
    return i, j, slot


def nama_kata(a):
    """Nama pendek aksi model dekat, mis. 'LSL #0 maju' (huruf setelah cermin)."""
    i, j, slot = kata_rs(a)
    kata = RS_KELUARGA[i][2]
    if _RS_VARIAN[j][4]:
        kata = FP._swap_lr(kata)
    return f"{''.join(kata)}{'~' if _RS_VARIAN[j][0] < 0 else ''} #{i * len(_RS_VARIAN) + j} {('maju', 'mundur')[slot]}"


def kata_rs_semua(start, goal, radius):
    """44 solusi kata RS dari start ke goal (sel, rad), urutan tetap kata_rs().

    Isinya [(jenis, panjang bertanda sel), ...] atau None bila kata itu tidak
    punya solusi. Sama dengan forklift_planner.reeds_shepp_paths - termasuk
    pemeriksaan integrasi numerik - hanya indeksnya tetap.
    """
    sx, sy, syaw = start
    gx, gy, gyaw = goal
    dx, dy = gx - sx, gy - sy
    c, s = math.cos(syaw), math.sin(syaw)
    x, y = (c * dx + s * dy) / radius, (-s * dx + c * dy) / radius
    phi = FP.wrap_angle(gyaw - syaw)
    xb = x * math.cos(phi) + y * math.sin(phi)
    yb = x * math.sin(phi) - y * math.cos(phi)
    out = [None] * N_KATA_RS
    for i, (_, fn, kata, susun, balik) in enumerate(RS_KELUARGA):
        xx, yy = (xb, yb) if balik else (x, y)
        for j, (tx, ty, tphi, tanda, cermin) in enumerate(_RS_VARIAN):
            r = fn(tx * xx, ty * yy, tphi * phi)
            if r is None:
                continue
            panjang = [tanda * v for v in susun(*r)]
            w = FP._swap_lr(kata) if cermin else kata
            segs = [(k, v) for k, v in zip(w, panjang) if abs(v) > 1e-9]
            px = py = pyaw = 0.0
            for k, v in segs:
                px, py, pyaw = FP._integrate_segment(px, py, pyaw, k, v, 1.0)
            if abs(px - x) > 1e-4 or abs(py - y) > 1e-4 or abs(FP.wrap_angle(pyaw - phi)) > 1e-4:
                continue
            out[i * len(_RS_VARIAN) + j] = [(k, v * radius) for k, v in segs]
    return out


def arah_kappa(planner, a, jenis):
    """Aksi -> (arah +1/-1, kappa dalam 1/sel). Panjangnya lihat aksi_info()."""
    d, k, _ = aksi_info(planner, a, jenis)
    return d, k


def aksi_dari(planner, arah, kappa, pecahan=1.0, jenis="jauh"):
    """Kebalikan aksi_info (model jauh): kappa dibulatkan ke tingkat terdekat."""
    if jenis != "jauh" or pecahan != 1.0:
        raise ValueError("hanya model jauh (primitif penuh) - model dekat memakai kata_rs()")
    i = int(np.argmin([abs(kappa - k) for k in planner.kappas]))
    return i + (0 if arah > 0 else N_KEMUDI)


def tabel_aksi(planner, jenis):
    """Keterangan tiap aksi: (nama, arah, kappa, sudut kemudi derajat, radius sel).
    Model dekat: (nama kata RS, 0, 0, 0, inf) - arah & kemudi berubah per segmen."""
    if jenis == "dekat":
        return [(nama_kata(a), 0, 0.0, 0.0, float("inf")) for a in range(n_aksi(jenis))]
    baris = []
    for a in range(n_aksi(jenis)):
        d, k, f = aksi_info(planner, a, jenis)
        sudut = math.degrees(math.atan(k * planner.v.L))
        radius = abs(1.0 / k) if k else float("inf")
        baris.append((f"{'maju' if d > 0 else 'mundur'} {sudut:+.1f}", d, k, sudut, radius))
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
        self.pose_tabrak = None
        self.selesai = True
        self._obs_c = None          # cache observasi(), lihat di sana

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
        self._obs_c = None
        self.omap = self.arena.omap if omap is None else omap
        pl, v = self.planner, self.v
        sx, sy, syaw = awal
        gx, gy, gyaw = tujuan
        self.awal = (sx, sy, syaw)
        self.tujuan = (gx, gy, gyaw)
        self._pose = []
        self.pose_tabrak = None
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

        # Keadaan bersama di objek perencana, diisi persis seperti plan(). Model
        # dekat tidak memakai heuristik Dijkstra (rumus RS tidak memakainya).
        if self.jenis == "jauh":
            pl._hcost = pl._holonomic_heuristic(self.omap, gx, gy)
            self._hgrid = np.array(pl._hcost, dtype=np.float64)
        pl._approach_dirs = pl._feasible_approach_dirs(self.omap, self.tujuan)

        self.x, self.y, self.yaw = sx, sy, syaw
        self.d, self.k = d0, k0
        self.in_col = self.omap.pose_margin(v, sx, sy, syaw) <= 0
        if self.jenis == "jauh" and self._boleh_serah():
            return None, {"sah": False, "alasan": "sudah di radius dekat"}
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

    def step(self, a):
        """Kembalian (obs, reward, selesai, info). reward = -biaya langkah ini."""
        if self.selesai:
            raise RuntimeError("episode sudah selesai - panggil reset()")
        a = int(a)
        if not self._mask[a]:
            raise ValueError(f"aksi {a} ter-mask")
        self._obs_c = None
        if self.jenis == "dekat":
            return self._step_rs(a)

        pl = self.planner
        info = {"alasan": "", "terpotong": False}
        poses, biaya, col = self._calon[a]
        nd, nk, _ = aksi_info(pl, a, self.jenis)
        self.x, self.y, self.yaw = poses[-1][0], poses[-1][1], poses[-1][2]
        self.d, self.k, self.in_col = nd, nk, col
        self.dikunjungi.add((pl._key(self.x, self.y, self.yaw), nd))
        self._pose.extend(poses)
        self.biaya += biaya
        self.langkah_ke += 1
        reward = -biaya

        if self.jenis == "jauh":
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
                len(self._pose), self.biaya, self.langkah_ke)

    def pulihkan_keadaan(self, s):
        self._obs_c = None
        (self.x, self.y, self.yaw, self.d, self.k, self.in_col,
         n, self.biaya, self.langkah_ke) = s
        del self._pose[n:]
        self.selesai = self.sukses = False
        self.pose_tabrak = None
        if self.jenis == "jauh":
            self._hitung_calon()      # mask baru: sel yang dikunjungi sejak itu ikut tertutup
        # model dekat: episodenya satu langkah, keadaan yang dipulihkan selalu
        # pose awal - kandidat kata RS-nya tidak berubah

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
        if self.jenis == "dekat":
            return self._hitung_calon_rs()
        pl = self.planner
        self._calon = [None] * self.n_aksi
        self._mask = np.zeros(self.n_aksi, dtype=bool)
        for a in range(self.n_aksi):
            nd, nk, _ = aksi_info(pl, a, self.jenis)
            if nd < 0 and not self.boleh_mundur:
                continue
            hasil = self._ekspansi(nd, nk)
            if hasil is None:
                continue
            px, py, pyaw = hasil[0][-1][:3]
            if (pl._key(px, py, pyaw), nd) in self.dikunjungi:
                continue
            self._calon[a] = hasil
            self._mask[a] = True

    # ------------------------------------------------- model dekat: kata RS
    def _target_rs(self):
        """{slot: (pose titik sambung, segmen lurus penutup atau None)} - sama dengan _analytic."""
        pl = self.planner
        gx, gy, gyaw = self.tujuan
        if gyaw is None:
            b = math.atan2(gy - self.y, gx - self.x)
            return {0: ((gx, gy, b), None), 1: ((gx, gy, b + math.pi), None)}
        if pl._approach_dirs:
            c, s, la = math.cos(gyaw), math.sin(gyaw), pl.approach_len
            return {(0 if ad > 0 else 1): ((gx - ad * la * c, gy - ad * la * s, gyaw), ("S", ad * la))
                    for ad in pl._approach_dirs}
        return {0: ((gx, gy, gyaw), None)}

    def _hitung_calon_rs(self):
        """88 aksi = kata RS x slot sambung: (segmen, biaya rumus _rs_cost), mask = punya solusi.

        Tabrakan sengaja TIDAK masuk mask: kurva dicek saat dijalankan (step),
        seperti _best_free_rs mengecek kandidat satu per satu.
        """
        pl = self.planner
        self._calon = [None] * self.n_aksi
        self._mask = np.zeros(self.n_aksi, dtype=bool)
        for slot, (pose, tutup) in self._target_rs().items():
            for k, segs in enumerate(kata_rs_semua((self.x, self.y, self.yaw), pose, pl.radius)):
                if segs is None:
                    continue
                full = segs + ([tutup] if tutup is not None else [])
                if not self.boleh_mundur and any(v < 0 for _, v in full):
                    continue
                a = slot * N_KATA_RS + k
                self._calon[a] = (full, pl._rs_cost(full, self.d, self.k))
                self._mask[a] = True

    def _cek_kurva(self, segs, biaya):
        """Sampel kurva RS + cek badan di tiap sampel. (poses, biaya + penalti, indeks tabrak atau None)."""
        pl, v, om = self.planner, self.v, self.omap
        poses = FP.sample_rs_path((self.x, self.y, self.yaw), segs, pl.radius, pl.sample_ds)
        for i, p in enumerate(poses):
            m = om.pose_margin(v, p[0], p[1], p[2])
            if m <= 0:
                return poses, biaya, i
            biaya += pl._clearance_penalty(m, pl.sample_ds)
        return poses, biaya, None

    def _step_rs(self, a):
        """Jalankan satu kata RS utuh: bebas = TIBA tepat di tujuan, menabrak = gagal."""
        pl = self.planner
        segs, biaya0 = self._calon[a]
        poses, biaya, tabrak = self._cek_kurva(segs, biaya0)
        self.langkah_ke += 1
        self.selesai = True
        info = {"alasan": "", "terpotong": False}
        if tabrak is None:
            self._pose.extend(poses)
            p = poses[-1]
            self.x, self.y, self.yaw, self.d = p[0], p[1], p[2], p[3]
            akhir = segs[-1][0]
            self.k = 0.0 if akhir == "S" else (pl.kappa_max if akhir == "L" else -pl.kappa_max)
            self.biaya += biaya
            self.sukses = True
            reward = -biaya
            info["alasan"] = "tiba"
        else:
            self.pose_tabrak = poses[:tabrak + 1]
            reward = -self.biaya_buntu
            info["alasan"] = "tabrakan"
        info["sukses"] = self.sukses
        info["biaya"] = self.biaya
        return self.observasi(), reward, True, info

    def aksi_demonstrator(self):
        """Pilihan Reeds-Shepp di pose ini - PERSIS _best_free_rs: kandidat diurutkan
        menurut biaya rumus, paling banyak 6 dicek tabrakan, berhenti begitu kandidat
        berikutnya tidak lebih murah dari yang sudah bebas. None = tidak ada yang bebas."""
        sah = sorted(np.flatnonzero(self._mask), key=lambda a: self._calon[a][1])
        best = None
        for a in sah[:6]:
            segs, biaya = self._calon[a]
            if best is not None and biaya >= best[1]:
                break
            _, total, tabrak = self._cek_kurva(segs, biaya)
            if tabrak is None and (best is None or total < best[1]):
                best = (int(a), total)
        return None if best is None else best[0]

    def _tiba(self, x, y, yaw):
        """Tiba versi model dekat: TOL_DEKAT_CM dan TOL_DEKAT_DEG (arah bebas: posisi saja)."""
        gx, gy, gyaw = self.tujuan
        if math.hypot(gx - x, gy - y) > self.tol_pos:
            return False
        return gyaw is None or abs(math.remainder(yaw - gyaw, 2.0 * math.pi)) <= self.tol_yaw

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
        """Masukan jaringan: jauh 47 angka (_obs_jauh), dekat 9 angka (_obs_dekat).

        Dihitung SEKALI per keadaan (2026-11-04). Pencarian perencana_dqfd
        memanggilnya dua kali untuk keadaan yang sama: step() mengembalikannya
        (dibuang pemanggil), lalu buka()/_urutan() memintanya lagi untuk forward
        jaringan. Model jauh menghabiskan ~62% waktunya di 32 sinar, jadi
        hitungan kedua itu ~1/3 waktu DQfD (Gudang: 27,3 -> 18,2 ms median,
        jalur identik). Cache dikosongkan oleh reset(), step(), dan
        pulihkan_keadaan(). Kode yang MENGUBAH x/y/yaw/d/k langsung (mis.
        drl/visual_neuron.py) harus mengosongkan self._obs_c sendiri.
        Yang dikembalikan salinan: pemanggil boleh menyimpan larik itu (buffer latih).
        """
        if self._obs_c is None:
            self._obs_c = self._obs_jauh() if self.jenis == "jauh" else self._obs_dekat()
        return self._obs_c.copy()

    def _obs_dekat(self):
        """9 angka = masukan rumus Reeds-Shepp. Tanpa sinar, tanpa peta.

            0-1  tujuan relatif (maju, samping) / R   R = radius belok rencana (1/kappa_max)
            2-3  selisih arah hadap tujuan (cos, sin)  - (1, 0) bila arah bebas
            4    penanda arah tujuan bebas
            5-6  pendekatan lurus muat dari depan / dari belakang (_approach_dirs)
            7    arah gerak yang sedang berjalan       +1 / -1 / 0
            8    kelengkungan yang sedang berjalan     kappa / kappa_max
        Persis yang dipakai A* untuk menyusun dan menilai tembakan RS:
        reeds_shepp_paths((x, y, yaw), target, radius), target dari
        _approach_dirs, _rs_cost(segmen, d, k). Tabrakan dicek terpisah.
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
        itu tanpa melewatkan rintangan. Loopnya _sinar_loop (numba) atau
        _sinar_loop_numpy (tanpa numba); hasilnya identik.
        """
        dist = self.omap.dist
        off = self._pusat_off
        cx = self.x + off * math.cos(self.yaw)
        cy = self.y + off * math.sin(self.yaw)
        ang = self.yaw + self._sudut
        ux, uy = np.cos(ang), np.sin(ang)
        batas = self._tepi + self._jangkau
        if _sinar_inti is not None:
            t = _sinar_inti(dist, cx, cy, ux, uy, batas, _SINAR_ITERASI)
        else:
            t = _sinar_loop_numpy(dist, cx, cy, ux, uy, batas, _SINAR_ITERASI)
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
