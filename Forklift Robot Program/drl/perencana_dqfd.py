"""
perencana_dqfd.py - merencanakan jalur dengan model jauh DQfD + tembakan Reeds-Shepp.

    rencanakan(net, env, mundur=True, anggaran=600, batas_waktu=None)
    rencanakan_terbaik(net, env, anggaran=1200, batas_waktu=None, bobot=w)
    cari(net, env, metode, ...)          metode mundur / terbaik / gabungan
    rencanakan_dqfd(net, env, awal, tujuan, ...)   reset + cari: jalur model jauh + RS
    DQfDPlanner       antarmuka sama dengan HybridAStarPlanner

ATURAN ver-1-6. Model jauh (meniru Hybrid A*) memilih primitif sampai titik acuan
di dalam 60 cm dari tujuan DAN tidak ada halangan (saringan peta Dijkstra +
tembakan RS bebas tabrakan); tembakan Reeds-Shepp itulah ujung jalurnya. Semua
itu terjadi di lingkungan (drl/lingkungan.py), jadi pencarian di sini cukup
menjalankan model jauh sampai lingkungan menyatakan SERAH. Forward jaringan
memakai JaringanNumpy (tanpa PyTorch).

MUNDUR (rencanakan): greedy - dari tiap keadaan ambil aksi sah dengan Q
terbesar. Bila buntu dan mundur=True, kembali ke keadaan sebelumnya dan coba
aksi terbaik BERIKUTNYA (backtracking, pencarian mendalam berurutan Q). Sel
yang sudah pernah dicoba tetap tertutup setelah mundur, jadi cabang yang sama
tidak diulang. anggaran = jumlah langkah yang boleh dicoba seluruhnya.

TERBAIK-DULU (rencanakan_terbaik): tiap calon langkah diberi
    f = g + c(a) + w x h,   h = max(0, -Q(s,a) x skala - c(a))
g = biaya yang sudah ditempuh, c(a) = biaya primitif itu (tepat, dari
lingkungan), h = perkiraan jaringan biaya-sisa SETELAH langkah itu, termasuk
tembakan RS. w = bobot heuristik Hybrid A* (konfig.PARAM_RENCANA["h_weight"]),
jadi DQfD dan Hybrid A* memakai w yang SAMA. Satu forward di s memberi h untuk
semua calon sekaligus. Yang dibuka selalu calon dengan f terkecil dari SELURUH
pohon. Sel yang sudah dimasuki cabang mana pun tertutup untuk primitif dengan
arah yang sama (closed set, lihat MASK di lingkungan.py).

LAZY (pencarian="lazy", rencanakan_terbaik_malas): terbaik-dulu dengan f yang sama,
tetapi 14 primitif tidak disimulasi di muka - hanya aksi yang diambil dari heap yang
dicek tabrakannya. BAWAAN ROBOT (DQfDPlanner, PENCARIAN_ROBOT); fungsi modul ini
sendiri bawaannya eager (PENCARIAN_BAWAAN), dipakai validasi latih dan evaluasi.
Jalan pintas ruang terbuka (DQfDPlanner pintas=True, lihat lingkungan._terbuka)
opsional - di mode lazy hampir tanpa efek. Tahap backtracking tetap eager.

GABUNGAN (bawaan): terbaik-dulu dengan PORSI_WAKTU_TERBAIK dari batas waktu,
lalu mundur bila gagal. A* hanya muncul sebagai CADANGAN di mode "dqfd";
"dqfd-murni" tanpa cadangan.
"""
import heapq
import itertools
import math
import os
import time

import numpy as np

import konfig
from drl import lingkungan as L
from drl.jaringan_numpy import NetDenganP, SKALA_REWARD, muat_dari_pt, muat_model_p, periksa_profil
from drl.lingkungan import MAKS_LANGKAH

# FORKLIFT_DIR_MODEL: folder model lain (mis. model_dqn_gudang3 = model yang dilatih di Gudang3).
# Lewat variabel lingkungan supaya proses perencana anak (perencana_proses.py) ikut memakainya.
DIR_MODEL = os.path.abspath(os.environ["FORKLIFT_DIR_MODEL"]) if os.environ.get("FORKLIFT_DIR_MODEL") else \
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model_dqn")

ANGGARAN_BAWAAN = 600
METODE_BAWAAN = "gabungan"
# w terbaik-dulu = w Hybrid A* (aturan ver-1-6: w sama untuk pembanding,
# demonstrator, dan DQfD).
BOBOT_TERBAIK = konfig.PARAM_RENCANA["h_weight"]
ANGGARAN_TERBAIK = 1200
# Pencarian terbaik-dulu: "eager" = 14 primitif disimulasi di muka; "lazy" = hanya
# aksi yang diambil dari heap (rencanakan_terbaik_malas). Robot (DQfDPlanner, lewat
# rencana.Perencana) memakai lazy: 0,156 vs 0,356 ms per node, tiba sama, jalur
# berubah di 3% rute (catatan Obsidian 20 §10). Validasi latih (latih.py) dan
# evaluasi.py memakai fungsi di modul ini langsung dan tetap eager, supaya skor
# model yang sudah dilatih tetap sebanding.
# "belajar" (mode B, uji): lazy TANPA cek tabrakan primitif - kepala kelayakan model
# (P aksi menabrak, jaringan.JaringanDQfD kepala_tabrak) satu-satunya yang memutuskan;
# tanpa backtracking. Tembakan RS tetap dicek. Untuk mengukur, bukan untuk robot.
# "selektif" (uji): lazy, tetapi aksi yang diprediksi SANGAT aman (P < AMBANG_YAKIN_AMAN)
# dijalankan tanpa cek tabrakan; aksi lain tetap dicek seperti lazy.
# "belajar-periksa": mode B, lalu jalur akhirnya SAJA dicek tabrakannya (periksa_jalur); bila
# menabrak atau mode B gagal, rencana ulang dengan lazy (mode A), lalu backtracking.
PENCARIAN = ("eager", "lazy", "belajar", "selektif", "belajar-periksa")
PENCARIAN_BAWAAN = "eager"
# Robot sejak 2026-11-08 (siang): lazy (mode A) dengan P dari model P terpisah
# (model_dqn/id0N_kepala_p.pt) untuk urutan heap (P >= AMBANG_TABRAK di belakang) dan forward
# Q+P gabungan (jaringan_numpy.NetDenganP). Per node lebih mahal daripada "belajar-periksa"
# (Mac 0,143 vs 0,125 ms), tetapi node total 38% lebih sedikit, jadi per rute lebih cepat:
# median 4,5 vs 4,9 ms, p90 17,5 vs 24,6 ms, rata-rata 15,2 vs 21,4 ms (943 rute uji ID 01+02;
# laptop robot p90 30,4 vs 38,3 ms). Sebelumnya (pagi 2026-11-08) robot memakai "belajar-periksa".
PENCARIAN_ROBOT = "lazy"
PENCARIAN_TANPA_P = "lazy"
# Kepala kelayakan: aksi dengan P(menabrak) >= AMBANG_TABRAK diprediksi menabrak.
# lazy: tetap masuk heap tetapi di belakang semua aksi lain (f + URUTAN_BELAKANG), jadi
# pencarian tetap lengkap; belajar: tidak dimasukkan sama sekali.
AMBANG_TABRAK = 0.5
# FORKLIFT_TANPA_P=1: pencarian lazy TIDAK memakai P sama sekali (urutan heap dari Q saja, forward
# hanya model jauh). Untuk ablasi dan paper ver9: P terukur tidak mempercepat pencarian.
TANPA_P = os.environ.get("FORKLIFT_TANPA_P") == "1"
URUTAN_BELAKANG = 1e6
AMBANG_YAKIN_AMAN = 0.02
# Ambang P untuk "belajar-periksa", dipakai di KEDUA tahapnya (mode B dan cadangan lazy) -
# konfigurasi yang diukur: 0 jalur menabrak, tiba 99,8% (catatan 20 §15).
AMBANG_BELAJAR = 0.1
# Anggaran node disetel untuk arena 300 x 270 cm (Gudang, Gudang2, peta latih). Di peta yang
# lebih luas (Gudang3, 820 x 720 cm) anggaran dikali rasio luasnya, dibulatkan, paling banyak
# SKALA_ANGGARAN_MAKS kali; batas waktu tetap sama. Arena acuan atau lebih kecil: skala 1,
# jadi hasil di peta lama tidak berubah.
LUAS_ACUAN_CM2 = 300.0 * 270.0
SKALA_ANGGARAN_MAKS = 8.0


def skala_anggaran(env):
    """Pengali anggaran node menurut luas arena env (>= 1)."""
    om = env.omap
    luas = om.w * om.h * env.arena.cm_per_cell ** 2
    return min(SKALA_ANGGARAN_MAKS, max(1.0, luas / LUAS_ACUAN_CM2))


# Metode gabungan: bagian batas waktu untuk terbaik-dulu; sisanya untuk mundur.
PORSI_WAKTU_TERBAIK = 0.75

# GERBANG MUTU (mode "dqfd"): jalur DQfD ditolak - dan Hybrid A* dipakai - bila
# biayanya melebihi GERBANG_K x perkiraan jaringan sendiri, -V(s0) x skala.
# V dipelajari dari biaya A*, jadi jalur yang jauh lebih mahal dari
# perkiraannya adalah jalur yang "tersesat" (mundur berulang, berkelok).
# Perlu dikalibrasi ulang pada set validasi untuk model ver-1-6.
GERBANG_K = 1.3


def _jarak_tujuan(env):
    return math.hypot(env.tujuan[0] - env.x, env.tujuan[1] - env.y)


def mutu_jalur(jalur, tujuan, radius, cm):
    """Ganti arah (seluruh jalur / di dalam radius dari tujuan) dan galat pose akhir (cm, derajat)."""
    g, gd = hitung_ganti_arah(jalur, tujuan, radius)
    x, y, yaw = jalur[-1][:3]
    ep = math.hypot(tujuan[0] - x, tujuan[1] - y) * cm
    ey = abs(math.degrees(math.remainder(yaw - tujuan[2], 2.0 * math.pi))) if tujuan[2] is not None else None
    return {"ganti_arah": g, "ganti_arah_dekat": gd, "galat_posisi_cm": ep, "galat_arah_deg": ey}


def hitung_ganti_arah(jalur, tujuan, radius):
    """(titik balik di seluruh jalur, titik balik di dalam radius dari tujuan)."""
    semua = dekat = 0
    for a, b in zip(jalur[:-1], jalur[1:]):
        if a[3] != b[3]:
            semua += 1
            dekat += math.hypot(tujuan[0] - a[0], tujuan[1] - a[1]) < radius
    return semua, dekat


def _urutan(net, env):
    """Aksi sah, terurut Q menurun."""
    m = env.mask()
    sah = np.flatnonzero(m)
    if not len(sah):
        return []
    q = net.q(env.observasi())
    return list(sah[np.argsort(-q[sah], kind="stable")])


def rencanakan(net, env, mundur=True, anggaran=ANGGARAN_BAWAAN, batas_waktu=None):
    """env harus sudah di-reset. Kembalian dict: sukses, alasan, dicoba, mundur, waktu_s.

    batas_waktu (s): berhenti bila terlampaui, seperti time_limit A*. None = tanpa batas.
    """
    t0 = time.perf_counter()
    snap0 = env.simpan_keadaan()
    tumpukan = [[snap0, _urutan(net, env)]]
    di = snap0                  # keadaan tempat env berada sekarang (None = tidak di simpul mana pun)
    dicoba = n_mundur = 0
    alasan = "buntu"
    jarak_min = _jarak_tujuan(env)
    while tumpukan:
        if dicoba >= anggaran:
            alasan = "anggaran habis"
            break
        if batas_waktu is not None and time.perf_counter() - t0 > batas_waktu:
            alasan = "batas waktu habis"
            break
        snap, calon = tumpukan[-1]
        if not calon:
            tumpukan.pop()
            n_mundur += 1
            if not mundur:
                break
            continue
        a = calon.pop(0)
        if di is not snap:
            env.pulihkan_keadaan(snap)
            di = snap
        if not env.mask()[a]:
            continue            # tertutup oleh sel yang dikunjungi cabang lain
        _, _, akhir, info = env.step(a)
        dicoba += 1
        jarak_min = min(jarak_min, _jarak_tujuan(env))
        if info["sukses"]:
            return {"sukses": True, "alasan": info["alasan"], "dicoba": dicoba, "mundur": n_mundur,
                    "waktu_s": time.perf_counter() - t0, "jarak_min": 0.0}
        if akhir:
            di = None
            if not mundur:
                alasan = info["alasan"]
                break
            continue
        if env.langkah_ke >= MAKS_LANGKAH:
            di = None
            continue
        s = env.simpan_keadaan()
        tumpukan.append([s, _urutan(net, env)])
        di = s
    return {"sukses": False, "alasan": alasan, "dicoba": dicoba, "mundur": n_mundur,
            "waktu_s": time.perf_counter() - t0, "jarak_min": jarak_min}


def rencanakan_terbaik(net, env, anggaran=ANGGARAN_TERBAIK, batas_waktu=None, bobot=BOBOT_TERBAIK):
    """Pencarian terbaik-dulu berpandu Q. env harus sudah di-reset.

    Kembalian dict sama dengan rencanakan(): sukses, alasan, dicoba, mundur
    (= berapa kali pindah cabang), waktu_s. Jalur ada di env.jalur() bila sukses.
    """
    t0 = time.perf_counter()
    # Simpul: keadaan lingkungan + induk + pose yang ditambahkan langkahnya.
    # Jalur lengkap simpul dibangun ulang dari rantai induk saat dipulihkan,
    # karena simpul berikutnya bisa berasal dari cabang mana saja.
    snap, induk, potong = [env.simpan_keadaan()], [-1], [[]]
    heap, urut = [], itertools.count()

    def buka(i):
        m = env.mask()
        sah = np.flatnonzero(m)
        if not len(sah):
            return
        q = net.q(env.observasi())
        g = env.biaya
        for a in sah:
            a = int(a)
            c = env._calon[a][1]
            h = max(0.0, -float(q[a]) * SKALA_REWARD - c)
            heapq.heappush(heap, (g + c + bobot * h, next(urut), i, a))

    def pulihkan(i):
        rantai = []
        j = i
        while j > 0:
            rantai.append(potong[j])
            j = induk[j]
        env._pose = [p for bagian in reversed(rantai) for p in bagian]
        env.pulihkan_keadaan(snap[i])

    buka(0)
    di = 0                      # simpul tempat env berada sekarang (None = di simpul akhir)
    dicoba = pindah = 0
    alasan = "buntu"
    jarak_min = _jarak_tujuan(env)
    while heap:
        if dicoba >= anggaran:
            alasan = "anggaran habis"
            break
        if batas_waktu is not None and time.perf_counter() - t0 > batas_waktu:
            alasan = "batas waktu habis"
            break
        _, _, i, a = heapq.heappop(heap)
        if di != i:
            pulihkan(i)
            di = i
            pindah += 1
        if not env.mask()[a]:
            continue                # sel tujuannya sudah dimasuki cabang lain
        n0 = len(env._pose)
        _, _, akhir, info = env.step(a)
        dicoba += 1
        jarak_min = min(jarak_min, _jarak_tujuan(env))
        if info["sukses"]:
            return {"sukses": True, "alasan": info["alasan"], "dicoba": dicoba, "mundur": pindah,
                    "waktu_s": time.perf_counter() - t0, "jarak_min": 0.0}
        j = len(snap)
        snap.append(env.simpan_keadaan())
        induk.append(i)
        potong.append(env._pose[n0:])
        if akhir or env.langkah_ke >= MAKS_LANGKAH:
            di = None
            continue
        buka(j)
        di = j
    return {"sukses": False, "alasan": alasan, "dicoba": dicoba, "mundur": pindah,
            "waktu_s": time.perf_counter() - t0, "jarak_min": jarak_min}


def rencanakan_terbaik_malas(net, env, anggaran=ANGGARAN_TERBAIK, batas_waktu=None, bobot=BOBOT_TERBAIK,
                             belajar=False, selektif=None, ambang=None):
    """Terbaik-dulu LAZY: f dan urutan sama dengan rencanakan_terbaik, tetapi 14 primitif
    TIDAK disimulasi di muka.

    buka(): f dihitung dari Q dan biaya aritmetika gerak (env.calon_malas) - tanpa cek
    tabrakan dan tanpa penalti jarak-bebas, kecuali di ruang terbuka (env._terbuka) yang
    hasilnya pasti. Baru saat diambil dari heap aksi itu disimulasi dan dicek
    (env.siapkan_aksi): menabrak atau selnya sudah dimasuki -> dibuang; f tepat lebih
    besar dari f puncak heap -> dimasukkan ulang dengan f tepat (Lazy Weighted A*).
    Pindah cabang tidak menghitung ulang calon. Jalur tetap bebas tabrakan: setiap
    primitif yang dijalankan sudah dicek, atau dijamin bebas oleh ruang terbuka.

    Model dengan kepala kelayakan (net.kepala_tabrak): aksi yang diprediksi menabrak
    (P >= AMBANG_TABRAK) dimasukkan di belakang heap. belajar=True (mode B): aksi itu tidak
    dimasukkan, dan aksi yang diambil TIDAK dicek tabrakannya - jaringan saja yang memutuskan.
    selektif=x (verifikasi selektif): aksi dengan P < x dijalankan tanpa cek tabrakan, lainnya
    dicek seperti lazy.

    Kembalian dict sama dengan rencanakan_terbaik, ditambah dibuang (aksi yang ternyata
    menabrak / selnya sudah dimasuki), masuk_ulang, dan aksi_jalur (urutan aksi jalur
    yang berhasil; untuk memeriksa ulang jalur mode B).
    """
    kepala = bool(getattr(net, "kepala_tabrak", False)) and not (TANPA_P and not belajar and selektif is None)
    ambang = AMBANG_TABRAK if ambang is None else ambang
    if belajar and not kepala:
        raise ValueError("pencarian 'belajar' butuh model dengan kepala kelayakan (kepala_tabrak)")
    t0 = time.perf_counter()
    snap, induk, potong, aksi_simpul, q_simpul, p_simpul = [env.simpan_keadaan()], [-1], [[]], [None], {}, {}
    heap, urut = [], itertools.count()
    PERKIRAAN, TERBUKA, TEPAT = 0, 1, 2

    def buka(i):
        calon = env.calon_malas()
        if not calon:
            return
        if kepala:
            q, p = net.q_tabrak(env.observasi())
        else:
            q, p = net.q(env.observasi()), None
        q_simpul[i], p_simpul[i] = q, p
        g = env.biaya
        for a, c, terbuka in calon:
            h = max(0.0, -float(q[a]) * SKALA_REWARD - c)
            f = g + c + bobot * h
            if p is not None and not terbuka and p[a] >= ambang:
                if belajar:
                    continue                 # jaringan memutuskan: menabrak
                f += URUTAN_BELAKANG
            heapq.heappush(heap, (f, next(urut), i, a, TERBUKA if terbuka else PERKIRAAN))

    def pulihkan(i):
        rantai = []
        j = i
        while j > 0:
            rantai.append(potong[j])
            j = induk[j]
        env._pose = [p for bagian in reversed(rantai) for p in bagian]
        env.pulihkan_keadaan(snap[i], hitung_calon=False)

    buka(0)
    di = 0
    dicoba = pindah = dibuang = masuk_ulang = tanpa_cek_n = 0
    alasan = "buntu"
    jarak_min = _jarak_tujuan(env)

    def hasil(sukses, alasan_, jm, i_akhir=None, a_akhir=None):
        aksi_jalur = None
        if sukses:
            aksi_jalur = [a_akhir]
            j = i_akhir
            while j > 0:
                aksi_jalur.append(aksi_simpul[j])
                j = induk[j]
            aksi_jalur.reverse()
        return {"sukses": sukses, "alasan": alasan_, "dicoba": dicoba, "mundur": pindah,
                "waktu_s": time.perf_counter() - t0, "jarak_min": jm, "dibuang": dibuang,
                "masuk_ulang": masuk_ulang, "tanpa_cek": tanpa_cek_n, "aksi_jalur": aksi_jalur}

    while heap:
        if dicoba >= anggaran:
            alasan = "anggaran habis"
            break
        if batas_waktu is not None and time.perf_counter() - t0 > batas_waktu:
            alasan = "batas waktu habis"
            break
        f, _, i, a, jenis = heapq.heappop(heap)
        if di != i:
            pulihkan(i)
            di = i
            pindah += 1
        tanpa_cek = belajar or jenis == TERBUKA or (
            selektif is not None and jenis == PERKIRAAN and p_simpul.get(i) is not None and p_simpul[i][a] < selektif)
        tanpa_cek_n += tanpa_cek and jenis != TERBUKA
        calon = env.siapkan_aksi(a, tanpa_cek=tanpa_cek)
        if calon is None:
            dibuang += 1
            continue
        if jenis == PERKIRAAN and not tanpa_cek:
            c = calon[1]
            fe = env.biaya + c + bobot * max(0.0, -float(q_simpul[i][a]) * SKALA_REWARD - c)
            if heap and fe > f and fe > heap[0][0]:
                heapq.heappush(heap, (fe, next(urut), i, a, TEPAT))
                masuk_ulang += 1
                continue
        n0 = len(env._pose)
        _, _, akhir, info = env.step(a, hitung_calon=False)
        dicoba += 1
        jarak_min = min(jarak_min, _jarak_tujuan(env))
        if info["sukses"]:
            return hasil(True, info["alasan"], 0.0, i, a)
        j = len(snap)
        snap.append(env.simpan_keadaan())
        induk.append(i)
        aksi_simpul.append(a)
        potong.append(env._pose[n0:])
        if akhir or env.langkah_ke >= MAKS_LANGKAH:
            di = None
            continue
        buka(j)
        di = j
    return hasil(False, alasan, jarak_min)


def periksa_jalur(env, aksi_jalur):
    """Periksa SATU jalur hasil pencarian tanpa cek tabrakan (mode belajar): putar ulang aksi_jalur
    dari pose awal dan cek tabrakan HANYA primitif-primitif jalur itu (tembakan RS penutup dicek
    seperti biasa). True bila semuanya bebas dan berakhir serah - env kemudian berisi jalur itu
    dengan biaya tepat, termasuk penalti jarak-bebas. False: jalur ditolak."""
    env.ulang_dari_awal()
    for a in aksi_jalur or []:
        if env.selesai or env.siapkan_aksi(a) is None:
            return False
        env.step(a, hitung_calon=False)
    return env.sukses


def cari(net, env, metode=METODE_BAWAAN, batas_waktu=None, mulai_ulang=None, bobot=BOBOT_TERBAIK,
         pencarian=None):
    """env sudah di-reset. Kembalian dict rencanakan() + 'metode'.

    gabungan: terbaik-dulu (PORSI_WAKTU_TERBAIK dari batas waktu), lalu mundur
    bila gagal. mulai_ulang(): me-reset env ke keadaan awal yang sama - sel
    tertutup milik pencarian pertama tidak boleh menutup pencarian kedua.
    pencarian: "eager" (14 primitif disimulasi di muka), "lazy"
    (rencanakan_terbaik_malas), "belajar" (lazy tanpa cek tabrakan primitif, tanpa
    backtracking - mode B, uji), "selektif", atau "belajar-periksa" (mode B + periksa
    jalur akhir + cadangan lazy); hanya untuk tahap terbaik-dulu.
    """
    t0 = time.perf_counter()
    pencarian = pencarian or PENCARIAN_BAWAAN
    sk = skala_anggaran(env)
    a_terbaik, a_mundur = int(round(ANGGARAN_TERBAIK * sk)), int(round(ANGGARAN_BAWAAN * sk))
    if pencarian not in PENCARIAN:
        raise ValueError(f"pencarian DQfD '{pencarian}' tidak dikenal (pilihan: {', '.join(PENCARIAN)})")
    if metode == "mundur":
        h = rencanakan(net, env, mundur=True, anggaran=a_mundur, batas_waktu=batas_waktu)
        h["metode"] = "mundur"
        return h
    bt = None if batas_waktu is None else (PORSI_WAKTU_TERBAIK * batas_waktu if metode == "gabungan"
                                           else batas_waktu)
    if pencarian == "eager":
        h = rencanakan_terbaik(net, env, anggaran=a_terbaik, batas_waktu=bt, bobot=bobot)
    elif pencarian == "belajar-periksa":
        hb = rencanakan_terbaik_malas(net, env, anggaran=a_terbaik, batas_waktu=bt, bobot=bobot,
                                      belajar=True, ambang=AMBANG_BELAJAR)
        if hb["sukses"] and periksa_jalur(env, hb["aksi_jalur"]):
            hb["metode"] = "belajar, jalur lolos periksa"
            hb["waktu_s"] = time.perf_counter() - t0
            return hb
        sebab = "jalur belajar menabrak" if hb["sukses"] else f"belajar gagal ({hb['alasan']})"
        env.ulang_dari_awal()
        bt2 = None if bt is None else bt - (time.perf_counter() - t0)
        if bt2 is not None and bt2 <= 0:
            h = dict(hb, sukses=False, alasan=sebab)
        else:
            h = rencanakan_terbaik_malas(net, env, anggaran=a_terbaik, batas_waktu=bt2, bobot=bobot,
                                         ambang=AMBANG_BELAJAR)
            h["dicoba"] += hb["dicoba"]
        h["cadangan"] = sebab
        if h["sukses"]:
            h["metode"] = f"belajar -> lazy ({sebab})"
            h["waktu_s"] = time.perf_counter() - t0
            return h
    else:
        h = rencanakan_terbaik_malas(net, env, anggaran=a_terbaik, batas_waktu=bt, bobot=bobot,
                                     belajar=pencarian == "belajar",
                                     selektif=AMBANG_YAKIN_AMAN if pencarian == "selektif" else None)
    h["metode"] = "terbaik" if pencarian != "belajar-periksa" else "belajar -> lazy"
    if h["sukses"] or metode == "terbaik" or pencarian == "belajar":
        return h
    sisa = None if batas_waktu is None else batas_waktu - (time.perf_counter() - t0)
    if sisa is not None and sisa <= 0:
        return h
    if mulai_ulang is None or mulai_ulang() is None:
        return h
    hm = rencanakan(net, env, mundur=True, anggaran=a_mundur, batas_waktu=sisa)
    hm["metode"] = "terbaik -> mundur"
    hm["dicoba"] += h["dicoba"]
    hm["waktu_s"] = time.perf_counter() - t0
    if not hm["sukses"]:
        hm["alasan"] = f"terbaik: {h['alasan']}; mundur: {hm['alasan']}"
    return hm


def rencanakan_dqfd(net, env, awal, tujuan, omap=None, metode=METODE_BAWAAN, batas_waktu=None,
                    bobot=BOBOT_TERBAIK, pencarian=None):
    """Satu rute: reset lingkungan, cari dengan model jauh sampai serah (tembakan RS).

    Kembalian dict:
      sukses, alasan, metode, dicoba, waktu_s
      jalur        jalur model jauh + ekor RS (format plan()), [] bila gagal / sudah di tujuan
      biaya        biaya jalur (sel), termasuk tembakan RS
      n_titik_rs   titik ekor RS di ujung jalur
      perkiraan    -V(s0) x skala: perkiraan biaya total menurut jaringan (gerbang mutu),
                   None bila jalurnya RS saja dari pose awal
    """
    t0 = time.perf_counter()
    obs, info = env.reset(awal, tujuan, omap)
    hasil = {"sukses": False, "alasan": info["alasan"], "metode": "-", "dicoba": 0,
             "jalur": [], "biaya": 0.0, "n_titik_rs": 0, "perkiraan": None}
    if obs is None:
        if info["alasan"] == "sudah di tujuan":
            hasil.update(sukses=True)
        elif env.sukses:                                  # serah di pose awal: RS saja
            hasil.update(sukses=True, alasan="", metode="RS dari awal", jalur=env.jalur(),
                         biaya=env.biaya, n_titik_rs=env.n_titik_rs)
        hasil["waktu_s"] = time.perf_counter() - t0
        return hasil
    hasil["perkiraan"] = max(1.0, -net.qv(obs)[1] * SKALA_REWARD)
    sisa = None if batas_waktu is None else max(0.0, batas_waktu - (time.perf_counter() - t0))
    h = cari(net, env, metode, batas_waktu=sisa, bobot=bobot, pencarian=pencarian,
             mulai_ulang=lambda: env.reset(awal, tujuan, omap)[0])
    hasil.update(sukses=h["sukses"], alasan="" if h["sukses"] else h["alasan"], metode=h["metode"],
                 dicoba=h["dicoba"], aksi_jalur=h.get("aksi_jalur"))
    if h["sukses"]:
        hasil.update(jalur=env.jalur(), biaya=env.biaya, n_titik_rs=env.n_titik_rs)
    hasil["waktu_s"] = time.perf_counter() - t0
    return hasil


# ------------------------------------------------------------ operasional
_MODEL = {}


def jalur_model(robot_id, dir_model=None):
    return os.path.join(dir_model or DIR_MODEL, f"id{robot_id:02d}_{L.JENIS}.pt")


def jalur_model_p(robot_id, dir_model=None):
    """Model P terpisah robot ini (drl.kelayakan --latih), boleh tidak ada."""
    return os.path.join(dir_model or DIR_MODEL, f"id{robot_id:02d}_kepala_p.pt")


_MODEL_P = {}


def muat_model_p_robot(robot_id, dir_model=None):
    """ModelPNumpy robot ini (lewat cache .npz, di-cache), atau None bila tidak ada."""
    jalur = jalur_model_p(robot_id, dir_model)
    if not os.path.exists(jalur):
        return None
    if jalur not in _MODEL_P:
        mp, meta = muat_model_p(jalur)
        if meta.get("robot") not in (None, robot_id) or meta.get("n_obs") != L.N_OBS or meta.get("n_aksi") != L.N_AKSI:
            raise SystemExit(f"[!] {jalur} tidak cocok dengan robot {robot_id:02d} / bentuk masukan-keluaran model jauh")
        _MODEL_P[jalur] = mp
    return _MODEL_P[jalur]


_SUDAH_DIBERITAHU = set()


def _periksa_milik(meta, robot_id, jalur):
    if meta.get("robot") not in (None, robot_id):
        raise SystemExit(f"[!] {jalur} milik robot {meta['robot']:02d}, bukan {robot_id:02d}")
    periksa_profil(meta, konfig.profil(robot_id), jalur)


def muat_model_numpy(robot_id, dir_model=None):
    """JaringanNumpy model jauh robot ini (model_dqn/id0N_jauh.pt), lewat cache .npz, di-cache."""
    jalur = jalur_model(robot_id, dir_model)
    if not os.path.exists(jalur):
        raise SystemExit(f"[!] Model DQfD robot {robot_id:02d} belum ada: {jalur}\n"
                         f"    Bangkitkan demo lalu latih: python -m drl.skenario --id {robot_id} "
                         f"lalu python -m drl.latih --id {robot_id}")
    if jalur not in _MODEL:
        net, meta = muat_dari_pt(jalur)
        _periksa_milik(meta, robot_id, jalur)
        _MODEL[jalur] = net
    return _MODEL[jalur]


class DQfDPlanner:
    """Pengganti HybridAStarPlanner untuk Perencana (rencana.py).

    Anggota yang dipakai Perencana sama: plan(omap, start, goal), last_stats,
    time_limit, v. Jalur yang dikembalikan berformat sama persis - daftar
    (x, y, yaw, arah) dalam sel - jadi pelacak, wasit, dan navigasi tidak
    tahu jalurnya dari mana. last_stats["rs_titik"] = jumlah titik tembakan RS
    di ujung jalur, supaya navigasi.py menandainya dengan warna Reeds-Shepp.

    astar: Hybrid A* standar (peran "astar") untuk kendaraan & margin yang sama,
    dipakai sebagai CADANGAN bila cadangan=True dan DQfD gagal.
    pustaka: Hybrid A* peran "demonstrator" - pustaka gerak lingkungan model
    jauh (primitif, biaya, heuristik, tembakan RS < 60 cm).
    last_stats["perencana"] mencatat siapa yang akhirnya menghasilkan jalur.
    """

    def __init__(self, astar, pustaka, arena, profil, margin_cm, cadangan=True,
                 dir_model=None, gerbang_k=None, metode=METODE_BAWAAN, pencarian=None, pintas=None):
        if metode not in ("terbaik", "mundur", "gabungan"):
            raise ValueError(f"metode DQfD '{metode}' tidak dikenal")
        self.pencarian = pencarian or PENCARIAN_ROBOT
        if self.pencarian not in PENCARIAN:
            raise ValueError(f"pencarian DQfD '{self.pencarian}' tidak dikenal")
        self.astar = astar
        self.pustaka = pustaka
        self.metode = metode
        # Gerbang hanya bermakna bila ada cadangan; tanpa cadangan menolak
        # jalur yang sah hanya membuat robot diam.
        self.gerbang_k = (GERBANG_K if gerbang_k is None else gerbang_k) if cadangan else None
        self.v = astar.v
        self.time_limit = astar.time_limit
        self.cadangan = cadangan
        self.net = muat_model_numpy(profil.id, dir_model)
        mp = muat_model_p_robot(profil.id, dir_model)
        if mp is not None:
            self.net = NetDenganP(self.net, mp)          # P dari model P terpisah, Q dari model jauh
        if self.pencarian in ("belajar", "belajar-periksa", "selektif") and not self.net.kepala_tabrak:
            if (profil.id, self.pencarian) not in _SUDAH_DIBERITAHU:
                print(f"[i] DQfD ID {profil.id:02d}: model tanpa P (kepala kelayakan / id{profil.id:02d}_kepala_p.pt) - "
                      f"pencarian '{self.pencarian}' diganti '{PENCARIAN_TANPA_P}'")
                _SUDAH_DIBERITAHU.add((profil.id, self.pencarian))
            self.pencarian = PENCARIAN_TANPA_P
        self.env = L.LingkunganRencana(arena, profil, margin_cm=margin_cm, planner=pustaka)
        if pintas is not None:
            self.env.pintas_terbuka = bool(pintas)      # jalan pintas ruang terbuka, lihat lingkungan._terbuka
        self.last_stats = {}

    def plan(self, omap, start, goal):
        t0 = time.perf_counter()
        self._rencana_terakhir = (omap, start, goal)     # untuk memeriksa ulang jalur (drl/ukur_node.py)
        self.pustaka._t_rs = 0.0                 # waktu di dalam tembakan RS (HybridAStarBertanda)
        h = rencanakan_dqfd(self.net, self.env, start, goal, omap, metode=self.metode,
                            batas_waktu=self.time_limit, pencarian=self.pencarian)
        t_rs = self.pustaka._t_rs
        # Rincian waktu (detik) per bagian, dijumlahkan Perencana.rencanakan().
        rincian = {"dqfd_jauh": max(0.0, h["waktu_s"] - t_rs), "rs": t_rs}
        if h["sukses"] and not h["jalur"]:
            self.last_stats = {"expansions": 0, "time": h["waktu_s"], "reason": "sudah di tujuan",
                               "perencana": "dqfd", "rincian_s": rincian}
            return []
        if h["sukses"]:
            rasio = h["biaya"] / h["perkiraan"] if h["perkiraan"] else None
            if self.gerbang_k is None or rasio is None or rasio <= self.gerbang_k:
                self.last_stats = {
                    "expansions": h["dicoba"], "time": time.perf_counter() - t0, "reason": "",
                    "perencana": f"dqfd ({h['metode']})", "rasio_perkiraan": rasio,
                    "rs_titik": h["n_titik_rs"], "rincian_s": rincian, "aksi_jalur": h.get("aksi_jalur")}
                return h["jalur"]
            h["alasan"] = f"ditolak gerbang mutu (biaya {rasio:.2f} x perkiraan)"
        if not self.cadangan:
            self.last_stats = {"expansions": h["dicoba"], "time": time.perf_counter() - t0,
                               "reason": f"DQfD: {h['alasan']}", "perencana": "dqfd",
                               "rincian_s": rincian}
            return None
        self.astar.time_limit = self.time_limit
        jalur = self.astar.plan(omap, start, goal)
        self.last_stats = dict(self.astar.last_stats or {})
        for k, v in self.last_stats.get("rincian_s", {}).items():
            rincian[k] = rincian.get(k, 0.0) + v
        self.last_stats.update(time=time.perf_counter() - t0, perencana="A* (cadangan)",
                               dqfd_gagal=h["alasan"], rincian_s=rincian)
        return jalur
