"""
perencana_dqfd.py - merencanakan jalur dengan jaringan DQfD.

    rencanakan(net, env, mundur=True, anggaran=600, batas_waktu=None, terima=None)
    rencanakan_terbaik(net, env, anggaran=600, batas_waktu=None, bobot=1.0, terima=None)
    cari(net, env, metode, ...)        satu model, metode mundur / terbaik / gabungan
    rencanakan_dua(...)                DUA model: jauh sampai serah, lalu dekat
    DQfDPlanner       antarmuka sama dengan HybridAStarPlanner (tahap 9)

MUNDUR (rencanakan): greedy - dari tiap keadaan ambil aksi sah dengan Q
terbesar. Bila buntu dan mundur=True, kembali ke keadaan sebelumnya dan coba
aksi terbaik BERIKUTNYA (pencarian mendalam berurutan Q). Sel yang sudah pernah
dicoba tetap tertutup setelah mundur, jadi cabang yang sama tidak diulang.
anggaran = jumlah langkah yang boleh dicoba seluruhnya; habis = gagal. Tanpa
buntu, hasilnya persis sama dengan rollout greedy.

TERBAIK-DULU (rencanakan_terbaik): tiap calon langkah diberi
    f = g + c(a) + bobot x h,   h = max(0, -Q(s,a) x skala - c(a))
g = biaya yang sudah ditempuh, c(a) = biaya primitif itu (tepat, dari
lingkungan), h = perkiraan jaringan biaya-sisa SETELAH langkah itu. Satu
forward di s memberi h untuk semua calon sekaligus, jadi calon tidak perlu
dibuka dulu. Yang dibuka selalu calon dengan f terkecil dari SELURUH pohon -
bukan dari keadaan terakhir seperti mundur. Jaringan benar: jalannya sama
dengan greedy. Jaringan salah: pindah ke cabang yang secara keseluruhan lebih
murah, bukan sekadar mundur selangkah (putaran 1-2: mundur menyelamatkan
rute, tapi jalurnya berkelok - p90 biaya 2-4,5x Hybrid A*).
Calon yang langsung TIBA di toleransi tujuan diberi h = 0 (biayanya tepat,
terminal), jadi jalur ditutup bila langkah penutup itu memang bersaing. Sel
yang sudah dimasuki cabang mana pun tertutup untuk primitif penuh dengan arah
yang sama (closed set, lihat MASK di lingkungan.py).

FULL DQfD (2026-10-20): tidak ada Reeds-Shepp maupun A* di sini - SAMBUNG
dihapus, ekor RS demonstrator ditiru jaringan dengan primitif penuh + pendek.
A* hanya muncul sebagai CADANGAN di mode "dqfd"; "dqfd-murni" tanpa cadangan.

DUA MODEL (versi aksi 4, ver-1-2): model JAUH (meniru masukan & keluaran Hybrid
A*) mencari sampai di dalam 60 cm (batas terjauh) DAN tidak ada halangan untuk
mendarat; selama masih ada halangan model jauh MELANJUTKAN. "Tidak ada halangan"
diperiksa model DEKAT (meniru masukan & keluaran Reeds-Shepp) sendiri: ia
mencoba mendarat dari pose itu (rencanakan_dua -> cek_serah); berhasil tanpa
tabrakan = berhenti di situ.
"""
import copy
import heapq
import itertools
import math
import os
import time

import numpy as np

import konfig
from drl import lingkungan as L
from drl.jaringan_numpy import SKALA_REWARD, JaringanNumpy, muat_dari_pt, periksa_profil
from drl.lingkungan import MAKS_LANGKAH

# torch TIDAK diimpor di tingkat modul (ver-1-5, perubahan 2): perencana
# operasional (DQfDPlanner) memakai JaringanNumpy, jadi navigasi.py tidak perlu
# torch. Fungsi pencarian di bawah tetap menerima jaringan PyTorch (latih.py,
# dagger.py, evaluasi.py) - torch diimpor di _q/_qv hanya untuk jaringan itu.

DIR_MODEL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model_dqn")


def _q(net, obs):
    """Q semua aksi untuk satu observasi: JaringanNumpy atau jaringan PyTorch."""
    if isinstance(net, JaringanNumpy):
        return net.q(obs)
    import torch
    with torch.no_grad():
        return net(torch.as_tensor(obs[None]))[0].numpy()


def _qv(net, obs):
    """(Q, V) untuk satu observasi - V dipakai gerbang mutu."""
    if isinstance(net, JaringanNumpy):
        return net.qv(obs)
    import torch
    with torch.no_grad():
        q, akt = net(torch.as_tensor(obs[None]), aktivasi=True)
    return q[0].numpy(), float(akt["v"][0, 0])
ANGGARAN_BAWAAN = 600

# Metode operasional DQfDPlanner. Dipilih pada SET VALIDASI (160 rute, 60 Rumah),
# model putaran 2, 2026-09-30:
#                         tiba          Rumah tiba     rasio p90     ms p90
#   mundur (600)          97,5 / 95,6   96,7 / 95,0    2,24 / 2,27   104 / 176
#   terbaik w1,0 a600     94,4 / 95,6   91,7 / 93,3    1,03 / 1,11   773 / 552
#   terbaik w1,5 a600     95,0 / 96,9   91,7 / 96,7    1,19 / 1,15   576 / 413
#   terbaik w2,0 a1200    97,5 / 98,8   98,3 / 100     1,20 / 1,17   427 / 337   <- dipakai
# (ID 01 / ID 02). Jalurnya diperiksa ulang secara independen: semua sah.
#
# FULL DQfD (versi aksi 2, 2026-10-20) - set UJI, 466 / 477 rute, DQfD murni:
#                         tiba          biaya p90 vs A*   ganti arah   ms p90
#   mundur (600)          97,2 / 97,5   5,17 / 6,49       6,0 / 7,1    110 / 132
#   terbaik w2,0 a1200    68,9 / 60,0   1,22 / 1,24       1,1 / 1,1   1683 / 1681
# Terbaik-dulu menghasilkan jalur sebagus demonstrator, tapi 28-36% rute
# kehabisan anggaran di dalam 60 cm dari tujuan; mundur hampir selalu tiba,
# tapi maju-mundur 4-6x di dekat tujuan. Anggaran 3000 hanya menaikkan tiba
# terbaik 74 -> 80% (120 rute ID 01) dengan p90 4,2 s. GABUNGAN = terbaik
# dulu, mundur bila gagal: 99,2% tiba, p90 biaya 2,75x, 3,55 ganti arah,
# p90 1,8 s (120 rute ID 01). Dua-duanya DQfD - tanpa Reeds-Shepp / A*.
METODE_BAWAAN = "gabungan"
BOBOT_TERBAIK = 2.0
ANGGARAN_TERBAIK = 1200
# Metode gabungan: bagian batas waktu untuk terbaik-dulu; sisanya untuk mundur
# (median 41-45 ms, p90 110-132 ms di set uji).
PORSI_WAKTU_TERBAIK = 0.75

# GERBANG MUTU (mode "dqfd"): jalur DQfD ditolak - dan A* dipakai - bila
# biayanya melebihi GERBANG_K x perkiraan jaringan sendiri, -V(s0) x skala.
# V dipelajari dari biaya A*, jadi jalur yang jauh lebih mahal dari
# perkiraannya adalah jalur yang "tersesat" (mundur berulang, berkelok).
# Dikalibrasi pada SET VALIDASI dengan model putaran 2 (2026-09-30):
# log(biaya/perkiraan) berkorelasi 0,82 (ID 01) / 0,70 (ID 02) dengan
# log(biaya/A*). Pada 1,3: diterima 76,5% / 84,5%, p90 rasio jalur yang
# diterima 2,17 -> 1,29 / 2,13 -> 1,31, jalur baik yang ikut ditolak 15/322
# dan 7/339. (Kalibrasi pertama, model putaran 1 pada set uji, juga memberi 1,3.)
# Dengan metode terbaik-dulu jalur buruk sudah jarang (24/439, 21/447 pada
# validasi): gerbang 1,3 menahan 8 dan 2 di antaranya, dengan harga 15 dan 2
# jalur baik dialihkan ke Hybrid A* - hanya menambah waktu, bukan menurunkan
# mutu. Tetap dipasang sebagai jaring pengaman.
GERBANG_K = 1.3


def _jarak_tujuan(env):
    return math.hypot(env.tujuan[0] - env.x, env.tujuan[1] - env.y)


def mutu_jalur(jalur, tujuan, radius, cm):
    """Ganti arah (seluruh jalur / di dalam radius serah) dan galat pose akhir (cm, derajat)."""
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
    q = _q(net, env.observasi())
    return list(sah[np.argsort(-q[sah], kind="stable")])


def rencanakan(net, env, mundur=True, anggaran=ANGGARAN_BAWAAN, batas_waktu=None, terima=None):
    """env harus sudah di-reset. Kembalian dict: sukses, alasan, dicoba, mundur, waktu_s.

    batas_waktu (s): berhenti bila terlampaui, seperti time_limit A*. None = tanpa batas.
    terima: fungsi(env) -> bool, dipanggil saat episode selesai sukses (model
    jauh: serah). False = keadaan itu diperlakukan buntu dan pencarian berlanjut.
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
        if info["sukses"] and (terima is None or terima(env)):
            return {"sukses": True, "alasan": info["alasan"], "dicoba": dicoba, "mundur": n_mundur,
                    "waktu_s": time.perf_counter() - t0, "jarak_min": 0.0}
        if info["sukses"]:
            alasan = "semua pose serah gagal didaratkan"
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


def rencanakan_terbaik(net, env, anggaran=ANGGARAN_BAWAAN, batas_waktu=None, bobot=1.0, terima=None):
    """Pencarian terbaik-dulu berpandu Q. env harus sudah di-reset.

    Kembalian dict sama dengan rencanakan(): sukses, alasan, dicoba, mundur
    (= berapa kali pindah cabang), waktu_s. Jalur ada di env.jalur() bila sukses.
    terima: seperti rencanakan().
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
        q = _q(net, env.observasi())
        g = env.biaya
        for a in sah:
            a = int(a)
            poses, c, _ = env._calon[a]
            # Model dekat: langkah yang langsung tiba biayanya tepat. Model jauh
            # tidak: sisa biayanya (mendarat) justru yang diperkirakan Q-nya.
            if env.jenis == "dekat" and env._tiba(poses[-1][0], poses[-1][1], poses[-1][2]):
                f = g + c                                  # terminal: biaya tepat
            else:
                h = max(0.0, -float(q[a]) * SKALA_REWARD - c)
                f = g + c + bobot * h
            heapq.heappush(heap, (f, next(urut), i, a))

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
        if info["sukses"] and (terima is None or terima(env)):
            return {"sukses": True, "alasan": info["alasan"], "dicoba": dicoba, "mundur": pindah,
                    "waktu_s": time.perf_counter() - t0, "jarak_min": 0.0}
        if info["sukses"]:
            alasan = "semua pose serah gagal didaratkan"
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


def cari(net, env, metode=METODE_BAWAAN, batas_waktu=None, terima=None, mulai_ulang=None,
         bobot=BOBOT_TERBAIK):
    """Satu model, satu metode. env sudah di-reset. Kembalian dict rencanakan() + 'metode'.

    gabungan: terbaik-dulu (PORSI_WAKTU_TERBAIK dari batas waktu), lalu mundur
    bila gagal. mulai_ulang(): me-reset env ke keadaan awal yang sama - sel
    tertutup milik pencarian pertama tidak boleh menutup pencarian kedua.

    Model dekat (versi 6, kata RS) selalu memakai pencarian MUNDUR: episodenya
    satu langkah, jadi mundur = coba kata RS menurut urutan Q, yang menabrak
    dilewati, berhenti di kata pertama yang bebas.
    """
    t0 = time.perf_counter()
    if env.jenis == "dekat":
        h = rencanakan(net, env, mundur=True, anggaran=ANGGARAN_BAWAAN, batas_waktu=batas_waktu,
                       terima=terima)
        h["metode"] = "urutan Q"
        return h
    if metode == "mundur":
        h = rencanakan(net, env, mundur=True, anggaran=ANGGARAN_BAWAAN, batas_waktu=batas_waktu,
                       terima=terima)
        h["metode"] = "mundur"
        return h
    bt = None if batas_waktu is None else (PORSI_WAKTU_TERBAIK * batas_waktu if metode == "gabungan"
                                           else batas_waktu)
    h = rencanakan_terbaik(net, env, anggaran=ANGGARAN_TERBAIK, batas_waktu=bt, bobot=bobot,
                           terima=terima)
    h["metode"] = "terbaik"
    if h["sukses"] or metode == "terbaik":
        return h
    sisa = None if batas_waktu is None else batas_waktu - (time.perf_counter() - t0)
    if sisa is not None and sisa <= 0:
        return h
    if mulai_ulang is None or mulai_ulang() is None:
        return h
    hm = rencanakan(net, env, mundur=True, anggaran=ANGGARAN_BAWAAN, batas_waktu=sisa, terima=terima)
    hm["metode"] = "terbaik -> mundur"
    hm["dicoba"] += h["dicoba"]
    hm["waktu_s"] = time.perf_counter() - t0
    if not hm["sukses"]:
        hm["alasan"] = f"terbaik: {h['alasan']}; mundur: {hm['alasan']}"
    return hm


# Batas waktu satu percobaan pendaratan dari satu pose serah (model dekat), dan
# berapa pose serah yang boleh dicoba sebelum menyerah.
#
# MAKS_SERAH 6 -> 12 (2026-10-20, model ID 01 langkah jauh 55000 / dekat 95000):
#                            30 rute yang gagal      80 rute acak       waktu median (acak)
#   6, pose pertama          0/30                    73/80              205 ms
#   12, pose pertama         20/30                   76/80              151 ms
#   anggaran x3 + 2 s, 12    22/30                   -                  -
#   <=12 kandidat diurut g + V model dekat, coba 6: 10/30, 73/80 - urutan V tidak
#   membantu (model dekat buta rintangan, V-nya terlalu optimis di tempat sempit).
# Yang menentukan adalah BANYAKNYA pose serah yang dicoba, bukan anggaran per
# pencarian. Rute normal tidak melambat: biasanya berhasil di percobaan 1-2.
BATAS_DEKAT_S = 0.6
# MAKS_SERAH 12 -> 1000 (ver-1-4, model dekat memilih kata RS): satu percobaan
# pendaratan kini < 1 ms (rumus RS + cek tabrakan), bukan sampai 0,6 s, jadi
# model jauh boleh terus mencari pose yang kata RS-nya bebas - seperti Hybrid A*
# yang mencoba RS di setiap simpul. Yang membatasi tinggal batas waktu (2 s).
# 207 rute uji ID 01 (model jauh ver-1-3, model dekat v6 latih singkat):
#   12 -> tiba 94,7%; 120 -> 98,6%, median 4,1 ms; 1000 -> 10 dari 11 rute gagal
#   (pada jatah 12) diperbaiki.
MAKS_SERAH = 1000


def buat_cek_pendaratan(net_dekat, metode=METODE_BAWAAN, batas_waktu=BATAS_DEKAT_S, maks_coba=None):
    """(cek_serah, nilai_serah) untuk lingkungan jauh: PENDARATAN SUNGGUHAN model dekat.

    cek_serah(env): model dekat mencoba mendarat dari pose env (pencarian
    `metode`, paling lama batas_waktu detik); True bila berhasil tanpa tabrakan.
    Ini yang dipakai saat pengujian (rencanakan_dua), jadi lingkungan latih
    model jauh yang memakainya berhenti di pose yang SAMA dengan saat dipakai.
    nilai_serah(env): biaya jalur pendaratan yang baru ditemukan cek_serah di
    pose itu (dipanggil lingkungan tepat setelah cek_serah berhasil).
    maks_coba: jatah percobaan per episode (seperti MAKS_SERAH saat pengujian);
    sesudahnya cek mengembalikan None - episode jauh berakhir gagal.
    Satu lingkungan dekat per lingkungan jauh (disimpan di atributnya), berbagi
    objek perencana - tujuan & peta sama, jadi keadaan perencana tetap benar.
    """
    def cek(env):
        if maks_coba is not None and env.n_coba_serah > maks_coba:
            return None
        ed = getattr(env, "_env_dekat", None)
        if ed is None:
            ed = env._env_dekat = L.LingkunganRencana(env.arena, env.p, margin_cm=env.margin_cm,
                                                      planner=env.planner, jenis="dekat")
        mulai, d0, k0 = (env.x, env.y, env.yaw), env.d, env.k
        env._biaya_daratan = env.biaya_buntu
        obs, info = ed.reset(mulai, env.tujuan, env.omap, d0=d0, k0=k0)
        if obs is None:
            if info["alasan"] != "sudah di tujuan":
                return False
            env._biaya_daratan = 0.0
            return True
        h = cari(net_dekat, ed, metode, batas_waktu=batas_waktu,
                 mulai_ulang=lambda: ed.reset(mulai, env.tujuan, env.omap, d0=d0, k0=k0)[0])
        if h["sukses"]:
            env._biaya_daratan = ed.biaya
        return h["sukses"]

    def nilai(env):
        return float(getattr(env, "_biaya_daratan", env.biaya_buntu))

    return cek, nilai


def rencanakan_dua(net_jauh, net_dekat, env_jauh, env_dekat, awal, tujuan, omap=None,
                   metode=METODE_BAWAAN, batas_waktu=None, batas_dekat=BATAS_DEKAT_S, catat=None,
                   bobot=BOBOT_TERBAIK):
    """Model jauh sampai serah, lalu model dekat. Tanpa Reeds-Shepp, tanpa A*.

    Model jauh berhenti di dalam 60 cm bila TIDAK ADA HALANGAN untuk mendarat,
    dan melanjutkan selama masih ada. "Tidak ada halangan" diperiksa oleh
    model dekat sendiri: di setiap pose yang lolos saringan cepat lingkungan
    (di dalam 60 cm, peta Dijkstra bebas), model dekat mencoba mendarat
    (env_jauh.cek_serah). Berhasil tanpa tabrakan = berhenti di situ; gagal =
    model jauh lanjut memperkecil jarak. Paling banyak MAKS_SERAH percobaan.

    Kembalian dict:
      sukses, alasan, fase       'tiba' | 'gagal_jauh' (tidak ada pose yang lolos
                                 saringan sama sekali: masalah NAVIGASI) |
                                 'gagal_dekat' (ada, semua gagal didaratkan: PRESISI)
      jalur                      jalur gabungan (format plan()), [] bila gagal
      biaya                      biaya jalur jauh + dekat (sel)
      n_jauh_titik, n_serah      titik bagian jauh; percobaan pendaratan
      metode_jauh, metode_dekat, dicoba, waktu_s
      waktu_dekat_s              bagian waktu_s di dalam model dekat (semua percobaan
                                 pendaratan, termasuk yang gagal); sisanya model jauh
    catat: daftar; bila diberikan, tiap percobaan pendaratan ditambahkan sebagai
    dict(pose, d, k, sukses) - bahan DAgger (drl.dagger).
    """
    t0 = time.perf_counter()
    st = {"n_serah": 0, "dekat": None, "jalur_dekat": None, "biaya_dekat": 0.0, "t_dekat": 0.0}

    def sisa():
        return None if batas_waktu is None else max(0.0, batas_waktu - (time.perf_counter() - t0))

    def daratkan(mulai, d0, k0):
        obs, info = env_dekat.reset(mulai, tujuan, omap, d0=d0, k0=k0)
        if obs is None:
            if info["alasan"] == "sudah di tujuan":
                st.update(dekat={"sukses": True, "metode": "-", "dicoba": 0},
                          jalur_dekat=[(*mulai, d0 or 1)], biaya_dekat=0.0)
                return True
            return False
        bw = batas_dekat if sisa() is None else min(batas_dekat, sisa())
        h = cari(net_dekat, env_dekat, metode, batas_waktu=bw,
                 mulai_ulang=lambda: env_dekat.reset(mulai, tujuan, omap, d0=d0, k0=k0)[0])
        if h["sukses"]:
            st.update(dekat=h, jalur_dekat=env_dekat.jalur(), biaya_dekat=env_dekat.biaya)
        return h["sukses"]

    def cek_serah(e):
        """Tidak ada halangan = model dekat bisa mendarat dari pose ini."""
        if st["n_serah"] >= MAKS_SERAH or (sisa() is not None and sisa() <= 0):
            return False
        st["n_serah"] += 1
        t_d = time.perf_counter()
        ok = daratkan((e.x, e.y, e.yaw), e.d, e.k)
        st["t_dekat"] += time.perf_counter() - t_d
        if catat is not None:
            catat.append({"pose": (e.x, e.y, e.yaw), "d": e.d, "k": e.k, "sukses": ok})
        return ok

    hasil = {"sukses": False, "jalur": [], "biaya": 0.0, "n_jauh_titik": 0, "metode_jauh": "-",
             "metode_dekat": "-", "dicoba": 0}
    env_jauh.cek_serah = cek_serah
    try:
        obs, info = env_jauh.reset(awal, tujuan, omap)
        if obs is None and info["alasan"] == "sudah di radius dekat":
            hasil.update(sukses=True, fase="tiba", alasan="")
        elif obs is None:
            hasil.update(fase="gagal_dekat" if st["n_serah"] else "gagal_jauh", alasan=info["alasan"])
        else:
            hj = cari(net_jauh, env_jauh, metode, batas_waktu=sisa(),
                      mulai_ulang=lambda: env_jauh.reset(awal, tujuan, omap)[0], bobot=bobot)
            hasil.update(sukses=hj["sukses"], metode_jauh=hj["metode"], dicoba=hj["dicoba"],
                         alasan=hj["alasan"] if not hj["sukses"] else "")
            if hj["sukses"]:
                hasil.update(fase="tiba", n_jauh_titik=len(env_jauh.jalur()), biaya=env_jauh.biaya)
            else:
                hasil["fase"] = "gagal_dekat" if st["n_serah"] else "gagal_jauh"
    finally:
        env_jauh.cek_serah = None
    if hasil["sukses"]:
        jauh = env_jauh.jalur() if hasil["n_jauh_titik"] else []
        dekat = st["jalur_dekat"] or []
        hasil["jalur"] = jauh + dekat[1 if jauh else 0:]
        hasil["biaya"] += st["biaya_dekat"]
        hasil["metode_dekat"] = st["dekat"]["metode"]
        hasil["dicoba"] += st["dekat"].get("dicoba", 0)
    hasil.update(n_serah=st["n_serah"], waktu_s=time.perf_counter() - t0, waktu_dekat_s=st["t_dekat"])
    return hasil


# ------------------------------------------------------------ operasional
_MODEL = {}


def jalur_model(robot_id, jenis, dir_model=None):
    return os.path.join(dir_model or DIR_MODEL, f"id{robot_id:02d}_{jenis}.pt")


def _periksa_milik(meta, robot_id, jalur):
    if meta.get("robot") not in (None, robot_id):
        raise SystemExit(f"[!] {jalur} milik robot {meta['robot']:02d}, bukan {robot_id:02d}")
    periksa_profil(meta, konfig.profil(robot_id), jalur)


def _jalur_ada(robot_id, jenis, dir_model):
    jalur = jalur_model(robot_id, jenis, dir_model)
    if not os.path.exists(jalur):
        raise SystemExit(f"[!] Model DQfD {jenis} robot {robot_id:02d} belum ada: {jalur}\n"
                         f"    Latih dulu: python -m drl.latih --id {robot_id} --model dekat,jauh")
    return jalur


def muat_model(robot_id, jenis, dir_model=None):
    """Jaringan PyTorch model 'jauh' / 'dekat' robot ini (model_dqn/id0N_<jenis>.pt), di-cache, CPU.

    Untuk alat latih & analisis (dagger, visual, banding). Perencana operasional
    memakai muat_model_numpy. torch dibatasi SATU thread: thread tambahan justru
    berebut inti dengan loop kendali 20 Hz di navigasi.py.
    """
    jalur = _jalur_ada(robot_id, jenis, dir_model)
    if ("torch", jalur) not in _MODEL:
        import torch
        from drl.dqfd import muat_jaringan
        torch.set_num_threads(1)
        net, meta = muat_jaringan(jalur, jenis=jenis)
        _periksa_milik(meta, robot_id, jalur)
        _MODEL[("torch", jalur)] = net
    return _MODEL[("torch", jalur)]


def muat_model_numpy(robot_id, jenis, dir_model=None):
    """JaringanNumpy model 'jauh' / 'dekat' robot ini - bobot .pt yang sama, forward numpy.

    Dipakai DQfDPlanner (ver-1-5, perubahan 2). Lewat cache <model>.npz
    (jaringan_numpy.muat_dari_pt): tanpa torch selama .pt tidak berubah.
    """
    jalur = _jalur_ada(robot_id, jenis, dir_model)
    if ("numpy", jalur) not in _MODEL:
        net, meta = muat_dari_pt(jalur, jenis=jenis)
        _periksa_milik(meta, robot_id, jalur)
        _MODEL[("numpy", jalur)] = net
    return _MODEL[("numpy", jalur)]


class DQfDPlanner:
    """Pengganti HybridAStarPlanner untuk Perencana (rencana.py).

    Anggota yang dipakai Perencana sama: plan(omap, start, goal), last_stats,
    time_limit, v. Jalur yang dikembalikan berformat sama persis - daftar
    (x, y, yaw, arah) dalam sel - jadi pelacak, wasit, dan navigasi tidak
    tahu jalurnya dari mana.

    Dua model per robot (id0N_jauh.pt, id0N_dekat.pt), dijalankan lewat
    rencanakan_dua(). last_stats["rs_titik"] = jumlah titik bagian model dekat,
    supaya navigasi.py menandainya dengan warna bagian Reeds-Shepp.

    astar: HybridAStarPlanner untuk kendaraan & margin yang sama. Dipakai dua
    cara: SALINANNYA sebagai pustaka gerak lingkungan (keadaan _hcost milik
    salinan, tidak mengganggu aslinya), dan aslinya sebagai CADANGAN bila
    cadangan=True dan DQfD gagal. last_stats["perencana"] mencatat siapa yang
    akhirnya menghasilkan jalur.
    """

    def __init__(self, astar, arena, profil, margin_cm, cadangan=True,
                 anggaran=None, dir_model=None, gerbang_k=None, metode=METODE_BAWAAN,
                 bobot=BOBOT_TERBAIK, saring_serah=True):
        """Opsi varian (benchmark Tahap 1.1): bobot = w terbaik-dulu; metode "gabungan"
        = dengan cadangan backtracking, "terbaik" = tanpa; saring_serah False = model
        dekat mencoba mendarat di SETIAP pose < 60 cm, tanpa saringan peta Dijkstra."""
        if metode not in ("terbaik", "mundur", "gabungan"):
            raise ValueError(f"metode DQfD '{metode}' tidak dikenal")
        self.astar = astar
        self.metode = metode
        self.bobot = float(bobot)
        # Gerbang hanya bermakna bila ada cadangan; tanpa cadangan menolak
        # jalur yang sah hanya membuat robot diam.
        self.gerbang_k = (GERBANG_K if gerbang_k is None else gerbang_k) if cadangan else None
        self.v = astar.v
        self.time_limit = astar.time_limit
        self.cadangan = cadangan
        self.net_jauh = muat_model_numpy(profil.id, "jauh", dir_model)
        self.net_dekat = muat_model_numpy(profil.id, "dekat", dir_model)
        pustaka = copy.copy(astar)
        self.env_jauh = L.LingkunganRencana(arena, profil, margin_cm=margin_cm, planner=pustaka,
                                            jenis="jauh")
        self.env_jauh.saring_serah = bool(saring_serah)
        self.env_dekat = L.LingkunganRencana(arena, profil, margin_cm=margin_cm, planner=pustaka,
                                             jenis="dekat")
        self.last_stats = {}

    def _perkiraan(self, omap, start, goal):
        """-V(s0) x skala model yang memulai rute: perkiraan biaya total (gerbang mutu)."""
        # Tanpa percobaan pendaratan: di sini hanya perlu tahu model mana yang memulai.
        self.env_jauh.cek_serah = lambda e: True
        try:
            obs, info = self.env_jauh.reset(start, goal, omap)
        finally:
            self.env_jauh.cek_serah = None
        net = self.net_jauh
        if obs is None:
            obs, _ = self.env_dekat.reset(start, goal, omap)
            net = self.net_dekat
        if obs is None:
            return None
        _, v = _qv(net, obs)
        return max(1.0, -v * SKALA_REWARD)

    def plan(self, omap, start, goal):
        t0 = time.perf_counter()
        perkiraan = self._perkiraan(omap, start, goal) if self.gerbang_k is not None else None
        t_gerbang = time.perf_counter() - t0
        h = rencanakan_dua(self.net_jauh, self.net_dekat, self.env_jauh, self.env_dekat,
                           start, goal, omap, metode=self.metode, batas_waktu=self.time_limit,
                           bobot=self.bobot)
        # Rincian waktu (detik) per bagian, dijumlahkan Perencana.rencanakan().
        rincian = {"dqfd_jauh": h["waktu_s"] - h["waktu_dekat_s"], "dqfd_dekat": h["waktu_dekat_s"]}
        if perkiraan is not None:
            rincian["gerbang"] = t_gerbang
        if h["sukses"] and not h["jalur"]:
            self.last_stats = {"expansions": 0, "time": h["waktu_s"], "reason": "sudah di tujuan",
                               "perencana": "dqfd", "rincian_s": rincian}
            return []
        if h["sukses"]:
            rasio = h["biaya"] / perkiraan if perkiraan else None
            if self.gerbang_k is None or rasio is None or rasio <= self.gerbang_k:
                self.last_stats = {
                    "expansions": h["dicoba"], "time": time.perf_counter() - t0, "reason": "",
                    "perencana": f"dqfd (jauh {h['metode_jauh']}, dekat {h['metode_dekat']}, "
                                 f"serah dicoba {h['n_serah']})",
                    "rasio_perkiraan": rasio,
                    "rs_titik": len(h["jalur"]) - max(h["n_jauh_titik"], 1),
                    "rincian_s": rincian}
                return h["jalur"]
            h["alasan"] = f"ditolak gerbang mutu (biaya {rasio:.2f} x perkiraan)"
        if not self.cadangan:
            self.last_stats = {"expansions": h["dicoba"], "time": time.perf_counter() - t0,
                               "reason": f"DQfD ({h.get('fase', '')}): {h['alasan']}", "perencana": "dqfd",
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
