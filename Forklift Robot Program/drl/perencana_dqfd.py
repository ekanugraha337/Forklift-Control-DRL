"""
perencana_dqfd.py - merencanakan jalur dengan jaringan DQfD.

    rencanakan(net, env, mundur=True, anggaran=600, batas_waktu=None)
    rencanakan_terbaik(net, env, anggaran=600, batas_waktu=None, bobot=1.0)
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
"""
import copy
import heapq
import itertools
import math
import os
import time

import numpy as np
import torch

import konfig
from drl import lingkungan as L
from drl.jaringan import SKALA_REWARD
from drl.lingkungan import MAKS_LANGKAH

DIR_MODEL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model_dqn")
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


def rincian_fase(env, hasil):
    """Pemisah kegagalan & mutu pendaratan - dasar keputusan satu vs dua model.

    Batasnya radius DEKAT = analytic_dist perencana (60 cm): di dalamnya
    demonstrator memakai Reeds-Shepp. Kembalian dict:
      fase              'tiba' | 'gagal_jauh' (tidak pernah masuk radius dekat:
                        masalah NAVIGASI) | 'gagal_dekat' (masuk tapi tidak
                        mendarat: masalah PRESISI - tanda perlu model dekat)
      ganti_arah        titik balik maju/mundur di jalur (hanya bila tiba)
      ganti_arah_dekat  titik balik yang terjadi di dalam radius dekat
    hasil: kembalian rencanakan()/rencanakan_terbaik() (memuat jarak_min).
    """
    if hasil["sukses"]:
        g, gd = hitung_ganti_arah(env.jalur(), env.tujuan, env.planner.analytic_dist)
        return {"fase": "tiba", "ganti_arah": g, "ganti_arah_dekat": gd}
    fase = "gagal_dekat" if hasil.get("jarak_min", float("inf")) < env.planner.analytic_dist else "gagal_jauh"
    return {"fase": fase, "ganti_arah": None, "ganti_arah_dekat": None}


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
    with torch.no_grad():
        q = net(torch.as_tensor(env.observasi()[None]))[0].numpy()
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


def rencanakan_terbaik(net, env, anggaran=ANGGARAN_BAWAAN, batas_waktu=None, bobot=1.0):
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
        with torch.no_grad():
            q = net(torch.as_tensor(env.observasi()[None]))[0].numpy()
        g = env.biaya
        pl = env.planner
        for a in sah:
            a = int(a)
            poses, c, _ = env._calon[a]
            if pl._at_goal(poses[-1][0], poses[-1][1], poses[-1][2], env.tujuan):
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


# ------------------------------------------------------------ operasional
_MODEL = {}


def muat_model(robot_id, dir_model=None):
    """Jaringan robot ini dari model_dqn/id0N.pt, di-cache per proses, di CPU.

    torch dibatasi SATU thread: forward satu keadaan hanya 0,03 ms, dan thread
    tambahan justru berebut inti dengan loop kendali 20 Hz di navigasi.py.
    """
    jalur = os.path.join(dir_model or DIR_MODEL, f"id{robot_id:02d}.pt")
    if jalur not in _MODEL:
        if not os.path.exists(jalur):
            raise SystemExit(f"[!] Model DQfD robot {robot_id:02d} belum ada: {jalur}\n"
                             f"    Latih dulu: python -m drl.latih --id {robot_id}")
        from drl.dqfd import muat_jaringan, periksa_profil
        torch.set_num_threads(1)
        net, meta = muat_jaringan(jalur)
        if meta.get("robot") not in (None, robot_id):
            raise SystemExit(f"[!] {jalur} milik robot {meta['robot']:02d}, bukan {robot_id:02d}")
        periksa_profil(meta, konfig.profil(robot_id), jalur)
        _MODEL[jalur] = net
    return _MODEL[jalur]


class DQfDPlanner:
    """Pengganti HybridAStarPlanner untuk Perencana (rencana.py).

    Anggota yang dipakai Perencana sama: plan(omap, start, goal), last_stats,
    time_limit, v. Jalur yang dikembalikan berformat sama persis - daftar
    (x, y, yaw, arah) dalam sel - jadi pelacak, wasit, dan navigasi tidak
    tahu jalurnya dari mana.

    astar: HybridAStarPlanner untuk kendaraan & margin yang sama. Dipakai dua
    cara: SALINANNYA sebagai pustaka gerak lingkungan (keadaan _hcost milik
    salinan, tidak mengganggu aslinya), dan aslinya sebagai CADANGAN bila
    cadangan=True dan DQfD gagal. last_stats["perencana"] mencatat siapa yang
    akhirnya menghasilkan jalur.
    """

    def __init__(self, astar, arena, profil, margin_cm, cadangan=True,
                 anggaran=None, dir_model=None, gerbang_k=None, metode=METODE_BAWAAN):
        self.astar = astar
        self.metode = metode
        # Gerbang hanya bermakna bila ada cadangan; tanpa cadangan menolak
        # jalur yang sah hanya membuat robot diam.
        self.gerbang_k = (GERBANG_K if gerbang_k is None else gerbang_k) if cadangan else None
        self.v = astar.v
        self.time_limit = astar.time_limit
        self.cadangan = cadangan
        if metode not in ("terbaik", "mundur", "gabungan"):
            raise ValueError(f"metode DQfD '{metode}' tidak dikenal")
        # anggaran berlaku untuk metode tunggal; gabungan memakai bawaan masing-masing.
        self.anggaran = anggaran or (ANGGARAN_TERBAIK if metode == "terbaik" else ANGGARAN_BAWAAN)
        self.net = muat_model(profil.id, dir_model)
        self.env = L.LingkunganRencana(arena, profil, margin_cm=margin_cm, planner=copy.copy(astar))
        self.last_stats = {}

    def plan(self, omap, start, goal):
        t0 = time.perf_counter()
        obs, info = self.env.reset(start, goal, omap)
        if obs is None and info["alasan"] == "sudah di tujuan":
            self.last_stats = {"expansions": 0, "time": 0.0, "reason": "sudah di tujuan",
                               "perencana": "dqfd"}
            return []
        dq = {"sukses": False, "alasan": info["alasan"], "dicoba": 0, "mundur": 0, "waktu_s": 0.0}
        perkiraan = None
        if obs is not None:
            with torch.no_grad():
                _, akt = self.net(torch.as_tensor(obs[None]), aktivasi=True)
            perkiraan = max(1.0, -float(akt["v"][0, 0]) * SKALA_REWARD)
            if self.metode == "terbaik":
                dq = rencanakan_terbaik(self.net, self.env, anggaran=self.anggaran,
                                        batas_waktu=self.time_limit, bobot=BOBOT_TERBAIK)
                dq["metode"] = "terbaik"
            elif self.metode == "mundur":
                dq = rencanakan(self.net, self.env, mundur=True, anggaran=self.anggaran,
                                batas_waktu=self.time_limit)
                dq["metode"] = "mundur"
            else:
                dq = self._gabungan(omap, start, goal, t0)
        if dq["sukses"]:
            rasio = self.env.biaya / perkiraan
            if self.gerbang_k is None or rasio <= self.gerbang_k:
                self.last_stats = {"expansions": dq["dicoba"], "time": time.perf_counter() - t0,
                                   "reason": "", "perencana": f"dqfd ({dq['metode']})",
                                   "mundur": dq["mundur"], "rasio_perkiraan": rasio}
                return self.env.jalur()
            dq["alasan"] = f"ditolak gerbang mutu (biaya {rasio:.2f} x perkiraan)"
        if not self.cadangan:
            self.last_stats = {"expansions": dq["dicoba"], "time": time.perf_counter() - t0,
                               "reason": f"DQfD: {dq['alasan']}", "perencana": "dqfd"}
            return None
        self.astar.time_limit = self.time_limit
        jalur = self.astar.plan(omap, start, goal)
        return self._catat_cadangan(jalur, dq, t0)

    def _gabungan(self, omap, start, goal, t0):
        """Terbaik-dulu, lalu mundur bila gagal. Lingkungan di-reset di antaranya:
        sel tertutup milik pencarian pertama tidak boleh menutup pencarian kedua."""
        dq = rencanakan_terbaik(self.net, self.env, anggaran=ANGGARAN_TERBAIK,
                                batas_waktu=PORSI_WAKTU_TERBAIK * self.time_limit,
                                bobot=BOBOT_TERBAIK)
        dq["metode"] = "terbaik"
        if dq["sukses"]:
            return dq
        sisa = self.time_limit - (time.perf_counter() - t0)
        if sisa <= 0:
            return dq
        self.env.reset(start, goal, omap)
        dm = rencanakan(self.net, self.env, mundur=True, anggaran=ANGGARAN_BAWAAN, batas_waktu=sisa)
        dm["metode"] = "terbaik -> mundur"
        dm["dicoba"] += dq["dicoba"]
        if not dm["sukses"]:
            dm["alasan"] = f"terbaik: {dq['alasan']}; mundur: {dm['alasan']}"
        return dm

    def _catat_cadangan(self, jalur, dq, t0):
        self.last_stats = dict(self.astar.last_stats or {})
        self.last_stats.update(time=time.perf_counter() - t0, perencana="A* (cadangan)",
                               dqfd_gagal=dq["alasan"])
        return jalur
