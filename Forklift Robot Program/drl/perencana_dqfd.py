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
from drl.jaringan_numpy import SKALA_REWARD, muat_dari_pt, periksa_profil
from drl.lingkungan import MAKS_LANGKAH

DIR_MODEL = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model_dqn")

ANGGARAN_BAWAAN = 600
METODE_BAWAAN = "gabungan"
# w terbaik-dulu = w Hybrid A* (aturan ver-1-6: w sama untuk pembanding,
# demonstrator, dan DQfD).
BOBOT_TERBAIK = konfig.PARAM_RENCANA["h_weight"]
ANGGARAN_TERBAIK = 1200
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


def cari(net, env, metode=METODE_BAWAAN, batas_waktu=None, mulai_ulang=None, bobot=BOBOT_TERBAIK):
    """env sudah di-reset. Kembalian dict rencanakan() + 'metode'.

    gabungan: terbaik-dulu (PORSI_WAKTU_TERBAIK dari batas waktu), lalu mundur
    bila gagal. mulai_ulang(): me-reset env ke keadaan awal yang sama - sel
    tertutup milik pencarian pertama tidak boleh menutup pencarian kedua.
    """
    t0 = time.perf_counter()
    if metode == "mundur":
        h = rencanakan(net, env, mundur=True, anggaran=ANGGARAN_BAWAAN, batas_waktu=batas_waktu)
        h["metode"] = "mundur"
        return h
    bt = None if batas_waktu is None else (PORSI_WAKTU_TERBAIK * batas_waktu if metode == "gabungan"
                                           else batas_waktu)
    h = rencanakan_terbaik(net, env, anggaran=ANGGARAN_TERBAIK, batas_waktu=bt, bobot=bobot)
    h["metode"] = "terbaik"
    if h["sukses"] or metode == "terbaik":
        return h
    sisa = None if batas_waktu is None else batas_waktu - (time.perf_counter() - t0)
    if sisa is not None and sisa <= 0:
        return h
    if mulai_ulang is None or mulai_ulang() is None:
        return h
    hm = rencanakan(net, env, mundur=True, anggaran=ANGGARAN_BAWAAN, batas_waktu=sisa)
    hm["metode"] = "terbaik -> mundur"
    hm["dicoba"] += h["dicoba"]
    hm["waktu_s"] = time.perf_counter() - t0
    if not hm["sukses"]:
        hm["alasan"] = f"terbaik: {h['alasan']}; mundur: {hm['alasan']}"
    return hm


def rencanakan_dqfd(net, env, awal, tujuan, omap=None, metode=METODE_BAWAAN, batas_waktu=None,
                    bobot=BOBOT_TERBAIK):
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
    h = cari(net, env, metode, batas_waktu=sisa, bobot=bobot,
             mulai_ulang=lambda: env.reset(awal, tujuan, omap)[0])
    hasil.update(sukses=h["sukses"], alasan="" if h["sukses"] else h["alasan"], metode=h["metode"],
                 dicoba=h["dicoba"])
    if h["sukses"]:
        hasil.update(jalur=env.jalur(), biaya=env.biaya, n_titik_rs=env.n_titik_rs)
    hasil["waktu_s"] = time.perf_counter() - t0
    return hasil


# ------------------------------------------------------------ operasional
_MODEL = {}


def jalur_model(robot_id, dir_model=None):
    return os.path.join(dir_model or DIR_MODEL, f"id{robot_id:02d}_{L.JENIS}.pt")


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
                 dir_model=None, gerbang_k=None, metode=METODE_BAWAAN):
        if metode not in ("terbaik", "mundur", "gabungan"):
            raise ValueError(f"metode DQfD '{metode}' tidak dikenal")
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
        self.env = L.LingkunganRencana(arena, profil, margin_cm=margin_cm, planner=pustaka)
        self.last_stats = {}

    def plan(self, omap, start, goal):
        t0 = time.perf_counter()
        self.pustaka._t_rs = 0.0                 # waktu di dalam tembakan RS (HybridAStarBertanda)
        h = rencanakan_dqfd(self.net, self.env, start, goal, omap, metode=self.metode,
                            batas_waktu=self.time_limit)
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
                    "rs_titik": h["n_titik_rs"], "rincian_s": rincian}
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
