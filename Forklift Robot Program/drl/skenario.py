"""
skenario.py - pembangkit skenario + dataset demonstrasi Hybrid A* untuk DQfD (tahap 3).

Satu skenario = (peta, robot, margin, rintangan tambahan, awal, tujuan). Untuk
tiap skenario Hybrid A* dijalankan dengan parameter operasional
(konfig.PARAM_RENCANA), jalurnya diurai menjadi aksi dan diputar ulang di
lingkungan (adaptor_demo) - episode yang tidak identik dengan A* ditolak,
bukan disimpan.

SATU PENGECUALIAN: batas WAKTU A* (time_limit 2 s) dilepas; yang membatasi
hanya max_expansions (30000). Batas waktu membuat hasilnya bergantung beban
CPU - dengan 15 pekerja berebut inti, pencarian yang di robot selesai 1,5 s
ikut terpotong, dan dataset tidak bisa dibangkitkan ulang persis. Batas
ekspansi deterministik.

PETA
    Kalibrasi, Rumah2, Rumah   peta constraint asli
    sintetis:<seed>            peta acak meniru pola peta asli - dinding tepi,
                               blok persegi 12-60 cm, blok sudut, kadang dinding
                               sekat yang membentuk koridor. Supaya DQfD belajar
                               membaca sinar, bukan menghafal tiga peta.

VARIASI per skenario
    margin      8 cm (80%) atau cadangan 6 cm (20%) - dua-duanya dipakai Perencana
    rintangan   30%: badan robot lain parkir di pose acak (Perencana.peta_dengan)
    tujuan      30% heading bebas
    jarak       15%: 15-60 cm (penyesuaian parkir), sisanya 40-300 cm

KEGAGALAN A* disimpan terpisah sebagai SET UJI NEGATIF (tahap 1-2 menunjukkan
kegagalan di Rumah seluruhnya "tidak ada jalur", bukan batas waktu): DQfD harus
ikut buntu dengan cepat, bukan memaksakan jalur.

PEMBAGIAN UJI per tugas (satu tugas = satu peta, 20 skenario): setiap tugas
ke-10 masuk set uji. Untuk peta sintetis artinya petanya sendiri belum pernah
dilihat saat latihan.

KELUARAN (drl/data/)
    demo_id01.npz      transisi (obs, aksi, reward, obs_next, selesai, mask,
                       mask_next) + tabel episode (ep_*)
    negatif_id01.npz   skenario yang gagal di A*
    ringkasan_id01.json

Jalankan dari folder "Forklift Robot Program":
    python -m drl.skenario --id 1,2 --n 5000
    python -m drl.skenario --id 1 --n 200 --keluar drl/data/coba
    python -m drl.skenario --pratinjau          # gambar peta sintetis + jalur demo
    python -m drl.skenario --periksa            # integritas + rekonstruksi dari metadata
"""
import argparse
import json
import math
import os
import sys
import time
from collections import Counter, defaultdict

import cv2
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import arena as A                     # noqa: E402
import konfig                         # noqa: E402
import rencana as R                   # noqa: E402
from forklift_planner import ObstacleMap  # noqa: E402
from drl import lingkungan as L       # noqa: E402
from drl.adaptor_demo import putar_ulang, rencanakan_terekam, urai_aksi  # noqa: E402

PETA_ASLI = ("Kalibrasi", "Rumah2", "Rumah")
# Porsi episode SUKSES per jenis peta. Rumah paling besar: di situ A* paling
# lambat, jadi di situ pula DQfD paling berguna.
KOMPOSISI = {"Kalibrasi": 0.15, "Rumah2": 0.25, "Rumah": 0.30, "sintetis": 0.30}

SKENARIO_PER_TUGAS = 20
TUGAS_UJI_SETIAP = 10
PELUANG_MARGIN_CADANGAN = 0.2
PELUANG_RINTANGAN = 0.3
PELUANG_HEADING_BEBAS = 0.3
PELUANG_DEKAT = 0.15
JARAK_DEKAT_CM = (15.0, 60.0)
JARAK_JAUH_CM = (40.0, 300.0)
# Batas percobaan per jenis peta, kelipatan kuota. ID 02 di Rumah hanya ~24%
# pasangan acak yang punya jalur, jadi 20x masih longgar.
MAKS_PERCOBAAN_X = 20

DIR_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
_SEED_SINTETIS = 0x5117E715


# ------------------------------------------------------------ peta sintetis
class ArenaSintetis(A.Arena):
    """Arena dari occupancy grid buatan. Satuan & konversi sama dengan Arena.

    Tidak punya konteks tampilan (ctx) maupun zona - hanya yang dipakai
    perencana, lingkungan, pelacak, dan Perencana.peta_dengan.
    """

    def __init__(self, occ, nama, cm_per_cell=None):
        self.cm_per_cell = float(cm_per_cell or konfig.GRID_CM)
        self.ctx = None
        self.nama = nama
        self.zona = []
        self.asal = (0.0, 0.0)
        self.tinggi_sel, self.lebar_sel = occ.shape
        self.occ = occ
        self.omap = ObstacleMap(occ)


def _persegi(occ, cpc, cx, cy, w, h, sudut_deg):
    a = math.radians(sudut_deg)
    c, s = math.cos(a), math.sin(a)
    pts = [(cx + dx * c - dy * s, cy + dx * s + dy * c)
           for dx, dy in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2))]
    cv2.fillPoly(occ, [np.round(np.array(pts) / cpc).astype(np.int32)], 255)


def buat_peta_sintetis(seed, lebar_sel=None, tinggi_sel=None, cm_per_cell=None):
    """Peta acak deterministik dari seed, ukuran sama dengan peta asli."""
    if lebar_sel is None:
        ref = _arena("Kalibrasi")
        lebar_sel, tinggi_sel, cm_per_cell = ref.lebar_sel, ref.tinggi_sel, ref.cm_per_cell
    cpc = float(cm_per_cell or konfig.GRID_CM)
    rng = np.random.default_rng([_SEED_SINTETIS, seed])
    W, H = lebar_sel * cpc, tinggi_sel * cpc

    for _ in range(50):
        occ = np.zeros((tinggi_sel, lebar_sel), np.uint8)
        # Dinding tepi: tebal tiap ujung diacak sendiri, jadi sedikit miring -
        # seperti zona tepi peta asli yang digambar tangan.
        t = rng.uniform(7.0, 20.0, size=8)
        for poli in (((0, 0), (W, 0), (W, t[1]), (0, t[0])),
                     ((0, H), (W, H), (W, H - t[3]), (0, H - t[2])),
                     ((0, 0), (t[4], 0), (t[5], H), (0, H)),
                     ((W, 0), (W - t[6], 0), (W - t[7], H), (W, H))):
            cv2.fillPoly(occ, [np.round(np.array(poli) / cpc).astype(np.int32)], 255)
        # Blok sudut.
        for cx, cy in ((0, 0), (W, 0), (0, H), (W, H)):
            if rng.random() < 0.4:
                s = rng.uniform(20.0, 45.0)
                _persegi(occ, cpc, cx, cy, 2 * s, 2 * s, 0.0)
        # Dinding sekat: membentuk koridor dan ruang.
        for _ in range(rng.choice([0, 0, 1, 1, 2])):
            panjang, tebal = rng.uniform(60.0, 180.0), rng.uniform(7.0, 15.0)
            tegak = rng.random() < 0.5
            _persegi(occ, cpc, rng.uniform(0.2 * W, 0.8 * W), rng.uniform(0.2 * H, 0.8 * H),
                     tebal if tegak else panjang, panjang if tegak else tebal,
                     rng.normal(0.0, 2.0))
        # Blok: sebagian besar hampir sejajar sumbu, sebagian miring bebas.
        for _ in range(int(rng.integers(0, 10))):
            w, h = rng.uniform(12.0, 60.0, size=2)
            sudut = rng.uniform(0.0, 90.0) if rng.random() < 0.2 else rng.normal(0.0, 3.0)
            _persegi(occ, cpc, rng.uniform(0.1 * W, 0.9 * W), rng.uniform(0.1 * H, 0.9 * H),
                     w, h, sudut)
        terisi = (occ > 0).mean()
        if 0.08 <= terisi <= 0.45:
            return ArenaSintetis(occ, f"sintetis:{seed}", cpc)
    raise RuntimeError(f"peta sintetis seed {seed} tidak pernah masuk rentang kepadatan")


_CACHE_ARENA = {}


def _arena(spek):
    """'Rumah' atau 'sintetis:<seed>' -> Arena, di-cache per proses."""
    if spek not in _CACHE_ARENA:
        if spek.startswith("sintetis:"):
            _CACHE_ARENA[spek] = buat_peta_sintetis(int(spek.split(":", 1)[1]))
        else:
            _CACHE_ARENA[spek] = A.Arena(spek)
    return _CACHE_ARENA[spek]


# ------------------------------------------------------------------ sampling
def _pose_bebas(omap, v, rng, percobaan=2000):
    for _ in range(percobaan):
        x, y = rng.uniform(0, omap.w), rng.uniform(0, omap.h)
        yaw = rng.uniform(-math.pi, math.pi)
        if omap.pose_free(v, x, y, yaw):
            return x, y, yaw
    return None


def acak_pasangan(env, omap, rng):
    """(awal, tujuan) dalam sel, badan bebas dengan margin env. None bila tak ketemu."""
    s = env.arena.panjang_ke_sel
    lo, hi = JARAK_DEKAT_CM if rng.random() < PELUANG_DEKAT else JARAK_JAUH_CM
    lo, hi = s(lo), s(hi)
    for _ in range(200):
        awal = _pose_bebas(omap, env.v, rng)
        if awal is None:
            return None
        # Tujuan diambil di cincin jarak [lo, hi] dari awal, bukan seragam di
        # seluruh peta lalu disaring - untuk rentang dekat itu hampir selalu gagal.
        r = math.sqrt(rng.uniform(lo * lo, hi * hi))
        a = rng.uniform(-math.pi, math.pi)
        gx, gy = awal[0] + r * math.cos(a), awal[1] + r * math.sin(a)
        gyaw = rng.uniform(-math.pi, math.pi)
        if 0 <= gx < omap.w and 0 <= gy < omap.h and omap.pose_free(env.v, gx, gy, gyaw):
            if rng.random() < PELUANG_HEADING_BEBAS:
                gyaw = None
            return awal, (gx, gy, gyaw)
    return None


def acak_satu(arena, per, env, rng):
    """Satu skenario acak: margin, rintangan robot lain, lalu pasangan awal-tujuan.

    env: {margin_cm: LingkunganRencana}; per: Perencana arena ini (untuk
    peta_dengan). Kembalian (margin, omap, poli_cm atau None, (awal, tujuan)
    atau None). Dipakai pembangkit dataset DAN latihan sendiri DQfD, supaya
    sebaran skenario keduanya sama. Urutan pemanggilan rng jangan diubah -
    dataset yang sudah ada bergantung padanya.
    """
    cadangan = konfig.MARGIN_CADANGAN_CM[0]
    margin = cadangan if rng.random() < PELUANG_MARGIN_CADANGAN else konfig.SAFETY_MARGIN_CM
    e = env[margin]
    omap, poli = arena.omap, None
    if rng.random() < PELUANG_RINTANGAN:
        lain = _pose_bebas(arena.omap, e.v, rng)
        if lain is not None:
            xc, yc = arena.ke_cm(lain[0], lain[1])
            poli = np.asarray(A.jejak_badan(xc, yc, math.degrees(lain[2])), np.float64)
            omap = per.peta_dengan([poli])
    return margin, omap, poli, acak_pasangan(e, omap, rng)


# ---------------------------------------------------------------- pekerja
def kerja(tugas):
    """Satu tugas = satu peta, SKENARIO_PER_TUGAS skenario. Dijalankan di proses pekerja."""
    rid, spek, seed = tugas["id"], tugas["peta"], tugas["seed"]
    arena = _arena(spek)
    prof = konfig.profil(rid)
    per = R.Perencana(arena, prof)
    rng = np.random.default_rng([seed, rid])
    env = {m: L.LingkunganRencana(arena, prof, margin_cm=m)
           for m in (konfig.SAFETY_MARGIN_CM, *konfig.MARGIN_CADANGAN_CM)}
    for e in env.values():
        e.planner.time_limit = float("inf")      # lihat docstring modul

    hasil = {"tugas": tugas, "episode": [], "negatif": [], "galat": [], "waktu_s": 0.0}
    t_mulai = time.perf_counter()
    for i in range(SKENARIO_PER_TUGAS):
        margin, omap, poli, pasangan = acak_satu(arena, per, env, rng)
        if pasangan is None:
            continue
        e = env[margin]
        awal, tujuan = pasangan
        meta = {"peta": spek, "margin": margin, "awal": awal, "tujuan": tujuan,
                "rintangan": poli, "tugas": tugas["no"], "i": i, "uji": tugas["uji"]}

        t0 = time.perf_counter()
        jalur, rek = rencanakan_terekam(e.planner, omap, awal, tujuan)
        meta["waktu_astar"] = time.perf_counter() - t0
        if not jalur:
            meta["alasan"] = rek.stats.get("reason", "")
            meta["ekspansi"] = rek.stats.get("expansions", 0)
            hasil["negatif"].append(meta)
            continue
        try:
            aksi = urai_aksi(e.planner, jalur, rek.n_ekor)
        except ValueError as ex:
            hasil["galat"].append(f"{spek} #{i}: urai: {ex}")
            continue
        ep, galat = putar_ulang(e, awal, tujuan, aksi, omap)
        if galat is None and abs(e.biaya - rek.biaya) >= 1e-6:
            galat = f"biaya {e.biaya:.6f} != A* {rek.biaya:.6f}"
        if galat:
            hasil["galat"].append(f"{spek} #{i}: {galat}")
            continue
        meta.update(biaya=e.biaya, ep=ep, jalur=e.jalur() if tugas.get("jalur") else None)
        hasil["episode"].append(meta)
    hasil["waktu_s"] = time.perf_counter() - t_mulai
    return hasil


# ----------------------------------------------------------------- simpan
def _nan3(p):
    return [np.nan] * 3 if p is None else [p[0], p[1], np.nan if p[2] is None else p[2]]


def _kolom_meta(daftar):
    return {
        "peta": np.array([m["peta"] for m in daftar], dtype=np.str_),
        "margin": np.array([m["margin"] for m in daftar], np.float32),
        "awal": np.array([_nan3(m["awal"]) for m in daftar], np.float64).reshape(-1, 3),
        "tujuan": np.array([_nan3(m["tujuan"]) for m in daftar], np.float64).reshape(-1, 3),
        "rintangan": np.array([np.full((4, 2), np.nan) if m["rintangan"] is None else m["rintangan"]
                               for m in daftar], np.float64).reshape(-1, 4, 2),
        "uji": np.array([m["uji"] for m in daftar], bool),
        "waktu_astar": np.array([m["waktu_astar"] for m in daftar], np.float32),
    }


def simpan(rid, episode, negatif, dir_keluar):
    """Gabungkan episode menjadi larik transisi + tabel episode, lalu simpan."""
    episode = sorted(episode, key=lambda m: (m["tugas"], m["i"]))
    negatif = sorted(negatif, key=lambda m: (m["tugas"], m["i"]))
    obs, obs2, aksi, rew, sel, mask, mask2, ep_id, t_ke, mulai = [], [], [], [], [], [], [], [], [], []
    n = 0
    for k, m in enumerate(episode):
        ep = m["ep"]
        T = len(ep["aksi"])
        obs.append(ep["obs"][:-1])
        obs2.append(ep["obs"][1:])
        aksi.append(ep["aksi"])
        rew.append(ep["reward"])
        sel.append(ep["selesai"])
        mask.append(ep["mask"][:-1])
        mask2.append(ep["mask"][1:])
        ep_id.append(np.full(T, k, np.int32))
        t_ke.append(np.arange(T, dtype=np.int16))
        mulai.append(n)
        n += T

    meta = _kolom_meta(episode)
    os.makedirs(dir_keluar, exist_ok=True)
    jalur_demo = os.path.join(dir_keluar, f"demo_id{rid:02d}.npz")
    np.savez_compressed(
        jalur_demo,
        obs=np.concatenate(obs).astype(np.float32),
        obs_next=np.concatenate(obs2).astype(np.float32),
        aksi=np.concatenate(aksi).astype(np.int8),
        reward=np.concatenate(rew).astype(np.float32),
        selesai=np.concatenate(sel).astype(bool),
        mask=np.concatenate(mask).astype(bool),
        mask_next=np.concatenate(mask2).astype(bool),
        ep_id=np.concatenate(ep_id),
        t=np.concatenate(t_ke),
        ep_mulai=np.array(mulai, np.int64),
        ep_langkah=np.array([len(m["ep"]["aksi"]) for m in episode], np.int16),
        ep_biaya=np.array([m["biaya"] for m in episode], np.float64),
        **{f"ep_{k}": v for k, v in meta.items()},
    )
    jalur_neg = os.path.join(dir_keluar, f"negatif_id{rid:02d}.npz")
    if negatif:
        np.savez_compressed(
            jalur_neg,
            alasan=np.array([m["alasan"] for m in negatif], dtype=np.str_),
            ekspansi=np.array([m["ekspansi"] for m in negatif], np.int32),
            **_kolom_meta(negatif),
        )
    return jalur_demo, jalur_neg, n


# ------------------------------------------------------------- orkestrasi
def _jenis(spek):
    return "sintetis" if spek.startswith("sintetis:") else spek


def bangkitkan(ids, n_target, dir_keluar, seed, pekerja):
    """Bagikan tugas ke proses pekerja sampai kuota episode sukses tiap jenis peta terpenuhi."""
    import multiprocessing as mp

    kuota = {(rid, j): int(round(n_target * p)) for rid in ids for j, p in KOMPOSISI.items()}
    # Selama berjalan hanya untuk kemajuan & kapan berhenti; angka akhirnya
    # dihitung ulang secara deterministik di _pilih_deterministik().
    ditemukan = Counter()
    percobaan = Counter()
    per_tugas = defaultdict(dict)       # kunci -> {k: hasil kerja}
    no_tugas = Counter()
    t0 = time.time()

    def tugas_baru(rid, jenis):
        k = no_tugas[(rid, jenis)]
        no_tugas[(rid, jenis)] += 1
        # Nomor tugas unik lintas jenis, supaya urutan simpan stabil.
        no = k * 16 + list(KOMPOSISI).index(jenis)
        spek = f"sintetis:{seed * 100000 + rid * 10000 + k}" if jenis == "sintetis" else jenis
        return {"id": rid, "peta": spek, "seed": seed * 1000003 + no, "no": no, "k": k,
                "uji": k % TUGAS_UJI_SETIAP == TUGAS_UJI_SETIAP - 1}

    def perlu(kunci):
        return (ditemukan[kunci] < kuota[kunci]
                and percobaan[kunci] < MAKS_PERCOBAAN_X * max(kuota[kunci], 1))

    ctx = mp.get_context("spawn")
    with ctx.Pool(pekerja) as pool:
        berjalan = {}
        while True:
            # Isi antrean: tiap jenis yang belum penuh, sebanyak perkiraan
            # tugas yang masih dibutuhkan, maksimal 2x jumlah pekerja total.
            # Tugas per jenis selalu dikirim berurutan k = 0, 1, 2, ...
            for kunci in kuota:
                sedang = sum(1 for t, _ in berjalan.values() if (t["id"], _jenis(t["peta"])) == kunci)
                laju = ditemukan[kunci] / percobaan[kunci] if percobaan[kunci] else 0.5
                butuh = math.ceil((kuota[kunci] - ditemukan[kunci]) /
                                  max(0.05, laju) / SKENARIO_PER_TUGAS) if perlu(kunci) else 0
                while sedang < min(butuh, pekerja) and len(berjalan) < 2 * pekerja:
                    t = tugas_baru(*kunci)
                    berjalan[t["no"] * 10 + t["id"]] = (t, pool.apply_async(kerja, (t,)))
                    sedang += 1
            if not berjalan:
                break
            selesai = [k for k, (_, h) in berjalan.items() if h.ready()]
            if not selesai:
                time.sleep(0.05)
                continue
            for k in selesai:
                t, h = berjalan.pop(k)
                r = h.get()
                kunci = (t["id"], _jenis(t["peta"]))
                per_tugas[kunci][t["k"]] = r
                ditemukan[kunci] += len(r["episode"])
                percobaan[kunci] += len(r["episode"]) + len(r["negatif"])
            baris = "  ".join(f"ID{rid:02d} " + " ".join(
                f"{j[:4]} {min(ditemukan[(rid, j)], kuota[(rid, j)])}/{kuota[(rid, j)]}"
                for j in KOMPOSISI) for rid in ids)
            print(f"\r[{time.time() - t0:6.0f} s] {baris}", end="", flush=True)
    print()

    hasil, sukses, ditemukan, percobaan = _pilih_deterministik(per_tugas, kuota)

    for rid in ids:
        h = hasil[rid]
        jd, jn, n_trans = simpan(rid, h["episode"], h["negatif"], dir_keluar)
        ring = ringkas(rid, h, kuota, sukses, ditemukan, percobaan, n_trans, time.time() - t0)
        with open(os.path.join(dir_keluar, f"ringkasan_id{rid:02d}.json"), "w", encoding="utf-8") as f:
            json.dump(ring, f, indent=2, ensure_ascii=False)
        cetak_ringkasan(ring)
        print(f"    -> {jd}")
        if h["negatif"]:
            print(f"    -> {jn}")
    return hasil


def _pilih_deterministik(per_tugas, kuota):
    """Pilih episode yang disimpan tanpa bergantung urutan selesainya tugas.

    Tugas tiap jenis dikirim berurutan k = 0, 1, 2, ... dan semua yang
    terkirim ditunggu sampai selesai, jadi yang selesai selalu 0..K. Tugas
    mana yang SELESAI LEBIH DULU tergantung penjadwal OS - jadi yang disimpan
    adalah kuota episode pertama menurut (k, i), ditambah negatif dari tugas
    0..m, m = tugas terakhir yang dipakai. Tugas sesudah m dibuang seluruhnya,
    supaya jumlah negatif pun tidak bergantung berapa tugas sempat terkirim.
    Hanya waktu (jam dinding) yang tetap berbeda antar-jalan.

    Bila batas percobaan tercapai sebelum kuota, semua tugas dipakai - dan
    hanya dalam kasus itu hasilnya bisa bergantung penjadwal.
    """
    hasil = defaultdict(lambda: {"episode": [], "negatif": [], "galat": [], "waktu_s": 0.0})
    sukses, ditemukan, percobaan = Counter(), Counter(), Counter()
    for kunci, tugas in per_tugas.items():
        h = hasil[kunci[0]]
        diambil = 0
        for k in sorted(tugas):
            r = tugas[k]
            ruang = kuota[kunci] - diambil
            h["episode"] += r["episode"][:ruang]
            diambil += min(ruang, len(r["episode"]))
            h["negatif"] += r["negatif"]
            h["galat"] += r["galat"]
            h["waktu_s"] += r["waktu_s"]
            ditemukan[kunci] += len(r["episode"])
            percobaan[kunci] += len(r["episode"]) + len(r["negatif"])
            if diambil >= kuota[kunci]:
                break
        sukses[kunci] = diambil
    return hasil, sukses, ditemukan, percobaan


def ringkas(rid, h, kuota, sukses, ditemukan, percobaan, n_trans, dt):
    ep, neg = h["episode"], h["negatif"]
    per_jenis = {}
    for j in KOMPOSISI:
        e = [m for m in ep if _jenis(m["peta"]) == j]
        g = [m for m in neg if _jenis(m["peta"]) == j]
        lk = [len(m["ep"]["aksi"]) for m in e] or [0]
        per_jenis[j] = {
            "sukses": sukses[(rid, j)], "kuota": kuota[(rid, j)], "percobaan": percobaan[(rid, j)],
            "ditemukan": ditemukan[(rid, j)],
            "negatif": len(g), "alasan_negatif": dict(Counter(m["alasan"] for m in g)),
            "langkah_rata": float(np.mean(lk)), "langkah_maks": int(max(lk)),
            "astar_ms_rata_sukses": float(1000 * np.mean([m["waktu_astar"] for m in e])) if e else 0.0,
            "astar_ms_rata_gagal": float(1000 * np.mean([m["waktu_astar"] for m in g])) if g else 0.0,
        }
    semua = np.concatenate([m["ep"]["aksi"] for m in ep]) if ep else np.zeros(0, int)
    hist = np.bincount(semua, minlength=L.N_AKSI) if len(semua) else np.zeros(L.N_AKSI, int)
    return {
        "robot": rid, "episode": len(ep), "transisi": int(n_trans),
        "episode_uji": int(sum(m["uji"] for m in ep)), "negatif": len(neg),
        "galat_putar_ulang": h["galat"][:20], "jumlah_galat": len(h["galat"]),
        "waktu_dinding_s": dt, "waktu_cpu_s": h["waktu_s"],
        "per_jenis": per_jenis,
        "histogram_aksi": [int(x) for x in hist],
        "margin_cadangan": int(sum(m["margin"] < konfig.SAFETY_MARGIN_CM for m in ep)),
        "dengan_rintangan": int(sum(m["rintangan"] is not None for m in ep)),
        "heading_bebas": int(sum(m["tujuan"][2] is None for m in ep)),
    }


def cetak_ringkasan(r):
    print(f"\n=== ID {r['robot']:02d}: {r['episode']} episode ({r['episode_uji']} uji), "
          f"{r['transisi']} transisi, {r['negatif']} negatif, galat putar-ulang {r['jumlah_galat']}")
    print("    peta        sukses/kuota  percobaan  A* ketemu  langkah rata/maks  A* ms sukses/gagal  alasan gagal")
    for j, s in r["per_jenis"].items():
        laju = s["ditemukan"] / s["percobaan"] if s["percobaan"] else 0.0
        print(f"    {j:<10} {s['sukses']:>6}/{s['kuota']:<6} {s['percobaan']:>9}  {100 * laju:4.0f}%  "
              f"{s['langkah_rata']:8.1f} / {s['langkah_maks']:<4}  "
              f"{s['astar_ms_rata_sukses']:7.0f} / {s['astar_ms_rata_gagal']:<7.0f}  {s['alasan_negatif']}")
    h = np.array(r["histogram_aksi"], float)
    p = 100 * h / max(1.0, h.sum())
    print("    aksi (%)  maju  " + " ".join(f"{x:4.1f}" for x in p[:7]) +
          " | mundur " + " ".join(f"{x:4.1f}" for x in p[7:14]) + f" | SAMBUNG {p[14]:.1f}")
    print(f"    variasi: margin 6 cm {r['margin_cadangan']}, rintangan robot lain {r['dengan_rintangan']}, "
          f"heading bebas {r['heading_bebas']}")
    for g in r["galat_putar_ulang"][:5]:
        print(f"    [!] {g}")


# ---------------------------------------------------------------- periksa
def periksa(rid, dir_data, n_sampel=200, seed=0):
    """Periksa integritas dataset. True bila semua lulus.

    Konsistensi larik: aksi selalu lolos mask, reward negatif, obs_next[t] =
    obs[t+1] di dalam episode, 'selesai' hanya di langkah terakhir.
    Rekonstruksi: n_sampel episode diputar ulang HANYA dari metadata tersimpan
    (peta, margin, rintangan, awal, tujuan, aksi) - obs, reward, dan mask harus
    sama persis. Membuktikan dataset berdiri sendiri, tanpa perlu A* lagi.
    """
    with np.load(os.path.join(dir_data, f"demo_id{rid:02d}.npz")) as f:
        d = {k: f[k] for k in f.files}      # np.load terkompresi mendekompres ulang tiap akses
    prof = konfig.profil(rid)
    galat = []
    N, E = len(d["aksi"]), len(d["ep_mulai"])
    idx = np.arange(N)
    if not d["mask"][idx, d["aksi"]].all():
        galat.append("ada aksi demo yang ter-mask")
    if not (d["reward"] < 0).all():
        galat.append("ada reward >= 0")
    akhir = d["ep_mulai"] + d["ep_langkah"] - 1
    if not (d["selesai"][akhir].all() and d["selesai"].sum() == E):
        galat.append("'selesai' tidak tepat di langkah terakhir tiap episode")
    lanjut = np.setdiff1d(idx[:-1], akhir)
    if not np.array_equal(d["obs_next"][lanjut], d["obs"][lanjut + 1]):
        galat.append("obs_next tidak bersambung dengan obs berikutnya")
    if d["mask_next"][akhir].any():
        galat.append("mask_next langkah terakhir tidak kosong")

    rng = np.random.default_rng(seed)
    per_cache = {}
    for e in rng.choice(E, size=min(n_sampel, E), replace=False):
        spek, margin = str(d["ep_peta"][e]), float(d["ep_margin"][e])
        arena = _arena(spek)
        env = L.LingkunganRencana(arena, prof, margin_cm=margin)
        omap = arena.omap
        poli = d["ep_rintangan"][e]
        if not np.isnan(poli).any():
            if spek not in per_cache:
                per_cache[spek] = R.Perencana(arena, prof)
            omap = per_cache[spek].peta_dengan([poli])
        awal = tuple(d["ep_awal"][e])
        t = d["ep_tujuan"][e]
        tujuan = (t[0], t[1], None if np.isnan(t[2]) else t[2])
        s0, T = d["ep_mulai"][e], d["ep_langkah"][e]
        aksi = [int(a) for a in d["aksi"][s0:s0 + T]]
        ep, g = putar_ulang(env, awal, tujuan, aksi, omap)
        if g:
            galat.append(f"episode {e} ({spek}): {g}")
        elif not (np.array_equal(ep["obs"][:-1], d["obs"][s0:s0 + T])
                  and np.array_equal(ep["reward"].astype(np.float32), d["reward"][s0:s0 + T])
                  and np.array_equal(ep["mask"][:-1], d["mask"][s0:s0 + T])
                  and abs(env.biaya - d["ep_biaya"][e]) < 1e-9):
            galat.append(f"episode {e} ({spek}): hasil putar ulang berbeda dari tersimpan")

    uji = d["ep_uji"]
    print(f"[i] ID {rid:02d}: {E} episode ({uji.sum()} uji, {100 * uji.mean():.1f}%), {N} transisi, "
          f"{min(n_sampel, E)} episode diputar ulang dari metadata")
    for g in galat[:10]:
        print(f"    [!] {g}")
    print("    [OK] lulus" if not galat else f"    [!] {len(galat)} galat")
    return not galat


# --------------------------------------------------------------- pratinjau
def gambar_episode(arena, meta, jalur, v):
    """Occupancy + rintangan + jalur (hijau maju, oranye mundur) + badan awal/tujuan."""
    img = cv2.cvtColor(255 - arena.occ, cv2.COLOR_GRAY2BGR)
    if meta["rintangan"] is not None:
        cv2.fillPoly(img, [arena.sel_poligon(meta["rintangan"])], (160, 160, 160))
    for a, b in zip(jalur[:-1], jalur[1:]):
        warna = (0, 170, 0) if b[3] > 0 else (0, 140, 255)
        cv2.line(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), warna, 2, cv2.LINE_AA)
    awal, tuj = meta["awal"], meta["tujuan"]
    cv2.polylines(img, [v.footprint(*awal)], True, (255, 90, 0), 2)
    gyaw = tuj[2] if tuj[2] is not None else jalur[-1][2]
    cv2.polylines(img, [v.footprint(tuj[0], tuj[1], gyaw)], True, (0, 0, 220), 2)
    teks = f"{meta['peta']} m{meta['margin']:.0f} {len(meta['ep']['aksi'])} aksi"
    cv2.putText(img, teks, (22, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    return img


def pratinjau(rid, n_peta, seed, keluar):
    """Gambar n_peta peta sintetis, masing-masing dengan satu jalur demo."""
    gambar = []
    k = 0
    while len(gambar) < n_peta and k < 20 * n_peta:
        t = {"id": rid, "peta": f"sintetis:{seed * 100000 + rid * 10000 + k}", "seed": seed + k,
             "no": k, "uji": False, "jalur": True}
        k += 1
        r = kerja(t)
        if r["episode"]:
            m = max(r["episode"], key=lambda e: len(e["ep"]["aksi"]))
            arena = _arena(t["peta"])
            v = L.LingkunganRencana(arena, konfig.profil(rid), margin_cm=m["margin"]).v
            gambar.append(gambar_episode(arena, m, m["jalur"], v))
    kol = 3
    while len(gambar) % kol:
        gambar.append(np.full_like(gambar[0], 255))
    baris = [np.hstack(gambar[i:i + kol]) for i in range(0, len(gambar), kol)]
    os.makedirs(os.path.dirname(keluar) or ".", exist_ok=True)
    cv2.imwrite(keluar, np.vstack(baris))
    print(f"[OK] {len(gambar)} pratinjau -> {keluar}")


def main():
    ap = argparse.ArgumentParser(description="Dataset demonstrasi Hybrid A* untuk DQfD")
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--n", type=int, default=5000, help="episode sukses per robot")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--pekerja", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--keluar", default=DIR_DATA)
    ap.add_argument("--pratinjau", action="store_true", help="gambar 6 peta sintetis + jalur demo")
    ap.add_argument("--periksa", action="store_true", help="periksa integritas dataset di --keluar")
    args = ap.parse_args()
    ids = [int(t) for t in args.id.split(",") if t.strip()]

    if args.periksa:
        sys.exit(0 if all([periksa(rid, args.keluar) for rid in ids]) else 1)

    if args.pratinjau:
        for rid in ids:
            pratinjau(rid, 6, args.seed, os.path.join(args.keluar, f"pratinjau_id{rid:02d}.png"))
        return
    print(f"[i] {args.n} episode sukses per robot, komposisi {KOMPOSISI}, "
          f"{args.pekerja} pekerja, seed {args.seed}")
    bangkitkan(ids, args.n, args.keluar, args.seed, args.pekerja)


if __name__ == "__main__":
    main()
