"""
adaptor_demo.py - jalur Hybrid A* -> episode demonstrasi (s, a, r, s') untuk DQfD.

Karena lingkungan perencanaan deterministik dan memakai aritmetika A* sendiri,
aksi demonstrator tidak perlu ditebak - ia DIREKONSTRUKSI tepat dari jalur:

    jalur = [awal] + (n_sub pose per primitif) x N + (pose ekor Reeds-Shepp)
    tiap kelompok n_sub pose -> satu primitif: arah dari kolom arah,
                                kappa = delta-yaw / (arah x step), dibulatkan
    ekor Reeds-Shepp          -> per segmen: primitif penuh kemudi penuh/lurus
                                 + primitif pendek (1/2, 1/4, 1/8 langkah) untuk
                                 sisa panjangnya - HAMPIRAN, bukan persis
                                 (full DQfD: saat dipakai tidak ada Reeds-Shepp)

plan() tidak mengembalikan g-cost, jadi biaya A* direkam dari luar selama
plan() berjalan: _h dan _analytic dibungkus di tingkat OBJEK, dan heapq modul
forklift_planner ditukar sementara dengan perekam. forklift_planner.py sendiri
tidak disentuh.

DUA MODEL (ver-1-2): pisah_demo() memotong aksi di titik serah (masuk radius
60 cm) menjadi episode model jauh dan episode model dekat.

UJI PENERIMAAN:
    python -m drl.adaptor_demo --uji                      # ID 01 & 02, semua peta
    python -m drl.adaptor_demo --uji --id 2 --peta Kalibrasi --n 30

    Lulus bila, untuk SETIAP jalur yang ditemukan A* (lihat uji()):
      1. episode jauh lolos mask, selesai SERAH, dan jalurnya IDENTIK dengan
         awalan jalur A*;
      2. episode dekat TIBA dalam toleransi model dekat (1 cm & 2 deg), atau
         ditolak dengan alasan yang sah (menyentuh margin setelah diurai, atau
         A* tiba tanpa RS di toleransinya sendiri);
      3. observasi berukuran 47 dan berhingga.
    Bagian dekat dilaporkan: galat pose akhir, simpangan dari jalur A* + RS,
    selisih biaya.
"""
import argparse
import heapq
import math
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

if __package__ in (None, ""):
    # Dijalankan sebagai berkas (python drl/adaptor_demo.py), bukan modul.
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import arena as A                     # noqa: E402
import forklift_planner as FP         # noqa: E402
import jejak                          # noqa: E402
import konfig                         # noqa: E402
from drl import lingkungan as L       # noqa: E402

konfig.pasang_path_kamera()
import area_constraint as AC          # noqa: E402

# Bawaan: semua peta di Camera/peta_constraint/ (lihat main).
PETA_UJI = None
TOL_JALUR_SEL = 0.01
TOL_BIAYA = 1e-6


# ----------------------------------------------------------------- perekam
def rencanakan_terekam(planner, omap, awal, tujuan):
    """planner.plan() + g-cost simpul akhir dan ekor Reeds-Shepp-nya.

    Kembalian (jalur, rekaman). rekaman.biaya = g-cost total jalur yang
    dikembalikan; rekaman.n_ekor = banyak pose ekor RS di ujung jalur (0 bila
    tiba lewat primitif).

    g simpul diambil dua cara: untuk ekor RS langsung dari simpulnya (node[5],
    eksak); untuk tiba lewat primitif dari f - h_weight*h saat simpul didorong
    ke heap (galat pembulatan ~1e-13, jauh di bawah toleransi uji).
    """
    rek = SimpleNamespace(h=0.0, g={0: 0.0}, pop=None, rs=None)
    h_asli, analitik_asli = planner._h, planner._analytic

    def h(x, y, gx, gy):
        rek.h = h_asli(x, y, gx, gy)
        return rek.h

    def analitik(omap_, node, goal):
        r = analitik_asli(omap_, node, goal)
        if r is not None:
            rek.rs = (node[5], len(r[0]), r[1])
        return r

    def dorong(heap, item):
        rek.g[item[1]] = item[0] - planner.h_weight * rek.h
        heapq.heappush(heap, item)

    def ambil(heap):
        item = heapq.heappop(heap)
        rek.pop = item[1]
        return item

    planner._h, planner._analytic = h, analitik
    FP.heapq = SimpleNamespace(heappush=dorong, heappop=ambil)
    try:
        jalur = planner.plan(omap, awal, tujuan)
    finally:
        FP.heapq = heapq
        del planner._h, planner._analytic

    hasil = SimpleNamespace(biaya=None, n_ekor=0, g_sambung=None, stats=dict(planner.last_stats))
    if jalur:
        if rek.rs is not None:
            g, hasil.n_ekor, biaya_rs = rek.rs
            hasil.g_sambung = g
            hasil.biaya = g + biaya_rs
        else:
            hasil.biaya = rek.g[rek.pop]
    return jalur, hasil


def urai_aksi(planner, jalur, n_ekor):
    """Jalur A* + ekor Reeds-Shepp -> (aksi model jauh, aksi model dekat).

    Bagian A* diurai PERSIS menjadi primitif model jauh (satu kelompok n_sub
    pose = satu primitif). Ekor RS diurai menjadi segmen RS model dekat
    (urai_ekor_rs). ValueError bila strukturnya tak terduga.
    """
    n = planner.n_sub
    tubuh = jalur[1:len(jalur) - n_ekor]
    if len(tubuh) % n:
        raise ValueError(f"{len(tubuh)} pose primitif bukan kelipatan n_sub={n}")
    aksi = []
    yaw = jalur[0][2]
    for i in range(0, len(tubuh), n):
        kel = tubuh[i:i + n]
        arah = kel[0][3]
        if any(p[3] != arah for p in kel):
            raise ValueError(f"arah berubah di dalam primitif ke-{i // n}")
        kappa = (kel[-1][2] - yaw) / (arah * planner.step)
        a = L.aksi_dari(planner, arah, kappa, jenis="jauh")
        if abs(L.arah_kappa(planner, a, "jauh")[1] - kappa) > 1e-6 * planner.kappa_max + 1e-12:
            raise ValueError(f"kappa {kappa:.6g} bukan tingkat kemudi perencana")
        aksi.append(a)
        yaw = kel[-1][2]
    dekat = []
    if n_ekor:
        awal_ekor = jalur[len(jalur) - n_ekor - 1]
        dekat = urai_ekor_rs(planner, awal_ekor, jalur[len(jalur) - n_ekor:])
    return aksi, dekat


def segmen_ekor_rs(planner, awal, ekor):
    """Pose ekor RS (sample_rs_path) -> daftar segmen (jenis 'S'/'L'/'R', arah, panjang sel).

    sample_rs_path membagi tiap segmen menjadi sub-langkah sama panjang, jadi
    segmen bisa dibaca kembali dari perubahan yaw per sub-langkah: nol =
    lurus, selain itu busur dengan kelengkungan planner.kappa_max (Reeds-Shepp
    selalu memakai radius rencana). Segmen lurus pendekatan yang ditempel ke
    ujung RS ikut terbaca sebagai segmen 'S' biasa.
    """
    seg = []
    px, py, pyaw = awal[0], awal[1], awal[2]
    for x, y, yaw, d in ekor:
        dyaw = FP.wrap_angle(yaw - pyaw)
        if abs(dyaw) < 1e-9:
            jenis, panjang = "S", math.hypot(x - px, y - py)
        else:
            jenis = "L" if dyaw * d > 0 else "R"
            panjang = abs(dyaw) / planner.kappa_max
        if seg and seg[-1][0] == jenis and seg[-1][1] == d:
            seg[-1][2] += panjang
        else:
            seg.append([jenis, d, panjang])
        px, py, pyaw = x, y, yaw
    return [tuple(s) for s in seg]


def urai_ekor_rs(planner, awal, ekor):
    """Ekor Reeds-Shepp -> aksi model dekat (segmen RS).

    Tiap segmen = aksi panjang penuh sebanyak muatnya + aksi pendek untuk
    sisanya. Sisa dibulatkan ke kelipatan 1/16 langkah lalu ditulis biner
    (1/2, 1/4, 1/8, 1/16), jadi paling banyak empat aksi pendek per segmen;
    galatnya <= 1/32 langkah (0,23 cm) per segmen.
    """
    kappa = {"R": planner.kappas[L.KEMUDI_DEKAT[0]], "S": planner.kappas[L.KEMUDI_DEKAT[1]],
             "L": planner.kappas[L.KEMUDI_DEKAT[2]]}
    terkecil = L.PECAHAN_PENDEK[-1]
    aksi = []
    for jenis, d, panjang in segmen_ekor_rs(planner, awal, ekor):
        penuh = int(panjang // planner.step)
        unit = int(round((panjang - penuh * planner.step) / (terkecil * planner.step)))
        if unit * terkecil >= 1.0:
            penuh, unit = penuh + 1, 0
        aksi += [L.aksi_dari(planner, d, kappa[jenis], 1.0, "dekat")] * penuh
        for pecahan in L.PECAHAN_PENDEK:
            kali = int(round(pecahan / terkecil))
            if unit >= kali:
                aksi.append(L.aksi_dari(planner, d, kappa[jenis], pecahan, "dekat"))
                unit -= kali
    return aksi


def putar_ulang(env, awal, tujuan, aksi, omap=None, d0=0, k0=0.0):
    """Mainkan aksi di lingkungan. Kembalian (episode dict, pesan galat atau None).

    Episode boleh SELESAI SUKSES sebelum aksi habis: lingkungan jauh berhenti
    di titik serah (sisa aksinya milik model dekat), dan ekor RS yang diurai
    bisa masuk toleransi tujuan satu-dua aksi pendek sebelum ujungnya. Sisa
    aksinya dibuang - yang disimpan adalah yang benar-benar dimainkan.
    """
    obs, info = env.reset(awal, tujuan, omap, d0=d0, k0=k0)
    if obs is None:
        return None, f"reset ditolak: {info['alasan']}"
    ep = {"obs": [obs], "aksi": [], "reward": [], "mask": [env.mask()], "selesai": []}
    for i, a in enumerate(aksi):
        if not env.mask()[a]:
            return None, f"aksi demo ke-{i} ({a}) ter-mask"
        obs, r, selesai, info = env.step(a)
        ep["obs"].append(obs)
        ep["aksi"].append(a)
        ep["reward"].append(r)
        ep["mask"].append(env.mask() if not selesai else np.zeros(env.n_aksi, bool))
        ep["selesai"].append(selesai)
        if selesai and i != len(aksi) - 1:
            if env.sukses:
                break
            return None, f"episode berakhir di aksi ke-{i} dari {len(aksi)} ({info['alasan']})"
    if not env.sukses:
        return None, "aksi habis tanpa tiba di tujuan"
    return {k: np.asarray(v) for k, v in ep.items()}, None


# Kode alasan demo ditolak (pisah_demo -> "tolak"). Semua SAH, bukan galat:
TOLAK = {
    "jauh_margin": "bagian A* menyentuh margin (peta dengan robot lain)",
    "belum_bebas": "RS mulai sebelum peta Dijkstra bebas (RS memutari rintangan)",
    "tanpa_rs": "A* tiba tanpa ekor RS - tidak ada bagian model dekat",
    "dekat_margin": "ekor RS hasil urai menyentuh margin",
    "dekat_presisi": "ekor RS hasil urai tidak mendarat <= 1 cm & 2 deg",
}


def pisah_demo(e_jauh, e_dekat, awal, tujuan, aksi_jauh, aksi_dekat, omap=None):
    """Satu demo A* + RS -> episode model jauh dan episode model dekat.

    Model jauh memainkan primitif A*; lingkungannya (cek_serah bawaan =
    tembakan RS bebas) harus SERAH tepat setelah primitif terakhir - titik
    yang sama dengan saat Hybrid A* beralih ke RS. Model dekat memainkan ekor
    RS dari pose itu, dengan arah dan kelengkungan yang sedang berjalan. Start
    yang sudah boleh serah: hanya episode dekat.

    Kembalian dict: jauh / dekat (episode atau None), tolak (kode TOLAK atau
    None), galat (pesan galat struktur atau None), serah (x, y, yaw, d, k) atau
    None, obs_serah & mask_serah (observasi/mask model dekat di pose serah, untuk
    nilai akhir model jauh saat latih).
    """
    h = {"jauh": None, "dekat": None, "tolak": None, "galat": None, "serah": None,
         "obs_serah": None, "mask_serah": None}
    if not aksi_dekat:
        h["tolak"] = "tanpa_rs"
        return h
    obs, info = e_jauh.reset(awal, tujuan, omap)
    if obs is None and info["alasan"] == "sudah di radius dekat":
        if aksi_jauh:
            h["galat"] = "serah di pose awal, padahal demo masih punya primitif A*"
            return h
        mulai, d0, k0 = awal, 0, 0.0
    else:
        ep, galat = putar_ulang(e_jauh, awal, tujuan, aksi_jauh, omap)
        if galat:
            if galat_margin(galat):
                h["tolak"] = "jauh_margin"
            elif galat.startswith("aksi habis tanpa tiba"):
                h["tolak"] = "belum_bebas"
            else:
                h["galat"] = f"jauh: {galat}"
            return h
        if len(ep["aksi"]) != len(aksi_jauh):
            h["galat"] = (f"jauh serah setelah {len(ep['aksi'])} dari {len(aksi_jauh)} primitif - "
                          f"lebih awal dari demonstrator")
            return h
        h["jauh"] = ep
        mulai = (e_jauh.x, e_jauh.y, e_jauh.yaw)
        d0, k0 = e_jauh.d, e_jauh.k
        h["serah"] = (*mulai, d0, k0)
    obs, info = e_dekat.reset(mulai, tujuan, omap, d0=d0, k0=k0)
    if obs is not None:
        h["obs_serah"], h["mask_serah"] = obs, e_dekat.mask()
    ep, galat = putar_ulang(e_dekat, mulai, tujuan, aksi_dekat, omap, d0=d0, k0=k0)
    if galat:
        if galat_margin(galat):
            h["tolak"] = "dekat_margin"
        elif galat.startswith("aksi habis tanpa tiba"):
            h["tolak"] = "dekat_presisi"
        else:
            h["galat"] = f"dekat: {galat}"
    else:
        h["dekat"] = ep
    return h


def galat_margin(galat):
    """True bila galat adalah aksi yang ter-mask (menyentuh margin), bukan galat struktur."""
    return bool(galat) and galat.startswith("aksi demo ke-")


def simpang_ekor(asli, ulang):
    """Jarak terbesar (sel) dari pose ekor hasil urai ke polyline ekor RS asli."""
    a = np.asarray(asli, dtype=np.float64)[:, :2]
    terjauh = 0.0
    for p in np.asarray(ulang, dtype=np.float64)[:, :2]:
        ab = a[1:] - a[:-1]
        t = np.clip(np.einsum("ij,ij->i", p - a[:-1], ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-12), 0, 1)
        terjauh = max(terjauh, float(np.min(np.hypot(*(a[:-1] + t[:, None] * ab - p).T))))
    return terjauh


def selisih_jalur(j1, j2):
    """Selisih terbesar x/y (sel) dan yaw (rad) antara dua jalur, inf bila panjang beda."""
    if len(j1) != len(j2):
        return float("inf")
    if not j1:
        return 0.0
    a, b = np.asarray(j1, dtype=np.float64), np.asarray(j2, dtype=np.float64)
    return float(np.max(np.abs(a - b)))


def bandingkan_pelacak(arena, env, j_astar, j_ulang, langkah=6):
    """Umpankan pose yang sama ke pure pursuit pada kedua jalur. True = perintah identik."""
    t1 = jejak.buat("pursuit", j_astar, env.v, env.omap, arena)
    t2 = jejak.buat("pursuit", j_ulang, env.v, env.omap, arena)
    now = 0.0
    kolom = ("speed", "steer_deg", "dir", "seg", "done", "paused")
    for p in j_astar[::langkah]:
        now += konfig.RC_PERIODE_S
        c1, c2 = t1.update(p[0], p[1], p[2], now), t2.update(p[0], p[1], p[2], now)
        if any(c1.get(k) != c2.get(k) for k in kolom):
            return False
    return True


# -------------------------------------------------------------- skenario uji
def acak_skenario(env, rng, n, peluang_bebas=0.3, jarak_cm=(40.0, 250.0)):
    """n pasangan (awal, tujuan) dalam sel yang badannya bebas dengan margin penuh."""
    omap, v, arena = env.arena.omap, env.v, env.arena
    lo, hi = arena.panjang_ke_sel(jarak_cm[0]), arena.panjang_ke_sel(jarak_cm[1])

    def pose():
        while True:
            x, y = rng.uniform(0, omap.w), rng.uniform(0, omap.h)
            yaw = rng.uniform(-math.pi, math.pi)
            if omap.pose_free(v, x, y, yaw):
                return x, y, yaw

    out = []
    while len(out) < n:
        a, t = pose(), pose()
        if not lo <= math.hypot(t[0] - a[0], t[1] - a[1]) <= hi:
            continue
        if rng.random() < peluang_bebas:
            t = (t[0], t[1], None)
        out.append((a, t))
    return out


def uji(ids, peta, n, seed):
    """Uji penerimaan dua model: potong tiap demo A* + RS di titik serah.

    Lulus bila untuk setiap jalur A*:
      1. episode jauh (bila start belum boleh serah) lolos mask, SERAH tepat
         setelah primitif A* terakhir, dan jalurnya IDENTIK dengan bagian A*;
      2. episode dekat lolos mask dan TIBA dalam toleransi model dekat;
      3. atau demo ditolak dengan salah satu alasan sah di TOLAK;
      4. observasi berukuran n_obs jenisnya (47 / 47) dan berhingga.
    """
    print(f"[i] uji penerimaan dua model (jauh + dekat): {n} skenario per robot per peta, seed {seed}")
    print(f"[i] serah: di dalam {konfig.PARAM_RENCANA['analytic_dist']:.0f} cm DAN tidak ada halangan "
          f"(peta Dijkstra bebas atau < {L.RADIUS_SERAH_PAKSA_CM} cm, lalu tembakan RS bebas); toleransi dekat "
          f"{L.TOL_DEKAT_CM} cm & {L.TOL_DEKAT_DEG} deg, jalur jauh < {TOL_JALUR_SEL} sel")
    ringkas = []
    semua_lulus = True
    for rid in ids:
        prof = konfig.profil(rid)
        for nama in peta:
            arena = A.Arena(nama)
            e_jauh = L.LingkunganRencana(arena, prof, jenis="jauh")
            e_dekat = L.LingkunganRencana(arena, prof, jenis="dekat", planner=e_jauh.planner)
            pl = e_jauh.planner
            # Bukan hash(nama): hash string diacak per proses, jadi skenarionya
            # tidak akan sama di setiap jalan.
            rng = np.random.default_rng([seed, rid, *nama.encode()])
            s = SimpleNamespace(n=0, ditemukan=0, lulus=0, maks_jalur=0.0, galat=[],
                                n_jauh=[], n_dekat=[], pendek=[], hanya_dekat=0,
                                tolak={}, simpang=[], biaya=[],
                                galat_akhir=[], arah_akhir=[])
            cm = arena.panjang_ke_cm(1.0)
            for awal, tujuan in acak_skenario(e_jauh, rng, n):
                s.n += 1
                jalur, rek = rencanakan_terekam(pl, arena.omap, awal, tujuan)
                if not jalur:
                    continue
                s.ditemukan += 1
                try:
                    aksi_jauh, aksi_dekat = urai_aksi(pl, jalur, rek.n_ekor)
                except ValueError as e:
                    s.galat.append(f"urai: {e}")
                    continue
                h = pisah_demo(e_jauh, e_dekat, awal, tujuan, aksi_jauh, aksi_dekat)
                if h["galat"]:
                    s.galat.append(h["galat"])
                    continue
                if h["tolak"]:
                    s.tolak[h["tolak"]] = s.tolak.get(h["tolak"], 0) + 1
                    continue
                dj = 0.0
                if h["jauh"] is not None:
                    # Bagian jauh harus IDENTIK dengan bagian primitif A*.
                    ulang = e_jauh.jalur()
                    dj = selisih_jalur(jalur[:len(ulang)], ulang)
                    s.n_jauh.append(len(h["jauh"]["aksi"]))
                else:
                    s.hanya_dekat += 1
                ep = h["dekat"]
                s.n_dekat.append(len(ep["aksi"]))
                s.pendek.append(int(sum(L.aksi_info(pl, int(a), "dekat")[2] < 1.0 for a in ep["aksi"])))
                x, y, yaw = e_dekat.x, e_dekat.y, e_dekat.yaw
                s.galat_akhir.append(math.hypot(tujuan[0] - x, tujuan[1] - y) * cm)
                if tujuan[2] is not None:
                    s.arah_akhir.append(abs(math.degrees(FP.wrap_angle(yaw - tujuan[2]))))
                s.simpang.append(simpang_ekor(jalur, e_dekat.jalur()) * cm)
                biaya_dua = (e_jauh.biaya if h["jauh"] is not None else 0.0) + e_dekat.biaya
                s.biaya.append(biaya_dua / rek.biaya - 1.0)
                obs_ok = (ep["obs"].shape[1] == L.n_obs("dekat") and np.isfinite(ep["obs"]).all()
                          and (h["jauh"] is None or (h["jauh"]["obs"].shape[1] == L.n_obs("jauh")
                                                     and np.isfinite(h["jauh"]["obs"]).all())))
                s.maks_jalur = max(s.maks_jalur, dj)
                if dj < TOL_JALUR_SEL and obs_ok:
                    s.lulus += 1
                else:
                    s.galat.append(f"selisih jalur jauh {dj:.3g}, obs {'ok' if obs_ok else 'RUSAK'}")
            ok = not s.galat
            semua_lulus &= ok
            ringkas.append((rid, nama, s, ok))
            print(f"    ID {rid:02d} {nama:<9} lulus {s.lulus}/{s.ditemukan} (A* menemukan {s.ditemukan}/{s.n}; "
                  f"ditolak {s.tolak or '-'})  {'OK' if ok else 'GAGAL'}")
            for g in s.galat[:5]:
                print(f"        [!] {g}")

    print()
    print("  robot peta       lulus   aksi jauh / dekat (rata)  pendek  hanya-dekat  "
          "maks selisih jalur jauh   galat akhir posisi / arah (rata, maks)   simpang dari A*+RS   biaya vs A*+RS")
    for rid, nama, s, ok in ringkas:
        f = lambda v: (np.mean(v), max(v)) if v else (float("nan"), float("nan"))  # noqa: E731
        print(f"  ID {rid:02d} {nama:<9} {s.lulus:>3}/{s.ditemukan:<3}   "
              f"{np.mean(s.n_jauh or [0]):6.1f} / {np.mean(s.n_dekat or [0]):5.1f}           "
              f"{np.mean(s.pendek or [0]):5.1f}   {s.hanya_dekat:>5}        {s.maks_jalur:10.2e} sel          "
              f"{f(s.galat_akhir)[0]:4.2f} cm / {f(s.arah_akhir)[0]:4.2f} deg ({f(s.galat_akhir)[1]:4.2f}, "
              f"{f(s.arah_akhir)[1]:4.2f})          {f(s.simpang)[0]:4.2f} / {f(s.simpang)[1]:4.2f} cm   "
              f"{100 * f(s.biaya)[0]:+5.1f}%")
    print()
    print("[OK] SEMUA LULUS" if semua_lulus else "[!] ADA YANG GAGAL - lihat di atas")
    return semua_lulus


def main():
    ap = argparse.ArgumentParser(description="Adaptor demonstrasi Hybrid A* -> DQfD")
    ap.add_argument("--uji", action="store_true", help="jalankan uji penerimaan tahap 1-2")
    ap.add_argument("--id", default="1,2", help="robot, mis. 1 atau 1,2")
    ap.add_argument("--peta", default=None,
                    help="dipisah koma; bawaan semua peta di Camera/peta_constraint/")
    ap.add_argument("--n", type=int, default=15, help="skenario per robot per peta")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    if not args.uji:
        ap.print_help()
        return
    ids = [int(t) for t in args.id.split(",") if t.strip()]
    peta = ([t.strip() for t in args.peta.split(",") if t.strip()] if args.peta
            else AC.daftar_peta())
    sys.exit(0 if uji(ids, peta, args.n, args.seed) else 1)


if __name__ == "__main__":
    main()
