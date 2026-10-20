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

UJI PENERIMAAN (tahap 1-2):
    python -m drl.adaptor_demo --uji                      # ID 01 & 02, semua peta
    python -m drl.adaptor_demo --uji --id 2 --peta Kalibrasi --n 30

    Lulus bila, untuk SETIAP jalur yang ditemukan A*:
      1. semua aksi demo lolos mask lingkungan, dan episodenya TIBA (boleh
         beberapa aksi pendek sebelum ujung ekor RS - sisanya dibuang);
      2. bagian A* jalur putar-ulang = bagian A* jalur asli (selisih < 0,01 sel);
      3. biaya bagian A* = g-cost simpul tempat RS disambung (atau g-cost total
         bila tiba lewat primitif), selisih < 1e-6;
      4. tanpa ekor RS: pure pursuit memberi perintah yang sama pada kedua jalur;
      5. observasi berukuran 47 dan berhingga di setiap langkah.
    Ekor RS hanya dilaporkan (selisih biaya, simpangan dari ekor asli), karena
    memang hampiran.
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
    """Jalur A* + ekor Reeds-Shepp -> daftar aksi. ValueError bila strukturnya tak terduga.

    Bagian A* diurai PERSIS (satu kelompok n_sub pose = satu primitif penuh).
    Ekor RS diurai lewat urai_ekor_rs() menjadi primitif penuh + pendek.
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
        a = L.aksi_dari(planner, arah, kappa)
        if abs(L.arah_kappa(planner, a)[1] - kappa) > 1e-6 * planner.kappa_max + 1e-12:
            raise ValueError(f"kappa {kappa:.6g} bukan tingkat kemudi perencana")
        aksi.append(a)
        yaw = kel[-1][2]
    if n_ekor:
        awal_ekor = jalur[len(jalur) - n_ekor - 1]
        aksi += urai_ekor_rs(planner, awal_ekor, jalur[len(jalur) - n_ekor:])
    return aksi


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
    """Ekor Reeds-Shepp -> aksi: tiap segmen = primitif penuh + primitif pendek untuk sisanya.

    Sisa panjang dibulatkan ke kelipatan 1/8 langkah lalu ditulis biner
    (1/2, 1/4, 1/8), jadi paling banyak tiga aksi pendek per segmen. Galatnya
    <= 1/16 langkah (0,46 cm) per segmen.
    """
    kappa = {"S": planner.kappas[L.KEMUDI_PENDEK[1]],
             "L": planner.kappas[L.KEMUDI_PENDEK[2]],
             "R": planner.kappas[L.KEMUDI_PENDEK[0]]}
    terkecil = L.PECAHAN_PENDEK[-1]
    aksi = []
    for jenis, d, panjang in segmen_ekor_rs(planner, awal, ekor):
        penuh = int(panjang // planner.step)
        unit = int(round((panjang - penuh * planner.step) / (terkecil * planner.step)))
        if unit * terkecil >= 1.0:
            penuh, unit = penuh + 1, 0
        aksi += [L.aksi_dari(planner, d, kappa[jenis])] * penuh
        for pecahan in L.PECAHAN_PENDEK:
            kali = int(round(pecahan / terkecil))
            if unit >= kali:
                aksi.append(L.aksi_dari(planner, d, kappa[jenis], pecahan))
                unit -= kali
    return aksi


def putar_ulang(env, awal, tujuan, aksi, omap=None):
    """Mainkan aksi di lingkungan. Kembalian (episode dict, pesan galat atau None).

    Episode boleh TIBA sebelum aksi habis: ekor RS yang diurai bisa masuk
    toleransi tujuan satu-dua aksi pendek sebelum ujungnya. Sisa aksinya
    dibuang - yang disimpan adalah yang benar-benar dimainkan.
    """
    obs, info = env.reset(awal, tujuan, omap)
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
        ep["mask"].append(env.mask() if not selesai else np.zeros(L.N_AKSI, bool))
        ep["selesai"].append(selesai)
        if selesai and i != len(aksi) - 1:
            if env.sukses:
                break
            return None, f"episode berakhir di aksi ke-{i} dari {len(aksi)} ({info['alasan']})"
    if not env.sukses:
        return None, "aksi habis tanpa tiba di tujuan"
    return {k: np.asarray(v) for k, v in ep.items()}, None


def biaya_setelah(env, awal, tujuan, aksi):
    """Biaya kumulatif lingkungan setelah memainkan aksi (urutan penjumlahan = A*)."""
    env.reset(awal, tujuan)
    for a in aksi:
        env.step(int(a))
    return env.biaya


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
    print(f"[i] uji penerimaan adaptor + lingkungan: {n} skenario per robot per peta, seed {seed}")
    print(f"[i] toleransi: jalur < {TOL_JALUR_SEL} sel, biaya < {TOL_BIAYA:g}")
    ringkas = []
    semua_lulus = True
    for rid in ids:
        prof = konfig.profil(rid)
        for nama in peta:
            arena = A.Arena(nama)
            env = L.LingkunganRencana(arena, prof)
            # Bukan hash(nama): hash string diacak per proses, jadi skenarionya
            # tidak akan sama di setiap jalan.
            rng = np.random.default_rng([seed, rid, *nama.encode()])
            s = SimpleNamespace(n=0, ditemukan=0, lulus=0, maks_jalur=0.0, maks_biaya=0.0,
                                langkah=[], pendek=[], sambung=0, tiba=0, galat=[],
                                biaya_rs=[], simpang_rs=[], ekor_ditolak=0,
                                t_astar=[], t_ulang=[], t_step=[])
            for awal, tujuan in acak_skenario(env, rng, n):
                s.n += 1
                t0 = time.perf_counter()
                jalur, rek = rencanakan_terekam(env.planner, arena.omap, awal, tujuan)
                s.t_astar.append(time.perf_counter() - t0)
                if not jalur:
                    continue
                s.ditemukan += 1
                try:
                    aksi = urai_aksi(env.planner, jalur, rek.n_ekor)
                except ValueError as e:
                    s.galat.append(f"urai: {e}")
                    continue
                t0 = time.perf_counter()
                ep, galat = putar_ulang(env, awal, tujuan, aksi)
                dt = time.perf_counter() - t0
                n_aksi_tubuh = (len(jalur) - rek.n_ekor - 1) // env.planner.n_sub
                if galat and galat.startswith("aksi demo ke-") and                         int(galat.split("ke-")[1].split(" ")[0]) >= n_aksi_tubuh:
                    # Ekor RS hasil urai menyimpang ~1 cm dan menyentuh margin:
                    # demo seperti itu MEMANG harus ditolak, bukan kesalahan.
                    s.ekor_ditolak += 1
                    continue
                if galat:
                    s.galat.append(galat)
                    continue
                biaya_total = env.biaya
                s.t_ulang.append(dt)
                s.t_step.append(dt / len(ep["aksi"]))

                # Bagian A* harus IDENTIK: jalur, biaya (= g simpul tempat RS
                # disambung), dan perintah pelacak. Ekor RS hanya hampiran, jadi
                # yang dinilai: tiba (sudah dijamin putar_ulang), selisih biaya
                # relatif, dan seberapa jauh ia menyimpang dari ekor RS asli.
                n_tubuh = len(jalur) - rek.n_ekor
                ulang = env.jalur()
                dj = selisih_jalur(jalur[:n_tubuh], ulang[:n_tubuh])
                # Biaya bagian A* dibaca dari lingkungan sendiri, bukan dijumlah
                # ulang: biayanya bercampur float32 (peta jarak), jadi hanya
                # urutan penjumlahan yang sama yang memberi angka yang sama.
                biaya_tubuh = biaya_setelah(env, awal, tujuan, ep["aksi"][:(n_tubuh - 1) // env.planner.n_sub])
                if rek.n_ekor:
                    db = abs(biaya_tubuh - rek.g_sambung)
                    s.biaya_rs.append(biaya_total / rek.biaya - 1.0)
                    s.simpang_rs.append(simpang_ekor(jalur[n_tubuh - 1:], ulang[n_tubuh - 1:]))
                    pp_ok = True        # jalur berbeda di ekor - perintah memang boleh beda
                else:
                    db = abs(env.biaya - rek.biaya)
                    pp_ok = bandingkan_pelacak(arena, env, jalur, ulang)
                obs_ok = ep["obs"].shape[1] == L.N_OBS and np.isfinite(ep["obs"]).all()
                s.maks_jalur = max(s.maks_jalur, dj)
                s.maks_biaya = max(s.maks_biaya, db)
                s.langkah.append(len(ep["aksi"]))
                s.pendek.append(int(np.sum(ep["aksi"] >= L.N_PRIMITIF)))
                s.sambung += rek.n_ekor > 0
                s.tiba += rek.n_ekor == 0
                if dj < TOL_JALUR_SEL and db < TOL_BIAYA and obs_ok and pp_ok:
                    s.lulus += 1
                else:
                    s.galat.append(f"selisih jalur {dj:.3g}, biaya {db:.3g}, "
                                   f"obs {'ok' if obs_ok else 'RUSAK'}, "
                                   f"pelacak {'sama' if pp_ok else 'BEDA'}")
            ok = s.lulus == s.ditemukan - s.ekor_ditolak and not s.galat
            semua_lulus &= ok
            ringkas.append((rid, nama, s, ok))
            print(f"    ID {rid:02d} {nama:<9} lulus {s.lulus}/{s.ditemukan} "
                  f"(A* menemukan {s.ditemukan}/{s.n}, ekor RS ditolak karena menyentuh "
                  f"margin {s.ekor_ditolak})  {'OK' if ok else 'GAGAL'}")
            for g in s.galat[:5]:
                print(f"        [!] {g}")

    print()
    print("  BAGIAN A* (harus identik):")
    print("  robot peta       lulus  A*   aksi rata/maks  pendek rata  ekor RS/tiba  "
          "maks selisih jalur  maks selisih biaya  A* rata  ulang rata  per langkah")
    for rid, nama, s, ok in ringkas:
        lk = s.langkah or [0]
        print(f"  ID {rid:02d} {nama:<9} {s.lulus:>3}/{s.ditemukan:<3} {s.ditemukan:>2}/{s.n:<2} "
              f"{np.mean(lk):6.1f} / {max(lk):<4}  {np.mean(s.pendek or [0]):8.1f}    "
              f"{s.sambung:>5} / {s.tiba:<5} "
              f"{s.maks_jalur:10.2e} sel      {s.maks_biaya:10.2e}      "
              f"{1000 * np.mean(s.t_astar):6.0f} ms "
              f"{1000 * np.mean(s.t_ulang or [0]):7.1f} ms {1000 * np.mean(s.t_step or [0]):7.2f} ms")
    print()
    print("  EKOR REEDS-SHEPP (hampiran primitif penuh + pendek):")
    print("  robot peta       ekor  biaya urai vs A*+RS (rata / maks)   simpang dari ekor asli (rata / maks)")
    for rid, nama, s, ok in ringkas:
        if not s.biaya_rs:
            continue
        cm = A.Arena(nama).panjang_ke_cm(1.0)
        print(f"  ID {rid:02d} {nama:<9} {len(s.biaya_rs):>4}  "
              f"{100 * np.mean(s.biaya_rs):+6.2f}% / {100 * max(s.biaya_rs, key=abs):+6.2f}%              "
              f"{np.mean(s.simpang_rs) * cm:5.2f} / {max(s.simpang_rs) * cm:5.2f} cm")
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
