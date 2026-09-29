"""validasi_nyata.py - menilai model ruang-keadaan terhadap JALAN NYATA.

Bedanya dengan state_space.validasi(), dan ini bukan beda kecil:

    state_space.validasi()   model linear  vs  SumberSimulasi   (dua-duanya MODEL)
    validasi_nyata.py        model tak linear  vs  pose AprilTag (model vs ROBOT)

Yang pertama menjawab "apakah linearisasinya sah". Hanya yang kedua menjawab
"apakah modelnya cocok dengan robot" - dan sampai 2026-09-27 pertanyaan itu
belum pernah diuji sama sekali.

CARA UJINYA meniru cara pose.py memakai model di perangkat, supaya yang diukur
memang ketelitian yang benar-benar dipakai: mulai dari pose kamera + perintah
yang sedang dikirim, integrasikan model tak linear dengan perintah yang
BENAR-BENAR TEREKAM, lalu bandingkan ke pose kamera beberapa saat kemudian.
Pembanding dasarnya "pose beku" - apa yang terjadi kalau pose tidak diramalkan
sama sekali. Kalau model tidak mengalahkan pose beku, ia tidak layak dipakai.

Masukannya CSV dari "navigasi.py --log", dengan kolom:
    t,state,x_cm,y_cm,yaw_deg,speed_sps,steer_deg,deviasi_cm,margin_cm,terlihat,diramalkan
Log sebelum 2026-09-27 (termasuk cusp.csv) belum punya kolom terakhir; tetap
terbaca, tapi dengan peringatan batas nomor 2 di bawah.

    python validasi_nyata.py                      # cusp.csv
    python validasi_nyata.py jalan.csv
    python validasi_nyata.py jalan.csv --horizon 0.4
    python validasi_nyata.py --v0 perintah        # pose.py SEBELUM perbaikan

Ramalan di sini meniru pose.py SEKARANG: kecepatan awal diambil dari taksiran
ramp (SumberKamera.perintah), bukan dari perintah seketika. --v0 perintah
memutar ulang perilaku lama, supaya sebelum-sesudah bisa dibandingkan.

TIGA BATAS YANG TIDAK BISA DIHILANGKAN OLEH PROGRAM INI, jangan lupa saat
mengutip angkanya:

1. Acuannya pose kamera lewat konversi geometri yang SAMA dengan yang dipakai
   model (ke_poros_depan, tinggi_tag_cm). Galat sistematis di situ saling
   menghapus dan TIDAK akan tertangkap. Yang teruji ramalan inkremental, bukan
   ketelitian posisi absolut.

2. Kolom "terlihat" BUKAN penanda "diukur kamera". pose.py menyetel terlihat=True
   juga untuk pose RAMALAN; ia baru False setelah ramalan melewati RAMAL_MAKS_S.
   Pada log tanpa kolom "diramalkan", sebagian baris bisa saja pose ramalan -
   model sebagian dibandingkan dengan keluarannya sendiri, dan hasilnya condong
   TERLALU BAIK. Program ini memperingatkan bila kolom itu tidak ada. Log yang
   ditulis navigasi.py sejak 2026-09-27 punya kolom itu, dan terukur() membuang
   pose ramalan - batas ini hanya berlaku untuk log lama.

3. delta tidak terukur di mana pun, jadi tetapan waktu kemudi tau TIDAK ikut
   terkonfirmasi oleh ujian ini - hanya akibatnya pada lintasan yang terlihat.
"""
import argparse
import csv
import math
import os

import numpy as np

import konfig
import state_space as SS

# Lompatan waktu lebih dari ini berarti dua sesi berbeda, bukan satu lintasan.
# Mengintegrasikan model melewati lompatan itu akan menghasilkan galat raksasa
# yang tidak berarti apa-apa.
JEDA_SEGMEN_S = 0.3

# Toleransi pencocokan baris pembanding. Loop kendali tidak berjalan tepat 20 Hz,
# jadi baris pada t0 + horizon jarang ada persis; yang terdekat dipakai selama
# masih di dalam toleransi ini.
TOL_COCOK_S = 0.08

# Perintah gas dianggap "baru berubah" bila ragamnya dalam jendela ini melebihi
# TENANG_SPS. Inilah pemilah yang membelah hasilnya menjadi dua kelompok tajam.
LIHAT_BELAKANG_S = 1.5
TENANG_SPS = 200

KOLOM_WAJIB = ("t", "x_cm", "y_cm", "yaw_deg", "speed_sps", "steer_deg", "terlihat")


# ===========================================================================
# MEMUAT
# ===========================================================================
def muat(path):
    """CSV -> daftar segmen sinambung. Kembalikan (segmen, ada_kolom_diramalkan)."""
    with open(path) as fh:
        pembaca = csv.DictReader(fh)
        kurang = [k for k in KOLOM_WAJIB if k not in (pembaca.fieldnames or [])]
        if kurang:
            raise SystemExit(f"[!] {path} tidak punya kolom: {', '.join(kurang)}. "
                             f"Berkas ini bukan keluaran 'navigasi.py --log'.")
        ada_diramalkan = "diramalkan" in pembaca.fieldnames
        baris = []
        for r in pembaca:
            try:
                baris.append(dict(
                    t=float(r["t"]), state=r.get("state", ""),
                    x=float(r["x_cm"]), y=float(r["y_cm"]), yaw=float(r["yaw_deg"]),
                    sps=float(r["speed_sps"]), steer=float(r["steer_deg"]),
                    terlihat=r["terlihat"] == "1",
                    diramalkan=(r.get("diramalkan") == "1") if ada_diramalkan else False,
                ))
            except (ValueError, KeyError):
                continue          # baris rusak/terpotong saat program dimatikan
    if len(baris) < 50:
        raise SystemExit(f"[!] {path} hanya punya {len(baris)} baris terpakai.")

    baris.sort(key=lambda r: r["t"])
    segmen, kini = [], [baris[0]]
    for a, b in zip(baris, baris[1:]):
        if b["t"] - a["t"] > JEDA_SEGMEN_S:
            segmen.append(kini)
            kini = []
        kini.append(b)
    segmen.append(kini)
    return [s for s in segmen if len(s) > 20], ada_diramalkan


def terukur(r):
    """Baris ini layak dipakai sebagai acuan kebenaran?

    Pose ramalan TIDAK boleh - membandingkan model dengan ramalan model sendiri
    bukan validasi. Kalau kolom "diramalkan" tidak ada, ini hanya bisa menyaring
    yang sudah kedaluwarsa, dan itulah batas nomor 2 di docstring.
    """
    return r["terlihat"] and not r["diramalkan"]


# ===========================================================================
# LANTAI GALAT
# ===========================================================================
def derau_kamera(segmen):
    """Sebaran pose saat robot DIAM. Ramalan tidak mungkin lebih teliti dari ini.

    Tanpa angka ini, galat 0,2 cm terdengar seperti prestasi model - padahal bisa
    jadi ia sudah menyentuh lantai ketelitian kameranya.
    """
    dxy, dyaw = [], []
    for seg in segmen:
        for a, b in zip(seg, seg[1:]):
            if a["sps"] == 0 and b["sps"] == 0 and terukur(a) and terukur(b) \
                    and b["t"] - a["t"] < JEDA_SEGMEN_S:
                dxy.append(math.hypot(b["x"] - a["x"], b["y"] - a["y"]))
                dyaw.append(abs(b["yaw"] - a["yaw"]))
    return np.array(dxy), np.array(dyaw)


# ===========================================================================
# RAMALAN
# ===========================================================================
def perintah_pada(seg, t):
    """Perintah yang berlaku pada waktu t (zero-order hold, seperti aliran rc)."""
    pilih = seg[0]
    for r in seg:
        if r["t"] <= t:
            pilih = r
        else:
            break
    return pilih["steer"], pilih["sps"] * konfig.CM_PER_STEP


def jejak_v(seg):
    """Taksiran kecepatan motor di tiap baris, PERSIS seperti SumberKamera.perintah().

    Algoritmanya harus identik dengan pose.py - urutan zero-order hold (selama
    selang yang lewat, motor mengejar perintah SEBELUMNYA), batas dt 0,2 detik,
    dan nilai awal 0. Kalau keduanya menyimpang, yang tervalidasi di sini bukan
    kode yang berjalan di robot.
    """
    a = konfig.ACCEL_SPS2 * konfig.CM_PER_STEP
    v, out = 0.0, [0.0]
    for r0, r1 in zip(seg, seg[1:]):
        dt = min(r1["t"] - r0["t"], 0.2)
        v_cmd = r0["sps"] * konfig.CM_PER_STEP
        v += max(-a * dt, min(a * dt, v_cmd - v))
        out.append(v)
    return out


def ramal(seg, i, horizon, v0, h=0.01):
    """Integrasikan model tak linear dari baris i selama horizon detik -> (x, y).

    Keadaan awalnya SENGAJA sama dengan pose.py._mulai_ramal(). Kalau di sini
    dibuat lebih pintar daripada di perangkat, yang terukur bukan lagi
    ketelitian yang benar-benar dialami robot. v0 dipilih pemanggil: taksiran
    ramp (pose.py sekarang) atau perintah seketika (pose.py sebelum 2026-09-27).
    """
    r0 = seg[i]
    xi = np.array([r0["x"], r0["y"], math.radians(r0["yaw"]),
                   math.radians(r0["steer"]), v0])
    t, akhir = r0["t"], r0["t"] + horizon
    while t < akhir - 1e-9:
        langkah = min(h, akhir - t)
        d_deg, v_cmd = perintah_pada(seg, t)
        xi = xi + SS.turunan(xi, np.array([math.radians(d_deg), v_cmd])) * langkah
        t += langkah
    return float(xi[0]), float(xi[1])


def bandingkan(segmen, horizon, v0="ramp"):
    """Satu cuplikan per baris yang layak. -> daftar dict.

    v0 = "ramp"     : seperti pose.py sekarang
    v0 = "perintah" : seperti pose.py sebelum perbaikan, untuk pembanding
    """
    out = []
    for seg in segmen:
        v_jejak = jejak_v(seg) if v0 == "ramp" else None
        for i, r0 in enumerate(seg):
            if not terukur(r0) or abs(r0["sps"]) < konfig.SPS_MINIMUM / 6:
                continue          # robot praktis diam: tidak ada yang diramalkan
            target = r0["t"] + horizon
            cocok = None
            for r1 in seg[i + 1:]:
                if r1["t"] >= target - TOL_COCOK_S / 2:
                    cocok = r1
                    break
            if cocok is None or not terukur(cocok) \
                    or abs(cocok["t"] - target) > TOL_COCOK_S:
                continue
            belakang = [r["sps"] for r in seg[:i + 1]
                        if r0["t"] - r["t"] <= LIHAT_BELAKANG_S]
            if len(belakang) < 5:
                continue          # awal segmen: riwayat perintah belum cukup
            awal = v_jejak[i] if v0 == "ramp" else r0["sps"] * konfig.CM_PER_STEP
            hx, hy = ramal(seg, i, cocok["t"] - r0["t"], awal)
            out.append(dict(
                galat=math.hypot(hx - cocok["x"], hy - cocok["y"]),
                galat_beku=math.hypot(r0["x"] - cocok["x"], r0["y"] - cocok["y"]),
                laju=abs(r0["sps"]) * konfig.CM_PER_STEP,
                naik_lalu=max(belakang) - min(belakang),
            ))
    return out


# ===========================================================================
# LAPORAN
# ===========================================================================
def _ringkas(a):
    return (f"median {np.median(a):5.2f}  rata {a.mean():5.2f}  "
            f"p95 {np.percentile(a, 95):6.2f}  maks {a.max():6.2f}")


def lapor_umum(data, horizon):
    g = np.array([d["galat"] for d in data])
    b = np.array([d["galat_beku"] for d in data])
    print(f"\n  horizon {horizon:.2f} detik   (n={len(g)})")
    print(f"    model (diramalkan)  {_ringkas(g)}")
    print(f"    pose beku           {_ringkas(b)}")
    print(f"    model lebih baik pada {100.0 * float((g < b).mean()):.0f}% cuplikan")
    if np.median(g) >= np.median(b):
        print(f"    [!] model TIDAK mengalahkan pose beku - ada yang salah.")


def lapor_laju_x_ramp(data):
    """Tabel bagian 6a dokumen: laju x apakah perintah gas baru berubah.

    Dipilah dua arah karena satu arah saja MENYESATKAN: laju tinggi di data
    nyata hampir selalu bersamaan dengan perintah yang baru berubah, sehingga
    efek ramp terbaca seolah efek laju (dan dulu sempat disalahartikan sebagai
    selip).
    """
    print(f"\n  {'laju (cm/detik)':<18}{'perintah TUNAK':<34}{'perintah BARU BERUBAH':<34}")
    for lo, hi in ((0, 5), (5, 10), (10, 15), (15, 30)):
        sel = []
        for tunak in (True, False):
            a = np.array([d["galat"] for d in data
                          if lo <= d["laju"] < hi
                          and (d["naik_lalu"] < TENANG_SPS) == tunak])
            sel.append(f"median {np.median(a):5.2f}  p95 {np.percentile(a, 95):5.2f}  "
                       f"n={len(a):<5d}"
                       if len(a) >= 5 else f"(n={len(a)}, terlalu sedikit)")
        print(f"  {f'{lo}-{hi}':<18}{sel[0]:<34}{sel[1]:<34}")


def lapor_ramp(data):
    """Tabel bagian 6b dokumen: galat menurut perubahan perintah gas sebelumnya."""
    print(f"\n  perubahan perintah gas dalam {LIHAT_BELAKANG_S} detik SEBELUMNYA:")
    for lo, hi in ((0, 200), (200, 1000), (1000, 3000), (3000, 10 ** 9)):
        a = np.array([d["galat"] for d in data if lo <= d["naik_lalu"] < hi])
        if len(a) < 5:
            continue
        nama = f"{lo}-{hi} sps" if hi < 10 ** 9 else f"> {lo} sps"
        print(f"    {nama:<16} n={len(a):<5d} {_ringkas(a)}")

    lewat = [d["galat"] for d in data if d["galat"] > konfig.SAFETY_MARGIN_CM]
    if lewat:
        print(f"\n    [!] {len(lewat)} cuplikan galatnya MELEBIHI "
              f"SAFETY_MARGIN_CM = {konfig.SAFETY_MARGIN_CM:.1f} cm "
              f"(terburuk {max(lewat):.2f} cm).")
        print(f"        Satu ramp akselerasi bersamaan dengan tag kabur bisa "
              f"menghabiskan seluruh jarak aman.")


def main():
    ap = argparse.ArgumentParser(
        description="Validasi model ruang-keadaan terhadap jalan nyata")
    ap.add_argument("csv", nargs="?", default="cusp.csv",
                    help="CSV dari 'navigasi.py --log' (bawaan: cusp.csv)")
    ap.add_argument("--horizon", type=float, nargs="*", default=[0.2, 0.4, 1.0, 2.0],
                    help="horizon ramalan, detik (bawaan: 0.2 0.4 1.0 2.0)")
    ap.add_argument("--v0", choices=("ramp", "perintah"), default="ramp",
                    help="kecepatan awal ramalan: 'ramp' = pose.py sekarang (bawaan), "
                         "'perintah' = pose.py sebelum perbaikan 2026-09-27")
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        raise SystemExit(f"[!] {args.csv} tidak ada. Rekam dulu: "
                         f"python navigasi.py --peta <peta> --log {args.csv}")

    segmen, ada_diramalkan = muat(args.csv)
    n = sum(len(s) for s in segmen)
    lama = sum(s[-1]["t"] - s[0]["t"] for s in segmen)
    print(f"=== DATA: {args.csv} ===")
    print(f"  {len(segmen)} segmen sinambung, {n} baris, {lama:.1f} detik")
    for s in segmen:
        print(f"    {len(s):5d} baris  {s[-1]['t'] - s[0]['t']:6.1f} detik")

    # Model yang diuji adalah model SEKARANG, bukan model saat log direkam.
    # Itu memang yang diinginkan - tapi harus terlihat, supaya angka lama tidak
    # dikira dihasilkan parameter yang sama.
    print(f"\n  parameter yang diuji (dari konfig.py):")
    print(f"    CM_PER_STEP {konfig.CM_PER_STEP:.6f} | L {konfig.WHEELBASE_CM:.2f} cm | "
          f"tau {konfig.SIM_KEMUDI_TAU_S:.2f} s | accel {konfig.ACCEL_SPS2:.0f} sps/s^2")
    print(f"    kecepatan awal ramalan: "
          + ("taksiran RAMP (pose.py sekarang)" if args.v0 == "ramp"
             else "PERINTAH seketika (pose.py sebelum perbaikan)"))

    if not ada_diramalkan:
        print(f"\n  [!] {args.csv} tidak punya kolom 'diramalkan'.")
        print(f"      Kolom 'terlihat' juga bernilai 1 untuk pose RAMALAN, jadi "
              f"sebagian baris")
        print(f"      bisa saja keluaran model itu sendiri - hasil di bawah "
              f"condong TERLALU BAIK.")

    dxy, dyaw = derau_kamera(segmen)
    print(f"\n=== LANTAI GALAT: derau kamera saat robot DIAM ===")
    if len(dxy) >= 5:
        print(f"  geser posisi antar-frame  median {np.median(dxy):.2f} cm, "
              f"p95 {np.percentile(dxy, 95):.2f} cm, maks {dxy.max():.2f} cm")
        print(f"  geser yaw antar-frame     median {np.median(dyaw):.2f} deg, "
              f"p95 {np.percentile(dyaw, 95):.2f} deg")
        print(f"  -> ramalan tidak mungkin lebih teliti dari angka ini.")
    else:
        print(f"  tidak ada cuplikan robot diam; lantai galat tidak terukur.")

    print(f"\n=== RAMALAN MODEL vs POSE KAMERA ===")
    utama = None
    for hz in args.horizon:
        data = bandingkan(segmen, hz, args.v0)
        if not data:
            print(f"\n  horizon {hz:.2f} detik: tidak ada cuplikan yang cocok.")
            continue
        lapor_umum(data, hz)
        if utama is None or abs(hz - konfig.RAMAL_MAKS_S) < abs(utama[0] - konfig.RAMAL_MAKS_S):
            utama = (hz, data)

    if utama is None:
        return
    hz, data = utama
    print(f"\n=== DIPILAH, pada horizon {hz:.2f} detik "
          f"(paling dekat ke RAMAL_MAKS_S = {konfig.RAMAL_MAKS_S}) ===")
    lapor_laju_x_ramp(data)
    lapor_ramp(data)

    if args.v0 == "perintah":
        print(f"\n  Mode pembanding: beginilah pose.py SEBELUM perbaikan - v disetel")
        print(f"  ke kecepatan yang diperintahkan seketika, padahal motor masih")
        print(f"  menanjak. Jalankan tanpa --v0 untuk kode yang berjalan sekarang.")
        return

    # Hanya bila selisihnya memang besar. Sebelum 2026-09-27 pesan ini selalu
    # tercetak dan menunjuk ramp motor/accel - dua dugaan yang kemudian
    # terbantah: penyebab utamanya latensi kamera 0,68 s (dekoder 16 thread).
    tunak = np.array([d["galat"] for d in data if d["naik_lalu"] < TENANG_SPS])
    ubah = np.array([d["galat"] for d in data if d["naik_lalu"] >= TENANG_SPS])
    if len(tunak) >= 20 and len(ubah) >= 20:
        p_tunak, p_ubah = np.percentile(tunak, 95), np.percentile(ubah, 95)
        if p_ubah > 3.0 * max(p_tunak, 0.1):
            print(f"\n  [!] Kelompok BARU BERUBAH jauh lebih buruk dari TUNAK "
                  f"(p95 {p_ubah:.2f} vs {p_tunak:.2f} cm).")
            print(f"  Periksa dulu latensi kamera (identifikasi.py --mode gas, bagian")
            print(f"  TELEMETRI FIRMWARE): pose yang terlambat membuat perintah tampak")
            print(f"  bekerja lebih lambat dari kenyataannya. Log sebelum 2026-09-27")
            print(f"  direkam dengan kamera ~0,68 s.")
        else:
            print(f"\n  Kelompok TUNAK dan BARU BERUBAH setara (p95 {p_tunak:.2f} vs "
                  f"{p_ubah:.2f} cm): model mengikuti robot juga saat perintah berubah.")


if __name__ == "__main__":
    main()
