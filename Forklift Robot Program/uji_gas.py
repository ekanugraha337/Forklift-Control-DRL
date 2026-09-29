"""uji_gas.py - uji-mandiri uji tangga gas di identifikasi.py.

Parameter yang DIKETAHUI ditanam di simulator, lalu diperiksa apakah uji gas
menemukannya kembali. Tanpa robot, tanpa kamera.

    python uji_gas.py          ringkasan + pemeriksaan
    python uji_gas.py -v       + keluaran lengkap cocokkan_gas tiap skenario

Tabel "Uji tangga gas" di README dihasilkan dari sini. Jalankan ulang setiap
kali pasang_gas(), _s_ramp(), atau cocokkan_gas() diubah.

SKENARIO (semua dengan latensi kamera simulator: 25 ms + frame 40 ms + jitter):
  A  dasar             akselerasi 12000, tanpa tunda tambahan
  B  tunda tertanam    + 0,15 s antara perintah gas dan motor
  C  akselerasi rendah motor 4000 walau konfig 12000
  D  akselerasi melemah  a = 12000 x max(0,15, 1 - sps/9000)  -> tidak linear
  E  = A + derau kamera 0,3 cm
  F  = C + derau kamera 0,3 cm

Yang HARUS terjadi: B menaikkan tunda ~0,15 s TANPA mengubah akselerasi, dan C
menurunkan akselerasi ke ~4000 TANPA mengubah tunda. Kalau B terbaca sebagai
akselerasi rendah, estimatornya gagal persis di hal yang ia janjikan - dan
itulah yang terjadi pada rancangan pertamanya, yang memisahkan keduanya dalam
satu lintasan (korelasi +0,98).

Benih acak dikunci (7), jadi angkanya sama setiap kali dijalankan.

BAGIAN KEDUA - telemetri firmware. Perekamannya butuh MQTT sungguhan dan tidak
bisa diuji di sini; yang diuji adalah PENGURAIANNYA. Robot simulasi diberi tunda
gerak, lalu telemetri sintetis dibuat seolah tunda itu berada di satu tempat
tertentu, dan diperiksa apakah laporannya menunjuk tempat yang benar:
  jaringan   perintah tiba 0,9 s terlambat di firmware
  firmware   perintah tiba segera, stepper baru mulai 0,9 s kemudian
  balik      semua segera, tapi telemetri terlambat 0,9 s kembali ke PC
  kamera     firmware & stepper segera, gerak baru terlihat 0,9 s kemudian
  normal     tidak ada tunda besar
Selain kalimat laporannya, angka presisinya diperiksa: waktu stepper mulai
(dari kurva pos) harus tepat dalam 0,03 s, dan latensi kamera dalam rentang
latensi kamera simulator itu sendiri.

BAGIAN KETIGA - pencocokan kemudi uji tangga (identifikasi.pasang_kemudi).
Di robot nyata ia menemukan tau ~ 0 (tunda murni 0,41-0,44 s). Hasil itu baru
boleh dipercaya kalau pencocoknya terbukti SANGGUP melihat tau ketika tau ada:
  servo lamban   simulator bawaan, tunda 0,22 + tau 0,72 -> harus terbaca tau ~0,72
  servo cepat    tunda 0,35 + tau 0,02                   -> harus terbaca tau ~0

LOG SIMULASI TIDAK DITULIS KE logs_identifikasi/. Pelaksana menyimpan CSV tiap
lintasan ke sana, dan folder itu berisi data robot SUNGGUHAN - mencampurnya
dengan data simulasi membuat keduanya tidak bisa dibedakan lagi. Di sini
Pelaksana.folder diarahkan ke folder sementara.
"""
import argparse
import bisect
import contextlib
import csv
import glob
import io
import os
import shutil
import sys
import tempfile

import numpy as np

import identifikasi as I
import konfig
import robot as RB

TUNDA_TAMBAH = 0.15


def pelaksana(derau, folder):
    a = argparse.Namespace(peta="Kalibrasi", simulasi=True, derau=derau, derau_deg=0.0,
                           url=None, host=None, sps_busur=4000, ulang=1)
    with contextlib.redirect_stdout(io.StringIO()):
        p = I.Pelaksana(a)
    p.folder = folder
    return p


def tanam_tunda(sim, D):
    """Motor baru menerima perintah gas D detik setelah dikirim."""
    t_hist, v_hist = [-1e9], [0.0]
    asli_p, asli_g = sim.perintah, sim._gas

    def perintah(speed, steer):
        t_hist.append(sim.waktu)
        v_hist.append(float(speed))
        asli_p(speed, steer)

    def gas(dt):
        i = bisect.bisect_right(t_hist, sim.waktu - D) - 1
        simpan = sim.speed_cmd
        sim.speed_cmd = v_hist[i]
        asli_g(dt)
        sim.speed_cmd = simpan

    sim.perintah, sim._gas = perintah, gas


def tanam_melemah(sim, A0=12000.0, vmax=9000.0):
    """Akselerasi turun seiring laju - bentuk kasar torsi stepper."""
    asli_g = sim._gas

    def gas(dt):
        sim.accel = A0 * max(0.15, 1.0 - abs(sim.speed_sps) / vmax)
        asli_g(dt)

    sim._gas = gas


def jalankan(nama, folder, derau=0.0, siapkan=None):
    np.random.seed(7)
    p = pelaksana(derau, folder)
    if siapkan:
        siapkan(p.sim)
    with contextlib.redirect_stdout(io.StringIO()):
        rek = p.gas()
    hasil = [h for h in (I.pasang_gas(r) for r in rek) if h is not None]
    teks = io.StringIO()
    with contextlib.redirect_stdout(teks):
        I.cocokkan_gas(rek)
    out = teks.getvalue()

    baris = {"nama": nama, "n_lintasan": len(hasil)}
    if len(hasil) >= 3:
        for arah, kunci, lompat in (("naik", "L_naik", [h["target"] for h in hasil]),
                                    ("turun", "L_turun",
                                     [h["target"] - h["rendah"] for h in hasil])):
            (td, kem), (td_se, kem_se), chi2, dof = I._garis(
                lompat, [h[kunci] for h in hasil], [h[kunci + "_se"] for h in hasil])
            A = 1 / (2 * kem) if kem > 0 else float("nan")
            baris[arah] = (td, td_se, A, A * kem_se / kem if kem > 0 else float("nan"),
                           chi2 / max(dof, 1))
    baris["tidak_linear"] = "TIDAK linear" in out
    baris["autokor_maks"] = max(h["autokor"] for h in hasil) if hasil else float("nan")
    baris["rho"] = [h["rho"] for h in hasil]
    return baris, out


def telemetri_sintetis(peristiwa, t_akhir, d_net, d_fw, d_balik, basi=4000):
    """Telemetri 4 Hz seperti netPublishTelemetry(), dengan tunda di tempat tertentu.

    d_net    perintah dikirim -> tiba di firmware (tgt_sps berubah)
    d_fw     tiba di firmware -> stepper mulai menanjak (sps berubah)
    d_balik  firmware menerbitkan telemetri -> telemetri tiba di PC
    basi     tgt_sps sisa perintah sebelumnya (driveRc tidak menolkannya)
    """
    tiba = [(t + d_net, lv) for t, lv in peristiwa]
    a = konfig.ACCEL_SPS2
    h = 0.005
    tt = np.arange(-0.5, t_akhir, h)
    v = np.zeros_like(tt)
    for i in range(1, len(tt)):
        tuju = 0.0
        for ta, lv in tiba:
            if ta + d_fw <= tt[i]:
                tuju = lv
        v[i] = v[i - 1] + np.clip(tuju - v[i - 1], -a * h, a * h)
    langkah = 5000.0 + np.cumsum(v) * h          # pos: langkah yang sudah keluar
    out = []
    tau = -0.5 + 0.07                       # fase terbit sembarang
    while tau < t_akhir:
        tgt = basi
        for ta, lv in tiba:
            if ta <= tau:
                tgt = lv
        sps = int(round(float(np.interp(tau, tt, v))))
        pos = int(round(float(np.interp(tau, tt, langkah))))
        out.append((tau + d_balik, {"tgt_sps": tgt, "sps": sps, "pos": pos, "rc": 1,
                                    "dir": 1, "drv": "RC"}))
        tau += 0.25
    return out


def uji_telemetri(cek):
    print("\n--- telemetri: perekaman di robot.py ---")
    r = RB.Robot()

    class Pesan:
        topic = konfig.TOPIK_TELEMETRI
        payload = b'{"tgt_sps": 2100, "sps": 0}'

    r._on_message(None, None, Pesan)              # belum merekam: tidak disimpan
    r.mulai_rekam_telemetri()
    r._on_message(None, None, Pesan)
    r._on_message(None, None, Pesan)
    rek = r.selesai_rekam_telemetri()
    cek("hanya pesan SELAMA merekam yang disimpan (2)", len(rek) == 2)
    cek("isi dan waktu terima tersimpan",
        rek[0][1].get("tgt_sps") == 2100 and rek[0][0] > 0)
    cek("setelah selesai, perekaman berhenti", r._rekam_telemetri is None)

    print("\n--- telemetri: penguraian menunjuk tempat tunda yang benar ---")
    skenario = [
        # nama,      d_net, d_fw, d_balik, tunda gerak robot, kalimat yang HARUS muncul
        ("jaringan", 0.9, 0.0, 0.02, 0.9, "Firmware baru MENERIMA perintah"),
        ("firmware", 0.02, 0.9, 0.02, 0.92, "stepper baru MULAI"),
        ("balik", 0.02, 0.0, 0.9, 0.02, "SEBELUM langkahnya tercatat"),
        ("kamera", 0.02, 0.0, 0.02, 0.9, "KAMERA terlambat"),
        ("normal", 0.02, 0.0, 0.02, 0.02, "Tidak ada tunda besar"),
    ]
    # Latensi kamera simulator sendiri (25 ms + umur frame + jitter): 0,02-0,09 s.
    KAM = (0.02, 0.09)
    folder_induk = tempfile.mkdtemp(prefix="uji_telemetri_")
    try:
        for nama, d_net, d_fw, d_balik, d_gerak, kalimat in skenario:
            folder = os.path.join(folder_induk, nama)
            os.makedirs(folder)
            np.random.seed(7)
            p = pelaksana(0.0, folder)
            tanam_tunda(p.sim, d_gerak)
            with contextlib.redirect_stdout(io.StringIO()):
                p.gas()
            for path in glob.glob(os.path.join(folder, "gas_*_perintah.csv")):
                with open(path) as fh:
                    per = [(float(x["t"]), int(x["sps"])) for x in csv.DictReader(fh)]
                with open(path.replace("_perintah", "")) as fh:
                    t_akhir = float(list(csv.DictReader(fh))[-1]["t"]) + 1.2
                tl = telemetri_sintetis(per, t_akhir, d_net, d_fw, d_balik)
                with open(path.replace("_perintah", "_telemetri"), "w") as fh:
                    fh.write(",".join(("t",) + I.KOLOM_TELEMETRI) + "\n")
                    for t, d in tl:
                        fh.write(f"{t:.3f}," + ",".join(str(d[k]) for k in I.KOLOM_TELEMETRI) + "\n")

            rek, _ = I.muat_gas(folder)
            teks = io.StringIO()
            with contextlib.redirect_stdout(teks):
                I.cocokkan_gas(rek)
            out = teks.getvalue()
            u = [I.urai_telemetri(x) for x in rek]
            # "diterima" = bolak-balik; bisa terlambat sampai satu selang 0,25 s
            harap = d_net + d_balik
            dalam = all(harap - 0.01 <= x["terima_naik"] <= harap + 0.26 for x in u)
            cek(f"{nama:<9}: laporan menunjuk tempat yang benar ('{kalimat}')",
                kalimat in out)
            terbaca = ", ".join("%.2f" % x["terima_naik"] for x in u)
            cek(f"{nama:<9}: 'diterima' naik di rentang {harap:.2f}..{harap + 0.26:.2f} s "
                f"({terbaca})", dalam)

            # Angka PRESISI dari kurva pos - harus jauh lebih teliti dari 0,25 s
            ws = [I.waktu_stepper(x) for x in rek]
            S_harap = d_net + d_fw + d_balik
            S = [w["mulai_naik"] for w in ws] + [w["mulai_turun"] for w in ws]
            cek(f"{nama:<9}: stepper mulai = {S_harap:.2f} s +-0,03 "
                f"({min(S):.3f}..{max(S):.3f})", all(abs(x - S_harap) < 0.03 for x in S))
            g_harap = (d_gerak - d_net - d_fw) - d_balik
            G = [I.latensi_kamera(x, w)[0] for x, w in zip(rek, ws)]
            cek(f"{nama:<9}: latensi kamera = {g_harap + KAM[0]:.2f}..{g_harap + KAM[1]:.2f} s "
                f"({min(G):.3f}..{max(G):.3f})",
                all(g_harap + KAM[0] <= x <= g_harap + KAM[1] for x in G))
    finally:
        shutil.rmtree(folder_induk, ignore_errors=True)


def uji_kemudi(cek):
    print("\n--- kemudi: pencocokan yaw uji tangga ---")
    # Latensi kamera simulator (25 ms + umur frame + jitter), terukur 0,04-0,07 s.
    kam = (0.03, 0.08)
    folder = tempfile.mkdtemp(prefix="uji_kemudi_")
    try:
        for nama, tunda, tau in (("servo lamban", 0.22, 0.72), ("servo cepat", 0.35, 0.02)):
            np.random.seed(7)
            p = pelaksana(0.0, folder)
            p.sim.tunda, p.sim.tau = tunda, tau
            with contextlib.redirect_stdout(io.StringIO()):
                rek = p.tangga()
            if not rek:
                cek(f"{nama}: tangga berjalan di simulator", False)
                continue
            k = I.pasang_kemudi(rek[0])
            print(f"    {nama}: ditanam tunda {tunda:.2f} + tau {tau:.2f} -> terbaca "
                  f"d {k['d']:.3f}, tau {k['tau']:.3f}, sisa {k['rms']:.2f} deg")
            # Servo yang CEPAT dibatasi laju servo firmware (500 deg/s: lompatan
            # 20 deg butuh 0,04 s). Pada tau sekecil itu, pencocok wajar
            # memasukkan setengah waktu tempuh itu plus tau-nya ke d - bukan
            # kesalahan. Pada servo lamban batas laju tidak pernah mengikat.
            extra = (20.0 / konfig.SIM_SERVO_SLEW_DEG_S / 2 + tau) if tau < 0.1 else 0.0
            lo, hi = tunda + kam[0], tunda + kam[1] + extra
            cek(f"{nama}: d = tunda + latensi kamera sim"
                + (" + batas laju servo" if extra else "") + f" ({lo:.2f}..{hi:.2f})",
                lo <= k["d"] <= hi)
            cek(f"{nama}: tau terbaca {tau:.2f} +-0,10", abs(k["tau"] - tau) <= 0.10)
    finally:
        shutil.rmtree(folder, ignore_errors=True)


def main():
    skenario = [
        ("A dasar", 0.0, None),
        ("B tunda +0,15 s", 0.0, lambda s: tanam_tunda(s, TUNDA_TAMBAH)),
        ("C accel 4000", 0.0, lambda s: setattr(s, "accel", 4000.0)),
        ("D accel melemah", 0.0, tanam_melemah),
        ("E dasar + derau", 0.3, None),
        ("F accel 4000 + derau", 0.3, lambda s: setattr(s, "accel", 4000.0)),
    ]
    folder = tempfile.mkdtemp(prefix="uji_gas_")
    hasil = {}
    try:
        print(f"{'skenario':<22} {'lin':>3} {'tunda naik':>16} {'A naik':>16} "
              f"{'chi2/dof':>9} {'tunda turun':>16} {'A turun':>16} {'autokor':>8} "
              f"{'nonlin?':>8}")
        for nama, derau, siapkan in skenario:
            b, out = jalankan(nama, folder, derau, siapkan)
            hasil[nama] = (b, out)
            if "naik" not in b:
                print(f"{nama:<22} {b['n_lintasan']:>3}  (lintasan sah kurang dari 3)")
                continue
            tn, tns, An, Ans, cn = b["naik"]
            tt, tts, At, Ats, _ = b["turun"]
            print(f"{nama:<22} {b['n_lintasan']:>3} {tn:>7.3f} +-{tns:<6.3f} "
                  f"{An:>7.0f} +-{Ans:<6.0f} {cn:>9.1f} {tt:>7.3f} +-{tts:<6.3f} "
                  f"{At:>7.0f} +-{Ats:<6.0f} {b['autokor_maks']:>+8.2f} "
                  f"{'YA' if b['tidak_linear'] else 'tidak':>8}")
    finally:
        shutil.rmtree(folder, ignore_errors=True)

    print("\n--- pemeriksaan ---")
    gagal = 0

    def cek(teks, syarat):
        nonlocal gagal
        print(f"  [{'OK' if syarat else 'GAGAL'}] {teks}")
        gagal += not syarat

    kurang = [k for k, (b, _) in hasil.items() if "naik" not in b and not k.startswith("D")]
    if kurang:
        print(f"  [GAGAL] lintasan sah kurang dari 3 pada: {', '.join(kurang)}")
        sys.exit(1)

    A = hasil["A dasar"][0]["naik"]
    B = hasil["B tunda +0,15 s"][0]["naik"]
    C = hasil["C accel 4000"][0]["naik"]
    E = hasil["E dasar + derau"][0]["naik"]
    F = hasil["F accel 4000 + derau"][0]["naik"]
    cek(f"A: akselerasi pulih ke 12000 dalam 15% ({A[2]:.0f})", abs(A[2] - 12000) < 1800)
    cek(f"A: tunda = latensi kamera saja, 0,02-0,09 s ({A[0]:.3f})", 0.02 < A[0] < 0.09)
    cek(f"B: tunda naik ~0,15 dari A ({B[0] - A[0]:+.3f} s)",
        abs((B[0] - A[0]) - TUNDA_TAMBAH) < 0.04)
    cek(f"B: akselerasi TIDAK ikut turun ({B[2]:.0f} vs A {A[2]:.0f})",
        abs(B[2] - A[2]) < 0.15 * A[2])
    cek(f"C: akselerasi pulih ke 4000 dalam 15% ({C[2]:.0f})", abs(C[2] - 4000) < 600)
    cek(f"C: tunda TIDAK ikut naik ({C[0]:.3f} vs A {A[0]:.3f})", abs(C[0] - A[0]) < 0.04)
    cek("D: ditandai TIDAK linear", hasil["D accel melemah"][0]["tidak_linear"])
    cek("A, B, C: TIDAK ditandai tidak linear",
        not any(hasil[k][0]["tidak_linear"]
                for k in ("A dasar", "B tunda +0,15 s", "C accel 4000")))
    cek("D: tunda negatif diperingatkan", "tunda NEGATIF" in hasil["D accel melemah"][1])
    cek("A-C, E-F: tidak ada peringatan tunda negatif palsu",
        not any("tunda NEGATIF" in hasil[k][1] for k in hasil if not k.startswith("D")))
    cek(f"E (derau): akselerasi dalam 20% dari 12000 ({E[2]:.0f})", abs(E[2] - 12000) < 2400)
    cek(f"F (derau): akselerasi dalam 20% dari 4000 ({F[2]:.0f})", abs(F[2] - 4000) < 800)
    rho = [r for b, _ in hasil.values() for r in b["rho"]]
    cek(f"skala cm/langkah pulih di semua lintasan (rho {min(rho):.3f}..{max(rho):.3f})",
        all(abs(r - 1) < 0.02 for r in rho))

    uji_telemetri(cek)
    uji_kemudi(cek)

    print(f"\n{'SEMUA LULUS' if not gagal else f'{gagal} GAGAL'}")
    if "-v" in sys.argv:
        for nama, (_, out) in hasil.items():
            print(f"\n######## {nama}\n{out}")
    sys.exit(1 if gagal else 0)


if __name__ == "__main__":
    main()
