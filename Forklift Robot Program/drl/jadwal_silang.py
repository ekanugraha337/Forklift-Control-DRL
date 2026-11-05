"""
jadwal_silang.py - jadwal "siapa mengalah" untuk skenario yang jalur kedua robotnya
bersilangan. Hasilnya dibaca oleh `drl.uji_robot_nyata --jadwal`.

GAGASAN. Tanpa jadwal, konflik di titik silang diselesaikan wasit saat robot sudah
berjalan: menunggu, merencana ulang memutar, atau minggir (catatan Obsidian 19).
Padahal simulasi meramalkan titik dan waktu konflik dengan baik (titik konflik
simulasi vs robot nyata berselisih 3,7 cm). Jadi konflik diselesaikan SEBELUM uji:

    1. Kedua robot direncanakan dari AWAL ke TUJUAN-nya persis seperti saat uji
       (tujuan bersamaan, tanpa robot lain sebagai rintangan), untuk SETIAP
       perencana uji - jalurnya berbeda per perencana.
    2. Untuk setiap pilihan "robot Y mengalah kepada robot P":
         - titik konflik  = titik pertama di jalur Y yang badannya lebih dekat dari
                            --jarak-cm ke badan P di titik mana pun jalurnya;
         - titik tunggu   = titik di jalur Y, --penyangga-cm sebelum titik konflik.
                            Di sana Y berhenti dengan badan >= --jarak-cm dari
                            seluruh jalur P, jadi P bebas lewat;
         - dilepas        = setelah sisa jalur P (dan badannya) lebih jauh dari
                            --jarak-cm dari sisa jalur Y, atau P sudah tiba.
       Makespan diperkirakan dengan laju sama untuk kedua robot (dalam cm tempuh):
       P tiba di L_P; Y tiba di max(L_Y sampai tunggu, L_P sampai lepas) + sisa L_Y.
    3. Dipilih yang perkiraan makespan-nya terkecil.

Di uji (drl/uji_robot_nyata.py --jadwal) berkas ini menentukan SIAPA yang mengalah.
Titik tunggu dan saat dilepas dihitung ulang selama uji dari jalur yang SEBENARNYA
dipakai, dengan aturan yang sama (jarak_cm, penyangga_cm): pose awal setelah pulang
meleset beberapa cm dan derajat, dan jalur DQfD dari pose itu bisa bergeser belasan
cm dari jalur di sini (terukur 2026-11-05: 18,9 cm). Titik tunggu di berkas menjadi
prediksi pembanding. Pelepasan memakai posisi robot P yang sebenarnya, bukan jam -
jadi tetap benar walau robot nyata lebih lambat atau lebih cepat daripada simulasi.
Bila P sudah parkir di dekat sisa jalur Y, Y merencana ulang memutarinya saat
dilepas. Wasit tetap aktif sebagai pengaman.

MENGAPA BUKAN "JALUR LAIN". Bila jalur P menghubungkan dua sisi ruangan (misalnya
Gudang2 S1: dari slot di pojok atas ke teluk di dinding bawah) dan awal serta tujuan
Y ada di sisi yang berbeda, SETIAP jalur Y pasti memotong jalur P. Yang bisa
diatur hanyalah KAPAN Y melintas.

    python -m drl.jadwal_silang --skenario uji_nyata/Gudang2.json
    python -m drl.jadwal_silang --skenario uji_nyata/Gudang2.json --keluar uji_nyata/Gudang2_jadwal.json \\
        --gambar jadwal.png
    # lalu:
    python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang2.json --jadwal uji_nyata/Gudang2_jadwal.json --simulasi
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                       # noqa: E402

JARAK_CM = 15.0      # jarak badan minimum yang dijaga jadwal (wasit sendiri 5 cm; + simpangan pelacakan)
PENYANGGA_CM = 10.0  # titik tunggu sekian cm sebelum titik konflik (jarak pengereman + simpangan)


def _jalur_cm(arena, jalur):
    return [tuple(arena.pose_ke_cm(p[0], p[1], p[2])) + (int(p[3]),) for p in jalur]


def _kumulatif(jc):
    L = [0.0]
    for a, b in zip(jc, jc[1:]):
        L.append(L[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return L


def rancang(jc_y, jc_p, jarak_cm=JARAK_CM, penyangga_cm=PENYANGGA_CM):
    """Jadwal untuk "robot Y mengalah kepada robot P" dari jalur keduanya (cm).

    Kembalian dict: konflik (bool), k_tahan, tahan (x, y, yaw), konflik_di (x, y),
    lepas_di (x, y) atau None bila Y baru boleh lanjut setelah P tiba, dan perkiraan
    tempuh (cm) - lihat docstring modul.
    """
    import arena as A
    import wasit as W
    By = [A.jejak_badan(*p[:3]) for p in jc_y]
    Bp = [A.jejak_badan(*p[:3]) for p in jc_p]
    Ly, Lp = _kumulatif(jc_y), _kumulatif(jc_p)
    d_y = [W.jarak_himpunan([b], Bp, batas=jarak_cm) for b in By]
    konflik = [k for k, d in enumerate(d_y) if d < jarak_cm]
    if not konflik:
        return {"konflik": False, "makespan_cm": max(Ly[-1], Lp[-1])}
    k_konf = konflik[0]
    calon = [k for k in range(k_konf + 1) if Ly[k_konf] - Ly[k] >= penyangga_cm]
    k_tahan = calon[-1] if calon else 0
    y_sisa = By[k_tahan:]
    e_p = [W.jarak_himpunan([b], y_sisa, batas=jarak_cm) for b in Bp]
    dekat = [j for j, e in enumerate(e_p) if e < jarak_cm]
    j_lepas = (dekat[-1] + 1) if dekat else 0
    sampai_tiba = j_lepas >= len(jc_p)
    t_lepas = Lp[-1] if sampai_tiba else Lp[j_lepas]
    t_y = max(Ly[k_tahan], t_lepas) + (Ly[-1] - Ly[k_tahan])
    return {"konflik": True, "k_tahan": k_tahan, "k_konflik": k_konf,
            "tahan": [round(v, 2) for v in jc_y[k_tahan][:3]],
            "konflik_di": [round(v, 1) for v in jc_y[k_konf][:2]],
            "lepas_di": None if sampai_tiba else [round(v, 1) for v in jc_p[j_lepas][:2]],
            "tunggu_sampai_tiba": sampai_tiba,
            "tempuh_cm": {"lewat": round(Lp[-1], 1), "mengalah": round(Ly[-1], 1),
                          "mengalah_sampai_tunggu": round(Ly[k_tahan], 1), "lewat_sampai_lepas": round(t_lepas, 1)},
            "makespan_cm": round(max(Lp[-1], t_y), 1)}


def tabrakan_tanpa_jadwal(jc1, jc2):
    """Titik tengah kedua robot saat badannya pertama bersentuhan, bila berangkat
    bersamaan dengan laju sama dan tanpa wasit; None bila tidak bertemu."""
    import arena as A
    import wasit as W
    L1, L2 = _kumulatif(jc1), _kumulatif(jc2)

    def pada(jc, L, s):
        if s >= L[-1]:
            return jc[-1][:3]
        return jc[int(np.searchsorted(L, s, side="right")) - 1][:3]
    for s in np.arange(0.0, max(L1[-1], L2[-1]) + 1.0, 1.0):
        p1, p2 = pada(jc1, L1, s), pada(jc2, L2, s)
        if W.jarak_poligon(A.jejak_badan(*p1), A.jejak_badan(*p2)) <= 0:
            return {"tempuh_cm": round(float(s), 1),
                    "titik": [round((p1[0] + p2[0]) / 2, 1), round((p1[1] + p2[1]) / 2, 1)]}
    return None


def main():
    from drl import uji_robot_nyata as U
    import arena as A
    import rencana as R
    ap = argparse.ArgumentParser(description="Jadwal siapa mengalah untuk skenario dengan jalur bersilangan")
    ap.add_argument("--skenario", required=True, help="berkas skenario (uji_nyata/....json)")
    ap.add_argument("--keluar", default=None, help="berkas jadwal (bawaan: <skenario>_jadwal.json)")
    ap.add_argument("--perencana", default=",".join(U.PERENCANA_UJI))
    ap.add_argument("--jarak-cm", type=float, default=JARAK_CM, help="jarak badan minimum yang dijaga jadwal")
    ap.add_argument("--penyangga-cm", type=float, default=PENYANGGA_CM, help="titik tunggu sebelum titik konflik")
    ap.add_argument("--gambar", default=None, help="simpan gambar jalur, titik konflik, dan titik tunggu")
    args = ap.parse_args()
    jenis = [j.strip() for j in args.perencana.split(",") if j.strip()]
    data = U.muat_skenario(args.skenario)
    ids = sorted({int(r) for sk in data["skenario"] for r in sk["robot"]})
    konfig.pilih_robot(ids[0])
    ar = A.Arena(data["peta"])
    per = {}
    for rid in ids:
        konfig.pilih_robot(rid)
        per[rid] = {j: R.Perencana(ar, konfig.profil(rid), jenis=j) for j in jenis}

    keluar = {"dibuat": time.strftime("%Y-%m-%d %H:%M:%S"), "skenario": args.skenario, "peta": data["peta"],
              "jarak_cm": args.jarak_cm, "penyangga_cm": args.penyangga_cm, "jadwal": {}}
    gambar = []
    for sk in data["skenario"]:
        rb = sk["robot"]
        if len(rb) != 2:
            print(f"[i] {sk['nama']}: bukan dua robot - dilewati")
            continue
        r1, r2 = sorted(rb)
        for j in jenis:
            jc = {}
            for rid in (r1, r2):
                jl, info = per[rid][j].rencanakan(tuple(rb[rid]["awal"]), tuple(rb[rid]["tujuan"]))
                if not jl:
                    print(f"[!] {sk['nama']} {j}: robot {rid:02d} gagal direncanakan ({info.get('alasan')}) - dilewati")
                    break
                jc[rid] = _jalur_cm(ar, jl)
            if len(jc) != 2:
                continue
            tanpa = tabrakan_tanpa_jadwal(jc[r1], jc[r2])
            pilihan = {}
            for y, p in ((r1, r2), (r2, r1)):
                pilihan[y] = rancang(jc[y], jc[p], args.jarak_cm, args.penyangga_cm)
            if not any(v["konflik"] for v in pilihan.values()):
                print(f"[i] {sk['nama']} {j:10s}: jalur tidak saling mendekat < {args.jarak_cm:.0f} cm - tanpa jadwal")
                continue
            y = min((k for k in pilihan if pilihan[k]["konflik"]), key=lambda k: pilihan[k]["makespan_cm"])
            p = r2 if y == r1 else r1
            e = pilihan[y]
            keluar["jadwal"].setdefault(sk["nama"], {})[j] = {
                "mengalah": y, "lewat": p, "tahan": e["tahan"], "konflik_di": e["konflik_di"],
                "lepas_di": e["lepas_di"], "tunggu_sampai_tiba": e["tunggu_sampai_tiba"],
                "tempuh_cm": e["tempuh_cm"], "makespan_cm": e["makespan_cm"],
                "tanpa_jadwal": tanpa,
                "pilihan": {str(k): {"makespan_cm": v["makespan_cm"], "tahan": v.get("tahan")} for k, v in pilihan.items()}}
            print(f"[OK] {sk['nama']} {j:10s}: robot {y:02d} mengalah kepada {p:02d} | tunggu di "
                  f"({e['tahan'][0]:.0f}, {e['tahan'][1]:.0f}) sebelum konflik ({e['konflik_di'][0]:.0f}, "
                  f"{e['konflik_di'][1]:.0f}) | dilepas "
                  + ("setelah robot lewat tiba" if e["tunggu_sampai_tiba"] else
                     f"setelah robot {p:02d} melewati ({e['lepas_di'][0]:.0f}, {e['lepas_di'][1]:.0f})")
                  + f" | perkiraan makespan {e['makespan_cm']:.0f} cm tempuh"
                  + " (pilihan lain: " + ", ".join(f"{k:02d} mengalah {v['makespan_cm']:.0f} cm"
                                                   for k, v in pilihan.items() if k != y and v["konflik"]) + ")"
                  + (f" | tanpa jadwal bertabrakan di ({tanpa['titik'][0]:.0f}, {tanpa['titik'][1]:.0f}) "
                     f"setelah {tanpa['tempuh_cm']:.0f} cm" if tanpa else ""))
            gambar.append((sk["nama"], j, jc, y, p, e, tanpa))

    path = args.keluar or os.path.splitext(args.skenario)[0] + "_jadwal.json"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(keluar, f, indent=1)
    print(f"[OK] {path}")
    if args.gambar and gambar:
        _gambar(args.gambar, ar, gambar)


def _gambar(path, ar, isi):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    import arena as A
    x0, y0 = ar.asal
    W_, H_ = ar.lebar_sel * ar.cm_per_cell, ar.tinggi_sel * ar.cm_per_cell
    fig, sumbu = plt.subplots(1, len(isi), figsize=(7 * len(isi), 6.4), squeeze=False)
    warna = {1: "#1f77b4", 2: "#d62728"}
    for ax, (sn, j, jc, y, p, e, tanpa) in zip(sumbu[0], isi):
        ax.imshow(ar.occ > 0, cmap="Greys", alpha=0.55, extent=(x0, x0 + W_, y0 + H_, y0), origin="upper")
        for rid, c in jc.items():
            q = np.asarray([v[:2] for v in c])
            ax.plot(q[:, 0], q[:, 1], color=warna.get(rid, "k"), lw=1.4,
                    label=f"robot {rid:02d}" + (" (mengalah)" if rid == y else " (lewat)"))
        ax.add_patch(Polygon(A.jejak_badan(*e["tahan"]), fill=False, ec=warna.get(y, "k"), lw=2))
        ax.plot(*e["tahan"][:2], "o", color=warna.get(y, "k"), ms=6)
        ax.annotate("titik tunggu", e["tahan"][:2], xytext=(8, -12), textcoords="offset points", fontsize=8)
        ax.plot(*e["konflik_di"], "x", color="k", ms=9, mew=2)
        ax.annotate("konflik", e["konflik_di"], xytext=(8, 4), textcoords="offset points", fontsize=8)
        if e["lepas_di"]:
            ax.plot(*e["lepas_di"], "s", color=warna.get(p, "k"), ms=6)
            ax.annotate(f"lepas saat {p:02d} lewat sini", e["lepas_di"], xytext=(8, 4),
                        textcoords="offset points", fontsize=8)
        if tanpa:
            ax.plot(*tanpa["titik"], "*", color="#ff7f0e", ms=12)
            ax.annotate("tabrakan tanpa jadwal", tanpa["titik"], xytext=(8, -14), textcoords="offset points",
                        fontsize=8, color="#ff7f0e")
        ax.set_xlim(x0, x0 + W_); ax.set_ylim(y0 + H_, y0); ax.set_aspect("equal")
        ax.set_title(f"{sn} {j}: robot {y:02d} mengalah", fontsize=10)
        ax.legend(fontsize=8, loc="upper center")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print(f"[OK] {path}")


if __name__ == "__main__":
    main()
