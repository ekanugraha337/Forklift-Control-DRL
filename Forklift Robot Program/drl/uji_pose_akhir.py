"""
uji_pose_akhir.py - POSE AKHIR robot simulasi: DQfD murni vs Hybrid A* + Reeds-Shepp.

Satu robot berangkat dari titik yang SAMA ke tujuan yang SAMA dengan dua
perencana. Yang dijalankan adalah navigasi.py sungguhan tanpa jendela (seperti
drl.uji_navigasi): perencana -> pelacak pure pursuit -> robot simulasi dengan
jeda kemudi dan kamera, 20 Hz waktu nyata. Pose akhir diambil dari pose
SEBENARNYA simulator setelah robot berhenti, jadi galatnya = galat rencana +
galat pelacakan.

    python -m drl.uji_pose_akhir --id 1                      # 3 skenario bawaan
    python -m drl.uji_pose_akhir --id 1 --ulang 3 --derau 1.2  # derau kamera, 3 kali per skenario
    python -m drl.uji_pose_akhir --id 1 --batas-waktu 10      # batas waktu rencana 10 s (robot: 2 s)
    python -m drl.uji_pose_akhir --id 1 --peta Rumah --awal 60,200,0 --tujuan 150,120,90   # skenario sendiri (cm, cm, deg)

SKENARIO BAWAAN - dipilih otomatis dari rute UJI dataset (tanpa robot lain,
margin 8 cm, tujuan berarah hadap), deterministik. Koordinatnya dicetak supaya
bisa diulang dengan klik di navigasi.py:
    1 lapang  Kalibrasi, start > 150 cm dari tujuan, tujuan kira-kira searah
              gerak (selisih arah <= 45 deg): navigasi biasa, pendaratan mudah
    2 balik   Rumah2, start > 100 cm, arah hadap tujuan BERLAWANAN dengan arah
              awal (>= 135 deg): robot harus berbalik / masuk mundur
    3 geser   Rumah, start sudah di dalam 60 cm, tujuan bergeser ke SAMPING
              (|samping| > |maju|) dengan arah hampir sama (<= 30 deg): manuver
              geser seperti parkir paralel - murni bagian dekat, jadi
              membandingkan model dekat dengan Reeds-Shepp

Diukur per perencana:
    rencana   waktu, panjang, ganti arah maju-mundur, galat pose UJUNG JALUR
              (Hybrid A* + RS eksak; DQfD <= 1 cm & 2 deg)
    eksekusi  tiba, waktu tempuh, galat posisi & arah pose AKHIR robot, jarak
              terdekat badan ke zona

Keluaran (model_dqn/pose_akhir/):
    id0N_<skenario>.png   peta, kedua jalur rencana, lintasan robot, pose awal,
                          tujuan, dan pose akhir; panel kanan diperbesar di tujuan
    pose_akhir_id0N.json  semua angka + koordinat skenario
"""
import argparse
import contextlib
import io
import json
import math
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                       # noqa: E402

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PERENCANA = ("astar", "dqfd-murni")
NAMA = {"astar": "Hybrid A* + RS", "dqfd-murni": "DQfD murni"}
WARNA = {"astar": "#1f77b4", "dqfd-murni": "#d62728"}
BATAS_S = 120.0
TENANG_S = 1.0          # setelah SELESAI: tunggu robot benar-benar diam


def _sudut(a):
    return (a + 180.0) % 360.0 - 180.0


def _cm(arena, p):
    """Pose sel dataset -> (x_cm, y_cm, yaw_deg atau None)."""
    x, y = arena.ke_cm(p[0], p[1])
    return (float(x), float(y), None if p[2] is None or np.isnan(p[2]) else math.degrees(p[2]))


# ------------------------------------------------------------ skenario
def _kandidat(rid, berkas, peta):
    import arena as A
    from drl import rollout as RO
    p = os.path.join(AKAR, "drl", "data", berkas.format(rid=rid))
    if not os.path.exists(p):
        return []
    d = RO.muat_npz(p)
    ar = A.Arena(peta)
    out = []
    for e in np.flatnonzero(d["ep_uji"]):
        if (d["ep_peta"][e] != peta or not np.isnan(d["ep_rintangan"][e]).all()
                or d["ep_margin"][e] < konfig.SAFETY_MARGIN_CM or np.isnan(d["ep_tujuan"][e][2])):
            continue
        a, t = _cm(ar, d["ep_awal"][e]), _cm(ar, d["ep_tujuan"][e])
        dx, dy = t[0] - a[0], t[1] - a[1]
        c, s = math.cos(math.radians(a[2])), math.sin(math.radians(a[2]))
        out.append({"e": int(e), "awal": a, "tujuan": t, "jarak": math.hypot(dx, dy),
                    "maju": c * dx + s * dy, "samping": -s * dx + c * dy,
                    "d_arah": abs(_sudut(t[2] - a[2])),
                    "d_datang": abs(_sudut(t[2] - math.degrees(math.atan2(dy, dx))))})
    return out


def _median(kand, kunci="jarak"):
    kand = sorted(kand, key=lambda k: k[kunci])
    return kand[len(kand) // 2] if kand else None


def skenario_bawaan(rid):
    """Tiga skenario dari rute uji. Syarat dilonggarkan bila tidak ada yang cocok."""
    out = []
    k = _kandidat(rid, "demo_jauh_id{rid:02d}.npz", "Kalibrasi")
    pilih = (_median([x for x in k if x["jarak"] >= 150 and x["d_arah"] <= 45 and x["d_datang"] <= 45])
             or _median([x for x in k if x["jarak"] >= 120 and x["d_datang"] <= 60]))
    if pilih:
        out.append(("lapang", "Kalibrasi", pilih, "start > 150 cm, tujuan searah gerak"))
    k = _kandidat(rid, "demo_jauh_id{rid:02d}.npz", "Rumah2")
    pilih = (_median([x for x in k if x["jarak"] >= 100 and x["d_arah"] >= 135])
             or _median([x for x in k if x["d_arah"] >= 120]))
    if pilih:
        out.append(("balik", "Rumah2", pilih, "arah hadap tujuan berlawanan dengan arah awal"))
    pilih = None
    for peta in ("Rumah", "Rumah2", "Kalibrasi"):
        k = _kandidat(rid, "demo_dekat_tambahan_id{rid:02d}.npz", peta)
        pilih = (_median([x for x in k if 20 <= x["jarak"] <= 50 and abs(x["samping"]) > abs(x["maju"])
                          and x["d_arah"] <= 30])
                 or _median([x for x in k if abs(x["samping"]) > abs(x["maju"]) and x["d_arah"] <= 45]))
        if pilih:
            out.append(("geser", peta, pilih, "start < 60 cm, tujuan bergeser ke samping, arah hampir sama"))
            break
    return out


# ------------------------------------------------------------ satu jalan
def jalankan(tugas):
    """Satu perencana, satu skenario, di proses pekerja (navigasi.py tanpa jendela)."""
    import arena as A
    import navigasi as N
    konfig.pasang_path_kamera()
    import area_constraint as AC

    rid, (x0, y0, yaw0), tj = tugas["id"], tugas["awal"], tugas["tujuan"]
    args = SimpleNamespace(peta=tugas["peta"], sel=None, tracker=konfig.PELACAK_BAWAAN,
                           perencana=tugas["perencana"], simulasi=True, derau=tugas["derau"],
                           derau_deg=0.0, margin=10.0, url=None, host=None, log=None)
    teks = io.StringIO()
    rencana, lintasan = [], []
    hasil = {"skenario": tugas["skenario"], "perencana": tugas["perencana"], "ulang": tugas["ulang"]}
    with contextlib.redirect_stdout(teks):
        konfig.pilih_robot(rid)
        if tugas.get("batas_waktu"):
            konfig.PARAM_RENCANA["time_limit"] = tugas["batas_waktu"]   # kedua perencana
        nav = N.Navigasi(args, [rid])
        u = nav.units[0]
        u.sim.x, u.sim.y, u.sim.yaw = x0, y0, yaw0
        asli = u.per.rencanakan

        def catat(*a, **k):
            jalur, info = asli(*a, **k)
            rencana.append({"jalur_cm": [list(nav.arena.pose_ke_cm(p[0], p[1], p[2])) + [p[3]] for p in jalur]
                            if jalur else [],
                            "oleh": info.get("perencana", "A*") if jalur else "gagal",
                            "ms": 1000 * info.get("waktu_s", 0.0), "rs_titik": info.get("rs_titik", 0),
                            "alasan": "" if jalur else (info.get("error") or info.get("alasan") or "")})
            return jalur, info
        u.per.rencanakan = catat

        t0 = time.time()
        nav.satu_langkah(t0)
        u.terima_tujuan(tj)
        zona_min, langgar, t_tiba, t_diam = float("inf"), False, None, None
        while True:
            now = time.time()
            nav.satu_langkah(now)
            lintasan.append((u.sim.x, u.sim.y, u.sim.yaw))
            fp = A.jejak_badan(u.sim.x, u.sim.y, u.sim.yaw)
            s, _, jarak = AC.evaluasi(fp, nav.arena.zona, 0.0)
            if np.isfinite(jarak):
                zona_min = min(zona_min, jarak)
            langgar |= s == AC.PELANGGARAN
            if u.state == N.SELESAI and t_tiba is None:
                t_tiba = now - t0
            gagal = u.state == N.SIAP and u.tracker is None and not u._sedang_merencana and now - t0 > 1.0
            if u.state == N.BAHAYA or gagal or now - t0 > BATAS_S:
                break
            if t_tiba is not None and now - t0 - t_tiba >= TENANG_S:
                break
            sisa = konfig.RC_PERIODE_S - (time.time() - now)
            if sisa > 0:
                time.sleep(sisa)
        hasil.update(tiba=u.state == N.SELESAI, state=u.state, alasan=u.alasan, t_s=t_tiba,
                     akhir=[u.sim.x, u.sim.y, u.sim.yaw], masuk_zona=bool(langgar), zona_min_cm=zona_min,
                     rencana=rencana, lintasan=lintasan[::2])
        u.tutup()
    hasil["log_akhir"] = teks.getvalue().splitlines()[-6:]
    return hasil


# ------------------------------------------------------------ ukuran
def ukuran(h, tj):
    """Angka rencana (jalur pertama) dan eksekusi (pose akhir robot)."""
    r = {"tiba": h["tiba"], "state": h["state"], "alasan": h["alasan"], "t_s": h["t_s"],
         "masuk_zona": h["masuk_zona"], "zona_min_cm": h["zona_min_cm"], "n_rencana": len(h["rencana"])}
    pertama = next((x for x in h["rencana"] if x["jalur_cm"]), None)
    if pertama:
        j = pertama["jalur_cm"]
        ex, ey, eyaw = j[-1][0], j[-1][1], j[-1][2]          # pose_ke_cm: yaw sudah derajat
        r.update(rencana_oleh=pertama["oleh"], rencana_ms=pertama["ms"],
                 panjang_cm=float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(j[:-1], j[1:]))),
                 ganti_arah=int(sum(a[3] != b[3] for a, b in zip(j[:-1], j[1:]))),
                 rencana_galat_cm=math.hypot(ex - tj[0], ey - tj[1]),
                 rencana_galat_deg=None if tj[2] is None else abs(_sudut(eyaw - tj[2])))
    x, y, yaw = h["akhir"]
    r.update(galat_cm=math.hypot(x - tj[0], y - tj[1]),
             galat_deg=None if tj[2] is None else abs(_sudut(yaw - tj[2])),
             galat_maju_cm=(x - tj[0]) * math.cos(math.radians(tj[2] or 0)) + (y - tj[1]) * math.sin(math.radians(tj[2] or 0)),
             galat_samping_cm=-(x - tj[0]) * math.sin(math.radians(tj[2] or 0)) + (y - tj[1]) * math.cos(math.radians(tj[2] or 0)))
    return r


# ------------------------------------------------------------ gambar
def gambar(rid, sk, hasil, keluar):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    import arena as A

    ar = A.Arena(sk["peta"])
    x0, y0 = ar.asal
    W, H = ar.lebar_sel * ar.cm_per_cell, ar.tinggi_sel * ar.cm_per_cell
    tj, aw = sk["tujuan"], sk["awal"]
    fig, sumbu = plt.subplots(1, 2, figsize=(15, 6.6), gridspec_kw={"width_ratios": [1.25, 1]})

    def badan(ax, pose, **kw):
        ax.add_patch(Polygon(np.asarray(A.jejak_badan(pose[0], pose[1], pose[2] or 0.0)), closed=True, **kw))

    def panah(ax, pose, warna, pj=10):
        if pose[2] is not None:
            ax.annotate("", xy=(pose[0] + pj * math.cos(math.radians(pose[2])), pose[1] + pj * math.sin(math.radians(pose[2]))),
                        xytext=(pose[0], pose[1]), arrowprops=dict(arrowstyle="-|>", color=warna, lw=1.6))

    for i, ax in enumerate(sumbu):
        ax.imshow(ar.occ > 0, cmap="Greys", alpha=0.55, extent=(x0, x0 + W, y0 + H, y0), origin="upper",
                  interpolation="nearest")
        badan(ax, aw, fill=False, ec="#555555", lw=1.0, ls=":")
        badan(ax, tj, fill=False, ec="black", lw=1.6, ls="--")
        panah(ax, tj, "black", 12 if i else 18)
        for per in PERENCANA:
            h = hasil.get(per)
            if h is None:
                continue
            w = WARNA[per]
            for rc in h["rencana"][:1]:
                j = np.asarray(rc["jalur_cm"])
                if len(j):
                    ax.plot(j[:, 0], j[:, 1], color=w, lw=1.0, ls="--", alpha=0.8)
            lt = np.asarray(h["lintasan"])
            if len(lt):
                ax.plot(lt[:, 0], lt[:, 1], color=w, lw=2.0, alpha=0.9,
                        label=f"{NAMA[per]}" if i == 0 else None)
            badan(ax, h["akhir"], fill=True, fc=w, ec=w, alpha=0.18, lw=1.2)
            badan(ax, h["akhir"], fill=False, ec=w, lw=1.2)
            panah(ax, h["akhir"], w, 12 if i else 18)
            ax.plot(*h["akhir"][:2], "o", color=w, ms=4)
        ax.plot(*tj[:2], "x", color="black", ms=9, mew=2)
        ax.set_aspect("equal")
        if i == 0:
            ax.set_xlim(x0, x0 + W)
            ax.set_ylim(y0 + H, y0)
            ax.set_title(f"ID {rid:02d} - skenario '{sk['nama']}' di {sk['peta']} ({sk['ket']})", fontsize=10)
            ax.legend(loc="lower right", fontsize=8)
        else:
            r = 35
            ax.set_xlim(tj[0] - r, tj[0] + r)
            ax.set_ylim(tj[1] + r, tj[1] - r)
            ax.set_title("diperbesar di tujuan (garis putus = jalur rencana, tebal = lintasan robot)", fontsize=9)
            baris = []
            for per in PERENCANA:
                m = sk["ukuran"].get(per)
                if not m:
                    continue
                g = f"{m['galat_cm']:.2f} cm / {m['galat_deg']:.2f} deg" if m.get("galat_deg") is not None else f"{m['galat_cm']:.2f} cm"
                rc = (f"rencana {m['rencana_galat_cm']:.2f} cm / {m['rencana_galat_deg']:.2f} deg"
                      if m.get("rencana_galat_deg") is not None else "rencana -")
                baris.append(f"{NAMA[per]}: {'tiba' if m['tiba'] else 'TIDAK tiba'} | akhir {g} | {rc}")
            ax.text(0.01, 0.01, "\n".join(baris), transform=ax.transAxes, fontsize=8, va="bottom",
                    bbox=dict(boxstyle="round", fc="white", alpha=0.85))
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
    fig.tight_layout()
    fig.savefig(keluar, dpi=110)
    plt.close(fig)


# ------------------------------------------------------------ utama
def main():
    ap = argparse.ArgumentParser(description="Pose akhir robot simulasi: DQfD murni vs Hybrid A* + RS")
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--derau", type=float, default=0.0, help="derau pose kamera (cm); 0 = deterministik")
    ap.add_argument("--ulang", type=int, default=1, help="jalankan tiap skenario sekian kali (berguna dengan --derau)")
    ap.add_argument("--peta", default=None, help="skenario sendiri: nama peta")
    ap.add_argument("--awal", default=None, help="skenario sendiri: x,y,deg (cm)")
    ap.add_argument("--tujuan", default=None, help="skenario sendiri: x,y,deg (cm); deg boleh dikosongkan = arah bebas")
    ap.add_argument("--batas-waktu", type=float, default=None,
                    help="batas waktu perencanaan (s) untuk KEDUA perencana; bawaan konfig (2 s, seperti robot)")
    ap.add_argument("--pekerja", type=int, default=6)
    args = ap.parse_args()

    if args.awal or args.tujuan:
        if not (args.peta and args.awal and args.tujuan):
            ap.error("skenario sendiri butuh --peta, --awal, dan --tujuan")
        a = [float(v) for v in args.awal.split(",")]
        t = [float(v) if v.strip() else None for v in (args.tujuan.split(",") + [""])[:3]]
        daftar = [{"nama": "sendiri", "peta": args.peta, "ket": "skenario sendiri", "e": None,
                   "awal": (a[0], a[1], a[2]), "tujuan": (t[0], t[1], t[2])}]
    else:
        daftar = [{"nama": n, "peta": p, "ket": ket, "e": k["e"], "awal": k["awal"], "tujuan": k["tujuan"]}
                  for n, p, k, ket in skenario_bawaan(args.id)]
    print(f"[i] ID {args.id:02d}, derau {args.derau} cm, ulang {args.ulang}x, batas waktu rencana "
          f"{args.batas_waktu or konfig.PARAM_RENCANA['time_limit']} s, perencana: "
          + ", ".join(NAMA[p] for p in PERENCANA))
    for sk in daftar:
        a, t = sk["awal"], sk["tujuan"]
        jarak = math.hypot(t[0] - a[0], t[1] - a[1])
        print(f"    {sk['nama']:<7} {sk['peta']:<9} awal ({a[0]:.1f}, {a[1]:.1f}, {a[2]:.1f} deg) -> tujuan "
              f"({t[0]:.1f}, {t[1]:.1f}, {('%.1f deg' % t[2]) if t[2] is not None else 'bebas'}), "
              f"jarak {jarak:.0f} cm" + (f"  [rute uji #{sk['e']}]" if sk["e"] is not None else ""))

    tugas = [{"id": args.id, "skenario": sk["nama"], "peta": sk["peta"], "awal": sk["awal"], "tujuan": sk["tujuan"],
              "perencana": per, "derau": args.derau, "ulang": u, "batas_waktu": args.batas_waktu}
             for sk in daftar for per in PERENCANA for u in range(args.ulang)]
    t0 = time.time()
    import multiprocessing as mp
    with mp.get_context("spawn").Pool(max(1, min(args.pekerja, len(tugas)))) as pool:
        semua = []
        for k, h in enumerate(pool.imap_unordered(jalankan, tugas)):
            semua.append(h)
            print(f"\r[{time.time() - t0:5.0f} s] {k + 1}/{len(tugas)} jalan selesai", end="", flush=True)
    print()

    dir_keluar = os.path.join(AKAR, "model_dqn", "pose_akhir")
    os.makedirs(dir_keluar, exist_ok=True)
    print(f"\n{'skenario':<8} {'perencana':<15} {'tiba':>5} {'rencana':>9} {'panjang':>8} {'balik':>5} "
          f"{'galat ujung rencana':>21} {'GALAT POSE AKHIR ROBOT':>24} {'maju/samping':>14} {'tempuh':>7} {'zona min':>8}")
    for sk in daftar:
        sk["ukuran"], sk["ulang"] = {}, {}
        for per in PERENCANA:
            hs = sorted([h for h in semua if h["skenario"] == sk["nama"] and h["perencana"] == per],
                        key=lambda h: h["ulang"])
            ms = [ukuran(h, sk["tujuan"]) for h in hs]
            sk["ulang"][per] = ms
            sk["ukuran"][per] = ms[0]
            for k, m in enumerate(ms):
                rc = (f"{m['rencana_galat_cm']:5.2f} cm {m['rencana_galat_deg']:5.2f} deg"
                      if m.get("rencana_galat_deg") is not None else f"{m.get('rencana_galat_cm', float('nan')):5.2f} cm")
                ak = (f"{m['galat_cm']:6.2f} cm {m['galat_deg']:6.2f} deg" if m["galat_deg"] is not None
                      else f"{m['galat_cm']:6.2f} cm")
                print(f"{sk['nama'] if k == 0 else '':<8} {NAMA[per] if k == 0 else '':<15} "
                      f"{'ya' if m['tiba'] else 'TIDAK':>5} {m.get('rencana_ms', float('nan')):7.0f}ms "
                      f"{m.get('panjang_cm', float('nan')):6.0f}cm {m.get('ganti_arah', 0):>5} {rc:>21} {ak:>24} "
                      f"{m['galat_maju_cm']:+6.2f}/{m['galat_samping_cm']:+5.2f} "
                      f"{(m['t_s'] or float('nan')):6.1f}s {m['zona_min_cm']:7.1f}cm"
                      + ("" if m["tiba"] else f"  ({m['state']}: {m['alasan']})"))
            if len(ms) > 1:
                g = [m["galat_cm"] for m in ms if m["tiba"]]
                gd = [m["galat_deg"] for m in ms if m["tiba"] and m["galat_deg"] is not None]
                if g:
                    print(f"{'':<8} {'':<15} rata {np.mean(g):.2f} cm (maks {np.max(g):.2f})"
                          + (f", {np.mean(gd):.2f} deg (maks {np.max(gd):.2f})" if gd else "")
                          + f", tiba {sum(m['tiba'] for m in ms)}/{len(ms)}")
        hasil_satu = {per: next((h for h in semua if h["skenario"] == sk["nama"] and h["perencana"] == per
                                 and h["ulang"] == 0), None) for per in PERENCANA}
        png = os.path.join(dir_keluar, f"id{args.id:02d}_{sk['nama']}.png")
        gambar(args.id, sk, hasil_satu, png)
        print(f"{'':<8} -> {png}")
    with open(os.path.join(dir_keluar, f"pose_akhir_id{args.id:02d}.json"), "w", encoding="utf-8") as f:
        json.dump({"robot": args.id, "derau": args.derau, "batas_waktu": args.batas_waktu, "skenario": daftar,
                   "jalan": [{k: v for k, v in h.items() if k != "lintasan"} for h in semua]},
                  f, indent=1, default=float)
    print("\nGalat pose akhir robot = galat ujung jalur rencana + galat pelacakan (pure pursuit, jeda kemudi). "
          "maju/samping = galat posisi dalam kerangka arah hadap tujuan.")


if __name__ == "__main__":
    main()
