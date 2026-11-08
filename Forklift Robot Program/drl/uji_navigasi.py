"""
uji_navigasi.py - robot BERJALAN di simulasi dengan perencana A* / DQfD (tahap 9).

Bukan simulasi perencanaan: navigasi.py yang sebenarnya dijalankan tanpa
jendela - pelacak pure pursuit, jeda kemudi & kamera SumberSimulasi, derau
pose, wasit dua robot. Tujuan diberikan lewat Unit.terima_tujuan(), persis
seperti klik mouse, lalu Navigasi.satu_langkah() dipanggil tiap 50 ms dalam
waktu nyata.

    python -m drl.uji_navigasi                       # satu robot + pasangan, A* vs DQfD
    python -m drl.uji_navigasi --perencana astar,dqfd,dqfd-murni --n 5

SKENARIO
    satu robot  rute uji dataset (tanpa rintangan, margin 8 cm) di setiap peta
                asli yang ada di dataset DAN di Camera/peta_constraint/ - n per
                peta, untuk ID 01 dan ID 02
    pasangan    rute uji ID 01 + rute uji ID 02 di peta yang sama yang jalur
                A*-nya BERPOTONGAN (< 20 cm), supaya wasit benar-benar bekerja

Yang diukur dari pose SEBENARNYA simulator (bukan pose terukur berderau):
tiba, waktu, galat posisi/arah akhir, badan masuk zona, jarak terdekat ke
zona, dan untuk pasangan jarak badan terdekat antar-robot.

Keluaran: model_dqn/uji_navigasi.json
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


def daftar_peta(data):
    """Peta asli yang ada di dataset kedua robot dan masih ada di Camera/peta_constraint/."""
    konfig.pasang_path_kamera()
    import area_constraint as AC
    ada = set(AC.daftar_peta())
    di_data = [{str(p) for p in d["ep_peta"] if not str(p).startswith("sintetis:")} for d in data.values()]
    return sorted(ada.intersection(*di_data))
BATAS_S_SATU, BATAS_S_PASANGAN = 120.0, 180.0


def _cm(arena, p):
    """Pose sel dataset -> (x_cm, y_cm, yaw_deg atau None)."""
    x, y = arena.ke_cm(p[0], p[1])
    return (x, y, None if p[2] is None or np.isnan(p[2]) else math.degrees(p[2]))


def pilih_skenario(n_per_peta, seed=0, ids=(1, 2)):
    """Skenario satu robot & pasangan, dari rute uji dataset. Deterministik. Pasangan hanya
    bila ID 01 dan ID 02 sama-sama dipilih (dan datanya ada)."""
    import arena as A
    from drl import rollout as RO
    data = {rid: RO.muat_npz(os.path.join(AKAR, "drl", "data", f"demo_jauh_id{rid:02d}.npz")) for rid in ids}
    peta_asli = daftar_peta(data)
    satu, kandidat = [], {}
    for rid, d in data.items():
        pab = RO.PabrikEnv(konfig.profil(rid))
        for peta in peta_asli:
            ar = A.Arena(peta)
            idx = [e for e in np.flatnonzero(d["ep_uji"])
                   if d["ep_peta"][e] == peta and np.isnan(d["ep_rintangan"][e]).all()
                   and d["ep_margin"][e] >= konfig.SAFETY_MARGIN_CM and d["ep_langkah"][e] >= 3]
            idx = sorted(idx, key=lambda e: d["ep_biaya"][e])
            pilih = [idx[int(i)] for i in np.linspace(0, len(idx) - 1, min(n_per_peta, len(idx)))]
            for e in pilih:
                satu.append({"jenis": "satu", "peta": peta, "ids": [rid],
                             "awal": [_cm(ar, d["ep_awal"][e])], "tujuan": [_cm(ar, d["ep_tujuan"][e])],
                             "nama": f"ID{rid:02d} {peta} #{e}"})
            # kandidat pasangan: jalur A* (diputar ulang) dalam cm
            kand = []
            for e in idx[::max(1, len(idx) // 25)]:
                sk = RO.dari_demo(d, e)
                env, _, _ = pab.buat(sk)
                for a in sk.aksi_demo:
                    env.step(a)
                kand.append((e, np.array([ar.ke_cm(p[0], p[1]) for p in env.jalur()]),
                             _cm(ar, d["ep_awal"][e]), _cm(ar, d["ep_tujuan"][e])))
            kandidat[(rid, peta)] = kand

    pasangan = []
    for peta in (peta_asli if {1, 2} <= set(data) else []):
        n = 0
        for e1, j1, a1, t1 in kandidat[(1, peta)]:
            for e2, j2, a2, t2 in kandidat[(2, peta)]:
                if n >= 2:
                    break
                ttk = [a1, t1, a2, t2]
                jauh = all(math.hypot(p[0] - q[0], p[1] - q[1]) > 60 for i, p in enumerate(ttk)
                           for q in ttk[i + 1:])
                if not jauh:
                    continue
                dmin = np.min(np.hypot(j1[:, None, 0] - j2[None, :, 0], j1[:, None, 1] - j2[None, :, 1]))
                if dmin < 20:
                    pasangan.append({"jenis": "pasangan", "peta": peta, "ids": [1, 2],
                                     "awal": [a1, a2], "tujuan": [t1, t2],
                                     "nama": f"ID01+02 {peta} #{e1}/#{e2}"})
                    n += 1
            if n >= 2:
                break
    return satu, pasangan


def jalankan(sk):
    """Satu skenario di proses pekerja. Keluaran navigasi disimpan, bukan dicetak."""
    import arena as A
    import navigasi as N
    import wasit as W
    konfig.pasang_path_kamera()
    import area_constraint as AC

    args = SimpleNamespace(peta=sk["peta"], sel=None, tracker=konfig.PELACAK_BAWAAN,
                           perencana=sk["perencana"], simulasi=True, derau=sk["derau"], derau_deg=0.0,
                           margin=10.0, url=None, host=None, log=None)
    teks = io.StringIO()
    hasil = {"nama": sk["nama"], "perencana": sk["perencana"], "jenis": sk["jenis"], "robot": []}
    konfig.WASIT_ZONA = sk.get("wasit_zona", True)
    hasil["wasit_zona"] = konfig.WASIT_ZONA
    with contextlib.redirect_stdout(teks):
        konfig.pilih_robot(sk["ids"][0])
        nav = N.Navigasi(args, sk["ids"])
        for u, (x, y, yaw) in zip(nav.units, sk["awal"]):
            u.sim.x, u.sim.y, u.sim.yaw = x, y, yaw or 0.0
        rencana = {u.id: [] for u in nav.units}
        for u in nav.units:
            asli = u.per.rencanakan

            def hitung(*a, _asli=asli, _log=rencana[u.id], **k):
                j, info = _asli(*a, **k)
                _log.append((info.get("perencana", "A*") if j else "gagal", info.get("waktu_s", 0.0)))
                return j, info
            u.per.rencanakan = hitung

        t0 = time.time()
        nav.satu_langkah(t0)
        for u, tj in zip(nav.units, sk["tujuan"]):
            u.terima_tujuan(tj)
        batas = BATAS_S_PASANGAN if len(nav.units) > 1 else BATAS_S_SATU
        jarak_zona = {u.id: float("inf") for u in nav.units}
        langgar = {u.id: False for u in nav.units}
        jarak_robot, t_tiba = float("inf"), {}
        while True:
            now = time.time()
            nav.satu_langkah(now)
            badan = {}
            for u in nav.units:
                fp = A.jejak_badan(u.sim.x, u.sim.y, u.sim.yaw)
                badan[u.id] = fp
                s, _, jarak = AC.evaluasi(fp, nav.arena.zona, 0.0)
                if np.isfinite(jarak):
                    jarak_zona[u.id] = min(jarak_zona[u.id], jarak)
                langgar[u.id] |= s == AC.PELANGGARAN
                if u.state == N.SELESAI and u.id not in t_tiba:
                    t_tiba[u.id] = now - t0
            if len(badan) > 1:
                b = list(badan.values())
                jarak_robot = min(jarak_robot, W.jarak_poligon(b[0], b[1]))
            gagal = [u for u in nav.units if u.state == N.SIAP and u.tracker is None
                     and not u._sedang_merencana and now - t0 > 1.0]
            akhir = all(u.state in (N.SELESAI, N.BAHAYA) or u in gagal for u in nav.units)
            if akhir or now - t0 > batas:
                break
            sisa = konfig.RC_PERIODE_S - (time.time() - now)
            if sisa > 0:
                time.sleep(sisa)
        for u in nav.units:
            tj = sk["tujuan"][nav.units.index(u)]
            e_pos = math.hypot(u.sim.x - tj[0], u.sim.y - tj[1])
            e_yaw = None if tj[2] is None else abs((u.sim.yaw - tj[2] + 180) % 360 - 180)
            hasil["robot"].append({
                "id": u.id, "tiba": u.state == N.SELESAI, "state": u.state, "alasan": u.alasan,
                "t_s": t_tiba.get(u.id), "galat_cm": e_pos, "galat_deg": e_yaw,
                "masuk_zona": langgar[u.id], "jarak_zona_min_cm": jarak_zona[u.id],
                "henti_zona": getattr(u, "henti_zona", 0), "rencana": rencana[u.id]})
            u.tutup()
        hasil["jarak_robot_min_cm"] = jarak_robot if len(nav.units) > 1 else None
        hasil["durasi_s"] = time.time() - t0
    hasil["log_akhir"] = teks.getvalue().splitlines()[-8:]
    return hasil


def ringkas(hasil):
    baris = []
    for per in sorted({h["perencana"] for h in hasil}):
        for jenis in ("satu", "pasangan"):
            h = [x for x in hasil if x["perencana"] == per and x["jenis"] == jenis]
            if not h:
                continue
            r = [rb for x in h for rb in x["robot"]]
            tiba = [rb for rb in r if rb["tiba"]]
            oleh = {}
            for rb in r:
                for p, _ in rb["rencana"]:
                    oleh[p] = oleh.get(p, 0) + 1
            baris.append({
                "perencana": per, "jenis": jenis, "skenario": len(h), "robot": len(r),
                "tiba": len(tiba), "masuk_zona": sum(rb["masuk_zona"] for rb in r),
                "henti_zona": sum(rb.get("henti_zona", 0) for rb in r),
                "robot_henti_zona": sum(rb.get("henti_zona", 0) > 0 for rb in r),
                "galat_cm_rata": float(np.mean([rb["galat_cm"] for rb in tiba])) if tiba else None,
                "galat_cm_maks": float(np.max([rb["galat_cm"] for rb in tiba])) if tiba else None,
                "t_s_median": float(np.median([rb["t_s"] for rb in tiba])) if tiba else None,
                "jarak_zona_min_cm": float(np.min([rb["jarak_zona_min_cm"] for rb in r])),
                "jarak_robot_min_cm": (float(np.min([x["jarak_robot_min_cm"] for x in h]))
                                       if jenis == "pasangan" else None),
                "rencana_oleh": oleh,
            })
    return baris


def main():
    ap = argparse.ArgumentParser(description="Robot berjalan di simulasi dengan A* / DQfD")
    ap.add_argument("--perencana", default="astar,dqfd")
    ap.add_argument("--n", type=int, default=5, help="rute per peta per robot")
    ap.add_argument("--derau", type=float, default=1.2, help="derau pose (cm), seperti uji README")
    ap.add_argument("--pekerja", type=int, default=6,
                    help="proses serempak. Jangan terlalu banyak: simulasinya waktu nyata")
    ap.add_argument("--tanpa-pasangan", action="store_true")
    ap.add_argument("--hanya-pasangan", action="store_true")
    ap.add_argument("--ulang", type=int, default=1, help="ulangi tiap skenario (derau acak berbeda)")
    ap.add_argument("--id", default="1,2", help="robot yang diuji (pasangan hanya bila 1 dan 2)")
    ap.add_argument("--tanpa-wasit-zona", action="store_true",
                    help="pembanding: aturan zona wasit dimatikan (hanya estop pemantau keselamatan)")
    ap.add_argument("--keluaran", default=None, help="berkas JSON hasil (bawaan model_dqn/uji_navigasi.json)")
    args = ap.parse_args()
    import multiprocessing as mp

    satu, pasangan = pilih_skenario(args.n, ids=tuple(int(t) for t in args.id.split(",") if t.strip()))
    if args.tanpa_pasangan:
        pasangan = []
    if args.hanya_pasangan:
        satu = []
    daftar = [dict(s, perencana=p, derau=args.derau, wasit_zona=not args.tanpa_wasit_zona)
              for p in args.perencana.split(",") for s in satu + pasangan for _ in range(args.ulang)]
    print(f"[i] {len(satu)} skenario satu robot + {len(pasangan)} pasangan x "
          f"{args.perencana} = {len(daftar)} jalan, {args.pekerja} pekerja, derau {args.derau} cm")
    t0 = time.time()
    hasil = []
    # ProcessPoolExecutor, bukan Pool: pekerja Pool adalah proses daemon dan
    # tidak boleh membuat proses anak - padahal setiap Navigasi membuat proses
    # perencana per robot.
    from concurrent.futures import ProcessPoolExecutor, as_completed
    with ProcessPoolExecutor(args.pekerja, mp_context=mp.get_context("spawn")) as pool:
        for h in (f.result() for f in as_completed([pool.submit(jalankan, d) for d in daftar])):
            hasil.append(h)
            r = " ".join(f"ID{rb['id']:02d}:{'tiba' if rb['tiba'] else rb['state']}"
                         f"{'!ZONA' if rb['masuk_zona'] else ''}"
                         f"{' henti-zona x' + str(rb['henti_zona']) if rb.get('henti_zona') else ''}"
                         for rb in h["robot"])
            print(f"  [{time.time() - t0:5.0f} s] {h['perencana']:<10} {h['nama']:<28} {r}"
                  + (f"  jarak robot {h['jarak_robot_min_cm']:.1f} cm" if h["jarak_robot_min_cm"] else ""),
                  flush=True)
    ring = ringkas(hasil)
    print("\n  perencana   jenis     tiba      masuk zona  henti zona  galat akhir rata/maks  waktu median  "
          "jarak zona min  jarak robot min  jalur oleh")
    for b in ring:
        jr = f"{b['jarak_robot_min_cm']:.1f} cm" if b["jarak_robot_min_cm"] is not None else "-"
        print(f"  {b['perencana']:<10} {b['jenis']:<9} {b['tiba']:>3}/{b['robot']:<4} {b['masuk_zona']:>5}"
              f"       {b['henti_zona']:>3} ({b['robot_henti_zona']} robot)"
              f"  {b['galat_cm_rata'] or 0:5.1f} / {b['galat_cm_maks'] or 0:4.1f} cm"
              f"      {b['t_s_median'] or 0:5.1f} s      {b['jarak_zona_min_cm']:6.2f} cm      {jr:>9}"
              f"      {b['rencana_oleh']}")
    keluar = args.keluaran or os.path.join(AKAR, "model_dqn", "uji_navigasi_pasangan.json" if args.hanya_pasangan
                                           else "uji_navigasi.json")
    with open(keluar, "w", encoding="utf-8") as f:
        json.dump({"ringkasan": ring, "hasil": hasil}, f, indent=1,
                  default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print(f"\n  -> {keluar}  ({(time.time() - t0) / 60:.1f} menit)")


if __name__ == "__main__":
    main()
