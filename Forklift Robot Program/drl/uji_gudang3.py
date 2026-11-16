"""
uji_gudang3.py - Hybrid A* + RS vs DQfD murni di peta simulasi Gudang3 (760 x 660 cm).

Rute OPERASIONAL, bukan pose acak: stasiun = dok terima (D), slot rak (R, robot
menghadap rak dari lorong), teluk kirim (K), pengisian daya (C). Alur:
    simpan    dok -> slot rak          ambil    slot rak -> teluk kirim
    pindah    slot rak -> slot rak     kembali  teluk kirim -> dok
    isi_daya  dok/teluk/slot -> C      keluar   C -> dok
Kelompok besar dicuplik acak (--per-alur, benih tetap). Model DQfD = model robot
yang ada (dilatih di arena 300 x 270 cm), tanpa latih ulang.

    python -m drl.uji_gudang3 --id 1
    python -m drl.uji_gudang3 --id 1 --per-alur 150 --keluar model_dqn/uji_gudang3_id01.json
"""
import argparse
import json
import math
import os
import random
import sys

import numpy as np

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if AKAR not in sys.path:
    sys.path.insert(0, AKAR)

import konfig            # noqa: E402
import arena as A        # noqa: E402
import rencana as R      # noqa: E402

PETA = "Gudang3"
JENIS = ("astar", "dqfd-murni")
# Poros depan berjarak ini dari muka rak (paling dekat yang bebas pada margin 8 cm: 18 cm).
# Lorong rak 80 cm: diukur, pose tegak lurus rak baru bisa dimasuki DAN ditinggalkan mulai
# lebar lorong 70 cm (60 cm: tidak bisa keluar; 65 cm: sebagian).
JARAK_RAK_CM = 20.0


def stasiun(per):
    """{nama: (x, y, yaw)} - hanya pose yang bebas pada margin penuh."""
    s = {}
    for i, x in enumerate([27.5, 87.5, 147.5, 207.5]):
        s[f"D{i + 1}"] = (x, 35.0, -90.0)                      # di dok, menghadap dinding (masuk maju)
    for i in range(6):
        s[f"K{i + 1}"] = (147.5 + 60.0 * i, 625.0, 90.0)      # di teluk, menghadap dinding
    s["C1"], s["C2"] = (656.0, 228.0, -90.0), (726.0, 228.0, -90.0)
    # Slot rak: tiap 40 cm sepanjang rak, dari kedua sisi tiap baris.
    baris = [190, 294, 398, 502]
    for b, y in enumerate(baris):
        for sisi, (ys, yaw) in (("a", (y - JARAK_RAK_CM, 90.0)), ("b", (y + 24 + JARAK_RAK_CM, -90.0))):
            for blok, (x0, x1) in (("L", (75, 325)), ("R", (400, 560))):
                for k, x in enumerate(range(x0 + 20, x1, 40)):
                    s[f"R{b + 1}{sisi}{blok}{k + 1}"] = (float(x), ys, yaw)
    # Blok rak padat: lorong 50 cm terlalu sempit untuk pose tegak lurus rak, jadi
    # robot berhenti sejajar lorong (menghadap ujung buntu).
    for k, y in enumerate((420.0, 460.0)):
        s[f"P{k + 1}"] = (699.0, y, -90.0)
    return {n: p for n, p in s.items() if per.pose_bebas(*p)}


def stasiun_gudang4(per):
    """Stasiun Gudang4 (peta uji kedua, tidak dipakai melatih): dok di dinding kiri,
    teluk di dinding kanan, pengisian daya di dinding atas, slot menghadap rak tegak."""
    s = {}
    for i, y in enumerate([177.5, 237.5, 297.5, 357.5, 417.5]):
        s[f"D{i + 1}"] = (35.0, y, 180.0)                     # dok, menghadap dinding kiri
        s[f"K{i + 1}"] = (725.0, y, 0.0)                      # teluk, menghadap dinding kanan
    s["C1"], s["C2"] = (572.5, 35.0, -90.0), (632.5, 35.0, -90.0)
    for b, x in enumerate([150, 254, 358, 462, 566]):
        for sisi, (xs, yaw) in (("a", (x - JARAK_RAK_CM, 0.0)), ("b", (x + 24 + JARAK_RAK_CM, 180.0))):
            for blok, (y0, y1) in (("U", (150, 290)), ("L", (370, 510))):
                for k, y in enumerate(range(y0 + 20, y1, 40)):
                    s[f"R{b + 1}{sisi}{blok}{k + 1}"] = (xs, float(y), yaw)
    return {n: p for n, p in s.items() if per.pose_bebas(*p)}


STASIUN = {"Gudang3": stasiun, "Gudang4": stasiun_gudang4}


def susun_rute(st, per_alur, benih):
    rng = random.Random(benih)
    dok = [n for n in st if n[0] == "D"]
    teluk = [n for n in st if n[0] == "K"]
    daya = [n for n in st if n[0] == "C"]
    slot = [n for n in st if n[0] in "RP"]
    alur = {
        "simpan": [(a, b) for a in dok for b in slot],
        "ambil": [(a, b) for a in slot for b in teluk],
        "pindah": [(a, b) for a in slot for b in slot if a != b],
        "kembali": [(a, b) for a in teluk for b in dok],
        "isi_daya": [(a, b) for a in dok + teluk + slot for b in daya],
        "keluar": [(a, b) for a in daya for b in dok],
    }
    rute = []
    for nama, pas in alur.items():
        if len(pas) > per_alur:
            pas = rng.sample(pas, per_alur)
        rute += [(nama, a, b) for a, b in pas]
    return rute


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--per-alur", type=int, default=150)
    ap.add_argument("--benih", type=int, default=3)
    ap.add_argument("--keluar", default=None)
    ap.add_argument("--peta", default=PETA, choices=sorted(STASIUN))
    ap.add_argument("--rintangan", action="store_true",
                    help="letakkan badan robot lain di tengah jalur A* tanpa rintangan (bila titik itu > 60 cm "
                         "dari awal dan tujuan), lalu rencanakan ulang dengan rintangan itu - uji perubahan rintangan")
    ap.add_argument("--skala-maks", type=float, default=None,
                    help="batas pengali anggaran node DQfD (1 = anggaran lama 1200/600)")
    ap.add_argument("--dir-model", default=None,
                    help="folder model DQfD (bawaan model_dqn/); mis. model_dqn_gudang3 = model yang dilatih di Gudang3")
    a = ap.parse_args()
    if a.dir_model:
        from drl import perencana_dqfd as PD
        PD.DIR_MODEL = os.path.abspath(a.dir_model)
    if a.skala_maks is not None:
        from drl import perencana_dqfd as PD
        PD.SKALA_ANGGARAN_MAKS = a.skala_maks
    konfig.pilih_robot(a.id)
    ar = A.Arena(a.peta)
    per = {j: R.Perencana(ar, konfig.profil(a.id), jenis=j) for j in JENIS}
    from drl.banding_baseline import PencatatNode
    catat = {j: PencatatNode(per[j]) for j in JENIS}     # node SEMUA panggilan perencana per rute
    st = STASIUN[a.peta](per["astar"])
    rute = susun_rute(st, a.per_alur, a.benih)
    print(f"{a.peta}: {len(st)} stasiun, {len(rute)} rute, robot ID {a.id:02d}", flush=True)
    for j in JENIS:                                            # pemanasan numba dan cache
        per[j].rencanakan(st["D2"], st["K3"], None, None)
    hasil = []
    for i, (alur, na, nb) in enumerate(rute):
        r = dict(alur=alur, dari=na, ke=nb, awal=st[na], tujuan=st[nb],
                 lurus_cm=math.hypot(st[na][0] - st[nb][0], st[na][1] - st[nb][1]))
        rint = None
        if a.rintangan:
            # Badan robot lain diparkir di tengah jalur A* yang direncanakan TANPA rintangan.
            dasar, _ = per["astar"].rencanakan(st[na], st[nb], None, None)
            if dasar:
                q = dasar[len(dasar) // 2]
                xc, yc, yd = ar.pose_ke_cm(q[0], q[1], q[2])
                if min(math.hypot(xc - st[na][0], yc - st[na][1]), math.hypot(xc - st[nb][0], yc - st[nb][1])) > 60.0:
                    rint = [np.asarray(A.jejak_badan(xc, yc, yd), np.float64)]
                    r["rintangan_di"] = (xc, yc, yd)
        r["ada_rintangan"] = rint is not None
        for j in JENIS:
            catat[j].total = 0
            jalur, info = per[j].rencanakan(st[na], st[nb], rint, None)
            st_ = per[j].planner.last_stats or {}
            # Jarak bebas terkecil badan (sudah digembungkan margin yang dipakai) terhadap zona,
            # sepanjang jalur, dalam sel: >= 0 berarti seluruh jalur bebas tabrakan.
            bebas = None
            if jalur:
                k = per[j].kendaraan_dipakai
                bebas = float(min(ar.omap.pose_margin(k, q[0], q[1], q[2]) for q in jalur))   # jalur dalam sel
            r[j] = dict(ok=bool(jalur), cpu_ms=info.get("cpu_s", 0.0) * 1000.0, panjang_cm=info.get("panjang_cm"),
                        perencana=info.get("perencana"), margin_cm=info.get("margin_cm"),
                        alasan=info.get("alasan"), node_terakhir=st_.get("expansions"),
                        node=catat[j].total, bebas_min_sel=bebas,
                        ganti_arah=(sum(1 for p, q in zip(jalur, jalur[1:]) if len(p) > 3 and p[3] != q[3])
                                    if jalur else None))
        hasil.append(r)
        if i % 50 == 0:
            print(f"  {i}/{len(rute)}", flush=True)
    keluar = a.keluar or os.path.join(AKAR, "model_dqn", f"uji_{a.peta.lower()}_id{a.id:02d}.json")
    with open(keluar, "w") as f:
        json.dump(dict(peta=a.peta, robot=a.id, stasiun=st, rute=hasil), f)
    print("tersimpan:", keluar)


if __name__ == "__main__":
    main()
