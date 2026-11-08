"""
ukur_node.py - waktu proses PER NODE: Hybrid A* + RS (astar) vs DQfD (dqfd-murni), satu robot.

    python -m drl.ukur_node --id 1                  # tiga uji berurutan, tiap uji di proses sendiri
    python -m drl.ukur_node --id 1 --uji waktu      # waktu proses per node (CPU thread, median 3 ulangan)
    python -m drl.ukur_node --id 1 --uji rincian    # isi waktu per node DQfD, forward pass, jumlah parameter
    python -m drl.ukur_node --id 1 --uji tabrak     # apakah Q tahu aksi mana yang menabrak?
    python -m drl.ukur_node --id 1 --uji varian     # DQfD eager / +jalan pintas / lazy / lazy + jalan pintas
    python -m drl.ukur_node --id 1 --maks 10        # uji cepat: 10 rute pertama tiap peta

Node = satu ekspansi Hybrid A* atau satu langkah DQfD (env.step), dijumlah atas semua
panggilan perencana di Perencana.rencanakan() (sapuan arah hadap, margin cadangan).
Rute = rute uji dataset (drl.banding_baseline.rute_uji), sama untuk kedua perencana.
Pelengkap drl/banding_komputasi.py, yang melaporkan waktu per RUTE.

--pencarian  eager / lazy untuk DQfD di uji waktu dan rincian; bawaan seperti robot
             (perencana_dqfd.PENCARIAN_ROBOT = lazy). Uji tabrak selalu eager.

uji waktu    waktu CPU thread perencana (info["cpu_s"]) dan jam dinding, tiap rute
             --ulang kali, diambil median, setelah pemanasan. Per node dilaporkan tiga
             cara: median per rute (bias: biaya tetap tiap rencana dibagi node yang
             sedikit), total waktu / total node, dan kemiringan regresi waktu rute
             terhadap jumlah node.
uji rincian  jam dinding tiap bagian pencarian DQfD, EKSKLUSIF (jumlahnya = total),
             pemanasan tidak dihitung; isi _hitung_calon (primitif dan cek tabrakan per
             panggilan); forward pass satu keadaan (numpy per lapis, PyTorch pembanding
             bila terpasang); jumlah parameter jaringan. Pencatat waktu menambah ~2%.
uji tabrak   Q ke-14 aksi TANPA mask di tiap ekspansi terbaik-dulu, dibanding status
             tiap aksi (sah / menabrak / sel sudah dikunjungi / mundur dilarang).
uji varian   DQfD dengan empat cara pencarian (VARIAN): waktu proses per node seperti
             uji waktu, lalu satu lintasan hitung: simulasi primitif dengan cek tabrakan
             per node, porsi keadaan "ruang terbuka", jalur dan biaya dibanding eager,
             dan pemeriksaan jalan pintas (setiap kali ruang terbuka, ke-14 primitif
             juga disimulasi lengkap dan hasilnya harus sama persis).

uji rincian dan tabrak menjalankan SALINAN BERTANDA perencana_dqfd.rencanakan_terbaik
(_terbaik_bertanda) atau rencanakan_terbaik_malas (_malas_bertanda): logika sama,
ditambah pencatat. Bila fungsi aslinya berubah, salinannya harus disamakan;
_periksa_salinan() memperingatkan lewat md5 sumbernya.

Jangan jalankan bersamaan dengan pelatihan / uji lain: yang diukur waktu.
Keluaran: model_dqn/ukur_node_id0N_<mesin>.json, satu kunci per uji.
Catatan Obsidian: 20 Waktu Proses per Node ver-1-6.
"""
import argparse
import collections
import hashlib
import heapq
import inspect
import itertools
import json
import os
import platform
import subprocess
import sys
import time

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import forklift_planner as FP                   # noqa: E402
import konfig                                   # noqa: E402
import rencana as R                             # noqa: E402
from drl import banding_baseline as BB          # noqa: E402
from drl import lingkungan as L                 # noqa: E402
from drl import perencana_dqfd as PD            # noqa: E402
from drl import jaringan_numpy as JN            # noqa: E402
from drl.jaringan_numpy import jalur_cache      # noqa: E402

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JENIS = ("astar", "dqfd-murni")
UJI = ("waktu", "rincian", "tabrak", "varian")
# uji varian: (nama, pencarian terbaik-dulu, jalan pintas ruang terbuka)
VARIAN = (("eager", "eager", False), ("eager+pintas", "eager", True),
          ("lazy", "lazy", False), ("lazy+pintas", "lazy", True))
# md5 sumber fungsi asli saat salinan bertanda di sini dibuat (_periksa_salinan).
_MD5_TERBAIK = "5db045286f75970e78dfeb42030b32e8"      # rencanakan_terbaik -> _terbaik_bertanda
_MD5_MALAS = "b627aada09a4903ebc63d1a96f2dfd1a"        # rencanakan_terbaik_malas -> _malas_bertanda
pc = time.perf_counter


# ------------------------------------------------------------------ bersama
def _cpu():
    if sys.platform == "darwin":
        try:
            return subprocess.run(["sysctl", "-n", "machdep.cpu.brand_string"], capture_output=True,
                                  text=True, timeout=5).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            pass
    return platform.processor() or platform.machine()


def _mesin():
    try:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=AKAR, capture_output=True,
                                text=True, timeout=10).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        commit = ""
    try:
        import numba
        v_numba = numba.__version__
    except ImportError:
        v_numba = None
    return {"platform": platform.platform(), "cpu": _cpu(), "python": sys.version.split()[0],
            "numpy": np.__version__, "numba": v_numba, "commit": commit,
            "tanggal": time.strftime("%Y-%m-%d %H:%M")}


def _keluaran_bawaan(rid):
    mesin = {"darwin": "mac", "win32": "windows"}.get(sys.platform, sys.platform)
    return os.path.join(AKAR, "model_dqn", f"ukur_node_id{rid:02d}_{mesin}.json")


def _simpan(jalur, isi):
    """Gabungkan isi ke berkas JSON hasil; kunci uji lain tetap."""
    d = {}
    if os.path.exists(jalur):
        with open(jalur, encoding="utf-8") as f:
            d = json.load(f)
    d.update(isi)
    d["mesin"] = _mesin()
    os.makedirs(os.path.dirname(jalur) or ".", exist_ok=True)
    sementara = jalur + ".tmp"
    with open(sementara, "w", encoding="utf-8") as f:
        json.dump(d, f, indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    os.replace(sementara, jalur)
    print(f"[OK] {jalur} <- {', '.join(isi)}", flush=True)


def _peta_rute(rid, maks):
    for peta in BB.daftar_peta(rid):
        ar, rute = BB.rute_uji(rid, peta)
        yield peta, ar, (rute[:maks] if maks else rute)


def _perencana(ar, prof, jenis):
    """Perencana seperti navigasi.py, semua margin cadangan sudah dibuat (tidak ikut terukur)."""
    per = R.Perencana(ar, prof, jenis=jenis)
    per._untuk(konfig.SAFETY_MARGIN_CM)
    for m in konfig.MARGIN_CADANGAN_CM:
        per._untuk(m)
    return per


def _periksa_salinan():
    for asli, salinan, kunci, md5_lama in ((PD.rencanakan_terbaik, "_terbaik_bertanda", "_MD5_TERBAIK", _MD5_TERBAIK),
                                           (PD.rencanakan_terbaik_malas, "_malas_bertanda", "_MD5_MALAS", _MD5_MALAS)):
        md5 = hashlib.md5(inspect.getsource(asli).encode()).hexdigest()
        if md5 != md5_lama:
            print(f"[!] perencana_dqfd.{asli.__name__} berubah (md5 sumber {md5}). Samakan {salinan} "
                  f"di drl/ukur_node.py dengan fungsi aslinya, lalu perbarui {kunci}.", flush=True)


def _malas_bertanda(net, env, anggaran, batas_waktu, bobot, catat=None, belajar=False, selektif=None, ambang=None):
    """SALINAN perencana_dqfd.rencanakan_terbaik_malas, logika sama persis, dengan kait
    catat(bagian, detik) untuk waktu per bagian."""
    tandai = catat or (lambda k, dt: None)
    kepala = bool(getattr(net, "kepala_tabrak", False))
    ambang = PD.AMBANG_TABRAK if ambang is None else ambang
    if belajar and not kepala:
        raise ValueError("pencarian 'belajar' butuh model dengan kepala kelayakan (kepala_tabrak)")
    t0 = pc()
    snap, induk, potong, aksi_simpul, q_simpul, p_simpul = [env.simpan_keadaan()], [-1], [[]], [None], {}, {}
    heap, urut = [], itertools.count()
    PERKIRAAN, TERBUKA, TEPAT = 0, 1, 2

    def buka(i):
        a0 = pc()
        calon = env.calon_malas()
        if calon:
            if kepala:
                q, p = net.q_tabrak(env.observasi())
            else:
                q, p = net.q(env.observasi()), None
            q_simpul[i], p_simpul[i] = q, p
            g = env.biaya
            for a, c, terbuka in calon:
                h = max(0.0, -float(q[a]) * PD.SKALA_REWARD - c)
                f = g + c + bobot * h
                if p is not None and not terbuka and p[a] >= ambang:
                    if belajar:
                        continue
                    f += PD.URUTAN_BELAKANG
                heapq.heappush(heap, (f, next(urut), i, a, TERBUKA if terbuka else PERKIRAAN))
        tandai("buka", pc() - a0)

    def pulihkan(i):
        a0 = pc()
        rantai = []
        j = i
        while j > 0:
            rantai.append(potong[j])
            j = induk[j]
        env._pose = [p for bagian in reversed(rantai) for p in bagian]
        a1 = pc()
        env.pulihkan_keadaan(snap[i], hitung_calon=False)
        tandai("susun_jalur", a1 - a0)
        tandai("pulihkan_keadaan", pc() - a1)

    buka(0)
    di = 0
    dicoba = pindah = dibuang = masuk_ulang = tanpa_cek_n = 0
    alasan = "buntu"
    jarak_min = PD._jarak_tujuan(env)

    def hasil(sukses, alasan_, jm, i_akhir=None, a_akhir=None):
        aksi_jalur = None
        if sukses:
            aksi_jalur = [a_akhir]
            j = i_akhir
            while j > 0:
                aksi_jalur.append(aksi_simpul[j])
                j = induk[j]
            aksi_jalur.reverse()
        return {"sukses": sukses, "alasan": alasan_, "dicoba": dicoba, "mundur": pindah,
                "waktu_s": pc() - t0, "jarak_min": jm, "dibuang": dibuang, "masuk_ulang": masuk_ulang,
                "tanpa_cek": tanpa_cek_n, "aksi_jalur": aksi_jalur}

    while heap:
        if dicoba >= anggaran:
            alasan = "anggaran habis"
            break
        if batas_waktu is not None and pc() - t0 > batas_waktu:
            alasan = "batas waktu habis"
            break
        a0 = pc()
        f, _, i, a, jenis = heapq.heappop(heap)
        tandai("pop", pc() - a0)
        if di != i:
            pulihkan(i)
            di = i
            pindah += 1
        a1 = pc()
        tanpa_cek = belajar or jenis == TERBUKA or (
            selektif is not None and jenis == PERKIRAAN and p_simpul.get(i) is not None and p_simpul[i][a] < selektif)
        tanpa_cek_n += tanpa_cek and jenis != TERBUKA
        calon = env.siapkan_aksi(a, tanpa_cek=tanpa_cek)
        tandai("siapkan", pc() - a1)
        if calon is None:
            dibuang += 1
            continue
        if jenis == PERKIRAAN and not tanpa_cek:
            a2 = pc()
            c = calon[1]
            fe = env.biaya + c + bobot * max(0.0, -float(q_simpul[i][a]) * PD.SKALA_REWARD - c)
            ulang = bool(heap) and fe > f and fe > heap[0][0]
            tandai("f_tepat", pc() - a2)
            if ulang:
                heapq.heappush(heap, (fe, next(urut), i, a, TEPAT))
                masuk_ulang += 1
                continue
        n0 = len(env._pose)
        a3 = pc()
        _, _, akhir, info = env.step(a, hitung_calon=False)
        a4 = pc()
        tandai("step", a4 - a3)
        dicoba += 1
        jarak_min = min(jarak_min, PD._jarak_tujuan(env))
        if info["sukses"]:
            tandai("simpan", pc() - a4)
            return hasil(True, info["alasan"], 0.0, i, a)
        j = len(snap)
        snap.append(env.simpan_keadaan())
        induk.append(i)
        aksi_simpul.append(a)
        potong.append(env._pose[n0:])
        tandai("simpan", pc() - a4)
        if akhir or env.langkah_ke >= PD.MAKS_LANGKAH:
            di = None
            continue
        buka(j)
        di = j
    return hasil(False, alasan, jarak_min)


def _terbaik_bertanda(net, env, anggaran, batas_waktu, bobot, catat=None, saat_buka=None):
    """SALINAN perencana_dqfd.rencanakan_terbaik, logika sama persis, dengan dua kait:
    catat(bagian, detik) untuk waktu per bagian, dan saat_buka(env, q) di setiap buka()
    (dengan kait ini Q juga dihitung untuk simpul tanpa aksi sah)."""
    tandai = catat or (lambda k, dt: None)
    t0 = pc()
    snap, induk, potong = [env.simpan_keadaan()], [-1], [[]]
    heap, urut = [], itertools.count()

    def buka(i):
        a0 = pc()
        m = env.mask()
        sah = np.flatnonzero(m)
        q = None
        if saat_buka is not None:
            q = net.q(env.observasi())
            saat_buka(env, q)
        if len(sah):
            if q is None:
                q = net.q(env.observasi())
            g = env.biaya
            for a in sah:
                a = int(a)
                c = env._calon[a][1]
                h = max(0.0, -float(q[a]) * PD.SKALA_REWARD - c)
                heapq.heappush(heap, (g + c + bobot * h, next(urut), i, a))
        tandai("buka", pc() - a0)

    def pulihkan(i):
        a0 = pc()
        rantai = []
        j = i
        while j > 0:
            rantai.append(potong[j])
            j = induk[j]
        env._pose = [p for bagian in reversed(rantai) for p in bagian]
        a1 = pc()
        env.pulihkan_keadaan(snap[i])
        tandai("susun_jalur", a1 - a0)
        tandai("pulihkan_keadaan", pc() - a1)

    buka(0)
    di = 0
    dicoba = pindah = 0
    alasan = "buntu"
    jarak_min = PD._jarak_tujuan(env)
    while heap:
        if dicoba >= anggaran:
            alasan = "anggaran habis"
            break
        if batas_waktu is not None and pc() - t0 > batas_waktu:
            alasan = "batas waktu habis"
            break
        a0 = pc()
        _, _, i, a = heapq.heappop(heap)
        tandai("pop", pc() - a0)
        if di != i:
            pulihkan(i)
            di = i
            pindah += 1
        a1 = pc()
        ok = env.mask()[a]
        tandai("cek_mask", pc() - a1)
        if not ok:
            continue
        n0 = len(env._pose)
        a2 = pc()
        _, _, akhir, info = env.step(a)
        a3 = pc()
        tandai("step", a3 - a2)
        dicoba += 1
        jarak_min = min(jarak_min, PD._jarak_tujuan(env))
        if info["sukses"]:
            tandai("simpan", pc() - a3)
            return {"sukses": True, "alasan": info["alasan"], "dicoba": dicoba, "mundur": pindah,
                    "waktu_s": pc() - t0, "jarak_min": 0.0}
        j = len(snap)
        snap.append(env.simpan_keadaan())
        induk.append(i)
        potong.append(env._pose[n0:])
        tandai("simpan", pc() - a3)
        if akhir or env.langkah_ke >= PD.MAKS_LANGKAH:
            di = None
            continue
        buka(j)
        di = j
    return {"sukses": False, "alasan": alasan, "dicoba": dicoba, "mundur": pindah,
            "waktu_s": pc() - t0, "jarak_min": jarak_min}


# ------------------------------------------------------------------ uji waktu
def uji_waktu(args):
    rid = args.id
    konfig.pilih_robot(rid)
    prof = konfig.profil(rid)
    hasil = {j: [] for j in JENIS}
    pencarian_dipakai = {}
    t_mulai = time.time()
    for peta, ar, rute in _peta_rute(rid, args.maks):
        for jenis in JENIS:
            per = _perencana(ar, prof, jenis)
            if jenis != "astar":
                _pasang_kepala_p(per, args)
                if args.pencarian:
                    _atur_varian(per, args.pencarian, False)
            cat = BB.PencatatNode(per)
            if jenis != "astar":
                pl0 = per.planner
                pencarian_dipakai[jenis] = (f"{pl0.pencarian}, P {type(pl0.net).__name__}"
                                            + (", forward gabungan" if getattr(pl0.net, "gabung", False) else ""))
            r0 = rute[0]
            per.rencanakan(r0["awal"], r0["tujuan"], r0["rintangan"])            # pemanasan
            for r in rute:
                wall, cpu = [], []
                for _ in range(args.ulang):
                    cat.total = 0
                    t0 = pc()
                    jalur, info = per.rencanakan(r["awal"], r["tujuan"], r["rintangan"])
                    wall.append(pc() - t0)
                    cpu.append(info["cpu_s"])
                hasil[jenis].append({"peta": peta, "e": r["e"], "sukses": bool(jalur), "node": cat.total,
                                     "ms": 1000 * float(np.median(wall)), "cpu_ms": 1000 * float(np.median(cpu)),
                                     "rs_ms": 1000 * info.get("rincian_s", {}).get("rs", 0.0),
                                     "panjang_cm": float(info.get("panjang_cm", 0.0)) if jalur else None,
                                     "perencana": info.get("perencana")})
            print(f"  {peta:<18} {jenis:<11} {len(rute):3d} rute ({time.time() - t_mulai:.0f} s)", flush=True)

    ring, regresi = {}, {}
    print(f"\n=== waktu proses per node, rute uji ID {rid:02d}")
    for jenis, xs in hasil.items():
        ok = [x for x in xs if x["node"] > 0]
        cpu = np.array([x["cpu_ms"] / x["node"] for x in ok])
        wall = np.array([x["ms"] / x["node"] for x in ok])
        node = np.array([x["node"] for x in ok], float)
        n_all = np.array([x["node"] for x in xs], float)
        c_all = np.array([x["cpu_ms"] for x in xs])
        b, a = np.polyfit(n_all, c_all, 1)
        regresi[jenis] = {"ms_per_node": float(b), "tetap_ms": float(a)}
        ring[jenis] = {"pencarian": None if jenis == "astar" else pencarian_dipakai.get(jenis),
                       "rute": len(xs), "rute_node_positif": len(ok), "node_total": int(n_all.sum()),
                       "tiba": 100.0 * float(np.mean([x["sukses"] for x in xs])),
                       "cpu_ms_per_node_median": float(np.median(cpu)),
                       "cpu_ms_per_node_p10": float(np.percentile(cpu, 10)),
                       "cpu_ms_per_node_p90": float(np.percentile(cpu, 90)),
                       "cpu_ms_per_node_total": float(sum(x["cpu_ms"] for x in ok) / node.sum()),
                       "ms_per_node_median": float(np.median(wall)),
                       "node_median": float(np.median(node)), "node_p90": float(np.percentile(node, 90)),
                       "cpu_ms_median": float(np.median(c_all)), "cpu_ms_p90": float(np.percentile(c_all, 90)),
                       "cpu_ms_rata": float(np.mean(c_all)), "cpu_ms_maks": float(np.max(c_all)),
                       "panjang_cm_median": float(np.median([x["panjang_cm"] for x in xs if x["panjang_cm"]]))
                       if any(x["panjang_cm"] for x in xs) else None}
        g = ring[jenis]
        print(f"{jenis:<11}: {g['rute']} rute, tiba {g['tiba']:.1f}% | per node: total/total "
              f"{g['cpu_ms_per_node_total']:.3f} ms, regresi {b:.3f} ms (+{a:.2f} ms tetap), median per rute "
              f"{g['cpu_ms_per_node_median']:.3f} ms | node median {g['node_median']:.0f} (p90 {g['node_p90']:.0f}), "
              f"total {g['node_total']} | per rute median {g['cpu_ms_median']:.1f} ms (p90 {g['cpu_ms_p90']:.1f})")
    a, d = ring["astar"], ring["dqfd-murni"]
    print(f"rasio per node DQfD / A*: total/total {d['cpu_ms_per_node_total'] / a['cpu_ms_per_node_total']:.2f}x, "
          f"regresi {regresi['dqfd-murni']['ms_per_node'] / regresi['astar']['ms_per_node']:.2f}x")
    rp = [y["panjang_cm"] / x["panjang_cm"] for x, y in zip(hasil["astar"], hasil["dqfd-murni"])
          if x["panjang_cm"] and y["panjang_cm"]]
    if rp:
        ring["rasio_panjang_dqfd_astar"] = {"rute": len(rp), "median": float(np.median(rp)),
                                            "rata": float(np.mean(rp)), "p90": float(np.percentile(rp, 90)),
                                            "maks": float(np.max(rp))}
        print(f"rasio panjang jalur DQfD / A* ({len(rp)} rute tiba keduanya): median {np.median(rp):.3f}, "
              f"rata-rata {np.mean(rp):.3f}, p90 {np.percentile(rp, 90):.3f}, maks {np.max(rp):.2f}")
    _simpan(args.keluaran, {"ringkasan": ring, "regresi_cpu_ms_per_rute": regresi, "rute": hasil})


# ------------------------------------------------------------------ uji rincian
def uji_rincian(args):
    rid = args.id
    T = collections.defaultdict(float)
    N = collections.defaultdict(int)
    rekam, fase, sampel = [False], ["luar"], []

    def nama_pemanggil(lewati):
        # 0 = fungsi ini, 1 = _pembungkus, 2 = pemanggil; pembungkus lain dilewati
        fr = sys._getframe(2)
        while True:
            while fr.f_code.co_name == "_pembungkus":
                fr = fr.f_back
            if lewati == 0:
                break
            fr = fr.f_back
            lewati -= 1
        if fr.f_code.co_name == "q":                 # JaringanNumpy.q -> qv: yang dicatat pemanggil q
            fr = fr.f_back
            while fr.f_code.co_name == "_pembungkus":
                fr = fr.f_back
        return fr.f_code.co_name

    def bungkus(obj, nama, kunci, lewati=0, pemanggil=True, simpan_obs=False):
        asli = getattr(obj, nama)

        def _pembungkus(*a, **k):
            t0 = pc()
            r = asli(*a, **k)
            dt = pc() - t0
            if rekam[0]:
                kk = f"{fase[0]}|{kunci}" + (f"<-{nama_pemanggil(lewati)}" if pemanggil else "")
                T[kk] += dt
                N[kk] += 1
                if simpan_obs and N[kk] % 15 == 0 and len(sampel) < 2000:
                    sampel.append(np.array(a[0], dtype=np.float32))
            return r
        setattr(obj, nama, _pembungkus)

    def catat_terbaik(k, dt):
        if rekam[0]:
            T["terbaik|" + k] += dt
            N["terbaik|" + k] += 1

    def terbaik(net, env, anggaran=PD.ANGGARAN_TERBAIK, batas_waktu=None, bobot=PD.BOBOT_TERBAIK):
        lama, fase[0] = fase[0], "terbaik"
        t0 = pc()
        try:
            return _terbaik_bertanda(net, env, anggaran, batas_waktu, bobot, catat=catat_terbaik)
        finally:
            fase[0] = lama
            if rekam[0]:
                T["terbaik|total"] += pc() - t0
                N["terbaik|total"] += 1

    mundur_asli = PD.rencanakan

    def mundur(*a, **k):
        lama, fase[0] = fase[0], "mundur"
        t0 = pc()
        try:
            h = mundur_asli(*a, **k)
        finally:
            fase[0] = lama
        if rekam[0]:
            T["mundur|total"] += pc() - t0
            N["mundur|total"] += 1
            N["mundur|langkah"] += h["dicoba"]
        return h

    def malas(net, env, anggaran=PD.ANGGARAN_TERBAIK, batas_waktu=None, bobot=PD.BOBOT_TERBAIK, belajar=False,
              selektif=None, ambang=None):
        lama, fase[0] = fase[0], "terbaik"
        t0 = pc()
        try:
            h = _malas_bertanda(net, env, anggaran, batas_waktu, bobot, catat=catat_terbaik, belajar=belajar,
                                selektif=selektif, ambang=ambang)
        finally:
            fase[0] = lama
            if rekam[0]:
                T["terbaik|total"] += pc() - t0
                N["terbaik|total"] += 1
        if rekam[0]:
            N["terbaik|dibuang"] += h["dibuang"]
            N["terbaik|masuk_ulang"] += h["masuk_ulang"]
        return h

    def bungkus_ekspansi(e):
        """_ekspansi dicatat per jenis (dengan / tanpa cek tabrakan) dan per pemanggil."""
        asli = e._ekspansi

        def _pembungkus(nd, nk, cek=True):
            t0 = pc()
            r = asli(nd, nk, cek)
            dt = pc() - t0
            if rekam[0]:
                kk = f"{fase[0]}|eks_{'cek' if cek else 'aritmetika'}<-{nama_pemanggil(0)}"
                T[kk] += dt
                N[kk] += 1
                if cek and r is None:
                    N[f"{fase[0]}|eks_tabrak"] += 1
            return r
        e._ekspansi = _pembungkus

    pencarian = args.pencarian or PD.PENCARIAN_ROBOT
    malas_mode = pencarian in ("lazy", "belajar", "selektif", "belajar-periksa")
    PD.rencanakan_terbaik, PD.rencanakan_terbaik_malas, PD.rencanakan = terbaik, malas, mundur

    # ---- (1) lintasan waktu
    konfig.pilih_robot(rid)
    prof = konfig.profil(rid)
    daftar, net_ok = [], set()
    for peta, ar, rute in _peta_rute(rid, args.maks):
        per = _perencana(ar, prof, "dqfd-murni")
        _atur_varian(per, pencarian, False)
        for _, pl in per._cache.values():
            e, pu = pl.env, pl.pustaka
            if malas_mode:
                bungkus(e, "calon_malas", "calon_malas", pemanggil=False)
                bungkus(e, "siapkan_aksi", "siapkan_aksi", pemanggil=False)
                bungkus_ekspansi(e)
            bungkus(pl, "plan", "plan", pemanggil=False)
            bungkus(e, "reset", "reset")
            bungkus(e, "_hitung_calon", "calon")
            bungkus(e, "_obs_jauh", "obs", lewati=1)
            bungkus(e, "_sinar", "sinar", pemanggil=False)
            bungkus(e, "_arah_heuristik", "arah_hD", pemanggil=False)
            bungkus(e, "_boleh_serah", "serah")
            bungkus(pu, "_analytic", "rs", lewati=2)
            bungkus(pu, "_holonomic_heuristic", "dijkstra", pemanggil=False)
            bungkus(pu, "_feasible_approach_dirs", "arah_datang", pemanggil=False)
        if id(per.planner.net) not in net_ok:
            bungkus(per.planner.net, "qv", "forward", simpan_obs=True)
            if getattr(per.planner.net, "kepala_tabrak", False):
                bungkus(per.planner.net, "q_tabrak", "forward", simpan_obs=True)
            net_ok.add(id(per.planner.net))
        bungkus(per, "peta_dengan", "peta_dengan", pemanggil=False)
        daftar.append((per, BB.PencatatNode(per), rute))

    node = 0
    total = 0.0
    n_rute = 0
    for per, cat, rute in daftar:
        rekam[0] = False
        per.rencanakan(rute[0]["awal"], rute[0]["tujuan"], rute[0]["rintangan"])      # pemanasan
        rekam[0] = True
        for r in rute:
            cat.total = 0
            t0 = pc()
            per.rencanakan(r["awal"], r["tujuan"], r["rintangan"])
            total += pc() - t0
            node += cat.total
            n_rute += 1
        rekam[0] = False

    def g(k):
        return T.get(k, 0.0)

    def n_(k):
        return N.get(k, 0)

    def cocok(akhir):
        return (sum(v for k, v in T.items() if k.endswith(akhir)),
                sum(v for k, v in N.items() if k.endswith(akhir)))

    baris = []

    def tambah(grup, nama, detik, panggilan):
        baris.append({"grup": grup, "bagian": nama, "ms_per_node": 1000 * detik / node,
                      "persen": 100 * detik / total,
                      "panggilan_per_node": (panggilan / node) if panggilan else None,
                      "us_per_panggilan": (1e6 * detik / panggilan) if panggilan else None})

    c_step, c_pul = g("terbaik|calon<-step"), g("terbaik|calon<-pulihkan_keadaan")
    o_step = g("terbaik|obs<-step")
    s_step, rs_step = g("terbaik|serah<-step"), g("terbaik|rs<-step")
    f_buka = g("terbaik|forward<-buka")
    A = f"A pencarian terbaik-dulu ({pencarian}), per node"
    if not malas_mode:
        tambah(A, "calon 14 primitif + cek tabrakan, sesudah step", c_step, n_("terbaik|calon<-step"))
        tambah(A, "calon 14 primitif + cek tabrakan, saat pindah cabang", c_pul,
               n_("terbaik|calon<-pulihkan_keadaan"))
    else:
        cm = g("terbaik|calon_malas")
        s_ar, s_cek = g("terbaik|eks_aritmetika<-siapkan_aksi"), g("terbaik|eks_cek<-siapkan_aksi")
        tambah(A, "calon lazy: aritmetika 14 primitif + cek sel", cm, n_("terbaik|calon_malas"))
        tambah(A, "aksi diambil: simulasi 1 primitif + cek tabrakan", s_cek, n_("terbaik|eks_cek<-siapkan_aksi"))
        tambah(A, "aksi diambil: aritmetika gerak + cek sel", s_ar, n_("terbaik|eks_aritmetika<-siapkan_aksi"))
        tambah(A, "aksi diambil: sisa", g("terbaik|siapkan_aksi") - s_ar - s_cek, n_("terbaik|siapkan_aksi"))
        tambah(A, "f tepat + cek masuk ulang", g("terbaik|f_tepat"), n_("terbaik|f_tepat"))
    tambah(A, "observasi 47 angka", o_step, n_("terbaik|obs<-step"))
    tambah(A, "forward pass jaringan (numpy)", f_buka, n_("terbaik|forward<-buka"))
    tambah(A, "tembakan RS (di dalam 60 cm)", rs_step, n_("terbaik|rs<-step"))
    tambah(A, "saringan serah (60 cm + peta Dijkstra)", s_step - rs_step, n_("terbaik|serah<-step"))
    tambah(A, "step: pose baru, sel dikunjungi, biaya", g("terbaik|step") - c_step - o_step - s_step,
           n_("terbaik|step"))
    buka_lain = g("terbaik|buka") - f_buka - (g("terbaik|calon_malas") if malas_mode else 0.0)
    tambah(A, "buka: f = g + c + w.h, masuk heap", buka_lain, n_("terbaik|buka"))
    tambah(A, "pulihkan: susun jalur + keadaan", g("terbaik|susun_jalur") + g("terbaik|pulihkan_keadaan") - c_pul,
           n_("terbaik|pulihkan_keadaan"))
    tambah(A, "ambil simpul dari heap", g("terbaik|pop"), n_("terbaik|pop"))
    if malas_mode:
        tambah(A, "simpan keadaan + jarak", g("terbaik|simpan"), None)
        bag = sum(g("terbaik|" + k) for k in ("pop", "susun_jalur", "pulihkan_keadaan", "siapkan", "f_tepat",
                                               "step", "simpan", "buka"))
        tambah(A, "sisa loop (cek anggaran / batas waktu, aksi dibuang)",
               g("terbaik|total") - bag + (g("terbaik|siapkan") - g("terbaik|siapkan_aksi")), None)
    else:
        tambah(A, "cek mask + simpan keadaan + jarak", g("terbaik|cek_mask") + g("terbaik|simpan"), None)
        bag = sum(g("terbaik|" + k) for k in ("pop", "susun_jalur", "pulihkan_keadaan", "cek_mask", "step",
                                               "simpan", "buka"))
        tambah(A, "sisa loop (cek anggaran / batas waktu)", g("terbaik|total") - bag, None)
    tambah("B backtracking (setelah terbaik-dulu gagal)", "seluruhnya", g("mundur|total"), n_("mundur|langkah"))
    reset_tot = sum(v for k, v in T.items() if "|reset<-" in k)
    n_reset = sum(v for k, v in N.items() if "|reset<-" in k)
    dij, n_dij = cocok("|dijkstra")
    dat, n_dat = cocok("|arah_datang")
    fv, n_fv = cocok("forward<-rencanakan_dqfd")
    plan_tot, n_plan = cocok("|plan")
    peta_t, n_peta = cocok("|peta_dengan")
    C = "C biaya tetap per rencana"
    tambah(C, "peta Dijkstra h_D", dij, n_dij)
    tambah(C, "arah datang yang layak", dat, n_dat)
    tambah(C, "sisa reset (cek awal, calon & observasi awal)", reset_tot - dij - dat, n_reset)
    tambah(C, "forward V(s0) (perkiraan biaya)", fv, n_fv)
    tambah(C, "sisa plan (jalur hasil, gerbang)", plan_tot - reset_tot - fv - g("terbaik|total") - g("mundur|total"),
           n_plan)
    tambah(C, "peta rintangan sementara", peta_t, n_peta)
    tambah(C, "sisa Perencana (sapuan arah, konversi)", total - plan_tot - peta_t, n_rute)

    print(f"(1) {n_rute} rute, {node} node, total {1000 * total:.0f} ms -> {1000 * total / node:.4f} ms per node | "
          f"langkah terbaik-dulu {n_('terbaik|step')}, langkah backtracking {n_('mundur|langkah')}, "
          f"panggilan plan {n_plan}")
    for b in baris:
        ppn = (f"{b['panggilan_per_node']:6.3f} x/node {b['us_per_panggilan']:8.1f} us/x"
               if b["panggilan_per_node"] else "")
        print(f"  {b['grup'][:1]} {b['bagian']:<52} {b['ms_per_node']:.4f} ms {b['persen']:5.1f}%  {ppn}")
    print(f"  jumlah bagian = {sum(b['ms_per_node'] for b in baris):.4f} ms per node")
    ob = sum(v for k, v in T.items() if "|obs<-" in k)
    n_ob = sum(v for k, v in N.items() if "|obs<-" in k)
    si, n_si = cocok("|sinar")
    ah, n_ah = cocok("|arah_hD")
    if n_si and n_ah:
        obs_us = {"total": 1e6 * ob / n_ob, "sinar_32": 1e6 * si / n_si, "arah_turun_hD": 1e6 * ah / n_ah}
        obs_us["sisa"] = obs_us["total"] - (1e6 * si + 1e6 * ah) / n_ob
        print(f"  observasi per panggilan {obs_us['total']:.1f} us = 32 sinar {obs_us['sinar_32']:.1f} us + arah turun "
              f"h_D {obs_us['arah_turun_hD']:.1f} us + sisa {obs_us['sisa']:.1f} us")
    else:
        # Kernel numba (cepat.obs_isi): sinar dan arah turun h_D di dalam satu panggilan, tidak terpisah.
        obs_us = {"total": 1e6 * ob / n_ob if n_ob else None, "sinar_32": None, "arah_turun_hD": None, "sisa": None,
                  "catatan": "satu kernel numba (cepat.obs_isi)"}
        print(f"  observasi per panggilan {obs_us['total']:.1f} us (satu kernel numba, sinar termasuk)")

    # ---- (2) isi _hitung_calon: hitung saja, tanpa waktu (mode eager)
    if malas_mode:
        # Di mode lazy _hitung_calon hanya dipanggil di reset dan backtracking; yang dilaporkan
        # jumlah panggilan per node dari lintasan waktu.
        calon = {k: n_(f"terbaik|{k}") / node for k in (
            "calon_malas", "eks_aritmetika<-calon_malas", "siapkan_aksi", "eks_aritmetika<-siapkan_aksi",
            "eks_cek<-siapkan_aksi", "eks_tabrak", "dibuang", "masuk_ulang")}
        print("(2) per node: " + ", ".join(f"{k} {v:.3f}" for k, v in calon.items()))
    else:
        H = collections.Counter()
        hitung = [False]
        pose_sampel = []
        pm_asli = FP.ObstacleMap.pose_margin

        def pm_hitung(self, vehicle, x, y, yaw):
            if hitung[0] and sys._getframe(1).f_code.co_name == "_ekspansi":
                H["pose_margin"] += 1
                if H["pose_margin"] % 11 == 0 and len(pose_sampel) < 20000:
                    pose_sampel.append((self, vehicle, x, y, yaw))
            return pm_asli(self, vehicle, x, y, yaw)

        FP.ObstacleMap.pose_margin = pm_hitung
        n_lingkaran = set()
        try:
            for per, cat, rute in daftar:
                for _, pl in per._cache.values():
                    e = pl.env
                    n_lingkaran.add(len(e.v.circle_offsets))
                    for nama in ("_hitung_calon", "_ekspansi"):
                        asli = getattr(e, nama)

                        def _hit(*a, _asli=asli, _nama=nama, **k):
                            r = _asli(*a, **k)
                            if hitung[0]:
                                H[_nama] += 1
                                if _nama == "_ekspansi" and r is None:
                                    H["ekspansi_tabrak"] += 1
                            return r
                        setattr(e, nama, _hit)
                hitung[0] = True
                for r in rute:
                    per.rencanakan(r["awal"], r["tujuan"], r["rintangan"])
                hitung[0] = False
        finally:
            FP.ObstacleMap.pose_margin = pm_asli
        for (om, v, x, y, yaw) in pose_sampel[:2000]:
            om.pose_margin(v, x, y, yaw)
        t0 = pc()
        for _ in range(5):
            for (om, v, x, y, yaw) in pose_sampel:
                om.pose_margin(v, x, y, yaw)
        us_pm = (pc() - t0) / (5 * len(pose_sampel)) * 1e6
        n_calon = n_("terbaik|calon<-step") + n_("terbaik|calon<-pulihkan_keadaan")
        us_calon = 1e6 * (c_step + c_pul) / n_calon
        pm_per = H["pose_margin"] / H["_hitung_calon"]
        calon = {"primitif_per_panggilan": H["_ekspansi"] / H["_hitung_calon"],
                 "primitif_tabrak_per_panggilan": H["ekspansi_tabrak"] / H["_hitung_calon"],
                 "pose_margin_per_panggilan": pm_per, "us_per_pose_margin": us_pm, "us_per_panggilan": us_calon,
                 "lingkaran_badan": sorted(n_lingkaran)}
        print(f"(2) per panggilan _hitung_calon: {calon['primitif_per_panggilan']:.2f} primitif disimulasi "
              f"({calon['primitif_tabrak_per_panggilan']:.2f} berhenti karena tabrakan), {pm_per:.1f} cek tabrakan "
              f"pose_margin x {us_pm:.2f} us = {pm_per * us_pm:.1f} us dari {us_calon:.1f} us; "
              f"lingkaran badan {sorted(n_lingkaran)}")

    # ---- (3) forward pass satu keadaan dan jumlah parameter
    net = PD.muat_model_numpy(rid)
    for nama in ("qv", "q_tabrak"):              # lepas pembungkus waktu, kembali ke metode kelas
        net.__dict__.pop(nama, None)
    X = np.array(sampel, dtype=np.float32)
    for o in X[:300]:
        net.q(o)
    ts = np.empty(10 * len(X))
    i = 0
    for _ in range(10):
        for o in X:
            t0 = time.perf_counter_ns()
            net.q(o)
            ts[i] = time.perf_counter_ns() - t0
            i += 1
    ts /= 1000.0
    t0 = pc()
    for _ in range(10):
        for o in X:
            net.q(o)
    rata = (pc() - t0) / (10 * len(X)) * 1e6
    seg = np.empty((10 * len(X), 6))
    i = 0
    pcn = time.perf_counter_ns
    for _ in range(10):
        for x0 in X:
            t_0 = pcn()
            x = np.asarray(x0, dtype=np.float32)
            t_1 = pcn()
            h = x @ net.w1
            h += net.b1
            np.maximum(h, 0.0, out=h)
            t_2 = pcn()
            h2 = h @ net.w2
            h2 += net.b2
            np.maximum(h2, 0.0, out=h2)
            t_3 = pcn()
            hva = h2 @ net.w_va
            hva += net.b_va
            np.maximum(hva, 0.0, out=hva)
            t_4 = pcn()
            o = hva @ net.w_out
            o += net.b_out
            t_5 = pcn()
            a = o[1:]
            _ = (a - a.mean()) + o[0]
            _ = float(o[0])
            t_6 = pcn()
            seg[i] = (t_1 - t_0, t_2 - t_1, t_3 - t_2, t_4 - t_3, t_5 - t_4, t_6 - t_5)
            i += 1
    seg_med = np.median(seg, axis=0) / 1000.0
    nama_seg = ("float32", "FC1 47x256+ReLU", "FC2 256x256+ReLU", "V|A-hidden 256x256+ReLU", "keluaran 256x15",
                "Q = V + A - rata(A)")
    print(f"(3) forward numpy satu keadaan ({len(X)} observasi nyata x 10): median {np.median(ts):.2f} us, "
          f"p10 {np.percentile(ts, 10):.2f}, p90 {np.percentile(ts, 90):.2f}, rata-rata loop {rata:.2f} us")
    print("    per lapis (median us): " + ", ".join(f"{n} {s:.2f}" for n, s in zip(nama_seg, seg_med)))
    # q_tabrak model jauh + model P (satu panggilan per simpul pencarian robot): gabungan vs terpisah
    q_tabrak_us = None
    mp = PD.muat_model_p_robot(rid)
    if mp is not None:
        q_tabrak_us = {}
        for nama_f, gab in (("gabungan", True), ("terpisah", False)):
            nd = JN.NetDenganP(net, mp, gabung=gab)
            for o in X[:300]:
                nd.q_tabrak(o)
            ulang_us = []
            for _ in range(5):
                t0 = pc()
                for o in X:
                    nd.q_tabrak(o)
                ulang_us.append((pc() - t0) / len(X) * 1e6)
            q_tabrak_us[nama_f] = float(np.median(ulang_us))
        print(f"    q_tabrak model jauh + model P: gabungan {q_tabrak_us['gabungan']:.2f} us, "
              f"terpisah {q_tabrak_us['terpisah']:.2f} us (median 5 x {len(X)} panggilan)")

    with np.load(jalur_cache(PD.jalur_model(rid))) as f:
        bobot = {k: f[k] for k in f.files if k.endswith((".weight", ".bias"))}
    hasil_torch = None
    try:
        import torch
        from drl.jaringan import JaringanDQfD
    except ImportError:
        print("    PyTorch tidak terpasang: pembanding PyTorch dilewati")
    else:
        # Model dengan kepala kelayakan (t2) sejak lapis c: strukturnya ikut bobot.
        m = JaringanDQfD(L.N_AKSI, L.N_OBS, kepala_tabrak="t2.weight" in bobot)
        m.load_state_dict({k: torch.from_numpy(np.ascontiguousarray(v)) for k, v in bobot.items()})
        m.eval()
        tt = []
        with torch.no_grad():
            for o in X[:300]:
                m(torch.from_numpy(o)[None])
            for _ in range(3):
                for o in X:
                    t0 = pcn()
                    m(torch.from_numpy(o)[None])
                    tt.append(pcn() - t0)
            selisih = float(np.abs(m(torch.from_numpy(X)).numpy() - net.q_batch(X)).max())
        tt = np.array(tt) / 1000.0
        hasil_torch = {"versi": torch.__version__, "thread": torch.get_num_threads(),
                       "median": float(np.median(tt)), "p90": float(np.percentile(tt, 90)),
                       "selisih_q_maks": selisih}
        print(f"    PyTorch {torch.__version__} ({torch.get_num_threads()} thread) satu keadaan: median "
              f"{np.median(tt):.2f} us, p90 {np.percentile(tt, 90):.2f} us | selisih Q maks numpy vs torch "
              f"{selisih:.1e}")
    lap = []
    for nm in ("fc1", "fc2", "v1", "v2", "a1", "a2"):
        w, b = bobot[nm + ".weight"], bobot[nm + ".bias"]
        lap.append((nm, f"{w.shape[1]} -> {w.shape[0]}", int(w.size), int(b.size)))
    n_w, n_b = sum(x[2] for x in lap), sum(x[3] for x in lap)
    mac_np = net.w1.size + net.w2.size + net.w_va.size + net.w_out.size
    print("    parameter: " + ", ".join(f"{n} {s}: {w}+{b}" for n, s, w, b in lap) +
          f" | total {n_w + n_b} (bobot {n_w}, bias {n_b}); float32 {4 * (n_w + n_b) / 1024:.0f} KiB; "
          f"MAC model {n_w}, MAC numpy (blok digabung) {mac_np}")

    _simpan(args.keluaran, {("detail_dqfd_lazy" if malas_mode else "detail_dqfd"): {
        "catatan": f"pencarian {pencarian}, jam dinding, {n_rute} rute uji ID {rid:02d}, pemanasan tidak dihitung; pencatat menambah beberapa persen",
        "pencarian": pencarian,
        "node": node, "total_ms": 1000 * total, "ms_per_node": 1000 * total / node, "bagian": baris,
        "observasi_us": obs_us, "calon": calon,
        "forward_numpy_us": {"median": float(np.median(ts)), "p10": float(np.percentile(ts, 10)),
                             "p90": float(np.percentile(ts, 90)), "rata_loop": rata,
                             "per_lapis_median": dict(zip(nama_seg, map(float, seg_med)))},
        "forward_torch_us": hasil_torch,
        "q_tabrak_us": q_tabrak_us,
        "parameter": {"lapis": [dict(nama=n, ukuran=s, bobot=w, bias=b) for n, s, w, b in lap],
                      "total": n_w + n_b, "bobot": n_w, "bias": n_b, "mac_model": n_w, "mac_numpy": int(mac_np)}}})


# ------------------------------------------------------------------ uji tabrak
def uji_tabrak(args):
    rid = args.id
    catatan = []

    def status_aksi(env):
        """0 sah, 1 menabrak, 2 sel sudah dikunjungi, 3 mundur dilarang."""
        pl = env.planner
        st = np.zeros(L.N_AKSI, np.int8)
        for a in range(L.N_AKSI):
            if env._mask[a]:
                continue
            nd, nk, _ = L.aksi_info(pl, a)
            if nd < 0 and not env.boleh_mundur:
                st[a] = 3
            elif env._ekspansi(nd, nk) is None:
                st[a] = 1
            else:
                st[a] = 2
        return st

    def saat_buka(env, q):
        net = env._net_uji
        p = net.q_tabrak(env.observasi())[1] if getattr(net, "kepala_tabrak", False) else None
        catatan.append((np.array(q, np.float32), status_aksi(env),
                        env.omap.pose_margin(env.v, env.x, env.y, env.yaw), p))

    def terbaik(net, env, anggaran=PD.ANGGARAN_TERBAIK, batas_waktu=None, bobot=PD.BOBOT_TERBAIK):
        env._net_uji = net
        return _terbaik_bertanda(net, env, anggaran, batas_waktu, bobot, saat_buka=saat_buka)

    PD.rencanakan_terbaik = terbaik
    konfig.pilih_robot(rid)
    prof = konfig.profil(rid)
    n_rute = 0
    for peta, ar, rute in _peta_rute(rid, args.maks):
        per = _perencana(ar, prof, "dqfd-murni")
        _pasang_kepala_p(per, args)
        _atur_varian(per, "eager", False)          # uji ini membedah mask pencarian eager
        for r in rute:
            per.rencanakan(r["awal"], r["tujuan"], r["rintangan"])
            n_rute += 1

    Q = np.stack([c[0] for c in catatan])
    S = np.stack([c[1] for c in catatan])
    M = np.array([c[2] for c in catatan])
    n = len(Q)
    idx = np.arange(n)
    tabrak, sah = S == 1, S == 0
    jml = tabrak.sum(1)
    ada = jml > 0
    top_t = tabrak[idx, Q.argmax(1)]
    acak = jml / L.N_AKSI
    num = den = 0.0
    for q, t, s in zip(Q[ada], tabrak[ada], sah[ada]):
        if s.any():
            qv, qc = q[s], q[t]
            num += (qv[:, None] > qc[None, :]).sum() + 0.5 * (qv[:, None] == qc[None, :]).sum()
            den += len(qv) * len(qc)
    auc = num / den if den else float("nan")
    di_atas = np.array([(q[t] > q[s].max()).sum() for q, t, s in zip(Q[ada], tabrak[ada], sah[ada]) if s.any()])
    rk = np.argsort(np.argsort(-Q, axis=1), axis=1) + 1
    per_margin = []
    for lo, hi, label in ((-np.inf, 2, "<=2"), (2, 6, "2-6"), (6, 12, "6-12"), (12, np.inf, ">12")):
        b = (M > lo) & (M <= hi)
        if b.any():
            per_margin.append({"margin_sel": label, "porsi": 100 * float(b.mean()),
                               "ada_menabrak": 100 * float(ada[b].mean()),
                               "q_tertinggi_menabrak": 100 * float(top_t[b].mean())})
    h = {"catatan": f"Q 14 aksi tanpa mask di tiap ekspansi terbaik-dulu DQfD ID {rid:02d}, {n_rute} rute uji",
         "ekspansi": n, "aksi_menabrak_per_ekspansi_rata": float(jml.mean()),
         "persen_ekspansi_ada_aksi_menabrak": 100 * float(ada.mean()),
         "sebaran_jumlah_aksi_menabrak": {int(k): 100 * float((jml == k).mean()) for k in np.unique(jml)},
         "status_aksi_persen": {"sah": 100 * float(sah.mean()), "menabrak": 100 * float(tabrak.mean()),
                                "sel_dikunjungi": 100 * float((S == 2).mean()),
                                "mundur_dilarang": 100 * float((S == 3).mean())},
         "q_tertinggi_menabrak_persen": {"semua_ekspansi": 100 * float(top_t.mean()),
                                         "ekspansi_ada_tabrakan": 100 * float(top_t[ada].mean()),
                                         "pembanding_acak": 100 * float(acak[ada].mean())},
         "auc_q_sah_vs_menabrak": float(auc),
         "menabrak_di_atas_sah_terbaik": {"rata": float(di_atas.mean()), "persen_nol": 100 * float((di_atas == 0).mean())},
         "peringkat_median": {"menabrak": float(np.median(rk[tabrak])), "sah": float(np.median(rk[sah])),
                              "menabrak_di_3_besar_persen": 100 * float((rk[tabrak] <= 3).mean())},
         "per_margin_pose_sel": per_margin}
    print(f"{n_rute} rute, {n} ekspansi | ekspansi dengan >= 1 aksi menabrak {h['persen_ekspansi_ada_aksi_menabrak']:.1f}% "
          f"| status aksi: " + ", ".join(f"{k} {v:.1f}%" for k, v in h["status_aksi_persen"].items()))
    qt = h["q_tertinggi_menabrak_persen"]
    print(f"Q tertinggi tanpa mask = aksi menabrak: {qt['semua_ekspansi']:.2f}% semua ekspansi, "
          f"{qt['ekspansi_ada_tabrakan']:.1f}% ekspansi yang punya aksi menabrak (acak {qt['pembanding_acak']:.1f}%) | "
          f"AUC {auc:.3f}")
    for b in per_margin:
        print(f"  margin {b['margin_sel']:>5} sel: {b['porsi']:5.1f}% ekspansi, ada aksi menabrak "
              f"{b['ada_menabrak']:5.1f}%, Q tertinggi menabrak {b['q_tertinggi_menabrak']:5.1f}%")
    if catatan and catatan[0][3] is not None:
        P = np.stack([c[3] for c in catatan])
        num = den = 0.0
        for pp, t, sh in zip(P[ada], tabrak[ada], sah[ada]):
            if sh.any():
                pv, pc_ = pp[sh], pp[t]
                num += (pc_[:, None] > pv[None, :]).sum() + 0.5 * (pc_[:, None] == pv[None, :]).sum()
                den += len(pv) * len(pc_)
        kep = {"auc_ekspansi_sama": float(num / den) if den else float("nan"),
               "auc_global": _auc(P[tabrak | sah], tabrak[tabrak | sah]), "per_ambang": {}}
        for thr in (0.5, 0.3, 0.2, 0.1, 0.05):
            aman = P < thr
            top = np.where(aman, Q, -np.inf).argmax(1)
            ada_aman = aman.any(1)
            kep["per_ambang"][str(thr)] = {
                "terdeteksi_persen": 100 * float((P[tabrak] >= thr).mean()),
                "alarm_palsu_persen": 100 * float((P[sah] >= thr).mean()),
                "q_tertinggi_aman_menabrak_persen": 100 * float(tabrak[idx, top][ada_aman].mean())}
        h["kepala_tabrak"] = kep
        print(f"kepala P(menabrak): AUC (ekspansi sama) {kep['auc_ekspansi_sama']:.3f}, global {kep['auc_global']:.3f}")
        for thr, v in kep["per_ambang"].items():
            print(f"  ambang {thr:>4}: terdeteksi {v['terdeteksi_persen']:5.1f}%, alarm palsu {v['alarm_palsu_persen']:5.1f}%, "
                  f"Q tertinggi di antara 'aman' ternyata menabrak {v['q_tertinggi_aman_menabrak_persen']:.2f}% ekspansi")
    _simpan(args.keluaran, {(("uji_q_tabrak_kepala_p" if args.kepala_p else "uji_q_tabrak_kepala")
                             if "kepala_tabrak" in h else "uji_q_tabrak"): h})


def _auc(skor, label):
    skor, label = np.asarray(skor, float).ravel(), np.asarray(label, bool).ravel()
    n_pos, n_neg = int(label.sum()), int((~label).sum())
    if not n_pos or not n_neg:
        return float("nan")
    urut = np.argsort(skor, kind="mergesort")
    pk = np.empty(len(skor))
    s_urut = skor[urut]
    awal = np.r_[0, np.flatnonzero(np.diff(s_urut)) + 1]
    akhir = np.r_[awal[1:], len(skor)]
    for a, b in zip(awal, akhir):
        pk[urut[a:b]] = (a + b + 1) / 2.0
    return float((pk[label].sum() - n_pos * (n_pos + 1) / 2.0) / (n_pos * n_neg))


# ------------------------------------------------------------------ uji varian
def _atur_varian(per, pencarian, pintas):
    for _, pl in per._cache.values():
        pl.pencarian = pencarian
        pl.env.pintas_terbuka = pintas


_MODEL_P = {}


def _pasang_kepala_p(per, args):
    """--kepala-p: P dari model terpisah ini (Q tetap dari model jauh). --tanpa-kepala-p: lepas model P
    yang dipasang DQfDPlanner otomatis (id0N_kepala_p.pt), jadi P dari kepala bawaan."""
    from drl.jaringan_numpy import NetDenganP, muat_model_p
    if getattr(args, "tanpa_kepala_p", False):
        for _, pl in per._cache.values():
            if isinstance(pl.net, NetDenganP):
                pl.net = pl.net.net
        return
    if not getattr(args, "kepala_p", None):
        return
    if args.kepala_p not in _MODEL_P:
        _MODEL_P[args.kepala_p] = muat_model_p(args.kepala_p)[0]
    for _, pl in per._cache.values():
        if not isinstance(pl.net, NetDenganP):
            pl.net = NetDenganP(pl.net, _MODEL_P[args.kepala_p])


def _uraikan_varian(nama):
    """"lazy+pintas" -> (nama, "lazy", True, None); "belajar@0.3" -> (nama, "belajar", False, 0.3).
    "@x" = AMBANG_TABRAK, kecuali "selektif@x" = AMBANG_YAKIN_AMAN."""
    dasar, _, ambang = nama.partition("@")
    pencarian, _, pintas = dasar.partition("+")
    if pencarian not in PD.PENCARIAN or pintas not in ("", "pintas"):
        raise SystemExit(f"[!] varian '{nama}' tidak dikenal (contoh: eager, lazy+pintas, belajar@0.3)")
    return nama, pencarian, pintas == "pintas", float(ambang) if ambang else None


def _periksa_ulang(pl):
    """Putar ulang aksi_jalur rencana terakhir DQfDPlanner pl dengan cek tabrakan lengkap (eager).
    None bila tidak ada aksi_jalur (jalur eager / backtracking - sudah dicek saat mencari)."""
    st = pl.last_stats or {}
    aksi = st.get("aksi_jalur")
    if not aksi or getattr(pl, "_rencana_terakhir", None) is None:
        return None
    omap, awal, tujuan = pl._rencana_terakhir
    e0 = pl.env
    env = L.LingkunganRencana(e0.arena, e0.p, margin_cm=e0.margin_cm, planner=pl.pustaka)
    obs, _ = env.reset(awal, tujuan, omap)
    if obs is None:
        return {"tabrak": False, "biaya": env.biaya if env.sukses else None}
    for a in aksi:
        if env.selesai or not env._mask[a]:
            return {"tabrak": bool(env._tabrak[a]) or env.selesai, "biaya": None}
        env.step(a)
    return {"tabrak": not env.sukses, "biaya": env.biaya if env.sukses else None}


def uji_varian(args):
    rid = args.id
    konfig.pilih_robot(rid)
    prof = konfig.profil(rid)
    kepala = bool(getattr(PD.muat_model_numpy(rid), "kepala_tabrak", False)) or bool(args.kepala_p)
    if args.varian:
        daftar_v = [_uraikan_varian(v.strip()) for v in args.varian.split(",") if v.strip()]
    elif kepala:
        daftar_v = [_uraikan_varian(v) for v in ("eager", "lazy", "belajar")]
    else:
        daftar_v = [(n, p, k, None) for n, p, k in VARIAN]
    nama_v = [v[0] for v in daftar_v]
    print(f"[i] model ID {rid:02d} {'dengan' if kepala else 'tanpa'} kepala kelayakan; varian: {', '.join(nama_v)}")
    waktu = {v: [] for v in nama_v}
    hitung = {v: [] for v in nama_v}
    H = {v: collections.Counter() for v in nama_v}
    aktif, diam = [None], [False]
    ambang_asli, yakin_asli, belajar_asli = PD.AMBANG_TABRAK, PD.AMBANG_YAKIN_AMAN, PD.AMBANG_BELAJAR

    malas_asli = PD.rencanakan_terbaik_malas

    def malas(*a, **k):
        h = malas_asli(*a, **k)
        if aktif[0]:
            H[aktif[0]]["dibuang"] += h["dibuang"]
            H[aktif[0]]["masuk_ulang"] += h["masuk_ulang"]
            H[aktif[0]]["tanpa_cek"] += h.get("tanpa_cek", 0)
        return h
    PD.rencanakan_terbaik_malas = malas

    pm_asli = FP.ObstacleMap.pose_margin

    def pm_hitung(self, vehicle, x, y, yaw):
        if aktif[0] and not diam[0] and sys._getframe(1).f_code.co_name in ("_ekspansi", "_terbuka"):
            H[aktif[0]]["pose_margin"] += 1
        return pm_asli(self, vehicle, x, y, yaw)

    def pasang_hitung(per, nama):
        """Pembungkus hitung (lintasan hitung saja, sesudah lintasan waktu peta itu)."""
        for _, pl in per._cache.values():
            e = pl.env
            eks_asli, ter_asli = e._ekspansi, e._terbuka

            def eks(nd, nk, cek=True, _asli=eks_asli):
                if not diam[0]:
                    H[nama]["simulasi_cek" if cek else "aritmetika"] += 1
                return _asli(nd, nk, cek)

            def ter(_asli=ter_asli, _e=e, _eks=eks_asli):
                r = _asli()
                H[nama]["terbuka" if r else "tidak_terbuka"] += 1
                if r:                   # jalan pintas harus sama persis dengan simulasi lengkap
                    diam[0] = True
                    for a in range(L.N_AKSI):
                        nd, nk, _ = L.aksi_info(_e.planner, a)
                        if nd < 0 and not _e.boleh_mundur:
                            continue
                        r1, r2 = _eks(nd, nk, True), _eks(nd, nk, False)
                        H[nama]["pintas_dicek"] += 1
                        if r1 is None or r1 != r2:
                            H[nama]["pintas_selisih"] += 1
                    diam[0] = False
                return r
            e._ekspansi, e._terbuka = eks, ter

    t_mulai = time.time()
    for peta, ar, rute in _peta_rute(rid, args.maks):
        for nama, pencarian, pintas, ambang in daftar_v:
            if pencarian == "selektif":
                PD.AMBANG_YAKIN_AMAN = yakin_asli if ambang is None else ambang
            elif pencarian == "belajar-periksa":
                PD.AMBANG_BELAJAR = belajar_asli if ambang is None else ambang
            else:
                PD.AMBANG_TABRAK = ambang_asli if ambang is None else ambang
            per = _perencana(ar, prof, "dqfd-murni")
            _pasang_kepala_p(per, args)
            _atur_varian(per, pencarian, pintas)
            cat = BB.PencatatNode(per)
            r0 = rute[0]
            per.rencanakan(r0["awal"], r0["tujuan"], r0["rintangan"])            # pemanasan
            for r in rute:                                                     # lintasan waktu
                cpu = []
                for _ in range(args.ulang):
                    cat.total = 0
                    jalur, info = per.rencanakan(r["awal"], r["tujuan"], r["rintangan"])
                    cpu.append(info["cpu_s"])
                waktu[nama].append({"node": cat.total, "cpu_ms": 1000 * float(np.median(cpu))})
            pasang_hitung(per, nama)                                           # lintasan hitung
            FP.ObstacleMap.pose_margin = pm_hitung
            aktif[0] = nama
            try:
                for r in rute:
                    cat.total = 0
                    jalur, info = per.rencanakan(r["awal"], r["tujuan"], r["rintangan"])
                    diam[0] = True
                    cek = _periksa_ulang(per.planner) if jalur else None
                    diam[0] = False
                    biaya = per.planner.env.biaya if jalur else None
                    pr = (per.planner.last_stats or {}).get("perencana", "?")
                    kat = ("jalur belajar lolos periksa" if "lolos periksa" in pr else
                           "cadangan lazy: jalur belajar menabrak" if "menabrak" in pr else
                           "cadangan lazy: belajar gagal" if "belajar gagal" in pr else
                           "backtracking" if "mundur" in pr else pr.replace("dqfd (", "").rstrip(")"))
                    H[nama]["metode:" + kat] += 1
                    hitung[nama].append({"node": cat.total, "jalur": jalur, "sukses": bool(jalur),
                                         "biaya": biaya, "diperiksa": cek is not None,
                                         "tabrak": bool(cek and cek["tabrak"]),
                                         "biaya_tepat": cek["biaya"] if cek else biaya})
            finally:
                aktif[0] = None
                FP.ObstacleMap.pose_margin = pm_asli
                PD.AMBANG_TABRAK, PD.AMBANG_YAKIN_AMAN, PD.AMBANG_BELAJAR = ambang_asli, yakin_asli, belajar_asli
        print(f"  {peta:<18} {len(rute):3d} rute ({time.time() - t_mulai:.0f} s)", flush=True)

    ring = {}
    dasar = hitung["eager"] if "eager" in hitung else hitung[nama_v[0]]
    print(f"\n=== DQfD ID {rid:02d}: varian pencarian ({len(dasar)} rute)")
    for nama, pencarian, pintas, ambang in daftar_v:
        xs, ys, h = waktu[nama], hitung[nama], H[nama]
        n_node = sum(x["node"] for x in xs)
        node = np.array([x["node"] for x in xs], float)
        c_all = np.array([x["cpu_ms"] for x in xs])
        rasio, identik = [], 0
        for y, y0 in zip(ys, dasar):
            identik += y["jalur"] == y0["jalur"]
            if y["biaya_tepat"] is not None and y0["biaya_tepat"] is not None and not y["tabrak"]:
                rasio.append(y["biaya_tepat"] / y0["biaya_tepat"])
        rasio = np.array(rasio)
        n_ter = h["terbuka"] + h["tidak_terbuka"]
        n_cek = sum(y["diperiksa"] for y in ys)
        n_tabrak = sum(y["tabrak"] for y in ys)
        g = {"pencarian": pencarian, "pintas": pintas,
             "ambang": ({"selektif": yakin_asli, "belajar-periksa": belajar_asli}.get(pencarian, ambang_asli)
                        if ambang is None else ambang),
             "kepala_tabrak": kepala, "kepala_p": args.kepala_p, "rute": len(xs),
             "tiba": 100.0 * float(np.mean([y["sukses"] for y in ys])),
             "tiba_tanpa_tabrakan": 100.0 * float(np.mean([y["sukses"] and not y["tabrak"] for y in ys])),
             "jalur_diperiksa_ulang": n_cek, "jalur_menabrak": n_tabrak,
             "node_total": n_node, "node_median": float(np.median(node)), "node_p90": float(np.percentile(node, 90)),
             "node_sama_lintasan_hitung": n_node == sum(y["node"] for y in ys),
             "cpu_ms_per_node_total": float(c_all.sum() / max(1, n_node)),
             "cpu_ms_median": float(np.median(c_all)), "cpu_ms_p90": float(np.percentile(c_all, 90)),
             "per_node": {k: h[k] / max(1, n_node) for k in ("simulasi_cek", "aritmetika", "pose_margin",
                                                               "dibuang", "masuk_ulang", "tanpa_cek")},
             "terbuka_persen": 100.0 * h["terbuka"] / n_ter if n_ter else 0.0,
             "pintas_dicek": h["pintas_dicek"], "pintas_selisih": h["pintas_selisih"],
             "metode": {k[7:]: v for k, v in h.items() if k.startswith("metode:")},
             "vs_dasar": {"dasar": "eager" if "eager" in hitung else nama_v[0],
                          "jalur_identik_persen": 100.0 * identik / len(ys),
                          "rasio_biaya_median": float(np.median(rasio)) if len(rasio) else None,
                          "rasio_biaya_p90": float(np.percentile(rasio, 90)) if len(rasio) else None,
                          "rasio_biaya_maks": float(rasio.max()) if len(rasio) else None,
                          "rasio_biaya_min": float(rasio.min()) if len(rasio) else None,
                          "lebih_mahal_1persen": 100.0 * float(np.mean(rasio > 1.01)) if len(rasio) else None,
                          "lebih_murah_1persen": 100.0 * float(np.mean(rasio < 0.99)) if len(rasio) else None}}
        ring[nama] = g
        pn, ve = g["per_node"], g["vs_dasar"]
        print(f"{nama:<13}: tiba {g['tiba']:.1f}% (tanpa tabrakan {g['tiba_tanpa_tabrakan']:.1f}%, jalur menabrak "
              f"{n_tabrak} dari {n_cek} diperiksa ulang) | {g['cpu_ms_per_node_total']:.3f} ms/node, node {n_node} "
              f"(median {g['node_median']:.0f}, p90 {g['node_p90']:.0f}), per rute median {g['cpu_ms_median']:.1f} ms "
              f"p90 {g['cpu_ms_p90']:.1f} | per node: simulasi+cek {pn['simulasi_cek']:.2f}, aritmetika "
              f"{pn['aritmetika']:.1f}, cek pose {pn['pose_margin']:.1f}, dibuang {pn['dibuang']:.2f}, "
              f"tanpa cek {pn['tanpa_cek']:.2f}, "
              f"masuk ulang {pn['masuk_ulang']:.3f} | terbuka {g['terbuka_persen']:.1f}% (pintas dicek "
              f"{g['pintas_dicek']}, selisih {g['pintas_selisih']}) | vs {ve['dasar']}: jalur identik "
              f"{ve['jalur_identik_persen']:.1f}%, rasio biaya median {ve['rasio_biaya_median'] or 0:.4f} "
              f"p90 {ve['rasio_biaya_p90'] or 0:.4f} min {ve['rasio_biaya_min'] or 0:.3f} "
              f"maks {ve['rasio_biaya_maks'] or 0:.3f}")
    _simpan(args.keluaran, {("uji_varian_kepala_p" if args.kepala_p else "uji_varian_kepala") if kepala
                            else "uji_varian": ring})


# ------------------------------------------------------------------ utama
def main():
    ap = argparse.ArgumentParser(description="Waktu proses per node: Hybrid A* + RS vs DQfD")
    ap.add_argument("--id", type=int, default=1, help="robot (model dan rute uji)")
    ap.add_argument("--uji", default="semua", choices=("semua",) + UJI)
    ap.add_argument("--maks", type=int, default=0, help="batasi rute per peta (0 = semua)")
    ap.add_argument("--ulang", type=int, default=3, help="ulangan per rute untuk uji waktu")
    ap.add_argument("--kepala-p", default=None,
                    help="P dari model terpisah (drl.kelayakan), mis. model_dqn/id01_kepala_p.pt")
    ap.add_argument("--tanpa-kepala-p", action="store_true",
                    help="jangan pakai model P terpisah id0N_kepala_p.pt yang dipasang perencana otomatis")
    ap.add_argument("--varian", default=None,
                    help="uji varian: daftar, mis. eager,lazy,belajar,belajar@0.3,lazy+pintas "
                         "(bawaan: tiga itu untuk model dengan kepala kelayakan, selain itu VARIAN)")
    ap.add_argument("--pencarian", choices=PD.PENCARIAN, default=None,
                    help="pencarian terbaik-dulu DQfD untuk uji waktu dan rincian (bawaan: seperti robot)")
    ap.add_argument("--forward-terpisah", action="store_true",
                    help="Q dan P dengan dua forward terpisah (bawaan: satu forward gabungan, "
                         "jaringan_numpy.GABUNG_FORWARD)")
    ap.add_argument("--keluaran", default=None, help="berkas JSON hasil (bawaan model_dqn/ukur_node_id0N_<mesin>.json)")
    args = ap.parse_args()
    if args.forward_terpisah:
        JN.GABUNG_FORWARD = False
    args.keluaran = os.path.abspath(args.keluaran or _keluaran_bawaan(args.id))
    if args.uji == "semua":
        # Proses terpisah: uji rincian dan tabrak mengganti fungsi di drl.perencana_dqfd.
        for u in UJI:
            print(f"\n##### uji {u}", flush=True)
            perintah = [sys.executable, "-m", "drl.ukur_node", "--id", str(args.id), "--uji", u,
                        "--maks", str(args.maks), "--ulang", str(args.ulang), "--keluaran", args.keluaran]
            if args.pencarian:
                perintah += ["--pencarian", args.pencarian]
            if args.varian:
                perintah += ["--varian", args.varian]
            if args.kepala_p:
                perintah += ["--kepala-p", args.kepala_p]
            if args.tanpa_kepala_p:
                perintah += ["--tanpa-kepala-p"]
            if args.forward_terpisah:
                perintah += ["--forward-terpisah"]
            subprocess.run(perintah, cwd=AKAR, check=True)
        print(f"\nSELESAI -> {args.keluaran}")
        return
    if args.uji != "waktu":
        _periksa_salinan()
    {"waktu": uji_waktu, "rincian": uji_rincian, "tabrak": uji_tabrak, "varian": uji_varian}[args.uji](args)


if __name__ == "__main__":
    main()
