"""
perencana_proses.py - PROTOTIPE: perencana setiap robot di proses terpisah.

Latar (catatan Obsidian 18, bagian 5): navigasi.py merencana di thread Python.
Thread berbagi GIL, jadi
  - dua robot yang merencana bersamaan saling memperlambat (waktu jam dinding
    masing-masing ikut memuat pekerjaan robot lain), dan
  - loop kendali 20 Hz terlambat 7-28 ms per detak selama rencana Hybrid A*
    yang panjang (+-0,5 s).
Menambah thread tidak menolong. Proses terpisah punya interpreter dan GIL
sendiri, jadi perencana benar-benar paralel dan loop kendali tidak terganggu.

Satu-satunya cara navigasi.py (dan semua alat di atasnya: drl/uji_robot_nyata,
drl/uji_navigasi, drl/uji_pose_akhir) merencana - perencanaan di thread sudah
dihapus. Mode thread di --banding hanya pembanding pengukuran. Isinya:

    PerencanaProses(rid, peta, jenis=("astar", "dqfd-murni"))
        .kirim(jenis, awal, tujuan, rintangan=None, cadangan=True) -> nomor
        .ambil(tunggu_s=0.0) -> None | (nomor, jalur, info)     # tidak memblok
        .rencanakan(jenis, awal, tujuan, ...) -> (jalur, info)   # memblok
        .tutup()

    PerencanaDiProses(perencana_lokal, proses, jenis)
        pengganti rencana.Perencana: rencanakan() di proses anak, sisanya lokal

    jalur dan info sama dengan rencana.Perencana.rencanakan() (jalur dalam SEL),
    ditambah info["pulang_pergi_s"] (kirim -> hasil diterima, diukur induk) dan
    info["ipc_s"] = pulang_pergi_s - waktu_s (biaya antar-proses + antre).

Proses anak (spawn) membuat Arena dan Perencana sendiri dari nama peta dan
nomor robot - objek perencana tidak dikirim antar-proses, hanya pose, poligon
rintangan, dan hasil.

    python perencana_proses.py --banding            # thread vs proses, peta Gudang
    python perencana_proses.py --banding --ulang 7 --beban-ms 5
"""
import argparse
import itertools
import json
import multiprocessing as mp
import os
import sys
import threading
import time

AKAR = os.path.dirname(os.path.abspath(__file__))
if AKAR not in sys.path:
    sys.path.insert(0, AKAR)


# ====================================================================== anak
def _pekerja(pipa, rid, peta, jenis, cm_per_cell):
    """Isi proses anak: buat perencana, lalu layani permintaan sampai None."""
    try:
        t0 = time.perf_counter()
        import konfig
        import arena as A
        import rencana as R
        konfig.pilih_robot(rid)
        arena = A.Arena(peta, cm_per_cell=cm_per_cell)
        per = {j: R.Perencana(arena, konfig.profil(rid), jenis=j) for j in jenis}
        pipa.send(("siap", {"siap_s": time.perf_counter() - t0, "pid": os.getpid()}))
    except Exception as e:                                   # dilaporkan ke induk
        pipa.send(("error", f"{type(e).__name__}: {e}"))
        return
    while True:
        try:
            pesan = pipa.recv()
        except (EOFError, OSError):
            return
        if pesan is None:
            return
        nomor, j, awal, tujuan, rintangan, cadangan = pesan
        try:
            jalur, info = per[j].rencanakan(awal, tujuan, rintangan, cadangan)
        except Exception as e:
            jalur, info = None, {"error": f"{type(e).__name__}: {e}"}
        info["pid"] = os.getpid()               # bukti: rencana dikerjakan proses ini
        pipa.send(("hasil", (nomor, jalur, info)))


# ===================================================================== induk
class PerencanaProses:
    """Perencana satu robot di proses anak. Satu permintaan diproses pada satu
    waktu, berurutan; permintaan berikutnya boleh dikirim sebelum hasil diambil.

    tunggu=False: proses dimulai tanpa menunggu siap - panggil tunggu_siap()
    kemudian. Dengan begitu beberapa robot memuat peta dan model BERSAMAAN.
    """

    def __init__(self, rid, peta, jenis=("astar", "dqfd-murni"), cm_per_cell=None, batas_siap_s=60.0,
                 tunggu=True):
        ctx = mp.get_context("spawn")
        self.rid = rid
        self.jenis = tuple(jenis)
        self._pipa, anak = ctx.Pipe()
        self._t0 = time.perf_counter()
        self._batas_siap_s = batas_siap_s
        self.proses = ctx.Process(target=_pekerja, args=(anak, rid, peta, self.jenis, cm_per_cell),
                                  daemon=True, name=f"perencana-{rid:02d}")
        self.proses.start()
        anak.close()
        self._nomor = itertools.count(1)
        self._dikirim = {}
        # rencanakan() bisa dipanggil dari thread pekerja beberapa Unit/fase;
        # satu pipa hanya boleh dipakai satu pemanggil pada satu waktu.
        self._kunci = threading.Lock()
        self.siap_s = self.siap_anak_s = None
        if tunggu:
            self.tunggu_siap()

    def tunggu_siap(self):
        if self.siap_s is not None:
            return
        if not self._pipa.poll(self._batas_siap_s):
            self.tutup()
            raise TimeoutError(f"perencana robot {self.rid:02d} tidak siap dalam {self._batas_siap_s:.0f} s")
        jenis_pesan, isi = self._pipa.recv()
        if jenis_pesan != "siap":
            self.tutup()
            raise RuntimeError(f"perencana robot {self.rid:02d} gagal dimulai: {isi}")
        self.siap_s = time.perf_counter() - self._t0      # termasuk spawn + impor + muat model
        self.siap_anak_s = isi["siap_s"]                  # di dalam anak saja

    def kirim(self, jenis, awal, tujuan, rintangan=None, cadangan=True):
        nomor = next(self._nomor)
        self._dikirim[nomor] = time.perf_counter()
        self._pipa.send((nomor, jenis, tuple(awal), tuple(tujuan), rintangan, cadangan))
        return nomor

    @property
    def pipa(self):
        """Ujung pipa induk, untuk multiprocessing.connection.wait() di loop kendali."""
        return self._pipa

    @property
    def sibuk(self):
        return bool(self._dikirim)

    def ambil(self, tunggu_s=0.0):
        if not self._pipa.poll(tunggu_s):
            return None
        jenis_pesan, (nomor, jalur, info) = self._pipa.recv()
        t_kirim = self._dikirim.pop(nomor, None)
        if t_kirim is not None:
            info["pulang_pergi_s"] = time.perf_counter() - t_kirim
            info["ipc_s"] = info["pulang_pergi_s"] - info.get("waktu_s", 0.0)
        return nomor, jalur, info

    def rencanakan(self, jenis, awal, tujuan, rintangan=None, cadangan=True):
        """Memblok sampai hasil tiba. Menunggu dengan recv() - bukan poll(batas):
        di Windows batas waktu poll memakai timer +-15,6 ms, recv() bangun tepat
        saat data tiba. Selama menunggu, thread pemanggil tidak memegang GIL."""
        with self._kunci:
            nomor = self.kirim(jenis, awal, tujuan, rintangan, cadangan)
            t_kirim = self._dikirim[nomor]
            while True:
                try:
                    _, (n, jalur, info) = self._pipa.recv()
                except (EOFError, OSError) as e:
                    self._dikirim.pop(nomor, None)
                    return None, {"error": f"proses perencana robot {self.rid:02d} berhenti: {e}"}
                self._dikirim.pop(n, None)
                if n == nomor:
                    info["pulang_pergi_s"] = time.perf_counter() - t_kirim
                    info["ipc_s"] = info["pulang_pergi_s"] - info.get("waktu_s", 0.0)
                    return jalur, info

    def tutup(self):
        try:
            self._pipa.send(None)
        except (OSError, BrokenPipeError):
            pass
        self.proses.join(2.0)
        if self.proses.is_alive():
            self.proses.terminate()


class PerencanaDiProses:
    """Pengganti rencana.Perencana untuk navigasi.py: rencanakan() dikerjakan
    proses anak robot ini, semua yang lain (margin_untuk_pose, peta_dengan,
    pose_bebas, _untuk, ...) oleh Perencana LOKAL yang identik.

    navigasi.py memanggil rencanakan() dari thread penunggu; thread itu hanya
    menunggu pipa, jadi tidak memegang GIL dan tidak mengganggu loop kendali.
    navigasi.Unit.minta_rencana menolak perencana yang bukan kelas ini.
    """

    def __init__(self, lokal, proses, jenis):
        if jenis not in proses.jenis:
            raise ValueError(f"proses robot {proses.rid:02d} tidak memuat perencana '{jenis}'")
        self.lokal, self.proses, self.jenis = lokal, proses, jenis

    def rencanakan(self, awal_cm, tujuan_cm, rintangan_cm=None, cadangan=True):
        jalur, info = self.proses.rencanakan(self.jenis, awal_cm, tujuan_cm, rintangan_cm, cadangan)
        # Keadaan "jalur terakhir" yang dibaca navigasi dari perencana lokal.
        if "kendaraan" in info:
            self.lokal.kendaraan_dipakai = info["kendaraan"]
        if "margin_cm" in info:
            self.lokal.margin_dipakai = info["margin_cm"]
        return jalur, info

    def __getattr__(self, nama):
        return getattr(self.lokal, nama)


# =================================================================== banding
def _kasus(peta_skenario):
    """[(nama, jenis, {rid: (awal, tujuan)})]: skenario uji + satu rencana A* panjang."""
    with open(peta_skenario, encoding="utf-8") as f:
        data = json.load(f)
    out = []
    for jenis in ("astar", "dqfd-murni"):
        for sk in data["skenario"]:
            out.append((f"{sk['nama']}", jenis,
                        {int(r): (tuple(v["awal"]), tuple(v["tujuan"])) for r, v in sk["robot"].items()}))
    # Rencana A* panjang (+-0,5 s) - kasus yang membuat loop kendali terlambat.
    out.append(("A*-panjang", "astar", {1: ((195.75, 26.0, -90.0), (32.0, 170.0, -90.0)),
                                        2: ((32.0, 40.0, 90.0), (196.25, 167.4, -90.0))}))
    return data["peta"], out


def _sibuk(ms):
    s = time.perf_counter()
    while time.perf_counter() - s < ms / 1000.0:
        pass


class _ModeThread:
    """Seperti navigasi.py: satu thread per rencana, bergiliran (kunci) atau bersamaan."""

    def __init__(self, arena, ids, jenis, bergiliran):
        import konfig
        import rencana as R
        self.per = {}
        for rid in ids:
            konfig.pilih_robot(rid)
            self.per[rid] = {j: R.Perencana(arena, konfig.profil(rid), jenis=j) for j in jenis}
        self.kunci = threading.Lock() if bergiliran else None
        self.hasil = {}

    def kirim(self, rid, jenis, awal, tujuan):
        t0 = time.perf_counter()

        def kerja():
            if self.kunci:
                with self.kunci:
                    j, info = self.per[rid][jenis].rencanakan(awal, tujuan)
            else:
                j, info = self.per[rid][jenis].rencanakan(awal, tujuan)
            info["pulang_pergi_s"] = time.perf_counter() - t0
            self.hasil[rid] = (j, info)
        threading.Thread(target=kerja, daemon=True).start()

    def ambil(self, rid):
        return self.hasil.pop(rid, None)

    def tunggu(self, sisa_s):
        time.sleep(sisa_s)


class _ModeProses:
    def __init__(self, peta, ids, jenis):
        self.p = {rid: PerencanaProses(rid, peta, jenis) for rid in ids}
        self.hasil = {}

    def kirim(self, rid, jenis, awal, tujuan):
        self.p[rid].kirim(jenis, awal, tujuan)

    def ambil(self, rid):
        return self.hasil.pop(rid, None)

    def tunggu(self, sisa_s):
        """Tidur sampai detak berikutnya, tetapi terima hasil saat tiba (+-1 ms),
        supaya waktu kirim->hasil tidak dibulatkan ke periode loop.

        Bukan multiprocessing.connection.wait(): di Windows batas waktunya
        memakai timer +-15,6 ms dan bisa tidur jauh melewati detak (terukur:
        loop terlambat sampai 15 ms). poll(0) + time.sleep() kecil - sleep
        Python di Windows beresolusi tinggi.
        """
        batas = time.perf_counter() + sisa_s
        while True:
            for rid, p in self.p.items():
                if p.sibuk:
                    h = p.ambil(0.0)
                    if h is not None:
                        self.hasil[rid] = (h[1], h[2])
            sisa = batas - time.perf_counter()
            if sisa <= 0:
                return
            time.sleep(min(sisa, 0.001) if any(p.sibuk for p in self.p.values()) else sisa)

    def tutup(self):
        for p in self.p.values():
            p.tutup()


def _jalankan_kasus(mode, robot, jenis, periode_s, beban_ms):
    """Semua robot diberi tujuan pada detak 0; loop berdetak sampai semua hasil masuk."""
    for rid, (a, t) in robot.items():
        mode.kirim(rid, jenis, a, t)
    t_detak = time.perf_counter()
    telat, hasil = [], {}
    while len(hasil) < len(robot):
        _sibuk(beban_ms)                                    # kerja loop (gambar, pelacak, wasit)
        for rid in robot:
            if rid not in hasil:
                h = mode.ambil(rid)
                if h is not None:
                    hasil[rid] = h
        t_detak += periode_s
        sisa = t_detak - time.perf_counter()
        if sisa > 0:
            mode.tunggu(sisa)
        telat.append(1000.0 * (time.perf_counter() - t_detak))
    makespan_rencana = max(h[1]["pulang_pergi_s"] for h in hasil.values())
    return hasil, telat, makespan_rencana


def banding(args):
    import numpy as np
    import konfig
    import arena as A
    peta, kasus = _kasus(args.skenario)
    ids = sorted({rid for _, _, r in kasus for rid in r})
    jenis = sorted({j for _, j, _ in kasus})
    konfig.pilih_robot(ids[0])
    arena = A.Arena(peta)
    periode = konfig.RC_PERIODE_S

    print(f"[i] peta {peta}, robot {ids}, periode loop {1000 * periode:.0f} ms, beban loop {args.beban_ms:.0f} ms, "
          f"{args.ulang} ulangan, Python {sys.version.split()[0]}")
    t0 = time.perf_counter()
    mode_proses = _ModeProses(peta, ids, jenis)
    print(f"[i] proses anak siap dalam {time.perf_counter() - t0:.2f} s "
          f"({', '.join(f'{rid:02d}: {p.siap_s:.2f} s, di anak {p.siap_anak_s:.2f} s' for rid, p in mode_proses.p.items())})")
    semua_mode = {"thread bersamaan": _ModeThread(arena, ids, jenis, bergiliran=False),
                  "thread bergiliran": _ModeThread(arena, ids, jenis, bergiliran=True),
                  "proses per robot": mode_proses}
    try:
        # pemanasan: satu kali tiap kasus di tiap mode (cache, JIT)
        for m in semua_mode.values():
            for _, j, robot in kasus:
                _jalankan_kasus(m, robot, j, periode, 0.0)
        baris = []
        for nama, j, robot in kasus:
            for nama_mode, m in semua_mode.items():
                rek = {"jam": [], "cpu": [], "pp": [], "ipc": [], "selesai": [], "telat_maks": [], "telat_med": []}
                per_robot = {rid: {"jam": [], "pp": []} for rid in robot}
                for _ in range(args.ulang):
                    hasil, telat, selesai = _jalankan_kasus(m, robot, j, periode, args.beban_ms)
                    for rid, (jl, info) in hasil.items():
                        per_robot[rid]["jam"].append(1000 * info["waktu_s"])
                        per_robot[rid]["pp"].append(1000 * info["pulang_pergi_s"])
                        rek["jam"].append(1000 * info["waktu_s"])
                        rek["cpu"].append(1000 * info.get("cpu_s", float("nan")))
                        rek["pp"].append(1000 * info["pulang_pergi_s"])
                        rek["ipc"].append(1000 * info.get("ipc_s", float("nan")))
                    rek["selesai"].append(1000 * selesai)
                    rek["telat_maks"].append(max(telat))
                    rek["telat_med"].append(float(np.median(telat)))
                med = {k: float(np.nanmedian(v)) if v else float("nan") for k, v in rek.items()}
                med["telat_maks"] = float(max(rek["telat_maks"]))
                med["robot"] = {rid: {k: float(np.median(v)) for k, v in d.items()} for rid, d in per_robot.items()}
                baris.append((nama, j, nama_mode, med))
                rinci = "  ".join(f"{rid:02d}: {d['jam']:.1f}/{d['pp']:.1f}" for rid, d in med["robot"].items())
                print(f"  {nama:10s} {j:10s} {nama_mode:17s} | rencana jam {med['jam']:7.1f}  cpu {med['cpu']:7.1f} | "
                      f"kirim->hasil {med['pp']:7.1f}  ipc {med['ipc']:5.1f} | kedua robot selesai {med['selesai']:7.1f} ms | "
                      f"telat loop maks {med['telat_maks']:5.1f}  median {med['telat_med']:5.1f} ms | "
                      f"per robot jam/kirim->hasil {rinci}")
        if args.json:
            with open(args.json, "w", encoding="utf-8") as f:
                json.dump([{"kasus": n, "jenis": j, "mode": md, **v} for n, j, md, v in baris], f, indent=1)
            print(f"[OK] {args.json}")
        return baris
    finally:
        mode_proses.tutup()


def main():
    ap = argparse.ArgumentParser(description="Prototipe perencana multi-proses: banding dengan thread")
    ap.add_argument("--banding", action="store_true", help="jalankan perbandingan thread vs proses")
    ap.add_argument("--skenario", default=os.path.join(AKAR, "uji_nyata", "Gudang.json"))
    ap.add_argument("--ulang", type=int, default=5)
    ap.add_argument("--beban-ms", type=float, default=5.0, help="kerja loop utama per detak (ms)")
    ap.add_argument("--json", default=None, help="simpan hasil banding ke berkas JSON ini")
    args = ap.parse_args()
    if not args.banding:
        ap.print_help()
        return
    banding(args)


if __name__ == "__main__":
    main()
