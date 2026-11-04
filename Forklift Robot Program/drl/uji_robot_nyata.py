"""
uji_robot_nyata.py - uji DUA robot nyata: dqfd-murni vs Hybrid A* + RS, dengan
robot kembali sendiri ke titik awal sebelum setiap percobaan.

Yang dijalankan adalah navigasi.py sungguhan (kamera bersama, wasit, pure
pursuit, keselamatan zona), hanya klik mouse diganti protokol:

    PULANG      setiap robot yang belum di pose AWAL-nya dikirim ke sana dengan
                perencana pulang (bawaan astar). Tidak diukur. Bila sesudah
                --coba-pulang kali masih di luar toleransi, program berhenti dan
                operator memilih: n = terima pose sekarang, p = coba pulang lagi
                (robot boleh digeser dengan tangan dulu).
    KONFIRMASI  pose awal SEBENARNYA diukur (rata-rata 1 s). Operator menekan n
                untuk mulai - kecuali --tanpa-konfirmasi.
    UJI         kedua robot diberi tujuannya dengan perencana yang diuji,
                bersamaan, dan berjalan sampai keduanya selesai, gagal, atau
                --batas-uji habis. Pose akhir diukur (rata-rata 1 s).

Urutan perencana diselang-seling per ulangan (A B, B A, A B, ...), supaya baterai
yang melemah, panas motor, atau kamera yang bergeser tidak hanya menimpa satu
perencana.

    # 1. tetapkan skenario (sekali):
    python -m drl.uji_robot_nyata --peta Rumah --id 1,2 --buat uji_nyata/rumah.json
    # 2. jalankan:
    python -m drl.uji_robot_nyata --skenario uji_nyata/rumah.json --ulang 5
    # gladi bersih tanpa robot (simulasi, tanpa konfirmasi):
    python -m drl.uji_robot_nyata --skenario uji_nyata/rumah.json --simulasi --ulang 1
    # lanjutkan uji yang terputus (percobaan yang sudah tercatat dilewati):
    python -m drl.uji_robot_nyata --skenario uji_nyata/rumah.json --ulang 5 --lanjut uji_nyata/20261024_101500_Rumah
    # rincian waktu perencanaan untuk data lama (sebelum rincian_ms dicatat):
    python -m drl.uji_robot_nyata --rinci uji_nyata/20261024_101500_Rumah
    # ringkasan, gambar lintasan, dan baris tabel LaTeX dari hasil:
    python -m drl.uji_robot_nyata --ringkas uji_nyata/20261024_101500_Rumah

MODE --buat (jendela navigasi):
    1 / 2              pilih robot aktif
    klik-tarik kiri    TUJUAN robot aktif (wajib ditarik: arah hadap diukur)
    klik-tarik kanan   AWAL robot aktif (simulasi: robot dipindah ke sana)
    a                  AWAL semua robot = pose terukur sekarang
    w                  simpan skenario ini, mulai skenario berikutnya
    q                  simpan berkas dan keluar
  Setiap tujuan diperiksa dengan SEMUA perencana uji dari awalnya (robot lain
  di pose awalnya sebagai rintangan). Skenario yang gagal direncanakan salah
  satu perencana ditolak - uji itu hanya akan mengukur kegagalan yang sudah
  diketahui.

MODE UJI, tombol:
    n  lanjut / konfirmasi        p  coba pulang lagi (saat diminta operator)
    spasi  tahan semua robot      e  estop semua (percobaan dicatat estop manual)
    x  lewati percobaan ini        q  keluar (hasil yang sudah ada tersimpan)

Keluaran (folder uji_nyata/<tanggal>_<peta>/):
    skenario.json   salinan skenario yang dipakai
    hasil.jsonl     satu baris per percobaan (ditulis segera - aman bila terputus)
    lintasan.csv    pose setiap siklus 20 Hz, semua robot, semua fase
    (--ringkas)     ringkasan.json, ringkasan_tabel.tex, lintasan_<skenario>.png,
                    rincian_tabel.tex (waktu perencanaan: Hybrid A*, RS, DQfD jauh, DQfD dekat)
    (--rinci)       rincian_ulang.json - setiap perencanaan yang tercatat diulang
                    offline untuk mengukur rinciannya (data lama tanpa rincian_ms)
"""
import argparse
import csv
import json
import math
import os
import shutil
import sys
import time
from types import SimpleNamespace

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                       # noqa: E402

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIR_HASIL = os.path.join(AKAR, "uji_nyata")
PERENCANA_UJI = ("astar", "dqfd-murni")
NAMA = {"astar": "Hybrid A* + RS", "dqfd": "DQfD + cadangan A*", "dqfd-murni": "DQfD murni"}
WARNA_PLOT = {"astar": "#1f77b4", "dqfd": "#2ca02c", "dqfd-murni": "#d62728"}

TENANG_S = 1.0          # robot dianggap diam sekian detik setelah SELESAI; pose diukur di sini
GAGAL_RENCANA_S = 1.0   # SIAP tanpa jalur selama ini setelah tujuan diberikan = perencanaan gagal

PULANG, KONFIRMASI, UJI, OPERATOR, TAMAT = "PULANG", "KONFIRMASI", "UJI", "OPERATOR", "TAMAT"

# Bagian waktu perencanaan (kunci info["rincian_s"] dari rencana.Perencana).
BAGIAN = (("astar", "Hybrid A*"), ("rs", "RS"), ("dqfd_jauh", "DQfD jauh"), ("dqfd_dekat", "DQfD dekat"),
          ("gerbang", "gerbang mutu"), ("lain", "lain"))


def _sudut(a):
    return (a + 180.0) % 360.0 - 180.0


def _galat(pose, target):
    """(jarak_cm, |selisih arah| deg, maju_cm, samping_cm) pose terhadap target berarah."""
    dx, dy = pose[0] - target[0], pose[1] - target[1]
    c, s = math.cos(math.radians(target[2])), math.sin(math.radians(target[2]))
    return math.hypot(dx, dy), abs(_sudut(pose[2] - target[2])), c * dx + s * dy, -s * dx + c * dy


def _rata_pose(sampel):
    """Rata-rata pose (x, y, yaw); yaw dirata-rata secara melingkar. None bila kosong."""
    if not sampel:
        return None
    a = np.asarray(sampel, dtype=float)
    yaw = math.degrees(math.atan2(np.mean(np.sin(np.radians(a[:, 2]))),
                                  np.mean(np.cos(np.radians(a[:, 2])))))
    return [float(a[:, 0].mean()), float(a[:, 1].mean()), yaw]


def _jalur_cm(arena, jalur):
    return [list(arena.pose_ke_cm(p[0], p[1], p[2])) + [int(p[3])] for p in jalur]


def _ringkas_jalur(jalur_cm):
    """(panjang_cm, ganti_arah) jalur rencana dalam cm."""
    j = jalur_cm
    panjang = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(j[:-1], j[1:]))
    return float(panjang), int(sum(a[3] != b[3] for a, b in zip(j[:-1], j[1:])))


def muat_skenario(path):
    with open(path, encoding="utf-8") as f:
        d = json.load(f)
    for sk in d["skenario"]:
        sk["robot"] = {int(k): v for k, v in sk["robot"].items()}
        for r in sk["robot"].values():
            if r.get("tujuan") is None or r["tujuan"][2] is None:
                raise SystemExit(f"[!] {path}: tujuan skenario {sk['nama']} harus punya arah hadap.")
    return d


def daftar_percobaan(skenario, perencana, ulang):
    """Urutan percobaan: per skenario, per ulangan, perencana diselang-seling."""
    out = []
    for sk in skenario:
        for k in range(ulang):
            urut = list(perencana) if k % 2 == 0 else list(reversed(perencana))
            for i, per in enumerate(urut):
                out.append({"skenario": sk["nama"], "ulang": k, "urutan": i, "perencana": per})
    return out


# ====================================================================== sesi
class Sesi:
    """Navigasi.py tanpa klik: satu objek untuk mode --buat dan mode uji."""

    def __init__(self, args, ids, peta):
        import arena as A
        import navigasi as N
        import rencana as R
        self.A, self.N, self.R = A, N, R
        self.args = args
        nav_args = SimpleNamespace(peta=peta, sel=None, tracker=konfig.PELACAK_BAWAAN, perencana="astar",
                                   simulasi=args.simulasi, derau=args.derau, derau_deg=0.0, margin=10.0,
                                   url=args.url, host=args.host, log=None)
        self.nav = N.Navigasi(nav_args, ids)
        self.arena = self.nav.arena
        self.nav.tunjuk = A.PenunjukPose(self.arena.ctx)
        # Satu objek perencana per (robot, jenis). Semua dibuat SEKARANG, supaya
        # model DQfD yang hilang atau tidak cocok dengan profil kemudi ketahuan
        # sebelum robot pertama bergerak, bukan di tengah uji.
        jenis = set(args.perencana) | {args.perencana_pulang}
        self.per = {u.id: {j: R.Perencana(self.arena, u.p, jenis=j) for j in sorted(jenis)}
                    for u in self.nav.units}
        if getattr(args, "perencana_proses", False):
            # Satu proses per robot memuat SEMUA jenis yang diuji; rencanakan()
            # dikerjakan di sana, kueri lain oleh Perencana lokal di atas.
            import perencana_proses as PP
            proses = self.nav.pasang_perencana_proses({u.id: tuple(sorted(jenis)) for u in self.nav.units})
            self.per = {rid: {j: PP.PerencanaDiProses(per, proses[rid], j) for j, per in d.items()}
                        for rid, d in self.per.items()}
        # Panggilan perencana dicatat ke daftar ini selama fase UJI (None = tidak dicatat).
        self.catat_ke = None
        # time.time() saat _mulai_uji meminta rencana semua robot (acuan mulai_ms).
        self.t_minta = None
        for u in self.nav.units:
            for per in self.per[u.id].values():
                self._pasang_catat(u.id, per)

    def _pasang_catat(self, rid, per):
        asli = per.rencanakan
        arena = self.arena

        def catat(awal, tujuan, *a, **k):
            t0 = time.time()
            jalur, info = asli(awal, tujuan, *a, **k)
            daftar = self.catat_ke
            if daftar is not None:          # dipanggil dari thread pekerja; append atomik
                daftar[rid].append({"t": t0, "tujuan": list(tujuan), "ms": 1000.0 * info.get("waktu_s", 0.0),
                                    "cpu_ms": 1000.0 * info.get("cpu_s", 0.0),
                                    # --perencana-proses: biaya antar-proses (kirim -> hasil - rencana)
                                    "ipc_ms": (1000.0 * info["ipc_s"] if "ipc_s" in info else None),
                                    # jeda dari permintaan rencana pertama percobaan ini
                                    # (_mulai_uji) sampai rencana ini mulai: giliran
                                    # KUNCI_RENCANA + thread yang belum sempat berjalan
                                    "mulai_ms": (1000.0 * (t0 - self.t_minta) if self.t_minta is not None
                                                 else None),
                                    "rincian_ms": {k: 1000.0 * v for k, v in info.get("rincian_s", {}).items()},
                                    "ok": bool(jalur), "oleh": info.get("perencana", "A*") if jalur else "gagal",
                                    "margin_cm": info.get("margin_cm"),
                                    "alasan": "" if jalur else (info.get("error") or info.get("alasan") or ""),
                                    "jalur_cm": _jalur_cm(arena, jalur) if jalur else None})
            return jalur, info
        per.rencanakan = catat

    def pakai(self, jenis):
        for u in self.nav.units:
            u.per = self.per[u.id][jenis]
            u.args.perencana = jenis         # tampil di baris info navigasi

    def unit(self, rid):
        return self.nav.unit(rid)

    def langkah(self, now):
        self.nav.satu_langkah(now)

    def tutup(self):
        for u in self.nav.units:
            u.tutup()
        time.sleep(0.2)
        for u in self.nav.units:
            u.robot.putus()
        if self.nav.kamera is not None:
            self.nav.kamera.berhenti()

    def pindah_sim(self, rid, pose):
        u = self.unit(rid)
        if u.sim is not None:
            u.sim.x, u.sim.y, u.sim.yaw = pose

    def pose_bersih(self, u):
        """Pose terukur (bukan ramalan) robot ini, atau None."""
        p = u.pose
        if p is None or not p.terlihat or p.diramalkan:
            return None
        return (p.x, p.y, p.yaw)

    def gambar(self, sk, baris, aktif=None):
        import cv2
        img = self.nav.gambar()
        A, ctx = self.A, self.arena.ctx
        if sk is not None:
            for rid, r in sk["robot"].items():
                w = self.N.WARNA_ID.get(rid, (255, 255, 255))
                if r.get("awal"):
                    A.gambar_robot(img, ctx, *r["awal"], (160, 160, 160), tebal=1)
                    u, v = ctx.px([r["awal"][:2]])[0]
                    cv2.putText(img, f"A{rid}", (int(u) - 8, int(v) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                (160, 160, 160), 1, cv2.LINE_AA)
                if r.get("tujuan"):
                    A.gambar_robot(img, ctx, *r["tujuan"], w, tebal=1)
                    u, v = ctx.px([r["tujuan"][:2]])[0]
                    cv2.putText(img, f"T{rid}", (int(u) - 8, int(v) + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                w, 1, cv2.LINE_AA)
        y = img.shape[0] - 52 - 20 * (len(baris) - 1)
        for teks in baris:
            self.N.AC.label(img, teks, (10, y), (0, 255, 255), 0.5)
            y += 20
        return img


# ================================================================ mode --buat
def periksa_skenario(sesi, sk, perencana):
    """Daftar masalah skenario (kosong = layak). Setiap robot direncanakan dari
    awalnya ke tujuannya dengan setiap perencana, robot lain di pose AWAL-nya."""
    A = sesi.A
    masalah = []
    ids = list(sk["robot"])
    for rid in ids:
        r = sk["robot"][rid]
        if not r.get("awal") or not r.get("tujuan"):
            masalah.append(f"robot {rid:02d}: awal dan tujuan harus ditetapkan")
            continue
        lain = [A.jejak_badan(*sk["robot"][j]["awal"]) for j in ids if j != rid and sk["robot"][j].get("awal")]
        for jenis in perencana:
            per = sesi.per[rid][jenis]
            if per.margin_untuk_pose(*r["tujuan"], lain) is None:
                masalah.append(f"robot {rid:02d}: tujuan tidak muat (zona atau awal robot lain)")
                break
            simpan, sesi.catat_ke = sesi.catat_ke, None
            try:
                jalur, info = per.rencanakan(tuple(r["awal"]), tuple(r["tujuan"]), lain)
            finally:
                sesi.catat_ke = simpan
            if not jalur:
                masalah.append(f"robot {rid:02d}: {NAMA.get(jenis, jenis)} tidak menemukan jalur "
                               f"({info.get('alasan') or 'gagal'})")
            else:
                print(f"    {rid:02d} {NAMA.get(jenis, jenis):<15} {info['panjang_cm']:5.0f} cm  "
                      f"{1000 * info['waktu_s']:5.0f} ms")
    # Tujuan dua robot tidak boleh bertumpuk, dan tujuan tidak boleh di atas awal robot lain.
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            ra, rb = sk["robot"][a], sk["robot"][b]
            if ra.get("tujuan") and rb.get("tujuan"):
                import wasit as W
                if W.jarak_poligon(A.jejak_badan(*ra["tujuan"]), A.jejak_badan(*rb["tujuan"])) < konfig.MULTI_JARAK_AMAN_CM:
                    masalah.append(f"tujuan robot {a:02d} dan {b:02d} bertumpuk")
    return masalah


def mode_buat(args, ids):
    import cv2
    path = args.buat
    if os.path.exists(path):
        data = muat_skenario(path)
        if data["peta"] != args.peta:
            raise SystemExit(f"[!] {path} untuk peta {data['peta']}, bukan {args.peta}.")
        print(f"[i] {path}: {len(data['skenario'])} skenario lama, skenario baru ditambahkan.")
    else:
        data = {"peta": args.peta, "robot": ids, "skenario": []}
    sesi = Sesi(args, ids, args.peta)
    A = sesi.A
    tunjuk_awal = A.PenunjukPose(sesi.arena.ctx, tombol=cv2.EVENT_RBUTTONDOWN)
    aktif = ids[0]

    def baru():
        return {"nama": f"S{len(data['skenario']) + 1}", "robot": {rid: {"awal": None, "tujuan": None} for rid in ids}}
    sk = baru()
    judul = f"Buat skenario uji - {sesi.arena.nama}"
    cv2.namedWindow(judul, cv2.WINDOW_AUTOSIZE)

    def on_mouse(event, x, y, flags, param):
        sesi.nav.tunjuk.on_mouse(event, x, y, flags, param)
        tunjuk_awal.on_mouse(event, x, y, flags, param)
    cv2.setMouseCallback(judul, on_mouse)

    def simpan():
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=1)
        print(f"[OK] {path}: {len(data['skenario'])} skenario")

    try:
        while True:
            now = time.time()
            sesi.langkah(now)
            t = sesi.nav.tunjuk.ambil()
            if t is not None:
                if t[2] is None:
                    print("[!] tujuan harus DITARIK - arah hadap tujuan dipakai mengukur galat arah.")
                else:
                    sk["robot"][aktif]["tujuan"] = [round(v, 1) for v in t]
                    print(f"[i] {sk['nama']} robot {aktif:02d}: tujuan {sk['robot'][aktif]['tujuan']}")
            a = tunjuk_awal.ambil()
            if a is not None:
                if a[2] is None:
                    print("[!] awal harus DITARIK - robot dikembalikan ke arah hadap ini.")
                else:
                    sk["robot"][aktif]["awal"] = [round(v, 1) for v in a]
                    sesi.pindah_sim(aktif, a)
                    print(f"[i] {sk['nama']} robot {aktif:02d}: awal {sk['robot'][aktif]['awal']}")
            baris = [f"{sk['nama']} | robot aktif {aktif:02d} | kiri-tarik: TUJUAN  kanan-tarik: AWAL  "
                     "a: awal = pose sekarang  w: simpan skenario  q: keluar"]
            cv2.imshow(judul, sesi.gambar(sk, baris))
            k = cv2.waitKey(max(1, int(1000 * (konfig.RC_PERIODE_S - (time.time() - now))))) & 0xFF
            if k in (ord("q"), 27):
                break
            if ord("1") <= k <= ord("9") and k - ord("0") in ids:
                aktif = k - ord("0")
                print(f"[i] robot aktif: {aktif:02d}")
            elif k == ord("a"):
                for rid in ids:
                    p = sesi.pose_bersih(sesi.unit(rid))
                    if p is None:
                        print(f"[!] robot {rid:02d}: pose belum terukur (tag tidak terlihat)")
                    else:
                        sk["robot"][rid]["awal"] = [round(v, 1) for v in p]
                        print(f"[i] {sk['nama']} robot {rid:02d}: awal = pose sekarang {sk['robot'][rid]['awal']}")
            elif k == ord("w"):
                print(f"[i] memeriksa {sk['nama']} dengan {', '.join(args.perencana)} ...")
                masalah = periksa_skenario(sesi, sk, args.perencana)
                if masalah:
                    for m in masalah:
                        print(f"[!] {m}")
                    print("[!] skenario BELUM disimpan - perbaiki lalu tekan w lagi.")
                else:
                    data["skenario"].append(sk)
                    simpan()
                    sk = baru()
    finally:
        sesi.tutup()
        cv2.destroyAllWindows()
    if data["skenario"]:
        simpan()


# ================================================================== mode uji
class Protokol:
    """Mesin keadaan PULANG -> KONFIRMASI -> UJI untuk setiap percobaan."""

    def __init__(self, sesi, args, data, dir_hasil):
        self.sesi, self.args = sesi, args
        self.sk_map = {sk["nama"]: sk for sk in data["skenario"]}
        self.ids = [u.id for u in sesi.nav.units]
        self.dir = dir_hasil
        sudah = set()
        p_hasil = os.path.join(dir_hasil, "hasil.jsonl")
        if os.path.exists(p_hasil):
            with open(p_hasil, encoding="utf-8") as f:
                for baris in f:
                    h = json.loads(baris)
                    if h.get("status") != "dilewati":
                        sudah.add((h["skenario"], h["ulang"], h["perencana"]))
        semua = daftar_percobaan(data["skenario"], args.perencana, args.ulang)
        self.antrian = [p for p in semua if (p["skenario"], p["ulang"], p["perencana"]) not in sudah]
        if len(self.antrian) < len(semua):
            print(f"[i] {len(semua) - len(self.antrian)} percobaan sudah tercatat di {p_hasil} - dilewati.")
        self.f_hasil = open(p_hasil, "a", encoding="utf-8")
        p_lin = os.path.join(dir_hasil, "lintasan.csv")
        baru = not os.path.exists(p_lin)
        self.f_lin = open(p_lin, "a", newline="")
        self.w_lin = csv.writer(self.f_lin)
        if baru:
            self.w_lin.writerow(["t", "skenario", "ulang", "perencana", "fase", "robot", "state", "x_cm", "y_cm",
                                 "yaw_deg", "terlihat", "diramalkan", "speed_sps", "steer_deg"])
        self.i = 0
        self.fase = None
        self.terima_paksa = False
        self.awal_ukur = {}
        self.pesan = ""
        self.tunggu_n = False
        self._mulai_percobaan()

    # ------------------------------------------------------------ percobaan
    @property
    def kini(self):
        return self.antrian[self.i] if self.i < len(self.antrian) else None

    @property
    def sk(self):
        return self.sk_map[self.kini["skenario"]] if self.kini else None

    def _mulai_percobaan(self):
        if self.kini is None:
            self.fase = TAMAT
            self.pesan = "semua percobaan selesai - q untuk keluar"
            print(f"[OK] {self.pesan}")
            return
        p = self.kini
        print(f"\n[i] percobaan {self.i + 1}/{len(self.antrian)}: {p['skenario']} ulangan {p['ulang'] + 1}, "
              f"{NAMA.get(p['perencana'], p['perencana'])}")
        self.n_pulang = 0
        self.catatan_pulang = []
        self._mulai_pulang()

    def _mulai_pulang(self):
        self.fase = PULANG
        self.t_fase = time.time()
        self.sesi.pakai(self.args.perencana_pulang)
        self.selesai_sejak = None
        self.pulang_ids = []
        for rid in self.ids:
            u = self.sesi.unit(rid)
            p = self.sesi.pose_bersih(u)
            awal = self.sk["robot"][rid]["awal"]
            if p is not None and self._di_awal(p, awal):
                continue
            u.batalkan()
            u.terima_tujuan(tuple(awal))
            if u.tujuan is None or tuple(u.tujuan) != tuple(awal):
                print(f"[!] {rid:02d}: pulang ditolak - {u.alasan}")
            self.pulang_ids.append(rid)
        if self.pulang_ids:
            self.n_pulang += 1
            self.pesan = f"PULANG ke titik awal (coba {self.n_pulang}): robot " + \
                         ", ".join(f"{r:02d}" for r in self.pulang_ids)
        self.sampel = {rid: [] for rid in self.ids}

    def _di_awal(self, pose, awal):
        g_cm, g_deg, _, _ = _galat(pose, awal)
        return g_cm <= self.args.toleransi_cm and g_deg <= self.args.toleransi_deg

    def _berhenti(self, u, sejak):
        """'jalan' / 'tiba' / 'gagal' untuk robot yang diberi tujuan sejak waktu itu."""
        N = self.sesi.N
        if u.state == N.SELESAI:
            return "tiba"
        if u.state == N.BAHAYA:
            return "gagal"
        if u.state == N.SIAP and u.tracker is None and not u._sedang_merencana and not u.perlu_rencana \
                and time.time() - sejak > GAGAL_RENCANA_S:
            return "gagal"
        return "jalan"

    # ----------------------------------------------------------------- loop
    def langkah(self, now):
        self.sesi.langkah(now)
        self._tulis_lintasan(now)
        if self.fase == PULANG and self.terima_paksa:
            self._langkah_terima(now)
        elif self.fase == PULANG:
            self._langkah_pulang(now)
        elif self.fase == KONFIRMASI:
            self._langkah_konfirmasi(now)
        elif self.fase == UJI:
            self._langkah_uji(now)

    def _tulis_lintasan(self, now):
        p = self.kini or {"skenario": "", "ulang": "", "perencana": ""}
        for u in self.sesi.nav.units:
            if u.pose is None:
                continue
            sp, st = u._kirim_lalu
            self.w_lin.writerow([f"{now:.3f}", p["skenario"], p["ulang"], p["perencana"], self.fase, u.id, u.state,
                                 f"{u.pose.x:.2f}", f"{u.pose.y:.2f}", f"{u.pose.yaw:.2f}", int(u.pose.terlihat),
                                 int(u.pose.diramalkan), int(sp), f"{st:.2f}"])

    def _kumpul_sampel(self):
        for rid in self.ids:
            p = self.sesi.pose_bersih(self.sesi.unit(rid))
            if p is not None:
                self.sampel[rid].append(p)

    def _langkah_pulang(self, now):
        status = {rid: self._berhenti(self.sesi.unit(rid), self.t_fase) for rid in self.pulang_ids}
        if any(s == "jalan" for s in status.values()) and now - self.t_fase < self.args.batas_pulang:
            return
        if self.selesai_sejak is None:
            self.selesai_sejak = now
            self.sampel = {rid: [] for rid in self.ids}
        self._kumpul_sampel()
        if now - self.selesai_sejak < TENANG_S:
            return
        awal_ukur = {rid: _rata_pose(self.sampel[rid]) for rid in self.ids}
        jauh = [rid for rid in self.ids
                if awal_ukur[rid] is None or not self._di_awal(awal_ukur[rid], self.sk["robot"][rid]["awal"])]
        self.catatan_pulang.append({"coba": self.n_pulang, "status": status,
                                    "pose": {str(r): awal_ukur[r] for r in self.ids}})
        if not jauh:
            self._ke_konfirmasi(awal_ukur)
        elif self.n_pulang < self.args.coba_pulang:
            print(f"[i] robot {', '.join(f'{r:02d}' for r in jauh)} belum di titik awal - pulang lagi.")
            self._mulai_pulang()
        else:
            self.fase = OPERATOR
            self.awal_ukur = awal_ukur
            teks = []
            for rid in jauh:
                if awal_ukur[rid] is None:
                    teks.append(f"{rid:02d}: pose tak terukur")
                else:
                    g = _galat(awal_ukur[rid], self.sk["robot"][rid]["awal"])
                    teks.append(f"{rid:02d}: {g[0]:.1f} cm / {g[1]:.1f} deg")
            self.pesan = ("TIDAK sampai titik awal (" + "; ".join(teks) +
                          "). Geser dengan tangan bila perlu, lalu n = terima, p = pulang lagi")
            print(f"[!] {self.pesan}")

    def _langkah_terima(self, now):
        """Operator menerima pose sekarang (n): ukur ulang tanpa mengirim robot."""
        if self.selesai_sejak is None:
            self.selesai_sejak = now
            self.sampel = {rid: [] for rid in self.ids}
        self._kumpul_sampel()
        if now - self.selesai_sejak >= TENANG_S:
            awal = {rid: _rata_pose(self.sampel[rid]) for rid in self.ids}
            self.catatan_pulang.append({"coba": "operator", "pose": {str(r): awal[r] for r in self.ids}})
            self.terima_paksa = False
            self._ke_konfirmasi(awal)

    def _ke_konfirmasi(self, awal_ukur):
        self.awal_ukur = awal_ukur
        self.fase = KONFIRMASI
        self.t_fase = time.time()
        p = self.kini
        g = "; ".join(f"{rid:02d} {_galat(awal_ukur[rid], self.sk['robot'][rid]['awal'])[0]:.1f} cm"
                      for rid in self.ids if awal_ukur[rid] is not None)
        self.pesan = (f"SIAP: {p['skenario']} ulangan {p['ulang'] + 1}, {NAMA.get(p['perencana'], p['perencana'])} "
                      f"| galat awal {g}" + ("" if self.args.tanpa_konfirmasi else " | n = mulai"))
        print(f"[i] {self.pesan}")
        self.tunggu_n = not self.args.tanpa_konfirmasi

    def _langkah_konfirmasi(self, now):
        if not self.tunggu_n and now - self.t_fase >= 0.5:
            self._mulai_uji()

    def _mulai_uji(self):
        p = self.kini
        self.fase = UJI
        self.sesi.pakai(p["perencana"])
        self.t_fase = time.time()
        self.selesai_sejak = None
        self.rencana = {rid: [] for rid in self.ids}
        self.sesi.catat_ke = self.rencana
        self.ukur = {rid: {"t_tiba": None, "t_menunggu": 0.0, "n_minggir": 0, "zona_min_cm": float("inf"),
                           "jarak_cm": 0.0, "lalu": None, "state_lalu": None, "bahaya": "", "ditolak": ""}
                     for rid in self.ids}
        for rid in self.ids:
            u = self.sesi.unit(rid)
            u.batalkan()
        # Semua tujuan ditetapkan DULU, baru semua robot merencana. Tanpa ini
        # robot pertama merencana saat robot berikutnya belum punya tujuan
        # (jadi dianggap diam): hanya robot pertama yang menghindari pose awal
        # robot lain dan menanggung biaya peta rintangan sementara.
        for rid in self.ids:
            u = self.sesi.unit(rid)
            tj = tuple(self.sk["robot"][rid]["tujuan"])
            u.terima_tujuan(tj, tunda=True)
            if u.tujuan is None or tuple(u.tujuan) != tj:
                self.ukur[rid]["ditolak"] = u.alasan or "tujuan ditolak"
                print(f"[!] {rid:02d}: tujuan ditolak - {u.alasan}")
        self.sesi.t_minta = time.time()
        for rid in self.ids:
            u = self.sesi.unit(rid)
            if u.tujuan is not None:
                u.minta_rencana()
        self.pesan = f"UJI {p['skenario']} ulangan {p['ulang'] + 1}: {NAMA.get(p['perencana'], p['perencana'])}"
        print(f"[i] {self.pesan}")

    def _langkah_uji(self, now):
        N, AC, A = self.sesi.N, self.sesi.N.AC, self.sesi.A
        dt = konfig.RC_PERIODE_S
        status = {}
        for rid in self.ids:
            u, m = self.sesi.unit(rid), self.ukur[rid]
            status[rid] = "gagal" if m["ditolak"] else self._berhenti(u, self.t_fase)
            if u.state == N.MENUNGGU:
                m["t_menunggu"] += dt
            if u.state == N.MINGGIR and m["state_lalu"] != N.MINGGIR:
                m["n_minggir"] += 1
            if u.state == N.BAHAYA and not m["bahaya"]:
                m["bahaya"] = u.alasan
            if status[rid] == "tiba" and m["t_tiba"] is None:
                m["t_tiba"] = now - self.t_fase
            m["state_lalu"] = u.state
            p = self.sesi.pose_bersih(u)
            if p is not None:
                _, _, jarak = AC.evaluasi(A.jejak_badan(*p), self.sesi.arena.zona, 0.0)
                if np.isfinite(jarak):
                    m["zona_min_cm"] = min(m["zona_min_cm"], float(jarak))
                if m["lalu"] is not None and status[rid] == "jalan":
                    m["jarak_cm"] += math.hypot(p[0] - m["lalu"][0], p[1] - m["lalu"][1])
                m["lalu"] = p
        habis = now - self.t_fase > self.args.batas_uji
        if any(s == "jalan" for s in status.values()) and not habis:
            return
        if habis and self.selesai_sejak is None:
            for rid, s in status.items():
                if s == "jalan":
                    self.sesi.unit(rid).batalkan()
                    self.ukur[rid]["bahaya"] = self.ukur[rid]["bahaya"] or "batas waktu uji habis"
        if self.selesai_sejak is None:
            self.selesai_sejak = now
            self.status_akhir = status
            self.sampel = {rid: [] for rid in self.ids}
        self._kumpul_sampel()
        if now - self.selesai_sejak >= TENANG_S:
            self._selesai_uji(status="selesai")

    def _selesai_uji(self, status, catatan="", lanjut=True):
        self.sesi.catat_ke = None
        p, sk = self.kini, self.sk
        rec = {"skenario": p["skenario"], "ulang": p["ulang"], "urutan": p["urutan"], "perencana": p["perencana"],
               "status": status, "catatan": catatan, "waktu_mulai": time.strftime("%Y-%m-%d %H:%M:%S",
                                                                                    time.localtime(self.t_fase)),
               "simulasi": bool(self.args.simulasi), "pulang": self.catatan_pulang, "robot": {}}
        for rid in self.ids:
            r, m = sk["robot"][rid], self.ukur.get(rid, {})
            akhir = _rata_pose(self.sampel.get(rid, [])) if status == "selesai" else None
            if akhir is None and self.sesi.pose_bersih(self.sesi.unit(rid)) is not None:
                akhir = list(self.sesi.pose_bersih(self.sesi.unit(rid)))
            calls = self.rencana.get(rid, [])
            ok = [c for c in calls if c["ok"]]
            hasil = {"awal": r["awal"], "tujuan": r["tujuan"], "awal_ukur": self.awal_ukur.get(rid),
                     "tiba": self.status_akhir.get(rid) == "tiba" if status == "selesai" else False,
                     "t_tempuh_s": m.get("t_tiba"), "t_menunggu_s": m.get("t_menunggu"),
                     "n_minggir": m.get("n_minggir"), "estop": m.get("bahaya") or m.get("ditolak") or "",
                     "zona_min_cm": m.get("zona_min_cm"), "jarak_tempuh_cm": m.get("jarak_cm"),
                     "n_rencana": len(calls), "ms_rencana_total": float(sum(c["ms"] for c in calls)),
                     "rencana": calls, "akhir_ukur": akhir}
            if self.awal_ukur.get(rid) is not None:
                g = _galat(self.awal_ukur[rid], r["awal"])
                hasil.update(galat_awal_cm=g[0], galat_awal_deg=g[1])
            if ok:
                panjang, ganti = _ringkas_jalur(ok[0]["jalur_cm"])
                hasil.update(rencana_ms=ok[0]["ms"], rencana_oleh=ok[0]["oleh"], panjang_cm=panjang,
                             ganti_arah=ganti)
            elif calls:
                hasil.update(rencana_ms=calls[0]["ms"], rencana_oleh="gagal")
            if akhir is not None:
                g = _galat(akhir, r["tujuan"])
                hasil.update(galat_cm=g[0], galat_deg=g[1], galat_maju_cm=g[2], galat_samping_cm=g[3])
            rec["robot"][str(rid)] = hasil
        tiba = [rec["robot"][str(r)]["tiba"] for r in self.ids]
        rec["semua_tiba"] = all(tiba)
        tt = [rec["robot"][str(r)]["t_tempuh_s"] for r in self.ids]
        rec["makespan_s"] = max(tt) if all(t is not None for t in tt) else None
        self.f_hasil.write(json.dumps(rec, default=float) + "\n")
        self.f_hasil.flush()
        self.f_lin.flush()
        ringkas = "  ".join(
            f"{rid:02d}: {'tiba' if rec['robot'][str(rid)]['tiba'] else 'GAGAL'}"
            + (f" {rec['robot'][str(rid)]['galat_cm']:.1f} cm/{rec['robot'][str(rid)]['galat_deg']:.1f} deg"
               if rec['robot'][str(rid)].get('galat_cm') is not None else "")
            + (f" [{rec['robot'][str(rid)]['estop']}]" if rec['robot'][str(rid)]['estop'] else "")
            for rid in self.ids)
        print(f"[OK] {p['skenario']} u{p['ulang'] + 1} {p['perencana']} ({status}): {ringkas}")
        self.i += 1
        if lanjut:
            self._mulai_percobaan()

    # --------------------------------------------------------------- tombol
    def tombol(self, k):
        """True = keluar."""
        nav = self.sesi.nav
        if k in (ord("q"), 27):
            return True
        if k == ord(" "):
            nav.ditahan = not nav.ditahan
            if nav.ditahan:
                for u in nav.units:
                    u.robot.berhenti()
        elif k == ord("e"):
            for u in nav.units:
                u.henti_darurat("estop manual")
            if self.fase == UJI:
                # Dicatat sebagai percobaan gagal, tapi percobaan berikutnya
                # BELUM dimulai: robot tetap diam sampai operator menekan n.
                self.status_akhir = {rid: "gagal" for rid in self.ids}
                self.sampel = {}
                self._selesai_uji(status="estop manual", lanjut=False)
            self.fase = OPERATOR if self.kini is not None else TAMAT
            self.pesan = "ESTOP MANUAL - n = lanjut (pulang ke titik awal), q = keluar"
        elif k == ord("x") and self.kini is not None:
            if self.fase == UJI:
                for u in nav.units:
                    u.batalkan()
            self.sesi.catat_ke = None
            self.f_hasil.write(json.dumps({**self.kini, "status": "dilewati"}) + "\n")
            self.f_hasil.flush()
            print("[i] percobaan dilewati.")
            self.i += 1
            self._mulai_percobaan()
        elif k == ord("n"):
            if self.fase == KONFIRMASI:
                self.tunggu_n = False
            elif self.fase == OPERATOR and self.kini is not None:
                if self.pesan.startswith("TIDAK"):
                    # Operator menerima pose sekarang (mungkin sudah digeser
                    # dengan tangan): diukur ulang, robot tidak dikirim.
                    for u in nav.units:
                        u.batalkan()
                    self.fase = PULANG
                    self.selesai_sejak = None
                    self.terima_paksa = True
                else:
                    self.n_pulang = 0
                    self._mulai_pulang()
        elif k == ord("p") and self.fase == OPERATOR and self.kini is not None:
            self.terima_paksa = False
            self.n_pulang = 0
            self._mulai_pulang()
        return False

    def tutup(self):
        self.f_hasil.close()
        self.f_lin.close()


def mode_uji(args, ids):
    import cv2
    data = muat_skenario(args.skenario)
    ids_sk = sorted({rid for sk in data["skenario"] for rid in sk["robot"]})
    if ids != ids_sk:
        print(f"[i] robot mengikuti skenario: {', '.join(f'{r:02d}' for r in ids_sk)}")
        ids = ids_sk
    if args.lanjut:
        dir_hasil = args.lanjut
    else:
        dir_hasil = os.path.join(DIR_HASIL, time.strftime("%Y%m%d_%H%M%S_") + data["peta"]
                                 + ("_sim" if args.simulasi else ""))
    os.makedirs(dir_hasil, exist_ok=True)
    shutil.copy(args.skenario, os.path.join(dir_hasil, "skenario.json"))
    with open(os.path.join(dir_hasil, "pengaturan.json"), "w", encoding="utf-8") as f:
        json.dump({k: v for k, v in vars(args).items()}, f, indent=1)
    print(f"[i] hasil ke {dir_hasil}")

    sesi = Sesi(args, ids, data["peta"])
    judul = f"Uji robot nyata - {sesi.arena.nama}" + (" (SIMULASI)" if args.simulasi else "")
    cv2.namedWindow(judul, cv2.WINDOW_AUTOSIZE)
    if args.simulasi:
        # Gladi bersih: robot langsung ditaruh di awal skenario pertama.
        for rid in ids:
            sesi.pindah_sim(rid, data["skenario"][0]["robot"][rid]["awal"])
    # Pose semua robot harus sudah terukur sebelum protokol memutuskan siapa
    # yang perlu pulang - tanpa pose, setiap robot akan dikirim pulang.
    t0 = time.time()
    while time.time() - t0 < 15.0:
        now = time.time()
        sesi.langkah(now)
        if all(sesi.pose_bersih(u) is not None for u in sesi.nav.units) and now - t0 > 1.0:
            break
        cv2.imshow(judul, sesi.gambar(data["skenario"][0], ["menunggu pose semua robot dari kamera ..."]))
        if (cv2.waitKey(max(1, int(1000 * (konfig.RC_PERIODE_S - (time.time() - now))))) & 0xFF) == ord("q"):
            sesi.tutup()
            return
    else:
        print("[!] pose sebagian robot belum terukur setelah 15 s - robot itu akan dikirim pulang begitu terlihat.")
    proto = Protokol(sesi, args, data, dir_hasil)
    try:
        while True:
            now = time.time()
            proto.langkah(now)
            baris = [f"[{proto.fase}] {proto.pesan}",
                     f"percobaan {min(proto.i + 1, len(proto.antrian))}/{len(proto.antrian)} | "
                     "n: lanjut  spasi: tahan  e: estop  x: lewati  q: keluar"]
            cv2.imshow(judul, sesi.gambar(proto.sk, baris))
            k = cv2.waitKey(max(1, int(1000 * (konfig.RC_PERIODE_S - (time.time() - now))))) & 0xFF
            if k != 255 and proto.tombol(k):
                break
            if proto.fase == TAMAT and args.simulasi and args.tanpa_konfirmasi:
                break
    finally:
        sesi.tutup()
        proto.tutup()
        cv2.destroyAllWindows()
    ringkas(dir_hasil)


# ================================================================== ringkasan
def _muat_hasil(dir_hasil):
    with open(os.path.join(dir_hasil, "hasil.jsonl"), encoding="utf-8") as f:
        return [json.loads(b) for b in f if b.strip()]


def _stat(v, f="{:.1f}"):
    v = [x for x in v if x is not None]
    if not v:
        return "-"
    if len(v) == 1:
        return f.format(v[0])
    return f"{f.format(np.mean(v))} ± {f.format(np.std(v, ddof=1))}"


def ringkas(dir_hasil):
    """Tabel per perencana (dan per skenario), ringkasan.json, tabel LaTeX, gambar lintasan."""
    semua = [h for h in _muat_hasil(dir_hasil) if h.get("status") != "dilewati"]
    if not semua:
        print("[i] belum ada percobaan tercatat.")
        return
    perencana = [p for p in ("astar", "dqfd", "dqfd-murni") if any(h["perencana"] == p for h in semua)]
    ids = sorted({int(r) for h in semua for r in h["robot"]})
    skenario = sorted({h["skenario"] for h in semua})
    out = {"dir": dir_hasil, "per_perencana": {}}
    print(f"\n=== {dir_hasil}: {len(semua)} percobaan, skenario {', '.join(skenario)} ===")
    kepala = (f"{'perencana':<15} {'robot':>5} {'tiba':>7} {'rencana ms (med/p90)':>21} {'panjang cm':>12} "
              f"{'ganti arah':>10} {'tempuh s':>12} {'galat cm':>12} {'galat deg':>12} {'estop':>5} "
              f"{'tunggu s':>9} {'galat awal cm':>14}")
    print(kepala)
    for per in perencana:
        hs = [h for h in semua if h["perencana"] == per]
        o = {"n": len(hs), "pasangan_tiba": sum(h["semua_tiba"] for h in hs),
             "makespan_s": [h["makespan_s"] for h in hs if h["makespan_s"] is not None], "robot": {}}
        for rid in ids:
            rs = [h["robot"][str(rid)] for h in hs if str(rid) in h["robot"]]
            tiba = [r for r in rs if r["tiba"]]
            ms = [r["rencana_ms"] for r in rs if r.get("rencana_ms") is not None]
            ro = {"n": len(rs), "tiba": len(tiba),
                  "rencana_ms_median": float(np.median(ms)) if ms else None,
                  "rencana_ms_p90": float(np.percentile(ms, 90)) if ms else None,
                  "panjang_cm": [r.get("panjang_cm") for r in rs], "ganti_arah": [r.get("ganti_arah") for r in rs],
                  "t_tempuh_s": [r["t_tempuh_s"] for r in tiba], "galat_cm": [r.get("galat_cm") for r in tiba],
                  "galat_deg": [r.get("galat_deg") for r in tiba], "estop": sum(bool(r["estop"]) for r in rs),
                  "t_menunggu_s": [r.get("t_menunggu_s") for r in rs], "galat_awal_cm": [r.get("galat_awal_cm") for r in rs],
                  "galat_awal_deg": [r.get("galat_awal_deg") for r in rs]}
            o["robot"][rid] = ro
            med = f"{ro['rencana_ms_median']:.0f}/{ro['rencana_ms_p90']:.0f}" if ms else "-"
            print(f"{NAMA.get(per, per):<15} {rid:>5} {ro['tiba']:>3}/{ro['n']:<3} {med:>21} "
                  f"{_stat(ro['panjang_cm'], '{:.0f}'):>12} {_stat(ro['ganti_arah'], '{:.1f}'):>10} "
                  f"{_stat(ro['t_tempuh_s']):>12} {_stat(ro['galat_cm']):>12} {_stat(ro['galat_deg']):>12} "
                  f"{ro['estop']:>5} {_stat(ro['t_menunggu_s']):>9} {_stat(ro['galat_awal_cm']):>14}")
        print(f"{'':<15} pasangan tiba {o['pasangan_tiba']}/{o['n']}, makespan {_stat(o['makespan_s'])} s")
        out["per_perencana"][per] = o
    panggilan = _panggilan_tercatat(semua)
    judul = "rincian waktu perencanaan (tercatat saat uji)"
    if not panggilan and os.path.exists(os.path.join(dir_hasil, "rincian_ulang.json")):
        with open(os.path.join(dir_hasil, "rincian_ulang.json"), encoding="utf-8") as f:
            panggilan = json.load(f)["panggilan"]
        judul = "rincian waktu perencanaan (diukur ulang offline, --rinci)"
    if panggilan:
        out["rincian_rencana"] = _rincian_ringkas(panggilan, perencana, ids)
        _cetak_rincian(out["rincian_rencana"], judul)
        _tabel_rincian_latex(dir_hasil, out["rincian_rencana"])
    with open(os.path.join(dir_hasil, "ringkasan.json"), "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1, default=float)
    _tabel_latex(dir_hasil, out, perencana, ids)
    for sk in skenario:
        try:
            _gambar(dir_hasil, sk, [h for h in semua if h["skenario"] == sk], perencana, ids)
        except Exception as e:                          # gambar tidak boleh menggagalkan ringkasan
            print(f"[!] gambar {sk}: {type(e).__name__}: {e}")


def _panggilan_tercatat(semua):
    """Setiap panggilan perencana selama UJI yang punya rincian_ms, sebagai daftar datar."""
    out = []
    for h in semua:
        for rid, r in h["robot"].items():
            calls = r.get("rencana", [])
            i_ok = next((i for i, c in enumerate(calls) if c["ok"]), None)
            for i, c in enumerate(calls):
                if c.get("rincian_ms"):
                    out.append({"perencana": h["perencana"], "robot": int(rid), "skenario": h["skenario"],
                                "ulang": h["ulang"], "ke": i, "pertama": i == i_ok, "ok": c["ok"],
                                "ms_tercatat": c["ms"], "rincian_ms": c["rincian_ms"]})
    return out


def _rincian_ringkas(panggilan, perencana, ids):
    """Per perencana & robot: median tiap bagian pada rencana PERTAMA (sama dengan
    kolom 'rencana ms' di tabel utama), dan jumlah semua panggilan per percobaan
    (termasuk rencana ulang di tengah jalan)."""
    out = {}
    for per in perencana:
        for rid in ids:
            ps = [p for p in panggilan if p["perencana"] == per and p["robot"] == rid]
            if not ps:
                continue
            pertama = [p["rincian_ms"] for p in ps if p["pertama"]]
            kunci = [k for k, _ in BAGIAN if any(k in p["rincian_ms"] for p in ps)]
            total = {}
            for p in ps:
                c = (p["skenario"], p["ulang"])
                total[c] = total.get(c, 0.0) + sum(p["rincian_ms"].values())
            out.setdefault(per, {})[rid] = {
                "n_panggilan": len(ps), "n_pertama": len(pertama),
                "pertama_median_ms": ({k: float(np.median([r.get(k, 0.0) for r in pertama])) for k in kunci}
                                      if pertama else {}),
                "semua_per_percobaan_ms": list(total.values())}
    return out


def _cetak_rincian(r, judul):
    kunci = [k for k, _ in BAGIAN if any(k in ro["pertama_median_ms"] for o in r.values() for ro in o.values())]
    print(f"\n{judul} - ms, median rencana pertama (porsi)")
    print(f"{'perencana':<15} {'robot':>5} " + " ".join(f"{dict(BAGIAN)[k]:>14}" for k in kunci)
          + f" {'jumlah':>8} {'semua rencana/percobaan':>24}")
    for per, o in r.items():
        for rid, ro in o.items():
            m = ro["pertama_median_ms"]
            jml = sum(m.values())
            sel = " ".join(f"{m[k]:7.1f} ({100 * m[k] / jml:3.0f}%)" if k in m and jml > 0 else f"{'-':>14}"
                           for k in kunci)
            print(f"{NAMA.get(per, per):<15} {rid:>5} {sel} {jml:8.1f} "
                  f"{_stat(ro['semua_per_percobaan_ms']):>16} (n={ro['n_panggilan']})")


def _tabel_rincian_latex(dir_hasil, r):
    """Baris tabel rincian waktu perencanaan: Hybrid A* | RS | DQfD jauh | DQfD dekat | jumlah (ms)."""
    def f(v):
        return "--" if v is None else f"{v:.1f}".replace(".", "{,}")
    baris = ["% dibuat drl.uji_robot_nyata --ringkas " + os.path.basename(dir_hasil),
             "% perencana & robot & Hybrid A* & RS & DQfD jauh & DQfD dekat & jumlah (ms, median rencana pertama)"]
    for per, o in r.items():
        nama = NAMA.get(per, per).replace("*", "\\textsuperscript{*}")
        for rid, ro in o.items():
            m = ro["pertama_median_ms"]
            sel = " & ".join(f(m.get(k)) for k in ("astar", "rs", "dqfd_jauh", "dqfd_dekat"))
            baris.append(f"{nama} & ID {rid:02d} & {sel} & {f(sum(m.values()))} \\\\")
    p = os.path.join(dir_hasil, "rincian_tabel.tex")
    with open(p, "w", encoding="utf-8") as fh:
        fh.write("\n".join(baris) + "\n")
    print(f"[OK] {p}")


def ukur_ulang_rincian(dir_hasil, n_ulang=5):
    """Rincian waktu perencanaan untuk hasil uji LAMA (hasil.jsonl tanpa rincian_ms).

    Setiap panggilan perencana yang tercatat diulang offline dengan awal (titik
    pertama jalurnya), tujuan, perencana, dan robot yang sama. Rintangan saat
    itu tidak tercatat: badan robot lain bisa ikut atau tidak (tergantung ia
    diam atau tidak). Keduanya dicoba, dan yang dipakai adalah yang menghasilkan
    jalur dengan jumlah titik sama seperti rekaman ("cocok"). Pose robot lain
    diambil dari lintasan.csv pada saat panggilan itu.

    Waktu offline biasanya lebih kecil dari yang tercatat (tidak ada loop
    kamera & kendali 20 Hz yang berebut CPU) - yang dibandingkan adalah PORSI
    tiap bagian, bukan angka mutlaknya.
    """
    import bisect
    import arena as A
    import rencana as R
    with open(os.path.join(dir_hasil, "skenario.json"), encoding="utf-8") as f:
        peta = json.load(f)["peta"]
    semua = [h for h in _muat_hasil(dir_hasil) if h.get("status") != "dilewati"]
    ar = A.Arena(A.pilih(peta))
    lin = {}
    with open(os.path.join(dir_hasil, "lintasan.csv"), newline="") as f:
        for b in csv.DictReader(f):
            if b["terlihat"] == "1":
                lin.setdefault(int(b["robot"]), []).append(
                    (float(b["t"]), float(b["x_cm"]), float(b["y_cm"]), float(b["yaw_deg"])))
    waktu = {rid: [p[0] for p in v] for rid, v in lin.items()}

    def pose_pada(rid, t):
        if rid not in lin:
            return None
        i = bisect.bisect_left(waktu[rid], t)
        j = min((j for j in (i - 1, i) if 0 <= j < len(lin[rid])), key=lambda j: abs(waktu[rid][j] - t))
        return lin[rid][j][1:] if abs(waktu[rid][j] - t) < 1.0 else None

    per = {}
    panggilan = []
    print(f"[i] mengulang perencanaan dari {dir_hasil} ({n_ulang}x tiap panggilan) ...")
    for h in semua:
        for rid_s, r in h["robot"].items():
            rid = int(rid_s)
            if (rid, h["perencana"]) not in per:
                per[(rid, h["perencana"])] = R.Perencana(ar, konfig.profil(rid), jenis=h["perencana"])
            P = per[(rid, h["perencana"])]
            calls = r.get("rencana", [])
            i_ok = next((i for i, c in enumerate(calls) if c["ok"]), None)
            for i, c in enumerate(calls):
                if not c.get("jalur_cm"):
                    print(f"[!] {h['skenario']} u{h['ulang'] + 1} {h['perencana']} {rid:02d} #{i}: "
                          "panggilan gagal (awal tidak tercatat) - dilewati")
                    continue
                awal, tujuan = tuple(c["jalur_cm"][0][:3]), tuple(c["tujuan"])
                lain = [A.jejak_badan(*q) for q in (pose_pada(int(j), c["t"]) for j in h["robot"] if int(j) != rid)
                        if q is not None]
                terbaik = None
                for rint in ([None, lain] if lain else [None]):
                    jalur, _ = P.rencanakan(awal, tujuan, rint)
                    beda = abs((len(jalur) if jalur else 0) - len(c["jalur_cm"]))
                    if terbaik is None or beda < terbaik[0]:
                        terbaik = (beda, rint)
                beda, rint = terbaik
                ukur = [P.rencanakan(awal, tujuan, rint)[1] for _ in range(n_ulang)]
                kunci = [k for k, _ in BAGIAN if any(k in u["rincian_s"] for u in ukur)]
                rinc = {k: float(np.median([1000.0 * u["rincian_s"].get(k, 0.0) for u in ukur])) for k in kunci}
                p = {"perencana": h["perencana"], "robot": rid, "skenario": h["skenario"], "ulang": h["ulang"],
                     "ke": i, "pertama": i == i_ok, "ok": c["ok"], "ms_tercatat": c["ms"],
                     "ms_ulang": float(np.median([1000.0 * u["waktu_s"] for u in ukur])),
                     "cocok": beda == 0, "rintangan_robot_lain": rint is not None, "rincian_ms": rinc}
                panggilan.append(p)
                print(f"    {h['skenario']} u{h['ulang'] + 1} {NAMA.get(h['perencana'], h['perencana']):<15} "
                      f"{rid:02d} #{i}: tercatat {c['ms']:6.1f} ms, ulang {p['ms_ulang']:6.1f} ms "
                      f"[{'cocok' if p['cocok'] else f'beda {beda} titik'}"
                      f"{', robot lain = rintangan' if p['rintangan_robot_lain'] else ''}]  "
                      + "  ".join(f"{dict(BAGIAN)[k]} {v:.1f}" for k, v in rinc.items()))
    with open(os.path.join(dir_hasil, "rincian_ulang.json"), "w", encoding="utf-8") as f:
        json.dump({"dir": dir_hasil, "n_ulang": n_ulang, "panggilan": panggilan}, f, indent=1)
    print(f"[OK] {os.path.join(dir_hasil, 'rincian_ulang.json')}")


def _tabel_latex(dir_hasil, out, perencana, ids):
    """Baris tabel untuk paper (koma desimal, rata-rata ± simpangan baku)."""
    def f(v, d=1):
        v = [x for x in v if x is not None]
        if not v:
            return "--"
        s = f"{np.mean(v):.{d}f}" + (f" $\\pm$ {np.std(v, ddof=1):.{d}f}" if len(v) > 1 else "")
        return s.replace(".", "{,}")
    baris = ["% dibuat drl.uji_robot_nyata --ringkas " + os.path.basename(dir_hasil)]
    for per in perencana:
        o = out["per_perencana"][per]
        for rid in ids:
            r = o["robot"][rid]
            med = (f"{r['rencana_ms_median']:.0f}" if r["rencana_ms_median"] is not None else "--")
            nama = NAMA.get(per, per).replace("*", "\\textsuperscript{*}")
            baris.append(f"{nama} & ID {rid:02d} & "
                         f"{r['tiba']}/{r['n']} & {med} & {f(r['panjang_cm'], 0)} & {f(r['ganti_arah'])} & "
                         f"{f(r['t_tempuh_s'])} & {f(r['galat_cm'])} & {f(r['galat_deg'])} & {r['estop']} \\\\")
        baris.append(f"\\multicolumn{{10}}{{@{{}}l}}{{\\quad pasangan tiba {o['pasangan_tiba']}/{o['n']}, "
                     f"makespan {f(o['makespan_s'])}\\,s}} \\\\")
    p = os.path.join(dir_hasil, "ringkasan_tabel.tex")
    with open(p, "w", encoding="utf-8") as fh:
        fh.write("\n".join(baris) + "\n")
    print(f"[OK] {p}")


def _gambar(dir_hasil, nama_sk, hs, perencana, ids):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    import arena as A
    with open(os.path.join(dir_hasil, "skenario.json"), encoding="utf-8") as f:
        data = json.load(f)
    sk = next(s for s in data["skenario"] if s["nama"] == nama_sk)
    ar = A.Arena(data["peta"])
    lin = {}
    with open(os.path.join(dir_hasil, "lintasan.csv"), newline="") as f:
        for b in csv.DictReader(f):
            if b["fase"] != UJI or b["skenario"] != nama_sk or b["terlihat"] != "1":
                continue
            lin.setdefault((b["perencana"], b["ulang"], int(b["robot"])), []).append(
                (float(b["x_cm"]), float(b["y_cm"])))
    x0, y0 = ar.asal
    W, H = ar.lebar_sel * ar.cm_per_cell, ar.tinggi_sel * ar.cm_per_cell
    fig, sumbu = plt.subplots(1, len(perencana), figsize=(6.4 * len(perencana), 6), squeeze=False)
    gaya = {ids[0]: "-", ids[-1]: "--"} if len(ids) > 1 else {ids[0]: "-"}
    for ax, per in zip(sumbu[0], perencana):
        ax.imshow(ar.occ > 0, cmap="Greys", alpha=0.55, extent=(x0, x0 + W, y0 + H, y0), origin="upper",
                  interpolation="nearest")
        for rid in ids:
            r = sk["robot"][str(rid)]
            ax.add_patch(Polygon(np.asarray(A.jejak_badan(*r["awal"])), closed=True, fill=False, ec="#777777",
                                 lw=1, ls=":"))
            ax.add_patch(Polygon(np.asarray(A.jejak_badan(*r["tujuan"])), closed=True, fill=False, ec="black",
                                 lw=1.4, ls="--"))
            ax.text(r["awal"][0], r["awal"][1], f"A{rid}", fontsize=8, color="#555555", ha="center")
            ax.text(r["tujuan"][0], r["tujuan"][1], f"T{rid}", fontsize=8, ha="center")
            for h in hs:
                if h["perencana"] != per:
                    continue
                t = lin.get((per, str(h["ulang"]), rid))
                if t:
                    t = np.asarray(t)
                    ax.plot(t[:, 0], t[:, 1], gaya.get(rid, "-"), color=WARNA_PLOT.get(per, "k"), lw=1.2,
                            alpha=0.75)
                akhir = h["robot"][str(rid)].get("akhir_ukur")
                if akhir:
                    ax.plot(akhir[0], akhir[1], "o", color=WARNA_PLOT.get(per, "k"), ms=3)
        n = sum(h["perencana"] == per for h in hs)
        tiba = sum(h["semua_tiba"] for h in hs if h["perencana"] == per)
        ax.set_title(f"{NAMA.get(per, per)} - {nama_sk} di {data['peta']} ({tiba}/{n} pasangan tiba)", fontsize=10)
        ax.set_xlim(x0, x0 + W)
        ax.set_ylim(y0 + H, y0)
        ax.set_aspect("equal")
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
    fig.tight_layout()
    p = os.path.join(dir_hasil, f"lintasan_{nama_sk}.png")
    fig.savefig(p, dpi=120)
    plt.close(fig)
    print(f"[OK] {p}")


# ======================================================================= main
def main():
    import navigasi as N
    ap = argparse.ArgumentParser(description="Uji dua robot nyata: dqfd-murni vs Hybrid A* + RS, "
                                             "dengan pulang otomatis ke titik awal")
    ap.add_argument("--id", default="1,2", help="robot (mode --buat); mode uji mengikuti skenario")
    ap.add_argument("--peta", default=None, help="mode --buat: nama peta")
    ap.add_argument("--buat", metavar="JSON", help="tetapkan skenario dan simpan ke berkas ini")
    ap.add_argument("--skenario", metavar="JSON", help="jalankan uji dengan skenario ini")
    ap.add_argument("--ringkas", metavar="DIR", help="hanya ringkas hasil uji di folder ini")
    ap.add_argument("--rinci", metavar="DIR",
                    help="ulangi offline setiap perencanaan di folder ini untuk rincian waktunya, lalu ringkas")
    ap.add_argument("--lanjut", metavar="DIR", help="lanjutkan uji di folder ini (yang sudah tercatat dilewati)")
    ap.add_argument("--perencana", default=",".join(PERENCANA_UJI),
                    help=f"perencana yang diuji, dipisah koma (bawaan {','.join(PERENCANA_UJI)})")
    ap.add_argument("--perencana-pulang", default="astar", help="perencana untuk kembali ke titik awal")
    ap.add_argument("--perencana-proses", action="store_true",
                    help="setiap robot merencana di proses sendiri (perencana_proses.py), bukan di thread")
    ap.add_argument("--ulang", type=int, default=3, help="ulangan per perencana per skenario")
    ap.add_argument("--toleransi-cm", type=float, default=5.0, help="robot dianggap di titik awal dalam jarak ini")
    ap.add_argument("--toleransi-deg", type=float, default=10.0, help="... dan selisih arah hadap ini")
    ap.add_argument("--coba-pulang", type=int, default=3, help="percobaan pulang sebelum operator diminta")
    ap.add_argument("--batas-uji", type=float, default=180.0, help="batas waktu satu percobaan (s)")
    ap.add_argument("--batas-pulang", type=float, default=180.0, help="batas waktu satu perjalanan pulang (s)")
    ap.add_argument("--tanpa-konfirmasi", action="store_true",
                    help="mulai setiap percobaan tanpa menunggu n (selalu begitu di --simulasi)")
    ap.add_argument("--simulasi", action="store_true", help="gladi bersih tanpa kamera & robot")
    ap.add_argument("--derau", type=float, default=0.0, help="simulasi: derau pose (cm)")
    ap.add_argument("--url", default=None, help="override URL RTSP")
    ap.add_argument("--host", default=None, help="override alamat broker MQTT")
    args = ap.parse_args()

    if args.ringkas:
        ringkas(args.ringkas)
        return
    if args.rinci:
        ukur_ulang_rincian(args.rinci)
        ringkas(args.rinci)
        return
    args.perencana = [p.strip() for p in args.perencana.split(",") if p.strip()]
    if args.simulasi:
        args.tanpa_konfirmasi = True
    ids = N.urai_id(args.id)
    if args.buat:
        if not args.peta:
            ap.error("--buat butuh --peta")
    elif not args.skenario:
        ap.error("pilih salah satu: --buat, --skenario, atau --ringkas")

    # Pemeriksaan konfigurasi sama dengan navigasi.py.
    if args.skenario:
        ids = sorted({int(r) for sk in muat_skenario(args.skenario)["skenario"] for r in sk["robot"]})
    pesan = []
    for rid in ids:
        konfig.pilih_robot(rid)
        for p in konfig.periksa():
            if p not in pesan:
                pesan.append(p)
    konfig.pilih_robot(ids[0])
    fatal = False
    for tingkat, teks in pesan:
        print(f"[{tingkat}] {teks}")
        fatal |= tingkat == "x"
    if fatal and not args.simulasi:
        raise SystemExit("[!] Perbaiki dulu yang bertanda [x].")

    if args.buat:
        mode_buat(args, ids)
    else:
        mode_uji(args, ids)


if __name__ == "__main__":
    main()
