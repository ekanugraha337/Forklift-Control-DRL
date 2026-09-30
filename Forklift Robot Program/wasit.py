"""
wasit.py - pencegah tabrakan antar-robot.

Rancangannya ada di "Multi Robot Navigation and Control.md" (Obsidian).

Perencana hanya mengenal zona peta. Robot lain BUKAN zona: ia bergerak, dan
navigasi kedua robot sengaja dibiarkan bebas saling memotong jalur. Yang
mencegah tabrakan adalah wasit ini. navigasi.py memanggilnya sekali per siklus
kendali (20 Hz), SETELAH setiap robot menghitung perintahnya dan SEBELUM
perintah itu dikirim, lalu robot yang diputuskan harus menunggu mengirim nol.

BAYANGAN. Robot yang sedang berjalan membawa bayangan: jejak badannya di
sepanjang jalur rencana ke depan, sejauh jarak yang PASTI ia tempuh sebelum
benar-benar diam (lihat konfig bagian MULTI-ROBOT):

    L = MULTI_LIHAT_MIN_CM + v * MULTI_T_REAKSI_S + v^2 / (2 * a_rem)

Bayangan mengikuti JALUR, bukan kotak lurus di depan: robot berkemudi
belakang, buntutnya mengayun keluar saat berbelok, dan jalur bisa berbalik
arah di titik ganti arah.

ATURAN, untuk setiap pasangan robot A dan B:

    kontak   badan A dan badan B bersinggungan -> keduanya estop.
    A->B     bayangan A MENDEKAT ke badan B sampai kurang dari
             MULTI_JARAK_AMAN_CM -> A menunggu. Prioritas tidak berlaku di
             sini: pemenang prioritas pun tidak boleh menabrak badan yang ada
             di depannya.
    B->A     sebaliknya. Bila A->B dan B->A sekaligus (berhadapan), keduanya
             menunggu, dan yang kalah prioritas lebih dulu menghitung ulang
             jalur memutar (MULTI_TUNGGU_KALAH_S).
    silang   hanya bayangan dengan bayangan yang bertemu: jalur akan
             berpotongan, tapi belum ada yang menuju badan lawan -> yang KALAH
             prioritas menunggu.

"Mendekat" berarti jarak pada bayangan lebih kecil daripada jarak badan
sekarang. Tanpa syarat ini dua robot yang berdiri berdampingan tidak akan bisa
berpisah: bayangan robot yang sedang MENJAUH pun masih dekat ke badan
tetangganya, dan ia akan menunggu selamanya.

BAYANGAN MENGIKUTI BADAN. Pose jalur adalah tempat robot SEHARUSNYA berada,
bukan tempat ia berada. Sebelum dibandingkan, bayangan dipindahkan dengan
selisih pose badan sekarang terhadap titik jalur terdekat (geser + putar),
jadi yang diuji adalah "robot meneruskan jalurnya dengan simpangan yang
sedang ia punya". Ditemukan 2026-09-30 (catatan Obsidian "Deep Reinforcement
Learning"): robot pemenang prioritas berjalan 3,5 cm di samping jalurnya dan
menyerempet robot yang sedang menunggu sampai 1,7 cm - bayangan di atas
jalurnya sendiri melihat 5,7 cm, badannya 1,6 cm. Terjadi dengan A* maupun
DQfD.

PRIORITAS
    0. robot yang sedang MINGGIR didahulukan: ia sedang membuka jalan untuk
       lawannya, dan menghentikannya di tengah manuver justru meninggalkan
       badannya di jalur yang hendak dibuka (terukur di simulasi: lawannya
       ikut minggir, dan keduanya berputar-putar)
    1. robot yang tidak berjalan tidak bisa mengalah - yang berjalan kalah
    2. robot yang sedang MUNDUR didahulukan: mundur selalu manuver rapat, dan
       berhenti di tengahnya meninggalkan badan melintang di jalur
    3. yang sisa jalurnya lebih pendek didahulukan - cepat keluar dari ruang
       bersama
    4. ID lebih kecil didahulukan
Robot yang terhalang oleh robot yang sedang minggir menunggu tanpa batas waktu
(tidak menghitung ulang jalur): jalannya sedang dibukakan.

Keputusan prioritas DIKUNCI selama konflik pasangan itu berlangsung. Sisa jalur
berubah terus saat robot bergerak; tanpa kunci, pemenang bisa bertukar di
tengah konflik dan kedua robot bergantian berhenti-jalan.

Wasit TIDAK tahu apa-apa soal MQTT, kamera, atau pelacak. Ia hanya menerima
Peserta (poligon dalam cm) dan mengembalikan Keputusan. Karena itu ia bisa diuji
tanpa robot sama sekali - lihat bagian mandiri di bawah.
"""
import math
from dataclasses import dataclass, field

import numpy as np

import arena as A
import konfig

TUNGGU, KONTAK = "tunggu", "kontak"


@dataclass
class Peserta:
    """Keadaan satu robot yang perlu diketahui wasit, semua dalam cm."""
    id: int
    badan: np.ndarray = None       # 4x2, jejak badan sekarang; None = belum pernah terlihat
    terlihat: bool = True          # pose masih layak dipakai (tag tidak hilang)
    berjalan: bool = False         # punya jalur aktif (MELACAK / MENUNGGU)
    bayangan: list = field(default_factory=list)   # poligon 4x2 di jalur DEPAN
    arah: int = 0                  # +1 maju, -1 mundur, 0 diam
    sisa_cm: float = 0.0           # sisa panjang jalur sampai tujuan
    minggir: bool = False          # sedang menepi untuk memberi jalan robot lain


@dataclass
class Keputusan:
    jenis: str                     # TUNGGU atau KONTAK
    alasan: str
    lawan: int
    # Menunggu lebih lama dari ini -> hitung ulang jalur memutari robot lain.
    # inf = jangan pernah (misalnya posisi lawan tidak diketahui).
    tunggu_maks: float = konfig.MULTI_TUNGGU_MAKS_S


# ------------------------------------------------------------------ geometri
def panjang_bayangan(v_cm_s, a_rem_cm_s2):
    """Jarak (cm) yang ditempuh robot pada laju v sebelum benar-benar diam."""
    v = abs(v_cm_s)
    return (konfig.MULTI_LIHAT_MIN_CM + v * konfig.MULTI_T_REAKSI_S
            + v * v / (2.0 * a_rem_cm_s2))


def sapuan(titik_cm, panjang_cm):
    """Jejak badan di sepanjang jalur depan, sampai sejauh panjang_cm.

    titik_cm: [(x_cm, y_cm, yaw_deg), ...] mulai dari titik jalur terdekat
    robot. Titik jalur perencana berjarak ~2,4 cm (sample_ds), jauh lebih rapat
    dari panjang badan 27 cm, jadi tidak perlu disisipi.
    """
    poli = []
    tempuh = 0.0
    lalu = None
    for x, y, yaw in titik_cm:
        if lalu is not None:
            tempuh += math.hypot(x - lalu[0], y - lalu[1])
        poli.append(A.jejak_badan(x, y, yaw))
        if tempuh >= panjang_cm:
            break
        lalu = (x, y)
    return poli


def ikut_badan(bayangan, badan):
    """Bayangan dipindahkan (geser + putar) supaya poligon pertamanya jatuh tepat
    di badan sekarang.

    bayangan[0] adalah jejak badan di titik jalur terdekat robot (sapuan()
    dimulai dari sana), dan poligon jejak_badan() selalu bersudut dalam urutan
    yang sama, jadi transformasi kaku di antara keduanya adalah simpangan robot
    dari jalurnya - lateral maupun heading. Robot yang tepat di jalurnya:
    transformasinya identitas, bayangan tidak berubah.
    """
    if not bayangan or badan is None:
        return bayangan
    P0 = np.asarray(bayangan[0], float)
    B = np.asarray(badan, float)
    c0, cb = P0.mean(axis=0), B.mean(axis=0)
    e0, eb = P0[1] - P0[0], B[1] - B[0]
    dth = math.atan2(eb[1], eb[0]) - math.atan2(e0[1], e0[0])
    c, s = math.cos(dth), math.sin(dth)
    R = np.array([[c, -s], [s, c]])
    return [(np.asarray(P, float) - c0) @ R.T + cb for P in bayangan]


def _berpotongan(P, Q):
    """Uji sumbu pemisah untuk dua poligon cembung."""
    for R in (P, Q):
        e = np.roll(R, -1, axis=0) - R
        for n in np.stack([-e[:, 1], e[:, 0]], axis=1):
            a, b = P @ n, Q @ n
            if a.max() < b.min() or b.max() < a.min():
                return False
    return True


def _titik_ke_tepi(T, P):
    """Jarak terkecil dari titik-titik T ke tepi poligon P."""
    a = P
    ab = np.roll(P, -1, axis=0) - P
    at = T[:, None, :] - a[None, :, :]
    t = np.clip((at * ab[None]).sum(-1) / (ab * ab).sum(-1)[None], 0.0, 1.0)
    proyeksi = a[None] + t[..., None] * ab[None]
    return float(np.linalg.norm(T[:, None, :] - proyeksi, axis=-1).min())


def jarak_poligon(P, Q):
    """Jarak (cm) antara dua poligon cembung; 0 bila bersinggungan."""
    P, Q = np.asarray(P, float), np.asarray(Q, float)
    if _berpotongan(P, Q):
        return 0.0
    return min(_titik_ke_tepi(P, Q), _titik_ke_tepi(Q, P))


def jarak_himpunan(Ps, Qs, batas=float("inf")):
    """Jarak terkecil antara dua himpunan poligon.

    Pasangan yang lingkaran luarnya saja sudah lebih jauh dari batas dilewati.
    Hasil inf berarti "tidak ada yang lebih dekat dari batas", bukan jarak
    sebenarnya - pemanggil memang hanya bertanya apakah ada yang di bawah batas.
    """
    def lingkaran(P):
        c = P.mean(axis=0)
        return c, float(np.linalg.norm(P - c, axis=1).max())

    terbaik = float("inf")
    lq = [(lingkaran(Q), Q) for Q in Qs]
    for P in Ps:
        cp, rp = lingkaran(P)
        for (cq, rq), Q in lq:
            if np.linalg.norm(cp - cq) - rp - rq >= min(terbaik, batas):
                continue
            terbaik = min(terbaik, jarak_poligon(P, Q))
    return terbaik


# --------------------------------------------------------------------- wasit
class Wasit:
    def __init__(self):
        # (id_kecil, id_besar) -> id yang KALAH, selama konflik pasangan itu ada
        self._kalah = {}
        # Konflik siklus terakhir, untuk digambar: [(id_a, id_b, jenis_teks)]
        self.konflik = []

    def putuskan(self, peserta):
        """{id: Keputusan} untuk robot yang harus menunggu atau estop."""
        hasil = {}
        self.konflik = []
        ada = [p for p in peserta if p.badan is not None]
        for i, a in enumerate(ada):
            for b in ada[i + 1:]:
                self._pasangan(a, b, hasil)
        return hasil

    # ----------------------------------------------------------- bantuan
    @staticmethod
    def _setel(hasil, rid, kep):
        """Simpan keputusan; kontak mengalahkan tunggu, tunggu tersingkat menang."""
        lama = hasil.get(rid)
        if lama is None or (kep.jenis == KONTAK and lama.jenis != KONTAK) or \
                (kep.jenis == lama.jenis == TUNGGU and kep.tunggu_maks < lama.tunggu_maks):
            hasil[rid] = kep

    def _pilih_kalah(self, a, b, kunci):
        if a.minggir != b.minggir:
            # Mengalahkan kunci yang lama: kunci itu dibuat sebelum robot ini
            # mulai minggir, dan minggir-nya justru hasil dari kekalahan itu.
            kalah = b if a.minggir else a
            self._kalah[kunci] = kalah.id
            return kalah
        if kunci in self._kalah:
            return a if a.id == self._kalah[kunci] else b
        if a.berjalan != b.berjalan:
            kalah = a if a.berjalan else b
        elif (a.arah < 0) != (b.arah < 0):
            kalah = b if a.arah < 0 else a
        elif abs(a.sisa_cm - b.sisa_cm) > konfig.MULTI_SISA_SETARA_CM:
            kalah = a if a.sisa_cm > b.sisa_cm else b
        else:
            kalah = a if a.id > b.id else b
        self._kalah[kunci] = kalah.id
        return kalah

    def _pasangan(self, a, b, hasil):
        kunci = (min(a.id, b.id), max(a.id, b.id))

        # Posisi lawan tidak diketahui: jangan menebak bahwa jalannya bebas.
        # Robot yang tag-nya hilang juga sedang di-estop oleh pemantau
        # keselamatannya sendiri, jadi menunggu di sini tidak akan lama.
        if not (a.terlihat and b.terlihat):
            for p, q in ((a, b), (b, a)):
                if p.berjalan and not q.terlihat:
                    self._setel(hasil, p.id, Keputusan(
                        TUNGGU, f"posisi robot {q.id:02d} tidak diketahui (tag hilang)",
                        q.id, float("inf")))
            self._kalah.pop(kunci, None)
            return

        d_kini = jarak_poligon(a.badan, b.badan)
        if d_kini <= 0.0:
            for p, q in ((a, b), (b, a)):
                self._setel(hasil, p.id, Keputusan(
                    KONTAK, f"badan bersentuhan dengan robot {q.id:02d} - pisahkan dulu", q.id))
            self.konflik.append((a.id, b.id, "kontak"))
            return

        if not (a.berjalan or b.berjalan):
            self._kalah.pop(kunci, None)
            return

        # Batas "mendekat": di bawah jarak aman DAN lebih dekat dari sekarang.
        # Kelonggaran 0,5 cm menyerap derau pose supaya robot yang menjauh
        # sejajar tidak terbaca mendekat oleh getaran pengukuran.
        batas = min(konfig.MULTI_JARAK_AMAN_CM, d_kini - min(0.5, d_kini / 2.0))
        # Bayangan dipindah ke badan sebenarnya - lihat docstring modul.
        bay_a, bay_b = ikut_badan(a.bayangan, a.badan), ikut_badan(b.bayangan, b.badan)
        ab = a.berjalan and jarak_himpunan(bay_a, [b.badan], batas) < batas
        ba = b.berjalan and jarak_himpunan(bay_b, [a.badan], batas) < batas

        def batas_tunggu(p, q, bawaan):
            # q sedang minggir untuk p: jalannya sedang dibukakan, jangan
            # memutar sendiri - itu hanya membuat dua robot sama-sama menepi.
            return float("inf") if q.minggir and not p.minggir else bawaan

        if ab and ba:
            kalah = self._pilih_kalah(a, b, kunci)
            for p, q in ((a, b), (b, a)):
                self._setel(hasil, p.id, Keputusan(
                    TUNGGU, f"berhadapan dengan robot {q.id:02d}", q.id,
                    batas_tunggu(p, q, konfig.MULTI_TUNGGU_KALAH_S if p is kalah
                                 else konfig.MULTI_TUNGGU_MAKS_S)))
            self.konflik.append((a.id, b.id, "berhadapan"))
        elif ab or ba:
            p, q = (a, b) if ab else (b, a)
            self._setel(hasil, p.id, Keputusan(
                TUNGGU, f"robot {q.id:02d} ada di jalur", q.id,
                batas_tunggu(p, q, konfig.MULTI_TUNGGU_MAKS_S)))
            self.konflik.append((p.id, q.id, "di jalur"))
        elif a.berjalan and b.berjalan and \
                jarak_himpunan(bay_a, bay_b, batas) < batas:
            kalah = self._pilih_kalah(a, b, kunci)
            menang = b if kalah is a else a
            self._setel(hasil, kalah.id, Keputusan(
                TUNGGU, f"memberi jalan ke robot {menang.id:02d}", menang.id,
                batas_tunggu(kalah, menang, konfig.MULTI_TUNGGU_MAKS_S)))
            self.konflik.append((kalah.id, menang.id, "silang"))
        else:
            self._kalah.pop(kunci, None)


# ------------------------------------------------------------------ mandiri
def _uji():
    """Uji aturan wasit dengan skenario buatan. Tanpa kamera, tanpa robot."""
    v = konfig.FORWARD_SPEED * konfig.CM_PER_STEP
    a_rem = konfig.ACCEL_SPS2 * konfig.CM_PER_STEP
    L = panjang_bayangan(v, a_rem)

    def lurus(x, y, yaw, panjang, arah=1):
        """Titik jalur lurus sepanjang panjang cm, jarak 2,4 cm."""
        r = math.radians(yaw)
        n = int(panjang / 2.4) + 1
        return [(x + arah * i * 2.4 * math.cos(r), y + arah * i * 2.4 * math.sin(r), yaw)
                for i in range(n)]

    def peserta(rid, x, y, yaw, berjalan=True, arah=1, sisa=100.0, geser=0.0):
        """geser: badan menyimpang sekian cm ke KIRI jalurnya (galat pelacakan)."""
        bay = sapuan(lurus(x, y, yaw, 100, arah), L) if berjalan else []
        r = math.radians(yaw)
        badan = A.jejak_badan(x - geser * math.sin(r), y + geser * math.cos(r), yaw)
        return Peserta(rid, badan, True, berjalan, bay, arah, sisa)

    # Jarak di kasus-kasus ini diturunkan dari L, supaya ujiannya tetap
    # bermakna kalau kecepatan atau akselerasi di konfig berubah.
    F, B = konfig.BODY_FRONT_CM, konfig.BODY_REAR_CM
    kasus = [
        ("beriring: 02 diam, celah < L",
         [peserta(1, 0, 0, 0), peserta(2, F + (L - 2) + B, 0, 0, berjalan=False)],
         {1: TUNGGU}),
        ("beriring: 02 diam, celah 2L",
         [peserta(1, 0, 0, 0), peserta(2, F + 2 * L + B, 0, 0, berjalan=False)],
         {}),
        # Ujung depan terpisah L: bayangan masing-masing sampai ke badan lawan
        # -> keduanya menunggu.
        ("berhadapan dekat di koridor",
         [peserta(1, 0, 0, 0), peserta(2, 2 * F + L, 0, 180)],
         {1: TUNGGU, 2: TUNGGU}),
        # Terpisah 1,5 L: yang bertemu baru bayangan dengan bayangan -> silang.
        # Sisa jalur sama, jadi ID 02 yang mengalah.
        ("berhadapan jauh",
         [peserta(1, 0, 0, 0), peserta(2, 2 * F + 1.5 * L, 0, 180)],
         {2: TUNGGU}),
        ("bersilangan tegak lurus, 02 lebih dekat tujuan",
         [peserta(1, -(L + 2), 0, 0, sisa=150), peserta(2, 0, -(L + 2), 90, sisa=60)],
         {1: TUNGGU}),
        # 01 mundur ke arah -x (buntut di depan), 02 maju ke +y. Sisa jalur 01
        # lebih panjang, tapi yang mundur didahulukan.
        ("bersilangan, 01 sedang mundur",
         [peserta(1, L + B - 2, 0, 0, arah=-1, sisa=150), peserta(2, 0, -(L + 2), 90, sisa=60)],
         {2: TUNGGU}),
        ("berdampingan 5 cm, 01 maju sejajar",
         [peserta(1, 0, 0, 90),
          peserta(2, konfig.BODY_WIDTH_CM + 5, 0, 90, berjalan=False)],
         {}),
        ("berdampingan 5 cm, 01 membelok ke arah 02",
         [peserta(1, 0, 0, 30),
          peserta(2, konfig.BODY_WIDTH_CM + 5, 0, 90, berjalan=False)],
         {1: TUNGGU}),
        ("badan bersentuhan",
         [peserta(1, 0, 0, 0), peserta(2, 5, 3, 0, berjalan=False)],
         {1: KONTAK, 2: KONTAK}),
        # Kasus 2026-09-30: 01 berjalan 3,5 cm di kiri jalurnya, 02 diam di depan
        # kirinya. Menurut JALUR, 01 lewat 5,7 cm dari 02 (>= jarak aman, lolos);
        # menurut BADAN-nya ia lewat 2,2 cm. Harus menunggu.
        ("menyerempet: 01 menyimpang 3,5 cm ke arah 02 yang diam",
         [peserta(1, 0, 0, 0, geser=3.5),
          peserta(2, F + L / 2 + B, konfig.BODY_WIDTH_CM + 5.7, 0, berjalan=False)],
         {1: TUNGGU}),
        # Penjaga: simpangan yang sama tapi SEJAJAR 5 cm dari 02 - tidak mendekat,
        # jadi tetap boleh jalan (seperti "berdampingan 5 cm, 01 maju sejajar").
        ("menyimpang 3,5 cm, sejajar 5 cm dari 02",
         [peserta(1, 0, 0, 0, geser=3.5),
          peserta(2, 0, 3.5 + konfig.BODY_WIDTH_CM + 5, 0, berjalan=False)],
         {}),
    ]
    print(f"[i] laju {v:.1f} cm/detik, a_rem {a_rem:.1f} cm/detik^2 -> bayangan {L:.1f} cm")
    gagal = 0
    for nama, ps, harap in kasus:
        hasil = Wasit().putuskan(ps)
        dapat = {k: kep.jenis for k, kep in hasil.items()}
        ok = dapat == harap
        gagal += not ok
        print(f"  [{'OK' if ok else 'GAGAL'}] {nama}: {dapat or 'semua jalan'}"
              + ("" if ok else f"  (diharapkan {harap or 'semua jalan'})"))
        for kep in hasil.values():
            print(f"         - {kep.alasan}")
    print("[OK] semua kasus lolos." if not gagal else f"[!] {gagal} kasus gagal.")
    return gagal


if __name__ == "__main__":
    raise SystemExit(_uji())
