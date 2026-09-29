"""
jejak.py - pelacak jalur (path tracking). Titik tukar algoritma.

Saat ini hanya pure pursuit. Menambah pelacak lain (MPC, Stanley, LQR) cukup
dengan satu fungsi berdekorator @daftar("nama") di bawah - navigasi.py tidak
perlu disentuh sama sekali, karena ia hanya memanggil buat().

KONTRAK PELACAK
    tracker.update(gx, gy, yaw_rad, now) -> dict perintah
    tracker.done        : bool, True bila jalur sudah habis
    tracker.deviation   : float, simpangan dari jalur (SEL)

    dict perintah berisi minimal:
        speed     langkah/detik BERTANDA (+maju, -mundur)
        steer_deg sudut kemudi logis, derajat
        dir       +1 maju, -1 mundur, 0 berhenti
        seg       nomor segmen yang sedang dilacak; BERUBAH = titik ganti arah
        done      bool
        paused    bool
        target    (gx, gy) titik bidikan, boleh None - hanya untuk digambar

SATUAN. Pelacak bekerja dalam SEL grid, sama seperti perencana - karena jalur
yang dilacaknya memang keluaran perencana. Hanya kecepatan (langkah/detik) dan
sudut (derajat) yang bersatuan fisik.

Pelacak TIDAK menghitung sudut servo. Itu urusan robot.py, supaya pelacak baru
tidak perlu tahu bahwa titik lurus servo robot ini 89,45 derajat.
"""
import math

import konfig
from forklift_planner import PathTracker

PELACAK = {}


def daftar(nama):
    def bungkus(fn):
        PELACAK[nama] = fn
        return fn
    return bungkus


def _bersihkan(perintah):
    """Buang kolom 'servo' dari pustaka.

    PathTracker ikut menghitung sudut servo sendiri, tapi angka titik-lurus di
    sana adalah salinan kedua dari yang sudah tersimpan di NVS firmware. Supaya
    hanya ada SATU tempat yang menghitungnya (robot.steer_ke_servo), kolom itu
    dibuang di sini - bukan dibiarkan ikut mengalir dan berpeluang terpakai.
    """
    perintah.pop("servo", None)
    return perintah


class PurePursuit:
    """Pembungkus tipis PathTracker supaya memenuhi kontrak di atas."""

    def __init__(self, jalur, kendaraan, omap, arena, param):
        s = arena.panjang_ke_sel
        self.arena, self.omap, self.kendaraan = arena, omap, kendaraan
        self.sempit = 1.0            # faktor perlambatan celah terakhir
        self._sempit_mulai = s(konfig.SEMPIT_MULAI_CM)
        self._sempit_penuh = s(konfig.SEMPIT_PENUH_CM)
        self._sempit_lihat = s(konfig.SEMPIT_LIHAT_CM)
        self.inti = PathTracker(
            jalur, kendaraan, omap=omap,
            lookahead=s(param["lookahead"]),
            seg_end_tol=s(param["seg_end_tol"]),
            forward_speed=konfig.FORWARD_SPEED,
            reverse_speed=konfig.REVERSE_SPEED,
            slowdown_dist=s(param["slowdown_dist"]),
            min_speed_ratio=konfig.MIN_SPEED_RATIO,
            cusp_pause=konfig.CUSP_PAUSE_S,
            # Konversi servo dimatikan: center 0 dan gain 1 membuat kolom
            # "servo" identik dengan "steer_deg", dan kolomnya dibuang di
            # _bersihkan(). Batasnya dibuat lebar supaya tidak ikut memotong
            # sudut kemudi - pemotongan yang sah sudah dilakukan PathTracker
            # terhadap vehicle.max_steer.
            servo_center=0.0, servo_per_deg=1.0, servo_min=-180, servo_max=180,
            lag_comp=param["lag_comp"],
            curve_speed_ratio=konfig.CURVE_SPEED_RATIO,
            # Acuan kelengkungan = tikungan tertajam yang benar-benar
            # DIRENCANAKAN, bukan batas fisik kendaraan. Tanpa ini ratio tidak
            # pernah terpakai penuh karena tikungan rencana selalu jauh lebih
            # landai dari batas fisik.
            curve_k_ref=math.tan(kendaraan.max_steer * konfig.PLAN_STEER_RATIO) / kendaraan.L,
            final_turn_ratio=konfig.FINAL_TURN_RATIO,
            final_turn_dist=s(param["final_turn_dist"]),
        )

        # Jarak bebas tiap titik jalur dihitung SEKALI di sini. Jalurnya tetap
        # dan petanya statis, jadi menghitungnya ulang tiap frame hanya
        # membuang waktu di loop yang harus mengejar 20 Hz.
        self._margin_titik = [
            [omap.pose_margin(kendaraan, p[0], p[1], p[2]) for p in pts]
            for pts, _ in self.inti.segments
        ]

    @property
    def done(self):
        return self.inti.done

    @property
    def deviation(self):
        return self.inti.deviation

    def _margin_depan(self):
        """Jarak bebas terkecil pada potongan jalur di depan robot (sel)."""
        seg = self.inti.seg_i
        if seg >= len(self._margin_titik):
            return None
        m = self._margin_titik[seg]
        cum = self.inti.cum[seg]
        if not m:
            return None
        i0 = min(self.inti.idx, len(m) - 1)
        batas = cum[i0] + self._sempit_lihat
        i1 = i0
        while i1 + 1 < len(m) and cum[i1 + 1] <= batas:
            i1 += 1
        return min(m[i0:i1 + 1])

    def _faktor_sempit(self, gx, gy, yaw_rad):
        """Faktor kecepatan 0..1 berdasarkan kedekatan badan ke zona.

        Dua sumber digabung dan diambil yang terkecil:
          - jalur DI DEPAN, supaya robot mengerem sebelum masuk celah, bukan
            setelah telanjur berada di dalamnya;
          - pose SEBENARNYA, karena robot bisa menyimpang dari jalur dan jarak
            bebas yang nyata bisa lebih buruk daripada yang direncanakan.
        """
        m = self.omap.pose_margin(self.kendaraan, gx, gy, yaw_rad)
        depan = self._margin_depan()
        if depan is not None:
            m = min(m, depan)

        if m >= self._sempit_mulai:
            return 1.0
        if m <= self._sempit_penuh:
            return konfig.SEMPIT_RATIO
        t = (m - self._sempit_penuh) / (self._sempit_mulai - self._sempit_penuh)
        return konfig.SEMPIT_RATIO + t * (1.0 - konfig.SEMPIT_RATIO)

    def update(self, gx, gy, yaw_rad, now):
        perintah = _bersihkan(self.inti.update(gx, gy, yaw_rad, now))

        # Nomor segmen ikut dilaporkan supaya pemanggil bisa mengenali PERGANTIAN
        # segmen - yaitu titik ganti arah. Dilewatkan lewat dict perintah, bukan
        # dibaca langsung dari self.inti, agar pelacak lain (MPC, Stanley) cukup
        # mengisi kolom yang sama tanpa harus punya struktur internal serupa.
        perintah["seg"] = self.inti.seg_i

        self.sempit = self._faktor_sempit(gx, gy, yaw_rad)
        laju = perintah.get("speed", 0)
        if self.sempit < 1.0 and laju:
            laju = int(laju * self.sempit)
            # Jangan sampai tumpukan faktor perlambatan menembus RC_MIN_SPS
            # firmware (100) - di bawah itu motor justru BERHENTI, bukan merayap,
            # dan robot akan mogok di tengah celah sempit.
            if abs(laju) < konfig.SPS_MINIMUM:
                laju = int(math.copysign(konfig.SPS_MINIMUM, laju))
            perintah["speed"] = laju
        perintah["sempit"] = self.sempit
        return perintah


@daftar("pursuit")
def _buat_pursuit(jalur, kendaraan, omap, arena, param):
    return PurePursuit(jalur, kendaraan, omap, arena, param)


def buat(nama, jalur, kendaraan, omap, arena):
    """Bangun pelacak bernama <nama> untuk <jalur>."""
    if nama not in PELACAK:
        raise SystemExit(f"[!] Pelacak '{nama}' tidak dikenal. Yang ada: "
                         f"{', '.join(sorted(PELACAK))}")
    param = konfig.PARAM_PELACAK.get(nama)
    if param is None:
        raise SystemExit(f"[!] konfig.PARAM_PELACAK belum punya bagian '{nama}'.")
    return PELACAK[nama](jalur, kendaraan, omap, arena, param)


def tersedia():
    return sorted(PELACAK)
