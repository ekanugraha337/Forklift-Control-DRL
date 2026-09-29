"""
state_space.py - model ruang-keadaan forklift (ID 01 dan ID 02).

Bentuk persamaannya sama untuk kedua robot; yang berbeda hanya parameter di
konfig.ROBOT. Pilih robot dengan --id (bawaan: FORKLIFT_ID, atau 1).

Dua bentuk, untuk dua keperluan berbeda:

  1. MODEL LENGKAP (tak linear) - untuk simulasi & prediksi
        keadaan  xi = [x, y, theta, delta, v]
        masukan  u  = [delta_cmd, v_cmd]

  2. MODEL GALAT JALUR (linear) - untuk perancangan kendali (LQR/MPC)
        keadaan  x = [e_y, e_theta, delta]
        masukan  u = delta_cmd
        gangguan w = kappa (kelengkungan jalur acuan)

Bentuk kedua itulah yang dipakai kalau nanti pelacak MPC dipasang lewat seam di
jejak.py. Pure pursuit tidak memerlukannya - ia geometris, bukan berbasis model.

KENAPA ACUANNYA POROS RODA DEPAN. Robot ini berkemudi BELAKANG. Roda depan tidak
berbelok, jadi kecepatan di poros depan selalu searah heading badan - itulah yang
membuat model sepeda baku berlaku tanpa suku tambahan. Kalau acuannya dipindah ke
poros belakang, persamaannya jadi punya suku menyamping dan tidak lagi sederhana.

TANDA SUDUT KEMUDI. delta di sini adalah sudut kemudi LOGIS, konvensi kendaraan
berkemudi depan. Pembalikan untuk roda belakang yang sebenarnya dikerjakan
firmware lewat setelan str_inv, jadi model ini tidak perlu mengurusnya - dan
itu memang harus begitu, supaya modelnya cocok dengan apa yang dikirim robot.py.

Jalankan berkas ini untuk mencetak matriks, memeriksa keterkendalian, dan
memvalidasi model linear terhadap simulator tak-linear:
    python state_space.py
    python state_space.py --laju 20        (pada 20 cm/detik)
"""
import argparse
import math

import numpy as np
from scipy.linalg import expm

import konfig


# ===========================================================================
# 1. MODEL LENGKAP (tak linear)
# ===========================================================================
def turunan(xi, u, L=None, tau=None, accel_sps2=None):
    """d(xi)/dt untuk xi = [x, y, theta, delta, v], u = [delta_cmd, v_cmd].

    Satuan: x,y cm | theta,delta radian | v cm/detik | delta_cmd radian |
    v_cmd cm/detik.

    Tunda murni pada kemudi TIDAK ada di sini - ia bukan keadaan berdimensi
    hingga. Pemanggil yang mengurusnya (lihat Model.diskret, yang menambahkan
    rantai tunda sebagai keadaan tambahan).
    """
    L = konfig.WHEELBASE_CM if L is None else L
    tau = konfig.SIM_KEMUDI_TAU_S if tau is None else tau
    a_sps = konfig.SIM_ACCEL_SPS2 if accel_sps2 is None else accel_sps2
    a = a_sps * konfig.CM_PER_STEP          # cm/detik^2

    x, y, th, d, v = xi
    dcmd, vcmd = u
    dmax = konfig.MAX_STEER_RAD
    d = float(np.clip(d, -dmax, dmax))

    return np.array([
        v * math.cos(th),
        v * math.sin(th),
        v * math.tan(d) / L,
        (float(np.clip(dcmd, -dmax, dmax)) - d) / tau,
        float(np.clip(vcmd - v, -a, a)) if abs(vcmd - v) > 1e-9 else 0.0,
    ])


# ===========================================================================
# 2. MODEL GALAT JALUR (linear)
# ===========================================================================
class Model:
    """Model galat jalur yang dilinearkan pada kecepatan tetap v.

        e_y'     = v * sin(e_theta)        ~ v * e_theta
        e_theta' = v * tan(delta) / L - kappa * v   ~ (v/L) * delta - v * kappa
        delta'   = (delta_cmd - delta) / tau

    Linearisasinya sah di sekitar e_theta kecil dan delta kecil. Pada sudut
    kemudi penuh 35 derajat, tan(delta) = 0,700 sedangkan delta = 0,611 radian -
    meleset 15%. Jadi untuk manuver rapat, pakai model tak linear di atas atau
    linearkan ulang di sekitar delta kerja (delta_ref).
    """

    def __init__(self, v_cm_s, tau=None, tunda_s=None, L=None, dt=None):
        self.v = float(v_cm_s)
        self.L = konfig.WHEELBASE_CM if L is None else L
        self.tau = konfig.SIM_KEMUDI_TAU_S if tau is None else tau
        self.tunda = konfig.SIM_KEMUDI_TUNDA_S if tunda_s is None else tunda_s
        self.dt = konfig.RC_PERIODE_S if dt is None else dt

        # --- kontinu ---
        self.A = np.array([[0.0, self.v, 0.0],
                           [0.0, 0.0, self.v / self.L],
                           [0.0, 0.0, -1.0 / self.tau]])
        self.B = np.array([[0.0], [0.0], [1.0 / self.tau]])
        # gangguan kelengkungan jalur acuan
        self.E = np.array([[0.0], [-self.v], [0.0]])
        # yang benar-benar diukur kamera: simpangan lateral & galat heading
        self.C = np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0]])

        self.n_tunda = int(round(self.tunda / self.dt))
        self._diskretkan()

    def _diskretkan(self):
        """Diskretisasi tepat (zero-order hold) + rantai tunda masukan.

        Tunda murni diwakili rantai penggeser: masukan hari ini baru sampai ke
        kemudi setelah n_tunda cacah. Cara ini tepat untuk tunda kelipatan dt,
        berbeda dengan pendekatan Pade yang menambah kutub palsu.
        """
        n = self.A.shape[0]
        # [[A, B],[0,0]] -> exp -> blok kanan atas = integral ZOH
        M = np.zeros((n + 1, n + 1))
        M[:n, :n] = self.A
        M[:n, n:] = self.B
        Md = expm(M * self.dt)
        Ad, Bd = Md[:n, :n], Md[:n, n:]

        Me = np.zeros((n + 1, n + 1))
        Me[:n, :n] = self.A
        Me[:n, n:] = self.E
        Ed = expm(Me * self.dt)[:n, n:]

        self.Ad, self.Bd, self.Ed = Ad, Bd, Ed

        k = self.n_tunda
        if k == 0:
            self.Ad_aug, self.Bd_aug, self.Ed_aug = Ad, Bd, Ed
            return
        N = n + k
        Aa = np.zeros((N, N))
        Ba = np.zeros((N, 1))
        Ea = np.zeros((N, 1))
        Aa[:n, :n] = Ad
        Aa[:n, n] = Bd[:, 0]          # tap tunda tertua masuk ke dinamika
        for i in range(k - 1):
            Aa[n + i, n + i + 1] = 1.0  # geser rantai
        Ba[N - 1, 0] = 1.0              # masukan baru masuk di ujung rantai
        Ea[:n, 0] = Ed[:, 0]
        self.Ad_aug, self.Bd_aug, self.Ed_aug = Aa, Ba, Ea

    # ------------------------------------------------------------- analisis
    def keterkendalian(self, pakai_tunda=False):
        A = self.Ad_aug if pakai_tunda else self.Ad
        B = self.Bd_aug if pakai_tunda else self.Bd
        n = A.shape[0]
        M = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])
        return np.linalg.matrix_rank(M), n

    def kutub(self, pakai_tunda=False):
        A = self.Ad_aug if pakai_tunda else self.Ad
        return np.linalg.eigvals(A)

    def langkah(self, x, u, kappa=0.0, pakai_tunda=True):
        A = self.Ad_aug if pakai_tunda else self.Ad
        B = self.Bd_aug if pakai_tunda else self.Bd
        E = self.Ed_aug if pakai_tunda else self.Ed
        return A @ x + B.ravel() * float(u) + E.ravel() * float(kappa)

    def cetak(self):
        print(f"=== MODEL GALAT JALUR pada v = {self.v:.1f} cm/detik ===")
        print(f"  keadaan  x = [e_y (cm), e_theta (rad), delta (rad)]")
        print(f"  masukan  u = delta_cmd (rad)   gangguan w = kappa (1/cm)")
        print(f"  L = {self.L:.2f} cm | tau = {self.tau:.3f} s | "
              f"tunda = {self.tunda:.3f} s = {self.n_tunda} cacah | dt = {self.dt:.3f} s")
        np.set_printoptions(precision=4, suppress=True)
        print("  A (kontinu) =\n", self.A)
        print("  B (kontinu) =\n", self.B.ravel())
        print("  E (kontinu) =\n", self.E.ravel())
        print("  Ad (diskret) =\n", self.Ad)
        print("  Bd (diskret) =\n", self.Bd.ravel())
        r, n = self.keterkendalian()
        print(f"  keterkendalian tanpa tunda : rank {r}/{n} "
              f"-> {'TERKENDALI' if r == n else 'TIDAK terkendali'}")
        r, n = self.keterkendalian(pakai_tunda=True)
        print(f"  keterkendalian dengan tunda: rank {r}/{n} "
              f"-> {'TERKENDALI' if r == n else 'TIDAK terkendali'}")
        print(f"  kutub diskret (tanpa tunda): {np.round(self.kutub(), 4)}")
        if self.v > 0:
            print(f"  CATATAN: dua kutub di 1,0 adalah integrator ganda "
                  f"posisi<-heading. Sistem ini TIDAK stabil sendiri - "
                  f"tanpa kendali, galat kecil tumbuh linear lalu kuadratik.")


# ===========================================================================
# VALIDASI terhadap simulator tak-linear
# ===========================================================================
def validasi(v_cm_s=None, detik=6.0, amplitudo_deg=8.0, tampil=True):
    """Bandingkan ramalan model linear dengan simulator penuh pose.py.

    Keduanya diberi urutan perintah kemudi yang sama pada lintasan acuan LURUS
    (kappa = 0), lalu simpangan lateralnya dibandingkan. Kalau modelnya benar,
    keduanya harus berimpit selama sudutnya masih kecil.
    """
    import pose as P

    v = konfig.FORWARD_SPEED * konfig.CM_PER_STEP if v_cm_s is None else v_cm_s
    sps = v / konfig.CM_PER_STEP
    dt = konfig.RC_PERIODE_S
    n = int(detik / dt)

    # Simulator penuh: mulai lurus di sumbu x, acuan = garis y = 0
    sim = P.SumberSimulasi(0.0, 0.0, 0.0, dt_tetap=dt)
    sim.perintah(sps, 0.0)
    for _ in range(int(2.0 / dt)):        # biarkan gas mencapai penuh
        sim.langkah()
    sim.x, sim.y, sim.yaw = 0.0, 0.0, 0.0

    model = Model(v)
    x = np.zeros(3 + model.n_tunda)

    rng = np.random.default_rng(1)
    perintah = []
    sudut = 0.0
    for i in range(n):
        if i % int(1.0 / dt) == 0:
            sudut = float(rng.uniform(-amplitudo_deg, amplitudo_deg))
        perintah.append(sudut)

    ey_sim, ey_mod = [], []
    for i in range(n):
        u_deg = perintah[i]
        sim.perintah(sps, u_deg)
        sim.langkah()
        ey_sim.append(sim.y)                       # acuan y = 0 -> e_y = y
        x = model.langkah(x, math.radians(u_deg), kappa=0.0)
        ey_mod.append(x[0])

    ey_sim = np.array(ey_sim)
    ey_mod = np.array(ey_mod)
    galat = ey_mod - ey_sim
    rms = float(np.sqrt(np.mean(galat ** 2)))
    lingkup = float(np.max(np.abs(ey_sim))) or 1.0

    if tampil:
        print(f"=== VALIDASI model linear vs simulator tak-linear ===")
        print(f"  v = {v:.1f} cm/detik, {detik:.0f} detik, kemudi acak +-{amplitudo_deg:.0f} deg")
        print(f"  simpangan simulator : {ey_sim.min():7.2f} .. {ey_sim.max():7.2f} cm")
        print(f"  simpangan model     : {ey_mod.min():7.2f} .. {ey_mod.max():7.2f} cm")
        print(f"  galat ramalan RMS   : {rms:.3f} cm  ({100.0 * rms / lingkup:.1f}% dari lingkup)")
    return rms, lingkup


def main():
    ap = argparse.ArgumentParser(description="Model ruang-keadaan forklift")
    ap.add_argument("--laju", type=float, default=None,
                    help="kecepatan kerja (cm/detik); bawaan = FORWARD_SPEED")
    ap.add_argument("--tanpa-validasi", action="store_true")
    ap.add_argument("--id", type=int, default=konfig.ROBOT_ID, help="robot, lihat konfig.ROBOT")
    args = ap.parse_args()
    konfig.pilih_robot(args.id)
    print(f"=== ROBOT ID {konfig.ROBOT_ID:02d}: CM_PER_STEP {konfig.CM_PER_STEP:.6f}, "
          f"accel {konfig.ACCEL_SPS2:.0f} sps/s^2 = {konfig.ACCEL_SPS2 * konfig.CM_PER_STEP:.2f} "
          f"cm/s^2 ===")

    v = konfig.FORWARD_SPEED * konfig.CM_PER_STEP if args.laju is None else args.laju
    Model(v).cetak()
    print()
    if not args.tanpa_validasi:
        validasi(v)
        print()
        print("  Galat ramalan tumbuh bersama sudut kemudi, karena linearisasi")
        print("  mengganti tan(delta) dengan delta. Bandingkan:")
        for amp in (4.0, 10.0, 20.0, 30.0):
            rms, lingkup = validasi(v, amplitudo_deg=amp, tampil=False)
            print(f"    kemudi +-{amp:4.0f} deg -> galat RMS {rms:5.2f} cm "
                  f"({100.0 * rms / lingkup:4.1f}% dari lingkup)")


if __name__ == "__main__":
    main()
