"""
robot.py - sambungan MQTT ke firmware Forklift Control v2.

Ini SATU-SATUNYA berkas yang tahu soal servo dan MQTT. Pelacak mengeluarkan
besaran fisik (sudut kemudi derajat, kecepatan langkah/detik) dan berkas inilah
yang menerjemahkannya ke sudut servo. Dengan begitu pelacak lain - MPC, Stanley,
apa pun - tidak perlu tahu bahwa titik lurus servo robot ini 89,45 derajat.

Topik v2 (berbeda dari v1 yang memakai "forklift/#"):
    forklift01/in/rc         <- "<langkah/detik bertanda> <sudut servo>"
    forklift01/in/cmd        <- "estop", "center", "stop", ...
    forklift01/out/telemetry -> JSON 4x per detik
    forklift01/out/state     -> online / offline / ota

Uji mandiri (menggerakkan motor - pastikan robot aman):
    python robot.py --uji
"""
import argparse
import json
import math
import threading
import time

import paho.mqtt.client as mqtt

import konfig


def steer_ke_servo(steer_deg, profil=None):
    """Sudut kemudi logis (derajat) -> sudut servo yang dimengerti firmware.

    Pembalikan untuk roda belakang TIDAK dilakukan di sini: firmware yang
    mengurusnya lewat setelan str_inv, di satu tempat, supaya semua jalur
    kendali ikut benar.

    profil: konfig.profil() robot tujuan; None = robot yang sedang dipilih.
    """
    p = profil or konfig.profil()
    steer_deg = max(-p.max_steer_deg, min(p.max_steer_deg, steer_deg))
    servo = p.steer_center + steer_deg * konfig.STEER_SERVO_PER_DEG
    return max(konfig.STEER_SERVO_MIN, min(konfig.STEER_SERVO_MAX, servo))


class Robot:
    """Sambungan ke firmware. Aman dipakai walau broker sedang putus.

    profil: konfig.profil() robot ini. Wajib diisi bila satu proses memegang
    lebih dari satu robot - tanpa itu topik dan str_center diambil dari global
    konfig, yaitu milik robot yang terakhir dipilih.
    """

    def __init__(self, host=None, port=None, user=None, sandi=None, simulasi=None,
                 profil=None):
        self.p = profil or konfig.profil()
        self.host = host or konfig.MQTT_HOST
        self.port = port or konfig.MQTT_PORT
        self.simulasi = simulasi          # objek SumberSimulasi, atau None

        self.tersambung = False
        self.state = None                 # online / offline / ota
        self.telemetri = {}
        self.waktu_telemetri = 0.0
        self._kunci = threading.Lock()

        self._rc_terakhir = None
        self._waktu_rc = 0.0
        self._stall_awal = None
        self.stall_baru = False           # True sekali saat hitungan stall naik
        # Riwayat telemetri, hanya selama merekam (identifikasi uji gas). Di
        # luar itu tetap None: navigasi berjalan berjam-jam dan tidak butuh
        # riwayat, jadi tidak ada gunanya menumpuk 4 pesan per detik.
        self._rekam_telemetri = None

        self.cli = mqtt.Client()
        if (user or konfig.MQTT_USER):
            self.cli.username_pw_set(user or konfig.MQTT_USER, sandi or konfig.MQTT_PASS)
        self.cli.on_connect = self._on_connect
        self.cli.on_disconnect = self._on_disconnect
        self.cli.on_message = self._on_message
        # Tanpa ini, putusnya broker lolos diam-diam dan program mengira
        # perintahnya masih sampai - pelajaran dari v1.
        self.cli.reconnect_delay_set(min_delay=1, max_delay=5)

    # ---------------------------------------------------------------- MQTT
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        self.tersambung = (rc == 0)
        if rc == 0:
            client.subscribe([(self.p.topik_telemetri, 0), (self.p.topik_state, 0)])
            print(f"[MQTT] {self.p.prefix} tersambung ke {self.host}:{self.port}")
        else:
            print(f"[MQTT] {self.p.prefix} ditolak broker (rc={rc})")

    def _on_disconnect(self, client, userdata, rc, properties=None, reason=None):
        self.tersambung = False
        print(f"[MQTT] {self.p.prefix} TERPUTUS - robot akan berhenti sendiri setelah "
              f"{konfig.RC_TIMEOUT_S:.1f} detik tanpa perintah.")

    def _on_message(self, client, userdata, msg):
        try:
            teks = msg.payload.decode("utf-8", "ignore").strip()
        except Exception:
            return
        if msg.topic == self.p.topik_state:
            self.state = teks
            return
        if msg.topic != self.p.topik_telemetri:
            return
        try:
            data = json.loads(teks)
        except json.JSONDecodeError:
            return
        with self._kunci:
            self.telemetri = data
            self.waktu_telemetri = time.time()
            if self._rekam_telemetri is not None:
                self._rekam_telemetri.append((self.waktu_telemetri, data))
            # Firmware v2 melaporkan stall StallGuard. Stall berarti posisi
            # perintah tidak lagi mewakili posisi roda, jadi jalur harus
            # dihitung ulang dari pose AprilTag yang segar.
            stall = data.get("stall")
            if stall is not None:
                if self._stall_awal is None:
                    self._stall_awal = stall
                elif stall > self._stall_awal:
                    self._stall_awal = stall
                    self.stall_baru = True

    def sambung(self, timeout=5.0):
        if self.simulasi is not None:
            return True
        try:
            self.cli.connect(self.host, self.port, keepalive=15)
        except OSError as e:
            print(f"[MQTT] gagal menyambung ke {self.host}:{self.port}: {e}")
            return False
        self.cli.loop_start()
        batas = time.time() + timeout
        while not self.tersambung and time.time() < batas:
            time.sleep(0.05)
        return self.tersambung

    def putus(self):
        if self.simulasi is not None:
            return
        try:
            self.cli.loop_stop()
            self.cli.disconnect()
        except Exception:
            pass

    # ------------------------------------------------------------- perintah
    def kirim_rc(self, speed_sps, steer_deg, paksa=False):
        """Perintah gabungan gas + kemudi. Dipanggil >= 20x per detik.

        Firmware menghentikan motor bila aliran ini putus lebih lama dari
        RC_TIMEOUT_S, jadi perintah yang SAMA pun tetap harus dikirim ulang -
        bukan hanya saat nilainya berubah.
        """
        servo = steer_ke_servo(steer_deg, self.p)
        speed = int(round(speed_sps))
        pesan = f"{speed} {servo:.1f}"

        if self.simulasi is not None:
            self.simulasi.perintah(speed, steer_deg)
            self._rc_terakhir, self._waktu_rc = pesan, time.time()
            return True

        if not paksa and pesan == self._rc_terakhir and \
                time.time() - self._waktu_rc < konfig.RC_PERIODE_S:
            return True
        self._rc_terakhir, self._waktu_rc = pesan, time.time()
        if not self.tersambung:
            return False
        self.cli.publish(self.p.topik_rc, pesan)
        return True

    def cmd(self, teks):
        if self.simulasi is not None:
            # Bedakan keduanya seperti firmware membedakannya: estop =
            # forceStop() (seketika), stop = stopMove() (melandai). Kalau
            # disamakan, simulasi tidak bisa dipakai menilai apakah estop di
            # akhir gerakan maju benar-benar mengurangi luncuran.
            if teks == "estop" and hasattr(self.simulasi, "henti_mendadak"):
                self.simulasi.henti_mendadak()
            elif teks in ("estop", "stop"):
                self.simulasi.perintah(0, 0)
            return True
        if not self.tersambung:
            return False
        self.cli.publish(self.p.topik_cmd, teks)
        return True

    def estop(self, alasan=""):
        print(f"[ESTOP {self.p.id:02d}] {alasan}")
        self.cmd("estop")
        # rc netral dikirim juga: kalau estop hilang di jaringan, aliran rc
        # bernilai nol tetap membuat motor berhenti.
        self.kirim_rc(0, 0, paksa=True)

    def berhenti(self):
        self.cmd("stop")
        self.kirim_rc(0, 0, paksa=True)

    def tengah(self):
        self.cmd("center")

    # -------------------------------------------------------------- keadaan
    def ambil_telemetri(self):
        with self._kunci:
            return dict(self.telemetri), self.waktu_telemetri

    def mulai_rekam_telemetri(self):
        """Mulai menyimpan SETIAP pesan telemetri beserta waktu terimanya di PC.

        Dipakai uji gas untuk melacak ke mana tunda perintah->gerak hilang:
        tgt_sps berubah saat firmware MENERIMA perintah rc, sps saat stepper
        MULAI menanjak. Waktu yang dicatat adalah waktu TERIMA di PC - pesan
        telemetri sendiri harus kembali lewat broker yang sama.
        """
        with self._kunci:
            self._rekam_telemetri = []

    def selesai_rekam_telemetri(self):
        """Hentikan perekaman -> [(waktu_terima, data), ...]."""
        with self._kunci:
            hasil, self._rekam_telemetri = self._rekam_telemetri or [], None
        return hasil

    # Bendera DRV_STATUS, harus sama dengan enum di firmware drive.h.
    DRV_OTPW, DRV_OT, DRV_SHORT, DRV_OPEN = 1, 2, 4, 8

    def gangguan_driver(self):
        """Nama gangguan driver dari telemetri, atau None bila sehat.

        Ini satu-satunya hal di seluruh telemetri yang benar-benar HASIL
        PENGINDERAAN: sensor suhu, deteksi short, dan deteksi kumparan terbuka
        ada di dalam chip TMC2209. Sisanya (pos, sps, str) adalah perintah yang
        dipantulkan balik, bukan pengukuran.
        """
        if self.simulasi is not None:
            return None
        t, umur = self.ambil_telemetri()
        if not t or time.time() - umur > 3.0:
            return None            # telemetri basi: jangan mengarang kesimpulan
        f = int(t.get("drv_flag", 0))
        nama = []
        if f & self.DRV_OT:
            nama.append("OVERTEMP (driver mematikan diri)")
        if f & self.DRV_OTPW:
            nama.append("suhu tinggi")
        if f & self.DRV_SHORT:
            nama.append("SHORT ke ground")
        if f & self.DRV_OPEN:
            nama.append("kumparan motor terbuka")
        return ", ".join(nama) if nama else None

    def ambil_stall(self):
        if self.stall_baru:
            self.stall_baru = False
            return True
        return False

    def ringkas(self):
        t, umur = self.ambil_telemetri()
        if self.simulasi is not None:
            return "SIMULASI"
        if not self.tersambung:
            return "MQTT PUTUS"
        if not t:
            return "tersambung, telemetri belum masuk"
        basi = time.time() - umur
        return (f"{t.get('drv', '?')} pos {t.get('pos', 0)} sps {t.get('sps', 0)} "
                f"kemudi {t.get('str', '?')} rssi {t.get('rssi', '?')} "
                f"tmc {'ok' if t.get('tmc') else 'MATI'} stall {t.get('stall', 0)}"
                + (f" [telemetri basi {basi:.1f}s]" if basi > 2 else ""))


# ------------------------------------------------------------------ mandiri
def main():
    ap = argparse.ArgumentParser(description="Uji sambungan ke firmware v2")
    ap.add_argument("--uji", action="store_true", help="kendali manual (MENGGERAKKAN MOTOR)")
    ap.add_argument("--host", default=None)
    ap.add_argument("--id", type=int, default=konfig.ROBOT_ID, help="robot, lihat konfig.ROBOT")
    args = ap.parse_args()
    konfig.pilih_robot(args.id)
    if konfig.STEER_SERVO_CENTER is None:
        raise SystemExit(f"[!] steer_center robot ID {args.id:02d} belum diisi di konfig.ROBOT.")

    r = Robot(host=args.host)
    print(f"[i] menyambung ke {r.host}:{r.port} ...")
    if not r.sambung():
        raise SystemExit("[!] Tidak tersambung. Cek broker, Wi-Fi, dan firmware.")

    if not args.uji:
        print("[i] memantau telemetri 10 detik (tanpa menggerakkan motor). "
              "Pakai --uji untuk kendali manual.")
        for _ in range(40):
            print("\r  " + r.ringkas().ljust(110), end="", flush=True)
            time.sleep(0.25)
        print()
        r.putus()
        return

    print("[i] KENDALI MANUAL - motor akan bergerak.")
    print("    w/s = gas maju/mundur | a/d = kemudi | spasi = berhenti | e = estop | q = keluar")
    import cv2
    import numpy as np
    win = "Uji robot (fokuskan jendela ini)"
    cv2.namedWindow(win)
    gas, kemudi = 0, 0.0
    try:
        while True:
            r.kirim_rc(gas, kemudi, paksa=True)
            img = np.zeros((140, 760, 3), np.uint8)
            cv2.putText(img, f"gas {gas:+6d} sps   kemudi {kemudi:+6.1f} deg -> servo "
                             f"{steer_ke_servo(kemudi):.1f}", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(img, r.ringkas()[:88], (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(img, "w/s gas | a/d kemudi | spasi stop | e estop | q keluar", (10, 118),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)
            cv2.imshow(win, img)

            k = cv2.waitKey(int(konfig.RC_PERIODE_S * 1000)) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("w"):
                gas = min(konfig.FORWARD_SPEED, gas + 500)
            elif k == ord("s"):
                gas = max(-konfig.REVERSE_SPEED, gas - 500)
            elif k == ord("a"):
                kemudi = max(-konfig.MAX_STEER_DEG, kemudi - 2)
            elif k == ord("d"):
                kemudi = min(konfig.MAX_STEER_DEG, kemudi + 2)
            elif k == ord(" "):
                gas, kemudi = 0, 0.0
            elif k == ord("e"):
                gas, kemudi = 0, 0.0
                r.estop("tombol e")
    finally:
        r.berhenti()
        time.sleep(0.2)
        r.putus()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
