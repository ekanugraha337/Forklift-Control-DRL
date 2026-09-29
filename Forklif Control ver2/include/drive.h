#pragma once

#include <Arduino.h>

#include "log.h"

// ---------------------------------------------------------------------------
// PENGGERAK - stepper NEMA17 lewat TMC2209
//
// Pulsa STEP dibangkitkan peripheral RMT/MCPWM oleh FastAccelStepper, bukan di
// loop(), sehingga gerak motor tidak terganggu aktivitas WiFi/OTA/MQTT.
//
// Beda pokok dengan v1: driver sekarang dikendalikan lewat UART (PDN_UART), jadi
// arus motor, microstepping, dan mode chopper diatur dari konfigurasi - bukan
// lewat trimpot Vref dan jumper. UART juga membuat status driver (overtemp,
// short, open load) bisa DIBACA, bukan hanya ditebak dari satu pin DIAG.
// ---------------------------------------------------------------------------

// Bendera status driver, hasil pembacaan register DRV_STATUS.
enum : uint8_t {
  DRV_FLAG_OTPW    = 1 << 0,   // peringatan suhu
  DRV_FLAG_OT      = 1 << 1,   // overtemp, driver mematikan diri
  DRV_FLAG_SHORT   = 1 << 2,   // short ke ground (s2ga/s2gb)
  DRV_FLAG_OPEN    = 1 << 3,   // kumparan terbuka / kabel motor lepas
  DRV_FLAG_STEALTH = 1 << 4,   // sedang di mode stealthChop
};

void driveBegin();

// --- Perintah dari manusia / skrip. Balasan ditulis ke out. ---------------
void driveForward(uint32_t sps, Print &out);
void driveBackward(uint32_t sps, Print &out);
void driveMove(long steps, Print &out);
void driveStop(Print &out);
void driveEmergencyStop(Print &out);
void driveEnable(Print &out);
void driveDisable(Print &out);
void driveZero(Print &out);
bool driveSetSpeed(uint32_t sps, Print &out);
bool driveSetAccel(uint32_t accel, Print &out);

// --- Mode RC (gamepad / navigasi otomatis) --------------------------------
// "rc <sps_bertanda> <sudut>" dikirim puluhan kali per detik. Sengaja senyap.
void driveRc(long sps, float angle);

// Hentikan motor bila aliran perintah RC terputus. Kendaraan tidak boleh
// melaju tanpa kendali.
void driveRcWatchdog();

// Dipakai pengaman lain (broker putus, OTA mulai) yang bukan perintah manusia.
void driveSafeStop();
void driveHardStop();

// --- Keadaan ---------------------------------------------------------------
bool     driveRunning();
long     drivePosition();
long     driveCurrentSps();
uint32_t driveTargetSps();   // kecepatan yang sedang diperintahkan

// "run" / "idle" / "none" - sama dengan field "drv" pada telemetri v1.
const char *driveStateName();
int      driveDir();
bool     driveRcActive();
bool     driveContinuous();
uint32_t driveIndexCount();
bool     driveDiagPin();
bool     driveDiagEvent();

// Menangani pin DIAG yang baru aktif: membedakan stall dari gangguan driver,
// melaporkannya, dan (bila sg_stop=1) menghentikan motor. Dipanggil tiap loop -
// murah, karena hanya bekerja saat ada kejadian baru.
void     driveDiagService();

// --- TMC2209 lewat UART ---------------------------------------------------
// Dipanggil berkala dari loop(): membaca DRV_STATUS dan mencoba menyambung
// ulang bila UART tidak menjawab.
void     driveTmcPoll();
bool     driveTmcOk();
uint8_t  driveTmcFlags();

// --- Diagnosa integritas STEP dan deteksi stall ---------------------------
//
// PENTING: tidak ada satu pun dari ini yang merupakan umpan balik posisi poros.
// TMC2209 open loop dan tidak punya masukan encoder. MSCNT adalah penghitung
// microstep INTERNAL DRIVER yang maju karena pulsa STEP, jadi ia menjawab
// "apakah setiap pulsa STEP benar-benar diterima driver", bukan "apakah roda
// benar-benar berputar". Hanya StallGuard yang tahu rotor kehilangan sinkron,
// dan itu pun hanya di stealthChop.

// Laju langkah hasil ukur driver, diturunkan dari register TSTEP. MSCNT dan
// TSTEP mentah tidak diekspor: keduanya hanya berarti setelah diolah, dan
// pengolahannya ada di driveTmcPrint() ("show drv").
uint32_t driveMeasuredSps();

// Selisih fasa MSCNT terhadap posisi perintah, satuan microstep 1/256.
// Bukan nol = ada pulsa STEP yang tidak sampai. Hanya sah saat motor diam.
bool     drivePhaseKnown();
int      drivePhaseError();

uint16_t driveSgResult();       // beban StallGuard4, makin kecil makin berat
bool     driveStallArmed();     // true bila deteksi stall benar-benar aktif
uint32_t driveStallCount();
bool     driveStallEvent();

// Kirim ulang arus / microstep / mode chopper dari konfigurasi ke driver.
void     driveApplyConfig();

void     drivePrint(Print &out);
void     driveTmcPrint(Print &out);
