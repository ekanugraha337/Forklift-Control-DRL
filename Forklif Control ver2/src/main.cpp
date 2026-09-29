#include <Arduino.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "drive.h"
#include "led.h"
#include "log.h"
#include "mmi.h"
#include "net.h"
#include "steering.h"

// ---------------------------------------------------------------------------
// FORKLIFT CONTROL v2 - Waveshare ESP32-S3-Nano (Robot ID 01)
//
//   Penggerak maju/mundur : motor stepper NEMA17 via driver TMC2209 (UART)
//   Kemudi (steering)     : servo MG995 di roda belakang
//   Kendali               : MQTT, MMI telnet port 23, USB Serial
//   Update firmware       : OTA lewat WiFi
//   Konfigurasi           : Preferences.h (NVS), diubah lewat MMI
//   Pengaman              : Task Watchdog Timer
//
// Beda pokok dengan v1:
//   - TMC2209 dikendalikan lewat UART (PDN_UART), jadi arus motor, micro-
//     stepping, dan mode chopper diatur dari konfigurasi, bukan trimpot dan
//     jumper; status driver juga bisa dibaca, bukan ditebak dari pin DIAG.
//   - Sudut kemudi bertipe float sampai ke perhitungan pulsa, sehingga titik
//     lurus 89,45 derajat dipakai apa adanya - trim pulsa 5 us di v1 hilang.
//   - Seluruh setelan tersimpan di NVS dan bisa diubah saat berjalan lewat MMI,
//     tidak lagi dipatok di secrets.h saat kompilasi.
//   - Topik MQTT mengikuti ID robot: forklift01/in/cmd, forklift01/in/rc,
//     forklift01/out/telemetry.
//
// Peta pin ada di include/hardware.h, nilai bawaan di include/defaults.h.
// ---------------------------------------------------------------------------

// Watchdog. 8 detik dipilih dengan longgar: bagian paling lambat di loop()
// adalah satu transaksi UART ke TMC2209 (beberapa milidetik) dan percobaan
// sambung MQTT, keduanya jauh di bawah itu. Yang ingin ditangkap adalah board
// yang benar-benar menggantung - stack WiFi buntu, atau loop() tidak kembali.
//
// Dua tempat menyuapi watchdog DI LUAR loop(), karena keduanya tidak kembali ke
// loop() selama bekerja: penantian WiFi 20 detik di netBegin(), dan callback
// progres OTA saat flash ditulis.
static const uint32_t WDT_TIMEOUT_S = 8;

static const uint32_t TELEMETRY_INTERVAL_MS = 250;   // 4x per detik

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("=== FORKLIFT CONTROL v2 - ESP32-S3 ===");

  // Watchdog dipasang paling awal supaya kegagalan saat setup pun tertangkap.
  // panic = true: board di-restart, bukan sekadar mencatat lalu diam.
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);
  Serial.printf("Watchdog aktif, batas %lu detik.\n", (unsigned long)WDT_TIMEOUT_S);

  // Konfigurasi dimuat PERTAMA: seluruh modul di bawah membaca cfg saat begin.
  configBegin();

  // LED dinyalakan sedini mungkin supaya sudah berkedip selama penantian WiFi
  // di netBegin() - itu bagian terlama dari boot, dan tanpa tanda hidup papan
  // terlihat seperti mati.
  ledBegin();

  driveBegin();
  steeringBegin();
  netBegin();
  mmiBegin();

  Log.printf("Forklift ID %u siap.\n", (unsigned)cfg.id);
  netServiceDelay(500);
}

void loop() {
  esp_task_wdt_reset();

  netLoop();
  if (netOtaInProgress()) return;   // flash sedang ditulis, jangan sentuh apa pun

  ledUpdate();
  mmiLoop();
  steeringUpdate();
  driveRcWatchdog();
  driveTmcPoll();
  driveDiagService();   // stall / gangguan driver, hanya bekerja saat ada kejadian

  static uint32_t telemetryLastMs = 0;
  if (millis() - telemetryLastMs >= TELEMETRY_INTERVAL_MS) {
    telemetryLastMs = millis();
    netPublishTelemetry();
  }

  delay(1);
}
