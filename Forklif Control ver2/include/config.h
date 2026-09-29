#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// KONFIGURASI TERSIMPAN (Preferences / NVS)
//
// Seluruh setelan disimpan di flash lewat Preferences.h, namespace "forklift".
// Saat processor pertama kali hidup, isi NVS dibaca ke variable di RAM; jadi
// untuk selanjutnya variable-nya bernilai sama dengan konfigurasi yang terakhir
// disimpan. Board baru (atau NVS yang versinya berbeda) diisi dengan nilai
// bawaan dari defaults.h lalu langsung ditulis, supaya isi NVS selalu lengkap.
//
// Semua setelan dideklarasikan sekali di tabel CFG_ITEMS. Tabel itulah yang
// dipakai untuk memuat, menyimpan, menampilkan ("show config"), dan mengubah
// ("set <key> <value>") - jadi menambah setelan baru cukup satu baris, tanpa
// menyentuh MMI maupun rutin simpan/muat.
//
// Nama key WAJIB <= 15 karakter: itu batas keras kunci NVS.
// ---------------------------------------------------------------------------

struct Config {
  // identitas
  uint8_t  id;

  // kemudi (derajat, float - lihat catatan di defaults.h)
  float    steerCenter;
  float    steerMin;
  float    steerMax;
  bool     steerInvert;

  // jaringan
  char     ssid[33];
  char     wifiPass[65];
  char     ip[16];
  char     gateway[16];
  char     subnet[16];
  char     dns[16];

  // broker MQTT
  char     mqttHost[41];
  uint16_t mqttPort;
  char     mqttUser[33];
  char     mqttPass[65];

  // OTA
  char     otaHost[33];
  char     otaPass[33];

  // penggerak
  uint32_t driveSpeed;
  uint32_t driveAccel;
  uint16_t tmcCurrent;    // mA RMS
  uint16_t tmcMicrosteps; // 16 bit: nilai 256 tidak muat di uint8_t
  bool     tmcSpreadCycle;

  // LED tanda hidup
  uint8_t  ledPin;
  bool     ledInvert;     // true bila LED menyala saat pin LOW
  bool     ledRgb;        // true bila LED-nya WS2812/NeoPixel, bukan LED biasa

  // deteksi stall (StallGuard4, hanya sah di stealthChop)
  uint8_t  sgThrs;        // 0 = mati
  uint32_t sgMinSps;      // kecepatan minimum agar deteksi diaktifkan
  bool     sgStop;        // true = stall memicu emergency stop
};

extern Config cfg;

enum CfgType : uint8_t { CFG_U8, CFG_U16, CFG_U32, CFG_FLOAT, CFG_BOOL, CFG_STR };

struct CfgItem {
  const char *key;
  CfgType     type;
  void       *ptr;
  size_t      cap;      // kapasitas buffer, hanya untuk CFG_STR
  const char *deflt;    // nilai bawaan SEBAGAI TEKS, dipakai "factory"
  bool        reboot;   // true bila perubahannya baru berlaku setelah restart
  const char *help;
};

extern const CfgItem CFG_ITEMS[];
extern const size_t  CFG_COUNT;

// Baca NVS ke cfg (mengisi bawaan bila NVS kosong). Dipanggil paling awal.
void configBegin();

// Tulis seluruh cfg ke NVS.
void configSave();

// Buang perubahan yang belum disimpan, baca ulang dari NVS.
void configReload();

// Kembalikan cfg ke nilai bawaan defaults.h lalu simpan.
void configFactory();

// Ubah satu setelan di RAM. Mengembalikan false + alasan bila nilainya ditolak.
// Tidak menyentuh NVS: perubahan baru permanen setelah configSave().
bool configSet(const char *key, const char *value, String &err);

// Nilai satu setelan sebagai teks (untuk "show config").
String configValue(const CfgItem &item);

void configPrint(Print &out);

// --- Topik MQTT ------------------------------------------------------------
// Dirakit dari cfg.id: id 1 -> "forklift01/in/cmd" dan seterusnya. Dipanggil
// ulang oleh configBegin(), jadi mengubah id lalu restart otomatis memindahkan
// seluruh topik.
const char *topicCmd();
const char *topicRc();
const char *topicTelemetry();
const char *topicLog();
const char *topicState();
const char *topicPrefix();   // "forklift01" - dipakai juga sebagai client id
