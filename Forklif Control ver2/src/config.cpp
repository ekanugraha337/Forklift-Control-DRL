#include "config.h"

#include <Preferences.h>

#include "defaults.h"
#include "hardware.h"
#include "log.h"

Config cfg;

static Preferences prefs;
static const char *NVS_NAMESPACE = "forklift";

// Dinaikkan kalau susunan CFG_ITEMS berubah sampai isi NVS lama tidak lagi
// masuk akal. Board yang menyimpan versi lain akan diisi ulang dengan bawaan,
// jadi tidak ada setelan setengah-terisi setelah firmware diganti.
static const char *NVS_KEY_VERSION = "cfg_ver";
static const uint8_t CFG_VERSION   = 1;

const CfgItem CFG_ITEMS[] = {
  // key              tipe        ptr                      cap  bawaan                     reboot  keterangan
  { "id",             CFG_U8,     &cfg.id,                 0,   DEF_FORKLIFT_ID,           true,   "ID robot, menentukan nama topik MQTT" },

  { "str_center",     CFG_FLOAT,  &cfg.steerCenter,        0,   DEF_STEER_CENTER,          false,  "sudut kemudi lurus (derajat)" },
  { "str_min",        CFG_FLOAT,  &cfg.steerMin,           0,   DEF_STEER_MIN,             false,  "batas kemudi kiri (derajat)" },
  { "str_max",        CFG_FLOAT,  &cfg.steerMax,           0,   DEF_STEER_MAX,             false,  "batas kemudi kanan (derajat)" },
  { "str_inv",        CFG_BOOL,   &cfg.steerInvert,        0,   DEF_STEER_INVERT,          false,  "balik kiri<->kanan (roda belakang)" },

  { "ssid",           CFG_STR,    cfg.ssid,                sizeof(cfg.ssid),     DEF_WIFI_SSID,   true,   "nama jaringan WiFi" },
  { "wifi_pass",      CFG_STR,    cfg.wifiPass,            sizeof(cfg.wifiPass), DEF_WIFI_PASS,   true,   "sandi WiFi" },
  { "ip",             CFG_STR,    cfg.ip,                  sizeof(cfg.ip),       DEF_IP,          true,   "alamat IP static board" },
  { "gateway",        CFG_STR,    cfg.gateway,             sizeof(cfg.gateway),  DEF_GATEWAY,     true,   "gateway" },
  { "subnet",         CFG_STR,    cfg.subnet,              sizeof(cfg.subnet),   DEF_SUBNET,      true,   "subnet mask" },
  { "dns",            CFG_STR,    cfg.dns,                 sizeof(cfg.dns),      DEF_DNS,         true,   "server DNS" },

  { "mqtt_host",      CFG_STR,    cfg.mqttHost,            sizeof(cfg.mqttHost), DEF_MQTT_HOST,   true,   "alamat broker MQTT" },
  { "mqtt_port",      CFG_U16,    &cfg.mqttPort,           0,   DEF_MQTT_PORT,             true,   "port broker MQTT" },
  { "mqtt_user",      CFG_STR,    cfg.mqttUser,            sizeof(cfg.mqttUser), DEF_MQTT_USER,   true,   "username MQTT (kosongkan bila tanpa auth)" },
  { "mqtt_pass",      CFG_STR,    cfg.mqttPass,            sizeof(cfg.mqttPass), DEF_MQTT_PASS,   true,   "sandi MQTT" },

  { "ota_host",       CFG_STR,    cfg.otaHost,             sizeof(cfg.otaHost),  DEF_OTA_HOST,    true,   "hostname OTA / mDNS" },
  { "ota_pass",       CFG_STR,    cfg.otaPass,             sizeof(cfg.otaPass),  DEF_OTA_PASS,    true,   "sandi OTA" },

  { "speed",          CFG_U32,    &cfg.driveSpeed,         0,   DEF_DRIVE_SPEED,           false,  "kecepatan bawaan (langkah/detik)" },
  { "accel",          CFG_U32,    &cfg.driveAccel,         0,   DEF_DRIVE_ACCEL,           false,  "akselerasi (langkah/detik^2)" },
  { "tmc_ma",         CFG_U16,    &cfg.tmcCurrent,         0,   DEF_TMC_CURRENT_MA,        false,  "arus RMS motor (mA)" },
  { "usteps",         CFG_U16,    &cfg.tmcMicrosteps,      0,   DEF_TMC_MICROSTEPS,        false,  "microstepping (1/2/4/8/16/32/64/128/256)" },
  { "spread",         CFG_BOOL,   &cfg.tmcSpreadCycle,     0,   DEF_TMC_SPREADCYCLE,       false,  "1=spreadCycle (torsi), 0=stealthChop (senyap)" },

  { "led_pin",        CFG_U8,     &cfg.ledPin,             0,   DEF_LED_PIN,               true,   "GPIO LED tanda hidup" },
  { "led_inv",        CFG_BOOL,   &cfg.ledInvert,          0,   DEF_LED_INVERT,            false,  "1=LED menyala saat pin LOW" },
  { "led_rgb",        CFG_BOOL,   &cfg.ledRgb,             0,   DEF_LED_RGB,               false,  "1=LED WS2812/NeoPixel, 0=LED biasa" },

  { "sg_thrs",        CFG_U8,     &cfg.sgThrs,             0,   DEF_SG_THRS,               false,  "ambang stall 0-255, 0=mati (butuh spread=0)" },
  { "sg_min_sps",     CFG_U32,    &cfg.sgMinSps,           0,   DEF_SG_MIN_SPS,            false,  "kecepatan minimum agar deteksi stall aktif" },
  { "sg_stop",        CFG_BOOL,   &cfg.sgStop,             0,   DEF_SG_STOP,               false,  "1=stall memicu estop, 0=hanya dilaporkan" },
};

const size_t CFG_COUNT = sizeof(CFG_ITEMS) / sizeof(CFG_ITEMS[0]);

// --- Topik -----------------------------------------------------------------

static char tPrefix[16];
static char tCmd[32], tRc[32], tTelemetry[40], tLog[32], tState[32];

static void buildTopics() {
  snprintf(tPrefix,    sizeof(tPrefix),    "forklift%02u", (unsigned)cfg.id);
  snprintf(tCmd,       sizeof(tCmd),       "%s/in/cmd", tPrefix);
  snprintf(tRc,        sizeof(tRc),        "%s/in/rc", tPrefix);
  snprintf(tTelemetry, sizeof(tTelemetry), "%s/out/telemetry", tPrefix);
  snprintf(tLog,       sizeof(tLog),       "%s/out/log", tPrefix);
  snprintf(tState,     sizeof(tState),     "%s/out/state", tPrefix);
}

const char *topicPrefix()    { return tPrefix; }
const char *topicCmd()       { return tCmd; }
const char *topicRc()        { return tRc; }
const char *topicTelemetry() { return tTelemetry; }
const char *topicLog()       { return tLog; }
const char *topicState()     { return tState; }

// --- Muat / simpan ---------------------------------------------------------

// true selama applyDefaults() berjalan. Dipakai validate() untuk melewati
// pemeriksaan yang membandingkan satu setelan dengan setelan lain, karena saat
// tabel diisi dari nol pasangannya belum tentu sudah terisi.
static bool applyingDefaults = false;

// Tulis nilai bawaan (teks) ke variable, tanpa menyentuh NVS.
static void applyDefaults() {
  String err;
  applyingDefaults = true;
  for (size_t i = 0; i < CFG_COUNT; i++) {
    // Bawaan tidak boleh gagal validasi; kalau gagal, itu bug di tabel dan
    // lebih baik kelihatan di log daripada diam-diam terlewat.
    if (!configSet(CFG_ITEMS[i].key, CFG_ITEMS[i].deflt, err)) {
      Log.printf("BUG: bawaan \"%s\" = \"%s\" ditolak (%s)\n",
                 CFG_ITEMS[i].key, CFG_ITEMS[i].deflt, err.c_str());
    }
  }
  applyingDefaults = false;
}

static void loadFromNvs() {
  for (size_t i = 0; i < CFG_COUNT; i++) {
    const CfgItem &it = CFG_ITEMS[i];
    switch (it.type) {
      case CFG_U8:    *(uint8_t *)it.ptr  = prefs.getUChar(it.key, *(uint8_t *)it.ptr);   break;
      case CFG_U16:   *(uint16_t *)it.ptr = prefs.getUShort(it.key, *(uint16_t *)it.ptr); break;
      case CFG_U32:   *(uint32_t *)it.ptr = prefs.getULong(it.key, *(uint32_t *)it.ptr);  break;
      case CFG_FLOAT: *(float *)it.ptr    = prefs.getFloat(it.key, *(float *)it.ptr);     break;
      case CFG_BOOL:  *(bool *)it.ptr     = prefs.getBool(it.key, *(bool *)it.ptr);       break;
      case CFG_STR: {
        String v = prefs.getString(it.key, (const char *)it.ptr);
        snprintf((char *)it.ptr, it.cap, "%s", v.c_str());
        break;
      }
    }
  }
}

// Isi NVS tidak melewati validate() saat dibaca - nilainya memang sudah
// tervalidasi ketika ditulis. Yang tetap diperiksa hanya hubungan antar setelan
// yang bisa membuat kendaraan tidak terkendali kalau ternyata rusak (NVS dari
// firmware lain, flash yang cacat). Lebih baik kembali ke bawaan yang waras
// daripada berjalan dengan kemudi terkunci di satu ujung.
static void sanityCheck() {
  if (cfg.steerMin >= cfg.steerMax) {
    Log.printf("PERINGATAN: batas kemudi di NVS tertukar (%.2f >= %.2f), "
               "dikembalikan ke bawaan %s-%s.\n",
               (double)cfg.steerMin, (double)cfg.steerMax,
               DEF_STEER_MIN, DEF_STEER_MAX);
    cfg.steerMin = atof(DEF_STEER_MIN);
    cfg.steerMax = atof(DEF_STEER_MAX);
  }
  if (cfg.steerCenter < cfg.steerMin || cfg.steerCenter > cfg.steerMax) {
    Log.printf("PERINGATAN: titik lurus %.2f di luar batas kemudi %.2f-%.2f, "
               "dikembalikan ke bawaan %s.\n",
               (double)cfg.steerCenter, (double)cfg.steerMin, (double)cfg.steerMax,
               DEF_STEER_CENTER);
    cfg.steerCenter = atof(DEF_STEER_CENTER);
  }
  if (cfg.driveSpeed == 0) cfg.driveSpeed = atof(DEF_DRIVE_SPEED);
  if (cfg.driveAccel == 0) cfg.driveAccel = atof(DEF_DRIVE_ACCEL);
  if (cfg.id < 1) cfg.id = atoi(DEF_FORKLIFT_ID);
}

static void writeToNvs() {
  for (size_t i = 0; i < CFG_COUNT; i++) {
    const CfgItem &it = CFG_ITEMS[i];
    switch (it.type) {
      case CFG_U8:    prefs.putUChar(it.key, *(uint8_t *)it.ptr);   break;
      case CFG_U16:   prefs.putUShort(it.key, *(uint16_t *)it.ptr);  break;
      case CFG_U32:   prefs.putULong(it.key, *(uint32_t *)it.ptr);   break;
      case CFG_FLOAT: prefs.putFloat(it.key, *(float *)it.ptr);      break;
      case CFG_BOOL:  prefs.putBool(it.key, *(bool *)it.ptr);        break;
      case CFG_STR:   prefs.putString(it.key, (const char *)it.ptr); break;
    }
  }
  prefs.putUChar(NVS_KEY_VERSION, CFG_VERSION);
}

// Preferences::begin() menolak dipanggil dua kali tanpa end(), dan balasannya
// false - persis seperti kegagalan sungguhan. Statusnya dicatat sendiri supaya
// "reload" di MMI tidak salah melaporkan NVS rusak.
static bool nvsOpen = false;

void configBegin() {
  applyDefaults();               // supaya tidak ada variable tak terisi

  if (!nvsOpen) nvsOpen = prefs.begin(NVS_NAMESPACE, false);
  if (!nvsOpen) {
    Log.println("PERINGATAN: NVS tidak bisa dibuka, konfigurasi jalan dari bawaan.");
    buildTopics();
    return;
  }

  uint8_t ver = prefs.getUChar(NVS_KEY_VERSION, 0);
  if (ver != CFG_VERSION) {
    Log.printf("NVS %s (versi %u, diharapkan %u) - diisi nilai bawaan.\n",
               ver == 0 ? "masih kosong" : "versinya berbeda", ver, CFG_VERSION);
    writeToNvs();
  } else {
    loadFromNvs();
    sanityCheck();
    Log.println("Konfigurasi dimuat dari NVS.");
  }

  buildTopics();
}

void configSave() {
  if (!nvsOpen) return;
  writeToNvs();
  buildTopics();
}

// Buang perubahan yang belum disimpan, kembali ke isi NVS.
void configReload() {
  applyDefaults();
  if (nvsOpen) { loadFromNvs(); sanityCheck(); }
  buildTopics();
}

void configFactory() {
  applyDefaults();
  if (nvsOpen) writeToNvs();
  buildTopics();
}

// --- Baca / ubah satu setelan ---------------------------------------------

String configValue(const CfgItem &it) {
  char buf[24];
  switch (it.type) {
    case CFG_U8:    snprintf(buf, sizeof(buf), "%u",  (unsigned)*(uint8_t *)it.ptr);  return buf;
    case CFG_U16:   snprintf(buf, sizeof(buf), "%u",  (unsigned)*(uint16_t *)it.ptr); return buf;
    case CFG_U32:   snprintf(buf, sizeof(buf), "%lu", (unsigned long)*(uint32_t *)it.ptr); return buf;
    case CFG_FLOAT: snprintf(buf, sizeof(buf), "%.2f", *(float *)it.ptr);             return buf;
    case CFG_BOOL:  return *(bool *)it.ptr ? "1" : "0";
    case CFG_STR:   return String((const char *)it.ptr);
  }
  return "";
}

static const CfgItem *findItem(const char *key) {
  for (size_t i = 0; i < CFG_COUNT; i++) {
    if (strcasecmp(CFG_ITEMS[i].key, key) == 0) return &CFG_ITEMS[i];
  }
  return nullptr;
}

// Batas akal sehat per setelan. Bukan sekadar kerapian: sudut kemudi di luar
// 0-180 membuat servo mentok dan menarik arus terus, microstepping bukan pangkat
// dua diabaikan diam-diam oleh driver, dan arus terlalu besar memanggang motor.
static bool validate(const CfgItem &it, double num, const char *text, String &err) {
  if (strcmp(it.key, "id") == 0 && (num < 1 || num > 99)) {
    err = "id harus 1-99 (topik memakai dua angka)"; return false;
  }
  if (strncmp(it.key, "str_", 4) == 0 && it.type == CFG_FLOAT && (num < 0 || num > 180)) {
    err = "sudut kemudi harus 0-180"; return false;
  }
  // Batas kiri dan kanan harus tetap berurutan. Kalau tertukar, constrain() di
  // steeringSet() akan mengunci kemudi di satu ujung tanpa pesan apa pun -
  // gejalanya "robot selalu membelok maksimum", jauh dari sebabnya.
  //
  // Pemeriksaan ini melihat NILAI PASANGANNYA, jadi dilewati saat applyDefaults()
  // sedang mengisi tabel dari nol: di sana pasangannya memang belum tentu terisi,
  // dan nilai bawaan 10/170 sudah pasti berurutan.
  if (!applyingDefaults) {
    if (strcmp(it.key, "str_min") == 0 && num >= cfg.steerMax) {
      err = "str_min harus lebih kecil dari str_max"; return false;
    }
    if (strcmp(it.key, "str_max") == 0 && num <= cfg.steerMin) {
      err = "str_max harus lebih besar dari str_min"; return false;
    }
  }
  if (strcmp(it.key, "mqtt_port") == 0 && (num < 1 || num > 65535)) {
    err = "port harus 1-65535"; return false;
  }
  if (strcmp(it.key, "speed") == 0 && (num < 1 || num > 200000)) {
    err = "kecepatan harus 1-200000 langkah/detik"; return false;
  }
  if (strcmp(it.key, "accel") == 0 && (num < 1 || num > 1000000)) {
    err = "akselerasi harus 1-1000000"; return false;
  }
  if (strcmp(it.key, "tmc_ma") == 0 && (num < 100 || num > 2000)) {
    err = "arus harus 100-2000 mA RMS"; return false;
  }
  // Pin LED tidak boleh menimpa pin yang sudah memegang motor, servo, atau UART
  // driver. Tanpa penjagaan ini, satu salah ketik lewat MMI bisa membuat pin
  // STEP atau DIR dikedipkan 2x per detik - robot bergerak sendiri, dan
  // sebabnya tidak akan terlihat di mana pun.
  if (strcmp(it.key, "led_pin") == 0) {
    if (num < 0 || num > 48) { err = "GPIO harus 0-48"; return false; }
    struct { int pin; const char *pakai; } dipakai[] = {
      { PIN_SERVO,  "servo kemudi" }, { PIN_STEP,   "STEP" },
      { PIN_DIR,    "DIR" },          { PIN_EN,     "EN" },
      { PIN_INDEX,  "INDEX" },        { PIN_DIAG,   "DIAG" },
      { PIN_TMC_TX, "UART TX ke TMC2209" }, { PIN_TMC_RX, "UART RX dari TMC2209" },
    };
    for (auto &d : dipakai) {
      if ((int)num == d.pin) {
        err = String("GPIO") + d.pin + " sudah dipakai " + d.pakai;
        return false;
      }
    }
  }
  if (strcmp(it.key, "sg_thrs") == 0 && (num < 0 || num > 255)) {
    err = "ambang stall harus 0-255 (0 = mati)"; return false;
  }
  if (strcmp(it.key, "sg_min_sps") == 0 && (num < 1 || num > 200000)) {
    err = "kecepatan minimum harus 1-200000 langkah/detik"; return false;
  }
  if (strcmp(it.key, "usteps") == 0) {
    // Driver hanya menerima pangkat dua 1..256. Nilai lain diabaikan diam-diam
    // oleh TMC2209, jadi lebih baik ditolak di sini daripada membuat skala
    // langkah berbeda dari yang tertulis di konfigurasi.
    const int allowed[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
    bool ok = false;
    for (int a : allowed) if ((int)num == a) { ok = true; break; }
    if (!ok) { err = "microstepping harus 1,2,4,8,16,32,64,128 atau 256"; return false; }
  }
  // Alamat IP diuji dengan parser yang nanti benar-benar dipakai WiFi.config(),
  // bukan dengan pola sendiri - supaya yang lolos di sini pasti lolos di sana.
  if (it.type == CFG_STR &&
      (strcmp(it.key, "ip") == 0 || strcmp(it.key, "gateway") == 0 ||
       strcmp(it.key, "subnet") == 0 || strcmp(it.key, "dns") == 0)) {
    IPAddress probe;
    if (!probe.fromString(text)) { err = "bukan alamat IP yang sah"; return false; }
  }
  return true;
}

bool configSet(const char *key, const char *value, String &err) {
  const CfgItem *it = findItem(key);
  if (it == nullptr) { err = "setelan tidak dikenal"; return false; }

  double num = atof(value);

  if (it->type != CFG_STR && strlen(value) == 0) { err = "nilai kosong"; return false; }
  if (!validate(*it, num, value, err)) return false;

  switch (it->type) {
    case CFG_U8:    *(uint8_t *)it->ptr  = (uint8_t)(long)num;   break;
    case CFG_U16:   *(uint16_t *)it->ptr = (uint16_t)(long)num;  break;
    case CFG_U32:   *(uint32_t *)it->ptr = (uint32_t)(long)num;  break;
    case CFG_FLOAT: *(float *)it->ptr    = (float)num;           break;
    case CFG_BOOL:
      *(bool *)it->ptr = (strcasecmp(value, "1") == 0 || strcasecmp(value, "on") == 0 ||
                          strcasecmp(value, "true") == 0 || strcasecmp(value, "ya") == 0);
      break;
    case CFG_STR:
      if (strlen(value) >= it->cap) {
        err = String("terlalu panjang, maksimum ") + String((int)it->cap - 1) + " karakter";
        return false;
      }
      snprintf((char *)it->ptr, it->cap, "%s", value);
      break;
  }
  return true;
}

void configPrint(Print &out) {
  out.println("---- KONFIGURASI (* = berlaku setelah restart) ----");
  for (size_t i = 0; i < CFG_COUNT; i++) {
    const CfgItem &it = CFG_ITEMS[i];
    // Sandi tidak ditampilkan utuh: MMI berjalan di atas telnet tanpa enkripsi.
    bool secret = (strstr(it.key, "pass") != nullptr);
    String val = configValue(it);
    if (secret && val.length()) val = "***";
    out.printf("  %-12s %-18s %s%s\n", it.key, val.c_str(),
               it.reboot ? "* " : "  ", it.help);
  }
  out.printf("  topik        %s/in/cmd, %s/in/rc, %s/out/{telemetry,log,state}\n",
             topicPrefix(), topicPrefix(), topicPrefix());
  out.println("  simpan dengan \"save\", kembalikan bawaan dengan \"factory\"");
  out.println("--------------------------------------------------");
}
