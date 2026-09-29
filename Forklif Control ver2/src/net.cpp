#include "net.h"

#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <math.h>

#include "config.h"
#include "drive.h"
#include "hardware.h"
#include "led.h"
#include "log.h"
#include "mmi.h"
#include "steering.h"

static WiFiClient   netClient;
static PubSubClient mqtt(netClient);

static bool     otaInProgress   = false;
static bool     wifiStaticActive = false;
static uint32_t mqttLastTryMs   = 0;

// --- Sink log ke MQTT ------------------------------------------------------

static void mqttLogSink(const char *line) {
  if (mqtt.connected()) mqtt.publish(topicLog(), line);
}

// --- Pesan masuk -----------------------------------------------------------

// Topik RC dipisah dari CMD supaya perintah yang datang puluhan kali per detik
// tidak melewati parser perintah umum.
static void mqttOnMessage(char *topic, byte *payload, unsigned int len) {
  String msg;
  msg.reserve(len);
  for (unsigned int i = 0; i < len; i++) {
    char c = (char)payload[i];
    if (c >= 32 && c <= 126) msg += c;
  }
  msg.trim();
  if (msg.length() == 0) return;

  if (strcmp(topic, topicRc()) == 0) {
    int sp = msg.indexOf(' ');
    if (sp <= 0) return;
    driveRc(msg.substring(0, sp).toInt(), msg.substring(sp + 1).toFloat());
    return;
  }

  // Balasan perintah dari MQTT diarahkan ke Log, jadi jawabannya sampai ke
  // topik .../out/log - tempat pengirimnya memang sudah menonton.
  mmiRunCommand(msg, Log);
}

// --- Sambung ulang tanpa memblokir ----------------------------------------

static void mqttEnsureConnected() {
  static bool wasConnected = false;

  if (mqtt.connected()) {
    mqtt.loop();
    wasConnected = true;
    return;
  }

  // PENGAMAN: koneksi broker hilang saat motor jalan terus-menerus. Kendaraan
  // tidak boleh melaju tanpa kendali.
  if (wasConnected) {
    wasConnected = false;
    Serial.println("MQTT terputus.");
    driveSafeStop();
  }

  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - mqttLastTryMs < 2000) return;
  mqttLastTryMs = millis();

  // Last Will: broker mengumumkan "offline" bila board mati mendadak.
  bool ok = (strlen(cfg.mqttUser) > 0)
      ? mqtt.connect(topicPrefix(), cfg.mqttUser, cfg.mqttPass,
                     topicState(), 0, true, "offline")
      : mqtt.connect(topicPrefix(), topicState(), 0, true, "offline");

  if (!ok) {
    Serial.printf("MQTT gagal menyambung ke %s:%u (rc=%d), coba lagi 2 detik.\n",
                  cfg.mqttHost, (unsigned)cfg.mqttPort, mqtt.state());
    return;
  }

  mqtt.publish(topicState(), "online", true);
  mqtt.subscribe(topicCmd());
  mqtt.subscribe(topicRc());
  Serial.printf("MQTT tersambung ke %s:%u, berlangganan %s dan %s\n",
                cfg.mqttHost, (unsigned)cfg.mqttPort, topicCmd(), topicRc());
  Log.printf("Forklift %s siap. Kirim perintah ke %s.\n", topicPrefix(), topicCmd());
}

// --- Telemetri -------------------------------------------------------------

// Sengaja dirakit manual dengan snprintf: satu objek datar seperti ini tidak
// sepadan dengan biaya ArduinoJson, dan ukurannya jadi pasti.
//
// Susunan field sampai "servo" DIPERTAHANKAN sama dengan v1 supaya skrip di
// tools/ (monitor2.py, MQTT_1tag.py, kalibrasi_kemudi.py) tidak perlu diubah.
// Field baru khusus v2 ditambahkan DI BELAKANG - pembaca yang memakai .get()
// tidak terganggu, sedangkan yang butuh status driver kini bisa melihatnya.
//
// "str"/"str_tgt" tetap bilangan bulat demi kecocokan itu; sudut pecahannya
// terlihat lengkap di MMI ("status").
void netPublishTelemetry() {
  if (!mqtt.connected()) return;

  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"up\":%lu,\"drv\":\"%s\",\"pos\":%ld,\"sps\":%ld,\"tgt_sps\":%lu,"
    "\"acc\":%lu,\"dir\":%d,\"rc\":%d,\"str\":%d,\"str_tgt\":%d,\"inv\":%d,"
    "\"idx\":%lu,\"diag\":%d,\"diag_ev\":%d,\"rssi\":%d,\"heap\":%lu,\"servo\":%d,"
    "\"id\":%u,\"tmc\":%d,\"drv_flag\":%u,\"mmi\":%d,"
    "\"sg\":%u,\"stall\":%lu,\"sg_on\":%d,\"step_err\":%d,\"m_sps\":%lu}",
    (unsigned long)(millis() / 1000),
    driveStateName(),
    drivePosition(), driveCurrentSps(),
    (unsigned long)driveTargetSps(), (unsigned long)cfg.driveAccel,
    driveDir(), driveRcActive() ? 1 : 0,
    (int)lroundf(steeringCurrent()), (int)lroundf(steeringTarget()),
    cfg.steerInvert ? 1 : 0,
    (unsigned long)driveIndexCount(), driveDiagPin() ? 1 : 0,
    driveDiagEvent() ? 1 : 0,
    WiFi.RSSI(), (unsigned long)ESP.getFreeHeap(), steeringReady() ? 1 : 0,
    (unsigned)cfg.id, driveTmcOk() ? 1 : 0, (unsigned)driveTmcFlags(),
    mmiClientCount(),
    // "sg" dipakai untuk MENYETEL sg_thrs: jalankan robot dengan beban normal,
    // lihat nilai terendah yang muncul, pasang sg_thrs sekitar setengahnya.
    (unsigned)driveSgResult(), (unsigned long)driveStallCount(),
    driveStallArmed() ? 1 : 0,
    drivePhaseKnown() ? drivePhaseError() : 0,
    (unsigned long)driveMeasuredSps());

  mqtt.publish(topicTelemetry(), buf);
}

// --- Status ----------------------------------------------------------------

void netPrintStatus(Print &out) {
  if (WiFi.status() == WL_CONNECTED) {
    out.printf("WiFi    : \"%s\"  IP %s  RSSI %d dBm\n", cfg.ssid,
               WiFi.localIP().toString().c_str(), WiFi.RSSI());
    out.printf("  Alamat  : %s, mask %s, gateway %s\n",
               wifiStaticActive ? "static" : "DHCP",
               WiFi.subnetMask().toString().c_str(),
               WiFi.gatewayIP().toString().c_str());
    out.printf("  MQTT    : %s:%u  %s\n", cfg.mqttHost, (unsigned)cfg.mqttPort,
               mqtt.connected() ? "tersambung" : "TIDAK tersambung");
    out.printf("  Topik   : %s/in/{cmd,rc}  %s/out/{telemetry,log,state}\n",
               topicPrefix(), topicPrefix());
    out.printf("  MMI     : telnet %s port 23, %d sesi aktif\n",
               WiFi.localIP().toString().c_str(), mmiClientCount());
  } else {
    out.printf("WiFi    : BELUM tersambung (status=%d).\n", WiFi.status());
  }
}

bool netMqttConnected()    { return mqtt.connected(); }
bool netWifiStaticActive() { return wifiStaticActive; }
bool netOtaInProgress()    { return otaInProgress; }

// --- Setup -----------------------------------------------------------------

static void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // Alamat dipatok sebelum begin(): tanpa ini stack menjalankan DHCP dan IP
  // board bisa berubah tiap AP restart, yang bikin upload OTA meleset. Kalau
  // nilainya salah ketik atau ditolak driver, sengaja jatuh kembali ke DHCP
  // daripada board tidak bisa dijangkau sama sekali.
  IPAddress ip, gw, mask, dns;
  if (!ip.fromString(cfg.ip) || !gw.fromString(cfg.gateway) ||
      !mask.fromString(cfg.subnet) || !dns.fromString(cfg.dns)) {
    Serial.println("PERINGATAN: alamat static di konfigurasi tidak valid, pakai DHCP.");
  } else if (!WiFi.config(ip, gw, mask, dns)) {
    Serial.println("PERINGATAN: WiFi.config() ditolak, pakai DHCP.");
  } else {
    wifiStaticActive = true;
  }

  WiFi.begin(cfg.ssid, cfg.wifiPass);
  Serial.printf("Menyambung ke WiFi \"%s\"", cfg.ssid);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    // Watchdog tetap disuapi: menunggu 20 detik di sini tidak boleh dianggap
    // board menggantung.
    esp_task_wdt_reset();
    // Kedipan ikut dijalankan supaya penantian ini terlihat sebagai "sedang
    // mencari WiFi", bukan "board mati". Karena delay-nya 400 ms, iramanya di
    // sini memang tidak serapi 250/250 - itu hanya berlaku selama boot.
    ledUpdate();
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  netPrintStatus(Serial);
}

static void setupOTA() {
  ArduinoOTA.setHostname(cfg.otaHost);
  ArduinoOTA.setPassword(cfg.otaPass);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    // Hentikan dan matikan motor sebelum flash ditulis: firmware akan
    // di-restart, motor tidak boleh ditinggal berjalan.
    driveHardStop();
    steeringDetach();
    mmiCloseAll();
    Serial.println("\nOTA dimulai: motor dihentikan, PWM servo dilepas, MMI ditutup.");
    if (mqtt.connected()) {
      mqtt.publish(topicState(), "ota", true);
      mqtt.disconnect();
    }
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // ArduinoOTA.handle() tidak kembali ke loop() selama upload berlangsung,
    // jadi watchdog HARUS disuapi dari sini - kalau tidak, board reset di
    // tengah penulisan flash.
    esp_task_wdt_reset();
    Serial.printf("OTA %u%%\r", (progress * 100) / total);
  });

  ArduinoOTA.onEnd([]() { Serial.println("\nOTA selesai, board restart."); });

  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    Serial.printf("OTA ERROR [%u]\n", error);
    steeringBegin();     // pasang lagi PWM servo yang dilepas di onStart
  });

  ArduinoOTA.begin();
  Serial.printf("OTA siap (hostname %s).\n", cfg.otaHost);
}

void netBegin() {
  setupWiFi();
  setupOTA();

  mqtt.setServer(cfg.mqttHost, cfg.mqttPort);
  mqtt.setCallback(mqttOnMessage);
  mqtt.setBufferSize(768);       // bawaan 256 byte, kurang untuk baris status
  mqtt.setKeepAlive(15);         // deteksi putus lebih cepat untuk kendaraan

  logAddSink(mqttLogSink);
  mqttEnsureConnected();
}

void netLoop() {
  ArduinoOTA.handle();
  if (otaInProgress) return;
  mqttEnsureConnected();
}

void netServiceDelay(uint32_t ms) {
  uint32_t start = millis();
  do {
    esp_task_wdt_reset();
    ArduinoOTA.handle();
    mqttEnsureConnected();
    steeringUpdate();
    ledUpdate();
    delay(1);
  } while (millis() - start < ms);
}
