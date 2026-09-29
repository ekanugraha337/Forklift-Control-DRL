#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// JARINGAN - WiFi, MQTT, OTA
//
// Topik diambil dari config.h (dirakit dari ID robot):
//   <prefix>/in/cmd        <- perintah teks (estop, center, status, ...)
//   <prefix>/in/rc         <- "<langkah/detik bertanda> <sudut>", 20x per detik
//   <prefix>/out/telemetry -> JSON, 4x per detik
//   <prefix>/out/log       -> baris log
//   <prefix>/out/state     -> online / offline / ota (retained)
// ---------------------------------------------------------------------------

void netBegin();

// Dipanggil tiap loop: melayani OTA dan MQTT (termasuk sambung ulang).
void netLoop();

void netPublishTelemetry();

// True selama flash sedang ditulis OTA: loop() harus berhenti menyentuh apa pun.
bool netOtaInProgress();

bool netMqttConnected();
bool netWifiStaticActive();

void netPrintStatus(Print &out);

// Pengganti delay() yang tetap melayani OTA, MQTT, kemudi, dan watchdog.
void netServiceDelay(uint32_t ms);
