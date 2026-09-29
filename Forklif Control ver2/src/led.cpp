#include "led.h"

#include "config.h"

// Periode dan duty cycle yang diminta: 500 ms, 50%.
// Keduanya sengaja tetap (bukan setelan NVS) supaya kedipannya punya satu arti
// yang selalu sama: "board masih hidup". Kalau nanti perlu diubah, cukup dua
// angka di bawah ini.
static const uint32_t LED_PERIOD_MS = 500;
static const uint32_t LED_DUTY_PCT  = 50;
static const uint32_t LED_ON_MS     = LED_PERIOD_MS * LED_DUTY_PCT / 100;   // 250

// Kecerahan saat memakai WS2812. Tidak penuh 255: LED NeoPixel menyilaukan dan
// menarik arus tak perlu dari rel yang sama dengan servo.
static const uint8_t LED_RGB_LEVEL = 32;

static bool     ledState  = false;
static bool     ledOnPin  = false;   // true bila pin sudah pernah disiapkan
static uint32_t ledEpochMs = 0;

static void ledWrite(bool on) {
  bool level = cfg.ledInvert ? !on : on;

  if (cfg.ledRgb) {
    // neopixelWrite() mengurus protokol WS2812 sendiri, termasuk pinMode-nya.
    uint8_t v = level ? LED_RGB_LEVEL : 0;
    neopixelWrite(cfg.ledPin, v, v, v);
    return;
  }

  digitalWrite(cfg.ledPin, level ? HIGH : LOW);
}

void ledBegin() {
  ledEpochMs = millis();
  ledApplyConfig();
}

void ledApplyConfig() {
  if (!cfg.ledRgb) {
    pinMode(cfg.ledPin, OUTPUT);
  }
  ledOnPin = true;
  ledWrite(ledState);
}

void ledUpdate() {
  if (!ledOnPin) return;

  // Fase dihitung dari selisih waktu, bukan dengan menumpuk penambahan tiap
  // kedipan. Cara ini tidak menghanyutkan periode walau loop() sesekali
  // tersendat (misalnya saat percobaan sambung MQTT), dan tetap benar ketika
  // millis() berputar kembali ke nol setelah ~49 hari.
  uint32_t phase = (millis() - ledEpochMs) % LED_PERIOD_MS;
  bool on = (phase < LED_ON_MS);

  if (on == ledState) return;    // tidak ada yang berubah, jangan sentuh pin
  ledState = on;
  ledWrite(ledState);
}

void ledPrint(Print &out) {
  out.printf("LED     : GPIO%u %s%s, kedip %lu ms duty %lu%% (%s)\n",
             (unsigned)cfg.ledPin,
             cfg.ledRgb ? "WS2812" : "biasa",
             cfg.ledInvert ? ", aktif LOW" : "",
             (unsigned long)LED_PERIOD_MS, (unsigned long)LED_DUTY_PCT,
             ledState ? "sedang menyala" : "sedang mati");
}
