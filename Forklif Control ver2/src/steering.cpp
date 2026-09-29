#include "steering.h"

#include <math.h>

#include "config.h"
#include "hardware.h"

static int   pulseMinUs = 500;    // ~0 derajat   (bisa diubah: cal <min> <max>)
static int   pulseMaxUs = 2500;   // ~180 derajat

static float steerCurrent = 90.0f;
static float steerTarget  = 90.0f;
static uint32_t steerLastMs = 0;
static bool  servoReady = false;

// Kemudi digerakkan bertahap (slew), bukan lompat, supaya servo tidak menarik
// arus kejut dan mekanik kemudi tidak tersentak. Dari log v1, jeda total
// perintah -> kelengkungan nyata mencapai 0,65 detik; 12 ms/derajat terlalu
// lambat untuk navigasi otomatis dan 5 ms pun masih menyumbang ~0,2 detik pada
// ayunan 40 derajat, jadi dipakai 2 ms/derajat.
// JANGAN dihilangkan sama sekali: slew inilah yang mencegah sentakan itu.
static const uint32_t STEER_STEP_MS  = 2;
static const float    STEER_STEP_DEG = 1.0f;

static uint32_t usToDuty(int us) {
  const uint32_t periodUs = 1000000UL / LEDC_FREQ;   // 20000
  return (uint32_t)((uint64_t)us * ((1UL << LEDC_BITS) - 1) / periodUs);
}

static void writeMicros(int us) { ledcWrite(LEDC_CH, usToDuty(us)); }

// Sudut -> lebar pulsa. Dihitung dalam float lalu dibulatkan sekali di akhir;
// map() versi integer akan membuang justru pecahan 0,45 derajat yang menjadi
// alasan seluruh perubahan ini.
static void writeAngle(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  float us = pulseMinUs + (deg / 180.0f) * (float)(pulseMaxUs - pulseMinUs);
  writeMicros((int)lroundf(us));
}

// Terjemahkan sudut kemudi logis (kecil=kiri, besar=kanan) ke sudut servo
// fisik. Pembalikan roda belakang dilakukan HANYA di sini.
static void applySteer(float deg) {
  writeAngle(cfg.steerInvert ? (2.0f * cfg.steerCenter - deg) : deg);
}

void steeringBegin() {
  steerCurrent = cfg.steerCenter;
  steerTarget  = cfg.steerCenter;

  if (ledcSetup(LEDC_CH, LEDC_FREQ, LEDC_BITS) == 0) {
    Log.println("GAGAL: ledcSetup() ditolak (channel penuh / bit width > 14).");
    servoReady = false;
    return;
  }
  ledcAttachPin(PIN_SERVO, LEDC_CH);
  servoReady = true;
  applySteer(steerCurrent);
  Log.printf("Servo siap di GPIO%d, kemudi di tengah (%.2f derajat).\n",
             PIN_SERVO, (double)cfg.steerCenter);
}

void steeringUpdate() {
  if (!servoReady) return;

  float diff = steerTarget - steerCurrent;
  if (fabsf(diff) < 0.01f) {
    // Sisa pecahan diselesaikan sekali supaya current benar-benar sama dengan
    // target; tanpa ini nilai keduanya beda tipis selamanya dan telemetri
    // terlihat seperti kemudi yang tidak pernah sampai.
    if (steerCurrent != steerTarget) {
      steerCurrent = steerTarget;
      applySteer(steerCurrent);
    }
    return;
  }

  uint32_t now = millis();
  if (now - steerLastMs < STEER_STEP_MS) return;
  steerLastMs = now;

  float step = (fabsf(diff) < STEER_STEP_DEG) ? diff
                                              : (diff > 0 ? STEER_STEP_DEG : -STEER_STEP_DEG);
  steerCurrent += step;
  applySteer(steerCurrent);
}

void steeringSet(float deg) {
  steerTarget = constrain(deg, cfg.steerMin, cfg.steerMax);
}

void steeringCenter() { steerTarget = cfg.steerCenter; }

void steeringNudge(float delta) {
  steerTarget = constrain(steerTarget + delta, cfg.steerMin, cfg.steerMax);
}

bool steeringInvertToggle() {
  cfg.steerInvert = !cfg.steerInvert;
  applySteer(steerCurrent);      // langsung terlihat, tanpa menunggu slew
  return cfg.steerInvert;
}

float steeringCurrent() { return steerCurrent; }
float steeringTarget()  { return steerTarget; }
bool  steeringReady()   { return servoReady; }
int   steeringPulseMin(){ return pulseMinUs; }
int   steeringPulseMax(){ return pulseMaxUs; }

bool steeringCalibrate(int minUs, int maxUs, String &err) {
  if (minUs < 300 || maxUs > 3000 || minUs >= maxUs) {
    err = "rentang tidak masuk akal (min 300, max 3000, min < max)";
    return false;
  }
  pulseMinUs = minUs;
  pulseMaxUs = maxUs;
  applySteer(steerCurrent);
  return true;
}

void steeringRawPulse(int us) { writeMicros(us); }

void steeringDetach() {
  ledcDetachPin(PIN_SERVO);
  pinMode(PIN_SERVO, INPUT);
  servoReady = false;
}

void steeringApplyConfig() {
  // Target dijepit ulang: batas baru bisa saja lebih sempit dari sudut sekarang.
  steerTarget = constrain(steerTarget, cfg.steerMin, cfg.steerMax);
  if (servoReady) applySteer(steerCurrent);
}

void steeringPrint(Print &out) {
  out.printf("Kemudi  : %.2f derajat (target %.2f, batas %.2f-%.2f, lurus %.2f), arah %s\n",
             (double)steerCurrent, (double)steerTarget,
             (double)cfg.steerMin, (double)cfg.steerMax, (double)cfg.steerCenter,
             cfg.steerInvert ? "dibalik" : "normal");
  out.printf("Servo   : pulsa %d-%d us %s\n", pulseMinUs, pulseMaxUs,
             servoReady ? "" : "[PWM TIDAK AKTIF]");
}
