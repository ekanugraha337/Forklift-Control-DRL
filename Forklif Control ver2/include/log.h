#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// LOG BERSAMA
//
// Satu objek Log dipakai seluruh firmware. Keluarannya selalu ke USB Serial,
// dan SELAIN itu diteruskan ke setiap "sink" yang mendaftar: MQTT (topik
// .../out/log) dan sesi MMI yang menyalakan "debug on".
//
// Log diterbitkan PER BARIS, bukan per karakter: satu publish MQTT untuk tiap
// byte akan membanjiri broker dan membuat board tersendat.
//
// Aturan pemakaian:
//   - Kejadian ASINKRON (estop, RC timeout, DIAG aktif, broker putus) ditulis
//     ke Log, supaya semua penonton melihatnya.
//   - BALASAN sebuah perintah ditulis ke Print& out milik pemanggil, supaya
//     jawabannya kembali ke orang yang mengetik, bukan ke semua orang.
//
// Sink TIDAK BOLEH menulis ke Log lagi - itu rekursi tanpa dasar.
// ---------------------------------------------------------------------------

typedef void (*LogSink)(const char *line);

static const size_t LOG_MAX_SINKS = 4;
static const size_t LOG_MAX_LINE  = 220;

void logAddSink(LogSink sink);

class DualLog : public Print {
 public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buf, size_t size) override;

 private:
  String line_;
};

extern DualLog Log;
