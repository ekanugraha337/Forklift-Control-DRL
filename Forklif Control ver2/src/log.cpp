#include "log.h"

static LogSink sinks[LOG_MAX_SINKS];
static size_t  sinkCount = 0;

// Penjaga rekursi: kalau sebuah sink ternyata ikut menulis ke Log, barisnya
// dibuang daripada menumpuk stack sampai board reset.
static bool inSink = false;

void logAddSink(LogSink sink) {
  if (sink == nullptr || sinkCount >= LOG_MAX_SINKS) return;
  sinks[sinkCount++] = sink;
}

size_t DualLog::write(uint8_t c) {
  Serial.write(c);

  if (c == '\n') {
    if (line_.length() && !inSink) {
      inSink = true;
      for (size_t i = 0; i < sinkCount; i++) sinks[i](line_.c_str());
      inSink = false;
    }
    line_ = "";
  } else if (c != '\r') {
    if (line_.length() < LOG_MAX_LINE) line_ += (char)c;
  }
  return 1;
}

size_t DualLog::write(const uint8_t *buf, size_t size) {
  for (size_t i = 0; i < size; i++) write(buf[i]);
  return size;
}

DualLog Log;
