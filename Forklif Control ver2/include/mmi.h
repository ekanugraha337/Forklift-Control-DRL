#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// MMI - Man Machine Interface
//
// CLI untuk Setup Configuration, Show Configuration, dan Debug. Diakses lewat
// Telnet TCP port 23, dan perintah yang sama juga berlaku di USB Serial serta
// di topik MQTT <prefix>/in/cmd.
//
// Balasan perintah selalu dikirim ke pengirimnya saja (Print &out). Aliran log
// kejadian hanya sampai ke sesi telnet yang menyalakan "debug on" - kalau tidak,
// dua orang yang sedang menyetel board akan saling menimpa layar.
// ---------------------------------------------------------------------------

static const int MMI_PORT        = 23;
static const int MMI_MAX_CLIENTS = 2;

void mmiBegin();
void mmiLoop();

// Jalankan satu baris perintah. Dipakai telnet, USB Serial, dan MQTT.
void mmiRunCommand(const String &line, Print &out);

// Tutup semua sesi (dipakai sebelum OTA menulis flash).
void mmiCloseAll();

int mmiClientCount();
