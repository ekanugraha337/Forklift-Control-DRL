#pragma once

#include <Arduino.h>

#include "log.h"

// ---------------------------------------------------------------------------
// KEMUDI - servo MG995 di roda belakang
//
// Sudut kemudi bertipe FLOAT di seluruh jalur, sampai ke perhitungan lebar
// pulsa. Itu perbedaan pokok dengan v1: titik lurus robot ini 89,45 derajat,
// dan di v1 angka itu harus dipecah menjadi sudut bulat 89 plus trim pulsa 5 us
// karena sudutnya integer. Di sini 89,45 dipakai langsung.
//
// Resolusi pulsa tetap cukup: LEDC 14 bit pada 50 Hz = 1,22 us per hitungan,
// sedangkan 0,45 derajat = 5 us = 4 hitungan.
// ---------------------------------------------------------------------------

void  steeringBegin();

// Dipanggil terus dari loop(): menggeser kemudi setahap menuju target.
void  steeringUpdate();

// Target absolut, dibatasi str_min..str_max.
void  steeringSet(float deg);
void  steeringCenter();
void  steeringNudge(float delta);

bool  steeringInvertToggle();
float steeringCurrent();
float steeringTarget();
bool  steeringReady();

// Rentang pulsa servo (us). Dipakai perintah "cal".
bool  steeringCalibrate(int minUs, int maxUs, String &err);
int   steeringPulseMin();
int   steeringPulseMax();

// Pulsa mentah tanpa terjemahan sudut, untuk mengukur servo. Sengaja TIDAK
// mengubah target: begitu steeringUpdate() bergerak lagi, sudut normal kembali.
void  steeringRawPulse(int us);

// Lepas PWM sebelum OTA menulis flash.
void  steeringDetach();

// Dipanggil setelah konfigurasi kemudi berubah lewat MMI, supaya batas baru
// langsung berlaku tanpa restart.
void  steeringApplyConfig();

void  steeringPrint(Print &out);
