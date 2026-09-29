#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// LED TANDA HIDUP (heartbeat)
//
// Berkedip tetap: periode 500 ms, duty cycle 50% - 250 ms menyala, 250 ms mati.
//
// Gunanya bukan hiasan: selama LED masih berkedip, loop() masih berputar dan
// watchdog masih disuapi. LED yang membeku menyala atau mati terus berarti
// board menggantung, dan itu terlihat dari seberang ruangan tanpa perlu telnet
// maupun USB.
//
// Pin, polaritas, dan jenis LED diambil dari konfigurasi karena profil board
// yang dipakai generik - lihat catatan di defaults.h.
// ---------------------------------------------------------------------------

void ledBegin();

// Dipanggil sesering mungkin dari loop(). Murah: hanya menulis ke pin saat
// keadaan berubah, yaitu 4 kali per detik.
void ledUpdate();

// Dipanggil setelah pin/polaritas/jenis LED diubah lewat MMI.
void ledApplyConfig();

void ledPrint(Print &out);
