#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// PETA PIN - Waveshare ESP32-S3-Nano
//
// Nomor di bawah adalah nomor GPIO ESP32-S3 APA ADANYA, karena platformio.ini
// memakai profil board generik esp32-s3-devkitc-1, bukan arduino_nano_esp32
// (profil itu me-remap pin ala Arduino dan memakai upload protocol DFU yang
// tidak ada di papan Waveshare).
//
// Catatan penting:
//   - LEDC pada ESP32-S3 maksimum 14 bit. Lebih dari itu ledcSetup() gagal dan
//     PWM servo tidak keluar sama sekali.
//   - GPIO3 adalah strapping pin (pemilih sumber JTAG). Aman sebagai output
//     setelah boot, tapi jangan diberi pull-up/pull-down kuat di luar.
// ---------------------------------------------------------------------------

// --- Servo kemudi MG995 ----------------------------------------------------
static const int PIN_SERVO      = 7;    // PWM

// --- Stepper NEMA17 lewat driver TMC2209 -----------------------------------
static const int PIN_STEP       = 3;
static const int PIN_DIR        = 4;
static const int PIN_EN         = 2;    // aktif LOW
// INDEX: bawaannya (GCONF.index_otpw = 0) pulsa terbit tiap kali penghitung
// microstep melewati nol, yaitu SEKALI PER PERIODE LISTRIK = 4 langkah penuh.
// Untuk motor 1,8 derajat (200 langkah/putaran) berarti 50 pulsa per putaran,
// BUKAN satu pulsa per langkah. Jadi indexCount x 4 = langkah penuh yang
// benar-benar diproses driver.
//
// Yang dihitung tetap langkah yang DIPERINTAHKAN, bukan putaran poros: pulsa ini
// dibangkitkan penghitung internal driver saat menerima STEP, jadi berguna untuk
// memastikan pulsa STEP sampai - bukan untuk mengetahui roda benar-benar berputar.
static const int PIN_INDEX      = 8;    // input
static const int PIN_DIAG       = 6;    // input, aktif saat stall/overtemp/short

// --- UART ke TMC2209 (PDN_UART) -------------------------------------------
// Keduanya menuju SATU pin yang sama di driver (PDN_UART). Inilah sambungan
// "single wire" baku TMC2209: jalur TX diberi resistor 1K sebagai pembatas
// arus supaya MCU dan driver tidak saling mendorong saat driver membalas,
// sedangkan jalur RX menyadap titik yang sama tanpa resistor.
static const int PIN_TMC_TX     = 17;   // -> resistor 1K -> PDN_UART
static const int PIN_TMC_RX     = 18;   // -> PDN_UART

// MS1 dan MS2 di papan ini DIJUMPER KE 3.3V. Dalam mode standalone itu berarti
// microstepping 1/16. Begitu UART dipakai, kedua pin berganti fungsi menjadi
// pemilih alamat slave: MS1 = bit0, MS2 = bit1, jadi keduanya HIGH = alamat 3.
// Kalau alamat ini salah, seluruh pembacaan UART kembali 0xFF dan
// test_connection() gagal walau kabelnya benar.
static const uint8_t TMC_UART_ADDR = 0b11;

// Resistor sense pada modul TMC2209 lazim (BigTreeTech / FYSETC) = 0,11 ohm.
// Angka ini dipakai library untuk menghitung arus RMS; kalau modulnya lain,
// nilainya harus ikut diubah atau arus motor akan meleset.
static const float TMC_R_SENSE  = 0.11f;

static const uint32_t TMC_UART_BAUD = 115200;

// --- Servo: parameter LEDC -------------------------------------------------
static const int LEDC_CH        = 0;
static const int LEDC_FREQ      = 50;   // 50 Hz -> periode 20000 us
static const int LEDC_BITS      = 14;   // batas maksimum ESP32-S3
