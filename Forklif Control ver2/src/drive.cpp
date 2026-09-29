#include "drive.h"

#include <FastAccelStepper.h>
#include <TMCStepper.h>

#include "config.h"
#include "hardware.h"
#include "steering.h"

// --- Stepper ---------------------------------------------------------------

static FastAccelStepperEngine engine = FastAccelStepperEngine();
static FastAccelStepper *stepper = nullptr;

// true bila motor sedang jalan terus-menerus (fwd/rev), bukan gerak terukur.
static bool continuousDrive = false;

// Kecepatan yang SEDANG diperintahkan ke motor. Sengaja dipisah dari
// cfg.driveSpeed: cfg adalah nilai bawaan yang tersimpan di NVS, sedangkan
// perintah "rc" mengganti kecepatan puluhan kali per detik. Kalau keduanya satu
// variable, sekali "save" akan mengabadikan kecepatan stik gamepad yang
// kebetulan terakhir lewat sebagai setelan permanen board.
// Hanya perintah "speed" yang sengaja mengubah keduanya.
static uint32_t driveSpeedHz = 0;

// --- Mode RC ---------------------------------------------------------------
// Kalau aliran perintah berhenti (gamepad dicabut, WiFi putus, skrip mati),
// motor harus berhenti sendiri.
static const uint32_t RC_TIMEOUT_MS = 500;
static const uint32_t RC_MIN_SPS    = 100;   // di bawah ini dianggap netral
static int      rcDir    = 0;                // -1 mundur, 0 diam, +1 maju
static uint32_t rcLastMs = 0;
static bool     rcActive = false;

static volatile uint32_t indexCount = 0;

// diagFlag LEKAT: sekali aktif tetap 1 sampai "zero", dipakai telemetri
// (diag_ev) sebagai "pernah ada gangguan sejak terakhir dinolkan".
// diagPending sekali pakai: dikonsumsi driveDiagService() supaya setiap kejadian
// baru dilaporkan, bukan hanya yang pertama sepanjang umur board.
static volatile bool diagFlag    = false;
static volatile bool diagPending = false;

static void IRAM_ATTR onIndexPulse() { indexCount++; }
static void IRAM_ATTR onDiagRise()   { diagFlag = true; diagPending = true; }

// --- TMC2209 lewat UART ----------------------------------------------------

static TMC2209Stepper driver(&Serial1, TMC_R_SENSE, TMC_UART_ADDR);
static bool     tmcOk    = false;
static uint8_t  tmcFlags = 0;
static uint8_t  tmcCsActual = 0;
static uint32_t tmcLastPollMs = 0;
static const uint32_t TMC_POLL_MS  = 500;
static const uint32_t TMC_RETRY_MS = 5000;

// Osilator internal TMC2209. Dipakai menerjemahkan TSTEP <-> langkah/detik.
// Nilainya tipikal, bukan terkalibrasi (toleransi pabrik beberapa persen), jadi
// laju hasil ukur di bawah berguna sebagai pemeriksaan kasar - bukan alat ukur.
static const uint32_t TMC_FCLK_HZ = 12000000UL;

static uint16_t tmcMscnt    = 0;
static uint32_t tmcTstep    = 0xFFFFF;
static uint16_t tmcSgResult = 0;

// --- Deteksi stall ---------------------------------------------------------
static uint32_t stallCount    = 0;
static bool     stallEvent    = false;   // lekat, dinolkan oleh "zero"
static uint32_t diagLastLogMs = 0;
static const uint32_t DIAG_LOG_MIN_MS = 2000;   // jangan membanjiri log

// --- Pemeriksaan integritas STEP ------------------------------------------
// MSCNT maju satu hitungan per microstep 1/256, jadi satu langkah kita bernilai
// 256/microstep hitungan. Fasa yang diharapkan dihitung dari posisi perintah
// FastAccelStepper; selisihnya terhadap MSCNT yang sebenarnya = pulsa STEP yang
// tidak sampai ke driver.
static bool     phaseRefValid   = false;
static uint16_t phaseRefMscnt   = 0;
static long     phaseRefPos     = 0;
static long     phaseLastPos    = 0;
static int      phaseError      = 0;
static bool     phaseErrorKnown = false;

// Arah MSCNT relatif arah posisi. 0 = belum dipastikan. Dideteksi sendiri pada
// pemeriksaan pertama, karena polaritas DIR (dan bit GCONF.shaft) menentukan
// apakah MSCNT naik atau turun saat posisi naik - tanpa ini, pemasangan kabel
// DIR yang terbalik akan terlihat seperti pulsa STEP hilang terus-menerus.
static int      phaseSign       = 0;

// true bila pengambilan acuan diminta saat motor masih jalan, jadi harus
// diulang begitu berhenti.
static bool     phaseNeedCapture = false;

static uint32_t mscntUnitsPerStep() {
  uint16_t ms = cfg.tmcMicrosteps ? cfg.tmcMicrosteps : 16;
  return 256u / ms;
}

static uint32_t spsFromTstep(uint32_t tstep) {
  if (tstep == 0 || tstep >= 0xFFFFF) return 0;   // 0xFFFFF = diam / sangat lambat
  uint64_t d = (uint64_t)tstep * mscntUnitsPerStep();
  if (d == 0) return 0;
  return (uint32_t)(TMC_FCLK_HZ / d);
}

static uint32_t tstepFromSps(uint32_t sps) {
  uint64_t d = (uint64_t)sps * mscntUnitsPerStep();
  if (d == 0) return 0xFFFFF;
  uint64_t t = TMC_FCLK_HZ / d;
  return (uint32_t)(t > 0xFFFFF ? 0xFFFFF : t);
}

// --- Pembantu --------------------------------------------------------------

static bool stepperReady(Print &out) {
  if (stepper == nullptr) {
    out.println("GAGAL: stepper belum terinisialisasi.");
    return false;
  }
  return true;
}

// Terjemahkan kode balikan move()/runForward() jadi pesan yang bisa dibaca.
static void reportMoveResult(MoveResultCode rc, Print &out) {
  switch (rc) {
    case MOVE_OK: return;
    case MOVE_ERR_NO_DIRECTION_PIN:
      out.println("GAGAL: arah mundur diminta tapi pin DIR belum diset."); break;
    case MOVE_ERR_SPEED_IS_UNDEFINED:
      out.println("GAGAL: kecepatan belum diset (pakai: speed <sps>)."); break;
    case MOVE_ERR_ACCELERATION_IS_UNDEFINED:
      out.println("GAGAL: akselerasi belum diset (pakai: accel <sps2>)."); break;
    default:
      out.printf("GAGAL: %s\n", toString(rc)); break;
  }
}

// Ambil titik acuan fasa. Dipanggil saat UART tersambung dan setiap "zero".
//
// Kalau motor sedang jalan, pengambilan DITUNDA sampai berhenti: MSCNT yang
// dibaca di tengah gerakan belum menetap (interpolasi MicroPlyer masih menyusul),
// dan acuan yang meleset sedikit akan muncul sebagai peringatan "pulsa STEP
// hilang" yang palsu di setiap pemeriksaan berikutnya.
static void phaseCapture() {
  if (!tmcOk) { phaseRefValid = false; return; }
  if (driveRunning()) { phaseNeedCapture = true; return; }

  phaseNeedCapture = false;
  tmcMscnt        = driver.MSCNT();
  phaseRefMscnt   = tmcMscnt;
  phaseRefPos     = drivePosition();
  phaseLastPos    = phaseRefPos;
  phaseRefValid   = true;
  phaseErrorKnown = false;
  phaseError      = 0;
}

static long phaseExpected(long pos, int sign) {
  // int64 dipakai sengaja: posisi bisa mencapai ratusan ribu langkah dalam satu
  // sesi, dan dikali 16 hitungan per langkah angka itu mendekati batas long
  // 32 bit. Meluap di sini akan memunculkan "pulsa STEP hilang" yang palsu.
  int64_t delta = (int64_t)(pos - phaseRefPos) * (int64_t)mscntUnitsPerStep();
  int64_t e = ((int64_t)phaseRefMscnt + (int64_t)sign * delta) % 1024;
  if (e < 0) e += 1024;
  return (long)e;
}

static int phaseWrap(long diff) {
  diff %= 1024;
  if (diff > 512)  diff -= 1024;
  if (diff < -512) diff += 1024;
  return (int)diff;
}

// Hanya dipanggil saat motor DIAM: interpolasi MicroPlyer di dalam driver masih
// menyusul target selama motor bergerak, jadi selisih saat jalan tidak berarti
// pulsa hilang. Diam juga satu-satunya saat MSCNT pasti sudah menetap.
static void phaseCheck() {
  if (!phaseRefValid || !tmcOk) return;

  long pos = drivePosition();
  tmcMscnt = driver.MSCNT();

  int errPlus  = phaseWrap((long)tmcMscnt - phaseExpected(pos, +1));
  int errMinus = phaseWrap((long)tmcMscnt - phaseExpected(pos, -1));

  if (phaseSign == 0) {
    // Jangan menyimpulkan arah dari perpindahan yang terlalu kecil: pada
    // perpindahan kelipatan 4 langkah penuh kedua arah memberi selisih nol.
    if (errPlus == 0 && errMinus != 0)      phaseSign = +1;
    else if (errMinus == 0 && errPlus != 0) {
      phaseSign = -1;
      Log.println("TMC2209: MSCNT bergerak berlawanan arah posisi (polaritas DIR "
                  "atau bit GCONF.shaft) - pemeriksaan fasa disesuaikan.");
    }
  }

  phaseError      = (phaseSign < 0) ? errMinus : errPlus;
  phaseErrorKnown = true;

  if (phaseError != 0 && millis() - diagLastLogMs >= DIAG_LOG_MIN_MS) {
    diagLastLogMs = millis();
    Log.printf("PERINGATAN integritas STEP: fasa driver meleset %d microstep "
               "(1/256) dari posisi perintah - ada pulsa STEP yang tidak "
               "sampai. Cek kabel STEP GPIO%d dan derau di sekitarnya.\n",
               phaseError, PIN_STEP);
  }
}

// --- Inisialisasi ----------------------------------------------------------

// Kirim seluruh setelan driver dari konfigurasi. Dipisah supaya bisa dipanggil
// ulang setelah "set tmc_ma ..." tanpa perlu restart.
void driveApplyConfig() {
  if (stepper) {
    stepper->setAcceleration(cfg.driveAccel);
    stepper->applySpeedAcceleration();
  }

  if (!tmcOk) return;

  // PDN_UART harus dinonaktifkan sebagai input power-down begitu pin itu dipakai
  // untuk UART, dan microstepping harus dipindah ke register - kalau tidak,
  // MS1/MS2 (keduanya 3.3V = 1/16) yang tetap menentukan dan microsteps() di
  // bawah diabaikan diam-diam. Keduanya juga diset oleh begin(), diulang di sini
  // supaya tidak bergantung pada versi library.
  driver.pdn_disable(true);
  driver.mstep_reg_select(true);

  // Arus diambil dari register, bukan dari trimpot VREF. Inilah yang membuat
  // "tmc_ma" di konfigurasi benar-benar menentukan arus motor.
  driver.I_scale_analog(false);

  driver.toff(5);              // 0 = driver mati; wajib bukan nol
  driver.blank_time(24);
  driver.rms_current(cfg.tmcCurrent);
  driver.microsteps(cfg.tmcMicrosteps);

  if (cfg.tmcSpreadCycle) {
    driver.en_spreadCycle(true);
  } else {
    driver.en_spreadCycle(false);
    driver.pwm_autoscale(true);   // wajib untuk stealthChop
  }

  // StallGuard4. TCOOLTHRS adalah ambang KECEPATAN dalam satuan TSTEP, dan
  // karena TSTEP mengecil saat kecepatan naik, syaratnya TSTEP <= TCOOLTHRS -
  // artinya deteksi baru hidup DI ATAS sg_min_sps.
  if (cfg.sgThrs > 0) {
    driver.SGTHRS(cfg.sgThrs);
    driver.TCOOLTHRS(tstepFromSps(cfg.sgMinSps));
    if (cfg.tmcSpreadCycle) {
      Log.println("PERINGATAN: sg_thrs diset tapi spread=1. StallGuard4 pada "
                  "TMC2209 hanya bekerja di stealthChop, jadi deteksi stall "
                  "TIDAK aktif. Pakai \"set spread 0\" bila memang diinginkan.");
    }
  } else {
    driver.SGTHRS(0);
    driver.TCOOLTHRS(0);
  }

  // Microstepping boleh berubah lewat MMI, dan itu mengubah jumlah hitungan
  // MSCNT per langkah - acuan fasa yang lama tidak lagi berlaku.
  phaseSign = 0;
  phaseCapture();
}

static bool tmcConnect() {
  driver.begin();
  uint8_t result = driver.test_connection();   // 0 = OK
  tmcOk = (result == 0);
  if (!tmcOk) {
    Log.printf("TMC2209 TIDAK menjawab di UART (test_connection=%u, alamat %u). "
               "Cek kabel GPIO%d/GPIO%d ke PDN_UART, resistor 1K, dan jumper MS1/MS2.\n",
               result, (unsigned)TMC_UART_ADDR, PIN_TMC_TX, PIN_TMC_RX);
    return false;
  }
  Log.printf("TMC2209 tersambung (versi register 0x%02X).\n", driver.version());
  driveApplyConfig();
  Log.printf("Driver: %u mA RMS, 1/%u microstep, %s.\n",
             (unsigned)cfg.tmcCurrent, (unsigned)cfg.tmcMicrosteps,
             cfg.tmcSpreadCycle ? "spreadCycle" : "stealthChop");
  return true;
}

void driveBegin() {
  pinMode(PIN_INDEX, INPUT);
  pinMode(PIN_DIAG, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_INDEX), onIndexPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_DIAG), onDiagRise, RISING);

  Serial1.begin(TMC_UART_BAUD, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);

  driveSpeedHz = cfg.driveSpeed;

  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  if (stepper == nullptr) {
    Log.printf("GAGAL: tidak bisa memakai GPIO%d sebagai STEP.\n", PIN_STEP);
  } else {
    stepper->setDirectionPin(PIN_DIR);
    stepper->setEnablePin(PIN_EN, true);   // true = aktif LOW, sesuai TMC2209
    stepper->setAutoEnable(true);          // driver hidup hanya saat bergerak
    stepper->setSpeedInHz(driveSpeedHz);
    stepper->setAcceleration(cfg.driveAccel);
    Log.printf("Stepper siap: STEP=GPIO%d DIR=GPIO%d EN=GPIO%d\n",
               PIN_STEP, PIN_DIR, PIN_EN);
  }

  tmcConnect();
}

// --- Perintah --------------------------------------------------------------

void driveForward(uint32_t sps, Print &out) {
  if (!stepperReady(out)) return;
  driveSpeedHz = sps;
  stepper->setSpeedInHz(sps);
  stepper->setAcceleration(cfg.driveAccel);
  stepper->applySpeedAcceleration();
  stepper->enableOutputs();
  MoveResultCode rc = stepper->runForward();
  reportMoveResult(rc, out);
  if (rc != MOVE_OK) return;
  continuousDrive = true;
  out.printf("MAJU terus, %lu langkah/detik.\n", (unsigned long)sps);
}

void driveBackward(uint32_t sps, Print &out) {
  if (!stepperReady(out)) return;
  driveSpeedHz = sps;
  stepper->setSpeedInHz(sps);
  stepper->setAcceleration(cfg.driveAccel);
  stepper->applySpeedAcceleration();
  stepper->enableOutputs();
  MoveResultCode rc = stepper->runBackward();
  reportMoveResult(rc, out);
  if (rc != MOVE_OK) return;
  continuousDrive = true;
  out.printf("MUNDUR terus, %lu langkah/detik.\n", (unsigned long)sps);
}

void driveMove(long steps, Print &out) {
  if (!stepperReady(out)) return;
  stepper->setSpeedInHz(driveSpeedHz);
  stepper->setAcceleration(cfg.driveAccel);
  stepper->applySpeedAcceleration();
  stepper->enableOutputs();
  continuousDrive = false;
  MoveResultCode rc = stepper->move(steps);
  reportMoveResult(rc, out);
  if (rc != MOVE_OK) return;
  out.printf("Gerak %ld langkah (%s).\n", steps, steps >= 0 ? "maju" : "mundur");
}

void driveStop(Print &out) {
  if (!stepperReady(out)) return;
  rcActive = false;
  rcDir = 0;
  continuousDrive = false;
  stepper->stopMove();           // berhenti dengan deselerasi
  out.println("STOP (deselerasi).");
}

void driveEmergencyStop(Print &out) {
  if (!stepperReady(out)) return;
  rcActive = false;
  rcDir = 0;
  continuousDrive = false;
  stepper->forceStop();          // berhenti mendadak, tanpa ramp
  stepper->disableOutputs();
  out.println("EMERGENCY STOP: motor berhenti mendadak, driver dimatikan.");
}

void driveEnable(Print &out) {
  if (!stepperReady(out)) return;
  stepper->enableOutputs();
  out.println("Driver TMC2209 dihidupkan.");
}

void driveDisable(Print &out) {
  if (!stepperReady(out)) return;
  continuousDrive = false;
  rcActive = false;
  rcDir = 0;
  stepper->forceStop();
  stepper->disableOutputs();
  out.println("Driver TMC2209 dimatikan (motor bebas berputar).");
}

void driveZero(Print &out) {
  if (stepper) stepper->setCurrentPosition(0);
  indexCount = 0;
  diagFlag = false;
  stallEvent = false;
  stallCount = 0;
  // Acuan fasa ikut diambil ulang: posisi perintah baru saja dipindah ke nol,
  // jadi acuan lama akan melaporkan selisih sebesar posisi yang dibuang itu.
  phaseCapture();
  out.println("Posisi, hitungan INDEX, dan penanda stall dinolkan.");
}

bool driveSetSpeed(uint32_t sps, Print &out) {
  if (sps == 0) { out.println("Kecepatan harus > 0."); return false; }
  // Perintah eksplisit: ubah yang sedang berlaku SEKALIGUS nilai bawaan, supaya
  // "speed 9000" lalu "save" benar-benar mengabadikan 9000.
  driveSpeedHz = sps;
  cfg.driveSpeed = sps;
  if (stepper) { stepper->setSpeedInHz(sps); stepper->applySpeedAcceleration(); }
  out.printf("Kecepatan = %lu langkah/detik (\"save\" untuk menyimpan).\n",
             (unsigned long)sps);
  return true;
}

bool driveSetAccel(uint32_t accel, Print &out) {
  if (accel == 0) { out.println("Akselerasi harus > 0."); return false; }
  cfg.driveAccel = accel;
  if (stepper) { stepper->setAcceleration(accel); stepper->applySpeedAcceleration(); }
  out.printf("Akselerasi = %lu langkah/detik^2.\n", (unsigned long)accel);
  return true;
}

// --- RC --------------------------------------------------------------------

void driveRc(long sps, float angle) {
  rcLastMs = millis();
  rcActive = true;
  steeringSet(angle);

  if (stepper == nullptr) return;

  int dir = (sps > 0) - (sps < 0);
  uint32_t mag = (uint32_t)labs(sps);

  if (dir == 0 || mag < RC_MIN_SPS) {        // stik di posisi netral
    if (rcDir != 0) {
      stepper->stopMove();
      rcDir = 0;
      continuousDrive = false;
    }
    return;
  }

  driveSpeedHz = mag;
  stepper->setSpeedInHz(mag);
  stepper->setAcceleration(cfg.driveAccel);

  // Motor disuruh jalan lagi bukan hanya saat ARAH berubah, tapi juga bila
  // stepper ternyata diam padahal arah yang diminta sama. Tanpa cek isRunning()
  // itu, sekali motor berhenti karena sebab lain (estop, stall, putus broker)
  // sementara rcDir masih +-1, semua perintah rc berikutnya hanya jatuh ke
  // applySpeedAcceleration() pada ramp yang sudah mati: robot membeku permanen
  // walau perintah tetap mengalir 20x per detik.
  if (dir != rcDir || !stepper->isRunning()) {
    // estop mematikan driver lewat disableOutputs(); tanpa dihidupkan lagi di
    // sini, runForward/runBackward diterima (MOVE_OK, rcDir ikut terkunci) tapi
    // motor tidak pernah melangkah.
    stepper->enableOutputs();
    MoveResultCode rc = (dir > 0) ? stepper->runForward() : stepper->runBackward();
    if (rc != MOVE_OK) { reportMoveResult(rc, Log); return; }
    rcDir = dir;
    continuousDrive = true;
  } else {                                   // hanya kecepatan yang berubah
    stepper->applySpeedAcceleration();
  }
}

void driveRcWatchdog() {
  if (!rcActive || millis() - rcLastMs <= RC_TIMEOUT_MS) return;
  rcActive = false;
  if (rcDir != 0 && stepper) stepper->stopMove();
  rcDir = 0;
  continuousDrive = false;
  steeringCenter();
  Log.println("RC timeout: perintah gamepad berhenti -> motor stop, kemudi ke tengah.");
}

void driveSafeStop() {
  if (continuousDrive && stepper) stepper->stopMove();
  continuousDrive = false;
  rcActive = false;
  rcDir = 0;
}

void driveHardStop() {
  if (stepper) { stepper->forceStop(); stepper->disableOutputs(); }
  continuousDrive = false;
  rcActive = false;
  rcDir = 0;
}

// --- Keadaan ---------------------------------------------------------------

bool     driveRunning()     { return stepper && stepper->isRunning(); }
long     drivePosition()    { return stepper ? (long)stepper->getCurrentPosition() : 0; }
long     driveCurrentSps()  { return stepper ? (long)(stepper->getCurrentSpeedInMilliHz() / 1000) : 0; }
uint32_t driveTargetSps()   { return driveSpeedHz; }

// "none" sengaja dibedakan dari "idle": stepper yang gagal diinisialisasi tidak
// boleh terlihat sama dengan motor yang sekadar sedang diam.
const char *driveStateName() {
  if (stepper == nullptr) return "none";
  return stepper->isRunning() ? "run" : "idle";
}
int      driveDir()         { return rcDir; }
bool     driveRcActive()    { return rcActive; }
bool     driveContinuous()  { return continuousDrive; }
uint32_t driveIndexCount()  { return indexCount; }
bool     driveDiagPin()     { return digitalRead(PIN_DIAG) != 0; }
bool     driveDiagEvent()   { return diagFlag; }

// --- TMC: pemantauan -------------------------------------------------------

void driveTmcPoll() {
  uint32_t now = millis();
  uint32_t interval = tmcOk ? TMC_POLL_MS : TMC_RETRY_MS;
  if (now - tmcLastPollMs < interval) return;
  tmcLastPollMs = now;

  if (!tmcOk) { tmcConnect(); return; }

  // DRV_STATUS dibaca SEKALI lalu bit-nya diurai sendiri. Memakai accessor
  // driver.ot(), driver.otpw(), dan seterusnya akan menembakkan satu transaksi
  // UART untuk tiap bendera - enam kali kerja untuk satu register yang sama.
  uint32_t st = driver.DRV_STATUS();

  // Semua bit 1 berarti tidak ada yang menjawab: tandai putus supaya rutin ini
  // mencoba menyambung ulang, bukan melaporkan alarm palsu.
  if (st == 0xFFFFFFFFUL) {
    tmcOk = false;
    tmcFlags = 0;
    Log.println("TMC2209: UART berhenti menjawab, mencoba menyambung ulang.");
    return;
  }

  uint8_t flags = 0;
  if (st & (1UL << 0))  flags |= DRV_FLAG_OTPW;
  if (st & (1UL << 1))  flags |= DRV_FLAG_OT;
  if (st & (3UL << 2))  flags |= DRV_FLAG_SHORT;   // s2ga | s2gb
  if (st & (3UL << 4))  flags |= DRV_FLAG_OPEN;    // ola  | olb
  if (st & (1UL << 8))  flags |= DRV_FLAG_STEALTH;
  tmcCsActual = (uint8_t)((st >> 16) & 0x1F);

  // Laporkan hanya saat bendera BERUBAH, jangan tiap 500 ms.
  static uint8_t lastReported = 0;
  uint8_t alarm = flags & (DRV_FLAG_OTPW | DRV_FLAG_OT | DRV_FLAG_SHORT | DRV_FLAG_OPEN);
  uint8_t lastAlarm = lastReported & (DRV_FLAG_OTPW | DRV_FLAG_OT | DRV_FLAG_SHORT | DRV_FLAG_OPEN);
  if (alarm != lastAlarm) {
    if (alarm == 0) {
      Log.println("TMC2209: kondisi kembali normal.");
    } else {
      Log.printf("TMC2209 PERINGATAN:%s%s%s%s\n",
                 (alarm & DRV_FLAG_OT)    ? " OVERTEMP(driver mati)" : "",
                 (alarm & DRV_FLAG_OTPW)  ? " suhu-tinggi" : "",
                 (alarm & DRV_FLAG_SHORT) ? " SHORT-ke-ground" : "",
                 (alarm & DRV_FLAG_OPEN)  ? " kumparan-terbuka" : "");
    }
  }
  lastReported = flags;
  tmcFlags = flags;

  // SG_RESULT hanya dibaca bila deteksi stall benar-benar aktif; di spreadCycle
  // angkanya tidak berarti apa-apa, jadi transaksi UART-nya sia-sia.
  if (driveStallArmed()) tmcSgResult = driver.SG_RESULT();

  tmcTstep = driver.TSTEP();

  // Pemeriksaan fasa dijalankan saat motor DIAM dan posisinya berubah sejak
  // pemeriksaan terakhir - yaitu tepat setelah selesai bergerak. Tidak perlu
  // tiap 500 ms: selama diam hasilnya tidak akan berubah.
  if (!driveRunning()) {
    if (phaseNeedCapture) {
      phaseCapture();            // acuan yang tertunda, diambil setelah berhenti
    } else {
      long pos = drivePosition();
      if (pos != phaseLastPos) {
        phaseLastPos = pos;
        phaseCheck();
      }
    }
  }
}

// --- DIAG: stall atau gangguan driver? -------------------------------------

void driveDiagService() {
  if (!diagPending) return;
  diagPending = false;

  // Pin DIAG dipakai bersama oleh gangguan driver DAN stall. Bendera DRV_STATUS
  // yang membedakannya, dan register itu dibaca ULANG di sini - bukan memakai
  // hasil poll yang bisa berumur setengah detik. Overtemp yang baru muncul
  // sesaat sebelum poll berikutnya akan salah digolongkan sebagai stall kalau
  // memakai nilai lama.
  bool fault = false;
  if (tmcOk) {
    uint32_t st = driver.DRV_STATUS();
    if (st != 0xFFFFFFFFUL) {
      uint8_t f = 0;
      if (st & (1UL << 0)) f |= DRV_FLAG_OTPW;
      if (st & (1UL << 1)) f |= DRV_FLAG_OT;
      if (st & (3UL << 2)) f |= DRV_FLAG_SHORT;
      if (st & (3UL << 4)) f |= DRV_FLAG_OPEN;
      if (st & (1UL << 8)) f |= DRV_FLAG_STEALTH;
      tmcFlags = f;
      fault = (f & (DRV_FLAG_OT | DRV_FLAG_OTPW | DRV_FLAG_SHORT | DRV_FLAG_OPEN)) != 0;
    }
  }

  if (!fault && driveStallArmed()) {
    stallCount++;
    stallEvent = true;
    if (millis() - diagLastLogMs >= DIAG_LOG_MIN_MS) {
      diagLastLogMs = millis();
      Log.printf("STALL terdeteksi (SG_RESULT %u, ambang %u, kejadian ke-%lu).%s\n",
                 (unsigned)tmcSgResult, (unsigned)(cfg.sgThrs * 2),
                 (unsigned long)stallCount,
                 cfg.sgStop ? " Motor dihentikan." : " Motor DIBIARKAN jalan (sg_stop=0).");
    }
    if (cfg.sgStop) {
      // Stall berarti posisi perintah sudah tidak mewakili posisi roda yang
      // sebenarnya. Meneruskan perjalanan dengan koordinat yang salah lebih
      // berbahaya daripada berhenti, jadi ini estop penuh - bukan stopMove().
      // Perintah rc berikutnya akan menghidupkan driver lagi lewat
      // enableOutputs() di driveRc().
      driveHardStop();
    }
    return;
  }

  if (millis() - diagLastLogMs < DIAG_LOG_MIN_MS) return;
  diagLastLogMs = millis();

  if (fault) {
    // Rinciannya sudah dicetak driveTmcPoll() saat bendera berubah; di sini
    // cukup menautkan pin DIAG ke sebab yang sudah diketahui.
    Log.printf("Pin DIAG aktif karena gangguan driver:%s%s%s%s\n",
               (tmcFlags & DRV_FLAG_OT)    ? " OVERTEMP" : "",
               (tmcFlags & DRV_FLAG_OTPW)  ? " suhu-tinggi" : "",
               (tmcFlags & DRV_FLAG_SHORT) ? " SHORT-ke-ground" : "",
               (tmcFlags & DRV_FLAG_OPEN)  ? " kumparan-terbuka" : "");
    return;
  }

  // Tidak ada bendera gangguan dan deteksi stall tidak aktif: kemungkinan besar
  // stall, tapi tanpa StallGuard tidak ada yang bisa memastikannya. Dikatakan
  // apa adanya, bukan didaftar sebagai tiga kemungkinan yang sama-sama mungkin.
  Log.printf("Pin DIAG aktif tanpa bendera gangguan driver - kemungkinan besar "
             "STALL, tapi tidak bisa dipastikan karena deteksi stall mati "
             "(%s).\n",
             cfg.sgThrs == 0 ? "sg_thrs=0" : "butuh set spread 0");
}

bool    driveTmcOk()    { return tmcOk; }
uint8_t driveTmcFlags() { return tmcFlags; }

uint32_t driveMeasuredSps() { return spsFromTstep(tmcTstep); }
bool     drivePhaseKnown()  { return phaseErrorKnown; }
int      drivePhaseError()  { return phaseError; }
uint16_t driveSgResult()    { return tmcSgResult; }
uint32_t driveStallCount()  { return stallCount; }
bool     driveStallEvent()  { return stallEvent; }

// Deteksi stall baru benar-benar hidup bila ketiganya terpenuhi. StallGuard4
// pada TMC2209 TIDAK tersedia di spreadCycle - itu bukan pilihan firmware,
// melainkan batasan chip.
bool driveStallArmed() {
  return tmcOk && cfg.sgThrs > 0 && !cfg.tmcSpreadCycle;
}

// --- Tampilan --------------------------------------------------------------

void drivePrint(Print &out) {
  if (stepper) {
    out.printf("Stepper : %s, posisi %ld, kecepatan %lu sps, akselerasi %lu\n",
               stepper->isRunning() ? (continuousDrive ? "JALAN (terus)" : "JALAN (terukur)")
                                    : "diam",
               drivePosition(),
               (unsigned long)driveSpeedHz, (unsigned long)cfg.driveAccel);
  } else {
    out.println("Stepper : TIDAK TERINISIALISASI");
  }
  out.printf("RC      : %s, arah %s\n",
             rcActive ? "aktif" : "tidak aktif",
             rcDir > 0 ? "MAJU" : (rcDir < 0 ? "MUNDUR" : "netral"));
  out.printf("INDEX   : %lu pulsa\n", (unsigned long)indexCount);
  out.printf("DIAG    : pin %s%s\n", driveDiagPin() ? "HIGH" : "LOW",
             diagFlag ? "  [pernah aktif - cek stall/overtemp]" : "");
}

void driveTmcPrint(Print &out) {
  out.println("---- TMC2209 (UART) ----");
  if (!tmcOk) {
    out.printf("UART    : TIDAK MENJAWAB (alamat %u, TX=GPIO%d RX=GPIO%d)\n",
               (unsigned)TMC_UART_ADDR, PIN_TMC_TX, PIN_TMC_RX);
    out.println("Selama UART mati, driver tetap jalan dengan setelan terakhirnya");
    out.println("(atau setelan standalone MS1/MS2 = 1/16 bila belum pernah diset).");
    out.println("------------------------");
    return;
  }
  out.printf("UART    : OK, versi register 0x%02X, alamat %u\n",
             driver.version(), (unsigned)TMC_UART_ADDR);
  out.printf("Arus    : %u mA RMS diminta, CS_ACTUAL %u/31\n",
             (unsigned)cfg.tmcCurrent, (unsigned)tmcCsActual);
  out.printf("Microstep: 1/%u\n", (unsigned)cfg.tmcMicrosteps);
  out.printf("Chopper : %s (bendera stealth %s)\n",
             cfg.tmcSpreadCycle ? "spreadCycle" : "stealthChop",
             (tmcFlags & DRV_FLAG_STEALTH) ? "aktif" : "mati");
  out.printf("Status  : %s%s%s%s%s\n",
             tmcFlags == 0 || tmcFlags == DRV_FLAG_STEALTH ? "normal" : "",
             (tmcFlags & DRV_FLAG_OT)    ? "OVERTEMP " : "",
             (tmcFlags & DRV_FLAG_OTPW)  ? "suhu-tinggi " : "",
             (tmcFlags & DRV_FLAG_SHORT) ? "SHORT " : "",
             (tmcFlags & DRV_FLAG_OPEN)  ? "kumparan-terbuka " : "");

  // --- Integritas STEP ---
  // Dibaca segar di sini, bukan memakai nilai poll, supaya "show drv" benar-benar
  // menunjukkan keadaan saat perintah diketik.
  tmcMscnt = driver.MSCNT();
  tmcTstep = driver.TSTEP();

  out.println("-- integritas STEP (bukan posisi poros) --");
  out.printf("MSCNT   : %u/1023 (%lu hitungan per langkah, arah %s)\n",
             (unsigned)tmcMscnt, (unsigned long)mscntUnitsPerStep(),
             phaseSign == 0 ? "belum dipastikan" : (phaseSign > 0 ? "searah" : "berlawanan"));
  if (phaseErrorKnown) {
    out.printf("Selisih : %d microstep 1/256 %s\n", phaseError,
               phaseError == 0 ? "- semua pulsa STEP sampai"
                               : "- ADA PULSA STEP YANG HILANG");
  } else {
    out.println("Selisih : belum diperiksa (diperiksa otomatis tiap motor berhenti)");
  }
  uint32_t measured = spsFromTstep(tmcTstep);
  if (tmcTstep >= 0xFFFFF) {
    out.printf("TSTEP   : %lu (motor diam / sangat lambat)\n", (unsigned long)tmcTstep);
  } else {
    out.printf("TSTEP   : %lu -> laju ukur %lu sps, diperintahkan %lu sps\n",
               (unsigned long)tmcTstep, (unsigned long)measured,
               (unsigned long)driveSpeedHz);
    out.println("          (fCLK 12 MHz tipikal, jadi laju ukur bersifat kasar)");
  }

  // --- StallGuard ---
  out.println("-- deteksi stall (StallGuard4) --");
  if (!driveStallArmed()) {
    if (cfg.sgThrs == 0) {
      out.println("Status  : MATI (sg_thrs=0)");
    } else if (cfg.tmcSpreadCycle) {
      out.println("Status  : TIDAK AKTIF - sg_thrs diset tapi spread=1.");
      out.println("          StallGuard4 pada TMC2209 hanya bekerja di stealthChop.");
      out.println("          Pakai \"set spread 0\" lalu \"save\" - dengan konsekuensi");
      out.println("          torsi turun di kecepatan tinggi.");
    }
  } else {
    out.printf("Status  : AKTIF di atas %lu sps (TCOOLTHRS %lu)\n",
               (unsigned long)cfg.sgMinSps, (unsigned long)tstepFromSps(cfg.sgMinSps));
    out.printf("SG_RESULT: %u (stall bila < %u; makin kecil makin berat)\n",
               (unsigned)tmcSgResult, (unsigned)(cfg.sgThrs * 2));
    out.printf("Kejadian: %lu stall%s\n", (unsigned long)stallCount,
               cfg.sgStop ? ", tiap stall memicu estop" : ", hanya dilaporkan (sg_stop=0)");
  }
  out.println("------------------------");
}
