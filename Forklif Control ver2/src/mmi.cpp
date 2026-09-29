#include "mmi.h"

#include <WiFi.h>

#include "config.h"
#include "drive.h"
#include "led.h"
#include "log.h"
#include "net.h"
#include "steering.h"

static WiFiServer server(MMI_PORT);

// ---------------------------------------------------------------------------
// Penerjemah akhir baris untuk telnet.
//
// Telnet menuntut CR LF. Arduino println() memang sudah mengirim "\r\n", tetapi
// printf("...\n") hanya mengirim LF - dan LF saja MENURUNKAN kursor tanpa
// mengembalikannya ke kolom 0, sehingga keluaran menjadi menangga ke kanan.
// Firmware ini punya puluhan baris printf, dan menambal satu per satu berarti
// setiap baris baru di kemudian hari harus ingat menulis "\r\n".
//
// Karena itu terjemahannya dilakukan di SATU tempat: tiap LF yang tidak
// didahului CR diberi CR. println() yang sudah benar tidak terkena dua kali.
//
// Sekalian keluarannya disangga per baris. WiFiClient::write(uint8_t) melakukan
// satu panggilan kirim untuk TIAP byte, dan dengan setNoDelay(true) itu berarti
// satu paket TCP per huruf. Dengan sangga ini satu baris = satu kiriman, lebih
// hemat daripada keadaan sebelumnya yang dua kiriman per baris.
// ---------------------------------------------------------------------------
class TelnetPrint : public Print {
 public:
  void attach(WiFiClient *c) { client_ = c; len_ = 0; last_ = 0; }
  void detach()              { len_ = 0; client_ = nullptr; }

  size_t write(uint8_t c) override {
    if (c == '\n' && last_ != '\r') push('\r');
    push(c);
    last_ = c;
    if (c == '\n') flush();
    return 1;
  }

  size_t write(const uint8_t *buf, size_t size) override {
    for (size_t i = 0; i < size; i++) write(buf[i]);
    return size;
  }

  void flush() override {
    if (len_ && client_ && client_->connected()) client_->write(buf_, len_);
    len_ = 0;
  }

 private:
  void push(uint8_t c) {
    if (len_ >= sizeof(buf_)) flush();
    buf_[len_++] = c;
  }

  WiFiClient *client_ = nullptr;
  uint8_t     buf_[192];
  size_t      len_    = 0;
  uint8_t     last_   = 0;
};

struct MmiSession {
  WiFiClient  client;
  TelnetPrint out;            // semua keluaran ke sesi ini lewat sini
  String      buffer;
  bool        debug = false;  // true bila sesi ini ikut menerima aliran log
};

static MmiSession sessions[MMI_MAX_CLIENTS];
static bool serverStarted = false;

// --- Sink log ke sesi telnet ----------------------------------------------

static void telnetLogSink(const char *line) {
  for (int i = 0; i < MMI_MAX_CLIENTS; i++) {
    if (sessions[i].debug && sessions[i].client && sessions[i].client.connected()) {
      sessions[i].out.println(line);
    }
  }
}

// --- Telnet ---------------------------------------------------------------

// Telnet baku mengharuskan SATU sisi yang meng-echo. Board menyatakan WILL ECHO
// dan WILL SUPPRESS-GO-AHEAD supaya klien (telnet Windows, PuTTY) beralih ke
// mode karakter dan berhenti meng-echo sendiri; sebagai gantinya board yang
// memantulkan tiap huruf. Tanpa ini, yang diketik tidak terlihat sama sekali.
static const uint8_t TELNET_IAC  = 255;
static const uint8_t TELNET_WILL = 251;
static const uint8_t TELNET_OPT_ECHO = 1;
static const uint8_t TELNET_OPT_SGA  = 3;

// Prompt sengaja tanpa akhir baris, jadi sangga harus didorong sendiri supaya
// "> " muncul sebelum pemakai mengetik.
static void mmiPrompt(MmiSession &s) {
  s.out.print("> ");
  s.out.flush();
}

static void telnetGreet(MmiSession &s) {
  const uint8_t negotiate[] = {
    TELNET_IAC, TELNET_WILL, TELNET_OPT_ECHO,
    TELNET_IAC, TELNET_WILL, TELNET_OPT_SGA,
  };
  // Negosiasi ditulis langsung ke soket: byte IAC (255) bukan teks, dan tidak
  // boleh ikut diterjemahkan akhir barisnya.
  s.client.write(negotiate, sizeof(negotiate));

  s.out.println();
  s.out.printf("=== MMI FORKLIFT %s ===\n", topicPrefix());
  s.out.println("Ketik \"help\" untuk daftar perintah, \"show config\" untuk konfigurasi.");
  mmiPrompt(s);
}

// Buang urutan negosiasi telnet (IAC + perintah + opsi) supaya tidak masuk ke
// buffer perintah sebagai sampah.
static bool skipTelnetNegotiation(WiFiClient &c, uint8_t first) {
  if (first != TELNET_IAC) return false;
  if (c.available()) {
    uint8_t cmd = (uint8_t)c.read();
    // 251..254 = WILL/WONT/DO/DONT, selalu diikuti satu byte opsi.
    if (cmd >= 251 && cmd <= 254 && c.available()) c.read();
  }
  return true;
}

static void pollSession(MmiSession &s) {
  while (s.client.available()) {
    uint8_t b = (uint8_t)s.client.read();

    if (skipTelnetNegotiation(s.client, b)) continue;

    if (b == '\n' || b == '\r') {
      // CR LF akan terbaca dua kali; baris kosong diabaikan, jadi aman.
      s.out.println();
      if (s.buffer.length()) {
        mmiRunCommand(s.buffer, s.out);
        s.buffer = "";
      }
      mmiPrompt(s);
      continue;
    }

    if (b == 8 || b == 127) {                       // backspace / delete
      if (s.buffer.length()) {
        s.buffer.remove(s.buffer.length() - 1);
        s.out.print("\b \b");
        s.out.flush();
      }
      continue;
    }

    if (b >= 32 && b <= 126) {
      if (s.buffer.length() < 80) {
        s.buffer += (char)b;
        s.out.write(b);                             // echo
        s.out.flush();                              // huruf harus tampak seketika
      }
    }
  }
}

void mmiBegin() {
  server.begin();
  server.setNoDelay(true);      // perintah pendek tidak boleh menunggu Nagle
  serverStarted = true;
  logAddSink(telnetLogSink);
  Log.printf("MMI siap: telnet port %d.\n", MMI_PORT);
}

void mmiLoop() {
  // --- USB Serial memakai CLI yang sama ---
  static String usbBuffer;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbBuffer.length()) { mmiRunCommand(usbBuffer, Log); usbBuffer = ""; }
    } else if (c >= 32 && c <= 126) {
      if (usbBuffer.length() < 80) usbBuffer += c;
    }
  }

  if (!serverStarted || WiFi.status() != WL_CONNECTED) return;

  // --- Sesi baru ---
  while (server.hasClient()) {
    int slot = -1;
    for (int i = 0; i < MMI_MAX_CLIENTS; i++) {
      if (!sessions[i].client || !sessions[i].client.connected()) { slot = i; break; }
    }
    if (slot < 0) {
      // Tolak dengan pesan, bukan diam-diam: sesi yang menggantung di sisi lain
      // jadi kelihatan sebagai sebab, bukan misteri "telnet tidak mau masuk".
      WiFiClient reject = server.available();
      reject.println("MMI penuh, semua sesi sedang terpakai.");
      reject.stop();
      break;
    }
    sessions[slot].client = server.available();
    sessions[slot].buffer = "";
    sessions[slot].debug  = false;
    sessions[slot].client.setNoDelay(true);
    sessions[slot].out.attach(&sessions[slot].client);
    telnetGreet(sessions[slot]);
    Log.printf("MMI: sesi baru dari %s.\n",
               sessions[slot].client.remoteIP().toString().c_str());
  }

  // --- Sesi berjalan ---
  for (int i = 0; i < MMI_MAX_CLIENTS; i++) {
    MmiSession &s = sessions[i];
    if (!s.client) continue;
    if (!s.client.connected()) {
      s.out.detach();
      s.client.stop();
      s.debug = false;
      s.buffer = "";
      continue;
    }
    pollSession(s);
  }
}

void mmiCloseAll() {
  for (int i = 0; i < MMI_MAX_CLIENTS; i++) {
    if (sessions[i].client) {
      sessions[i].out.println();
      sessions[i].out.println("MMI ditutup: OTA dimulai.");
      sessions[i].out.flush();
      sessions[i].out.detach();
      sessions[i].client.stop();
    }
    sessions[i].debug = false;
    sessions[i].buffer = "";
  }
}

int mmiClientCount() {
  int n = 0;
  for (int i = 0; i < MMI_MAX_CLIENTS; i++) {
    if (sessions[i].client && sessions[i].client.connected()) n++;
  }
  return n;
}

// --- Tampilan --------------------------------------------------------------

static void printStatus(Print &out) {
  out.println("---- STATUS FORKLIFT ----");
  out.printf("Robot   : ID %u (%s), uptime %lu detik\n",
             (unsigned)cfg.id, topicPrefix(), (unsigned long)(millis() / 1000));
  steeringPrint(out);
  drivePrint(out);
  out.printf("TMC UART: %s%s\n", driveTmcOk() ? "OK" : "TIDAK MENJAWAB",
             driveTmcFlags() & ~DRV_FLAG_STEALTH ? "  [ada peringatan, lihat \"show drv\"]" : "");
  ledPrint(out);
  out.printf("Heap    : %lu byte bebas\n", (unsigned long)ESP.getFreeHeap());
  netPrintStatus(out);
  out.println("-------------------------");
}

static void printHelp(Print &out) {
  out.println("---- PERINTAH ----");
  out.println("PENGGERAK (stepper TMC2209)");
  out.println("  fwd [sps]      maju terus, kecepatan opsional");
  out.println("  rev [sps]      mundur terus");
  out.println("  stop           berhenti dengan deselerasi");
  out.println("  estop          berhenti mendadak + driver dimatikan");
  out.println("  move <steps>   gerak terukur, negatif = mundur");
  out.println("  speed <sps>    set kecepatan default");
  out.println("  accel <sps2>   set akselerasi");
  out.println("  en | dis       hidupkan / matikan driver");
  out.println("  zero           nolkan posisi & hitungan INDEX");
  out.println("  rc <sps> <deg> gas+kemudi sekaligus (dipakai gamepad, senyap)");
  out.println("KEMUDI (servo MG995)");
  out.println("  steer <deg>    sudut kemudi absolut (boleh pecahan)");
  out.println("  left | right   geser 10 derajat");
  out.println("  center         kembali ke tengah");
  out.println("  inv            balik arah kemudi kiri<->kanan");
  out.println("  cal <min> <max>  ubah rentang pulsa servo (us)");
  out.println("  p<us>          pulsa mentah untuk kalibrasi");
  out.println("KONFIGURASI");
  out.println("  show           ringkasan status");
  out.println("  show config    seluruh setelan tersimpan");
  out.println("  show net       jaringan, MQTT, topik");
  out.println("  show drv       status driver TMC2209: arus, integritas STEP, stall");
  out.println("  set <key> <nilai>   ubah satu setelan (belum permanen)");
  out.println("  save           simpan seluruh setelan ke NVS");
  out.println("  reload         buang perubahan, baca ulang dari NVS");
  out.println("  factory        kembalikan seluruh setelan ke bawaan lalu simpan");
  out.println("  reboot         restart board");
  out.println("DEBUG");
  out.println("  debug on|off   alirkan log kejadian ke sesi ini");
  out.println("  status | ip | help");
  out.println("------------------");
}

// --- Parser ----------------------------------------------------------------

// Sesi telnet yang sedang menjalankan perintah, supaya "debug on/off" tahu
// sesi mana yang dimaksud. Perintah dari Serial/MQTT tidak punya sesi.
static MmiSession *sessionOf(Print &out) {
  for (int i = 0; i < MMI_MAX_CLIENTS; i++) {
    if (sessions[i].client && (Print *)&sessions[i].out == &out) return &sessions[i];
  }
  return nullptr;
}

void mmiRunCommand(const String &raw, Print &out) {
  String line = raw;
  line.trim();
  if (line.length() == 0) return;

  // Pisahkan kata pertama sebagai perintah, sisanya sebagai argumen.
  String cmd = line;
  String args = "";
  int sp = line.indexOf(' ');
  if (sp > 0) {
    cmd  = line.substring(0, sp);
    args = line.substring(sp + 1);
    args.trim();
  }
  cmd.toLowerCase();

  // ---- penggerak ----
  if (cmd == "fwd" || cmd == "maju") {
    uint32_t sps = args.length() ? (uint32_t)args.toInt() : driveTargetSps();
    if (sps == 0) { out.println("Kecepatan harus > 0."); return; }
    driveForward(sps, out);

  } else if (cmd == "rev" || cmd == "mundur") {
    uint32_t sps = args.length() ? (uint32_t)args.toInt() : driveTargetSps();
    if (sps == 0) { out.println("Kecepatan harus > 0."); return; }
    driveBackward(sps, out);

  } else if (cmd == "rc") {
    int sp2 = args.indexOf(' ');
    if (sp2 <= 0) { out.println("Pakai: rc <sps_bertanda> <sudut>"); return; }
    driveRc(args.substring(0, sp2).toInt(), args.substring(sp2 + 1).toFloat());

  } else if (cmd == "stop") {
    driveStop(out);

  } else if (cmd == "estop") {
    // Estop dilaporkan ke SEMUA penonton, bukan hanya ke pengetiknya: ini
    // kejadian yang menyangkut keselamatan.
    driveEmergencyStop(Log);

  } else if (cmd == "move") {
    if (!args.length()) { out.println("Pakai: move <steps>"); return; }
    driveMove(args.toInt(), out);

  } else if (cmd == "speed") {
    driveSetSpeed((uint32_t)args.toInt(), out);

  } else if (cmd == "accel") {
    driveSetAccel((uint32_t)args.toInt(), out);

  } else if (cmd == "en") {
    driveEnable(out);

  } else if (cmd == "dis") {
    driveDisable(out);

  } else if (cmd == "zero") {
    driveZero(out);

  // ---- kemudi ----
  } else if (cmd == "steer" || cmd == "kemudi") {
    float deg = args.toFloat();
    if (!args.length() || deg < cfg.steerMin || deg > cfg.steerMax) {
      out.printf("Pakai: steer <%.2f-%.2f>\n", (double)cfg.steerMin, (double)cfg.steerMax);
      return;
    }
    steeringSet(deg);
    out.printf("Kemudi menuju %.2f derajat.\n", (double)steeringTarget());

  } else if (cmd == "left" || cmd == "kiri") {
    steeringNudge(-10.0f);
    out.printf("Kemudi kiri -> %.2f derajat.\n", (double)steeringTarget());

  } else if (cmd == "right" || cmd == "kanan") {
    steeringNudge(10.0f);
    out.printf("Kemudi kanan -> %.2f derajat.\n", (double)steeringTarget());

  } else if (cmd == "center" || cmd == "tengah") {
    steeringCenter();
    out.printf("Kemudi kembali ke tengah (%.2f derajat).\n", (double)cfg.steerCenter);

  } else if (cmd == "inv") {
    bool inv = steeringInvertToggle();
    out.printf("Arah kemudi %s (\"save\" untuk menyimpan).\n",
               inv ? "DIBALIK (roda belakang)" : "normal");

  } else if (cmd == "cal") {
    int sp2 = args.indexOf(' ');
    if (sp2 <= 0) { out.println("Pakai: cal <min_us> <max_us>"); return; }
    String err;
    if (!steeringCalibrate(args.substring(0, sp2).toInt(),
                           args.substring(sp2 + 1).toInt(), err)) {
      out.printf("GAGAL: %s\n", err.c_str());
      return;
    }
    out.printf("Rentang pulsa servo = %d-%d us.\n",
               steeringPulseMin(), steeringPulseMax());

  } else if (cmd.startsWith("p") && cmd.length() > 1 && isDigit(cmd[1])) {
    int us = cmd.substring(1).toInt();
    if (us < 300 || us > 3000) { out.println("Nilai pulsa harus 300-3000 us."); return; }
    steeringRawPulse(us);
    out.printf("Pulsa mentah %d us.\n", us);

  // ---- konfigurasi ----
  } else if (cmd == "show") {
    args.toLowerCase();
    if (args.length() == 0 || args == "status")   printStatus(out);
    else if (args == "config" || args == "cfg")   configPrint(out);
    else if (args == "net" || args == "ip")       netPrintStatus(out);
    else if (args == "drv" || args == "tmc")      driveTmcPrint(out);
    else out.println("Pakai: show [status|config|net|drv]");

  } else if (cmd == "set") {
    int sp2 = args.indexOf(' ');
    if (sp2 <= 0) { out.println("Pakai: set <key> <nilai>   (lihat \"show config\")"); return; }
    String key = args.substring(0, sp2);
    String val = args.substring(sp2 + 1);
    val.trim();
    String err;
    if (!configSet(key.c_str(), val.c_str(), err)) {
      out.printf("GAGAL: %s = \"%s\" ditolak (%s)\n", key.c_str(), val.c_str(), err.c_str());
      return;
    }
    // Setelan yang bisa berlaku seketika langsung dikirim ke perangkatnya;
    // sisanya (jaringan, MQTT, OTA, id) baru hidup setelah restart.
    steeringApplyConfig();
    driveApplyConfig();
    ledApplyConfig();
    out.printf("%s = %s. Belum permanen - ketik \"save\".\n", key.c_str(), val.c_str());

  } else if (cmd == "save") {
    configSave();
    out.println("Konfigurasi disimpan ke NVS.");

  } else if (cmd == "reload") {
    configReload();
    steeringApplyConfig();
    driveApplyConfig();
    ledApplyConfig();
    out.println("Konfigurasi dibaca ulang dari NVS (perubahan yang belum disimpan dibuang).");

  } else if (cmd == "factory") {
    configFactory();
    steeringApplyConfig();
    driveApplyConfig();
    ledApplyConfig();
    out.println("Konfigurasi dikembalikan ke bawaan dan disimpan.");
    out.println("Setelan jaringan baru berlaku setelah \"reboot\".");

  } else if (cmd == "reboot" || cmd == "restart") {
    // Motor dimatikan dulu: board yang restart sambil motor berjalan akan
    // meninggalkan kendaraan melaju beberapa ratus milidetik tanpa kendali.
    driveHardStop();
    out.println("Board restart...");
    out.flush();
    delay(200);
    ESP.restart();

  // ---- debug ----
  } else if (cmd == "debug") {
    MmiSession *s = sessionOf(out);
    if (s == nullptr) {
      out.println("\"debug\" hanya berlaku di sesi telnet (Serial dan MQTT selalu menerima log).");
      return;
    }
    args.toLowerCase();
    if (args == "on" || args == "1") {
      s->debug = true;
      out.println("Debug AKTIF: log kejadian dialirkan ke sesi ini.");
    } else if (args == "off" || args == "0") {
      s->debug = false;
      out.println("Debug mati.");
    } else {
      out.printf("Debug sedang %s. Pakai: debug on|off\n", s->debug ? "aktif" : "mati");
    }

  // ---- lain-lain ----
  } else if (cmd == "status") {
    printStatus(out);
  } else if (cmd == "ip") {
    netPrintStatus(out);
  } else if (cmd == "help" || cmd == "?") {
    printHelp(out);
  } else {
    out.printf("Perintah \"%s\" tidak dikenal. Ketik help.\n", cmd.c_str());
  }
}
