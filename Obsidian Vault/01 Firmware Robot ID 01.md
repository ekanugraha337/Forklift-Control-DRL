
Forklift ini berjalan menggunakan processor ESP32-S3 dengan board ESP32-S3-Nano dari waveshare. 

Terdapat dua (2) penggerak, yaitu :
- penggerak roda depan untuk maju dan mundur menggunakan motor stepper NEMA17 dengan drive TMC2209v2
- steering wheel belakang yang digerakann menggunakan motor servo MG995

Konfigurasi TMC2209 MS1 dan MS2 terhubung dengan 3.3V

Berikut pin assignment pada ESP32-S3 :
- ESP32 GPIO-08 -> TMC2209 Index
- ESP32 GPIO-06 -> TMC2209 Diag
- ESP32 GPIO-02 -> TMC2209 Enable
- ESP32 GPIO-03 -> TCM2209 Step
- ESP32 GPIO-04 -> TCM2209 Direction
- ESP32 GPIO-17 -> Resitor 1K ohm -> TMC2209 PDN_Uart
- ESP32 GPIO-18 -> TCM2209 PDN_Uart
- ESP32 GPIO-07 -> Motor Servo MG995 PWM

Fitur yang ada di firmware ini : 
- Watchdog
- MQTT
- ESP32 memory configuration -> library Preference.h
- Man Machine Interface
- OTA

Untuk Robot id 01 memiliki central steering degree 89.45. Ini di simpan ke configuration. 

MQTT  Untuk topic dan payload nya control robot :
- forklift01/in/cmd : estop -> Emergency Stop
- forklift01/in/rc : speed angle -> perintah kecepatan maju speed positif, kecepatan mundur speed negatif
- forklift01/in/cmd : center -> steering position ke center
- forklift01/out/telemetry : 
{"up":38,"drv":"idle","pos":0,"sps":0,"tgt_sps":2000,"acc":4000,"dir":0,"rc":0,"str":89,"str_tgt":89,"inv":1,"idx":0,"diag":0,"diag_ev":0,"rssi":-65,"heap":259548,"servo":1}

memory configuration (default) : 
- Forklift ID : 1
- Central Steering : 89.45
- Steering Min : 10
- Steering Max : 170
- IP Address Wifi Static 10.210.1.101/24
- Gateway : 10.210.1.1
- MQTT Username : forklift01
- MQTT Password : testing88
- MQTT Host : 10.210.1.1
- MQTT Port : 1883
- SSID : Forklift
- SSID Password : testing88
- OTA Hostname : forklift-control
- OTA Password : forklift123

Memory ini akan di load ke konfigurasi variable pertama kali saat processor pertama kali hidup / jalan. Jadi untuk selanjutnya, variable nya akan memiliki nilai yang sama dengan konfigurasi yang terakhir di simpan

Man Machine Interface (MMI): 
- MMI adalah fitur CLI yang memungkinan user untuk melakukan Setup Configuration, Show Configuration, Debug.
- MMI diakses menggunakan Telnet TCP Port 23
