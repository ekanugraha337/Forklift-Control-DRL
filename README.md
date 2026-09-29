# Forklift Control

Forklift robot kecil yang dinavigasi otomatis dari kamera atas (AprilTag), satu
atau dua robot sekaligus.

| Folder | Isi |
|---|---|
| `Forklif Control ver2/` | Firmware ESP32 (PlatformIO): stepper, servo kemudi, MQTT, OTA |
| `Forklift Robot Program/` | Program PC: perencanaan Hybrid A\*, pure pursuit, wasit dua robot. Lihat `README.md` di dalamnya |
| `Camera/` | Kamera ONVIF: koreksi lensa, homografi, deteksi AprilTag, peta constraint |
| `Obsidian Vault/` | Catatan rancangan: model & firmware tiap robot, navigasi multi-robot |

Repo ini adalah salinan dari folder kerja di laptop, bukan tempat kerja utamanya.
