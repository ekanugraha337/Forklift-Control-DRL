# Uji robot nyata ver-1-4: Hybrid A\* + RS vs DQfD murni (peta Gudang)

Tanggal: 2026-10-24 · Branch `ver-1-4` · Dua robot (ID 01, ID 02) bergerak bersamaan · Hasil: `uji_nyata/20261024_073221_Gudang` · Perbandingan model: [[08 Perbandingan Hybrid A-star RS vs DQfD|Perbandingan Hybrid A-star RS vs DQfD]] · Latar: [[06 Deep Reinforcement Learning|Deep Reinforcement Learning]]

Uji robot nyata pertama untuk ver-1-4. Uji ini menjawab butir 2 "Langkah berikutnya" di [[08 Perbandingan Hybrid A-star RS vs DQfD|Perbandingan Hybrid A-star RS vs DQfD]].

> [!summary] Ringkasan
> - **Keandalan setara**: 11/11 percobaan, pasangan robot tiba semua, 0 estop.
> - **Makespan**: DQfD sedikit lebih cepat (19,0 ± 10,3 s vs 21,7 ± 11,4 s), tetapi sebarannya tumpang-tindih. Di S1 (rute bersilangan) makespan ditentukan oleh **interaksi antar-robot** (menunggu, rencana ulang), bukan oleh perencana.
> - **Konsistensi**: di S2 (rute tidak bersilangan) DQfD 9,6–10,0 s dengan lintasan hampir menumpuk; Hybrid A\* 10,2–20,6 s.
> - **Presisi**: galat akhir DQfD 1,8 ± 0,8 cm vs 2,4 ± 0,8 cm. Selisihnya terutama dari ID 01 di S1 (0,9 vs 2,4 cm).
> - **ID 02 meleset 5–8° arah hadap dengan kedua perencana**. Ini masalah robot (kemudi, trim, atau letak tag), bukan masalah perencana.
> - **Waktu rencana**: Hybrid A\* + RS sekitar 30 ms, DQfD sekitar 148 ms (5× lebih lambat). Hampir semuanya di bagian Hybrid A\* / model jauh. RS dan model dekat hanya 0,2–3 ms.
> - Belum lengkap: S2 ulangan 3 DQfD belum dijalankan (11 dari 12 percobaan).

---

## 1. Pengaturan

| | |
|---|---|
| Program | `python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang.json --ulang 3` |
| Perencana uji | `astar` (Hybrid A\* + Reeds-Shepp) dan `dqfd-murni` (model jauh + model dekat, tanpa cadangan A\*) |
| Urutan | per skenario, per ulangan; perencana diselang-seling (ulangan genap A\* dulu, ganjil DQfD dulu) |
| Pulang ke titik awal | otomatis dengan Hybrid A\*, toleransi 5 cm / 10°, lalu operator menekan `n` |
| Pelacak | pure pursuit, 20 Hz, pose dari kamera AprilTag |
| Margin keselamatan | 8 cm |
| Serah Hybrid A\* → RS | hanya bila titik acuan < 60 cm dari tujuan (`analytic_dist=60`, `analytic_every=None`) |
| Serah model jauh → dekat | juga di 60 cm, jadi kedua perencana memakai batas serah yang sama |

### Skenario (cm, derajat)

| Skenario | Robot | Awal | Tujuan | Sifat |
|---|---|---|---|---|
| S1 | ID 01 | (27,4; 159,7; −88,2°) | (191,1; 31,4; −88,3°) | rute diagonal **bersilangan** di tengah arena |
| | ID 02 | (27,0; 39,9; 91,7°) | (195,1; 164,4; 87,9°) | |
| S2 | ID 01 | (26,2; 156,8; −91,2°) | (106,7; 167,8; 90,0°) | balik arah masuk teluk tengah; **tidak bersilangan** |
| | ID 02 | (24,4; 42,0; 98,0°) | (111,1; 33,5; −90,0°) | |

## 2. Hasil per skenario

| | S1 Hybrid A\* | S1 DQfD | S2 Hybrid A\* | S2 DQfD |
|---|---|---|---|---|
| Pasangan tiba | 3/3 | 3/3 | 3/3 | 2/2 |
| **Makespan (s)** | 29,0 ± 11,7 (17,5–40,8) | 25,1 ± 8,6 (16,4–33,6) | 14,5 ± 5,4 (10,2–20,6) | **9,8 ± 0,3** (9,6–10,0) |
| Menunggu robot lain (s) | 3,4 ± 2,5 | 1,9 ± 1,9 | 0 | 0 |
| Rencana per robot per percobaan | 1,7 (maks 5) | 1,2 (maks 2) | 1,0 | 1,0 |
| Panjang jalur rencana (cm) | 274 ± 23 | 266 ± 21 | 156 ± 11 | 152 ± 6 |
| Ganti arah per jalur | 2,0 | 2,0 (0–4) | 2,0 | 2,0 |
| Galat akhir (cm) | 2,5 ± 0,4 | **1,7 ± 0,8** | 2,2 ± 1,1 | 2,0 ± 0,7 |
| Galat arah akhir (°) | 4,6 ± 2,5 | 3,3 ± 1,7 | 5,3 ± 3,7 | 5,9 ± 1,4 |
| Jarak terdekat ke zona (cm) | 7,3–15,9 | 9,7–15,1 | 7,9–14,1 | 11,8–14,6 |

Semua skenario digabung:

| | Hybrid A\* + RS (n=6) | DQfD murni (n=5) |
|---|---|---|
| Pasangan tiba / estop | 6/6 / 0 | 5/5 / 0 |
| Makespan | 21,7 ± 11,4 s | 19,0 ± 10,3 s |
| Galat akhir ID 01 | 2,1 ± 0,8 cm / 3,1 ± 3,1° | **1,4 ± 0,6 cm** / 3,3 ± 2,1° |
| Galat akhir ID 02 | 2,7 ± 0,6 cm / 6,8 ± 1,6° | 2,3 ± 0,6 cm / 5,3 ± 1,4° |
| Rencana pertama (median / p90) | ~31 / 61–80 ms | ~150 / 165–217 ms |

### Per percobaan

| Skenario | Ul. | Perencana | Makespan | ID 01: tempuh / tunggu / rencana / galat | ID 02: tempuh / tunggu / rencana / galat |
|---|---|---|---|---|---|
| S1 | 1 | Hybrid A\* | 40,8 s | 29,0 s / 5,3 s / 1 / 2,3 cm 1,4° | 40,8 s / 7,0 s / **5** / 2,4 cm 4,9° |
| S1 | 1 | DQfD | 33,6 s | 33,6 s / 4,7 s / **2** / 0,8 cm 1,4° | 24,4 s / 1,2 s / 1 / 2,8 cm 4,7° |
| S1 | 2 | DQfD | 25,2 s | 15,5 s / 0 / 1 / 1,0 cm 3,5° | 25,2 s / 3,3 s / 1 / 2,1 cm 4,3° |
| S1 | 2 | Hybrid A\* | 28,6 s | 28,6 s / 3,9 s / 1 / 2,0 cm 1,9° | 27,2 s / 2,0 s / 1 / 2,4 cm 6,7° |
| S1 | 3 | Hybrid A\* | 17,5 s | 17,5 s / 2,1 s / 1 / 2,8 cm 7,4° | 15,8 s / 0 / 1 / 3,0 cm 5,4° |
| S1 | 3 | DQfD | 16,4 s | 16,4 s / 2,5 s / 1 / 1,1 cm 1,0° | 14,5 s / 0 / 1 / 2,3 cm 4,6° |
| S2 | 1 | Hybrid A\* | 12,7 s | 12,7 s / 0 / 1 / 0,5 cm 0,7° | 10,1 s / 0 / 1 / 1,8 cm 6,8° |
| S2 | 1 | DQfD | 10,0 s | 9,2 s / 0 / 1 / 1,5 cm 4,8° | 10,0 s / 0 / 1 / 1,4 cm 5,1° |
| S2 | 2 | DQfD | 9,6 s | 9,3 s / 0 / 1 / 2,4 cm 5,9° | 9,6 s / 0 / 1 / 2,8 cm 7,9° |
| S2 | 2 | Hybrid A\* | 10,2 s | 9,4 s / 0 / 1 / 2,8 cm 6,6° | 10,2 s / 0 / 1 / 3,6 cm 9,3° |
| S2 | 3 | Hybrid A\* | 20,6 s | **20,6 s** / 0 / 1 / 2,0 cm 0,6° (jalur 178 cm) | 9,8 s / 0 / 1 / 2,8 cm 7,7° |
| S2 | 3 | DQfD | — | belum dijalankan | |

![[Uji nyata Gudang 2026-10-24 S1.png]]
![[Uji nyata Gudang 2026-10-24 S2.png]]

Garis penuh: ID 01. Garis putus-putus: ID 02. Kotak titik-titik: pose awal. Kotak putus-putus: pose tujuan.

## 3. Waktu perencanaan per bagian

Rincian ini **tercatat langsung saat uji** (`rincian_ms` di `hasil.jsonl`, ditambahkan ke `rencana.py` dan `drl/perencana_dqfd.py` pada 2026-10-23). Angka di tabel adalah median rencana pertama, dalam ms.

| Perencana | Robot | Hybrid A\* | RS | DQfD jauh | DQfD dekat | Lain | Jumlah | Semua rencana per percobaan |
|---|---|---|---|---|---|---|---|---|
| Hybrid A\* + RS | ID 01 | 27,4 (93%) | 0,3 (1%) | – | – | 1,6 | 29,3 | 38,9 ± 18,4 |
| | ID 02 | 31,8 (99%) | 0,2 (1%) | – | – | 0,0 | 32,1 | 70,1 ± 89,8 |
| DQfD murni | ID 01 | – | – | 147,3 (98%) | 1,3 (1%) | 1,4 | 150,1 | 161,5 ± 120,6 |
| | ID 02 | – | – | 144,8 (99%) | 1,7 (1%) | 0,0 | 146,5 | 139,5 ± 77,7 |

- **Hybrid A\***: bagian Hybrid A\* mencakup heuristik Dijkstra (sekitar 2 ms) dan ekspansi primitif. Tembakan RS dari dalam 60 cm hanya 0,2–0,6 ms.
- **DQfD**: 98–99% waktunya di **model jauh**, yang melangkah per primitif dengan inferensi jaringan dan cek tabrakan di setiap langkah. Model dekat hanya memilih satu kata RS menurut urutan Q (0,6–3,3 ms), jadi biayanya setara RS di Hybrid A\*.
- Rencana DQfD sekitar 150 ms tetap kecil dibanding waktu tempuh (10–34 s), jadi tidak terlihat pada makespan.
- Anomali satu kali: pada S1 ulangan 2 Hybrid A\*, ID 01 punya **54,5 ms di "lain"**, yaitu di luar perencana. Kemungkinan perencana berebut CPU dengan loop kamera dan kendali.
- Waktu yang tercatat di robot lebih besar daripada waktu offline. Pada uji 2026-10-23 (`20261023_090440_Gudang`, diukur ulang dengan `--rinci`), DQfD 184–235 ms saat uji vs 64–79 ms offline, sedangkan Hybrid A\* hampir sama. Untuk laporan, gunakan **porsi (%)** atau angka yang tercatat saat uji.

### Kenapa Hybrid A\* selalu 2 ganti arah

Serah ke RS terjadi < 60 cm dari tujuan. Diukur pada rencana uji 2026-10-23, jarak serah 41,6–57,1 cm, tetapi bagian RS-nya 84–125 cm (34–45% titik jalur). Saat masuk batas 60 cm, Hybrid A\* menghadap **ke** tujuan, bukan searah arah hadap tujuan. Akibatnya:
- RS memutar arah dengan maju-mundur;
- lalu menutup dengan segmen lurus 32,4 cm (`approach_len`).

Ini sesuai catatan di `konfig.py`: memilih 60 cm menaikkan ganti arah, dan itu diterima dengan sadar.

## 4. Kesimpulan

1. **Keandalan dan keamanan setara.** Semua pasangan tiba, tanpa estop. Jarak terdekat Hybrid A\* ke zona 7,3 cm (ID 02, S1 ulangan 1) dan 7,9 cm (ID 01, S2 ulangan 3), sedikit di bawah margin 8 cm. Minimum DQfD 9,7 cm.
2. **Di S1, makespan ditentukan interaksi dua robot.**
   - Percobaan tercepat kedua perencana hampir sama (17,5 vs 16,4 s).
   - Percobaan terlambat masing-masing terjadi saat robot saling menghalangi: ID 02 dengan A\* merencanakan ulang 5 kali dan menunggu 7 s; ID 01 dengan DQfD merencanakan ulang sekali dan menunggu 4,7 s.
   - Selisih rata-rata (29,0 vs 25,1 s) lebih kecil dari sebarannya.
   - Kesan dari uji 2026-10-23 bahwa "DQfD 2× lebih cepat" berasal dari satu percobaan A\* yang kebetulan lambat.
3. **Di S2, DQfD lebih konsisten.** Makespan 9,6–10,0 s dengan lintasan nyaris identik. Hybrid A\* sekali memberi ID 01 jalur 178 cm (biasanya sekitar 147 cm) dengan manuver tambahan, sehingga makespan 20,6 s. Tanpa percobaan itu, Hybrid A\* sekitar 11 s.
4. **Presisi akhir**: DQfD unggul terutama untuk ID 01 di S1 (0,9 ± 0,1 cm vs 2,4 cm). Di S2 keduanya sekitar 2 cm.
5. **ID 02 punya bias arah hadap 5–8°** dengan kedua perencana, juga pada uji 2026-10-23. Yang perlu diperiksa: kalibrasi kemudi atau trim ID 02 dan letak AprilTag-nya, bukan perencananya.
6. **Waktu rencana**: Hybrid A\* + RS sekitar 5× lebih cepat (~30 vs ~148 ms). Selisih seluruhnya ada di model jauh. Bagian pendaratan (RS / model dekat) murah di keduanya.

> [!warning] Keterbatasan
> - n = 2–3 per sel. Perbedaan di atas adalah **kecenderungan**, belum terbukti signifikan secara statistik.
> - S2 ulangan 3 DQfD belum ada. Lengkapi dengan `--lanjut uji_nyata/20261024_073221_Gudang`.
> - Di S1, interaksi antar-robot menutupi pengaruh perencana. Untuk membandingkan perencananya sendiri, tambah skenario tanpa persilangan (seperti S2) dan ulangan (≥ 5).

## 5. Sumber angka

| Angka | Sumber |
|---|---|
| Semua tabel bagian 2 | `Forklift Robot Program/uji_nyata/20261024_073221_Gudang/hasil.jsonl`, `ringkasan.json`, `ringkasan_tabel.tex` |
| Rincian waktu bagian 3 | `rincian_ms` di `hasil.jsonl` yang sama; `rincian_tabel.tex`; `python -m drl.uji_robot_nyata --ringkas uji_nyata/20261024_073221_Gudang` |
| Gambar lintasan | `lintasan_S1.png`, `lintasan_S2.png` di folder hasil (disalin ke `Gambar/`) |
| Waktu offline vs tercatat, jarak serah RS | `uji_nyata/20261023_090440_Gudang` (S1, 3 percobaan), `rincian_ulang.json` (`--rinci`) |
| Folder `uji_nyata/20261024_071540_Gudang` | kosong: program ditutup di fase KONFIRMASI sebelum percobaan pertama |
