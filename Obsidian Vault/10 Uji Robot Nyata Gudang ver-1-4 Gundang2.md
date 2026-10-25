# Uji robot nyata ver-1-4: Hybrid A\* + RS vs DQfD murni (peta Gudang2)

Tanggal: 2026-10-24 · Branch `ver-1-4` · Dua robot (ID 01, ID 02) bergerak bersamaan · Hasil: `uji_nyata/20261024_082405_Gudang2` · Uji sebelumnya: [[09 Uji Robot Nyata Gudang ver-1-4 Gundang1|Uji Robot Nyata Gudang ver-1-4 Gundang1]] · Perbandingan model: [[08 Perbandingan Hybrid A-star RS vs DQfD|Perbandingan Hybrid A-star RS vs DQfD]]

Gudang2 adalah peta dan skenario yang **sengaja dirancang** supaya DQfD murni merencanakan lebih cepat daripada Hybrid A\*. Di peta Gudang (uji sebelumnya) yang terjadi sebaliknya: Hybrid A\* sekitar 30 ms, DQfD sekitar 148 ms. Uji ini memeriksa apakah rancangan itu berlaku di robot nyata.

> [!summary] Ringkasan
> - **Tujuan rancangan tercapai**: DQfD lebih cepat merencanakan di **36 dari 36** rencana. Median 58 ms vs 198 ms, jadi **3,4× lebih cepat**. Rencana DQfD yang paling lambat (69 ms) masih jauh di bawah rencana Hybrid A\* yang paling cepat (134 ms).
> - **DQfD sangat stabil**: 59 ± 5 ms di semua skenario. Hybrid A\* 204 ± 45 ms, berkisar 134–301 ms.
> - **Keandalan setara**: 18/18 percobaan tiba, 0 estop, 0 waktu menunggu, 1 rencana per robot per percobaan.
> - **Makespan sama persis**: 29,6 ± 3,2 s vs 29,6 ± 2,9 s. Waktu rencana (< 0,3 s) kurang dari 1% waktu tempuh, dan jalur kedua perencana hampir identik.
> - **Presisi**: ID 02 baik di keduanya (sekitar 2 cm). ID 01 meleset di dua tujuan yang menghadap dinding atas (S2, S3), dengan **kedua** perencana. Penyebabnya rancangan skenario: di kedua tujuan itu, segmen lurus pendekatan tidak muat karena terhalang ujung papan pembatas, sehingga jalur masuk ke tujuan sambil masih berbelok (bagian 4).

---

## 1. Rancangan peta dan skenario

### Dasar rancangan

Saya mengukur 505 rute uji di Kalibrasi, Rumah2, dan Rumah (Hybrid A\* vs `dqfd-murni`, offline). DQfD lebih cepat ketika robot **mulai membelakangi tujuan** dan **tujuannya jauh**:

| Sudut arah hadap awal terhadap arah ke tujuan | Jarak | DQfD lebih cepat | Rasio DQfD/A\* | A\* | DQfD |
|---|---|---|---|---|---|
| 0–45° | ≥ 170 cm | 0% | 3,39× | 8 ms | 34 ms |
| 135–180° | 100–170 cm | 79% | 0,63× | 32 ms | 16 ms |
| 135–180° | ≥ 170 cm | **89%** | **0,21×** | 177 ms | 36 ms |

Heuristik Dijkstra Hybrid A\* tidak memperhitungkan arah hadap. Kalau robot harus berbalik dulu, A\* membuka ribuan simpul. Model jauh DQfD sudah belajar gerakan berbalik, jadi waktunya hanya bergantung pada panjang jalur. Ini juga menjelaskan hasil Gudang: di sana robot mulai menghadap ±90° terhadap tujuan, dan di kondisi itu A\* lebih cepat.

Empat kandidat peta dibandingkan, masing-masing dengan 12 skenario acak (robot membelakangi tujuan, jarak ≥ 170 cm):

| Kandidat | DQfD lebih cepat | Rasio median |
|---|---|---|
| terbuka (hanya dinding luar) | 24/24 | 0,13 |
| **pembatas tengah (dipilih)** | **24/24** | **0,17** |
| pembatas + sekat teluk | 20/24 | 0,45 |
| sekat Gudang | 16/24 | 0,43 |

Pembatas tengah dipilih karena tiap robot mendapat lajur sendiri. Akibatnya tidak ada waktu menunggu dan rencana ulang antar-robot, yang di Gudang S1 menutupi pengaruh perencana.

### Peta Gudang2 (`Camera/peta_constraint/Gudang2.json`)

```
y=0   ┌──────────────────────────────────────────────┐
      │  lajur ID 01                                  │
y≈105 │      ████████████████████████████████         │  ← pembatas x 45–194 cm
      │  lajur ID 02                                  │
y=210 └──────────────────────────────────────────────┘
      x=0                                         x≈238
```

- Z1–Z4: dinding luar, sama dengan Gudang.
- Z5: papan 81 × 5 cm di x 45–126.
- Z6: papan 68 × 5 cm di x 126–194.
- Kedua papan berada di y 102,5–107,5. Celah sekitar 45 cm di kedua ujung dibiarkan terbuka.

### Skenario (`uji_nyata/Gudang2.json`, cm, derajat)

| | ID 01: awal → tujuan | ID 02: awal → tujuan |
|---|---|---|
| S1 | (210, 55, 0°) → (35, 55, 180°) | (35, 160, 180°) → (210, 160, 0°) |
| S2 | (210, 50, 0°) → (35, 40, −90°) | (35, 165, 180°) → (210, 170, 90°) |
| S3 | (40, 60, 180°) → (205, 40, −90°) | (205, 150, 0°) → (40, 185, 90°) |

Pada setiap awal, robot membelakangi tujuannya. Titik acuan robot adalah poros roda depan, dan badannya 23 cm ke belakang, jadi robot harus **mundur jauh dulu** lalu berbalik.

Validasi offline sebelum uji, dengan 13 pose awal per robot (nominal + 12 digeser ±4 cm / ±8°) dan robot lain sebagai rintangan:
- DQfD lebih cepat di 78/78 rencana (A\* 100–160 ms, DQfD 16–20 ms);
- tidak ada yang gagal;
- perjalanan pulang dengan A\* ke semua awal berhasil dengan margin penuh 8 cm.

## 2. Pengaturan uji

| | |
|---|---|
| Program | `python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang2.json --ulang 3` |
| Perencana uji | `astar` (Hybrid A\* + Reeds-Shepp) dan `dqfd-murni` (model jauh + model dekat, tanpa cadangan A\*) |
| Urutan | per skenario, per ulangan; perencana diselang-seling |
| Pulang | otomatis dengan Hybrid A\*, toleransi 5 cm / 10°, lalu operator menekan `n` |
| Pelacak | pure pursuit, 20 Hz, pose dari kamera AprilTag |
| Margin keselamatan | 8 cm |
| Serah ke RS / model dekat | < 60 cm dari tujuan, sama untuk kedua perencana |

## 3. Waktu perencanaan

### Per skenario (rata-rata rencana pertama, ms)

| | Hybrid A\* ID 01 | DQfD ID 01 | Rasio | Hybrid A\* ID 02 | DQfD ID 02 | Rasio |
|---|---|---|---|---|---|---|
| S1 | 199 | 63 | 0,31 | 289 | 61 | 0,21 |
| S2 | 177 | 60 | 0,34 | 179 | 54 | 0,30 |
| S3 | 183 | 54 | 0,29 | 200 | 60 | 0,30 |
| **Semua** | **187 ± 26** | **59 ± 5** | | **222 ± 55** | **58 ± 6** | |

### Per bagian (median rencana pertama, tercatat saat uji)

| Perencana | Robot | Hybrid A\* | RS | DQfD jauh | DQfD dekat | Lain | Jumlah |
|---|---|---|---|---|---|---|---|
| Hybrid A\* + RS | ID 01 | 190,1 (99%) | 0,5 (0%) | – | – | 1,4 | 192,1 |
| | ID 02 | 218,3 (100%) | 0,2 (0%) | – | – | 0,0 | 218,6 |
| DQfD murni | ID 01 | – | – | 53,7 (93%) | 2,2 (4%) | 2,0 | 57,9 |
| | ID 02 | – | – | 54,3 (97%) | 1,9 (3%) | 0,0 | 56,3 |

- Kelambatan Hybrid A\* seluruhnya ada di **pencarian**, yang harus memutar arah hadap robot. Tembakan RS tetap murah (0,2–0,5 ms per rencana). Pada S1 ID 02 sempat 11–14 ms, karena ada lebih banyak percobaan RS yang gagal.
- DQfD tetap didominasi model jauh. Model dekat 1–6 ms.

### Saat uji vs offline

Ke-36 rencana diulang offline (`--rinci`), dan semuanya menghasilkan jalur yang **cocok** dengan rekaman:

| | Tercatat saat uji (median) | Offline (median) | Faktor |
|---|---|---|---|
| Hybrid A\* + RS | 198 ms | 99 ms | 1,8× (1,5–2,3) |
| DQfD murni | 58 ms | 19 ms | 3,0× (2,6–3,7) |
| Rasio DQfD / A\* | **0,29** | **0,19** | |

- Di robot nyata, DQfD melambat lebih banyak daripada A\*. Ini konsisten dengan uji Gudang (sekitar 3×). Dugaannya, inferensi torch yang kecil-kecil berebut CPU dengan loop kamera dan kendali 20 Hz. Dugaan ini belum diperiksa.
- Meski begitu, keunggulan DQfD tetap besar: 3,4× di robot, 5,2× offline.

## 4. Hasil gerak

| | S1 Hybrid A\* | S1 DQfD | S2 Hybrid A\* | S2 DQfD | S3 Hybrid A\* | S3 DQfD |
|---|---|---|---|---|---|---|
| Pasangan tiba | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 | 3/3 |
| **Makespan (s)** | 33,8 ± 0,3 | 33,3 ± 0,3 | 28,4 ± 0,3 | 28,9 ± 0,5 | 26,6 ± 0,4 | 26,7 ± 0,2 |
| Panjang jalur rencana (cm) | 242 ± 5 | 242 ± 4 | 210 ± 14 | 208 ± 15 | 216 ± 16 | 224 ± 8 |
| Ganti arah (ID 01 / ID 02) | 3 / 3 | 3 / 3 | 1 / 1 | 1 / 1 | 1–3 / 1 | 3 / 1 |
| Galat ID 01 (cm / °) | 2,2 / 1,5 | 1,8 / 3,6 | **5,9** / 3,9 | 4,1 / **19,6** | 2,5 / **16,8** | 1,7 / **14,4** |
| Galat ID 02 (cm / °) | 2,8 / 5,2 | 3,1 / 6,9 | 1,4 / 3,6 | 1,1 / 3,3 | 1,4 / 1,5 | 1,7 / 2,8 |
| Jarak terdekat ke zona (cm) | 9,4–17,8 | 10,6–16,3 | 10,3–13,0 | 9,2–14,8 | 12,0–21,5 | 18,8–21,4 |

Semua skenario digabung:

| | Hybrid A\* + RS (n=9) | DQfD murni (n=9) |
|---|---|---|
| Pasangan tiba / estop / menunggu | 9/9 / 0 / 0 s | 9/9 / 0 / 0 s |
| Makespan | 29,6 ± 3,2 s | 29,6 ± 2,9 s |
| Rencana pertama | 204 ± 45 ms | **59 ± 5 ms** |
| Panjang jalur rencana | 223 ± 19 cm | 224 ± 17 cm |
| Galat ID 01 | 3,5 ± 1,8 cm / 7,4 ± 7,6° | 2,5 ± 1,5 cm / 12,5 ± 10,4° |
| Galat ID 02 | 1,9 ± 0,8 cm / 3,5 ± 1,7° | 2,0 ± 1,0 cm / 4,3 ± 2,2° |

### Per percobaan

Format sel: rencana / tempuh / galat.

| Skenario | Ul. | Perencana | Makespan | ID 01 | ID 02 |
|---|---|---|---|---|---|
| S1 | 1 | Hybrid A\* | 33,6 s | 214 ms / 31,0 s / 2,2 cm 1,3° | 302 ms / 33,6 s / 2,8 cm 5,1° |
| S1 | 1 | DQfD | 33,2 s | 61 ms / 32,3 s / 1,8 cm 6,6° | 59 ms / 33,2 s / 2,6 cm 6,5° |
| S1 | 2 | DQfD | 33,0 s | 61 ms / 30,9 s / 2,2 cm 0,7° | 58 ms / 33,0 s / 2,9 cm 5,4° |
| S1 | 2 | Hybrid A\* | 33,6 s | 208 ms / 30,0 s / 2,3 cm 1,8° | 284 ms / 33,6 s / 2,7 cm 5,8° |
| S1 | 3 | Hybrid A\* | 34,1 s | 176 ms / 31,2 s / 2,0 cm 1,5° | 281 ms / 34,1 s / 2,9 cm 4,8° |
| S1 | 3 | DQfD | 33,6 s | 67 ms / 33,6 s / 1,3 cm 3,7° | 67 ms / 32,0 s / 3,7 cm 8,9° |
| S2 | 1 | Hybrid A\* | 28,6 s | 134 ms / 26,2 s / 6,2 cm 0,1° | 171 ms / 28,6 s / 2,3 cm 3,8° |
| S2 | 1 | DQfD | 29,0 s | 64 ms / 22,4 s / 2,9 cm **28,8°** | 51 ms / 29,0 s / 1,3 cm 3,9° |
| S2 | 2 | DQfD | 29,4 s | 57 ms / 27,5 s / 6,3 cm 2,4° | 56 ms / 29,4 s / 1,0 cm 3,3° |
| S2 | 2 | Hybrid A\* | 28,1 s | 192 ms / 25,6 s / 5,8 cm 7,5° | 201 ms / 28,1 s / 0,8 cm 3,9° |
| S2 | 3 | Hybrid A\* | 28,6 s | 206 ms / 27,0 s / 5,8 cm 4,1° | 165 ms / 28,6 s / 1,1 cm 3,0° |
| S2 | 3 | DQfD | 28,3 s | 60 ms / 22,6 s / 3,2 cm **27,6°** | 55 ms / 28,3 s / 0,9 cm 2,6° |
| S3 | 1 | Hybrid A\* | 26,2 s | 195 ms / 23,9 s / 2,8 cm **20,9°** | 219 ms / 26,2 s / 1,4 cm 1,3° |
| S3 | 1 | DQfD | 26,7 s | 56 ms / 26,7 s / 1,4 cm 13,8° | 59 ms / 25,7 s / 1,6 cm 2,4° |
| S3 | 2 | DQfD | 26,8 s | 53 ms / 26,8 s / 1,8 cm 14,6° | 69 ms / 26,5 s / 1,6 cm 2,6° |
| S3 | 2 | Hybrid A\* | 26,8 s | 159 ms / 26,8 s / 1,8 cm 14,2° | 220 ms / 26,5 s / 1,4 cm 0,9° |
| S3 | 3 | Hybrid A\* | 26,9 s | 194 ms / 26,9 s / 2,8 cm 15,3° | 160 ms / 26,2 s / 1,2 cm 2,4° |
| S3 | 3 | DQfD | 26,4 s | 53 ms / 26,4 s / 1,9 cm 14,7° | 53 ms / 25,7 s / 1,9 cm 3,5° |

![[Uji nyata Gudang2 2026-10-24 S1.png]]
![[Uji nyata Gudang2 2026-10-24 S2.png]]
![[Uji nyata Gudang2 2026-10-24 S3.png]]

Garis penuh: ID 01. Garis putus-putus: ID 02. Kotak titik-titik: pose awal. Kotak putus-putus: pose tujuan.

### Pengamatan lintasan

- **S1**: kedua perencana memberi jalur yang sama. Robot mundur lurus sekitar 110–120 cm, berbalik dengan satu titik balik, lalu maju masuk ke tujuan (3 ganti arah). Makespan 33–34 s, terlama karena mundur jauh.
- **S2 ID 01**: Hybrid A\* selalu lewat jalur bawah (sekitar 197 cm). DQfD dua kali memilih **jalur atas** dekat dinding (191 cm, tempuh 22,4 / 22,6 s, lebih cepat sekitar 4 s dari A\*), dan sekali jalur bawah. Makespan tetap sama karena ditentukan oleh ID 02 (sekitar 28–29 s).
- **S3 ID 01**: jalurnya sama, kecuali Hybrid A\* ulangan 1 yang memutar lewat atas (188 cm, 1 ganti arah).

### Galat ID 01 di tujuan yang menghadap dinding atas

| Tujuan ID 01 | Hybrid A\* | DQfD |
|---|---|---|
| S2 (35, 40, −90°), sudut kiri atas | selalu bergeser **5,4 cm ke samping** (akhir x ≈ 40,4) | dua kali berhenti **miring 28–29°** (yaw −61°/−62°), sekali bergeser 6,0 cm ke samping |
| S3 (205, 40, −90°) | 14–21° | 14–15°, konsisten di yaw sekitar −75° |

- **Penyebab (diperiksa 2026-10-24): segmen lurus pendekatan tidak muat.** Hybrid A\* menutup jalur dengan segmen lurus `approach_len` = 32,4 cm. Model dekat DQfD meniru demonstrator itu. Di (35, 40, −90°) dan (205, 40, −90°), badan robot di awal segmen itu (titik acuan di y ≈ 72, badan 23 cm ke belakang, ditambah margin 8 cm) menyentuh **ujung papan pembatas** (x 45 / 194, y 102,5). Akibatnya:
  - `_feasible_approach_dirs` mengembalikan `[]`;
  - RS menyambung langsung ke tujuan;
  - di **semua** rencana ID 01 untuk S2 dan S3 (kedua perencana), jalur masih berbelok **68–71° dalam 30 cm terakhir**.
  
  Di semua tujuan lain, 30 cm terakhir jalur lurus (0°). Pelacak tiba sambil masih berbelok, sehingga berhenti miring atau bergeser ke samping.
- Pendekatan lurus muat bila tujuan digeser ke **y ≤ 34**: (35, 34, −90°) dan (205, 34, −90°) sudah diperiksa muat, dengan margin penuh. Ini kesalahan rancangan skenario, bukan kelemahan salah satu perencana.
- Galat ujung rencana kedua perencana tetap 0. Galat muncul karena pelacak harus mengikuti tikungan sampai titik tujuan.
- Di tujuan ID 01 lainnya (S1, menghadap 180°) galatnya kecil (1,3–2,3 cm / 0,7–6,6°).
- ID 02 tidak mengalami ini. Bias arah hadap ID 02 juga lebih kecil daripada di Gudang (rata-rata 3,5–4,3°, dibanding 5–8°).

## 5. Kesimpulan

1. **Rancangan berhasil.** Di Gudang2, DQfD murni merencanakan 3,4× lebih cepat daripada Hybrid A\* + RS di robot nyata (58 vs 198 ms, 36/36 rencana). Di Gudang, A\* yang 5× lebih cepat. Keunggulan komputasi bergantung pada **jenis rute**:
   - robot membelakangi tujuan dan tujuannya jauh → DQfD lebih cepat;
   - robot menghadap tujuan → Hybrid A\* lebih cepat.
2. **Rentang waktu DQfD lebih sempit.** Rencana pertama di kedua peta:
   - DQfD 48–235 ms (sekitar 5× antara tercepat dan terlambat);
   - Hybrid A\* 12–301 ms (sekitar 25×), tergantung seberapa banyak robot harus memutar arah.
   
   Di dalam Gudang2 saja, DQfD 51–69 ms. Untuk batas waktu kendali, perilaku DQfD lebih mudah diprediksi.
3. **Komputasi tidak tampak di makespan.** Waktu rencana < 1% waktu tempuh, dan jalur kedua perencana hampir identik, sehingga makespan sama (29,6 s).
4. **Keamanan dan keandalan setara.** 18/18 tiba, tanpa estop. Jarak terdekat ke zona: 9,4 cm (A\*) dan 9,2 cm (DQfD).
5. **Presisi** sama baiknya untuk ID 02. Galat ID 01 di S2/S3 berasal dari rancangan skenario: pendekatan lurus ke tujuan terhalang ujung pembatas, sehingga kedua perencana sama-sama berakhir di tikungan. Perbaikannya: geser kedua tujuan itu ke y ≤ 34.

> [!warning] Keterbatasan
> - Peta dan skenario dipilih **khusus** untuk keunggulan komputasi DQfD. Hasilnya tidak boleh digeneralisasi menjadi "DQfD lebih cepat". Bersama Gudang, kesimpulan yang adil adalah: **tergantung jenis rute**.
> - n = 3 per sel. Tetapi untuk waktu rencana, tidak ada tumpang-tindih sama sekali (DQfD maks 69 ms < A\* min 134 ms).
> - Penyebab DQfD melambat sekitar 3× di robot (vs 1,8× untuk A\*) belum diperiksa.

## 6. Sumber angka

| Angka | Sumber |
|---|---|
| Bagian 3–4 | `Forklift Robot Program/uji_nyata/20261024_082405_Gudang2/hasil.jsonl`, `ringkasan.json`, `ringkasan_tabel.tex`, `rincian_tabel.tex` |
| Waktu offline | `rincian_ulang.json` di folder yang sama (`python -m drl.uji_robot_nyata --rinci ...`), 36/36 jalur cocok |
| Pola rute (bagian 1) | 505 rute uji `drl/data/demo_jauh_id0N.npz` di Kalibrasi, Rumah2, Rumah; `dqfd-murni` vs `astar`, median 2 kali ulang, offline |
| Pemilihan peta (bagian 1) | 4 kandidat × 12 skenario acak; validasi 3 skenario dengan 13 pose awal per robot |
| Gambar | `lintasan_S1–S3.png` di folder hasil (disalin ke `Gambar/`) |
