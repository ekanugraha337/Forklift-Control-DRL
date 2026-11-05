# Peta Gudang2 baru: simulasi skenario silang (wasit bekerja), Hybrid A\* + RS vs DQfD murni

Tanggal: 2026-11-05 · Branch `ver-1-5`, kode commit `4d26e77` · Lanjutan dari [[18 Skenario Gudang Baru dan Simulasi Makespan serta Waktu Proses|Skenario Gudang Baru dan Simulasi Makespan serta Waktu Proses]] · Wasit: [[05 Multi Robot Navigation and Control|Multi Robot Navigation and Control]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Peta Gudang2 diganti dengan rancangan baru: pulau rak di tengah, teluk parkir di dinding bawah, dan slot ambil di dinding atas. Berbeda dengan skenario Gudang di catatan 18, yang sengaja dibuat tanpa interaksi, skenario Gudang2 ini **sengaja membuat jalur kedua robot bersilangan**. Tanpa wasit, kedua robot akan bertabrakan, sehingga wasit harus memutuskan siapa yang menunggu, memutar, atau minggir.

Catatan ini merekam peta, skenario, dan hasil **simulasi** (`--simulasi --ulang 2`, bagian 1–7) dan **uji robot nyata** (`--ulang 3`, bagian 8) dengan parameter yang sama seperti catatan 18: makespan, galat pose akhir, dan waktu proses per bagian (Hybrid A\*, RS, model jauh, model dekat). Ditambah ukuran khusus wasit dan titik konflik.

> [!summary] Ringkasan: simulasi (`20261105_061315_Gudang2_sim`, 2 ulangan) dan robot nyata (`20261105_070646_Gudang2`, 3 ulangan)
> - **Wasit bekerja di semua 10 percobaan, tanpa sentuhan dan tanpa estop.** Jarak badan terdekat kedua robot 8,0–10,5 cm di simulasi dan 7,0–9,7 cm di robot nyata; tanpa wasit 0 cm.
> - **Titik konflik praktis sama** di simulasi dan robot nyata: di bawah pulau, di antara kedua teluk dalam. Rata-ratanya (121, 145) vs (122, 149) cm, selisih 3,7 cm, terjadi ±7 s setelah tujuan diberikan (bagian 8.6).
> - **Makespan rata-rata** (Hybrid A\* + RS vs DQfD murni):
>
>   | | Simulasi | Robot nyata |
>   |---|---|---|
>   | Hybrid A\* + RS | 43,45 s (52,26 / 34,64) | 37,11 s (34,80 / 40,84 / 35,70) |
>   | DQfD murni | **30,61 s** (34,23 / 27,00) | 37,39 s (48,37 / 32,43 / 31,37) |
>   | Selisih DQfD vs A\* | **−30%** | **+1%** (setara) |
>
>   Makespan di sini mengukur **perencana dan wasit sekaligus**, dan selalu ditentukan robot yang mengalah.
>   - Di simulasi, A\* ulangan 1 memutari pulau (+18 s), dan robot yang mengalah berganti-ganti.
>   - Di robot nyata tidak ada jalan memutar. **Robot 02 mengalah di 5 dari 6 percobaan.** Satu percobaan DQfD memakan 48,37 s karena dua rencana ulang gagal dan robot 01 menunggu 11,5 s.
> - **Galat pose akhir** (posisi / arah, rata-rata):
>
>   | | Simulasi | Robot nyata |
>   |---|---|---|
>   | Hybrid A\* + RS | 1,92 cm / 7,30° (maks 4,10 cm / 17,10°) | 1,64 cm / 4,55° (maks 2,98 cm / 9,05°) |
>   | DQfD murni | 3,33 cm / 6,28° (maks 4,99 cm / 10,20°) | **1,10 cm / 2,85°** (maks 2,10 cm / 6,73°) |
>
> - **Waktu proses rencana pertama:** simulasi A\* 17,8 vs DQfD 15,9 ms; robot nyata A\* 20,8 vs DQfD **13,4 ms**. Bagian dekat: RS 2,2–3,0 ms vs model dekat 0,4–0,5 ms. Jam dinding = CPU.
> - **Rencana ulang akibat wasit:** simulasi A\* 15 / DQfD 21 kali, robot nyata A\* 41 / DQfD 35 kali. Rencana ulang DQfD yang gagal umumnya mahal (0,75–1,54 s), sedangkan A\* gagal dalam ≤ 1 ms. Loop kendali tidak terganggu (celah terbesar 0,11 s).
> - **n = 2 (simulasi) dan 3 (robot nyata).** Keputusan wasit (siapa mengalah, minggir atau memutar) sangat memengaruhi makespan.

---

## 1. Peta dan skenario

### 1.1 Peta Gudang2 (`Camera/peta_constraint/Gudang2.json`)

![[Peta Gudang2 2026-11-05.png]]

| Zona | Fungsi | x (cm) | y (cm) |
|---|---|---|---|
| Z1–Z4 | dinding luar, sisi dalam **tepat** di garis ubin 0 / 240 / 0 / 210 | – | – |
| P1 | pulau rak 60 × 30 | 90 – 150 | 75 – 105 |
| T1, T2, T3 | sekat teluk bawah (tebal 5 cm, panjang 30 cm) | 57,5–62,5 · 117,5–122,5 · 177,5–182,5 | 180 – 210 |
| A1, A2 | sekat slot atas | 57,5–62,5 · 177,5–182,5 | 0 – 30 |

Ruang yang terbentuk:
- lorong kiri dan kanan 90 cm;
- pita atas 75 cm (y 0–75) dan pita bawah 105 cm (y 105–210, termasuk teluk sedalam 30 cm);
- teluk selebar 57,5 / 55 / 55 / 57,5 cm, dan tiga slot atas.

**Mengapa pulau 60 × 30, bukan 60 × 60:** robot yang menghadap tegak dengan margin 8 cm butuh ruang di atas teluk dalam (x = 90 dan 150) supaya bisa masuk dengan garis lurus. Dengan pulau 60 × 60 (y 75–135), bagian lurus terakhir rencana menuju teluk dalam hanya ±2 cm. Dengan 60 × 30 (y 75–105), bagian lurus itu 30–35 cm, sama seperti menuju teluk luar. Angka ini dari perencanaan offline (`draf_gudang2_v2.py`).

### 1.2 Skenario S1: silang X di bawah pulau (`Forklift Robot Program/uji_nyata/Gudang2.json`)

Pose (x cm, y cm, yaw °). Titik acuan pose adalah poros depan. Yaw 90° berarti menghadap +y (ke bawah pada gambar).

| Robot | Awal | Tujuan | Jalur |
|---|---|---|---|
| 01 | slot kiri-atas (28,75; 40,0; 90) | **teluk dalam kanan** (150,0; 185,0; 90), masuk maju | turun lorong kiri, menyeberang diagonal ke kanan di bawah pulau |
| 02 | slot kanan-atas (211,25; 40,0; 90) | **teluk dalam kiri** (90,0; 185,0; 90), masuk maju | turun lorong kanan, menyeberang diagonal ke kiri di bawah pulau |

- **Jalur bersilangan.** Jarak kedua robot ke titik silang (x ≈ 120, di bawah pulau) hampir sama, sehingga keduanya tiba di sana hampir bersamaan.
- **Bukti akan bertabrakan tanpa wasit.** Bila kedua robot berangkat bersamaan dengan laju sama di sepanjang rencana pertamanya, jarak badannya menjadi **0 cm** setelah 142 cm tempuh (DQfD) atau 168 cm tempuh (Hybrid A\*).
- **Masuk teluk maju** sedalam y = 185, dengan bagian lurus terakhir 30–35 cm. Di tujuan, kedua badan berjarak 42,5 cm, dipisahkan sekat T2.
- Semua pose muat sebagai tujuan dengan margin 8 cm, termasuk bila badan robot lain berada di pose awalnya. Rencana kedua perencana dan jalur pulang (Hybrid A\*, 283 dan 307 cm) berhasil.

### 1.3 Skenario dan heatmap Dijkstra

Panel kiri menampilkan pose awal (titik-titik) dan tujuan (garis putus). Panel tengah dan kanan adalah heuristik Dijkstra ke tujuan setiap robot (`_holonomic_heuristic`, sel 5 cm), yang dipakai Hybrid A\* dan lingkungan DQfD. Kondisinya tanpa badan robot lain sebagai rintangan, sama dengan saat uji.

![[Skenario Gudang2 2026-11-05 S1.png]]

| Robot | Biaya Dijkstra di titik awal | Rencana pertama Hybrid A\* (ulangan 1 / 2) | Rencana pertama DQfD murni |
|---|---:|---:|---:|
| 01 | 197 cm | 245,5 / 240,1 cm (+22–25%) | 217,3 / 229,7 cm (+10–17%) |
| 02 | 195 cm | 289,8 / 237,3 cm (+22–49%) | 230,1 / 241,9 cm (+18–24%) |

Semua rencana pertama punya 2 ganti arah, yaitu koreksi akhir di dekat tujuan.

## 2. Metode pengukuran

Sama dengan [[18 Skenario Gudang Baru dan Simulasi Makespan serta Waktu Proses|catatan 18]] bagian 2:
- protokol pulang → uji, urutan perencana diselang-seling;
- tujuan kedua robot diberikan bersamaan;
- satu proses perencana per robot;
- `hasil.jsonl` mencatat `ms`, `cpu_ms`, `rincian_ms`, `mulai_ms`, `ipc_ms`, dan `pid` untuk **setiap** rencana, termasuk rencana ulang dari wasit.

```powershell
python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang2.json --simulasi --ulang 2
```

**Parameter wasit** (`konfig.py`, rancangan di `wasit.py` dan [[05 Multi Robot Navigation and Control|catatan 05]]):

| Parameter | Nilai | Arti |
|---|---:|---|
| `MULTI_JARAK_AMAN_CM` | 5 cm | jarak bebas yang harus tersisa antara bayangan satu robot dan badan robot lain |
| `MULTI_T_REAKSI_S`, `MULTI_LIHAT_MIN_CM` | 0,2 s, 5 cm | panjang bayangan = 5 cm + v·0,2 s + jarak pengereman |
| `MULTI_TUNGGU_KALAH_S` | 1 s | bila berhadapan, robot yang kalah prioritas merencana ulang setelah sekian detik |
| `MULTI_TUNGGU_MAKS_S` | 5 s | setelah menunggu sekian detik, jalur dihitung ulang memutari semua robot lain |
| `MULTI_MINGGIR_JARAK_CM` | 35 / 50 / 70 / 90 cm | jarak calon pose minggir dari jalur lawan |
| `MULTI_MINGGIR_CALON`, `MULTI_MINGGIR_BEBAS_CM` | 10, 10 cm | jumlah calon pose minggir, dan jarak bebas minimum dari jalur lawan |
| `MULTI_MINGGIR_MAKS_S` | 30 s | batas lama minggir |

Ukuran tambahan untuk wasit:
- **waktu menunggu** (`t_menunggu_s`) dan **jumlah minggir** (`n_minggir`) per robot;
- **jumlah rencana** (`n_rencana`) dan rencana ulang (terhitung, gagal, total, dan terlama);
- **jarak tempuh aktual** (`jarak_tempuh_cm`) dibanding panjang rencana pertama;
- **jarak badan kedua robot terhadap waktu**, dari `lintasan.csv`.

## 3. Makespan

Run `20261105_061315_Gudang2_sim`. 4/4 percobaan selesai, semua robot tiba, tanpa estop.

| | Hybrid A\* + RS | DQfD murni |
|---|---:|---:|
| Makespan ulangan 1 | 52,26 s | 34,23 s |
| Makespan ulangan 2 | 34,64 s | 27,00 s |
| **Makespan rata-rata** | 43,45 s | **30,61 s (−30%)** |

| Percobaan | Robot | Waktu tempuh | Rencana pertama | Jarak tempuh aktual | Menunggu | Minggir | Rencana |
|---|---|---:|---:|---:|---:|---:|---:|
| A\* ulangan 1 | 01 | **52,26 s** | 245,5 cm | **563,5 cm** | 7,45 s | 0 | 2 |
| A\* ulangan 1 | 02 | 32,63 s | 289,8 cm | 287,1 cm | 0 s | 0 | 1 |
| A\* ulangan 2 | 01 | 21,16 s | 240,1 cm | 237,1 cm | 1,95 s | 0 | 1 |
| A\* ulangan 2 | 02 | 34,64 s | 237,3 cm | 321,7 cm | 1,20 s | 0 | 15 |
| DQfD ulangan 1 | 01 | 34,23 s | 217,3 cm | 270,2 cm | 0,80 s | **1** | 15 |
| DQfD ulangan 1 | 02 | 21,94 s | 230,1 cm | 215,4 cm | 5,10 s | 0 | 1 |
| DQfD ulangan 2 | 01 | 18,03 s | 229,7 cm | 214,8 cm | 4,00 s | 0 | 1 |
| DQfD ulangan 2 | 02 | 27,00 s | 241,9 cm | 318,6 cm | 0,80 s | **1** | 8 |

![[Simulasi Gudang2 2026-11-05 S1.png]]

- **Makespan ditentukan robot yang mengalah.** Robot yang lewat lebih dulu tiba dalam 18–33 s. Robot yang menunggu, memutar, atau minggir tiba 27–52 s.
- **A\* ulangan 1 paling lambat** karena robot 01 memutari pulau dari atas (gambar kiri). Rencana memutar itu 423 cm, dan robot 01 menempuh total 564 cm, 2,3× rencana pertamanya.
- Di tiga percobaan lainnya, jarak tempuh robot yang mengalah 24–36% lebih panjang daripada rencana pertamanya, karena keluar jalur untuk minggir lalu kembali.

### 3.1 Galat pose akhir

Cara ukur sama dengan catatan 18 bagian 3.1: rata-rata 1 s setelah berhenti, diuraikan menjadi komponen **maju** (+ melewati tujuan, − berhenti sebelum tujuan) dan **samping**.

| Percobaan | Robot | Posisi | Arah | Maju | Samping |
|---|---|---:|---:|---:|---:|
| A\* ulangan 1 | 01 | 4,10 cm | **17,10°** | +1,06 cm | −3,96 cm |
| A\* ulangan 1 | 02 | 2,70 cm | 5,54° | −2,38 cm | −1,29 cm |
| A\* ulangan 2 | 01 | 0,40 cm | 4,44° | +0,08 cm | +0,39 cm |
| A\* ulangan 2 | 02 | 0,46 cm | 2,12° | −0,46 cm | +0,01 cm |
| DQfD ulangan 1 | 01 | 1,94 cm | 2,89° | −1,06 cm | −1,62 cm |
| DQfD ulangan 1 | 02 | 1,57 cm | 2,48° | −0,07 cm | +1,57 cm |
| DQfD ulangan 2 | 01 | 4,99 cm | 9,53° | +3,49 cm | −3,56 cm |
| DQfD ulangan 2 | 02 | 4,83 cm | 10,20° | +2,88 cm | +3,88 cm |

| Ringkasan (4 pose per perencana) | Hybrid A\* + RS | DQfD murni |
|---|---:|---:|
| Galat posisi: rata-rata / median / maks | **1,92** / 1,58 / 4,10 cm | 3,33 / 3,39 / 4,99 cm |
| Galat arah: rata-rata / median / maks | 7,30 / 4,99 / 17,10° | **6,28** / 6,21 / 10,20° |
| Komponen maju: rata-rata \|maju\| (rata-rata bertanda) | 1,00 (−0,43) cm | 1,88 (+1,31) cm |
| Komponen samping: rata-rata \|samping\| / maks | 1,41 / 3,96 cm | 2,66 / 3,88 cm |
| Jarak terdekat ke zona | 7,4–11,1 cm | 6,2–13,5 cm |
| Galat awal | 0 (percobaan pertama) – 3,9 cm | 3,6–3,8 cm |

- **Galat terbesar muncul setelah interaksi:** robot 01 A\* ulangan 1 tiba miring 17,1° setelah jalan memutar, dan DQfD ulangan 2 tiba dengan 4,8–5,0 cm / 9,5–10,2°. Di Gudang tanpa interaksi (catatan 18), galat simulasi rata-rata 2,5–2,7 cm / 4,4–4,5°.
- Percobaan dengan galat kecil (A\* ulangan 2: ≤ 0,5 cm) dan besar terjadi pada skenario yang sama. Dengan n = 2, perbandingan galat antar-perencana belum bisa disimpulkan.

## 4. Waktu proses

### 4.1 Rencana pertama (saat tujuan diberikan)

| Bagian | Hybrid A\* + RS | DQfD murni |
|---|---:|---:|
| **Pencarian jauh:** Hybrid A\* / model jauh | **14,66 ms** (83%) | 15,44 ms (97%) |
| **Penyelesaian dekat:** RS / model dekat | 3,02 ms (17%) | **0,39 ms** (2%) |
| Lain-lain | 0,07 ms | 0,07 ms |
| **Total (jam dinding), rata-rata / median** | 17,76 / 17,99 ms | **15,90 / 14,14 ms** |
| **Total (CPU), rata-rata** | 17,74 ms | **15,87 ms** |

| Percobaan | Robot | Mulai | Jam / CPU | Hybrid A\* | RS | Model jauh | Model dekat |
|---|---|---:|---:|---:|---:|---:|---:|
| A\* ulangan 1 | 01 / 02 | +1,1 / +2,2 ms | 18,2 / 17,8 ms | 14,97 / 11,38 | 3,10 / 6,36 | – | – |
| A\* ulangan 2 | 01 / 02 | +0,8 / +1,7 ms | 18,3 / 16,8 ms | 15,94 / 16,36 | 2,29 / 0,33 | – | – |
| DQfD ulangan 1 | 01 / 02 | +0,8 / +1,9 ms | 14,0 / 23,5 ms | – | – | 13,54 / 23,00 | 0,42 / 0,41 |
| DQfD ulangan 2 | 01 / 02 | +0,8 / +1,8 ms | 14,3 / 11,8 ms | – | – | 13,84 / 11,37 | 0,36 / 0,38 |

- Di skenario ini, **bagian jauh Hybrid A\* sedikit lebih cepat** daripada model jauh (14,7 vs 15,4 ms). DQfD unggul total karena **RS lebih lambat** di sini (3,0 ms, sampai 6,4 ms; di Gudang 0,3–0,4 ms), sedangkan model dekat tetap ±0,4 ms. Penyelesaian dekat ke teluk dalam lebih sulit bagi RS.
- IPC 0,62–0,90 ms per rencana. Rencana dikerjakan pid 34640 (robot 01) dan 46656 (robot 02), yaitu proses perencana masing-masing.

### 4.2 Rencana ulang akibat wasit

| Percobaan | Robot | Rencana ulang | Gagal | Total waktu | Terlama | Jenis |
|---|---|---:|---:|---:|---:|---|
| A\* ulangan 1 | 01 | 1 | 0 | 1 767 ms | **1 767 ms** | memutari robot lain (423 cm) |
| A\* ulangan 2 | 02 | 14 | 1 | 156 ms | 41 ms | rencana minggir dan rencana lanjut |
| DQfD ulangan 1 | 01 | 14 | 2 | 2 762 ms | **1 540 ms** (gagal) | rencana minggir dan rencana lanjut |
| DQfD ulangan 2 | 02 | 7 | 1 | 2 156 ms | 821 ms | rencana minggir dan rencana lanjut |
| **Jumlah** | | A\* **15** (1 gagal), DQfD **21** (3 gagal) | | A\* 1,9 s, DQfD 4,9 s | | |

- Rencana ulang terjadi di proses perencana. Meski satu rencana memakan 1,5–1,8 s, **loop kendali tetap berdetak** (celah terbesar sepanjang run 0,11 s).
- Rencana ulang DQfD yang **gagal** butuh 0,75–1,54 s, karena model mencoba sampai anggarannya habis. Rencana ulang A\* yang gagal selesai dalam 0,7 ms.

## 5. Kerja wasit

![[Wasit Gudang2 2026-11-05 S1.png]]

Garis hitam adalah jarak badan kedua robot selama UJI. Bilah berwarna adalah keadaan robot 01 (R01) dan robot 02 (R02): menunggu, merencana, minggir, selesai. Garis titik merah adalah jarak aman wasit 5 cm.

| Percobaan | Urutan kejadian (log) | Jarak terdekat |
|---|---|---:|
| **A\* ulangan 1** | Robot 01 *"menunggu – robot 02 ada di jalur"* (bayangannya mendekati badan robot 02). Setelah 5,0 s *"jalur dihitung ulang memutari robot lain"*: jalur 423 cm (1 767 ms), yang memutari pulau dari atas. Robot 02 terus jalan tanpa menunggu | 8,0 cm (+31,2 s) |
| **A\* ulangan 2** | **Berhadapan.** Robot 02 *"memberi jalan ke robot 01"*, robot 01 menunggu. Setelah 1,5 s, robot 02 mendapat jalur **minggir** 56 cm. Robot 01 lewat lalu tiba, dan robot 02 *"robot 01 sudah diam – mencoba langsung ke tujuan"* (157 cm) | 9,9 cm (+8,9 s) |
| **DQfD ulangan 1** | **Berhadapan.** Robot 01 memberi jalan. Setelah 1,0 s, robot 01 mendapat jalur **minggir** 45 cm lalu *"menepi"*. Robot 02 lewat dan tiba, robot 01 *"selesai minggir (0,7 s)"*, lalu merencana 143 cm ke tujuan | 9,9 cm (+15,0 s) |
| **DQfD ulangan 2** | **Berhadapan**, kebalikan ulangan 1. Robot 02 memberi jalan, **minggir** 50 cm, dan menepi 1,3 s. Robot 01 lewat dan tiba, lalu robot 02 merencana 144 cm ke tujuan | 10,5 cm (+19,6 s) |

- **Tidak ada sentuhan dan tidak ada estop.** Jarak badan tidak pernah di bawah 8 cm, lebih besar dari jarak aman 5 cm.
- **Dua cara wasit menyelesaikan konflik:**
  - **menunggu lalu memutar** (A\* ulangan 1): mahal, +18 s dan +318 cm;
  - **berhadapan lalu yang kalah minggir** (tiga percobaan lain): lebih murah, robot yang minggir tiba +9 sampai +14 s setelah robot yang lewat.

  Cara mana yang terjadi bergantung pada geometri saat kedua bayangan bertemu. Di A\* ulangan 1, badan robot 02 sudah berada di jalur robot 01, jadi robot 01 menunggu dengan aturan "A→B" (bayangan A mendekati badan B), bukan aturan berhadapan.
- **Robot yang mengalah berganti-ganti:** robot 01 di A\* ulangan 1 dan DQfD ulangan 1, robot 02 di A\* ulangan 2 dan DQfD ulangan 2. Aturan prioritas wasit (robot dengan sisa jalur lebih pendek didahulukan) bergantung pada posisi saat konflik, bukan pada perencananya.
- **Saat pulang pun wasit ikut bekerja**, karena jalur pulang juga bersilangan, tetapi fase pulang tidak diukur.

## 6. Kesimpulan dan langkah berikutnya

1. **Skenario silang berhasil memaksa wasit bekerja**, dan wasit mencegah tabrakan di keempat percobaan (jarak terdekat 8,0–10,5 cm).
2. **Makespan DQfD lebih kecil (−30%)**, tetapi selisihnya terutama ditentukan oleh **keputusan wasit**: satu jalan memutar di A\* ulangan 1 (+18 s), bukan oleh kualitas rencana pertama. Rencana pertama A\* dan DQfD sama-sama 217–290 cm.
3. **Waktu proses rencana pertama hampir setara** (A\* 17,8 vs DQfD 15,9 ms). Di Gudang2, bagian jauh A\* sedikit lebih cepat, sedangkan RS lebih lambat daripada model dekat. Rencana ulang DQfD yang gagal mahal (0,75–1,54 s).
4. **Galat pose akhir membesar setelah interaksi** (sampai 17° / 5 cm).
5. Langkah berikutnya:
   - ~~Uji robot nyata dengan skenario ini~~: sudah dilakukan (`--ulang 3`), lihat bagian 8.
   - Tambah ulangan (`--ulang 5`), supaya terlihat seberapa sering wasit memilih memutar vs minggir untuk setiap perencana.

> [!warning] Batasan
> - **n = 2, simulasi tanpa derau.** Keputusan wasit (memutar atau minggir, siapa yang mengalah) berubah antar-percobaan, jadi rata-rata makespan sangat dipengaruhi satu kejadian.
> - **Makespan di sini bukan ukuran perencana saja**, melainkan perencana + wasit + rencana ulang. Untuk membandingkan perencana saja, gunakan skenario tanpa interaksi (catatan 18).
> - **Titik start:** percobaan pertama (A\* ulangan 1) dimulai tepat di pose awal; percobaan lain dari pose hasil pulang (galat awal 3,6–3,9 cm).

## 7. Sumber angka

| Angka | Sumber |
|---|---|
| Peta | `Camera/peta_constraint/Gudang2.json` (pulau 60 × 30) |
| Skenario | `Forklift Robot Program/uji_nyata/Gudang2.json`, salinannya di folder hasil (`skenario.json`) |
| Bagian 1.1 (pulau 60 × 30) | skrip sementara di luar repo `draf_gudang2_v2.py` (panjang bagian lurus terakhir rencana untuk pulau 60 × 60 dan 60 × 30) |
| Bagian 1.2 (bukti tabrakan) | skrip sementara `draf_gudang2_silang.py` (jarak badan pada waktu yang sama di sepanjang rencana pertama) dan `periksa_gudang2.py` (pemeriksaan lewat `Sesi`) |
| Bagian 1.3, gambar peta, skenario, linimasa wasit | skrip sementara `gambar_gudang2.py` |
| Bagian 3–5 | `Forklift Robot Program/uji_nyata/20261105_061315_Gudang2_sim/` (`hasil.jsonl`, `lintasan.csv`, `ringkasan.json`, `lintasan_S1.png`) dan log simulasi, direkap dengan skrip sementara `analisis_u2.py` |
| Parameter wasit | `Forklift Robot Program/konfig.py` (bagian MULTI-ROBOT), `wasit.py` |

---

## 8. Uji robot nyata

Bagian ini terpisah dari simulasi di bagian 1–7. Peta, skenario, dan metodenya sama, tetapi dengan robot ID 01 dan ID 02 sungguhan, pose dari kamera AprilTag, dan perintah lewat MQTT.

### 8.1 Pengaturan dan jalannya uji

| | |
|---|---|
| Program | `python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang2.json --ulang 3` (tanpa `--simulasi`) |
| Folder hasil | `Forklift Robot Program/uji_nyata/20261105_070646_Gudang2`, 2026-11-05 pukul 07:07–07:16 (6 percobaan) |
| Konfirmasi | operator menekan `n` sebelum setiap percobaan |
| Perencana | satu proses per robot: pid 17948 (robot 01) dan 27952 (robot 02) di setiap rencana, termasuk rencana ulang |

- **Semua 6 percobaan selesai**, dan semua robot tiba. Tidak ada estop dan tidak ada sentuhan.
- **Pulang:** di setiap percobaan kedua robot tiba di pose awal pada percobaan pulang **pertama**, padahal jalur pulangnya juga bersilangan. Tidak ada campur tangan operator. Galat awal 2,5–3,8 cm.
- **Kamera:** tag kedua robot terlihat di **100%** baris fase UJI. Hanya dua baris berupa ramalan, untuk robot 02.
- **Loop kendali:** celah terbesar selama UJI 0,11 s, dan periode median 64 ms, sama dengan uji lain.
- Run sebelumnya tidak dipakai: `20261105_065405_Gudang2` (3 percobaan, terhenti), `20261105_070514_Gudang2` (1 percobaan), serta `20261105_070159_Gudang2` dan `20261105_070347_Gudang2` (kosong).

### 8.2 Makespan

| | Hybrid A\* + RS | DQfD murni |
|---|---:|---:|
| Makespan ulangan 1 | 34,80 s | 48,37 s |
| Makespan ulangan 2 | 40,84 s | 32,43 s |
| Makespan ulangan 3 | 35,70 s | 31,37 s |
| **Makespan rata-rata** | **37,11 s** | 37,39 s (+1%) |

| Percobaan | Robot | Waktu tempuh | Rencana pertama | Jarak tempuh aktual | Menunggu | Minggir | Rencana |
|---|---|---:|---:|---:|---:|---:|---:|
| A\* ulangan 1 | 01 | 23,35 s | 243,7 cm | 240,1 cm | 4,50 s | 0 | 1 |
| A\* ulangan 1 | 02 | **34,80 s** | 242,2 cm | 338,1 cm | 0,80 s | **1** | 14 |
| A\* ulangan 2 | 01 | 23,53 s | 242,1 cm | 238,3 cm | 4,75 s | 0 | 1 |
| A\* ulangan 2 | 02 | **40,84 s** | 246,8 cm | 355,4 cm | 0,80 s | **1** | 15 |
| A\* ulangan 3 | 01 | 23,06 s | 242,7 cm | 246,0 cm | 3,00 s | 0 | 1 |
| A\* ulangan 3 | 02 | **35,70 s** | 243,5 cm | 335,7 cm | 0,80 s | **1** | 15 |
| DQfD ulangan 1 | 01 | 29,18 s | 231,6 cm | 223,9 cm | **11,50 s** | 0 | 1 |
| DQfD ulangan 1 | 02 | **48,37 s** | 245,9 cm | 370,3 cm | 4,80 s | **1** | 7 |
| DQfD ulangan 2 | 01 | **32,43 s** | 220,4 cm | 299,7 cm | 0,80 s | **1** | 16 |
| DQfD ulangan 2 | 02 | 22,64 s | 259,1 cm | 262,1 cm | 2,10 s | 0 | 1 |
| DQfD ulangan 3 | 01 | 18,92 s | 226,3 cm | 209,9 cm | 4,95 s | 0 | 1 |
| DQfD ulangan 3 | 02 | **31,37 s** | 228,1 cm | 317,1 cm | 0,80 s | 0 | 15 |

![[Uji nyata Gudang2 2026-11-05 S1.png]]

- **Makespan kedua perencana setara** (37,11 vs 37,39 s). Rencana pertama keduanya mirip (A\* 242–247 cm, DQfD 220–259 cm), dan semuanya bersilangan di bawah pulau.
- **Makespan selalu ditentukan robot yang mengalah**, yang tiba 31,4–48,4 s, sedangkan robot yang lewat tiba 18,9–29,2 s. Jarak tempuh robot yang mengalah 36–51% lebih panjang daripada rencana pertamanya, karena keluar jalur untuk minggir atau memutar lalu kembali.
- **Percobaan terlama, DQfD ulangan 1 (48,37 s):** robot 01 menunggu 11,5 s. Robot 02 membutuhkan 6 rencana ulang, dua di antaranya gagal (terlama 1,08 s), sebelum minggir.

### 8.3 Galat pose akhir

| Percobaan | Robot | Posisi | Arah | Maju | Samping |
|---|---|---:|---:|---:|---:|
| A\* ulangan 1 | 01 | 1,18 cm | 2,83° | −0,43 cm | +1,10 cm |
| A\* ulangan 1 | 02 | 1,83 cm | 7,12° | −0,75 cm | +1,67 cm |
| A\* ulangan 2 | 01 | 1,24 cm | 2,28° | −0,38 cm | +1,19 cm |
| A\* ulangan 2 | 02 | 2,98 cm | 9,05° | −0,70 cm | −2,90 cm |
| A\* ulangan 3 | 01 | 1,59 cm | 4,77° | +1,00 cm | +1,24 cm |
| A\* ulangan 3 | 02 | 1,04 cm | 1,22° | −1,03 cm | +0,17 cm |
| DQfD ulangan 1 | 01 | 0,46 cm | 0,24° | −0,02 cm | −0,46 cm |
| DQfD ulangan 1 | 02 | 1,04 cm | 0,11° | −1,04 cm | +0,06 cm |
| DQfD ulangan 2 | 01 | 1,03 cm | 3,37° | −0,00 cm | −1,03 cm |
| DQfD ulangan 2 | 02 | 1,34 cm | 6,18° | −0,47 cm | −1,26 cm |
| DQfD ulangan 3 | 01 | 2,10 cm | 6,73° | −0,24 cm | −2,09 cm |
| DQfD ulangan 3 | 02 | 0,61 cm | 0,46° | −0,23 cm | −0,57 cm |

| Ringkasan (6 pose per perencana) | Hybrid A\* + RS | DQfD murni |
|---|---:|---:|
| Galat posisi: rata-rata / median / maks | 1,64 / 1,42 / 2,98 cm | **1,10** / 1,04 / 2,10 cm |
| Galat arah: rata-rata / median / maks | 4,55 / 3,80 / 9,05° | **2,85** / 1,92 / 6,73° |
| Komponen maju: rata-rata \|maju\| (rata-rata bertanda) | 0,72 (−0,38) cm | 0,33 (−0,33) cm |
| Komponen samping: rata-rata \|samping\| / maks | 1,38 / 2,90 cm | 0,91 / 2,09 cm |
| Jarak terdekat ke zona | 8,6–14,8 cm | 7,5–15,6 cm |

- **Semua pose akhir ≤ 3,0 cm dari tujuan.** DQfD sedikit lebih presisi, tetapi selisihnya kecil dibanding sebaran antar-percobaan.
- **Robot hampir selalu berhenti sedikit sebelum tujuan** (maju negatif di 11 dari 12 pose), sama polanya dengan Gudang (catatan 18).

### 8.4 Waktu proses

**Rencana pertama** (6 rencana per perencana):

| Bagian | Hybrid A\* + RS | DQfD murni |
|---|---:|---:|
| **Pencarian jauh:** Hybrid A\* / model jauh | 18,47 ms (89%) | **12,80 ms** (96%) |
| **Penyelesaian dekat:** RS / model dekat | 2,19 ms (11%) | **0,48 ms** (4%) |
| Lain-lain | 0,10 ms | 0,09 ms |
| **Total (jam dinding), rata-rata / median** | 20,76 / 18,68 ms | **13,37 / 14,59 ms** |
| **Total (CPU), rata-rata** | 20,67 ms | **13,31 ms** |

| Percobaan | Robot | Mulai | Jam / CPU | Hybrid A\* | RS | Model jauh | Model dekat |
|---|---|---:|---:|---:|---:|---:|---:|
| A\* ulangan 1 | 01 / 02 | +1,3 / +2,6 ms | 16,4 / 34,3 ms | 14,23 / 31,62 | 2,10 / 2,58 | – | – |
| A\* ulangan 2 | 01 / 02 | +1,3 / +2,1 ms | 16,4 / 18,2 ms | 13,47 / 14,29 | 2,86 / 3,77 | – | – |
| A\* ulangan 3 | 01 / 02 | +0,9 / +1,6 ms | 20,0 / 19,2 ms | 19,70 / 17,49 | 0,23 / 1,59 | – | – |
| DQfD ulangan 1 | 01 / 02 | +1,2 / +2,1 ms | 13,4 / 9,3 ms | – | – | 12,90 / 8,75 | 0,44 / 0,42 |
| DQfD ulangan 2 | 01 / 02 | +1,7 / +2,8 ms | 15,8 / 8,7 ms | – | – | 15,00 / 8,14 | 0,67 / 0,43 |
| DQfD ulangan 3 | 01 / 02 | +1,0 / +2,1 ms | 17,0 / 16,1 ms | – | – | 16,46 / 15,52 | 0,51 / 0,43 |

**Rencana ulang akibat wasit:**

| Percobaan | Robot | Rencana ulang | Gagal | Total waktu | Terlama |
|---|---|---:|---:|---:|---:|
| A\* ulangan 1 | 02 | 13 | 1 | 335 ms | 115 ms |
| A\* ulangan 2 | 02 | 14 | 1 | 454 ms | 95 ms |
| A\* ulangan 3 | 02 | 14 | 1 | 375 ms | 123 ms |
| DQfD ulangan 1 | 02 | 6 | 2 | 1 933 ms | **1 084 ms** |
| DQfD ulangan 2 | 01 | 15 | 1 | 91 ms | 15 ms |
| DQfD ulangan 3 | 02 | 14 | 1 | 2 282 ms | **1 420 ms** |
| **Jumlah** | | A\* **41** (3 gagal), DQfD **35** (4 gagal) | | A\* 1,2 s, DQfD 4,3 s | |

- **DQfD lebih cepat di rencana pertama** (13,4 vs 20,8 ms rata-rata), baik di bagian jauh maupun dekat. Di simulasi bagian jauh A\* sedikit lebih cepat; di robot nyata model jauh yang lebih cepat (12,8 vs 18,5 ms).
- **Rencana ulang A\* murah dan konsisten** (terlama 123 ms; yang gagal selesai dalam 1 ms). **Rencana ulang DQfD kadang sangat mahal:** tiga dari empat yang gagal butuh 0,79–1,08 s, dan satu rencana memutar yang berhasil (DQfD ulangan 3) butuh 1,42 s.
- IPC 0,66–2,03 ms per rencana. Robot 01 mulai merencana +0,9–1,7 ms setelah permintaan, robot 02 +1,6–2,8 ms.

### 8.5 Kerja wasit

![[Wasit uji nyata Gudang2 2026-11-05 S1.png]]

| Percobaan | Yang mengalah | Kejadian (dari keadaan robot dan `hasil.jsonl`) | Jarak terdekat |
|---|---|---|---:|
| A\* ulangan 1 | **robot 02** | konflik +7,0 s; robot 02 merencana ulang (13×) lalu **minggir** ±6 s; robot 01 menunggu 4,5 s, lalu lewat dan tiba 23,4 s | 7,0 cm |
| A\* ulangan 2 | **robot 02** | konflik +6,9 s; robot 02 merencana ulang (14×) lalu minggir; robot 01 menunggu 4,8 s | 7,2 cm |
| A\* ulangan 3 | **robot 02** | konflik +7,4 s; robot 02 merencana ulang (14×) lalu minggir ±7 s; robot 01 menunggu 3,0 s | 9,4 cm |
| DQfD ulangan 1 | **robot 02** | konflik +6,9 s; robot 02 merencana ulang (6×, 2 gagal, sampai 1,08 s); robot 01 menunggu **11,5 s**; robot 02 minggir, lalu menunggu lagi 4,8 s | 7,8 cm |
| DQfD ulangan 2 | **robot 01** | konflik +9,2 s; robot 01 merencana ulang (15×) lalu **minggir**; robot 02 menunggu 2,1 s, lalu lewat dan tiba 22,6 s | 9,7 cm |
| DQfD ulangan 3 | **robot 02** | konflik +6,9 s; robot 02 merencana ulang (14×, sampai 1,42 s) dan **memutar tanpa minggir** (317 cm); robot 01 menunggu 5,0 s | 7,8 cm |

- **Tidak ada sentuhan dan tidak ada estop.** Jarak badan terdekat 7,0–9,7 cm, sedikit lebih dekat daripada di simulasi (8,0–10,5 cm), tetapi tetap di atas jarak aman 5 cm.
- **Robot 02 mengalah di 5 dari 6 percobaan**, sedangkan di simulasi yang mengalah berganti-ganti (2 : 2). Dugaan: di catatan 18 robot 02 nyata lebih lambat daripada robot 01, sehingga saat konflik posisinya lebih tidak menguntungkan menurut aturan prioritas wasit. Ini belum ditelusuri.
- **Tidak ada jalan memutar mengelilingi pulau** seperti A\* ulangan 1 di simulasi. Konflik selalu diselesaikan dengan minggir atau memutar pendek di bawah pulau.

### 8.6 Simulasi vs robot nyata

| | Simulasi: Hybrid A\* + RS / DQfD | Robot nyata: Hybrid A\* + RS / DQfD |
|---|---|---|
| Jumlah percobaan | 2 / 2 | 3 / 3 |
| **Makespan rata-rata** | 43,45 / **30,61 s** | **37,11** / 37,39 s |
| Makespan, rentang | 34,64–52,26 / 27,00–34,23 s | 34,80–40,84 / 31,37–48,37 s |
| Robot yang lewat lebih dulu tiba | 21,2–32,6 / 18,0–21,9 s | 23,1–23,5 / 18,9–29,2 s |
| Waktu menunggu total per percobaan | 7,45; 3,15 / 5,90; 4,80 s | 5,30; 5,55; 3,80 / 16,30; 2,90; 5,75 s |
| Penyelesaian konflik | memutar 1, minggir 1 (rencana) / minggir 2 | minggir 3 / minggir 2, memutar 1 |
| Yang mengalah: robot 01 / robot 02 | 1 / 1 dan 1 / 1 | 0 / 3 dan 1 / 2 |
| Rencana ulang (gagal) | 15 (1) / 21 (3) | 41 (3) / 35 (4) |
| **Galat posisi rata-rata (maks)** | 1,92 (4,10) / 3,33 (4,99) cm | 1,64 (2,98) / **1,10** (2,10) cm |
| **Galat arah rata-rata (maks)** | 7,30 (17,10) / 6,28 (10,20)° | 4,55 (9,05) / **2,85** (6,73)° |
| Waktu proses rencana pertama | 17,76 / 15,90 ms | 20,76 / **13,37 ms** |
| Jarak badan terdekat | 8,0–9,9 / 9,9–10,5 cm | 7,0–9,4 / 7,8–9,7 cm |
| Galat awal | 0–3,9 cm | 2,5–3,8 cm |

**Titik konflik.** Karena wasit mencegah tabrakan, tidak ada titik tabrakan sungguhan. Yang dibandingkan:
- **prediksi titik tabrakan**: titik tengah kedua robot saat badannya pertama kali bersentuhan, bila keduanya berangkat bersamaan dan melaju sama cepat di sepanjang rencana pertamanya tanpa wasit;
- **titik konflik**: titik tengah kedua robot saat wasit pertama kali menahan salah satunya.

| Uji | Percobaan | Prediksi titik tabrakan | Titik konflik | Saat konflik | Yang ditahan |
|---|---|---|---|---:|---|
| Simulasi | A\* ulangan 1 | (133, 151) | (126, 161) | +7,7 s | robot 01 |
| Simulasi | A\* ulangan 2 | (118, 157) | (124, 147) | +7,0 s | robot 02 |
| Simulasi | DQfD ulangan 1 | (125, 138) | (109, 133) | +8,5 s | robot 01 |
| Simulasi | DQfD ulangan 2 | (124, 145) | (124, 142) | +6,7 s | robot 02 |
| Nyata | A\* ulangan 1 | (120, 158) | (123, 155) | +7,0 s | robot 02 |
| Nyata | A\* ulangan 2 | (122, 159) | (125, 154) | +6,9 s | robot 02 |
| Nyata | A\* ulangan 3 | (120, 159) | (121, 159) | +7,4 s | robot 02 |
| Nyata | DQfD ulangan 1 | (125, 148) | (124, 145) | +6,9 s | robot 02 |
| Nyata | DQfD ulangan 2 | (136, 153) | (114, 147) | +9,2 s | robot 01 |
| Nyata | DQfD ulangan 3 | (120, 141) | (126, 133) | +6,9 s | robot 02 |

| Rata-rata | Simulasi | Robot nyata | Selisih |
|---|---|---|---:|
| Prediksi titik tabrakan | (125, 148) | (124, 153) | 5,7 cm |
| Titik konflik | (121, 145) | (122, 149) | **3,7 cm** |

1. **Titik konflik di simulasi dan robot nyata praktis sama**: di bawah pulau, di antara kedua teluk dalam (x 109–126, y 133–161), sekitar 7 s setelah tujuan diberikan. Selisih rata-ratanya 3,7 cm, lebih kecil daripada sebaran antar-percobaan (±15 cm). Simulasi meramalkan dengan baik di mana dan kapan kedua robot akan berebut ruang.
2. **Titik silang bergantung pada perencananya, dan konsisten di kedua uji.** Jalur A\* bersilangan lebih ke bawah (prediksi y 151–159), sedangkan jalur DQfD lebih ke atas (y 138–153).
3. **Titik jarak terdekat berbeda dari titik konflik.** Jarak terdekat sering terjadi belakangan, misalnya saat robot yang mengalah kembali dari pose minggir melewati robot lain yang sudah parkir. Contohnya A\* ulangan 1 nyata pada +27 s di dekat teluk robot 01.
4. **Makespan:** keunggulan DQfD −30% di simulasi tidak muncul di robot nyata (+1%). Keunggulan di simulasi berasal dari satu jalan memutar A\* (ulangan 1) yang tidak terjadi di robot nyata. Di robot nyata, DQfD justru punya percobaan terlama (48,37 s) karena rencana ulang yang gagal.
5. **Galat pose akhir lebih kecil di robot nyata** untuk kedua perencana (A\* 1,64 vs 1,92 cm, DQfD 1,10 vs 3,33 cm). Galat besar di simulasi datang dari percobaan dengan jalan memutar (A\* ulangan 1, 17,1°) dan dari DQfD ulangan 2, di mana kedua robot tiba miring ±10°. Dengan n = 2–3, ini belum bisa disimpulkan sebagai sifat perencana.

### 8.7 Kesimpulan uji nyata

1. **Wasit bekerja dengan andal di robot nyata.** Keenam konflik diselesaikan tanpa sentuhan (jarak terdekat ≥ 7 cm), di titik yang sama seperti yang diramalkan simulasi.
2. **Makespan Hybrid A\* dan DQfD setara di robot nyata** (37,1 vs 37,4 s). Pada skenario dengan interaksi, makespan lebih ditentukan oleh cara wasit menyelesaikan konflik daripada oleh perencananya.
3. **DQfD lebih presisi** (1,10 cm / 2,85° vs 1,64 cm / 4,55°) dan **lebih cepat merencana** (13,4 vs 20,8 ms rencana pertama), tetapi rencana ulangnya bisa mahal (0,79–1,42 s), sedangkan rencana ulang A\* ≤ 123 ms.

> [!warning] Batasan uji nyata
> - **n = 3 per perencana, satu sesi.** Satu kejadian (misalnya rencana ulang yang gagal) menggeser rata-rata makespan sampai ±11 s.
> - **Robot 02 mengalah di 5 dari 6 percobaan**, jadi perilaku wasit belum seimbang antar-robot.
> - **Galat awal 2,5–3,8 cm** di setiap percobaan, karena robot dimulai dari pose hasil pulang.

### 8.8 Sumber angka

| Angka | Sumber |
|---|---|
| Semua angka bagian 8 | `Forklift Robot Program/uji_nyata/20261105_070646_Gudang2/` (`hasil.jsonl`, `lintasan.csv`, `ringkasan.json`, `lintasan_S1.png`), direkap dengan skrip sementara `analisis_u2.py` |
| Linimasa wasit, keterlihatan tag, celah loop | `lintasan.csv` (kolom `state`, `terlihat`, `diramalkan`, `t`), skrip sementara `gambar_wasit.py` |
| Titik konflik dan prediksi titik tabrakan (8.6) | `lintasan.csv` (keadaan `MENUNGGU` pertama di fase UJI) dan `rencana[0].jalur_cm` di `hasil.jsonl` kedua run, dihitung dengan skrip sementara |
