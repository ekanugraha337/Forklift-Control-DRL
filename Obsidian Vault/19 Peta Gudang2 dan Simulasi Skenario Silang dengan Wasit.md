# Peta Gudang2 baru: simulasi skenario silang (wasit bekerja), Hybrid A\* + RS vs DQfD murni

Tanggal: 2026-11-05 · Branch `ver-1-5`, kode commit `4d26e77` · Lanjutan dari [[18 Skenario Gudang Baru dan Simulasi Makespan serta Waktu Proses|Skenario Gudang Baru dan Simulasi Makespan serta Waktu Proses]] · Wasit: [[05 Multi Robot Navigation and Control|Multi Robot Navigation and Control]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Peta Gudang2 diganti dengan rancangan baru: pulau rak di tengah, teluk parkir di dinding bawah, dan slot ambil di dinding atas. Berbeda dengan skenario Gudang di catatan 18, yang sengaja dibuat tanpa interaksi, skenario Gudang2 ini **sengaja membuat jalur kedua robot bersilangan**. Tanpa wasit, kedua robot akan bertabrakan, sehingga wasit harus memutuskan siapa yang menunggu, memutar, atau minggir.

Catatan ini merekam peta, skenario, dan hasil **simulasi** (`--simulasi --ulang 2`) dengan parameter yang sama seperti catatan 18: makespan, galat pose akhir, dan waktu proses per bagian (Hybrid A\*, RS, model jauh, model dekat). Ditambah ukuran khusus wasit. Uji robot nyata menyusul.

> [!summary] Ringkasan (run `20261105_061315_Gudang2_sim`, 1 skenario × 2 perencana × 2 ulangan)
> - **Wasit bekerja di keempat percobaan, tanpa sentuhan.** Jarak badan terdekat kedua robot 8,0–10,5 cm; tanpa wasit 0 cm. Tidak ada estop.
>   - **Hybrid A\* ulangan 1:** robot 01 menunggu 5 s, lalu **memutari pulau dari atas** (jalur baru 423 cm, robot menempuh 564 cm).
>   - **Tiga percobaan lain:** kedua robot berhadapan, dan robot yang mengalah **minggir** (dua kali di DQfD, sekali sebagai rencana minggir di A\*).
> - **Makespan:**
>
>   | | Hybrid A\* + RS | DQfD murni |
>   |---|---|---|
>   | Ulangan 1 / 2 → rata-rata | 52,26 / 34,64 → 43,45 s | 34,23 / 27,00 → **30,61 s** (−30%) |
>
>   Makespan di sini mengukur **perencana dan wasit sekaligus**. Satu jalan memutar A\* menambah ±18 s.
> - **Waktu proses rencana pertama** (4 rencana per perencana): A\* 17,8 ms vs DQfD **15,9 ms**. Bagian jauh: Hybrid A\* **14,7 ms** vs model jauh 15,4 ms. Bagian dekat: RS 3,0 ms vs model dekat **0,39 ms**. Jam dinding = CPU (selisih ≤ 0,1 ms).
> - **Rencana ulang akibat wasit:** A\* 15 kali (1 gagal, terlama 1,77 s), DQfD 21 kali (3 gagal, terlama 1,54 s). Loop kendali tidak terganggu (celah terbesar 0,11 s), karena perencana berjalan di proses sendiri.
> - **Galat pose akhir** lebih besar daripada di Gudang tanpa interaksi: A\* 1,92 cm / 7,30° (maks 4,10 cm / **17,10°**), DQfD 3,33 cm / 6,28° (maks 4,99 cm / 10,20°).
> - **n = 2, simulasi tanpa derau.** Hasil wasit sangat bergantung pada waktu kedua robot tiba di titik silang.

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
   - **Uji robot nyata** dengan skenario ini:
     ```powershell
     python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang2.json --ulang 2
     ```
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
