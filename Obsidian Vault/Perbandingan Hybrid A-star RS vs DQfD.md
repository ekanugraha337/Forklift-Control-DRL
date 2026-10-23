# Perbandingan DQfD: branch RS-optimize vs ver-1-4

Tanggal: 2026-10-23 · Branch `RS-optimize` (diukur 2026-10-02/03) dan `ver-1-4` (diukur 2026-10-23) · Struktur model ver-1-4: [[Struktur DQfD Full]]

Catatan ini hanya membandingkan perencana DQfD dari kedua branch. Hybrid A\* + Reeds-Shepp muncul sebagai pembanding yang **diukur di branch yang sama**, karena kedua branch memakainya sebagai demonstrator dan acuan.

> [!summary] Ringkasan
> - **Waktu**: DQfD ver-1-4 lebih cepat dari DQfD RS-optimize di p90 (109 / 96 ms vs 267 / 276 ms) dan rata-rata (91 / 115 ms vs 151 / 145 ms) pada tiga peta yang sama. Median hampir sama (23 / 20 ms vs 19 / 29 ms).
> - **Pendaratan 4–7× lebih murah**: RS-optimize mencoba Reeds-Shepp di 66–81 simpul per rute. ver-1-4 hanya mencoba 9–10 kali per rute, di dalam 60 cm.
> - **Mutu jalur**: biaya p90 vs A\* 1,19 / 1,17× (ver-1-4) vs 1,27 / 1,29× (RS-optimize). Tiba 99,7 / 99,7% vs 96,8 / 95,5% di set uji masing-masing.
> - **Terhadap Hybrid A\* di branch-nya sendiri**: RS-optimize unggul jauh di p90 (0,37–0,39×), karena Hybrid A\*-nya lambat dengan radius belok besar. ver-1-4 masih unggul di p90 (0,73–0,79×), tetapi Hybrid A\* dengan kemudi 55° sendiri sudah 5× lebih cepat di p90.
> - **Belum sebanding**: profil kemudi dan dataset berbeda (bagian 2). Uji jalan tertutup ber-derau dan uji robot nyata baru ada untuk RS-optimize.

---

## 1. Dua branch

| | RS-optimize | ver-1-4 |
|---|---|---|
| Model per robot | **satu** model DQfD | **dua**: model jauh + model dekat |
| Masukan / keluaran | 47 / 15 (14 primitif + aksi SAMBUNG) | jauh 47 / 14 primitif; dekat 9 / 88 kata Reeds-Shepp |
| Bagian akhir jalur | aksi SAMBUNG = `_analytic` Reeds-Shepp, hanya ≤ 100 cm dari tujuan | model dekat: kata RS diurutkan menurut Q, dicek tabrakan berurutan; panjang segmen dari rumus RS |
| Kapan mencoba mendarat | setiap simpul ≤ 100 cm (aksi SAMBUNG ikut dinilai pencarian) | < 60 cm, peta Dijkstra bebas, lalu cek pendaratan; gagal = model jauh lanjut |
| Pencarian operasional | terbaik-dulu, batas waktu 2 s | gabungan (terbaik-dulu, lalu mundur), batas waktu 2 s |
| Galat akhir rencana | 0 (kurva RS) | 0 (kurva RS) |
| Profil kemudi | ID 01 35°, ID 02 22° (R rencana 45,6 / 76,0 cm) | 55° kedua robot (R rencana 25,7 cm) |

## 2. Data dan cara mengukur

> [!warning] Yang membuat angka absolut tidak sepenuhnya sebanding
> - **Profil kemudi berbeda.** ver-1-4 memakai profil setelah perbaikan mekanisme kemudi (55°). Radius belok yang jauh lebih kecil membuat Hybrid A\* dan DQfD sama-sama lebih mudah di ruang sempit.
> - **Dataset dan rute berbeda.** Tiap branch membangkitkan dataset untuk profilnya sendiri, jadi rute ujinya berbeda.
> - Karena itu, bandingkan terutama **rasio terhadap Hybrid A\* di pengukuran yang sama** dan **porsi waktu per bagian**.

| Pengukuran | RS-optimize | ver-1-4 |
|---|---|---|
| Set uji (evaluasi) | seluruh set uji, terbaik-dulu, batas waktu 2 s | `drl.evaluasi`, rute penuh: 379 (ID 01) / 358 (ID 02), metode gabungan |
| Waktu per rute dan rincian per bagian | `drl/rincian_komputasi.py`: semua rute uji di Kalibrasi, Rumah2, Rumah (317 / 318 rute) | skrip dengan kategori yang sama: semua rute uji di tiga peta yang sama (266 / 241 rute) |

## 3. Hasil set uji

| | RS-optimize ID 01 / 02 | **ver-1-4 ID 01 / 02** |
|---|---|---|
| Tiba DQfD | 96,8 / 95,5% | **99,7 / 99,7%** |
| Tiba Hybrid A\* + RS (branch yang sama) | 95,8–100% per peta | 99,5 / 100% |
| Biaya vs A\*, median | 1,00 / 1,00 | 1,00 / 1,00 |
| Biaya vs A\*, p90 | 1,27 / 1,29× | **1,19 / 1,17×** |
| Ganti arah | lebih sering dari Hybrid A\* | rata-rata 1,66 / 1,67 (Hybrid A\* 1,43 / 1,43) |
| Skenario "tidak ada jalur" menurut A\* (150 per robot) | 29 / 54 jalur sah ditemukan | 1 / 3 jalur sah ditemukan, sisanya menyerah |

- ver-1-4 lebih sering tiba dan biaya p90-nya lebih rendah.
- RS-optimize menemukan lebih banyak jalur di skenario yang Hybrid A\* nyatakan buntu. Skenario negatifnya berbeda: di ver-1-4 skenario itu dipilih oleh Hybrid A\* dengan kemudi 55°, yang lebih jarang buntu. Kemungkinan karena itu skenario yang tersisa lebih jarang benar-benar punya jalan, tetapi ini belum diperiksa.

## 4. Waktu pada tiga peta yang sama

Kalibrasi, Rumah2, Rumah, semua rute uji, ms per rencana:

| | RS-optimize ID 01 | **ver-1-4 ID 01** | RS-optimize ID 02 | **ver-1-4 ID 02** |
|---|---|---|---|---|
| Rute | 317 | 266 | 318 | 241 |
| Hybrid A\*: tiba / median / p90 / rata | 97,8% / 8,3 / 692 / 233 | 100% / 15,8 / 137 / 59 | 98,7% / 22,2 / 741 / 243 | 99,2% / 13,5 / 132 / 82 |
| DQfD: tiba / median / p90 / rata | 99,1% / 18,8 / 267 / 151 | **100% / 23,4 / 109 / 91** | 97,8% / 28,5 / 276 / 145 | **100% / 19,8 / 96 / 115** |
| DQfD ÷ A\*, median | 2,27× | **1,48×** | 1,28× | 1,47× |
| DQfD ÷ A\*, p90 | 0,39× | 0,79× | 0,37× | 0,73× |
| DQfD ÷ A\*, rata-rata | 0,65× | 1,53× | 0,60× | 1,40× |

Per peta, DQfD median / p90 / rata (ms):

| Peta | RS-optimize ID 01 | ver-1-4 ID 01 | RS-optimize ID 02 | ver-1-4 ID 02 |
|---|---|---|---|---|
| Kalibrasi | 4,2 / 35 / 17 | 14,9 / 35 / 20 | 5,4 / 30 / 14 | 13,8 / 36 / 20 |
| Rumah2 | 13,5 / 95 / 80 | 21,0 / 49 / 49 | 32,3 / 219 / 143 | 13,9 / 45 / 71 |
| Rumah | 46,7 / 991 / 265 | 41,2 / 354 / 179 | 53,3 / 415 / 203 | 41,0 / 433 / 224 |

- **ver-1-4 lebih cepat di rute sulit**: p90 Rumah ID 01 991 → 354 ms, p90 Rumah2 ID 02 219 → 45 ms.
- **RS-optimize lebih cepat di rute mudah**: median Kalibrasi 4–5 ms vs 14–15 ms. ver-1-4 membayar ongkos tetap untuk heuristik Dijkstra dan beberapa langkah model jauh sebelum boleh mendarat di dalam 60 cm. RS-optimize bisa langsung SAMBUNG dari ≤ 100 cm.
- **Rasio terhadap A\*** menguntungkan RS-optimize, karena Hybrid A\*-nya sendiri lambat dengan radius belok lama (p90 ~700 ms). Dengan kemudi 55°, Hybrid A\* 5× lebih cepat di p90, jadi pembanding ver-1-4 jauh lebih berat.

## 5. Rincian waktu per bagian

Rata-rata ms per rute, tiga peta. ver-1-4 dipisah: bagian model jauh dan bagian model dekat (pendaratan).

| Bagian | RS-optimize ID 01 | **ver-1-4 ID 01** | RS-optimize ID 02 | **ver-1-4 ID 02** |
|---|---|---|---|---|
| Sinar (32 sinar per simpul) | 66,0 (43%) | **47,1 (50%)** | 51,4 (35%) | **62,4 (54%)** |
| Primitif (ekspansi + cek badan) | 24,7 (16%) | 21,5 (23%) | 21,4 (15%) | 22,9 (20%) |
| Jaringan | 15,6 (10%) | 8,1 (jauh 6,9 + dekat 1,2) | 13,5 (9%) | 10,7 (jauh 9,7 + dekat 1,0) |
| Pencarian lain (sisa) | 14,6 (10%) | 7,2 (8%) | 13,2 (9%) | 10,3 (9%) |
| Heuristik Dijkstra + observasi lain | 4,4 | 4,5 | 4,1 | 4,9 |
| **Pendaratan** | **Reeds-Shepp 26,9 (18%)** | **model dekat 7,1 (8%)** | **Reeds-Shepp 42,8 (29%)** | **model dekat 5,9 (5%)** |
| — percobaan per rute | 66 panggilan `_analytic` | 10,2 | 81 panggilan `_analytic` | 8,8 |
| — isinya | rumus RS + cek tabrakan, ≤ 6 kandidat termurah | cek tabrakan 3,2 · sisa 1,4 · jaringan 1,2 · rumus RS 0,8 · arah pendekatan 0,4 · observasi 0,3 | rumus RS + cek tabrakan, ≤ 6 kandidat termurah | cek tabrakan 2,5 · sisa 1,2 · jaringan 1,0 · rumus RS 0,7 · arah pendekatan 0,3 · observasi 0,2 |
| **Total** | **152,4** | **94,2** | **146,5** | **116,1** |

- **Pendaratan 4–7× lebih murah.** RS-optimize menilai SAMBUNG di setiap simpul ≤ 100 cm yang dibuka pencarian. ver-1-4 hanya mencoba mendarat di dalam 60 cm setelah saringan Dijkstra, ±0,65 ms per percobaan.
- **Jaringan setengahnya**: model jauh ver-1-4 membuka lebih sedikit simpul.
- **Sinar memakan separuh waktu di kedua branch, dan porsinya naik di ver-1-4** (50–54%), karena bagian lain sudah lebih murah. Ini target optimasi berikutnya. RS-optimize sudah mencatat bahwa ongkosnya ada di iterasi loop numpy, sehingga perlu kompilasi (mis. numba) atau observasi yang lebih ringan.

## 6. Uji jalan tertutup dan robot nyata

Kedua branch memakai uji yang **berbeda**, jadi tabel ini tidak dibandingkan baris per baris.

**RS-optimize** (`drl.uji_navigasi`, kedua robot, derau pose 1,2 cm, sekali jalan):

| Perencana | Satu robot | Pasangan | Masuk zona | Galat akhir rata / maks |
|---|---|---|---|---|
| Hybrid A\* | 28/30 | 11/12 | 0 | 4,1 / 6,1 cm |
| DQfD (`dqfd-murni`) | 29/30 | 10/12 | 0 | 4,3 / 6,7 cm |

Robot nyata, peta Rumah, dua robot bersamaan, satu jalan per perencana: DQfD tiba 7 dari 20 tujuan dengan 7 estop, Hybrid A\* 6 dari 18 dengan 5 estop. Rencana pertama DQfD 0,16–0,24 s, Hybrid A\* 0,7–1,3 s. Semua estop karena badan terukur menyentuh zona di koridor sempit.

**ver-1-4** (`drl.uji_pose_akhir`: `navigasi.py` tanpa jendela, pure pursuit, jeda kemudi, 20 Hz, tanpa derau, 3 skenario × 2 robot):

| Skenario | Perencana | Rencana | Panjang | Ganti arah | Galat ujung rencana | **Galat pose akhir robot** | Tempuh |
|---|---|---|---|---|---|---|---|
| ID 01 lapang (Kalibrasi) | Hybrid A\* + RS | 6 ms | 172 cm | 2 | 0 | 2,32 cm / 5,09° | 10,0 s |
| | DQfD | 44 ms | 170 cm | 2 | 0 | **0,85 cm / 0,06°** | 10,1 s |
| ID 01 balik (Rumah2) | Hybrid A\* + RS | 56 ms | 178 cm | 1 | 0 | 1,70 cm / 4,17° | 23,0 s |
| | DQfD | 61 ms | 224 cm | 2 | 0 | 3,43 cm / 2,54° | 18,0 s |
| ID 01 geser (Rumah) | Hybrid A\* + RS | 3 ms | 82 cm | 1 | 0 | 6,86 cm / 22,96° | 19,4 s |
| | DQfD | 5 ms | 82 cm | 1 | 0 | 6,87 cm / 22,98° | 19,4 s |
| ID 02 lapang (Kalibrasi) | Hybrid A\* + RS | 8 ms | 194 cm | 1 | 0 | 1,53 cm / 17,84° | 10,7 s |
| | DQfD | 17 ms | 179 cm | 0 | 0 | 3,44 cm / 17,90° | 12,9 s |
| ID 02 balik (Rumah2) | Hybrid A\* + RS | 12 ms | 202 cm | 1 | 0 | 3,35 cm / 1,99° | 15,2 s |
| | DQfD | 21 ms | 198 cm | 1 | 0 | 3,39 cm / 1,98° | 14,8 s |
| ID 02 geser (Rumah) | Hybrid A\* + RS | 3 ms | 104 cm | 1 | 0 | 3,08 cm / 18,75° | 12,7 s |
| | DQfD | 5 ms | 104 cm | 1 | 0 | 3,14 cm / 18,94° | 12,7 s |

- **12/12 jalan tiba**, tidak ada yang masuk zona. Galat ujung rencana 0 untuk kedua perencana.
- **Skenario geser: jalur DQfD identik dengan Hybrid A\* + RS.** Model dekat memilih kata RS yang sama dengan demonstrator.
- **Galat pose akhir robot hampir seluruhnya dari pelacakan**: besarnya sama untuk kedua perencana. Perbaikannya ada di pelacak, bukan di perencana.
- **Belum dilakukan untuk ver-1-4**: `drl.uji_navigasi` dengan derau (sebanding dengan tabel RS-optimize di atas) dan uji robot nyata.

![[Pose akhir ver-1-4 ID01 balik.png]]
![[Pose akhir ver-1-4 ID01 geser.png]]
![[Pose akhir ver-1-4 ID02 lapang.png]]

Gambar lain: `Gambar/Pose akhir ver-1-4 ID0N <skenario>.png`.

## 7. Kesimpulan

| | Lebih baik di |
|---|---|
| Tiba dan mutu jalur (set uji) | **ver-1-4**: tiba 99,7% vs 95,5–96,8%, biaya p90 1,17–1,19× vs 1,27–1,29× |
| Waktu rute sulit (p90, Rumah / Rumah2) | **ver-1-4**: p90 tiga peta 96–109 ms vs 267–276 ms |
| Waktu rute mudah (median Kalibrasi) | **RS-optimize**: 4–5 ms vs 14–15 ms |
| Ongkos pendaratan | **ver-1-4**: 6–7 ms vs 27–43 ms per rute |
| Jalur di skenario yang A\* nyatakan buntu | **RS-optimize**: 29 / 54 vs 1 / 3 (skenario berbeda) |
| Bukti di robot | **RS-optimize**: sudah diuji jalan tertutup ber-derau dan di robot nyata |

Pembagian dua model di ver-1-4 membuat pendaratan murah dan eksak. Sisa waktu DQfD di kedua branch didominasi sinar model jauh. Langkah berikutnya untuk ver-1-4:
1. jalankan `drl.uji_navigasi --perencana astar,dqfd-murni` supaya uji jalan tertutupnya sebanding dengan RS-optimize;
2. uji robot nyata;
3. optimasi sinar.

## 8. Sumber angka

| Angka | Sumber |
|---|---|
| RS-optimize: set uji, uji jalan tertutup, robot nyata, skenario negatif | Branch `RS-optimize`, `Obsidian Vault/Deep Reinforcement Learning.md`, bagian "Branch RS-optimize" dan "Kesimpulan umum" (2026-10-02/03) |
| RS-optimize: waktu per peta dan per bagian | `model_dqn/rincian_komputasi.json` (2026-10-02, `drl/rincian_komputasi.py` di branch RS-optimize) |
| ver-1-4: set uji | `model_dqn/evaluasi_id0N.json` (2026-10-23) |
| ver-1-4: waktu per peta dan per bagian | profil dengan kategori yang sama dengan `rincian_komputasi.py`, skrip sementara di luar repo (2026-10-23) |
| ver-1-4: navigasi simulasi | `model_dqn/pose_akhir/pose_akhir_id0N.json` (`python -m drl.uji_pose_akhir --id N`) |
| Struktur model ver-1-4 | [[Struktur DQfD Full]] |
