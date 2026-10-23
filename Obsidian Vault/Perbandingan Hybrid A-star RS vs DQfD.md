	# Perbandingan Hybrid A\* + Reeds-Shepp vs DQfD (model jauh + model dekat)

Tanggal: 2026-10-21 · Branch `ver-1-2` · Struktur model: [[Struktur DQfD Full]]

> [!success] Pembaruan 2026-10-23 (ver-1-4): model dekat memilih kata Reeds-Shepp + model jauh dilatih lanjut
> Model dekat versi 6 mengeluarkan Q untuk 88 kata RS; panjang segmen dari rumus RS, kata dicek tabrakan berurutan menurut Q. Model jauh dilatih lanjut dari ver-1-3 dengan pendaratan model dekat v6. Set uji, rute penuh, metode gabungan:
>
> | | ver-1-3 | **ver-1-4** | Hybrid A\* + RS |
> |---|---|---|---|
> | Tiba ID 01 / ID 02 | 97,6 / 96,1% | **99,7 / 99,7%** | 99,5 / 100% |
> | Waktu median | 101 / 343 ms | **22 / 20 ms** | 17 / 15 ms |
> | Waktu p90 | 1.732 / 2.366 ms | **111 / 129 ms** | 150 / 138 ms |
> | Biaya p90 | 1,81 / 1,55× | **1,19 / 1,17×** | 1 |
> | Galat akhir rencana | ±0,7 cm / 0,9° | **0** | 0 |
>
> Bagian dekat kini setara Reeds-Shepp (rute dekat saja: tiba 100%, 2,6 ms, biaya 1,00×). Perbandingan dengan branch RS-optimize, rincian waktu per bagian, dan uji navigasi simulasi: **bagian 10**. Angka bagian 1–9 di bawah adalah versi 4 (ver-1-2).

> [!tip] Pembaruan 2026-10-22 (ver-1-3): model dekat sadar rintangan
> Angka di catatan ini adalah **versi 4** (model dekat 9 masukan). Di ver-1-3, model dekat diberi 32 sinar + 6 ruang manuver (47 masukan) dan dilatih mulai dari bobot v4, sedangkan model jauh tetap v4. Detail: [[Struktur DQfD Full]] bagian 0.3.
>
> | | v4 (catatan ini) | **v5** | Hybrid A\* + RS |
> |---|---|---|---|
> | Tiba ID 01 / ID 02 | 96,9 / 97,8% | **99,0 / 98,4%** | 99,8 / 100% |
> | Gagal dekat | 3,0 / 2,1% | **0,8 / 1,5%** | – |
> | Rumah | 90,7 / 93,0% | **97,7 / 94,5%** | 100% |
> | Biaya median / p90 | 1,04 / 1,71 · 1,04 / 1,58× | **1,02 / 1,45 · 1,02 / 1,50×** | 1 |
> | Waktu median | 233 / 191 ms | **49 / 136 ms** | 3,4 / 4,6 ms |
> | Tiba ≤ 2 s | 88,5 / 89,0% | **95,0 / 93,3%** | 99,8 / 100% |
>
> Hasil ini menjawab saran perbaikan nomor 2 di bagian 8. Rincian waktu per bagian (bagian 3, 4, dan 6) belum diukur ulang untuk v5.

Perbandingan dipasangkan per bagian, sesuai yang ditiru masing-masing model:

| Bagian jalur | Demonstrator | Peniru |
|---|---|---|
| Jauh: dari start sampai sekitar 60 cm dari tujuan | **Hybrid A\*** (primitif gerak) | **Model jauh** |
| Dekat: dari titik serah sampai tujuan | **Reeds-Shepp** (kurva analitik) | **Model dekat** |

> [!summary] Ringkasan
> **Model jauh vs Hybrid A\***: setara.
> - Titik serahnya sama (median 56 cm dari tujuan), panjang jalur median 1,00× Hybrid A\*, tidak pernah gagal jauh di sampel ini (0,1% di set uji penuh).
> - Waktunya median **18–19 ms vs 11–13 ms**, p90 sekitar 100–110 ms untuk keduanya.
> - Model jauh hampir tidak mencari: median 15–18 langkah = jumlah primitif jalurnya, sedangkan Hybrid A\* mengekspansi 74–91 simpul.
>
> **Model dekat vs Reeds-Shepp**: mutu setara, waktu dan keandalan kalah jauh.
> - Dari pose start yang sama, jalur model dekat **sama panjang** (median 0,99×, lebih pendek di sekitar 60% rute) dan biayanya median 1,01–1,02×.
> - Galat akhir ≤ 1 cm & 2°, sedangkan RS eksak.
> - Waktunya median **78 ms vs 0,2 ms**, sekitar 400× lebih lambat.
> - Separuh percobaan pendaratannya gagal, dan setiap kegagalan menghabiskan sekitar 0,56 s. Dari titik serah model jauh (rute penuh), percobaan yang berhasil hanya 36–39%.
> - Model dekat memakan **87–92% waktu DQfD**. Reeds-Shepp hanya 8–10% waktu Hybrid A\*.
>
> **Keseluruhan**: Hybrid A\* + RS tiba 99,8 / 100%, DQfD 96,9 / 97,8% (ID 01 / ID 02). Waktu median 3,6–4,5 ms vs 191–233 ms. Dalam batas waktu robot 2 s, DQfD hanya tiba di sekitar 89% rute.

---

## 1. Data dan cara mengukur

| Data | Isi |
|---|---|
| **Evaluasi penuh** (`python -m drl.evaluasi --id 1,2`) | Semua rute uji: ID 01 828 rute, ID 02 808 rute (rute penuh 379 / 358, rute dekat saja 449 / 450), + 150 skenario negatif per robot. DQfD metode gabungan tanpa batas waktu; Hybrid A\* batas 2 s seperti di robot. Dipakai untuk bagian 2 dan 5 |
| **Pengukuran per bagian** | 192 rute uji per robot: 24 per kombinasi peta × asal (96 rute penuh + 96 rute dekat saja). Setelan sama. Dipakai untuk bagian 3 dan 4 |
| **Profil waktu** | 96 rute uji per robot, setiap fungsi diberi pengukur waktu eksklusif (jumlah bagian = 99,3% waktu total). Dipakai untuk bagian 6 |

- Kedua perencana mendapat rute, margin, dan rintangan (badan robot lain) yang sama persis.
- CPU, PyTorch 1 thread, sama dengan `navigasi.py`. GPU tidak dipakai saat perencanaan.
- Hybrid A\* di pengukuran per bagian dan profil diukur dengan `plan()` biasa, median dari 3 kali. Di evaluasi penuh ia diukur lewat perekam biaya, jadi sedikit lebih lambat.

**Cara memotong waktu dan jalur per bagian:**

| | Bagian jauh | Bagian dekat |
|---|---|---|
| Hybrid A\* + RS | Waktu total − waktu semua tembakan RS (Dijkstra ikut di sini); jalur sampai titik sambung RS | Waktu semua tembakan RS, termasuk yang terhalang; jalur ekor RS |
| DQfD | Waktu total − waktu di dalam percobaan pendaratan (Dijkstra ikut di sini); jalur model jauh | Waktu di dalam semua percobaan pendaratan, termasuk yang gagal; jalur model dekat |

| | Hybrid A\* + RS | DQfD |
|---|---|---|
| Bagian jauh | Pencarian A\* atas 14 primitif (7 kemudi × maju/mundur, 7,3 cm), heuristik Dijkstra | **Model jauh**: 47 masukan → Q 14 primitif yang sama, 145.807 parameter |
| Peralihan | Tembakan RS di setiap simpul < 60 cm; diambil begitu bebas tabrakan | Di setiap pose < 60 cm yang lolos saringan Dijkstra, model dekat mencoba mendarat; maks 12 percobaan × 0,6 s |
| Bagian dekat | **Reeds-Shepp**: satu rumus, sampel pose, cek tabrakan | **Model dekat**: 9 masukan → Q 30 segmen RS (R/S/L × maju/mundur × 5 panjang), 138.143 parameter, sampai ≤ 1 cm & 2° |
| Model yang diuji | – | ID 01: jauh langkah 60.000, dekat 90.000. ID 02: jauh 95.000, dekat 90.000 (setelah DAgger + nilai akhir pendaratan) |
| Ukuran / muat | – | 1,77 + 1,67 MB, dimuat 6–11 ms sekali di awal |

---

## 2. Hasil keseluruhan (evaluasi penuh)

| | ID 01 DQfD | ID 01 A\* + RS | ID 02 DQfD | ID 02 A\* + RS |
|---|---|---|---|---|
| **Tiba** | **96,9%** | 99,8% | **97,8%** | 100% |
| Gagal di **model jauh** / di **model dekat** | 0,1 / 3,0% | – | 0,1 / 2,1% | – |
| Kalibrasi / Rumah / Rumah2 / sintetis | 100 / 90,7 / 99,5 / 98,0% | 100 / 100 / 100 / 99,2% | 100 / 93,0 / 98,9 / 99,2% | 100% semua |
| Rute penuh / rute dekat saja | 94,2 / 99,1% | 99,5 / 100% | 96,1 / 99,1% | 100 / 100% |
| Biaya total vs A\* + RS, median / p90 | 1,04 / 1,71× | 1 | 1,04 / 1,58× | 1 |
| Ganti arah per rute, rata-rata | 3,78 | 1,35 | 3,15 | 1,38 |
| Galat akhir, rata / maks | 0,67 cm, 0,83° / 1,00 cm, 1,98° | 0 | 0,68 cm, 0,79° / 1,00 cm, 1,99° | 0 |
| Jalur tiba yang tidak sah (periksa independen) | 0 | – | 0 | – |
| Negatif: ikut buntu, jalur ditemukan (sah) | 98,7%, 2 (2) | – | 94,7%, 8 (8) | – |
| **Waktu median / p90** | **233 / 2.324 ms** | **4,5 / 83 ms** | **191 / 2.204 ms** | **3,6 / 58 ms** |

Hampir semua kegagalan DQfD terjadi di **model dekat**: ID 01 25 dari 26, ID 02 17 dari 18. Sebagian besar di Rumah (20 dan 14 rute).

---

## 3. Model jauh vs bagian Hybrid A\*

Rute penuh (start > 60 cm), keduanya tiba: ID 01 88 rute, ID 02 92 rute.

### 3.1 Performa

| | ID 01 model jauh | ID 01 bagian A\* | ID 02 model jauh | ID 02 bagian A\* |
|---|---|---|---|---|
| Gagal jauh (sampel / set uji penuh) | 0% / 0,1% | – | 0% / 0,1% | – |
| **Jarak titik serah ke tujuan**, median | **56,4 cm** | **55,7 cm** | **55,7 cm** | **56,1 cm** |
| Panjang bagian, median (rata) | 73,0 cm (95,1) | 65,7 cm (79,1) | 80,3 cm (91,2) | 83,9 cm (87,0) |
| Panjang model jauh / A\* per rute, median / p90 | 1,00 / 1,54× | | 1,00 / 1,56× | |
| Biaya bagian, median (rata) | 140 (179) | 136 (140) | 142 (176) | 127 (149) |
| Ganti arah di bagian ini, rata-rata (maks) | 0,50 (8) | 0,12 (1) | 0,45 (3) | 0,24 (3) |
| Titik jalur, median | 41 | 37 | 45 | 47 |

- Model jauh berhenti di titik yang **sama** dengan saat Hybrid A\* beralih ke RS, sekitar 56 cm, tepat di bawah batas 60 cm.
- Di separuh rute, jalurnya sama panjang dengan Hybrid A\*. Ekornya lebih panjang: p90 1,5×, dan satu rute ID 01 sampai 1.051 cm (model jauh berputar lama setelah beberapa pendaratan gagal).

### 3.2 Waktu

| | ID 01 model jauh | ID 01 bagian A\* | ID 02 model jauh | ID 02 bagian A\* |
|---|---|---|---|---|
| **Median** | **18,4 ms** | **13,2 ms** | **19,1 ms** | **10,5 ms** |
| Rata-rata | 75,6 ms | 37,1 ms | 34,8 ms | 47,0 ms |
| p90 | 113,6 ms | 110,2 ms | 101,4 ms | 131,9 ms |
| Maksimum | 1.319 ms | 283 ms | 291 ms | 594 ms |
| Dari itu heuristik Dijkstra | sekitar 2 ms | 1,9 ms | sekitar 2 ms | 1,9 ms |
| Langkah pencarian, median (rata) | 15 aksi (69) | 91 ekspansi (303) | 17,5 aksi (31) | 73,5 ekspansi (393) |
| Forward jaringan, median | 15 | – | 17,5 | – |

- Model jauh dan bagian Hybrid A\* **sama kelasnya**: median sekitar 1,5–2× lebih lambat, p90 hampir sama, dan rata-rata ID 02 malah lebih cepat.
- Caranya berbeda. Model jauh hampir tidak mencari: jumlah langkahnya median 15–18, sama dengan jumlah primitif jalurnya, jadi aksi pertama menurut Q hampir selalu benar. Hybrid A\* mengekspansi 74–91 simpul. Satu langkah model jauh lebih mahal (observasi 47 angka sekitar 0,4–0,5 ms, forward sekitar 0,13–0,15 ms), tapi jumlahnya jauh lebih sedikit.
- Rute penuh yang gagal mendarat membuat model jauh terus mencari pose serah lain. Karena itu rata-rata waktu model jauh di **semua** rute penuh, termasuk yang gagal, naik menjadi 249 / 123 ms, dengan p90 1.199 / 116 ms.

---

## 4. Model dekat vs bagian Reeds-Shepp

### 4.1 Dari pose start yang SAMA (rute dekat saja, start ≤ 60 cm)

Perbandingan paling bersih: kedua perencana mulai dari pose yang persis sama, dan Hybrid A\* hampir selalu langsung menembak RS dari start (median 1 ekspansi). 96 rute per robot, semua tiba di kedua perencana.

| | ID 01 model dekat | ID 01 Reeds-Shepp | ID 02 model dekat | ID 02 Reeds-Shepp |
|---|---|---|---|---|
| Tiba | 100% (set uji penuh 99,1%) | 100% | 100% (set uji penuh 99,1%) | 100% |
| **Panjang model dekat / RS per rute, median / p90** | **0,99 / 1,15×** | 1 | **0,99 / 1,18×** | 1 |
| Rute yang jalur model dekatnya **lebih pendek** | 62,5% | – | 59,4% | – |
| Biaya model dekat / RS per rute, median / p90 | 1,01 / 1,33× | 1 | 1,02 / 1,47× | 1 |
| Panjang, median | 81,2 cm | 86,2 cm | 86,9 cm | 86,3 cm |
| Ganti arah, rata-rata (maks) | 1,62 (7) | 1,25 (3) | 2,02 (15) | 1,38 (3) |
| Titik jalur, median | 46 | 39 | 49,5 | 38 |
| Galat akhir, rata / maks | 0,70 cm, 0,76° / 1,0 cm, 1,8° | 0 | 0,73 cm, 0,89° / 1,0 cm, 2,0° | 0 |
| **Waktu median** | **78,9 ms** | **0,19 ms** | **77,8 ms** | **0,20 ms** |
| Waktu rata-rata / p90 / maks | 514 / 1.045 / 6.829 ms | 0,24 / 0,39 / 0,9 ms | 318 / 762 / 3.507 ms | 0,24 / 0,33 / 2,3 ms |
| Percobaan per rute (berhasil per percobaan) | 1,64 (61%) | 1,01 (99%) | 1,31 (76%) | 1,07 (93%) |
| Langkah pencarian, median | 159 | – | 164 | – |

> [!note] Mutu jalur model dekat setara Reeds-Shepp
> Dari pose yang sama, model dekat menemukan jalur yang sama panjang dengan Reeds-Shepp, bahkan lebih pendek di sekitar 60% rute. Ini mungkin karena RS terikat pada segmen pendekatan lurus 32,4 cm di ujung (`approach_len`), sedangkan model dekat cukup masuk toleransi. Yang tertinggal adalah **waktu**: median sekitar 400× lebih lambat.

### 4.2 Di rute penuh (dari titik serah masing-masing)

Model dekat mulai dari pose serah **model jauh**, sedangkan RS dari titik sambung **Hybrid A\***. Posenya mirip (sekitar 56 cm), tapi tidak sama.

| | ID 01 model dekat | ID 01 Reeds-Shepp | ID 02 model dekat | ID 02 Reeds-Shepp |
|---|---|---|---|---|
| Gagal dekat (sampel 96 rute) | 8,3% | 0% | 4,2% | 0% |
| **Percobaan per rute (berhasil per percobaan)** | **2,55 (36%)** | 30,2 tembakan (3,3%) | **2,46 (39%)** | 16,7 tembakan (6,0%) |
| Panjang, median (rata) | 95,8 cm (142) | 90,8 cm (87,6) | 87,1 cm (136) | 85,5 cm (85,0) |
| Biaya, median (rata) | 248 (469) | 219 (209) | 222 (421) | 209 (195) |
| Ganti arah, rata-rata (maks) | 4,30 (105) | 1,16 (3) | 3,55 (79) | 1,07 (4) |
| Titik jalur, median | 55 | 41 | 49 | 38,5 |
| Galat akhir, rata | 0,63 cm, 0,82° | 0 | 0,66 cm, 0,87° | 0 |
| **Waktu median** (tiba) | **129 ms** | **0,24 ms** | **349 ms** | **0,22 ms** |
| Waktu rata-rata / p90 (tiba) | 581 / 1.595 ms | 1,5 / 2,7 ms | 782 / 2.669 ms | 3,0 / 5,5 ms |
| Langkah pencarian, median | 272 | – | 724 | – |

- **Titik serah model jauh lebih sulit didaratkan.** Keberhasilan per percobaan turun dari 61–76% (pose start dataset) menjadi 36–39% (pose serah model jauh). Ini masalah yang ingin diatasi DAgger; perbaikannya belum cukup.
- Mediannya masih dekat dengan RS (panjang 1,02–1,06×), tapi rata-ratanya jauh lebih buruk. Di beberapa rute sulit model dekat bermanuver panjang: sampai 105 ganti arah dan 1,7–1,9 m jalur di dalam radius 60 cm.
- Hybrid A\* juga sering gagal menembak RS (hanya 3–6% tembakan bebas), tapi satu tembakan hanya sekitar 0,3 ms. Kegagalan itu murah.

### 4.3 Percobaan pendaratan model dekat

Semua 192 rute:

| | ID 01 | ID 02 |
|---|---|---|
| Percobaan per rute, rata-rata | 2,09 | 1,89 |
| Berhasil per percobaan | 45,8% | 51,9% |
| Waktu percobaan **berhasil**, median / p90 | 61 / 468 ms | 68 / 467 ms |
| Waktu percobaan **gagal**, median / maks | **564 / 607 ms** | **559 / 588 ms** |
| Waktu untuk percobaan gagal, rata-rata per rute | **643 ms** | **508 ms** |
| Waktu untuk percobaan berhasil, rata-rata per rute | 163 ms | 166 ms |

Pendaratan yang gagal hampir selalu menghabiskan jatah penuh `BATAS_DEKAT_S` = 0,6 s. Pencarian baru berhenti setelah waktunya habis, bukan saat jelas buntu. Itu sekitar **70–80% waktu model dekat**.

---

## 5. Waktu keseluruhan (evaluasi penuh)

### 5.1 Sebaran

| | ID 01 DQfD | ID 01 A\* + RS | ID 02 DQfD | ID 02 A\* + RS |
|---|---|---|---|---|
| Median | 233 ms | 4,5 ms | 191 ms | 3,6 ms |
| Rata-rata | 957 ms | 38 ms | 849 ms | 33 ms |
| p90 | 2.324 ms | 83 ms | 2.204 ms | 58 ms |
| Maksimum | 10.565 ms | 2.027 ms | 11.240 ms | 1.288 ms |
| Median DQfD / A\* per rute | 17× | | 22× | |
| Rute DQfD lebih cepat dari A\* | 2,3% | | 2,5% | |
| Rute DQfD yang gagal, median | 10,1 s | | 10,2 s | |
| **Porsi waktu di bagian dekat** (pengukuran per bagian) | **86,5%** (model dekat) | **10,1%** (RS) | **91,5%** (model dekat) | **7,5%** (RS) |

Median per peta (DQfD vs A\* + RS): Kalibrasi 79 / 114 ms vs 4,8 / 3,2 ms · Rumah 493 / 470 ms vs 4,0 / 3,3 ms · Rumah2 175 / 123 ms vs 4,1 / 2,9 ms · sintetis 226 / 185 ms vs 4,8 / 3,9 ms (ID 01 / ID 02).

### 5.2 Tiba dalam batas waktu

Perkiraan dari evaluasi tanpa batas: % rute yang tiba dengan waktu ≤ batas.

| Batas | ID 01 DQfD | ID 01 A\* + RS | ID 02 DQfD | ID 02 A\* + RS |
|---|---|---|---|---|
| 0,5 s | 66,9% | 99,0% | 68,4% | 98,6% |
| 1 s | 78,0% | 99,6% | 78,3% | 99,6% |
| **2 s (`time_limit` robot)** | **88,5%** | 99,8% | **89,0%** | 100% |
| 5 s | 95,3% | 99,8% | 95,7% | 100% |
| Tanpa batas | 96,9% | 99,8% | 97,8% | 100% |

> [!warning] Akibat di robot
> `DQfDPlanner` memakai `time_limit` = 2 s, sama dengan Hybrid A\*. Dengan `--perencana dqfd-murni`, sekitar 11% tujuan (Rumah sekitar 23%) belum mendapat jalur dalam 2 s dan robot tidak berangkat.

---

## 6. Rincian waktu di dalam setiap bagian (profil)

Rata-rata per rute, 96 rute per robot:

| Bagian | ID 01 | ID 02 | Per panggilan |
|---|---|---|---|
| **Total DQfD** | **858 ms** | **861 ms** | |
| **Model dekat** | **745 ms (87%)** | **778 ms (91%)** | |
| ↳ ekspansi + cek tabrakan 30 aksi | 457 ms (54%) | 478 ms (56%) | 138–154 µs, sekitar 3.200 kali |
| ↳ forward jaringan dekat | 150 ms (18%) | 157 ms (18%) | 97–107 µs, sekitar 1.500 kali |
| ↳ logika pencarian (antrian, urutan Q, Python) | 100 ms (12%) | 102 ms (12%) | |
| ↳ step lain, observasi 9 angka, simpan/pulihkan | 30 ms (4%) | 32 ms (4%) | observasi 2,4–2,7 µs |
| ↳ Dijkstra ulang di tiap percobaan | 7 ms (0,8%) | 8 ms (0,9%) | 2,1–2,5 ms |
| **Model jauh** | **108 ms (13%)** | **79 ms (9%)** | |
| ↳ observasi 47 angka (32 sinar + fitur Dijkstra) | 68 ms (8%) | 49 ms (6%) | 400–485 µs |
| ↳ ekspansi + cek tabrakan 14 primitif | 26 ms (3%) | 18 ms (2%) | 132–161 µs |
| ↳ forward jaringan jauh | 10 ms (1,2%) | 7 ms (0,8%) | 125–153 µs |
| ↳ Dijkstra, saringan serah, lain-lain | 4 ms | 4 ms | |

| **Hybrid A\* + RS** | ID 01 | ID 02 |
|---|---|---|
| Total, rata-rata | 47,8 ms | 23,8 ms |
| ↳ bagian A\*: pencarian (ekspansi, cek tabrakan, heap) | 39,8 ms (83%) | 19,6 ms (82%) |
| ↳ bagian A\*: heuristik Dijkstra | 2,1 ms (4%) | 2,5 ms (11%) |
| ↳ **bagian RS: tembakan Reeds-Shepp** | **5,9 ms (12%)** | **1,6 ms (7%)** |
| Satu tembakan RS, median | 0,30 ms | 0,31 ms |

---

## 7. Kenapa model dekat lambat, dan model jauh tidak

**Model jauh** menggantikan pencarian A\* dengan jaringan yang hampir selalu memilih primitif yang benar di percobaan pertama. Ia membayar observasi yang lebih mahal per langkah, tapi butuh 4–6× lebih sedikit langkah. Hasilnya waktu yang sama kelasnya.

**Model dekat** menggantikan **rumus analitik** dengan pencarian:
1. Reeds-Shepp menghitung kurva pendaratan secara eksak dalam sekitar 0,2–0,3 ms. Model dekat menyusunnya segmen demi segmen (sampai 1/16 langkah) dan memeriksa 30 aksi di tiap keadaan: median 159–724 langkah, ratusan sampai belasan ribu di rute sulit.
2. Separuh percobaannya gagal, dan tiap kegagalan menghabiskan sekitar 0,56 s. Bagi RS, tembakan yang gagal hanya 0,3 ms.
3. Model dekat tidak melihat rintangan (9 masukan). Di ruang sempit, urutan aksi menurut Q sering menuju jalan buntu, sehingga pencarian banyak mundur. Inilah yang membuat Rumah paling lambat dan paling sering gagal.

Yang **bukan** penyebab: memuat model (6–11 ms sekali), Dijkstra (< 1%), dan GPU (forward satu keadaan lebih cepat di CPU).

---

## 8. Kesimpulan

| Aspek | Model jauh vs Hybrid A\* | Model dekat vs Reeds-Shepp |
|---|---|---|
| Keberhasilan | **Setara** (gagal jauh 0,1%) | **RS unggul** (gagal dekat 2–3% di set uji, RS 0%) |
| Titik serah | Setara (56 cm) | – |
| Panjang / biaya jalur | Setara di median (1,00×), ekor model jauh lebih panjang | **Setara dari pose yang sama** (0,99× panjang, 1,01–1,02× biaya); ekor panjang di rute sulit |
| Ganti arah | Sedikit lebih banyak (0,5 vs 0,1–0,2) | Lebih banyak (1,6–2,0 vs 1,3–1,4 dari pose sama; 3,6–4,3 vs 1,1 di rute penuh) |
| Presisi akhir | – | **RS unggul** (eksak vs ≤ 1 cm & 2°) |
| Waktu | **Setara** (median 18–19 vs 11–13 ms) | **RS unggul jauh** (median 78–349 ms vs 0,2 ms) |

**Model jauh berhasil meniru Hybrid A\*** dari sisi mutu dan waktu. **Model dekat berhasil meniru mutu jalur Reeds-Shepp, tapi belum keandalan dan kecepatannya.** Rumus analitik RS sulit dikalahkan oleh pencarian berpemandu jaringan yang tidak melihat rintangan. Seluruh selisih waktu DQfD dan hampir seluruh kegagalannya berasal dari model dekat.

Arah perbaikan model dekat, menurut dampaknya:
1. **Hentikan pendaratan lebih awal bila jelas buntu**, misalnya anggaran langkah, tidak menunggu 0,6 s habis. Ini memangkas bagian terbesar waktu model dekat (70–80%).
2. **Model dekat yang melihat rintangan** (32 sinar + 6 ruang manuver, lihat [[Struktur DQfD Full]] bagian 13). Tujuannya menaikkan keberhasilan per percobaan dari titik serah model jauh (36–39%) dan memperbaiki Rumah.
3. **Forward berkelompok dan cek tabrakan tervektorisasi** di pencarian, untuk memangkas 54% + 18% waktu model dekat.

---

## 9. Sumber angka

| Angka | Sumber |
|---|---|
| Bagian 2, 5 | `model_dqn/evaluasi_id01.json` (2026-10-21 17:35), `evaluasi_id02.json` (18:15) |
| Bagian 3, 4, porsi waktu di 5.1 | Pengukuran per bagian sekali jalan, 192 rute uji per robot (24 per peta × asal), skrip sementara di luar repo |
| Bagian 6 | Profil waktu sekali jalan, 96 rute uji per robot (12 per peta × asal), skrip sementara di luar repo |
| Struktur model, parameter | [[Struktur DQfD Full]] |
| Bagian 10.1 | `model_dqn/evaluasi_id0N.json` (2026-10-23), pembanding `model_dqn/arsip_v6_dasar/`, `arsip_v5_dekat_sinar/` |
| Bagian 10.2, 10.3 | RS-optimize: `model_dqn/rincian_komputasi.json` (2026-10-02, `drl/rincian_komputasi.py` di branch RS-optimize). ver-1-4: profil dengan kategori yang sama, semua rute uji Kalibrasi / Rumah2 / Rumah, skrip sementara di luar repo |
| Bagian 10.4 | `model_dqn/pose_akhir/pose_akhir_id0N.json` (`python -m drl.uji_pose_akhir --id N`), ver-1-3 di `model_dqn/pose_akhir/ver-1-3/` |

---

## 10. ver-1-4 (2026-10-23): vs RS-optimize, rincian waktu, navigasi simulasi

Model yang diuji: model dekat v6 (9 masukan, 88 kata RS, langkah 5000), model jauh dilatih lanjut (ID 01 langkah 10000, ID 02 langkah 25000). Perencana operasional `dqfd-murni` (tanpa cadangan A\*, metode gabungan, batas waktu 2 s). Detail model: [[Struktur DQfD Full]] bagian 13.

### 10.1 Evaluasi penuh (set uji)

| Rute penuh, gabungan | ver-1-3 ID 01 / 02 | v6 dasar ID 01 / 02 | **ver-1-4 ID 01 / 02** | Hybrid A\* + RS |
|---|---|---|---|---|
| Tiba | 97,6 / 96,1% | 98,7 / 99,2% | **99,7 / 99,7%** | 99,5 / 100% |
| Waktu median / p90 (ms) | 101 / 1.732 · 343 / 2.366 | 21 / 189 · 28 / 365 | **22 / 111 · 20 / 129** | 17 / 150 · 15 / 138 |
| Biaya p90 vs A\* | 1,81 / 1,55× | 1,29 / 1,31× | **1,19 / 1,17×** | 1 |
| Ganti arah, rata-rata | 3,85 / 2,88 | 1,80 / 1,96 | **1,66 / 1,67** | 1,43 / 1,43 |
| Percobaan pendaratan per rute | 1,7 / 1,9 | 10,3 / 13,2 | **5,8 / 7,7** | – |

"v6 dasar" = model dekat v6 dengan model jauh ver-1-3 (sebelum dilatih lanjut).

### 10.2 vs branch RS-optimize: waktu keseluruhan

RS-optimize = model jauh (satu model, 14 primitif + aksi SAMBUNG) + Reeds-Shepp analitik `_analytic` yang dicoba di banyak simpul. ver-1-4 = model jauh + model dekat yang memilih kata RS.

> [!warning] Profil kemudi berbeda
> RS-optimize diukur 2026-10-02 dengan profil kemudi lama: ID 01 35°, ID 02 22° (radius belok rencana 45,6 / 76,0 cm). ver-1-4 memakai 55° untuk kedua robot (25,7 cm), setelah perbaikan mekanisme. Rutenya juga dari dataset yang berbeda. Hybrid A\* di kedua pengukuran pun berbeda jauh, jadi bandingkan terutama **rasio terhadap Hybrid A\* di pengukuran yang sama** dan **porsi waktu per bagian**.

Gabungan 3 peta (Kalibrasi, Rumah2, Rumah), semua rute uji:

| | RS-optimize ID 01 | **ver-1-4 ID 01** | RS-optimize ID 02 | **ver-1-4 ID 02** |
|---|---|---|---|---|
| Rute | 317 | 266 | 318 | 241 |
| Hybrid A\*: tiba / median / p90 / rata (ms) | 97,8% / 8,3 / 692 / 233 | 100% / 15,8 / 137 / 59 | 98,7% / 22,2 / 741 / 243 | 99,2% / 13,5 / 132 / 82 |
| DQfD: tiba / median / p90 / rata (ms) | 99,1% / 18,8 / 267 / 151 | **100% / 23,4 / 109 / 91** | 97,8% / 28,5 / 276 / 145 | **100% / 19,8 / 96 / 115** |
| DQfD ÷ A\*, median | 2,27× | **1,48×** | 1,28× | 1,47× |
| DQfD ÷ A\*, p90 | 0,39× | 0,79× | 0,37× | 0,73× |
| DQfD ÷ A\*, rata-rata | 0,65× | 1,53× | 0,60× | 1,40× |

- **Secara absolut, DQfD ver-1-4 lebih cepat dari DQfD RS-optimize**: p90 109 vs 267 ms dan 96 vs 276 ms, rata-rata 91 vs 151 ms dan 115 vs 145 ms. Tiba 100% di kedua robot.
- **RS-optimize tampak "lebih cepat dari A\*" pada rata-rata dan p90** karena Hybrid A\*-nya sendiri sangat lambat dengan radius belok lama yang besar (p90 ~700 ms, terutama di Rumah). Dengan kemudi 55°, Hybrid A\* menjadi 5× lebih cepat di p90, jadi pembandingnya jauh lebih berat.
- ver-1-4 tetap **lebih cepat dari Hybrid A\* di p90** (0,73–0,79×), tetapi lebih lambat di median (1,5×) dan rata-rata (1,4–1,5×). Ekor lambatnya ada di peta Rumah: rata-rata DQfD 179 / 224 ms vs A\* 96 / 104 ms.

### 10.3 Rincian waktu per bagian DQfD

Rata-rata ms per rute, 3 peta. ver-1-4 dipisah: bagian model jauh dan bagian model dekat (pendaratan).

| Bagian | RS-optimize ID 01 | **ver-1-4 ID 01** | RS-optimize ID 02 | **ver-1-4 ID 02** |
|---|---|---|---|---|
| Sinar (32 sinar per simpul) | 66,0 (43%) | **47,1 (50%)** | 51,4 (35%) | **62,4 (54%)** |
| Primitif (ekspansi + cek badan) | 24,7 (16%) | 21,5 (23%) | 21,4 (15%) | 22,9 (20%) |
| Jaringan | 15,6 (10%) | 8,1 (jauh 6,9 + dekat 1,2) | 13,5 (9%) | 10,7 (jauh 9,7 + dekat 1,0) |
| Pencarian lain (sisa) | 14,6 (10%) | 7,2 (8%) | 13,2 (9%) | 10,3 (9%) |
| Heuristik Dijkstra + observasi lain | 4,4 | 4,5 | 4,1 | 4,9 |
| **Pendaratan** | **Reeds-Shepp 26,9 (18%)** | **model dekat 7,1 (8%)** | **Reeds-Shepp 42,8 (29%)** | **model dekat 5,9 (5%)** |
| — jumlah percobaan per rute | 66 panggilan `_analytic` | 10,2 | 81 panggilan `_analytic` | 8,8 |
| — isinya | rumus RS + cek tabrakan, 6 kandidat termurah | cek tabrakan 3,2 · sisa 1,4 · jaringan 1,2 · rumus RS 0,8 · arah pendekatan 0,4 · observasi 0,3 | | cek tabrakan 2,5 · sisa 1,2 · jaringan 1,0 · rumus RS 0,7 · arah pendekatan 0,3 · observasi 0,2 |
| **Total** | **152,4** | **94,2** | **146,5** | **116,1** |

- **Pendaratan 4–7× lebih murah dari RS-optimize.** RS-optimize mencoba Reeds-Shepp di 66–81 simpul per rute (aksi SAMBUNG, tanpa batas jarak). ver-1-4 hanya mencoba di dalam 60 cm dan setelah saringan Dijkstra: 9–10 kali per rute, masing-masing ±0,65 ms.
- **Jaringan setengahnya**: model jauh ver-1-4 membuka lebih sedikit simpul.
- **Sinar sekarang memakan separuh waktu DQfD** (50–54%). Dengan model dekat yang sudah murah, inilah target optimasi berikutnya: 32 sinar sphere tracing dihitung di setiap simpul yang dibuka model jauh.

### 10.4 Navigasi simulasi: pose akhir robot

`python -m drl.uji_pose_akhir --id N`: `navigasi.py` sungguhan tanpa jendela. Alurnya: perencana → pelacak pure pursuit → robot simulasi dengan jeda kemudi, 20 Hz, tanpa derau. Galat pose akhir = galat ujung rencana + galat pelacakan.

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

- **12/12 jalan tiba**, tidak ada yang masuk zona. Di ver-1-3, skenario geser DQfD sempat gagal (batas waktu habis).
- **Galat ujung rencana 0 untuk kedua perencana**: bagian dekat DQfD adalah kurva RS eksak.
- **Skenario geser: jalur DQfD identik dengan Hybrid A\* + RS** (82 / 104 cm, galat akhir sama sampai 0,1 cm). Model dekat memilih kata RS yang sama dengan demonstrator.
- **Galat pose akhir robot hampir seluruhnya dari pelacakan.** Contohnya geser 6,9 cm / 23° dan lapang ID 02 ~18°, dengan besar yang sama untuk kedua perencana. Perbaikannya ada di pelacak (pure pursuit, jeda kemudi), bukan di perencana.

![[Pose akhir ver-1-4 ID01 balik.png]]
![[Pose akhir ver-1-4 ID01 geser.png]]
![[Pose akhir ver-1-4 ID02 lapang.png]]

Gambar lain: `Gambar/Pose akhir ver-1-4 ID0N <skenario>.png`.
