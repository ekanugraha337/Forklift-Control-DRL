# Waktu komputasi per tahapan: Hybrid A\* + RS vs DQfD (peta Gudang)

Tanggal: 2026-11-04 · Branch `ver-1-4` · Skenario: `uji_nyata/Gudang.json`, sama dengan [[09 Uji Robot Nyata Gudang ver-1-4 Gundang1|Uji Robot Nyata Gudang ver-1-4 Gundang1]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]] · Rincian per bagian sebelumnya: [[08 Perbandingan Hybrid A-star RS vs DQfD|Perbandingan Hybrid A-star RS vs DQfD]] bagian 5

Catatan ini memecah **satu kali perencanaan**, dari pose awal dan pose tujuan masuk sampai jalur jadi, menjadi tahapan-tahapan pekerjaan. Waktu tiap tahapan diukur untuk dua perencana:

- `astar`: Hybrid A\* + Reeds-Shepp;
- `dqfd-murni`: model jauh + model dekat, tanpa cadangan A\*.

DQfD diukur dengan `drl/lingkungan.py` yang menghitung observasi **sekali per keadaan** (cache `observasi()`, 2026-11-04).

> [!summary] Ringkasan
> - **Satu kasus Gudang di laptop uji (Windows):** Hybrid A\* + RS rata-rata **23,7 ms**, DQfD **30,5 ms** (median 22,7 vs 28,8 ms). DQfD sekitar **1,3× lebih lambat**. Di MacBook: median 15,4 vs 18,0 ms (1,17×).
> - **Hybrid A\***: sekitar 81% waktunya di **ekspansi primitif**, dan cek tabrakan footprint saja 52%. Heuristik Dijkstra 10%. Tembakan RS hanya 1,7%.
> - **DQfD**: **46% waktunya di observasi 32 sinar** model jauh. Selebihnya:
>   - cek tabrakan calon primitif 11,5%;
>   - forward jaringan jauh 10,1%;
>   - model dekat secara keseluruhan sekitar 2% (0,72 ms, satu percobaan pendaratan).
> - **Pencarian DQfD jauh lebih ringkas**: 10–44 langkah vs 78–143 ekspansi, dan cek tabrakan 2–10× lebih sedikit. Tetapi tiap langkahnya lebih mahal. DQfD menang di S2-ID01 (16,1 vs 30,5 ms), setara di S2-ID02 (18,2 vs 18,2 ms), dan kalah di S1 (1,7–2,1× lebih lambat).
> - **Waktu proses hanya 0,1–0,2% dari makespan** di uji robot nyata (A\* 21,7 s, DQfD 19,0 s), jadi tidak terlihat pada makespan (bagian 6).
>   - Hybrid A\* dan model jauh memakan >92% waktu proses, tetapi hanya 43–56% waktu tempuh.
>   - RS dan model dekat ≤2% waktu proses, tetapi 44–57% waktu tempuh, karena manuver ganti arah dan pelurusan akhir ada di bagian dekat.
>   - Bagian model dekat ditempuh lebih singkat daripada bagian RS (6,6 vs 9,8 s).

---

## 1. Data dan cara mengukur

| | |
|---|---|
| Peta | Gudang, grid 300 × 270 sel |
| Kasus | 4: skenario S1 dan S2 × robot ID 01 dan ID 02 (`uji_nyata/Gudang.json`) |
| Rintangan tambahan | badan robot lain yang **diam di pose awalnya** (`A.jejak_badan`), seperti `navigasi.py` saat rencana pertama |
| Pipa yang diukur | validasi tujuan (`margin_untuk_pose`, seperti `Unit.terima_tujuan`) + `Perencana.rencanakan()` (margin 8 cm, cadangan 6 cm) |
| Ulangan | 30 per kasus, setelah satu pemanasan |
| Total tanpa instrumentasi | `astar` dan `dqfd-murni` **diselang-seling** di setiap ulangan (ulangan genap A\* dulu, ganjil DQfD dulu) |
| Rincian per tahapan | pencatat waktu `perf_counter` **eksklusif**: waktu fungsi anak dikurangkan dari induknya. `pose_margin` dicatat menurut fungsi pemanggilnya |
| Run Windows | rincian A\*: 3 run. DQfD dan total diselang-seling: 3 run |
| Run Mac | 1 run untuk masing-masing |
| Mesin uji | laptop Windows: AMD Ryzen 7 260, Python 3.14.3, PyTorch 2.11, OpenCV 5.0, terhubung charger, power plan *Performance* |
| Mesin pembanding | MacBook (Apple Silicon): Python 3.13, PyTorch 2.15, OpenCV 5.0 |
| PyTorch | 1 thread (`muat_model`), CPU |

Hasil yang sama di keempat kasus, untuk kedua perencana:
- semua kasus **berhasil dengan margin 8 cm**, tanpa turun ke margin cadangan;
- DQfD tidak pernah memakai A\* cadangan, dan model dekat selalu **mendarat pada percobaan pertama**;
- jalur dari versi berinstrumen **identik** dengan versi tanpa instrumen;
- file perencana, model `.pt`, peta, dan skenario di Windows identik dengan versi di Mac (hash MD5).

Saat pengukuran, repo di Windows belum memuat perubahan `drl/lingkungan.py`. Karena itu, `lingkungan.py` terbaru dimuat dari folder sementara, menggantikan modul `drl.lingkungan` hanya selama pengukuran. Repo di Windows tidak diubah.

## 2. Hybrid A\* + Reeds-Shepp per tahapan

ms per kasus, rata-rata 4 kasus.

| No | Tahapan | Isi pekerjaan | Windows (ms) | Porsi | Mac (ms) | Porsi |
|---|---|---|---:|---:|---:|---:|
| 0 | *Persiapan (sekali saat program mulai)* | muat peta Gudang dan distance transform; bangun `VehicleModel` dan perencana | *13,5 + 0,07* | — | *9,5 + 0,05* | — |
| 1 | Masukan | pose awal (x, y, ψ), pose tujuan (x, y, ψ) dalam cm/derajat, poligon badan robot lain | 0,042 | 0,2% | 0,019 | 0,1% |
| 2 | Validasi pose tujuan | `margin_untuk_pose`, sekali tanpa robot lain dan sekali dengan robot lain | 0,593 | 2,3% | 0,348 | 2,1% |
| 3 | Peta rintangan sementara | `fillPoly` badan robot lain, distance transform ulang | 0,490 | 1,9% | 0,309 | 1,9% |
| 4 | Konversi pose cm → sel | `pose_ke_sel` awal dan tujuan, pilih kendaraan sesuai margin | 0,008 | <0,1% | 0,003 | <0,1% |
| 5 | Pemeriksaan awal | tujuan bebas tabrakan? robot sudah di tujuan? | 0,010 | <0,1% | 0,004 | <0,1% |
| 6 | Heuristik holonomik | Dijkstra 2D 8-tetangga dari tujuan, sadar rintangan | 2,542 | 9,8% | 2,021 | 12,4% |
| 7 | Arah pendekatan lurus | cek footprint sepanjang 32,4 cm maju/mundur ke tujuan | 0,091 | 0,3% | 0,052 | 0,3% |
| 8 | Inisialisasi pencarian | simpul awal, open list (heap), closed set | 0,010 | <0,1% | 0,005 | <0,1% |
| 9 | Ambil simpul | pop f terkecil, cek closed set, cek tiba (±107 ekspansi, 42 duplikat) | 0,317 | 1,2% | 0,176 | 1,1% |
| 10a | Ekspansi primitif: cek tabrakan | `pose_margin` footprint, ±5.724 cek per kasus | 13,554 | **52,1%** | 8,784 | **53,7%** |
| 10b | Ekspansi primitif: kinematik dan biaya | integrasi model sepeda, biaya, push heap (±1.484 primitif, 460 simpul baru) | 7,476 | **28,8%** | 4,174 | **25,5%** |
| 11 | RS: bangkitkan kandidat | `reeds_shepp_paths` + segmen pendekatan, `_rs_cost`, urutkan (±7 kandidat) | 0,142 | 0,5% | 0,082 | 0,5% |
| 12 | RS: verifikasi | sampling dan cek tabrakan kandidat (±2,2 kandidat dicek) | 0,311 | 1,2% | 0,166 | 1,0% |
| 13 | Rekonstruksi jalur | telusur balik parent, gabung ekor RS | 0,016 | 0,1% | 0,008 | <0,1% |
| 14 | Pasca-proses | panjang jalur, konversi ke cm, statistik | 0,090 | 0,3% | 0,051 | 0,3% |
| — | Overhead loop lain | | 0,302 | 1,2% | 0,146 | 0,9% |
| | **Total (dengan pencatat waktu)** | | **25,99** | 100% | **16,35** | 100% |
| | **Total tanpa pencatat waktu** (run rincian A\*) | | **22,6** | | **15,1** | |

- Tahap 6–13 berada di dalam `HybridAStarPlanner.plan()`. Tahap 11–12 adalah ekspansi analitik `_analytic` → `_best_free_rs`, yang hanya dicoba saat titik acuan robot < 60 cm (`analytic_dist`) dari tujuan. Di keempat kasus, tembakan pertama langsung berhasil.
- Pencatat waktu di dalam loop cek tabrakan menambah ±3,4 ms di Windows dan ±1,3 ms di Mac. Jadi porsi baris 10a sedikit terlalu besar.
- Untuk perbandingan dengan DQfD (bagian 4–5), dipakai total A\* dari run yang diselang-seling dengan DQfD: Windows 23,7 / 22,7 ms, Mac 15,5 / 15,4 ms (rata-rata / median).

## 3. DQfD (`dqfd-murni`) per tahapan

ms per kasus, rata-rata 4 kasus. Jalurnya: `Perencana.rencanakan()` → `DQfDPlanner.plan()` → `rencanakan_dua()`. Model jauh memakai pencarian **terbaik-dulu** (f = g + c + 2·h, h dari Q, anggaran 1200), dan beralih ke pencarian mundur bila gagal. Mundur tidak pernah terpakai di kasus ini.

| No | Tahapan | Isi pekerjaan | Windows (ms) | Porsi | Mac (ms) | Porsi |
|---|---|---|---:|---:|---:|---:|
| 0 | *Persiapan (sekali saat program mulai)* | import torch; muat model jauh dan dekat; bangun perencana dan lingkungan | *2.622 + 19,0 + 0,34* | — | *830 + 13,4 + 0,19* | — |
| 1 | Masukan | pose awal, pose tujuan, poligon robot lain | 0,019 | 0,1% | 0,015 | 0,1% |
| 2 | Validasi pose tujuan | `margin_untuk_pose`, tanpa dan dengan robot lain | 0,553 | 1,7% | 0,357 | 1,8% |
| 3 | Peta rintangan sementara | `fillPoly` dan distance transform | 0,481 | 1,5% | 0,314 | 1,6% |
| 4 | Konversi dan orkestrasi | `pose_ke_sel`, `DQfDPlanner.plan`, `rencanakan_dua` | 0,053 | 0,2% | 0,031 | 0,2% |
| **J** | **Model jauh** (meniru Hybrid A\*) | | | | | |
| J1 | Reset lingkungan jauh | cek tujuan, pose awal, cek apakah sudah di radius serah | 0,167 | 0,5% | 0,109 | 0,6% |
| J2 | Heuristik Dijkstra | sama dengan A\* tahap 6, dipakai di observasi (h dan arah turunnya) | 2,627 | 8,0% | 2,071 | 10,7% |
| J3 | Arah pendekatan lurus | sama dengan A\* tahap 7 | 0,098 | 0,3% | 0,062 | 0,3% |
| J4 | Observasi: 32 sinar | sphere tracing di distance transform (`_sinar`), sekali per keadaan: ±25 kali per kasus (langkah + 1) | **15,171** | **46,2%** | **9,641** | **49,8%** |
| J5 | Observasi: sisa 47 masukan | tujuan relatif, h Dijkstra, arah turun heuristik, margin badan | 0,631 | 1,9% | 0,317 | 1,6% |
| J6 | Forward jaringan jauh | 47 → 256 → 256 → (V, A) → 14 Q, ±24 kali | 3,301 | 10,1% | 1,129 | 5,8% |
| J7 | Calon primitif: cek tabrakan | 14 primitif per simpul untuk mask, ±1.555 `pose_margin` | 3,761 | 11,5% | 2,423 | 12,5% |
| J8 | Calon primitif: kinematik dan biaya | integrasi model sepeda (salinan aritmetika A\*), biaya, mask sel dikunjungi | 2,940 | 9,0% | 1,576 | 8,1% |
| J9 | Pencarian terbaik-dulu | f per calon, heap, `step`, simpan/pulihkan keadaan, saringan serah (60 cm + Dijkstra bebas) | 2,256 | 6,9% | 0,910 | 4,7% |
| **D** | **Model dekat** (meniru RS, 1 percobaan pendaratan) | | | | | |
| D1 | Reset lingkungan dekat | cek tujuan, arah pendekatan dihitung ulang | 0,130 | 0,4% | 0,082 | 0,4% |
| D2 | Kandidat kata RS | 44 kata × slot sambung (`kata_rs_semua`), `_rs_cost` | 0,163 | 0,5% | 0,096 | 0,5% |
| D3 | Observasi dekat | 9 masukan rumus RS | 0,015 | <0,1% | 0,007 | <0,1% |
| D4 | Forward jaringan dekat | 9 → 256 → 256 → (V, A) → 88 Q, 1 kali | 0,180 | 0,5% | 0,055 | 0,3% |
| D5 | Urutkan Q dan pilih kata | argsort Q pada kata sah, `step` kata pertama | 0,086 | 0,3% | 0,042 | 0,2% |
| D6 | Verifikasi kurva | sampling kurva RS dan cek tabrakan (±33 pose) | 0,143 | 0,4% | 0,085 | 0,4% |
| 14 | Pasca-proses | gabung jalur jauh + dekat, panjang jalur | 0,032 | 0,1% | 0,021 | 0,1% |
| | **Total (dengan pencatat waktu)** | | **32,83** | 100% | **19,35** | 100% |
| | **Total tanpa pencatat waktu, rata-rata** | | **30,5** | | **18,1** | |
| | **Total tanpa pencatat waktu, median** | | **28,8** | | **18,0** | |

- Pencatat waktu menambah ±7–8% (Windows 32,8 vs 30,5 ms, Mac 19,4 vs 18,1 ms).
- Model dekat (D1–D6) totalnya 0,72 ms. Tembakan RS di A\* 0,45 ms. Selisihnya karena model dekat menghitung ulang arah pendekatan dan **semua** 44 kata RS. `_best_free_rs` hanya membangkitkan sekitar 7 kandidat dari `reeds_shepp_paths`.

## 4. Perbandingan per fase

ms per kasus, dengan pencatat waktu (dari bagian 2 dan 3). Baris total tanpa pencatat waktu diambil dari pengukuran yang diselang-seling.

| Fase | Tahap A\* | Tahap DQfD | A\* Windows | DQfD Windows | A\* Mac | DQfD Mac |
|---|---|---|---:|---:|---:|---:|
| Pra-proses (masukan, validasi, peta rintangan, konversi) | 1–5 | 1–4 | 1,14 | 1,11 | 0,68 | 0,72 |
| Heuristik Dijkstra 2D | 6 | J2 | 2,54 | 2,63 | 2,02 | 2,07 |
| Arah pendekatan dan inisialisasi | 7–8 | J1, J3 | 0,10 | 0,27 | 0,06 | 0,17 |
| **Bagian jauh** | 9–10 (ekspansi primitif) | J4–J9 (model jauh) | **21,35** | **28,06** | **13,13** | **16,00** |
| **Bagian dekat** | 11–12 (tembakan RS) | D1–D6 (model dekat) | **0,45** | **0,72** | **0,25** | **0,37** |
| Pasca-proses dan overhead | 13–14, lain | 14 | 0,41 | 0,03 | 0,21 | 0,02 |
| Total dengan pencatat waktu | | | 25,99 | 32,83 | 16,35 | 19,35 |
| **Total tanpa pencatat waktu, rata-rata** | | | **23,7** | **30,5** | **15,5** | **18,1** |
| **Total tanpa pencatat waktu, median** | | | **22,7** | **28,8** | **15,4** | **18,0** |
| **DQfD ÷ A\*** (median) | | | | **1,27×** | | **1,17×** |

- **Fase yang sama persis** (pra-proses, Dijkstra, arah pendekatan) memakan waktu yang sama di kedua perencana, sekitar 3,8–4,0 ms di Windows. Ini adalah ongkos tetap yang tidak bisa dihindari model jauh, karena Dijkstra adalah bagian dari observasinya.
- **Hampir seluruh selisih ada di bagian jauh**: 21,4 ms (A\*) vs 28,1 ms (DQfD) di Windows. Bagian dekat setara (<1 ms di keduanya).

## 5. Per kasus

Windows, tanpa pencatat waktu, rata-rata 3 run × 30 ulangan. A\* dan DQfD diselang-seling di setiap ulangan.

| Kasus | A\* rata / median (ms) | DQfD rata / median (ms) | Ekspansi A\* / langkah model jauh | Sinar dihitung | Cek tabrakan A\* / DQfD jauh | Titik RS / titik model dekat | Panjang jalur A\* / DQfD (cm) |
|---|---:|---:|---:|---:|---:|---:|---:|
| S1-ID01 | 23,2 / 21,5 | 49,6 / 46,0 | 115 / 44 | 45 | 6.228 / 3.010 | 41 / 31 | 261 / 257 |
| S2-ID01 | 30,5 / 30,0 | **16,1 / 15,8** | 143 / 10 | 11 | 7.679 / 655 | 37 / 35 | 157 / 152 |
| S1-ID02 | 22,8 / 21,2 | 38,2 / 35,4 | 92 / 29 | 30 | 4.940 / 1.832 | 48 / 30 | 278 / 262 |
| S2-ID02 | 18,2 / 18,0 | **18,2 / 18,0** | 78 / 12 | 13 | 4.048 / 721 | 56 / 35 | 206 / 165 |
| **Rata-rata** | **23,7 / 22,7** | **30,5 / 28,8** | 107 / 24 | 25 | 5.724 / 1.555 | 46 / 33 | 226 / 209 |

Mac (median, ms): A\* 16,2 / 19,3 / 13,5 / 12,5 · DQfD 31,5 / 9,3 / 19,5 / 11,7.

- **Waktu DQfD mengikuti jumlah langkah model jauh**: S1 (29–44 langkah) 38–50 ms, S2 (10–12 langkah) 16–18 ms. Waktu A\* lebih rata (18–31 ms), karena ekspansinya 78–143 di semua kasus.
- **Biaya per langkah DQfD sekitar 1,0 ms.** Dihitung secara marginal dari selisih S1-ID01 dan S2-ID01: 33,5 ms untuk 34 langkah. Hybrid A\* rata-rata ±0,22 ms per ekspansi, termasuk ongkos tetap.
- **Pemenangnya ditentukan rasio ekspansi A\* / langkah DQfD**:
  - S1: rasio 2,6–3,2, A\* menang;
  - S2-ID02: rasio 6,5, setara;
  - S2-ID01: rasio 14, DQfD menang.

  Jadi di Gudang titik impasnya sekitar rasio 6–7. Ini hanya dari 4 kasus. Lihat juga [[11 Perbandingan Uji Robot Nyata Gudang1 vs Gudang2|Perbandingan Uji Robot Nyata Gudang1 vs Gudang2]] bagian 3.1.
- Jalur DQfD lebih pendek di keempat kasus. Tetapi yang dioptimalkan kedua perencana adalah **biaya** (mundur, ganti arah, kemudi, jarak ke zona), bukan panjang. Jadi ini bukan ukuran mutu.

## 6. Waktu proses dan makespan per komponen

Tabel ini menyandingkan dua hal untuk keempat komponen:
- **waktu proses perencanaan** (offline, bagian 2–3, Windows, observasi sekali hitung);
- **waktu tempuh robot** di bagian jalur milik komponen itu, dari uji robot nyata Gudang ([[09 Uji Robot Nyata Gudang ver-1-4 Gundang1|Uji Robot Nyata Gudang ver-1-4 Gundang1]], `uji_nyata/20261024_073221_Gudang`).

| Perencana | Komponen | Waktu proses per rencana (ms) | Porsi waktu proses | Waktu tempuh robot di bagian ini (s) | Porsi waktu tempuh | Panjang jalur bagian ini (cm) | Kecepatan saat bergerak (cm/s) |
|---|---|---:|---:|---:|---:|---:|---:|
| Hybrid A\* + RS | **Hybrid A\*** (heuristik + ekspansi primitif) | 24,02 | 92,4% | 7,5 ± 3,5 | 43% | 116 ± 58 | 16,2 |
| | **RS** (tembakan analitik) | 0,45 | 1,7% | 9,8 ± 5,6 | 57% | 96 ± 23 | 10,7 |
| | pra/pasca-proses umum | 1,53 | 5,9% | — | — | — | — |
| | **Jumlah per robot** | **25,99** (tanpa pencatat waktu 23,7) | 100% | **17,3 ± 7,9** | 100% | 212 | |
| | **Makespan per percobaan** (n = 6) | | | **21,7 ± 11,4** | | | |
| DQfD murni | **Model jauh** | 30,95 | 94,3% | 8,3 ± 4,1 | 56% | 133 ± 57 | 17,5 |
| | **Model dekat** | 0,72 | 2,2% | 6,6 ± 3,3 | 44% | 80 ± 17 | 12,4 |
| | pra/pasca-proses umum | 1,14 | 3,5% | — | — | — | — |
| | **Jumlah per robot** | **32,83** (tanpa pencatat waktu 30,5) | 100% | **14,9 ± 6,3** | 100% | 213 | |
| | **Makespan per percobaan** (n = 5) | | | **19,0 ± 10,3** | | | |

Per skenario, waktu tempuh bagian jauh / bagian dekat (s):

| Skenario | Hybrid A\* / RS | Model jauh / model dekat |
|---|---|---|
| S1 (rute bersilangan, ±260–280 cm) | 11,0 ± 1,4 / 12,6 ± 6,3 (n = 5) | 11,4 ± 2,2 / 7,7 ± 4,3 (n = 5) |
| S2 (balik arah masuk teluk, ±150–160 cm) | 4,6 ± 0,6 / 7,5 ± 3,9 (n = 6) | 4,3 ± 0,4 / 5,2 ± 0,2 (n = 4) |

**Cara memotong waktu tempuh:**
1. Setiap robot dengan **satu** rencana direncanakan ulang offline dengan kode sekarang (sinar dihitung 1×), dari pose awal jalur yang tercatat. Ada 11 robot-percobaan A\* dan 9 DQfD. Dua robot-percobaan yang merencanakan ulang saat jalan (2 dan 5 rencana) tidak dipakai.
2. **Semua 20 rencana ulang identik** dengan jalur yang tercatat saat uji. Dari rencana ulang itu diambil titik serah: titik terakhir bagian primitif sebelum RS / model dekat.
3. Lintasan 20 Hz (`lintasan.csv`, fase UJI) ditelusuri per segmen arah seperti pelacak. Waktu serah adalah saat kemajuan robot melewati titik serah. Saat itu robot berjarak 0,4–2,9 cm dari titik serah.
4. Bagian jauh = awal fase UJI sampai serah (termasuk waktu rencana dan menunggu). Bagian dekat = serah sampai tiba (`t_tempuh_s`).
5. Kecepatan dihitung tanpa waktu menunggu robot lain:
   - A\*: menunggu 0,36 s di bagian jauh dan 0,84 s di bagian dekat;
   - DQfD: menunggu 0,64 s di bagian jauh dan 0,13 s di bagian dekat.

**Cara membaca tabel ini:**
- **Waktu proses tidak terlihat pada makespan.** Jumlah waktu proses hanya 0,1–0,2% dari makespan (A\* 23,7 ms dari 21,7 s, DQfD 30,5 ms dari 19,0 s). Makespan ditentukan oleh **gerak robot**: panjang jalur, ganti arah, kecepatan pelacak, dan menunggu robot lain.
- **Komponen yang mahal diproses justru yang singkat dijalani, dan sebaliknya.**
  - Hybrid A\* dan model jauh memakan >92% waktu proses, tetapi bagian jalurnya ditempuh relatif cepat (16–18 cm/s), karena didominasi gerak maju lurus atau berbelok halus.
  - RS dan model dekat hanya ≤2% waktu proses, tetapi 44–57% waktu tempuh. Di bagian inilah manuver ganti arah dan pelurusan akhir terjadi: jeda ganti arah 0,6 s dan perlambatan di belokan terakhir. Akibatnya kecepatannya turun ke 11–12 cm/s.
- **Bagian dekat DQfD lebih singkat daripada bagian RS** (6,6 vs 9,8 s):
  - bagian jalurnya lebih pendek (80 vs 96 cm) dan ditempuh lebih cepat (12,4 vs 10,7 cm/s);
  - sebarannya juga jauh lebih kecil (S2: 5,2 ± 0,2 s vs 7,5 ± 3,9 s);
  - waktu RS terpanjang terjadi di S1 (18,3–18,4 s) dan S2 ulangan 3 ID 01 (15,0 s, bagian RS 127 cm);
  - model jauh menyerahkan sedikit lebih lambat (bagian jauh 133 vs 116 cm), sehingga sisa jalur untuk model dekat lebih pendek.
- **Bagian jauh setara** (7,5 vs 8,3 s, kecepatan 16,2 vs 17,5 cm/s).

> [!warning] Batasan tabel ini
> - **Kode saat uji vs kode sekarang.** Uji robot nyata dilakukan sebelum observasi sekali hitung. Jalurnya identik (20/20 rencana ulang sama persis), jadi waktu tempuh dan makespan tidak berubah. Yang berubah hanya waktu rencana, dan itu tidak dipakai di tabel ini. Waktu proses di tabel ini berasal dari profil offline (bagian 2–3).
> - **Sampel kecil**: 11 / 9 robot-percobaan dan 6 / 5 percobaan. DQfD S2 ulangan 3 belum dijalankan. Selisih rata-rata antar-perencana masih tumpang-tindih dengan sebarannya.
> - **Waktu tempuh adalah hasil gabungan perencana dan pelacak** (pure pursuit, jeda ganti arah, perlambatan akhir). Waktu tempuh bukan ukuran kecepatan komputasi komponen.

## 7. Temuan

1. **Bottleneck kedua perencana berbeda.**
   - Hybrid A\*: cek tabrakan footprint (`pose_margin`, ±2,4 µs per cek di Windows) yang dikalikan ribuan primitif.
   - DQfD: observasi 32 sinar (±0,6 ms per panggilan di Windows), yaitu loop numpy 256 iterasi `_sinar`.
   - Jaringan **bukan** bottleneck: forward jauh ±0,14 ms dan forward dekat ±0,18 ms per panggilan.
2. **Sinar tetap porsi terbesar DQfD** (46%), meskipun sudah dihitung sekali per keadaan (langkah + 1 panggilan). Langkah optimasi berikutnya adalah mempercepat `_sinar` itu sendiri (mis. numba, atau sinar lebih sedikit), sesuai [[08 Perbandingan Hybrid A-star RS vs DQfD|Perbandingan Hybrid A-star RS vs DQfD]] bagian 7.
3. **Model dekat murah, tetapi menghitung ulang yang sudah ada.** `reset` dekat memanggil `_feasible_approach_dirs` lagi, padahal tujuan dan peta sama dengan model jauh (±0,1 ms). Model dekat juga menyusun semua 44 kata RS. Dampaknya kecil (<1 ms).
4. **Ongkos sekali DQfD.** Import PyTorch 0,8–2,6 s dan muat model ±13–19 ms per robot. Ongkos ini dibayar sekali saat `navigasi.py` mulai, bukan per rencana.

> [!warning] Batasan
> - **Offline vs saat uji.** Waktu rencana yang tercatat saat uji robot nyata lebih besar daripada waktu offline. Contohnya, rencana pertama A\* di `uji_nyata/20261024_073221_Gudang` rata-rata 41,4 ms. Saat uji, perencana berbagi CPU dengan thread kamera, kendali 20 Hz, dan tampilan (lihat [[09 Uji Robot Nyata Gudang ver-1-4 Gundang1|Uji Robot Nyata Gudang ver-1-4 Gundang1]] bagian 3). Uji robot nyata itu dilakukan **sebelum** observasi sekali hitung, jadi waktu DQfD saat uji belum diukur dengan kode ini. Untuk laporan, gunakan **porsi (%)** dari catatan ini, atau angka yang tercatat saat uji.
> - **Fluktuasi Windows.** Di Windows, tahapan yang bebannya sama di semua kasus (misalnya Dijkstra) berubah 1,9–3,3 ms antar-kasus dan antar-run. Dugaannya, proses dipindah antara core Zen 5 dan Zen 5c, tetapi ini belum dibuktikan. Karena itu, bandingkan perencana dari pengukuran yang **diselang-seling** (bagian 4–5, baris total).
> - **Hanya 4 kasus satu peta.** Ini cukup untuk melihat struktur waktu, tetapi bukan untuk statistik umum. Di peta Gudang2, A\* membuka jauh lebih banyak ekspansi, sehingga gambarannya berbalik. Lihat [[11 Perbandingan Uji Robot Nyata Gudang1 vs Gudang2|Perbandingan Uji Robot Nyata Gudang1 vs Gudang2]].

## 8. Sumber angka

| Angka | Sumber |
|---|---|
| Bagian 2 (rincian Hybrid A\*) | skrip sementara di luar repo `profil_tahapan.py`: salinan `HybridAStarPlanner.plan()` dengan logika identik, ditambah pencatat waktu per blok. Windows 3 run × 30 ulangan, Mac 1 run × 30 ulangan (2026-11-04) |
| Bagian 3–5 (rincian DQfD dan total diselang-seling) | skrip sementara di luar repo `profil_dqfd.py`: fungsi `drl/lingkungan.py`, `drl/perencana_dqfd.py`, `rencana.py`, dan `forklift_planner.py` dibungkus pencatat waktu eksklusif, tanpa mengubah kode. Windows 3 run × 30 ulangan, Mac 1 run × 30 ulangan (2026-11-04) |
| Kode DQfD yang diukur | `drl/lingkungan.py` dengan cache `observasi()` (dikosongkan oleh `reset`, `step`, `pulihkan_keadaan`) |
| Kasus | `Forklift Robot Program/uji_nyata/Gudang.json` (S1, S2 × ID 01, ID 02) |
| Waktu rencana saat uji nyata (batasan) | `uji_nyata/20261024_073221_Gudang/hasil.jsonl`, field `rencana[].ms` |
| Bagian 6 (waktu tempuh per komponen, makespan) | `uji_nyata/20261024_073221_Gudang/hasil.jsonl` (`t_tempuh_s`, `t_menunggu_s`, `makespan_s`, `rencana[].jalur_cm`) dan `lintasan.csv`. Dipotong dengan skrip sementara di luar repo `pecah_makespan.py`: rencana ulang offline, titik serah dari `rs_titik`, penelusuran lintasan per segmen arah (2026-11-04) |
| Model | `model_dqn/id0N_jauh.pt`, `model_dqn/id0N_dekat.pt` |
