# Skenario Gudang baru (homografi ulang): simulasi makespan dan waktu proses Hybrid A\* + RS vs DQfD murni

Tanggal: 2026-11-04 · Branch `ver-1-5`, commit `1f029d3` · Lanjutan dari [[17 Forward Numpy ver-1-5 dan Perbandingan Waktu Proses serta Makespan|Forward Numpy ver-1-5 dan Perbandingan Waktu Proses serta Makespan]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Homografi kamera diulang, lalu skenario uji Gudang dibuat dari awal dengan `drl.uji_robot_nyata --buat`. Skenario lama dihapus di commit `1f029d3`. Catatan ini merekam bentuk skenario baru dan hasil gladi bersih simulasinya (`--simulasi --ulang 1`), dengan perbandingan **makespan** dan **waktu proses** yang dipecah menjadi Hybrid A\*, RS, model jauh, dan model dekat.

> [!summary] Ringkasan
> - **Makespan:** DQfD murni **11,21 s** vs Hybrid A\* + RS **19,41 s** di S1 (**−42%**). Di S2 keduanya setara (23,97 vs 23,93 s). Jumlah kedua skenario: 35,18 vs 43,34 s (−19%).
> - **Sumber selisih makespan adalah bentuk jalur.** Di S1, DQfD menempuh lengkung S tanpa ganti arah (robot 01), 159/171 cm, sedangkan A\* membuat manuver mundur-maju, 223/221 cm. Di S2 kedua perencana membuat putar balik tiga titik yang hampir identik.
> - **Waktu proses per rencana:** DQfD **11,8 ms** vs A\* **31,1 ms** (**2,6×** lebih cepat).
>   - Pencarian jauh: model jauh **10,34 ms** vs Hybrid A\* **29,75 ms** (2,9×).
>   - Penyelesaian dekat: model dekat **0,64 ms** vs RS **0,33 ms** (RS 2× lebih cepat, tetapi keduanya < 1 ms).
> - **Waktu proses ≤ 0,2% dari makespan.** Makespan ditentukan oleh jalur, bukan oleh komputasi.
> - **n = 1 per sel, simulasi.** Belum bisa dipakai sebagai klaim statistik.

---

## 1. Persiapan skenario

### 1.1 Homografi dan peta
- Homografi diulang dengan `python cctv_lurus.py homografi`. Kotaknya tetap 8 × 7 ubin (240 × 210 cm) dengan titik asal yang sama, jadi koordinat zona di `Camera/peta_constraint/Gudang.json` tetap berlaku. Hasilnya ada di `Camera/kalibrasi_lensa.json`.
- Zona peta Gudang **belum** ditempelkan ke garis ubin. Sisi zona masih meleset 0,5–3 cm dari garis ubin 30 cm (Z3, Z2, Z4, Z6, Z7). Lihat bagian 4.

### 1.2 Skenario akhir (`Forklift Robot Program/uji_nyata/Gudang.json`)

Pose (x cm, y cm, yaw °). Titik acuan pose adalah poros depan. Yaw 90° berarti menghadap +y (ke bawah pada gambar).

| Skenario | Robot | Awal | Tujuan | Gerak |
|---|---|---|---|---|
| S1 | 01 | (32,00; 40,0; 90) | (108,25; 169,8; 90) | lorong kiri-atas → lorong tengah-bawah |
| S1 | 02 | (107,75; 40,0; 90) | (196,25; 167,4; 90) | lorong tengah-atas → lorong kanan-bawah |
| S2 | 01 | (107,75; 33,8; −90) | (108,25; 174,5; 90) | lorong tengah, putar balik 180° |
| S2 | 02 | (195,75; 26,0; −90) | (196,25; 164,4; 90) | lorong kanan, putar balik 180° |

Penyesuaian setelah skenario dibuat dengan klik:
1. **Yaw dibulatkan ke kelipatan 90°** supaya lurus terhadap dinding zona. Sebelumnya −95,7° sampai −86,1°.
2. **x dipindah ke tengah lorong**, yaitu titik tengah antara dua dinding yang rentang y-nya bersinggungan dengan badan robot. Lorongnya: Z3–Z6 (62 cm), Z6–Z5 (82,5 cm), Z7–Z8 (81,5 cm), Z5–Z2 (85,5 cm), Z8–Z2 (84,5 cm).
3. **y awal S1 dinaikkan ke 40,0** (sebelumnya 34,5 dan 31,1). Lihat bagian 1.3.
4. **Arah tujuan S2 dibalik** dari −90° ke 90°, supaya skenario lebih menantang (robot harus putar balik di lorong). Panjang jalur naik dari ±140 cm ke ±195 cm.

Setiap pose diperiksa **sebagai tujuan** untuk kedua perencana, dan semuanya muat dengan margin 8 cm. Jalur setiap robot juga direncanakan ulang dengan kedua perencana.

### 1.3 Temuan: pose awal yang tidak bisa dipakai pulang
- Pose awal S1 yang pertama, y = 34,5 dan 31,1 dengan yaw 90°, menaruh buntut badan (23 cm di belakang poros depan) 8–12 cm dari dinding Z1.
- Pose itu lolos pemeriksaan `w` di mode `--buat`, karena `periksa_skenario` hanya memeriksa **tujuan** dan merencanakan **dari** awal. Perencana sengaja longgar untuk pose awal.
- Namun saat **pulang**, pose awal menjadi tujuan dan ditolak ("tujuan terlalu rapat ke zona – badan robot tidak muat walau dengan jarak aman 6 cm"). Robot diam, dan setelah 3 kali mencoba program meminta operator.
- Batas y terkecil: **36,0 / 36,1** dengan margin 6 cm, dan **38,0 / 38,1** dengan margin 8 cm. Dipilih 40,0.
- Sesudah diperbaiki, jalur pulang dari T ke A memang mundur: mundur ±135–150 cm sambil berbelok → maju 4–12 cm → mundur 30 cm lurus ke titik awal. Ini berlaku untuk Hybrid A\* maupun DQfD murni.

> [!tip] Untuk skenario berikutnya
> Pose awal harus juga lolos sebagai **tujuan** (`margin_untuk_pose`), karena protokol uji mengirim robot pulang ke sana sebelum setiap percobaan.

## 2. Makespan

Simulasi `python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang.json --simulasi --ulang 1`. Perencana pulang Hybrid A\*, derau 0, toleransi awal 5 cm / 10°. Kedua robot memakai perencana yang sama secara bersamaan. Semua robot tiba, tanpa estop, tanpa menunggu, dan tanpa rencana ulang.

| Skenario | Hybrid A\* + RS | DQfD murni | Selisih |
|---|---:|---:|---:|
| S1 | 19,41 s | **11,21 s** | **−8,20 s (−42%)** |
| S2 | 23,93 s | 23,97 s | +0,04 s (±0%) |
| **Jumlah** | 43,34 s | **35,18 s** | **−8,16 s (−19%)** |

| | Hybrid A\* + RS | DQfD murni |
|---|---|---|
| S1 panjang jalur, robot 01 / 02 | 223,0 / 221,4 cm | **159,2 / 171,3 cm** |
| S1 ganti arah | 2 / 2 | **0** / 2 |
| S1 waktu tempuh | 19,41 / 19,02 s | 11,21 / 10,21 s |
| S2 panjang jalur | 192,0 / 194,8 cm | 192,1 / 194,7 cm |
| S2 ganti arah | 3 / 3 | 3 / 3 |
| S2 waktu tempuh | 23,55 / 23,93 s | 23,40 / 23,97 s |
| Galat akhir (semua) | 2,3–2,9 cm, 2,8–3,2° | 1,5–2,7 cm, 0,9–4,4° |
| Jarak terdekat ke zona | 9,8–14,9 cm | 11,7–17,5 cm |

![[Simulasi Gudang 2026-11-04 S1.png]]

- **S1:** Hybrid A\* memutar dengan manuver mundur-maju di tengah ruangan (y ≈ 115–125) sebelum masuk ke T. DQfD langsung melengkung dari A ke T. Jalurnya 50–64 cm lebih pendek.

![[Simulasi Gudang 2026-11-04 S2.png]]

- **S2:** kedua perencana membuat putar balik tiga titik yang sama: mundur lurus, berbelok ke kiri di y ≈ 110–120, lalu maju masuk ke T menghadap bawah. Panjang jalurnya berbeda < 0,2 cm, sehingga S2 **tidak membedakan** perencana dari sisi makespan.

## 3. Waktu proses per rencana

Setiap robot merencanakan **satu kali** per percobaan, jadi ada 4 rencana per perencana. Angka diambil dari `rincian_ms` di `hasil.jsonl`, dicatat saat uji berjalan, sehingga perencana berbagi CPU dengan loop simulasi 20 Hz.

### 3.1 Rata-rata per bagian

| Bagian | Hybrid A\* + RS | DQfD murni | Rasio |
|---|---:|---:|---:|
| **Pencarian jauh:** Hybrid A\* / model jauh | 29,75 ms (96%) | **10,34 ms** (87%) | DQfD 2,9× lebih cepat |
| **Penyelesaian dekat:** RS / model dekat | **0,33 ms** (1%) | 0,64 ms (5%) | RS 2× lebih cepat |
| Lain-lain | 1,04 ms (3%) | 0,85 ms (7%) | |
| **Total per rencana** | 31,1 ms | **11,8 ms** | DQfD 2,6× lebih cepat |

### 3.2 Rincian setiap rencana (ms)

| Percobaan | Robot | Hybrid A\* | RS | Model jauh | Model dekat | Lain | Total |
|---|---|---:|---:|---:|---:|---:|---:|
| S1 Hybrid A\* + RS | 01 | 24,39 | 0,38 | – | – | 2,59 | 27,36 |
| S1 Hybrid A\* + RS | 02 | 19,87 | 0,31 | – | – | 0,04 | 20,21 |
| S2 Hybrid A\* + RS | 01 | 33,08 | 0,29 | – | – | 1,50 | 34,88 |
| S2 Hybrid A\* + RS | 02 | 41,66 | 0,32 | – | – | 0,03 | 42,01 |
| S1 DQfD murni | 01 | – | – | 9,56 | 0,95 | 1,98 | 12,49 |
| S1 DQfD murni | 02 | – | – | 15,77 | 0,41 | 0,02 | 16,20 |
| S2 DQfD murni | 01 | – | – | 9,29 | 0,71 | 1,33 | 11,33 |
| S2 DQfD murni | 02 | – | – | 6,74 | 0,50 | 0,06 | 7,29 |

Pengamatan:
- **Hybrid A\* naik di S2** (33–42 ms) dibanding S1 (20–24 ms), karena putar balik memperluas pencarian. **Model jauh tidak naik** (6,7–9,3 ms di S2 vs 9,6–15,8 ms di S1).
- **RS vs model dekat:** RS konstan ±0,3 ms. Model dekat 0,4–1,0 ms. Keduanya kurang dari 3% dari total A\*, jadi selisihnya tidak berarti.
- **"Lain" 1,3–2,6 ms selalu muncul pada robot 01**, yaitu robot yang merencanakan lebih dulu di setiap percobaan, untuk kedua perencana. Pada robot 02 hanya ≤ 0,06 ms. Penyebabnya belum ditelusuri.
- **Dampak ke makespan bisa diabaikan.** Rencana terlama 42 ms, < 0,2% dari makespan 11–24 s.

## 4. Kesimpulan dan langkah berikutnya

1. **Keunggulan makespan DQfD murni sepenuhnya dari S1**, karena jalurnya lebih pendek dan tanpa ganti arah. S2 (putar balik di lorong sendiri) tidak membedakan perencana.
2. **Waktu proses DQfD murni unggul dengan konsisten, terutama di model jauh vs Hybrid A\*.** Di bagian dekat, RS sedikit lebih cepat, tetapi tidak berpengaruh ke total.
3. Langkah berikutnya:
   - Ulangi dengan **`--ulang 5`**, di simulasi dengan derau dan di robot nyata, untuk mendapat sebaran dan uji signifikansi.
   - Tambah skenario yang **memaksa pilihan rute**, misalnya pindah lorong lewat celah di ujung Z5–Z8. Ini lebih mungkin membedakan makespan daripada putar balik di tempat.
   - Rapikan zona peta Gudang ke garis ubin (bagian 1.1), lalu periksa ulang tengah lorong dan semua pose skenario.
   - Telusuri biaya "lain" pada rencana pertama setiap percobaan.

> [!warning] Batasan
> - **n = 1 per sel (skenario × perencana), simulasi tanpa derau.** Tidak ada sebaran, jadi belum ada uji statistik.
> - **Skenario berbeda dari catatan 15 dan 17**, sehingga angkanya tidak bisa dibandingkan langsung. Catatan 17 juga memakai `drl.uji_navigasi` dengan derau 1,2 cm, sedangkan catatan ini memakai protokol `drl.uji_robot_nyata` (pulang → uji).
> - **Waktu rencana dicatat saat simulasi berjalan.** Angkanya lebih besar daripada pengukuran offline, misalnya A\* 24,5 ms di catatan 17.
> - **Uji robot nyata dengan skenario ini belum dilakukan.**

## 5. Sumber angka

| Angka | Sumber |
|---|---|
| Skenario | `Forklift Robot Program/uji_nyata/Gudang.json`, salinannya di folder hasil (`skenario.json`) |
| Bagian 1.2–1.3 | skrip sementara di luar repo: `lurus_tengah.py` (yaw dan tengah lorong), `ubah_y.py` dan `cari_y.py` (batas y awal), `cek_pulang.py` (jalur pulang), `balik_s2.py` (arah tujuan S2) |
| Bagian 2–3 | `Forklift Robot Program/uji_nyata/20261104_224635_Gudang_sim/` (`hasil.jsonl`, `ringkasan.json`, `rincian_tabel.tex`, `lintasan_S1.png`, `lintasan_S2.png`), direkap dengan skrip sementara `rekap.py` |
