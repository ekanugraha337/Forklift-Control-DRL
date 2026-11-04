# Skenario Gudang baru (homografi ulang): simulasi makespan dan waktu proses Hybrid A\* + RS vs DQfD murni

Tanggal: 2026-11-04, diperbarui 2026-11-05 · Branch `ver-1-5`, kode commit `4d26e77` · Lanjutan dari [[17 Forward Numpy ver-1-5 dan Perbandingan Waktu Proses serta Makespan|Forward Numpy ver-1-5 dan Perbandingan Waktu Proses serta Makespan]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Homografi kamera diulang, lalu skenario uji Gudang dibuat dari awal dengan `drl.uji_robot_nyata --buat`. Skenario lama dihapus di commit `1f029d3`. Pada 2026-11-05 ditambahkan S3 dan S4, dua skenario yang memaksa robot **pindah lorong lewat celah di ujung dinding**. Catatan ini merekam bentuk keempat skenario, hasil **simulasi** (`--simulasi --ulang 2`, bagian 1–6), dan hasil **uji robot nyata** dengan skenario yang sama (`--ulang 2`, bagian 7). Keduanya membandingkan **makespan** dan **waktu proses** yang dipecah menjadi Hybrid A\*, RS, model jauh, dan model dekat.

Semua angka diukur dengan metode di bagian 2: tujuan kedua robot diberikan bersamaan, setiap robot merencana di **proses** sendiri, dan waktu dicatat sebagai jam dinding dan CPU.

> [!summary] Ringkasan: 4 skenario × 2 perencana × 2 ulangan, di simulasi dan di robot nyata
> **Makespan rata-rata** (DQfD murni vs Hybrid A\* + RS):
>
> | Skenario | Simulasi (`20261105_002833_Gudang_sim`) | Robot nyata (`20261105_010053_Gudang`) |
> |---|---|---|
> | S1, lintas lorong diagonal | **10,33** vs 18,47 s (**−44%**) | **12,05** vs 16,36 s (**−26%**) |
> | S2, putar balik di lorong sendiri | 24,00 vs 23,99 s (setara) | **25,07** vs 28,01 s (−10%) |
> | S3, pindah lorong mengitari ujung Z6/Z8 | 11,88 vs **10,82 s** (DQfD **+10%**) | **11,60** vs 15,22 s (**−24%**) |
> | S4, pindah lorong mengitari ujung Z7/Z5 | **10,74** vs 17,60 s (**−39%**) | **12,64** vs 13,31 s (−5%) |
> | **Jumlah** | **56,95 vs 70,88 s (−20%)** | **61,36 vs 72,90 s (−16%)** |
>
> - **Selisih makespan berasal dari bentuk jalur.** A\* kalah bila membuat manuver mundur-maju di tengah jalan:
>   - di simulasi: S1 (robot 01 dua kali, robot 02 sekali) dan S4 (robot 02 dua kali);
>   - di robot nyata: S1 dan S3 (robot 02, masing-masing sekali).
>
>   DQfD tidak pernah membuat mundur-maju di tengah jalan. DQfD kalah sekali, di S3 simulasi, karena busurnya mengitari Z8 terlalu lebar. **Di robot nyata DQfD lebih cepat di keempat skenario.** Kapan A\* membuat mundur-maju berubah antar-percobaan, mengikuti pose awal hasil pulang (bagian 7.5).
> - **Waktu proses per rencana** (16 rencana per perencana; jam dinding = CPU, selisih ≤ 0,4 ms):
>
>   | | Simulasi: Hybrid A\* + RS / DQfD | Robot nyata: Hybrid A\* + RS / DQfD |
>   |---|---|---|
>   | Total, rata-rata | 26,3 / **10,5 ms** (2,5×) | 24,5 / **9,2 ms** (2,6×) |
>   | Total, median | 18,5 / **10,0 ms** (1,8×) | 19,4 / **9,2 ms** (2,1×) |
>   | Pencarian jauh: Hybrid A\* / model jauh | 25,8 / **9,9 ms** | 24,1 / **8,5 ms** |
>   | Penyelesaian dekat: RS / model dekat | **0,40** / 0,49 ms | **0,34** / 0,64 ms |
>
> - **Waktu proses di robot nyata setara dengan simulasi** (selisih ≤ 15%), karena perencana berjalan di proses sendiri. Waktu proses ≤ 0,8% dari makespan, jadi makespan ditentukan oleh jalur, bukan oleh komputasi.
> - **Galat pose akhir** (16 pose per perencana; posisi / arah, rata-rata):
>
>   | | Hybrid A\* + RS | DQfD murni |
>   |---|---|---|
>   | Simulasi | 2,73 cm / 4,42° (maks 4,25 cm / 8,96°) | 2,53 cm / 4,51° (maks 4,98 cm / 9,85°) |
>   | Robot nyata | 2,51 cm / 5,07° (maks 3,84 cm / 9,28°) | 2,12 cm / 4,64° (maks 3,61 cm / 10,47°) |
>
>   Kedua perencana setara presisinya. Galat arah terbesar di S3–S4 (tujuan sesaat setelah putaran di ujung dinding). Di robot nyata, robot 02 miring ±5° dan bergeser ±2 cm secara sistematis, sedangkan robot 01 hampir tepat (bagian 7.3).
> - **Tanpa interaksi antar-robot** di kedua uji: waktu menunggu 0 s, minggir 0 kali. Jarak badan terdekat kedua robot 30,5–71,3 cm di simulasi dan 18,5–51,6 cm di robot nyata.
> - **n = 2 per sel.** Belum cukup untuk uji statistik.

---

## 1. Persiapan skenario

### 1.1 Homografi dan peta
- Homografi diulang dengan `python cctv_lurus.py homografi`. Kotaknya tetap 8 × 7 ubin (240 × 210 cm) dengan titik asal yang sama, jadi koordinat zona di `Camera/peta_constraint/Gudang.json` tetap berlaku. Hasilnya ada di `Camera/kalibrasi_lensa.json`.
- Zona peta Gudang **belum** ditempelkan ke garis ubin. Sisi zona masih meleset 0,5–3 cm dari garis ubin 30 cm (Z3, Z2, Z4, Z6, Z7). Lihat bagian 5.

![[Peta Gudang 2026-11-04.png]]

Gambar di atas menampilkan zona larangan (merah), grid ubin 30 cm (cyan), dan lebar lorong. Lorong atas dibatasi Z6 dan Z5 (ujung y = 55 dan 50,5). Lorong bawah dibatasi Z7 dan Z8 (ujung y = 144,5 dan 134). Ruang tengah, y ≈ 55–134, terbuka selebar arena.

### 1.2 Skenario (`Forklift Robot Program/uji_nyata/Gudang.json`)

Pose (x cm, y cm, yaw °). Titik acuan pose adalah poros depan. Yaw 90° berarti menghadap +y (ke bawah pada gambar).

| Skenario | Robot | Awal | Tujuan | Gerak |
|---|---|---|---|---|
| S1 | 01 | (32,00; 40,0; 90) | (108,25; 169,8; 90) | lorong kiri-atas → lorong tengah-bawah |
| S1 | 02 | (107,75; 40,0; 90) | (196,25; 167,4; 90) | lorong tengah-atas → lorong kanan-bawah |
| S2 | 01 | (107,75; 33,8; −90) | (108,25; 174,5; 90) | lorong tengah, putar balik 180° |
| S2 | 02 | (195,75; 26,0; −90) | (196,25; 164,4; 90) | lorong kanan, putar balik 180° |
| S3 | 01 | (32,00; 40,0; 90) | (107,75; 40,0; −90) | kiri-atas → tengah-atas, mengitari ujung bawah **Z6** |
| S3 | 02 | (108,25; 170,0; −90) | (196,25; 170,0; 90) | tengah-bawah → kanan-bawah, mengitari ujung atas **Z8** |
| S4 | 01 | (108,25; 170,0; −90) | (31,75; 170,0; 90) | tengah-bawah → kiri-bawah, mengitari ujung atas **Z7** |
| S4 | 02 | (195,75; 40,0; 90) | (107,75; 40,0; −90) | kanan-atas → tengah-atas, mengitari ujung bawah **Z5** |

Penyesuaian S1 dan S2 setelah dibuat dengan klik:
1. **Yaw dibulatkan ke kelipatan 90°** supaya lurus terhadap dinding zona. Sebelumnya −95,7° sampai −86,1°.
2. **x dipindah ke tengah lorong**, yaitu titik tengah antara dua dinding yang rentang y-nya bersinggungan dengan badan robot. Lorongnya: Z3–Z6 (62 cm), Z6–Z5 (82,5 cm), Z7–Z8 (81,5 cm), Z5–Z2 (85,5 cm), Z8–Z2 (84,5 cm).
3. **y awal S1 dinaikkan ke 40,0** (sebelumnya 34,5 dan 31,1). Lihat bagian 1.4.
4. **Arah tujuan S2 dibalik** dari −90° ke 90°, supaya robot harus putar balik di lorong.

**Rancangan S3 dan S4 (pindah lorong lewat celah di ujung dinding):**
- Setiap robot pindah ke lorong sebelah dengan **mengitari ujung dinding**. Tidak ada rute lurus: robot harus keluar dari lorongnya ke pita tengah, berbalik arah 180°, lalu masuk ke lorong sebelah.
- **Seimbang antar-robot.** Di S3, robot 01 mengitari ujung dinding yang menggantung dari atas (Z6) dan robot 02 ujung dinding yang berdiri dari bawah (Z8). Di S4 peran keduanya ditukar (Z7 dan Z5).
- **Tanpa interaksi.** Pita tengah Gudang hanya ±79 cm, sedangkan satu putaran di ujung dinding memakan ±40 cm, sehingga jarak sapuan kedua robot hanya 8–27 cm. Karena itu di setiap skenario satu robot **berakhir** di lorong tengah dan robot lainnya **berangkat** dari lorong tengah. Kedekatan di kolom tengah menjadi terpisah waktu. Perkiraan jarak badan terdekat pada waktu yang sama (laju sama, berangkat bersamaan): 27–63 cm. Terukur di simulasi: 30,5–71,3 cm (bagian 3).
- Pose di tengah lorong dengan yaw ±90°, sama seperti S1 dan S2.

Setiap pose diperiksa **sebagai tujuan** untuk kedua perencana, dan semuanya muat dengan margin 8 cm. Semua rencana uji dan semua jalur pulang (tujuan → awal, Hybrid A\*) berhasil.

### 1.3 Skenario dan heatmap Dijkstra

Panel kiri menampilkan pose awal (titik-titik) dan tujuan (garis putus) dengan panah arah hadap. Biru adalah robot 01, merah robot 02. Panel tengah dan kanan adalah **heuristik Dijkstra** ke tujuan setiap robot, yaitu biaya-sisa holonomik dalam cm. Peta ini dihitung oleh `_holonomic_heuristic` (Dijkstra 8-tetangga, sel 5 cm, sel diblokir bila jarak bebasnya < setengah lebar badan). Fungsi yang sama dipakai Hybrid A\* sebagai heuristik dan lingkungan DQfD sebagai peta Dijkstra observasi (`drl/lingkungan.py:430`), jadi satu peta per robot berlaku untuk **kedua** perencana. Kondisinya sama dengan saat uji: kedua robot merencana tanpa badan robot lain sebagai rintangan.

![[Skenario Gudang 2026-11-05 S1.png]]

![[Skenario Gudang 2026-11-05 S2.png]]

![[Skenario Gudang 2026-11-05 S3.png]]

![[Skenario Gudang 2026-11-05 S4.png]]

Panjang jalur dari run `20261105_002833_Gudang_sim` (ulangan 1 / ulangan 2):

| Skenario | Robot | Biaya Dijkstra di titik awal | Jalur Hybrid A\* | Jalur DQfD murni |
|---|---|---:|---:|---:|
| S1 | 01 | 156 cm | 223,0 / 215,5 cm | 156,5 / 156,5 cm |
| S1 | 02 | 162 cm | 221,4 / 170,1 cm | 169,6 / 170,0 cm |
| S2 | 01 | 140 cm | 192,0 / 192,0 cm | 192,1 / 192,0 cm |
| S2 | 02 | 135 cm | 194,8 / 194,7 cm | 194,7 / 194,7 cm |
| S3 | 01 | 96 cm | 144,7 / 148,1 cm | 150,9 / 149,4 cm |
| S3 | 02 | 143 cm | 171,1 / 175,6 cm | 192,8 / 183,7 cm |
| S4 | 01 | 120 cm | 150,6 / 157,3 cm | 152,9 / 154,2 cm |
| S4 | 02 | 107 cm | 199,0 / 190,4 cm | 165,2 / 158,8 cm |

- **Dijkstra mengabaikan arah hadap dan jari-jari belok**, jadi nilainya batas bawah kasar untuk panjang jalur.
- **S1:** jalur DQfD 0–5% di atas biaya Dijkstra. A\* 37–43% di atasnya karena manuver mundur-maju di ruang tengah, kecuali robot 02 di ulangan 2 (170,1 cm, +5%).
- **S2:** putar balik 180° menambah 52–60 cm untuk kedua perencana. S2 tidak membedakan perencana.
- **S3:** robot 01 (mengitari Z6) setara, +51–57% untuk keduanya. Robot 02 (mengitari Z8): A\* +20–23%, DQfD **+28–35%**.
- **S4:** robot 01 (mengitari Z7) setara, +26–31%. Robot 02 (mengitari Z5): DQfD +48–54%, A\* **+78–86%**.

### 1.4 Temuan: pose awal harus muat sebagai tujuan pulang
- Pose awal S1 yang pertama, y = 34,5 dan 31,1 dengan yaw 90°, menaruh buntut badan (23 cm di belakang poros depan) 8–12 cm dari dinding Z1.
- Pose itu lolos pemeriksaan `w` di mode `--buat`, karena `periksa_skenario` hanya memeriksa **tujuan** dan merencanakan **dari** awal. Perencana sengaja longgar untuk pose awal.
- Namun saat **pulang**, pose awal menjadi tujuan dan ditolak ("tujuan terlalu rapat ke zona – badan robot tidak muat walau dengan jarak aman 6 cm"). Robot diam, dan setelah 3 kali mencoba program meminta operator.
- Batas y terkecil: **36,0 / 36,1** dengan margin 6 cm, dan **38,0 / 38,1** dengan margin 8 cm. Dipilih 40,0.
- Sesudah diperbaiki, jalur pulang dari T ke A memang mundur: mundur ±135–150 cm sambil berbelok → maju 4–12 cm → mundur 30 cm lurus ke titik awal. Ini berlaku untuk Hybrid A\* maupun DQfD murni.

> [!tip] Untuk skenario berikutnya
> Pose awal harus juga lolos sebagai **tujuan** (`margin_untuk_pose`), karena protokol uji mengirim robot pulang ke sana sebelum setiap percobaan.

## 2. Metode pengukuran

```powershell
python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang.json --simulasi --ulang 2
```

- **Protokol** (`drl/uji_robot_nyata.py`): sebelum setiap percobaan, robot pulang ke pose awal dengan Hybrid A\* (tidak diukur, toleransi 5 cm / 10°). Lalu kedua robot diberi tujuannya dengan perencana yang diuji, bersamaan. Urutan perencana diselang-seling: ulangan 1 A\* lalu DQfD, ulangan 2 DQfD lalu A\*. Ada 16 percobaan, simulasi tanpa derau.
- **Tujuan simetris.** Tujuan semua robot ditetapkan dulu (`terima_tujuan(tunda=True)`), baru setiap robot diminta merencana. Tidak ada robot yang merencana dengan pose awal robot lain sebagai rintangan.
- **Satu proses perencana per robot** (`perencana_proses.py`, commit `4d26e77`). Setiap robot merencana di proses dan interpreter Python sendiri, sehingga tidak berbagi GIL dengan loop kendali atau dengan perencana robot lain. Perencanaan di thread sudah dihapus, karena waktu jam dinding yang tercatat ikut memuat pekerjaan lain. `navigasi.Unit.minta_rencana()` menolak perencana yang bukan `PerencanaDiProses`.
- **Yang dicatat untuk setiap rencana** (`hasil.jsonl`):
  - `ms`: waktu jam dinding `rencanakan()` di proses perencana.
  - `cpu_ms`: waktu CPU thread perencana. Di Windows memakai `QueryThreadCycleTime`, karena `time.thread_time()` di sana hanya naik per 15,625 ms.
  - `rincian_ms`: bagian Hybrid A\*, RS, model jauh, model dekat, dan lain-lain.
  - `mulai_ms`: jeda dari permintaan rencana sampai rencana mulai.
  - `ipc_ms`: biaya antar-proses.
  - `pid`: proses yang merencana.
- **Bukti hanya proses yang merencana:** setiap rencana robot 01 dikerjakan pid 35092 dan robot 02 pid 49120. Keduanya proses anak, bukan proses utama. Robot 01 mulai merencana +0,7–1,0 ms setelah permintaan, robot 02 +1,3–2,2 ms. Celah loop kendali terbesar 0,08 s.

## 3. Makespan

Run `20261105_002833_Gudang_sim`. 16/16 percobaan selesai, semua robot tiba. Tidak ada estop, tidak ada rencana ulang, waktu menunggu 0 s, dan minggir 0 kali.

| Skenario | Hybrid A\* + RS (ulangan 1 / 2 → rata-rata) | DQfD murni (ulangan 1 / 2 → rata-rata) | Selisih |
|---|---:|---:|---:|
| S1 | 19,23 / 17,71 → 18,47 s | 10,36 / 10,30 → **10,33 s** | **−8,14 s (−44%)** |
| S2 | 23,91 / 24,07 → 23,99 s | 24,02 / 23,99 → 24,00 s | +0,01 s (±0%) |
| S3 | 10,75 / 10,89 → **10,82 s** | 11,36 / 12,39 → 11,88 s | +1,06 s (+10%) |
| S4 | 17,61 / 17,59 → 17,60 s | 10,19 / 11,29 → **10,74 s** | **−6,86 s (−39%)** |
| **Jumlah** | 70,88 s | **56,95 s** | **−13,93 s (−20%)** |

| Skenario | Robot | Hybrid A\* + RS: waktu tempuh · ganti arah | DQfD murni: waktu tempuh · ganti arah |
|---|---|---|---|
| S1 | 01 | 19,23 / 17,71 s · 2 | 10,36 / 10,30 s · **0** |
| S1 | 02 | 18,98 / 10,12 s · 2 | 10,17 / 10,18 s · 2 |
| S2 | 01 | 23,40 / 23,43 s · 3 | 23,39 / 23,36 s · 3 |
| S2 | 02 | 23,91 / 24,07 s · 3 | 24,02 / 23,99 s · 3 |
| S3 | 01 | 9,01 / 9,35 s · 2 | 9,45 / 12,39 s · 2 |
| S3 | 02 | 10,75 / 10,89 s · 2 | 11,36 / 10,98 s · 2 |
| S4 | 01 | 9,67 / 10,22 s · 2 | 10,13 / 9,88 s · 2 |
| S4 | 02 | 17,61 / 17,59 s · 2 | 10,19 / 11,29 s · 2 |

| | Hybrid A\* + RS | DQfD murni |
|---|---|---|
| Galat akhir S1–S2 | 1,6–3,0 cm, 2,7–4,1° | 1,3–2,9 cm, 1,7–4,1° |
| Galat akhir S3–S4 | 1,6–4,3 cm, 2,5–9,0° | 1,9–5,0 cm, 4,3–9,9° |
| Jarak terdekat ke zona | 9,8–18,7 cm | 11,1–20,2 cm |
| Jarak badan terdekat kedua robot | 37,3–71,3 cm | 30,5–52,2 cm |

![[Simulasi Gudang 2026-11-05 S1.png]]

- **S1:** A\* memutar dengan manuver mundur-maju di ruang tengah (y ≈ 115–125) sebelum masuk ke T. Di ulangan 2, robot 02 menemukan jalur mulus 170 cm, sehingga makespan A\* turun ke 17,71 s, dibatasi robot 01. DQfD menempuh lengkung S yang mulus di kedua ulangan.

![[Simulasi Gudang 2026-11-05 S2.png]]

- **S2:** kedua perencana membuat putar balik tiga titik yang sama: mundur lurus, berbelok ke kiri di y ≈ 110–120, lalu maju masuk ke T menghadap bawah. Panjang jalur berbeda ≤ 0,1 cm.

![[Simulasi Gudang 2026-11-05 S3.png]]

- **S3:** keduanya menempuh busur mulus mengitari ujung dinding. Busur DQfD robot 02 mengitari Z8 **lebih lebar** (puncak y ≈ 102, A\* ≈ 110), sehingga jalurnya 8–22 cm lebih panjang. Di ulangan 2 robot 01 DQfD juga lebih lambat (12,39 s untuk 149 cm, ulangan 1: 9,45 s), sehingga DQfD kalah 10% di skenario ini.

![[Simulasi Gudang 2026-11-05 S4.png]]

- **S4:** robot 01 (mengitari Z7) setara. A\* robot 02 membuat **manuver mundur-maju** di sekitar (160, 70) sebelum mengitari Z5 (190–199 cm, 17,6 s). DQfD menempuh busur mulus (159–165 cm, 10,2–11,3 s).
- **Galat arah akhir di S3–S4 lebih besar** (sampai 9–10°, di S1–S2 ≤ 4,1°) untuk kedua perencana. Dugaan penyebabnya: di S3 dan S4 tujuan tercapai sesaat setelah putaran mengitari ujung dinding, sehingga bagian lurus terakhir pendek (10–15 cm di lorong atas, dari ujung dinding y 50,5–55 ke tujuan y 40; 25–36 cm di lorong bawah) dan pelacak tidak sempat meluruskan arah sepenuhnya. Hubungan ini belum diukur tersendiri.

### 3.1 Galat pose akhir (simulasi)

Pose akhir diukur sebagai rata-rata 1 s setelah robot berhenti. Galatnya dihitung terhadap pose tujuan dan diuraikan menjadi:
- **posisi**: jarak titik acuan ke tujuan;
- **arah**: selisih yaw mutlak;
- **maju**: komponen searah arah hadap tujuan. Nilai **+** berarti melewati tujuan, **−** berarti berhenti sebelum tujuan;
- **samping**: komponen tegak lurus. Tandanya bergantung pada arah belokan skenario, jadi yang dibandingkan magnitudonya.

| Skenario | Robot | Hybrid A\* + RS: posisi (ulangan 1 / 2) | Hybrid A\* + RS: arah | DQfD murni: posisi | DQfD murni: arah |
|---|---|---:|---:|---:|---:|
| S1 | 01 | 2,96 / 2,96 cm | 3,21 / 3,05° | 2,76 / 2,58 cm | 1,73 / 1,76° |
| S1 | 02 | 2,44 / 1,59 cm | 3,18 / 4,11° | 1,31 / 1,46 cm | 3,94 / 4,13° |
| S2 | 01 | 2,40 / 2,71 cm | 2,70 / 2,85° | 2,72 / 2,60 cm | 2,74 / 2,71° |
| S2 | 02 | 2,84 / 2,86 cm | 3,14 / 3,12° | 2,93 / 2,67 cm | 3,20 / 3,12° |
| S3 | 01 | 3,76 / 4,25 cm | 8,67 / 8,96° | 3,90 / 1,99 cm | 8,53 / 4,82° |
| S3 | 02 | 1,55 / 1,87 cm | 3,90 / 4,58° | 2,17 / 1,89 cm | 4,32 / 4,41° |
| S4 | 01 | 2,72 / 3,19 cm | 6,60 / 7,39° | 2,32 / 2,25 cm | 6,38 / 6,12° |
| S4 | 02 | 2,74 / 2,83 cm | 2,52 / 2,67° | 4,98 / 1,96 cm | 9,85 / 4,37° |

| Ringkasan (16 pose per perencana) | Hybrid A\* + RS | DQfD murni |
|---|---:|---:|
| Galat posisi: rata-rata / median / maks | 2,73 / 2,79 / 4,25 cm | **2,53** / 2,45 / 4,98 cm |
| Galat arah: rata-rata / median / maks | **4,42** / 3,20 / 8,96° | 4,51 / 4,23 / 9,85° |
| Komponen maju: rata-rata \|maju\| (rata-rata bertanda) | 1,92 (−1,11) cm | 1,46 (−0,65) cm |
| Komponen samping: rata-rata \|samping\| / maks | 1,67 / 3,40 cm | 1,75 / 3,65 cm |
| Galat posisi per skenario, rata-rata S1 / S2 / S3 / S4 | 2,48 / 2,70 / 2,86 / 2,87 cm | 2,03 / 2,73 / 2,49 / 2,88 cm |
| Galat arah per skenario, rata-rata S1 / S2 / S3 / S4 | 3,39 / 2,95 / 6,53 / 4,80° | 2,89 / 2,94 / 5,52 / 6,68° |

- **Kedua perencana setara**: posisi rata-rata 2,5–2,7 cm, arah 4,4–4,5°. Semua pose akhir ≤ 5 cm dari tujuan.
- **Di S1–S2 robot berhenti sedikit sebelum tujuan.** Komponen maju negatif di semua 16 pose S1–S2 (−0,3 sampai −2,8 cm); di S2 selalu −2,2 sampai −2,8 cm, untuk kedua perencana. Di S3–S4 robot 01 justru sering melewati tujuan (maju sampai +2,7 cm).
- **Galat arah terbesar di S3–S4 robot 01** (rata-rata 7,2°, sampai 9,0°), yaitu tujuan yang dicapai sesaat setelah putaran di ujung dinding.
- Di simulasi kedua robot berperilaku mirip. Di S1–S2, galat arah robot 01 rata-rata 2,6° dan robot 02 3,5°.

## 4. Waktu proses per rencana

Setiap robot merencanakan **satu kali** per percobaan, jadi ada 16 rencana per perencana.

### 4.1 Rata-rata semua rencana

| Bagian | Hybrid A\* + RS | DQfD murni | Rasio |
|---|---:|---:|---:|
| **Pencarian jauh:** Hybrid A\* / model jauh | 25,78 ms (98%) | **9,94 ms** (94%) | DQfD 2,6× lebih cepat |
| **Penyelesaian dekat:** RS / model dekat | **0,40 ms** (2%) | 0,49 ms (5%) | RS 1,2× lebih cepat |
| Lain-lain | 0,06 ms | 0,09 ms | |
| **Total (jam dinding), rata-rata** | 26,25 ms | **10,52 ms** | DQfD 2,5× lebih cepat |
| **Total (jam dinding), median** | 18,47 ms | **10,01 ms** | DQfD 1,8× lebih cepat |
| **Total (CPU), rata-rata** | 26,17 ms | **10,43 ms** | DQfD 2,5× lebih cepat |

### 4.2 Per skenario (rata-rata [terkecil–terbesar] dari 4 rencana)

| Skenario | Hybrid A\* + RS: total | Hybrid A\* / RS | DQfD murni: total | Model jauh / model dekat |
|---|---:|---:|---:|---:|
| S1 | **14,9 ms** [11,4–24,3] | 14,28 / 0,53 | 16,2 ms [15,3–17,3] | 15,67 / 0,45 |
| S2 | 35,2 ms [31,7–39,6] | 34,81 / 0,30 | **6,1 ms** [5,9–6,6] | 5,53 / 0,52 |
| S3 | 40,0 ms [8,1–82,1] | 39,64 / 0,25 | **8,9 ms** [5,6–14,1] | 8,31 / 0,52 |
| S4 | 15,0 ms [11,0–19,0] | 14,39 / 0,54 | **10,8 ms** [6,7–13,3] | 10,25 / 0,46 |

- **DQfD lebih cepat di S2, S3, dan S4.** Di S1 justru A\* sedikit lebih cepat (14,9 vs 16,2 ms).
- **Rencana terlama adalah A\* robot 02 di S3** (mengitari Z8): 57,8 dan 82,1 ms. Untuk kasus yang sama, DQfD butuh 9,0–14,1 ms.
- **Kedua perencana peka terhadap pose awal hasil pulang.** Contohnya A\* S1 robot 02, yang jalurnya 221 cm di ulangan 1 dan 170 cm di ulangan 2, dengan waktu rencana 11,6 vs 24,3 ms. Robot memulai percobaan dari pose hasil pulang yang meleset 2,3–3,9 cm dari titik awal, dan pose itu berbeda setiap ulangan.

## 5. Kesimpulan dan langkah berikutnya

1. **Makespan:** DQfD murni lebih cepat di dua dari empat skenario (S1 −44%, S4 −39%), setara di S2, dan lebih lambat di S3 (+10%). Totalnya −20%. Skenario pindah lorong (S3, S4) memang membedakan perencana, tetapi ke dua arah. A\* rugi besar bila membuat manuver mundur-maju yang tidak perlu. DQfD rugi bila busurnya terlalu lebar.
2. **Waktu proses:** DQfD murni 2,5× lebih cepat rata-rata (1,8× median), terutama di model jauh vs Hybrid A\*. Di bagian dekat, RS sedikit lebih cepat. Waktu proses ≤ 0,8% dari makespan.
3. Langkah berikutnya:
   - Tambah ulangan (misalnya `--ulang 5`) dan derau simulasi untuk sebaran dan uji signifikansi. Uji robot nyata dengan skenario ini sudah dilakukan (bagian 7).
   - Uji peta Gudang2 (`Camera/peta_constraint/Gudang2.json` dan `uji_nyata/Gudang2.json`, peta baru dengan pulau rak dan teluk parkir).
   - Rapikan zona peta Gudang ke garis ubin (bagian 1.1), lalu periksa ulang tengah lorong dan semua pose skenario.
   - Jalankan `drl.uji_pose_akhir` sekali. Alat ini sudah dipindah ke `ProcessPoolExecutor`, tetapi belum diuji.

> [!warning] Batasan
> - **n = 2 per sel (skenario × perencana), simulasi tanpa derau.** Belum ada uji statistik.
> - **Titik start tidak identik antar-percobaan.** Percobaan pertama (S1 A\* ulangan 1) dimulai tepat di pose awal (galat awal 0). Percobaan lain dimulai dari pose hasil pulang dengan galat awal 2,3–3,9 cm, dan jalur kedua perencana peka terhadap selisih ini.
> - **Galat arah akhir S3–S4 sampai 10°**, kemungkinan karena bagian lurus terakhir pendek (bagian 3).
> - **Skenario berbeda dari catatan 15 dan 17**, sehingga angkanya tidak bisa dibandingkan langsung. Catatan 15 dan 17 juga diukur dengan perencanaan di thread dan tujuan yang diberikan berurutan.
> - **Bagian 1–6 adalah simulasi.** Hasil robot nyata ada di bagian 7.

## 6. Sumber angka

| Angka | Sumber |
|---|---|
| Skenario | `Forklift Robot Program/uji_nyata/Gudang.json`, salinannya di folder hasil (`skenario.json`) |
| Bagian 1.2 dan 1.4 | skrip sementara di luar repo: `lurus_tengah.py` (yaw dan tengah lorong), `ubah_y.py` dan `cari_y.py` (batas y awal), `cek_pulang.py` (jalur pulang), `balik_s2.py` (arah tujuan S2), `draf_gudang_rute.py` (rancangan S3/S4, jarak sapuan, jarak pada waktu yang sama), `periksa_gudang2.py` (pemeriksaan lewat `Sesi`) |
| Bagian 1.3 (gambar peta, skenario, Dijkstra) | skrip sementara `gambar_peta.py`: `HybridAStarPlanner._holonomic_heuristic` pada `Perencana.peta_dengan()`. Panjang jalur dari `20261105_002833_Gudang_sim` |
| Bagian 2 (metode) | kode commit `4d26e77`: `perencana_proses.py`, `navigasi.py`, `rencana.py` (`jam_cpu_thread`), `drl/uji_robot_nyata.py` |
| Bagian 3–4 | `Forklift Robot Program/uji_nyata/20261105_002833_Gudang_sim/` (`hasil.jsonl`, `lintasan.csv`, `ringkasan.json`, `lintasan_S1.png` – `lintasan_S4.png`), direkap dengan skrip sementara `analisis_u2.py` (jarak badan kedua robot dan celah loop dari `lintasan.csv`) |

---

## 7. Uji robot nyata

Bagian ini terpisah dari simulasi di bagian 1–6. Skenario, perencana, dan metode pengukurannya sama (bagian 1.2 dan 2), tetapi dengan robot ID 01 dan ID 02 sungguhan, pose dari kamera AprilTag, dan perintah lewat MQTT.

### 7.1 Pengaturan dan jalannya uji

| | |
|---|---|
| Program | `python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang.json --ulang 2` (tanpa `--simulasi`) |
| Folder hasil | `Forklift Robot Program/uji_nyata/20261105_010053_Gudang`, 2026-11-05 pukul 01:00–01:14 (16 percobaan dalam 14 menit) |
| Pose | kamera dengan homografi baru (bagian 1.1), AprilTag 36h11, tinggi tag 16,5 cm |
| Perintah | broker MQTT `10.210.1.1:1883`, 20 Hz |
| Konfirmasi | operator menekan `n` sebelum setiap percobaan |
| Perencana | satu proses per robot: pid 50496 (robot 01) dan 8820 (robot 02) di setiap rencana |

- **Semua 16 percobaan selesai**, dan semua robot tiba. Tidak ada estop, tidak ada rencana ulang, waktu menunggu 0 s, dan minggir 0 kali.
- **Pulang:** di setiap percobaan, kedua robot tiba di pose awal pada percobaan pulang **pertama**, tanpa campur tangan operator. Galat awal 0,9–3,9 cm.
- **Kamera:** tag kedua robot terlihat di **100%** baris fase UJI. Hanya satu baris per perencana yang berupa ramalan, untuk robot 02.
- **Loop kendali:** celah terbesar selama UJI 65 ms. Periode median 64 ms, sama dengan simulasi. Jadi loop berjalan ±15,6 Hz, bukan 20 Hz. Dugaannya resolusi timer `cv2.waitKey` di Windows (±15,6 ms), tetapi ini belum diperiksa. Pengaruhnya sama untuk kedua perencana.
- Dua run sebelumnya (`20261105_005722_Gudang`, `20261105_005849_Gudang`) berhenti saat pulang tanpa satu percobaan pun tercatat, jadi tidak dipakai.

### 7.2 Makespan

| Skenario | Hybrid A\* + RS (ulangan 1 / 2 → rata-rata) | DQfD murni (ulangan 1 / 2 → rata-rata) | Selisih |
|---|---:|---:|---:|
| S1 | 14,99 / 17,73 → 16,36 s | 10,40 / 13,71 → **12,05 s** | **−4,31 s (−26%)** |
| S2 | 27,53 / 28,49 → 28,01 s | 25,42 / 24,72 → **25,07 s** | −2,94 s (−10%) |
| S3 | 11,56 / 18,88 → 15,22 s | 11,40 / 11,81 → **11,60 s** | **−3,62 s (−24%)** |
| S4 | 13,07 / 13,55 → 13,31 s | 12,58 / 12,69 → **12,64 s** | −0,67 s (−5%) |
| **Jumlah** | 72,90 s | **61,36 s** | **−11,54 s (−16%)** |

| Skenario | Robot | Hybrid A\* + RS: waktu tempuh · panjang jalur | DQfD murni: waktu tempuh · panjang jalur |
|---|---|---|---|
| S1 | 01 | 14,99 / 11,05 s · 184,6 / 179,5 cm | 10,27 / 10,14 s · 157,8 / 157,8 cm (**0** ganti arah) |
| S1 | 02 | 11,43 / 17,73 s · 184,0 / 210,4 cm | 10,40 / 13,71 s · 174,6 / 178,2 cm |
| S2 | 01 | 23,71 / 23,46 s · 192,7 / 192,1 cm | 24,15 / 24,02 s · 192,6 / 193,3 cm |
| S2 | 02 | 27,53 / 28,49 s · 194,4 / 193,9 cm | 25,42 / 24,72 s · 194,6 / 194,0 cm |
| S3 | 01 | 9,27 / 9,71 s · 144,2 / 154,4 cm | 10,70 / 9,58 s · 150,6 / 150,9 cm |
| S3 | 02 | 11,56 / 18,88 s · 182,8 / 214,5 cm | 11,40 / 11,81 s · 180,6 / 192,7 cm |
| S4 | 01 | 9,64 / 11,96 s · 149,1 / 155,4 cm | 10,61 / 10,08 s · 155,0 / 154,0 cm |
| S4 | 02 | 13,07 / 13,55 s · 157,5 / 154,0 cm | 12,58 / 12,69 s · 159,6 / 159,4 cm |

Ganti arah sama untuk kedua perencana: 2 di S1, S3, dan S4, dan 3 di S2. Satu-satunya pengecualian adalah DQfD S1 robot 01 (0).

| | Hybrid A\* + RS | DQfD murni |
|---|---|---|
| Galat akhir S1–S2 | 0,6–3,8 cm, 0,5–6,5° | 0,1–3,0 cm, 0,1–8,4° |
| Galat akhir S3–S4 | 2,0–3,6 cm, 4,1–9,3° | 1,2–3,6 cm, 4,5–10,5° |
| Jarak terdekat ke zona | **5,4**–16,0 cm | 11,3–19,4 cm |
| Jarak badan terdekat kedua robot | 18,5–46,3 cm | 29,0–51,6 cm |

![[Uji nyata Gudang 2026-11-05 S1.png]]

- **S1:** A\* robot 02 membuat **manuver mundur-maju** di sekitar (163, 117) pada ulangan 2 (210,4 cm, 17,73 s). Ulangan 1 tanpa manuver di tengah (184,0 cm, 11,43 s). Robot 01 A\* tanpa mundur-maju di ruang tengah di kedua ulangan (180–185 cm; di simulasi 216–223 cm dengan mundur-maju). DQfD robot 01 menempuh lengkung S mulus tanpa ganti arah (157,8 cm, sama dengan simulasi 156,5 cm).

![[Uji nyata Gudang 2026-11-05 S2.png]]

- **S2:** kedua perencana membuat putar balik tiga titik yang sama, dengan panjang jalur berbeda ≤ 1 cm. Makespan ditentukan **robot 02**, yang lebih lambat daripada robot 01 dengan kedua perencana (A\* 27,5–28,5 s vs 23,5–23,7 s). Jalur A\* robot 02 juga mendekati zona sampai **5,4–7,8 cm**, di bawah margin rencana 8 cm, tetapi tidak menyentuh zona.

![[Uji nyata Gudang 2026-11-05 S3.png]]

- **S3:** A\* robot 02 di ulangan 2 membuat **mundur-maju** di sekitar (170, 118) sebelum mengitari Z8 (214,5 cm, 18,88 s). Ulangan 1 mulus (182,8 cm, 11,56 s). DQfD mulus di kedua ulangan (180,6 / 192,7 cm).

![[Uji nyata Gudang 2026-11-05 S4.png]]

- **S4:** jalur A\* robot 02 kali ini **mulus** (154–158 cm; di simulasi 190–199 cm dengan mundur-maju), setara DQfD (159–160 cm). Selisih makespan hanya 5%.

### 7.3 Galat pose akhir (robot nyata)

Cara ukur dan uraiannya sama dengan bagian 3.1. Pose akhir adalah rata-rata 1 s pose kamera setelah robot berhenti.

| Skenario | Robot | Hybrid A\* + RS: posisi (ulangan 1 / 2) | Hybrid A\* + RS: arah | DQfD murni: posisi | DQfD murni: arah |
|---|---|---:|---:|---:|---:|
| S1 | 01 | 0,58 / 2,61 cm | 1,48 / 6,43° | 1,17 / 0,08 cm | 0,05 / 0,25° |
| S1 | 02 | 3,84 / 2,36 cm | 6,50 / 4,99° | 2,96 / 0,57 cm | 8,42 / 0,09° |
| S2 | 01 | 1,69 / 2,41 cm | 0,64 / 0,51° | 2,35 / 1,61 cm | 0,12 / 0,14° |
| S2 | 02 | 3,15 / 2,90 cm | 4,90 / 5,86° | 2,74 / 2,22 cm | 5,15 / 5,27° |
| S3 | 01 | 3,62 / 2,96 cm | 9,28 / 8,49° | 3,14 / 3,61 cm | 10,47 / 9,99° |
| S3 | 02 | 2,66 / 2,82 cm | 7,08 / 5,17° | 2,16 / 1,95 cm | 4,56 / 4,50° |
| S4 | 01 | 2,01 / 2,53 cm | 4,23 / 6,90° | 1,22 / 3,21 cm | 5,09 / 9,12° |
| S4 | 02 | 1,95 / 2,04 cm | 4,62 / 4,06° | 2,25 / 2,70 cm | 4,81 / 6,25° |

| Ringkasan (16 pose per perencana) | Hybrid A\* + RS | DQfD murni |
|---|---:|---:|
| Galat posisi: rata-rata / median / maks | 2,51 / 2,57 / 3,84 cm | **2,12** / 2,23 / 3,61 cm |
| Galat arah: rata-rata / median / maks | 5,07 / 5,08 / 9,28° | **4,64** / 4,95 / 10,47° |
| Komponen maju: rata-rata \|maju\| (rata-rata bertanda) | 1,49 (−0,18) cm | 1,29 (+0,05) cm |
| Komponen samping: rata-rata \|samping\| / maks | 1,86 / 2,78 cm | 1,43 / 2,96 cm |
| Galat posisi per skenario, rata-rata S1 / S2 / S3 / S4 | 2,35 / 2,54 / 3,02 / 2,13 cm | 1,19 / 2,23 / 2,72 / 2,34 cm |
| Galat arah per skenario, rata-rata S1 / S2 / S3 / S4 | 4,85 / 2,98 / 7,50 / 4,96° | 2,20 / 2,67 / 7,38 / 6,32° |

| S1–S2 (8 pose per robot, kedua perencana) | Robot 01 | Robot 02 |
|---|---:|---:|
| Galat arah: rata-rata / median | 1,20 / 0,38° | 5,15 / 5,21° |
| Komponen samping: rata-rata \|samping\| | 0,41 cm | 2,13 cm |
| Galat posisi: rata-rata | 1,56 cm | 2,59 cm |

- **DQfD sedikit lebih presisi** (posisi 2,12 vs 2,51 cm, arah 4,64 vs 5,07° rata-rata), tetapi selisihnya kecil dibanding sebaran antar-percobaan. Semua pose akhir ≤ 3,9 cm dari tujuan.
- **Robot 02 punya galat sistematis.** Di S1–S2, robot 01 tiba hampir tepat (arah median 0,4°, samping 0,4 cm), sedangkan robot 02 miring ±5° dan bergeser ±2 cm ke samping. Ini terjadi dengan kedua perencana, kecuali satu percobaan DQfD (0,09°). Di simulasi kedua robot mirip (bagian 3.1), jadi selisih ini sifat robot 02 sendiri. Hasil ini sejalan dengan catatan 09 (ID 02 meleset 5–8° arah hadap).
- **S2 berhenti sebelum tujuan** di robot nyata juga: komponen maju −1,5 sampai −2,4 cm di 8 pose, mirip simulasi (−2,2 sampai −2,8 cm).
- **Galat arah terbesar tetap di S3–S4 robot 01** (rata-rata 8,0°, sampai 10,5°), sama polanya dengan simulasi.

### 7.4 Waktu proses per rencana

| Bagian | Hybrid A\* + RS | DQfD murni | Rasio |
|---|---:|---:|---:|
| **Pencarian jauh:** Hybrid A\* / model jauh | 24,06 ms (98%) | **8,51 ms** (92%) | DQfD 2,8× lebih cepat |
| **Penyelesaian dekat:** RS / model dekat | **0,34 ms** (1%) | 0,64 ms (7%) | RS 1,9× lebih cepat |
| Lain-lain | 0,07 ms | 0,10 ms | |
| **Total (jam dinding), rata-rata** | 24,47 ms | **9,24 ms** | DQfD 2,6× lebih cepat |
| **Total (jam dinding), median** | 19,43 ms | **9,15 ms** | DQfD 2,1× lebih cepat |
| **Total (CPU), rata-rata** | 24,37 ms | **9,20 ms** | DQfD 2,6× lebih cepat |

| Skenario | Hybrid A\* + RS: total | Hybrid A\* / RS | DQfD murni: total | Model jauh / model dekat |
|---|---:|---:|---:|---:|
| S1 | 14,8 ms [10,9–23,3] | 14,39 / 0,35 | **12,0 ms** [8,8–14,6] | 11,48 / 0,40 |
| S2 | 34,3 ms [29,3–41,1] | 33,90 / 0,36 | **7,1 ms** [6,8–7,3] | 6,23 / 0,71 |
| S3 | 32,8 ms [11,9–58,8] | 32,39 / 0,32 | **8,2 ms** [6,1–10,7] | 7,71 / 0,45 |
| S4 | 16,0 ms [11,5–22,9] | 15,57 / 0,31 | **9,7 ms** [7,8–11,0] | 8,61 / 0,99 |

- **DQfD lebih cepat di keempat skenario**, termasuk S1, yang di simulasi sedikit lebih cepat dengan A\*.
- **Rencana terlama tetap A\* robot 02 di S3** (mengitari Z8): 47,9 dan 58,8 ms. DQfD untuk kasus yang sama 10,0–10,7 ms. Rencana terlama itu ≤ 0,6% dari makespan.
- Robot 01 mulai merencana +0,8–2,2 ms setelah permintaan, robot 02 +1,4–3,0 ms.

### 7.5 Simulasi vs robot nyata

| | Simulasi: Hybrid A\* + RS / DQfD | Robot nyata: Hybrid A\* + RS / DQfD |
|---|---|---|
| Makespan S1 | 18,47 / **10,33 s** | 16,36 / **12,05 s** |
| Makespan S2 | 23,99 / 24,00 s | 28,01 / **25,07 s** |
| Makespan S3 | **10,82** / 11,88 s | 15,22 / **11,60 s** |
| Makespan S4 | 17,60 / **10,74 s** | 13,31 / **12,64 s** |
| **Makespan, jumlah** | 70,88 / **56,95 s (−20%)** | 72,90 / **61,36 s (−16%)** |
| Waktu proses, rata-rata | 26,25 / **10,52 ms** | 24,47 / **9,24 ms** |
| Pencarian jauh: Hybrid A\* / model jauh | 25,78 / **9,94 ms** | 24,06 / **8,51 ms** |
| Penyelesaian dekat: RS / model dekat | **0,40** / 0,49 ms | **0,34** / 0,64 ms |
| Galat pose akhir, posisi: rata-rata (maks) | 2,73 (4,25) / **2,53** (4,98) cm | 2,51 (3,84) / **2,12** (3,61) cm |
| Galat pose akhir, arah: rata-rata (maks) | **4,42** (8,96) / 4,51 (9,85)° | 5,07 (9,28) / **4,64** (10,47)° |
| Galat arah S1–S2, robot 01 / robot 02 (rata-rata) | 2,6 / 3,5° | 1,2 / 5,2° |
| Galat awal | 0–3,9 cm | 0,9–3,9 cm |
| Jarak badan terdekat kedua robot | 30,5–71,3 cm | 18,5–51,6 cm |

1. **Waktu proses di robot nyata setara dengan simulasi:** Hybrid A\* −7%, DQfD −12%. Perencanaan per proses memberi angka yang sama, baik di robot nyata maupun di simulasi. Pada uji robot nyata ver-1-4 ([[09 Uji Robot Nyata Gudang ver-1-4 Gundang1|catatan 09]], kode lama dengan perencanaan di thread, skenario lain), DQfD tercatat 184–235 ms saat uji, padahal offline 64–79 ms.
2. **Keunggulan makespan DQfD bertahan di robot nyata (−16%)**, tetapi sebarannya per skenario berbeda dari simulasi. Penyebab utamanya **manuver mundur-maju A\* yang muncul dan hilang** mengikuti pose awal hasil pulang:

   | Skenario | Mundur-maju A\* di simulasi | Mundur-maju A\* di robot nyata |
   |---|---|---|
   | S1 | robot 01 (kedua ulangan), robot 02 (ulangan 1) | robot 02 (ulangan 2) |
   | S3 | – | robot 02 (ulangan 2) |
   | S4 | robot 02 (kedua ulangan) | – |

   Ditentukan dari titik ganti arah pada jalur rencana (`jalur_cm`) yang berjarak > 40 cm dari tujuan. Ganti arah di dekat tujuan adalah koreksi akhir yang ada di kedua perencana, dan tiga ganti arah di S2 adalah putar balik tiga titik yang juga ada di kedua perencana. DQfD tidak pernah membuat mundur-maju di tengah jalan, baik di simulasi maupun di robot nyata.

   Jalur DQfD lebih konsisten. Contohnya S1 robot 01 157,8 cm (simulasi 156,5 cm), S4 robot 02 159–160 cm (simulasi 159–165 cm), dan S3 robot 02 181–193 cm di kedua uji.
3. **S3 berbalik.** Di simulasi A\* lebih cepat (10,82 vs 11,88 s), sedangkan di robot nyata DQfD lebih cepat (11,60 vs 15,22 s), karena satu mundur-maju A\* di ulangan 2. Dengan n = 2, satu percobaan seperti itu menggeser rata-rata sampai 3,7 s.
4. **S2 lebih lambat di robot nyata** (A\* +4,0 s, DQfD +1,1 s) karena robot 02 lebih lambat daripada robot 01 untuk jalur yang hampir sama panjang. Jalur yang sama di simulasi tidak menunjukkan perbedaan antar-robot, jadi ini kemungkinan sifat robot 02. Ini sejalan dengan galat arah akhir robot 02 di S1–S2, yaitu 4,9–8,4° dengan kedua perencana (kecuali satu percobaan 0,1°). Galat robot 01 umumnya ≤ 1,5°. Catatan 09 juga mencatat ID 02 meleset 5–8° arah hadap.
5. **Galat pose akhir di robot nyata setara dengan simulasi** (bagian 3.1 dan 7.3): posisi 2,1–2,5 vs 2,5–2,7 cm, arah 4,6–5,1 vs 4,4–4,5° rata-rata. Polanya juga sama: arah terbesar di S3–S4 robot 01, dan S2 berhenti ±2 cm sebelum tujuan. Bedanya, robot 01 nyata lebih presisi daripada di simulasi (S1–S2: 1,2 vs 2,6°), sedangkan robot 02 nyata kurang presisi (5,2 vs 3,5°).

### 7.6 Kesimpulan uji nyata

1. **DQfD murni lebih cepat di keempat skenario di robot nyata**: S1 −26%, S2 −10%, S3 −24%, S4 −5%, total **−16%**.
2. **Waktu proses DQfD 2,6× lebih cepat rata-rata** (2,1× median), terutama di model jauh vs Hybrid A\* (2,8×). RS tetap lebih cepat daripada model dekat, tetapi keduanya < 1 ms.
3. **Hasil robot nyata konsisten dengan simulasi** untuk waktu proses dan arah keunggulan makespan total. Yang tidak konsisten adalah makespan per skenario, karena A\* kadang membuat manuver mundur-maju dan kadang tidak.

> [!warning] Batasan uji nyata
> - **n = 2 per sel, satu sesi.** Satu manuver mundur-maju A\* menggeser rata-rata skenario sampai 3,7 s. Untuk klaim per skenario dibutuhkan lebih banyak ulangan.
> - **Galat awal 0,9–3,9 cm**, berbeda di setiap percobaan, dan pilihan jalur A\* peka terhadap selisih ini.
> - **Robot 02 lebih lambat dan galat arahnya lebih besar** daripada robot 01 (bagian 7.5), terlepas dari perencananya.
> - **Jarak ke zona sampai 5,4 cm** (A\* S2 robot 02), di bawah margin rencana 8 cm, akibat simpangan pelacakan di robot nyata.

### 7.7 Sumber angka

| Angka | Sumber |
|---|---|
| Semua angka bagian 7 | `Forklift Robot Program/uji_nyata/20261105_010053_Gudang/` (`hasil.jsonl`, `lintasan.csv`, `ringkasan.json`, `lintasan_S1.png` – `lintasan_S4.png`), direkap dengan skrip sementara `analisis_u2.py` |
| Pulang, keterlihatan tag, celah loop | `hasil.jsonl` (kolom `pulang`) dan `lintasan.csv` (kolom `terlihat`, `diramalkan`, `t`), dihitung dengan skrip sementara |
| Galat pose akhir (bagian 3.1 dan 7.3) | kolom `galat_cm`, `galat_deg`, `galat_maju_cm`, `galat_samping_cm` di `hasil.jsonl` kedua run (pose akhir = rata-rata 1 s setelah berhenti, `_galat` di `drl/uji_robot_nyata.py`), direkap dengan skrip sementara |
