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
> - **Tambahan 1,3–2,6 ms di robot 01** berasal dari peta rintangan sementara (badan robot 02), karena robot 01 selalu diberi tujuan lebih dulu (bagian 3.3). Biayanya sama untuk kedua perencana.
> - **n = 1 per sel, simulasi.** Belum bisa dipakai sebagai klaim statistik.
>
> > [!warning] Pembaruan (bagian 5): angka waktu proses di bagian 3 terkontaminasi
> > Kedua robot merencana di thread Python yang bersamaan, sehingga waktu jam dindingnya saling memuat. Pemberian tujuan juga asimetris. Setelah diperbaiki (tujuan simetris, perencanaan bergiliran, waktu CPU thread), run final memberi:
> > - **Waktu proses per rencana:** A\* **24,2 ms** vs DQfD **14,3 ms** (**1,7×**).
> > - **Pencarian jauh:** Hybrid A\* 23,8 ms vs model jauh 13,7 ms (1,7×).
> > - **Penyelesaian dekat:** RS 0,33 ms vs model dekat 0,51 ms.
> > - **Makespan:** S1 19,22 vs **10,34 s** (−46%), S2 23,89 vs 24,30 s.
> > - **Offline** (tanpa loop simulasi): A\* 18,5 ms vs DQfD 8,6 ms (2,1×).
>
> > [!success] Pembaruan (bagian 5.5): perencana per robot di proses terpisah (`--perencana-proses`)
> > - **Makespan sama:** S1 19,19 vs **10,36 s**, S2 24,05 vs 23,98 s.
> > - **Waktu proses bersih** (jam dinding = CPU): A\* **23,3 ms** vs DQfD **11,2 ms** per rencana (**2,1×**, sama dengan offline).
> > - **Pencarian jauh:** Hybrid A\* 22,9 ms vs model jauh **10,6 ms**. **Penyelesaian dekat:** RS 0,35 ms vs model dekat 0,49 ms.
> > - **Robot 02 mulai merencana +1,5–2,0 ms** (thread: +9–39 ms). **Celah loop kendali maks 0,07 s** (thread: 0,21–0,43 s).

---

## 1. Persiapan skenario

### 1.1 Homografi dan peta
- Homografi diulang dengan `python cctv_lurus.py homografi`. Kotaknya tetap 8 × 7 ubin (240 × 210 cm) dengan titik asal yang sama, jadi koordinat zona di `Camera/peta_constraint/Gudang.json` tetap berlaku. Hasilnya ada di `Camera/kalibrasi_lensa.json`.
- Zona peta Gudang **belum** ditempelkan ke garis ubin. Sisi zona masih meleset 0,5–3 cm dari garis ubin 30 cm (Z3, Z2, Z4, Z6, Z7). Lihat bagian 4.

![[Peta Gudang 2026-11-04.png]]

Gambar di atas menampilkan zona larangan (merah), grid ubin 30 cm (cyan), dan lebar lorong. Lorong atas dibatasi Z6 dan Z5 (ujung y = 55 dan 50,5). Lorong bawah dibatasi Z7 dan Z8 (ujung y = 144,5 dan 134). Ruang tengah, y ≈ 55–134, terbuka selebar arena.

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

### 1.2.1 Skenario dan heatmap Dijkstra

Panel kiri menampilkan pose awal (titik-titik) dan tujuan (garis putus) dengan panah arah hadap. Biru adalah robot 01, merah robot 02. Panel tengah dan kanan adalah **heuristik Dijkstra** ke tujuan setiap robot, yaitu biaya-sisa holonomik dalam cm. Peta ini dihitung oleh `_holonomic_heuristic` (Dijkstra 8-tetangga, sel 5 cm, sel diblokir bila jarak bebasnya < setengah lebar badan). Fungsi yang sama dipakai Hybrid A\* sebagai heuristik dan lingkungan DQfD sebagai peta Dijkstra observasi (`drl/lingkungan.py:430`), jadi satu peta per robot berlaku untuk **kedua** perencana.

Kondisinya disamakan dengan uji sesudah perbaikan di bagian 5 (tujuan simetris, commit `8c2d1fd`): **kedua robot merencana tanpa badan robot lain sebagai rintangan**. Pada versi gambar sebelumnya, peta robot 01 masih memuat badan robot 02 di pose awalnya sebagai rintangan. Itu kondisi run asimetris (bagian 3.3), dan gambarnya bisa dibuat ulang dengan `gambar_peta.py --asimetris`. Biaya Dijkstra di titik awal tidak berubah karena badan itu tidak berada di rute terpendek. Yang berubah hanya sel di sekitar A2 yang sebelumnya tertutup.

![[Skenario Gudang 2026-11-04 S1.png]]

![[Skenario Gudang 2026-11-04 S2.png]]

Panjang jalur dari run final (`20261104_232645_Gudang_sim`):

| Skenario | Robot | Biaya Dijkstra di titik awal | Jalur Hybrid A\* | Jalur DQfD murni |
|---|---|---:|---:|---:|
| S1 | 01 | 156 cm | 223,0 cm | 156,4 cm |
| S1 | 02 | 162 cm | 221,4 cm | 170,6 cm |
| S2 | 01 | 140 cm | 192,0 cm | 192,0 cm |
| S2 | 02 | 135 cm | 194,7 cm | 194,7 cm |

- **Dijkstra mengabaikan arah hadap dan jari-jari belok**, jadi nilainya batas bawah kasar untuk panjang jalur.
- Di **S1**, jalur DQfD hanya 0–5% di atas biaya Dijkstra. Jalur robot 01 praktis sama dengan batas bawahnya. Jalur Hybrid A\* 37–43% di atasnya, karena manuver mundur-maju di ruang tengah.
- Di **S2**, Dijkstra hanya "melihat" garis lurus ke bawah (135–140 cm). Putar balik 180° menambah 52–60 cm, dan biaya itu sama untuk kedua perencana. Karena itu S2 tidak membedakan perencana.

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

> [!warning] Angka di bagian 3.1–3.2 terkontaminasi
> Ini run asimetris dengan dua perencana berjalan bersamaan. Waktu jam dinding satu robot ikut memuat pekerjaan robot lain. Angka yang dipakai adalah bagian 5.

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
- **"Lain" 1,3–2,6 ms selalu muncul pada robot 01**, untuk kedua perencana. Pada robot 02 hanya ≤ 0,06 ms. Penyebabnya ada di bagian 3.3.
- **Dampak ke makespan bisa diabaikan.** Rencana terlama 42 ms, < 0,2% dari makespan 11–24 s.

### 3.3 Mengapa robot 01 punya tambahan "lain" 1,3–2,6 ms

**Penyebab: robot 01 merencana dengan badan robot 02 sebagai rintangan, robot 02 tidak.** Peta rintangan sementara itu dibangun di dalam `rencanakan()`, di luar perencana, sehingga waktunya masuk ke "lain".

Urutan kejadian di awal setiap percobaan (`drl/uji_robot_nyata.py`, `_mulai_uji`):
1. Semua robot di-`batalkan()`: `tujuan = None`, state SIAP.
2. Robot 01 menerima tujuan, dan `terima_tujuan()` langsung memanggil `minta_rencana()` (`navigasi.py:265`). Saat itu robot 02 belum punya tujuan, jadi `diam = True` (`navigasi.py:301`). `rintangan_untuk()` pun memasukkan badan robot 02.
3. Di `Perencana.rencanakan()` (`rencana.py:263`), `peta_dengan(rintangan)` menyalin occupancy grid, menggambar poligon badan robot 02, lalu membangun `ObstacleMap` baru (distance transform 270 × 300 sel). Langkah ini tidak dihitung sebagai bagian perencana, jadi masuk ke `lain = total − Σ rincian`.
4. Robot 02 menerima tujuan. Robot 01 sudah MERENCANA, jadi tidak diam dan tidak dijadikan rintangan. `peta_dengan(None)` langsung mengembalikan peta arena tanpa biaya, sehingga "lain" ≈ 0.

Pengukuran offline (Windows, median 7 ulangan, rencana robot 01):

| Perencana | Skenario | "lain" dengan badan robot 02 | "lain" tanpa rintangan |
|---|---|---:|---:|
| Hybrid A\* + RS | S1 | 0,56 ms | 0,01 ms |
| Hybrid A\* + RS | S2 | 0,62 ms | 0,02 ms |
| DQfD murni | S1 | 0,39 ms | 0,01 ms |
| DQfD murni | S2 | 0,43 ms | 0,02 ms |

- `peta_dengan()` sendiri: median **0,57–0,62 ms** offline, dan **0,0001 ms** tanpa rintangan.
- Di simulasi angkanya 1,3–2,6 ms, sekitar 2–4× angka offline. Rasionya serupa dengan bagian perencana (A\* S1 robot 01: 24,4 ms di simulasi vs 10,8 ms offline), karena perencana berjalan di thread belakang dan berbagi CPU dengan loop simulasi 20 Hz.
- Biaya ini **sama untuk kedua perencana**, jadi tidak membuat perbandingan berat sebelah. Tetapi ada **asimetri antar-robot**: hanya jalur robot 01 yang menghindari pose awal robot 02, sedangkan jalur robot 02 tidak menghindari robot 01. Yang mencegah tabrakan adalah wasit, sesuai rancangan di [[05 Multi Robot Navigation and Control|Multi Robot Navigation and Control]].
- Rintangan itu juga sedikit menambah waktu **di dalam** perencana. Offline, DQfD S1 robot 01 butuh 11,4 ms dengan rintangan vs 8,4 ms tanpa rintangan. A\* hanya bertambah 0,4–0,5 ms.

## 4. Kesimpulan dan langkah berikutnya

1. **Keunggulan makespan DQfD murni sepenuhnya dari S1**, karena jalurnya lebih pendek dan tanpa ganti arah. S2 (putar balik di lorong sendiri) tidak membedakan perencana.
2. **Waktu proses DQfD murni unggul dengan konsisten, terutama di model jauh vs Hybrid A\*.** Di bagian dekat, RS sedikit lebih cepat, tetapi tidak berpengaruh ke total.
3. Langkah berikutnya:
   - Ulangi dengan **`--ulang 5`**, di simulasi dengan derau dan di robot nyata, untuk mendapat sebaran dan uji signifikansi.
   - Tambah skenario yang **memaksa pilihan rute**, misalnya pindah lorong lewat celah di ujung Z5–Z8. Ini lebih mungkin membedakan makespan daripada putar balik di tempat.
   - Rapikan zona peta Gudang ke garis ubin (bagian 1.1), lalu periksa ulang tengah lorong dan semua pose skenario.
   - ~~Simetriskan urutan pemberian tujuan~~: sudah dikerjakan, lihat bagian 5.
   - ~~Prototipe perencana multi-proses per robot~~: sudah dipasang (`--perencana-proses`), lihat bagian 5.5. Berikutnya: uji di robot nyata, dan jadikan bawaan bila hasilnya sama.

> [!warning] Batasan
> - **n = 1 per sel (skenario × perencana), simulasi tanpa derau.** Tidak ada sebaran, jadi belum ada uji statistik.
> - **Skenario berbeda dari catatan 15 dan 17**, sehingga angkanya tidak bisa dibandingkan langsung. Catatan 17 juga memakai `drl.uji_navigasi` dengan derau 1,2 cm, sedangkan catatan ini memakai protokol `drl.uji_robot_nyata` (pulang → uji).
> - **Waktu rencana dicatat saat simulasi berjalan.** Angkanya lebih besar daripada pengukuran offline, misalnya A\* 24,5 ms di catatan 17.
> - **Uji robot nyata dengan skenario ini belum dilakukan.**

## 5. Perbaikan pengukuran: tujuan simetris, perencanaan bergiliran, waktu CPU thread

### 5.1 Masalah yang ditemukan
1. **Pemberian tujuan asimetris** (bagian 3.3). Hanya robot 01 yang merencana dengan badan robot 02 sebagai rintangan.
2. **Dua perencana berjalan bersamaan.** Setiap robot merencana di thread Python sendiri (`navigasi.py`, `threading.Thread(target=kerja)`), dan thread berbagi **GIL**. Setelah tujuan disimetriskan, kedua robot mulai tepat bersamaan. Waktu jam dinding tiap robot hampir sama dengan **jumlah** kedua rencana offline:

| Rencana | Run asimetris | Run simetris, bersamaan | Offline (median 9×) |
|---|---|---|---|
| A\* S1 robot 01 / 02 | 27,4 / 20,2 ms | 15,0 / 12,1 ms | 10,2 / 11,0 ms |
| A\* S2 robot 01 / 02 | 34,9 / 42,0 ms | 58,6 / 51,8 ms | 29,3 / 23,4 ms |
| DQfD S1 robot 01 / 02 | 12,5 / 16,2 ms | 37,3 / 29,0 ms | 8,6 / 15,4 ms |
| DQfD S2 robot 01 / 02 | 11,3 / 7,3 ms | 11,8 / 11,4 ms | 5,4 / 5,1 ms |

Contoh: DQfD S1 offline 8,6 + 15,4 = 24 ms, sedangkan di simulasi tercatat 29–37 ms **per robot**.

### 5.2 Perubahan kode
| Berkas | Perubahan |
|---|---|
| `navigasi.py` | `terima_tujuan(baru, tunda=True)`: tujuan diterima, perencanaan ditunda (`perlu_rencana`). `diam`: robot dengan perencanaan tertunda tidak dihitung diam. `batalkan()` mereset `perlu_rencana`. `KUNCI_RENCANA`: satu perencanaan pada satu waktu untuk semua robot, sedangkan daftar rintangan tetap diambil sebelum menunggu giliran |
| `rencana.py` | `jam_cpu_thread()`: waktu CPU thread pemanggil. `rencanakan()` mengembalikan `cpu_s` |
| `drl/uji_robot_nyata.py` | `_mulai_uji`: semua tujuan ditetapkan dulu (`tunda=True`), baru `minta_rencana()` untuk setiap robot. `hasil.jsonl` mencatat `cpu_ms` dan `mulai_ms` (jeda dari permintaan rencana pertama sampai rencana ini mulai) |

- **Waktu CPU di Windows.** `time.thread_time()` memakai `GetThreadTimes`, yang hanya naik per **15,625 ms**: terukur 0; 15,625; 31,25; … ms. Itu lebih kasar daripada satu rencana. Karena itu dipakai `QueryThreadCycleTime` (siklus CPU thread), dikalibrasi sekali ke detik terhadap `perf_counter` (20 ms saat modul dimuat, ±3,79 GHz di laptop ini). Di sistem lain tetap `time.thread_time()`.
- **Validasi.** Dua thread berebut GIL: jam dinding 71–79 ms, CPU 61–64 ms, sedangkan sendirian 58 ms. Offline tanpa perebutan: CPU = jam dinding, selisih < 0,1 ms.

### 5.3 Hasil run final (`uji_nyata/20261104_232645_Gudang_sim`)

Semua robot tiba, tanpa estop, tanpa menunggu.

| Skenario | Hybrid A\* + RS | DQfD murni | Selisih |
|---|---:|---:|---:|
| S1 | 19,22 s | **10,34 s** | **−8,88 s (−46%)** |
| S2 | 23,89 s | 24,30 s | +0,41 s (+1,7%) |
| **Jumlah** | 43,11 s | **34,64 s** | **−8,47 s (−20%)** |

| Rencana | Mulai | Jam dinding | CPU thread | Bagian jauh | Bagian dekat |
|---|---:|---:|---:|---:|---:|
| A\* S1 robot 01 / 02 | +1,1 / +14,9 ms | 13,3 / 12,1 ms | 12,0 / 12,0 ms | 12,75 / 11,71 | 0,47 / 0,26 |
| A\* S2 robot 01 / 02 | +0,9 / +39,0 ms | 37,8 / 33,7 ms | 36,0 / 32,8 ms | 37,47 / 33,29 | 0,27 / 0,31 |
| DQfD S1 robot 01 / 02 | +0,9 / +25,2 ms | 24,1 / 18,3 ms | 21,0 / 17,8 ms | 23,49 / 17,81 | 0,53 / 0,41 |
| DQfD S2 robot 01 / 02 | +0,9 / +9,1 ms | 7,9 / 6,9 ms | 6,8 / 6,4 ms | 7,32 / 6,14 | 0,54 / 0,54 |

| Bagian (rata-rata 4 rencana) | Hybrid A\* + RS | DQfD murni | Rasio |
|---|---:|---:|---:|
| Pencarian jauh: Hybrid A\* / model jauh | 23,8 ms | **13,7 ms** | DQfD 1,7× lebih cepat |
| Penyelesaian dekat: RS / model dekat | **0,33 ms** | 0,51 ms | RS 1,5× lebih cepat |
| Lain-lain | 0,07 ms | 0,11 ms | |
| **Total (jam dinding)** | 24,2 ms | **14,3 ms** | DQfD 1,7× lebih cepat |
| **Total (CPU thread)** | 23,2 ms | **13,0 ms** | DQfD 1,8× lebih cepat |
| Offline, kondisi simetris (median 9×) | 18,5 ms | **8,6 ms** | DQfD 2,1× lebih cepat |

Pengamatan:
- **Tumpang tindih hilang.** Robot 02 mulai tepat setelah robot 01 selesai, dan jam dinding ≈ CPU (selisih 0,1–3,1 ms). Jeda mulai robot 02 (9–39 ms) kurang dari 0,4% dari makespan.
- **"Lain" ≤ 0,22 ms untuk kedua robot.** Asimetri di bagian 3.3 sudah hilang.
- **DQfD S1 di simulasi (18–24 ms) lebih lama daripada offline (8,6 / 15,4 ms).** CPU-nya juga 18–21 ms, jadi ini pekerjaan sungguhan, bukan perebutan GIL. Di simulasi robot mulai dari pose hasil pulang yang meleset 1,5–3 cm dari titik awal, dan DQfD menempuh pencarian yang berbeda: jalur robot 01 156 cm, sedangkan offline dari pose persis 177 cm. **Waktu dan jalur DQfD peka terhadap pose awal**, sehingga n = 1 per skenario belum stabil.
- Simulasi tetap lebih lambat daripada offline (A\* +31%, DQfD +66%) karena loop simulasi 20 Hz ikut memakai GIL.

### 5.4 Temuan: loop kendali terlambat saat rencana A\* panjang
Diukur offline (Python 3.14.3, Windows): loop utama berdetak 20 Hz seperti `navigasi.py`, sementara satu thread merencana.

| Rencana | Durasi | Keterlambatan detak (maks / median) |
|---|---:|---:|
| A\* S2 robot 01 | 30 ms | 0,6 / 0,6 ms |
| A\* (195,75; 26; −90) → (32; 170; −90) | 503–642 ms | **16,5–28,5 / 7,4–13,9 ms** |
| DQfD, ketiga kasus | 6–14 ms | ≤ 0,7 ms |

| Interval GIL (`sys.setswitchinterval`) | Beban loop | Keterlambatan maks / median |
|---|---|---:|
| 5 ms (bawaan) | 0 ms | 28,5 / 13,9 ms |
| 5 ms (bawaan) | 10 ms | 23,5 / 12,7 ms |
| 1 ms | 0 ms | 16,7 / 7,4 ms |
| 1 ms | 10 ms | 16,5 / 8,4 ms |

- Loop kendali **tidak berhenti total**, tetapi selama rencana A\* yang panjang setiap detak terlambat 7–28 ms. Itu masih di bawah satu periode kendali (50 ms).
- Hal yang sama membuat `Thread.start()` kadang baru kembali setelah rencana pendek selesai: 12,2 ms dari 12,6 ms. Karena itu `mulai_ms` diukur dari permintaan rencana pertama, bukan dari saat menunggu kunci.
- **Tambah thread tidak menolong**, karena GIL hanya mengizinkan satu thread menjalankan Python pada satu waktu. Jalan keluar yang benar-benar paralel:
  1. **Multi-proses**: satu proses perencana per robot, sehingga loop kendali tidak terganggu.
  2. **Python free-threaded** (3.14t): dukungan numba dan OpenCV perlu dicek dulu.
  3. Melepas GIL di bagian berat (numba `nogil=True`), tetapi Hybrid A\* sebagian besar Python murni.
- Untuk pengukuran makespan dan waktu proses, perencanaan bergiliran dan waktu CPU sudah cukup. Multi-proses lebih bernilai untuk keselamatan robot nyata dan rencana ulang yang lama, misalnya kasus 1,3–1,8 s di [[17 Forward Numpy ver-1-5 dan Perbandingan Waktu Proses serta Makespan|catatan 17]].
- `drl/uji_navigasi.py` (simulasi catatan 15 dan 17) masih memberi tujuan berurutan. Hasil catatan 15 dan 17 perlu dibaca dengan asimetri ini.

### 5.5 Perencana per robot di proses terpisah (`--perencana-proses`)

Commit `e981f0d`. Bawaan tetap thread. Diaktifkan dengan:
```powershell
python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang.json --simulasi --ulang 1 --perencana-proses
python navigasi.py --id 1,2 --perencana dqfd-murni --perencana-proses
```

**Rancangan** (`perencana_proses.py`):
- **`PerencanaProses`**: satu proses anak (*spawn*) per robot. Proses itu membuat `Arena` dan `Perencana` sendiri dari nama peta dan nomor robot, untuk semua jenis yang diuji. Yang dikirim lewat pipa hanya pose, poligon rintangan, dan hasil (jalur + info).
- **`PerencanaDiProses`**: pengganti `rencana.Perencana`. `rencanakan()` dikerjakan proses anak. Kueri lain (`margin_untuk_pose`, `peta_dengan`, `pose_bebas`, `_untuk`, …) dijawab `Perencana` lokal yang identik.
- **`navigasi.py` hampir tidak berubah.** Thread pekerja tetap memanggil `self.per.rencanakan()`, tetapi kini hanya menunggu pipa (`recv()`). Thread yang menunggu I/O tidak memegang GIL, jadi loop kendali bebas. `KUNCI_RENCANA` hanya dipakai di mode thread.
- **Dua jebakan Windows yang dihindari:**
  - `multiprocessing.connection.wait()` dan `poll(batas)` memakai timer ±15,6 ms dan bisa tidur jauh melewati batasnya. Karena itu dipakai `recv()` (bangun tepat saat data tiba), atau `poll(0)` ditambah `sleep` 1 ms.
  - `jam_cpu_thread()` dulu memakai satu penampung `c_ulonglong` bersama, sehingga dua thread yang bersamaan bisa membaca siklus thread lain (terukur CPU 11 ms untuk rencana 26 ms). Sekarang setiap panggilan memakai penampung baru. Angka bagian 5.3 tidak terpengaruh, karena perencanaannya bergiliran.

**Pemeriksaan:** dari pose awal persis, jalur di proses anak **identik** dengan perencana lokal, 8/8 (S1, S2 × robot 01/02 × A\*/DQfD). IPC 0,6–1,0 ms per rencana. Dua proses siap dalam 1,10–1,13 s, dimuat paralel.

**Banding tanpa robot** (`python perencana_proses.py --banding`, loop 20 Hz dengan beban 5 ms/detak, median 5 ulangan):

| Kasus | Ukuran | Thread bersamaan | Thread bergiliran | **Proses per robot** |
|---|---|---:|---:|---:|
| S1 A\* | kedua robot selesai | 21,5 ms | 19,0 ms | **14,0 ms** |
| S2 A\* | kedua robot selesai | 62,4 ms | 58,9 ms | **33,4 ms** |
| S1 DQfD | kedua robot selesai | 32,5 ms | 29,9 ms | **17,8 ms** |
| S2 DQfD | kedua robot selesai | 12,4 ms | 13,8 ms | **9,2 ms** |
| A\* panjang | rencana robot 01 | 630 ms | 568 ms | **518 ms** |
| A\* panjang | robot 02 (rencana 34–87 ms) menerima hasil setelah | 104 ms | 606 ms | **37 ms** |
| A\* panjang | keterlambatan loop maks / median | 29,0 / 11,3 ms | 21,3 / 11,1 ms | **1,1 / 0,6 ms** |

**Simulasi Gudang** (`--ulang 1`): thread bergiliran `20261104_232645_Gudang_sim` vs proses `20261104_234613_Gudang_sim`.

| | Thread bergiliran | **Proses per robot** |
|---|---:|---:|
| Makespan S1: A\* / DQfD | 19,22 / 10,34 s | 19,19 / 10,36 s |
| Makespan S2: A\* / DQfD | 23,89 / 24,30 s | 24,05 / 23,98 s |
| Rencana A\* rata-rata (jam / CPU) | 24,2 / 23,2 ms | **23,3 / 23,3 ms** |
| Rencana DQfD rata-rata (jam / CPU) | 14,3 / 13,0 ms | **11,2 / 11,2 ms** |
| Bagian jauh: Hybrid A\* / model jauh | 23,8 / 13,7 ms | 22,9 / **10,6 ms** |
| Bagian dekat: RS / model dekat | 0,33 / 0,51 ms | 0,35 / 0,49 ms |
| Rasio waktu proses A\* / DQfD | 1,7× | **2,1×** (offline 2,1×) |
| Robot 02 mulai merencana | +9 sampai +39 ms | **+1,5 sampai +2,0 ms** |
| Celah loop terbesar sepanjang run | 0,43 s; ±0,2 s di awal setiap run thread | **0,07 s** |

| Rencana (proses, ms) | Hybrid A\* | RS | Model jauh | Model dekat | Total |
|---|---:|---:|---:|---:|---:|
| S1 robot 01 / 02 | 13,14 / 10,47 | 0,50 / 0,33 | – | – | 13,71 / 10,86 |
| S2 robot 01 / 02 | 35,39 / 32,68 | 0,25 / 0,33 | – | – | 35,72 / 33,10 |
| S1 robot 01 / 02 | – | – | 17,12 / 13,10 | 0,53 / 0,48 | 17,74 / 13,68 |
| S2 robot 01 / 02 | – | – | 7,27 / 5,00 | 0,42 / 0,54 | 7,77 / 5,65 |

Pengamatan:
- **Makespan tidak berubah** (selisih ≤ 0,3 s), karena rencana di skenario ini hanya ≤ 40 ms. Keunggulan DQfD di S1 tetap: **−46%**.
- **Waktu proses sekarang bersih.** Jam dinding = CPU, dan DQfD turun 14,3 → 11,2 ms karena tidak lagi berbagi GIL dengan loop simulasi. Rasio 2,1× sama dengan pengukuran offline, jadi angka simulasi dan offline kini konsisten.
- **Loop kendali mulus.** Celah ±0,2 s yang selalu muncul di awal percobaan pertama mode thread juga hilang.
- **Satu run proses sebelumnya (`20261104_234155_Gudang_sim`) dibuang.** Loop berhenti 2,22 s di tengah S1 DQfD (+7,7 s, saat tidak ada rencana berjalan) dan dua kali saat pulang. Akibatnya makespan DQfD S1 menjadi 12,46 s. Run ulang dengan jendela tidak disentuh tidak menunjukkan celah > 0,07 s. Celah sebesar itu juga pernah muncul di mode thread (4,71 s dan 1,07 s di run awal). Kemungkinan penyebabnya interaksi dengan jendela OpenCV atau sistem, bukan perencana.
- **Biaya:** persiapan +1,1 s sekali per sesi, memori satu salinan peta dan model per robot, dan IPC ±1 ms per rencana. Bila jumlah robot melebihi jumlah inti CPU, proses akan kembali berebut CPU.

## 6. Sumber angka

| Angka | Sumber |
|---|---|
| Skenario | `Forklift Robot Program/uji_nyata/Gudang.json`, salinannya di folder hasil (`skenario.json`) |
| Bagian 1.2–1.3 | skrip sementara di luar repo: `lurus_tengah.py` (yaw dan tengah lorong), `ubah_y.py` dan `cari_y.py` (batas y awal), `cek_pulang.py` (jalur pulang), `balik_s2.py` (arah tujuan S2) |
| Bagian 1.2.1 (gambar peta, skenario, Dijkstra) | skrip sementara `gambar_peta.py` (tanpa opsi = kondisi simetris; `--asimetris` = kondisi lama): `HybridAStarPlanner._holonomic_heuristic` pada `Perencana.peta_dengan()`; panjang jalur dari `20261104_232645_Gudang_sim` |
| Bagian 2–3.2 | `Forklift Robot Program/uji_nyata/20261104_224635_Gudang_sim/` (`hasil.jsonl`, `ringkasan.json`, `rincian_tabel.tex`, `lintasan_S1.png`, `lintasan_S2.png`), direkap dengan skrip sementara `rekap.py` |
| Bagian 3.3 | skrip sementara `ukur_peta.py` (`peta_dengan`, 50 ulangan) dan `ukur_lain.py` (`rencanakan` robot 01 dengan/tanpa rintangan, 7 ulangan), Windows, offline |
| Bagian 5.1 | run asimetris `20261104_224635_Gudang_sim`, run simetris tanpa kunci `20261104_230522_Gudang_sim`, offline `offline_simetris.py` (median 9 ulangan) |
| Bagian 5.2 | `uji_siklus.py` (resolusi `thread_time`, kalibrasi siklus, uji dua thread) |
| Bagian 5.3 | run final `20261104_232645_Gudang_sim` (kode di commit bagian 5.2), direkap `rekap.py` |
| Bagian 5.4 | `uji_gil.py` (`Thread.start()` dan celah loop) dan `uji_loop.py`, `uji_loop2.py` (detak 20 Hz, interval GIL 5/1 ms, beban loop 0/10 ms, 3 ulangan) |
| Bagian 5.5 | commit `e981f0d`. Pemeriksaan jalur: skrip sementara `cek_proxy.py`. Banding: `python perencana_proses.py --banding --ulang 5`. Simulasi: `20261104_232645_Gudang_sim` (thread) dan `20261104_234613_Gudang_sim` (proses), direkap `rekap.py`; celah loop dari `lintasan.csv`; run dibuang `20261104_234155_Gudang_sim` |
