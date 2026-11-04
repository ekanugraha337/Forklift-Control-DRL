# Skenario Gudang baru (homografi ulang): simulasi makespan dan waktu proses Hybrid A\* + RS vs DQfD murni

Tanggal: 2026-11-04 · Branch `ver-1-5`, kode commit `4d26e77` · Lanjutan dari [[17 Forward Numpy ver-1-5 dan Perbandingan Waktu Proses serta Makespan|Forward Numpy ver-1-5 dan Perbandingan Waktu Proses serta Makespan]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Homografi kamera diulang, lalu skenario uji Gudang dibuat dari awal dengan `drl.uji_robot_nyata --buat`. Skenario lama dihapus di commit `1f029d3`. Catatan ini merekam bentuk skenario baru dan hasil simulasinya (`--simulasi --ulang 1`, dua run), dengan perbandingan **makespan** dan **waktu proses** yang dipecah menjadi Hybrid A\*, RS, model jauh, dan model dekat.

Semua angka diukur dengan metode di bagian 2: tujuan kedua robot diberikan bersamaan, setiap robot merencana di **proses** sendiri, dan waktu dicatat sebagai jam dinding dan CPU.

> [!summary] Ringkasan (run `20261104_235558_Gudang_sim`, diulang di `20261104_234613_Gudang_sim`)
> - **Makespan:** di S1, DQfD murni **10,31 s** vs Hybrid A\* + RS **19,26 s** (**−46%**). Di S2 keduanya setara (24,01 vs 23,96 s). Jumlah kedua skenario: 34,32 vs 43,22 s (**−21%**).
> - **Sumber selisih makespan adalah bentuk jalur.** Di S1, DQfD menempuh lengkung S 157/171 cm (robot 01 tanpa ganti arah), hanya 0–5% di atas biaya Dijkstra. A\* membuat manuver mundur-maju sepanjang 223/221 cm. Di S2 kedua perencana membuat putar balik tiga titik yang sama.
> - **Waktu proses per rencana** (rata-rata dua run, 8 rencana): DQfD **12,4 ms** vs A\* **23,3 ms** (**1,9×** lebih cepat). Jam dinding = CPU (selisih ≤ 0,33 ms).
>   - Pencarian jauh: model jauh **11,8 ms** vs Hybrid A\* **22,9 ms** (1,9×).
>   - Penyelesaian dekat: RS **0,36 ms** vs model dekat 0,52 ms (RS 1,4× lebih cepat, tetapi keduanya < 1 ms).
> - **Waktu proses ≤ 0,2% dari makespan.** Makespan ditentukan oleh jalur, bukan oleh komputasi.
> - **Kedua robot merencana paralel.** Robot 02 mulai +1,6–2,0 ms setelah permintaan. Celah loop kendali terbesar 0,07 s.
> - **n = 1 per sel per run, dua run, simulasi tanpa derau.** Belum bisa dipakai sebagai klaim statistik.

---

## 1. Persiapan skenario

### 1.1 Homografi dan peta
- Homografi diulang dengan `python cctv_lurus.py homografi`. Kotaknya tetap 8 × 7 ubin (240 × 210 cm) dengan titik asal yang sama, jadi koordinat zona di `Camera/peta_constraint/Gudang.json` tetap berlaku. Hasilnya ada di `Camera/kalibrasi_lensa.json`.
- Zona peta Gudang **belum** ditempelkan ke garis ubin. Sisi zona masih meleset 0,5–3 cm dari garis ubin 30 cm (Z3, Z2, Z4, Z6, Z7). Lihat bagian 5.

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
3. **y awal S1 dinaikkan ke 40,0** (sebelumnya 34,5 dan 31,1). Lihat bagian 1.4.
4. **Arah tujuan S2 dibalik** dari −90° ke 90°, supaya skenario lebih menantang: robot harus putar balik di lorong.

Setiap pose diperiksa **sebagai tujuan** untuk kedua perencana, dan semuanya muat dengan margin 8 cm. Jalur setiap robot juga direncanakan dengan kedua perencana.

### 1.3 Skenario dan heatmap Dijkstra

Panel kiri menampilkan pose awal (titik-titik) dan tujuan (garis putus) dengan panah arah hadap. Biru adalah robot 01, merah robot 02. Panel tengah dan kanan adalah **heuristik Dijkstra** ke tujuan setiap robot, yaitu biaya-sisa holonomik dalam cm. Peta ini dihitung oleh `_holonomic_heuristic` (Dijkstra 8-tetangga, sel 5 cm, sel diblokir bila jarak bebasnya < setengah lebar badan). Fungsi yang sama dipakai Hybrid A\* sebagai heuristik dan lingkungan DQfD sebagai peta Dijkstra observasi (`drl/lingkungan.py:430`), jadi satu peta per robot berlaku untuk **kedua** perencana. Kondisinya sama dengan saat uji: kedua robot merencana tanpa badan robot lain sebagai rintangan.

![[Skenario Gudang 2026-11-04 S1.png]]

![[Skenario Gudang 2026-11-04 S2.png]]

Panjang jalur dari run `20261104_235558_Gudang_sim`:

| Skenario | Robot | Biaya Dijkstra di titik awal | Jalur Hybrid A\* | Jalur DQfD murni |
|---|---|---:|---:|---:|
| S1 | 01 | 156 cm | 223,0 cm | 156,6 cm |
| S1 | 02 | 162 cm | 221,4 cm | 170,6 cm |
| S2 | 01 | 140 cm | 192,0 cm | 192,1 cm |
| S2 | 02 | 135 cm | 194,7 cm | 194,7 cm |

- **Dijkstra mengabaikan arah hadap dan jari-jari belok**, jadi nilainya batas bawah kasar untuk panjang jalur.
- Di **S1**, jalur DQfD hanya 0–5% di atas biaya Dijkstra. Jalur robot 01 praktis sama dengan batas bawahnya. Jalur Hybrid A\* 37–43% di atasnya, karena manuver mundur-maju di ruang tengah.
- Di **S2**, Dijkstra hanya "melihat" garis lurus ke bawah (135–140 cm). Putar balik 180° menambah 52–60 cm, dan biaya itu sama untuk kedua perencana. Karena itu S2 tidak membedakan perencana.

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
python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang.json --simulasi --ulang 1
```

- **Protokol** (`drl/uji_robot_nyata.py`): sebelum setiap percobaan, robot pulang ke pose awal dengan Hybrid A\* (tidak diukur, toleransi 5 cm / 10°). Lalu kedua robot diberi tujuannya dengan perencana yang diuji, bersamaan. Urutan perencana diselang-seling per ulangan. Simulasi tanpa derau.
- **Tujuan simetris.** Tujuan semua robot ditetapkan dulu (`terima_tujuan(tunda=True)`), baru setiap robot diminta merencana. Tidak ada robot yang merencana dengan pose awal robot lain sebagai rintangan.
- **Satu proses perencana per robot** (`perencana_proses.py`, commit `4d26e77`). Setiap robot merencana di proses dan interpreter Python sendiri, sehingga tidak berbagi GIL dengan loop kendali atau dengan perencana robot lain. Perencanaan di thread sudah dihapus, karena waktu jam dinding yang tercatat ikut memuat pekerjaan lain. `navigasi.Unit.minta_rencana()` menolak perencana yang bukan `PerencanaDiProses`.
- **Yang dicatat untuk setiap rencana** (`hasil.jsonl`):
  - `ms`: waktu jam dinding `rencanakan()` di proses perencana.
  - `cpu_ms`: waktu CPU thread perencana. Di Windows memakai `QueryThreadCycleTime`, karena `time.thread_time()` di sana hanya naik per 15,625 ms.
  - `rincian_ms`: bagian Hybrid A\*, RS, model jauh, model dekat, dan lain-lain.
  - `mulai_ms`: jeda dari permintaan rencana sampai rencana mulai.
  - `ipc_ms`: biaya antar-proses.
  - `pid`: proses yang merencana.
- **Bukti hanya proses yang merencana:** di run `235558`, setiap rencana robot 01 dikerjakan pid 51608 dan robot 02 pid 46904. Keduanya proses anak, bukan proses utama.

## 3. Makespan

Run `20261104_235558_Gudang_sim`. Semua robot tiba, tanpa estop, tanpa menunggu, dan tanpa rencana ulang.

| Skenario | Hybrid A\* + RS | DQfD murni | Selisih |
|---|---:|---:|---:|
| S1 | 19,26 s | **10,31 s** | **−8,95 s (−46%)** |
| S2 | 23,96 s | 24,01 s | +0,05 s (±0%) |
| **Jumlah** | 43,22 s | **34,32 s** | **−8,90 s (−21%)** |

| | Hybrid A\* + RS | DQfD murni |
|---|---|---|
| S1 panjang jalur, robot 01 / 02 | 223,0 / 221,4 cm | **156,6 / 170,6 cm** |
| S1 ganti arah | 2 / 2 | **0** / 2 |
| S1 waktu tempuh | 19,26 / 19,13 s | 10,31 / 10,18 s |
| S2 panjang jalur | 192,0 / 194,7 cm | 192,1 / 194,7 cm |
| S2 ganti arah | 3 / 3 | 3 / 3 |
| S2 waktu tempuh | 23,38 / 23,96 s | 23,43 / 24,01 s |
| Galat akhir | 2,4–3,0 cm, 2,7–3,2° | 1,4–2,7 cm, 1,9–4,2° |
| Jarak terdekat ke zona | 9,9–14,8 cm | 11,7–18,2 cm |

![[Simulasi Gudang 2026-11-04 S1.png]]

- **S1:** Hybrid A\* memutar dengan manuver mundur-maju di ruang tengah (y ≈ 115–125) sebelum masuk ke T. DQfD langsung melengkung dari A ke T, dengan jalur 51–66 cm lebih pendek.

![[Simulasi Gudang 2026-11-04 S2.png]]

- **S2:** kedua perencana membuat putar balik tiga titik yang sama: mundur lurus, berbelok ke kiri di y ≈ 110–120, lalu maju masuk ke T menghadap bawah. Panjang jalurnya berbeda ≤ 0,1 cm, sehingga S2 **tidak membedakan** perencana dari sisi makespan.

## 4. Waktu proses per rencana

Setiap robot merencanakan **satu kali** per percobaan, jadi ada 4 rencana per perencana per run.

### 4.1 Rata-rata per bagian (run `235558`)

| Bagian | Hybrid A\* + RS | DQfD murni | Rasio |
|---|---:|---:|---:|
| **Pencarian jauh:** Hybrid A\* / model jauh | 22,87 ms (98%) | **12,91 ms** (95%) | DQfD 1,8× lebih cepat |
| **Penyelesaian dekat:** RS / model dekat | **0,37 ms** (2%) | 0,55 ms (4%) | RS 1,5× lebih cepat |
| Lain-lain | 0,06 ms | 0,10 ms | |
| **Total (jam dinding)** | 23,30 ms | **13,55 ms** | DQfD 1,7× lebih cepat |
| **Total (CPU)** | 23,23 ms | **13,51 ms** | DQfD 1,7× lebih cepat |

### 4.2 Rincian setiap rencana (run `235558`, ms)

| Percobaan | Robot | Mulai | Jam / CPU | Hybrid A\* | RS | Model jauh | Model dekat |
|---|---|---:|---:|---:|---:|---:|---:|
| S1 Hybrid A\* + RS | 01 / 02 | +0,8 / +1,7 | 12,9 / 12,6 | 12,31 / 12,20 | 0,57 / 0,33 | – | – |
| S2 Hybrid A\* + RS | 01 / 02 | +0,7 / +1,7 | 34,3 / 33,4 | 33,98 / 33,01 | 0,25 / 0,33 | – | – |
| S1 DQfD murni | 01 / 02 | +0,8 / +1,6 | 22,0 / 19,0 | – | – | 21,18 / 18,22 | 0,68 / 0,60 |
| S2 DQfD murni | 01 / 02 | +0,9 / +2,0 | 5,9 / 7,4 | – | – | 5,36 / 6,86 | 0,42 / 0,48 |

IPC per rencana 0,49–1,02 ms. Dua proses perencana siap dalam 0,92 s.

### 4.3 Pengulangan: dua run

Run `234613` dijalankan dengan commit `e981f0d` dan opsi `--perencana-proses`, yaitu cara merencana yang sama sebelum opsinya dijadikan satu-satunya cara. Kolom `pid` belum dicatat di run itu.

| | Run `234613` | Run `235558` | **Rata-rata** |
|---|---:|---:|---:|
| Makespan S1: A\* / DQfD | 19,19 / 10,36 s | 19,26 / 10,31 s | **19,23 / 10,34 s** |
| Makespan S2: A\* / DQfD | 24,05 / 23,98 s | 23,96 / 24,01 s | **24,01 / 24,00 s** |
| Rencana A\* (total) | 23,35 ms | 23,30 ms | **23,3 ms** |
| Rencana DQfD (total) | 11,21 ms | 13,55 ms | **12,4 ms** |
| Bagian jauh: Hybrid A\* / model jauh | 22,92 / 10,62 ms | 22,87 / 12,91 ms | **22,9 / 11,8 ms** |
| Bagian dekat: RS / model dekat | 0,35 / 0,49 ms | 0,37 / 0,55 ms | **0,36 / 0,52 ms** |
| Rasio waktu proses A\* / DQfD | 2,1× | 1,7× | **1,9×** |

- **Hybrid A\* stabil antar-run**: S1 10,9–13,7 ms dan S2 33,1–35,7 ms per rencana.
- **DQfD S1 bervariasi** (13,7–22,0 ms untuk rencana yang sama). Robot memulai percobaan dari pose hasil pulang yang meleset 3,4–3,6 cm dari titik awal, dan pose itu sedikit berbeda setiap run. DQfD pun menempuh pencarian yang sedikit berbeda. Waktu dan jalur DQfD peka terhadap pose awal.
- **Dampak ke makespan bisa diabaikan.** Rencana terlama 36 ms, < 0,2% dari makespan 10–24 s.

## 5. Kesimpulan dan langkah berikutnya

1. **Keunggulan makespan DQfD murni sepenuhnya dari S1** (−46%), karena jalurnya lebih pendek dan tanpa ganti arah. S2 (putar balik di lorong sendiri) tidak membedakan perencana.
2. **Waktu proses DQfD murni unggul dengan konsisten (1,9×), terutama di model jauh vs Hybrid A\*.** Di bagian dekat, RS sedikit lebih cepat, tetapi tidak berpengaruh ke total.
3. Langkah berikutnya:
   - Ulangi dengan **`--ulang 5`**, di simulasi dengan derau dan di robot nyata, untuk mendapat sebaran dan uji signifikansi.
   - Tambah skenario yang **memaksa pilihan rute**, misalnya pindah lorong lewat celah di ujung Z5–Z8. Ini lebih mungkin membedakan makespan daripada putar balik di tempat.
   - Rapikan zona peta Gudang ke garis ubin (bagian 1.1), lalu periksa ulang tengah lorong dan semua pose skenario.
   - Jalankan `drl.uji_pose_akhir` sekali. Alat ini sudah dipindah ke `ProcessPoolExecutor`, tetapi belum diuji.

> [!warning] Batasan
> - **n = 1 per sel (skenario × perencana) per run, dua run, simulasi tanpa derau.** Belum ada uji statistik.
> - **Titik start tidak identik antar-percobaan.** Percobaan pertama (S1 A\*) dimulai tepat di pose awal (galat awal 0). Percobaan lain dimulai dari pose hasil pulang dengan galat awal 2,3–3,6 cm.
> - **Skenario berbeda dari catatan 15 dan 17**, sehingga angkanya tidak bisa dibandingkan langsung. Catatan 15 dan 17 juga diukur dengan perencanaan di thread dan tujuan yang diberikan berurutan.
> - **Uji robot nyata dengan skenario ini belum dilakukan.**

## 6. Sumber angka

| Angka | Sumber |
|---|---|
| Skenario | `Forklift Robot Program/uji_nyata/Gudang.json`, salinannya di folder hasil (`skenario.json`) |
| Bagian 1.2 dan 1.4 | skrip sementara di luar repo: `lurus_tengah.py` (yaw dan tengah lorong), `ubah_y.py` dan `cari_y.py` (batas y awal), `cek_pulang.py` (jalur pulang), `balik_s2.py` (arah tujuan S2) |
| Bagian 1.3 (gambar peta, skenario, Dijkstra) | skrip sementara `gambar_peta.py`: `HybridAStarPlanner._holonomic_heuristic` pada `Perencana.peta_dengan()`. Panjang jalur dari `20261104_235558_Gudang_sim` |
| Bagian 2 (metode) | kode commit `4d26e77`: `perencana_proses.py`, `navigasi.py`, `rencana.py` (`jam_cpu_thread`), `drl/uji_robot_nyata.py` |
| Bagian 3–4 | `Forklift Robot Program/uji_nyata/20261104_235558_Gudang_sim/` (`hasil.jsonl`, `ringkasan.json`, `lintasan_S1.png`, `lintasan_S2.png`), ulangan `20261104_234613_Gudang_sim`, direkap dengan skrip sementara `rekap.py` |
