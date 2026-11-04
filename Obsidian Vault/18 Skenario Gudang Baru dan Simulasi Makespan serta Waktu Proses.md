# Skenario Gudang baru (homografi ulang): simulasi makespan dan waktu proses Hybrid A\* + RS vs DQfD murni

Tanggal: 2026-11-04, diperbarui 2026-11-05 · Branch `ver-1-5`, kode commit `4d26e77` · Lanjutan dari [[17 Forward Numpy ver-1-5 dan Perbandingan Waktu Proses serta Makespan|Forward Numpy ver-1-5 dan Perbandingan Waktu Proses serta Makespan]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Homografi kamera diulang, lalu skenario uji Gudang dibuat dari awal dengan `drl.uji_robot_nyata --buat`. Skenario lama dihapus di commit `1f029d3`. Pada 2026-11-05 ditambahkan S3 dan S4, dua skenario yang memaksa robot **pindah lorong lewat celah di ujung dinding**. Catatan ini merekam bentuk keempat skenario dan hasil simulasinya (`--simulasi --ulang 2`), dengan perbandingan **makespan** dan **waktu proses** yang dipecah menjadi Hybrid A\*, RS, model jauh, dan model dekat.

Semua angka diukur dengan metode di bagian 2: tujuan kedua robot diberikan bersamaan, setiap robot merencana di **proses** sendiri, dan waktu dicatat sebagai jam dinding dan CPU.

> [!summary] Ringkasan (run `20261105_002833_Gudang_sim`, 4 skenario × 2 perencana × 2 ulangan)
> - **Makespan, rata-rata dua ulangan:**
>   - S1 (lintas lorong diagonal): DQfD murni **10,33 s** vs Hybrid A\* + RS 18,47 s (**−44%**).
>   - S2 (putar balik di lorong sendiri): 24,00 vs 23,99 s (setara).
>   - S3 (pindah lorong mengitari ujung Z6/Z8): DQfD 11,88 vs **A\* 10,82 s** (DQfD **+10%**).
>   - S4 (pindah lorong mengitari ujung Z7/Z5): DQfD **10,74 s** vs A\* 17,60 s (**−39%**).
>   - Jumlah keempat skenario: **56,95 vs 70,88 s (−20%)**.
> - **Selisih makespan berasal dari bentuk jalur, dan skenario pindah lorong membedakan perencana ke dua arah.** A\* kalah bila membuat manuver mundur-maju (S1, dan robot 02 di S4). DQfD kalah bila busurnya mengitari ujung dinding terlalu lebar (robot 02 di S3).
> - **Waktu proses per rencana** (16 rencana per perencana): DQfD **10,5 ms** vs A\* **26,3 ms** rata-rata (**2,5×**; median 10,0 vs 18,5 ms, 1,8×). Jam dinding = CPU (selisih ≤ 0,3 ms).
>   - Pencarian jauh: model jauh **9,9 ms** vs Hybrid A\* **25,8 ms** (2,6×).
>   - Penyelesaian dekat: RS **0,40 ms** vs model dekat 0,49 ms (RS 1,2× lebih cepat, keduanya < 1 ms).
> - **Waktu proses ≤ 0,8% dari makespan** (rencana terlama 82 ms). Makespan ditentukan oleh jalur, bukan oleh komputasi.
> - **Tanpa interaksi antar-robot:** waktu menunggu 0 s, minggir 0 kali, jarak badan terdekat kedua robot 30,5–71,3 cm.
> - **n = 2 per sel, simulasi tanpa derau.** Belum cukup untuk uji statistik.

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
   - Tambah ulangan (misalnya `--ulang 5`) dan derau simulasi, lalu uji di robot nyata, untuk sebaran dan uji signifikansi.
   - Uji peta Gudang2 (`Camera/peta_constraint/Gudang2.json` dan `uji_nyata/Gudang2.json`, peta baru dengan pulau rak dan teluk parkir).
   - Rapikan zona peta Gudang ke garis ubin (bagian 1.1), lalu periksa ulang tengah lorong dan semua pose skenario.
   - Jalankan `drl.uji_pose_akhir` sekali. Alat ini sudah dipindah ke `ProcessPoolExecutor`, tetapi belum diuji.

> [!warning] Batasan
> - **n = 2 per sel (skenario × perencana), simulasi tanpa derau.** Belum ada uji statistik.
> - **Titik start tidak identik antar-percobaan.** Percobaan pertama (S1 A\* ulangan 1) dimulai tepat di pose awal (galat awal 0). Percobaan lain dimulai dari pose hasil pulang dengan galat awal 2,3–3,9 cm, dan jalur kedua perencana peka terhadap selisih ini.
> - **Galat arah akhir S3–S4 sampai 10°**, kemungkinan karena bagian lurus terakhir pendek (bagian 3).
> - **Skenario berbeda dari catatan 15 dan 17**, sehingga angkanya tidak bisa dibandingkan langsung. Catatan 15 dan 17 juga diukur dengan perencanaan di thread dan tujuan yang diberikan berurutan.
> - **Uji robot nyata dengan skenario ini belum dilakukan.**

## 6. Sumber angka

| Angka | Sumber |
|---|---|
| Skenario | `Forklift Robot Program/uji_nyata/Gudang.json`, salinannya di folder hasil (`skenario.json`) |
| Bagian 1.2 dan 1.4 | skrip sementara di luar repo: `lurus_tengah.py` (yaw dan tengah lorong), `ubah_y.py` dan `cari_y.py` (batas y awal), `cek_pulang.py` (jalur pulang), `balik_s2.py` (arah tujuan S2), `draf_gudang_rute.py` (rancangan S3/S4, jarak sapuan, jarak pada waktu yang sama), `periksa_gudang2.py` (pemeriksaan lewat `Sesi`) |
| Bagian 1.3 (gambar peta, skenario, Dijkstra) | skrip sementara `gambar_peta.py`: `HybridAStarPlanner._holonomic_heuristic` pada `Perencana.peta_dengan()`. Panjang jalur dari `20261105_002833_Gudang_sim` |
| Bagian 2 (metode) | kode commit `4d26e77`: `perencana_proses.py`, `navigasi.py`, `rencana.py` (`jam_cpu_thread`), `drl/uji_robot_nyata.py` |
| Bagian 3–4 | `Forklift Robot Program/uji_nyata/20261105_002833_Gudang_sim/` (`hasil.jsonl`, `lintasan.csv`, `ringkasan.json`, `lintasan_S1.png` – `lintasan_S4.png`), direkap dengan skrip sementara `analisis_u2.py` (jarak badan kedua robot dan celah loop dari `lintasan.csv`) |
