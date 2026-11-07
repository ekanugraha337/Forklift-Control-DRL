# Peta Gudang2 baru: skenario silang dan jadwal silang

Tanggal: 2026-11-05 · Branch `ver-1-5`, kode commit `4d26e77` · Skenario Gudang: [[18 Skenario Gudang Baru|catatan 18]] · Wasit: [[05 Multi Robot Navigation and Control|Multi Robot Navigation and Control]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Peta Gudang2 diganti dengan rancangan baru: pulau rak di tengah, teluk parkir di dinding bawah, dan slot ambil di dinding atas. Berbeda dengan skenario Gudang di catatan 18, yang sengaja dibuat tanpa interaksi, skenario Gudang2 ini **sengaja membuat jalur kedua robot bersilangan**. Tanpa wasit, kedua robot akan bertabrakan, sehingga wasit harus memutuskan siapa yang menunggu, memutar, atau minggir.

Catatan ini merekam peta, skenario, metode ukur, dan cara kerja jadwal silang.

> [!warning] Dibersihkan 2026-11-07 (aturan ver-1-6)
> Semua hasil simulasi dan uji robot nyata (makespan, waktu proses, galat pose akhir, kerja wasit, isi berkas jadwal, panjang jalur) dihapus, karena diukur dengan Hybrid A\* + RS dan DQfD dua model aturan lama. Peta, skenario, metode ukur, dan cara kerja jadwal silang tetap berlaku.

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
- **Masuk teluk maju** sedalam y = 185, dengan bagian lurus terakhir 30–35 cm. Di tujuan, kedua badan berjarak 42,5 cm, dipisahkan sekat T2.
- Semua pose muat sebagai tujuan dengan margin 8 cm, termasuk bila badan robot lain berada di pose awalnya.

### 1.3 Skenario dan heatmap Dijkstra

Panel kiri menampilkan pose awal (titik-titik) dan tujuan (garis putus). Panel tengah dan kanan adalah heuristik Dijkstra ke tujuan setiap robot (`_holonomic_heuristic`, sel 5 cm), yang dipakai Hybrid A\* dan lingkungan DQfD. Kondisinya tanpa badan robot lain sebagai rintangan, sama dengan saat uji.

![[Skenario Gudang2 2026-11-05 S1.png]]

## 2. Metode pengukuran

Sama dengan [[18 Skenario Gudang Baru|catatan 18]] bagian 2:
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
- **waktu menunggu** (`t_menunggu_s`) dan **jumlah minggir** (`n_minggir`) per robot. Cara menghitungnya ada di bagian 3 (koreksi waktu menunggu);
- **jumlah rencana** (`n_rencana`) dan rencana ulang (terhitung, gagal, total, dan terlama);
- **jarak tempuh aktual** (`jarak_tempuh_cm`) dibanding panjang rencana pertama;
- **jarak badan kedua robot terhadap waktu**, dari `lintasan.csv`.

## 3. Cara kerja jadwal silang

Konflik diselesaikan **sebelum** uji. Salah satu robot ditetapkan mengalah, menunggu di titik aman, dan baru melanjutkan setelah robot lain lewat.

1. **Sebelum uji**, `drl/jadwal_silang.py` merencanakan kedua robot dari pose awal ke tujuannya untuk setiap perencana, sama seperti saat uji.
   - Untuk setiap pilihan "robot Y mengalah kepada robot P", program menghitung titik konflik, titik tunggu, kapan Y boleh lanjut, dan perkiraan makespan (dalam cm tempuh, dengan laju yang sama).
   - Pilihan dengan makespan terkecil disimpan ke `uji_nyata/Gudang2_jadwal.json`.
2. **Saat uji**, `drl/uji_robot_nyata.py --jadwal` mengatur robot yang mengalah:
   - **Ditahan** begitu badannya di sepanjang 10 cm jalur depannya masuk dalam 15 cm dari badan dan sisa jalur robot yang lewat. Ditahan berarti roda diperintah nol dan jalur tetap dipegang. Bagi wasit, robot itu dilihat sebagai badan tanpa bayangan.
   - **Dilepas** setelah salah satu terjadi: badan dan sisa jalur robot yang lewat sudah lebih jauh dari 15 cm dari seluruh sisa jalurnya; robot yang lewat sudah tiba; atau sudah ditahan 60 s. Yang dipakai adalah posisi robot yang lewat, bukan jam, jadi tetap benar walau robot nyata lebih lambat atau lebih cepat daripada simulasi.
   - **Langsung merencana ulang saat dilepas**, bila sisa jalurnya lewat dalam 10 cm dari robot yang sudah parkir. Perencanaannya sama dengan yang dilakukan wasit setelah menunggu 5 s (memutari semua robot lain), hanya tanpa menunggu. Robot tetap ditahan sampai jalur barunya terpasang.
3. **Wasit tetap aktif** sebagai pengaman, dengan parameter bagian 2.

| Parameter | Nilai | Arti |
|---|---:|---|
| `jarak_cm` (berkas jadwal) | 15 cm | jarak badan yang dijaga antara robot yang mengalah dan badan + sisa jalur robot yang lewat (wasit: 5 cm) |
| `penyangga_cm` (berkas jadwal) | 10 cm | panjang jalur depan yang diperiksa, supaya robot berhenti sebelum masuk daerah 15 cm |
| `LEPAS_RENCANA_ULANG_CM` | 10 cm | saat dilepas, sisa jalur sedekat ini ke robot yang sudah parkir memicu rencana ulang langsung |
| `RENCANA_ULANG_MAKS_S` | 3 s | batas menunggu jalur memutar itu; setelahnya robot dilepas dan wasit yang menjaga |
| `TAHAN_MAKS_S` | 60 s | batas lama ditahan |

Aturan tahan dihitung dari jalur sebenarnya saat uji (bukan titik tunggu dari berkas), dan robot yang mengalah merencana ulang langsung saat dilepas.

> [!note] Koreksi waktu menunggu
> `t_menunggu_s` di `hasil.jsonl` menjumlah `RC_PERIODE_S` (50 ms) per siklus, padahal satu siklus sebenarnya 64 ms, karena `cv2.waitKey` di Windows dibulatkan ke detak 15,6 ms, sehingga tercatat **±22% terlalu kecil**.
>
> Sejak uji jadwal, waktu menunggu dijumlah dari selang nyata, yaitu dari stempel waktu `lintasan.csv`. Makespan, waktu tempuh, dan waktu proses tidak terpengaruh.
