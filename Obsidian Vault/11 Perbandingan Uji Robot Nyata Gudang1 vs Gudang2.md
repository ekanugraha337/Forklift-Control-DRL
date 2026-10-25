# Perbandingan uji robot nyata ver-1-4: Gudang (1) vs Gudang2

Tanggal: 2026-10-24 · Branch `ver-1-4` · Uji: [[09 Uji Robot Nyata Gudang ver-1-4 Gundang1|Uji Robot Nyata Gudang ver-1-4 Gundang1]] (`uji_nyata/20261024_073221_Gudang`) dan [[10 Uji Robot Nyata Gudang ver-1-4 Gundang2|Uji Robot Nyata Gudang ver-1-4 Gundang2]] (`uji_nyata/20261024_082405_Gudang2`) · Model: [[08 Perbandingan Hybrid A-star RS vs DQfD|Perbandingan Hybrid A-star RS vs DQfD]]

Dua uji, dua robot, perencana dan pengaturan yang sama (`astar` vs `dqfd-murni`, margin 8 cm, pure pursuit 20 Hz, pulang dengan Hybrid A\*). Yang berbeda hanya peta dan skenario. Hasilnya berbalik:
- di **Gudang**, Hybrid A\* + RS merencanakan sekitar 5× lebih cepat;
- di **Gudang2**, DQfD murni sekitar 3,4× lebih cepat.

Catatan ini menjabarkan penyebabnya dan keunggulan lain masing-masing perencana.

> [!summary] Ringkasan
> - **Waktu rencana ditentukan jenis rute, bukan peta.**
>   - Hybrid A\* membayar sekitar 0,15 ms per ekspansi, dan jumlah ekspansinya **meledak** bila robot harus berbalik (Gudang 69–493, Gudang2 565–1169).
>   - DQfD membayar sekitar 1 ms per langkah, tetapi jumlah langkahnya kira-kira sebanding **panjang jalur** (Gudang 12–83, Gudang2 17–26).
>   - DQfD lebih cepat bila ekspansi A\* lebih dari sekitar 7× langkah DQfD (offline), atau sekitar 12× di robot.
> - **Gudang: A\* unggul** karena robot mulai menghadap 50–100° dari arah tujuan. Heuristik A\* (jarak Dijkstra 2D, tanpa arah hadap) hampir tepat, jadi A\* langsung menuju tujuan.
> - **Gudang2: DQfD unggul** karena robot mulai membelakangi tujuan (161–180°). Heuristik A\* menyesatkan, sehingga A\* membuka ratusan simpul untuk "menemukan" manuver mundur-lalu-berbalik. DQfD sudah belajar manuver itu dari demonstrasi.
> - **Di robot nyata DQfD melambat sekitar 3×, A\* sekitar 1,9×** (dibanding offline), di kedua peta. Ini memperkecil keunggulan DQfD dan memperbesar keunggulan A\*.
> - **Gerak**: makespan dan jalur hampir sama di kedua peta. Perbedaan yang ada berasal dari interaksi antar-robot (Gudang) dan rancangan skenario (Gudang2), bukan dari perencananya.

---

## 1. Dua uji

| | Gudang | Gudang2 |
|---|---|---|
| Hasil | `20261024_073221_Gudang` | `20261024_082405_Gudang2` |
| Peta | dinding luar + 4 sekat pendek dari tepi atas/bawah (x ≈ 65 dan 151) | dinding luar + 1 pembatas panjang di tengah (x 45–194, y ≈ 105) |
| Skenario | S1: rute diagonal **bersilangan**; S2: balik arah ke teluk tengah, tidak bersilangan | S1–S3: tiap robot di **lajurnya sendiri**, mulai membelakangi tujuan |
| Sudut arah hadap awal terhadap arah ke tujuan (median) | S1 53–56°, S2 95° | 170–178° |
| Jarak lurus awal → tujuan | S1 ≈ 208 cm, S2 ≈ 82 cm | 165–175 cm |
| Percobaan | 11 (S2 ulangan 3 DQfD tidak ada) | 18 |
| Dirancang untuk | — (skenario biasa) | keunggulan komputasi DQfD |

## 2. Hasil berdampingan

| | Gudang A\* | Gudang DQfD | Gudang2 A\* | Gudang2 DQfD |
|---|---|---|---|---|
| Pasangan tiba / estop | 6/6 / 0 | 5/5 / 0 | 9/9 / 0 | 9/9 / 0 |
| **Rencana pertama, median** | **31 ms** | 149 ms | 198 ms | **58 ms** |
| Rencana pertama, rata-rata | 41 ± 24 ms | 130 ± 64 ms | 204 ± 45 ms | 59 ± 5 ms |
| Lebih cepat merencanakan | **A\*: ~5×** | | | **DQfD: ~3,4× (36/36)** |
| Makespan | 21,7 ± 11,4 s | 19,0 ± 10,3 s | 29,6 ± 3,2 s | 29,6 ± 2,9 s |
| Menunggu robot lain (rata-rata per robot) | 1,7 s | 1,2 s | 0 | 0 |
| Rencana per robot per percobaan | 1,3 (maks 5) | 1,1 (maks 2) | 1,0 | 1,0 |
| Panjang jalur rencana | 215 cm | 220 cm | 223 cm | 224 cm |
| Galat akhir | 2,4 ± 0,8 cm / 5,0 ± 3,0° | 1,8 ± 0,8 cm / 4,3 ± 2,0° | 2,7 ± 1,6 cm / 5,4 ± 5,7° | 2,2 ± 1,3 cm / 8,4 ± 8,4° |
| Jarak terdekat ke zona | 7,3 cm | 9,7 cm | 9,4 cm | 9,2 cm |

## 3. Penyebab keunggulan komputasi

### 3.1 Dua model biaya yang berbeda

Rencana pertama dari kedua uji diulang offline (jalur cocok dengan rekaman: 22/22 Gudang, 36/36 Gudang2). Yang dicatat: jumlah ekspansi Hybrid A\* dan jumlah langkah pencarian DQfD (forward model jauh).

| | Ekspansi A\* (median, rentang) | ms per ekspansi | Langkah DQfD (median, rentang) | ms per langkah | A\* offline | DQfD offline |
|---|---|---|---|---|---|---|
| Gudang S1 | 140 (70–493) | 0,15 | 54 (34–83) | 1,26 | 21 ms | 67 ms |
| Gudang S2 | 91 (69–158) | 0,16 | 15 (12–20) | 1,27 | 15 ms | 20 ms |
| Gudang2 S1 | 969 (598–1169) | 0,15 | 20 (19–26) | 0,94 | 142 ms | 19 ms |
| Gudang2 S2 | 652 (565–776) | 0,14 | 17 (17–20) | 1,03 | 92 ms | 19 ms |
| Gudang2 S3 | 680 (613–871) | 0,14 | 18 (18–25) | 0,91 | 98 ms | 17 ms |

- **Satu ekspansi Hybrid A\* murah** (sekitar 0,15 ms): 14 primitif gerak, cek badan, dan nilai heuristik dari tabel Dijkstra.
- **Satu langkah DQfD sekitar 7× lebih mahal** (0,9–1,3 ms). Isinya: observasi dengan 32 sinar jarak, cek badan 14 primitif untuk mask, dan satu forward jaringan. Sinar adalah porsi terbesar (sekitar 50% waktu DQfD, lihat [[08 Perbandingan Hybrid A-star RS vs DQfD|Perbandingan Hybrid A-star RS vs DQfD]] bagian 5).
- Karena itu, yang menentukan pemenang adalah **jumlahnya**:
  - waktu A\* ≈ 0,15 ms × ekspansi;
  - waktu DQfD ≈ 1 ms × langkah.
  
  DQfD lebih cepat bila ekspansi A\* > sekitar 7 × langkah DQfD. Rasio ekspansi/langkah: Gudang S1 2,6 dan S2 6 (A\* menang); Gudang2 sekitar 35 (DQfD menang jauh).

### 3.2 Kenapa Hybrid A\* unggul di Gudang

- **Heuristiknya hampir tepat.** Heuristik A\* adalah jarak Dijkstra 2D dari tujuan, yang memperhitungkan dinding tetapi **tidak memperhitungkan arah hadap**. Di Gudang robot mulai menghadap 50–100° dari arah tujuan, jadi primitif maju sambil berbelok langsung menurunkan heuristik. Pencarian bergerak hampir lurus ke tujuan: 69–158 ekspansi untuk sebagian besar rencana.
- **Tembakan RS menutup sisa jalur.** Begitu < 60 cm dari tujuan, RS (0,2–0,3 ms) langsung menyambung, dan pencarian berhenti.
- **DQfD harus membayar per langkah.** Rute S1 sekitar 260–290 cm, jadi model jauh butuh sekitar 25 langkah sampai 60 cm dari tujuan. Pencarian terbaik-dulu DQfD bahkan memakai 34–83 langkah. Langkah tambahan itu adalah cabang yang dibuka lalu ditinggalkan di sekitar sekat dan badan robot lain. Dengan sekitar 1,26 ms per langkah, ini 43–100 ms offline.
- Contoh ekstrem: Gudang S1 ID 01 A\* 70–171 ekspansi (12–25 ms) vs DQfD 37–83 langkah (49–93 ms).
- Pengecualian: ID 02 di S1 ulangan 1–2 (A\* 319–493 ekspansi), tempat sekat dan robot lain memperpanjang pencarian A\*. Bahkan di situ A\* tetap 46–70 ms, setara atau lebih cepat daripada DQfD ID 02 (43–100 ms).

### 3.3 Kenapa DQfD unggul di Gudang2

- **Heuristik A\* menyesatkan.** Robot mulai membelakangi tujuan. Heuristik menilai simpul di dekat tujuan murah tanpa melihat arah hadap. Gerak mundur juga diberi biaya 2× (`reverse_cost`) dan ganti arah +32,4 cm (`switch_cost`). Akibatnya A\* membuka ratusan simpul ke berbagai arah dan sudut sebelum menemukan manuver termurah: mundur lurus, satu titik balik, lalu maju ke tujuan. Hasilnya 565–1169 ekspansi (81–180 ms offline).
- **DQfD sudah tahu manuvernya.** Model jauh dilatih meniru demonstrasi Hybrid A\* + RS, termasuk manuver berbalik seperti ini. Jaringan langsung memberi nilai Q tertinggi ke primitif mundur, lalu ke titik balik. Pencarian hampir tidak bercabang:
  - jumlah langkah 17–26 ≈ forward 15–19 + sedikit cabang;
  - itu kira-kira panjang bagian jauh / 7,3 cm.
- **Waktu DQfD hanya bergantung pada panjang jalur.** Hasilnya 15–25 ms offline dan 51–69 ms di robot, nyaris konstan di semua skenario Gudang2.
- **Lajur terbuka membuat langkah DQfD lebih murah** (0,91–1,03 ms vs 1,26 ms di Gudang). Ini terukur, tetapi bagian mana dari langkah yang menjadi lebih murah (sinar, mask, atau cek badan) belum dirinci.

### 3.4 Pola yang sama di data uji offline

Pola ini sudah terlihat pada 505 rute uji di Kalibrasi, Rumah2, dan Rumah sebelum Gudang2 dirancang:

| Sudut awal terhadap arah ke tujuan | Jarak | DQfD lebih cepat | Rasio DQfD/A\* |
|---|---|---|---|
| 0–45° | ≥ 170 cm | 0% | 3,39× |
| 45–90° | 100–170 cm | 33% | 1,63× |
| 135–180° | 100–170 cm | 79% | 0,63× |
| 135–180° | ≥ 170 cm | **89%** | **0,21×** |

Gudang S1 (sekitar 55°, 208 cm) dan S2 (sekitar 95°, 82 cm) ada di wilayah A\*. Gudang2 (sekitar 175°, sekitar 170 cm) ada di wilayah DQfD.

### 3.5 Faktor robot nyata

| | Gudang: uji / offline | Gudang2: uji / offline |
|---|---|---|
| Hybrid A\* + RS | 1,5–2,0× | 1,8× |
| DQfD murni | 2,7–3,2× | 3,2× |

- Di robot, perencana berjalan di thread pekerja bersama loop kamera dan kendali 20 Hz. DQfD selalu melambat lebih banyak daripada A\*. Dugaannya, ratusan forward torch kecil lebih terganggu oleh perebutan CPU/GIL. Ini belum diukur.
- Akibatnya batas impas bergeser. Di robot, DQfD baru lebih cepat bila ekspansi A\* lebih dari sekitar 12× langkah DQfD, bukan sekitar 7×.
- Contoh: Gudang S2 hampir seimbang offline (15 vs 20 ms), tetapi di robot A\* jelas menang (28 vs 63 ms).

## 4. Penyebab keunggulan gerak

| Keunggulan | Di mana | Penyebab |
|---|---|---|
| Makespan DQfD sedikit lebih kecil (19,0 vs 21,7 s) | Gudang | **Interaksi antar-robot**, bukan perencana. Rute S1 bersilangan, dan percobaan terlambat masing-masing karena saling menghalangi: A\* S1 ulangan 1, ID 02 merencanakan ulang 5× dan menunggu 7 s (40,8 s); DQfD S1 ulangan 1, ID 01 merencanakan ulang sekali dan menunggu 4,7 s (33,6 s). Percobaan tercepat keduanya hampir sama (17,5 vs 16,4 s). Selisihnya lebih kecil dari sebarannya. |
| Lintasan DQfD lebih konsisten | Gudang S2 | Sekali Hybrid A\* memberi ID 01 jalur 178 cm (biasanya sekitar 147 cm, makespan 20,6 s). Pose awal yang sedikit berbeda dapat menggeser pencarian A\* ke sel/sudut diskret lain, sehingga jalurnya berubah. DQfD memberi jalur yang nyaris sama ketiga kalinya. Penyebab pasti di A\* belum diperiksa per rencana. |
| Presisi ID 01 lebih baik dengan DQfD (0,9 vs 2,4 cm) | Gudang S1 | Belum diketahui. Kedua perencana berakhir dengan segmen lurus (30 cm terakhir 0° belok) dan galat rencana 0. Panjang segmen lurus terakhir berbeda-beda di keduanya (30–245 cm), tanpa pola yang jelas. Perlu ulangan lebih banyak. |
| Makespan dan jalur sama | Gudang2 | Robot tidak berinteraksi (lajur terpisah). DQfD meniru demonstrator Hybrid A\* + RS, sehingga jalurnya hampir identik (223 vs 224 cm, ganti arah sama). |
| Galat ID 01 besar dengan kedua perencana | Gudang2 S2/S3 | **Kesalahan rancangan skenario.** Di (35, 40, −90°) dan (205, 40, −90°), segmen lurus pendekatan 32,4 cm tidak muat: badan robot ditambah margin menyentuh ujung pembatas. Jalur kedua perencana masuk ke tujuan sambil masih berbelok 68–71° dalam 30 cm terakhir. Dengan tujuan di y ≤ 34, pendekatan lurusnya muat. Ini tidak menguntungkan atau merugikan salah satu perencana. |
| Bias arah hadap ID 02 | keduanya | Gudang 5–8°, Gudang2 3,5–4,3°, sama untuk kedua perencana. Ini masalah robot (kemudi, trim, atau tag), bukan perencana. |

## 5. Kesimpulan

1. **Tidak ada perencana yang selalu lebih cepat.** Waktu rencana ditentukan oleh seberapa jauh arah hadap awal robot dari arah tujuan:

   | Kondisi rute | Lebih cepat | Alasan |
   |---|---|---|
   | Robot menghadap kira-kira ke tujuan (< 90°), atau rute pendek | **Hybrid A\* + RS** | heuristik tepat → sedikit ekspansi murah (0,15 ms) |
   | Robot membelakangi tujuan (> 135°) dan tujuan jauh (≥ 100–170 cm) | **DQfD murni** | heuristik A\* menyesatkan → ratusan ekspansi; DQfD menjalankan manuver hasil belajar dalam sekitar 20 langkah |

2. **DQfD lebih mudah diprediksi.** Di kedua uji, waktu DQfD 48–235 ms (sekitar 5× antara tercepat dan terlambat). A\* 12–301 ms (sekitar 25×). Waktu DQfD mengikuti panjang jalur, sedangkan waktu A\* mengikuti kesulitan heuristik.
3. **Waktu rencana tidak memengaruhi gerak.** Di kedua peta, waktu rencana (< 0,3 s) jauh lebih kecil daripada waktu tempuh (10–34 s). Makespan dan jalur hampir sama. Perbedaan gerak berasal dari interaksi antar-robot dan rancangan skenario.
4. **Keamanan dan keandalan setara.** 29 percobaan, semua tiba, 0 estop.
5. **Untuk laporan**: sajikan kedua uji bersama. Gudang2 menunjukkan kondisi DQfD unggul, Gudang menunjukkan kondisi A\* unggul. Masing-masing sendirian akan memberi kesimpulan yang bias.

> [!warning] Keterbatasan
> - n = 2–3 per sel. Untuk waktu rencana di Gudang2 tidak ada tumpang-tindih (DQfD maks 69 ms < A\* min 134 ms). Untuk makespan dan galat, perbedaannya masih berupa kecenderungan.
> - Gudang2 dirancang khusus untuk keunggulan DQfD. Gudang tidak dirancang untuk keunggulan A\*.
> - Penyebab DQfD melambat sekitar 3× di robot, perbedaan ms per langkah DQfD antar-peta, dan keunggulan presisi ID 01 di Gudang S1 belum diperiksa.
> - Langkah lanjut: (a) Gudang2 dengan tujuan ID 01 di y = 34 untuk S2/S3; (b) skenario sudut awal 90–135° untuk mencari batas impas di robot; (c) profil waktu perencana di dalam loop kamera.

## 6. Sumber angka

| Angka | Sumber |
|---|---|
| Hasil uji | `hasil.jsonl`, `ringkasan.json`, `rincian_tabel.tex` di kedua folder hasil |
| Ekspansi, langkah, ms offline (bagian 3.1) | rencana pertama setiap robot setiap percobaan diulang offline 3× (median), rintangan robot lain dipilih supaya jalur cocok dengan rekaman; forward model dihitung dengan `PenghitungForward` dari `drl/banding_komputasi.py` |
| Pola 505 rute (bagian 3.4) | rute uji `drl/data/demo_jauh_id0N.npz` di Kalibrasi, Rumah2, Rumah; median 2× ulang, offline |
| Faktor uji/offline (bagian 3.5) | `rincian_ulang.json` (`--rinci`) dan pengulangan offline di atas |
| Pendekatan lurus Gudang2 (bagian 4) | `HybridAStarPlanner._feasible_approach_dirs` di tujuan skenario; belokan 30 cm terakhir dari `jalur_cm` di `hasil.jsonl` |
