# Sinar dengan numba (ver-1-5): waktu proses dan makespan Hybrid A\* + RS vs DQfD

Tanggal: 2026-11-04 · Branch `ver-1-5` · Lanjutan dari [[14 Waktu Komputasi per Tahapan Hybrid A-star RS vs DQfD Gudang|Waktu Komputasi per Tahapan Hybrid A-star RS vs DQfD Gudang]] · Uji nyata: [[09 Uji Robot Nyata Gudang ver-1-4 Gundang1|Uji Robot Nyata Gudang ver-1-4 Gundang1]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Catatan 14 menemukan bahwa 46% waktu `dqfd-murni` habis di observasi 32 sinar model jauh. Di ver-1-5, loop sinar itu dikompilasi dengan **numba**. Model, algoritma, dan Hybrid A\* + RS tidak diubah.

> [!summary] Ringkasan
> - **Sinar 21× lebih cepat di Windows** (16,65 → 0,80 ms per kasus) dan **35× di Mac** (9,64 → 0,27 ms). Hasilnya identik bit demi bit, jadi model tidak perlu dilatih ulang.
> - **Waktu proses DQfD turun hampir setengah**: Windows 32,6 → **17,4 ms**, Mac 18,1 → **8,6 ms**. Sekarang DQfD lebih cepat daripada Hybrid A\* + RS:
>   - Windows: 17,4 vs 25,0 ms (0,70×), DQfD menang di 3 dari 4 kasus;
>   - Mac: 8,6 vs 15,1 ms (0,57×), DQfD menang di 4 dari 4 kasus.
> - **Makespan simulasi** (dua robot, 5 ulangan per skenario): DQfD **17,3 ± 5,3 s** vs Hybrid A\* **20,2 ± 6,2 s**. Waktu proses hanya 0,2–0,4% dari makespan, jadi selisih ini datang dari jalur dan gerak robot, bukan dari komputasi. Dengan sampel sekecil ini, selisihnya belum signifikan.
> - **Bottleneck DQfD sekarang adalah bagian yang dipakai bersama A\***: cek tabrakan calon primitif, kinematik, dan Dijkstra. Ditambah satu bagian khusus DQfD: forward PyTorch, yang 2,5× lebih lambat di Windows.
> - **Mac lebih cepat dari Windows terutama karena overhead pemanggilan pustaka C** (numpy, PyTorch, numba), yang 1,5–2,5× lebih besar di Windows. Kecepatan dasar interpreter Python hanya berbeda 1,18×.

---

## 1. Perubahan

Perubahannya ada di `drl/lingkungan.py`:
- **`_sinar_loop`** adalah loop sphere tracing per sinar. Fungsi ini dikompilasi numba: `@njit(cache=True, nogil=True)`.
- **`_sinar_loop_numpy`** adalah loop numpy lama, dipertahankan untuk dua keperluan:
  - cadangan otomatis bila numba tidak terpasang atau gagal kompilasi;
  - acuan verifikasi.
- **`_sinar()`** tetap menyiapkan `cx`, `cy`, `cos`/`sin` dengan numpy seperti sebelumnya, lalu memanggil salah satu loop di atas.
- **`SINAR_NUMBA`** bernilai `True` bila numba aktif.

Supaya hasilnya identik dengan versi lama:
- `cos`/`sin` **tidak** dihitung di dalam numba. libm LLVM bisa berbeda satu ulp dari numpy, dan `floor()` akan memperbesar beda sekecil itu menjadi sel yang berbeda.
- Kernel numba hanya mengerjakan `t·u + c`, `floor`, perbandingan, `d − 0,5`, dan `+`, dengan urutan yang sama persis seperti versi numpy. Semua operasi itu hasilnya pasti identik menurut IEEE.
- `d − 0,5` tetap eksak walaupun `dist` bertipe float32, selama jarak < 2²² sel.

Alasan versi numba lebih cepat:
- Versi numpy lambat bukan karena hitungannya. Setiap iterasi menjalankan sekitar 28 operasi numpy pada larik 32 elemen, dan setiap operasi punya overhead pemanggilan ±0,3–0,5 µs.
- Selain itu, sinar yang sudah berhenti tetap ikut dihitung sampai sinar **terpanjang** selesai: rata-rata 22 iterasi, padahal hanya 9,8 dari 32 sinar yang masih aktif.
- Di versi numba, setiap sinar berhenti sendiri, dan semua operasi menjadi instruksi mesin.

Hal operasional:
- **Kompilasi:** dilakukan saat `lingkungan.py` diimpor, bukan saat rencana pertama robot. Hasilnya disimpan di `drl/__pycache__` (sudah di-ignore git). Impor pertama sekitar 0,9 s, impor berikutnya sekitar 0,4 s, yang sebagian besar adalah waktu impor numba sendiri.
- **`nogil=True`:** thread perencana melepas GIL selama sinar dihitung, sehingga tidak menahan loop kamera dan kendali di `navigasi.py`.
- **Dependensi:** `pip install numba`. Di laptop Windows terpasang `numba 0.68.1` dan `llvmlite 0.50.0`. Sebelumnya sudah dicek dengan `--dry-run`: numpy 2.4.2 tidak berubah.

## 2. Verifikasi

| Pemeriksaan | Hasil |
|---|---|
| Sinar numba vs numpy, 20.000 pose acak di semua peta: 2.000 per peta × 2 robot, termasuk pose menempel zona dan di luar peta | **20.000 / 20.000 identik**, selisih maksimum 0,0 |
| `python -m drl.adaptor_demo --uji` | **semua lulus**, sama dengan ver-1-4 |
| Jalur `dqfd-murni`, numba vs numpy (md5): 20 rute uji × Kalibrasi, Rumah, Rumah2 × ID 01, ID 02, ditambah 4 kasus Gudang | **123 / 124 identik** |
| Rute yang berbeda: `id02/Rumah/521`, tujuan **tanpa arah hadap** | tanpa batas waktu, jalurnya **identik**. Lihat penjelasan di bawah tabel |

Penjelasan rute `id02/Rumah/521`:
- Tujuan tanpa arah hadap dibatasi 0,4 s per arah, dan pencarian terbaik-dulu hanya mendapat 75% dari itu (0,3 s).
- Versi numpy butuh 512 ms. Ia kehabisan waktu lalu beralih ke pencarian mundur, sehingga jalurnya berbeda.
- Versi numba selesai dalam 279 ms dengan pencarian terbaik-dulu.
- Jadi algoritmanya sama. Versi numba hanya lebih jarang kehabisan waktu.

## 3. Waktu proses per rencana

Cara mengukurnya sama dengan catatan 14:
- 4 kasus Gudang (`uji_nyata/Gudang.json`, badan robot lain sebagai rintangan), 30 ulangan per kasus, A\* dan DQfD diselang-seling per ulangan;
- Windows: 3 run ver-1-5, **diselang-seling per run** dengan 2 run ver-1-4 (urutan: 1-5, 1-4, 1-5, 1-4, 1-5);
- Mac: 1 run per versi.

### 3.1 Total (tanpa pencatat waktu)

| | Windows ver-1-4 | **Windows ver-1-5** | Mac ver-1-4 | **Mac ver-1-5** |
|---|---:|---:|---:|---:|
| DQfD rata-rata / median (ms) | 32,6 / 31,6 | **17,4 / 17,2** | 18,1 / 18,0 | **8,6 / 8,5** |
| Hybrid A\* + RS rata-rata / median (ms), sesi yang sama | 24,8 / 24,2 | **25,0 / 24,9** | 15,5 / 15,4 | **15,1 / 15,1** |
| **DQfD ÷ A\*** (rata-rata) | 1,31× | **0,70×** | 1,17× | **0,57×** |
| Sinar per kasus (dengan pencatat waktu) | 16,65 ms | **0,80 ms** | 9,64 ms | **0,27 ms** |

Per kasus, rata-rata (ms):

| Kasus | Langkah DQfD / ekspansi A\* | Windows: DQfD 1-4 → **1-5** / A\* | Mac: DQfD 1-4 → **1-5** / A\* |
|---|---:|---|---|
| S1-ID01 | 44 / 115 | 52,2 → **27,6** / 25,2 | 31,5 → **13,6** / 16,1 |
| S2-ID01 | 10 / 143 | 17,6 → **10,4** / 32,4 | 9,4 → **5,5** / 19,2 |
| S1-ID02 | 29 / 92 | 39,8 → **20,2** / 23,1 | 19,8 → **9,6** / 13,5 |
| S2-ID02 | 12 / 78 | 20,9 → **11,5** / 19,3 | 11,7 → **5,7** / 11,5 |

- Di Windows, DQfD menang di 3 dari 4 kasus. Hanya kasus dengan langkah terbanyak, S1-ID01 (44 langkah), yang masih sedikit kalah: 27,6 vs 25,2 ms. Di Mac, DQfD menang di keempat kasus.
- Mac ver-1-4 diambil dari run terpisah, tidak diselang-seling, dari [[14 Waktu Komputasi per Tahapan Hybrid A-star RS vs DQfD Gudang|catatan 14]]. Waktu A\*-nya hampir sama (15,5 vs 15,1 ms), jadi kondisi mesinnya sebanding.

### 3.2 Rincian DQfD ver-1-5 per tahapan

ms per kasus, rata-rata 4 kasus, dengan pencatat waktu. Nama tahapan sama dengan catatan 14 bagian 3.

| No | Tahapan | Windows (ms) | Porsi | Mac (ms) | Porsi |
|---|---|---:|---:|---:|---:|
| 1–4 | Pra-proses (masukan, validasi, peta rintangan, konversi) | 1,189 | 6,1% | 0,679 | 7,4% |
| J1 | Reset lingkungan jauh | 0,201 | 1,0% | 0,095 | 1,0% |
| J2 | Heuristik Dijkstra | 2,750 | 14,2% | 1,975 | 21,6% |
| J3 | Arah pendekatan lurus | 0,103 | 0,5% | 0,058 | 0,6% |
| J4 | **Observasi: 32 sinar (numba)** | **0,800** | **4,1%** | **0,274** | **3,0%** |
| J5 | Observasi: sisa 47 masukan | 0,604 | 3,1% | 0,250 | 2,7% |
| J6 | Forward jaringan jauh (PyTorch) | 3,624 | 18,7% | 0,961 | 10,5% |
| J7 | Calon primitif: cek tabrakan | 3,870 | 20,0% | 2,322 | 25,4% |
| J8 | Calon primitif: kinematik dan biaya | 3,059 | 15,8% | 1,463 | 16,0% |
| J9 | Pencarian terbaik-dulu | 2,345 | 12,1% | 0,711 | 7,8% |
| D1–D6 | Model dekat (1 percobaan pendaratan) | 0,792 | 4,1% | 0,322 | 3,5% |
| 14 | Pasca-proses | 0,035 | 0,2% | 0,020 | 0,2% |
| | **Total (dengan pencatat waktu)** | **19,37** | 100% | **9,13** | 100% |

Dibanding fase yang sama pada Hybrid A\* (catatan 14 bagian 4, Windows): bagian jauh **A\* 21,35 ms vs model jauh 14,30 ms**, dan bagian dekat 0,45 vs 0,79 ms.

## 4. Makespan simulasi

**Cara mengukur:**
- Dipakai `drl.uji_navigasi.jalankan()` apa adanya: `navigasi.py` sungguhan tanpa jendela, dua robot simulasi (jeda kemudi dan kamera), wasit, pure pursuit, 20 Hz waktu nyata.
- Tujuan diberikan lewat `Unit.terima_tujuan()`, seperti klik.
- Skenario `uji_nyata/Gudang.json` S1 dan S2, derau pose 1,2 cm, 5 ulangan per skenario, perencana diselang-seling per ulangan, kode ver-1-5, laptop Windows.
- Waktu tempuh dihitung dari tujuan diberikan sampai robot selesai, **termasuk** waktu perencanaan. Makespan adalah waktu tempuh terlama dari kedua robot.

| | Hybrid A\* + RS | DQfD murni |
|---|---|---|
| S1 makespan (s) | 20,9 ± 4,5 (median 19,1; 17,0–27,3) | 19,6 ± 2,6 (median 19,3; 17,2–23,7) |
| S2 makespan (s) | 19,5 ± 8,0 (median 18,4; 12,2–32,6) | 14,9 ± 6,5 (median 11,7; 11,4–26,4) |
| **Semua (n = 10)** | **20,2 ± 6,2** (median 18,7) | **17,3 ± 5,3** (median 17,3) |
| Tiba / masuk zona | 20/20 / 0 | 20/20 / 0 |
| Rencana pertama di simulasi, rata-rata | 42 ms | 51 ms |
| Robot yang merencanakan ulang | 1 | 1 |
| Galat akhir rata-rata | 3,5 cm / 5,7° | 3,5 cm / 4,4° |
| Jarak terdekat ke zona / antar-robot | 8,2 / 8,3 cm | 8,3 / 8,4 cm |

Makespan per ulangan (s):

| Skenario | Hybrid A\* + RS | DQfD murni |
|---|---|---|
| S1 | 17,3 · 17,0 · 19,1 · 27,3 · 24,0 | 19,3 · 20,5 · 17,2 · 23,7 · 17,5 |
| S2 | 32,6 · 12,2 · 14,1 · 18,4 · 20,4 | 11,7 · 26,4 · 11,5 · 11,4 · 13,7 |

Pembanding, uji robot nyata Gudang 2026-10-24 (catatan 09): Hybrid A\* **21,7 ± 11,4 s** (n = 6), DQfD **19,0 ± 10,3 s** (n = 5). Uji itu memakai kode lama, tetapi jalur ver-1-5 identik dengan jalurnya, jadi makespan tidak berubah. Arahnya sejalan dengan simulasi.

**Cara membaca tabel ini:**
- **Waktu proses tidak menentukan makespan.** Rencana hanya 25–77 ms, dibanding makespan 11–33 s (0,2–0,4%). Percepatan numba (±15 ms per rencana) tidak terlihat pada makespan.
- **Sebarannya didominasi interaksi antar-robot dan derau.** Jalan terlama A\* di S2 (32,6 s) dan jalan terlama DQfD di S2 (26,4 s, satu robot merencanakan ulang) jauh di atas median. Median S1 hampir sama (19,1 vs 19,3 s). Di S2, median DQfD lebih rendah (11,7 vs 18,4 s), sejalan dengan [[14 Waktu Komputasi per Tahapan Hybrid A-star RS vs DQfD Gudang|catatan 14]] bagian 6: bagian model dekat ditempuh lebih singkat daripada bagian RS.
- **Waktu rencana di simulasi lebih besar daripada offline** (DQfD S1 ±77 ms vs 27,6 ms offline). Perencana berbagi CPU dengan loop simulasi 20 Hz, seperti saat uji robot nyata.

## 5. Kenapa Mac lebih cepat daripada Windows

| | Mac | Windows (laptop uji) |
|---|---|---|
| CPU | Apple **M1 Pro**: 6 core performa + 2 core efisiensi | AMD **Ryzen 7 260** (Hawk Point, **Zen 4**): 8 core identik / 16 thread, base 3,8 GHz |
| RAM | 16 GB | 16 GB |
| Python | 3.13.17, dikompilasi Clang | 3.14.3, dikompilasi MSVC |
| numpy / PyTorch | 2.5.4 / 2.15.0 (arm64, CPU) | 2.4.2 / 2.11.0+cu128 (build CUDA, dipakai di CPU) |
| Daya | tidak dicatat | charger, power plan *Performance* |

Microbenchmark yang sama di kedua mesin (satu thread, waktu terbaik dari 7 pengulangan):

| Jenis pekerjaan | Dipakai oleh | Mac | Windows | Windows ÷ Mac |
|---|---|---:|---:|---:|
| Loop Python murni, 10.000 iterasi (interpreter) | semua | 799 µs | 942 µs | **1,18×** |
| Heuristik Dijkstra (heapq, list Python) | A\* dan DQfD | 2,03 ms | 2,52 ms | 1,24× |
| Satu cek tabrakan `pose_margin` (Python + baca elemen numpy) | A\* dan DQfD | 1,19 µs | 1,83 µs | 1,54× |
| Operasi numpy larik 32: `a + b` | sinar numpy (lama) | 0,27 µs | 0,41 µs | 1,54× |
| Operasi numpy larik 32: `np.where` | sinar numpy (lama) | 0,87 µs | 1,80 µs | 2,07× |
| Sinar, satu panggilan, numpy (lama) | DQfD ver-1-4 | 220 µs | 366 µs | 1,67× |
| Sinar, satu panggilan, numba (termasuk ongkos panggil) | DQfD ver-1-5 | 1,39 µs | 2,61 µs | 1,88× |
| Forward jaringan jauh (PyTorch, 1 thread) | DQfD | 35 µs | 89 µs | **2,52×** |

- **Kecepatan dasar interpreter hampir sama** (1,18×). Satu core M1 Pro dan satu core Zen 4 laptop sama-sama cepat untuk Python murni.
- **Selisih besar muncul saat memanggil pustaka C**: numpy 1,5–2,1×, PyTorch 2,5×, dan ongkos panggil numba 1,9×. Pekerjaan per pemanggilan di sini sangat kecil (32 angka, satu matriks 47 × 256), jadi waktunya didominasi overhead pemanggilan: konversi argumen, alokasi larik sementara, dan dispatch PyTorch. Overhead itu lebih besar di Windows. Kemungkinan penyebabnya (belum dipastikan satu per satu):
  - CPython dan numpy dikompilasi MSVC di Windows, bukan Clang;
  - alokator memori Windows lebih lambat untuk banyak alokasi kecil;
  - PyTorch build CUDA punya lapisan dispatch tambahan, walaupun dijalankan di CPU.
- **Karena itu rasio Windows ÷ Mac berbeda antar-perencana.** Hybrid A\* 25,0 / 15,1 = 1,66×, karena didominasi Python murni dan `pose_margin`. DQfD 17,4 / 8,6 = 2,0×, karena sekarang porsi PyTorch-nya besar (forward jauh 3,6 ms di Windows vs 1,0 ms di Mac).
- **Implikasinya:** mengganti forward PyTorch dengan numpy (perubahan 2) paling menguntungkan di Windows, dan di mesin itulah robot dijalankan.

## 6. Temuan dan langkah berikutnya

1. **Sinar tidak lagi menjadi bottleneck** (4% di Windows, 3% di Mac).
2. **Bottleneck DQfD sekarang** (Windows):
   - cek tabrakan calon primitif 20% dan kinematik 16%: keduanya kode yang sama dengan ekspansi Hybrid A\*;
   - forward PyTorch 19%;
   - Dijkstra 14%: juga sama dengan A\*.
3. **Bagian yang dipakai bersama tetap Python murni untuk kedua perencana.** Yang dikompilasi hanya sinar, dan sinar hanya ada di DQfD. Kalau nanti cek tabrakan, `_ekspansi`, dan Dijkstra juga dikompilasi, kedua perencana sama-sama dipercepat.
4. **Langkah berikutnya yang paling berdampak untuk DQfD di Windows** adalah forward dengan numpy (perubahan 2). Bobot dari file `.pt` dipakai apa adanya, jadi tidak perlu latih ulang. Hasil Q-nya berbeda sekitar 1e-6 dari PyTorch, sehingga jalur perlu diverifikasi di banyak rute.

> [!warning] Batasan
> - **Makespan simulasi hanya 5 ulangan per skenario.** Sebaran S2 besar karena interaksi antar-robot. Selisih DQfD vs A\* belum signifikan.
> - **Makespan robot nyata belum diukur ulang dengan ver-1-5.** Jalurnya identik dengan uji 2026-10-24, jadi makespan nyata diperkirakan sama. Yang berubah hanya waktu rencana (puluhan ms).
> - **Mac ver-1-4 diambil dari run terpisah**, tidak diselang-seling dengan ver-1-5.
> - **Penyebab overhead yang lebih besar di Windows** (MSVC, alokator, build CUDA PyTorch) adalah kemungkinan yang belum diuji satu per satu.

## 7. Sumber angka

| Angka | Sumber |
|---|---|
| Perubahan kode | `drl/lingkungan.py`: `_sinar_loop`, `_sinar_loop_numpy`, `SINAR_NUMBA`, `LingkunganRencana._sinar` (commit ver-1-5) |
| Bagian 2 (verifikasi) | skrip sementara di luar repo `verif_numba.py` (pose acak, 124 rute) dan `adaptor_demo --uji` |
| Bagian 3 (waktu proses) | skrip sementara `profil_dqfd.py` (sama dengan catatan 14). Windows: 3 run ver-1-5 + 2 run ver-1-4, diselang-seling. Mac: 1 run ver-1-5, ditambah run ver-1-4 dari catatan 14 |
| Bagian 4 (makespan simulasi) | skrip sementara `sim_gudang.py`, memanggil `drl.uji_navigasi.jalankan()`. Windows, 5 ulangan, derau 1,2 cm |
| Bagian 4 (makespan nyata) | `uji_nyata/20261024_073221_Gudang/hasil.jsonl` (catatan 09) |
| Bagian 5 (microbenchmark) | skrip sementara `bench_mesin.py`, dijalankan dengan kode ver-1-5 yang sama di kedua mesin |
