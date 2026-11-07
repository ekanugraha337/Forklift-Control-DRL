# Waktu proses per node (ver-1-6)

Tanggal: 2026-11-08 · Branch `ver-1-6`, commit `df1412e` · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]] · Terkait: [[15 Sinar Numba ver-1-5|Sinar Numba]], [[17 Forward Numpy ver-1-5|Forward Numpy]]

Catatan ini membandingkan waktu proses **per node** antara Hybrid A\* + RS standar dan DQfD model ID 01. Isinya juga rincian ke mana 0,36 ms per node DQfD habis, ukuran forward pass dan jumlah parameter, serta uji apakah Q sudah "tahu" aksi yang menabrak.

> [!summary] Ringkasan
> - **Per node:** Hybrid A\* **0,19 ms**, DQfD **0,36 ms** (±1,9×). Tetapi di rute sulit DQfD butuh jauh lebih sedikit node (p90 110 vs 388), jadi waktu per rute p90-nya hampir separuh A\*.
> - **63% waktu per node DQfD** habis untuk simulasi 14 primitif dengan cek tabrakan. Lebih dari separuhnya adalah hitung ulang setiap kali pencarian pindah cabang.
> - **Jaringan saraf hanya ±10%:** observasi 19 µs + forward pass 19 µs. Satu forward pass (numpy, satu keadaan) = **16,6 µs**, dan jaringannya punya **145.807 parameter**.
> - **Q tidak tahu tabrakan:** AUC 0,45. Penyebabnya, semua loss latih memakai mask. Jadi cek tabrakan tetap wajib.
> - **Tahap 1 (§10):** pencarian lazy menurunkan DQfD ke **0,156 ms per node**, di bawah A\* 0,192 ms. Waktu p90 per rute jadi 15,3 ms (A\* 68,0 ms). Model sama, tanpa latih ulang. **Lazy sekarang menjadi bawaan perencana robot**, sedangkan Hybrid A\* tetap standar.
> - **Rincian DQfD lazy (§11):** tidak ada lagi satu bagian yang dominan. Tiga terbesar adalah aritmetika 14 primitif untuk cek sel (18%), peta Dijkstra per rencana (15%), dan tembakan RS (13%). Jaringan saraf (observasi + forward) sekitar 19%.

---

## 1. Cara ukur

| Hal | Isi |
|---|---|
| Mesin | Mac, Apple M1 Pro, macOS 27.2 · Python 3.13.18, numpy 2.5.4 (Accelerate), numba 0.68.1, PyTorch 2.15.0 |
| Model | `model_dqn/id01_jauh.pt`, checkpoint terbaik langkah gradien 50.000 (skor 97,8), versi aksi 9 |
| Rute | 470 rute uji ID 01: Gudang 60, Gudang2 64, Kalibrasi 63, Rumah 57, Rumah2 69, sintetis 157 (11 peta). Sama untuk kedua perencana, termasuk margin dan badan robot lain sebagai rintangan |
| Perencana | `astar`: Hybrid A\* + RS standar (w = 2, RS tiap ekspansi ke-4, h = max(h_D, h_RS)) · `dqfd-murni`: model jauh + tembakan RS < 60 cm, tanpa gerbang mutu dan tanpa cadangan A\* |
| Node | A\*: 1 ekspansi · DQfD: 1 langkah pencarian (`env.step`). Dijumlah atas semua panggilan perencana dalam satu `Perencana.rencanakan()` (sapuan arah hadap, margin cadangan) |
| Waktu proses | Waktu CPU thread perencana (`info["cpu_s"]`). Tiap rute 3 ulangan, diambil median, setelah pemanasan |
| Rincian (§3–§5) | Jam dinding, 1 ulangan, pemanasan tidak dihitung |

## 2. Hasil utama

| | Hybrid A\* + RS | DQfD ID 01 |
|---|---:|---:|
| **Waktu proses per node** (total waktu / total node) | **0,192 ms** | **0,353 ms** |
| Kemiringan regresi waktu rute terhadap node | 0,189 ms (+0,88 ms tetap) | 0,360 ms (−0,57 ms tetap) |
| Total node, 470 rute | 153.911 | 40.540 |
| Node per rute, median / p90 | 4 / 388 | 16 / 110 |
| Waktu proses per rute, median / p90 | 3,4 / 68,0 ms | 5,8 / 36,0 ms |
| Tiba | 99,6% | 100% |
| *Median "ms per node" tiap rute (menyesatkan)* | *0,643 ms* | *0,388 ms* |

**Membaca hasil:**
- Satu node DQfD ±1,9× lebih mahal daripada satu ekspansi A\*.
- Di rute sulit, DQfD butuh jauh lebih sedikit node, sehingga waktu p90 per rutenya hampir separuh A\*.
- Di rute mudah, A\* lebih cepat karena tembakan RS di ekspansi ke-4 sering langsung berhasil. Median rute A\* hanya 4 node.

> [!warning] Jangan pakai median "ms per node" tiap rute
> Angka itu terbalik (A\* 0,64 vs DQfD 0,39 ms) karena biaya tetap tiap rencana dibagi ke node yang sangat sedikit, misalnya peta Dijkstra 2,2 ms per rencana. Rute A\* median hanya 4 node. Ukuran per node yang benar adalah total waktu / total node, atau kemiringan regresi.

## 3. Rincian 0,36 ms per node DQfD

Pembagian ini **eksklusif**: setiap bagian dihitung sekali dan jumlahnya sama dengan total. Pencatat waktu per bagian menambah ±2%, jadi totalnya 0,367 ms. Tanpa pencatat bagian, totalnya 0,359 ms.

| Bagian | ms per node | % | Panggilan per node | µs per panggilan |
|---|---:|---:|---:|---:|
| **A. Pencarian terbaik-dulu** | | | | |
| Simulasi 14 primitif + cek tabrakan, sesudah langkah | 0,103 | 28,1 | 0,93 | 111 |
| Simulasi 14 primitif + cek tabrakan, saat pindah cabang | 0,130 | 35,4 | 1,15 | 113 |
| Tembakan RS (di dalam 60 cm) | 0,023 | 6,3 | 0,086 | 270 |
| Observasi 47 angka | 0,018 | 4,9 | 0,94 | 19 |
| Forward pass jaringan (numpy) | 0,018 | 4,9 | 0,92 | 19 |
| Buka simpul: f = g + c + w·h, masuk heap | 0,009 | 2,5 | 0,92 | 10 |
| Langkah: pose baru, sel dikunjungi, biaya | 0,005 | 1,3 | 0,94 | 5 |
| Pulihkan: susun jalur + keadaan | 0,004 | 1,2 | 1,15 | 4 |
| Ambil dari heap, cek mask, simpan keadaan, saringan serah, sisa loop | 0,008 | 2,1 | | |
| **B. Backtracking** setelah terbaik-dulu gagal (5,8% node) | 0,017 | 4,7 | 0,058 | 301 |
| **C. Biaya tetap per rencana**, dibagi rata ke node | 0,031 | 8,5 | 0,012 | |
| ↳ peta Dijkstra h_D | 0,026 | 7,1 | 0,012 | 2.166 |
| ↳ reset, forward V(s0), peta rintangan sementara, Perencana | 0,005 | 1,4 | | |
| **Total** | **0,367** | 100 | | |

**Observasi (19,3 µs per panggilan)** terdiri dari 32 sinar numba 10,9 µs, arah turun h_D 2,7 µs, dan sisanya 5,8 µs (pose relatif tujuan, h_D, margin).

## 4. Isi simulasi calon (`_hitung_calon`)

Satu panggilan (112 µs) menjalankan **14 primitif × 4 sub-langkah** dengan model kinematik Ackermann.
- Di tiap sub-langkah, **4 lingkaran badan** (termasuk safety margin) dicek terhadap peta jarak lewat `pose_margin`.
- Rata-rata **54,6 cek pose** per panggilan, masing-masing 1,71 µs, jadi totalnya **93 µs**. Rata-rata 1 primitif berhenti lebih awal karena menabrak.
- Sisanya, ±19 µs, untuk aritmetika gerak, biaya c(a), dan cek sel yang sudah dikunjungi.
- Aritmetikanya salinan persis loop ekspansi Hybrid A\*. A\* juga menjalankan 14 primitif yang sama setiap ekspansi.
- Cek tabrakan ditulis dalam **Python murni**. Bandingkan dengan 32 sinar: walau tiap sinar sampai 256 iterasi, semuanya hanya 11 µs karena memakai **numba**.

## 5. Forward pass dan jumlah parameter

**Satu forward pass (numpy float32, satu keadaan): 16,6 µs (0,017 ms).**
- Angka ini median dari 2.000 observasi nyata × 10, dengan p10–p90 16,2–17,0 µs.
- Di dalam pencarian angkanya 19,4 µs, karena bobot tidak selalu ada di cache.
- Sebagai pembanding, PyTorch 2.15 (6 thread) butuh 38,0 µs, 2,3× lebih lambat. Selisih Q numpy vs PyTorch paling besar 4,8·10⁻⁶.

| Bagian forward (median) | µs |
|---|---:|
| Ubah ke float32 | 0,25 |
| FC1 47 × 256 + ReLU | 2,71 |
| FC2 256 × 256 + ReLU | 3,54 |
| V\|A-hidden 256 × 256 + ReLU (digabung) | 3,63 |
| Keluaran 256 × 15 (V dan A, blok digabung) | 2,50 |
| Q = V + A − rata(A) | 4,75 |

Langkah terakhir hanya mengolah 14 angka, tetapi justru paling mahal. Artinya waktunya didominasi overhead tiap operasi numpy (±2 µs per operasi), bukan perkalian matriksnya.

**Satu forward pass menghasilkan Q untuk ke-14 aksi sekaligus.** Jaringan menghitung Q(s, a) dari observasi keadaan induk. Kalau jaringan menilai tiap keadaan penerus dengan V(s′), seperti heuristik A\* menilai h(s′), per simpul dibutuhkan 14 observasi dan 14 forward pass. Cek tabrakan sama sekali tidak memakai jaringan.

**Jumlah parameter: 145.807** (145.024 bobot + 783 bias), dalam float32 = 570 KiB.

| Lapis | Ukuran | Bobot | Bias | Jumlah |
|---|---|---:|---:|---:|
| FC1 | 47 → 256 | 12.032 | 256 | 12.288 |
| FC2 | 256 → 256 | 65.536 | 256 | 65.792 |
| V-hidden | 256 → 128 | 32.768 | 128 | 32.896 |
| V | 128 → 1 | 128 | 1 | 129 |
| A-hidden | 256 → 128 | 32.768 | 128 | 32.896 |
| A | 128 → 14 | 1.792 | 14 | 1.806 |
| **Total** | | **145.024** | **783** | **145.807** |

Satu forward pass berisi ±145 ribu perkalian-tambah (≈0,29 MFLOP). Versi numpy menghitung 146.944 karena blok nol di matriks keluaran yang digabung.

## 6. Apakah Q tahu aksi yang menabrak?

Uji ini mencatat Q ke-14 aksi **tanpa mask** di 37.431 ekspansi terbaik-dulu (470 rute uji ID 01), lalu membandingkannya dengan status sebenarnya tiap aksi.

| Hasil | Nilai |
|---|---:|
| Status semua aksi: sah / menabrak / sel sudah dikunjungi | 72,4% / 7,4% / 20,2% |
| Ekspansi dengan minimal 1 aksi menabrak | 26,5% |
| Di ekspansi itu, aksi dengan Q tertinggi ternyata menabrak | **25,1%** (memilih acak: 27,8%) |
| Q tertinggi menabrak, dari semua ekspansi | 6,65% |
| AUC Q aksi sah > Q aksi menabrak (0,5 = tidak membedakan) | **0,45** |

| Margin pose sekarang | Porsi ekspansi | Ada aksi menabrak | Q tertinggi menabrak |
|---|---:|---:|---:|
| ≤ 2 sel | 8,4% | 97,7% | 43,6% |
| 2–6 sel | 18,1% | 72,5% | 14,4% |
| 6–12 sel | 25,0% | 20,5% | 1,5% |
| > 12 sel | 48,5% | 0% | 0% |

**Penyebabnya ada di cara latih, bukan di 32 sinar.**
- Semua loss memakai mask. J_DQ dan J_n hanya dihitung untuk aksi yang diambil, yang selalu sah. Max di J_E dan argmax di target hanya atas aksi sah (`bermask` di `drl/dqfd.py`).
- Akibatnya Q aksi yang menabrak tidak pernah mendapat gradien.
- Sinar membantu jaringan **mengurutkan aksi-aksi yang sah**, bukan melarang aksi yang tidak sah.
- Dugaan (belum dicek): aksi yang menabrak sering mengarah lurus ke tujuan menembus rintangan, sehingga Q tebakannya tinggi dan AUC-nya di bawah 0,5.

**Akibatnya:**
- **Cek tabrakan tetap wajib,** karena robot tidak membawa sensor dan jalur harus dijamin bebas tabrakan.
- **Banyak cek sebenarnya sia-sia:** di 73,5% ekspansi tidak ada satu pun primitif yang menabrak.

## 7. Tahapan: Hybrid A\*, DQfD sekarang (eager), DQfD lazy (usulan)

Kolom "DQfD lazy" adalah **usulan, belum ada di kode**.

**Per simpul (loop pencarian)**

| Tahap | Hybrid A\* (pembanding) | DQfD sekarang (eager) | DQfD lazy (usulan) |
|---|---|---|---|
| Isi heap | Simpul: pose hasil primitif, dengan g dan h sendiri | Pasangan (simpul, aksi); keadaan penerusnya belum dibuat | Sama dengan eager |
| Ambil dari heap | Simpul dengan f terkecil | Pasangan dengan f terkecil | Sama |
| Pulihkan konteks | Tidak perlu; setiap simpul menyimpan pose, g, dan induknya | Bila dari cabang lain: pulihkan keadaan, susun ulang jalur, **hitung ulang 14 primitif** untuk memperbarui mask | Pulihkan keadaan dan jalur saja |
| Validasi gerakan terpilih | Sudah sah sejak dibangkitkan | Sudah sah lewat mask; cek sel belum dimasuki cabang lain | Cek sel, lalu **simulasi 1 primitif + cek tabrakan + biaya tepat**. Menabrak → dibuang. f tepat > puncak heap → masuk ulang |
| Tutup sel | Key (x, y, yaw) ditutup saat simpul diambil | (x, y, yaw, arah gerak) ditutup saat dimasuki | Sama dengan eager |
| Coba selesai | Tiba di tujuan, atau tembakan RS di ekspansi ke-4, 8, 12, … pada jarak berapa pun | Tembakan RS di setiap langkah yang berakhir < 60 cm dari tujuan, bila peta Dijkstra tidak memaksa memutar (atau sudah < 25 cm) | Sama dengan eager |
| Bangkitkan calon | 14 primitif × 4 sub-langkah dengan cek tabrakan | Sama, untuk keadaan baru | Pose ujung 14 aksi saja, **tanpa cek tabrakan** |
| Observasi | Tidak ada | 47 angka: 32 sinar, h_D dan arah turunnya, margin, pose relatif tujuan | Sama |
| Penilai h | h = max(h_D, h_RS) per penerus, lewat lookup peta Dijkstra + tabel RS | 1 forward pass → 14 Q; h = max(0, −100·Q − c) | Sama, tetapi c sementara tanpa penalti jarak-bebas |
| f dan masuk heap | f = g + w·h untuk penerus sah yang g-nya lebih baik (best_g) | f = g + c + w·h untuk aksi sah; tanpa best_g | f sementara untuk semua aksi yang selnya belum dimasuki |

**Per rencana**

| Tahap | Hybrid A\* | DQfD eager | DQfD lazy |
|---|---|---|---|
| Persiapan | Cek tujuan, peta Dijkstra h_D, arah datang, lalu simpul awal masuk heap | Sama, ditambah cek serah di pose awal, 14 primitif awal, observasi awal, forward V(s0) | Sama, tetapi 14 primitif awal tanpa cek tabrakan |
| Bila belum ketemu | Lanjut sampai batas waktu atau iterasi habis | Terbaik-dulu dibatasi 75% waktu dan 1.200 langkah, lalu reset dan backtracking 600 langkah | Sama; backtracking juga memvalidasi aksi saat dicoba |
| Penutup | Rantai induk + ekor RS | Pose langkah + ekor RS. Mode `dqfd`: gerbang mutu 1,3 × perkiraan V, kalau gagal dipakai A\* | Sama |

**Jumlah operasi per simpul**

| | Hybrid A\* | DQfD eager (terukur) | DQfD lazy (perkiraan) |
|---|---|---|---|
| Simulasi primitif dengan cek tabrakan | 14 | ±29 (14 × 2,08) | ±1–2 |
| Penilaian arah | Hingga 14 lookup h_D + h_RS | 1 observasi + 1 forward pass | Sama |
| Tembakan RS | 1 tiap 4 ekspansi | 0,086, hanya < 60 cm | Sama |

**Perbedaan A\* vs DQfD yang berlaku di kedua mode:**
- Sel ditutup per arah gerak, bukan per sel saja.
- Tidak ada best_g.
- Tembakan RS hanya < 60 cm. Ini meniru demonstrator, bukan A\* pembanding.
- Ada tahap backtracking dan gerbang mutu.

## 8. Opsi perbaikan

Opsi 3 dan 4 sudah dicoba di tahap 1 (§10). Yang lain belum dikerjakan.

| # | Opsi | Jalur hasil | Perkiraan per node | Catatan |
|---|---|---|---|---|
| 1 | Biarkan eager, sama dengan A\* | sama | 0,36 ms | Cukup dijelaskan di tesis |
| 2 | Simpan calon per simpul; saat pindah cabang hanya saring ulang sel yang dikunjungi | identik | ±0,23 ms | Hanya DQfD |
| 3 | Mode lazy (§7) | berubah | ±0,17 ms | Perlu validasi ulang. Kalau hanya DQfD yang lazy, sebagian selisih berasal dari teknik implementasi |
| 4 | Lewati cek tabrakan di ruang terbuka: bila margin pose > gerak terjauh badan dalam 1 primitif + batas penalti jarak-bebas | identik | belum diukur | Berlaku juga untuk A\*. Margin > 12 sel = 48,5% ekspansi tanpa tabrakan |
| 5 | Kernel numba untuk simulasi + cek tabrakan, dipakai A\* dan DQfD | perlu dicek | belum diukur | Trigonometri numba bisa beda 1 ulp dari Python |
| 6 | Latih sadar tabrakan | berubah | — | Lihat di bawah |

**Opsi 6, latih ulang agar Q sadar tabrakan.** Label tabrakan ke-14 aksi sudah dihitung `_hitung_calon` saat membuat demo dan rollout, tinggal dipakai di loss. Ada dua cara:
- **Struktur tetap (145.807 parameter):** tambah loss yang menekan Q aksi menabrak di bawah Q aksi sah terendah, dengan margin.
- **Kepala kelayakan:** 14 keluaran sigmoid P(menabrak) dengan binary cross-entropy. Tambahannya 1.806 parameter (dari A-hidden 128) atau 3.598 parameter (dari FC2 256). Q tetap murni biaya-sisa, dan mutu prediksi bisa diukur langsung. Ini cara yang disarankan.

Dua cara itu sama-sama perlu versi kontrak baru, demo ulang, dan latih ulang kedua robot. **Cek tetap tidak bisa dihapus seluruhnya:**
- Verifikasi 1 primitif terpilih tetap perlu demi keselamatan.
- Cek sel yang sudah dikunjungi juga tetap perlu, karena itu keadaan pencarian yang tidak terlihat oleh jaringan.

Yang hilang adalah simulasi 14 primitif di muka.

## 9. Data dan skrip

- **Hasil di catatan ini:** `Forklift Robot Program/model_dqn/ukur_node_id01_mac_ver-1-6.json`. File ini hanya ada di Mac karena `model_dqn/` di-ignore git. Kuncinya:
  - `ringkasan`, `rute`, `regresi_cpu_ms_per_rute` (§2)
  - `rincian_dqfd_ms_per_node` (tanpa pencatat bagian)
  - `detail_dqfd` (§3–§5)
  - `uji_q_tabrak` (§6)
- **Skrip:** `Forklift Robot Program/drl/ukur_node.py`, dijalankan dari folder `Forklift Robot Program`:

```
python -m drl.ukur_node --id 1                                  # semua uji berurutan (beberapa menit di Mac)
python -m drl.ukur_node --id 1 --uji waktu --pencarian eager    # §2
python -m drl.ukur_node --id 1 --uji rincian --pencarian eager  # §3–§5 (kunci detail_dqfd)
python -m drl.ukur_node --id 1 --uji tabrak                     # §6 (selalu eager)
python -m drl.ukur_node --id 1 --uji varian                     # §10
python -m drl.ukur_node --id 1 --uji rincian                    # §11, lazy seperti robot (kunci detail_dqfd_lazy)
python -m drl.ukur_node --id 1 --maks 3                         # uji cepat: 3 rute pertama tiap peta
```

  - Hasil ulang ditulis ke `model_dqn/ukur_node_id01_mac.json`, atau `_windows.json` di laptop robot.
  - Uji rincian dan tabrak memakai salinan bertanda `rencanakan_terbaik`. Skrip memperingatkan bila fungsi aslinya berubah.
  - Jangan jalankan bersamaan dengan pelatihan, karena yang diukur adalah waktu.
- Pelengkap: `drl/banding_komputasi.py` mengukur waktu per **rute**.
- Tahap 1 (§10): kunci `uji_varian` di file JSON yang sama, dibuat dengan `python -m drl.ukur_node --id 1 --uji varian`.

## 10. Tahap 1: pencarian lazy dan jalan pintas ruang terbuka

Dikerjakan 2026-11-08 dengan model ID 01 yang sama, tanpa latih ulang. Setelah hasilnya keluar, **lazy dijadikan bawaan perencana robot** (`DQfDPlanner`, `PENCARIAN_ROBOT = "lazy"`), untuk mode `dqfd` maupun `dqfd-murni`.
- Validasi latih (`latih.py`) dan `evaluasi.py` tetap eager (`PENCARIAN_BAWAAN`), supaya skor model yang sudah dilatih tetap sebanding.
- Jalan pintas ruang terbuka tetap mati, karena di mode lazy hampir tanpa efek.
- Hybrid A\* tetap standar.

**Perubahan kode**

| File | Isi |
|---|---|
| `drl/lingkungan.py` | `_terbuka()`: ruang terbuka bila margin pose ≥ **17,2 sel** = clearance_pref 6,1 + 1,05 × (gerak maks 9,19 + √2). `_ekspansi(cek=False)`: aritmetika gerak saja. `calon_malas()`, `siapkan_aksi()`. `step(…, hitung_calon=False)` dan `pulihkan_keadaan(…, hitung_calon=False)`. Atribut `pintas_terbuka` (bawaan mati). Jalan pintas juga berlaku di `_hitung_calon` |
| `drl/perencana_dqfd.py` | `rencanakan_terbaik_malas()` (lazy, dengan aturan masuk ulang ala Lazy Weighted A\*). `cari(…, pencarian=)`. `DQfDPlanner(…, pencarian="eager"/"lazy", pintas=)`. `PENCARIAN_ROBOT = "lazy"` (robot), `PENCARIAN_BAWAAN = "eager"` (validasi latih dan evaluasi). Tahap backtracking tetap eager |
| `drl/ukur_node.py` | `--uji varian`; `--pencarian eager/lazy` untuk uji waktu dan rincian (bawaan seperti robot) |

**Kenapa ambangnya aman:**
- Pusat lingkaran badan bergeser paling jauh 7,3 × √(1 + (19,62 × 0,039)²) = 9,19 sel dalam satu primitif.
- Nilai peta jarak chamfer turun paling banyak 1,02 × (geser + √2). Angka √2 berasal dari pembulatan ke piksel, dan 1,05 dipakai sebagai sisa aman.
- Jadi bila margin sekarang ≥ 17,2 sel, setiap sub-titik bermargin ≥ 6,1 sel: tidak menabrak, dan penalti jarak-bebasnya tepat 0.

**Verifikasi**
- `python -m drl.adaptor_demo --uji --id 1 --n 40` **lulus**, dengan maupun tanpa jalan pintas. Jalur env tetap identik dengan jalur A\*.
- Selama uji varian, setiap kali keadaan dinyatakan terbuka, ke-14 primitif juga disimulasi lengkap. Hasilnya **587.874 pemeriksaan (395.878 eager + 191.996 lazy), 0 selisih.**

**Hasil** (470 rute uji ID 01, waktu CPU thread, median 3 ulangan; kolom A\* dari §2)

| | Eager (sekarang) | Eager + pintas | **Lazy** | Lazy + pintas | Hybrid A\* |
|---|---:|---:|---:|---:|---:|
| Waktu proses per node | 0,356 ms | 0,300 ms | **0,156 ms** | 0,156 ms | 0,192 ms |
| Simulasi primitif + cek tabrakan per node | 30,3 | 20,6 | **2,4** | 2,0 | ≤ 14 per ekspansi |
| Cek pose tabrakan per node | 118 | 81 | **9,0** | 8,2 | |
| Aritmetika gerak saja per node | 0 | 9,8 | 15,0 | 15,1 | |
| Aksi dibuang per node (menabrak / sel sudah dimasuki) | 0 | 0 | 0,88 | 0,88 | |
| Masuk ulang ke heap | | | 0 | 0 | |
| Porsi keadaan ruang terbuka | | 32,2% | | 32,9% | |
| Total node | 40.540 | 40.540 | 40.441 | 40.441 | 153.911 |
| Waktu proses per rute, median / p90 | 5,8 / 35,7 ms | 5,2 / 30,4 ms | **4,3 / 15,3 ms** | 4,3 / 15,4 ms | 3,4 / 68,0 ms |
| Tiba | 100% | 100% | 100% | 100% | 99,6% |
| Jalur identik dengan eager | | 100% | 97,0% | 97,0% | |

**Membaca hasil:**
- **Lazy memangkas waktu per node 56%** (0,356 → 0,156 ms), sekarang 0,81× A\*. Simulasi dengan cek tabrakan turun dari 30,3 ke 2,4 per node.
- Waktu p90 per rute DQfD jadi 15,3 ms, 4,4× lebih cepat dari A\*. Di rute mudah A\* masih sedikit lebih cepat: median 3,4 vs 4,3 ms.
- **Jalan pintas memberi hasil identik dan mempercepat eager 16%,** tetapi di mode lazy hampir tanpa efek karena simulasinya sudah tinggal sekitar 2 per node.
- **Lazy mengubah jalur di 14 dari 470 rute (3,0%).**
  - 7 rute lebih murah lebih dari 1% (rasio terendah 0,765).
  - 4 rute lebih mahal lebih dari 1% (rasio tertinggi 1,405).
  - Sisanya selisih biaya di bawah 1%, dengan median rasio 1,000.
  - Penyebabnya, c(a) sementara tidak memuat penalti jarak-bebas, sehingga urutan primitif yang dekat zona sedikit berubah.
  - Masuk ulang ke heap tidak pernah terjadi. Dengan w = 2 dan h > 0, f = g − 200·Q − c, jadi f tepat ≤ f sementara.
- **Latih ulang sadar tabrakan (tahap 2) kini hanya sedikit menghemat waktu.** Aksi dibuang hanya 0,88 per node, masing-masing paling banyak satu simulasi primitif (±8 µs). Jadi penghematannya paling banyak ±7 µs per node, sekitar 5%.
- **Kesetaraan:** lazy dan jalan pintas juga bisa dipakai Hybrid A\*. Kolom A\* di atas masih A\* standar (eager).

## 11. Rincian waktu per node DQfD lazy (bawaan robot)

Cara ukurnya sama dengan §3: jam dinding, eksklusif, pemanasan tidak dihitung, 470 rute uji ID 01, 40.441 node.
- Total dengan pencatat **0,178 ms per node**, sedangkan tanpa pencatat 0,156 ms (§10).
- Pencatat di mode ini lebih berat (±14%) karena setiap `_ekspansi` ikut dibungkus, ±17 kali per node. Kelebihannya paling banyak masuk ke baris "calon lazy" dan "aksi diambil".

| Bagian | ms per node | % | Panggilan per node | µs per panggilan |
|---|---:|---:|---:|---:|
| **A. Pencarian terbaik-dulu lazy** | | | | |
| Calon lazy: aritmetika 14 primitif + cek sel (`calon_malas`) | **0,031** | **17,6** | 0,94 | 33 |
| Tembakan RS (di dalam 60 cm) | 0,023 | 13,0 | 0,087 | 267 |
| Observasi 47 angka | 0,017 | 9,7 | 0,94 | 18 |
| Forward pass jaringan (numpy) | 0,017 | 9,6 | 0,93 | 18 |
| Aksi diambil: simulasi 1 primitif + cek tabrakan | 0,010 | 5,4 | 1,16 | 8,3 |
| Aksi diambil: aritmetika gerak + cek sel, sisa | 0,006 | 3,5 | 1,82 | 3,5 |
| Buka simpul: f dan heap | 0,007 | 4,1 | 0,94 | 7,8 |
| Pulihkan, langkah, heap, simpan, f tepat, saringan serah, sisa loop | 0,016 | 9,3 | | |
| **B. Backtracking (eager)** setelah terbaik-dulu gagal (5,8% node) | 0,018 | 10,0 | 0,058 | 307 |
| **C. Biaya tetap per rencana**, dibagi rata ke node | 0,031 | 17,6 | 0,012 | |
| ↳ peta Dijkstra h_D | **0,026** | **14,5** | 0,012 | 2.155 |
| ↳ reset, forward V(s0), peta rintangan sementara, Perencana | 0,005 | 3,1 | | |
| **Total** | **0,178** | 100 | | |

Observasi 18,5 µs terdiri dari 32 sinar 10,1 µs, arah turun h_D 2,7 µs, dan sisanya 5,6 µs. Forward pass satu keadaan 16,3 µs (PyTorch 38,8 µs).

**Dibanding eager (§3):**
- Simulasi calon (0,233 ms, 63%) hilang. Gantinya aritmetika 14 primitif per simpul (0,031 ms) ditambah simulasi 1,16 primitif yang benar-benar diambil (0,010 ms). Dari aksi yang diambil, 0,22 per node ternyata menabrak, dan total yang dibuang 0,88 per node.
- **Tidak ada lagi satu bagian yang dominan.** Tiga terbesar:
  - aritmetika calon untuk cek sel (18%)
  - peta Dijkstra per rencana (15%)
  - tembakan RS (13%)
- Jaringan saraf (observasi + forward) sekitar 19%.
- Peta Dijkstra dan tembakan RS memakai fungsi yang sama persis dengan Hybrid A\*, jadi keduanya juga ditanggung A\*.

**Bila masih ingin lebih cepat:**
- Aritmetika calon bisa dikompilasi numba (14 primitif × 4 sub-langkah), tetapi kesamaan bit dengan Python harus dicek.
- Backtracking bisa dibuat lazy juga: 307 µs per langkah untuk hanya 5,8% node.
- Peta Dijkstra bisa disimpan untuk tujuan dan peta yang sama, misalnya antar sapuan arah hadap. Ini berlaku juga untuk A\*.
