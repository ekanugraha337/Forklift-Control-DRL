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
> - **Lapis c (§12):** ID 01 dilatih ulang dengan kepala P(menabrak). Kepala itu jelas belajar dari 32 sinar: AUC 0,985, sedangkan Q 0,51.
>   - Mode A (lazy + verifikasi): **0,143 ms per node**, 0 jalur menabrak.
>   - Mode B (tanpa cek tabrakan primitif): 0,113–0,115 ms per node, tetapi 3,5–18,6% jalurnya menabrak, tergantung ambang. Verifikasi geometris tetap diperlukan.

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

## 12. Lapis c: kepala kelayakan P(menabrak), ID 01 dilatih ulang

Dikerjakan 2026-11-08, commit `1f6edaf`. Tujuannya menguji hipotesis bahwa 32 sinar sepanjang 1 m ditambah posisi tujuan sudah cukup bagi jaringan untuk mengenali aksi yang menabrak, asalkan diberi sinyal latih.

**Perubahan.** Struktur jaringan dan cara latihnya tetap seperti §5, ditambah:
- Kepala **T**: A-hidden 128 → 14 logit → sigmoid = P(aksi menabrak). Tambahannya **1.806 parameter**, jadi totalnya 147.613.
- Loss **J_T**: binary cross-entropy atas ke-14 aksi di setiap keadaan buffer.
- Label tabrakan berasal dari `_hitung_calon`: badan plus safety margin menabrak zona di salah satu dari 4 sub-langkah primitif. Aksi ke sel yang sudah dikunjungi bukan tabrakan.
- Label demo dibuat oleh `drl/label_tabrak.py` dengan memutar ulang episode, jadi demo A\* tidak dibangkitkan ulang. Di laptop robot pemutaran ulang cocok persis: 0 mask beda, 0 observasi beda. Hasilnya 8,0% aksi menabrak, dan 31,4% keadaan punya minimal satu aksi yang menabrak.
- Versi kontrak tetap 9. Model ID 02 tanpa kepala tetap terbaca dan tetap dipakai.

**Pelatihan.** Laptop robot, RTX 5050, 24,2 menit. Model lama dicadangkan di `Documents\GitHub\_arsip_model_id01_tanpa_kepala_20261108` (laptop) dan `model_dqn/arsip_id01_tanpa_kepala_20261108/` (Mac).

| Validasi (checkpoint terbaik) | Model lama | Model baru |
|---|---:|---:|
| Langkah gradien | 50.000 | 75.000 |
| Skor | 97,8 | **98,0** |
| Tiba | 99,6% | 99,6% |
| Rasio biaya median / p90 | 0,99 / 1,18 | 1,00 / 1,16 |
| AUC tabrakan, keadaan demo validasi | – | 0,980 |

**Mutu kepala P pada ekspansi pencarian** (43.405 ekspansi eager, 470 rute uji)
- AUC global P = **0,985**. AUC dalam ekspansi yang sama = 0,938.
- AUC Q tetap 0,51, karena Q memang tidak dilatih soal tabrakan.

| Ambang P | Tabrakan terdeteksi | Alarm palsu (aksi sah dianggap menabrak) | Q tertinggi di antara aksi "aman" ternyata menabrak |
|---:|---:|---:|---:|
| 0,5 | 70,7% | 1,2% | 3,51% ekspansi |
| 0,3 | 81,9% | 2,5% | 2,59% |
| 0,2 | 87,4% | 3,7% | 1,92% |
| 0,1 | 93,3% | 6,1% | 1,11% |
| 0,05 | 96,4% | 8,8% | 0,65% |

**Mode A dan mode B** (Mac, waktu CPU, median 3 ulangan, 470 rute uji). Semua jalur dari pencarian lazy diputar ulang dengan cek tabrakan lengkap.
- **Mode A (`lazy`, bawaan robot):** aksi dengan P ≥ 0,5 ditaruh di urutan belakang heap, dan setiap aksi yang diambil dicek tabrakannya.
- **Mode B (`belajar`):** aksi dengan P ≥ ambang tidak dimasukkan, dan aksi yang diambil tidak dicek tabrakannya, tanpa backtracking. Tembakan RS tetap dicek.

| | Eager (model baru) | **Mode A** | Mode B, ambang 0,5 | Mode B, 0,3 | Mode B, 0,1 |
|---|---:|---:|---:|---:|---:|
| **Waktu proses per node** | 0,347 ms | **0,143 ms** | 0,115 ms | 0,113 ms | 0,113 ms |
| Simulasi primitif + cek tabrakan per node | 31,6 | 2,3 | 0,1* | 0,1* | 0,1* |
| Aksi dibuang per node | 0 | 0,78 | 0,79 | 0,80 | 0,84 |
| Total node | 47.777 | 49.543 | 68.755 | 70.837 | 72.240 |
| Node p90 per rute | 151 | 157 | 201 | 285 | 301 |
| Waktu per rute, median / p90 | 6,7 / 48,7 ms | **4,6 / 18,9 ms** | 4,5 / 25,5 ms | 4,5 / 25,6 ms | 4,6 / 28,4 ms |
| Tiba | 99,8% | 99,8% | 97,7% | 97,4% | 96,4% |
| **Jalur menabrak** (dari yang diperiksa ulang) | 0 | **0 / 453** | **85 / 458** | 56 / 457 | 16 / 451 |
| Tiba tanpa tabrakan | 99,8% | **99,8%** | 79,6% | 85,5% | 93,0% |
| Jalur identik dengan eager | 100% | 85,5% | 70,4% | 69,6% | 61,5% |
| Rute lebih mahal / lebih murah > 1% | | 5,3% / 7,2% | 4,0% / 6,1% | 7,0% / 9,5% | 14,9% / 16,9% |

\* Sisa simulasi dari `reset` (calon awal).

Sebagai pembanding: model lama dalam mode lazy 0,156 ms per node (§10), dan Hybrid A\* 0,192 ms per ekspansi (§2).

**Kesimpulan:**
- **Hipotesis sebagian terbukti.** Dengan sinyal latih, jaringan belajar mengenali aksi yang menabrak dari 32 sinar: AUC 0,985, padahal sebelumnya 0,45–0,51. Kualitas pencariannya juga tidak turun (skor 98,0).
- **Tetapi akurasi per aksi belum cukup untuk jalur yang aman.**
  - Satu jalur berisi puluhan keputusan, sehingga kesalahan kecil per aksi terakumulasi.
  - Di ambang 0,1 masih 3,5% jalur menabrak (16 dari 451), sementara alarm palsu sudah 6,1%. Pencarian jadi lebih panjang: node p90 naik dari 157 ke 301, dan biaya jalur lebih menyebar.
  - Mode B juga hanya sekitar 20% lebih cepat per node daripada mode A (0,113 vs 0,143 ms), karena verifikasi mode A memang tinggal 2,3 simulasi per node.
- **Mode A tetap bawaan robot.**
  - Mode A lebih cepat 8% per node daripada model lama (0,143 vs 0,156 ms).
  - Aksi yang dibuang turun dari 0,88 ke 0,78 per node.
  - Waktu p90 per rute 18,9 ms, sedangkan A\* 68,0 ms.
- **Catatan mode A:**
  - Jalur yang identik dengan eager turun ke 85,5% (model lama 97%), karena urutan di heap dipengaruhi P.
  - Ada satu rute dengan rasio biaya 7,1× terhadap eager, walau p90 rasionya tetap 1,00. Penyebab yang paling mungkin: aksi sah terbaik kena alarm palsu dan terdorong ke belakang. Ini perlu dicek, misalnya dengan penalti yang lebih lunak atau ambang urutan yang lebih tinggi.

**Data:** `model_dqn/ukur_node_id01_mac_kepala.json`, kunci `uji_varian_kepala` dan `uji_q_tabrak_kepala`. Perintah:

```
python -m drl.ukur_node --id 1 --uji varian --varian eager,lazy,belajar,belajar@0.3,belajar@0.1
python -m drl.ukur_node --id 1 --uji tabrak
```

## 13. Model P terpisah dan verifikasi selektif (Mac)

Dikerjakan 2026-11-08 di Mac, dengan model jauh ID 01 dari §12. Q tetap dari model jauh; yang diganti hanya sumber P.

**1. Keadaan pencarian.** `python -m drl.kelayakan --id 1 --kumpulkan`
- Pencarian lazy dan `belajar` dijalankan di 1.500 rute latih dan 470 rute uji.
- Setiap keadaan yang dibuka pencarian direkam, dan labelnya dihitung dengan simulasi lengkap ke-14 aksi.
- Hasilnya 277.450 keadaan dari rute latih dan 86.002 dari rute uji, dengan 8,4–9,0% aksi menabrak. Waktunya 90 detik.

**2. Model P terpisah.** `python -m drl.kelayakan --id 1 --latih`
- Struktur 47 → 256 → 256 → 14 (ReLU, sigmoid), 81.678 parameter.
- Masukannya 47 angka yang sama dengan model jauh.
- Supervised (bukan DRL): binary cross-entropy dengan bobot 3× untuk aksi menabrak, Adam, batch 2.048.
- Data latih: keadaan demo episode latih ditambah keadaan pencarian rute latih. Epoch dipilih dengan 10% rute latih yang disisihkan. Rute uji tidak dipakai untuk melatih maupun memilih epoch.
- Berhenti otomatis di epoch 83 (71 detik, CPU). File: `model_dqn/id01_kepala_p.pt`; versi 40 epoch di `id01_kepala_p_e40.pt`.
- Latih lebih lama hampir tidak membantu: dari 40 ke 83 epoch, AUC rute uji hanya naik dari 0,9942 ke 0,9951, dan model mulai sedikit overfit (rute latih 0,9968).

| Mutu P | Demo validasi | Pencarian rute latih | **Pencarian rute uji** |
|---|---:|---:|---:|
| Kepala bawaan t2: AUC / AUC dalam keadaan sama | 0,980 / 0,947 | 0,978 / 0,926 | 0,980 / 0,930 |
| Kepala bawaan: terdeteksi / alarm palsu @0,1 | 94,6 / 9,1% | 89,5 / 6,5% | 89,7 / 5,9% |
| **Model P terpisah: AUC / AUC dalam keadaan sama** | 0,992 / 0,975 | 0,997 / 0,987 | **0,995 / 0,978** |
| **Model P terpisah: terdeteksi / alarm palsu @0,1** | 98,5 / 8,6% | 99,5 / 5,6% | **98,0 / 5,0%** |
| Model P terpisah: terdeteksi / alarm palsu @0,02 | 99,7 / 15,9% | 99,9 / 10,6% | 99,4 / 9,3% |

Pergeseran distribusi dari keadaan demo ke keadaan pencarian memang ada, tetapi kecil. Peningkatan terbesar datang dari model terpisah yang dilatih dengan data pencarian.

**3. Mode A, mode B, dan verifikasi selektif dengan P terpisah.** `python -m drl.ukur_node --id 1 --uji varian --kepala-p model_dqn/id01_kepala_p.pt --varian …` (Mac, waktu CPU, median 3 ulangan, 470 rute uji; semua jalur lazy diputar ulang dengan cek tabrakan lengkap).
- **`selektif@x`**: lazy, tetapi aksi dengan P < x dijalankan tanpa cek tabrakan. Aksi lain tetap dicek, dan backtracking tetap ada.
- Waktu per node sudah termasuk forward model P (±10 µs per simpul).

| Varian | ms per node | Simulasi + cek per node | Tanpa cek per node | Node p90 | Waktu per rute, median / p90 | Tiba | **Jalur menabrak** | Tiba tanpa tabrakan | Identik dengan eager |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Eager | 0,341 | 31,6 | 0 | 151 | 6,6 / 47,9 ms | 99,8% | 0 | 99,8% | 100% |
| **Mode A (lazy)** | **0,145** | 2,26 | 0 | 163 | 4,7 / 20,1 ms | **99,8%** | **0 / 450** | **99,8%** | 81,3% |
| Mode B @0,5 | 0,122 | 0,10 | 1,82 | 200 | 4,6 / 22,2 ms | 97,2% | 19 / 456 | 93,2% | 74,0% |
| Mode B @0,2 | 0,120 | 0,10 | 1,85 | 250 | 4,6 / 26,0 ms | 96,8% | 2 / 454 | 96,4% | 67,7% |
| **Mode B @0,1** | **0,122** | 0,10 | 1,87 | 221 | 4,6 / 24,8 ms | 96,8% | **0 / 454** | 96,8% | 62,6% |
| Mode B @0,05 | 0,121 | 0,11 | 1,86 | 225 | 4,7 / 25,7 ms | 96,2% | 0 / 450 | 96,2% | 58,3% |
| Mode B @0,02 | 0,119 | 0,11 | 1,85 | 207 | 4,8 / 23,4 ms | 95,3% | 0 / 446 | 95,3% | 55,1% |
| Selektif @0,05 | 0,138 | 1,40 | 1,56 | 159 | 4,5 / 18,7 ms | 99,8% | 1 / 450 | 99,6% | 81,1% |
| **Selektif @0,02** | **0,140** | 1,43 | 1,50 | 159 | **4,5 / 18,3 ms** | **99,8%** | **0 / 450** | **99,8%** | 81,1% |
| Selektif @0,01 | 0,141 | 1,46 | 1,46 | 159 | 4,5 / 18,5 ms | 99,8% | 0 / 451 | 99,8% | 81,1% |

Pembanding dengan kepala bawaan (§12): mode B @0,1 menghasilkan 16 jalur menabrak, dan mode A 0,143 ms per node.

**Kesimpulan:**
- **Dengan model P terpisah, mode B pada ambang ≤ 0,1 menghasilkan 0 jalur menabrak** di sekitar 450 rute uji yang diperiksa ulang. Dengan kepala bawaan angkanya 16. Hipotesis bahwa 32 sinar ditambah posisi tujuan cukup untuk mendeteksi tabrakan jauh lebih kuat dengan model khusus.
  - Tetapi 0 dari 454 bukan jaminan. Batas atas 95% (aturan tiga) masih sekitar 0,7% jalur.
- **Harga mode B:**
  - Tiba turun dari 99,8% ke 96,8% karena tidak ada backtracking dan sebagian aksi aman ikut dibuang (alarm palsu ±5–7%).
  - Biaya jalur lebih menyebar: 13% rute lebih mahal dan 18% lebih murah > 1%, dengan rasio maksimum 1,79.
  - Waktu p90 per rute lebih lambat daripada mode A (24,8 vs 20,1 ms), walau per node 16% lebih cepat (0,122 vs 0,145 ms).
- **Verifikasi selektif @0,02 adalah yang paling seimbang.** Hasilnya 0 tabrakan, tiba 99,8%, 0,140 ms per node, dan p90 per rute 18,3 ms, yang terbaik dari semua varian. Penghematannya dibanding mode A kecil (±3% per node), karena verifikasi mode A sudah murah.
  - Di ambang 0,05 sudah muncul 1 jalur menabrak, jadi ambangnya sensitif.
- **Untuk robot, mode A tetap yang paling aman** (dijamin geometris). Pilihan lainnya dibahas di tesis sebagai temuan.
- **Rute dengan rasio biaya 7,1× terhadap eager masih muncul** di mode A dan selektif, dengan P mana pun. Jadi penyebabnya cara urutan lazy, bukan mutu P. Ini belum diselidiki.

**Data:** `drl/data/keadaan_cari_id01.npz` (keadaan pencarian), `model_dqn/id01_kepala_p.pt`, dan `model_dqn/ukur_node_id01_mac_kepala.json` (kunci `kelayakan_nilai`, `uji_varian_kepala_p`). Modulnya `drl/kelayakan.py`, dan mode `selektif` ada di `perencana_dqfd.py`.

## 14. Uji jangkauan sinar 3 m untuk model P

`python -m drl.kelayakan --id 1 --uji-jangkau 300 --epoch 80 --sabar 8` (Mac, 2026-11-08).
- Pencariannya sama dengan §13. Di setiap keadaan dihitung dua versi 32 sinar: jangkauan 100 cm (yang dipakai Q, seperti sekarang) dan 300 cm (sphere tracing 1.024 iterasi).
- Tiga model P dilatih dengan struktur 47/79 → 256 → 256 → 14, data, dan pengaturan yang sama. Data latih hanya keadaan pencarian: 253.833 dari rute latih; 23.617 keadaan disisihkan untuk memilih epoch.
- Penilaian di 86.002 keadaan pencarian rute uji.
- Catatan: 1 sel = 1 cm, dan arena 300 × 270 cm, jadi sinar 3 m praktis menjangkau seluruh arena.

| Masukan sinar | AUC | AUC dalam keadaan sama | Terdeteksi / alarm palsu @0,1 | @0,02 |
|---|---:|---:|---:|---:|
| 100 cm (sekarang), 47 masukan | 0,9941 | 0,9740 | 98,1 / 5,6% | 99,5 / 10,0% |
| 300 cm (pengganti), 47 masukan | 0,9923 | 0,9645 | 97,4 / 6,0% | 99,3 / 11,1% |
| 100 cm + 300 cm, 79 masukan | 0,9950 | 0,9768 | 97,8 / 4,7% | 99,4 / 8,7% |

Waktu 32 sinar per keadaan: 100 cm **7,7 µs**, 300 cm **9,2 µs**.

**Kesimpulan:**
- **Mengganti sinar menjadi 3 m memperburuk P.** Semua metriknya turun. Normalisasi 0–1 atas jangkauan 3 m memampatkan resolusi jarak dekat (10 cm menjadi 0,033, bukan 0,1), padahal tabrakan satu primitif hanya ditentukan oleh jarak sekitar 10 cm di sekitar badan.
- **Menambahkan sinar 3 m di samping sinar 1 m hanya membantu sangat sedikit:** AUC +0,001, dan alarm palsu −0,9 poin di ambang 0,1. Harganya 32 masukan tambahan dan ±9 µs per node. Kalau dipakai untuk Q juga, model jauh harus dilatih ulang. Tidak sepadan.
- **Kalau akurasi P masih ingin dinaikkan,** yang lebih masuk akal adalah resolusi di dekat badan: sinar lebih rapat, sinar diukur dari badan beserta margin, atau sinar dari beberapa titik pangkal. Bukan jangkauan.

**Data:** `drl/data/keadaan_cari_id01_sinar300.npz`, dan kunci `kelayakan_jangkau_300` di `model_dqn/ukur_node_id01_mac_kepala.json`.

## 15. Mode B + periksa jalur akhir + cadangan mode A (`belajar-periksa`)

Dikerjakan 2026-11-08 di Mac. Mode pencarian `belajar-periksa` (`perencana_dqfd.cari`, `periksa_jalur`, `LingkunganRencana.ulang_dari_awal`) berjalan dalam tiga tahap:
1. **Mode B:** tanpa cek tabrakan primitif; P memutuskan aksi mana yang dicoba.
2. **Periksa jalur akhir sekali.** Jalur diputar ulang dari pose awal, dan hanya primitif jalur itu yang dicek. Tembakan RS penutup tetap dicek, dan peta Dijkstra tidak dihitung ulang. Kalau lolos, biaya jalur langsung tepat.
3. **Cadangan:** kalau jalurnya menabrak atau mode B gagal, rencana ulang dengan lazy (mode A), lalu backtracking.

Jadi jalur yang diberikan ke robot selalu sudah dicek bebas zona. Latar belakangnya: wasit hanya mencegah tabrakan antar-robot, sedangkan pelanggaran zona baru di-estop setelah terjadi, dan robot tidak bisa merencanakan keluar dari zona.

Diukur di 470 rute uji, waktu CPU, median 3 ulangan. Data: `model_dqn/ukur_node_id01_mac_periksa.json` (P terpisah) dan `…_periksa_bawaan.json` (kepala bawaan).

| Varian (sumber P) | ms per node | Total node | Node p90 | Waktu per rute, median / p90 | Tiba | Jalur menabrak | Identik dengan eager |
|---|---:|---:|---:|---:|---:|---:|---:|
| Eager | 0,342 | 47.777 | 151 | 6,7 / 48,2 ms | 99,8% | 0 | 100% |
| **Mode A, lazy (P terpisah)** | 0,148 | 52.283 | 163 | 4,8 / 20,6 ms | 99,8% | 0 | 81,3% |
| **Mode A, lazy (kepala bawaan)** | 0,141 | 49.543 | 157 | **4,6 / 18,5 ms** | 99,8% | 0 | 85,5% |
| Mode B murni @0,1 (P terpisah) | 0,124 | 69.065 | 221 | 4,7 / 24,7 ms | 96,8% | 0 | 62,6% |
| belajar-periksa @0,5 (P terpisah) | 0,131 | 87.641 | 231 | 5,3 / 29,3 ms | 99,8% | 0 | 80,4% |
| belajar-periksa @0,2 (P terpisah) | 0,125 | 88.263 | 282 | 5,2 / 27,5 ms | 99,8% | 0 | 72,3% |
| **belajar-periksa @0,1 (P terpisah)** | 0,127 | 88.271 | 224 | 5,2 / 26,6 ms | **99,8%** | **0** | 67,0% |
| belajar-periksa @0,05 (P terpisah) | 0,129 | 92.325 | 303 | 5,2 / 32,5 ms | 99,8% | 0 | 62,8% |
| belajar-periksa @0,1 (kepala bawaan) | 0,116 | 92.122 | 323 | 5,0 / 32,3 ms | 99,8% | 0 | 66,0% |

Bagaimana rute diselesaikan, dari 470:

| Varian | Jalur mode B lolos periksa | Cadangan: jalur mode B menabrak | Cadangan: mode B gagal | Sampai backtracking |
|---|---:|---:|---:|---:|
| @0,5, P terpisah | 418 | 17 | 16 | 17 |
| @0,2, P terpisah | 427 | 1 | 21 | 19 |
| @0,1, P terpisah | 425 | 0 | 22 | 21 |
| @0,05, P terpisah | 414 | 0 | 30 | 24 |
| @0,1, kepala bawaan | 409 | 13 | 25 | 21 |

**Kesimpulan:**
- **Tujuannya tercapai.** Tiba kembali 99,8%, jalur yang diberikan ke robot 0 yang menabrak (dijamin oleh periksa jalur), dan waktu per node 14–22% lebih rendah daripada mode A (0,116–0,131 vs 0,141–0,148 ms).
- **Tetapi per rute justru lebih lambat daripada mode A:** median 5,0–5,3 vs 4,6–4,8 ms, dan p90 26,6–32,5 vs 18,5–20,6 ms.
  - Total node naik dari sekitar 50 ribu ke sekitar 88 ribu. Penyebabnya: rute yang gagal di mode B harus dicari dua kali (mode B lalu lazy), dan pencarian mode B sendiri butuh lebih banyak node, karena tanpa backtracking dan sebagian aksi aman ikut dibuang oleh alarm palsu.
  - Waktu per node yang lebih kecil tidak menutup jumlah node yang lebih besar.
- **Yang dirasakan robot adalah waktu per rute, dan mode A (lazy dengan kepala bawaan) tetap yang terbaik:** p90 18,5 ms, median 4,6 ms. Bandingkan dengan Hybrid A\*: p90 68,0 ms, median 3,4 ms.
- **Untuk tesis:** ms per node saja menyesatkan bila jumlah node berubah. Perbandingan perencana harus melihat waktu per rute, atau ms per node bersama jumlah node.
- **Bawaan robot tetap mode A (`lazy`).** `belajar-periksa` tersedia sebagai opsi.

## 16. Bawaan robot: mode B + periksa jalur (`belajar-periksa`)

Mulai 2026-11-08 perencana robot memakai `belajar-periksa` (`perencana_dqfd.PENCARIAN_ROBOT`), atas keputusan pengguna.
- **Sumber P:** `DQfDPlanner` otomatis memasang model P terpisah `model_dqn/id0N_kepala_p.pt` bila ada (`NetDenganP`): Q dari model jauh, P dari model P. Model dimuat lewat cache `.npz`, jadi torch tidak diimpor saat robot berjalan.
- **Ambang:** `AMBANG_BELAJAR = 0,1`, dipakai di kedua tahap (mode B dan cadangan lazy).
- **Robot tanpa P:** pencarian otomatis diganti `lazy` (`PENCARIAN_TANPA_P`), dengan pemberitahuan sekali. Ini berlaku untuk ID 02, yang modelnya tanpa kepala dan tanpa `id02_kepala_p.pt`.
- **Tidak berubah:** validasi latih dan `evaluasi.py` tetap eager.
- **Pengukuran:** `ukur_node --tanpa-kepala-p` melepas model P supaya bisa membandingkan dengan kepala bawaan.

**Astar vs dqfd-murni, satu sesi ukur** (Mac, 470 rute uji ID 01, waktu CPU, median 3 ulangan; `python -m drl.ukur_node --id 1 --uji waktu`, data `model_dqn/ukur_node_id01_mac_robot.json`):

| | Hybrid A\* + RS (astar) | DQfD (dqfd-murni, belajar-periksa) | DQfD / A\* |
|---|---:|---:|---:|
| **Waktu proses per node** (total / total) | 0,198 ms | **0,131 ms** | 0,66× |
| Kemiringan regresi waktu rute terhadap node | 0,197 ms (+0,53 ms tetap) | 0,113 ms (+3,44 ms tetap) | 0,57× |
| Total node | 151.680 | 88.271 | 0,58× |
| Node per rute, median / p90 | 4 / 388 | 19 / 225 | |
| **Waktu per rute, median** | **3,5 ms** | 5,2 ms | 1,5× |
| **Waktu per rute, p90** | 70,8 ms | **27,1 ms** | 0,38× |
| **Waktu per rute, rata-rata** | 64,1 ms | **24,6 ms** | 0,38× |
| Waktu per rute, maksimum | 3.972 ms | **689 ms** | 0,17× |
| Tiba | 99,6% | 99,8% | |

Sebagai pembanding dari sesi lain (§15), mode A (lazy, kepala bawaan) mencatat 0,141 ms per node dan waktu per rute median 4,6 / p90 18,5 / rata-rata 14,8 ms. Jadi per rute, mode A sedikit lebih cepat daripada `belajar-periksa`.

## 17. ID 02 dilatih ulang dengan kepala kelayakan + model P terpisah

Dikerjakan 2026-11-08 di laptop robot (RTX 5050), dengan prosedur yang sama dengan ID 01 (§12–§13).
- **Cadangan model lama:** `Documents\GitHub\_arsip_model_id02_tanpa_kepala_20261108`.
- **Label demo:** dibuat dengan memutar ulang episode, 0 mask beda.

| | Model ID 02 lama | **Model ID 02 baru** |
|---|---:|---:|
| Latih | 22,9 menit | 24,6 menit |
| Checkpoint terbaik (langkah gradien) | 25.000 | 70.000 |
| Skor validasi | 97,1 | **97,8** |
| Tiba / rasio biaya median / p90 | 99,2% / 1,00 / 1,21 | 100% / 1,00 / 1,22 |
| AUC kepala P (demo validasi) | – | 0,981 |

**Model P terpisah ID 02** (`model_dqn/id02_kepala_p.pt`, 81.678 parameter). Dinilai pada 70.834 keadaan pencarian rute uji, dengan data latih 255.624 keadaan pencarian rute latih ditambah demo:

| | AUC | AUC dalam keadaan sama | Terdeteksi / alarm palsu @0,1 | @0,02 |
|---|---:|---:|---:|---:|
| Kepala bawaan t2 | 0,980 | 0,923 | 88,6 / 5,7% | 96,8 / 12,6% |
| **Model P terpisah** | **0,996** | **0,980** | **98,8 / 5,2%** | 99,7 / 9,3% |

Hasil ini sejalan dengan ID 01 (§13). Sekarang **ID 01 dan ID 02 sama-sama memakai `belajar-periksa` dengan model P terpisah**. Di laptop robot, keduanya mencapai 15/15 rute uji, semuanya dengan "jalur belajar lolos periksa".

## 18. Tabel kelayakan: Hybrid A\* konvensional vs DQfD

Tujuan penelitian: apakah DQfD layak diimplementasikan dibanding Hybrid A\* konvensional.

**Cara ukur** (Mac, 2026-11-08, commit `def8410`):
- **Perencana yang dibandingkan.** Hybrid A\* + RS standar (`astar`). DQfD `dqfd-murni` = bawaan robot `belajar-periksa` dengan model P terpisah, model ID 01 dan ID 02 yang baru.
- **Perencanaan.** Semua rute uji: ID 01 470 rute, ID 02 473 rute. Waktu CPU, median 3 ulangan. `python -m drl.ukur_node --id N --uji waktu`.
- **Makespan.** Simulasi navigasi waktu nyata (`navigasi.py` tanpa jendela, pelacak pure pursuit, derau pose 1,2 cm, wasit aktif): 50 rute satu robot (25 ID 01 + 25 ID 02) dan 10 pasangan ID 01 + ID 02 yang jalurnya berpotongan. `python -m drl.uji_navigasi --id 1,2 --perencana astar,dqfd-murni --n 5`.
- **Definisi makespan.** Satu robot: dari tujuan diberikan sampai tiba (merencanakan + berjalan). Pasangan: sampai kedua robot tiba.
- **Data.** `model_dqn/tabel_performa_waktu_id01.json`, `…_id02.json`, `…_navigasi.json`.

| # | Kriteria | Ukuran | Hybrid A\* | DQfD | DQfD / A\* | Keputusan |
|---|---|---|---:|---:|---:|---|
| 1 | **Waktu proses per node** | Total CPU ÷ total node | 0,189 ms | **0,126 ms** | 0,67× | **Layak**: lebih singkat |
| | | Kemiringan regresi | 0,187 ms | **0,107 ms** | 0,57× | |
| 2 | **Waktu proses per rute** | Median | **3,3 ms** | 4,9 ms | 1,48× | A\* lebih singkat (+1,6 ms) |
| | | p90 | 56,2 ms | **25,4 ms** | 0,45× | **Layak** |
| | | Rata-rata | 47,9 ms | **21,7 ms** | 0,45× | **Layak** |
| | | Maksimum | 4.019 ms | **715 ms** | 0,18× | **Layak** |
| 3 | **Panjang jalur** | Median | **163,3 cm** | 166,8 cm | 1,02× | Hampir setara |
| | | Rata-rata | **175,2 cm** | 191,3 cm | 1,09× | DQfD lebih panjang |
| | | Rasio per rute, median / p90 / maks | – | – | 1,014 / 1,228 / 7,73 | 22,5% rute > 10% lebih panjang; 3,4% > 50% |
| 4 | **Tingkat tiba** | Perencanaan (943 rute) | 99,8% | 99,8% | – | **Setara** |
| | | Simulasi navigasi (70 robot) | 69/70 | 69/70 | – | **Setara** |
| 5 | **Makespan** | Satu robot, median / rata-rata | 16,6 / 17,2 s | **15,9 / 16,3 s** | 0,95× (median per skenario) | **Layak**: DQfD lebih singkat di 58% skenario |
| | | Dua robot, median / rata-rata | **16,4 / 19,4 s** | 21,5 / 25,8 s | 1,15× (median per skenario) | A\* lebih singkat: DQfD lebih singkat hanya di 20% skenario |
| – | Keselamatan (syarat) | Badan masuk zona / kontak antar-robot | 0 / 0 | 0 / 0 | – | Setara |
| | | Estop zona (pose terukur) / jarak robot terdekat | 1 / 6,3 cm | 0 / 7,4 cm | – | |

Per robot, polanya sama:

| | Waktu per node A\* / DQfD | Waktu per rute (median / p90 / rata-rata), A\* → DQfD | Rasio panjang (median / p90) | Tiba A\* / DQfD |
|---|---|---|---|---|
| ID 01 | 0,189 / 0,124 ms | 3,4 / 68,0 / 61,9 → 5,1 / 25,9 / 23,3 ms | 1,009 / 1,215 | 99,6 / 99,8% |
| ID 02 | 0,189 / 0,129 ms | 3,3 / 44,7 / 33,9 → 4,8 / 23,4 / 20,1 ms | 1,020 / 1,249 | 100 / 99,8% |

**Kesimpulan:**
- **Komputasi: DQfD layak.**
  - Per node 33% lebih singkat.
  - Per rute, rata-rata dan p90 sekitar 2,2× lebih singkat, dan kasus terburuknya 5,6× lebih singkat (0,7 vs 4,0 s).
  - A\* hanya menang di median per rute, karena tembakan RS di ekspansi ke-4 langsung tembus di rute mudah. Selisihnya 1,6 ms, sekitar 0,01% dari makespan, jadi tidak terasa.
- **Tingkat tiba dan keselamatan: setara.**
- **Makespan satu robot: DQfD sedikit lebih singkat** (median −4%).
- **Panjang jalur dan makespan dua robot: DQfD belum layak.** Keduanya bersumber dari hal yang sama: tahap cadangan **backtracking** (`terbaik -> mundur`) menghasilkan jalur memutar yang sangat panjang.
  - Contohnya rute Rumah #1713: 21 m, padahal A\* 3,5 m, dan habis waktu.
  - Di pasangan dua robot, rute yang dihitung ulang memutari robot lain sering jatuh ke backtracking. Contohnya Gudang2 #1685/#1671 (31 vs 13 s) dan Rumah2 #3514/#1725 (45 vs 28 s). Skenario tanpa rencana ulang hampir setara.
- **Perbaikan yang paling menjanjikan:** ganti backtracking dengan Hybrid A\* sebagai cadangan bila DQfD gagal. Ini mode `dqfd`, yang juga punya gerbang mutu 1,3 × perkiraan V. Hasilnya perlu diukur dengan tabel yang sama.
- **Catatan sampel:** 10 pasangan dua robot masih sedikit.
