Catatan ini menjelaskan cara kerja **Reeds-Shepp (RS)** di perencana jalur, dan perbedaannya untuk ketiga pilihan `--perencana`: `astar`, `dqfd`, dan `dqfd-murni`. Dibuat 2026-10-02 dari kode di branch `RS-optimize` (sama dengan `main` di `060cff4`).

Lihat juga: [[Deep Reinforcement Learning]], [[Multi Robot Navigation and Control]].

> [!note] Ringkasnya
> Ketiga perencana memakai **fungsi RS yang sama** (`HybridAStarPlanner._analytic()` → `_best_free_rs()`). Bedanya ada pada **kapan RS dicoba** dan **kapan hasilnya diterima**:
>
> | Perencana | Kapan RS dicoba | Kapan RS diterima |
> |---|---|---|
> | `astar` | simpul yang di-pop, bila jarak ke tujuan < 101 cm **atau** setiap ekspansi ke-4 | **langsung**, begitu ada satu RS bebas tabrakan; pencarian selesai |
> | `dqfd-murni` | di setiap keadaan yang dikunjungi, **hanya bila jarak ke tujuan < 100 cm** (sejak 2026-10-02) | sebagai aksi **SAMBUNG** (aksi 14), hanya bila f-nya terkecil di seluruh pohon pencarian |
> | `dqfd` | di **setiap** keadaan, tanpa batas jarak; bila gagal atau ditolak gerbang mutu, seperti `astar` | seperti SAMBUNG di atas, lalu harus lolos gerbang mutu; kalau tidak, seperti `astar` |

---

## 1. Bagian bersama: dari pose ke jalur RS bebas tabrakan

Semua kode RS ada di `forklift_planner.py`. Berkas itu salinan v1 dan tidak diubah. DQfD memanggil fungsi yang sama lewat objek `HybridAStarPlanner`.

### 1.1 Radius belok

RS dihitung dengan radius **tetap**, yaitu radius rencana, bukan radius minimum fisik:

```
κ_max  = tan(max_steer × PLAN_STEER_RATIO) / L        PLAN_STEER_RATIO = 0,75, L = 22,5 cm
radius = 1 / κ_max
```

| Robot | max_steer | Sudut rencana | **Radius RS** |
|---|---|---|---|
| ID 01 | 35,0° | 26,25° | **45,6 cm** |
| ID 02 | 22,0° | 16,5° | **76,0 cm** |

Sisa 25% sudut kemudi dicadangkan untuk koreksi pelacak (pure pursuit). Karena radius ID 02 hampir dua kali radius ID 01, kurva RS ID 02 jauh lebih lebar. Akibatnya, di peta sempit RS lebih sering menabrak dan gagal.

### 1.2 Pembangkitan kandidat — `reeds_shepp_paths(start, goal, radius)`

1. Tujuan dipindahkan ke kerangka pose awal lalu dinormalisasi dengan radius (x, y, φ).
2. `_rs_words()` mencoba 12 rumus keluarga RS (implementasi OMPL), masing-masing dalam 4 varian (asli, *time-flip*, *reflect*, dan keduanya). Jadi ada **hingga 48 kandidat per target**:
   - **CSC**: L S L, L S R
   - **CCC**: L R L (dua rumus)
   - **CCCC**: L R L R (dua rumus)
   - **CCSC**: L R S L, L R S R, L S R L, R S R L
   - **CCSCC**: L R S L R
3. Setiap kandidat **diverifikasi** dengan mengintegrasikan segmennya. Bila titik akhirnya meleset > 1e-4 dari tujuan, kandidat dibuang. Rumus yang salah tidak akan lolos.
4. Hasilnya berupa daftar `[(segmen, panjang_total)]`. Tiap segmen ditulis `('L'|'R'|'S', panjang bertanda)`, dengan panjang negatif berarti **mundur**.

### 1.3 Target RS — `_analytic(omap, node, goal)`

RS tidak selalu diarahkan tepat ke tujuan. Targetnya tergantung tujuan:

| Kondisi tujuan | Target RS | Ekor |
|---|---|---|
| Arah hadap bebas (`gyaw = None`) | (gx, gy, *bearing*) dan (gx, gy, *bearing* + 180°) | — |
| Arah hadap tertentu, pendekatan lurus muat | titik **32,4 cm sebelum** tujuan (`approach_len`), untuk setiap arah yang lolos `_feasible_approach_dirs` (maju dan/atau mundur) | segmen lurus `('S', ±32,4 cm)` masuk ke tujuan |
| Arah hadap tertentu, pendekatan lurus tidak muat | tepat (gx, gy, gyaw) | — |

Segmen lurus penutup memberi pelacak jarak untuk meluruskan heading sebelum berhenti. Arah pendekatan dihitung **sekali per perencanaan** (`_approach_dirs`): badan harus bebas zona sepanjang 32,4 cm di belakang (pendekatan maju) atau di depan (pendekatan mundur) tujuan.

### 1.4 Pemilihan dan pemeriksaan tabrakan — `_best_free_rs()`

1. Semua kandidat dari semua target diberi biaya `_rs_cost()`, lalu **diurutkan dari termurah**.
2. Hanya **6 kandidat termurah** yang diperiksa tabrakannya.
3. Tiap kandidat disampel setiap `sample_ds` = 2,4 cm (`sample_rs_path`). Setiap pose harus punya `pose_margin > 0`, yaitu footprint badan penuh dengan margin keselamatan. Satu pose yang menempel sudah cukup untuk membuang kandidat itu.
4. Pose yang lolos dikenai **penalti clearance**: `clearance_cost × max(0, 6,1 cm − margin) × ds / step`.
5. Pemeriksaan berhenti lebih awal bila biaya dasar kandidat berikutnya sudah ≥ biaya total kandidat terbaik.
6. Hasilnya `(poses, biaya)` untuk RS bebas termurah, atau `None` bila keenam kandidat menabrak.

Biaya RS memakai **bobot yang sama** dengan primitif Hybrid A\*, dan dimulai dari arah (`d`) dan kelengkungan (`k`) simpul asal:

```
_rs_cost = Σ per segmen [ |l|·(1 bila maju, reverse_cost=2,0 bila mundur)
                        + steer_cost(0,3)·|l|·|κ|/κ_max
                        + steer_change_cost(6,0)·|κ − κ_sebelum|/κ_max
                        + switch_cost(32,4)·[ganti arah] ]
biaya RS = _rs_cost + Σ penalti clearance sepanjang RS
```

Karena biayanya sejenis, biaya RS bisa dijumlahkan langsung dengan g-cost simpul. Itulah yang membuat DQfD bisa membandingkan SAMBUNG dengan primitif biasa (bagian 3).

> [!warning] Hanya 6 kandidat yang diperiksa
> Kalau keenam RS termurah menabrak, RS dianggap **gagal** walaupun kandidat ke-7 dan seterusnya mungkin bebas. Pencarian lalu harus berjalan lebih jauh dengan primitif sebelum mencoba lagi.

### 1.5 Hasil RS di dalam jalur

Pose RS disambung di **ujung** jalur sebagai "ekor" berformat sama `(x, y, yaw, arah)`. Pelacak tidak membedakan ekor RS dari primitif. `split_segments()` memecah jalur di setiap pergantian arah, termasuk *cusp* di dalam RS, dan pure pursuit berhenti sejenak di sana.

---

## 2. `astar` — Hybrid A\* dengan ekspansi analitik

Di dalam loop `HybridAStarPlanner.plan()`, setiap simpul yang di-pop dari heap diperiksa dengan urutan berikut:

```
pop simpul dengan f = g + 1,3·h terkecil
 ├─ sudah dalam toleransi tujuan (3,2 cm, 12°)?   → selesai, tanpa RS
 ├─ badan masih menempel rintangan (in_col)?      → lewati RS
 ├─ jarak ke tujuan < analytic_dist (101 cm)
 │    ATAU expansions % analytic_every(4) == 0?
 │       └─ _analytic(): ada RS bebas?            → TERIMA, pencarian selesai
 └─ ekspansi 14 primitif (7 kemudi × maju/mundur)
```

Sifat-sifatnya:

- **Kapan dicoba.** Dekat tujuan (< 101 cm), RS dicoba di **setiap** simpul. Jauh dari tujuan, hanya di setiap ekspansi ke-4, berapa pun jaraknya. Karena itu ada demo yang tersambung dari > 101 cm.
- **Diterima langsung.** RS bebas pertama yang ditemukan langsung dipakai, dan biayanya tidak dibandingkan dengan alternatif lain. Simpul asalnya memang simpul dengan f terkecil saat itu, tetapi f hanya memakai heuristik Dijkstra holonomik, bukan biaya RS yang sebenarnya. Jadi ekor RS bisa panjang atau berkelok kalau pada saat itu hanya RS mahal yang bebas.
- **Tidak dicoba saat menempel.** Bila pose awal menempel rintangan, RS baru dicoba setelah gerakan "lepas diri" membuat badan bebas lagi.
- **Peta lapang.** Di peta Kalibrasi, Hybrid A\* hampir selalu langsung tersambung RS (±2 ekspansi). Di Rumah, RS sering menabrak, sehingga pencarian primitif yang menentukan waktu.
- **Sumber demo DQfD.** Ekor RS di jalur A\* direkonstruksi oleh `drl/adaptor_demo.py` sebagai **satu aksi SAMBUNG**. Biayanya (`node.g + biaya RS`) direkam dengan membungkus `_analytic`, dan diuji sama dengan g-cost A\* sampai 1e-6.

---

## 3. `dqfd-murni` — RS sebagai aksi SAMBUNG

`DQfDPlanner` (`drl/perencana_dqfd.py`) memakai **salinan** objek Hybrid A\* (`copy.copy(astar)`) sebagai pustaka gerak `LingkunganRencana` (`drl/lingkungan.py`). Jadi `_analytic`, `_best_free_rs`, radius, target pendekatan, dan bobot biayanya sama persis dengan bagian 1.

### 3.1 Kapan RS dihitung

`_hitung_calon()` dijalankan setiap kali lingkungan **mencapai keadaan baru**, yaitu setelah `reset()` dan setelah `step()`. Fungsi ini menghitung ke-15 aksi. Sejak optimasi 2026-10-02 (bagian 6), hasilnya disimpan per simpul (`simpan_calon()`), dan `pulihkan_keadaan()` di pencarian terbaik-dulu memakai simpanan itu, sehingga RS **tidak dihitung ulang** saat pencarian pindah cabang:

```
aksi 0..13 : _ekspansi() primitif, di-mask bila menabrak atau mendarat di sel yang sudah dikunjungi
aksi 14    : SAMBUNG = _analytic(); di-mask bila in_col, jarak ≥ jarak_sambung, atau tidak ada RS bebas
```

- **`dqfd-murni`: RS hanya di akhir jalur (sejak 2026-10-02, branch `RS-optimize`).** `jarak_sambung = 100 cm` (`perencana_dqfd.JARAK_SAMBUNG_MURNI_CM`, dipasang oleh `rencana.Perencana` lewat `DQfDPlanner(jarak_sambung_cm=...)`). Jarak diukur lurus dari titik acuan robot (poros roda depan) ke titik tujuan. Di luar 100 cm, `_analytic()` tidak dipanggil sama sekali dan SAMBUNG ter-mask, jadi robot harus mendekat dengan primitif. Tidak ada "setiap ke-4" seperti A\*.
- **`dqfd` dan data latih: tanpa batas jarak** (`jarak_sambung = ∞`, bawaan `LingkunganRencana`). Batas jarak sempat dicoba saat membangun dataset, tetapi membuat sebagian demo A\* ter-mask, karena A\* juga menyambung dari > 101 cm. Jaringan dilatih dalam kondisi ini.
- Biaya terukur: **0,3 ms rata-rata (maks 0,7 ms)** per pemanggilan, di jarak mana pun.

**Hasil uji batas 100 cm** (2026-10-02, seluruh rute uji dataset, metode terbaik-dulu, batas waktu 2 s, model `id0N.pt` yang sama; rasio = biaya / g-cost A\* dataset):

| | ID 01 tanpa batas | ID 01 ≤ 100 cm | ID 02 tanpa batas | ID 02 ≤ 100 cm |
|---|---|---|---|---|
| Rute | 463 | 463 | 471 | 471 |
| Tiba | 95,9% | **96,8%** | 95,3% | **95,5%** |
| Rasio biaya median / p90 | 1,00 / 1,12 | 1,00 / **1,27** | 1,00 / 1,13 | 1,00 / **1,29** |
| Waktu median / p90 (ms) | 8 / 358 | **18** / 349 | 7 / 352 | **22** / 351 |
| Jarak awal RS maksimum | 219 cm | **100 cm** | 221 cm | **100 cm** |

Per peta, tingkat tiba Rumah ID 01 naik 90,1 → 93,0%, Rumah2 naik di kedua robot, dan peta sintetis turun 0,6 poin. Rute yang gagal dengan batas ini habis anggaran (6 / 8) atau batas waktu (9 / 13). Jadi batas 100 cm tidak menurunkan tingkat tiba, tetapi 10% jalur termahal menjadi ±13–14% lebih mahal, dan median waktu naik 2–3×, karena robot harus mendekat dengan primitif sebelum boleh menyambung.

> [!warning] Jaringan dilatih tanpa batas jarak
> Di `dqfd-murni`, Q(s, primitif) masih memperkirakan biaya-sisa seolah SAMBUNG boleh dari jarak berapa pun. Dari jauh, perkiraan itu bisa terlalu optimistis, karena jalur sebenarnya harus mendekat dulu dengan primitif. Pencarian terbaik-dulu tetap benar (SAMBUNG dinilai dengan biaya RS sebenarnya), hanya bisa lebih banyak pindah cabang.

### 3.2 Kapan SAMBUNG dipilih — metode terbaik-dulu (bawaan)

`METODE_BAWAAN = "terbaik"`, bobot 2,0, anggaran 1200 langkah, batas waktu = `time_limit` A\* (2 s). Setiap aksi sah dari simpul yang dibuka didorong ke **satu heap global**:

```
primitif : f = g + c(a) + 2,0 × h,    h = max(0, −Q(s,a)·SKALA_REWARD − c(a))
SAMBUNG  : f = g + c(SAMBUNG)          (terminal: biaya RS sebenarnya, tanpa h)
```

- SAMBUNG **bersaing** dengan semua calon primitif di seluruh pohon. RS yang mahal (panjang, banyak mundur, banyak ganti arah) baru dipilih kalau tidak ada cabang lain yang diperkirakan lebih murah. Ini berbeda dengan `astar` yang langsung menerima RS bebas pertama.
- Begitu SAMBUNG di-pop dan dijalankan, episode **sukses** dan pencarian selesai. Karena f SAMBUNG adalah biaya total yang sebenarnya, jalur ditutup hanya bila sambungan itu memang kompetitif.
- Q jaringan hanya memengaruhi f **primitif**. Untuk SAMBUNG, nilai Q tidak dipakai, jadi keputusan "sambung sekarang" ditentukan biaya RS yang sebenarnya dibandingkan perkiraan cabang lain.
- Jalan lain untuk sukses adalah primitif yang mendarat dalam toleransi tujuan ("tiba"), tanpa RS.

Pada metode lama `mundur` (greedy + backtracking), SAMBUNG dipilih bila **Q(s, SAMBUNG) terbesar** di antara aksi sah. Di sana jaringan sendiri yang belajar kapan menyambung, dari demo A\*: 9–12% aksi demo adalah SAMBUNG, dan rewardnya bisa sampai −956 untuk RS yang panjang.

### 3.3 Bila gagal

Bila anggaran atau waktu habis, atau semua cabang buntu, `plan()` mengembalikan `None` dengan alasan `"DQfD: ..."`. **Tidak ada cadangan**: robot tidak mendapat jalur. Mode ini dipakai untuk menilai DQfD sendirian.

---

## 4. `dqfd` — DQfD dengan Hybrid A\* sebagai cadangan

```
DQfDPlanner.plan()
 ├─ DQfD terbaik-dulu (RS = SAMBUNG, persis bagian 3)
 │    └─ sukses → gerbang mutu: biaya / perkiraan_V ≤ 1,3 ?
 │          ├─ ya    → pakai jalur DQfD       (perencana = "dqfd")
 │          └─ tidak → ditolak
 └─ gagal / ditolak → astar.plan() ASLI       (perencana = "A* (cadangan)")
                       RS persis bagian 2
```

- **RS masuk ke gerbang mutu.** `env.biaya` sudah termasuk biaya RS ekor, sehingga RS yang panjang atau berkelok ikut menaikkan rasio dan bisa membuat jalur DQfD ditolak.
- **Cadangan memakai objek A\* asli**, bukan salinan yang dipakai lingkungan, sehingga `_hcost` dan `_approach_dirs` dihitung ulang dari nol. RS di sana mengikuti aturan `astar`: ekspansi ke-4 atau < 101 cm, lalu langsung diterima.
- `last_stats["perencana"]` mencatat siapa yang akhirnya menghasilkan jalur, dan `dqfd_gagal` menyimpan alasan DQfD gagal.

---

## 5. Perbandingan lengkap

| Aspek | `astar` | `dqfd-murni` | `dqfd` |
|---|---|---|---|
| Fungsi RS | `_analytic` → `_best_free_rs` | sama (salinan objek A\*) | sama |
| Radius | 45,6 cm (ID 01) / 76,0 cm (ID 02) | sama | sama |
| Target & ekor lurus 32,4 cm | ya | ya | ya |
| Kandidat yang diperiksa tabrakan | 6 termurah | 6 termurah | 6 termurah |
| Frekuensi RS | < 101 cm: setiap simpul; selain itu setiap ekspansi ke-4 | **hanya < 100 cm**: sekali per simpul baru (disimpan, tidak diulang saat pindah cabang); ≥ 100 cm: tidak pernah | DQfD: sekali per simpul baru, tanpa batas jarak; cadangan: seperti `astar` |
| Saat menempel rintangan | tidak dicoba | SAMBUNG ter-mask | sama |
| Penerimaan | RS bebas pertama → selesai | SAMBUNG di-pop dari heap global (f = g + biaya RS) | seperti `dqfd-murni` + gerbang mutu 1,3 |
| Bila RS tak pernah bebas | terus ekspansi sampai tiba primitif / batas 2 s atau 30.000 ekspansi | terus sampai tiba primitif / anggaran 1200 / 2 s, lalu `None` | jatuh ke A\* |
| Biaya RS dipakai untuk | tidak dibandingkan, hanya dijumlahkan | memilih SAMBUNG vs cabang lain | sama + gerbang mutu |

---

## 6. Pengamatan dan optimasi RS

Daftar ini awalnya pengamatan dari membaca kode. Butir 1 sudah diukur dan diperbaiki (2026-10-02). Rincian per tahap dan hasilnya ada di [[Deep Reinforcement Learning]], bagian "Branch RS-optimize".

1. ✅ **RS dihitung ulang di DQfD — sudah diperbaiki.** Dulu `pulihkan_keadaan()` memanggil `_hitung_calon()` lagi setiap kali terbaik-dulu pindah cabang. Terukur di Rumah ID 01: 332 pemulihan per rencana untuk 148 langkah, jadi ±69% hitungan RS dan primitif adalah ulangan. Sekarang calon disimpan per simpul, dan entri heap yang basi dibuang tanpa memulihkan simpulnya. Panggilan RS turun 277 → 118 per rencana (Rumah ID 01), waktu RS 106 → 46 ms, dan jalur yang dihasilkan **identik**.
2. **Frekuensi RS di `dqfd` 4× lebih sering** daripada A\* untuk simpul yang jauh dari tujuan (`dqfd-murni` sudah dibatasi 100 cm, lihat bagian 3.1).
   Pembanding: di Hybrid A\*, RS adalah tahap **terberat** (58–73% waktu di Rumah2 dan Rumah, sampai 725 panggilan per rencana), karena sebagian besar percobaannya menabrak.
3. **Hanya 6 kandidat yang diperiksa.** Ambang ini memotong biaya, tetapi bisa membuat RS bebas terlewat di peta sempit. Efeknya lebih terasa pada ID 02 (radius 76 cm).
4. **Pemeriksaan tabrakan per pose** (`pose_margin` setiap 2,4 cm) adalah bagian terberat `_best_free_rs`. Sampel bisa dibuat per-batch, atau diperiksa dulu dengan langkah lebih kasar.
5. **A\* menerima RS bebas pertama.** Ekor RS panjang di jalur A\* ikut masuk ke demo DQfD, lalu dipelajari jaringan. Membandingkan biaya RS sebelum menerimanya (seperti DQfD) bisa memperbaiki demo, tetapi `forklift_planner.py` saat ini tidak boleh diubah.
