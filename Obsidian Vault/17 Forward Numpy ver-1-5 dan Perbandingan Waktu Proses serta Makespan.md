# Forward jaringan dengan numpy (ver-1-5): waktu proses dan makespan Hybrid A\* + RS vs DQfD

Tanggal: 2026-11-04 · Branch `ver-1-5`, commit `9cf9eee` · Perubahan 2 dari [[16 Rencana Optimasi Waktu Proses Perencana|Rencana Optimasi Waktu Proses Perencana]] · Lanjutan dari [[15 Sinar Numba ver-1-5 dan Perbandingan Waktu Proses serta Makespan|Sinar Numba ver-1-5 dan Perbandingan Waktu Proses serta Makespan]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Setelah sinar dikompilasi numba (catatan 15), bagian khusus DQfD yang terbesar adalah forward PyTorch: 3,6 ms per kasus di Windows, 19% dari waktu DQfD. Forward itu sekarang dihitung dengan **numpy**, memakai bobot model yang sama dan tanpa latih ulang. Hybrid A\* + RS tidak diubah.

> [!summary] Ringkasan
> - **Forward jaringan jauh 3,6× lebih cepat di Windows** (3,58 → 1,01 ms per kasus) dan 2,1× di Mac (0,96 → 0,46 ms).
> - **Waktu proses DQfD**:
>   - Windows: 17,2 → **12,7 ms (−26%)**, Hybrid A\* + RS 24,5 ms (**0,52×**);
>   - Mac: 8,6 → **8,0 ms (−7%)**, Hybrid A\* + RS 15,4 ms (0,52×);
>   - DQfD sekarang lebih cepat dari A\* di **keempat** kasus Gudang, di kedua mesin.
> - **Startup `navigasi.py` di Windows** (impor + muat model + rencana pertama): 2,40 → **0,87 s**. PyTorch tidak lagi diimpor di jalur perencana.
> - **Hasil setara:** selisih Q ≤ 4,8e-6, urutan aksi sama di semua observasi (8.765 di Mac, 8.532 di Windows), jalur identik 124/124 (Mac) dan 123/124 (Windows; satu rute berbeda hanya karena batas waktu).
> - **Makespan simulasi**, digabung 10 ulangan per skenario: S2 DQfD **14,0 s** vs A\* **19,7 s**, berbeda signifikan (Mann-Whitney p = 0,007). S1 20,9 vs 21,9 s, tidak berbeda (p = 0,79). Waktu proses tetap hanya ±0,2% dari makespan.
> - **Temuan baru:** di satu jalan, DQfD butuh 1,3–1,8 s saat merencanakan ulang dengan badan robot lain di dekatnya. Ini kelemahan model, bukan efek perubahan ini.

---

## 1. Perubahan

| Berkas | Isi |
|---|---|
| `drl/jaringan_numpy.py` (baru, **tanpa torch**) | `JaringanNumpy`: forward Dueling DQN float32. `muat_dari_pt`: bobot dari `.pt` lewat cache `.npz`. `periksa_kontrak` dan `periksa_profil`, dipindah dari `dqfd.py`. `SKALA_REWARD` |
| `drl/perencana_dqfd.py` | forward lewat `_q()` / `_qv()`, yang menerima `JaringanNumpy` **atau** jaringan PyTorch. `DQfDPlanner` memakai `muat_model_numpy()`. `import torch` hanya di jalur PyTorch |
| `drl/dqfd.py`, `drl/jaringan.py` | mengimpor pemeriksaan dan `SKALA_REWARD` dari `jaringan_numpy`. Impor lama (`from drl.dqfd import periksa_profil`, dll.) tetap berfungsi |

**Forward numpy.**
- Bentuk jaringannya sama dengan `drl/jaringan.py`: 47/9 → 256 → 256 → (V 128 | A 128) → V + A − rata(A).
- Cabang V dan A **digabung**: v1|a1 menjadi satu matriks 256 × 256, dan v2, a2 menjadi satu matriks blok 256 × (1 + n_aksi). Alasannya, yang mahal adalah overhead per operasi numpy, bukan hitungannya, jadi jumlah operasi dibuat sesedikit mungkin.
- Satu forward model jauh di Mac:

| Versi | Waktu |
|---|---:|
| PyTorch | 37 µs |
| numpy, cabang terpisah | 22 µs |
| **numpy, cabang digabung** | **16 µs** |

**Cache `.npz`.**
- Bobot disalin sekali ke `model_dqn/id0N_<jenis>.npz`, bersama md5 file `.pt` dan meta-nya. Folder `model_dqn/` sudah di-ignore git.
- Selama md5 `.pt` sama, model dimuat **tanpa mengimpor torch**.
- Bila model dilatih ulang, cache dibangun ulang otomatis. Hanya pada saat itulah torch masih diimpor.
- Kontrak versi dan profil kemudi tetap diperiksa setiap kali model dimuat.
- Cache sudah dibangun untuk keempat model di Mac dan di laptop Windows.

**Pelatihan tidak berubah.** `latih.py`, `dagger.py`, `evaluasi.py`, dan `visual_neuron.py` tetap memakai PyTorch. Fungsi pencarian (`rencanakan_dua`, `cari`, `buat_cek_pendaratan`) menerima kedua jenis jaringan.

## 2. Verifikasi

| Pemeriksaan | Mac | Windows |
|---|---|---|
| Selisih Q numpy vs PyTorch, pada observasi nyata selama merencanakan 124 rute | maks **4,8e-6** (jauh) / 1,9e-6 (dekat) | maks **3,8e-6** / 1,4e-6 |
| Aksi terbaik (argmax) sama | 8.765 / 8.765 | 8.532 / 8.532 |
| **Urutan semua aksi** (14 / 88) sama | **8.765 / 8.765** | **8.532 / 8.532** |
| Jalur identik (md5): 20 rute uji × Kalibrasi, Rumah, Rumah2 × ID 01, ID 02, ditambah 4 kasus Gudang | **124 / 124** | **123 / 124** |
| `python -m drl.adaptor_demo --uji` | lulus semua | lulus semua |
| torch diimpor selama merencanakan (setelah cache ada) | tidak | tidak |
| Modul latih dan analisis masih bisa diimpor | ya | — |

Rute yang berbeda di Windows adalah `id01/Rumah/551`, tujuan tanpa arah hadap.
- Urutan aksinya identik di semua observasi, jadi perbedaannya **murni efek batas waktu** (0,4 s per arah hadap; terbaik-dulu 0,3 s).
- Versi PyTorch butuh 956 ms dan menyentuh batas, versi numpy 281 ms.
- Rute yang sama juga berbeda karena alasan serupa saat cache observasi diterapkan ([[14 Waktu Komputasi per Tahapan Hybrid A-star RS vs DQfD Gudang|catatan 14]]).

## 3. Waktu proses per rencana

Cara mengukurnya sama dengan catatan 14 dan 15:
- 4 kasus Gudang, 30 ulangan per kasus, A\* dan DQfD diselang-seling per ulangan;
- Windows: 3 run kode baru, **diselang-seling per run** dengan 2 run kode lama (commit `041abf1`, sinar numba + forward PyTorch);
- Mac: 1 run kode baru, dibanding run kode lama dari catatan 15.

### 3.1 Total (tanpa pencatat waktu)

| | Windows lama | **Windows baru** | Mac lama | **Mac baru** |
|---|---:|---:|---:|---:|
| Forward jaringan jauh per kasus (dengan pencatat waktu) | 3,58 ms | **1,01 ms** | 0,96 ms | **0,46 ms** |
| Forward jaringan dekat per kasus | 0,20 ms | **0,08 ms** | 0,05 ms | **0,03 ms** |
| **DQfD rata-rata / median** | 17,2 / 17,2 ms | **12,7 / 12,2 ms** | 8,6 / 8,5 ms | **8,0 / 8,0 ms** |
| Hybrid A\* + RS rata-rata (sesi yang sama) | 24,8 ms | 24,5 ms | 15,1 ms | 15,4 ms |
| **DQfD ÷ A\*** | 0,69× | **0,52×** | 0,57× | **0,52×** |

Per kasus, rata-rata (ms):

| Kasus | Langkah DQfD / ekspansi A\* | Windows: DQfD lama → **baru** / A\* | Mac: DQfD lama → **baru** / A\* |
|---|---:|---|---|
| S1-ID01 | 44 / 115 | 26,4 → **18,5** / 23,7 | 13,6 → **12,3** / 16,4 |
| S2-ID01 | 10 / 143 | 10,5 → **8,4** / 32,4 | 5,5 → **5,3** / 19,3 |
| S1-ID02 | 29 / 92 | 20,5 → **14,9** / 22,6 | 9,6 → **8,9** / 13,7 |
| S2-ID02 | 12 / 78 | 11,2 → **9,1** / 19,5 | 5,7 → **5,6** / 12,1 |

- **Untungnya jauh lebih besar di Windows** (−26%) daripada di Mac (−7%), sesuai [[15 Sinar Numba ver-1-5 dan Perbandingan Waktu Proses serta Makespan|catatan 15]] bagian 5: forward PyTorch di Windows 2,5× lebih lambat daripada di Mac.
- **Selisih antar-mesin sekarang sama untuk kedua perencana.** Windows ÷ Mac: DQfD 12,7 / 8,0 = 1,59×, A\* 24,5 / 15,4 = 1,59×. Sebelumnya DQfD 2,0× karena porsi PyTorch-nya.
- **Overhead pencarian ikut turun** (J9: 2,3 → 1,1 ms di Windows), karena konversi tensor PyTorch ↔ numpy di setiap langkah juga hilang.

### 3.2 Startup

Diukur dari impor modul, memuat model, sampai rencana pertama selesai (Gudang S1-ID01, proses Python baru, 3 kali):

| | Kode lama | **Kode baru** |
|---|---:|---:|
| Windows | 2,40 s (torch diimpor) | **0,87 s** (tanpa torch) |
| Mac | 0,89 s (torch diimpor) | **0,38 s** (tanpa torch) |

- Semua angka diambil dalam kondisi "hangat": hasil kompilasi numba sudah tersimpan di cache, dan berkas sudah ada di cache disk. Mac kode lama diukur dari git worktree sementara di commit `041abf1`.
- Pertama kali setelah model dilatih ulang, cache `.npz` dibangun, sehingga torch diimpor sekali (Windows 2,66 s, Mac 1,78 s).

### 3.3 Rincian DQfD per tahapan (kode baru)

ms per kasus, rata-rata 4 kasus, dengan pencatat waktu. Nama tahapan sama dengan catatan 14 bagian 3.

| No | Tahapan | Windows (ms) | Porsi | Mac (ms) | Porsi |
|---|---|---:|---:|---:|---:|
| 1–4 | Pra-proses (masukan, validasi, peta rintangan, konversi) | 1,140 | 7,7% | 0,684 | 8,0% |
| J1 | Reset lingkungan jauh | 0,209 | 1,4% | 0,097 | 1,1% |
| J2 | Heuristik Dijkstra | 2,734 | **18,5%** | 1,994 | **23,4%** |
| J3 | Arah pendekatan lurus | 0,115 | 0,8% | 0,058 | 0,7% |
| J4 | Observasi: 32 sinar (numba) | 0,593 | 4,0% | 0,274 | 3,2% |
| J5 | Observasi: sisa 47 masukan | 0,507 | 3,4% | 0,248 | 2,9% |
| J6 | **Forward jaringan jauh (numpy)** | **1,006** | **6,8%** | **0,456** | **5,4%** |
| J7 | Calon primitif: cek tabrakan | 3,807 | **25,7%** | 2,339 | **27,5%** |
| J8 | Calon primitif: kinematik dan biaya | 2,914 | **19,7%** | 1,486 | **17,5%** |
| J9 | Pencarian terbaik-dulu | 1,116 | 7,5% | 0,544 | 6,4% |
| D1–D6 | Model dekat (1 percobaan pendaratan) | 0,593 | 4,0% | 0,298 | 3,5% |
| 14 | Pasca-proses | 0,034 | 0,2% | 0,020 | 0,2% |
| | **Total (dengan pencatat waktu)** | **14,77** | 100% | **8,50** | 100% |

Per fase, dibanding Hybrid A\* (catatan 14 bagian 4, Windows):

| Fase | Hybrid A\* + RS | DQfD lama (`041abf1`) | **DQfD baru** |
|---|---:|---:|---:|
| Pra-proses | 1,14 | 1,17 | 1,14 |
| Heuristik Dijkstra | 2,54 | 2,75 | 2,73 |
| **Bagian jauh** | **21,35** | **14,44** | **9,94** |
| **Bagian dekat** | 0,45 | 0,80 | **0,59** |

**Bottleneck DQfD sekarang adalah kode yang sama dengan Hybrid A\*:** cek tabrakan calon primitif (26%), kinematik (20%), dan Dijkstra (19%), total ±64%. Bagian khusus DQfD (sinar, observasi, forward, pencarian) tinggal ±22%.

## 4. Makespan simulasi

Cara mengukurnya sama dengan [[15 Sinar Numba ver-1-5 dan Perbandingan Waktu Proses serta Makespan|catatan 15]] bagian 4:
- `drl.uji_navigasi.jalankan()`: `navigasi.py` tanpa jendela, dua robot simulasi, wasit, pure pursuit, 20 Hz waktu nyata;
- skenario `uji_nyata/Gudang.json` S1 dan S2, derau pose 1,2 cm, 5 ulangan, perencana diselang-seling, laptop Windows;
- waktu tempuh dihitung dari tujuan diberikan sampai selesai, termasuk waktu perencanaan.

### 4.1 Kode baru (5 ulangan)

| | Hybrid A\* + RS | DQfD murni |
|---|---|---|
| S1 makespan (s) | 22,9 ± 4,8 (median 23,6; 17,1–27,7) | 22,3 ± 7,5 (median **17,8**; 17,4–34,8) |
| S2 makespan (s) | 19,9 ± 1,9 (median 20,1; 17,0–22,1) | **13,0 ± 1,2** (median 13,5; 11,5–14,2) |
| Semua (n = 10) | 21,4 ± 3,8 | 17,6 ± 7,0 |
| Tiba / masuk zona | 20/20 / 0 | 20/20 / 0 |
| Rencana pertama di simulasi, rata-rata | 37 ms | 40 ms |
| Galat akhir rata-rata | 3,5 cm / 5,5° | 3,1 cm / 5,4° |
| Jarak terdekat ke zona / antar-robot | 7,7 / 10,5 cm | 6,2 / **4,4** cm |

Makespan per ulangan (s):

| Skenario | Hybrid A\* + RS | DQfD murni |
|---|---|---|
| S1 | 18,7 · 17,1 · 23,6 · 27,2 · 27,7 | 23,7 · 17,5 · **34,8** · 17,4 · 17,8 |
| S2 | 20,1 · 22,1 · 20,7 · 17,0 · 19,5 | 14,0 · 14,2 · 11,5 · 11,8 · 13,5 |

### 4.2 Digabung dengan simulasi catatan 15 (10 ulangan per skenario)

Jalur ver-1-5 dengan forward PyTorch dan dengan forward numpy identik, jadi kedua simulasi menguji perencana yang sama. Selisih waktu rencana ±5 ms, dan itu tidak berarti dibanding makespan.

| Skenario | Hybrid A\* + RS | DQfD murni | Mann-Whitney (dua arah) |
|---|---|---|---|
| S1 (n = 10 / 10) | 21,9 ± 4,5 s (median 21,4) | 20,9 ± 5,5 s (median 18,6) | p = 0,79, **tidak berbeda** |
| S2 (n = 10 / 10) | 19,7 ± 5,5 s (median 19,8) | **14,0 ± 4,5 s** (median 12,7) | **p = 0,007** |
| Semua (n = 20 / 20) | 20,8 ± 5,0 s (median 19,8) | **17,5 ± 6,1 s** (median 17,3) | p = 0,034 |

Pembanding, uji robot nyata Gudang 2026-10-24 ([[09 Uji Robot Nyata Gudang ver-1-4 Gundang1|catatan 09]]): A\* 21,7 ± 11,4 s, DQfD 19,0 ± 10,3 s. Jalurnya identik dengan kode sekarang.

**Cara membaca tabel ini:**
- **S2 (balik arah masuk teluk): DQfD lebih cepat secara konsisten.** Sebarannya kecil (13,0 ± 1,2 s). Ini sejalan dengan [[14 Waktu Komputasi per Tahapan Hybrid A-star RS vs DQfD Gudang|catatan 14]] bagian 6: bagian model dekat ditempuh lebih singkat daripada bagian RS.
- **S1 (rute bersilangan): tidak berbeda.** Makespan ditentukan oleh interaksi antar-robot (menunggu, rencana ulang), bukan oleh perencananya.
- **Waktu proses tidak menentukan makespan.** Rencana pertama ±40 ms, sekitar 0,2% dari makespan. Percepatan perubahan 2 (±4,5 ms per rencana) tidak terlihat pada makespan.

### 4.3 Kejadian: rencana ulang DQfD di dekat robot lain

Pada S1 ulangan 3, robot ID 01 dengan DQfD terhalang ID 02. Ia merencanakan ulang dengan badan ID 02 sebagai rintangan, sebanyak 5 rencana:

| Rencana | Hasil | Waktu |
|---|---|---:|
| 1 | berhasil (rencana awal) | 57 ms |
| 2 | **gagal** | **1.790 ms** |
| 3 | berhasil, 624 percobaan serah | **1.286 ms** |
| 4 | berhasil | 33 ms |
| 5 | berhasil | 14 ms |

Akibatnya:
- makespan jalan itu 34,8 s, dibanding median 17,8 s;
- jarak antar-badan terdekat **4,4 cm** (tidak bersentuhan, tidak masuk zona).

Ini **bukan efek perubahan 2**, karena jalurnya identik. Ini kelemahan model: ketika badan robot lain berada di dekat jalur, model jauh mencoba ratusan pose serah sebelum menemukan pendaratan yang bebas. Kemungkinan karena situasi itu jarang ada di data latih. Dugaan ini belum diselidiki. Hybrid A\* tidak mengalami kejadian serupa di simulasi ini (rencana ulangnya di catatan 15 hanya 60 ms).

## 5. Temuan dan langkah berikutnya

1. **Perubahan 2 berhasil tanpa mengubah jalur:** DQfD −26% di Windows, startup −1,5 s, tanpa latih ulang.
2. **Bagian khusus DQfD sudah kecil (±22%).** Sisa waktunya kode yang sama dengan Hybrid A\*: cek tabrakan, kinematik, Dijkstra. Itulah perubahan 3 dan 4 di [[16 Rencana Optimasi Waktu Proses Perencana|catatan 16]], yang akan mempercepat kedua perencana sekaligus. A\* akan untung lebih besar (bagian jauhnya 21 ms vs 10 ms DQfD).
3. **Prioritas baru: rencana ulang DQfD di dekat robot lain** (bagian 4.3).
   - Dari sisi waktu proses, ini satu-satunya kejadian yang melewati 1 s.
   - Dari sisi keselamatan, jarak 4,4 cm terdekat dari semua simulasi juga muncul di kejadian ini.
   - Langkah awalnya: kumpulkan pose dan rintangan saat kejadian, lalu ulangi secara offline.

> [!warning] Batasan
> - **Makespan simulasi digabung dari dua sesi** (catatan 15 dan 17). Jalurnya identik, tetapi kondisi mesin berbeda. A\* sendiri 20,2 vs 21,4 s antar-sesi, jadi derau antar-sesi sebesar itu.
> - **S1 dipengaruhi interaksi antar-robot.** 10 ulangan per skenario belum cukup untuk memisahkan efek perencana di S1.
> - **Uji robot nyata dengan kode ini belum dilakukan.**
> - **Waktu rencana di simulasi lebih besar daripada offline**, karena perencana berbagi CPU dengan loop simulasi 20 Hz.

## 6. Sumber angka

| Angka | Sumber |
|---|---|
| Perubahan kode | commit `9cf9eee` (ver-1-5): `drl/jaringan_numpy.py`, `drl/perencana_dqfd.py`, `drl/dqfd.py`, `drl/jaringan.py` |
| Bagian 2 (verifikasi) | skrip sementara di luar repo `verif_numpy.py` (Q dan md5 jalur, numpy vs PyTorch) dan `cek_tanpa_torch.py`; `adaptor_demo --uji` |
| Bagian 3 (waktu proses) | skrip sementara `profil_dqfd.py` (forward numpy dicatat sebagai J6/D4). Windows: 3 run kode baru + 2 run kode lama, diselang-seling. Mac: 1 run kode baru; kode lama dari catatan 15 |
| Bagian 3.2 (startup) | `cek_tanpa_torch.py`, proses Python baru, Gudang S1-ID01, 3 kali. Mac kode lama dari git worktree sementara (`041abf1`) |
| Bagian 4 (makespan) | skrip sementara `sim_gudang.py` → `drl.uji_navigasi.jalankan()`, Windows, 5 ulangan, derau 1,2 cm. Penggabungan dengan simulasi catatan 15, uji Mann-Whitney (scipy) |
| Bagian 4.3 | `rencana` dan `log_akhir` hasil simulasi, S1 ulangan 3 |
