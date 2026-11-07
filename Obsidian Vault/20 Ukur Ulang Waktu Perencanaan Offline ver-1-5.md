# Ukur ulang waktu perencanaan offline dengan program ver-1-5: Hybrid A\* + RS vs DQfD murni

Tanggal: 2026-11-06 · Branch `ver-1-5`, kode commit `91edda3` · Diukur di laptop robot (Windows) · Lanjutan dari [[17 Forward Numpy ver-1-5 dan Perbandingan Waktu Proses serta Makespan|Forward Numpy ver-1-5]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Catatan ini menjadi **sumber Tabel I paper** (ver5): waktu perencanaan offline di semua rute uji, diukur dengan program ver-1-5 (sinar Numba, forward NumPy). Angka offline lama berasal dari program sebelum ver-1-5 dan tidak dipakai lagi.

> [!summary] Ringkasan (run 2, semua rute uji; ID 01 / ID 02)
> - **Rute uji lengkap (379 / 358 rute, 3 peta + 8 peta sintetis per robot):** DQfD tiba 99,7% / 100%, Hybrid A\* + RS 99,5% / 98,3%. Median DQfD **10,2 / 9,5 ms** vs Hybrid A\* 22,0 / 19,3 ms (0,46–0,49×). p90 **56 / 53 ms** vs 222 / 203 ms (0,25–0,26×). DQfD lebih cepat di 73,9% / 73,5% rute.
> - **Tiga peta (Kalibrasi, Rumah2, Rumah; 266 / 241 rute):** DQfD tiba 100% / 100%, Hybrid A\* 100% / 98,3%. Median **9,3 / 9,3 ms** vs 21,2 / 17,4 ms. p90 **37 / 41 ms** vs 189 / 174 ms. Mean 33 / 58 ms vs 74 / 183 ms. DQfD lebih cepat di **72,6% / 75,9%** rute.
> - **Biaya per node:** Hybrid A\* 0,24 ms per ekspansi, DQfD 0,58 ms per langkah (median per rute), jadi satu langkah DQfD sekitar **2,4×** lebih mahal.
> - **Arah hadap awal (503 rute tiga peta yang diselesaikan keduanya):** DQfD lebih cepat di 94% rute 135–180° dan ≥ 170 cm (median rasio waktu 0,07), 81% di 45–135° (0,50), dan hanya 40% di 0–45° (1,12).
> - **0–45° bukan satu kelompok:** Hybrid A\* unggul jelas hanya di rute hampir lurus atau pendek (0–15°: 21%, rasio 1,19; < 100 cm: 30%, 1,15). Di 30–45° keduanya setara (51%, 0,99), sama seperti Gudang S1 dan Gudang2 di [[18 Skenario Gudang Baru dan Simulasi Makespan serta Waktu Proses|catatan 18]] dan [[19 Peta Gudang2 dan Simulasi Skenario Silang dengan Wasit|catatan 19]].
> - **Dua run konsisten:** median berbeda ≤ 1,7 ms dan persentase "DQfD lebih cepat" berbeda ≤ 3 poin.

---

## 1. Metode

- **Mesin:** laptop robot (Windows, AMD Ryzen 7 260, Python 3.14.3), PC yang sama dengan uji robot nyata. Tidak ada program lain yang aktif (beban CPU 0% sebelum mulai; dua proses Python lama dalam keadaan diam).
- **Rute:** semua episode uji (`ep_uji`) di `drl/data/demo_jauh_id0N.npz`. Peta asli dibangun dengan `A.Arena`, peta sintetis dengan `drl.skenario._arena("sintetis:<seed>")`.
- **Perencana:** `rencana.Perencana(arena, profil, jenis)` dengan `jenis` = `astar` dan `dqfd-murni`, dipanggil lewat `rencanakan()` di dalam satu proses (bukan `PerencanaDiProses`). Batas waktu 2 s per percobaan, termasuk percobaan ulang dengan margin cadangan.
- **Pengukuran:** satu rute pemanasan per perencana, lalu setiap rute direncanakan **3 kali** dan diambil **median** waktu jamnya. Jumlah node dari `Perencana.stats["expansions"]` (ekspansi Hybrid A\*, langkah pencarian DQfD).
- **Skrip:** `model_dqn/banding_v15_2026-11-06.py`. Metodenya sama dengan `drl/banding_komputasi.py`, tetapi tanpa penghitung forward PyTorch.

> [!warning] `drl/banding_komputasi.py` rusak di ver-1-5
> Baris 103 memanggil `muat_model(rid)`, padahal fungsi itu sekarang butuh argumen `jenis`. Alat itu berhenti dengan `TypeError` sebelum mengukur apa pun. Penghitung forward yang gagal itu juga tidak berguna lagi, karena perencana memakai `JaringanNumpy`, bukan jaringan PyTorch.

```powershell
cd "Forklift Robot Program"
python -u model_dqn\banding_v15_2026-11-06.py
```

## 2. Hasil per kelompok rute (run 2)

| Kelompok | Robot | Rute | Tiba A\* / DQfD | Median A\* / DQfD [ms] | p90 A\* / DQfD [ms] | Mean A\* / DQfD [ms] | Rasio median / p90 / mean | DQfD lebih cepat |
|---|---|---:|---|---|---|---|---|---:|
| Semua rute uji | ID 01 | 379 | 99,5 / 99,7% | 22,0 / **10,2** | 222 / **56** | 126 / **41** | 0,46 / 0,25 / 0,33 | 73,9% |
| Semua rute uji | ID 02 | 358 | 98,3 / **100%** | 19,3 / **9,5** | 203 / **53** | 189 / **52** | 0,49 / 0,26 / 0,28 | 73,5% |
| Tiga peta | ID 01 | 266 | 100 / 100% | 21,2 / **9,3** | 189 / **37** | 74 / **33** | 0,44 / 0,20 / 0,44 | 72,6% |
| Tiga peta | ID 02 | 241 | 98,3 / **100%** | 17,4 / **9,3** | 174 / **41** | 183 / **58** | 0,53 / 0,23 / 0,31 | 75,9% |
| Peta sintetis | ID 01 | 113 | 98,2 / 99,1% | 28,1 / **12,3** | 330 / **112** | 246 / **61** | 0,44 / 0,34 / 0,25 | 77,0% |
| Peta sintetis | ID 02 | 117 | 98,3 / **100%** | 21,1 / **10,8** | 225 / **63** | 201 / **42** | 0,51 / 0,28 / 0,21 | 68,4% |

Rute yang gagal:

| Robot | Perencana | Peta, episode (waktu sampai gagal) |
|---|---|---|
| ID 01 | Hybrid A\* | sintetis:110009 e580 (4,0 s), sintetis:110029 e1729 (10,5 s) |
| ID 01 | DQfD | sintetis:110049 e2895 (1,7 s) |
| ID 02 | Hybrid A\* | Kalibrasi e2799 (10,5 s), Rumah e521 (0,4 s), Rumah e2822 (8,9 s), Rumah2 e531 (5,7 s), sintetis:120069 e3576 (7,3 s) dan e3582 (7,7 s) |
| ID 02 | DQfD | – |

- Waktu sampai gagal bisa lebih dari 2 s karena `Perencana` mencoba ulang dengan margin cadangan, dan setiap percobaan punya batas 2 s.
- Kegagalan di dekat batas waktu bisa berbeda antar-run dan antar-mesin. Mutu jalur (rasio biaya, balik arah, galat akhir rencana) tidak diukur ulang, karena jalur program ver-1-5 identik dengan jalur model yang sama ([[17 Forward Numpy ver-1-5 dan Perbandingan Waktu Proses serta Makespan|catatan 17]]: 124/124).

## 3. Konsistensi antar-run (tiga peta)

Run 1 hanya mencakup tiga peta. Run 2 mencakup semua rute uji.

| Robot | Run | Median A\* / DQfD [ms] | p90 A\* / DQfD [ms] | Mean A\* / DQfD [ms] | Tiba A\* | DQfD lebih cepat |
|---|---|---|---|---|---:|---:|
| ID 01 | 1 | 19,5 / 9,1 | 177 / 37 | 75 / 33 | 100% | 70,3% |
| ID 01 | 2 | 21,2 / 9,3 | 189 / 37 | 74 / 33 | 100% | 72,6% |
| ID 02 | 1 | 17,7 / 9,8 | 175 / 52 | 186 / 59 | 98,3% | 73,4% |
| ID 02 | 2 | 17,4 / 9,3 | 174 / 41 | 183 / 58 | 98,3% | 75,9% |

Median berbeda ≤ 1,7 ms. Persentase "DQfD lebih cepat" berbeda ≤ 3 poin. Rute Hybrid A\* yang gagal sama di kedua run.

## 4. Biaya per node

| | Hybrid A\* (per ekspansi) | DQfD (per langkah) | Rasio |
|---|---:|---:|---:|
| Tiga peta, median per rute | 0,241 ms | 0,576 ms | 2,39× |
| Semua rute, median per rute | 0,240 ms | 0,570 ms | 2,37× |

DQfD lebih cepat hanya bila Hybrid A\* membuka lebih dari sekitar **2,4×** node. Ini cocok dengan data robot nyata: di Gudang DQfD membuka 6,8× lebih sedikit node dan merencana 2,6× lebih cepat, sedangkan di Gudang2 dengan jadwal hanya 2,0× lebih sedikit node dan waktunya setara (catatan 18, 19, dan Tabel II paper).

## 5. Arah hadap awal

Selisih antara arah hadap awal robot dan arah garis lurus ke tujuan. Rute tiga peta yang diselesaikan kedua perencana (503 rute).

| Kelompok | Rute | DQfD lebih cepat | Median rasio waktu DQfD / A\* | Median node A\* / DQfD |
|---|---:|---:|---:|---|
| 135–180°, jarak ≥ 170 cm | 18 | 94,4% | 0,07 | – |
| 135–180°, semua jarak | 158 | 87,3% | 0,39 | 116 / 13 |
| 45–135° | 234 | 81,2% | 0,50 | 118 / 22 |
| 0–45° | 111 | 39,6% | 1,12 | 22 / 15 |

Kelompok 0–45° dipecah lagi:

| Kelompok | Rute | DQfD lebih cepat | Median rasio | Median node A\* / DQfD | Median waktu A\* / DQfD |
|---|---:|---:|---:|---|---|
| 0–15° | 29 | 20,7% | 1,19 | 11 / 11 | 5,0 / 6,1 ms |
| 15–30° | 47 | 42,6% | 1,03 | 20 / 17 | 8,2 / 9,5 ms |
| 30–45° | 35 | 51,4% | 0,99 | 28 / 17 | 8,4 / 9,1 ms |
| 0–45°, < 100 cm | 53 | 30,2% | 1,15 | 8 / 8 | 5,1 / 5,5 ms |
| 0–45°, 100–150 cm | 42 | 42,9% | 1,11 | 26 / 18 | 8,0 / 9,1 ms |
| 0–45°, ≥ 150 cm | 16 | 62,5% | 0,92 | 42 / 34 | 12,1 / 14,6 ms |
| 25–45°, ≥ 130 cm (mirip Gudang S1) | 19 | 57,9% | 0,99 | 48 / 30 | 12,1 / 13,7 ms |

- **Hybrid A\* unggul jelas hanya di rute hampir lurus atau pendek.** Di sana Hybrid A\* hanya butuh sekitar 10 ekspansi, sama dengan DQfD, sehingga langkah DQfD yang 2,4× lebih mahal membuatnya kalah tipis (sekitar 5 vs 6 ms).
- **Di 30–45° atau jarak ≥ 150 cm keduanya setara.** Gudang S1 (30–35°, sekitar 150 cm, melintasi sekat) dan Gudang2 (sekitar 40°, sekitar 190 cm) berada di titik impas ini. Karena itu di catatan 18 S1 nyata DQfD lebih cepat (12,0 vs 14,8 ms), sedangkan di simulasi Hybrid A\* yang lebih cepat (14,9 vs 16,2 ms).
- **Makin robot membelakangi tujuan, makin besar keunggulan DQfD**, karena heuristik Dijkstra Hybrid A\* tidak memperhitungkan arah hadap.

## 6. Perubahan di paper (ver5)

| Bagian | Sebelum (program lama) | Sesudah (catatan ini) |
|---|---|---|
| Tabel I atas: tiba DQfD / A\* | 99,7 / 99,7% vs 99,5 / 100% | 99,7 / 100% vs 99,5 / 98,3% |
| Tabel I atas: median, p90 | 9,5 / 9,9 vs 23,2 / 18,2 ms; 46 / 56 vs 196 / 184 ms | 10,2 / 9,5 vs 22,0 / 19,3 ms; 56 / 53 vs 222 / 203 ms |
| Tabel I bawah: median, p90, mean | 8,5 / 9,0 vs 19,1 / 17,0; 33 / 55 vs 153 / 174; 24 / 44 vs 62 / 75 | 9,3 / 9,3 vs 21,2 / 17,4; 37 / 41 vs 189 / 174; 33 / 58 vs 74 / 183 |
| DQfD lebih cepat (tiga peta) | 79% / 78% | 73% / 76% |
| Biaya per node | 0,20 / 0,53 ms (2,6×) | 0,24 / 0,58 ms (2,4×) |
| Arah hadap 0–45° | 56%, rasio 0,95 ("setara") | 40%, rasio 1,12; setara hanya di 30–45° |
| Kalimat "Landing takes 12–18%…" | ada | dihapus (tidak diukur ulang) |

> [!warning] Batasan
> - **Waktu di dalam satu proses.** Di robot, perencana berjalan di proses sendiri (`PerencanaDiProses`), yang menambah IPC sekitar 1 ms per rencana. Catatan 18 menunjukkan waktu di robot setara dengan simulasi dalam 15%.
> - **Satu mesin, dua run.** Variasi antar-run sekitar ±5–10% untuk p90 dan mean.
> - **Kegagalan Hybrid A\* bergantung waktu.** Rute yang dekat batas 2 s bisa tiba di satu run dan gagal di run lain.
> - **Kelompok 135–180° dengan ≥ 170 cm hanya 18 rute.**

## 7. Sumber angka

| Angka | Sumber |
|---|---|
| Run 2 (semua rute uji) | `Forklift Robot Program/model_dqn/banding_v15_2026-11-06.json` (1 474 baris: peta, robot, perencana, episode, sukses, ms, node, panjang) |
| Run 1 (tiga peta) | `Forklift Robot Program/model_dqn/banding_v15_tigapeta_2026-11-06.json` |
| Skrip pengukuran | `Forklift Robot Program/model_dqn/banding_v15_2026-11-06.py` |
| Arah hadap dan biaya per node | dihitung dari JSON run 2 dengan pose awal dan tujuan dari `demo_jauh_id0N.npz` (skrip analisis sementara di luar repo) |

`model_dqn/` diabaikan oleh git, jadi ketiga berkas di atas tidak ikut tersinkron lewat commit. Salinannya disimpan manual di `model_dqn/` laptop robot (Windows) dan di Mac.
