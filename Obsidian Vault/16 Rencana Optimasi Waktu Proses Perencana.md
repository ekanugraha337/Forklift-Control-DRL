# Rencana optimasi waktu proses perencana

Tanggal: 2026-11-04 · Branch `ver-1-5` · Dasar angka: [[14 Waktu Komputasi per Tahapan Hybrid A-star RS vs DQfD Gudang|Waktu Komputasi per Tahapan Hybrid A-star RS vs DQfD Gudang]] dan [[15 Sinar Numba ver-1-5 dan Perbandingan Waktu Proses serta Makespan|Sinar Numba ver-1-5 dan Perbandingan Waktu Proses serta Makespan]]

Daftar perubahan yang sudah dan akan dilakukan untuk mempercepat `dqfd-murni` dan Hybrid A\* + RS. Angka waktu diambil dari profil ver-1-5 di laptop uji Windows: peta Gudang, rata-rata 4 kasus. DQfD 19,4 ms per kasus dengan pencatat waktu dan 17,4 ms tanpa pencatat waktu. Hybrid A\* + RS 25,0 ms.

## 1. Sudah selesai

| | Perubahan | Branch | Hasil |
|---|---|---|---|
| ✅ | Observasi dihitung sekali per keadaan (cache `observasi()`) | ver-1-4 | DQfD −31%, jalur identik ([[14 Waktu Komputasi per Tahapan Hybrid A-star RS vs DQfD Gudang\|catatan 14]]) |
| ✅ | **Perubahan 1**: sinar dikompilasi numba | ver-1-5 | sinar 16,65 → 0,80 ms, DQfD 32,6 → 17,4 ms, jalur identik ([[15 Sinar Numba ver-1-5 dan Perbandingan Waktu Proses serta Makespan\|catatan 15]]) |

## 2. Rencana berikutnya

Urut dari dampak terbesar.

| No | Perubahan | Waktu sekarang (Windows) | Dipakai oleh | Perkiraan hemat | Konsekuensi utama |
|---|---|---:|---|---|---|
| 2 | **Forward jaringan dengan numpy, bukan PyTorch** | 3,6 ms (19%), ditambah import torch 1,6–2,6 s saat startup | DQfD saja | ±2,5–3 ms per rencana; startup lebih cepat. Paling terasa di Windows: forward PyTorch di sana 2,5× lebih lambat dari Mac | Q berbeda sekitar 1e-6, jadi jalur bisa sedikit berbeda di kasus yang hampir seri dan perlu diverifikasi di banyak rute. Bobot dari `.pt` dipakai apa adanya, tanpa latih ulang. `visual_neuron.py` dan gerbang mutu mode `dqfd` butuh nilai V |
| 3 | **Cek tabrakan dan kinematik primitif dengan numba** (`pose_margin`, `_ekspansi`, loop ekspansi A\*) | DQfD 6,9 ms (36%); A\* ±21 ms | **A\* dan DQfD** | sebagian besar dari angka itu, di kedua perencana | Harus tetap identik bit per bit dengan loop `HybridAStarPlanner.plan()` (diperiksa `adaptor_demo --uji`). A\* akan untung lebih besar, sehingga selisih waktu DQfD vs A\* bisa berbalik lagi |
| 4 | **Heuristik Dijkstra dengan numba** | 2,75 ms (14%) | A\* dan DQfD | ±2 ms di keduanya | nilai heuristik harus identik |
| 5 | **Distance transform tidak dihitung dua kali** (`peta_dengan` di validasi tujuan dan di `rencanakan`) | ±0,5 ms | A\* dan DQfD | ±0,5 ms | perlu cache peta rintangan per daftar rintangan |
| 6 | **Model dekat tidak menghitung ulang** arah pendekatan dan 44 kata RS | ±0,2 ms | DQfD | ±0,1–0,2 ms | kecil, prioritas rendah |
| 7 | **Overhead pencarian terbaik-dulu** (heap, `step`, simpan/pulihkan keadaan, salinan mask) | 2,3 ms (12%) | DQfD | sebagian kecil | perapian kode Python, tidak mengubah algoritma |
| 8 | **Kurangi langkah yang terbuang** (S1-ID01: 18 dari 44 langkah) | pengali semua biaya di atas | DQfD | belum bisa diperkirakan | perlu latih ulang atau DAgger di rute bersilangan |

Angka "perkiraan hemat" berasal dari profil, **belum diukur**.

## 3. Validasi yang belum dilakukan

| No | Kegiatan | Alasan |
|---|---|---|
| 9 | Uji robot nyata dengan ver-1-5 | jalurnya identik dengan uji 2026-10-24, tetapi waktu rencana saat uji belum diukur dengan kode baru |
| 10 | Simulasi makespan dengan ulangan lebih banyak (≥ 20 per skenario) | selisih makespan simulasi sekarang (DQfD 17,3 vs A\* 20,2 s, 5 ulangan) belum signifikan |

## 4. Saran urutan

1. **Perubahan 2 dulu.** Hanya menyentuh DQfD, dan dampaknya paling besar di Windows, mesin yang menjalankan robot.
2. **Perubahan 3 dan 4 sebagai satu paket.** Keduanya mempercepat A\* dan DQfD sekaligus. Setelahnya, bagian yang dipakai bersama sama-sama dikompilasi untuk kedua perencana, sehingga perbandingannya tetap setara.
3. **Perubahan 5–7** kecil dan bisa dikerjakan kapan saja.
4. **Perubahan 8** perlu latih ulang, jadi sebaiknya dijadwalkan tersendiri.

## 5. Cara verifikasi setiap perubahan

Sama dengan perubahan sebelumnya ([[15 Sinar Numba ver-1-5 dan Perbandingan Waktu Proses serta Makespan|catatan 15]] bagian 2–3):

1. **Hasil identik** dengan versi sebelumnya: observasi atau nilai antara dibandingkan bit per bit pada banyak pose acak. Untuk perubahan 2, yang dicek adalah selisih Q dan jumlah jalur yang berubah.
2. **`python -m drl.adaptor_demo --uji`** lulus semua.
3. **md5 jalur** pada 124 rute (rute uji dataset + 4 kasus Gudang). Jalur yang berbeda diperiksa ulang tanpa batas waktu.
4. **Waktu proses** diukur di Windows dengan run versi lama dan baru diselang-seling, A\* diukur bersama. Lalu di Mac.
5. **Makespan simulasi** (`drl.uji_navigasi.jalankan`, skenario Gudang) bila jalurnya berubah.
