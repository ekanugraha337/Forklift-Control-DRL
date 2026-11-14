# Bukti temuan dan anggaran halaman

Tanggal: 2026-11-06 · Pendukung [[00 Prioritas Perbaikan Paper ver5]] · Kode: branch `ver-1-5`, commit `91edda3` · Path kode relatif terhadap `Forklift Robot Program/`

> [!warning] Dibersihkan 2026-11-07 (aturan ver-1-6)
> Butir yang bergantung pada aturan lama (baseline Hybrid A\* lama, model dekat, data `model_dqn/` yang sudah dihapus, dan angka hasil lama) dihapus. Aturan ver-1-6: Hybrid A\* + RS standar (w = 2,0, h = max(h_RS, h_D), RS setiap ekspansi ke-4); DQfD = model jauh dari demo Hybrid A\* w = 2,0 + tembakan RS di dalam 60 cm; tanpa model dekat.

> [!summary] Ringkasan
> - Memecah kalimat saja menambah 9–20% baris. Halaman baru berkurang kalau angka yang dobel dihapus dari teks.

---

## 1. Penjelasan metode untuk reviewer

| Klaim reviewer | Hasil cek | Bukti |
|---|---|---|
| Heading bebas (5) | Saat operasi, heading dicoba berurutan: arah start→goal, ±30°, ±60°, …, ±180°, 0,4 s per percobaan, untuk kedua planner. Saat membuat dataset dan evaluasi: arah dari pose sekarang ke goal, atau +180° | `rencana.py` (`Perencana._sekali`); `forklift_planner.py` (`_analytic`) |
| Makespan jadwal silang belum formal (4.6) | Rumusnya sudah ada di kode dan memasukkan waktu tunggu (lihat di bawah) | `drl/jadwal_silang.py` |

**Rumus makespan jadwal silang.** Satuan cm; kedua robot dianggap berkecepatan sama; L = panjang kumulatif jalur rencana pertama; Y = robot yang mengalah, P = robot yang lewat.

$$t_Y = \max\left(L_Y[k_\text{tahan}],\ L_P[j_\text{lepas}]\right) + \left(L_Y^\text{total} - L_Y[k_\text{tahan}]\right), \qquad \text{makespan} = \max\left(L_P^\text{total},\ t_Y\right)$$

- k_konf: indeks pertama Y yang badannya < 15 cm dari badan P di sepanjang jalur P. k_tahan: indeks terakhir yang ≥ 10 cm sebelum k_konf.
- j_lepas: indeks terakhir P yang badannya < 15 cm dari sisa jalur Y, ditambah 1. Kalau melewati ujung, P sudah tiba.
- Rumus ini tidak memperhitungkan akselerasi, jeda di titik balik arah, kecepatan mundur yang lebih rendah, dan robot yang meluncur saat berhenti.

## 2. Anggaran halaman dan gaya penulisan

Diukur dengan IEEEtran yang sama dengan paper: lebar kolom 252 pt, baris 12 pt, 56 baris per kolom.

### 2.1 Posisi sekarang

- ver5 = 7 halaman. Acuan [18]–[25] (27 baris acuan) meluber ke hal. 7, setara ±20 baris teks. Kalau batas 6 halaman termasuk acuan, ver5 harus dipangkas ±20 baris.
- Satu acuan ≈ 2,5 baris teks, jadi 10–12 acuan baru ≈ +25–30 baris.
- Bagian crossing schedule: II-D 16 baris, IV-F 30 baris, V-C (tiga paragraf Gudang2) 32 baris. Totalnya ±77 baris, ditambah dua blok Tabel II dan separuh Fig. 3.

### 2.2 Uji coba gaya penulisan

| Paragraf | Asli | Kalimat dipecah saja | Dipecah + rincian ke tabel |
|---|---|---|---|
| V-C *Warehouse: separate routes* | 23 baris | 25 (+2) | 12 teks + ±10 tabel S1–S4 baru ≈ 22 (−1) |
| V-B *When DQfD is faster* | 15 baris | 18 (+3) | 13 teks + 4 baris di Tabel I ≈ 17 (+2) |

- Jumlah baris versi asli sama persis dengan PDF ver5.
- Tabel baru memakan ±4 baris (caption, garis, jarak float), ditambah satu baris per baris data.
- Kepadatan ver5: rata-rata 27 kata per kalimat (kesimpulan 35), 44 titik koma, 62 tanda kurung berisi angka, dan 149 angka dalam ±1.400 kata di bagian hasil.

### 2.3 Yang benar-benar menghemat

| Perubahan | Hemat |
|---|---:|
| Abstrak ±260 → 150–200 kata | 5–8 baris |
| Rincian per skenario di V-C dipindah ke tabel S1–S4 | netral; tabelnya jadi "gratis" |
| Crossing schedule dipadatkan (bila memilih cakupan 2a) | ±50 baris (±1 kolom) |
