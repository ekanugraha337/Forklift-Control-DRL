# Baseline Hybrid A\* yang adil dan ablasi DQfD (Tahap 1.1)

Tanggal: 2026-11-07 · Branch `ver-1-5` (commit `91edda3` + perubahan Tahap 1.1, belum di-commit) · Diukur di MacBook Pro M1 Pro, karena laptop robot tidak bisa dihubungi · Lanjutan dari [[20 Ukur Ulang Waktu Perencanaan Offline ver-1-5|catatan 20]] · Rencana kerja: [[00 Prioritas Perbaikan Paper ver5]] butir 1 (review pembimbing 4.1 dan 4.2)

Catatan ini menjadi **sumber Tabel I paper ver6**. Pertanyaannya: apakah keunggulan DQfD bertahan bila Hybrid A\* diberi heuristik dan bobot yang adil?

> [!summary] Ringkasan (737 rute uji kedua robot, satu run, M1 Pro)
> - **Baseline yang kuat menutup sebagian besar selisih waktu.** Hybrid A\* dengan h = max(RS tanpa rintangan, Dijkstra) dan w = 2,0 (A3/A4) membuka **26 / 504 node** (median / p90), dibanding 86 / 1137 untuk Hybrid A\* robot (A0). Waktunya 5,7 / 63 ms, sedangkan A0 13,1 / 129 ms.
> - **DQfD tetap butuh langkah paling sedikit:** 19 / 97. Jumlah ini 4,5× / 11,7× lebih sedikit dari A0 dan 1,4× / 5,2× lebih sedikit dari A4. Tetapi satu langkah DQfD (0,65 ms) di mesin ini 2,8× lebih mahal daripada satu ekspansi A4 (0,24 ms). Akibatnya **A4 lebih cepat di median (5,7 vs 11,4 ms) dan setara di p90 (63 vs 61 ms)**. DQfD lebih cepat dari A4 hanya di 23% rute, dan dari A0 di 53% rute.
> - **Di rute tersulit DQfD tetap unggul:** pada 10% rute dengan node A4 terbanyak (74 rute), DQfD lebih cepat di 70% rute (median 142 vs 180 ms; 249 vs 1415 node).
> - **Heuristik RS menutup kelemahan arah hadap.** Pada rute 135–180° dan ≥ 170 cm, DQfD lebih cepat dari A0 di 97% rute (rasio 0,13; 1273 vs 34 node), tetapi dari A4 hanya di 34% rute (rasio 1,40; 74 vs 34 node). Kritik reviewer 4.1 terbukti.
> - **Bobot w = 2,0 saja (A1) sudah memotong node A0 2,3× di median**, dengan harga biaya p90 1,27 (lebih mahal 27%). DQfD dengan w = 2,0 juga membayar biaya serupa (p90 1,19).
> - **Ablasi:** saringan handover tidak berpengaruh, baik di Hybrid A\* (A4 = A3) maupun di DQfD (D2 = D0). DQfD dengan w = 1,3 (D1) lebih lambat di ekor (p90 100 vs 61 ms). Tanpa backtracking (D3) gagal di 4 rute, bukan 1. Tetapi 9 rute yang diselesaikan backtracking rata-rata memuat 19 kali ganti arah.
> - **Replay rencana pertama uji robot nyata:** di Gudang nyata (24 rencana non-S2), A4 masih menyisipkan mundur di tengah rute (> 60 cm dari tujuan) di **8 rencana**, A0 di 6, dan DQfD di **0**. Jalur A4 juga 9% lebih panjang (186 vs 171 cm). Jadi klaim makespan rute terpisah tetap berlaku terhadap baseline yang kuat, dan **uji robot nyata ulang tidak diperlukan**.
> - **Offline, DQfD tidak lebih jarang ganti arah:** 38% jalur DQfD punya ganti arah > 60 cm dari tujuan, A0 35%, A4 46%. "Tidak ada mundur di tengah rute" hanya berlaku di rute Gudang, bukan sifat umum (review 2.4).
> - **Simulasi closed-loop (bagian 11):** di Gudang (rute terpisah), jumlah makespan DQfD **115,0 s**, A0 140,1 s, A3 **156,7 s**. DQfD 27% lebih pendek dari A3 dan 18% dari A0. A3 lebih lambat dari A0 karena jalurnya lebih banyak mundur dan berbelok di S3 dan S4. Di Gudang2 hasilnya campuran: dengan wasit saja, A3 32,2 s, DQfD 34,6 s, A0 48,8 s; dengan jadwal silang, DQfD 24,3 s, A0 27,4 s, A3 29,9 s.

---

## 1. Tujuan

Review pembimbing 4.1 menyebut tiga hal yang membuat perbandingan tidak adil, dan ketiganya terbukti di kode ([[01 Bukti Temuan dan Anggaran Halaman]] bagian 1):

1. Heuristik Hybrid A\* hanya peta Dijkstra yang buta arah hadap, tanpa suku RS/Dubins seperti Dolgov dkk. [1].
2. Bobot heuristik berbeda: Hybrid A\* memakai w = 1,3, DQfD memakai w = 2,0.
3. Saringan handover ("peta Dijkstra tanpa detour") hanya dipakai DQfD.

Review 4.2 juga meminta ablasi DQfD tanpa saringan handover, tanpa backtracking, dan dengan variasi w.

## 2. Perubahan kode

Nilai bawaan tidak berubah, jadi `navigasi.py` dan uji robot tetap memakai A0 dan D0.

| Berkas | Perubahan |
|---|---|
| `heuristik_rs.py` (baru) | Tabel biaya RS tanpa rintangan: biaya Eq. (2) jalur RS termurah (rumus `_rs_cost`: maju 1×, mundur 2×, ganti arah 32,4, kelengkungan, ganti kelengkungan; tanpa penalti clearance) dari pose node ke pose tujuan, di kerangka tujuan (u, v, ψ). Grid ±340 cm, 5 cm, 72 sudut (5°) = 137 × 137 × 72. Dibangun sekali dalam **6 s** (8 proses, simetri cermin), lalu disimpan di `model_dqn/tabel_rs_bbf617e534.npz`. Galat pembulatan grid: median 0,4%, p90 0,9%. Satu pembacaan **0,43 µs**. Di luar grid dipakai jarak lurus |
| `forklift_planner.py` | `HybridAStarPlanner(h_rs=None, serah_saring=False, serah_paksa=0)`. Metode baru `_h_cari()` = max(Dijkstra, RS) bila `h_rs` ada. `_h()` tetap Dijkstra saja karena dipakai sebagai observasi model DQfD. `_serah_ok()` = saringan handover yang sama dengan `LingkunganRencana.bebas_ke_tujuan` |
| `rencana.py` | `buat_perencana(arena, kendaraan, opsi)` dan `Perencana(..., opsi=...)`. Opsi A\*: `h_weight`, `heuristik` (`"dijkstra"`/`"rs"`), `saring_serah`. Opsi DQfD diteruskan ke `DQfDPlanner`. Hybrid A\* di dalam DQfD (pustaka gerak dan cadangan) selalu konfigurasi robot |
| `drl/perencana_dqfd.py` | `DQfDPlanner(bobot=2.0, saring_serah=True, metode=...)`; `cari()` dan `rencanakan_dua()` menerima `bobot`. Tanpa backtracking = `metode="terbaik"` (seluruh batas waktu untuk terbaik-dulu) |
| `drl/lingkungan.py` | Atribut `saring_serah` (bawaan True). False = model dekat mencoba mendarat di setiap pose < 60 cm |
| `drl/banding_baseline.py` (baru) | Benchmark semua varian dalam satu run |
| `drl/ringkas_baseline.py` (baru) | Ringkasan per robot, tiga peta, dan sudut heading |
| `drl/ulang_rencana_pertama.py` (baru) | Replay rencana pertama uji Gudang/Gudang2 per varian |

## 3. Metode

- **Mesin:** MacBook Pro **Apple M1 Pro**, 16 GB, Python 3.13.18. Laptop robot (10.210.1.206) tidak bisa dihubungi lewat SSH maupun ping saat itu. Atas keputusan pengguna, semua angka offline ver6 diambil dari Mac. Node, biaya, ganti arah, dan arrival tidak bergantung pada mesin. Waktu (ms) bergantung pada mesin (lihat bagian 9).
- **Rute:** semua episode uji `drl/data/demo_jauh_id0N.npz`, yaitu 379 (ID 01) + 358 (ID 02) = **737 rute**. Sebanyak 507 rute di Kalibrasi, Rumah2, Rumah, dan 230 rute di 16 peta sintetis yang tidak pernah dilihat saat latih.
- **Perencana:** `rencana.Perencana(...).rencanakan()`, jalur kode yang sama dengan robot (sapuan heading bebas, margin cadangan). Batas waktu 2 s per percobaan untuk semua varian.
- **Pengukuran:** satu rute pemanasan per varian per peta, lalu setiap rute direncanakan **3 kali** dan diambil **median** waktu jam. Node = jumlah ekspansi Hybrid A\* atau langkah DQfD di **semua** panggilan planner dalam satu `rencanakan()`, termasuk sapuan heading bebas (catatan 20 hanya menghitung panggilan terakhir).
- **Biaya:** Eq. (2) dihitung ulang dari daftar pose jalur dengan **satu fungsi untuk semua varian** (`biaya_jalur`). Kelengkungan diambil dari selisih yaw dan dibulatkan ke tingkat kemudi planner, ditambah penalti clearance per panjang. Rasio biaya = biaya varian / biaya A0 pada rute yang keduanya berhasil.
- **Ganti arah "di tengah":** titik balik arah yang lebih dari 60 cm dari tujuan.
- Lama run: 18 menit (1073 s).

```bash
cd "Forklift Robot Program"
python heuristik_rs.py                       # bangun & uji tabel RS (sekali)
python -m drl.banding_baseline               # 737 rute x 9 varian x 3 ulangan
python -m drl.ringkas_baseline model_dqn/banding_baseline_2026-11-07_m1pro.json
python -m drl.ulang_rencana_pertama          # replay rencana pertama uji dua robot
```

Varian:

| Kode | Perencana | Heuristik | w | Saringan handover | Backtracking |
|---|---|---|---:|---|---|
| **A0** | Hybrid A\* + RS (dipakai robot) | Dijkstra | 1,3 | – | – |
| A1 | Hybrid A\* + RS | Dijkstra | 2,0 | – | – |
| A2 | Hybrid A\* + RS | max(RS, Dijkstra) | 1,3 | – | – |
| A3 | Hybrid A\* + RS | max(RS, Dijkstra) | 2,0 | – | – |
| A4 | Hybrid A\* + RS ("adil") | max(RS, Dijkstra) | 2,0 | ya | – |
| **D0** | DQfD (dipakai robot) | Q model jauh | 2,0 | ya | ya |
| D1 | DQfD | Q model jauh | 1,3 | ya | ya |
| D2 | DQfD | Q model jauh | 2,0 | tidak | ya |
| D3 | DQfD | Q model jauh | 2,0 | ya | tidak |

## 4. Hasil gabungan (737 rute, kedua robot)

| Varian | Tiba | Waktu median / p90 / mean [ms] | Node median / p90 | Rasio biaya median / p90 | Ganti arah per jalur | Jalur dgn ganti arah di tengah | ms per node |
|---|---:|---|---|---|---:|---:|---:|
| A0 | 99,9% | 13,1 / 129 / 69 | 86 / 1137 | 1,000 / 1,00 | 1,89 | 35,3% | 0,155 |
| A1 | 100% | 7,2 / **50** / 33 | 37 / 337 | 1,000 / 1,27 | 1,94 | 33,1% | 0,200 |
| A2 | 99,7% | 8,1 / 105 / 61 | 45 / 827 | 0,993 / 1,13 | 1,74 | 41,6% | 0,188 |
| A3 | 99,9% | **5,7** / 63 / 43 | 26 / 504 | 1,002 / 1,21 | 1,80 | 46,3% | 0,234 |
| A4 | 99,9% | **5,7** / 63 / 43 | 26 / 504 | 1,002 / 1,21 | 1,80 | 46,3% | 0,236 |
| D0 | 99,9% | 11,4 / 61 / 51 | **19 / 97** | 1,000 / 1,19 | 2,14 | 38,2% | 0,652 |
| D1 | 99,9% | 13,1 / 100 / 61 | 21 / 155 | 1,000 / 1,14 | 2,20 | 38,3% | 0,642 |
| D2 | 99,9% | 11,3 / 63 / 51 | 19 / 97 | 1,000 / 1,19 | 2,14 | 38,3% | 0,649 |
| D3 | 99,5% | 11,3 / 63 / 53 | 19 / 97 | 1,000 / 1,17 | 1,94 | 37,7% | 0,649 |

- Galat akhir rencana 0 untuk semua varian dan semua rute yang berhasil.
- Rute gagal: A0 1 (sintetis:120069 e3582), A2 2, A3 1, A4 1 (Rumah e521, ID 02), D0 1 (sintetis:110049 e2895, ID 01), D1 1, D2 1, D3 4.
- **Ekor waktu:** p95 / p99 / maks: A0 246 / 801 / 7654 ms, A4 180 / 612 / 3930 ms, D0 191 / 872 / 3592 ms.
- **Rute tersulit bagi A4** (node ≥ p90 A4, 74 rute): A4 median 1415 node / 180 ms, D0 249 langkah / 142 ms. DQfD lebih cepat di 70% rute ini.
- **Peta sintetis (230 rute) vs tiga peta (507 rute):** D0 tiba 99,6% / 100%, rasio biaya p90 1,26 / 1,16; A4 tiba 100% / 99,8%, p90 1,21 / 1,18; A0 tiba 99,6% / 100%.

Per robot (median / p90 ms; median node):

| Robot | A0 | A1 | A3 | A4 | D0 |
|---|---|---|---|---|---|
| ID 01 (379) | 14,0 / 131; 98 | 7,5 / 56; 41 | 6,2 / 66; 30 | 6,2 / 68; 30 | 11,9 / 64; 19 |
| ID 02 (358) | 12,1 / 124; 79 | 7,0 / 45; 36 | 5,2 / 57; 22 | 5,3 / 58; 22 | 11,0 / 57; 18 |

## 5. Menurut sudut heading awal (D0 vs varian, rute yang keduanya tiba)

| Kelompok rute | n | vs A0: D0 lebih cepat, rasio waktu, node A0 / D0 | vs A1 | vs A4 |
|---|---:|---|---|---|
| 135–180° dan ≥ 170 cm | 29 | **97%**, 0,13, 1273 / 34 | 59%, 0,79, 207 / 34 | 34%, 1,40, 74 / 34 |
| 45–135° | 350 | 57%, 0,89, 116 / 23 | 28%, 1,35, 50 / 23 | 22%, 1,78, 29 / 23 |
| 0–45° | 169 | 17%, 1,52, 24 / 15 | 11%, 1,76, 16 / 15 | 16%, 1,81, 13 / 15 |

- Pola terhadap A0 sama dengan catatan 20 di laptop robot (135–180°: 94%, rasio 0,07). Heuristik RS memberi Hybrid A\* informasi arah hadap yang sebelumnya hanya dimiliki model jauh, sehingga node A4 di kelompok 135–180° turun dari 1273 ke 74.
- Di 0–45° jumlah node hampir sama, jadi selisih waktu berasal dari biaya per langkah.

## 6. Ablasi

- **Saringan handover:** tidak berpengaruh pada test time. A4 sama persis dengan A3 (node, arrival, biaya), dan D2 sama dengan D0. Penghematan RS yang gagal terlalu kecil untuk terlihat. Peran saringan tinggal di pelatihan (episode model jauh berakhir di pose yang memang bisa didaratkan). Ini perlu masuk ke butir 5 Tahap 2 (posisi kontribusi 3).
- **w = 1,3 untuk DQfD (D1):** lebih lambat dan lebih banyak langkah (p90 155 vs 97; 100 vs 61 ms), biayanya sedikit lebih murah (p90 1,14 vs 1,19). Bobot 2,0 tetap pilihan yang tepat untuk DQfD.
- **Tanpa backtracking (D3):** 4 rute gagal, D0 hanya 1. Dengan seluruh batas waktu untuk terbaik-dulu, 6 dari 10 rute yang gagal di terbaik-dulu D0 bisa diselesaikan. D0 memakai backtracking di **9 rute** dengan rata-rata **19 ganti arah** per jalur. Rute inilah yang menaikkan rata-rata ganti arah D0 (2,14; tanpa 9 rute itu 1,93).
- **w = 2,0 untuk Hybrid A\* (A1):** node median turun 2,3× dan p90 3,4×. Ini penghemat terbesar per satu perubahan. Harganya rasio biaya p90 1,27.

## 7. Replay rencana pertama uji dua robot

`drl/ulang_rencana_pertama.py`, 52 trial (Gudang dan Gudang2, sim dan nyata), 2 robot per trial. Pose start = titik pertama jalur rencana pertama yang tercatat (pose setelah pulang, diukur kamera). **Replay setia: 104/104** jalur varian yang dipakai di trial identik dengan rekaman.

| Run | Varian | Node rata-rata [min–maks] | Panjang [cm] | Rencana dengan mundur di tengah (non-S2) |
|---|---|---|---:|---:|
| Gudang nyata (32 rencana) | A0 | 137,6 [43–402] | 180,5 | 6 / 24 |
| | A1 | 40,1 [14–168] | 186,1 | 6 / 24 |
| | A3 | 46,4 [9–173] | 186,5 | 9 / 24 |
| | A4 | 46,6 [9–173] | 186,0 | 8 / 24 |
| | D0 | 19,8 [8–43] | 171,5 | **0 / 24** |
| Gudang sim (32) | A0 / A4 / D0 | 142,2 / 47,2 / 22,3 | 183,9 / 182,7 / 171,3 | 9 / 7 / 1 dari 24 |
| Gudang2 wasit nyata (12) | A0 / A4 / D0 | 98,1 / 49,8 / 35,2 | 241,5 / 224,8 / 234,7 | 0 / 0 / 0 |
| Gudang2 jadwal nyata (12) | A0 / A4 / D0 | 72,1 / 48,1 / 33,7 | 239,0 / 225,7 / 236,0 | 0 / 0 / 0 |

- A0 dan D0 cocok dengan kolom Nodes Tabel II (137 dan 20 di Gudang nyata).
- Baseline yang kuat **tidak** menghilangkan manuver mundur di tengah rute Gudang. Hanya DQfD yang tidak pernah melakukannya, dan jalurnya paling pendek. Maka keunggulan makespan 16% di rute terpisah kemungkinan bertahan terhadap A4. Uji robot nyata ulang tidak diperlukan untuk menjawab review 4.1.
- Di Gudang2 (rute silang), tidak ada varian yang mundur di tengah rute. Klaim jadwal silang tidak bergantung pada planner.

## 8. Ganti arah offline vs robot nyata

- Offline, jalur DQfD **tidak** lebih jarang ganti arah: 2,14 per jalur (1,93 tanpa rute backtracking) vs A0 1,89. Jalur dengan ganti arah di tengah: 38% vs 35%.
- Jadi "DQfD tidak pernah menyisipkan mundur di tengah rute" adalah hasil di rute Gudang (posisi start di lajur, tujuan di ujung lajur), bukan sifat umum. Kalimat di abstrak dan V-C harus dibatasi ke "in our trials", sesuai review 2.4 dan butir 7 Tahap 2.

## 9. Mesin: M1 Pro vs laptop robot

| | A0 per ekspansi | D0 per langkah | Rasio | D0 lebih cepat dari A0 |
|---|---:|---:|---:|---:|
| Laptop robot, Ryzen 7 260 (catatan 20) | 0,24 ms | 0,58 ms | 2,4× | 73–74% rute |
| MacBook Pro M1 Pro (catatan ini) | 0,155 ms | 0,652 ms | 4,2× | 53% rute |

- Di M1 Pro, Python murni (ekspansi Hybrid A\*) relatif lebih cepat, sedangkan langkah DQfD (sinar Numba + forward NumPy) relatif lebih lambat. Karena itu persentase "DQfD lebih cepat" di catatan ini **tidak** bisa dibandingkan langsung dengan catatan 20 dan Tabel II (yang diukur di laptop robot).
- Ekspansi A4 lebih mahal daripada A0 (0,236 vs 0,155 ms) karena pembacaan tabel RS dan heuristik ganda. Di laptop robot rasio langkah DQfD terhadap ekspansi A4 kemungkinan lebih kecil dari 2,8×, tetapi **belum diukur**.

> [!todo] Ulangi di laptop robot
> Satu perintah: `python -m drl.banding_baseline` di folder `Forklift Robot Program` laptop robot. Salin dulu `heuristik_rs.py`, berkas yang berubah, dan `model_dqn/tabel_rs_bbf617e534.npz`. Hasilnya menggantikan kolom waktu Tabel I. Node, biaya, dan arrival seharusnya sama.

## 10. Dampak ke paper (ver6)

- **Tabel I** diganti tabel 9 varian (gabungan 737 rute, satu run). Bagian "tiga peta" dihapus.
- **III-C:** varian A0–A4 dijelaskan, dan tabel RS disebut (6 s, 0,4 µs per baca).
- **V-A:** waktu offline diukur di M1 Pro, waktu robot di laptop robot. Node dilaporkan sebagai metrik yang tidak bergantung pada implementasi.
- **V-B:** ditulis ulang. Terhadap A0, DQfD 4,5× lebih sedikit node di median. Terhadap A4 hanya 1,4× di median tetapi 5,2× di p90. A4 lebih cepat di median dan setara di p90. Ablasi D1–D3 dan A4 ikut dilaporkan.
- **V-C:** replay rencana pertama (A4 8/24 vs DQfD 0/24 mundur di tengah rute), dan pembatasan "in our trials".
- **Abstrak, pendahuluan, kesimpulan:** klaim "DQfD plans faster on 73–76% of routes" diganti dengan klaim langkah pencarian dan ekor waktu.
- **Tabel II (setelah bagian 11):** baris A3 ditambahkan di ketiga kelompok, hanya kolom sim: makespan 19,6 / 32,2 / 29,9 s, rencana pertama 15,2 / 17,5 / 14,3 ms (M1 Pro), node 51 / 54 / 46 (replay, 24/24 identik). Catatan kaki † memuat A0 dan D0 dari run Mac yang sama. Huruf tebal tetap membandingkan A0 dan D0. Planner diberi label A0 / A3 / D0.
- **V-C:** paragraf Warehouse ditambah hasil simulasi A3 (19,6 vs 17,5 vs 14,4 s; mundur 18–20 vs 6–9 cm di S3/S4; −27%). Paragraf Warehouse2 ditambah A3 setara DQfD dengan wasit saja (32,2 vs 34,6 s) dan jadwal hanya −7% untuk A3. Abstrak dan kesimpulan ditambah "27% shorter than with the stronger Hybrid A* in simulation". V-A: "A3 runs in simulation only".

## 11. Simulasi closed-loop A0 vs A3 vs D0

Untuk mengukur **makespan** A3 tanpa robot fisik, skenario yang sama dengan uji nyata dijalankan dengan stack navigasi lengkap (perencana per proses, pure pursuit, wasit, jadwal silang) dan robot simulasi, tanpa derau. Ketiga perencana ada di run yang sama, dengan 2 ulangan per skenario, dijalankan di M1 Pro tanpa jendela.

- **Kode:** jenis perencana baru `astar-rs` di `rencana.py`. `OPSI_JENIS["astar-rs"]` = heuristik RS dan w = 2,0, yaitu A3. Bawaan robot tetap `astar`.
- **Jadwal silang** untuk ketiga perencana: `uji_nyata/Gudang2_jadwal_tahap11.json`. Jadwal A0 dan DQfD identik dengan `Gudang2_jadwal.json` (uji nyata). Untuk A3, ID 01 yang mengalah, dengan perkiraan makespan 374,8 cm (DQfD 374,7 cm).
- **Run:** `uji_nyata/20261107_135006_Gudang_sim` (24 percobaan), `20261107_140808_Gudang2_sim` (wasit, 6), `20261107_141602_Gudang2_sim` (jadwal, 6). Lama total 33 menit.
- **Pemeriksaan konsistensi:** A0 dan DQfD di run Mac ini cocok dengan simulasi lama di laptop robot (Tabel II): Gudang 17,5 vs 17,7 s dan 14,4 vs 14,2 s.

```bash
cd "Forklift Robot Program"
python -m drl.jadwal_silang --skenario uji_nyata/Gudang2.json --perencana astar,astar-rs,dqfd-murni --keluar uji_nyata/Gudang2_jadwal_tahap11.json
python drl/uji_tanpa_jendela.py --skenario uji_nyata/Gudang.json  --simulasi --tanpa-konfirmasi --ulang 2 --perencana astar,astar-rs,dqfd-murni
python drl/uji_tanpa_jendela.py --skenario uji_nyata/Gudang2.json --simulasi --tanpa-konfirmasi --ulang 2 --perencana astar,astar-rs,dqfd-murni
python drl/uji_tanpa_jendela.py --skenario uji_nyata/Gudang2.json --simulasi --tanpa-konfirmasi --ulang 2 --perencana astar,astar-rs,dqfd-murni --jadwal uji_nyata/Gudang2_jadwal_tahap11.json
python drl/ringkas_uji_varian.py uji_nyata/20261107_135006_Gudang_sim uji_nyata/20261107_140808_Gudang2_sim uji_nyata/20261107_141602_Gudang2_sim
```

Hasil (mean per percobaan ± SD; rencana pertama di M1 Pro):

| Run | Perencana | Tiba | Makespan [s] | Jumlah makespan [s] | Rencana-1 [ms] | Replan (gagal) | Rencana-1 dgn mundur di tengah (non-S2) |
|---|---|---|---|---:|---:|---|---:|
| Gudang S1–S4 | A0 | 8/8 | 17,5 ± 4,9 | 140,1 | 31,2 | 0 | 5/12 |
| | **A3** | 8/8 | 19,6 ± 2,1 | 156,7 | **15,2** | 0 | 5/12 |
| | **D0** | 8/8 | **14,4 ± 6,0** | **115,0** | 24,2 | 0 | **0/12** |
| Gudang2 wasit | A0 | 2/2 | 48,8 ± 18,0 | 97,5 | 21,8 | 6 (1) | 0/4 |
| | A3 | 2/2 | **32,2 ± 4,9** | 64,3 | **17,5** | 22 (2) | 0/4 |
| | D0 | 2/2 | 34,6 ± 4,9 | 69,3 | 36,3 | 21 (3) | 0/4 |
| Gudang2 jadwal | A0 | 2/2 | 27,4 ± 1,0 | 54,8 | 22,4 | 2 (0) | 0/4 |
| | A3 | 2/2 | 29,9 ± 2,2 | 59,9 | **14,3** | 2 (0) | 0/4 |
| | D0 | 2/2 | **24,3 ± 3,8** | 48,6 | 31,2 | 1 (0) | 0/4 |

Tidak ada pelanggaran zona. Galat akhir 1,6–3,4 cm / 2,4–7,7°.

Makespan Gudang per skenario [s]:

| | S1 | S2 | S3 | S4 |
|---|---:|---:|---:|---:|
| A0 | 18,6 | 23,9 | 12,6 | 14,9 |
| A3 | 17,2 | **21,5** | 20,2 | 19,4 |
| D0 | **10,3** | 24,0 | **11,2** | **12,0** |

**Kenapa A3 lebih lambat di S3 dan S4.** Rata-rata jalur rencana pertama:

| Skenario | Perencana | Maju [cm] | Mundur [cm] | Total belok [°] | Waktu tempuh [s] |
|---|---|---:|---:|---:|---:|
| S3 | A0 / A3 / D0 | 157 / 161 / 164 | 6 / **20** / 6 | 207 / **266** / 205 | 11,4 / **18,2** / 11,1 |
| S4 | A0 / A3 / D0 | 155 / 152 / 154 | 9 / **18** / 7 | 234 / **257** / 212 | 12,3 / **15,8** / 11,4 |
| S1 | A0 / A3 / D0 | 200 / 176 / 166 | 18 / 11 / 2 | 250 / 170 / 132 | 18,5 / 13,7 / 10,1 |
| S2 | A0 / A3 / D0 | 83 / 139 / 82 | 110 / **50** / 112 | 180 / 223 / 182 | 23,6 / **20,5** / 23,8 |

- w = 2 dan heuristik RS membuat A3 lebih rakus mengikuti kurva RS tanpa rintangan. Jalurnya memuat 2–3× lebih banyak mundur dan lebih banyak belok. Robot jauh lebih lambat saat mundur (`REVERSE_SPEED` 4000 ≈ 10 cm/s, sedangkan `FORWARD_SPEED` 10000 ≈ 31 cm/s). Setiap titik balik menambah jeda 0,6 s (`CUSP_PAUSE_S`) dan perlambatan menjelang ujung segmen, dan tikungan juga memperlambat.
- Di S2 (berbalik 180° di lajur), A3 justru menemukan manuver dengan mundur lebih sedikit (50 vs 110 cm), sehingga paling cepat.
- DQfD paling cepat di S1, S3, dan S4, dan setara A0 di S2.

**Kesimpulan:**
- Rute terpisah (Gudang): keunggulan makespan DQfD **bertahan** terhadap baseline yang adil, bahkan lebih besar: −27% terhadap A3, −18% terhadap A0 (simulasi lama −20%, robot nyata −16%).
- Rute silang (Gudang2): dengan wasit saja, A3 setara DQfD (32,2 vs 34,6 s, SD 4,9 s, 2 percobaan). Dengan jadwal silang, DQfD terbaik. Jadwal silang memperbaiki A3 hanya 7% (A0 44%, DQfD 30%), karena tanpa jadwal pun A3 sudah tidak banyak memutar. Hanya 2 percobaan per sel, jadi Gudang2 hanya bisa dibaca sebagai tren.
- Waktu rencana pertama di M1 Pro: A3 tercepat (14–18 ms). Planner tetap ≤ 0,8% dari makespan, jadi selisih makespan datang dari bentuk jalur.

## Berkas

- Hasil: `model_dqn/banding_baseline_2026-11-07_m1pro.json` (+ `.log`), `model_dqn/ringkas_baseline_2026-11-07_m1pro.json`, `model_dqn/ulang_rencana_pertama_2026-11-07_m1pro.json`
- Simulasi closed-loop: `model_dqn/sim_closed_loop_A0_A3_D0_2026-11-07_m1pro.json` (+ `.log`), folder run di `uji_nyata/20261107_*_sim`, skrip `drl/uji_tanpa_jendela.py` dan `drl/ringkas_uji_varian.py`
- Tabel RS: `model_dqn/tabel_rs_bbf617e534.npz`
- `model_dqn/` diabaikan git, jadi berkas ini hanya ada di Mac.
