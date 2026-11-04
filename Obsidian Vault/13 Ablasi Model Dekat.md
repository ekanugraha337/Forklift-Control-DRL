# Ablasi urutan model dekat

Tanggal: 2026-11-03 · Branch `ver-1-4` · Model: `id01_dekat.pt` / `id02_dekat.pt` (langkah 5.000), model jauh sesuai [[07 Struktur DQfD Full|Struktur DQfD Full]] bagian 13 · Paper: Bagian VI-B dan Tabel V

Catatan ini menjawab satu pertanyaan: **apakah urutan Q hasil belajar model dekat lebih baik daripada aturan sederhana tanpa jaringan?** Bagian yang "diangkat" hanya **urutan** pengecekan kata. Rumus RS (geometri kata), *mask*, dan cek tabrakan dibuat sama untuk semua metode.

> [!summary] Ringkasan
> - **Urutan Q setara dengan urutan biaya rumus RS (analitik).** Cek tabrakan 1,54 vs 1,56 (ID 01) dan 1,36 vs 1,40 (ID 02), biaya jalur sama (rasio median dan p90 1,00). Q sekitar 0,06 ms lebih lambat per percobaan karena satu *forward pass* lebih mahal daripada mengurutkan 88 angka.
> - **Jaringan memang belajar**: urutan acak butuh sekitar 2× cek dan jalurnya 14–22% lebih mahal di median. Tetapi jaringan **tidak melampaui** urutan biaya.
> - **Aturan demonstrator (≤ 6 kandidat termurah) gagal di 5/378 (ID 01) dan 5/357 (ID 02)** pose serah yang sebenarnya punya kata bebas. Pada tujuh pose yang tujuannya berarah, arah hadap pose serah berbeda **> 90°** dari arah hadap tujuan. Model dekat dan urutan analitik mendarat di semua pose itu karena **mencoba semua kata sah**, bukan karena hasil belajar.
> - **Hanya 17% / 13% pose serah yang punya kata bebas.** Di sisanya, Q dan analitik mengecek semua kata sah (10,1 / 8,0 cek). Jadi ongkos pendaratan lebih ditentukan oleh **pemilihan pose serah oleh model jauh** daripada oleh urutan model dekat.
> - Nilai model dekat ada pada **ketepatan yang dijamin rumus RS** dan perannya sebagai **imbalan akhir model jauh**, bukan pada urutan yang lebih baik.

---

## 1. Metode

Empat urutan pengecekan, semuanya memakai kandidat `env._calon` (rumus RS), *mask* yang sama, dan cek tabrakan `env._cek_kurva` yang sama:

| Metode | Urutan | Berhenti |
|---|---|---|
| `q` | Q model dekat menurun (sistem sekarang) | kata pertama yang bebas |
| `biaya` (analitik) | biaya rumus RS menaik, semua kata sah | kata pertama yang bebas |
| `demo` | aturan Hybrid A\* + RS (`_best_free_rs`): ≤ 6 termurah | berhenti dini bila kandidat berikutnya tidak lebih murah |
| `acak` | permutasi acak, rata-rata 5 kali | kata pertama yang bebas |
| `oracle` | cek **semua** kata sah | acuan biaya termurah yang bebas |

Sumber pose:
- **serah**: setiap percobaan pendaratan saat `rencanakan_dua` (metode gabungan, batas 2 s) merencanakan rute uji penuh `demo_jauh_id0N.npz` (379 / 358 rute) → 2.192 / 2.757 pose;
- **dekat**: pose awal episode uji `demo_dekat_tambahan_id0N.npz` (rute pendek di dalam 60 cm) → 449 / 450 pose.

## 2. Hasil: pose serah yang punya kata bebas (378 / 357 pose)

| Metode | Tiba [%] | Cek tabrakan rata-rata | ms per percobaan | Rasio biaya median / p90 | Pilih kata termurah | Cek pertama langsung bebas |
|---|---|---|---|---|---|---|
| **Q** | 100 / 100 | **1,54 / 1,36** | 0,150 / 0,134 | 1,00 / 1,00 · 1,00 / 1,00 | 91,8 / 91,6% | 79,1 / 86,0% |
| Analitik | 100 / 100 | 1,56 / 1,40 | **0,082 / 0,077** | 1,00 / 1,00 · 1,00 / 1,00 | 96,8 / 98,3% | 78,3 / 85,2% |
| Demonstrator | **98,7 / 98,6** | 1,77 / 1,53 | 0,094 / 0,085 | 1,00 / 1,00 · 1,00 / 1,00 | 96,8 / 97,5% | 68,8 / 77,3% |
| Acak | 100 / 100 | 2,99 / 2,79 | 0,145 / 0,141 | 1,14 / 2,28 · 1,22 / 2,71 | 45,8 / 40,9% | 18,8 / 21,6% |

Format sel: ID 01 / ID 02. Waktu tidak termasuk perhitungan rumus RS untuk *mask*, yang sama untuk semua metode.

**Pose serah tanpa kata bebas** (83% / 87% dari semua pose serah): Q, analitik, dan acak mengecek semua kata sah (rata-rata **10,06 / 8,04** cek). Demonstrator berhenti setelah 6 (5,99 / 6,00).

## 3. Hasil: rute pendek (449 / 450 pose, semua punya kata bebas)

| Metode | Tiba [%] | Cek tabrakan rata-rata | Rasio biaya p90 | Pilih kata termurah |
|---|---|---|---|---|
| **Q** | 100 / 100 | **1,49 / 1,54** | 1,00 / 1,00 | 86,2 / 87,1% |
| Analitik | 100 / 100 | 1,54 / 1,68 | 1,00 / 1,00 | 96,4 / 97,3% |
| Demonstrator | 100 / 100 | 1,84 / 1,94 | 1,00 / 1,00 | 98,2 / 98,2% |
| Acak | 100 / 100 | 2,58 / 2,39 | 2,15 / 1,99 | 31,0 / 30,4% |

Polanya sama dengan pose serah. Di rute pendek, Q sedikit lebih hemat cek daripada analitik (3–8%), tetapi selisihnya kecil.

## 4. Kapan demonstrator gagal dan model dekat berhasil

Kegagalan demonstrator dikelompokkan menurut **selisih arah hadap pose serah terhadap arah hadap tujuan**:

| Selisih arah hadap | ID 01: n / demo gagal | ID 02: n / demo gagal | Q gagal |
|---|---|---|---|
| 0–45° | 79 / 0% | 87 / 0% | 0% |
| 45–90° | 64 / 0% | 81 / 0% | 0% |
| 90–135° | 71 / 0% | 55 / 3,6% | 0% |
| **135–180°** | 55 / **7,3%** | 38 / **2,6%** | 0% |

Median pada pose tempat demonstrator gagal vs pose tempat demonstrator tiba:

| | ID 01: gagal / tiba | ID 02: gagal / tiba |
|---|---|---|
| Selisih arah hadap | **174°** / 78° | **117°** / 67° |
| Sudut ke titik tujuan | 158° / 44° | 147° / 37° |
| Kata bebas / kata sah | **2 / 14** / 3 / 10 | **2 / 15** / 4 / 10 |

Sepuluh pose itu:

| Robot | Peta | Pose (x, y) cm | Selisih hadap | Sudut ke tujuan | Jarak | Bebas / sah |
|---|---|---|---|---|---|---|
| ID 01 | Rumah | (121,3; 96,3) | 173° | 175° | 15,0 cm | 2 / 20 |
| ID 01 | Kalibrasi | (63,3; 160,3) | – (tujuan tanpa arah) | 136° | 55,9 cm | 3 / 14 |
| ID 01 | Rumah | (184,5; 76,1) | 175° | 104° | 2,3 cm | 2 / 8 |
| ID 01 | Rumah | (144,1; 127,3) | 176° | 172° | 55,5 cm | 2 / 16 |
| ID 01 | Rumah | (176,8; 95,6) | 173° | 158° | 36,3 cm | 2 / 8 |
| ID 02 | Kalibrasi | (72,0; 198,9) | 117° | 147° | 48,8 cm | 10 / 18 |
| ID 02 | Rumah | (130,5; 173,9) | – (tujuan tanpa arah) | 157° | 47,5 cm | 2 / 14 |
| ID 02 | Rumah | (95,6; 92,3) | 147° | 53° | 7,4 cm | 2 / 19 |
| ID 02 | Rumah2 | (111,0; 168,4) | – (tujuan tanpa arah) | 164° | 53,4 cm | 2 / 15 |
| ID 02 | sintetis | (255,7; 49,2) | 105° | 91° | 45,1 cm | 1 / 10 |

- Saat robot hampir berlawanan arah dengan arah hadap tujuan, kurva RS termurah biasanya manuver putar yang rapat dan menabrak. Kurva yang bebas lebih mahal dan berada di luar enam kandidat pertama.
- Urutan analitik yang mencoba semua kata juga mendarat di kesepuluh pose ini. Jadi keunggulan model dekat atas demonstrator berasal dari **jumlah kata yang dicoba**, bukan dari urutan hasil belajar.
- Kejadiannya jarang (10 dari 735 pose, 1,4%). Di rute penuh, Hybrid A\* yang gagal di pose seperti ini tidak langsung gagal total; pencariannya berlanjut ke simpul lain dengan tambahan waktu.

## 5. Kesimpulan

1. **Urutan Q ≈ urutan biaya analitik.** Jaringan berhasil mempelajari urutan biaya rumus RS (jauh lebih baik dari acak), tetapi tidak melampauinya, dan sedikit lebih lambat.
2. **Keunggulan atas Hybrid A\* + RS berasal dari mencoba semua kata sah**, dan hanya tampak saat arah hadap pose serah berbeda > 90° dari arah hadap tujuan.
3. **Ongkos pendaratan ditentukan model jauh.** 83–87% pose serah tidak punya kata bebas sama sekali, dan di pose itu semua metode (kecuali demonstrator) harus mengecek semua kata.
4. Model dekat tidak melihat rintangan. Urutan yang lebih baik daripada aturan analitik kemungkinan membutuhkan masukan rintangan. Sebagai catatan, model dekat sadar rintangan (v7) pernah diuji dan ditolak karena tidak lebih cepat (pesan *commit* `ad99f6a`, 2026-10-23); rinciannya tidak lagi ada di [[07 Struktur DQfD Full|Struktur DQfD Full]].

> [!warning] Keterbatasan
> - Pose serah dikumpulkan dari rute uji **offline** dengan batas waktu 2 s, bukan dari uji robot nyata.
> - Waktu per percobaan diukur pada laptop pengembangan; yang dibandingkan adalah selisih antar-metode di mesin yang sama.

## 6. Perintah dan sumber angka

```
python -m drl.ablasi_dekat --id 1,2         # bagian 2-3, hasil: model_dqn/ablasi_dekat.json
python -m drl.ablasi_dekat_arah --id 1,2    # bagian 4,   hasil: model_dqn/ablasi_dekat_arah.json
```

| Angka | Sumber |
|---|---|
| Bagian 2–3 (Tabel V paper) | `Forklift Robot Program/model_dqn/ablasi_dekat.json` |
| Bagian 4 (paragraf VI-B paper) | `Forklift Robot Program/model_dqn/ablasi_dekat_arah.json`, termasuk daftar `pose_demo_gagal` |
| Kode | `drl/ablasi_dekat.py` (`nilai_pose`, `pose_serah`, `demo_hitung`), `drl/ablasi_dekat_arah.py` |
