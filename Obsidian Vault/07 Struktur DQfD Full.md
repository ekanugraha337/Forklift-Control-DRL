Struktur **model jauh DQfD** per robot, versi aksi 6, branch `ver-1-4`, 2026-10-23. Konsep dasar DQfD ada di [[06 Deep Reinforcement Learning|Deep Reinforcement Learning]].

> [!warning] Dibersihkan 2026-11-07 (aturan ver-1-6)
> Semua bagian model dekat, hasil model lama, dan keputusan terbuka yang sudah usang dihapus. Aturan yang berlaku sekarang (branch `ver-1-6`):
> - **Hybrid A\* + RS (`astar`):** w = 2,0, h = max(h_RS, h_D), tembakan RS dicoba setiap ekspansi ke-4.
> - **DQfD:** hanya model jauh, dilatih dari demo Hybrid A\* dengan w = 2,0 dan h = max(h_D, h_RS), dan pencarian DQfD juga w = 2,0. Masukan heuristik model jauh juga h = max(h_D, h_RS). Tembakan RS dicoba bila < 60 cm dari tujuan dan tidak ada halangan. Tidak ada model dekat.
> - **Pelatihan:** gradien dengan PyTorch, forward dengan NumPy, 32 sinar dengan Numba.

> [!note] Inti
> - **Hybrid A\* + Reeds-Shepp adalah demonstrator saat latihan.**
> - **Model jauh meniru Hybrid A\*:** 47 masukan (tujuan + peta rintangan) → Q untuk 14 primitif gerak. Saat dipakai tidak ada pencarian A\*.

---

## 1. Model jauh

### 1.1 Peran

| | **MODEL JAUH** (`id0N_jauh.pt`) |
|---|---|
| Meniru | Hybrid A\* |
| Bekerja | posisi awal → titik serah |
| Tugas | navigasi memutari rintangan |
| Melihat rintangan? | ya: 32 sinar + heuristik h = max(h_D, h_RS) |
| Masukan | **47** angka |
| Keluaran | **14** primitif |
| Satu episode | banyak langkah, sampai serah |
| Parameter | 145.807 |
| Data latih | `demo_jauh_id0N.npz` |

### 1.2 Keluaran

| | Model JAUH (14) |
|---|---|
| Bentuk | primitif: 7 tingkat kemudi × maju/mundur |
| Panjang | 7,3 cm tetap |
| Kemudi | −41,3 … +41,3° (7 tingkat) |
| Indeks | 0–6 maju, 7–13 mundur |

## 2. Posisi di dalam sistem

- DQfD hanya menggantikan perencana jalur. Pelacak (pure pursuit), wasit, dan `robot.py` tidak berubah.
- ID 01 dan ID 02 memakai susunan yang sama, dengan bobot terpisah yang dilatih dari demo masing-masing.

## 3. Aturan berhenti model jauh

Model jauh berhenti, lalu tembakan Reeds-Shepp menutup jalur ke tujuan, bila **kedua** syarat terpenuhi:

1. **Titik acuan di dalam 60 cm** dari tujuan (`analytic_dist`), jarak yang sama dengan saat demonstrator mulai mencoba Reeds-Shepp.
2. **Tidak ada halangan:** peta Dijkstra tidak memaksa memutar (biaya Dijkstra = jarak oktil grid) atau sudah < 25 cm (`RADIUS_SERAH_PAKSA_CM`), **dan** tembakan RS dari pose itu bebas tabrakan (syarat yang sama dengan saat Hybrid A\* beralih ke RS).

Bila belum terpenuhi, model jauh **berjalan lanjut** satu primitif lalu mencoba lagi.

Di dataset, model jauh berhenti **tepat** di titik demonstrator beralih ke Reeds-Shepp. Jadi demo jauh = primitif Hybrid A\*.

## 4. Demo dan uji penerimaan

```
jalur demo = [awal] + (4 pose per primitif A*) × N + (pose ekor Reeds-Shepp)
aksi jauh  = primitif A*, diurai PERSIS (1 kelompok 4 pose = 1 primitif)
```

Tujuan tanpa arah hadap disapu seperti `Perencana` (searah garis awal–tujuan dulu, lalu ±30°, ±60°, … ±180°); arah pertama yang menghasilkan jalur menjadi arah tujuan demo. Jadi demonstrator selalu memakai h = max(h_D, h_RS).

Demo yang tidak bisa dipakai dibuang dengan kode alasan (`adaptor_demo.TOLAK`):

| Kode | Arti |
|---|---|
| `belum_bebas` | RS demonstrator mulai saat peta Dijkstra masih menunjukkan halangan |
| `tanpa_rs` | Hybrid A\* tiba tanpa ekor RS |
| `jauh_margin` | bagian A\* menyentuh margin (peta dengan robot lain sebagai rintangan) |

## 5. MDP (`lingkungan.py`)

| | Model JAUH |
|---|---|
| Mulai dari | pose awal, diam |
| Satu aksi | primitif 7,3 cm |
| Selesai sukses | berhenti (bagian 3) |
| Selesai gagal | buntu (semua aksi ter-mask) |
| Reset ditolak bila | sudah boleh berhenti di pose awal |
| Imbalan | −biaya per primitif |

**Mask model jauh** (Q = −10⁹):
1. Badan menyentuh zona di salah satu sub-titik (aturan Hybrid A\*, termasuk kelonggaran "lepas diri" bila awalnya menempel).
2. Primitif yang mendarat di sel yang sudah dikunjungi **dengan arah yang sama** (sel `_key` Hybrid A\*: 4,9 cm, 10°).

**Biaya** memakai rumus Hybrid A\*, sehingga satuannya sama dengan g-cost A\*:

```
primitif (panjang p):  p · (1 atau reverse_cost 2,0) + steer_cost 0,3 · p · |κ|/κmax
                       + steer_change_cost 6,0 · |κ − κ_sebelum|/κmax
                       + switch_cost 32,4 · [ganti arah] + clearance_penalty
tembakan RS:           _rs_cost(segmen, d, k) (rumus yang sama, per segmen)
                       + clearance_penalty di tiap sampel 2,4 cm
```

| Akhir | Syarat | Catatan |
|---|---|---|
| Serah | bagian 3 | sukses |
| Buntu | semua aksi ter-mask | imbalan −5000 |
| Terpotong | 300 langkah | bukan keadaan terminal untuk DQN |

## 6. Masukan model jauh: 47 angka

Kerangka robot (x maju, y samping).

| Indeks | Jml | Isi | Skala |
|---|---|---|---|
| 0–1 | 2 | Posisi tujuan relatif: maju, samping | ÷ 300 cm |
| 2 | 1 | Jarak lurus ke tujuan | ÷ 300 cm |
| 3–4 | 2 | Selisih arah hadap tujuan (cos, sin) | (1, 0) bila arah bebas |
| 5 | 1 | Penanda arah tujuan bebas | 0/1 |
| 6–7 | 2 | Pendekatan lurus 32,4 cm muat dari depan / dari belakang | 0/1 |
| 8 | 1 | Arah gerak terakhir | +1 / −1 / 0 |
| 9 | 1 | Kelengkungan kemudi terakhir | κ/κmax |
| 10–41 | 32 | **Sinar**: jarak tepi badan ke zona, setiap 11,25°, dari pusat badan | 0 = menempel, 1 = ≥ 100 cm |
| 42 | 1 | **Heuristik h = max(h_D, h_RS)**: Dijkstra 2D memutari rintangan dan biaya Reeds-Shepp tanpa rintangan, sama dengan heuristik demonstrator | ÷ 300 cm |
| 43–44 | 2 | Arah turun h = max(h_D, h_RS) ke tetangga 8 grid heuristik (cos, sin) | kerangka robot |
| 45 | 1 | Margin keselamatan yang dipakai | 8 cm = 1,0 |
| 46 | 1 | Jarak bebas badan sekarang | ÷ 20 cm, dipotong −1…1 |

## 7. Susunan neuron: Dueling DQN

| Lapis | Model JAUH | Parameter |
|---|---|---|
| FC1 · ReLU | 47 → 256 | 12.288 |
| FC2 · ReLU | 256 → 256 | 65.792 |
| V-hidden · ReLU | 256 → 128 | 32.896 |
| V(s) | 128 → 1 | 129 |
| A-hidden · ReLU | 256 → 128 | 32.896 |
| A(s,a) | 128 → 14 | 1.806 |
| **Total** | | **145.807** |

- **Q(s,a) = V(s) + A(s,a) − rata-rata(A)** = −(biaya sisa) / 100 (`SKALA_REWARD`).
- Q model jauh mencakup biaya sampai tujuan.

### Diagram neuron

Lingkaran = neuron, ⋮ = neuron yang tidak digambar, garis abu-abu = bobot yang dilatih. Garis hijau dari A ke Q dan garis putus-putus dari V ke Q **tidak punya bobot**: itu rumus Q = V + A − rata(A).

**Model jauh**: 47 masukan dikelompokkan per indeks, 14 keluaran primitif.

![[Gambar/DQfD neuron model jauh.svg]]

## 8. Algoritma belajar (`dqfd.py`, `latih.py`)

### 8.1 Loss

```
J = J_DQ + λn · J_n + λe · J_E + L2
```

| Komponen | Isi | Nilai |
|---|---|---|
| J_DQ | TD 1 langkah, Double DQN (argmax jaringan online, nilai jaringan target) | γ = 0,995 |
| J_n | TD n langkah | n = 10, λn = 1,0 (fase A), 0,5 (fase B) |
| J_E | Large-margin, **hanya sampel demo**: max_a[Q(s,a) + l(a_E,a)] − Q(s,a_E) | margin l = 0,8, λe = 1,0 |
| L2 | weight decay Adam | 1e-5 |

Semua max/argmax hanya atas aksi sah (mask). Buffer: prioritized replay proporsional (α = 0,4). Data demo di depan dan permanen, dengan bonus prioritas ε_demo = 1,0. Data agen melingkar di belakangnya (300.000 transisi).

### 8.2 Fase

| | Fase A: pra-latih | Fase B: latihan sendiri |
|---|---|---|
| Data | Hanya demo (episode bukan uji) | Demo + hasil rollout jaringan sendiri |
| Panjang (bawaan) | 20.000 langkah gradien | 150.000 langkah lingkungan, 0,5 langkah gradien per langkah lingkungan |
| Learning rate | 1e-4 | 5e-5 |
| Eksplorasi | – | ε 0,1 → 0,01 di antara aksi sah, 16 rollout paralel |
| Skenario | – | 70% dari demo latih, 30% acak baru; 50% dari peta tersulit |

Lain-lain: batch 256, sinkron jaringan target setiap 2.000 langkah, evaluasi setiap 5.000 langkah, GPU `--perangkat cuda` (NVIDIA GeForce RTX 5050 Laptop, CUDA 12.8). Peta tersulit = jenis peta asli dengan waktu Hybrid A\* rata-rata terbesar (Rumah). `--awal <folder>` memulai latihan dari bobot model berjenis sama.

### 8.3 Validasi dan pemilihan model

- Set validasi: setiap episode latih ke-10 (bukan set uji). 40 rute per jenis peta, 60 untuk peta tersulit.
- **Skor** = % tiba − 10 × (p90 rasio biaya − 1). Model terbaik disimpan sebagai `id0N_<jenis>.pt`, yang terakhir sebagai `id0N_<jenis>_akhir.pt`, dan setiap evaluasi sebagai `id0N_<jenis>/ckpt_<langkah>.pt`.

## 9. Dataset

| Berkas (`drl/data/`) | Isi | Dibuat oleh |
|---|---|---|
| `demo_jauh_id0N.npz` | Episode model jauh (primitif A\*) | `drl.skenario` |
| `negatif_id0N.npz` | Skenario yang gagal di Hybrid A\*: DQfD harus ikut buntu | `drl.skenario` |

| Pembangkitan | Nilai |
|---|---|
| Demo A\* per robot | 5.000 (`--n`) |
| Peta | Semua di `Camera/peta_constraint/` dibagi rata + sintetis 30% |
| Satu tugas | Satu peta, 20 skenario. Setiap tugas ke-10 masuk set uji |
| Variasi | margin cadangan 6 cm 20%, badan robot lain sebagai rintangan 30%, arah tujuan bebas 30%, jarak dekat 15–60 cm 15% (sisanya 40–300 cm) |

## 10. Saat pengujian (`perencana_dqfd.py`)

| Mode `--perencana` | Isi |
|---|---|
| `dqfd-murni` | model jauh + tembakan RS, tanpa Hybrid A\*. Gagal = tidak berangkat |
| `dqfd` | sama, dengan Hybrid A\* + Reeds-Shepp sebagai cadangan dan gerbang mutu 1,3 |
| `astar` | Hybrid A\* + Reeds-Shepp, tanpa DQfD |

**Kompatibilitas model.** Checkpoint mencatat `max_steer_deg`, `r_plan_cm`, `versi_aksi`, `jenis`, `n_aksi`, dan `n_obs`. Checkpoint yang tidak cocok dengan profil kemudi atau ruang aksi program (`lingkungan.VERSI_MODEL`) ditolak dengan pesan yang menyuruh latih ulang.

## 11. Perintah

Dari folder `Forklift Robot Program`:

```powershell
python -m drl.adaptor_demo --uji                           # demonstrator & pemotongan lulus di semua peta?

python -m drl.skenario --id 1,2 --n 5000                   # demo jauh + negatif
python -m drl.skenario --id 1,2 --periksa                  # integritas semua dataset

python -m drl.latih --id 1,2 --model jauh --perangkat cuda
python -m drl.latih --id 1,2 --model jauh --awal <folder> --pra 2000 --mandiri 50000 --perangkat cuda   # latih lanjut
python -m drl.evaluasi --id 1,2                               # nilai di set uji
python -m drl.uji_pose_akhir --id 1                           # pose akhir robot simulasi vs Hybrid A* + RS
python navigasi.py --simulasi --id 1 --perencana dqfd-murni
```
