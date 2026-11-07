# Penelitian terkait dan novelty DQfD

Tanggal: 2026-10-25 · Branch `ver-1-4` · Terkait: [[07 Struktur DQfD Full|Struktur DQfD Full]]

> [!warning] Dibersihkan 2026-11-07 (aturan ver-1-6)
> Klaim novelty yang bergantung pada model dekat (kata RS sebagai ruang aksi, dua model dua guru, imbalan akhir dari pendaratan model dekat) dan salinan paragraf paper lama dihapus. Di ver-1-6 tidak ada model dekat: DQfD = model jauh + tembakan RS di dalam 60 cm.

> [!summary] Ringkasan
> - **Pembanding terdekat untuk model jauh: Bernhard dkk. 2018.** Mereka melatih Double DQN dengan **demonstrasi Hybrid A\*** dan primitif yang sama, lalu memakai Q sebagai heuristik di dalam Hybrid A\*. Ini mirip dengan pencarian berpandu Q kita, jadi klaim kontribusi (iii) di paper harus mengutip dan membedakannya.
> - **RL dan kurva RS untuk parkir: HOPE, RL-OGM-Parking, REAP.** Di sana RS dipakai sebagai aturan yang dipanggil, atau perencana aturan menjadi guru BC.
> - **Forklift + RL**: yang ditemukan hanya di simulasi (HMER, Gazebo).

> [!warning] Batas penelusuran
> Penelusuran: sekitar 10 kueri web dan halaman abstrak arXiv. Teks lengkap hanya dibaca untuk Bernhard 2018 (versi HTML arXiv). Scopus dan IEEE Xplore **belum** ditelusuri. Di paper, tulis "sepanjang penelusuran penulis", jangan "pertama di dunia".

---

## 1. Riset terdekat

| Riset | Yang dilakukan | Beda dengan kita |
|---|---|---|
| **Bernhard dkk., ITSC 2018** — *Experience-Based Heuristic Search: Robust Motion Planning with Deep Q-Learning* | Double DQN + *learning from demonstration* dengan **demonstrasi Hybrid A\***. Primitif sama dengan Hybrid A\* (kemudi diskret, maju/mundur). Q dipasang sebagai **heuristik di dalam Hybrid A\*** untuk parkir valet semi-terstruktur. | State 16 dimensi dengan *one-hot* tempat parkir, jadi terikat tata letak latihan. Hanya simulasi. Kita: 32 sinar + Dijkstra, robot nyata. |
| **Kim & Huh 2021** — *Neural Motion Planning for Autonomous Parking* (arXiv 2111.06739) | CVAE dari lintasan demonstrasi menuntun ekspansi Hybrid A\*. | Model generatif, bukan DQfD. Hybrid A\* tetap berjalan. |
| **HOPE** (Jiang dkk., IEEE T-ITS 2025) | RL + *action mask* + transformer. Beralih ke kurva RS begitu ada yang layak. | RS hanya aturan penyelesai. Tidak memakai demonstrasi Hybrid A\*. |
| **RL-OGM-Parking** (Wang dkk., arXiv 2502.18846) | Skema HOPE + peta okupansi lidar, dijalankan pada kendaraan nyata. | Sama dengan HOPE. |
| **REAP** (Li dkk., arXiv 2605.08713) | Perencana berbasis aturan disuling ke jaringan *end-to-end* lewat BC, lalu SAC. | BC, bukan DQfD. Keluarannya perintah gerak. Mobil. |
| **MultiPark** (Zheng dkk., arXiv 2508.11537) | Transformer *imitation learning*, prediksi segmen berikutnya. | Bukan DQfD. |
| **Tao dkk.** (arXiv 2601.22545) | DRL untuk parkir sempit dengan banyak pergantian arah. | Satu kebijakan RL, tanpa demonstrator. |
| **HMER** (Chen dkk., arXiv 2601.07304) | Forklift: sub-kebijakan navigasi + manipulasi, inisialisasi demonstrasi lalu RL. Galat penempatan < 1,5 cm. | **Hanya Gazebo.** |
| **Hester dkk., AAAI 2018** — DQfD | Metode dasar. Demonstrasi dari manusia (Atari). | Kita: demonstrator perencana klasik. |
| Klasifikasi tipe lintasan RS (ResearchGate, penulis dan tahun belum terkonfirmasi) | Jaringan saraf menebak tipe RS optimal (top-1 97,5%). | Hanya geometri, tanpa rintangan dan tanpa RL. **Cek sumber asli sebelum dikutip.** |

## 2. Daftar novelty

**Klaim metode:**
1. **Model jauh membaca rintangan (32 sinar + Dijkstra), bukan pengodean tempat parkir.** Ini yang membedakannya dari Bernhard 2018, sehingga model bisa dipakai di peta lain.

**Klaim yang harus ditulis hati-hati:**
2. **Pencarian berpandu Q** (kontribusi iii di paper). Gagasan memakai Q sebagai heuristik sudah ada di Bernhard 2018. Yang baru adalah kombinasinya dengan cadangan pencarian mundur.

**Klaim eksperimental dan sistem:**
3. **Rincian waktu perencanaan per bagian** (Hybrid A\*, RS, DQfD) pada peta dan skenario yang sama, offline dan di robot. Kebanyakan makalah parkir RL hanya melaporkan tingkat keberhasilan.
4. **Uji dua robot forklift nyata** (AprilTag, *pure pursuit* 20 Hz, stepper + servo kemudi belakang) di Gudang dan Gudang2. Riset RL forklift yang ditemukan hanya di simulasi.
5. **Integrasi multi-robot dengan wasit prioritas** di atas perencana DQfD. Ini kontribusi sistem, bukan metode, karena literatur MAPF gudang sudah luas.

## 3. Yang sudah dimasukkan ke paper

- `src/paper.tex`, bagian **Penelitian Terkait**: paragraf "Belajar dari demonstrasi" diperluas, lalu ditambah tiga paragraf baru: "Pembelajaran untuk menuntun \astar{}", "RL dan kurva RS untuk parkir", dan "Forklift otonom". Ditambah `\label{sec:search}` di subbagian Pencarian Saat Pengujian.
- `src/references.bib`: 8 entri baru (`bernhard2018experience`, `kim2021neural`, `jiang2025hope`, `wang2025rlogm`, `li2026reap`, `zheng2025multipark`, `tao2026adapting`, `chen2026hmer`).
- Cadangan sebelum diubah: `paper.tex.bak-20261025`, `references.bib.bak-20261025`.

> [!todo] Belum dikerjakan
> - Kontribusi (iii) di Pendahuluan belum menyebut Bernhard 2018. Sebaiknya ditambah "berangkat dari~\cite{bernhard2018experience}".
> - Telusuri Scopus/IEEE Xplore:
>   - `"demonstration" AND ("Hybrid A*" OR "hybrid A-star") AND ("Q-learning" OR "DQfD")`
>   - `"Reeds-Shepp" AND ("imitation" OR "demonstration") AND "reinforcement learning"`
>   - `forklift AND "reinforcement learning" AND ("real robot" OR "real-world")`
> - Halaman dan volume untuk Bernhard 2018 dan HOPE (T-ITS) belum diisi di `.bib`.

## Sumber

- [Bernhard dkk. 2018, arXiv 2102.03127](https://arxiv.org/abs/2102.03127) (DOI 10.1109/ITSC.2018.8569436)
- [Kim & Huh 2021, arXiv 2111.06739](https://arxiv.org/abs/2111.06739)
- [HOPE, arXiv 2405.20579](https://arxiv.org/abs/2405.20579) (DOI 10.1109/TITS.2025.3550417)
- [RL-OGM-Parking, arXiv 2502.18846](https://arxiv.org/abs/2502.18846)
- [REAP, arXiv 2605.08713](https://arxiv.org/abs/2605.08713)
- [MultiPark, arXiv 2508.11537](https://arxiv.org/abs/2508.11537)
- [Tao dkk., arXiv 2601.22545](https://arxiv.org/abs/2601.22545)
- [HMER, arXiv 2601.07304](https://arxiv.org/abs/2601.07304)
- [Hester dkk., DQfD, arXiv 1704.03732](https://arxiv.org/abs/1704.03732)
- [Klasifikasi tipe RS (ResearchGate)](https://www.researchgate.net/publication/243769084_Shortest_paths_for_the_reeds-shepp_car_a_worked_out_example_of_the_use_of_geometric_techniques_in_nonlinear_optimal_control)
