# Penelitian terkait dan novelty DQfD dua model

Tanggal: 2026-10-25 · Branch `ver-1-4` · Terkait: [[Struktur DQfD Full]], [[Perbandingan Hybrid A-star RS vs DQfD]], [[Perbandingan Uji Robot Nyata Gudang1 vs Gudang2]]

Pertanyaan yang dijawab: apakah sudah ada riset yang membuat model DQfD dari Hybrid A\* (model jauh) dan model DQfD dari RS (model dekat)? Apa novelty kita dibanding riset yang ada?

> [!summary] Ringkasan
> - **Belum ditemukan** riset yang memakai pembagian dua model DQfD dengan dua guru: Hybrid A\* untuk model jauh dan pilihan kata RS untuk model dekat.
> - **Pembanding terdekat untuk model jauh: Bernhard dkk. 2018.** Mereka melatih Double DQN dengan **demonstrasi Hybrid A\*** dan primitif yang sama, lalu memakai Q sebagai heuristik di dalam Hybrid A\*. Ini mirip dengan pencarian berpandu Q kita, jadi klaim kontribusi (iii) di paper harus mengutip dan membedakannya.
> - **Pembanding terdekat untuk model dekat: HOPE, RL-OGM-Parking, REAP.** Di sana RS dipakai sebagai aturan yang dipanggil, atau perencana aturan menjadi guru BC. Belum ditemukan yang menjadikan **kata RS sebagai ruang aksi yang dipelajari**.
> - **Forklift + RL**: yang ditemukan hanya di simulasi (HMER, Gazebo).

> [!warning] Batas penelusuran
> Penelusuran: sekitar 10 kueri web dan halaman abstrak arXiv. Teks lengkap hanya dibaca untuk Bernhard 2018 (versi HTML arXiv). Scopus dan IEEE Xplore **belum** ditelusuri. Di paper, tulis "sepanjang penelusuran penulis", jangan "pertama di dunia".

---

## 1. Riset terdekat

| Riset | Yang dilakukan | Beda dengan kita |
|---|---|---|
| **Bernhard dkk., ITSC 2018** — *Experience-Based Heuristic Search: Robust Motion Planning with Deep Q-Learning* | Double DQN + *learning from demonstration* dengan **demonstrasi Hybrid A\***. Primitif sama dengan Hybrid A\* (kemudi diskret, maju/mundur). Q dipasang sebagai **heuristik di dalam Hybrid A\*** untuk parkir valet semi-terstruktur. | State 16 dimensi dengan *one-hot* tempat parkir, jadi terikat tata letak latihan. Tidak ada model dekat terlatih. Hanya simulasi. Kita: 32 sinar + Dijkstra, berhenti di pendaratan model dekat, robot nyata. |
| **Kim & Huh 2021** — *Neural Motion Planning for Autonomous Parking* (arXiv 2111.06739) | CVAE dari lintasan demonstrasi menuntun ekspansi Hybrid A\*. | Model generatif, bukan DQfD. Hybrid A\* tetap berjalan. |
| **HOPE** (Jiang dkk., IEEE T-ITS 2025) | RL + *action mask* + transformer. Beralih ke kurva RS begitu ada yang layak. | RS hanya aturan penyelesai. Tidak ada model yang belajar memilih kata RS. Tidak memakai demonstrasi Hybrid A\*. |
| **RL-OGM-Parking** (Wang dkk., arXiv 2502.18846) | Skema HOPE + peta okupansi lidar, dijalankan pada kendaraan nyata. | Sama dengan HOPE. |
| **REAP** (Li dkk., arXiv 2605.08713) | Perencana berbasis aturan disuling ke jaringan *end-to-end* lewat BC, lalu SAC. | BC, bukan DQfD. Keluarannya perintah gerak, bukan kata RS. Mobil. |
| **MultiPark** (Zheng dkk., arXiv 2508.11537) | Transformer *imitation learning*, prediksi segmen berikutnya. | Bukan DQfD, bukan dua model dua guru. |
| **Tao dkk.** (arXiv 2601.22545) | DRL untuk parkir sempit dengan banyak pergantian arah. | Satu kebijakan RL, tanpa demonstrator. |
| **HMER** (Chen dkk., arXiv 2601.07304) | Forklift: sub-kebijakan navigasi + manipulasi, inisialisasi demonstrasi lalu RL. Galat penempatan < 1,5 cm. | **Hanya Gazebo.** |
| **Hester dkk., AAAI 2018** — DQfD | Metode dasar. Demonstrasi dari manusia (Atari). | Kita: demonstrator perencana klasik. |
| Klasifikasi tipe lintasan RS (ResearchGate, penulis dan tahun belum terkonfirmasi) | Jaringan saraf menebak tipe RS optimal (top-1 97,5%). | Hanya geometri, tanpa rintangan dan tanpa RL. **Cek sumber asli sebelum dikutip.** |

## 2. Daftar novelty

**Klaim metode (paling kuat):**
1. **Kata RS sebagai ruang aksi DQfD.** Model dekat memilih satu dari 88 kata RS utuh, lalu panjang segmennya dihitung rumus. Jaringan hanya mengurutkan kandidat, jadi galat akhir rencana selalu nol. Belum ditemukan yang melakukan ini.
2. **Dua model DQfD, dua guru, satu aturan serah.** Model jauh meniru Hybrid A\*, model dekat meniru pilihan kata RS. Aturan serah yang sama dipakai saat membuat *dataset*, latihan mandiri, dan pengujian. Pada HOPE perpindahannya ke RS klasik, bukan ke model kedua yang dilatih.
3. **Imbalan akhir model jauh = biaya pendaratan sungguhan model dekat.** Model jauh belajar berhenti di pose tempat model dekat benar-benar bisa mendarat.
4. **Model jauh membaca rintangan (32 sinar + Dijkstra), bukan pengodean tempat parkir.** Ini yang membedakannya dari Bernhard 2018, sehingga model bisa dipakai di peta lain (tiga peta di paper).

**Klaim yang harus ditulis hati-hati:**
5. **Pencarian berpandu Q** (kontribusi iii di paper). Gagasan memakai Q sebagai heuristik sudah ada di Bernhard 2018. Yang baru adalah kombinasinya dengan model dekat dan cadangan pencarian mundur.

**Klaim eksperimental dan sistem:**
6. **Rincian waktu perencanaan per bagian** (Hybrid A\*, RS, DQfD jauh, DQfD dekat) pada peta dan skenario yang sama, offline dan di robot. Kebanyakan makalah parkir RL hanya melaporkan tingkat keberhasilan.
7. **Uji dua robot forklift nyata** (AprilTag, *pure pursuit* 20 Hz, stepper + servo kemudi belakang) di Gudang dan Gudang2, termasuk analisis kapan masing-masing perencana lebih cepat dan mengapa (sudut hadap awal terhadap tujuan). Riset RL forklift yang ditemukan hanya di simulasi.
8. **Integrasi multi-robot dengan wasit prioritas** di atas perencana DQfD. Ini kontribusi sistem, bukan metode, karena literatur MAPF gudang sudah luas.

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

## 4. Paragraf Penelitian Terkait (salinan dari paper)

**Belajar dari demonstrasi.** DQfD menggabungkan *loss* TD 1 langkah dan *n* langkah dengan *loss* *large-margin* tersupervisi pada transisi demonstrasi, yang disimpan permanen di *buffer replay*. Pada makalah aslinya, demonstrasi berasal dari pemain manusia. Kami memakai DQfD untuk kedua model dengan perencana klasik sebagai demonstrator: Hybrid A\* untuk model jauh dan pilihan kata RS untuk model dekat.

**Pembelajaran untuk menuntun Hybrid A\*.** Pekerjaan yang paling dekat dengan model jauh kami adalah Bernhard dkk. Mereka melatih Double DQN dengan demonstrasi Hybrid A\*, memakai primitif kemudi yang sama dengan Hybrid A\*, lalu memasang fungsi Q hasil latihan sebagai pengganti heuristik di dalam Hybrid A\* untuk parkir valet semi-terstruktur. Kim dan Huh menuntun ekspansi Hybrid A\* dengan CVAE yang dilatih dari lintasan demonstrasi. Pencarian berpandu Q kami berangkat dari gagasan yang sama dengan Bernhard dkk., dengan tiga perbedaan. Pertama, keadaan pada pekerjaan mereka berupa vektor 16 dimensi yang memuat pengodean *one-hot* tempat parkir, sedangkan model jauh kami membaca rintangan lewat 32 sinar dan peta Dijkstra sehingga bisa dipakai di peta lain. Kedua, pencarian kami berhenti di pose tempat model dekat yang juga dilatih benar-benar mendarat, dan biaya pendaratan itu menjadi imbalan akhir model jauh. Ketiga, pengujian mereka hanya di simulasi, sedangkan kami menguji pada robot nyata.

**RL dan kurva RS untuk parkir.** HOPE menggabungkan agen RL ber-*action mask* dengan kurva RS: selama berjalan, perencana mencoba kurva RS dan beralih ke kurva itu begitu ada yang layak. RL-OGM-Parking memakai skema serupa dengan masukan peta okupansi lidar agar dapat dijalankan pada kendaraan nyata. REAP menyuling perencana berbasis aturan ke jaringan *end-to-end* lewat *behavior cloning* sebelum tahap RL, MultiPark meniru demonstrasi parkir segmen demi segmen dengan transformer, dan Tao dkk. melatih DRL untuk parkir sempit yang membutuhkan banyak pergantian arah. Pada pekerjaan-pekerjaan ini, RS dipanggil sebagai aturan geometris, atau perencana klasik menjadi guru bagi satu kebijakan yang mengeluarkan perintah gerak. Sepanjang penelusuran kami, belum ada yang menjadikan kata RS sebagai ruang aksi yang dipelajari. Model dekat kami memperlakukan 88 kata RS sebagai aksi DQfD dan belajar dari pilihan demonstrator. Jaringan hanya mengurutkan kandidat, sedangkan geometrinya tetap dihitung rumus, sehingga jalur selalu berakhir tepat di tujuan.

**Forklift otonom.** Chen dkk. membagi tugas forklift jangka panjang menjadi sub-kebijakan navigasi dan manipulasi yang diinisialisasi dari demonstrasi lalu disetel dengan RL. Galat penempatannya di bawah 1,5 cm, tetapi seluruh pengujiannya di Gazebo. Kami menguji dua robot forklift nyata di dua peta gudang, dengan lokalisasi AprilTag dan pelacak *pure pursuit*, dan membandingkannya dengan Hybrid A\* + RS pada skenario yang sama.

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
