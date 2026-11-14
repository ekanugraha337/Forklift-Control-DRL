# Prioritas perbaikan paper ver5 setelah review pembimbing

Tanggal: 2026-11-06 · Untuk: *Paper Eka N - Thesis ver5* · Sumber: review pembimbing (6 Nov 2026), cek kode branch `ver-1-5` (commit `91edda3`) · Bukti dan angka: [[01 Bukti Temuan dan Anggaran Halaman]]

> [!warning] Dibersihkan 2026-11-07 (aturan ver-1-6)
> Butir yang bergantung pada aturan lama (baseline Hybrid A\* lama, model dekat, data `model_dqn/` yang sudah dihapus, dan angka hasil lama) dihapus. Aturan ver-1-6: Hybrid A\* + RS standar (w = 2,0, h = max(h_RS, h_D), RS setiap ekspansi ke-4); DQfD = model jauh dari demo Hybrid A\* w = 2,0 + tembakan RS di dalam 60 cm; tanpa model dekat.

> [!summary] Ringkasan
> - Urutan mengikuti dua hal: dampak ke kelayakan paper dan ketergantungan antar-pekerjaan. Hasil tahap 1 menentukan isi tahap 2–4, jadi gaya penulisan dirapikan paling akhir.
> - Eksperimen baru: ablasi DQN/BC (butir 3). Sisanya penulisan dan keputusan.
> - Butir 2 diputuskan bersama pembimbing.
> - Anggaran halaman: ver5 tujuh halaman, lebih ±20 baris dari 6 halaman. Acuan baru menambah ±25–30 baris. Penghemat terbesar adalah keputusan cakupan (butir 2), bukan gaya penulisan.

Label: **[E]** eksperimen baru · **[D]** dari data yang sudah ada · **[T]** penulisan · **[K]** keputusan. "Review x.y" merujuk ke bagian di PDF review.

---

## Tahap 1 — Menentukan isi paper

- [ ] **2. Cakupan dan judul** [K] · review 2.1
	- (a) Fokus planner: crossing schedule dipadatkan, hemat ±1 kolom. Poin review 4.6 cukup ditulis sebagai keterbatasan.
	- (b) Planner + koordinasi: judul diubah. Perlu pembanding koordinasi lain (prioritized planning atau ADG, cukup di simulasi), beberapa skenario silang, dan aturan tahan yang memperhitungkan pengereman.
- [ ] **3. Ablasi DQfD vs DQN dari nol vs BC** [E] · review 4.2
	- Cukup model far, minimal ID 01. DQN dari nol diperkirakan ±3–4 jam GPU per robot, setara model far dasar. BC murah.
	- Kodenya belum ada: demonstrasi selalu dimuat dan loss large-margin selalu aktif.
	- Kalau waktunya tidak cukup, hapus klaim efisiensi sampel dari pendahuluan.

## Tahap 2 — Klaim dan isi (setelah hasil tahap 1)

- [ ] **6. Motivasi dan pertanyaan penelitian** [T] · review 2.2
	- Pilihannya: tunjukkan dengan data kapan waktu perencanaan penting, atau jadikan bentuk jalur dan prediktabilitas sebagai klaim utama.
	- Tulis 2–3 pertanyaan penelitian secara eksplisit, lalu jawab satu per satu di kesimpulan.
	- Sebut di pendahuluan bahwa HOPE dan RL-OGM-Parking juga sudah beralih ke kurva RS, lalu jelaskan apa yang belum terselesaikan.
- [ ] **8. Kelengkapan dan keselamatan** [T+D] · review 4.5
	- Aturan tahan dengan jarak pengereman (b = 10 cm + jarak pengereman): perbaiki bila memilih 2(b); bila 2(a), tulis sebagai keterbatasan.

## Tahap 3 — Statistik

- [ ] **9. Uji statistik offline** [D] · review 4.4
	- Wilcoxon signed-rank untuk waktu dan biaya, serta interval kepercayaan bootstrap untuk median dan p90.
- [ ] **10. Tabel per skenario S1–S4** [D] · review 4.4
	- Isi dengan mean ± SD atau min–maks per skenario.
- [ ] **11. Kejelasan data latih dan uji** [T] · review 4.3
	- Caption Tabel I: peta sintetis belum pernah dilihat saat latih; tiga peta lainnya sudah dilihat (dengan rute berbeda); bagian bawah adalah subset dari run yang sama.
	- Jelaskan kenapa ID 01 dan ID 02 punya model sendiri.
	- Tambahkan waktu latih (review 4.3).
	- Sebut keterbatasan skala: arena ±2,5 × 2,2 m, robot 27 cm, tanpa muatan dan tanpa sensor onboard.

## Tahap 4 — Acuan dan penulisan

- [ ] **12. Acuan** · review 1
	- Menambah 12 acuan saja hanya memberi 20/37 = 54%. Untuk mencapai ≥60%, ganti atau hapus juga 3–4 acuan lama, misalnya [5] Paden, [22] Olmi, dan [25] Numba (pindah ke catatan kaki).
	- Cepat: `li2024hybrid`, `zheng2025multipark`, dan `tao2026adapting` sudah ada di `references.bib` tetapi belum dikutip.
	- Cek apakah [12], [14], [15], dan [16] sudah terbit di jurnal atau konferensi.
	- Area yang masih kurang: learned heuristic untuk Hybrid A\* / neural A\*, RL + RS untuk parking, survei planning car-like 2022+, dan koordinasi multi-AGV 2021+.
- [ ] **13. Konsistensi angka dan istilah** [T] · review 5 + temuan baru
	- **Tabel dan pengukuran:** tulis satu kali di mana waktu diukur.
	- **Penjelasan metode:** jelaskan cara memilih heading bebas, alasan bobot Eq. (2) dan definisi c_clr, serta bahwa w = 2 membuat heuristik tidak admissible.
	- **Istilah dan rumus:** ganti "estop" dengan "zone violation", tulis rumus makespan jadwal silang, dan definisikan p90, ID 01/02, makespan, serta arbiter saat pertama muncul.
	- **Format:** tambahkan email penulis kedua, perbesar label dan tambahkan legenda di Fig. 3.
- [ ] **14. Abstrak dan kesimpulan** [T] · review 2.5, 2.6
	- **Abstrak:** 150–200 kata, 4–5 angka kunci, satu alur cerita, dan menyebut keterbatasan utama.
	- **Kesimpulan:** menjawab pertanyaan penelitian, menyebut kapan DQfD tidak lebih baik, dan future work: skala dan jumlah robot lebih besar, serta fallback yang menjamin kelengkapan.
- [ ] **15. Gaya penulisan dan muat 6 halaman** [T] · review 5
	- Satu klaim per kalimat, dan setiap angka cukup muncul di satu tempat. Jangan memecah kalimat secara mekanis.
	- Anggaran halaman: [[01 Bukti Temuan dan Anggaran Halaman#2. Anggaran halaman dan gaya penulisan|anggaran]]

> [!note] Untuk buku tesis
> Yang tidak muat di 6 halaman bisa masuk buku tesis: ablasi observasi (tanpa ray, tanpa Dijkstra descent) dan tabel lengkap per skenario. Di buku tesis tidak ada batas halaman, jadi saran gaya dari reviewer bisa diterapkan penuh.

## Sumber

- Review: `~/Documents/Master ITB/paper-ieee-dqfd-ver-1-4/Review Pembimbing/Review paper tesis Eka N Two-Model DQfD dari Hybrid A dan Reeds–Shepp.pdf`
- Paper: `~/Documents/Master ITB/paper-ieee-dqfd-ver-1-4/output/Paper Eka N - Thesis ver5.pdf`, sumber LaTeX `src/paper_en_v5.tex`
- Catatan di vault utama: 18 dan 19 (peta dan skenario Gudang dan Gudang2)
