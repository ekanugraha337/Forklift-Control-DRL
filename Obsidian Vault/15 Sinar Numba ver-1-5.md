# Sinar dengan numba (ver-1-5)

Tanggal: 2026-11-04 · Branch `ver-1-5` · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Di ver-1-5, loop sinar observasi model jauh dikompilasi dengan **numba**.

> [!warning] Dibersihkan 2026-11-07 (aturan ver-1-6)
> Perbandingan waktu proses dan makespan Hybrid A\* + RS vs DQfD dua model dihapus, karena diukur dengan aturan lama. Di ver-1-6, 32 sinar **wajib** memakai numba, tanpa cadangan numpy.

> [!summary] Ringkasan
> - **Sinar 21× lebih cepat di Windows** (16,65 → 0,80 ms per kasus) dan **35× di Mac** (9,64 → 0,27 ms). Hasilnya identik bit demi bit, jadi model tidak perlu dilatih ulang.

---

## 1. Perubahan

Perubahannya ada di `drl/lingkungan.py`:
- **`_sinar_loop`** adalah loop sphere tracing per sinar. Fungsi ini dikompilasi numba: `@njit(cache=True, nogil=True)`.
- **`_sinar_loop_numpy`** adalah loop numpy lama, dipakai sebagai acuan verifikasi.
- **`_sinar()`** tetap menyiapkan `cx`, `cy`, `cos`/`sin` dengan numpy seperti sebelumnya, lalu memanggil loop numba.
- **`SINAR_NUMBA`** bernilai `True` bila numba aktif.

Supaya hasilnya identik dengan versi lama:
- `cos`/`sin` **tidak** dihitung di dalam numba. libm LLVM bisa berbeda satu ulp dari numpy, dan `floor()` akan memperbesar beda sekecil itu menjadi sel yang berbeda.
- Kernel numba hanya mengerjakan `t·u + c`, `floor`, perbandingan, `d − 0,5`, dan `+`, dengan urutan yang sama persis seperti versi numpy. Semua operasi itu hasilnya pasti identik menurut IEEE.
- `d − 0,5` tetap eksak walaupun `dist` bertipe float32, selama jarak < 2²² sel.

Alasan versi numba lebih cepat:
- Versi numpy lambat bukan karena hitungannya. Setiap iterasi menjalankan sekitar 28 operasi numpy pada larik 32 elemen, dan setiap operasi punya overhead pemanggilan ±0,3–0,5 µs.
- Selain itu, sinar yang sudah berhenti tetap ikut dihitung sampai sinar **terpanjang** selesai: rata-rata 22 iterasi, padahal hanya 9,8 dari 32 sinar yang masih aktif.
- Di versi numba, setiap sinar berhenti sendiri, dan semua operasi menjadi instruksi mesin.

Hal operasional:
- **Kompilasi:** dilakukan saat `lingkungan.py` diimpor, bukan saat rencana pertama robot. Hasilnya disimpan di `drl/__pycache__` (sudah di-ignore git). Impor pertama sekitar 0,9 s, impor berikutnya sekitar 0,4 s, yang sebagian besar adalah waktu impor numba sendiri.
- **`nogil=True`:** thread perencana melepas GIL selama sinar dihitung, sehingga tidak menahan loop kamera dan kendali di `navigasi.py`.
- **Dependensi:** `pip install numba`. Di laptop Windows terpasang `numba 0.68.1` dan `llvmlite 0.50.0`. Sebelumnya sudah dicek dengan `--dry-run`: numpy 2.4.2 tidak berubah.

## 2. Verifikasi

| Pemeriksaan | Hasil |
|---|---|
| Sinar numba vs numpy, 20.000 pose acak di semua peta: 2.000 per peta × 2 robot, termasuk pose menempel zona dan di luar peta | **20.000 / 20.000 identik**, selisih maksimum 0,0 |
