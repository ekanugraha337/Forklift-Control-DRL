# Forward jaringan dengan numpy (ver-1-5)

Tanggal: 2026-11-04 · Branch `ver-1-5`, commit `9cf9eee` · Lanjutan dari [[15 Sinar Numba ver-1-5|Sinar Numba ver-1-5]] · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Forward jaringan DQfD di perencana operasional dihitung dengan **numpy**, memakai bobot model yang sama dan tanpa latih ulang.

> [!warning] Dibersihkan 2026-11-07 (aturan ver-1-6)
> Perbandingan waktu proses dan makespan Hybrid A\* + RS vs DQfD dua model dihapus, karena diukur dengan aturan lama. Di ver-1-6, forward numpy juga dipakai **saat pelatihan** (rollout dan evaluasi). PyTorch hanya dipakai untuk gradien.

---

## 1. Perubahan

| Berkas | Isi |
|---|---|
| `drl/jaringan_numpy.py` (baru, **tanpa torch**) | `JaringanNumpy`: forward Dueling DQN float32. `muat_dari_pt`: bobot dari `.pt` lewat cache `.npz`. `periksa_kontrak` dan `periksa_profil`, dipindah dari `dqfd.py`. `SKALA_REWARD` |
| `drl/perencana_dqfd.py` | forward lewat `_q()` / `_qv()`, yang menerima `JaringanNumpy` **atau** jaringan PyTorch. `DQfDPlanner` memakai `muat_model_numpy()`. `import torch` hanya di jalur PyTorch |
| `drl/dqfd.py`, `drl/jaringan.py` | mengimpor pemeriksaan dan `SKALA_REWARD` dari `jaringan_numpy`. Impor lama (`from drl.dqfd import periksa_profil`, dll.) tetap berfungsi |

**Forward numpy.**
- Bentuk jaringannya sama dengan `drl/jaringan.py`: 47 → 256 → 256 → (V 128 | A 128) → V + A − rata(A).
- Cabang V dan A **digabung**: v1|a1 menjadi satu matriks 256 × 256, dan v2, a2 menjadi satu matriks blok 256 × (1 + n_aksi). Alasannya, yang mahal adalah overhead per operasi numpy, bukan hitungannya, jadi jumlah operasi dibuat sesedikit mungkin.
- Satu forward model jauh di Mac:

| Versi | Waktu |
|---|---:|
| PyTorch | 37 µs |
| numpy, cabang terpisah | 22 µs |
| **numpy, cabang digabung** | **16 µs** |

**Cache `.npz`.**
- Bobot disalin sekali ke `model_dqn/id0N_<jenis>.npz`, bersama md5 file `.pt` dan meta-nya. Folder `model_dqn/` sudah di-ignore git.
- Selama md5 `.pt` sama, model dimuat **tanpa mengimpor torch**.
- Bila model dilatih ulang, cache dibangun ulang otomatis. Hanya pada saat itulah torch masih diimpor.
- Kontrak versi dan profil kemudi tetap diperiksa setiap kali model dimuat.

## 2. Verifikasi

| Pemeriksaan | Mac | Windows |
|---|---|---|
| Selisih Q numpy vs PyTorch model jauh, pada observasi nyata selama merencanakan 124 rute | maks **4,8e-6** | maks **3,8e-6** |
| torch diimpor selama merencanakan (setelah cache ada) | tidak | tidak |
