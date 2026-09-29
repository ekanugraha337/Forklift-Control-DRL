# Perbaikan Latensi Kamera: Dekoder OpenCV 16 Thread → 1 Thread

Diterapkan 2026-09-27. Latensi kamera ujung-ke-ujung turun dari **~0,68 s** ke **~0,09 s**.
Perubahannya hanya satu parameter dekoder.

---

## 1. Bagian yang diedit

**Berkas:** `Camera/cctv_lurus.py`

Ada dua perubahan yang saling berkaitan:

### a. Fungsi baru `buka_rtsp()` (baris 53–77)

```python
def buka_rtsp(url):
    """Buka aliran RTSP dengan dekoder SATU thread."""
    n_thread = getattr(cv2, "CAP_PROP_N_THREADS", None)
    if n_thread is None:            # OpenCV lama: tidak bisa diatur
        return cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    return cv2.VideoCapture(url, cv2.CAP_FFMPEG, [n_thread, 1])
    #                                            └────┬─────┘
    #                          pasangan (properti, nilai): CAP_PROP_N_THREADS = 1
```

Argumen ketiga `VideoCapture` adalah daftar parameter yang diterapkan **saat aliran dibuka**,
berupa pasangan `[properti, nilai, properti, nilai, ...]`. Di sini hanya satu pasangan, yaitu
`CAP_PROP_N_THREADS = 1`, yang menyuruh dekoder FFmpeg memakai satu thread.

Properti ini harus diberikan saat membuka aliran. Memanggil `cap.set(CAP_PROP_N_THREADS, 1)`
sesudahnya tidak berpengaruh, karena dekoder sudah terlanjur dibuat dengan 16 thread.

### b. `StreamReader._loop()` memanggil fungsi itu (baris 96)

```python
# SEBELUM (bawaan OpenCV: thread = jumlah prosesor logis)
cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)

# SESUDAH
cap = buka_rtsp(self.url)
```

> Bentuk "sebelum" diambil dari pembanding di `uji_kamera.py`, yang sengaja dibuat
> "persis seperti cctv_lurus.StreamReader" versi lama. Folder ini tidak memakai git,
> jadi tidak ada diff aslinya.

### Program yang ikut terpengaruh

Semua program membuka kamera lewat `StreamReader`, dan tidak ada yang memanggil
`cv2.VideoCapture` langsung ke RTSP. Jadi satu perubahan ini berlaku untuk semuanya:

| Program | Pemanggil |
|---|---|
| Navigasi robot | `Forklift Robot Program/pose.py` → `SumberKamera` → `C.StreamReader(...)` |
| Identifikasi parameter | `identifikasi.py` → `pose.SumberKamera` |
| Peta constraint | `Camera/area_constraint.py` (baris 351, 586) |
| Mode `lihat` / `kalibrasi` / `homografi` / `tag` | `Camera/cctv_lurus.py` (baris 345, 545, 730, 1016) |

---

## 2. Kenapa 16 thread justru memperlambat

### Dari mana angka 16

Backend FFmpeg di OpenCV membuka dekoder dengan thread **sebanyak prosesor logis**. PC ini
memakai AMD Ryzen 7 260 dengan 8 inti dan SMT, jadi ada **16 prosesor logis**.

### Frame threading menahan frame

Dekoder H.264 FFmpeg memakai *frame threading*: tiap thread mendekode **frame yang berbeda**
secara bersamaan. Frame tetap harus keluar **berurutan**, sehingga frame pertama baru bisa
dilepas setelah pipa berisi 16 thread itu terisi.

```
16 thread (bawaan)
  waktu →
  frame  1 ─► thread  1 ─┐
  frame  2 ─► thread  2  │
  frame  3 ─► thread  3  │   pipa harus penuh dulu;
     ...                 │   frame 1 keluar setelah ± 15 frame berikutnya masuk
  frame 16 ─► thread 16 ─┘
                           ─► frame 1 keluar   (tertahan ~15 frame)

1 thread (sesudah)
  frame n ─► didekode ─► langsung keluar       (tertahan ~0 frame)
```

Frame threading dirancang untuk **throughput** (frame per detik), misalnya saat memutar
video atau transkode. Untuk kendali waktu-nyata yang dipentingkan adalah **latensi**, dan di
situ pipa panjang justru merugikan.

### Hitungannya

| Besaran | Nilai |
|---|---|
| Laju kamera | ~26 fps → 1 frame ≈ 38 ms |
| Frame tertahan | 16 − 1 = 15 frame |
| **Tertunda di dekoder** | 15 × 38 ms ≈ **0,58 s** |

Angka ini cocok dengan selisih yang terukur, yaitu 0,59–0,61 s.

### Kenapa 1 thread tetap sanggup

Satu inti cukup cepat untuk mendekode satu frame 1280×720 dalam waktu kurang dari 38 ms,
sehingga dekoder tetap mengikuti 26 fps tanpa antrean. Hasil `uji_kamera.py` menunjukkan
latensinya tidak melayang (tidak bertambah seiring waktu) dan dekode tetap ~26 fps.

---

## 3. Bukti pengukuran

### a. Tanpa robot: `uji_kamera.py`

Aliran yang sama dibuka beberapa kali sekaligus dengan pengaturan berbeda. Semua sesi menerima
bitstream yang sama dan dekode H.264 bersifat deterministik, sehingga frame ke-k identik di semua
sesi. Tiap frame diberi sidik (hash), lalu dibandingkan kapan sidik yang sama muncul di tiap sesi.

| Pengaturan | Frame keluar lebih dulu dari bawaan | Perkiraan latensi |
|---|---|---|
| Bawaan (16 thread) | 0 ms (acuan) | ~0,68 s |
| **1 thread** | **589–611 ms** | **~0,07–0,09 s** |
| 2 thread | 558–561 ms | ~0,12 s |
| `StreamReader` terpasang | 590 ms | ~0,09 s |

`fflags;nobuffer` dan `flags;low_delay` juga sudah diuji dan **tidak menambah manfaat**
di atas 1 thread (−600 vs −611 ms).

### b. Di robot: `identifikasi.py --mode gas`

Posisi dari kamera dicocokkan dengan hitungan langkah stepper di telemetri firmware. Pergeseran
waktu yang dibutuhkan agar kedua kurva berimpit adalah latensi kamera.

| | Sebelum (`20260927_091649`) | Sesudah (`20260927_094202`) |
|---|---|---|
| Latensi kamera | 0,675 / 0,685 / 0,725 s | **0,085 / 0,095 / 0,095 s** |
| Stepper mulai setelah perintah | 0,06–0,34 s | 0,05–0,16 s |
| Akselerasi motor terukur | tak terukur (garis bengkok) | 11372 ± 493 sps² |
| Langkah hilang | tidak ada | tidak ada (cm/langkah dalam 0,2%) |

Karena cm/langkah cocok dan sisanya kecil, keterlambatan ini adalah pergeseran **waktu murni**.
Robot sebenarnya bergerak tepat waktu; yang terlambat adalah kameranya.

---

## 4. Dampak ke kendali robot

Pose yang dipakai navigasi tertinggal sejauh `v × latensi`:

| Laju | Sebelum (0,68 s) | Sesudah (0,09 s) |
|---|---|---|
| 17,4 cm/s (`FORWARD_SPEED = 7000`) | ~11,8 cm | ~1,6 cm |
| 24,9 cm/s (`FORWARD_SPEED = 10000`) | ~16,9 cm | ~2,2 cm |

Sebelum perbaikan, ketertinggalan ini **lebih besar dari `SAFETY_MARGIN_CM = 8`**.

Dalam loop pure pursuit, delay total `T` harus jauh lebih kecil dari horizon lookahead
`Ld / v`. Dengan `Ld = 26 cm`, perbaikan ini menurunkan `T` dari ~1,0 s ke ~0,4 s. Menurut
model linear, loop berpindah dari **ambang tidak stabil** ke **stabil dengan redaman sedang**.
Setelah perbaikan, penyumbang delay terbesar adalah kemudi (tunda 0,19 s + τ 0,10 s), bukan kamera.

Hasil di lapangan (`jalan_setelah_kamera.csv` dibandingkan dengan `cusp.csv`, rute berbeda
sehingga ini perbandingan indikatif):

| | Sebelum | Sesudah |
|---|---|---|
| Tujuan tercapai | 2 dari 3 | 4 dari 4 |
| Kejadian BAHAYA | 1 | 0 |
| Simpangan jalur rata / p95 / maks | 1,73 / 6,95 / 11,92 cm | 1,31 / 2,97 / 3,64 cm |
| Jarak terdekat ke zona | 0,55 cm | 4,29 cm |

---

## 5. Hal yang perlu dijaga

1. **Versi OpenCV.** Kalau `cv2.CAP_PROP_N_THREADS` tidak ada, yaitu pada OpenCV lama,
   `buka_rtsp()` **diam-diam kembali ke bawaan 16 thread** tanpa peringatan, dan latensi
   ~0,6 s muncul lagi. PC ini memakai OpenCV 5.0.0, jadi aman. Periksa setiap pindah PC
   atau setelah memperbarui paket:

   ```bash
   python -c "import cv2; print(cv2.__version__, hasattr(cv2, 'CAP_PROP_N_THREADS'))"
   ```

2. **Resolusi atau fps kamera.** Kalau aliran diganti ke resolusi yang jauh lebih besar,
   dekoder satu thread mungkin tidak lagi sanggup mengikuti fps. Frame akan menumpuk dan
   latensinya naik terus. Ukur ulang dengan `uji_kamera.py --detik 60`, yang melaporkan
   pelayangan itu. Kalau 1 thread tidak cukup, 2 thread hanya menambah ~1 frame (~50 ms).

3. **Rekaman lama.** Semua rekaman robot sebelum 2026-09-27 siang (`cusp.csv`, uji gas
   `085850` dan `091649`, `logs_identifikasi` yang lebih tua) dibuat dengan kamera ~0,68 s.
   Parameter yang diturunkan dari rekaman itu perlu dibaca dengan catatan ini.

4. **Simulator.** `konfig.SIM_KAMERA_DELAY_S = 0,025` ditambah umur frame dan jitter terukur
   ~0,04–0,05 s, jadi simulator kini hanya ~0,05 s lebih cepat dari robot. Selisih ini jauh
   lebih kecil dari sebelumnya (0,63 s), tetapi belum nol.

---

## 6. Cara memeriksa ulang

```bash
cd "Forklift Robot Program"

# tanpa robot: bandingkan latensi dekoder antar-pengaturan
python uji_kamera.py
python uji_kamera.py --detik 60        # cek apakah latensi melayang

# dengan robot: latensi kamera ujung-ke-ujung (robot menghadap lintasan lurus ~1,2 m)
python identifikasi.py --peta Kalibrasi --mode gas

# analisis ulang rekaman yang sudah ada
python identifikasi.py --dari logs_identifikasi/20260927_094202
```
