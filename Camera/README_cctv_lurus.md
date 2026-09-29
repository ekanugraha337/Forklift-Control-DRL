# cctv_lurus.py & area_constraint.py — Koreksi Lensa CCTV, Pelacakan Posisi & Zona Constraint Robot

Dua program Python yang saling terhubung:

**`cctv_lurus.py`** — kalibrasi & pelacakan:

1. **Menampilkan** video CCTV (RTSP) secara langsung.
2. **Meluruskan** gambar kamera yang cembung (distorsi lensa *barrel/fisheye*) memakai garis nat ubin lantai 30 × 30 cm.
3. **Memetakan** piksel kamera ke koordinat lantai dalam **cm** (homografi).
4. **Mendeteksi AprilTag** di atas robot/forklift dan menampilkan posisi **X, Y (cm)** serta **arah (derajat)** secara real-time, lengkap dengan koreksi tinggi tag.

**`area_constraint.py`** — keselamatan area:

5. **Menggambar zona constraint** (area larangan & batas kerja) pada beberapa **peta** yang bisa dipilih.
6. **Memantau** seluruh badan robot terhadap zona: status AMAN / PERINGATAN / PELANGGARAN / HILANG, alarm, log CSV, dan sinyal UDP ke kontroler robot.

---

## Daftar Isi

- [1. Persiapan](#1-persiapan)
- [2. Alur Kerja Singkat](#2-alur-kerja-singkat)
- [3. Mode `kalibrasi` — meluruskan lensa](#3-mode-kalibrasi--meluruskan-lensa)
- [4. Mode `lihat` — menonton video terkoreksi](#4-mode-lihat--menonton-video-terkoreksi)
- [5. Mode `homografi` — piksel ke cm lantai](#5-mode-homografi--piksel-ke-cm-lantai)
- [6. Menyiapkan AprilTag](#6-menyiapkan-apriltag)
- [7. Mode `tag` — posisi robot](#7-mode-tag--posisi-robot)
- [8. Uji Akurasi](#8-uji-akurasi)
- [9. Zona Constraint — area_constraint.py](#9-zona-constraint--area_constraintpy)
- [10. Referensi Opsi Baris Perintah](#10-referensi-opsi-baris-perintah)
- [11. File yang Dihasilkan](#11-file-yang-dihasilkan)
- [12. Cara Kerja (ringkas)](#12-cara-kerja-ringkas)
- [13. Pemecahan Masalah](#13-pemecahan-masalah)

---

## 1. Persiapan

### Kebutuhan

- Python 3.10+ (sudah diuji dengan Python 3.14)
- Paket: `opencv-python` (≥ 4.7, sudah diuji dengan 5.0) dan `numpy`

```
python -m pip install opencv-python numpy
```

Deteksi AprilTag memakai modul `cv2.aruco` yang sudah termasuk di `opencv-python`, jadi **tidak perlu** library tambahan.

### Konfigurasi kamera

Buka `cctv_lurus.py` dan sesuaikan bagian atas file:

```python
CAMERA_IP = "192.168.100.250"
CAMERA_PORT = 554
USERNAME = "onvifuser"
PASSWORD = "..."
STREAM_PATH = "/Streaming/Channels/101?transportmode=unicast&profile=Profile_1"
```

| Channel | Resolusi | Keterangan |
|---|---|---|
| `101` | 1280 × 720 | *Main stream* — **dipakai**, lebih tajam, lebih baik untuk deteksi tag |
| `102` | 640 × 360 | *Sub stream* — lebih ringan, tetapi tag kecil sulit terdeteksi |

> Kalibrasi disimpan relatif terhadap ukuran gambar, jadi berpindah channel **tidak perlu kalibrasi ulang** selama rasio gambarnya sama (16:9).

> ⚠️ Password tersimpan sebagai teks biasa di dalam script. Jangan membagikan file ini.

### Tips kamera (opsional)

Matikan teks OSD bawaan kamera (tanggal/jam dan "Camera 01") lewat web kamera
`http://192.168.100.250` → **Configuration → Image → OSD Settings**, agar teks tidak menutupi tag di pojok gambar.

---

## 2. Alur Kerja Singkat

```
 (sekali)                 (sekali)                  (sekali)               (setiap dipakai)
┌────────────┐   ┌──────────────────────┐   ┌──────────────────────┐   ┌───────────────────┐
│ kalibrasi  │ → │ homografi            │ → │ cetak & pasang tag,  │ → │ tag               │
│ lurus-kan  │   │ klik sudut ubin      │   │ ukur tinggi kamera   │   │ posisi X,Y + arah │
│ lensa      │   │ → piksel ke cm       │   │ & tinggi tag         │   │ robot real-time   │
└────────────┘   └──────────────────────┘   └──────────────────────┘   └───────────────────┘
       │                                                                          │
       └──→ lihat  (opsional: menonton video yang sudah lurus)                    │
                                                                                  ▼
                                  (sekali per peta)                     (setiap dipakai)
                              ┌──────────────────────┐            ┌──────────────────────────┐
                              │ area_constraint.py   │     →      │ area_constraint.py       │
                              │ setup                │            │ pantau                   │
                              │ gambar zona per peta │            │ cek robot vs zona        │
                              └──────────────────────┘            └──────────────────────────┘
```

| Perintah | Fungsi |
|---|---|
| `python cctv_lurus.py kalibrasi` | Mencari parameter lensa agar garis nat ubin lurus |
| `python cctv_lurus.py lihat` | Menampilkan video yang sudah diluruskan |
| `python cctv_lurus.py homografi` | Membuat pemetaan piksel → cm lantai |
| `python cctv_lurus.py tag` | Mendeteksi AprilTag dan menampilkan posisi robot |
| `python cctv_lurus.py` | Otomatis: `lihat` jika sudah ada kalibrasi, jika belum → `kalibrasi` |
| `python area_constraint.py daftar` | Melihat semua peta constraint |
| `python area_constraint.py setup` | Membuat / mengedit peta constraint (zona) |
| `python area_constraint.py pantau` | Memantau robot terhadap zona pada peta terpilih |

> `area_constraint.py` **tidak perlu** dijalankan bersamaan dengan `cctv_lurus.py`. Program ini membaca hasil kalibrasi, homografi, dan tinggi dari `kalibrasi_lensa.json`, lalu melakukan deteksi tag sendiri. Jangan jalankan `cctv_lurus.py tag` dan `area_constraint.py pantau` bersamaan (dua stream ke kamera, beban CPU ganda).

> **Penting:** jika kalibrasi lensa diulang, **homografi juga harus diulang**. Program akan memberi peringatan bila keduanya tidak cocok.

---

## 3. Mode `kalibrasi` — meluruskan lensa

```
python cctv_lurus.py kalibrasi
```

**Prinsip:** garis nat ubin di dunia nyata lurus. Anda menandai beberapa garis nat yang terlihat melengkung, lalu program mencari parameter lensa yang membuat semua garis itu lurus kembali.

Tiga jendela akan terbuka:

| Jendela | Isi |
|---|---|
| **Asli - klik titik di garis nat ubin** | Frame asli dari kamera, tempat Anda mengklik |
| **Hasil koreksi** | Pratinjau gambar setelah diluruskan |
| **Parameter** | Slider `model`, `f`, `k1`, `k2`, `zoom` |

### Langkah

1. Pada jendela **Asli**, klik **4–8 titik** di sepanjang **satu** garis nat, dari ujung ke ujung.
2. Tekan **`n`** untuk memulai garis berikutnya (warna berbeda).
3. Ulangi sampai **minimal 3 garis** (disarankan 5 atau lebih). Pilih garis yang:
   - dekat **tepi gambar** (distorsinya paling besar),
   - campuran **horizontal dan vertikal**,
   - termasuk garis pertemuan **lantai–dinding**.
4. Tekan **`a`** (AUTO). Angka *error kelurusan* di jendela Hasil harus turun jauh.
5. Periksa jendela **Hasil koreksi** — tekan `g` untuk grid bantu.
6. Atur slider **`zoom`** jika perlu (kecil = area lebih luas dengan tepi hitam, besar = tepi hitam hilang tetapi terpotong).
7. Tekan **`s`** untuk menyimpan, lalu **`q`** untuk keluar.

> ⚠️ **Satu garis = satu nat lurus.** Jangan menyambung titik yang berbelok (misalnya garis atas lalu turun ke garis kanan). Program menampilkan peringatan merah jika garis berbelok.

> Jangan mengubah `f`, `k1`, `k2` setelah AUTO — hanya `zoom` yang aman diubah.

### Tombol

| Tombol | Fungsi |
|---|---|
| Klik kiri | Tambah titik pada garis aktif |
| `n` | Mulai garis baru |
| `u` | Hapus titik terakhir |
| `r` | Hapus semua garis |
| `a` | AUTO — cari parameter terbaik (min. 3 garis) |
| `m` | Ganti model lensa: standar / fisheye (untuk kamera ini **fisheye** lebih baik) |
| `spasi` | Ambil frame baru dari kamera |
| `g` | Tampilkan/sembunyikan grid bantu |
| `s` | Simpan kalibrasi |
| `q` / `Esc` | Keluar |

---

## 4. Mode `lihat` — menonton video terkoreksi

```
python cctv_lurus.py lihat
```

| Tombol | Fungsi |
|---|---|
| `w` | Ganti tampilan: **potong** (luar kotak dihitamkan) → **kotak diluruskan** (persegi panjang) → **penuh** + garis kotak |
| `r` | Gambar ulang kotak area: klik 4 sudut |
| `b` | Bandingkan dengan video asli (berdampingan) |
| `s` | Screenshot (`capture_YYYYMMDD_HHMMSS.jpg`) |
| `q` / `Esc` | Keluar |

Kotak area dan pilihan tampilan disimpan otomatis di `kalibrasi_lensa.json`.

> Tampilan "kotak diluruskan" hanya untuk kenyamanan melihat, **bukan** untuk mengukur. Gunakan mode `homografi` + `tag` untuk pengukuran dalam cm.

---

## 5. Mode `homografi` — piksel ke cm lantai

```
python cctv_lurus.py homografi
```

Homografi memetakan bidang lantai di gambar ke koordinat nyata (X, Y) dalam cm, memakai ukuran ubin 30 cm.
Contoh bergambar ada di folder [`Contoh Homografi`](Contoh%20Homografi/).

**Sebelum mulai:** kosongkan lantai sebisa mungkin (tidak ada orang/kursi/tas yang menutupi nat).

### Langkah 1 — klik 4 sudut

Pilih persegi panjang ubin **sebesar mungkin** yang terlihat, lalu klik sudutnya **tepat di persilangan nat**, berurutan:

| Klik | Contoh posisi | Arti |
|---|---|---|
| **1** | kiri-atas | titik asal **(0, 0)** |
| **2** | kanan-atas | ujung arah sumbu **X** |
| **3** | kanan-bawah | sudut diagonal |
| **4** | kiri-bawah | ujung arah sumbu **Y** |

![Langkah 1](Contoh%20Homografi/Langkah%201%20-%20klik%204%20sudut.png)

### Langkah 2 — ketik jumlah ubin

Pastikan jendela program aktif, lalu:

1. Ketik jumlah **kotak ubin** dari titik 1 ke titik 2 (contoh `9`) → **Enter**
2. Ketik jumlah **kotak ubin** dari titik 1 ke titik 4 (contoh `7`) → **Enter**

> Yang dihitung adalah **jumlah kotak**, bukan jumlah garis nat (9 kotak = 10 garis).

![Langkah 2](Contoh%20Homografi/Langkah%202%20-%20ketik%20jumlah%20ubin.png)

### Langkah 3 — tambah titik

Program menggambar **garis biru** (tebakan grid). Klik **20–40 sudut ubin lain** yang terlihat jelas dan tersebar (tepi, tengah, pojok jauh). Setiap klik otomatis dicocokkan ke sudut ubin terdekat — tidak perlu mengetik apa pun.

![Langkah 3](Contoh%20Homografi/Langkah%203%20-%20klik%20titik%20tambahan.png)

Gunakan **lup** (kaca pembesar) di pojok kanan atas. Klik di **persilangan nat asli**, bukan di garis biru:

![Tips lup](Contoh%20Homografi/Tips%20-%20lup%20klik%20tepat.png)

Warna titik:

| Warna | Arti |
|---|---|
| 🟢 Hijau | Error < 1 cm — bagus |
| 🟡 Kuning | Error < 2,5 cm — cukup |
| 🔴 Merah | Kemungkinan salah klik → tekan `u` |

### Langkah 4 — simpan

Pastikan **error rata-rata (RMS) < 1 cm**, lalu tekan **`s`** dan **`q`**.

Layar juga menampilkan **perkiraan posisi & tinggi kamera**. Bandingkan dengan tinggi kamera hasil ukur meteran.

> Homografi yang sudah tersimpan akan dimuat otomatis saat mode ini dibuka lagi, sehingga Anda bisa langsung menambah titik. Tekan `r` untuk mulai dari awal.

### Tombol

| Tombol | Fungsi |
|---|---|
| Klik kiri | Tambah titik (4 sudut awal / sudut ubin tambahan) |
| `0`–`9`, `Backspace`, `Enter` | Mengetik jumlah ubin (Langkah 2) |
| `u` | Hapus titik terakhir |
| `r` | Ulang dari awal |
| `spasi` | Ambil frame baru (misalnya setelah orang lewat) |
| `s` | Simpan |
| `q` / `Esc` | Keluar (`Esc` saat mengetik = batal) |

---

## 6. Menyiapkan AprilTag

File siap cetak ada di folder [`AprilTag`](AprilTag/):

| File | Ukuran kotak hitam |
|---|---|
| `apriltag_36h11_ID0-3_10cm_A4.pdf` | 10 × 10 cm (ID 0–3) |
| `apriltag_36h11_ID0-3_15cm_A4.pdf` | 15 × 15 cm (ID 0–3) |

- **Family:** `tag36h11` (default program).
- Hasil simulasi pada kamera ini: tag **10 cm ≈ 28 px** di gambar dan terdeteksi di seluruh area lantai.

### Mencetak

1. Cetak di kertas **A4** dengan skala **100% / "Actual size"** — **jangan** "Fit to page".
2. Ukur hasilnya: garis penggaris dan sisi kotak hitam harus tepat 10 cm (atau 15 cm).
3. Gunakan kertas tebal atau laminasi **doff** (tidak mengkilap).
4. Potong di garis putus-putus — **sisakan tepi putih**.

### Memasang di robot

- Tempel **rata / datar** (sejajar lantai) di atas robot, sebaiknya dekat titik tengah robot.
- **Panah "DEPAN ROBOT" menghadap arah maju robot** (sisi atas tag = depan).
- Pastikan tidak tertutup bagian robot dari sudut pandang kamera.
- Gunakan **ID berbeda untuk setiap robot**.

### Ukur dua tinggi

| Ukuran | Cara ukur |
|---|---|
| Tinggi kamera | Lantai → lensa kamera (saat ini: **270 cm**) |
| Tinggi tag | Lantai → permukaan tag di atas robot |

---

## 7. Mode `tag` — posisi robot

```
python cctv_lurus.py tag --tinggi-tag 25 --tinggi-kamera 270
```

Nilai `--tinggi-tag` dan `--tinggi-kamera` **disimpan**, jadi selanjutnya cukup:

```
python cctv_lurus.py tag
```

### Jendela

**1. "Tag - posisi robot"** — dua pilihan tampilan (tombol `v`):

- **Tampilan ATAS (default):** video diubah menjadi pandangan dari atas dalam cm. Ubin tampak persegi dan grid biru (tiap 30 cm) tepat di atas nat asli.
  - Area = kotak titik homografi **+ offset 30 cm** di setiap sisi (ubah dengan `--offset`).
  - **Garis kuning** = batas area homografi asli.
- **Tampilan kamera penuh:** gambar kamera yang sudah diluruskan.

Di atas tag digambar kotak **hijau** dengan garis **merah** di sisi depan.

**2. "Peta lantai (cm)"** — peta 2D dari atas:

- titik hijau + panah merah = posisi & arah robot, beserta label `ID (X, Y)`,
- garis hijau tipis = jejak lintasan,
- silang kuning = titik asal (0, 0), silang biru = posisi kamera,
- teks atas = **fps** dan **waktu proses** per frame.

### Tombol

| Tombol | Fungsi |
|---|---|
| `v` | Ganti tampilan video: **atas** (cm) / kamera penuh |
| `g` | Tampilkan/sembunyikan grid ubin |
| `o` | Tampilkan/sembunyikan teks posisi di video (default: sembunyi) |
| `p` | Tampilkan/sembunyikan jendela peta |
| `s` | Screenshot (`tag_YYYYMMDD_HHMMSS.jpg`) |
| `q` / `Esc` | Keluar |

### Sistem koordinat

- **Satuan:** cm.
- **Titik asal (0, 0):** titik ke-1 yang diklik saat homografi.
- **Sumbu +X:** dari titik 1 ke titik 2. **Sumbu +Y:** dari titik 1 ke titik 4.
- **Sudut:** 0° = menghadap +X, 90° = menghadap +Y, bernilai −180° s.d. 180°.
- **Posisi robot** = titik tengah tag (rata-rata 4 sudut tag).

### Menyimpan data posisi (CSV)

```
python cctv_lurus.py tag --log posisi_forklift.csv
```

Format (data ditambahkan ke file yang sudah ada):

```
waktu,id,x_cm,y_cm,sudut_deg
1789626543.120,0,135.42,88.17,91.35
```

`waktu` = *Unix timestamp* (detik).

---

## 8. Uji Akurasi

Lakukan sebelum memakai sistem untuk robot.

### A. Tag di lantai

1. Tandai pusat tag: tarik dua diagonal kotak hitam dengan pensil tipis.
2. Jalankan `python cctv_lurus.py tag --tinggi-tag 0`
3. Letakkan tag **rata di lantai** dengan pusat tepat di sudut ubin, dan bandingkan:

| Posisi (sudut ubin) | X, Y seharusnya | Terbaca | Error |
|---|---|---|---|
| ubin (1, 1) | 30, 30 | | |
| ubin (4, 3) | 120, 90 | | |
| ubin (8, 6) | 240, 180 | | |
| ubin (8, 1) | 240, 30 | | |

   Koordinat seharusnya = indeks ubin × 30 cm.
4. Putar tag: panah ke +X ≈ 0°, ke +Y ≈ 90°, ke −X ≈ 180°.

**Target:** error posisi < 2–3 cm, sudut < 2°.

### B. Tag di robot

1. Jalankan dengan tinggi tag sebenarnya: `python cctv_lurus.py tag --tinggi-tag <cm>`
2. Posisikan robot sehingga pusat tag **tepat di atas** sudut ubin (pakai unting-unting atau penggaris tegak).
3. Bandingkan X, Y.

| Gejala | Kemungkinan penyebab |
|---|---|
| Di lantai akurat, di robot meleset | Tinggi tag / tinggi kamera salah diisi |
| Error membesar makin jauh dari kamera | Tinggi kamera salah |
| Error besar di tepi area | Titik homografi kurang di tepi, atau kalibrasi lensa kurang baik di tepi |
| Error besar di mana-mana | Ulangi homografi dengan lebih banyak titik |

---

## 9. Zona Constraint — area_constraint.py

`area_constraint.py` menentukan area yang **tidak boleh dilewati** robot dan memantaunya secara real-time. Program ini dibangun di atas `cctv_lurus.py` (lensa, homografi, koreksi tinggi, dan deteksi AprilTag dipakai ulang lewat `import`).

> ⚠️ Program ini **mendeteksi dan memperingatkan** — tidak dapat menghentikan robot secara fisik. Agar robot benar-benar berhenti, kontroler robot harus membaca status yang dikirim lewat UDP (lihat [9.6](#96-menghentikan-robot-lewat-udp)).

### 9.1 Prasyarat

Semua sudah diatur lewat `cctv_lurus.py` dan disimpan di `kalibrasi_lensa.json`:

1. `python cctv_lurus.py kalibrasi` — lensa
2. `python cctv_lurus.py homografi` — piksel → cm
3. `python cctv_lurus.py tag --tinggi-tag <cm> --tinggi-kamera <cm>` — simpan tinggi, lalu tekan `q`

> **Tinggi tag dan tinggi kamera hanya diatur di `cctv_lurus.py`.** `area_constraint.py` memakai nilai yang tersimpan. Jika tinggi tag belum diisi (0 cm), posisi robot bisa meleset (≈ 14 cm untuk tag 25 cm), sehingga pengecekan zona tidak akurat.

### 9.2 Konsep

**Peta constraint** = kumpulan zona untuk satu situasi/tata letak (misalnya `gudang_A`, `jalur_sempit`, `saat_loading`). Anda bisa membuat banyak peta dan memilih salah satu saat memantau.

**Jenis zona:**

| Jenis | Warna | Aturan |
|---|---|---|
| **larangan** | merah | Robot **tidak boleh menyentuh / masuk** zona ini (rak, meja, jalur orang) |
| **batas** | hijau (luar zona diarsir merah) | **Seluruh badan** robot harus tetap **di dalam** zona ini (batas area kerja). Biasanya cukup satu per peta. |

Zona boleh berbentuk poligon bebas (misalnya huruf L), tidak harus persegi.

**Badan robot:** yang dicek adalah persegi panjang **panjang × lebar** yang ikut berputar sesuai arah robot — bukan hanya titik tag.

```
              panjang (arah maju) →
        ┌───────────────────────────┐
        │                           │
 lebar  │     [tag]──geser──►●      │ ──► arah DEPAN robot
        │                  pusat    │
        └───────────────────────────┘
```

`geser` = jarak pusat robot **di depan** pusat tag (negatif jika pusat robot di belakang tag). Isi 0 jika tag dipasang di tengah robot.

**Status:**

| Status | Warna | Kondisi |
|---|---|---|
| **AMAN** | 🟢 | Jarak badan robot ke semua zona ≥ margin |
| **PERINGATAN** | 🟡 | Jarak < margin (default 20 cm), **atau** diprediksi melanggar dalam `prediksi` detik (default 0,5 s) dengan kecepatan saat ini |
| **PELANGGARAN** | 🔴 | Badan robot menyentuh zona larangan / keluar dari zona batas. Bingkai layar merah + alarm |
| **HILANG** | ⚪ | Tag tidak terdeteksi > 1 detik |

### 9.3 Mengelola peta

Setiap peta disimpan sebagai file sendiri di folder `peta_constraint/`.

```
python area_constraint.py daftar
```

```
Peta constraint di ...\peta_constraint:
  - gudang_A                  2 zona (1 larangan, 1 batas)        diubah 2026-09-17 15:28:52
  - jalur_sempit              3 zona (3 larangan, 0 batas)        diubah 2026-09-18 09:10:04
Robot: 60 x 40 cm, geser 0 cm, margin 20 cm, prediksi 0.5 s
```

| Perintah | Fungsi |
|---|---|
| `python area_constraint.py setup` | Tampilkan menu: ketik **nomor** peta untuk diedit, atau **nama baru** untuk membuat peta |
| `python area_constraint.py setup --peta gudang_A` | Langsung edit (atau buat) peta `gudang_A` |
| `python area_constraint.py setup --peta gudang_C --dari gudang_A` | Buat peta baru dengan zona awal **disalin** dari `gudang_A` |

- Nama peta tidak peka huruf besar/kecil (`gudang_a` = `Gudang_A`). Karakter selain huruf, angka, `_`, `-` diganti `_` (misalnya `Gudang A` → `Gudang_A`).
- `--dari` hanya untuk peta baru/kosong (tidak menimpa zona yang sudah ada).
- Untuk menghapus peta, hapus file `peta_constraint/<nama>.json`.

### 9.4 Mode `setup` — menggambar zona

```
python area_constraint.py setup --peta gudang_A
```

Jendela menampilkan **tampilan atas lantai** (sama seperti `cctv_lurus.py tag`) dengan grid 30 cm, kotak kuning area homografi, dan koordinat kursor dalam cm.

**Langkah:**

1. Pilih jenis zona dengan **`t`** (LARANGAN ↔ BATAS) — lihat teks di kiri atas.
2. (Opsional) tekan **`x`** untuk snap ke kelipatan 5 / 15 / **30 cm** (30 cm = tepat di sudut ubin).
3. **Klik kiri** setiap sudut zona secara berurutan (searah atau berlawanan jarum jam).
4. **Klik kanan** atau **Enter** untuk menutup zona (min. 3 titik). Zona diberi nama otomatis `Z1`, `Z2`, …
5. Ulangi untuk zona lain.
6. Tekan **`s`** untuk menyimpan, lalu **`q`** untuk keluar.

| Tombol | Fungsi |
|---|---|
| Klik kiri | Tambah titik sudut zona |
| Klik kanan / `Enter` | Tutup zona |
| `t` | Ganti jenis zona: larangan ↔ batas |
| `u` | Hapus titik terakhir (jika kosong: hapus zona terakhir) |
| `d` | Hapus zona di bawah kursor |
| `x` | Snap koordinat: bebas / 5 / 15 / 30 cm |
| `spasi` | Ambil frame baru dari kamera |
| `s` | Simpan peta |
| `q` / `Esc` | Keluar (jika belum disimpan, muncul peringatan — tekan `q` sekali lagi untuk tetap keluar) |

> Zona disimpan dalam **cm** (koordinat lantai), bukan piksel. Zona tetap valid meskipun `--offset` tampilan diubah. Jika homografi diulang dengan titik asal (0,0) yang **berbeda**, zona perlu digambar ulang.

### 9.5 Mode `pantau` — memantau robot

```
python area_constraint.py pantau --peta gudang_A --robot-panjang 60 --robot-lebar 40
```

- Tanpa `--peta`: muncul menu pilih peta di terminal (jika hanya ada satu peta, langsung dipakai).
- Ukuran robot, `geser`, `margin`, dan `prediksi` **disimpan** di `robot_constraint.json` (berlaku untuk semua peta), jadi selanjutnya cukup `python area_constraint.py pantau`.

**Tampilan:**

- Zona larangan diarsir merah; area di luar zona batas diarsir merah.
- Badan robot digambar sebagai persegi panjang berwarna sesuai status, dengan panah arah dan label, misalnya `ID 0 PERINGATAN 12 cm ke Z1 (prediksi)`.
- Garis tipis di depan robot = posisi **prediksi** (jika prediksi aktif dan robot bergerak).
- Teks kiri atas: nama peta aktif dan ringkasan status semua robot.
- Bingkai merah tebal di tepi layar jika ada pelanggaran.
- Status berubah → dicetak di terminal (dan log CSV jika `--log`).

| Tombol | Fungsi |
|---|---|
| `[` / `]` | Ganti ke peta sebelumnya / berikutnya **tanpa restart** |
| `r` | Muat ulang peta aktif dari file (setelah diedit di `setup`) |
| `g` | Tampilkan/sembunyikan grid |
| `z` | Tampilkan/sembunyikan zona |
| `m` | Alarm suara on/off |
| `s` | Screenshot (`zona_YYYYMMDD_HHMMSS.jpg`) |
| `q` / `Esc` | Keluar |

**Uji tanpa robot (simulasi):**

```
python area_constraint.py pantau --simulasi
```

"Robot" (ID 0) mengikuti mouse; tekan `a` / `d` untuk memutarnya. Gunakan untuk memeriksa bentuk zona, margin, dan prediksi sebelum dipakai robot sungguhan.

**Log CSV** (`--log pelanggaran.csv`) — satu baris setiap status berubah:

```
waktu,peta,id,status,zona,jarak_cm,x_cm,y_cm,sudut_deg
1789633776.883,gudang_A,0,PERINGATAN,Z1,16.0,114.00,105.00,0.00
1789633777.043,gudang_A,0,PELANGGARAN,Z1,0.0,130.00,105.00,0.00
```

### 9.6 Menghentikan robot lewat UDP

```
python area_constraint.py pantau --udp 192.168.100.50:5005
```

Setiap frame (±25 kali per detik) dikirim pesan JSON:

```json
{"waktu": 1789632793.46, "peta": "gudang_A",
 "robot": [{"id": 0, "status": "PERINGATAN", "zona": "Z1", "prediksi": true,
            "jarak_cm": 20.0, "x_cm": 110.0, "y_cm": 105.0, "sudut_deg": 0.0}]}
```

Contoh penerima minimal di sisi robot:

```python
import socket, json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", 5005))
s.settimeout(0.5)
while True:
    try:
        msg = json.loads(s.recv(4096))
        st = next((r["status"] for r in msg["robot"] if r["id"] == 0), "HILANG")
    except socket.timeout:
        st = "HILANG"                          # tidak ada pesan = kamera/PC mati
    if st in ("PELANGGARAN", "HILANG"):
        stop_robot()                           # ganti dengan perintah motor Anda
    elif st == "PERINGATAN":
        pelankan_robot()
```

> **Fail-safe:** perlakukan "tidak ada pesan" dan status **HILANG** sebagai **berhenti**, agar robot berhenti jika kamera, jaringan, atau PC bermasalah.

### 9.7 Tips keselamatan & akurasi

- **Margin** sebaiknya ≥ error posisi sistem (uji dulu, lihat [Uji Akurasi](#8-uji-akurasi)) + jarak pengereman robot. Mulai dari 20–30 cm.
- Pada 25 fps dan kecepatan robot 1 m/s, robot bergerak **4 cm per frame**. Tambahkan juga jeda jaringan UDP dan waktu respons motor.
- Gunakan **prediksi** (default 0,5 s) agar peringatan muncul sebelum robot menyentuh zona.
- Akurasi di area **offset** (di luar kotak kuning homografi) sedikit lebih rendah. Hindari menaruh batas zona penting di sana, atau tambah titik homografi.
- Zona tidak memperhitungkan tinggi benda: zona larangan berlaku untuk proyeksi badan robot di lantai.

---

## 10. Referensi Opsi Baris Perintah

### `cctv_lurus.py`

```
python cctv_lurus.py [mode] [opsi]
```

| Opsi | Default | Keterangan |
|---|---|---|
| `mode` | otomatis | `kalibrasi`, `lihat`, `homografi`, atau `tag` |
| `--url URL` | dari konfigurasi | Pakai URL RTSP lain |
| `--tinggi-tag CM` | tersimpan / 0 | Tinggi AprilTag dari lantai (disimpan) |
| `--tinggi-kamera CM` | tersimpan / perkiraan | Tinggi kamera dari lantai (disimpan) |
| `--family F` | `36h11` | Family AprilTag: `36h11`, `36h10`, `25h9`, `16h5` |
| `--log FILE` | – | Simpan posisi ke CSV |
| `--offset CM` | `30` | Perluasan tampilan atas dari tiap sisi area homografi (`0` = hanya area homografi) |

Contoh:

```
python cctv_lurus.py tag --tinggi-tag 25 --offset 60 --log uji1.csv
python cctv_lurus.py tag --family 25h9
python cctv_lurus.py lihat --url "rtsp://user:pass@192.168.100.250:554/Streaming/Channels/102"
```

### `area_constraint.py`

```
python area_constraint.py {daftar | setup | pantau} [opsi]
```

| Opsi | Mode | Default | Keterangan |
|---|---|---|---|
| `--peta NAMA` | setup, pantau | menu pilihan | Nama peta constraint |
| `--dari NAMA` | setup | – | Salin zona dari peta lain (untuk peta baru) |
| `--robot-panjang CM` | pantau | 40 | Panjang robot, searah maju (disimpan) |
| `--robot-lebar CM` | pantau | 30 | Lebar robot (disimpan) |
| `--geser CM` | pantau | 0 | Pusat robot di depan pusat tag; negatif = di belakang (disimpan) |
| `--margin CM` | pantau | 20 | Jarak PERINGATAN sebelum menyentuh zona (disimpan) |
| `--prediksi DETIK` | pantau | 0.5 | Lihat ke depan untuk prediksi pelanggaran; `0` = mati (disimpan) |
| `--udp HOST:PORT` | pantau | – | Kirim status JSON via UDP |
| `--log FILE` | pantau | – | Catat perubahan status ke CSV |
| `--simulasi` | pantau | – | Robot mengikuti mouse |
| `--tanpa-suara` | pantau | – | Matikan alarm suara |
| `--offset CM` | setup, pantau | 30 | Perluasan tampilan atas dari area homografi |
| `--family F` | pantau | `36h11` | Family AprilTag |
| `--url URL` | setup, pantau | dari `cctv_lurus.py` | Pakai URL RTSP lain |

Contoh:

```
python area_constraint.py setup --peta jalur_sempit --dari gudang_A
python area_constraint.py pantau --peta gudang_A --robot-panjang 60 --robot-lebar 40 --geser -10 --margin 30
python area_constraint.py pantau --peta gudang_A --udp 192.168.100.50:5005 --log pelanggaran.csv
```

---

## 11. File yang Dihasilkan

| File / Folder | Isi |
|---|---|
| `kalibrasi_lensa.json` | **Semua pengaturan tersimpan** (lihat tabel di bawah) |
| `kalibrasi_lensa_backup_manual.json` | Cadangan kalibrasi lensa manual sebelumnya |
| `Contoh Homografi/` | Gambar panduan langkah homografi |
| `AprilTag/` | Tag siap cetak (PDF & PNG) |
| `Hasil Foto/` | Screenshot / hasil uji |
| `capture_*.jpg`, `tag_*.jpg` | Screenshot dari mode `lihat` / `tag` |
| `peta_constraint/<nama>.json` | Zona satu peta constraint (`area_constraint.py setup`) |
| `robot_constraint.json` | Ukuran robot, geser, margin, prediksi (berlaku untuk semua peta) |
| `zona_*.jpg` | Screenshot dari `area_constraint.py pantau` |

Isi `kalibrasi_lensa.json`:

| Kunci | Diisi oleh | Arti |
|---|---|---|
| `model`, `f`, `k1`, `k2` | kalibrasi | Parameter lensa (`model` 1 = fisheye) |
| `zoom` | kalibrasi | Skala tampilan hasil koreksi |
| `roi`, `tampilan` | lihat | Kotak area & mode tampilan |
| `homografi` | homografi | Matriks `H`, daftar titik, ukuran ubin, parameter lensa saat dibuat |
| `tinggi_kamera_cm`, `tinggi_tag_cm` | tag | Tinggi hasil ukur |

> Simpan salinan file ini setelah kalibrasi berhasil. Jika kamera **digeser/diputar**, kalibrasi lensa masih berlaku, tetapi **homografi harus diulang**.

Contoh isi `peta_constraint/gudang_A.json`:

```json
{
  "nama": "gudang_A",
  "zona": [
    {"nama": "Z1", "tipe": "larangan", "titik": [[150, 60], [240, 60], [240, 150], [150, 150]]},
    {"nama": "Z2", "tipe": "batas",    "titik": [[0, 0], [270, 0], [270, 210], [0, 210]]}
  ],
  "diubah": "2026-09-17 15:28:52"
}
```

Koordinat `titik` dalam cm. File ini boleh diedit manual (misalnya untuk memasukkan koordinat yang tepat), lalu tekan `r` di mode `pantau` untuk memuat ulang.

---

## 12. Cara Kerja (ringkas)

```
frame kamera (terdistorsi)
   │  model lensa fisheye (f, k1, k2)       ← mode kalibrasi
   ▼
koordinat ternormalisasi (lurus)
   │  homografi H                           ← mode homografi
   ▼
titik di LANTAI (cm)
   │  koreksi tinggi (paralaks)             ← tinggi kamera & tinggi tag
   ▼
posisi tag sebenarnya (X, Y cm) + sudut
```

**Mengapa perlu koreksi tinggi?** Homografi hanya berlaku untuk titik di permukaan lantai. Tag di atas robot setinggi *h* terlihat bergeser menjauhi titik di bawah kamera sebesar kira-kira:

```
error ≈ d × h / H
d = jarak horizontal dari titik di bawah kamera,  h = tinggi tag,  H = tinggi kamera
```

Contoh: H = 270 cm, h = 25 cm, d = 150 cm → error ≈ **14 cm** bila tidak dikoreksi. Program mengoreksinya dengan:

```
P_sebenarnya = P_kamera_xy + (P_lantai − P_kamera_xy) × (H − h) / H
```

**Deteksi tag** dilakukan pada frame asli (tanpa interpolasi) hanya di area lantai, lalu 4 sudut tag diluruskan secara matematis → lebih presisi dan lebih cepat.

**Hasil simulasi** (kamera 280 cm, tag 50 cm, noise klik 0,7 px): error homografi 0,5 cm; posisi tag tanpa koreksi tinggi meleset 33–71 cm, dengan koreksi **0,1–0,8 cm**; sudut ≤ 0,1°.

**Kecepatan** (laptop Ryzen 7 260): deteksi ≈ 9 ms, pelurusan ≈ 1 ms per frame. Batas fps adalah kamera (**25 fps**). GPU tidak dipakai karena pada pipeline ini tidak mempercepat (OpenCL justru lebih lambat, dan deteksi AprilTag di OpenCV hanya berjalan di CPU).

**Pengecekan zona** (`area_constraint.py`) dilakukan pada koordinat lantai (cm):

```
posisi tag (X, Y) + sudut ──► badan robot (persegi panjang berputar, pusat digeser "geser")
                                   │
         ┌─────────────────────────┴─────────────────────────┐
         ▼                                                   ▼
  zona LARANGAN: melanggar jika                    zona BATAS: melanggar jika
  - sudut robot di dalam zona, atau                - sudut robot di luar/di garis zona, atau
  - sudut zona di dalam robot, atau                - sudut zona di dalam robot, atau
  - sisi robot memotong sisi zona                  - sisi robot memotong sisi zona
         │                                                   │
         └──────────── jarak bebas terkecil (cm) ────────────┘
                                   ▼
        < 0 → PELANGGARAN   |   < margin → PERINGATAN   |   lainnya → AMAN
```

Prediksi: kecepatan robot dihitung dari perubahan posisi antar-frame (dihaluskan). Badan robot pada posisi `sekarang + kecepatan × prediksi` juga dicek; jika posisi itu melanggar, status menjadi PERINGATAN "(prediksi)".

---

## 13. Pemecahan Masalah

| Masalah | Solusi |
|---|---|
| `Gagal terhubung ke kamera` | Cek IP, username/password, dan laptop satu jaringan dengan kamera. Coba URL di VLC. |
| AUTO kalibrasi tidak jalan | Butuh min. 3 garis terpisah (`n` di antara garis), tiap garis ≥ 3 titik, dan tidak berbelok. |
| Tepi gambar hitam setelah diluruskan | Normal. Atur slider `zoom` di mode kalibrasi. |
| Peringatan "kalibrasi lensa berubah sejak homografi dibuat" | Ulangi `python cctv_lurus.py homografi`. |
| Saat mengetik jumlah ubin tidak terjadi apa-apa | Klik jendela program dulu agar aktif, lalu ketik angka dan Enter. |
| Tag tidak terdeteksi | Cek family (`--family`), tag tidak mengkilap, tepi putih tidak terpotong, cahaya cukup, tag tidak tertutup teks OSD kamera. |
| Tag hilang saat robot bergerak cepat | *Motion blur*: perbesar tag, tambah cahaya, atau turunkan kecepatan robot. |
| Posisi meleset konsisten | Lihat tabel gejala di [Uji Akurasi](#8-uji-akurasi). |
| Dinding/benda tampak "melar" di tampilan atas | Normal — tampilan atas mengasumsikan semua objek berada di bidang datar. |
| Video terasa lambat | Lihat angka fps di jendela peta. Maksimum = 25 fps (batas kamera). Jika muncul "frame terlewat", tutup aplikasi lain yang berat. |
| `area_constraint.py`: "Belum ada peta constraint" | Buat peta dulu: `python area_constraint.py setup`. |
| `area_constraint.py`: "Peta ... tidak ditemukan" | Cek nama dengan `python area_constraint.py daftar`. |
| Zona tidak tertutup saat klik kanan | Zona butuh minimal 3 titik. |
| Robot PELANGGARAN padahal terlihat masih di luar zona | Cek ukuran robot (`--robot-panjang`, `--robot-lebar`) dan `--geser`; uji dengan `--simulasi`. Cek juga tinggi tag di `cctv_lurus.py`. |
| Status sering berkedip AMAN ↔ PERINGATAN | Normal di sekitar batas margin karena noise posisi. Perbesar margin atau abaikan PERINGATAN singkat di sisi robot. |
| Status HILANG terus | Tag tidak terdeteksi: lihat baris "Tag tidak terdeteksi" di atas; pastikan `--family` sama dengan tag yang dicetak. |
| Zona bergeser dari posisi seharusnya | Homografi diulang dengan titik asal (0,0) berbeda → gambar ulang zona. |
| Alarm tidak berbunyi | Tekan `m` (alarm mungkin OFF) atau periksa `--tanpa-suara`. |
