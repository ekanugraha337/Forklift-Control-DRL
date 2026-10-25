
Robot ini adalah robot forklift dengan satu steering wheel yang berada di belakang.

## Parameter fisik

Berikut adalah parameter fisik robot
- Jarak poros roda depan ke poros roda belakang : 22.5 cm
- jarak terluar antara dua roda depan : 13 cm
- lebar robot : 17.5 cm
- tinggi robot : 16.5 cm
- lebar roda : 2.5 cm
- dimensi April Tag : 10 x 10 cm
- jarak dari tengah April Tag ke robot bagian depan : 9.5 cm
- jarak poros roda depan ke ujung depan robot : 4 cm
- jarak poros roda depan ke ujung belakang robot : 23 cm
- tinggi AprilTag dari lantai : 16.5 cm

Catatan turunan (dihitung dari angka di atas, jangan diukur ulang):
- panjang total robot : 27 cm (4 + 23)
- jarak pusat AprilTag ke poros roda depan : 5.5 cm (9.5 - 4), poros di DEPAN tag
- jarak pusat badan ke pusat AprilTag : -4 cm (pusat badan di BELAKANG pusat tag)
- badan menjulur 0.5 cm di belakang poros roda belakang (23 - 22.5)

Titik acuan pose yang dipakai perencanaan dan pelacakan adalah **poros roda
depan**, bukan pusat AprilTag. Roda depan tidak berbelok, sehingga robot selalu
bergerak searah heading di titik itu - itulah yang membuat model sepeda berlaku
untuk kendaraan berkemudi belakang.

Tinggi AprilTag wajib diisi ke kalibrasi kamera, kalau tidak setiap posisi cm
meleset makin jauh dari titik tepat di bawah kamera :
	python cctv_lurus.py tag --tinggi-tag 16.5

---

# Ruang keadaan

Kode: `state_space.py`. Nilai parameternya diambil dari `konfig.py`, jadi angka
di bawah ini ikut berubah kalau konfigurasi berubah. Cetak ulang dengan
`python state_space.py`.

Ada **dua bentuk**, untuk dua keperluan yang berbeda. Yang tak linear untuk
simulasi dan ramalan pose; yang linear untuk perancangan kendali. Pure pursuit
yang dipakai sekarang **tidak** memakai keduanya - ia geometris, bukan berbasis
model. Bentuk linear disiapkan untuk LQR/MPC lewat seam di `jejak.py`.

## Dua kesepakatan yang menentukan bentuk persamaan

**Acuannya poros roda DEPAN.** Robot ini berkemudi belakang, dan roda depan
tidak berbelok - jadi kecepatan di poros depan selalu searah heading badan.
Itulah yang membuat model sepeda baku berlaku tanpa suku tambahan. Kalau
acuannya dipindah ke poros belakang (roda kemudi), persamaannya mendapat suku
menyamping dan tidak lagi sederhana.

**δ adalah sudut kemudi LOGIS**, memakai konvensi kendaraan berkemudi depan:
δ > 0 berarti robot berbelok ke kiri (yaw bertambah). Pembalikan untuk roda
belakang yang sebenarnya dikerjakan **firmware** lewat setelan `str_inv`, jadi
model ini tidak perlu mengurusnya - dan itu memang harus begitu, supaya model
cocok dengan apa yang benar-benar dikirim `robot.py`.

## 1. Model lengkap (tak linear)

Untuk simulasi, ramalan pose saat tag hilang, dan pemeriksaan kelayakan manuver
di `identifikasi.py`.

```
keadaan  ξ = [x, y, θ, δ, v]ᵀ        masukan  u = [δ_cmd, v_cmd]ᵀ

ẋ = v cos θ
ẏ = v sin θ
θ̇ = v tan δ / L
δ̇ = (sat(δ_cmd) − δ) / τ
v̇ = sat(v_cmd − v, ±a)
```

Satuan: `x, y` cm | `θ, δ, δ_cmd` radian | `v, v_cmd` cm/detik.

Titik `(x, y)` adalah poros roda depan. Batasan yang ikut ditegakkan di dalam
persamaan, bukan ditambahkan di luar:

| batasan | nilai | asal |
|---|---|---|
| \|δ\| ≤ δ_max | 35.0° = 0.6109 rad | `MAX_STEER_DEG` |
| \|v̇\| ≤ a | 29.9 cm/detik² | `ACCEL_SPS2` × `CM_PER_STEP` |
| v maju | ≤ +17.4 cm/detik | `FORWARD_SPEED` 7000 sps |
| v mundur | ≥ −10.0 cm/detik | `REVERSE_SPEED` 4000 sps |

Dua suku terakhir (δ̇ dan v̇) adalah **aktuator, bukan kendaraan**: yang pertama
jeda servo kemudi, yang kedua ramp akselerasi stepper. Keduanya ada di model
karena keduanya cukup lambat untuk mengubah hasil - keterlambatan kemudi efektif
~0.35 detik (terukur 2026-09-27, bagian 3) pada 17 cm/detik berarti ~6 cm
perjalanan sebelum perintah kemudi terlihat bekerja.

Tunda murni pada kemudi **tidak** ada di persamaan ini; ia bukan keadaan
berdimensi hingga. Yang mengurusnya adalah bentuk diskret di bagian 3.

### Radius yang keluar dari model ini

| besaran | nilai | keterangan |
|---|---|---|
| R_min = L / tan δ_max | 32.13 cm | kemudi mentok |
| R_plan (75% kemudi) | 45.63 cm | sisanya cadangan koreksi pelacak |
| R_sapu (sudut badan terluar) | 54.52 cm | → butuh lingkaran bebas ~109 cm |
| κ_max = 1/R_min | 0.0311 cm⁻¹ | kelengkungan terbesar yang bisa diikuti |
| κ_plan = 1/R_plan | 0.0219 cm⁻¹ | batas yang dipakai Hybrid A* |

## 2. Model galat jalur (linear)

Untuk perancangan kendali. Keadaannya bukan posisi absolut, melainkan
**simpangan dari jalur acuan** - yang memang jadi urusan pelacak.

```
x = [e_y, e_θ, δ]ᵀ        u = δ_cmd        w = κ (kelengkungan jalur acuan)

ė_y = v sin e_θ           ≈ v · e_θ
ė_θ = v tan δ / L − κ v   ≈ (v/L) δ − v κ
δ̇   = (δ_cmd − δ) / τ
```

Satuan: `e_y` cm | `e_θ, δ, δ_cmd` rad | `κ` cm⁻¹ | v cm/detik.

Dilinearkan pada kecepatan **tetap** v, jadi setiap kecepatan kerja punya
matriksnya sendiri:

```
      ⎡0   v   0  ⎤        ⎡ 0 ⎤        ⎡ 0 ⎤        ⎡1  0  0⎤
A  =  ⎢0   0  v/L ⎥   B =  ⎢ 0 ⎥   E =  ⎢−v ⎥   C =  ⎣0  1  0⎦
      ⎣0   0 −1/τ ⎦        ⎣1/τ⎦        ⎣ 0 ⎦
```

`C` adalah apa yang benar-benar terukur kamera: simpangan lateral dan galat
heading. Baris ketiga tidak ada karena **δ tidak terukur di mana pun** - robot
ini tidak punya sensor sudut kemudi.

Pada v = 17.4 cm/detik (`FORWARD_SPEED` 7000 sps), L = 22.5 cm, τ = 0.10 s:

```
      ⎡0  17.437   0    ⎤        ⎡ 0 ⎤        ⎡   0    ⎤
A  =  ⎢0   0       0.775⎥   B =  ⎢ 0 ⎥   E =  ⎢−17.437 ⎥
      ⎣0   0     −10    ⎦        ⎣10 ⎦        ⎣   0    ⎦
```

(Sebelum 2026-09-27, dengan τ = 0.72 s: A₃₃ = −1.3889, B₃ = 1.3889. τ itu
ternyata salah - bagian 3.)

## 3. Bentuk diskret + rantai tunda

Diskretisasi **tepat** (zero-order hold) pada dt = 0.05 s (20 Hz, sama dengan
periode loop kendali):

```
        ⎡1  0.8719  0.0144⎤         ⎡0.0025⎤
Ad  =   ⎢0  1       0.0305⎥    Bd = ⎢0.0083⎥
        ⎣0  0       0.6065⎦         ⎣0.3935⎦
```

Dengan τ = 0.10 s, kemudi menempuh 39% perjalanannya ke sudut perintah dalam
satu cacah (Bd₃ = 0.3935); dengan τ lama 0.72 s hanya 7%.

Tunda murni kemudi diwakili **rantai penggeser**, bukan pendekatan Padé - Padé
menambah kutub palsu yang tidak ada di robotnya. Keadaan bertambah menjadi
7 buah:

```
x_aug = [e_y, e_θ, δ, u_{k−4}, u_{k−3}, u_{k−2}, u_{k−1}]ᵀ
```

Masukan baru masuk di ujung rantai; yang menggerakkan kemudi adalah tap
tertua. Cara ini **tepat** untuk tunda yang kelipatan dt. Di sini
`SIM_KEMUDI_TUNDA_S` = 0.19 s dibulatkan menjadi 4 cacah = 0.20 s, jadi
0.01 detik tunda ditambahkan oleh pembulatan - kecil, dan jauh di bawah
ayunan jalur perintah (0.05-0.34 s).

### Sifat sistem

| pemeriksaan | hasil |
|---|---|
| keterkendalian tanpa tunda | rank 3/3 → terkendali |
| keterkendalian dengan tunda | rank 7/7 → terkendali |
| kutub diskret (tanpa tunda) | 1.0, 1.0, 0.6065 |

Dua kutub di z = 1 adalah **integrator ganda** posisi ← heading: galat heading
menumpuk menjadi galat posisi. **Sistem ini tidak stabil sendiri** - tanpa
kendali, galat kecil tumbuh linear lalu kuadratik. Kutub ketiga 0.6065 =
e^(−dt/τ) adalah jeda servo kemudi, satu-satunya yang punya redaman sendiri
(semula 0.9329 dengan τ 0.72 s).

> **Nilai kemudi diukur ulang dan diterapkan (2026-09-27).** Sebelumnya tunda
> 0.22 s + τ 0.72 s, disetel agar korelasi silang perintah-kemudi →
> kelengkungan memuncak di 0.65 s, angka v1 yang diukur **lewat kamera** -
> kamera yang ternyata terlambat ~0.68 s (bagian 4). Setelah kamera
> diperbaiki, dua uji tangga (4000 dan 3000 sps), dalam kerangka waktu kamera:
>
> | | uji 1 | uji 2 |
> |---|---|---|
> | korelasi silang (v1: 0.65 s) | 0.42 s | 0.30 s |
> | tunda d / τ | 0.41 / ~0 s | 0.12 / 0.12 s |
> | model bagian 2-3 (0.22 + 0.72), sisa yaw | 2.13° | 2.06° |
> | **tunda 0.25 + τ 0.10**, sisa yaw | **0.59°** | **0.57°** |
>
> Rincian d dan τ berbeda antar-uji (selisih ~0.17 s, sebesar variasi jalur
> perintah di uji gas), tapi keterlambatan efektif **d + τ ≈ 0.35 s** dan
> **τ kecil** konsisten. Tunda model di bagian 2-3 kurang lebih benar;
> **τ = 0.72 s-nya yang salah besar**. Pencocoknya terbukti bisa melihat τ bila
> ada (di simulator ia membaca 0.714 untuk τ tertanam 0.72).
>
> **Diterapkan:** `SIM_KEMUDI_TUNDA_S` 0.22 → **0.19**, `SIM_KEMUDI_TAU_S`
> 0.72 → **0.10** (setara simulator untuk 0.25 + 0.10 kerangka kamera, karena
> simulator memodelkan kamera terpisah ~0.06 s). Matriks di bagian 2-3 sudah
> memakai nilai ini. Uji rute tertutup di simulator, rute yang sama: 9/10
> tercapai pada kedua model, simpangan rata 1.97 → 1.45 cm, simpangan maks
> 11.87 → 6.45 cm - tanpa kemunduran.
>
> Matriks bagian 2-3 kini layak jadi **titik awal** perancangan LQR/MPC, dengan
> dua catatan: (1) tunda **pengukuran** ~0.09 s (kamera) tidak ada di model ini;
> pengendali yang memakai pose kamera sebaiknya memajukan pose itu dulu
> (prediktor keadaan), dan (2) jalur perintah berayun 0.05-0.34 s, sehingga
> tunda sebenarnya bisa ~0.1-0.2 s lebih panjang atau lebih pendek dari model.
> Pengendali harus tahan terhadap ayunan itu.

## 4. Sambungan ke perangkat

Model ini tidak berdiri sendiri; ia bertemu robot di dua tempat.

**Masukan → firmware** (`robot.py`):

```
speed_sps = v_cmd / CM_PER_STEP            CM_PER_STEP = 0.002491 cm/langkah
servo_deg = 89.45 + δ_cmd[deg]             dibatasi 10..170 (str_min/str_max)
```

MQTT `forklift01/in/rc` = `"<langkah/detik bertanda> <sudut servo>"`, dikirim
20x per detik. Firmware menghentikan motor bila aliran ini putus > 0.5 detik.

**Keluaran ← kamera** (`pose.py`). Yang dilihat kamera adalah pusat tag, bukan
poros depan:

```
x_tag = x − 5.5 cos θ        y_tag = y − 5.5 sin θ        (TAG_TO_FRONT_AXLE_CM)
```

**Jeda pengukurannya ~0.68 detik**, bukan 30-85 ms seperti yang dianggap
simulator (`SIM_KAMERA_DELAY_S` = 0.025 adalah latensi *pemrosesan*; latensi
aliran RTSP tidak pernah diukur sampai 2026-09-27). Cara ukurnya: posisi kamera
digeser terhadap hitungan langkah firmware (`pos` di telemetri) sampai
berimpit. Hasilnya 0.725 / 0.685 / 0.675 s pada tiga lintasan, sisa 0.08-0.17 cm,
dan cm/langkah cocok dalam 0.3%. Itu pergeseran waktu murni, dan angkanya batas
bawah (latensi telemetri kembali ke PC belum ditambahkan). Pada 17.4 cm/detik,
setiap pose yang dipakai navigasi tertinggal ~12 cm dari posisi robot
sebenarnya.

**Penyebabnya multithreading dekoder, dan sudah diperbaiki (2026-09-27).**
OpenCV membuka dekoder FFmpeg dengan 16 thread (= inti CPU), dan dekode
multithread per-frame menahan ~15 frame ≈ 0.6 s. `cctv_lurus.StreamReader`
sekarang memakai 1 thread. Frame yang sama keluar 589-611 ms lebih dulu
(`uji_kamera.py`, dicocokkan lewat sidik frame antar-sesi). **Dipastikan di
robot**: latensi kamera **0.085 / 0.095 / 0.095 s** (uji gas,
`logs_identifikasi/20260927_094202`), dan pose kini tertinggal ~1.6 cm pada
17.4 cm/detik. Pada jalan yang sama, akselerasi naik terukur 11372 ± 493
sps/s² (0.95× setelan, garis antar-target lurus), tanpa langkah hilang. Semua
data robot lain di dokumen ini (cusp.csv, uji gas sebelumnya) direkam
**sebelum** perbaikan.

Yang penting: **δ dan v tidak pernah terukur**. Field `pos` dan `sps` di
telemetri firmware adalah langkah yang *dikeluarkan*, bukan pengukuran - robot
ini tidak punya encoder. Pada uji gas 2026-09-27, `pos` × `CM_PER_STEP` cocok
dengan kamera dalam 0.3% (tidak ada langkah hilang), tapi itu pengamatan satu
sesi, bukan jaminan.

Karena itu, saat tag hilang, model tak linear di bagian 1 dipakai untuk
**meramalkan** pose dari perintah yang sedang dikirim. Dua keadaan yang tidak
terukur diisi dengan cara berbeda:

- **δ** dimulai dari sudut yang terakhir diperintahkan; jeda kemudi diurus
  persamaan δ̇ sesudahnya.
- **v** dimulai dari **taksiran ramp** yang dipelihara `SumberKamera.perintah()`
  tiap siklus kendali - mengintegrasikan `ACCEL_SPS2` dari aliran perintah,
  dengan urutan zero-order hold (selama selang yang lewat, motor mengejar
  perintah sebelumnya). Sebelum 2026-09-27 v disamakan ke perintah seketika;
  bagian 6b menjelaskan kenapa itu salah.

Sah hanya sebentar (`RAMAL_MAKS_S` = 0.4 s); setelah itu pose ditandai tidak
sah dan robot di-estop.

## 5. Parameter dan asalnya

| parameter | nilai | asal |
|---|---|---|
| L (wheelbase) | 22.5 cm | diukur |
| CM_PER_STEP | 0.002491 cm/langkah | diukur 2026-09-27 dari jalan nyata, 337 cuplikan tunak, ±0.000009 |
| δ_max | 35.0° | dari episode kemudi mentok di jalan nyata (tercapai 35.0°, memuncak 36.0°) |
| τ (tetapan waktu kemudi) | **0.10 s** | dua uji tangga 2026-09-27 setelah perbaikan kamera (bagian 3). Semula 0.72 s, disetel ke 0.65 s v1 yang diukur lewat kamera lambat |
| tunda murni kemudi | **0.19 s** | idem; setara simulator untuk 0.25 s kerangka kamera. Semula 0.22 s |
| latensi kamera | **~0.68 s** | diukur 2026-09-27 terhadap hitungan langkah firmware; simulator masih memakai 0.025 s |
| dt | 0.05 s | periode loop kendali, 20 Hz |
| a (akselerasi) | 12000 sps/s² = 29.9 cm/s² | **setelan** NVS, dipastikan `show config` 2026-09-27 |

> **Setelan `accel` sudah pasti: NVS = 12000**, sama dengan `konfig.ACCEL_SPS2`
> (dulu bernama `SIM_ACCEL_SPS2`). Sebelum dipastikan ada tiga angka yang
> berbeda: konfig 12000, README 8000, bawaan firmware `DEF_DRIVE_ACCEL` 4000
> (`include/defaults.h:68`). Yang terakhir itu tetap perlu diingat - NVS yang
> terhapus setelah flash ulang diam-diam kembali ke 4000.
>
> Kenapa harus sama persis: sejak 2026-09-27 nilai ini tidak lagi hanya dipakai
> simulator. `pose.py` memakainya di perangkat nyata untuk melacak kecepatan
> motor saat meramal pose (bagian 6b).
>
> **"~2000" di `cusp.csv` ternyata BUKAN akselerasi.** Sapuan 500-24000
> terhadap `cusp.csv` sempat menunjukkan kelompok transien paling cocok dengan
> ~2000. Uji tangga gas dengan telemetri firmware (2026-09-27,
> `logs_identifikasi/20260927_091649`) memilah keterlambatan ~1 detik yang
> terlihat di kamera:
>
> | tahap | terukur |
> |---|---|
> | perintah dikirim → stepper mulai | 0.06-0.34 s (biasanya ≤ 0.15) |
> | stepper → terlihat kamera | **0.675-0.725 s** |
>
> Stepper mengikuti ramp 12000 dengan tepat, dan tidak ada langkah hilang
> (cm/langkah cocok dalam 0.3%). Jadi akselerasinya sesuai setelan, dan yang
> terlambat adalah **kamera** (bagian 4).
>
> **Koreksi atas catatan sebelumnya.** Catatan versi sebelumnya menulis
> "tundanya tidak tetap" dan "kamera tidak lambat", berdasarkan separuh start
> di `cusp.csv` yang tampak bergerak dalam 0.06-0.12 s. Keenamnya ternyata
> start setelah jeda titik ganti arah 0.62 s, dan pada saat itu kamera **masih
> menayangkan gerak lama** (1.7-5.3 cm dalam 0.6 s sebelum perintah). Start
> dari diam lama selalu terlambat 1.1-1.4 s.
>
> Jalur perintah sisanya kecil tapi pernah tersendat: pada lintasan pertama,
> 13 pesan telemetri menumpuk lalu tiba sekaligus, dan perintahnya terlambat
> ~0.34 s. Wi-Fi power save bukan penyebabnya (`WiFi.setSleep(false)` di
> firmware).

`identifikasi.py` mengukur ulang sebagian parameter ini **di robot** lewat empat
percobaan (lurus, busur, tangga, gas). Diuji dengan memulihkan parameter yang
ditanam di simulator: skala langkah 0.002600 → terukur 0.002599, penguatan
kemudi 1.000 → 0.993, sudut maksimum 35.0° → 34.8°, jeda kemudi 0.65 s → 0.70 s.

Uji gas dirancang memisahkan **tunda** dari **akselerasi**, dua hal yang
sama-sama membuat robot "terlambat". Di robot nyata pemisahan antar-target
**tidak berlaku**, karena ia mengandaikan tunda tetap sementara tunda robot ini
berubah-ubah. Yang dipakai adalah awal gerak tanpa model (lihat di atas).
Rancangannya: dalam satu lintasan keduanya tidak terpisahkan (korelasi
+0.98: ramp 12000 hanya 4-12 frame kamera). Yang terukur baik per lintasan
adalah keterlambatan total `L = tunda + target / 2A`. Pada empat target berbeda,
L membentuk garis dengan intersep = tunda dan kemiringan = 1/2A, dan
kelurusan garis itu menguji apakah akselerasi tetap. Pemulihan di simulator:
tunda tertanam +0.15 s terbaca **+0.147 s dengan A tidak berubah** (11889 vs
11886), A tertanam 4000 terbaca **4002 dengan tunda tidak berubah**, dan
akselerasi yang melemah dengan laju **ditandai tidak linear**.

## 6. Seberapa jauh modelnya boleh dipercaya

Ada **dua pertanyaan berbeda** di sini, dan dua ujian berbeda menjawabnya.
Pertama: apakah model ini cocok dengan ROBOT? Kedua: apakah bentuk linearnya
cocok dengan bentuk tak linearnya? Jangan tertukar - yang kedua bisa sempurna
sementara yang pertama gagal.

### 6a. Terhadap robot nyata (pose kamera)

> **Baca bagian 6a-6b dengan catatan ini (2026-09-27).** Semua angka di bawah
> dihitung dengan anggapan pose kamera adalah posisi robot *saat itu*. Kamera
> ternyata terlambat ~0.68 s (bagian 4), jadi galat "transien" yang besar di
> bawah sebagian besar adalah salah penjajaran waktu, bukan kelemahan model.
> Angka yang sudah dikoreksi ada di akhir 6b, dan validasi pada jalan baru
> setelah kamera diperbaiki ada di 6c.

Diuji 2026-09-27 terhadap `cusp.csv`: 2619 baris, 162 detik jalan nyata dalam
dua segmen sinambung, pose AprilTag sebagai acuan. Caranya meniru cara `pose.py`
memakai model di perangkat - mulai dari pose kamera + perintah yang sedang
dikirim, integrasikan model tak linear dengan perintah yang benar-benar terekam,
bandingkan ke pose kamera beberapa saat kemudian.

Tabel 6a dan 6b di bawah adalah **diagnosis sebelum perbaikan** - itulah yang
mengungkap cacatnya. Dihasilkan ulang dengan:

```bash
python validasi_nyata.py cusp.csv --v0 perintah     # pose.py SEBELUM perbaikan
python validasi_nyata.py cusp.csv                   # pose.py SEKARANG (lihat 6b)
```

Hasilnya terbelah tajam, dan pembelahnya **bukan laju** melainkan apakah
perintah gas baru berubah. Horizon 0.4 detik (= `RAMAL_MAKS_S`), galat posisi:

| laju | perintah sudah tunak | perintah baru berubah (< 1.5 s) |
|---|---|---|
| 0-5 cm/detik | median 0.13 · p95 **0.34** cm | median 0.28 · p95 3.69 cm |
| 5-10 cm/detik | median 0.16 · p95 **0.32** cm | median 0.33 · p95 4.34 cm |
| 10-15 cm/detik | (tidak ada cuplikan) | median 3.74 · p95 6.17 cm |
| 15-30 cm/detik | median 0.63 · p95 **1.22** cm | median 2.66 · p95 8.14 cm |

**Dalam keadaan tunak modelnya teliti di bawah 1 cm sampai laju penuh.**
Dibandingkan pose beku (tidak diramalkan sama sekali), model lebih baik pada
89-96% cuplikan di semua horizon 0.2-2.0 detik.

Lantai galatnya: saat robot **diam**, pose kamera sendiri bergeser p95 0.36 cm
dan yaw p95 0.69° antar-frame. Ramalan tidak mungkin lebih teliti dari itu.

**Batas ujian ini - empat, dan semuanya perlu disebut kalau angkanya dikutip:**

1. **Galat geometri saling menghapus.** Acuannya pose kamera lewat konversi yang
   SAMA dengan yang dipakai model (`ke_poros_depan`, `tinggi_tag_cm`), jadi galat
   sistematis di `TAG_TO_FRONT_AXLE_CM` atau tinggi tag tidak akan tertangkap.
   Yang teruji ramalan **inkremental**, bukan ketelitian posisi absolut.
2. **Sebagian acuan mungkin bukan pengukuran.** `pose.py` menyetel `terlihat`
   True **juga untuk pose ramalan** - ia baru False setelah ramalan melewati
   `RAMAL_MAKS_S`. `cusp.csv` direkam tanpa kolom `diramalkan`, jadi sebagian
   barisnya bisa jadi keluaran model itu sendiri, dan angka di atas **condong
   terlalu baik**. Sejak 2026-09-27 `navigasi.py --log` mencatat kolom itu dan
   `validasi_nyata.py` membuang pose ramalan dari acuan - batas ini hilang untuk
   log baru, **tidak** untuk `cusp.csv`.
3. **Laju tinggi tipis.** Hanya 49 cuplikan tunak di 15-30 cm/detik, dan laju
   rata data ini cuma 6.8 cm/detik - padahal laju kerja 17.4 cm/detik.
4. **τ tidak ikut teruji.** δ tidak terukur di mana pun, jadi τ = 0.72 s hanya
   teruji lewat akibatnya pada lintasan. Dan ini satu sesi, satu lantai, satu
   tanggal - bukan pengulangan. (Sejak 2026-09-27 τ diukur langsung lewat dua
   uji tangga: ~0.10 s, bagian 3.)

### 6b. Ekor galatnya dari keadaan awal, bukan dari persamaan

Seluruh galat besar terkumpul di cuplikan yang perintah gasnya baru berubah:

| perubahan perintah gas dalam 1.5 detik sebelumnya | median | p95 | maks |
|---|---|---|---|
| < 200 sps | 0.16 cm | 0.54 cm | 1.42 cm |
| 200-1000 sps | 0.24 cm | 0.56 cm | 1.88 cm |
| 1000-3000 sps | 1.18 cm | 3.84 cm | 4.77 cm |
| > 3000 sps | 3.74 cm | 7.20 cm | 9.41 cm |

Sebabnya ada di `pose.py._mulai_ramal()`, bukan di persamaannya: `v` disetel ke
kecepatan yang **diperintahkan**, seketika. Model punya suku
`v̇ = sat(v_cmd − v, ±a)` yang sanggup mewakili ramp, tapi inisialisasi itu
membuangnya - saat motor masih menanjak, model mengira robot sudah cepat lalu
meramalkan terlalu jauh. Galat terburuknya **9.41 cm** di dalam jendela 0.4
detik yang dianggap aman - melebihi `SAFETY_MARGIN_CM` = 8.0.

**DIPERBAIKI 2026-09-27.** `SumberKamera.perintah()` sekarang melacak kecepatan
yang benar-benar dicapai motor dengan mengintegrasikan ramp `ACCEL_SPS2` dari
aliran perintah (tanpa sensor baru - robot ini memang tidak punya encoder), dan
`_mulai_ramal()` memakai taksiran itu. Diuji terhadap `cusp.csv` yang sama,
horizon 0.4 detik:

| | p95 | maks | > `SAFETY_MARGIN_CM` |
|---|---|---|---|
| sebelum (v = perintah) | 4.23 cm | 9.41 cm | **5 cuplikan** |
| sesudah (v = ramp) | 3.76 cm | 6.60 cm | **0** |

Kelompok tunak **tidak berubah** (p95 0.54 cm) - perbaikannya hanya menyentuh
kasus yang memang rusak. Kode di robot dan kode di `validasi_nyata.py` dibuktikan
**identik**: `SumberKamera.perintah()` yang asli disuapi 2619 baris `cusp.csv`,
selisih taksirannya 0.000 cm/detik di setiap baris.

**Yang belum selesai, jangan dibaca sebagai "sudah beres":**

- Kelompok transien masih jauh lebih buruk dari tunak (p95 5.34 vs 0.54 cm
  pada perubahan perintah > 3000 sps). Perbaikan ini menghapus pelanggaran
  margin, **tidak** menutup selisihnya. Sisa selisih itu sebagian besar
  **salah penjajaran waktu akibat latensi kamera ~0.68 s** (bagian 4): validasi
  ini menganggap pose kamera adalah posisi robot saat itu. Diulang dengan pose
  dicap pada waktu tangkap yang benar (t − 0.68 s), p95 turun 3.76 → **1.47 cm**,
  maks 6.60 → **4.52 cm**, p95 transien 4.31 → **2.01 cm** (dengan tambahan
  tunda perintah 0.1 s: 1.37 / 4.10 / 1.93 cm). Modelnya jauh lebih teliti dari
  angka 6a; koreksi yang dibutuhkan `pose.py` adalah memajukan SETIAP pose
  sebesar latensi kamera, bukan hanya pose ramalan.
- Pada horizon 1.0 detik galat terburuk masih 13.49 cm. Itu tidak dipakai di
  perangkat - ramalan dihentikan di `RAMAL_MAKS_S` = 0.4 detik lalu robot
  di-estop - tapi jangan menaikkan `RAMAL_MAKS_S` berdasarkan perbaikan ini.
- Setelah estop, robot meluncur bebas (arus motor diputus), bukan melambat
  mengikuti ramp. Pelacaknya tidak tahu soal estop. Pada setelan sekarang
  keduanya nyaris sama (30 vs 29.9 cm/detik²) - kebetulan, bukan rancangan.
- Batas nomor 2 di 6a (kolom `diramalkan` tidak tercatat di `cusp.csv`) tetap
  berlaku untuk angka sesudah maupun sebelum. Uji yang bebas dari batas itu
  butuh rekaman baru dengan log format 11 kolom.

### 6c. Setelah perbaikan kamera - validasi yang bersih

Direkam 2026-09-27 setelah latensi kamera turun ke ~0.09 s
(`jalan_setelah_kamera.csv`, peta Rumah, 161 detik, 4 rute). Ini validasi
pertama yang bebas dari dua batas di 6a: kolom `diramalkan` tercatat (dan tidak
ada satu pun pose ramalan), dan kamera tidak lagi menggeser waktu sebesar
0.68 s.

| horizon | model: median / p95 / maks | pose beku p95 | model lebih baik |
|---|---|---|---|
| 0.2 s | 0.14 / 0.37 / 1.23 cm | 2.13 cm | 98% |
| 0.4 s | 0.15 / **0.41** / 2.42 cm | 4.25 cm | 100% |
| 1.0 s | 0.23 / 1.00 / 3.31 cm | 11.15 cm | 100% |

Pada horizon 0.4 s, kelompok tunak dan transien kini **setara** (p95 0.30 vs
0.49 cm), dibanding 0.54 vs 4.31 cm di `cusp.csv`. Tidak ada cuplikan yang
mendekati `SAFETY_MARGIN_CM`.

Batas yang tersisa: (1) galat geometri yang saling menghapus tetap tidak
tertangkap (batas 1 di 6a); (2) laju di rute ini di bawah 15 cm/detik, jadi
laju penuh 17.4 cm/detik belum teruji; (3) rute ini tidak peka terhadap model
kemudi. Tabel di atas dihitung dengan τ lama 0.72 s; diulang dengan τ baru
0.10 s hasilnya praktis sama (horizon 0.4 s: p95 0.41 cm; 1.0 s: p95 1.02 vs
1.00 cm). Pada ~5 cm/detik dan kemudi pure pursuit yang halus, beda kedua
model kemudi hanya ~0.02 cm dalam 0.4 s. Model kemudi diuji dengan uji tangga
(bagian 3), bukan dengan rute ini.

### 6d. Terhadap simulator (menguji linearisasinya, bukan modelnya)

`python state_space.py` membandingkan bentuk linear dengan bentuk tak linear -
dua-duanya model, jadi ujian ini **tidak** menjawab apakah modelnya cocok dengan
robot. Jalur lurus (κ = 0), perintah kemudi acak yang sama, 6 detik, v = 17.4
cm/detik:

| amplitudo kemudi | galat RMS | % dari lingkup simpangan |
|---|---|---|
| ±4° | 0.01 cm | 0.3% |
| ±10° | 0.04 cm | 0.6% |
| ±20° | 0.24 cm | 1.6% |
| ±30° | 0.76 cm | 3.2% |

Pada 31 cm/detik: 0.03-0.97 cm (0.3-1.3%). Jalankan
`python state_space.py --laju <cm/detik>` untuk kecepatan lain.

Galat pada kemudi besar **naik** sejak τ diturunkan 0.72 → 0.10 s (semula
0.05 cm / 0.3% pada ±30°). Itu bukan model yang memburuk: kemudi yang lamban
dulu tidak pernah benar-benar mencapai sudut besar dalam 1 detik, jadi
selisih tan δ vs δ hampir tidak pernah muncul. Kemudi yang cepat mencapainya,
dan linearisasinya harus menanggung selisih itu.

**Batas kesahihan.** Linearisasi mengganti `tan δ` dengan `δ`. Pada kemudi penuh
35°, `tan δ` = 0.700 sedangkan `δ` = 0.611 rad - meleset 15%, artinya model
linear memperkirakan robot berbelok **lebih lambat** daripada yang sebenarnya
terjadi. Untuk manuver rapat, pakai bentuk tak linear atau linearkan ulang di
sekitar δ kerja. Galat RMS di tabel atas tetap kecil pada ±30° karena galatnya
sebagian saling menghapus pada kemudi bolak-balik; pada belokan **satu arah yang
dipertahankan** ia menumpuk.

**Yang sengaja tidak ada di model:**

- **Batas laju servo** 500 °/detik - ada di simulator, tidak di model linear.
  Terkonfirmasi dari sumber firmware v2: `STEER_STEP_DEG` = 1.0 setiap
  `STEER_STEP_MS` = 2 ms (`src/steering.cpp:22`). Pada lookahead 26 cm gerak
  servo ~4 °/frame = 82 °/detik, jadi batas ini jarang mengikat.
- **Selip - dicari, TIDAK ditemukan.** `konfig.py` mencurigainya: CM_PER_STEP
  terukur turun 12% pada 6000-9000 sps (0.002204 vs 0.002505 pada 1000-3000),
  n = 15. Diuji ulang terhadap `cusp.csv` 2026-09-27 dan **tidak terkonfirmasi**:
  korelasi sps vs cm/langkah hanya r = −0.245, dan laju tinggi yang TUNAK justru
  bersih (p95 1.22 cm di 15-30 cm/detik). Yang tadinya terlihat seperti efek laju
  sebenarnya efek ramp, karena di data itu laju tinggi hampir selalu bersamaan
  dengan perintah yang baru berubah. Belum terbukti - tapi juga belum tersanggah,
  karena jendela tunak di atas 4000 sps hanya ada 3.
- **Luncuran bebas setelah estop.** estop firmware memutus arus motor
  (`forceStop()` lalu `disableOutputs()`), jadi robot meluncur tanpa torsi
  penahan. Simulator memodelkannya dengan `SIM_LUNCUR_DESEL_CM_S2` = 30 cm/s²
  yang **belum diukur** (perkiraan dari torsi detent NEMA17, rentang 20-60).
- **Dinamika samping.** Model ini kinematik: roda dianggap menggelinding tanpa
  selip samping. Pada 17-31 cm/detik dengan R ≥ 32 cm, percepatan sentripetal
  ≤ 30 cm/s² - jauh di bawah batas gesekan, jadi asumsinya aman.

## Berkas terkait

- `state_space.py` - kedua model + uji linearisasi (`python state_space.py`)
- `validasi_nyata.py` - nilai model terhadap jalan nyata; menghasilkan seluruh
  tabel di bagian 6a dan 6b (`python validasi_nyata.py cusp.csv`)
- `konfig.py` - semua parameter, satu sumber kebenaran (`python konfig.py`)
- `pose.py` - `SumberSimulasi` (integrator model tak linear) + ramalan pose
- `identifikasi.py` - mengukur parameter model di robot (`--mode gas` untuk
  akselerasi motor yang sebenarnya)
- `uji_gas.py` - uji-mandiri uji gas: parameter ditanam di simulator, diperiksa
  pulihnya (`python uji_gas.py`)
- `robot.py` - pemetaan masukan model → MQTT/servo
