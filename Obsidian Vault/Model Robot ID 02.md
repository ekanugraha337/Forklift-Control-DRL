
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

Geometri di atas **sama persis** dengan [[Model Robot ID 01]]. Tabel `ROBOT` di
`konfig.py` menyimpan lima angka per robot: ID AprilTag, titik lurus kemudi,
skala langkah, akselerasi, dan sudut kemudi maksimum. Setelah diukur, **yang
benar-benar berbeda ada tiga**:

- ID tag: 2
- titik lurus kemudi: 96.15°
- **sudut kemudi maksimum: 23.5°**, karena ke kanan kemudi mentok jauh lebih
  awal dari ID 01 (bagian 5d)

Skala langkah, akselerasi, penguatan kemudi, dan dinamika kemudi sama dengan
ID 01 (bagian 5).

---

# Ruang keadaan

Kode: `state_space.py`, dengan parameter dari `konfig.py`. Cetak ulang untuk
robot ini dengan:

```bash
python state_space.py --id 2
```

Bentuk persamaannya **identik** dengan ID 01, karena bentuk itu hanya ditentukan
oleh susunan kendaraan (sepeda berkemudi belakang, acuan di poros depan), dan
susunan itu sama. Angka-angka di model linear juga ternyata sama: skala langkah
ID 02 terukur 0.002490 dan ID 01 0.002491 cm/langkah, sedangkan τ dan tunda
kemudi tidak berbeda. Jadi **matriks di bawah sama dengan matriks ID 01** pada
laju yang sama. Yang berbeda adalah **batas** di model tak linear: δ_max 23.5°,
bukan 35°. Semua parameter di bagian 5 sudah diukur di ID 02.

Ada dua bentuk, untuk dua keperluan: yang tak linear untuk simulasi dan ramalan
pose, yang linear untuk perancangan kendali (LQR/MPC lewat seam di `jejak.py`).
Pure pursuit yang dipakai sekarang tidak memakai keduanya.

## Dua kesepakatan yang menentukan bentuk persamaan

**Acuannya poros roda DEPAN.** Roda depan tidak berbelok, jadi kecepatan di
poros depan selalu searah heading badan. Karena itu model sepeda baku berlaku
tanpa suku tambahan.

**δ adalah sudut kemudi LOGIS**, dengan konvensi kendaraan berkemudi depan:
δ > 0 berarti robot berbelok ke kiri (yaw bertambah). Pembalikan untuk roda
belakang dikerjakan firmware (`str_inv = 1`, dipastikan lewat `show config`
2026-09-30). Hubungannya dengan servo:

```
sudut servo fisik = str_center − δ        (str_inv = 1, str_center = STEER_SERVO_CENTER)
```

**Tanda ini terkonfirmasi di ID 02.** Pada kalibrasi kemudi, servo fisik 79°
memberi δ = 96.15 − 79 = +17.15°, dan robot berbelok ke kiri (kelengkungan
+0.737 °/cm).

## 1. Model lengkap (tak linear)

```
keadaan  ξ = [x, y, θ, δ, v]ᵀ        masukan  u = [δ_cmd, v_cmd]ᵀ

ẋ = v cos θ
ẏ = v sin θ
θ̇ = v tan δ / L
δ̇ = (sat(δ_cmd) − δ) / τ
v̇ = sat(v_cmd − v, ±a)
```

Satuan: `x, y` cm | `θ, δ, δ_cmd` radian | `v, v_cmd` cm/detik. Titik `(x, y)`
adalah poros roda depan.

| batasan | nilai ID 02 | asal |
|---|---|---|
| \|δ\| ≤ δ_max | **23.5° = 0.4102 rad** | `max_steer_deg` di `konfig.ROBOT[2]`. Uji busur: kanan jenuh 23.7°, kiri 29° (bagian 5d) |
| \|v̇\| ≤ a | 29.88 cm/detik² | NVS `accel` 12000 × 0.002490 (semula 4000, lihat di bawah) |
| v maju | ≤ +24.9 cm/detik | `FORWARD_SPEED` 10000 sps × 0.002490 |
| v mundur | ≥ −10.0 cm/detik | `REVERSE_SPEED` 4000 sps × 0.002490 |

> **Akselerasi ID 02 semula hanya sepertiga ID 01. SUDAH DISETEL 12000
> (2026-09-30)** lewat MQTT (`set accel 12000`, `save`), dan dipastikan dengan
> `show config` serta telemetri `acc: 12000`.
>
> `show config` semula menunjukkan `accel = 4000`, yaitu bawaan firmware
> `DEF_DRIVE_ACCEL` yang belum pernah disetel. Ramp awal di lima lintasan
> kalibrasi memastikannya: yang paling cocok 4250-5000 sps/s², dan 12000 selalu
> lebih buruk (sisa 0.17-0.30 cm, dibanding 0.07-0.24 cm). Pada laju penuh
> 24.9 cm/detik:
>
> | | accel 4000 (semula) | accel 12000 (sekarang, = ID 01) |
> |---|---|---|
> | waktu mencapai laju penuh | 2.5 s | 0.83 s |
> | jarak berhenti berkuasa | **31.1 cm** | 10.4 cm |
>
> Pelacak dan margin keselamatan disetel dengan 12000. Pada ID 01, accel yang
> terlalu kecil (3000) terbukti menjadi penyebab tabrakan zona Z8. Kalau NVS
> terhapus (flash ulang, `factory`), robot kembali ke 4000 **tanpa
> pemberitahuan**, jadi periksa `show config` setelahnya.
>
> **Ramp 12000 diikuti tanpa langkah hilang**, sesuai uji gas 2026-09-30 (bagian
> 5e).

Suku δ̇ dan v̇ adalah **aktuator, bukan kendaraan**: yang pertama jeda servo
kemudi, yang kedua ramp akselerasi stepper. Tunda murni kemudi tidak ada di
persamaan ini; yang mengurusnya adalah bentuk diskret di bagian 3.

### Radius yang keluar dari model ini

Bergantung pada L, δ_max, dan badan robot. Karena δ_max ID 02 lebih kecil,
radiusnya jauh lebih besar dari ID 01:

| besaran | ID 02 | ID 01 | keterangan |
|---|---|---|---|
| R_min = L / tan δ_max | **51.75 cm** | 32.13 cm | kemudi mentok |
| R_plan (75% kemudi) | **70.82 cm** | 45.63 cm | sisanya cadangan koreksi pelacak |
| R_sapu (sudut badan terluar) | **79.67 cm** | 54.52 cm | ID 02 butuh lingkaran bebas **~159 cm**, ID 01 ~109 cm |
| κ_max = 1/R_min | 0.0193 cm⁻¹ | 0.0311 cm⁻¹ | kelengkungan terbesar yang bisa diikuti |
| κ_plan = 1/R_plan | 0.0141 cm⁻¹ | 0.0219 cm⁻¹ | batas yang dipakai Hybrid A* |

**Akibatnya pada perencanaan** (20 pasangan awal/tujuan acak yang sama,
arah hadap tujuan bebas, 2026-09-30):

| peta | ID 01 (35°) | ID 02 (23.5°) | panjang rute ID 02 / ID 01 |
|---|---|---|---|
| Rumah | 18/20 berhasil | **14/20** berhasil | median 1.25×, maks 1.99× |
| Kalibrasi | 20/20 | 20/20 | median 1.23×, maks 1.70× |

Semua rute yang berhasil untuk ID 02 juga berhasil untuk ID 01. ID 02 masih
bisa dinavigasi di peta Rumah, tapi lebih sering menolak tujuan dan lebih
banyak bermanuver maju-mundur.

## 2. Model galat jalur (linear)

```
x = [e_y, e_θ, δ]ᵀ        u = δ_cmd        w = κ (kelengkungan jalur acuan)

ė_y = v sin e_θ           ≈ v · e_θ
ė_θ = v tan δ / L − κ v   ≈ (v/L) δ − v κ
δ̇   = (δ_cmd − δ) / τ
```

```
      ⎡0   v   0  ⎤        ⎡ 0 ⎤        ⎡ 0 ⎤        ⎡1  0  0⎤
A  =  ⎢0   0  v/L ⎥   B =  ⎢ 0 ⎥   E =  ⎢−v ⎥   C =  ⎣0  1  0⎦
      ⎣0   0 −1/τ ⎦        ⎣1/τ⎦        ⎣ 0 ⎦
```

`C` adalah yang terukur kamera: simpangan lateral dan galat heading. δ tidak
terukur di mana pun, karena robot ini tidak punya sensor sudut kemudi.

Pada v = 24.9 cm/detik (`FORWARD_SPEED` 10000 sps × 0.002490), L = 22.5 cm,
τ = 0.10 s:

```
      ⎡0  24.90   0     ⎤        ⎡ 0 ⎤        ⎡   0   ⎤
A  =  ⎢0   0      1.1067⎥   B =  ⎢ 0 ⎥   E =  ⎢−24.90 ⎥
      ⎣0   0    −10     ⎦        ⎣10 ⎦        ⎣   0   ⎦
```

## 3. Bentuk diskret + rantai tunda

Diskretisasi tepat (zero-order hold) pada dt = 0.05 s (20 Hz, periode loop
kendali):

```
        ⎡1  1.2450  0.0294⎤         ⎡0.0051⎤
Ad  =   ⎢0  1       0.0435⎥    Bd = ⎢0.0118⎥
        ⎣0  0       0.6065⎦         ⎣0.3935⎦
```

Tunda murni kemudi diwakili rantai penggeser (bukan Padé, yang menambah kutub
palsu). `SIM_KEMUDI_TUNDA_S` = 0.19 s dibulatkan menjadi 4 cacah = 0.20 s:

```
x_aug = [e_y, e_θ, δ, u_{k−4}, u_{k−3}, u_{k−2}, u_{k−1}]ᵀ
```

### Sifat sistem

| pemeriksaan | hasil |
|---|---|
| keterkendalian tanpa tunda | rank 3/3, terkendali |
| keterkendalian dengan tunda | rank 7/7, terkendali |
| kutub diskret (tanpa tunda) | 1.0, 1.0, 0.6065 |

Dua kutub di z = 1 adalah integrator ganda posisi ← heading, jadi **sistem ini
tidak stabil sendiri**. Kutub 0.6065 = e^(−dt/τ) adalah jeda servo kemudi.

> **τ = 0.10 s dan tunda 0.19 s: diukur ulang di ID 02 dan dipertahankan.**
> Dua uji tangga ID 02 (bagian 5c) memberi optimum yang berbeda satu sama lain,
> sama seperti dua uji tangga ID 01. Tapi model konfig (0.19 + 0.10) hanya
> 0.05-0.15° lebih buruk dari optimum masing-masing uji.

## 4. Sambungan ke perangkat

**Masukan → firmware** (`robot.py`):

```
speed_sps = v_cmd / CM_PER_STEP            CM_PER_STEP = 0.002490 cm/langkah
servo_deg = 96.15 + δ_cmd[deg]             dibatasi 10..170 (str_min/str_max)
```

MQTT `forklift02/in/rc` = `"<langkah/detik bertanda> <sudut servo>"`, dikirim
20x per detik. Board di 10.210.1.102. Firmware menghentikan motor bila aliran
ini putus lebih dari 0.5 detik.

Rentang ±23.5° di sekitar 96.15 adalah 72.65-119.65°, masih di dalam batas servo
10-170. `robot.py` memotong perintah kemudi di ±`MAX_STEER_DEG`, jadi pelacak
tidak lagi meminta sudut yang tidak sanggup dicapai.

> **`str_center` di NVS robot harus sama dengan `STEER_SERVO_CENTER`.** Firmware
> membalik kemudi di sekitar `str_center` miliknya, sedangkan `robot.py`
> mengirim sudut mutlak `96.15 + δ`. Selisih d derajat antara keduanya membuat
> perintah "lurus" menjadi belok d derajat. NVS semula berisi 84.00, yang akan
> membuat robot belok sekitar 12° saat diperintah lurus. **Disetel 96.15 dan
> disimpan 2026-09-30**, lalu dipastikan dengan `show config`.

**Keluaran ← kamera** (`pose.py`, tag id 2). Yang dilihat kamera adalah pusat
tag, bukan poros depan:

```
x_tag = x − 5.5 cos θ        y_tag = y − 5.5 sin θ        (TAG_TO_FRONT_AXLE_CM)
```

Latensi kamera adalah sifat kamera dan pipeline dekoder, bukan sifat robot. Uji
gas ID 02 mengukurnya 0.110-0.135 s. Angka itu sedikit di atas ID 01
(0.085-0.095 s, 2026-09-27), tapi di rentang yang sama (bagian 5e). Di uji lurus ID 02,
robot mulai terlihat bergerak (0.3 cm) 0.32-0.42 s setelah perintah pertama.
Rinciannya: ramp sampai 0.3 cm butuh ~0.14 s, kamera ~0.09 s, sisanya
0.09-0.19 s adalah jalur perintah. Angka terakhir ini sesuai dengan yang terukur
di ID 01 (0.06-0.34 s).

δ dan v tidak pernah terukur, sama seperti ID 01. Saat tag hilang, `pose.py`
meramal pose dengan model bagian 1, dan v dimulai dari taksiran ramp yang
mengintegrasikan `ACCEL_SPS2` dari aliran perintah. Karena itu `accel_sps2` di
`konfig.ROBOT` **harus sama dengan NVS**. Ramalan ini sah hanya selama
`RAMAL_MAKS_S` = 0.4 s; setelah itu robot di-estop.

## 5. Parameter dan asalnya

| parameter | nilai ID 02 | ID 01 | asal untuk ID 02 |
|---|---|---|---|
| L (wheelbase) | 22.5 cm | 22.5 cm | diukur (bagian Parameter fisik) |
| geometri badan & tag | sama | | diukur |
| ID AprilTag | **2** | 1 | 36h11 |
| titik lurus kemudi | **96.15°** | 89.45° | **diukur**, dua cara, lihat 5a |
| CM_PER_STEP | 0.002490 | 0.002491 | **diukur**, dua cara, lihat 5b |
| a (akselerasi) | 12000 sps/s² = 29.88 cm/s² | 12000 | setelan NVS, disetel 2026-09-30; **diikuti** tanpa langkah hilang (5e) |
| penguatan kemudi | 0.92 (≤ 20°, busur) | ~0.92 | **diukur**, sama dengan ID 01, tidak diterapkan (5d) |
| τ (tetapan waktu kemudi) | 0.10 s | 0.10 s | **diukur ulang** 2026-09-30, lihat 5c |
| tunda murni kemudi | 0.19 s | 0.19 s | **diukur ulang** 2026-09-30, lihat 5c |
| δ_max | **23.5°** | 35.0° | **diukur** 2026-09-30, jenuh tidak simetris, lihat 5d |
| latensi kamera | 0.11-0.135 s | 0.085-0.095 s | diukur di uji gas masing-masing (5e) |
| dt | 0.05 s | 0.05 s | periode loop kendali |

Data: `tools/kemudi_id02.csv` (proyek Forklift Control, `kalibrasi_kemudi.py`,
5 lintasan ~70 cm pada 4000 sps, maju dan mundur), dan
`logs_identifikasi/20260930_*_id02` (`identifikasi.py`):

- `_025444`, `_025546`, `_025606`: 6 lintasan lurus 4000-12000 sps dan 2 uji
  tangga
- `_030300`: 14 busur ±5 sampai ±35°
- `_030802`: 4 lintasan gas beserta telemetri

Semua dianalisis ulang dari berkas, dengan fungsi `cocokkan_*` identifikasi.py
yang sama.

### 5a. Titik lurus dan penguatan kemudi

**Koreksi pivot pada kalibrasi.** `kalibrasi_kemudi.py` menganggap pivot
firmware 89 dan mengirim `178 − sudut`. Padahal `str_center` di NVS ID 02 saat
uji adalah 84 (board tidak restart di antaranya; telemetri `str: 71` sesudahnya
cocok dengan lintasan terakhir). Jadi sudut fisik sebenarnya = label skrip −
2 × (89 − 84) = **label − 10**.

| label skrip | servo fisik | δ logis | arah | sudut roda efektif | penguatan |
|---|---|---|---|---|---|
| 89 | 79 | +17.15° | maju | +16.15° | 0.942 |
| 95 | 85 | +11.15° | mundur | +9.85° | 0.883 |
| 103 | 93 | +3.15° | maju | +3.04° | 0.966 |
| 106 | 96 | +0.15° | mundur | +0.20° | - |
| 107 | 97 | −0.85° | maju | −0.91° | - |

Sudut roda efektif dihitung dari kelengkungan κ = laju yaw / laju, dengan laju
dari tali busur ujung-ke-ujung (lihat 5b).

**Titik lurus, dari dua cara:**

| sumber | arah | titik lurus fisik |
|---|---|---|
| kalibrasi, 3 lintasan | maju | 96.10° |
| kalibrasi, 2 lintasan | mundur | 96.25° |
| identifikasi lurus, 6 lintasan 4000-12000 sps | maju | **95.94° ± 0.08** |

Dari identifikasi: dengan str_center 96.15, robot melenceng ke kanan dengan
sudut setara −0.21° ± 0.08. Dengan pembalikan firmware, titik lurus yang
sebenarnya = 96.15 + (−0.21) = 95.94.

> **Saran otomatis identifikasi.py saat itu SALAH ARAH.** Programnya mencetak
> "STEER_SERVO_CENTER sebaiknya 96.36". Rumusnya (center − o) hanya benar tanpa
> pembalikan kemudi. Dengan `str_inv = 1` dan str_center = STEER_SERVO_CENTER,
> sudut fisik = center − δ, sehingga yang benar adalah center + o = 95.94.
> Kesimpulan ini cocok dengan kalibrasi yang arahnya bebas dari rumus itu:
> pada kalibrasi pun, lintasan maju menunjuk sedikit di bawah 96.15. Rumusnya
> sudah diperbaiki (`konfig.STEER_INVERT`). **Jangan pasang 96.36.**

**Keputusan: 96.15 dipertahankan.** Tiga taksiran di atas berbeda paling jauh
0.3°. Pola datanya menunjukkan backlash, bukan kalibrasi yang meleset: lintasan
maju cenderung lurus di sekitar 95.9-96.1°, lintasan mundur di sekitar 96.25°.
Nilai 96.15 berada di antaranya. Untuk navigasi yang hampir selalu maju, 96.0
akan sedikit lebih baik. Tapi selisih 0.15° hanya setara ~1.2° heading per
meter, dan pure pursuit mengoreksinya lewat kamera. Kalau diubah, ubah di dua
tempat sekaligus: `konfig.ROBOT[2]` dan robot (`set str_center`, `save`).

**Penguatan kemudi dari kalibrasi**: 0.94 dan 0.97 pada lintasan maju, 0.88 pada
satu lintasan mundur. Uji busur (5d) memberi angka yang lebih kuat: 0.92.

Uji tangga (5c) juga mencocokkan penguatan, dan hasilnya jauh lebih kecil
(0.70 dan 0.81). Tapi di sana penguatan dicocokkan **bersamaan** dengan titik
lurus, dan titik lurus hasil cocokan itu (−1.52° dan −0.71°) jauh dari yang
diukur langsung (−0.21°). Artinya kedua parameter itu tidak terpisahkan dengan
baik dalam uji tangga, sehingga penguatan dari tangga jangan dipakai.

### 5b. Skala langkah

| sumber | CM_PER_STEP |
|---|---|
| kalibrasi, 5 lintasan 4000 sps (tali busur) | 0.002490 ± 0.000007 |
| identifikasi lurus, 6 lintasan 4000/8000/12000 sps | 0.002486 |
| ID 01 (cusp.csv, 337 cuplikan tunak) | 0.002491 |

Dua sumber ID 02 cocok dalam 0.2%, dan keduanya cocok dengan ID 01. Pada
12000 sps angkanya sedikit lebih tinggi (0.002513-0.002519); arahnya berlawanan
dengan selip, jadi tidak ada tanda langkah hilang pada laju itu.

> **Koreksi (2026-09-30).** Versi pertama dokumen ini mencatat 0.002540 ("2%
> lebih besar dari ID 01, kemungkinan roda berbeda"), dari "dua cara yang saling
> bebas". Itu salah. Kedua cara memakai jarak yang **dijumlah dari |selisih
> posisi| antar-cuplikan**, dan jumlah itu bias ke atas oleh derau kamera:
> setiap suku positif, jadi deraunya tidak hilang dengan dirata-rata. Peringatan
> ini sudah tertulis di `Rekaman.laju_dan_kelengkungan` (identifikasi.py).
> Dengan jarak tali busur ujung-ke-ujung, data kalibrasi yang **sama** memberi
> 0.002490, dan biasnya +0.9% sampai +2.8% per lintasan. Kelengkungan di 5a
> sudah dihitung ulang dengan cara yang benar; titik lurusnya tidak terpengaruh,
> karena titik potong nol tidak bergantung pada skala jarak.
>
> Bias yang sama masih ada di `pasang_kemudi()` (uji tangga): laju tunaknya
> diambil dari jarak kumulatif, sehingga penguatan dari tangga terbaca ~2% lebih
> kecil. Pengaruhnya kecil dibanding ketidakpastian lain di sana, dan belum
> diperbaiki.

### 5c. Tunda dan tetapan waktu kemudi (uji tangga)

Pola kemudi 0 → +20° → 0 → −20°, tiap fase 2.5 s, pada 4000 sps. Yaw terukur
dicocokkan dengan model "tunda d lalu orde-1 τ":

| uji | d (terlihat kamera) | τ | d + τ | sisa yaw, optimum | sisa yaw, model konfig 0.19 + 0.10 | korelasi silang |
|---|---|---|---|---|---|---|
| 025444 | 0.00 s | 0.10 s | 0.10 s | 1.21° | 1.36° | 0.33 s (r 0.87) |
| 025606 | 0.21 s | 0.17 s | 0.37 s | 0.74° | 0.79° | 0.45 s (r 0.95) |
| *ID 01, dua uji* | *0.41 / 0.12 s* | *~0 / 0.12 s* | *0.41 / 0.24 s* | *0.49 / 0.44°* | *0.59 / 0.57°* | *0.42 / 0.30 s* |

Uji pertama kurang bisa dipercaya: d menempel di batas bawah 0, korelasinya
paling lemah (r 0.87), dan sisa yawnya paling besar. Uji kedua bersih, dan
d + τ = 0.37 s berada di rentang ID 01 (0.24-0.41 s).

**Kesimpulan: `SIM_KEMUDI_TUNDA_S` 0.19 dan `SIM_KEMUDI_TAU_S` 0.10 tetap.** Di
kedua uji, model konfig hanya 0.05-0.15° lebih buruk dari optimum uji itu
sendiri, dan tidak ada alasan memakai optimum satu uji: dua uji ID 02, seperti
dua uji ID 01, berbeda sebesar ayunan jalur perintah. Servo ID 02 tidak
terlihat lebih lambat atau lebih cepat dari ID 01.

### 5d. Busur: penguatan dan batas kemudi

Kemudi tetap selama 9 s pada 4000 sps, di 14 sudut. Sudut roda nyata dihitung
dari kelengkungan tunak (laju yaw ÷ laju tali busur), setelah membuang transien
0.49 s (tunda + 3τ):

| diminta | nyata kiri (+) | nyata kanan (−) | rata-rata \|nyata\| / diminta | geser kiri-kanan |
|---|---|---|---|---|
| 5° | +4.67° | −4.84° | 0.951 | −0.08° |
| 10° | +9.15° | −9.57° | 0.936 | −0.21° |
| 15° | +14.08° | −14.11° | 0.939 | −0.02° |
| 20° | +18.43° | −18.30° | 0.918 | +0.07° |
| 25° | +24.64° | **−21.65°** | 0.926 | +1.50° |
| 30° | +29.11° | **−23.63°** | 0.879 | +2.74° |
| 35° | **+28.81°** | **−23.75°** | 0.751 | +2.53° |

**Sampai 20° kemudi bersih dan simetris.** Penguatannya 0.924 (cocokan tan, sudut
≤ 20°), dengan intersep −0.06°. Geser kiri-kanan rata-rata −0.06°, yang
menegaskan titik lurus 96.15 (5a). Penguatan 0.92 **sama dengan ID 01**
(`konfig.py` bagian KEMUDI: rasio 0.92 pada 15-25°, dari cusp.csv), dan untuk
ID 01 `STEER_SERVO_PER_DEG` sengaja dibiarkan 1.0. Jadi untuk ID 02 juga tidak
diubah, walaupun `identifikasi.py` menyarankan 1.083. Pure pursuit menutup
kalang lewat kamera, dan kedua robot diperlakukan sama.

**Di atas 20° kemudi jenuh, dan tidak simetris:**

- **kiri** mentok di **~29°**: +30 dan +35 memberi R 40.4 dan 40.9 cm, praktis
  sama;
- **kanan** mentok di **~23.7°**: −30 dan −35 memberi R 51.4 dan 51.1 cm, dan
  −25 sudah tertahan di 21.65°.

Lintasan −35° juga paling tidak rapi (sisa yaw 6.6°, dibanding ≤ 1.8° di sudut
lain), seperti kemudi yang tertahan paksa. **`max_steer_deg` ID 02 = 23.5°**,
yaitu sisi yang terlemah, karena perencana menganggap batasnya simetris. ID 01
sanggup 35° (memuncak 36°).

**Kemungkinan besar penyebabnya mekanis.** Dengan `str_inv = 1`, belok kanan
berarti sudut servo fisik naik: kanan mentok di servo ~96.15 + 23.7/0.92 ≈ 122°,
kiri di ~96.15 − 29/0.92 ≈ 65°. Keduanya jauh dari batas servo (0-180) dan batas
firmware (10-170). Jadi yang menahan hampir pasti linkage atau roda yang
menyentuh badan, bukan servo atau firmware. Kalau itu diperbaiki, jalankan ulang
`identifikasi.py --id 2 --mode busur`, lalu naikkan `max_steer_deg` di
`konfig.ROBOT[2]`.

### 5e. Gas: akselerasi dan tunda perintah

Empat lintasan (target 3000/5500/8000/10000 sps: diam → naik ke target → turun
ke sepertiganya), dengan telemetri firmware direkam:

| target | stepper mulai (naik / turun) | latensi kamera | cm/langkah (pos vs kamera) | sisa |
|---|---|---|---|---|
| 3000 | 0.20 / 0.18 s | 0.110 s | 0.002492 | 0.10 cm |
| 5500 | 0.25 / 0.19 s | 0.135 s | 0.002487 | 0.15 cm |
| 8000 | 0.06 / 0.13 s | 0.115 s | 0.002503 | 0.19 cm |
| 10000 | 0.15 / 0.22 s | 0.115 s | 0.002505 | 0.24 cm |

- **Ramp 12000 diikuti, tanpa langkah hilang.** Posisi kamera berimpit dengan
  hitungan langkah firmware × CM_PER_STEP dalam 0.1-0.6%, dengan sisa
  ≤ 0.24 cm di seluruh lintasan, termasuk 10000 sps.
- **CM_PER_STEP dipastikan untuk ketiga kalinya**: 0.00249-0.00251, cocok
  dengan 5b.
- **Tunda perintah → stepper 0.06-0.25 s** dan berayun antar-lintasan, persis
  seperti ID 01 (0.06-0.34 s). Karena ayunan itu, pemisahan "tunda vs A" dari
  garis antar-target ditolak oleh programnya sendiri (chi² 1342), sama seperti
  di ID 01. Pakai angka telemetri di atas, bukan garis itu.
- Latensi kamera 0.11-0.135 s, sedikit di atas ID 01 (0.085-0.095 s). Selisih
  ~0.03 s setara ~0.7 cm pada laju penuh. Tidak diubah di simulator, tapi
  perlu diingat.

## 6. Seberapa jauh modelnya boleh dipercaya

### 6a. Terhadap robot nyata: **belum diuji**

Belum ada rekaman navigasi ID 02. Validasi ramalan di [[Model Robot ID 01]]
bagian 6a-6c (horizon 0.4 s, p95 0.41 cm) dilakukan di ID 01. Parameter model
linear kedua robot kini sama, jadi untuk belokan di bawah 20° hasilnya
**kemungkinan** berlaku juga di sini. Tapi itu belum dibuktikan, dan pada
belokan kanan di atas 20° model akan meleset sampai jenuhnya dipotong di
23.5°. Untuk mengujinya:

```bash
python navigasi.py --peta Rumah --id 2 --log jalan_id02.csv
$env:FORKLIFT_ID = 2; python validasi_nyata.py jalan_id02.csv
```

Log format 11 kolom mencatat kolom `diramalkan`, sehingga pose ramalan bisa
dibuang dari acuan.

### 6b. Terhadap simulator (menguji linearisasinya, bukan modelnya)

`python state_space.py --id 2`: jalur lurus, perintah kemudi acak, 6 detik,
v = 24.9 cm/detik:

| amplitudo kemudi | galat RMS | % dari lingkup simpangan |
|---|---|---|
| ±4° | 0.02 cm | 0.3% |
| ±8° | 0.05 cm | 0.4% |
| ±10° | 0.07 cm | 0.5% |
| ±20° | 0.36 cm | 1.2% |
| ±30° | 2.43 cm | 6.2% |

Ujian ini membandingkan dua model, jadi **tidak** menjawab apakah modelnya cocok
dengan robot. Sampai ±20° hasilnya sama dengan ID 01. Pada ±30° galatnya 6.2%
(ID 01: 2.3%), dan itu **bukan** karena linearisasi: model tak linear memotong
δ di 23.5°, sedangkan model linear tidak punya batas sama sekali. Jadi model
linear hanya sah di bawah δ_max. Pengendali yang dirancang darinya
(LQR/MPC) harus memberi batas masukan ±23.5° secara eksplisit. Linearisasi
`tan δ` ≈ `δ` sendiri meleset 6% pada 23.5° (0.435 vs 0.410).

**Yang sengaja tidak ada di model**, sama seperti ID 01: batas laju servo
(500 °/detik, jarang mengikat), luncuran bebas setelah estop
(`SIM_LUNCUR_DESEL_CM_S2` belum diukur), dan dinamika samping.

## 7. Status

1. ~~Robot: `set str_center 96.15`, `set accel 12000`, `save`~~. **Selesai
   2026-09-30**, dipastikan dengan `show config`.
2. ~~`konfig.ROBOT[2]` disamakan~~. **Selesai**. `python konfig.py` dengan
   `FORKLIFT_ID=2` menulis "semua beres".
3. ~~Uji tangga (τ, tunda)~~. **Selesai**, nilai konfig dipertahankan (5c).
4. ~~Skala langkah~~. **Selesai**, 0.002490 (5b).
5. ~~Uji busur~~. **Selesai**: penguatan 0.92 (= ID 01), δ_max 23.5° karena
   sisi kanan mentok (5d). `konfig.ROBOT[2]` sudah disetel.
6. ~~Uji gas~~. **Selesai**: ramp 12000 diikuti tanpa langkah hilang (5e).
7. **Disarankan:** periksa mekanik kemudi sisi kanan (5d). Kalau batasnya bisa
   dibuka sampai ~29° seperti sisi kiri, peta Rumah kembali lebih lapang untuk
   ID 02.
8. **Belum:** validasi ramalan dengan log navigasi (6a).

## Berkas terkait

- `konfig.py`: tabel `ROBOT` (ID 01 / ID 02), `pilih_robot()` (juga menghitung
  R_MIN/R_PLAN/R_SAPU dari `max_steer_deg`), `STEER_INVERT`
- `state_space.py`: kedua model + uji linearisasi (`python state_space.py --id 2`)
- `identifikasi.py --id 2`: ukur parameter di robot; log di
  `logs_identifikasi/<waktu>_id02`
- `navigasi.py --id 2` / `robot.py --id 2`: robot yang dikendalikan
- `tools/kalibrasi_kemudi.py` + `tools/kemudi_id02.csv` (proyek Forklift
  Control): data 5a-5b. **Catatan:** `PIVOT_FIRMWARE` di skrip itu harus sama
  dengan `str_center` robot saat uji, dan jarak `s_cm` yang ditulisnya adalah
  jarak kumulatif, yang bias oleh derau (5b)
