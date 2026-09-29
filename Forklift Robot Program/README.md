# Forklift Robot Program

Navigasi otomatis forklift Robot ID 01: klik tujuan dengan mouse, robot
merencanakan jalurnya sendiri dengan **Hybrid A\*** dan mengikutinya dengan
**pure pursuit**. Batas geraknya diambil dari peta constraint `area_constraint.py`.

Pasangan firmware: **Forklift Control v2** (topik `forklift01/...`).

## Berkas

| Berkas | Tugas | Uji mandiri |
|---|---|---|
| `konfig.py` | Semua parameter dalam cm + MQTT. Sumber tunggal geometri. | `python konfig.py` |
| `arena.py` | Peta constraint → occupancy grid → `ObstacleMap` | `python arena.py --peta Rumah` |
| `pose.py` | AprilTag → pose cm; juga simulator model sepeda | `python pose.py --simulasi` |
| `robot.py` | MQTT ke firmware v2, konversi kemudi→servo, estop | `python robot.py` |
| `rencana.py` | Hybrid A\* dalam sel grid | `python rencana.py --peta Rumah` |
| `jejak.py` | Pelacak jalur — **titik tukar algoritma** | (pustaka) |
| `navigasi.py` | Program utama, satu atau beberapa robot (`--id 1,2`) | `python navigasi.py --simulasi` |
| `wasit.py` | Pencegah tabrakan antar-robot: bayangan, aturan, prioritas | `python wasit.py` |
| `state_space.py` | Model ruang-keadaan + uji linearisasi (vs simulator) | `python state_space.py` |
| `validasi_nyata.py` | Nilai model terhadap **jalan nyata** (vs pose kamera) | `python validasi_nyata.py` |
| `identifikasi.py` | Ukur parameter model **di robot** | `python identifikasi.py --simulasi` |
| `uji_gas.py` | Uji-mandiri uji tangga gas: tanam parameter, periksa pulihnya | `python uji_gas.py` |
| `uji_kamera.py` | Bandingkan latensi dekode kamera antar-pengaturan, tanpa robot | `python uji_kamera.py` |
| `forklift_planner.py` | Pustaka Hybrid A\* + Reeds-Shepp + pure pursuit. **Salinan v1, jangan diubah.** | |

Urutannya berlapis: `konfig` → `arena` → `rencana`/`jejak` → `navigasi`.
Tiap berkas bisa dijalankan sendiri, dan tiga yang pertama tidak butuh kamera
maupun robot.

## Mulai cepat

```bash
python konfig.py                      # periksa kewarasan seluruh nilai
python navigasi.py --simulasi         # coba penuh tanpa perangkat
python navigasi.py --peta Rumah       # perangkat nyata
python navigasi.py --simulasi --id 1,2   # dua robot, tanpa perangkat
python navigasi.py --peta Rumah --id 1,2 # dua robot nyata (lihat "Dua robot")
```

Di jendela navigasi: **klik-tarik kiri** menetapkan tujuan — titik klik adalah
posisi, arah tarikan adalah arah hadap. Tarikan pendek (< 6 cm) berarti arah
hadap bebas, perencana yang memilih. Klik kanan membatalkan.

## Keputusan rancangan

**Semuanya dalam sentimeter.** Versi lama (`MQTT_1tag.py`) bekerja dalam piksel
dengan `PX_PER_CM` yang diturunkan dari ukuran AprilTag — dan ukuran itu
ternyata tidak cocok dengan dokumen model (14,1 vs 10 cm). Peta constraint sudah
menyimpan zona dalam cm lewat homografi terkalibrasi, jadi seluruh persoalan itu
hilang dengan bekerja dalam cm. Nilai tuning lama tetap terpakai, tinggal
dikonversi (`PX_PER_CM = 2,468`).

**Rintangan berasal dari peta, bukan dari penglihatan.** Zona di arena ini
virtual — tidak ada wujud fisiknya, lantainya kosong. Tidak ada yang bisa
dideteksi mata kamera, jadi petalah satu-satunya sumber kebenaran.

**Perencanaan di thread terpisah.** Firmware menghentikan motor bila aliran `rc`
putus lebih dari 0,5 detik, sedangkan Hybrid A\* di peta nyata terukur sampai
1,3 detik. Loop kendali berjalan tetap 20 Hz; perencanaan dikerjakan di belakang.

**AprilTag satu-satunya umpan balik posisi.** Robot ini tidak punya encoder.
Field `pos` di telemetri firmware adalah hitungan langkah yang *diperintahkan* —
kalau roda selip, ia tetap bertambah dan tidak akan pernah tahu. Satu-satunya
penginderaan sejati di telemetri ada di dalam chip TMC2209 (suhu, short,
kumparan terbuka, StallGuard), bukan sensor yang dipasang di robot.

Akibatnya, saat tag tidak terdeteksi, pose **diramalkan** dengan model
ruang-keadaan dari perintah yang sedang dikirim — tidak dibekukan di nilai
terakhir. Membekukannya berarti mengemudi berdasarkan posisi usang:

Diukur terhadap **jalan nyata** (`cusp.csv`, 162 detik, pose AprilTag sebagai
acuan) — bukan simulasi seperti versi tabel sebelumnya:

| tag hilang | pose beku | pose ramalan |
|---|---|---|
| 0,2 s | median 0,93 · p95 3,49 · maks 4,44 cm | median 0,18 · p95 1,98 · maks 3,84 cm |
| 0,4 s (jendela `RAMAL_MAKS_S`) | median 1,88 · p95 7,18 · maks 8,54 cm | median 0,23 · p95 3,76 · maks 6,60 cm |
| 1,0 s | median 5,11 · p95 18,86 · maks 21,09 cm | median 0,33 · p95 7,96 · maks 13,49 cm |

Ramalan mengalahkan pose beku pada 89–93% cuplikan. Sebarannya **bukan satu
gundukan**: dalam keadaan tunak galatnya di bawah 1 cm sampai laju penuh,
sedangkan saat perintah gas baru berubah ia naik ke p95 5,3 cm.

> **Diperbaiki 2026-09-27.** Sebelumnya `pose.py._mulai_ramal()` menyetel `v`
> ke kecepatan yang *diperintahkan* seketika, padahal motor masih menanjak
> mengikuti ramp. Galat terburuknya **9,41 cm** di dalam jendela 0,4 detik —
> melebihi `SAFETY_MARGIN_CM = 8,0` pada 5 cuplikan. Sekarang
> `SumberKamera.perintah()` melacak kecepatan motor dengan mengintegrasikan ramp
> `ACCEL_SPS2`, dan pelanggaran margin itu hilang (5 → 0, maks 9,41 → 6,60 cm).
> Pada horizon 0,2 detik, galat terburuk ramalan dulu bahkan **lebih buruk** dari
> pose beku (4,89 vs 4,44 cm); sekarang tidak lagi.
>
> **Selisih tunak–transien ternyata sebagian besar salah penjajaran waktu.**
> Kamera terlambat ~0,68 s (terukur 2026-09-27, lihat "Yang masih perlu Anda
> kerjakan" butir 3), sedangkan validasi di atas menganggap pose kamera adalah
> posisi robot *saat itu*. Diulang dengan pose dicap pada waktu tangkap yang
> benar: p95 3,76 → **1,47 cm**, maks 6,60 → **4,52 cm**, p95 transien 4,31 →
> **2,01 cm**. Model ruang-keadaannya jauh lebih teliti dari yang tabel di atas
> tunjukkan; yang belum dimodelkan adalah kameranya.
> Rinciannya di "Model Robot ID 01.md" bagian 6b; angka sebelum perbaikan bisa
> diputar ulang dengan `python validasi_nyata.py --v0 perintah`.

Lewat 0,4 detik pose ditandai tidak sah dan robot di-estop. Pose ramalan muncul
kuning di layar dengan keterangan, dan `identifikasi.py` **menolaknya** — ramalan
bukan pengukuran.

**Dua lapis keselamatan yang saling bebas.** Perencana menghindari zona lewat
`ObstacleMap` (jalur *rencana*); pemantau memeriksa badan robot pada pose
*nyata* dari AprilTag dan memicu estop. Ditambah lapis ketiga: telemetri `stall`
dari firmware v2 memicu hitung ulang, karena stall berarti posisi perintah tidak
lagi mewakili posisi roda.

**Pelacak bisa ditukar.** `jejak.py` memegang daftar pelacak; `navigasi.py` hanya
memanggil `jejak.buat(nama, ...)` dan tidak pernah mengimpor implementasinya.
Menambah MPC nanti cukup satu fungsi berdekorator `@daftar("mpc")` plus satu
bagian di `konfig.PARAM_PELACAK` — `navigasi.py` tidak perlu disentuh.
Kontraknya:

```python
tracker.update(gx, gy, yaw_rad, now) -> {"speed", "steer_deg", "done", "paused", "target"}
tracker.done        # bool
tracker.deviation   # simpangan dari jalur, satuan SEL
```

Pelacak sengaja **tidak** menghitung sudut servo — itu urusan `robot.py`, supaya
pelacak baru tidak perlu tahu titik lurus servo robot ini 89,45 derajat.

## Kecepatan

Robot berjalan **17,4 cm/detik** di lintasan lurus, melambat mengikuti
kelengkungan jalur, dan **tetap pelan (5,2 cm/detik) pada 35 cm terakhir
menjelang tujuan** — di situ galat tidak sempat dikoreksi lagi karena jalurnya
habis.

> Angka-angka di bagian ini sebelumnya menyebut 31 dan 9,4 cm/detik. Keduanya
> berasal dari `FORWARD_SPEED = 12000` dengan `CM_PER_STEP = 0,0026`; sekarang
> `FORWARD_SPEED = 7000` dan skala langkah terukur 0,002491. **Komentar sebaris
> di `konfig.py:266` masih menulis `# ~31 cm/detik` dan itu salah** — 7000 sps
> adalah 17,4 cm/detik. Seluruh tabel tuning di atasnya juga masih membahas
> 9000–15000, jadi tabel itu tidak lagi menggambarkan setelan yang berjalan.

Yang membatasi kecepatan ternyata bukan `FORWARD_SPEED`, melainkan rasio-rasio
perlambatan: jalur Hybrid A\* di peta rapat melengkung hampir di mana-mana,
sehingga robot hanya 0,5% waktunya berada di kecepatan puncak. Tabel
pengukurannya ada di `konfig.py`.

> **Firmware harus diset `accel 12000`, sama dengan `konfig.ACCEL_SPS2`.**
> Dipastikan 2026-09-27 lewat `show config`: NVS robot memang 12000. Anjuran
> 8000 yang dulu tertulis di sini sudah tidak berlaku.
>
> Kenapa harus sama persis: sejak 2026-09-27 `ACCEL_SPS2` (dulu
> `SIM_ACCEL_SPS2`) **tidak lagi khusus simulator** — `pose.py` memakainya di
> perangkat nyata untuk melacak kecepatan motor saat meramal pose. Selisih
> dengan NVS berarti ramalan yang salah.
>
> Kenapa 12000 dan bukan lebih rendah: pada accel rendah robot tidak sanggup
> mengikuti perlambatan yang diperintahkan di tengah rute (rasio laju
> nyata/perintah sampai 3,98, penyebab tabrakan Z8 — lihat komentar
> `ACCEL_SPS2` di `konfig.py`). Alasan lama soal zona belokan akhir sudah tidak
> berlaku pada `FORWARD_SPEED = 7000`: melambat 7000 → 2100 sps hanya butuh
> 13,9 cm bahkan pada accel 4000.
>
> **Periksa lagi setelah flash ulang.** Bawaan firmware `DEF_DRIVE_ACCEL` adalah
> 4000 (`include/defaults.h:68`), jadi NVS yang terhapus diam-diam kembali ke
> 4000. Lewat MMI:
>
> ```
> telnet 10.210.1.101 23
> show config
> set accel 12000
> save
> ```

### Melambat di celah sempit

Robot melambat ke 45% saat badannya mendekat ke zona — mulai terasa di bawah
4 cm sisa jarak, penuh saat menyentuh batas margin keselamatan. Jalur 20 cm di
depan ikut diperiksa supaya pengereman terjadi **sebelum** masuk celah. Angka 20
cm dulu dipilih untuk menyamai jarak pengereman dari 31 ke 14 cm/detik; pada
setelan sekarang (17,4 → 7,9 cm/detik) pengereman hanya butuh 6,1 cm pada accel
8000 atau 12,1 cm pada bawaan 4000, jadi `SEMPIT_LIHAT_CM = 20` kini longgar —
bukan ketat.

Status `CELAH SEMPIT 45%` muncul di baris atas jendela navigasi saat aktif.

Ongkosnya bergantung kerapatan peta, dan di peta rapat tidak murah:

| peta | perlambatan mati | perlambatan aktif | aktif berapa lama |
|---|---|---|---|
| `Kalibrasi` (lapang) | 57,0 s | 57,0 s | 0% |
| `Rumah` (rapat) | 95,4 s | 147,0 s | 95% |

Di `Rumah` fiturnya praktis selalu aktif, karena median sisa jarak di sepanjang
jalur memang hanya 5,6 cm. Kalau ingin peta itu lebih cepat, yang benar adalah
melebarkan koridor zonanya — bukan menaikkan `SEMPIT_MULAI_CM`.

Kalau di perangkat nyata muncul masalah, turunkan `FORWARD_SPEED` bertahap
(7000 → 6000 → 5000). Urutan lama yang tertulis di sini, 12000 → 10000 → 9000,
seluruhnya **di atas** setelan yang berjalan sekarang. Dua gejala yang perlu
dipantau:

- **Stall** — torsi stepper turun seiring kecepatan. Pantau `diag` dan `diag_ev`
  di telemetri, atau `show drv` di MMI. Tidak terlihat di simulasi.
- **AprilTag kabur** — catatan v1 menyebut 22 cm/detik sudah berisiko. Pada 17,4
  cm/detik setelan sekarang sudah di bawah ambang itu, tapi kalau kolom
  `terlihat` sering nol atau navigasi sering estop "tag robot hilang", itu
  sebabnya. Di `cusp.csv` hanya 24 dari 2619 baris yang kehilangan tag (0,9%),
  jadi pada laju ini gejalanya tidak muncul.

Jeda kemudi sendiri **sudah** dimodelkan simulator (`SIM_KEMUDI_TUNDA_S` +
`SIM_KEMUDI_TAU_S` + batas laju servo); catatan lama yang menyebut sebaliknya
sudah tidak berlaku.

## Model ruang-keadaan

`state_space.py` menyediakan dua bentuk. Yang **tak linear** (keadaan
`[x, y, θ, δ, v]`) dipakai simulasi dan pemeriksaan kelayakan manuver. Yang
**linear** adalah dinamika galat jalur, untuk perancangan LQR/MPC:

```
x = [e_y, e_θ, δ]ᵀ       u = δ_cmd        w = κ (kelengkungan jalur acuan)

      ⎡0   v   0  ⎤        ⎡ 0 ⎤        ⎡ 0 ⎤
A  =  ⎢0   0  v/L ⎥   B =  ⎢ 0 ⎥   E =  ⎢−v ⎥
      ⎣0   0 −1/τ ⎦        ⎣1/τ⎦        ⎣ 0 ⎦
```

Didiskretkan tepat (zero-order hold) pada 20 Hz, dengan tunda masukan diwakili
rantai penggeser 4 cacah — bukan pendekatan Padé, yang akan menambah kutub
palsu. Terkendali penuh (rank 7/7 dengan tunda). Dua kutub di z = 1 adalah
integrator ganda posisi←heading: **sistem ini tidak stabil sendiri**.

Divalidasi **dua kali, terhadap dua hal yang berbeda** — jangan tertukar:

- **Terhadap robot nyata** (`cusp.csv`, pose AprilTag): dalam keadaan tunak galat
  ramalan **di bawah 1 cm sampai laju penuh** (p95 0,32–1,22 cm pada horizon
  0,4 detik). Saat perintah gas baru berubah, p95 naik ke 7,2 cm — lihat bagian
  "AprilTag satu-satunya umpan balik posisi" di atas.
- **Terhadap simulator tak-linear** (`python state_space.py`): galat **0,3–3,2%**
  untuk kemudi ±4° sampai ±30° pada 17,4 cm/detik (0,3–1,3% pada 31 cm/detik).
  Ini hanya menguji linearisasinya — dua-duanya model, jadi ujian ini tidak
  mengatakan apa pun tentang kecocokan dengan robot. Galatnya naik dari
  0,3–0,4% sejak τ kemudi diturunkan 0,72 → 0,10 s (2026-09-27): kemudi yang
  cepat benar-benar mencapai sudut besar, sehingga selisih tan δ vs δ ikut
  terasa. Angka **2,6–3,0%** yang lebih tua lagi berasal dari setelan
  `FORWARD_SPEED = 12000`.

Linearisasi mengganti `tan δ` dengan `δ`, jadi untuk manuver rapat pakai bentuk
tak-linearnya. Rincian lengkap kedua validasi ada di "Model Robot ID 01.md"
bagian 6.

## Identifikasi di robot

`identifikasi.py` mengukur parameter model dari robot yang sebenarnya lewat empat
percobaan: **lurus** (skala langkah + titik lurus kemudi), **busur** (penguatan
kemudi + kejenuhan), **tangga** (tunda + tetapan waktu kemudi), dan **gas**
(akselerasi motor yang sebenarnya + tunda perintah→gerak). Gas tidak ikut
`--mode semua` — lihat di bawah.

Sebelum tiap manuver, lintasannya diramalkan dengan model tak-linear lalu
diperiksa ke peta constraint — manuver yang tidak muat dilewati, bukan dicoba
lalu ditabrakkan. Tiap manuver diikuti leg pulang agar robot tidak menjauh.

> Butuh area lapang. Busur kemudi penuh berjari-jari ~32 cm, jadi perlu
> lingkaran bebas ~110 cm. Peta rapat seperti `Rumah` tidak akan muat.

```bash
python identifikasi.py --simulasi              # uji programnya, tanpa robot
python identifikasi.py --peta Kalibrasi        # di perangkat (lurus+busur+tangga)
python identifikasi.py --peta Kalibrasi --mode tangga
python identifikasi.py --peta Kalibrasi --mode gas   # robot menghadap lintasan lurus ~1,2 m
```

### Uji tangga gas

Empat lintasan lurus: diam 0,5 s → lompat ke target → turun ke sepertiganya,
pada target 30/55/80/100% `FORWARD_SPEED`. Menjawab apakah motor benar-benar
menanjak mengikuti `ACCEL_SPS2`, berapa tunda perintah→gerak, dan apakah
akselerasinya tetap atau melemah di laju tinggi.

Dua keputusan rancangan yang tidak boleh diubah tanpa mengulang uji-mandirinya:

- **Posisi yang dicocokkan, bukan kecepatan.** Kecepatan dari kamera membaca
  laju palsu p95 5,75 cm/detik pada robot yang diam.
- **Tunda dan akselerasi dipisahkan ANTAR-target, bukan dalam satu lintasan.**
  Dalam satu lintasan keduanya berkorelasi +0,98 — ramp 12000 hanya 4–12 frame,
  dan "terlambat mulai" tidak bisa dibedakan dari "lambat menanjak". Yang
  terukur baik per lintasan adalah keterlambatan total L = tunda + target/2A;
  antar-target, intersep garis L adalah tunda dan kemiringannya 1/2A. Target
  keempat menguji apakah garisnya lurus, yaitu apakah akselerasi tetap.

Tidak ikut `--mode semua`, karena hasil parsialnya tidak berguna: butuh ≥ 3
target yang muat, dan setelah lurus + busur + tangga ruangnya sudah habis (di
simulasi peta `Kalibrasi` hanya 2 dari 4 yang muat).

Uji-mandiri dengan parameter tertanam di simulator (latensi kamera 25–65 ms),
dihasilkan ulang dengan `python uji_gas.py` — benih acak dikunci, jadi angkanya
sama setiap kali. Jalankan ulang setiap kali estimatornya diubah:

| ditanam | tunda terukur | akselerasi terukur | ditandai tak linear |
|---|---|---|---|
| dasar: A 12000 | 0,051 s | 11886 ± 287 | tidak |
| + tunda 0,15 s | 0,198 s (**+0,147**) | 11889 ± 269 (tidak berubah) | tidak |
| A 4000 | 0,053 s (tidak berubah) | 4002 ± 34 | tidak |
| A melemah dengan laju | (negatif — diperingatkan) | — | **ya** |
| dasar + derau 0,3 cm | 0,067 s | 12229 ± 990 | tidak |
| A 4000 + derau 0,3 cm | 0,060 s | 3973 ± 111 | tidak |

**Di robot nyata (2026-09-27) pengandaian "tunda tetap" itu GAGAL.** Keterlambatan
yang terlihat kamera ~1 detik dan berbeda antar-lintasan (0,79–1,13 s), sehingga
garis L bengkok karena tunda, bukan karena akselerasi. Telemetri kemudian
memilahnya: ~0,68 s latensi kamera (tetap) ditambah jalur perintah 0,06–0,34 s
(berubah-ubah). Versi pertama program ini tetap
menyimpulkan "akselerasi melemah" dari garis itu, dan kesimpulannya keliru: batas
atas tunda di pencocok hanya 1,0 s, sehingga tunda mentok dan sisanya diserap
akselerasi. Sekarang batasnya 3 s, kesimpulan dari garis hanya dicetak bila
garisnya sah, dan program selalu mencetak **awal gerak tanpa model**. Angka itu
tidak bergantung model ramp dan tetap sah ketika model gagal. Analisis ulang
rekaman lama tanpa robot:

```bash
python identifikasi.py --dari logs_identifikasi/20260927_085850
```

**Telemetri firmware** direkam selama tiap lintasan (hanya di perangkat;
simulator tidak punya MQTT) dan diurai menjadi dua angka presisi dari kolom
`pos` (langkah yang sudah dikeluarkan stepper):

- **stepper mulai**: kapan langkah benar-benar keluar, dicocokkan pada seluruh
  kurva `pos`, bukan dibaca dari cuplikan 4 Hz. Versi pertama laporan ini
  membaca cuplikan dan mendapat 0,57 s untuk stepper yang mulai ~0,34 s.
- **latensi kamera**: seberapa jauh kurva kamera harus digeser agar berimpit
  dengan kurva langkah. Pergeseran waktu murni (cm/langkah cocok, sisa kecil)
  berarti robotnya tepat waktu dan kameranya yang terlambat; langkah yang hilang
  tidak bisa ditutupi geseran waktu.

Keduanya memuat latensi telemetri kembali ke PC dengan tanda berlawanan, jadi
latensi kamera yang dilaporkan adalah batas bawah. Kalau kamera "mendahului"
langkah (geseran negatif), telemetrinyalah yang terlambat. Program juga
memperingatkan bila pesan telemetri menumpuk lalu tiba sekaligus.

`uji_gas.py` menanam tunda 0,9 s di jaringan, firmware, jalur balik, atau
kamera, plus satu skenario tanpa tunda. Laporannya menunjuk tempat yang benar
pada kelimanya; waktu stepper mulai tepat dalam ~0,01 s, dan latensi kamera
berada dalam rentang latensi kamera simulator.

Pemeriksaan autokorelasi sisa per lintasan hanya menangkap kegagalan kasar
(stall di tengah lintasan). Ia **tidak** menangkap akselerasi yang melemah —
+0,42 pada lintasan itu, sama dengan lintasan yang benar. Yang menangkapnya
uji chi² antar-target.

Diuji dengan memulihkan parameter yang ditanam di simulator:

| parameter | nilai sebenarnya | terukur |
|---|---|---|
| `CM_PER_STEP` | 0,002600 | 0,002599 |
| penguatan kemudi | 1,000 | 0,993 |
| sudut maksimum | 35,0° | 34,8° |
| jeda kemudi | 0,65 s (acuan v1) | 0,70 s |

Durasi busur bawaan 9 detik bukan kesewenangan: pada 5 detik penguatan terbaca
0,963 untuk robot yang penguatannya jelas 1,000 — kemudinya belum selesai
bergerak saat diukur. Program memperingatkan bila jendela ukurnya terlalu pendek.

### Titik lurus kemudi perlu pengulangan

Titik lurus diturunkan dari kelengkungan **sisa** pada lintasan yang seharusnya
lurus — besaran nyaris nol yang tenggelam di derau kamera. Satu lintasan tidak
cukup, jadi pakai `--ulang`:

```bash
python identifikasi.py --peta Kalibrasi --mode lurus --ulang 25
```

Di antara pengulangan robot **memutar arah**. Itu bukan variasi kosmetik:
mengulang di koridor yang sama membuat galatnya berkorelasi — distorsi homografi
yang tersisa di suatu tempat terulang identik setiap lintasan, jadi
merata-ratakan tidak menghapusnya. Titik lurus adalah sifat **robot** dan harus
sama di segala arah; distorsi kamera tidak. Arah putar dipilih yang menjauh dari
zona, kalau tidak robot mengembara dan menabrak sekitar pengulangan ke-7.

Yang dilaporkan adalah **galat baku rata-rata**, bukan sebaran — yang ditanya
"seberapa yakin pada rata-ratanya", dan itu menyusut ~1/√n:

| `--ulang` | lintasan | pergeseran | galat baku |
|---|---|---|---|
| 1 | 3 | +2,25° | 2,78° |
| 4 | 12 | −0,10° | 0,71° |
| 25 | 69 | **−0,02°** | **0,23°** |

Program menolak sendiri angkanya selama galat bakunya masih di atas 0,25° dan
menyebut berapa `--ulang` lagi yang dibutuhkan. Alternatifnya
`tools/kalibrasi_kemudi.py` dari v1, yang memang dibuat khusus untuk ini.

## Hasil uji simulasi

Peta `Rumah` + `Kalibrasi`, 11 rute termasuk 3 rute berderau pose 1,2 cm di peta
tersempit, **dengan model jeda aktif**:

```
tercapai 10/11
galat posisi akhir : rata 4,4 cm, maks 6,6 cm
simpangan jalur    : rata 2,21 cm
```

Simulatornya memodelkan tiga jeda: kemudi (tunda murni + orde-1 + batas laju
servo, korelasi silang memuncak di 0,60 detik — acuan v1 0,65), kamera (25 fps
+ latensi, usia frame berayun 30–85 ms), dan ramp akselerasi stepper. Tanpa
model itu angkanya terlihat jauh lebih bagus (simpangan 1,8 cm) tapi
menyesatkan — `SumberSimulasi(jeda=False)` untuk membandingkan.

`SAFETY_MARGIN_CM = 8,0` ditetapkan dari pengukuran, bukan tebakan. Padanan v1
hanya 2 cm dan itu membuat robot menyenggol zona di `Rumah`. Nilai 6 cm lolos
uji tanpa derau, tapi **gagal 3 dari 6 rute begitu derau pose 1,2 cm
ditambahkan** — uji tanpa derau terlalu bersahabat, karena kamera nyata selalu
berderau. Tabel lengkapnya ada di `konfig.py`.

## Dua robot

```bash
python navigasi.py --simulasi --id 1,2
python navigasi.py --peta Rumah --id 1,2 --log jalan.csv   # -> jalan_id01.csv, jalan_id02.csv
```

Rancangannya ada di catatan Obsidian "Multi Robot Navigation and Control".
Ringkasnya:

- **Satu proses, satu kamera.** `pose.KameraBersama` membuka satu aliran RTSP
  dan mendeteksi semua tag di satu frame; tiap robot mengambil tag miliknya.
  Semua nilai yang berbeda antar-robot (tag, topik MQTT, `str_center`,
  cm/langkah, akselerasi, batas kemudi) dibaca dari `konfig.profil(id)`, bukan
  dari global `konfig` - global itu milik robot yang terakhir dipilih.
- **Wasit (`wasit.py`)** dipanggil tiap siklus 20 Hz *sebelum* perintah dikirim.
  Setiap robot yang berjalan membawa **bayangan**: jejak badannya di sepanjang
  jalur ke depan, sepanjang `L = 5 + v·0,2 + v²/(2·a)` cm (a = ramp akselerasi
  firmware, 29,9 cm/s²; pada 24,9 cm/s → 20,4 cm). Robot yang bayangannya akan
  mengenai badan robot lain masuk **MENUNGGU** (jalur tetap dipegang, rc nol
  dikirim), dan lanjut setelah bayangannya - dihitung pada laju *tujuan* -
  bebas 0,5 s.
- **Prioritas** saat jalur hanya akan berpotongan: robot yang sedang minggir →
  yang sedang mundur → yang sisa jalurnya lebih pendek → ID lebih kecil.
- **Jalan keluar dari deadlock.** Menunggu > 5 s (1 s untuk yang kalah prioritas
  saat berhadapan) → jalur dihitung ulang dengan robot lain sebagai rintangan.
  Kalau itu tidak ada (dua robot bertukar tempat), robot **MINGGIR** ke pose di
  luar jalur lawan - hanya pose dari mana tujuannya sendiri masih tercapai
  setelah lawan parkir - lalu lanjut begitu lawan diam.
- Robot yang diam dijadikan rintangan saat merencanakan dan saat menerima
  tujuan; tujuan yang bertumpuk dengan tujuan robot lain ditolak.

Uji mandiri aturan wasit (tanpa kamera, tanpa robot): `python wasit.py`.

### Hasil uji simulasi dua robot

Lima skenario, derau pose 1,2 cm, 3 seed masing-masing. Jarak diukur dari pose
*sebenarnya* simulator, bukan pose terukur:

| Skenario | Peta | Lolos | Jarak badan terdekat | Waktu |
|---|---|---|---|---|
| Bersilangan tegak lurus | Kalibrasi | 3/3 | 12,6–14,5 cm | ~12 s |
| Berhadapan (bertukar tempat) | Kalibrasi | 3/3 | 6,2–10,7 cm | 28–54 s |
| Robot lain parkir di jalur | Kalibrasi | 3/3 | 9,0–10,8 cm | 28–62 s |
| Bersilangan di persimpangan | Rumah | 3/3 | 15,2–17,6 cm | ~16 s |
| Berhadapan di koridor 38 cm | Rumah | 1/3 | 12,3–13,2 cm | 136 s |

Di 15 run: **kontak 0, badan sebenarnya masuk zona 0**. Dua kegagalan di koridor
Rumah adalah estop keselamatan zona karena pose *terukur* (berderau) menyentuh
zona saat manuver panjang di sudut sempit - sifat peta Rumah yang sama dengan
satu robot (lihat `SAFETY_MARGIN_CM`), hanya lebih sering terpicu karena dua
robot yang bertukar tempat di koridor itu harus bermanuver lama. Koridor itu
lebih sempit dari dua badan robot; bertukar tempat di sana memang lambat.

Belum diuji di robot nyata. `MULTI_T_REAKSI_S` (0,2 s) juga belum diukur.

## Yang masih perlu Anda kerjakan

Sudah selesai, diperiksa 2026-09-27:

- ~~Mengisi `tinggi_tag_cm`~~ — terisi 16,5.
- ~~Menyamakan `robot_constraint.json`~~ — 27 × 17,5 geser −4,0;
  `python konfig.py` melaporkan "semua beres".
- ~~Memperbaiki keadaan awal ramalan pose~~ — `SumberKamera.perintah()` kini
  melacak ramp motor; pelanggaran `SAFETY_MARGIN_CM` di jendela 0,4 detik
  turun 5 → 0. Kode perangkat terbukti identik dengan kode uji
  (`validasi_nyata.py`), selisih 0,000 di 2619 baris.
- ~~Memastikan `accel` di NVS~~ — `show config` = 12000, sama dengan
  `konfig.ACCEL_SPS2`.
- ~~Mencatat kolom `diramalkan` di log~~ — `navigasi.py --log` kini menulis 11
  kolom, dan `validasi_nyata.py` membuang pose ramalan dari acuannya. Berkas log
  format lama (10 kolom) **ditolak** untuk ditambahi, supaya dua format tidak
  tercampur dalam satu CSV — pakai nama berkas baru.

Yang tersisa:

1. ~~**Rekam ulang satu rute dengan log format baru**~~ — **selesai
   2026-09-27** (`jalan_setelah_kamera.csv`, peta `Rumah`, setelah perbaikan
   kamera). Dibandingkan dengan `cusp.csv` (kamera 0,68 s). Rutenya berbeda,
   jadi ini perbandingan indikatif, bukan terkendali:

   | | `cusp.csv` | setelah perbaikan |
   |---|---|---|
   | tujuan tercapai | 2 dari 3 | **4 dari 4** |
   | kejadian BAHAYA | 1 | **0** |
   | simpangan jalur rata / p95 / maks | 1,73 / 6,95 / 11,92 cm | **1,31 / 2,97 / 3,64 cm** |
   | jarak terdekat ke zona | 0,55 cm | **4,29 cm** |
   | baris tag tidak terlihat | 24 | **0** |

   Validasi model (`validasi_nyata.py`) pada log ini, horizon 0,4 s: median
   0,15 cm, **p95 0,41 cm**, maks 2,42 cm, dibanding p95 3,76 / maks 6,60 cm di
   `cusp.csv`. Selisih tunak–transien praktis hilang (p95 0,30 vs 0,49 cm).
   Log ini juga yang pertama dengan kolom `diramalkan`, dan tidak ada satu pun
   pose ramalan di dalamnya, jadi angkanya bebas dari bias itu. Laju tertinggi
   di rute ini di bawah 15 cm/detik; laju penuh 17,4 cm/detik belum teruji.

2. ~~**Jeda kemudi terukur ulang**~~ — **diterapkan 2026-09-27**:
   `SIM_KEMUDI_TUNDA_S` 0,22 → **0,19**, `SIM_KEMUDI_TAU_S` 0,72 → **0,10**.
   Dua uji tangga setelah perbaikan kamera, dalam kerangka waktu kamera:

   | | uji 1 (4000 sps) | uji 2 (3000 sps) |
   |---|---|---|
   | korelasi silang (v1: 0,65 s) | 0,42 s | 0,30 s |
   | tunda d / τ | 0,41 / ~0 s | 0,12 / 0,12 s |
   | model lama (0,22 + 0,72), sisa yaw | 2,13° | 2,06° |
   | **tunda 0,25 + τ 0,10**, sisa yaw | **0,59°** | **0,57°** |

   Angka pastinya berbeda antar-uji, dengan selisih ~0,17 s — sebesar variasi
   jalur perintah di uji gas. Tapi keterlambatan efektif d + τ ≈ 0,35 s dan
   τ kecil konsisten di keduanya. Tunda 0,25 + τ 0,10 hanya ~0,1° dari optimum
   masing-masing uji. Jadi **tunda model lama kurang lebih benar; τ-nya yang
   salah besar** (0,72 → ~0,10). Pencocoknya terbukti bisa melihat τ bila ada:
   di simulator ia membaca 0,714 untuk τ tertanam 0,72. Nilai simulator 0,19 /
   0,10 adalah setara dari 0,25 / 0,10 kerangka kamera, karena simulator
   memodelkan kamera terpisah (~0,06 s). Hasil uji pertama saja (0,35 / 0,02)
   sempat tercatat di sini, lalu tidak terkonfirmasi oleh uji kedua.

   Perubahan ini menggeser simulator, `state_space.py` (kutub kemudi 0,9329 →
   0,6065), dan ramalan di `pose.py` sekaligus. Diuji sesudahnya:

   | uji | hasil |
   |---|---|
   | `uji_gas.py` | lulus semua |
   | `validasi_nyata.py jalan_setelah_kamera.csv` | praktis sama (0,4 s: p95 0,41 cm) — rute ini tidak peka terhadap model kemudi |
   | 10 rute tertutup di simulator, kemudi lama vs baru | 9/10 tercapai pada keduanya; simpangan rata 1,97 → **1,45 cm**, maks 11,87 → **6,45 cm** |

   Satu rute di peta `Rumah` gagal pada **kedua** model: badan robot melanggar
   Z6 setelah ~13,5 s dengan simpangan maks hanya ~5 cm. Itu bukan akibat
   perubahan ini, tapi layak diselidiki — Z6 juga salah satu tempat tabrakan
   nyata sebelumnya. Tabel tuning "diukur di simulasi" di `konfig.py` dibuat
   dengan kemudi lama yang ~7× lebih lamban; anggap sejarah.

   `cocokkan_tangga` dulu menyarankan "bagi kedua parameter dengan
   puncak/0,65". Saran itu terbalik arah (puncak lebih kecil → parameter
   membesar), dan sudah diganti pencocokan yaw di atas.

3. **Kamera terlambat ~0,68 detik. Ini temuan terbesar sejauh ini.** Uji tangga
   gas dengan telemetri firmware (2026-09-27,
   `logs_identifikasi/20260927_091649`) memilah keterlambatan ~1 detik yang
   terlihat di kamera:

   | tahap | terukur |
   |---|---|
   | perintah dikirim → stepper mulai | 0,06–0,34 s (biasanya ≤ 0,15) |
   | stepper → terlihat di kamera | **0,675 / 0,685 / 0,725 s** |

   Posisi kamera adalah hitungan langkah firmware yang digeser ~0,68 s:
   cm/langkah cocok dalam 0,3%, sisa 0,08–0,17 cm. Robotnya bergerak tepat
   waktu, stepper mengikuti ramp 12000 dengan tepat, dan tidak ada langkah
   hilang. `konfig.SIM_KAMERA_DELAY_S` masih 0,025 s: itu latensi
   *pemrosesan*, sedangkan latensi aliran RTSP tidak pernah diukur. Angka 0,68
   adalah batas bawah (latensi telemetri kembali ke PC belum ditambahkan).

   **Dua klaim sebelumnya keliru.** "Kamera tidak lambat" dan "tundanya tidak
   menentu" berasal dari `cusp.csv`, di mana separuh start tampak bergerak dalam
   0,06–0,12 s. Keenamnya ternyata start setelah jeda titik ganti arah 0,62 s,
   dan kamera **masih menayangkan gerak lama** (1,7–5,3 cm dalam 0,6 s sebelum
   perintah). Start dari diam lama selalu terlambat 1,1–1,4 s.

   Yang dipertaruhkan:
   - **Setiap pose yang dipakai navigasi setua ~0,68 s.** Pada 17,4 cm/detik
     itu ~12 cm, lebih besar dari `SAFETY_MARGIN_CM = 8`. Kemungkinan besar
     penyumbang utama tabrakan zona — dugaan, belum terbukti.
   - **Semua tuning berbasis simulasi** (margin, lookahead, `lag_comp = 0`,
     kecepatan) diukur dengan kamera ~27× lebih cepat dari kenyataan. Keputusan
     `lag_comp = 0` khususnya dibuat karena "lead lookahead lebih besar dari
     jeda" — dengan jeda sebenarnya, perhitungan itu berubah.
   - **Model kemudi** (`SIM_KEMUDI_TUNDA_S` + `_TAU_S`, disetel ke 0,65 s dari
     v1) kemungkinan besar sebagian besarnya latensi kamera, bukan servo.
   - `pose.py` memakai pose kamera seolah posisi *sekarang*. Pelacak ramp yang
     dipasang 2026-09-27 benar arahnya, tapi koreksi yang jauh lebih besar
     adalah memajukan setiap pose sebesar latensi kamera.

   **Penyebabnya ditemukan dan sudah diperbaiki (2026-09-27): multithreading
   dekoder.** OpenCV membuka dekoder FFmpeg dengan thread sebanyak inti CPU
   (16 di PC ini). Dekode multithread per-frame menahan ~15 frame, yaitu
   ~0,6 s pada ~26 fps. `cctv_lurus.StreamReader` sekarang membuka aliran
   dengan **1 thread** (`cctv_lurus.buka_rtsp`).

   Diukur dengan `uji_kamera.py`, tanpa robot. Beberapa sesi RTSP menerima
   bitstream yang sama dan dekode H.264 deterministik, sehingga frame yang sama
   bisa dikenali lewat sidiknya di tiap sesi:

   | pengaturan | frame keluar lebih dulu | perkiraan latensi |
   |---|---|---|
   | 1 thread | 589–611 ms | ~0,07–0,09 s |
   | 2 thread | 558–561 ms | ~0,12 s |
   | `StreamReader` terpasang | 590 ms | ~0,09 s |

   Latensinya tidak melayang dan dekode tetap mengikuti ~26 fps. Satu sesi
   sendirian berjalan tanpa galat dekode di kedua pengaturan. `fflags nobuffer`
   / `flags low_delay` tidak menambah manfaat.

   **Dipastikan di robot** (`identifikasi.py --mode gas`,
   `logs_identifikasi/20260927_094202`):

   | | sebelum | sesudah |
   |---|---|---|
   | latensi kamera | 0,675–0,725 s | **0,085–0,095 s** |
   | pose tertinggal pada 17,4 cm/s | ~12 cm | **~1,6 cm** |
   | stepper mulai setelah perintah | 0,06–0,34 s | 0,05–0,16 s |
   | akselerasi naik (garis antar-target) | tak terukur (garis bengkok) | **11372 ± 493** (0,95× setelan, chi²/dof 0,6) |

   Tidak ada langkah hilang (cm/langkah dalam 0,2%), dan kurva kamera
   berimpit dengan model ramp 12000 sampai ≤ 1,4 mm, termasuk saat melambat.

   Simulator (0,025 s + umur frame + jitter) terukur ~0,04–0,05 s dengan cara
   yang sama, jadi kini hanya ~0,05 s lebih cepat dari robot, dibanding 0,63 s
   sebelumnya. Semua rekaman robot sebelum perbaikan ini dibuat dengan kamera
   ~0,68 s.

   Jalur perintah sisanya kecil tapi pernah tersendat: pada lintasan 2100,
   13 pesan telemetri menumpuk lalu tiba sekaligus, dan perintahnya terlambat
   ~0,34 s. Itu terjadi di lintasan pertama, tepat setelah fase persiapan
   program. Wi-Fi power save **bukan** penyebabnya — firmware sudah
   `WiFi.setSleep(false)` (`net.cpp:172`).

4. **Perbaiki komentar sebaris `konfig.py:266`** yang masih menulis
   `# ~31 cm/detik` untuk `FORWARD_SPEED = 7000` (sebenarnya 17,4 cm/detik).

5. **Ulangi sapuan tuning kecepatan.** Seluruh tabel di `konfig.py:236-256`
   diukur pada 9000–15000 sps dengan `CM_PER_STEP = 0,0026`. Setelan yang
   berjalan sekarang, 7000 sps dengan skala 0,002491, tidak ada di tabel itu.

Geometri badan sudah diukur (poros depan → ujung depan 4 cm, → ujung belakang
23 cm) dan tercatat di "Model Robot ID 01.md", lengkap dengan model
ruang-keadaannya dan hasil validasi terhadap jalan nyata.

## Catatan arena

Robot butuh lingkaran bebas **~109 cm** untuk berputar penuh (radius rencana
45,6 cm + sapuan badan). Peta `Rumah` titik terlapangnya hanya 34 cm, jadi di
sana robot tidak pernah bisa memutar di tempat — Hybrid A\* akan memakai manuver
maju-mundur. Itu bekerja (terbukti 3/3 rute), hanya lebih lambat. Kalau ingin
manuver lebih lega, sisakan koridor lebih lebar saat menggambar zona.
