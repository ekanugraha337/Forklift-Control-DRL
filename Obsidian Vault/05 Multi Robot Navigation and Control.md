- navigation.py dapat menjalankan robot dengan id 01 dan id 02. 
- id ini merupakan id di firmware dan april tag id. 

Strategi untuk menghindari tabrakan :
- setiap robot memiliki kotak bayangan (di depan atau di belakang tergantung arah pergerakan) dengan jarak 500 ms perkiraan. Jadi kalau makin cepat, jarak ke bayangan semakin panjang. Dan jika robot semakin lambat, jarak bayangan semakin dekat. 
- Navigasi secara default untuk kedua robot bebas, jadi navigasinya bisa cross path.  
- Ketika robot akan menabrak robot di depan atau di belakangnya, robot yang akan menabrak harus estop, jika robot lawannya sudah lewat maka bisa melanjutkan navigasi

---

## Usulan Claude (2026-09-30)

> Hasil membaca catatan di atas terhadap `navigasi.py`, `konfig.py`, `pose.py`. Konsep bayangan yang panjangnya bergantung kecepatan dipertahankan; yang diubah adalah cara mewujudkannya supaya tidak deadlock dan benar-benar sempat berhenti.

### 1. Arsitektur: satu proses untuk dua robot

Kondisi sekarang: `navigasi.py --id N` mengendalikan **satu** robot per proses, dan `SumberKamera.baca()` membuang semua tag selain miliknya (`pose.py`, `if tid != self.tag_id: continue`). Dua proses = dua aliran RTSP, dua dekode, pose tidak sinkron, dan tidak ada yang tahu posisi robot lain.

Usulan:
- **Satu aliran kamera**, satu `Detektor`. Satu frame → pose ID 01 dan ID 02 sekaligus (timestamp sama).
- **Dua `Robot` MQTT** (`forklift01/…`, `forklift02/…`), dua perencana, dua pelacak. Konstanta per robot (`steer_center`, `max_steer_deg`, `R_MIN_CM`, …) harus menjadi atribut objek, bukan global `konfig` yang ditimpa `pilih_robot()`.
- **Satu wasit anti-tabrakan** di loop 20 Hz yang sama, dijalankan sebelum perintah `rc` dikirim.
- UI: tombol `1` / `2` memilih robot aktif; klik-tarik memberi tujuan ke robot aktif.

### 2. Panjang bayangan: waktu 500 ms tidak cukup

Bayangan 500 ms tumbuh **linear** terhadap v, jarak berhenti tumbuh **kuadrat**. Dengan angka konfig (maju 31 cm/s, luncuran estop ~30 cm/s², latensi kamera ~0,09 s + loop 0,05 s + MQTT):

| Kecepatan | Bayangan 500 ms | Reaksi (~0,2 s) | Berhenti v²/2a | Dibutuhkan |
|---|---|---|---|---|
| 31 cm/s (maju) | 15,5 cm | 6,2 cm | 16,0 cm | ~22 cm + margin |
| 10 cm/s (mundur) | 5 cm | 2 cm | 1,7 cm | ~4 cm + margin |

Rumus usulan:

```
L_bayangan = d_min + v · T_reaksi + v² / (2 · a_rem)
    d_min    = SAFETY_MARGIN_CM (8 cm)  -> bayangan tidak pernah nol saat diam
    T_reaksi ≈ 0,2 s                    -> ukur ulang dari log
    a_rem    ≈ 30 cm/s²                 -> SIM_LUNCUR_DESEL_CM_S2, belum diukur di robot
```

Hasil: ~30 cm pada 31 cm/s, ~12 cm pada 10 cm/s. `v` diambil dari **taksiran ramp** (`SumberKamera._v`), bukan dari perintah.

> [!note] Koreksi setelah implementasi
> - Kecepatan maju sebenarnya **24,9 cm/s** (`FORWARD_SPEED` 10000 × `CM_PER_STEP` 0,00249), bukan 31. Komentar "~31 cm/detik" di `konfig.py` sudah usang.
> - `a_rem` = ramp akselerasi firmware (`accel` 12000 × cm/langkah = **29,9 cm/s²**), karena MENUNGGU mengirim rc nol (melambat terkendali), bukan estop. Jadi tidak bergantung pada `SIM_LUNCUR_DESEL_CM_S2` yang belum diukur.
> - `d_min` dipecah dua: bayangan minimum `MULTI_LIHAT_MIN_CM` = 5 cm, dan jarak aman bayangan-ke-badan `MULTI_JARAK_AMAN_CM` = 5 cm (harus < margin cadangan perencana 6 cm).
> - Hasil: **L = 20,4 cm pada 24,9 cm/s** (bayangan 500 ms hanya 12,5 cm, padahal butuh ~15 cm untuk diam).

### 3. Bentuk bayangan: sapuan badan sepanjang jalur

Kotak lurus di depan/belakang meleset di tikungan, dan robot berkemudi belakang: buntutnya mengayun keluar (`R_SAPU_CM`). Jalur Hybrid A* sudah diketahui, jadi:

- Bayangan = gabungan `A.jejak_badan()` di titik-titik jalur mulai dari posisi pelacak sampai sejauh `L_bayangan` ke depan, **melewati titik ganti arah** bila ada.
- Robot yang diam (SIAP / SELESAI / MENUNGGU) cukup diwakili badannya + `d_min`.

### 4. Aturan "akan menabrak"

Robot A **harus mengalah** bila bayangan A bersinggungan dengan badan B (atau bayangan B), dengan pengecualian prioritas di bawah.

**Aturan prioritas** (memecah konflik simetris):
1. Robot yang sedang **mundur / manuver rapat** didahulukan, karena berhenti di tengah manuver meninggalkan badan melintang di jalur.
2. Kalau sama-sama maju: robot yang **lebih dekat ke tujuannya** didahulukan (cepat keluar dari arena bersama).
3. Seri: **ID lebih kecil** didahulukan.

Hanya robot yang kalah prioritas yang masuk MENUNGGU. Yang menang tetap jalan, dan bayangannya terus diperiksa; kalau badan kedua robot bersinggungan, keduanya estop.

### 5. State baru: MENUNGGU (bukan estop)

Estop di `navigasi.py` membawa state ke BAHAYA, dan itu **menghapus jalur, pelacak, dan tujuan**. Robot tidak bisa "melanjutkan navigasi". Estop juga memutus arus motor.

```
MELACAK ──(bayangan kena, kalah prioritas)──> MENUNGGU
MENUNGGU ──(bayangan pada v_tujuan bebas selama 0,5 s)──> MELACAK
MENUNGGU ──(menunggu > T_TUNGGU, mis. 5 s)──> MERENCANA (lawan = rintangan)
apa pun ──(BADAN kedua robot bersinggungan)──> BAHAYA (estop)
```

- MENUNGGU: jalur & pelacak **disimpan**, `rc(0,0)` tetap dikirim tiap siklus (firmware butuh aliran rc), perlambatan dengan `stop`, bukan `estop`.
- **Histeresis**: syarat lanjut memakai bayangan pada kecepatan **tujuan** (bukan kecepatan sekarang yang 0), dan harus bebas selama 0,5 s. Tanpa ini robot akan maju-berhenti berulang: diam → bayangan pendek → jalan → bayangan memanjang → kena lagi.
- Estop tetap ada sebagai jaring pengaman terakhir.

### 6. Robot lawan sebagai rintangan

"Navigasi bebas, boleh cross path" tetap dipakai untuk robot yang **bergerak**. Tapi:
- Robot yang **diam** (SIAP / SELESAI) dimasukkan ke peta rintangan saat merencanakan robot lain, badannya + `SAFETY_MARGIN_CM`. Kalau tidak, perencana bisa membuat jalur atau menerima tujuan tepat di atas robot yang sedang parkir, dan robot lain menunggu selamanya.
- Tujuan yang diklik ditolak kalau bertumpuk dengan badan robot lain.
- Setelah `T_TUNGGU` di MENUNGGU: rencanakan ulang dengan lawan sebagai rintangan statis (pemecah deadlock terakhir).

### 7. Kasus yang harus lolos uji

| # | Skenario | Hasil yang diharapkan |
|---|---|---|
| 1 | Beriring searah, B di depan lebih lambat | A menunggu, lanjut setelah B menjauh |
| 2 | Berhadapan di koridor | Satu menunggu (prioritas), lalu replan setelah `T_TUNGGU` |
| 3 | Bersilangan tegak lurus di persimpangan | Satu lewat, satu menunggu, tidak ada deadlock |
| 4 | B parkir (SELESAI) di jalur A | A replan memutari B |
| 5 | Tujuan A diklik di atas B | Ditolak di `_terima_tujuan` |
| 6 | Tag B hilang > `RAMAL_MAKS_S` | A ikut MENUNGGU (posisi B tidak diketahui) |
| 7 | A di tengah manuver mundur, B mendekat | B yang menunggu |
| 8 | Lanjut dari MENUNGGU | Tidak ada maju-berhenti berulang |

Semua diuji dulu di `--simulasi` (dua `SumberSimulasi`, derau pose 1,2 cm), di peta Rumah yang tersempit, baru di robot.

### 8. Urutan pengerjaan

1. Konstanta per robot menjadi atribut objek (tidak lagi global `konfig`).
2. `SumberKamera` membaca beberapa tag dari satu frame.
3. `navigasi.py` memegang dua robot + UI pilih robot aktif; simulasi dua robot.
4. Wasit: bayangan sepanjang jalur + rumus `L_bayangan` + state MENUNGGU + histeresis.
5. Prioritas + `T_TUNGGU` → replan dengan lawan sebagai rintangan.
6. Robot diam sebagai rintangan saat perencanaan & saat menerima tujuan.
7. Ukur `T_reaksi` dan `a_rem` di robot nyata (`--log`), lalu setel ulang rumus bayangan.

### Catatan kecil
- Nama berkasnya `navigasi.py`, bukan `navigation.py`.
- ID 02 punya kemudi maksimum 23,5° (ID 01: 35°), jadi radius beloknya lebih besar dan sapuan bayangannya di tikungan lebih lebar. Hitung per robot, jangan pakai satu nilai.

---

## Status implementasi (2026-09-30)

Langkah 1–6 dari urutan pengerjaan **sudah dikerjakan dan diuji di simulasi**. Langkah 7 (ukur di robot nyata) belum.

```
python navigasi.py --simulasi --id 1,2
python navigasi.py --peta Rumah --id 1,2
python wasit.py          # uji aturan wasit tanpa robot
```

| Usulan | Di kode |
|---|---|
| Konstanta per robot | `konfig.profil(id)`; dipakai `Robot`, `SumberKamera`, `SumberSimulasi`, `Perencana`, `state_space.turunan` |
| Satu kamera | `pose.KameraBersama` (satu RTSP, satu deteksi per frame) |
| Wasit + bayangan sepanjang jalur | `wasit.py` (baru); `jejak` mendapat `sisa_jalur()` |
| MENUNGGU + histeresis | `navigasi.Unit.terapkan()`, bayangan robot yang menunggu dihitung pada laju nominal |
| Prioritas | dikunci per pasangan selama konflik berlangsung (mencegah pemenang bertukar) |
| Robot diam sebagai rintangan | `Perencana.rencanakan(..., rintangan_cm)`, penolakan tujuan di `terima_tujuan` |
| Replan setelah menunggu | 5 s; 1 s untuk yang kalah saat berhadapan |

**Tambahan di luar usulan**, ditemukan lewat simulasi:

- **MINGGIR.** "Replan dengan lawan sebagai rintangan" gagal total saat dua robot *bertukar tempat*: tujuan masing-masing tertutup badan lawan. Keduanya menunggu 85 detik penuh. Solusinya, robot yang gagal memutar menepi ke pose di luar sisa jalur lawan, menunggu lawan diam, lalu lanjut. Pose minggir hanya diterima kalau dari sana tujuannya sendiri masih tercapai setelah lawan parkir. Tanpa syarat ini, robot minggir tepat di depan tujuan lawan dan terjebak.
- Robot yang sedang minggir **didahulukan** (prioritas 0), dan lawannya menunggu tanpa replan. Tanpa itu keduanya sama-sama minggir dan berputar-putar.
- Dua robot tidak boleh replan pada siklus yang sama. Masing-masing akan memutari posisi lawan yang sesaat lagi berubah.
- Jalur **opsional** (memutar/mengganti) hanya diterima dengan margin penuh 8 cm. Dengan margin 6 cm, jalan memutar 4–7 m di Rumah memicu estop zona karena derau.
- Bug lama ikut diperbaiki: hasil perencanaan yang datang setelah tujuan dibatalkan (klik kanan/estop) dulu tetap dipasang, dan robot berangkat ke tujuan yang sudah dibatalkan.

**Hasil simulasi** (derau 1,2 cm, 3 seed per skenario):

| Skenario | Lolos |
|---|---|
| Silang (Kalibrasi, Rumah) | 6/6 |
| Berhadapan / bertukar tempat (Kalibrasi) | 3/3 |
| Robot parkir di jalur | 3/3 |
| Berhadapan di koridor 38 cm (Rumah) | 1/3 |

Kontak 0, badan sebenarnya masuk zona 0. Dua kegagalan koridor adalah estop zona karena pose terukur berderau saat manuver panjang di sudut sempit, bukan tabrakan antar-robot. Koridor itu lebih sempit dari dua badan robot.

**Masih terbuka:**
- Uji di robot nyata. Ukur `MULTI_T_REAKSI_S` (0,2 s masih perkiraan) dari log dua robot.
- Kasus 6 (tag hilang) sudah ditangani di wasit (robot lain menunggu tanpa batas) tapi belum diuji di simulasi.
- Bertukar tempat di koridor sempit lambat (~2 menit). Kalau sering terjadi, pertimbangkan aturan koridor satu arah di peta.
## Wasit: aturan zona (2026-11-08)

Commit `80368bf` (branch `ver-1-6`). Robot yang **akan** menyentuh zona sekarang berhenti dan membuat jalur baru, bukan menunggu sampai badannya menyentuh zona lalu di-estop.

**Cara kerja (`wasit.Wasit._cek_zona`):**
- Bayangan badan sepanjang jarak henti `L` (rumus yang sama dengan aturan antar-robot) digeser dan diputar mengikuti simpangan badan dari jalurnya (`ikut_badan`).
- Bayangan itu diuji terhadap semua zona peta (larangan dan batas) memakai `area_constraint.jarak_zona`, rumus yang sama dengan pemantau keselamatan. Fungsi jaraknya diberikan oleh `navigasi.py`, jadi wasit tetap tidak bergantung pada modul kamera.
- Bila bayangan **mendekat** ke zona sampai jarak bebas ≤ `ZONA_JARAK_AMAN_CM` (1 cm), keluarlah keputusan **ZONA**. "Mendekat" berarti lebih dekat dari badan sekarang, dengan kelonggaran derau 0,5 cm, sehingga robot yang berjalan sejajar zona tidak dihentikan.
- `navigasi._henti_zona`: roda langsung nol, jalur yang menuju zona dibuang, dan jalur baru ke tujuan yang sama dihitung dari pose sekarang. Robot masih di luar zona, jadi perencana bisa.
- Wasit kini dipanggil juga untuk **satu robot**. Urutan kepentingan: KONTAK > ZONA > TUNGGU.
- Estop pemantau keselamatan tetap ada sebagai lapis terakhir.
- `konfig.WASIT_ZONA = False` mematikan aturan ini, hanya untuk pembanding.

**Uji mandiri:** `python wasit.py` lolos 18/18 kasus (7 kasus zona baru), di Mac dan di laptop robot.

**Simulasi navigasi** (`python -m drl.uji_navigasi --id 1 --tanpa-pasangan --perencana astar,dqfd-murni --n 3`, 15 rute ID 01 per perencana):

| Derau pose | Aturan zona | Tiba | Badan sebenarnya masuk zona | Estop zona (pose terukur) | Wasit menghentikan karena zona |
|---|---|---:|---:|---:|---:|
| 1,2 cm | ada | 29/30 | 0 | 0 | 0 |
| 3,0 cm | ada | **28/30** | 0 | **2** | 3 (2 robot lalu tiba) |
| 3,0 cm | tidak | 25/30 | 0 | 5 | – |

**Catatan hasil:**
- Pada derau 1,2 cm aturan ini tidak pernah memicu, jadi tidak ada alarm palsu. Satu rute yang gagal adalah jalur backtracking DQfD sepanjang 21 m yang habis waktu, bukan masalah zona.
- Pada derau 3 cm, rute Rumah #2588 berakhir estop di zona Z9 tanpa aturan zona (dengan astar maupun DQfD). Dengan aturan zona, robot berhenti saat bayangannya 0,7 cm dari Z9, membuat jalur baru, lalu tiba.
- **Yang tidak bisa dicegah:** lompatan pose terukur karena derau yang lebih besar dari jarak bebas robot ke zona. Contohnya Rumah #1713 (estop setelah berhenti) dan Rumah2 #2595 (estop tanpa peringatan).
- Sampelnya kecil dan derau setiap jalan acak. Untuk angka yang kokoh, perlu ulangan dengan derau yang sama di kedua kondisi.
