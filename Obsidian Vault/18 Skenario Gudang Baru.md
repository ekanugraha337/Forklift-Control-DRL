# Skenario Gudang baru (homografi ulang)

Tanggal: 2026-11-04, diperbarui 2026-11-05 · Branch `ver-1-5`, kode commit `4d26e77` · Model: [[07 Struktur DQfD Full|Struktur DQfD Full]]

Homografi kamera diulang, lalu skenario uji Gudang dibuat dari awal dengan `drl.uji_robot_nyata --buat`. Skenario lama dihapus di commit `1f029d3`. Pada 2026-11-05 ditambahkan S3 dan S4, dua skenario yang memaksa robot **pindah lorong lewat celah di ujung dinding**. Catatan ini merekam bentuk keempat skenario dan metode ukurnya.

> [!warning] Dibersihkan 2026-11-07 (aturan ver-1-6)
> Semua hasil simulasi dan uji robot nyata (makespan, waktu proses, galat pose akhir, panjang jalur) dihapus, karena diukur dengan Hybrid A\* + RS dan DQfD dua model aturan lama. Peta, skenario, dan metode ukur tetap berlaku.

---

## 1. Persiapan skenario

### 1.1 Homografi dan peta
- Homografi diulang dengan `python cctv_lurus.py homografi`. Kotaknya tetap 8 × 7 ubin (240 × 210 cm) dengan titik asal yang sama, jadi koordinat zona di `Camera/peta_constraint/Gudang.json` tetap berlaku. Hasilnya ada di `Camera/kalibrasi_lensa.json`.
- Zona peta Gudang **belum** ditempelkan ke garis ubin. Sisi zona masih meleset 0,5–3 cm dari garis ubin 30 cm (Z3, Z2, Z4, Z6, Z7). Lihat bagian 5.

![[Peta Gudang 2026-11-04.png]]

Gambar di atas menampilkan zona larangan (merah), grid ubin 30 cm (cyan), dan lebar lorong. Lorong atas dibatasi Z6 dan Z5 (ujung y = 55 dan 50,5). Lorong bawah dibatasi Z7 dan Z8 (ujung y = 144,5 dan 134). Ruang tengah, y ≈ 55–134, terbuka selebar arena.

### 1.2 Skenario (`Forklift Robot Program/uji_nyata/Gudang.json`)

Pose (x cm, y cm, yaw °). Titik acuan pose adalah poros depan. Yaw 90° berarti menghadap +y (ke bawah pada gambar).

| Skenario | Robot | Awal | Tujuan | Gerak |
|---|---|---|---|---|
| S1 | 01 | (32,00; 40,0; 90) | (108,25; 169,8; 90) | lorong kiri-atas → lorong tengah-bawah |
| S1 | 02 | (107,75; 40,0; 90) | (196,25; 167,4; 90) | lorong tengah-atas → lorong kanan-bawah |
| S2 | 01 | (107,75; 33,8; −90) | (108,25; 174,5; 90) | lorong tengah, putar balik 180° |
| S2 | 02 | (195,75; 26,0; −90) | (196,25; 164,4; 90) | lorong kanan, putar balik 180° |
| S3 | 01 | (32,00; 40,0; 90) | (107,75; 40,0; −90) | kiri-atas → tengah-atas, mengitari ujung bawah **Z6** |
| S3 | 02 | (108,25; 170,0; −90) | (196,25; 170,0; 90) | tengah-bawah → kanan-bawah, mengitari ujung atas **Z8** |
| S4 | 01 | (108,25; 170,0; −90) | (31,75; 170,0; 90) | tengah-bawah → kiri-bawah, mengitari ujung atas **Z7** |
| S4 | 02 | (195,75; 40,0; 90) | (107,75; 40,0; −90) | kanan-atas → tengah-atas, mengitari ujung bawah **Z5** |

Penyesuaian S1 dan S2 setelah dibuat dengan klik:
1. **Yaw dibulatkan ke kelipatan 90°** supaya lurus terhadap dinding zona. Sebelumnya −95,7° sampai −86,1°.
2. **x dipindah ke tengah lorong**, yaitu titik tengah antara dua dinding yang rentang y-nya bersinggungan dengan badan robot. Lorongnya: Z3–Z6 (62 cm), Z6–Z5 (82,5 cm), Z7–Z8 (81,5 cm), Z5–Z2 (85,5 cm), Z8–Z2 (84,5 cm).
3. **y awal S1 dinaikkan ke 40,0** (sebelumnya 34,5 dan 31,1). Lihat bagian 1.4.
4. **Arah tujuan S2 dibalik** dari −90° ke 90°, supaya robot harus putar balik di lorong.

**Rancangan S3 dan S4 (pindah lorong lewat celah di ujung dinding):**
- Setiap robot pindah ke lorong sebelah dengan **mengitari ujung dinding**. Tidak ada rute lurus: robot harus keluar dari lorongnya ke pita tengah, berbalik arah 180°, lalu masuk ke lorong sebelah.
- **Seimbang antar-robot.** Di S3, robot 01 mengitari ujung dinding yang menggantung dari atas (Z6) dan robot 02 ujung dinding yang berdiri dari bawah (Z8). Di S4 peran keduanya ditukar (Z7 dan Z5).
- **Tanpa interaksi.** Pita tengah Gudang hanya ±79 cm, sedangkan satu putaran di ujung dinding memakan ±40 cm, sehingga jarak sapuan kedua robot hanya 8–27 cm. Karena itu di setiap skenario satu robot **berakhir** di lorong tengah dan robot lainnya **berangkat** dari lorong tengah. Kedekatan di kolom tengah menjadi terpisah waktu. Perkiraan jarak badan terdekat pada waktu yang sama (laju sama, berangkat bersamaan): 27–63 cm. Terukur di simulasi: 30,5–71,3 cm (bagian 3).
- Pose di tengah lorong dengan yaw ±90°, sama seperti S1 dan S2.

Setiap pose diperiksa **sebagai tujuan** untuk kedua perencana, dan semuanya muat dengan margin 8 cm.

### 1.3 Skenario dan heatmap Dijkstra

Panel kiri menampilkan pose awal (titik-titik) dan tujuan (garis putus) dengan panah arah hadap. Biru adalah robot 01, merah robot 02. Panel tengah dan kanan adalah **heuristik Dijkstra** ke tujuan setiap robot, yaitu biaya-sisa holonomik dalam cm. Peta ini dihitung oleh `_holonomic_heuristic` (Dijkstra 8-tetangga, sel 5 cm, sel diblokir bila jarak bebasnya < setengah lebar badan). Fungsi yang sama dipakai Hybrid A\* sebagai heuristik dan lingkungan DQfD sebagai peta Dijkstra observasi (`drl/lingkungan.py:430`), jadi satu peta per robot berlaku untuk **kedua** perencana. Kondisinya sama dengan saat uji: kedua robot merencana tanpa badan robot lain sebagai rintangan.

![[Skenario Gudang 2026-11-05 S1.png]]

![[Skenario Gudang 2026-11-05 S2.png]]

![[Skenario Gudang 2026-11-05 S3.png]]

![[Skenario Gudang 2026-11-05 S4.png]]

### 1.4 Temuan: pose awal harus muat sebagai tujuan pulang
- Pose awal S1 yang pertama, y = 34,5 dan 31,1 dengan yaw 90°, menaruh buntut badan (23 cm di belakang poros depan) 8–12 cm dari dinding Z1.
- Pose itu lolos pemeriksaan `w` di mode `--buat`, karena `periksa_skenario` hanya memeriksa **tujuan** dan merencanakan **dari** awal. Perencana sengaja longgar untuk pose awal.
- Namun saat **pulang**, pose awal menjadi tujuan dan ditolak ("tujuan terlalu rapat ke zona – badan robot tidak muat walau dengan jarak aman 6 cm"). Robot diam, dan setelah 3 kali mencoba program meminta operator.
- Batas y terkecil: **36,0 / 36,1** dengan margin 6 cm, dan **38,0 / 38,1** dengan margin 8 cm. Dipilih 40,0.

> [!tip] Untuk skenario berikutnya
> Pose awal harus juga lolos sebagai **tujuan** (`margin_untuk_pose`), karena protokol uji mengirim robot pulang ke sana sebelum setiap percobaan.

## 2. Metode pengukuran

```powershell
python -m drl.uji_robot_nyata --skenario uji_nyata/Gudang.json --simulasi --ulang 2
```

- **Protokol** (`drl/uji_robot_nyata.py`): sebelum setiap percobaan, robot pulang ke pose awal dengan Hybrid A\* (tidak diukur, toleransi 5 cm / 10°). Lalu kedua robot diberi tujuannya dengan perencana yang diuji, bersamaan. Urutan perencana diselang-seling: ulangan 1 A\* lalu DQfD, ulangan 2 DQfD lalu A\*.
- **Tujuan simetris.** Tujuan semua robot ditetapkan dulu (`terima_tujuan(tunda=True)`), baru setiap robot diminta merencana. Tidak ada robot yang merencana dengan pose awal robot lain sebagai rintangan.
- **Satu proses perencana per robot** (`perencana_proses.py`, commit `4d26e77`). Setiap robot merencana di proses dan interpreter Python sendiri, sehingga tidak berbagi GIL dengan loop kendali atau dengan perencana robot lain. Perencanaan di thread sudah dihapus, karena waktu jam dinding yang tercatat ikut memuat pekerjaan lain. `navigasi.Unit.minta_rencana()` menolak perencana yang bukan `PerencanaDiProses`.
- **Yang dicatat untuk setiap rencana** (`hasil.jsonl`):
  - `ms`: waktu jam dinding `rencanakan()` di proses perencana.
  - `cpu_ms`: waktu CPU thread perencana. Di Windows memakai `QueryThreadCycleTime`, karena `time.thread_time()` di sana hanya naik per 15,625 ms.
  - `rincian_ms`: bagian Hybrid A\*, RS, model jauh, dan lain-lain.
  - `mulai_ms`: jeda dari permintaan rencana sampai rencana mulai.
  - `ipc_ms`: biaya antar-proses.
  - `pid`: proses yang merencana.
