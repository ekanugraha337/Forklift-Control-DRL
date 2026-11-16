# Uji robot nyata paper ver9 (validasi kelayakan, peta lab Gudang2)

Tujuan: menunjukkan bahwa jalur DQfD dan Hybrid A* + RS standar bisa dikendarai robot nyata
(tiba, tidak masuk zona, tidak bersentuhan), dan mengukur waktu perencanaan di laptop robot.
Bukan bukti keunggulan: rute lab pendek.

## Rancangan

- Peta: `Gudang2` (Warehouse2 di Gambar 1 paper), zona virtual.
- Skenario: `uji_nyata/lab_Gudang2.json`, dibuat oleh `uji_nyata/paper_ver9/buat_skenario_lab.py`
  (kriteria pemilihan dan waktu di Mac: `uji_nyata/lab_Gudang2_pemilihan.json`).
  L1 dan L2 = jalur terpisah, LX3 = jalur bersilangan.
- Perencana: `astar-standar` (Hybrid A* + RS di paper) dan `dqfd-murni` (DQfD di paper, tanpa cadangan A*).
- Model DQfD: `model_dqn_gudang3` (model akhir paper), DQfD **tanpa P_col**.
- 3 ulangan per perencana per skenario, urutan diselang-seling (bawaan untuk robot nyata).
  Total 18 percobaan, 36 perjalanan robot, sekitar 45 menit termasuk pulang.

## Sebelum mulai

1. Baterai kedua robot penuh. Kamera dan kalibrasi peta Gudang2 seperti biasa.
2. Di laptop robot, folder `Forklift Robot Program`, PowerShell:

```powershell
git pull                      # branch ver-1-6: skenario dan skrip terbaru
$env:FORKLIFT_TANPA_P = "1"                    # WAJIB: tanpa P_col, sesuai paper
$env:FORKLIFT_DIR_MODEL = "model_dqn_gudang3"  # WAJIB: model akhir paper
python -c "from drl import perencana_dqfd as PD; print('tanpa P:', PD.TANPA_P, '| model:', PD.DIR_MODEL)"
```

Baris terakhir harus mencetak `tanpa P: True | model: ...model_dqn_gudang3`. Kalau tidak, jangan mulai.
Variabel `$env:` hanya berlaku di jendela PowerShell itu: jalankan uji di jendela yang sama.

3. Gladi bersih singkat di laptop robot (tanpa robot bergerak, sekitar 5 menit):

```powershell
python -m drl.uji_robot_nyata --skenario uji_nyata/lab_Gudang2.json --simulasi --tanpa-konfirmasi --derau 1.2 --perencana astar-standar,dqfd-murni --ulang 1
```

## Uji

```powershell
python -m drl.uji_robot_nyata --skenario uji_nyata/lab_Gudang2.json --perencana astar-standar,dqfd-murni --ulang 3
```

Jangan pakai `--tanpa-konfirmasi` di robot nyata. Tombol: `n` lanjut/konfirmasi, `p` coba pulang lagi,
`spasi` tahan semua, `e` estop, `x` lewati percobaan, `q` keluar (hasil tersimpan).

Pulang ke pose awal bisa gagal masuk toleransi (5 cm dan 10 deg) setelah 3 percobaan; program lalu
menunggu operator. Di gladi simulasi (derau 1,2 cm) ini terjadi sekali sebelum L2 ulangan 3: ID 01
meleset sekitar 11 deg, ID 02 sekitar 5 cm. Kalau robot hanya meleset sedikit, tekan `n` (pose awal
sebenarnya tetap diukur dan dicatat sebagai galat awal). Kalau jauh, geser robot dengan tangan lalu `p`.
Kalau terputus, lanjutkan tanpa mengulang yang sudah tercatat:

```powershell
python -m drl.uji_robot_nyata --skenario uji_nyata/lab_Gudang2.json --perencana astar-standar,dqfd-murni --ulang 3 --lanjut uji_nyata/<folder hasil>
```

## Sesudah uji

```powershell
python -m drl.uji_robot_nyata --ringkas uji_nyata/<folder hasil>
```

Kirim folder `uji_nyata/<folder hasil>` ke Mac (commit + push, atau salin). Ringkasan untuk paper:

```bash
cd "Forklift Robot Program" && ../.venv/bin/python uji_nyata/paper_ver9/sim5.py uji_nyata/<folder hasil>
```

Catat juga: nama CPU laptop robot, kejadian estop atau dorongan tangan, dan apa pun yang tidak biasa.
