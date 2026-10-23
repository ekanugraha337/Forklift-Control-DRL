"""
latih.py - melatih DQfD per robot: pra-latih dari demo, lalu latihan sendiri.

    python -m drl.latih --id 1,2 --perangkat cuda               # dekat lalu jauh, ID 01 lalu ID 02
    python -m drl.latih --id 1 --model dekat --tampil           # satu model, dengan dasbor
    python -m drl.latih --id 1 --model dekat --pra 5000 --mandiri 0   # cepat, hanya pra-latih
    python -m drl.visual_neuron --putar model_dqn/id01_dekat    # putar ulang sesudahnya

DUA MODEL PER ROBOT (versi aksi 6, ver-1-4), lihat lingkungan.py - model jauh
meniru masukan & keluaran Hybrid A* (47 / 14), model dekat MEMILIH KATA
Reeds-Shepp (9 masukan RS / 88 = 44 kata x 2 titik sambung; panjang segmen dari
rumus RS; kata dicek tabrakan berurutan menurut Q). Episode model dekat = satu
aksi, imbalan = -biaya kurva (bebas) atau -biaya buntu (menabrak):
    dekat  model_dqn/id0N_dekat.pt - dari demo_dekat (+ demo_dekat_tambahan,
           + demo_dekat_dagger bila ada). Dilatih LEBIH DULU.
    jauh   model_dqn/id0N_jauh.pt - dari demo_jauh. Butuh id0N_dekat.pt.
           Imbalan di pose serah (--nilai-akhir):
             pendaratan (bawaan) - PENDARATAN SUNGGUHAN model dekat dari pose
               serah: biaya jalurnya bila berhasil, biaya buntu bila gagal.
               Demo: episode jauh diputar ulang, lalu model dekat mendarat dari
               pose akhirnya. Fase B: lingkungan jauh berhenti HANYA di pose
               tempat model dekat benar-benar bisa mendarat (sama dengan saat
               pengujian, rencanakan_dua), paling banyak --maks-pendaratan
               percobaan per episode, lalu gagal.
             v - perkiraan max_a Q model dekat di pose serah (cara lama;
               berhenti di pose tembakan RS bebas). Lebih cepat, tapi model
               dekat versi 4 yang buta rintangan sering terlalu optimis.
           Jadi model jauh belajar tiba di batas 60 cm di pose yang benar-benar
           bisa didaratkan.

FASE A - PRA-LATIH: hanya data demo (episode bukan-uji). Jaringan belajar
    meniru demonstrator (J_E) sekaligus memperkirakan biaya-sisa (J_DQ, J_n).
FASE B - LATIHAN SENDIRI: jaringan merencanakan sendiri (epsilon-greedy di
    antara aksi SAH), hasilnya masuk buffer di samping demo yang tetap ada.
    Skenario: 70% dari episode demo latih, 30% acak baru (skenario.acak_satu;
    model dekat: start di cincin 15-60 cm dari tujuan).

EVALUASI (tiap --eval-setiap langkah gradien) pada set VALIDASI - 10% episode
latih yang disisihkan, BUKAN set uji (python -m drl.evaluasi).
    dekat   tiba (<= 1 cm & 2 deg), rasio biaya vs demo, galat pose akhir,
            titik balik maju-mundur. Pencarian urutan Q.
    jauh    rute GABUNGAN jauh + dekat: tiba, gagal jauh / dekat, rasio biaya
            vs demo A* + RS seluruh rute, titik balik. Metode pencarian
            --metode-validasi, bawaan = metode operasional (gabungan).
    skor    tiba - 10 x (p90 rasio - 1), dua-duanya
Kunci riwayat "tiba_mundur" / "rasio_p90_mundur" tetap bernama begitu (riwayat
lama), isinya hasil metode validasi.

KELUARAN model_dqn/id0N_<jenis>/: ckpt_<langkah>.pt tiap evaluasi, riwayat.json;
model_dqn/id0N_<jenis>.pt = checkpoint SKOR TERBAIK, id0N_<jenis>_akhir.pt = terakhir.

Riwayat: putaran 2 (2026-09-30) menetapkan validasi + model terbaik, mode
mundur, fase B yang lebih stabil, pra-latih 20k dan latihan sendiri 150k.
Peta tidak ditulis mati: jenis peta dibaca dari dataset, PETA TERSULIT = jenis
peta asli dengan waktu A* rata-rata terbesar (--porsi-sulit, validasi terbanyak).
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import torch

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                                   # noqa: E402
from drl import lingkungan as L                 # noqa: E402
from drl import rollout as RO                   # noqa: E402
from drl import skenario as S                   # noqa: E402
from drl.dqfd import AgenDQfD, Buffer, meta_model, muat_jaringan, periksa_profil  # noqa: E402
from drl.jaringan import SKALA_REWARD, bermask  # noqa: E402
from drl.perencana_dqfd import (BATAS_DEKAT_S, MAKS_SERAH, METODE_BAWAAN, buat_cek_pendaratan,  # noqa: E402
                                jalur_model, mutu_jalur, rencanakan, rencanakan_dua)
from drl.visual_neuron import Dasbor            # noqa: E402

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIR_MODEL = os.path.join(AKAR, "model_dqn")
BATAS_LANGKAH_EVAL = 250
PORSI_DEMO_MANDIRI = 0.7
# Rute validasi per jenis peta. Peta tersulit terbanyak: di sana mutu jalur terburuk.
N_VALIDASI_JENIS = 40
N_VALIDASI_SULIT = 60
BOBOT_P90 = 10.0


def _jenis(p):
    return p.split(":")[0]


def jenis_peta(d):
    """(semua jenis peta di dataset, jenis tersulit, porsi episode per jenis)."""
    jenis = np.array([_jenis(p) for p in d["ep_peta"]])
    semua = sorted(set(jenis))
    asli = [j for j in semua if j != "sintetis"] or semua
    # nanmean: episode DAgger tidak punya waktu A* (NaN)
    w = {j: d["ep_waktu_astar"][jenis == j] for j in asli}
    sulit = max(asli, key=lambda j: float(np.nanmean(w[j])) if np.isfinite(w[j]).any() else -1.0)
    porsi = {j: float(np.mean(jenis == j)) for j in semua}
    return semua, sulit, porsi


def jumlah_validasi(d):
    semua, sulit, _ = jenis_peta(d)
    return {j: N_VALIDASI_SULIT if j == sulit else N_VALIDASI_JENIS for j in semua}


def pilih_per_jenis(d, episode, jumlah):
    """Dari himpunan episode, ambil jumlah[jenis] tiap jenis peta, urutan tetap."""
    jenis = np.array([_jenis(p) for p in d["ep_peta"]])
    episode = np.asarray(episode)
    out = []
    for j, n in jumlah.items():
        out += list(episode[jenis[episode] == j][:n])
    return out


def pilih_probe(d, episode):
    """Rute untuk dasbor: satu per jenis peta, panjangnya sedang."""
    jenis = np.array([_jenis(p) for p in d["ep_peta"]])
    lk = d["ep_langkah"]
    episode = np.asarray(episode)
    out = []
    semua, sulit, _ = jenis_peta(d)
    for j in [sulit] + [x for x in semua if x != sulit]:
        lo, hi = (12, 50) if j == sulit else (6, 40)
        idx = episode[(jenis[episode] == j) & (lk[episode] >= lo) & (lk[episode] <= hi)]
        if len(idx):
            out.append(int(idx[len(idx) // 2]))
    return out


def bagi_validasi(d):
    """(latih, validasi): setiap episode latih ke-10 disisihkan untuk validasi."""
    bukan_uji = np.flatnonzero(~d["ep_uji"])
    val = bukan_uji[::10]
    return np.setdiff1d(bukan_uji, val), val


def _p(x, q):
    return float(np.percentile(x, q)) if len(x) else float("nan")


def _rata(x):
    x = [v for v in x if v is not None]
    return float(np.mean(x)) if x else float("nan")


# ------------------------------------------------------------ model dekat -> jauh
def nilai_akhir(net_dekat, obs, mask, biaya_buntu):
    """Biaya mendarat menurut model dekat (sel) dari observasi/mask model dekat di pose serah.

    = -max_a Q x skala, dipotong >= 0. Mask kosong (model dekat tidak punya aksi
    sah di pose itu): biaya buntu - pose serah itu memang jalan buntu.
    """
    obs = torch.as_tensor(np.asarray(obs, np.float32).reshape(-1, L.n_obs("dekat")))
    mask = torch.as_tensor(np.asarray(mask, bool).reshape(len(obs), -1))
    with torch.no_grad():
        q = bermask(net_dekat(obs), mask).max(1).values.numpy()
    biaya = np.maximum(0.0, -q * SKALA_REWARD)
    return np.where(mask.numpy().any(1), biaya, biaya_buntu)


def buat_nilai_serah(net_dekat):
    """nilai_serah untuk lingkungan jauh saat latihan sendiri: model dekat di pose serah."""
    cache = {}

    def nilai(env):
        kunci = id(env)
        if kunci not in cache:
            cache[kunci] = L.LingkunganRencana(env.arena, env.p, margin_cm=env.margin_cm,
                                               planner=env.planner, jenis="dekat")
        ed = cache[kunci]
        obs, info = ed.reset((env.x, env.y, env.yaw), env.tujuan, env.omap, d0=env.d, k0=env.k)
        if obs is None:
            return 0.0 if info["alasan"] == "sudah di tujuan" else env.biaya_buntu
        return float(nilai_akhir(net_dekat, obs, ed.mask(), env.biaya_buntu)[0])

    return nilai


def nilai_akhir_pendaratan(d, episode, ulang, cek, nilai, cadangan):
    """Imbalan akhir demo jauh dari pendaratan sungguhan model dekat (sel).

    Tiap episode diputar ulang di lingkungan demonstrator (ulang: PabrikEnv
    jauh tanpa cek_serah - berhenti di pose tembakan RS bebas, persis akhir
    demo), lalu model dekat mendarat dari pose itu: biaya jalurnya bila
    berhasil, biaya buntu bila gagal. Episode yang tidak bisa diputar ulang
    memakai cadangan[e]. Kembalian (larik per episode dataset, n berhasil, n dicoba).
    """
    akhir = np.array(cadangan, dtype=np.float64)
    n_ok = n = 0
    t0 = time.time()
    for i, e in enumerate(episode):
        sk = RO.dari_demo(d, int(e))
        env, obs, _ = ulang.buat(sk)
        if obs is None:
            continue
        for a in sk.aksi_demo:
            if env.selesai:
                break
            env.step(int(a))
        if not env.sukses:
            continue
        n += 1
        if cek(env):
            n_ok += 1
            akhir[e] = nilai(env)
        else:
            akhir[e] = env.biaya_buntu
        if (i + 1) % 500 == 0:
            print(f"    pendaratan demo {i + 1}/{len(episode)}: berhasil {100.0 * n_ok / max(1, n):.1f}% "
                  f"({time.time() - t0:.0f} s)", flush=True)
    return akhir, n_ok, n


def bobot_awal(agen, jalur, jenis, prof, device):
    """Mulai latihan dari checkpoint lain berjenis sama (--awal).

    Bila checkpoint itu punya masukan LEBIH SEDIKIT - satu-satunya yang dikenal:
    model dekat versi 4, 9 masukan = indeks 0-8 versi 5 (identik, diperiksa di
    dataset) - bobot FC1-nya disalin ke kolom pertama, dan kolom masukan baru
    (sinar & ruang manuver) diberi bobot NOL. Di awal latihan jaringan menghitung
    Q persis seperti model lama; masukan baru baru dipakai sejauh menurunkan loss.
    Model terbaik dipilih dari validasi, termasuk evaluasi "awal" di langkah 0,
    jadi hasil latihan tidak bisa lebih buruk dari model lama di set validasi.
    """
    if not os.path.exists(jalur):
        raise SystemExit(f"[!] --awal: {jalur} tidak ada")
    ck = torch.load(jalur, map_location=device, weights_only=False)
    if ck.get("jenis") != jenis or ck.get("n_aksi") != L.n_aksi(jenis):
        raise SystemExit(f"[!] --awal: {jalur} adalah model '{ck.get('jenis')}' dengan {ck.get('n_aksi')} aksi, "
                         f"bukan model {jenis} dengan {L.n_aksi(jenis)} aksi")
    periksa_profil(ck, prof, jalur)
    lama, baru = ck["n_obs"], L.n_obs(jenis)
    sd = {k: v.clone() for k, v in ck["net"].items()}
    if lama != baru:
        if not (jenis == "dekat" and lama == 9 and baru > lama):
            raise SystemExit(f"[!] --awal: {jalur} punya {lama} masukan, program {baru}; pemetaan masukannya "
                             f"tidak dikenal (hanya model dekat 9 masukan RS -> lebih banyak)")
        w = sd["fc1.weight"]
        sd["fc1.weight"] = torch.cat([w, torch.zeros((w.shape[0], baru - lama), dtype=w.dtype, device=w.device)], 1)
    agen.net.load_state_dict(sd)
    agen.target.load_state_dict(sd)
    agen.sinkron_cpu()
    return lama, baru, ck.get("langkah")


# ------------------------------------------------------------ evaluasi
class Evaluator:
    """Evaluasi pada himpunan episode yang diberikan (validasi saat latih).

    jenis 'dekat': pendaratan model dekat sendirian dari pose awal episode dekat.
    jenis 'jauh': rute GABUNGAN - model jauh (yang dinilai) + net_dekat,
    pencarian `metode` (rencanakan_dua).
    """

    def __init__(self, d, neg, pabrik, episode, n_negatif, jenis, net_dekat=None, anggaran=600,
                 metode=METODE_BAWAAN):
        self.pabrik = pabrik
        self.metode = metode
        self.anggaran = anggaran
        self.jenis_model = jenis
        self.net_dekat = net_dekat
        self.jenis, self.sulit, _ = jenis_peta(d)
        self.sk = [RO.dari_demo(d, e) for e in pilih_per_jenis(d, episode, jumlah_validasi(d))]
        idx = np.concatenate([np.arange(d["ep_mulai"][e], d["ep_mulai"][e] + d["ep_langkah"][e])
                              for e in episode])
        self.obs_uji = torch.as_tensor(d["obs"][idx])
        self.mask_uji = torch.as_tensor(d["mask"][idx])
        self.aksi_uji = torch.as_tensor(d["aksi"][idx].astype(np.int64))
        self.neg = []
        if neg is not None and jenis == "jauh":
            ok = np.flatnonzero(neg["alasan"] == "tidak ada jalur")
            jn = np.array([_jenis(p) for p in neg["peta"]])
            for j in self.jenis:
                self.neg += [RO.dari_negatif(neg, i) for i in ok[jn[ok] == j][:max(1, n_negatif // 4)]]
        # Pembanding: titik balik demo di jalur jenis ini. Diputar ulang di
        # lingkungan demonstrator (tanpa cek_serah model dekat).
        ulang = RO.PabrikEnv(pabrik.p, jenis)
        self.balik_demo = {}
        for sk in self.sk:
            env, obs, _ = ulang.buat(sk)
            if obs is None:
                continue
            for a in sk.aksi_demo:
                env.step(int(a))
            self.balik_demo[sk.nama] = mutu_jalur(env.jalur(), env.tujuan, env.radius_serah, 1.0)["ganti_arah"]
        sulit = [s for s in self.sk if _jenis(s.peta) == self.sulit][:8]
        self.sk_waktu = sulit

    def _lingkungan(self, sk, jenis):
        return L.LingkunganRencana(self.pabrik.arena(sk.peta), self.pabrik.p, margin_cm=sk.margin,
                                   jenis=jenis)

    def __call__(self, net):
        with torch.no_grad():
            q = bermask(net(self.obs_uji), self.mask_uji)
            setuju = float((q.argmax(1) == self.aksi_uji).float().mean()) * 100
        h = self._jalankan(net, self.sk)
        tiba_greedy = 100.0 * sum(x["sukses"] for x in h) / max(1, len(h))
        hn = self._jalankan(net, self.neg) if self.neg else []
        t0 = time.perf_counter()
        for s in self.sk_waktu:
            self._jalankan(net, [s], n_slot=1)
        ms = (time.perf_counter() - t0) / max(1, len(self.sk_waktu)) * 1000

        hm = [self._satu(net, sk) for sk in self.sk]
        ok = [x for x in hm if x["sukses"]]
        rasio = [x["rasio"] for x in ok if x["rasio"]]
        tiba = 100.0 * len(ok) / max(1, len(hm))
        p90 = _p(rasio, 90)
        per_jenis = {j: 100.0 * sum(x["sukses"] for x in hm if _jenis(x["sk"].peta) == j)
                     / max(1, sum(_jenis(x["sk"].peta) == j for x in hm)) for j in self.jenis}
        ev = {
            "tiba_mundur": tiba, "rasio_p90_mundur": p90, "rasio_median_mundur": _p(rasio, 50),
            "skor": tiba - BOBOT_P90 * (p90 - 1.0) if rasio else -1e9,
            "setuju": setuju, "tiba": tiba_greedy, "ms_dqfd": ms, "tiba_per_jenis": per_jenis,
            "balik": _rata([x["ganti_arah"] for x in ok]),
            "balik_demo": _rata([self.balik_demo.get(x["sk"].nama) for x in ok]),
            "galat_posisi_cm": _rata([x["galat_posisi_cm"] for x in ok]),
            "galat_arah_deg": _rata([x["galat_arah_deg"] for x in ok]),
            "galat_arah_p90": _p([x["galat_arah_deg"] for x in ok if x["galat_arah_deg"] is not None], 90),
        }
        if self.jenis_model == "jauh":
            ev.update(
                gagal_jauh=100.0 * sum(x.get("fase") == "gagal_jauh" for x in hm) / max(1, len(hm)),
                gagal_dekat=100.0 * sum(x.get("fase") == "gagal_dekat" for x in hm) / max(1, len(hm)),
                balik_dekat=_rata([x["ganti_arah_dekat"] for x in ok]),
                neg_buntu=100.0 * sum(not x["sukses"] for x in hn) / max(1, len(hn)))
        return ev

    def _satu(self, net, sk):
        """Perencana operasional (mundur saat buntu) satu skenario."""
        cm = self.pabrik.arena(sk.peta).panjang_ke_cm(1.0)
        om = self.pabrik.omap(sk)
        if self.jenis_model == "dekat":
            env = self._lingkungan(sk, "dekat")
            obs, _ = env.reset(sk.awal, sk.tujuan, om, d0=sk.d0, k0=sk.k0)
            if obs is None:
                return {"sk": sk, "sukses": False}
            r = rencanakan(net, env, mundur=True, anggaran=self.anggaran)
            if not r["sukses"]:
                return {"sk": sk, "sukses": False}
            return {"sk": sk, "sukses": True, "rasio": env.biaya / sk.biaya_astar if sk.biaya_astar else None,
                    **mutu_jalur(env.jalur(), sk.tujuan, env.radius_serah, cm)}
        ej, ed = self._lingkungan(sk, "jauh"), self._lingkungan(sk, "dekat")
        r = rencanakan_dua(net, self.net_dekat, ej, ed, sk.awal, sk.tujuan, om, metode=self.metode)
        if not r["sukses"] or not r["jalur"]:
            return {"sk": sk, "sukses": False, "fase": r["fase"]}
        acuan = sk.biaya_astar_total or None
        return {"sk": sk, "sukses": True, "fase": "tiba", "rasio": r["biaya"] / acuan if acuan else None,
                **mutu_jalur(r["jalur"], sk.tujuan, ej.radius_serah, cm)}

    def _jalankan(self, net, daftar, n_slot=32):
        k = RO.KumpulanRollout(self.pabrik, n_slot)
        hasil, antre, milik = [None] * len(daftar), list(range(len(daftar))), {}
        while antre or k.aktif():
            for i in range(n_slot):
                while k.slot[i] is None and antre:
                    j = antre.pop(0)
                    ok, alasan = k.isi(i, daftar[j])
                    if ok:
                        milik[i] = j
                    else:
                        hasil[j] = {"sk": daftar[j], "sukses": False, "alasan": alasan, "rasio": None}
            for i, ep in k.langkah(net):
                hasil[milik.pop(i)] = ep
            # batas langkah evaluasi: potong episode yang berputar-putar
            for i in k.aktif():
                if len(k.slot[i]["aksi"]) >= BATAS_LANGKAH_EVAL:
                    hasil[milik.pop(i)] = {"sk": k.slot[i]["sk"], "sukses": False,
                                           "alasan": "batas langkah", "rasio": None}
                    k.slot[i] = None
        return hasil


def baris_eval(jenis, ev):
    if jenis == "dekat":
        return (f"setuju {ev['setuju']:5.1f}%  tiba greedy {ev['tiba']:5.1f}% / mundur {ev['tiba_mundur']:5.1f}%  "
                f"rasio p90 {ev['rasio_p90_mundur']:.2f}  galat akhir {ev['galat_posisi_cm']:.2f} cm / "
                f"{ev['galat_arah_deg']:.2f} deg  balik {ev['balik']:.2f} (demo {ev['balik_demo']:.2f})  "
                f"skor {ev['skor']:6.1f}")
    return (f"setuju {ev['setuju']:5.1f}%  serah greedy {ev['tiba']:5.1f}%  TIBA gabungan {ev['tiba_mundur']:5.1f}%  "
            f"gagal jauh/dekat {ev['gagal_jauh']:4.1f}/{ev['gagal_dekat']:4.1f}%  rasio p90 {ev['rasio_p90_mundur']:.2f}  "
            f"balik {ev['balik']:.2f} (<60cm {ev['balik_dekat']:.2f})  galat akhir {ev['galat_posisi_cm']:.2f} cm / "
            f"{ev['galat_arah_deg']:.2f} deg  skor {ev['skor']:6.1f}")


# ------------------------------------------------------------ latih
def muat_data(args, rid, jenis):
    berkas = S.berkas_demo(args.data, rid, jenis)
    if not berkas:
        raise SystemExit(f"[!] Dataset model {jenis} ID {rid:02d} tidak ada di {args.data}.\n"
                         f"    Bangkitkan dulu: python -m drl.skenario --id {rid} --n 5000"
                         + (" (lalu --hanya-dekat untuk demo dekat tambahan)" if jenis == "dekat" else ""))
    d = RO.muat_gabung(berkas)
    jalur_neg = os.path.join(args.data, f"negatif_id{rid:02d}.npz")
    neg = RO.muat_npz(jalur_neg) if jenis == "jauh" and os.path.exists(jalur_neg) else None
    return d, neg, berkas


def latih_model(rid, jenis, args, device):
    prof = konfig.profil(rid)
    n_aksi = L.n_aksi(jenis)
    d, neg, berkas = muat_data(args, rid, jenis)
    nama = f"id{rid:02d}_{jenis}"
    dir_keluar = os.path.join(args.keluar, nama)
    os.makedirs(dir_keluar, exist_ok=True)
    for f in os.listdir(dir_keluar):
        if f.startswith("ckpt_") and f.endswith(".pt"):
            os.remove(os.path.join(dir_keluar, f))
    rng = np.random.default_rng([args.seed, rid, len(jenis)])

    net_dekat = None
    if jenis == "jauh":
        jd = jalur_model(rid, "dekat", args.keluar)
        if not os.path.exists(jd):
            raise SystemExit(f"[!] Model jauh butuh model dekat yang sudah dilatih: {jd}\n"
                             f"    Latih dulu: python -m drl.latih --id {rid} --model dekat")
        net_dekat, _ = muat_jaringan(jd, jenis="dekat")
        if args.nilai_akhir == "pendaratan":
            cek, nilai = buat_cek_pendaratan(net_dekat, args.metode_pendaratan, args.batas_pendaratan,
                                             args.maks_pendaratan)
            pabrik = RO.PabrikEnv(prof, "jauh", nilai_serah=nilai, cek_serah=cek)
        else:
            pabrik = RO.PabrikEnv(prof, "jauh", nilai_serah=buat_nilai_serah(net_dekat))
    else:
        pabrik = RO.PabrikEnv(prof, "dekat", peluang_dekat=1.0)

    latih_ep, val_ep = bagi_validasi(d)
    n_demo = int(d["ep_langkah"][latih_ep].sum())
    buf = Buffer(n_demo + args.kapasitas, device, n_aksi, L.n_obs(jenis))
    akhir_jauh, info_akhir = None, ""
    if jenis == "jauh":
        # Imbalan di pose serah dari model dekat (dataset menyimpan reward TANPA nilai ini).
        biaya_buntu = pabrik.arena(str(d["ep_peta"][0])).panjang_ke_sel(L.BIAYA_BUNTU_CM)
        akhir_jauh = nilai_akhir(net_dekat, d["ep_obs_serah"], d["ep_mask_serah"], biaya_buntu)
        if args.nilai_akhir == "pendaratan":
            print(f"[i] ID {rid:02d} jauh: model dekat mendarat dari akhir {len(latih_ep)} demo "
                  f"({args.metode_pendaratan}, maks {args.batas_pendaratan} s) ...", flush=True)
            t = time.time()
            akhir_jauh, n_ok, n_coba = nilai_akhir_pendaratan(
                d, latih_ep, RO.PabrikEnv(prof, "jauh"), cek, nilai, akhir_jauh)
            info_akhir = (f"pendaratan model dekat berhasil di {n_ok}/{n_coba} pose akhir demo "
                          f"({100.0 * n_ok / max(1, n_coba):.1f}%, {time.time() - t:.0f} s); gagal = biaya buntu")
    for e in latih_ep:
        obs, aksi, rew, mask = RO.episode_demo(d, e)
        if akhir_jauh is not None:
            rew = rew.copy()
            rew[-1] -= akhir_jauh[e] / SKALA_REWARD
        buf.tambah(RO.transisi_episode(obs, aksi, rew, mask, True, args.gamma, args.n), demo=True)

    agen = AgenDQfD(device, n_aksi, L.n_obs(jenis), lr=args.lr, gamma=args.gamma, n=args.n, margin=args.margin,
                    target_setiap=args.target_setiap, seed=args.seed + rid)
    if args.transfer and rid != 1:
        sumber = jalur_model(1, jenis, args.keluar)
        ck = torch.load(sumber, map_location=device, weights_only=False)
        agen.net.load_state_dict(ck["net"])
        agen.target.load_state_dict(ck["net"])
        agen.sinkron_cpu()
        print(f"[i] ID {rid:02d} {jenis}: bobot awal dari {sumber} (transfer)")
    if args.awal:
        sumber = jalur_model(rid, jenis, args.awal)
        lama, baru, langkah = bobot_awal(agen, sumber, jenis, prof, device)
        print(f"[i] ID {rid:02d} {jenis}: bobot awal dari {sumber} (langkah {langkah}, {lama} masukan"
              + (f"; {baru - lama} masukan baru berbobot nol)" if baru != lama else ")"))

    evaluator = Evaluator(d, neg, pabrik, val_ep, args.n_negatif, jenis, net_dekat, metode=args.metode_validasi)
    probe_idx = pilih_probe(d, val_ep)
    das = Dasbor(prof, pabrik, [RO.dari_demo(d, e) for e in probe_idx],
                 judul=f"DQfD {jenis} ID {rid:02d}", fps=args.fps, tampil=args.tampil)

    total = args.pra + int(args.mandiri * args.rasio)
    riw = {"latih": {k: [] for k in ("langkah", "j_dq", "j_n", "j_e", "q", "setuju_batch")},
           "eval": {"langkah": []}, "batas_fase": []}
    ema = {}
    t0 = time.time()
    st = {"fase": "pra-latih (demo)", "target": total, "riwayat": riw["latih"],
          "eval_riwayat": riw["eval"], "batas_fase": riw["batas_fase"]}
    meta = {"robot": rid, "data_demo": [os.path.basename(b) for b in berkas], "probe": probe_idx,
            "target": total, "args": vars(args), **meta_model(prof, jenis)}
    terbaik = {"skor": -float("inf"), "langkah": None}
    print(f"\n=== ID {rid:02d} model {jenis.upper()} ({L.n_obs(jenis)} masukan, {n_aksi} aksi): "
          f"{len(latih_ep)} episode demo latih "
          f"({n_demo} transisi, dari {', '.join(meta['data_demo'])}), validasi {len(val_ep)} episode "
          f"({len(evaluator.sk)} rute dievaluasi) + {len(evaluator.neg)} negatif, target {total} langkah "
          f"gradien, perangkat {device}")
    print(f"    peta: {', '.join(evaluator.jenis)} | tersulit (waktu A* terbesar): {evaluator.sulit}")
    if akhir_jauh is not None:
        print(f"    nilai akhir di pose serah ({args.nilai_akhir}): median {np.median(akhir_jauh[latih_ep]):.0f} sel, "
              f"p90 {np.percentile(akhir_jauh[latih_ep], 90):.0f}")
        if info_akhir:
            print(f"    {info_akhir}")

    def catat(m):
        for k in ("j_dq", "j_n", "j_e", "q", "setuju_batch"):
            v = m[k]
            if np.isfinite(v):
                ema[k] = v if k not in ema else 0.98 * ema[k] + 0.02 * v
        if agen.langkah % 50 == 0:
            riw["latih"]["langkah"].append(agen.langkah)
            for k in ("j_dq", "j_n", "j_e", "q", "setuju_batch"):
                riw["latih"][k].append(ema.get(k, float("nan")))

    def evaluasi(fase):
        agen.sinkron_cpu()
        ev = evaluator(agen.cpu)
        riw["eval"]["langkah"].append(agen.langkah)
        for k, v in ev.items():
            if isinstance(v, (int, float)):
                riw["eval"].setdefault(k, []).append(None if not math.isfinite(v) else v)
        st["eval"] = ev
        agen.simpan(os.path.join(dir_keluar, f"ckpt_{agen.langkah:07d}.pt"), fase=fase, eval=ev, **meta)
        tanda = ""
        if ev["skor"] > terbaik["skor"]:
            terbaik.update(skor=ev["skor"], langkah=agen.langkah)
            agen.simpan(jalur_model(rid, jenis, args.keluar), fase=fase, eval=ev, terbaik=True, **meta)
            tanda = "  <- terbaik"
        st["terbaik"] = dict(terbaik)
        with open(os.path.join(dir_keluar, "riwayat.json"), "w", encoding="utf-8") as f:
            json.dump(riw, f)
        print(f"  [{(time.time() - t0) / 60:5.1f} mnt] {fase:<8} langkah {agen.langkah:>7}  "
              f"{baris_eval(jenis, ev)}  J_E {ema.get('j_e', 0):.3f}  J_n {ema.get('j_n', 0):.3f}{tanda}",
              flush=True)
        return ev

    def tampil():
        dt = time.time() - t0
        st["langkah"] = agen.langkah
        st["waktu"] = dt
        st["eta"] = dt / max(1, agen.langkah) * (total - agen.langkah)
        if agen.langkah % 200 == 0:
            agen.sinkron_cpu()
        return das.perbarui(agen.cpu, st)

    berhenti = None
    beta = lambda: 0.6 + 0.4 * min(1.0, agen.langkah / max(1, total))  # noqa: E731

    # ------------------------------------------------ FASE A: pra-latih
    evaluasi("awal")
    while agen.langkah < args.pra and not berhenti:
        catat(agen.latih_satu(buf, args.batch, beta(), rng))
        if agen.langkah % args.eval_setiap == 0:
            evaluasi("pra")
        berhenti = tampil()

    # ------------------------------------------------ FASE B: latihan sendiri
    if args.mandiri > 0 and not berhenti:
        riw["batas_fase"].append(agen.langkah)
        st["fase"] = "latihan sendiri"
        # Putaran 1: J_n naik terus sejak fase B, dan % tiba naik lalu turun lagi.
        # Return n-langkah dari data eksplorasi bias (off-policy), jadi bobotnya
        # dikurangi dan langkah belajar diperkecil.
        for g in agen.opt.param_groups:
            g["lr"] = args.lr_mandiri
        agen.lam_n = args.lam_n_mandiri
        agen.sinkron_cpu()
        _, peta_sulit, porsi_jenis = jenis_peta(d)
        sulit_ep = latih_ep[np.array([_jenis(p) for p in d["ep_peta"][latih_ep]]) == peta_sulit]
        kumpulan = RO.KumpulanRollout(pabrik, args.n_slot)
        pool_sintetis = [f"sintetis:{90_000_000 + rid * 100_000 + k}" for k in range(200)]
        langkah_env, utang = 0, 0.0
        tiba_ema = None

        def skenario_baru():
            sulit = rng.random() < args.porsi_sulit
            if rng.random() < PORSI_DEMO_MANDIRI:
                return RO.dari_demo(d, int(rng.choice(sulit_ep if sulit and len(sulit_ep) else latih_ep)))
            if sulit and peta_sulit != "sintetis":
                return pabrik.acak(rng, peta_sulit)
            p = porsi_jenis
            j = rng.choice(list(p), p=np.array(list(p.values())) / sum(p.values()))
            peta = pool_sintetis[int(rng.integers(len(pool_sintetis)))] if j == "sintetis" else j
            return pabrik.acak(rng, peta)

        while langkah_env < args.mandiri and not berhenti:
            for i in range(args.n_slot):
                while kumpulan.slot[i] is None:
                    sk = skenario_baru()
                    if sk is not None:
                        kumpulan.isi(i, sk)          # tidak sah sejak awal: slot tetap kosong, ambil lagi
            eps = args.eps_awal + (args.eps_akhir - args.eps_awal) * min(1.0, langkah_env / (0.5 * args.mandiri))
            n_aktif = len(kumpulan.aktif())
            for _, ep in kumpulan.langkah(agen.cpu, eps, rng):
                buf.tambah(RO.transisi_episode(ep["obs"], ep["aksi"], ep["reward"], ep["mask"],
                                               not ep["terpotong"], args.gamma, args.n))
                tiba_ema = ep["sukses"] * 100.0 if tiba_ema is None else 0.98 * tiba_ema + 2.0 * ep["sukses"]
            # potong episode yang berputar terlalu lama (disimpan sebagai terpotong)
            for i in kumpulan.aktif():
                s = kumpulan.slot[i]
                if len(s["aksi"]) >= BATAS_LANGKAH_EVAL:
                    buf.tambah(RO.transisi_episode(np.asarray(s["obs"], np.float32), np.asarray(s["aksi"]),
                                                   np.asarray(s["reward"], np.float32) / SKALA_REWARD,
                                                   np.asarray(s["mask"], bool), False, args.gamma, args.n))
                    kumpulan.slot[i] = None
            langkah_env += n_aktif
            utang += args.rasio * n_aktif
            while utang >= 1.0 and not berhenti:
                utang -= 1.0
                catat(agen.latih_satu(buf, args.batch, beta(), rng))
                if agen.langkah % 250 == 0:
                    agen.sinkron_cpu()
                if agen.langkah % args.eval_setiap == 0:
                    evaluasi("mandiri")
                st["mandiri"], st["eps"] = tiba_ema, eps
                berhenti = tampil()

    ev = evaluasi("akhir")
    das.tutup()
    agen.simpan(os.path.join(args.keluar, f"{nama}_akhir.pt"), fase="akhir", eval=ev, **meta)
    print(f"[OK] ID {rid:02d} model {jenis} selesai dalam {(time.time() - t0) / 60:.1f} menit. Model terbaik "
          f"(skor {terbaik['skor']:.1f}, langkah {terbaik['langkah']}) -> {jalur_model(rid, jenis, args.keluar)}")
    return berhenti


def main():
    ap = argparse.ArgumentParser(description="Latih DQfD per robot (model dekat & jauh)")
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--model", default="dekat,jauh",
                    help="dekat, jauh, atau dekat,jauh (bawaan). Jauh butuh model dekat yang sudah dilatih")
    ap.add_argument("--tampil", action="store_true", help="tampilkan dasbor")
    ap.add_argument("--pra", type=int, default=20000, help="langkah gradien pra-latih")
    ap.add_argument("--mandiri", type=int, default=150000, help="langkah lingkungan latihan sendiri")
    ap.add_argument("--lr-mandiri", type=float, default=5e-5, help="learning rate fase B")
    ap.add_argument("--lam-n-mandiri", type=float, default=0.5, help="bobot J_n fase B")
    ap.add_argument("--porsi-sulit", type=float, default=0.5, help="porsi skenario peta tersulit di fase B")
    ap.add_argument("--rasio", type=float, default=0.5, help="langkah gradien per langkah lingkungan")
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--lr", type=float, default=1e-4)
    ap.add_argument("--gamma", type=float, default=0.995)
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--margin", type=float, default=0.8)
    ap.add_argument("--target-setiap", type=int, default=2000)
    ap.add_argument("--kapasitas", type=int, default=300000, help="kapasitas buffer data agen")
    ap.add_argument("--eps-awal", type=float, default=0.1)
    ap.add_argument("--eps-akhir", type=float, default=0.01)
    ap.add_argument("--n-slot", type=int, default=16)
    ap.add_argument("--eval-setiap", type=int, default=5000, help="langkah gradien antar-evaluasi")
    ap.add_argument("--n-negatif", type=int, default=16)
    ap.add_argument("--fps", type=float, default=6.0)
    ap.add_argument("--nilai-akhir", default="pendaratan", choices=["pendaratan", "v"],
                    help="model jauh: imbalan di pose serah = pendaratan sungguhan model dekat "
                         "(bawaan) atau perkiraan V model dekat (cara lama)")
    # 2026-10-23: dengan validasi mundur, model jauh ID 01 yang dilatih lanjut tidak
    # pernah mengalahkan langkah 0 (skor 85,6), padahal di mode gabungan semua
    # checkpoint-nya lebih baik (langkah 10000: tiba 99,1 -> 99,4%, biaya p90 1,37
    # -> 1,17x, waktu p90 175 -> 115 ms, 345 rute validasi). Model dipilih dengan
    # metode yang dipakai saat operasi.
    ap.add_argument("--metode-validasi", default=METODE_BAWAAN, choices=["mundur", "terbaik", "gabungan"],
                    help="pencarian rute gabungan saat validasi model jauh (bawaan = metode operasional)")
    ap.add_argument("--metode-pendaratan", default=METODE_BAWAAN, choices=["mundur", "terbaik", "gabungan"],
                    help="pencarian model dekat untuk --nilai-akhir pendaratan (bawaan = pengujian)")
    ap.add_argument("--batas-pendaratan", type=float, default=BATAS_DEKAT_S,
                    help="detik per percobaan pendaratan (bawaan = pengujian). Mis. --metode-pendaratan "
                         "mundur --batas-pendaratan 0.3: ~4x lebih cepat, lebih pesimis")
    ap.add_argument("--maks-pendaratan", type=int, default=MAKS_SERAH,
                    help="percobaan pendaratan per episode fase B, lalu gagal (pengujian: MAKS_SERAH)")
    ap.add_argument("--transfer", action="store_true", help="ID 02 mulai dari bobot ID 01 (jenis sama)")
    ap.add_argument("--awal", default=None,
                    help="folder berisi id0N_<jenis>.pt untuk bobot awal, mis. model_dqn/arsip_v4_dekat_buta: "
                         "model dekat versi 4 (9 masukan), masukan rintangan baru mulai berbobot nol")
    ap.add_argument("--data", default=S.DIR_DATA)
    ap.add_argument("--keluar", default=DIR_MODEL)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--perangkat", default="auto", choices=["auto", "cuda", "cpu"],
                    help="auto = GPU (CUDA) bila ada; cuda = WAJIB GPU, berhenti bila tidak ada")
    ap.add_argument("--cpu", action="store_true", help="sama dengan --perangkat cpu")
    args = ap.parse_args()

    model = [m.strip() for m in args.model.split(",") if m.strip()]
    for m in model:
        if m not in L.JENIS:
            ap.error(f"--model {m}: pilih dari {', '.join(L.JENIS)}")
    if "dekat" in model and "jauh" in model:
        model = ["dekat", "jauh"]           # jauh memakai V model dekat

    if args.cpu or args.perangkat == "cpu":
        device = "cpu"
    elif torch.cuda.is_available():
        device = "cuda"
        print(f"[i] GPU: {torch.cuda.get_device_name(0)} (PyTorch {torch.__version__}, "
              f"CUDA {torch.version.cuda})")
    elif args.perangkat == "cuda":
        raise SystemExit(f"[!] --perangkat cuda, tapi PyTorch {torch.__version__} tidak melihat GPU. "
                         f"Pasang PyTorch versi CUDA (pytorch.org) atau periksa driver NVIDIA.")
    else:
        device = "cpu"
        print("[!] GPU tidak terdeteksi - latih di CPU (jauh lebih lambat).")

    for rid in [int(t) for t in args.id.split(",") if t.strip()]:
        for jenis in model:
            if latih_model(rid, jenis, args, device) == "semua":
                print("[i] dihentikan (ESC)")
                return


if __name__ == "__main__":
    main()
