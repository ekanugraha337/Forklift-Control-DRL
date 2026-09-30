"""
latih.py - melatih DQfD per robot: pra-latih dari demo, lalu latihan sendiri.

    python -m drl.latih --id 1,2 --tampil          # ID 01 lalu ID 02, dengan dasbor
    python -m drl.latih --id 1 --pra 5000 --mandiri 0   # cepat, hanya pra-latih
    python -m drl.visual_neuron --putar model_dqn/id01  # putar ulang sesudahnya

FASE A - PRA-LATIH: hanya data demo (episode bukan-uji). Jaringan belajar
    meniru A* (J_E) sekaligus memperkirakan biaya-sisa (J_DQ, J_n).
FASE B - LATIHAN SENDIRI: jaringan merencanakan sendiri (epsilon-greedy di
    antara aksi SAH), hasilnya masuk buffer di samping demo yang tetap ada.
    Skenario: 70% dari episode demo latih (biaya A*-nya diketahui), 30% acak
    baru dengan sebaran yang sama dengan dataset (skenario.acak_satu).

EVALUASI (tiap --eval-setiap langkah gradien) pada set VALIDASI - 10% episode
latih yang disisihkan, BUKAN set uji. Set uji hanya untuk penilaian akhir
(python -m drl.evaluasi), supaya pemilihan model tidak mengintip jawabannya.
    setuju       % transisi validasi yang argmax-nya = aksi A*
    tiba         % rute yang tersambung, jaringan greedy
    tiba_mundur  % rute yang tersambung dengan perencana operasional (mundur saat buntu)
    rasio        biaya DQfD (mundur) / biaya A*, median & p90
    skor         tiba_mundur - 10 x (rasio_p90 - 1): menghukum jalur berkelok,
                 bukan hanya gagal tiba (putaran 1: tiba 97% tapi p90 > 2x A*)

KELUARAN model_dqn/id0N/: ckpt_<langkah>.pt tiap evaluasi, riwayat.json;
model_dqn/id0N.pt = checkpoint dengan SKOR TERBAIK, id0N_akhir.pt = terakhir.

PUTARAN 2 (2026-09-30), dari hasil putaran 1: set validasi & model terbaik,
evaluasi dengan mode mundur, fase B lebih stabil (--lr-mandiri, --lam-n-mandiri),
latihan sendiri lebih sering di Rumah (--porsi-rumah), pra-latih 20k (putaran 1
mentok di ~5k) dan latihan sendiri 150k.
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import torch

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                                   # noqa: E402
from drl import rollout as RO                   # noqa: E402
from drl import skenario as S                   # noqa: E402
from drl.dqfd import AgenDQfD, Buffer, meta_profil  # noqa: E402
from drl.jaringan import SKALA_REWARD, bermask  # noqa: E402
from drl.perencana_dqfd import rencanakan       # noqa: E402
from drl.visual_neuron import Dasbor            # noqa: E402

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIR_MODEL = os.path.join(AKAR, "model_dqn")
BATAS_LANGKAH_EVAL = 200          # demo terpanjang 126 langkah
PORSI_DEMO_MANDIRI = 0.7
# Rute validasi per jenis peta. Rumah terbanyak: di sana mutu jalur terburuk.
N_VALIDASI = {"Kalibrasi": 20, "Rumah2": 40, "Rumah": 60, "sintetis": 40}
BOBOT_P90 = 10.0


def _jenis(p):
    return p.split(":")[0]


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
    for j, (lo, hi) in (("Rumah", (15, 40)), ("sintetis", (8, 30)), ("Rumah2", (6, 25)), ("Kalibrasi", (2, 10))):
        idx = episode[(jenis[episode] == j) & (lk[episode] >= lo) & (lk[episode] <= hi)]
        if len(idx):
            out.append(int(idx[len(idx) // 2]))
    return out


def bagi_validasi(d):
    """(latih, validasi): setiap episode latih ke-10 disisihkan untuk validasi."""
    bukan_uji = np.flatnonzero(~d["ep_uji"])
    val = bukan_uji[::10]
    return np.setdiff1d(bukan_uji, val), val


class Evaluator:
    """Evaluasi pada himpunan episode yang diberikan (validasi saat latih)."""

    def __init__(self, d, neg, pabrik, episode, n_negatif, anggaran=600):
        self.pabrik = pabrik
        self.anggaran = anggaran
        self.sk = [RO.dari_demo(d, e) for e in pilih_per_jenis(d, episode, N_VALIDASI)]
        idx = np.concatenate([np.arange(d["ep_mulai"][e], d["ep_mulai"][e] + d["ep_langkah"][e])
                              for e in episode])
        self.obs_uji = torch.as_tensor(d["obs"][idx])
        self.mask_uji = torch.as_tensor(d["mask"][idx])
        self.aksi_uji = torch.as_tensor(d["aksi"][idx].astype(np.int64))
        self.neg = []
        if neg is not None:
            ok = np.flatnonzero(neg["alasan"] == "tidak ada jalur")
            jenis = np.array([_jenis(p) for p in neg["peta"]])
            for j in S.KOMPOSISI:
                self.neg += [RO.dari_negatif(neg, i) for i in ok[jenis[ok] == j][:max(1, n_negatif // 4)]]
        rumah = [s for s in self.sk if s.peta == "Rumah"][:8]
        self.sk_waktu = rumah
        self.ms_astar = float(np.mean([d["ep_waktu_astar"][int(s.nama.split("#")[1])] for s in rumah]) * 1000) \
            if rumah else float("nan")

    def __call__(self, net):
        with torch.no_grad():
            q = bermask(net(self.obs_uji), self.mask_uji)
            setuju = float((q.argmax(1) == self.aksi_uji).float().mean()) * 100
        h = self._jalankan(net, self.sk)
        tiba = [x for x in h if x["sukses"]]
        rasio = [x["rasio"] for x in tiba if x["rasio"]]
        hn = self._jalankan(net, self.neg) if self.neg else []
        t0 = time.perf_counter()
        for s in self.sk_waktu:
            self._jalankan(net, [s], n_slot=1)
        ms = (time.perf_counter() - t0) / max(1, len(self.sk_waktu)) * 1000
        # Perencana OPERASIONAL (mundur saat buntu), satu per satu.
        hm = []
        for sk in self.sk:
            env, obs, info = self.pabrik.buat(sk)
            if obs is None:
                hm.append({"sk": sk, "sukses": False, "rasio": None})
                continue
            r = rencanakan(net, env, mundur=True, anggaran=self.anggaran)
            hm.append({"sk": sk, "sukses": r["sukses"],
                       "rasio": env.biaya / sk.biaya_astar if r["sukses"] else None})
        rasio_m = [x["rasio"] for x in hm if x["sukses"]]
        tiba_m = 100.0 * len(rasio_m) / max(1, len(hm))
        p90_m = float(np.percentile(rasio_m, 90)) if rasio_m else float("nan")
        per_jenis = {}
        for j in S.KOMPOSISI:
            hj = [x for x in hm if _jenis(x["sk"].peta) == j]
            per_jenis[j] = 100.0 * sum(x["sukses"] for x in hj) / max(1, len(hj))
        return {
            "tiba_mundur": tiba_m, "rasio_p90_mundur": p90_m,
            "rasio_median_mundur": float(np.median(rasio_m)) if rasio_m else float("nan"),
            "skor": tiba_m - BOBOT_P90 * (p90_m - 1.0) if rasio_m else -1e9,
            "setuju": setuju,
            "tiba": 100.0 * len(tiba) / max(1, len(h)),
            "rasio_median": float(np.median(rasio)) if rasio else float("nan"),
            "rasio_p90": float(np.percentile(rasio, 90)) if rasio else float("nan"),
            "neg_buntu": 100.0 * sum(x["alasan"] == "buntu" for x in hn) / max(1, len(hn)),
            "neg_jalur": 100.0 * sum(x["sukses"] for x in hn) / max(1, len(hn)),
            "ms_dqfd": ms, "ms_astar": self.ms_astar,
            "tiba_per_jenis": per_jenis,
            "alasan_gagal": {a: sum(x["alasan"] == a for x in h if not x["sukses"])
                             for a in {x["alasan"] for x in h if not x["sukses"]}},
        }

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


def latih_robot(rid, args, device):
    prof = konfig.profil(rid)
    jalur_demo = os.path.join(args.data, f"demo_id{rid:02d}.npz")
    jalur_neg = os.path.join(args.data, f"negatif_id{rid:02d}.npz")
    d = RO.muat_npz(jalur_demo)
    neg = RO.muat_npz(jalur_neg) if os.path.exists(jalur_neg) else None
    dir_keluar = os.path.join(args.keluar, f"id{rid:02d}")
    os.makedirs(dir_keluar, exist_ok=True)
    for f in os.listdir(dir_keluar):
        if f.startswith("ckpt_") and f.endswith(".pt"):
            os.remove(os.path.join(dir_keluar, f))
    rng = np.random.default_rng([args.seed, rid])
    pabrik = RO.PabrikEnv(prof)

    latih_ep, val_ep = bagi_validasi(d)
    n_demo = int(d["ep_langkah"][latih_ep].sum())
    buf = Buffer(n_demo + args.kapasitas, device)
    for e in latih_ep:
        obs, aksi, rew, mask = RO.episode_demo(d, e)
        buf.tambah(RO.transisi_episode(obs, aksi, rew, mask, True, args.gamma, args.n), demo=True)

    agen = AgenDQfD(device, lr=args.lr, gamma=args.gamma, n=args.n, margin=args.margin,
                    target_setiap=args.target_setiap, seed=args.seed + rid)
    if args.transfer and rid != 1:
        sumber = os.path.join(args.keluar, "id01.pt")
        ck = torch.load(sumber, map_location=device, weights_only=False)
        agen.net.load_state_dict(ck["net"])
        agen.target.load_state_dict(ck["net"])
        agen.sinkron_cpu()
        print(f"[i] ID {rid:02d}: bobot awal dari {sumber} (transfer)")

    evaluator = Evaluator(d, neg, pabrik, val_ep, args.n_negatif)
    probe_idx = pilih_probe(d, val_ep)
    das = Dasbor(prof, pabrik, [RO.dari_demo(d, e) for e in probe_idx],
                 judul=f"DQfD ID {rid:02d}", fps=args.fps, tampil=args.tampil)

    total = args.pra + int(args.mandiri * args.rasio)
    riw = {"latih": {k: [] for k in ("langkah", "j_dq", "j_n", "j_e", "q", "setuju_batch")},
           "eval": {k: [] for k in ("langkah", "setuju", "tiba", "rasio_median", "neg_buntu", "ms_dqfd",
                                     "tiba_mundur", "rasio_p90_mundur", "skor")},
           "batas_fase": []}
    ema = {}
    t0 = time.time()
    st = {"fase": "pra-latih (demo)", "target": total, "riwayat": riw["latih"],
          "eval_riwayat": riw["eval"], "batas_fase": riw["batas_fase"]}
    meta = {"robot": rid, "data_demo": jalur_demo, "probe": probe_idx, "target": total,
            "args": vars(args), **meta_profil(konfig.profil(rid))}
    terbaik = {"skor": -float("inf"), "langkah": None}
    print(f"\n=== ID {rid:02d}: {len(latih_ep)} episode demo latih ({n_demo} transisi), "
          f"validasi {len(val_ep)} episode ({len(evaluator.sk)} rute dievaluasi) + {len(evaluator.neg)} negatif, "
          f"target {total} langkah gradien, perangkat {device}")

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
        for k in riw["eval"]:
            if k == "langkah":
                continue
            v = ev[k]
            riw["eval"][k].append(None if v is None or not np.isfinite(v) else v)
        st["eval"] = ev
        agen.simpan(os.path.join(dir_keluar, f"ckpt_{agen.langkah:07d}.pt"), fase=fase, eval=ev, **meta)
        tanda = ""
        if ev["skor"] > terbaik["skor"]:
            terbaik.update(skor=ev["skor"], langkah=agen.langkah)
            agen.simpan(os.path.join(args.keluar, f"id{rid:02d}.pt"), fase=fase, eval=ev, terbaik=True, **meta)
            tanda = "  <- terbaik"
        st["terbaik"] = dict(terbaik)
        with open(os.path.join(dir_keluar, "riwayat.json"), "w", encoding="utf-8") as f:
            json.dump(riw, f)
        print(f"  [{(time.time() - t0) / 60:5.1f} mnt] {fase:<8} langkah {agen.langkah:>7}  "
              f"setuju {ev['setuju']:5.1f}%  tiba greedy {ev['tiba']:5.1f}% / mundur {ev['tiba_mundur']:5.1f}%  "
              f"rasio p90 {ev['rasio_p90_mundur']:.2f}  skor {ev['skor']:6.1f}  "
              f"J_E {ema.get('j_e', 0):.3f}  J_n {ema.get('j_n', 0):.3f}{tanda}", flush=True)
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
        rumah_ep = latih_ep[np.array([_jenis(p) for p in d["ep_peta"][latih_ep]]) == "Rumah"]
        kumpulan = RO.KumpulanRollout(pabrik, args.n_slot)
        pool_sintetis = [f"sintetis:{90_000_000 + rid * 100_000 + k}" for k in range(200)]
        peta_acak = [p for p in S.KOMPOSISI if p != "sintetis"]
        langkah_env, utang = 0, 0.0
        tiba_ema = None

        def skenario_baru():
            rumah = rng.random() < args.porsi_rumah
            if rng.random() < PORSI_DEMO_MANDIRI:
                return RO.dari_demo(d, int(rng.choice(rumah_ep if rumah and len(rumah_ep) else latih_ep)))
            if rumah:
                return pabrik.acak(rng, "Rumah")
            p = S.KOMPOSISI
            j = rng.choice(list(p), p=np.array(list(p.values())) / sum(p.values()))
            peta = pool_sintetis[int(rng.integers(len(pool_sintetis)))] if j == "sintetis" else j
            return pabrik.acak(rng, peta)

        while langkah_env < args.mandiri and not berhenti:
            for i in range(args.n_slot):
                while kumpulan.slot[i] is None:
                    sk = skenario_baru()
                    if sk is not None:
                        kumpulan.isi(i, sk)          # buntu sejak awal: slot tetap kosong, ambil lagi
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
    agen.simpan(os.path.join(args.keluar, f"id{rid:02d}_akhir.pt"), fase="akhir", eval=ev, **meta)
    print(f"[OK] ID {rid:02d} selesai dalam {(time.time() - t0) / 60:.1f} menit. Model terbaik (skor "
          f"{terbaik['skor']:.1f}, langkah {terbaik['langkah']}) -> {os.path.join(args.keluar, f'id{rid:02d}.pt')}")
    return berhenti


def main():
    ap = argparse.ArgumentParser(description="Latih DQfD per robot")
    ap.add_argument("--id", default="1,2")
    ap.add_argument("--tampil", action="store_true", help="tampilkan dasbor")
    ap.add_argument("--pra", type=int, default=20000, help="langkah gradien pra-latih")
    ap.add_argument("--mandiri", type=int, default=150000, help="langkah lingkungan latihan sendiri")
    ap.add_argument("--lr-mandiri", type=float, default=5e-5, help="learning rate fase B")
    ap.add_argument("--lam-n-mandiri", type=float, default=0.5, help="bobot J_n fase B")
    ap.add_argument("--porsi-rumah", type=float, default=0.5, help="porsi skenario Rumah di fase B")
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
    ap.add_argument("--transfer", action="store_true", help="ID 02 mulai dari bobot ID 01")
    ap.add_argument("--data", default=S.DIR_DATA)
    ap.add_argument("--keluar", default=DIR_MODEL)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--cpu", action="store_true", help="paksa latih di CPU")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() and not args.cpu else "cpu"
    for rid in [int(t) for t in args.id.split(",") if t.strip()]:
        if latih_robot(rid, args, device) == "semua":
            print("[i] dihentikan (ESC)")
            break


if __name__ == "__main__":
    main()
