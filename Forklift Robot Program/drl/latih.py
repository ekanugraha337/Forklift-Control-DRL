"""
latih.py - melatih model jauh DQfD per robot: pra-latih dari demo, lalu latihan sendiri.

    python -m drl.latih --id 1,2 --perangkat cuda               # ID 01 lalu ID 02
    python -m drl.latih --id 1 --tampil                         # dengan dasbor
    python -m drl.latih --id 1 --pra 5000 --mandiri 0           # cepat, hanya pra-latih
    python -m drl.visual_neuron --putar model_dqn/id01_jauh     # putar ulang sesudahnya

ATURAN ver-1-6. Satu model per robot - MODEL JAUH, meniru Hybrid A* (demo dari
Hybrid A* peran "demonstrator": w = 2,0, h = max(h_RS, h_D), RS di setiap node
< 60 cm). Episode berakhir SERAH: di dalam 60 cm, peta Dijkstra bebas, dan
tembakan RS bebas tabrakan; imbalan langkah terakhir sudah memuat biaya
tembakan RS (drl/lingkungan.py). Tidak ada model dekat.

PyTorch HANYA untuk gradien (dqfd.AgenDQfD.latih_satu). Semua forward -
rollout latihan sendiri, evaluasi, dasbor - memakai JaringanNumpy (agen.numpy),
salinan bobot yang disinkronkan dari jaringan online. 32 sinar observasi
dihitung dengan numba (drl/lingkungan.py).

FASE A - PRA-LATIH: hanya data demo (episode bukan-uji). Jaringan belajar
    meniru demonstrator (J_E) sekaligus memperkirakan biaya-sisa (J_DQ, J_n).
FASE B - LATIHAN SENDIRI: jaringan merencanakan sendiri (epsilon-greedy di
    antara aksi SAH), hasilnya masuk buffer di samping demo yang tetap ada.
    Skenario: 70% dari episode demo latih, 30% acak baru (skenario.acak_satu).

EVALUASI (tiap --eval-setiap langkah gradien) pada set VALIDASI - 10% episode
latih yang disisihkan, BUKAN set uji (python -m drl.evaluasi):
    tiba         pencarian operasional (perencana_dqfd.rencanakan_dqfd, --metode-validasi)
    tiba greedy  rollout argmax-Q saja
    rasio biaya  vs demo Hybrid A* + RS seluruh rute; ganti arah; galat pose akhir
    setuju       argmax Q = aksi demo, pada transisi validasi
    negatif      skenario tanpa jalur menurut A*: DQfD harus ikut buntu
    skor         tiba - 10 x (p90 rasio - 1)

KELUARAN model_dqn/id0N_jauh/: ckpt_<langkah>.pt tiap evaluasi, riwayat.json;
model_dqn/id0N_jauh.pt = checkpoint SKOR TERBAIK, id0N_jauh_akhir.pt = terakhir.
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
from drl import label_tabrak as LT              # noqa: E402
from drl import lingkungan as L                 # noqa: E402
from drl import rollout as RO                   # noqa: E402
from drl import skenario as S                   # noqa: E402
from drl.dqfd import AgenDQfD, Buffer, meta_model, muat_checkpoint, periksa_profil  # noqa: E402
from drl.jaringan_numpy import SKALA_REWARD, bermask  # noqa: E402
from drl.perencana_dqfd import METODE_BAWAAN, jalur_model, mutu_jalur, rencanakan_dqfd  # noqa: E402
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


def bobot_awal(agen, jalur, prof):
    """Mulai latihan dari checkpoint model jauh lain (--awal / --transfer). Kembalian: langkahnya."""
    if not os.path.exists(jalur):
        raise SystemExit(f"[!] bobot awal: {jalur} tidak ada")
    ck = muat_checkpoint(jalur, agen.device)
    periksa_profil(ck, prof, jalur)
    agen.muat_bobot(ck["net"])
    return ck.get("langkah")


# ------------------------------------------------------------ evaluasi
class Evaluator:
    """Evaluasi pada himpunan episode yang diberikan (validasi saat latih). Forward: JaringanNumpy."""

    def __init__(self, d, neg, pabrik, episode, n_negatif, metode=METODE_BAWAAN):
        self.pabrik = pabrik
        self.metode = metode
        self.jenis, self.sulit, _ = jenis_peta(d)
        self.sk = [RO.dari_demo(d, e) for e in pilih_per_jenis(d, episode, jumlah_validasi(d))]
        idx = np.concatenate([np.arange(d["ep_mulai"][e], d["ep_mulai"][e] + d["ep_langkah"][e])
                              for e in episode])
        self.obs_uji = d["obs"][idx]
        self.mask_uji = d["mask"][idx]
        self.aksi_uji = d["aksi"][idx].astype(np.int64)
        self.tabrak_uji = d["tabrak"][idx] if "tabrak" in d else None
        self.neg = []
        if neg is not None:
            ok = np.flatnonzero(neg["alasan"] == "tidak ada jalur")
            jn = np.array([_jenis(p) for p in neg["peta"]])
            for j in self.jenis:
                self.neg += [RO.dari_negatif(neg, i) for i in ok[jn[ok] == j][:max(1, n_negatif // 4)]]
        # Pembanding: titik balik jalur demo (primitif A* + ekor RS) di rute validasi.
        self.balik_demo = {}
        for sk in self.sk:
            env, obs, _ = pabrik.buat(sk)
            if obs is None:
                continue
            for a in sk.aksi_demo:
                if env.selesai:
                    break
                env.step(int(a))
            if env.sukses:
                self.balik_demo[sk.nama] = mutu_jalur(env.jalur(), env.tujuan, env.radius_serah, 1.0)["ganti_arah"]
        self.sk_waktu = [s for s in self.sk if _jenis(s.peta) == self.sulit][:8]

    def __call__(self, net):
        q = bermask(net.q_batch(self.obs_uji), self.mask_uji)
        setuju = float((q.argmax(1) == self.aksi_uji).mean()) * 100
        ev_tabrak = {}
        if getattr(net, "kepala_tabrak", False) and self.tabrak_uji is not None:
            P, Y = net.tabrak_batch(self.obs_uji), self.tabrak_uji
            ev_tabrak = {"auc_tabrak": auc(P, Y),
                         "tabrak_terdeteksi": 100.0 * float((P[Y] >= 0.5).mean()) if Y.any() else float("nan"),
                         "tabrak_alarm_palsu": 100.0 * float((P[~Y] >= 0.5).mean())}
        h = self._jalankan(net, self.sk)
        tiba_greedy = 100.0 * sum(x["sukses"] for x in h) / max(1, len(h))
        hn = self._jalankan(net, self.neg) if self.neg else []
        t0 = time.perf_counter()
        for s in self.sk_waktu:
            self._satu(net, s)
        ms = (time.perf_counter() - t0) / max(1, len(self.sk_waktu)) * 1000

        hm = [self._satu(net, sk) for sk in self.sk]
        ok = [x for x in hm if x["sukses"]]
        rasio = [x["rasio"] for x in ok if x["rasio"]]
        tiba = 100.0 * len(ok) / max(1, len(hm))
        p90 = _p(rasio, 90)
        per_jenis = {j: 100.0 * sum(x["sukses"] for x in hm if _jenis(x["sk"].peta) == j)
                     / max(1, sum(_jenis(x["sk"].peta) == j for x in hm)) for j in self.jenis}
        return {
            "tiba": tiba, "tiba_greedy": tiba_greedy, "rasio_p90": p90, "rasio_median": _p(rasio, 50),
            "skor": tiba - BOBOT_P90 * (p90 - 1.0) if rasio else -1e9,
            "setuju": setuju, "ms_dqfd": ms, "tiba_per_jenis": per_jenis,
            "balik": _rata([x["ganti_arah"] for x in ok]),
            "balik_demo": _rata([self.balik_demo.get(x["sk"].nama) for x in ok]),
            "galat_posisi_cm": _rata([x["galat_posisi_cm"] for x in ok]),
            "galat_arah_deg": _rata([x["galat_arah_deg"] for x in ok]),
            "neg_buntu": 100.0 * sum(not x["sukses"] for x in hn) / max(1, len(hn)),
            **ev_tabrak,
        }

    def _satu(self, net, sk):
        """Perencana operasional (model jauh + tembakan RS, metode validasi) satu skenario."""
        arena = self.pabrik.arena(sk.peta)
        env = self.pabrik.lingkungan(sk)
        h = rencanakan_dqfd(net, env, sk.awal, sk.tujuan, self.pabrik.omap(sk), metode=self.metode)
        if not h["sukses"] or not h["jalur"]:
            return {"sk": sk, "sukses": False, "alasan": h["alasan"]}
        return {"sk": sk, "sukses": True, "rasio": h["biaya"] / sk.biaya_astar if sk.biaya_astar else None,
                **mutu_jalur(h["jalur"], sk.tujuan, env.radius_serah, arena.panjang_ke_cm(1.0))}

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


def auc(skor, label):
    """Peluang skor contoh positif > skor contoh negatif (peringkat rata-rata untuk seri)."""
    skor, label = np.asarray(skor, float).ravel(), np.asarray(label, bool).ravel()
    n_pos, n_neg = int(label.sum()), int((~label).sum())
    if not n_pos or not n_neg:
        return float("nan")
    urut = np.argsort(skor, kind="mergesort")
    pk = np.empty(len(skor))
    s_urut = skor[urut]
    awal = np.r_[0, np.flatnonzero(np.diff(s_urut)) + 1]
    akhir = np.r_[awal[1:], len(skor)]
    for a, b in zip(awal, akhir):
        pk[urut[a:b]] = (a + b + 1) / 2.0
    return float((pk[label].sum() - n_pos * (n_pos + 1) / 2.0) / (n_pos * n_neg))


def baris_eval(ev):
    teks = (f"setuju {ev['setuju']:5.1f}%  tiba greedy {ev['tiba_greedy']:5.1f}%  TIBA {ev['tiba']:5.1f}%  "
            f"rasio med/p90 {ev['rasio_median']:.2f}/{ev['rasio_p90']:.2f}  balik {ev['balik']:.2f} "
            f"(demo {ev['balik_demo']:.2f})  galat akhir {ev['galat_posisi_cm']:.2f} cm / "
            f"{ev['galat_arah_deg']:.2f} deg  negatif buntu {ev['neg_buntu']:.0f}%  skor {ev['skor']:6.1f}")
    if "auc_tabrak" in ev:
        teks += (f"  tabrak: AUC {ev['auc_tabrak']:.3f}, terdeteksi {ev['tabrak_terdeteksi']:.1f}%, "
                 f"alarm palsu {ev['tabrak_alarm_palsu']:.1f}%")
    return teks


# ------------------------------------------------------------ latih
def muat_data(args, rid):
    berkas = S.berkas_demo(args.data, rid)
    if not berkas:
        raise SystemExit(f"[!] Dataset model jauh ID {rid:02d} tidak ada di {args.data}.\n"
                         f"    Bangkitkan dulu: python -m drl.skenario --id {rid} --n 5000")
    d = RO.muat_npz(berkas[0])
    if int(d.get("versi_aksi", 0)) != L.VERSI_AKSI:
        raise SystemExit(f"[!] {berkas[0]} dibuat dengan versi aksi {int(d.get('versi_aksi', 0))}, program "
                         f"{L.VERSI_AKSI}. Bangkitkan ulang: python -m drl.skenario --id {rid}")
    if args.kepala_tabrak and "tabrak" not in d:
        d = LT.tambahkan(berkas[0], konfig.profil(rid))
    jalur_neg = os.path.join(args.data, f"negatif_id{rid:02d}.npz")
    neg = RO.muat_npz(jalur_neg) if os.path.exists(jalur_neg) else None
    return d, neg, berkas[0]


def latih_model(rid, args, device):
    prof = konfig.profil(rid)
    d, neg, berkas = muat_data(args, rid)
    nama = f"id{rid:02d}_{L.JENIS}"
    dir_keluar = os.path.join(args.keluar, nama)
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
        tabrak = RO.tabrak_demo(d, e) if "tabrak" in d else None
        buf.tambah(RO.transisi_episode(obs, aksi, rew, mask, True, args.gamma, args.n, tabrak=tabrak), demo=True)

    agen = AgenDQfD(device, lr=args.lr, gamma=args.gamma, n=args.n, margin=args.margin,
                    target_setiap=args.target_setiap, seed=args.seed + rid,
                    kepala_tabrak=args.kepala_tabrak, lam_t=args.lam_t)
    if args.transfer and rid != 1:
        sumber = jalur_model(1, args.keluar)
        bobot_awal(agen, sumber, prof)
        print(f"[i] ID {rid:02d}: bobot awal dari {sumber} (transfer)")
    if args.awal:
        sumber = jalur_model(rid, args.awal)
        langkah = bobot_awal(agen, sumber, prof)
        print(f"[i] ID {rid:02d}: bobot awal dari {sumber} (langkah {langkah})")

    evaluator = Evaluator(d, neg, pabrik, val_ep, args.n_negatif, metode=args.metode_validasi)
    probe_idx = pilih_probe(d, val_ep)
    das = Dasbor(prof, pabrik, [RO.dari_demo(d, e) for e in probe_idx],
                 judul=f"DQfD model jauh ID {rid:02d}", fps=args.fps, tampil=args.tampil)

    total = args.pra + int(args.mandiri * args.rasio)
    riw = {"latih": {k: [] for k in ("langkah", "j_dq", "j_n", "j_e", "j_t", "q", "setuju_batch")},
           "eval": {"langkah": []}, "batas_fase": []}
    ema = {}
    t0 = time.time()
    st = {"fase": "pra-latih (demo)", "target": total, "riwayat": riw["latih"],
          "eval_riwayat": riw["eval"], "batas_fase": riw["batas_fase"]}
    meta = {"robot": rid, "data_demo": os.path.basename(berkas), "probe": probe_idx,
            "target": total, "args": vars(args), "kepala_tabrak": bool(args.kepala_tabrak), **meta_model(prof)}
    terbaik = {"skor": -float("inf"), "langkah": None}
    print(f"\n=== ID {rid:02d} model jauh ({L.N_OBS} masukan, {L.N_AKSI} aksi): {len(latih_ep)} episode demo latih "
          f"({n_demo} transisi, dari {meta['data_demo']}), validasi {len(val_ep)} episode "
          f"({len(evaluator.sk)} rute dievaluasi) + {len(evaluator.neg)} negatif, target {total} langkah "
          f"gradien, perangkat {device}, forward numpy")
    print(f"    peta: {', '.join(evaluator.jenis)} | tersulit (waktu A* terbesar): {evaluator.sulit}")

    def catat(m):
        for k in ("j_dq", "j_n", "j_e", "j_t", "q", "setuju_batch"):
            v = m[k]
            if np.isfinite(v):
                ema[k] = v if k not in ema else 0.98 * ema[k] + 0.02 * v
        if agen.langkah % 50 == 0:
            riw["latih"]["langkah"].append(agen.langkah)
            for k in ("j_dq", "j_n", "j_e", "j_t", "q", "setuju_batch"):
                riw["latih"][k].append(ema.get(k, float("nan")))

    def evaluasi(fase):
        ev = evaluator(agen.sinkron_numpy())
        riw["eval"]["langkah"].append(agen.langkah)
        for k, v in ev.items():
            if isinstance(v, (int, float)):
                riw["eval"].setdefault(k, []).append(None if not math.isfinite(v) else v)
        st["eval"] = ev
        agen.simpan(os.path.join(dir_keluar, f"ckpt_{agen.langkah:07d}.pt"), fase=fase, eval=ev, **meta)
        tanda = ""
        if ev["skor"] > terbaik["skor"]:
            terbaik.update(skor=ev["skor"], langkah=agen.langkah)
            agen.simpan(jalur_model(rid, args.keluar), fase=fase, eval=ev, terbaik=True, **meta)
            tanda = "  <- terbaik"
        st["terbaik"] = dict(terbaik)
        with open(os.path.join(dir_keluar, "riwayat.json"), "w", encoding="utf-8") as f:
            json.dump(riw, f)
        jt = f"  J_T {ema.get('j_t', 0):.3f}" if args.kepala_tabrak else ""
        print(f"  [{(time.time() - t0) / 60:5.1f} mnt] {fase:<8} langkah {agen.langkah:>7}  "
              f"{baris_eval(ev)}  J_E {ema.get('j_e', 0):.3f}  J_n {ema.get('j_n', 0):.3f}{jt}{tanda}",
              flush=True)
        return ev

    def tampil():
        dt = time.time() - t0
        st["langkah"] = agen.langkah
        st["waktu"] = dt
        st["eta"] = dt / max(1, agen.langkah) * (total - agen.langkah)
        if args.tampil and agen.langkah % 200 == 0:
            agen.sinkron_numpy()
        return das.perbarui(agen.numpy, st)

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
        # Return n-langkah dari data eksplorasi bias (off-policy), jadi bobotnya
        # dikurangi dan langkah belajar diperkecil.
        for g in agen.opt.param_groups:
            g["lr"] = args.lr_mandiri
        agen.lam_n = args.lam_n_mandiri
        agen.sinkron_numpy()
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
                        kumpulan.isi(i, sk)          # tidak sah / serah di pose awal: ambil lagi
            eps = args.eps_awal + (args.eps_akhir - args.eps_awal) * min(1.0, langkah_env / (0.5 * args.mandiri))
            n_aktif = len(kumpulan.aktif())
            for _, ep in kumpulan.langkah(agen.numpy, eps, rng):
                buf.tambah(RO.transisi_episode(ep["obs"], ep["aksi"], ep["reward"], ep["mask"],
                                               not ep["terpotong"], args.gamma, args.n, tabrak=ep["tabrak"]))
                tiba_ema = ep["sukses"] * 100.0 if tiba_ema is None else 0.98 * tiba_ema + 2.0 * ep["sukses"]
            # potong episode yang berputar terlalu lama (disimpan sebagai terpotong)
            for i in kumpulan.aktif():
                s = kumpulan.slot[i]
                if len(s["aksi"]) >= BATAS_LANGKAH_EVAL:
                    buf.tambah(RO.transisi_episode(np.asarray(s["obs"], np.float32), np.asarray(s["aksi"]),
                                                   np.asarray(s["reward"], np.float32) / SKALA_REWARD,
                                                   np.asarray(s["mask"], bool), False, args.gamma, args.n,
                                                   tabrak=np.asarray(s["tabrak"], bool)))
                    kumpulan.slot[i] = None
            langkah_env += n_aktif
            utang += args.rasio * n_aktif
            while utang >= 1.0 and not berhenti:
                utang -= 1.0
                catat(agen.latih_satu(buf, args.batch, beta(), rng))
                if agen.langkah % 250 == 0:
                    agen.sinkron_numpy()
                if agen.langkah % args.eval_setiap == 0:
                    evaluasi("mandiri")
                st["mandiri"], st["eps"] = tiba_ema, eps
                berhenti = tampil()

    ev = evaluasi("akhir")
    das.tutup()
    agen.simpan(os.path.join(args.keluar, f"{nama}_akhir.pt"), fase="akhir", eval=ev, **meta)
    print(f"[OK] ID {rid:02d} model jauh selesai dalam {(time.time() - t0) / 60:.1f} menit. Model terbaik "
          f"(skor {terbaik['skor']:.1f}, langkah {terbaik['langkah']}) -> {jalur_model(rid, args.keluar)}")
    return berhenti


def main():
    ap = argparse.ArgumentParser(description="Latih model jauh DQfD per robot")
    ap.add_argument("--id", default="1,2")
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
    ap.add_argument("--tanpa-kepala-tabrak", action="store_true",
                    help="tanpa kepala kelayakan P(aksi menabrak) - model seperti sebelum 2026-11-08")
    ap.add_argument("--lam-t", type=float, default=1.0, help="bobot J_T (binary cross-entropy kepala kelayakan)")
    ap.add_argument("--fps", type=float, default=6.0)
    ap.add_argument("--metode-validasi", default=METODE_BAWAAN, choices=["mundur", "terbaik", "gabungan"],
                    help="pencarian rute saat validasi (bawaan = metode operasional)")
    ap.add_argument("--transfer", action="store_true", help="ID 02 mulai dari bobot model jauh ID 01")
    ap.add_argument("--awal", default=None, help="folder berisi id0N_jauh.pt untuk bobot awal")
    ap.add_argument("--data", default=S.DIR_DATA)
    ap.add_argument("--keluar", default=DIR_MODEL)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--perangkat", default="auto", choices=["auto", "cuda", "cpu"],
                    help="perangkat gradien PyTorch. auto = GPU (CUDA) bila ada; cuda = WAJIB GPU")
    ap.add_argument("--cpu", action="store_true", help="sama dengan --perangkat cpu")
    args = ap.parse_args()
    args.kepala_tabrak = not args.tanpa_kepala_tabrak

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
        print("[!] GPU tidak terdeteksi - gradien di CPU.")

    for rid in [int(t) for t in args.id.split(",") if t.strip()]:
        if latih_model(rid, args, device) == "semua":
            print("[i] dihentikan (ESC)")
            return


if __name__ == "__main__":
    main()
