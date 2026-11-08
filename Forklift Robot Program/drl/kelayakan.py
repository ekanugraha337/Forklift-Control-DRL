"""
kelayakan.py - uji kepala kelayakan P(aksi menabrak): keadaan pencarian dan model P terpisah.

    python -m drl.kelayakan --id 1 --kumpulkan    # 1. rekam keadaan yang dikunjungi pencarian DQfD + label
    python -m drl.kelayakan --id 1 --latih        # 2. model P terpisah (supervised, PyTorch CPU)
    python -m drl.kelayakan --id 1 --nilai        #    bandingkan kepala bawaan vs model P terpisah
    python -m drl.kelayakan --id 1 --uji-jangkau 300   # akurasi P bila 32 sinar sepanjang 300 cm (bukan 100)
    python -m drl.ukur_node --id 1 --uji varian --kepala-p model_dqn/id01_kepala_p.pt ...   # 3.

Latar (catatan Obsidian 20 §12-§13): kepala bawaan (t2, berbagi batang dengan Q) dilatih
dari keadaan demo dan rollout, tetapi pencarian menilai banyak keadaan CABANG yang tidak
pernah dilihat saat latih. Di sini keadaan yang benar-benar dibuka pencarian (lazy dan
belajar) direkam bersama labelnya - ke-14 aksi disimulasi lengkap dengan cek tabrakan,
aturan yang sama dengan lingkungan._tabrak.

Pembagian rute mengikuti latih.bagi_validasi: rute UJI (ep_uji, 470 untuk ID 01) hanya
dipakai untuk menilai; model P dilatih dari keadaan demo episode latih dan keadaan
pencarian di rute latih. Pemilihan epoch memakai sebagian rute latih yang disisihkan.

Model P terpisah: 47 -> 256 -> 256 -> 14 (ReLU, sigmoid), binary cross-entropy dengan
pos_weight (aksi menabrak diberi bobot lebih - yang mahal adalah tabrakan yang lolos).
Untuk perencana, NetDenganP membungkus jaringan DQfD: Q dari model jauh, P dari model ini.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                                   # noqa: E402
from drl import label_tabrak as LT              # noqa: E402
from drl import lingkungan as L                 # noqa: E402
from drl import perencana_dqfd as PD            # noqa: E402
from drl import rollout as RO                   # noqa: E402
from drl import skenario as S                   # noqa: E402

AKAR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AMBANG = (0.5, 0.3, 0.2, 0.1, 0.05, 0.02, 0.01)


def jalur_data(rid):
    return os.path.join(AKAR, "drl", "data", f"keadaan_cari_id{rid:02d}.npz")


def jalur_model_p(rid):
    return os.path.join(AKAR, "model_dqn", f"id{rid:02d}_kepala_p.pt")


def bagi(d):
    """(latih, validasi, uji) episode - sama dengan latih.bagi_validasi + ep_uji."""
    bukan_uji = np.flatnonzero(~d["ep_uji"])
    val = bukan_uji[::10]
    return np.setdiff1d(bukan_uji, val), val, np.flatnonzero(d["ep_uji"])


def label_keadaan(env):
    """14 label tabrakan untuk keadaan env sekarang (simulasi lengkap, aturan _tabrak)."""
    lab = np.zeros(L.N_AKSI, bool)
    for a in range(L.N_AKSI):
        nd, nk, _ = L.aksi_info(env.planner, a)
        if nd < 0 and not env.boleh_mundur:
            continue
        lab[a] = env._ekspansi(nd, nk, True) is None
    return lab


def sinar_alt(env, jangkau_cm, iterasi=1024):
    """32 sinar seperti env._sinar(), tetapi dengan jangkauan lain (cm). 0..1 = 0..jangkauan."""
    off = env._pusat_off
    cx = env.x + off * math.cos(env.yaw)
    cy = env.y + off * math.sin(env.yaw)
    ang = env.yaw + env._sudut
    jangkau = env.arena.panjang_ke_sel(jangkau_cm)
    batas = env._tepi + jangkau
    t = L._sinar_inti(env.omap.dist, cx, cy, np.cos(ang), np.sin(ang), batas, iterasi)
    return np.clip((np.minimum(t, batas) - env._tepi) / jangkau, 0.0, 1.0).astype(np.float32)


def jalur_data_alt(rid, jangkau_cm):
    return os.path.join(AKAR, "drl", "data", f"keadaan_cari_id{rid:02d}_sinar{int(jangkau_cm)}.npz")


# ------------------------------------------------------------------ 1. kumpulkan
def kumpulkan(args):
    rid = args.id
    konfig.pilih_robot(rid)
    prof = konfig.profil(rid)
    d = RO.muat_npz(S.berkas_demo(S.DIR_DATA, rid)[0])
    net = PD.muat_model_numpy(rid)
    pabrik = RO.PabrikEnv(prof)
    latih_ep, _, uji_ep = bagi(d)
    rng = np.random.default_rng(args.seed)
    pilih = np.sort(rng.choice(latih_ep, size=min(args.n_latih, len(latih_ep)), replace=False))
    mode = [m.strip() for m in args.mode.split(",") if m.strip()]
    X, Y, M, B, E, Md, R = [], [], [], [], [], [], []
    jalt = getattr(args, "jangkau_alt", None)
    waktu_sinar = {"1m": 0.0, "alt": 0.0}
    t0 = time.time()
    for bagian, daftar in ((0, pilih), (1, uji_ep)):
        for k, e in enumerate(daftar):
            sk = RO.dari_demo(d, int(e))
            for im, m in enumerate(mode):
                env = pabrik.lingkungan(sk)
                asli = env.calon_malas

                def rekam(_env=env, _asli=asli, _b=bagian, _e=int(e), _m=im):
                    X.append(_env.observasi())
                    if jalt:
                        a0 = time.perf_counter()
                        _env._sinar()
                        a1 = time.perf_counter()
                        R.append(sinar_alt(_env, jalt))
                        waktu_sinar["1m"] += a1 - a0
                        waktu_sinar["alt"] += time.perf_counter() - a1
                    Y.append(label_keadaan(_env))
                    M.append(_env.omap.pose_margin(_env.v, _env.x, _env.y, _env.yaw))
                    B.append(_b)
                    E.append(_e)
                    Md.append(_m)
                    return _asli()
                env.calon_malas = rekam
                PD.rencanakan_dqfd(net, env, sk.awal, sk.tujuan, pabrik.omap(sk), pencarian=m, batas_waktu=1.0)
            if (k + 1) % 250 == 0:
                print(f"  {'latih' if bagian == 0 else 'uji'} {k + 1}/{len(daftar)} rute, {len(X)} keadaan "
                      f"({time.time() - t0:.0f} s)", flush=True)
    X, Y = np.asarray(X, np.float32), np.asarray(Y, bool)
    tambahan = {}
    keluar = jalur_data(rid)
    if jalt:
        tambahan = {"R_alt": np.asarray(R, np.float32), "jangkau_alt_cm": np.array(float(jalt)),
                    "us_sinar_1m": np.array(1e6 * waktu_sinar["1m"] / len(R)),
                    "us_sinar_alt": np.array(1e6 * waktu_sinar["alt"] / len(R))}
        keluar = jalur_data_alt(rid, jalt)
        print(f"[i] waktu 32 sinar per keadaan: 100 cm {1e6 * waktu_sinar['1m'] / len(R):.1f} us, {jalt:.0f} cm "
              f"{1e6 * waktu_sinar['alt'] / len(R):.1f} us")
    np.savez_compressed(keluar, X=X, Y=Y, margin=np.asarray(M, np.float32), bagian=np.asarray(B, np.int8),
                        episode=np.asarray(E, np.int32), mode=np.asarray(Md, np.int8), nama_mode=np.array(mode),
                        **tambahan)
    for b, nama in ((0, "latih"), (1, "uji")):
        s = np.asarray(B) == b
        print(f"[i] keadaan pencarian rute {nama}: {int(s.sum())}, aksi menabrak {100 * Y[s].mean():.1f}%, "
              f"keadaan dengan aksi menabrak {100 * Y[s].any(1).mean():.1f}%")
    print(f"[OK] {keluar} ({time.time() - t0:.0f} s)")


# ------------------------------------------------------------------ model P terpisah
# ModelPNumpy, NetDenganP, muat_model_p ada di drl/jaringan_numpy.py (dipakai perencana robot).
from drl.jaringan_numpy import ModelPNumpy, NetDenganP, muat_model_p  # noqa: E402,F401


# ------------------------------------------------------------------ metrik
def auc(skor, label):
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


def auc_keadaan_sama(P, Y):
    """AUC hanya antar-aksi di keadaan yang sama (keadaan yang punya aksi menabrak dan aksi aman)."""
    num = den = 0.0
    for p, y in zip(P, Y):
        if y.any() and (~y).any():
            pc, pa = p[y], p[~y]
            num += (pc[:, None] > pa[None, :]).sum() + 0.5 * (pc[:, None] == pa[None, :]).sum()
            den += len(pc) * len(pa)
    return float(num / den) if den else float("nan")


def metrik(P, Y):
    out = {"keadaan": int(len(Y)), "auc": auc(P, Y), "auc_keadaan_sama": auc_keadaan_sama(P, Y), "per_ambang": {}}
    for t in AMBANG:
        out["per_ambang"][str(t)] = {"terdeteksi": 100 * float((P[Y] >= t).mean()),
                                     "alarm_palsu": 100 * float((P[~Y] >= t).mean())}
    return out


# ------------------------------------------------------------------ 2. latih model P
def latih(args):
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    rid = args.id
    konfig.pilih_robot(rid)
    torch.manual_seed(args.seed)
    d = RO.muat_npz(S.berkas_demo(S.DIR_DATA, rid)[0])
    if "tabrak" not in d:
        print("[i] dataset demo belum berlabel - label dihitung di memori (memutar ulang demo) ...", flush=True)
        d["tabrak"], ring = LT.hitung(d, konfig.profil(rid))
        if ring["mask_beda"]:
            raise SystemExit(f"[!] pemutaran ulang demo tidak cocok ({ring['mask_beda']} mask beda)")
    latih_ep, val_ep, _ = bagi(d)

    def keadaan_demo(ep):
        idx = np.concatenate([np.arange(d["ep_mulai"][e], d["ep_mulai"][e] + d["ep_langkah"][e]) for e in ep])
        return d["obs"][idx].astype(np.float32), d["tabrak"][idx]

    Xd, Yd = keadaan_demo(latih_ep)
    Xdv, Ydv = keadaan_demo(val_ep)
    c = np.load(jalur_data(rid))
    lat = c["bagian"] == 0
    ep_lat = np.unique(c["episode"][lat])
    sisih = set(ep_lat[::10].tolist())                      # 10% rute latih untuk memilih epoch
    is_sisih = lat & np.isin(c["episode"], list(sisih))
    Xc, Yc = c["X"][lat & ~is_sisih], c["Y"][lat & ~is_sisih]
    Xcv, Ycv = c["X"][is_sisih], c["Y"][is_sisih]
    X = np.concatenate([Xd, Xc])
    Y = np.concatenate([Yd, Yc]).astype(np.float32)
    print(f"[i] data latih: {len(Xd)} keadaan demo + {len(Xc)} keadaan pencarian (rute latih) | pilih epoch: "
          f"{len(Xcv)} keadaan pencarian rute latih yang disisihkan + {len(Xdv)} keadaan demo validasi | "
          f"aksi menabrak {100 * Y.mean():.1f}%", flush=True)

    class ModelP(nn.Module):
        def __init__(self, n_obs, n_aksi, lebar):
            super().__init__()
            self.l1, self.l2, self.l3 = nn.Linear(n_obs, lebar), nn.Linear(lebar, lebar), nn.Linear(lebar, n_aksi)

        def forward(self, x):
            return self.l3(F.relu(self.l2(F.relu(self.l1(x)))))

    m = ModelP(L.N_OBS, L.N_AKSI, args.lebar)
    n_param = sum(p.numel() for p in m.parameters())
    opt = torch.optim.Adam(m.parameters(), lr=args.lr, weight_decay=1e-5)
    pw = torch.full((L.N_AKSI,), float(args.pos_weight))
    Xt, Yt = torch.from_numpy(X), torch.from_numpy(Y)
    rng = np.random.default_rng(args.seed)
    terbaik, sd_terbaik, sabar = -1.0, None, 0
    t0 = time.time()
    for ep in range(1, args.epoch + 1):
        m.train()
        perm = torch.from_numpy(rng.permutation(len(X)))
        jl = 0.0
        for i in range(0, len(X), args.batch):
            b = perm[i:i + args.batch]
            loss = F.binary_cross_entropy_with_logits(m(Xt[b]), Yt[b], pos_weight=pw)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            jl += loss.item() * len(b)
        m.eval()
        with torch.no_grad():
            pv = torch.sigmoid(m(torch.from_numpy(Xcv))).numpy()
            pdv = torch.sigmoid(m(torch.from_numpy(Xdv.astype(np.float32)))).numpy()
        a_cari, a_demo = auc(pv, Ycv), auc(pdv, Ydv)
        tanda = ""
        if a_cari > terbaik:
            terbaik, sd_terbaik, sabar, tanda = a_cari, {k: v.clone() for k, v in m.state_dict().items()}, 0, "  <- terbaik"
        else:
            sabar += 1
        print(f"  epoch {ep:2d}: loss {jl / len(X):.4f} | AUC keadaan pencarian (sisih) {a_cari:.4f}, "
              f"demo validasi {a_demo:.4f} ({time.time() - t0:.0f} s){tanda}", flush=True)
        if sabar >= args.sabar:
            break
    import torch as _t
    meta = {"robot": rid, "n_obs": L.N_OBS, "n_aksi": L.N_AKSI, "lebar": args.lebar, "parameter": n_param,
            "pos_weight": args.pos_weight, "auc_sisih": terbaik, "data_demo": int(len(Xd)), "data_cari": int(len(Xc))}
    _t.save({"sd": sd_terbaik, "meta": meta}, jalur_model_p(rid))
    print(f"[OK] model P terpisah ({n_param} parameter, 47 -> {args.lebar} -> {args.lebar} -> 14) -> "
          f"{jalur_model_p(rid)} | AUC terbaik {terbaik:.4f}")


# ------------------------------------------------------------------ nilai
def nilai(args):
    rid = args.id
    konfig.pilih_robot(rid)
    net = PD.muat_model_numpy(rid)
    d = RO.muat_npz(S.berkas_demo(S.DIR_DATA, rid)[0])
    if "tabrak" not in d:
        d["tabrak"], _ = LT.hitung(d, konfig.profil(rid))
    _, val_ep, _ = bagi(d)
    idx = np.concatenate([np.arange(d["ep_mulai"][e], d["ep_mulai"][e] + d["ep_langkah"][e]) for e in val_ep])
    c = np.load(jalur_data(rid))
    himpunan = {"demo validasi": (d["obs"][idx].astype(np.float32), d["tabrak"][idx]),
                "pencarian rute latih": (c["X"][c["bagian"] == 0], c["Y"][c["bagian"] == 0]),
                "pencarian rute uji": (c["X"][c["bagian"] == 1], c["Y"][c["bagian"] == 1])}
    model = {}
    if getattr(net, "kepala_tabrak", False):
        model["kepala bawaan (t2)"] = net.tabrak_batch
    if os.path.exists(jalur_model_p(rid)):
        mp, _ = muat_model_p(jalur_model_p(rid))
        model["model P terpisah"] = mp.p_batch
    hasil = {}
    for nm, f in model.items():
        hasil[nm] = {}
        for hn, (Xh, Yh) in himpunan.items():
            hasil[nm][hn] = metrik(f(Xh), Yh)
    for nm, per in hasil.items():
        print(f"\n=== {nm}")
        for hn, r in per.items():
            pa = r["per_ambang"]
            print(f"  {hn:<22} ({r['keadaan']:6d} keadaan): AUC {r['auc']:.4f}, dalam keadaan sama "
                  f"{r['auc_keadaan_sama']:.4f} | terdeteksi/alarm palsu @0,5 {pa['0.5']['terdeteksi']:.1f}/"
                  f"{pa['0.5']['alarm_palsu']:.1f}%  @0,1 {pa['0.1']['terdeteksi']:.1f}/{pa['0.1']['alarm_palsu']:.1f}%  "
                  f"@0,02 {pa['0.02']['terdeteksi']:.1f}/{pa['0.02']['alarm_palsu']:.1f}%")
    from drl.ukur_node import _simpan
    _simpan(args.keluaran, {"kelayakan_nilai": hasil})


def _latih_p(Xtr, Ytr, Xva, Yva, args, seed=0):
    """Latih satu model P (47+ -> lebar -> lebar -> 14) dan kembalikan (state_dict terbaik, AUC terbaik, epoch)."""
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    torch.manual_seed(seed)
    n_in = Xtr.shape[1]

    class ModelP(nn.Module):
        def __init__(self):
            super().__init__()
            self.l1, self.l2, self.l3 = nn.Linear(n_in, args.lebar), nn.Linear(args.lebar, args.lebar), \
                nn.Linear(args.lebar, L.N_AKSI)

        def forward(self, x):
            return self.l3(F.relu(self.l2(F.relu(self.l1(x)))))

    m = ModelP()
    opt = torch.optim.Adam(m.parameters(), lr=args.lr, weight_decay=1e-5)
    pw = torch.full((L.N_AKSI,), float(args.pos_weight))
    Xt, Yt, Xv = torch.from_numpy(Xtr), torch.from_numpy(Ytr.astype(np.float32)), torch.from_numpy(Xva)
    rng = np.random.default_rng(seed)
    terbaik, sd, sabar, ep_terbaik = -1.0, None, 0, 0
    for ep in range(1, args.epoch + 1):
        m.train()
        perm = torch.from_numpy(rng.permutation(len(Xtr)))
        for i in range(0, len(Xtr), args.batch):
            b = perm[i:i + args.batch]
            loss = F.binary_cross_entropy_with_logits(m(Xt[b]), Yt[b], pos_weight=pw)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
        m.eval()
        with torch.no_grad():
            a = auc(torch.sigmoid(m(Xv)).numpy(), Yva)
        if a > terbaik:
            terbaik, sd, sabar, ep_terbaik = a, {k: v.clone() for k, v in m.state_dict().items()}, 0, ep
        else:
            sabar += 1
            if sabar >= args.sabar:
                break
    m.load_state_dict(sd)
    m.eval()
    return m, terbaik, ep_terbaik, sum(p.numel() for p in m.parameters())


def uji_jangkau(args):
    """Bandingkan akurasi model P dengan 32 sinar 100 cm, 32 sinar sepanjang args.uji_jangkau, dan keduanya.
    Data: keadaan pencarian saja (rute latih untuk latih, 10% rute latih untuk memilih epoch, rute uji untuk menilai)."""
    import torch
    rid, jalt = args.id, float(args.uji_jangkau)
    jalur = jalur_data_alt(rid, jalt)
    if not os.path.exists(jalur):
        args.jangkau_alt = jalt
        kumpulkan(args)
    c = np.load(jalur)
    X, R, Y, B, E = c["X"], c["R_alt"], c["Y"], c["bagian"], c["episode"]
    lat = B == 0
    ep_lat = np.unique(E[lat])
    sisih = lat & np.isin(E, ep_lat[::10])
    tr, va, uj = lat & ~sisih, sisih, B == 1
    X3 = X.copy()
    X3[:, 10:10 + L.N_SINAR] = R
    varian = {"sinar 100 cm (sekarang)": X, f"sinar {jalt:.0f} cm": X3,
              f"sinar 100 cm + {jalt:.0f} cm": np.concatenate([X, R], axis=1)}
    print(f"[i] latih {int(tr.sum())}, pilih epoch {int(va.sum())}, uji {int(uj.sum())} keadaan pencarian | waktu 32 "
          f"sinar per keadaan: 100 cm {float(c['us_sinar_1m']):.1f} us, {jalt:.0f} cm {float(c['us_sinar_alt']):.1f} us",
          flush=True)
    hasil = {"waktu_sinar_us": {"100": float(c["us_sinar_1m"]), f"{jalt:.0f}": float(c["us_sinar_alt"])}}
    for nama, Xv in varian.items():
        t0 = time.time()
        m, a_va, ep, n_param = _latih_p(Xv[tr], Y[tr], Xv[va], Y[va], args)
        with torch.no_grad():
            P = torch.sigmoid(m(torch.from_numpy(Xv[uj]))).numpy()
        r = metrik(P, Y[uj])
        r.update(masukan=int(Xv.shape[1]), parameter=int(n_param), epoch_terbaik=ep, auc_pilih=a_va,
                 detik=time.time() - t0)
        hasil[nama] = r
        pa = r["per_ambang"]
        print(f"  {nama:<26} ({Xv.shape[1]} masukan, epoch {ep}): AUC uji {r['auc']:.4f}, dalam keadaan sama "
              f"{r['auc_keadaan_sama']:.4f} | terdeteksi/alarm palsu @0,5 {pa['0.5']['terdeteksi']:.1f}/"
              f"{pa['0.5']['alarm_palsu']:.1f}%  @0,1 {pa['0.1']['terdeteksi']:.1f}/{pa['0.1']['alarm_palsu']:.1f}%  "
              f"@0,02 {pa['0.02']['terdeteksi']:.1f}/{pa['0.02']['alarm_palsu']:.1f}% ({r['detik']:.0f} s)", flush=True)
    from drl.ukur_node import _simpan
    _simpan(args.keluaran, {f"kelayakan_jangkau_{jalt:.0f}": hasil})


def main():
    ap = argparse.ArgumentParser(description="Kepala kelayakan P(aksi menabrak): data pencarian, model P terpisah")
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--kumpulkan", action="store_true")
    ap.add_argument("--latih", action="store_true")
    ap.add_argument("--nilai", action="store_true")
    ap.add_argument("--uji-jangkau", type=float, default=None, help="jangkauan sinar pembanding (cm), mis. 300")
    ap.add_argument("--n-latih", type=int, default=1500, help="jumlah rute latih yang dicari untuk data")
    ap.add_argument("--mode", default="lazy,belajar", help="pencarian yang direkam")
    ap.add_argument("--lebar", type=int, default=256)
    ap.add_argument("--pos-weight", type=float, default=3.0)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--batch", type=int, default=2048)
    ap.add_argument("--epoch", type=int, default=40)
    ap.add_argument("--sabar", type=int, default=5)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--keluaran", default=os.path.join(AKAR, "model_dqn", "ukur_node_id01_mac_kepala.json"))
    args = ap.parse_args()
    if not (args.kumpulkan or args.latih or args.nilai or args.uji_jangkau):
        ap.error("pilih --kumpulkan, --latih, --nilai, dan/atau --uji-jangkau")
    if args.kumpulkan:
        kumpulkan(args)
    if args.latih:
        latih(args)
    if args.nilai:
        nilai(args)
    if args.uji_jangkau:
        uji_jangkau(args)


if __name__ == "__main__":
    main()
