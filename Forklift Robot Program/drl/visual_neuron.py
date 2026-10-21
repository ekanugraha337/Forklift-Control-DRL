"""
visual_neuron.py - dasbor pelatihan DQfD: progres, simulasi perencanaan, jaringan.

    +-----------------------------+------------------------------------------+
    | SIMULASI: jaringan menyusun | JARINGAN: input 47, aktivasi FC1/FC2     |
    | jalur primitif demi         | (16x16), cabang V/A (8x16), Q 32 aksi,   |
    | primitif; kipas 32 calon    | perkiraan vs kenyataan biaya-sisa        |
    | diwarnai Q; 32 sinar; A*    |                                          |
    +-----------------------------+------------------------------------------+
    | PETA NILAI -V(x,y) di       | PROGRES: fase, langkah, loss, % setuju,  |
    | samping heuristik Dijkstra  | % tiba, rasio biaya DQfD/A*              |
    +-----------------------------+------------------------------------------+

Dipanggil dari latih.py (--tampil). Tombol: q = hentikan robot ini (simpan),
ESC = hentikan semua, n = rute uji berikutnya, p = jeda/lanjut.

Putar ulang checkpoint setelah pelatihan:
    python -m drl.visual_neuron --putar model_dqn/id01
"""
import argparse
import glob
import json
import math
import os
import sys
import time

import cv2
import numpy as np
import torch

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "drl"

import konfig                                      # noqa: E402
from drl import lingkungan as L                    # noqa: E402
from drl import rollout as RO                      # noqa: E402
from drl.jaringan import SKALA_REWARD, bermask     # noqa: E402

LEBAR, TINGGI = 1500, 900
LATAR = (28, 28, 30)
TEKS = (225, 225, 225)
REDUP = (140, 140, 140)
HIJAU, ORANYE, BIRU, MERAH, KUNING = (60, 200, 60), (0, 150, 255), (255, 150, 40), (60, 60, 230), (0, 220, 255)
FONT = cv2.FONT_HERSHEY_SIMPLEX
LANGKAH_PETA_NILAI = 6            # sel per titik peta nilai
N_ARAH_PETA_NILAI = 8             # arah hadap yang dicoba per titik; diambil yang termurah
WARNA_ASTAR = (225, 200, 150)     # biru muda


def _teks(img, s, xy, skala=0.45, warna=TEKS, tebal=1):
    cv2.putText(img, s, (int(xy[0]), int(xy[1])), FONT, skala, warna, tebal, cv2.LINE_AA)


def _cmap(v, peta=cv2.COLORMAP_INFERNO):
    """v 0..1 (array) -> BGR uint8."""
    u8 = np.clip(np.nan_to_num(v) * 255, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(u8.reshape(-1, 1), peta).reshape(*np.shape(v), 3)


def _heatmap(img, x, y, nilai, baris, kolom, sel, judul):
    v = np.asarray(nilai, np.float64).reshape(baris, kolom)
    maks = v.max()
    warna = _cmap(v / maks if maks > 0 else v)
    blok = cv2.resize(warna, (kolom * sel, baris * sel), interpolation=cv2.INTER_NEAREST)
    img[y:y + baris * sel, x:x + kolom * sel] = blok
    cv2.rectangle(img, (x - 1, y - 1), (x + kolom * sel, y + baris * sel), (80, 80, 80), 1)
    _teks(img, f"{judul}  aktif {int((v > 0).sum())}/{v.size}", (x, y - 6), 0.42)


def _plot(img, rect, seri, judul, ylim=None, log=False, garis_v=()):
    """seri: [(nama, xs, ys, warna)]. Garis sederhana dengan sumbu."""
    x0, y0, w, h = rect
    cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (70, 70, 70), 1)
    _teks(img, judul, (x0 + 4, y0 - 6), 0.42)
    semua_x = [x for _, xs, _, _ in seri for x in xs]
    semua_y = [(math.log10(max(y, 1e-6)) if log else y) for _, _, ys, _ in seri for y in ys
               if y is not None and np.isfinite(y)]
    if not semua_x or not semua_y:
        _teks(img, "(belum ada data)", (x0 + 10, y0 + h // 2), 0.4, REDUP)
        return
    xa, xb = min(semua_x), max(max(semua_x), min(semua_x) + 1)
    ya, yb = ylim if ylim else (min(semua_y), max(semua_y))
    if yb - ya < 1e-9:
        yb = ya + 1.0
    for xv in garis_v:
        if xa <= xv <= xb:
            px = int(x0 + (xv - xa) / (xb - xa) * w)
            cv2.line(img, (px, y0), (px, y0 + h), (90, 90, 90), 1)
    def ky(y):
        t = ((math.log10(max(y, 1e-6)) if log else y) - ya) / (yb - ya)
        return int(y0 + h - min(1.0, max(0.0, t)) * h)      # di luar skala: menempel tepi

    for i, (nama, xs, ys, warna) in enumerate(seri):
        pts = [(int(x0 + (x - xa) / (xb - xa) * w), ky(y))
               for x, y in zip(xs, ys) if y is not None and np.isfinite(y)]
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts, np.int32)], False, warna, 1, cv2.LINE_AA)
        elif pts:
            cv2.circle(img, pts[0], 2, warna, -1)
        _teks(img, nama, (x0 + 6 + 95 * i, y0 + h + 14), 0.38, warna)
    lbl = (lambda v: f"1e{v:.0f}") if log else (lambda v: f"{v:.2g}")
    _teks(img, lbl(yb), (x0 + w + 3, y0 + 8), 0.35, REDUP)
    _teks(img, lbl(ya), (x0 + w + 3, y0 + h), 0.35, REDUP)


class Probe:
    """Satu rute uji yang dipakai dasbor: skenario, jalur A*, dan grid peta nilai."""

    def __init__(self, sk, pabrik):
        self.sk = sk
        self.arena = pabrik.arena(sk.peta)
        env, _, _ = pabrik.buat(sk)
        self.jalur_astar = []
        if sk.aksi_demo:
            e2, _, _ = pabrik.buat(sk)
            for a in sk.aksi_demo:
                if e2.selesai:          # cek_serah model dekat bisa berhenti lebih awal
                    break
                e2.step(a)
            self.jalur_astar = e2.jalur()
        self._grid(env)

    def _grid(self, env):
        """Observasi di grid posisi x N_ARAH_PETA_NILAI arah hadap, untuk peta nilai.

        Dihitung SEKALI per probe - observasi tidak bergantung jaringan, jadi
        tiap pembaruan tinggal satu forward. Satu arah hadap saja tidak cukup:
        di Rumah badan + margin 8 cm hanya muat di sedikit posisi untuk arah
        tertentu, dan petanya hampir kosong.
        """
        om, v, pl = env.omap, env.v, env.planner
        xs = np.arange(LANGKAH_PETA_NILAI / 2, om.w, LANGKAH_PETA_NILAI)
        ys = np.arange(LANGKAH_PETA_NILAI / 2, om.h, LANGKAH_PETA_NILAI)
        arah = np.arange(N_ARAH_PETA_NILAI) * (2 * math.pi / N_ARAH_PETA_NILAI)
        obs, sel, h = [], [], []
        r = pl.h_reso
        for iy, y in enumerate(ys):
            for ix, x in enumerate(xs):
                ada = False
                for yaw in arah:
                    if om.pose_free(v, x, y, yaw):
                        env.x, env.y, env.yaw, env.d, env.k = x, y, yaw, 0, 0.0
                        obs.append(env.observasi())
                        sel.append(iy * len(xs) + ix)
                        ada = True
                hv = env._hgrid[min(env._hgrid.shape[0] - 1, int(y // r)),
                                min(env._hgrid.shape[1] - 1, int(x // r))]
                h.append(hv if ada and np.isfinite(hv) else np.nan)
        self.gshape = (len(ys), len(xs))
        self.gsel = np.array(sel, np.int64)
        self.gobs = torch.as_tensor(np.array(obs, np.float32)) if obs else None
        self.gh = np.array(h, np.float64).reshape(self.gshape)

    def biaya_sisa_jaringan(self, net):
        """Biaya-sisa terkecil atas arah hadap yang muat, per titik grid (sel)."""
        out = np.full(self.gshape[0] * self.gshape[1], np.inf)
        if self.gobs is not None:
            with torch.no_grad():
                _, akt = net(self.gobs, aktivasi=True)
            np.minimum.at(out, self.gsel, -akt["v"].squeeze(1).numpy() * SKALA_REWARD)
        out[~np.isfinite(out)] = np.nan
        return out.reshape(self.gshape)


class Dasbor:
    def __init__(self, profil, pabrik, probes, judul="DQfD", fps=6.0, tampil=True):
        self.p, self.pabrik = profil, pabrik
        self.sk_probe = probes
        self._probe = {}
        self.i_probe = 0
        self.fps = fps
        self.tampil = tampil
        self.judul = judul
        self.t_frame = 0.0
        self.env = None
        self.hold = 0
        self.jeda = False
        self.planner_ref = None
        self.label_aksi = None
        if tampil:
            cv2.namedWindow(judul, cv2.WINDOW_AUTOSIZE)

    def tutup(self):
        if self.tampil:
            cv2.destroyWindow(self.judul)
            cv2.waitKey(1)

    # ------------------------------------------------------------ rollout
    def probe(self):
        k = self.i_probe % len(self.sk_probe)
        if k not in self._probe:
            self._probe[k] = Probe(self.sk_probe[k], self.pabrik)
        return self._probe[k]

    def _mulai(self, net):
        pr = self.probe()
        self.env, _, _ = self.pabrik.buat(pr.sk)
        if self.label_aksi is None:
            self.label_aksi = [b[0] for b in L.tabel_aksi(self.env.planner, self.env.jenis)]
        self.aksi = []
        self.v_prediksi = []
        self.biaya_langkah = []
        self.peta_nilai = pr.biaya_sisa_jaringan(net)
        self.selesai = False
        self.alasan = ""

    def _maju(self, net):
        """Hitung Q + aktivasi untuk keadaan sekarang, lalu ambil satu aksi."""
        env = self.env
        obs = env.observasi()
        mask = env.mask()
        with torch.no_grad():
            q, akt = net(torch.as_tensor(obs[None]), aktivasi=True)
        q = q[0].numpy()
        qm = np.where(mask, q, -np.inf)
        a = int(np.argmax(qm))
        demo = self.probe().sk.aksi_demo
        t = len(self.aksi)
        a_astar = demo[t] if demo and self.aksi == demo[:t] and t < len(demo) else None
        self.frame = {"obs": obs, "mask": mask, "q": q, "akt": {k: v[0].numpy() for k, v in akt.items()},
                      "a": a, "a_astar": a_astar,
                      # pose SEBELUM langkah: sinar, kipas calon, dan Q milik pose ini
                      "pose": (env.x, env.y, env.yaw),
                      "calon": [c for c in env._calon]}
        self.v_prediksi.append(-float(akt["v"][0, 0]) * SKALA_REWARD)
        _, r, akhir, info = env.step(a)
        self.aksi.append(a)
        self.biaya_langkah.append(-r)
        if akhir or len(self.aksi) >= 200:
            self.selesai = True
            self.alasan = info["alasan"] if akhir else "batas 200 langkah"
            self.hold = int(self.fps * 3)

    def perbarui(self, net, status, paksa=False):
        """Dipanggil dari loop latih. Menggambar bila sudah waktunya.
        Kembalian: None, 'robot' (hentikan robot ini), atau 'semua'."""
        if not self.tampil:
            return None
        now = time.time()
        if not paksa and now - self.t_frame < 1.0 / self.fps:
            return None
        self.t_frame = now
        if self.env is None:
            self._mulai(net)
        if self.selesai:
            self.hold -= 1
            if self.hold <= 0:
                self.i_probe += 1
                self._mulai(net)
        if not self.selesai:
            self._maju(net)
        img = self.gambar(status)
        cv2.imshow(self.judul, img)
        return self._tombol(cv2.waitKey(1) & 0xFF, net, status)

    def _tombol(self, k, net, status):
        if k == ord("q"):
            return "robot"
        if k == 27:
            return "semua"
        if k == ord("n"):
            self.i_probe += 1
            self._mulai(net)
        if k == ord("p"):
            while True:
                img = self.gambar(dict(status, jeda=True))
                cv2.imshow(self.judul, img)
                k2 = cv2.waitKey(100) & 0xFF
                if k2 == ord("p"):
                    break
                if k2 in (ord("q"), 27):
                    return "robot" if k2 == ord("q") else "semua"
        return None

    # ------------------------------------------------------------ gambar
    def gambar(self, st):
        img = np.full((TINGGI, LEBAR, 3), LATAR, np.uint8)
        self._panel_arena(img, (0, 0, 620, 580))
        self._panel_jaringan(img, (620, 0, 880, 580))
        self._panel_nilai(img, (0, 580, 620, 320))
        self._panel_progres(img, (620, 580, 880, 320), st)
        cv2.line(img, (620, 0), (620, TINGGI), (70, 70, 70), 1)
        cv2.line(img, (0, 580), (LEBAR, 580), (70, 70, 70), 1)
        return img

    def _panel_arena(self, img, rect):
        x0, y0, w, h = rect
        pr, env = self.probe(), self.env
        a = pr.arena
        S = min((w - 20) / a.lebar_sel, (h - 62) / a.tinggi_sel)
        ox, oy = x0 + 10, y0 + 40

        def P(x, y):
            return int(ox + x * S), int(oy + y * S)

        # Latar (peta + jalur A*) tetap selama satu probe: dibuat sekali, disalin tiap frame.
        if getattr(pr, "latar", None) is None:
            lat = np.zeros((h, w, 3), np.uint8)
            lat[:] = LATAR
            peta = cv2.cvtColor(np.where(env.omap.dist < 1, 55, 235).astype(np.uint8), cv2.COLOR_GRAY2BGR)
            if pr.sk.rintangan is not None:
                cv2.fillPoly(peta, [a.sel_poligon(pr.sk.rintangan)], (120, 120, 120))
            peta = cv2.resize(peta, (int(a.lebar_sel * S), int(a.tinggi_sel * S)),
                              interpolation=cv2.INTER_NEAREST)
            lat[oy - y0:oy - y0 + peta.shape[0], ox - x0:ox - x0 + peta.shape[1]] = peta
            if len(pr.jalur_astar) > 1:
                cv2.polylines(lat, [np.array([(int(ox - x0 + p[0] * S), int(oy - y0 + p[1] * S))
                                              for p in pr.jalur_astar], np.int32)],
                              False, WARNA_ASTAR, 5, cv2.LINE_AA)
            pr.latar = lat
        img[y0:y0 + h, x0:x0 + w] = pr.latar
        fr = getattr(self, "frame", None)
        if fr is not None and not self.selesai:
            px, py, _ = fr["pose"]
            self._sinar(img, P, env, fr["obs"], fr["pose"])
            qv = fr["q"][fr["mask"]]
            lo, hi = (qv.min(), qv.max()) if len(qv) else (0, 1)
            for ai, c in enumerate(fr["calon"]):
                if c is None or not fr["mask"][ai]:
                    continue
                t = (fr["q"][ai] - lo) / (hi - lo) if hi > lo else 1.0
                warna = tuple(int(u) for u in _cmap(np.array([t]), cv2.COLORMAP_TURBO)[0])
                pts = [P(px, py)] + [P(p[0], p[1]) for p in c[0]]
                cv2.polylines(img, [np.array(pts, np.int32)], False, warna,
                              3 if ai == fr["a"] else 1, cv2.LINE_AA)
        jalur = env.jalur()
        for p, q in zip(jalur[:-1], jalur[1:]):
            cv2.line(img, P(p[0], p[1]), P(q[0], q[1]), HIJAU if q[3] > 0 else ORANYE, 2, cv2.LINE_AA)
        v = env.v
        sk = pr.sk
        gy = sk.tujuan[2] if sk.tujuan[2] is not None else (jalur[-1][2] if jalur else 0.0)
        robot = fr["pose"] if fr is not None and not self.selesai else (env.x, env.y, env.yaw)
        for (x, y, yaw), warna in (((sk.tujuan[0], sk.tujuan[1], gy), MERAH), (robot, BIRU)):
            fp = v.footprint(x, y, yaw).astype(np.float64)
            cv2.polylines(img, [np.array([P(*q) for q in fp], np.int32)], True, warna, 2, cv2.LINE_AA)

        status = (f"SELESAI: {self.alasan}, {len(self.aksi)} aksi, biaya {env.biaya:.0f}"
                  if self.selesai else f"langkah {len(self.aksi)}  biaya {env.biaya:.0f}")
        ref = f"  | A*: {len(sk.aksi_demo)} aksi, biaya {sk.biaya_astar:.0f}" if sk.aksi_demo else ""
        _teks(img, f"SIMULASI  {sk.peta}  margin {sk.margin:.0f} cm", (x0 + 10, y0 + 18), 0.5, KUNING)
        _teks(img, status + ref, (x0 + 10, y0 + 34), 0.42)
        _teks(img, "hijau/oranye: DQfD maju/mundur   pita biru muda: A*   kipas: calon, warna = Q   ungu: sinar",
              (x0 + 10, y0 + h - 6), 0.36, REDUP)

    def _sinar(self, img, P, env, obs, pose):
        if env.jenis != "jauh":
            return                  # model dekat (Reeds-Shepp) tidak punya sinar
        x, y, yaw = pose
        off = env._pusat_off
        cx = x + off * math.cos(yaw)
        cy = y + off * math.sin(yaw)
        for i, sud in enumerate(env._sudut):
            t = env._tepi[i] + obs[10 + i] * env._jangkau
            ex = cx + t * math.cos(yaw + sud)
            ey = cy + t * math.sin(yaw + sud)
            cv2.line(img, P(cx, cy), P(ex, ey), (150, 90, 170) if obs[10 + i] < 1 else (95, 80, 105), 1)

    def _panel_jaringan(self, img, rect):
        x0, y0, w, h = rect
        fr = getattr(self, "frame", None)
        _teks(img, f"JARINGAN  {len(fr['obs']) if fr else '?'} -> 256 -> 256 -> V 128 / A 128 -> "
                   f"Q {len(fr['q']) if fr else '?'}", (x0 + 15, y0 + 18), 0.5, KUNING)
        if fr is None:
            return
        # input
        o = fr["obs"]
        sel = 13
        xi, yi = x0 + 15, y0 + 40
        for i, v in enumerate(o):
            t = (np.clip(v, -1, 1) + 1) / 2
            warna = tuple(int(u) for u in _cmap(np.array([t]), cv2.COLORMAP_COOL)[0])
            cv2.rectangle(img, (xi + i * sel, yi), (xi + i * sel + sel - 2, yi + 18), warna, -1)
        kelompok = ((("tujuan", 0, 8), ("prim.", 8, 10), ("32 sinar", 10, 42), ("heur.", 42, 45),
                     ("margin", 45, 47)) if len(o) == 47 else
                    (("tujuan /R", 0, 2), ("arah", 2, 5), ("dekat.", 5, 7), ("gerak", 7, 9)))
        for nama, a, b in kelompok:
            _teks(img, nama, (xi + a * sel, yi + 32), 0.36, REDUP)
            cv2.line(img, (xi + a * sel, yi + 21), (xi + b * sel - 2, yi + 21), REDUP, 1)
        akt = fr["akt"]
        _heatmap(img, x0 + 15, y0 + 100, akt["fc1"], 16, 16, 12, "FC1")
        _heatmap(img, x0 + 225, y0 + 100, akt["fc2"], 16, 16, 12, "FC2")
        _heatmap(img, x0 + 435, y0 + 100, akt["v_hidden"], 8, 16, 12, "V-hidden")
        _heatmap(img, x0 + 435, y0 + 220, akt["a_hidden"], 8, 16, 12, "A-hidden")

        # Q per aksi
        q, mask = fr["q"], fr["mask"]
        biaya = -q * SKALA_REWARD
        xq, yq = x0 + 660, y0 + 50
        _teks(img, "Q -> biaya-sisa (sel)", (xq, yq - 8), 0.42)
        sah = biaya[mask]
        lo, hi = (sah.min(), sah.max()) if len(sah) else (0, 1)
        # Sampai 38 baris (model dekat) harus muat di panel atas (sampai y 580).
        n_aksi = len(q)
        baris_px = max(12, min(21, 520 // max(1, n_aksi)))
        for i in range(n_aksi):
            yy = yq + i * baris_px
            nama = self.label_aksi[i] if self.label_aksi else str(i)
            warna_t = TEKS if mask[i] else (90, 90, 90)
            _teks(img, nama.replace("maju", "M").replace("mundur", "R"), (xq, yy + 11), 0.32, warna_t)
            if mask[i]:
                pj = int(12 + 70 * (1 - (biaya[i] - lo) / (hi - lo))) if hi > lo else 82
                warna = KUNING if i == fr["a"] else (150, 150, 150)
                cv2.rectangle(img, (xq + 92, yy + 2), (xq + 92 + pj, yy + 13), warna, -1)
                _teks(img, f"{biaya[i]:.0f}", (xq + 178, yy + 11), 0.32, warna_t)
            if fr["a_astar"] == i:
                cv2.rectangle(img, (xq + 90, yy), (xq + 176, yy + 15), WARNA_ASTAR, 1)
        _teks(img, "kuning: dipilih   kotak biru: pilihan demo (A* + RS)",
              (xq, yq + n_aksi * baris_px + 10), 0.32, REDUP)

        # perkiraan vs kenyataan biaya-sisa sepanjang rollout
        v_now = fr["akt"]["v"][0] * -SKALA_REWARD
        _teks(img, f"V(s): perkiraan biaya-sisa {v_now:.0f} sel", (x0 + 15, y0 + 350), 0.45)
        nyata = np.cumsum(self.biaya_langkah[::-1])[::-1] if self.selesai else None
        xs = list(range(len(self.v_prediksi)))
        seri = [("perkiraan -V", xs, self.v_prediksi, KUNING)]
        if nyata is not None:
            seri.append(("biaya-sisa nyata", xs, list(nyata[:len(xs)]), HIJAU))
        _plot(img, (x0 + 15, y0 + 380, 600, 160), seri, "perkiraan jaringan vs biaya-sisa sebenarnya (sel) per langkah")

    def _panel_nilai(self, img, rect):
        x0, y0, w, h = rect
        pr = self.probe()
        nilai = getattr(self, "peta_nilai", None)
        if nilai is None:
            return
        _teks(img, f"PETA NILAI  biaya-sisa ke tujuan (termurah dari {N_ARAH_PETA_NILAI} arah hadap)",
              (x0 + 10, y0 + 18), 0.47, KUNING)
        gab = np.concatenate([nilai[np.isfinite(nilai)], pr.gh[np.isfinite(pr.gh)]])
        lo, hi = (np.percentile(gab, 2), np.percentile(gab, 98)) if len(gab) else (0, 1)
        for i, (m, judul) in enumerate(((nilai, "hasil belajar (-V)"), (pr.gh, "heuristik Dijkstra A*"))):
            t = (m - lo) / (hi - lo) if hi > lo else m * 0
            warna = _cmap(1 - t, cv2.COLORMAP_VIRIDIS)
            warna[~np.isfinite(m)] = (60, 60, 60)
            ph, pw = h - 78, (w - 40) // 2
            blok = cv2.resize(warna, (pw, ph), interpolation=cv2.INTER_NEAREST)
            bx, by = x0 + 10 + i * (pw + 20), y0 + 40
            img[by:by + ph, bx:bx + pw] = blok
            sx, sy = pw / pr.arena.lebar_sel, ph / pr.arena.tinggi_sel
            tx, ty = pr.sk.tujuan[0], pr.sk.tujuan[1]
            cv2.circle(img, (int(bx + tx * sx), int(by + ty * sy)), 5, MERAH, -1)
            _teks(img, judul, (bx, by + ph + 15), 0.4)
        _teks(img, f"terang = murah ({lo:.0f})  gelap = mahal ({hi:.0f} sel)  abu = badan menabrak",
              (x0 + 10, y0 + h - 4), 0.36, REDUP)

    def _panel_progres(self, img, rect, st):
        x0, y0, w, h = rect
        _teks(img, f"PROGRES  ID {self.p.id:02d}  (kemudi rencana +-{math.degrees(math.atan(self.env.planner.kappa_max * self.env.v.L)):.1f} deg)",
              (x0 + 15, y0 + 18), 0.5, KUNING)
        baris = [
            f"fase      : {st.get('fase', '-')}" + ("   [JEDA]" if st.get("jeda") else ""),
            f"langkah   : {st.get('langkah', 0):,} / {st.get('target', 0):,}",
            f"waktu     : {st.get('waktu', 0) / 60:.1f} mnt   sisa ~{st.get('eta', 0) / 60:.1f} mnt",
        ]
        ev = st.get("eval") or {}
        if ev:
            baris += [
                f"setuju A* : {ev.get('setuju', 0):.1f} %   (validasi)",
                f"tiba      : {ev.get('tiba', 0):.1f} % greedy / {ev.get('tiba_mundur', float('nan')):.1f} % mundur",
                f"rasio p90 (mundur): {ev.get('rasio_p90_mundur', float('nan')):.2f}   skor {ev.get('skor', float('nan')):.1f}",
                f"rasio biaya median: {ev.get('rasio_median', float('nan')):.3f}",
                f"negatif buntu : {ev.get('neg_buntu', 0):.0f} %",
                # A* di sini dari dataset (dibangkitkan 15 proses serempak) - bukan
                # perbandingan adil; yang adil: python -m drl.evaluasi
                f"rencana   : {ev.get('ms_dqfd', 0):.0f} ms  (A* dataset {ev.get('ms_astar', 0):.0f} ms*)",
            ]
        if st.get("mandiri"):
            baris.append(f"latihan sendiri: tiba {st['mandiri']:.0f} %  eps {st.get('eps', 0):.3f}")
        for i, b in enumerate(baris):
            _teks(img, b, (x0 + 15, y0 + 45 + i * 22), 0.43)
        rw = st.get("riwayat", {})
        batas = st.get("batas_fase", ())
        xs = rw.get("langkah", [])
        _plot(img, (x0 + 330, y0 + 45, 230, 230),
              [("J_DQ", xs, rw.get("j_dq", []), BIRU), ("J_n", xs, rw.get("j_n", []), HIJAU),
               ("J_E", xs, rw.get("j_e", []), ORANYE)], "loss (log10)", log=True, garis_v=batas)
        ev_h = st.get("eval_riwayat", {})
        xe = ev_h.get("langkah", [])
        _plot(img, (x0 + 610, y0 + 45, 230, 230),
              [("setuju", xe, ev_h.get("setuju", []), KUNING), ("tiba", xe, ev_h.get("tiba_mundur") or ev_h.get("tiba", []), HIJAU),
               ("rasio x50", xe, [None if r is None else 50 * r for r in ev_h.get("rasio_median", [])], BIRU)],
              "evaluasi (%)", ylim=(0, 100), garis_v=batas)


# ------------------------------------------------------------ putar ulang
def putar(dir_model, fps=12.0):
    """Putar ulang checkpoint: tiap checkpoint menyusun rute uji yang sama."""
    from drl.dqfd import muat_jaringan
    ck = sorted(glob.glob(os.path.join(dir_model, "ckpt_*.pt")))
    if not ck:
        raise SystemExit(f"[!] tidak ada checkpoint di {dir_model}")
    with open(os.path.join(dir_model, "riwayat.json"), encoding="utf-8") as f:
        riw = json.load(f)
    _, meta = muat_jaringan(ck[0])
    rid = meta["robot"]
    prof = konfig.profil(rid)
    pabrik = RO.PabrikEnv(prof, meta.get("jenis", "dekat"))
    from drl import skenario as S
    d = RO.muat_gabung([os.path.join(S.DIR_DATA, os.path.basename(p)) for p in meta["data_demo"]])
    probes = [RO.dari_demo(d, e) for e in meta["probe"]]
    das = Dasbor(prof, pabrik, probes, judul=f"Putar ulang DQfD ID {rid:02d}", fps=fps)
    print("[i] n: checkpoint berikutnya, b: sebelumnya, r: rute berikutnya, q: keluar")
    i = 0
    while 0 <= i < len(ck):
        net, m = muat_jaringan(ck[i])
        das.env = None
        das.i_probe = das.i_probe
        das._mulai(net)
        n = m["langkah"]
        st = {"fase": f"{m.get('fase', '')}  checkpoint {i + 1}/{len(ck)}", "langkah": n,
              "target": meta.get("target", n), "eval": m.get("eval"),
              "riwayat": {k: [v for x, v in zip(riw["latih"]["langkah"], riw["latih"][k]) if x <= n]
                          for k in riw["latih"]},
              "eval_riwayat": {k: [v for x, v in zip(riw["eval"]["langkah"], riw["eval"][k]) if x <= n]
                               for k in riw["eval"]},
              "batas_fase": riw.get("batas_fase", [])}
        maju = 1
        while True:
            if not das.selesai:
                das._maju(net)
            cv2.imshow(das.judul, das.gambar(st))
            k = cv2.waitKey(int(1000 / fps)) & 0xFF
            if k == ord("q") or k == 27:
                return
            if k == ord("r"):
                das.i_probe += 1
                das._mulai(net)
            if k in (ord("n"), ord("b")):
                maju = 1 if k == ord("n") else -1
                break
            if das.selesai:
                das.hold -= 1
                if das.hold <= -int(fps * 1.5):
                    break
        i += maju
    cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description="Dasbor DQfD: putar ulang checkpoint")
    ap.add_argument("--putar", required=True, help="folder model, mis. model_dqn/id01_dekat")
    ap.add_argument("--fps", type=float, default=12.0)
    args = ap.parse_args()
    putar(args.putar, args.fps)


if __name__ == "__main__":
    main()
