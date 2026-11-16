"""Bedah makespan closed-loop: jarak, laju, jarak sempit (<4 cm di luar margin), mundur, tunggu, waktu rencana."""
import json, math, sys, os, collections
import numpy as np
sys.path.insert(0, os.getcwd())
import konfig, arena as A, rencana as R
konfig.pilih_robot(1)
ar = A.Arena("Gudang3")
kend = {rid: R.Perencana(ar, konfig.profil(rid), jenis="astar").kendaraan for rid in (1, 2)}
T = collections.defaultdict(lambda: collections.defaultdict(float)); mk = collections.defaultdict(dict)
for d in sys.argv[1:]:
    for line in open(os.path.join(d, "hasil.jsonl")):
        h = json.loads(line); per = h["perencana"]
        mk[h["skenario"]][per] = (round(h["makespan_s"], 1) if h["makespan_s"] is not None else float("nan"), h["semua_tiba"], h["status"], h.get("catatan"))
        for rid, r in h["robot"].items():
            if not r.get("rencana") or not r["rencana"][0].get("jalur_cm"):
                print(h["skenario"], per, rid, "TANPA JALUR", {a: b for a, b in r.items() if a != "rencana"}, [(x.get("ok"), x.get("alasan"), round(x.get("ms", 0))) for x in r.get("rencana", [])]); continue
            p = r["rencana"][0]["jalur_cm"]; k = kend[int(rid)]
            L = Ls = Lm = 0.0; rev = 0
            for a, b in zip(p, p[1:]):
                ds = math.hypot(b[0] - a[0], b[1] - a[1]); L += ds
                g = ar.pose_ke_sel(a[0], a[1], a[2])
                if ar.panjang_ke_cm(float(ar.omap.pose_margin(k, *g))) < 4.0: Ls += ds
                if a[3] < 0: Lm += ds
                rev += a[3] != b[3]
            mv = r["t_tempuh_s"] - r["t_menunggu_s"] - r["t_tahan_s"]
            t = T[per]
            for a_, b_ in (("jarak", r["jarak_tempuh_cm"]), ("rencana_cm", L), ("sempit", Ls), ("mundur", Lm), ("rev", rev),
                           ("t", r["t_tempuh_s"]), ("gerak", mv), ("tunggu", r["t_menunggu_s"] + r["t_tahan_s"]),
                           ("ms", r["ms_rencana_total"]), ("n_rencana", r["n_rencana"]), ("n", 1)): t[a_] += b_
            print(h["skenario"], per, rid, f"t={r['t_tempuh_s']:.1f} jarak={r['jarak_tempuh_cm']:.0f} sempit={Ls:.0f} mundur={Lm:.0f} rev={rev} tunggu={r['t_menunggu_s']:.1f} n_rencana={r['n_rencana']} ms={r['ms_rencana_total']:.0f} ms1={r['rencana'][0]['ms']:.0f} zona_min={r['zona_min_cm']:.1f}")
for sk in sorted(mk): print(sk, mk[sk])
for per, t in T.items():
    print(per, {a: round(b / t["n"], 1) for a, b in t.items()}, "laju", round(t["jarak"] / t["gerak"], 2))
pers = sorted({p for v in mk.values() for p in v})
for p in pers: print(p, "makespan rata", round(np.nanmean([mk[s][p][0] for s in mk if p in mk[s]]), 1), "S1-S4", round(np.nanmean([mk[s][p][0] for s in mk if p in mk[s] and s[0]=="S"]),1))
