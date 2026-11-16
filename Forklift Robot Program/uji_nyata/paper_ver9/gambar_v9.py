import sys, csv, json, os
sys.path.insert(0, ".")
os.environ["FORKLIFT_DIR_MODEL"] = "model_dqn_gudang3"; os.environ["FORKLIFT_TANPA_P"] = "1"
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import konfig, arena as A, rencana as R
from drl import uji_gudang3 as U
ar = A.Arena("Gudang3"); x0, y0, x1, y1 = ar.ctx.ext
def baca(folder, sk, per):
    out = {1: [], 2: []}
    with open(f"uji_nyata/{folder}/lintasan.csv") as f:
        r = csv.reader(f); h = next(r); ix = {k: i for i, k in enumerate(h)}
        for b in r:
            if b[ix["skenario"]] == sk and b[ix["perencana"]] == per and b[ix["fase"]] == "UJI":
                out[int(b[ix["robot"]])].append((float(b[ix["x_cm"]]), float(b[ix["y_cm"]])))
    return out
def mk(folder, sk, per):
    for l in open(f"uji_nyata/{folder}/hasil.jsonl"):
        d = json.loads(l)
        if d["skenario"] == sk and d["perencana"] == per: return d["makespan_s"], json.load(open(f"uji_nyata/{folder}/skenario.json"))
SK = "S3"
P = [("(a) Hybrid A* + RS", sys.argv[4], "astar-standar", "#1f77b4"),
     ("(b) DQfD, lab model", sys.argv[5], "dqfd-murni", "#d62728"),
     ("(c) DQfD", sys.argv[6], "dqfd-murni", "#2ca02c")]
fig, axs = plt.subplots(1, 4, figsize=(14.0, 3.3), dpi=300)
for ax, (judul, folder, per, w) in zip(axs, P):
    ax.imshow(ar.occ > 0, cmap="Greys", alpha=0.55, extent=(x0, x1, y1, y0), origin="upper", interpolation="nearest")
    L = baca(folder, SK, per); m, sken = mk(folder, SK, per)
    for rid, ls in ((1, "-"), (2, "--")):
        p = np.array(L[rid]); ax.plot(p[:, 0], p[:, 1], ls, color=w, lw=1.3)
    s = next(s for s in sken["skenario"] if s["nama"] == SK)
    for rid, r in s["robot"].items():
        ax.plot(*r["awal"][:2], "s", mfc="none", mec="k", ms=5, mew=0.8); ax.text(r["awal"][0] + 14, r["awal"][1] - 12, f"A{rid}", fontsize=6)
        ax.plot(*r["tujuan"][:2], "*", color="k", ms=6); ax.text(r["tujuan"][0] + 14, r["tujuan"][1] + 30, f"T{rid}", fontsize=6)
    ax.set_title(f"{judul}: {m:.1f} s", fontsize=8)
# panel d: Warehouse4, satu rute, kedua perencana (offline)
konfig.pilih_robot(1)
a4 = A.Arena("Gudang4"); ax = axs[3]
ax.imshow(a4.occ > 0, cmap="Greys", alpha=0.55, extent=(x0, x1, y1, y0), origin="upper", interpolation="nearest")
st = U.stasiun_gudang4(R.Perencana(a4, konfig.profil(1), jenis="astar"))
dari, ke = sys.argv[2], sys.argv[3]
info_s = []
for j, w, ls, nm in (("astar-standar", "#1f77b4", "-", "Hybrid A* + RS"), ("dqfd-murni", "#2ca02c", "--", "DQfD")):
    per = R.Perencana(a4, konfig.profil(1), jenis=j)
    per.rencanakan(st[dari], st[ke], None, None)
    jl, info = per.rencanakan(st[dari], st[ke], None, None)
    p = np.array([a4.pose_ke_cm(q[0], q[1], q[2])[:2] for q in jl])
    ax.plot(p[:, 0], p[:, 1], ls, color=w, lw=1.3, label=f"{nm}: {info['cpu_s']*1000:.0f} ms")
    print(j, round(info["panjang_cm"]), round(info["cpu_s"]*1000), info["perencana"])
ax.plot(*st[dari][:2], "s", mfc="none", mec="k", ms=5, mew=0.8); ax.text(st[dari][0] - 60, st[dari][1] - 14, "A", fontsize=6)
ax.plot(*st[ke][:2], "*", color="k", ms=6); ax.text(st[ke][0] + 16, st[ke][1] - 14, "T", fontsize=6)
ax.legend(fontsize=5.5, loc="lower center", framealpha=0.9, handlelength=2.2, borderpad=0.3)
ax.set_title("(d) Warehouse4 (unseen), one route", fontsize=8)
for ax in axs:
    ax.set_xlim(-10, 770); ax.set_ylim(670, -10); ax.set_aspect("equal"); ax.tick_params(labelsize=6); ax.set_xlabel("x (cm)", fontsize=7)
axs[0].set_ylabel("y (cm)", fontsize=7)
fig.tight_layout(pad=0.3); fig.savefig(sys.argv[1])
