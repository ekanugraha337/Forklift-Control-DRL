import json,sys,numpy as np
S=sys.argv[1]
def muat(f): return json.load(open(S+"/"+f))["rute"]
def arr(d,j,k): return np.array([np.nan if r[j].get(k) is None else float(r[j][k]) for r in d])
for peta,g in (("Gudang3","g3"),("Gudang4","g4")):
    d=muat(f"b4_{peta}.json"); h=muat(f"h4_{peta}.json"); v=muat(f"v2_{g}.json")
    print("=========",peta,len(d))
    J=["astar-standar","astar-lazy","dqfd-lab","dqfd-murni","astar"]; B="astar-standar"
    okB=arr(d,B,"ok")>0; LB=arr(d,B,"panjang_cm")
    for j in J:
        ok=arr(d,j,"ok")>0; n=arr(d,j,"node"); t=arr(d,j,"cpu_ms"); L=arr(d,j,"panjang_cm"); m=ok&okB; q=L[m]/LB[m]
        if j=="astar-lazy": p=arr(d,j,"primitif")
        elif j.startswith("dqfd"): p=arr(h,j,"primitif"); assert (arr(h,j,"node")==n).mean()>0.95, (arr(h,j,"node")==n).mean()
        else: p=arr(v,j,"primitif")
        rs=arr(v,j,"rs_kurva") if j in ("astar-standar","astar") else None
        pr=[r[j]["perencana"] for r in d]; pj=(LB>750)&okB
        print(f"-- {j}: tiba {100*ok.mean():.1f}% gagal {int((~ok).sum())} | prim med {np.median(p):.0f} p90 {np.percentile(p,90):.0f} total {p.sum():.0f} per node {p.sum()/n.sum():.3f} | node med {np.median(n):.0f} p90 {np.percentile(n,90):.0f}")
        print(f"   waktu med {np.median(t):.1f} p90 {np.percentile(t,90):.0f} mean {t.mean():.1f} maks {t.max():.0f} >1s {int((t>1000).sum())} | us/prim {1000*t.sum()/p.sum():.1f} | >7.5m: waktu med {np.median(t[pj]):.0f} prim med {np.median(p[pj]):.0f}")
        print(f"   panjang med {np.median(q):.3f} p90 {np.percentile(q,90):.3f} | ganti arah {np.nanmean(arr(d,j,'ganti_arah')[ok]):.2f} mundur {np.nanmean(arr(d,j,'mundur_cm')[ok]):.0f} sempit {np.nanmean(arr(d,j,'sempit_cm')[ok]):.0f} | mundur-balik {sum('mundur' in str(x) for x in pr)} RSawal {sum('RS dari awal' in str(x) for x in pr)}")
    al=np.array([r["alur"] for r in d])
    for x in sorted(set(al)): print("  ",x,int((al==x).sum()),{j:(round(100*(arr(d,j,'ok')[al==x]>0).mean(),1), round(arr(d,j,'cpu_ms')[al==x].mean())) for j in J})
