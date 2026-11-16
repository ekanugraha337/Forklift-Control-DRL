import json,sys,numpy as np
S=sys.argv[1]
def muat(f): return json.load(open(S+"/"+f))["rute"]
def arr(d,j,k): return np.array([np.nan if r[j].get(k) is None else float(r[j][k]) for r in d])
for peta,fb,hit in (("Gudang3","bersih_Gudang3.json",[("v2_g3.json","astar-standar","astar-standar"),("v2_g3_lab.json","dqfd-murni","dqfd-lab"),("v2_g3.json","dqfd-murni","dqfd-murni"),("v2_g3.json","astar","astar")]),
                    ("Gudang4","bersih_Gudang4.json",[("v2_g4.json","astar-standar","astar-standar"),("v3_g4_lab.json","dqfd-lab","dqfd-lab"),("v2_g4.json","dqfd-murni","dqfd-murni"),("v2_g4.json","astar","astar")])):
    d=muat(fb); print("=========",peta,len(d),"rute  (sesi bersih)")
    J=["astar-standar","dqfd-lab","dqfd-murni","astar"]; B="astar-standar"
    okB=arr(d,B,"ok")>0; LB=arr(d,B,"panjang_cm")
    print("panjang median jalur baseline (tiba): %.0f cm"%np.median(LB[okB]))
    for j in J:
        ok=arr(d,j,"ok")>0; n=arr(d,j,"node"); t=arr(d,j,"cpu_ms"); L=arr(d,j,"panjang_cm")
        m=ok&okB; q=L[m]/LB[m]
        pr=[r[j]["perencana"] for r in d]
        print(f"-- {j}: tiba {100*ok.mean():.1f}% gagal {int((~ok).sum())} {sorted(set(str(r[j]['alasan'])[:40] for r in d if not r[j]['ok']))}")
        print(f"   node med {np.median(n):.0f} p90 {np.percentile(n,90):.0f} | waktu med {np.median(t):.1f} p90 {np.percentile(t,90):.0f} mean {t.mean():.1f} maks {t.max():.0f} >1s {int((t>1000).sum())} | ms/node {t.sum()/n.sum():.4f}")
        print(f"   rasio panjang med {np.median(q):.3f} p90 {np.percentile(q,90):.3f} | ganti arah {np.nanmean(arr(d,j,'ganti_arah')[ok]):.2f} | sempit {np.nanmean(arr(d,j,'sempit_cm')[ok]):.0f} mundur {np.nanmean(arr(d,j,'mundur_cm')[ok]):.0f} bebas {np.nanmean(arr(d,j,'bebas_rata_cm')[ok]):.1f}")
        print(f"   bersama baseline (n={m.sum()}): ganti arah {np.nanmean(arr(d,j,'ganti_arah')[m]):.2f} vs {np.nanmean(arr(d,B,'ganti_arah')[m]):.2f}; sempit {np.nanmean(arr(d,j,'sempit_cm')[m]):.0f} vs {np.nanmean(arr(d,B,'sempit_cm')[m]):.0f}; mundur {np.nanmean(arr(d,j,'mundur_cm')[m]):.0f} vs {np.nanmean(arr(d,B,'mundur_cm')[m]):.0f}")
        print("   oleh:",{k:pr.count(k) for k in set(pr)})
        pj=LB>750
        print(f"   jalur baseline >7.5 m (n={int((pj&okB).sum())}): node med {np.median(n[pj&okB]):.0f} waktu med {np.median(t[pj&okB]):.0f}")
    al=np.array([r["alur"] for r in d])
    for x in sorted(set(al)):
        print("  ",x,int((al==x).sum()),{j:(round(100*(arr(d,j,'ok')[al==x]>0).mean(),1), round(arr(d,j,'cpu_ms')[al==x].mean())) for j in J})
    print("  --- hitungan (sesi berpencatat)")
    for f,k,nama in hit:
        h=muat(f); p=arr(h,k,"primitif"); n=arr(h,k,"node"); rs=arr(h,k,"rs_kurva")
        print(f"   {nama}: primitif med {np.median(p):.0f} p90 {np.percentile(p,90):.0f} mean {p.mean():.0f} total {p.sum():.0f} | per node {p.sum()/n.sum():.3f} | RS med {np.median(rs):.0f} p90 {np.percentile(rs,90):.0f} mean {rs.mean():.0f} | node med {np.median(n):.0f} (bersih {np.median(arr(d,nama,'node')):.0f})")
        if nama!=B:
            hb=muat(hit[0][0]); pb=arr(hb,B,"primitif")
            if len(pb)==len(p): print(f"      rasio median {np.median(pb)/np.median(p):.1f} p90 {np.percentile(pb,90)/np.percentile(p,90):.1f} total {pb.sum()/p.sum():.1f}; lebih sedikit di {int((p<pb).sum())} rute")
