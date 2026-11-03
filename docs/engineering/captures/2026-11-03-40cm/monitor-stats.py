import re,sys
rows=[]
for l in open(sys.argv[1]):
    if 'Range(' not in l: continue
    node=l.split()[0]
    d=int(re.search(r'distance_mm: (\d+)',l).group(1)); s=int(re.search(r'sigma_mm: (\d+)',l).group(1)); q=int(re.search(r'quality: (\d+)',l).group(1))
    rows.append((node,d,s,q))
for node in sorted({r[0] for r in rows}):
    sel=[r for r in rows if r[0]==node]
    ds=sorted(x[1] for x in sel); ss=sorted(x[2] for x in sel); qs=[x[3] for x in sel]
    print(f"{node[:4]}: n={len(sel):4d} median {ds[len(ds)//2]:5d} mm p10 {ds[len(ds)//10]:5d} p90 {ds[9*len(ds)//10]:5d} sigma {ss[len(ss)//2]:4d} quality {sum(qs)/len(qs):.0f}")
