import subprocess, numpy as np, json, glob, os, collections
from PIL import Image
R='/home/prince/avian_rev_c/SIH_AVIAN_FINAL'; V=f'{R}/media/avian_inspection_film.mp4'; F=f'{R}/media/_film'
log=json.load(open(f'{R}/scene/film/detection_log.json'))
EDIT=["title_open","wide_open","beat1","beat2","wide_mid_a","beat3","beat4","beat5","wide_mid_b","beat6","beat7","beat8","wide_close","title_end"]
LENS=dict(title_open=96,wide_open=144,beat1=192,beat2=192,wide_mid_a=48,beat3=192,beat4=192,beat5=192,wide_mid_b=48,beat6=192,beat7=192,beat8=192,wide_close=144,title_end=96)
starts={}; s=0
for n in EDIT: starts[n]=s; s+=LENS[n]-10
def orange(a):  # a: HxWx3 uint8
    a=a.astype(np.int16); r,g,b=a[...,0],a[...,1],a[...,2]
    return (r>200)&(g>110)&(g<215)&(b<110)&((r-b)>130)
def stroke_score(m, bb):
    x0,y0,x1,y1=[int(round(v)) for v in bb]; W=int(min(x1-x0,y1-y0)*0.18); L=max(14,W)
    hits=0;tot=0
    for px,py,sx,sy in ((x0,y0,1,1),(x1,y0,-1,1),(x0,y1,1,-1),(x1,y1,-1,-1)):
        # brackets are 2px wide: allow +-3 px search band for scale/rounding
        for t in range(3,L-2):
            xs=px+sx*t; ys=py+sy*t
            for (xx,yy) in ((xs,py),(px,ys)):
                if 0<=xx<1920 and 0<=yy<1080:
                    tot+=1; hits+= bool(m[max(0,yy-3):yy+4, max(0,xx-3):xx+4].any()) if False else 0
    return hits,tot
def score_old(m,bb):
    x0,y0,x1,y1=[int(round(v)) for v in bb]; L=max(14,int(min(x1-x0,y1-y0)*0.18)); hit=tot=0
    for px,py,sx,sy in ((x0,y0,1,1),(x1,y0,-1,1),(x0,y1,1,-1),(x1,y1,-1,-1)):
        for t in range(4,L-3,2):
            for (xx,yy) in ((px+sx*t,py),(px,py+sy*t)):
                if 3<=xx<1917 and 3<=yy<1077:
                    tot+=1; hit+= m[yy-3:yy+4, xx-3:xx+4].any()
    return hit/tot if tot else 0.0
def read_frames(a,b):
    cmd=f'ffmpeg -v error -i "{V}" -vf "select=between(n\\,{a}\\,{b})" -vsync 0 -f rawvideo -pix_fmt rgb24 -'
    p=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE)
    sz=1920*1080*3
    while True:
        buf=p.stdout.read(sz)
        if len(buf)<sz: break
        yield np.frombuffer(buf,np.uint8).reshape(1080,1920,3)

def score(diff,bb):
    x0,y0,x1,y1=[int(round(v)) for v in bb]; L=max(14,int(min(x1-x0,y1-y0)*0.18)); vals=[]
    for px,py,sx,sy in ((x0,y0,1,1),(x1,y0,-1,1),(x0,y1,1,-1),(x1,y1,-1,-1)):
        for t in range(4,L-3,2):
            for (xx,yy) in ((px+sx*t,py),(px,py+sy*t)):
                if 3<=xx<1917 and 3<=yy<1077:
                    vals.append(diff[yy-1:yy+2, xx-1:xx+2].max())
    return float(np.mean(np.array(vals)>60)) if vals else 0.0
summary={}
tot=collections.Counter()
for bn in range(1,9):
    name=f'beat{bn}'; d=glob.glob(f'{F}/{name}_*')[0]; shot=os.path.basename(d)
    rows={r['frame_index']:r for r in log if r['shot']==shot}
    drawn={i for i,r in rows.items() if r['drawn']}
    dbb=[r for r in rows.values() if r['drawn']]
    mean_bb=[float(np.mean([r[k] for r in dbb])) for k in('bbox_x0','bbox_y0','bbox_x1','bbox_y1')]
    lo,hi=11,181
    cats=collections.defaultdict(list)
    for j,fr in enumerate(read_frames(starts[name]+lo, starts[name]+hi)):
        i=lo+j
        raw=np.asarray(Image.open(f'{d}/f{i:04d}.png').convert('RGB')).astype(np.int16)
        diff=np.abs(fr.astype(np.int16)-raw).sum(2)
        bb=[rows[i][k] for k in('bbox_x0','bbox_y0','bbox_x1','bbox_y1')] if i in rows else mean_bb
        sc=score(diff,bb)
        cats['drawn' if i in drawn else ('logged_not_drawn' if i in rows else 'no_row')].append((i,sc))
    out={}
    for c,v in cats.items():
        sc=[x for _,x in v]; out[c]=(len(v), round(float(np.mean(sc)),3), sum(1 for x in sc if x>=0.5))
        tot[c+'_n']+=len(v); tot[c+'_hi']+=out[c][2]
    print(shot, {c:f"n={a} mean={b} n>=0.5:{cc}" for c,(a,b,cc) in out.items()}, flush=True)
print(dict(tot))
