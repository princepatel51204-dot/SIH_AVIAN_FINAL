import subprocess, numpy as np, json, os
from PIL import Image
R='/home/prince/avian_rev_c/SIH_AVIAN_FINAL/media'; V=f'{R}/avian_inspection_film.mp4'; C=f'{R}/_film/_clips'
EDIT=["title_open","wide_open","beat1","beat2","wide_mid_a","beat3","beat4","beat5","wide_mid_b","beat6","beat7","beat8","wide_close","title_end"]
LENS=dict(title_open=96,wide_open=144,beat1=192,beat2=192,wide_mid_a=48,beat3=192,beat4=192,beat5=192,wide_mid_b=48,beat6=192,beat7=192,beat8=192,wide_close=144,title_end=96)
XF=10
def frames(path, start, n, scale=(480,270)):
    cmd=f'ffmpeg -v error -i "{path}" -vf "select=between(n\\,{start}\\,{start+n-1}),scale={scale[0]}:{scale[1]}" -vsync 0 -f rawvideo -pix_fmt rgb24 -'
    b=subprocess.run(cmd,shell=True,capture_output=True).stdout
    a=np.frombuffer(b,dtype=np.uint8).reshape(-1,scale[1],scale[0],3).astype(np.float32)/255
    return a
start=0; out=[]
for k in range(1,14):
    start+=LENS[EDIT[k-1]]-XF
    M=frames(V,start,XF)
    A=frames(f'{C}/{EDIT[k-1]}.mp4',LENS[EDIT[k-1]]-XF,XF)
    B=frames(f'{C}/{EDIT[k]}.mp4',0,XF)
    alphas=[];res=[];resA=[];resB=[]
    for j in range(XF):
        d=(B[j]-A[j]).ravel(); m=(M[j]-A[j]).ravel()
        den=float(d@d)
        a=float(m@d/den) if den>1e-9 else float('nan')
        alphas.append(round(a,2))
        fit=A[j]+a*(B[j]-A[j]) if a==a else A[j]
        res.append(float(np.abs(M[j]-fit).mean())); resA.append(float(np.abs(M[j]-A[j]).mean())); resB.append(float(np.abs(M[j]-B[j]).mean()))
    mono=all(alphas[i]<=alphas[i+1]+0.03 for i in range(XF-1))
    print(f"{k:2d} {EDIT[k-1]:>10}->{EDIT[k]:<10} alpha {alphas}  fitres {np.mean(res):.4f} vsA {np.mean(resA):.4f} vsB {np.mean(resB):.4f} {'RAMP' if alphas[0]<0.25 and alphas[-1]>0.75 and mono else 'CHECK'}")
