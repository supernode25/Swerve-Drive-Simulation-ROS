"""Python twin of path_conditioner.cpp (same algorithm) for offline checks."""
import math, numpy as np
FP=[(0.42,0.25),(0.42,-0.25),(-0.48,-0.25),(-0.48,0.25)]
def ang(a): return (a+math.pi)%(2*math.pi)-math.pi
class Grid:
    def __init__(s,w,h,res=0.05,ox=0.0,oy=0.0): s.res=res;s.ox=ox;s.oy=oy;s.c=np.zeros((int(h/res),int(w/res)),np.uint8)
    def box(s,x0,y0,x1,y1):
        c0=int((x0-s.ox)/s.res);c1=int(math.ceil((x1-s.ox)/s.res));r0=int((y0-s.oy)/s.res);r1=int(math.ceil((y1-s.oy)/s.res))
        s.c[max(r0,0):max(r1,0),max(c0,0):max(c1,0)]=254
    def blocked(s,x,y):
        cx=int((x-s.ox)/s.res);cy=int((y-s.oy)/s.res)
        if cx<0 or cy<0 or cy>=s.c.shape[0] or cx>=s.c.shape[1]: return True
        return s.c[cy,cx]>=254
class Cond:
    def __init__(s,g,horizon=25.0,interval=0.05,start_check=0.8,start_search=3.0,start_align_tol=0.52,pull_max_seg=6.0,
                 margins=(0.06,0.03,0.01),move_pad=0.03,stage_min=0.6,stage_max=2.5,stage_connect=5.0,protect_end=1.0):
        s.g=g;s.__dict__.update(dict(horizon=horizon,interval=interval,start_check=start_check,start_search=start_search,start_align_tol=start_align_tol,
            pull_max_seg=pull_max_seg,margins=margins,move_pad=move_pad,stage_min=stage_min,stage_max=stage_max,stage_connect=stage_connect,protect_end=protect_end))
    def poseFree(s,x,y,yaw,pad):
        if s.g.blocked(x,y): return False
        fx=0.42+pad;rx=-0.48-pad;hw=0.25+pad;c=math.cos(yaw);sn=math.sin(yaw);st=max(0.02,s.g.res*0.5)
        ly=-hw
        while ly<=hw+1e-9:
            lx=rx
            while lx<=fx+1e-9:
                if s.g.blocked(x+c*lx-sn*ly,y+sn*lx+c*ly): return False
                lx+=st
            ly+=max(st,2*hw/4)
        ly=-hw
        while ly<=hw+1e-9:
            if s.g.blocked(x+c*fx-sn*ly,y+sn*fx+c*ly) or s.g.blocked(x+c*rx-sn*ly,y+sn*rx+c*ly): return False
            ly+=st
        return True
    def transFree(s,x0,y0,x1,y1,yaw,pad):
        L=math.hypot(x1-x0,y1-y0);st=max(0.025,2*s.g.res);n=max(1,math.ceil(L/st))
        return all(s.poseFree(x0+(x1-x0)*k/n,y0+(y1-y0)*k/n,yaw,pad) for k in range(1,n+1))
    def escapeFree(s,x0,y0,x1,y1,yaw,pad):
        start_free=s.poseFree(x0,y0,yaw,pad); L=math.hypot(x1-x0,y1-y0); st=max(0.025,2*s.g.res); n=max(1,math.ceil(L/st))
        for k in range(1,n+1):
            if not start_free and L*k/n<0.3: continue
            if not s.poseFree(x0+(x1-x0)*k/n,y0+(y1-y0)*k/n,yaw,pad): return False
        return True
    def segFree(s,x0,y0,x1,y1,pad): return s.transFree(x0,y0,x1,y1,math.atan2(y1-y0,x1-x0),pad)
    def rotFree(s,x,y,y0,y1,pad):
        d=ang(y1-y0);n=max(1,math.ceil(abs(d)/(math.pi/36)))
        return all(s.poseFree(x,y,y0+d*k/n,pad) for k in range(1,n+1))
    def line(s,a,b,yaw,inc):
        L=math.hypot(b[0]-a[0],b[1]-a[1]);n=max(1,math.ceil(L/s.interval))
        return [(a[0]+(b[0]-a[0])*k/n,a[1]+(b[1]-a[1])*k/n,yaw) for k in range(n+(1 if inc else 0))]
    def startShift(s,P,ry):
        acc=0;i0=1
        while i0<len(P):
            acc+=math.dist(P[i0-1][:2],P[i0][:2])
            if acc>=s.start_check: break
            i0+=1
        if i0>=len(P): return P,0
        d0=math.atan2(P[i0][1]-P[0][1],P[i0][0]-P[0][0]);e=abs(ang(d0-ry))
        if e<math.pi/4 or e>3*math.pi/4: return P,0
        acc=0
        for i in range(1,len(P)-1):
            acc+=math.dist(P[i-1][:2],P[i][:2])
            if acc>s.start_search: break
            if acc<0.3: continue
            j=i;a2=0
            while j+1<len(P) and a2<0.4: a2+=math.dist(P[j][:2],P[j+1][:2]); j+=1
            dl=math.atan2(P[j][1]-P[i][1],P[j][0]-P[i][0])
            if abs(ang(dl-ry))>s.start_align_tol: continue
            if not s.escapeFree(P[0][0],P[0][1],P[i][0],P[i][1],ry,s.move_pad): continue
            out=s.line(P[0],P[i],ry,False); se=len(out); return out+P[i:],se
        return P,0
    def stage(s,P):
        n=len(P);G=P[-1];gx,gy,gyaw=G
        ia=n-1;acc=0
        while ia>0 and acc<0.5: acc+=math.dist(P[ia-1][:2],P[ia][:2]); ia-=1
        arr=math.atan2(gy-P[ia][1],gx-P[ia][0])
        if abs(ang(gyaw-arr))<0.26: return P,None,'aligned'
        if s.rotFree(gx,gy,arr,gyaw,s.move_pad): return P,None,'turn at goal fits'
        acc=0
        for i in range(n-1,0,-1):
            acc+=math.dist(P[i-1][:2],P[i][:2])
            if acc<s.stage_min: continue
            if acc>s.stage_max: break
            c=P[i-1];here=math.atan2(P[i][1]-c[1],P[i][0]-c[0])
            if s.poseFree(c[0],c[1],gyaw,s.move_pad) and s.rotFree(c[0],c[1],here,gyaw,s.move_pad) and s.transFree(c[0],c[1],gx,gy,gyaw,s.move_pad):
                return P,None,f'on-plan staging at {acc:.1f} m (controller handles)'
        cum=[0.0]
        for i in range(1,n): cum.append(cum[-1]+math.dist(P[i-1][:2],P[i][:2]))
        cg,sg=math.cos(gyaw),math.sin(gyaw);dirs=[(-cg,-sg),(-sg,cg),(sg,-cg)]
        best=math.inf;bm=0;bc=None;k=s.stage_min
        while k<=s.stage_max+1e-9:
            for d in dirs:
                cx,cy=gx+k*d[0],gy+k*d[1]
                if not s.poseFree(cx,cy,gyaw,s.move_pad) or not s.transFree(cx,cy,gx,gy,gyaw,s.move_pad): continue
                for m in range(n-1,0,-1):
                    if cum[n-1]-cum[m-1]>s.stage_connect: break
                    pm=P[m-1];th=math.atan2(cy-pm[1],cx-pm[0])
                    if not s.rotFree(cx,cy,th,gyaw,s.move_pad): continue
                    if not s.segFree(pm[0],pm[1],cx,cy,s.move_pad): continue
                    cost=cum[m-1]+math.hypot(cx-pm[0],cy-pm[1])+k+0.2*abs(ang(gyaw-th))
                    if cost<best: best=cost;bm=m-1;bc=(cx,cy)
                    break
            k+=0.1
        if bc is None: return P,None,'no staging found'
        th=math.atan2(bc[1]-P[bm][1],bc[0]-P[bm][0])
        out=P[:bm]+s.line(P[bm],bc,th,False); pf=len(out); out+=s.line((bc[0],bc[1],gyaw),G,gyaw,False)+[G]
        return out,pf,f'reshaped through axis staging ({bc[0]:.2f},{bc[1]:.2f})'
    def pull(s,P,first,last):
        if last>=len(P) or last<=first+3: return P,0
        out=P[:first];i=first;pulled=0
        while i<last:
            cand=[];acc=0
            for j in range(i+1,last+1):
                acc+=math.dist(P[j-1][:2],P[j][:2])
                if acc>s.pull_max_seg: break
                cand.append(j)
            best=0
            for pad in s.margins:
                k=len(cand)
                while k>0:
                    j=cand[k-1]
                    if j<=best: break
                    if s.segFree(P[i][0],P[i][1],P[j][0],P[j][1],pad): best=j; break
                    k=k-5 if k>5 else 0
                if best>=i+3: break
            if best>=i+3:
                out+=s.line(P[i],P[best],math.atan2(P[best][1]-P[i][1],P[best][0]-P[i][0]),False); i=best; pulled+=1
            else: out.append(P[i]); i+=1
        return out+P[last:],pulled
    def condition(s,P,ry):
        notes=[]
        pf=len(P)-1;acc=0
        while pf>0 and acc<s.protect_end: acc+=math.dist(P[pf-1][:2],P[pf][:2]); pf-=1
        P2,spf,note=s.stage(P); notes.append(note)
        if spf is not None: P=P2; pf=spf
        n0=len(P); P,se=s.startShift(P,ry)
        if se: notes.append('start-shift'); pf+=len(P)-n0
        n1=len(P); P,pulled=s.pull(P,se,pf); pf+=len(P)-n1
        if pulled: notes.append(f'pulled x{pulled}')
        return P,notes
