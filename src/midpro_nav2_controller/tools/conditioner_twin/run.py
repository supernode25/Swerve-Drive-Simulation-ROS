import math, numpy as np
from cond import Grid, Cond
from PIL import Image, ImageDraw
def plan_poly(pts, step=0.05):
    out=[]
    for a,b in zip(pts[:-1],pts[1:]):
        L=math.dist(a,b); n=max(1,int(L/step))
        for k in range(n): out.append((a[0]+(b[0]-a[0])*k/n, a[1]+(b[1]-a[1])*k/n, 0.0))
    out.append((pts[-1][0],pts[-1][1],0.0)); return out
def draw(g,raw,cond,robot,fn,goal_yaw=None):
    S=60; H,W=g.c.shape; img=Image.new('RGB',(W*3,H*3),(255,255,255)); d=ImageDraw.Draw(img)
    P=lambda x,y:((x-g.ox)/g.res*3,(H-(y-g.oy)/g.res)*3)
    ys,xs=np.nonzero(g.c>=254)
    for y,x in zip(ys,xs): d.rectangle([x*3,(H-y-1)*3,x*3+3,(H-y)*3],fill=(80,80,80))
    d.line([P(p[0],p[1]) for p in raw],fill=(190,190,190),width=3)
    d.line([P(p[0],p[1]) for p in cond],fill=(31,95,168),width=3)
    x,y,yaw=robot; c,s=math.cos(yaw),math.sin(yaw)
    poly=[P(x+c*lx-s*ly,y+s*lx+c*ly) for lx,ly in [(0.42,0.25),(0.42,-0.25),(-0.48,-0.25),(-0.48,0.25)]]
    d.polygon(poly,outline=(31,95,168))
    img.save(fn)

# 1. screenshot case: robot against the lower rack heading east, corridor runs north-east; plan leaves NW first
g=Grid(10,6)
for i in range(200):                       # lower rack: band along y = 0.6 + 0.45 x (below), upper rack above
    x=i*0.05; g.box(x,0,x+0.05,0.30+0.45*x); g.box(x,2.6+0.45*x,x+0.05,6)
robot=(1.0,1.05,0.0)                       # front-right corner touching the sloped rack (squeezed start)
raw=plan_poly([(1.0,1.05),(0.75,1.55),(1.2,1.95),(3.0,2.75),(6.0,4.1)])
c=Cond(g); P,notes=c.condition(raw,robot[2]); print('1 start case :', notes)
draw(g,raw,P,robot,'case1.png')

# 2. wiggly corridor (boxes narrow it to 0.82 m in places)
g2=Grid(12,3,oy=-1.5)
g2.box(0,0.41,12,1.5)
for x0 in (2.0,4.2,7.0): g2.box(x0,-1.5,x0+1.0,-0.41)
g2.box(0,-1.5,12,-1.2)
raw2=[]
for i in range(int(11/0.05)):
    x=0.5+i*0.05; low=-0.41 if any(x0<=x<=x0+1.0 for x0 in (2.0,4.2,7.0)) else -1.2
    raw2.append((x,(0.41+low)/2,0.0))
for _ in range(4): raw2=[raw2[0]]+[(raw2[i][0],(raw2[i-1][1]+2*raw2[i][1]+raw2[i+1][1])/4,0) for i in range(1,len(raw2)-1)]+[raw2[-1]]
P2,notes2=Cond(g2).condition(raw2,0.0); ys=[p[1] for p in P2[:-25]]
print('2 corridor   :', notes2, '| conditioned y range %.2f..%.2f (raw %.2f..%.2f)'%(min(ys),max(ys),min(p[1] for p in raw2),max(p[1] for p in raw2)))
draw(g2,raw2,P2,(0.5,raw2[0][1],0.0),'case2.png')

# 3b handled below
# 3. U pocket open to the south; the plan arrives through a 0.70 m side corridor (no room to turn on the plan);
#    open floor south of the pocket mouth is off the plan
g3=Grid(8,6)
g3.box(3.0,4.1,5.0,4.4)                    # back wall (north)
g3.box(3.0,3.0,3.2,4.4); g3.box(4.6,3.0,4.8,4.4)   # side walls -> inner x 3.2..4.6 (1.4 m), opening y < 3.0
g3.box(0,2.15,3.0,6); g3.box(0,0,3.0,1.45) # west corridor y 1.45..2.15 (0.70 m) leading to the pocket mouth area
g3.box(4.8,0,8,6); g3.box(3.0,0,4.8,0.3)   # east wall, floor bottom
goal=(3.9,3.75,0.0)                        # parked east-west inside the pocket
raw3=plan_poly([(0.6,1.8),(3.5,1.8),(3.9,2.4),(3.9,3.75)]); raw3[-1]=(3.9,3.75,0.0)
P3,notes3=Cond(g3).condition(raw3,0.0); print('3 U pocket   :', notes3)
draw(g3,raw3,P3,(0.6,1.8,0.0),'case3.png')

# 3b. narrow U pocket (1.1 m inside) at the end of a 0.70 m corridor; open floor south of the corridor end (off the plan)
g4=Grid(8,6)
g4.box(3.2,4.15,4.6,4.45)                       # back wall
g4.box(3.15,3.25,3.35,4.45); g4.box(4.45,3.25,4.65,4.45)   # side walls -> inside x 3.35..4.45 (1.1 m)
g4.box(0,3.25,3.15,6); g4.box(4.65,0,8,6)       # north of the corridor (west part), east wall
g4.box(0,0,3.0,2.55)                            # south wall of the corridor (west part) -> corridor y 2.55..3.25
g4.box(3.0,0,4.65,0.3)
goal4=(3.9,3.72,0.0)
raw4=plan_poly([(0.6,2.9),(3.9,2.9),(3.9,3.72)]); raw4[-1]=(3.9,3.72,0.0)
c4=Cond(g4); P4,notes4=c4.condition(raw4,0.0); print('3b U pocket  :', notes4)
draw(g4,raw4,P4,(0.6,2.9,0.0),'case3b.png')

# 4. wall-facing slot: the plan slides along the wall (5 cm off it) to a goal facing the wall -> nowhere on the
#    plan to turn; open floor behind the goal (off the plan)
g5=Grid(8,5)
g5.box(0,3.0,8,5)                                # wall (north), face at y = 3.0
g5.box(0,0,8,0.3)
goal5=(4.0,2.53,math.pi/2)                       # facing the wall: front edge 5 cm off it
raw5=plan_poly([(0.8,2.70),(4.0,2.70),(4.0,2.53)]); raw5[-1]=(4.0,2.53,math.pi/2)
P5,notes5=Cond(g5).condition(raw5,0.0); print('4 wall slot  :', notes5)
draw(g5,raw5,P5,(0.8,2.70,0.0),'case4.png')
