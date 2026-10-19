"""Two-layer grid routing for R2. Native DRC is required, never inferred here."""
import heapq,itertools,json,math,time,zipfile
from collections import defaultdict
import numpy as np
import shapely
from shapely.geometry import Point,LineString,box
from inspect_geometry import ROOT,load_geometry

SOURCE=ROOT/'hardware_revision/2026Rc_R2_layout.epro'
OUT=ROOT/'hardware_revision/2026Rc_R2_routed.epro'
a,p,rows,objects,pads=load_geometry(SOURCE)
GRID=5.; X0=-2630.;Y0=-3900.;NX=528;NY=673
xs=X0+np.arange(NX)*GRID;ys=Y0+np.arange(NY)*GRID
xx,yy=np.meshgrid(xs,ys)
outline=box(-2629.6523+78.7402,-3898.5593+78.7402,.0785-78.7402,-544.9215-78.7402).buffer(78.7402,quad_segs=32)
for r in rows:
    if r[0]=='POLY' and r[6][0]=='CIRCLE':
        _,cx,cy,rad=r[6];outline=outline.difference(Point(cx,cy).buffer(rad,quad_segs=32))
by_net=defaultdict(list)
for name,net,layers,g in pads:
    if net and net!='GND':by_net[net].append((name,g.centroid.x,g.centroid.y,layers))
routed=[];lines=[];vias=[];log=[];serial=20000
def ident():
    global serial
    serial+=1;return 'e'+str(serial)
def raster(mask,g,layers):
    bx0,by0,bx1,by1=g.bounds
    i0=max(0,int(math.floor((bx0-X0)/GRID)));i1=min(NX,int(math.ceil((bx1-X0)/GRID))+1)
    j0=max(0,int(math.floor((by0-Y0)/GRID)));j1=min(NY,int(math.ceil((by1-Y0)/GRID))+1)
    if i1<=i0 or j1<=j0:return
    value=shapely.intersects_xy(g,xx[j0:j1,i0:i1],yy[j0:j1,i0:i1])
    for l in layers:mask[l-1,j0:j1,i0:i1]|=value
def masks(net,width):
    # 8 mil copper clearance and 12 mil board-edge clearance.
    allowed=shapely.contains_xy(outline.buffer(-(12+width/2)),xx,yy)
    mask=np.stack([~allowed,~allowed])
    viaallowed=shapely.contains_xy(outline.buffer(-(12+18)),xx,yy)
    vm=np.stack([~viaallowed,~viaallowed])
    for name,other,layers,g in objects+routed:
        if other==net:continue
        raster(mask,g.buffer(width/2+8.2),layers)
        raster(vm,g.buffer(18+8.2),layers)
    return mask,vm[0]|vm[1]
def node(x,y,l):return l*NX*NY+int(round((y-Y0)/GRID))*NX+int(round((x-X0)/GRID))
def unpack(n):
    l,n=divmod(n,NX*NY);j,i=divmod(n,NX);return l,j,i
def route(start,end,mask,vm):
    _,sx,sy,sl=start;_,tx,ty,tl=end
    targets={node(tx,ty,l-1) for l in tl}
    tj=int(round((ty-Y0)/GRID));ti=int(round((tx-X0)/GRID))
    starts=[node(sx,sy,l-1) for l in sl]
    def h(i,j):
        dx=abs(i-ti);dy=abs(j-tj);return max(dx,dy)+.41421356*min(dx,dy)
    queue=[];dist={};parent={}
    for n in starts:
        l,j,i=unpack(n)
        if not mask[l,j,i]:dist[n]=0.;heapq.heappush(queue,(h(i,j),0.,n))
    steps=0
    while queue:
        _,cost,n=heapq.heappop(queue)
        if cost!=dist.get(n):continue
        if n in targets:
            path=[n]
            while n in parent:n=parent[n];path.append(n)
            return path[::-1]
        l,j,i=unpack(n);steps+=1
        for di,dj,dc in ((1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,1.41421356),(1,-1,1.41421356),(-1,1,1.41421356),(-1,-1,1.41421356)):
            ni,nj=i+di,j+dj
            if not(0<=ni<NX and 0<=nj<NY) or mask[l,nj,ni]:continue
            if di and dj and (mask[l,j,ni] or mask[l,nj,i]):continue
            nn=l*NX*NY+nj*NX+ni
            nc=cost+dc*(1.015 if (l==0 and dj) or (l==1 and di) else 1.)
            if nc<dist.get(nn,float('inf')):
                dist[nn]=nc;parent[nn]=n;heapq.heappush(queue,(nc+h(ni,nj),nc,nn))
        if not vm[j,i] and not mask[1-l,j,i]:
            nn=(1-l)*NX*NY+j*NX+i;nc=cost+35
            if nc<dist.get(nn,float('inf')):
                dist[nn]=nc;parent[nn]=n;heapq.heappush(queue,(nc+h(i,j),nc,nn))
    return None
def add_line(net,layer,p1,p2,width):
    if math.dist(p1,p2)<1e-7:return
    row=['LINE',ident(),0,net,layer,*p1,*p2,width,0];lines.append(row)
    routed.append((row[1],net,(layer,),LineString([p1,p2]).buffer(width/2,quad_segs=16)))
def save_path(net,width,start,end,path):
    xyz=[(l+1,(xs[i],ys[j])) for l,j,i in map(unpack,path)]
    runs=[];current=[xyz[0][1]];layer=xyz[0][0]
    for newlayer,point in xyz[1:]:
        if newlayer!=layer:
            runs.append((layer,current));current=[point];layer=newlayer
            if not any(v[3]==net and math.dist(v[5:7],point)<.01 for v in vias):
                v=['VIA',ident(),0,net,'',*point,18,36,0,None,None,None,None,None,None,0]
                vias.append(v);routed.append((v[1],net,(1,2),Point(*point).buffer(18)))
        else:current.append(point)
    runs.append((layer,current))
    add_line(net,runs[0][0],start[1:3],runs[0][1][0],width)
    for layer,points in runs:
        if len(points)<2:continue
        previous=points[0];direction=None
        for k in range(1,len(points)):
            d=(round(points[k][0]-points[k-1][0],5),round(points[k][1]-points[k-1][1],5))
            if direction is not None and d!=direction:
                add_line(net,layer,previous,points[k-1],width);previous=points[k-1]
            direction=d
        add_line(net,layer,previous,points[-1],width)
    add_line(net,runs[-1][0],runs[-1][1][-1],end[1:3],width)

order=['IN12V','$1N1669','12V','7.4V','5V','3.3V']
order+=sorted(set(by_net)-set(order),key=lambda n:min(math.dist(x[1:3],y[1:3]) for x,y in itertools.combinations(by_net[n],2)) if len(by_net[n])>1 else 0)
t0=time.time()
for net in order:
    terminals=by_net[net]
    if len(terminals)<2:continue
    connected=[terminals[0]];todo=terminals[1:]
    target={'IN12V':80,'$1N1669':60,'12V':80,'7.4V':60,'5V':50,'3.3V':20}.get(net,10)
    maskcache={}
    while todo:
        _,start,end=min((math.dist(x[1:3],y[1:3]),x,y) for x in connected for y in todo)
        result=None
        # Allow short pad-access routes to retain connectivity if wide routes fail.
        widths=list(dict.fromkeys([target,min(target,40),min(target,25),min(target,10)]))
        for width in widths:
            if width not in maskcache:maskcache[width]=masks(net,width)
            mask,vm=maskcache[width]
            result=route(start,end,mask,vm)
            if result:break
        log.append({'net':net,'from':start[0],'to':end[0],'width':width,'routed':bool(result)})
        if result:save_path(net,width,start,end,result)
        connected.append(end);todo.remove(end)
    print(net,sum(x['routed'] for x in log if x['net']==net),'/',len(terminals)-1,'elapsed',round(time.time()-t0,1),flush=True)

project=json.loads(a.read('project.json'));project['config']['title']='2026Rc_R2_routed'
# VIA trailing fields must follow native source format.
from inspect_geometry import SOURCE as ORIGINAL
with zipfile.ZipFile(ORIGINAL) as original:
    native=[json.loads(l) for l in original.read(p).decode().splitlines()]
    template=next(r for r in native if r[0]=='VIA')
for i,v in enumerate(vias):
    native=list(template);native[:9]=v[:9];vias[i]=native
with zipfile.ZipFile(OUT,'w',zipfile.ZIP_DEFLATED) as z:
    for name in a.namelist():
        data=a.read(name)
        if name==p:data=('\n'.join(json.dumps(r,ensure_ascii=False) for r in rows+lines+vias)+'\n').encode()
        if name=='project.json':data=json.dumps(project,ensure_ascii=False).encode()
        z.writestr(name,data)
(OUT.parent/'r2_routing_log.json').write_text(json.dumps(log,indent=2),encoding='utf-8')
print('RESULT',len(lines),'tracks',len(vias),'vias',sum(not r['routed'] for r in log),'unrouted',flush=True)
