"""Independent geometry and compatibility checks, not a substitute for native DRC."""
import json, math, hashlib, sys
from pathlib import Path
from collections import Counter, defaultdict
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch
from shapely.strtree import STRtree
from shapely.geometry import MultiPoint
from shapely import affinity
from inspect_geometry import ROOT, SOURCE, load_geometry, records

target=Path(sys.argv[1]) if len(sys.argv)>1 else ROOT/'hardware_revision/2026Rc_R2_routed.epro'
a,p,rows,objects,pads=load_geometry(target)
b,_,original,_,_=load_geometry(SOURCE)
def parts(rr):
    names={r[3]:r[8] for r in rr if r[0]=='ATTR' and r[7]=='Designator'}
    return {names[r[1]]:r[3:7] for r in rr if r[0]=='COMPONENT'}, sorted((names[r[1]],r[2],r[3]) for r in rr if r[0]=='PAD_NET')
new,nn=parts(rows);old,on=parts(original)
fixed={n:new[n]==old[n] for n in old if n not in ('U1','U18')}
checks={'fixed_placements':fixed,'pin_nets_identical':nn==on,
        'outline_and_holes_identical':[r[2:] for r in rows if r[0]=='POLY']==[r[2:] for r in original if r[0]=='POLY'],
        'rules_identical':[r for r in rows if r[0]=='RULE']==[r for r in original if r[0]=='RULE']}
tree=STRtree([x[3] for x in objects]);viol=[]
for i,(name,net,layers,g) in enumerate(objects):
    for j in tree.query(g.buffer(4)):
        if j<=i:continue
        n2,net2,l2,g2=objects[j]
        if net==net2 or not set(layers)&set(l2):continue
        d=g.distance(g2)
        if d<3.999:viol.append([name,net,n2,net2,d])
checks['modeled_clearance_below_4mil']=viol
attrs={}
for r in rows:
    if r[0]=='ATTR':attrs.setdefault(r[3],{})[r[7]]=r[8]
bodies={}
for c in rows:
    if c[0]!='COMPONENT' or c[3]!=1:continue
    f=records(a.read('FOOTPRINT/'+attrs[c[1]]['Footprint']+'.efoo'))
    points=[]
    for r in f:
        if r[0]=='POLY' and r[4]==3 and len(r[6])==5 and r[6][2]=='L':
            points += [(r[6][0],r[6][1]),(r[6][3],r[6][4])]
    if not points:continue
    g=MultiPoint(points).convex_hull
    g=affinity.rotate(g,c[6],origin=(0,0));g=affinity.translate(g,c[4],c[5])
    bodies[attrs[c[1]]['Designator']]=g
checks['module_body_clearance_mil']={n:{k:round(g.distance(h),3) for k,h in bodies.items() if k!=n} for n,g in bodies.items() if n in ('U1','U18')}
widths=defaultdict(set)
for r in rows:
    if r[0]=='LINE' and r[4] in (1,2):widths[r[3]].add(r[9])
report={'file':str(target),'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),'checks':checks,'widths_mil':{k:sorted(v) for k,v in widths.items()},'native_drc':'NOT_VERIFIED_BY_THIS_SCRIPT'}
(ROOT/'hardware_revision/r2_geometry_audit.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf8')
fig,ax=plt.subplots(figsize=(9,11),facecolor='#101820');ax.set_facecolor('#123b31')
for layer,color in ((2,'#4389dc'),(1,'#e8554f')):
    for r in rows:
        if r[0]=='LINE' and r[4]==layer:ax.plot([r[5],r[7]],[r[6],r[8]],color=color,lw=r[9]*.18,alpha=.85,solid_capstyle='round')
for name,net,layers,g in pads:
    ax.add_patch(Patch(list(g.exterior.coords),facecolor='#c9b065',edgecolor='#edda98',lw=.5))
for n,g in bodies.items():
    ax.plot(*g.exterior.xy,color='#fff1a8',lw=1)
    ax.text(g.centroid.x,g.centroid.y,n,color='white',fontsize=8,ha='center')
for r in rows:
    if r[0]=='STRING':ax.text(r[4],r[5],r[6],color='#fff1a8',fontsize=r[8]*.11,rotation=r[13],ha='left',va='bottom')
ax.set_xlim(-2700,70);ax.set_ylim(-3970,-470);ax.set_aspect('equal');ax.axis('off')
fig.suptitle('2026Rc R2 — geometry preview (before native copper pour)',color='white',fontsize=12)
fig.savefig(ROOT/'hardware_revision/R2_geometry_preview.png',dpi=160,bbox_inches='tight');plt.close(fig)
print(json.dumps(checks,ensure_ascii=False,indent=2))
