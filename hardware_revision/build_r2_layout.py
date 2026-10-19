"""R2 compatible mechanical layout. Routing is generated separately and checked in EDA."""
import copy,json,zipfile,math
from pathlib import Path
from inspect_geometry import ROOT,SOURCE,records

OUT=ROOT/'hardware_revision/2026Rc_R2_layout.epro'
a=zipfile.ZipFile(SOURCE)
p=next(n for n in a.namelist() if n.endswith('.epcb'))
rows=records(a.read(p))
names={r[3]:r[8] for r in rows if r[0]=='ATTR' and r[7]=='Designator'}
# Both modules remain original footprints. 7.4V output now faces servo area.
placements={'U18':(-1295.3179,-1410,270),'U1':(-1295.3179,-2390,90)}
old={r[1]:copy.deepcopy(r) for r in rows if r[0]=='COMPONENT'}
out=[]
for r in rows:
    if r[0] in ('LINE','VIA','POUR','POURED','TEARDROP','CONNECT','STRING','IMAGE','FILL','RULE_SELECTOR'):
        continue
    if r[0]=='COMPONENT' and names[r[1]] in placements:
        r[4:7]=placements[names[r[1]]]
    if r[0]=='ATTR' and names.get(r[3]) in placements and r[5] is not None:
        previous=old[r[3]]
        nx,ny,angle=placements[names[r[3]]]
        dx,dy=r[5]-previous[4],r[6]-previous[5]
        theta=math.radians(angle-previous[6])
        r[5],r[6]=nx+dx*math.cos(theta)-dy*math.sin(theta),ny+dx*math.sin(theta)+dy*math.cos(theta)
    out.append(r)

serial=10000
def ident():
    global serial
    serial+=1
    return 'e'+str(serial)
def label(x,y,text,height=55,angle=0):
    out.append(['STRING',ident(),0,3,x,y,text,'default',height,6,0,0,3,angle,0,0,0,0])
label(-2370,-820,'2026 RC - R2',65)
label(-2370,-915,'12V / F103',50)
label(-2370,-1010,'2026.09',45)
label(-1710,-1880,'5V OUT',50)
label(-1590,-1940,'7.4V OUT',50)
label(-1730,-2870,'12V IN',50)
# External connector positions and pad pinouts are untouched.
for x,y,t,ang in [(-80,-1630,'SERVO1',90),(-2360,-2700,'SERVO4',90),
                 (-620,-1710,'S2',90),(-860,-1710,'S3',90),
                 (-1740,-1880,'U22',270),(-1770,-1210,'U23',270),
                 (-200,-1220,'12V IN',90)]: label(x,y,t,50,ang)
for y,left,right in [(-3070,'MOTOR5','MOTOR1'),(-3392,'MOTOR8','MOTOR7'),(-3715,'MOTOR4','MOTOR3')]:
    label(-1970,y,left,45);label(-1140,y,right,45)
# Use one continuous ground pour on each side, bounded by the existing outline.
for layer in (1,2):
    pour_id=ident()
    out.append(['POUR',pour_id,0,'GND',layer,0.2,'R2_GND_'+str(layer),0,[['R',-2630,-545,2630,3354,0,0]],['SOLID',8],0,0])
    out.append(['RULE_SELECTOR',['POUR',pour_id],2,{'1':'Copper_copperRegion'}])
project=json.loads(a.read('project.json'))
project['config']['title']='2026Rc_R2_layout'
project['boards']={'Board1':{'schematic':next(iter(project['schematics'])),'pcb':next(iter(project['pcbs']))}}
with zipfile.ZipFile(OUT,'w',zipfile.ZIP_DEFLATED) as z:
    for n in a.namelist():
        data=a.read(n)
        if n==p:data=('\n'.join(json.dumps(r,ensure_ascii=False) for r in out)+'\n').encode()
        if n=='project.json':data=json.dumps(project,ensure_ascii=False).encode()
        z.writestr(n,data)
print(OUT)
