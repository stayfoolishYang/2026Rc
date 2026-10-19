"""Read-only geometry inspection of the exported EasyEDA Pro board (mil)."""
import json
import math
import zipfile
from pathlib import Path
from shapely.geometry import Point, LineString, box
from shapely import affinity

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / '2026Rc_review_20260908.epro'

def records(data):
    return [json.loads(line) for line in data.decode('utf-8').splitlines() if line.strip()]

def load_geometry(source=SOURCE):
    archive = zipfile.ZipFile(source)
    pcb_path = next(n for n in archive.namelist() if n.endswith('.epcb'))
    pcb = records(archive.read(pcb_path))
    attrs = {}
    for r in pcb:
        if r[0] == 'ATTR':
            attrs.setdefault(r[3], {})[r[7]] = r[8]
    pad_nets = {(r[1], r[2]): r[3] for r in pcb if r[0] == 'PAD_NET' and r[2]}
    objects = []
    pads = []
    for c in pcb:
        if c[0] != 'COMPONENT':
            continue
        meta = attrs[c[1]]
        footprint = records(archive.read('FOOTPRINT/' + meta['Footprint'] + '.efoo'))
        for p in footprint:
            if p[0] != 'PAD':
                continue
            shape, width, height = p[10][:3]
            if shape == 'RECT':
                g = box(-width/2, -height/2, width/2, height/2)
            elif shape == 'ELLIPSE':
                g = affinity.scale(Point(0, 0).buffer(1, quad_segs=32), width/2, height/2)
            elif shape == 'OVAL':
                if width >= height:
                    g = LineString([(-(width-height)/2, 0), ((width-height)/2, 0)]).buffer(height/2)
                else:
                    g = LineString([(0, -(height-width)/2), (0, (height-width)/2)]).buffer(width/2)
            else:
                raise ValueError(shape)
            g = affinity.rotate(g, p[8], origin=(0, 0))
            g = affinity.translate(g, p[6], p[7])
            # Pro's bottom component mirror is about the local X axis.
            if c[3] == 2:
                g = affinity.scale(g, 1, -1, origin=(0, 0))
            g = affinity.rotate(g, c[6], origin=(0, 0))
            g = affinity.translate(g, c[4], c[5])
            net = pad_nets.get((c[1], p[5]), '')
            layers = (1, 2) if p[4] == 12 else (p[4],)
            item = (meta['Designator'] + '.' + p[5], net, layers, g)
            pads.append(item)
            objects.append(item)
    for r in pcb:
        if r[0] == 'VIA':
            objects.append((r[1], r[3], (1, 2), Point(r[5], r[6]).buffer(r[8]/2)))
        elif r[0] == 'LINE' and r[4] in (1, 2):
            objects.append((r[1], r[3], (r[4],), LineString([(r[5], r[6]), (r[7], r[8])]).buffer(r[9]/2)))
    return archive, pcb_path, pcb, objects, pads

if __name__ == '__main__':
    archive, pcb_path, pcb, objects, pads = load_geometry()
    for name, net, layers, g in pads:
        if name in ('J9.2', 'J11.2', 'J12.2', 'U4.81', 'U4.67', 'U18.1'):
            print(name, net, tuple(round(x, 4) for x in g.centroid.coords[0]))
    summary = {}
    blocked = []
    for r in pcb:
        if r[0] != 'LINE' or r[3] not in ('12V', '7.4V', '5V', '$1N1669'):
            continue
        center = LineString([(r[5], r[6]), (r[7], r[8])])
        distances = [(center.distance(g), name) for name, net, layers, g in objects if r[4] in layers and net != r[3]]
        distance, obstruction = min(distances)
        available = max(r[9], math.floor(2*(distance-8)))
        target = {'12V':80, '7.4V':60, '5V':50, '$1N1669':80}[r[3]]
        new_width = min(target, available)
        summary.setdefault(r[3], []).append((r[9], new_width))
        if new_width < target:
            blocked.append((r[1], r[3], r[9], new_width, obstruction, round(distance, 2)))
    from collections import Counter
    print('Width opportunities:', {k:dict(Counter(v)) for k,v in summary.items()})
    print('Bottlenecks:', blocked)
