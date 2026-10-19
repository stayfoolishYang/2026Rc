"""Verify the native EDA export against the untouched source and generated revision."""
import hashlib
import json
from collections import Counter
from inspect_geometry import ROOT, SOURCE, load_geometry, records

final = ROOT / 'hardware_revision/2026Rc_R1_DRC_checked.epro'
generated = ROOT / 'hardware_revision/2026Rc_R1b_power.epro'
a, _, pcb, _, _ = load_geometry(final)
b, _, original, _, _ = load_geometry(SOURCE)
c, _, expected, _, _ = load_geometry(generated)

def components(rows):
    names = {r[3]:r[8] for r in rows if r[0]=='ATTR' and r[7]=='Designator'}
    placements = {names[r[1]]:r[2:] for r in rows if r[0]=='COMPONENT'}
    nets = sorted((names[r[1]],r[2],r[3]) for r in rows if r[0]=='PAD_NET')
    return placements, nets

def geometry(rows, kind):
    def canonical(v):
        if isinstance(v,list): return [canonical(x) for x in v]
        if isinstance(v,(int,float)): return round(float(v),5)
        return v
    # Native writer quantizes coordinates to 0.00001 mil.
    return Counter(json.dumps(canonical(r[2:]),sort_keys=True) for r in rows if r[0]==kind)

def tracks_match(actual, planned):
    remaining=[r for r in actual if r[0]=='LINE']
    for row in (r for r in planned if r[0]=='LINE'):
        matches=[r for r in remaining if r[2:5]==row[2:5] and r[9:]==row[9:]
                 and max(abs(r[i]-row[i]) for i in range(5,9))<=0.000011]
        if len(matches)!=1: return False
        remaining.remove(matches[0])
    return not remaining

checks = {
    'component_placements_and_pin_nets_preserved': components(pcb)==components(original),
    'board_outline_and_holes_preserved': geometry(pcb,'POLY')==geometry(original,'POLY'),
    'exported_tracks_match_revision': tracks_match(pcb,expected),
    'exported_vias_match_revision': geometry(pcb,'VIA')==geometry(expected,'VIA'),
    'design_rules_preserved': [r for r in pcb if r[0]=='RULE']==[r for r in original if r[0]=='RULE'],
    'native_copper_generated': sum(r[0]=='POURED' for r in pcb)>0,
}
# Native export duplicates adjacent points in schematic graphic separator lines.
def normalize_schematic(rows):
    out=[]
    for row in rows:
        row=list(row)
        if row[0]=='POLY':
            points=list(zip(row[2][::2],row[2][1::2]))
            unique=[p for i,p in enumerate(points) if i==0 or p!=points[i-1]]
            row[2]=[v for p in unique for v in p]
        out.append(row)
    return out
for name in b.namelist():
    if name.endswith('.esch'):
        checks['schematic_preserved']=normalize_schematic(records(a.read(name)))==normalize_schematic(records(b.read(name)))
assert all(checks.values()), checks
manifest_path=final.parent/'revision_manifest.json'
manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
manifest['final_export']=str(final)
manifest['final_sha256']=hashlib.sha256(final.read_bytes()).hexdigest()
manifest['native_validation']={'app':'LCEDA Pro 3.2.127','project':'2026Rc_R1b_power','date':'2026-09-08','repour':'all 5 GND pours rebuilt','pcb_drc_errors':0,'evidence':'Native completed DRC panel showed All (0) after earlier 9 spacing errors were corrected. No rule changes.'}
manifest['export_verification']=checks
manifest['native_poured_records']=sum(r[0]=='POURED' for r in pcb)
manifest_path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(checks,indent=2))
print('SHA256',manifest['final_sha256'])
