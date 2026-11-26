"""Sample the Fusion solids along the third-edition MATLAB writing path."""
import json
import math
import runpy
from pathlib import Path
import adsk.core as core

HERE=Path(__file__).resolve().parent

def run(preview=False,design=None):
    r=runpy.run_path(str(HERE.parent/'joint_limits_fusion.py'))['Robot'](design)
    traj=json.loads((HERE/'trajectory_check_poses.json').read_text(encoding='utf8'))
    indices=list(range(len(traj['t'])))
    if preview:
        indices=sorted(set(indices[::12]+[indices[-1]]))
    log=HERE/('trajectory_preview.jsonl' if preview else 'trajectory_check.jsonl')
    relevant=[]
    drum=next(o for o in r.root.occurrences if o.component.name.startswith('Wave_Drum')).bRepBodies.item(0)
    mm=core.Application.get().measureManager
    for i,g in enumerate(r.robot):
        for o in g.childOccurrences:
            for b in o.bRepBodies:
                relevant.append((i,o.component.name,b))
    # The drum is a solid of revolution. For clearance alone, yaw leaves the
    # result invariant. Translation plus arc length bounds the motion of any
    # point of a rigid body since the previous exact distance measurement.
    cache={}
    static_clearance={}
    try:
        with log.open('w',encoding='utf8') as out:
            for idx in indices:
                t,q=traj['t'][idx],traj['q'][idx]
                own=r.collisions(q)
                close={}
                minimum=float('inf')
                for j,(g,name,b) in enumerate(relevant):
                    if g in (0,1) and j in static_clearance:
                        if g: minimum=min(minimum,static_clearance[j])
                        continue
                    pen=name=='D_Q4_Pen'
                    prev=cache.get(j)
                    if prev and not pen:
                        old,clear=prev
                        delta=abs(q[1]-old[1]) if g>=2 else 0
                        if g>=3: delta+=400*math.radians(abs(q[2]-old[2]))
                        if g==4: delta+=65*math.radians(abs(q[3]-old[3]))
                        if clear-delta>1:
                            minimum=min(minimum,clear-delta)
                            continue
                    bb=b.boundingBox
                    lo,hi=bb.minPoint,bb.maxPoint
                    maxr=10*math.hypot(max(abs(lo.x),abs(hi.x)),max(abs(lo.z),abs(hi.z)))
                    bound=min(lo.y*10,(370-maxr)/1.2)
                    if bound>1 and not pen:
                        clear=bound
                    else:
                        clear=10*mm.measureMinimumDistance(b,drum).value
                    if g in (0,1):
                        static_clearance[j]=clear
                    cache[j]=(q,clear)
                    if g and not pen: minimum=min(minimum,clear)
                    if clear<.01 and (g or idx==indices[0]):
                        close[name]=round(clear,5)
                row=dict(index=idx,t=t,q=q,robot_interferences_mm3=own,drum_contact=close,moving_clearance_lower_bound_mm=round(minimum,4))
                out.write(json.dumps(row)+'\n')
                out.flush()
    finally:
        r.restore()
    print('Checked poses',len(indices),log.name)

def recheck_heads():
    """Recheck both edited parts against every neighbour at every earlier pose."""
    before=[json.loads(s) for s in (HERE/'trajectory_check.jsonl').read_text(encoding='utf8').splitlines()]
    expected=json.loads((HERE/'trajectory_check_poses.json').read_text(encoding='utf8'))
    assert len(before)==len(expected['t']) and all(a['q']==q for a,q in zip(before,expected['q']))
    r=runpy.run_path(str(HERE.parent/'joint_limits_fusion.py'))['Robot']()
    changed_names={'D_Q4_Stub_M3','D_Q4_Transfer_stub'}
    changed=[b for b in r.bodies if b.parentComponent.name in changed_names]
    assert len(changed)==3
    # Cache conservative group-frame boxes for the unchanged geometry. Move
    # the boxes into the carriage frame analytically before invoking the kernel.
    from itertools import product
    def box(b):
        a=b.boundingBox
        return tuple((10*u,10*v) for u,v in zip(a.minPoint.asArray(),a.maxPoint.asArray()))
    items=[]
    windows=[]
    for gi,grp in enumerate(r.robot):
        natives={o.name:o for o in grp.component.occurrences}
        for occ in grp.childOccurrences:
            native=natives[occ.name]
            for k,b in enumerate(occ.bRepBodies):
                bounds=box(native.bRepBodies.item(k))
                if occ.component.name in changed_names: windows.append(bounds)
                else: items.append((gi,b,tuple(product(*bounds))))
    def relative(g,points,q):
        yaw,slide,sh,wr=q
        ang=math.radians(sh+(wr if g==4 else 0))
        c,s=math.cos(ang),math.sin(ang)
        cy,sy=math.cos(math.radians(yaw)),math.sin(math.radians(yaw))
        arm=math.radians(sh)
        def point(x,y,z):
            if g==0: return (cy*x-sy*z-slide,y-225,sy*x+cy*z)
            if g==1: return (x-slide,y-225,z)
            if g==2: return (x,y,z)
            u,v=c*x-s*y,s*x+c*y
            if g==4: u,v=u+300*math.cos(arm),v+300*math.sin(arm)
            return (u,v,z)
        return tuple((min(axis),max(axis)) for axis in zip(*(point(*p) for p in points)))
    def intersects(a,b):
        return all(u<y and x<v for (u,v),(x,y) in zip(a,b))
    try:
        with (HERE/'trajectory_verified.jsonl').open('w',encoding='utf8') as f:
            for i,row in enumerate(before):
                q=row['q']
                old=row['robot_interferences_mm3']
                keep={k:v for k,v in old.items() if not any(n in k for n in changed_names)}
                candidates=[]
                for g,body,corners in items:
                    if g==2 and i: continue
                    bounds=relative(g,corners,q)
                    if any(intersects(a,bounds) for a in windows): candidates.append(body)
                # Across all q2, the flange lies above the floor, well inside
                # the minimum wall radius. Yaw leaves this bound unchanged.
                for a in windows:
                    assert 225+a[1][0]>0 and max(math.hypot(q[1]+x,z) for x in a[0] for z in a[2])<370
                if candidates or i==0:
                    r.place(q)
                    coll=core.ObjectCollection.create()
                    for b in changed+candidates: coll.add(b)
                    inp=r.design.createInterferenceInput(coll)
                    inp.areCoincidentFacesIncluded=False
                    for hit in r.design.analyzeInterference(inp):
                        names=[b.parentComponent.name for b in (hit.entityOne,hit.entityTwo)]
                        if changed_names.intersection(names):
                            key=' | '.join(sorted(names))
                            keep[key]=keep.get(key,0)+1000*hit.interferenceBody.volume
                row['changed_part_candidates']=len(candidates)
                row['robot_interferences_mm3']=keep
                f.write(json.dumps(row)+'\n')
                f.flush()
    finally:
        r.restore()
    print('Final changed-part checks',len(before))
