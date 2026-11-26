"""Fusion-native revision 3 operations, on the dedicated build document only."""
from pathlib import Path
import json
import math
import runpy
import sys
import importlib
import adsk.core as core
import adsk.fusion as fusion

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
import cadlib as L
importlib.reload(L)

def start():
    app=core.Application.get()
    existing=[d for d in app.documents if d.name=='E_Writing_v3_build']
    if existing:
        existing[0].activate()
    else:
        opts=app.importManager.createFusionArchiveImportOptions(str(HERE/'E_Writing_v3_build.f3d'))
        doc=app.importManager.importToNewDocument(opts)
        doc.name='E_Writing_v3_build'
    inspect()

def guard():
    app=core.Application.get()
    if app.activeDocument.name!='E_Writing_v3_build':
        raise RuntimeError('Expected separate v3 working document, got '+app.activeDocument.name)
    return L.design()

def inspect():
    app=core.Application.get()
    print([d.name for d in app.documents],app.activeDocument.name)
    guard()
    for g,n in [('D_02','D_EF08_End_support:1'),('D_03','D_Q2_Nut_bracket:1')]:
        o=L.native_child(g,n)
        print(n,o.transform2.translation.asArray(),o.initialTransform.translation.asArray(),o.isVaildForEditInitialPosition)

def checkpoint(stage):
    design=guard()
    design.computeAll()
    problems=L.timeline_problems()
    if problems:
        raise ValueError(str(problems))
    path=HERE/('E_Writing_'+stage+'.f3d')
    if not design.exportManager.execute(design.exportManager.createFusionArchiveExportOptions(str(path))):
        raise IOError(str(path))
    print('Saved',path.name,'timeline',design.timeline.count)

def flange():
    guard()
    runpy.run_path(str(HERE/'step3_flange_round_holes.py'))['run']('')
    checkpoint('A')

def saddle():
    guard()
    runpy.run_path(str(HERE/'step45a_saddle_and_q2.py'))['run']('')
    checkpoint('AD')

def verify_slide(report_file='AD_slide_check.json'):
    design=guard()
    robot=runpy.run_path(str(HERE.parent/'joint_limits_fusion.py'))['Robot']()
    body_set=core.ObjectCollection.create()
    for group in robot.robot[1:3]:
        for o in group.childOccurrences:
            for b in o.bRepBodies:
                body_set.add(b)
    report=[]
    try:
        for q in [-1,0,*range(5,141,5),141,142]:
            robot.place([0,q,0,0])
            nut=next(o for o in robot.robot[2].childOccurrences if o.component.name=='D_Q2_Nut_bracket')
            t=nut.transform2.translation.asArray()
            assert math.dist(t,[(q-6)/10,20.1,8.6])<1e-6,(q,t)
            inp=design.createInterferenceInput(body_set)
            inp.areCoincidentFacesIncluded=False
            pairs=[]
            for hit in design.analyzeInterference(inp):
                pairs.append([hit.entityOne.parentComponent.name,hit.entityTwo.parentComponent.name,round(hit.interferenceBody.volume*1000,4)])
            report.append(dict(q2_mm=q,interferences=pairs))
    finally:
        robot.restore()
    (HERE/report_file).write_text(json.dumps(report,indent=2),encoding='utf8')
    print([(r['q2_mm'],r['interferences']) for r in report if r['interferences']][:3])

def clear_bridge():
    guard()
    comp=L.comp('D_Q2_Saddle')
    for name in ('Bridge_wall_profile','Nut_side_bridge_profile','Extended_nut_mounting_bridge_profile'):
        sk=comp.sketches.itemByName(name)
        assert L.move_points(sk,lambda x,y:abs(x+74)<.01,dx=12)==2
    checkpoint('AD_clear')
    verify_slide()

def transfer():
    guard()
    import mechanism
    importlib.reload(mechanism).fittings()
    checkpoint('F')

def motor():
    guard()
    import mechanism
    importlib.reload(mechanism).drive()
    checkpoint('G')

def arm():
    guard()
    import mechanism
    importlib.reload(mechanism).long_drive()
    checkpoint('H')

def windows():
    guard()
    import mechanism as m
    records=[]
    for name,pitch,r in [('D_GT3M_120T_W9_D8',34,12),('D_GT3M_120T_W15_D40',38,9),('D_GT3M_120T_W15_D15',34,12),('D_GT3M_80T_W9_D8',24,7)]:
        c=L.comp(name)
        body=c.bRepBodies.item(0)
        box=body.boundingBox
        z=10*box.minPoint.z
        h=10*box.maxPoint.z-z
        centres=[(pitch*math.cos(i*math.tau/6),pitch*math.sin(i*math.tau/6)) for i in range(6)]
        before=body.volume
        m.holes(c,'Six_web_lightening_windows',z-1,h+2,centres,r)
        removed=(before-body.volume)*1000
        ideal=6*math.pi*r*r*h
        assert abs(removed/ideal-1)<.01,(name,removed,ideal)
        records.append(dict(component=name,windows=6,diameter_mm=2*r,pitch_radius_mm=pitch,thickness_mm=h,removed_mm3=removed))
    checkpoint('M_wheels')
    (HERE/'wheel_windows.json').write_text(json.dumps(records,indent=2),encoding='utf8')
    print(records)

def window_dimensions():
    d=guard()
    import mechanism as m
    for name,prefix,r,diam in [('D_GT3M_120T_W9_D8','q1Mid',34,24),('D_GT3M_120T_W15_D40','q1Out',38,18),('D_GT3M_120T_W15_D15','q3Out',34,24),('D_GT3M_80T_W9_D8','q3Mid',24,14)]:
        for suffix,value in [('WindowDia',diam),('WindowRadius',r)]:
            d.userParameters.add(prefix+suffix,core.ValueInput.createByString(f'{value} mm'),'mm',name+' web windows')
        sk=L.comp(name).sketches.itemByName('Six_web_lightening_windows_profile')
        sk.timelineObject.rollTo(False)
        for circle in sk.sketchCurves.sketchCircles:
            c=circle.centerSketchPoint
            x,y=c.geometry.x*10,c.geometry.y*10
            dims=sk.sketchDimensions
            dim=dims.addDiameterDimension(circle,m.P(x+diam/2+1,y+diam/2+1))
            dim.parameter.expression=prefix+'WindowDia'
            for val,orient in [(x,fusion.DimensionOrientations.HorizontalDimensionOrientation),(y,fusion.DimensionOrientations.VerticalDimensionOrientation)]:
                if abs(val)<1e-5:
                    if orient==fusion.DimensionOrientations.HorizontalDimensionOrientation:
                        sk.geometricConstraints.addVerticalPoints(sk.originPoint,c)
                    else:
                        sk.geometricConstraints.addHorizontalPoints(sk.originPoint,c)
                else:
                    dim=dims.addDistanceDimension(sk.originPoint,c,orient,m.P(x*.5+1,y*.5+1))
                    ratio=abs(val)/r
                    dim.parameter.expression=f'{prefix}WindowRadius * {ratio:.12g}'
            assert circle.isFullyConstrained,(name,x,y)
        d.timeline.moveToEnd()
    checkpoint('dimensioned')

def roundbase():
    guard()
    import round_base
    importlib.reload(round_base).run()
    checkpoint('J')

def long_idler():
    guard()
    import mounts
    importlib.reload(mounts).long_idler()
    checkpoint('H_idler')

def geometry():
    guard()
    for g,names in [('D_03',['D_Q3_Drive_cheek','D_Q3_Outboard_bridge','D_Q3_Idler_bracket']),('D_04',['D_Q3_Link_40x40x3','D_Q3_Shaft_15','D_GT3M_120T_W15_D15']),('D_05',['D_Q4_Pen','D_Q4_Pen_carrier']),('D_02',['D_Q2_Rail_plate'])]:
        for o in L.group(g).component.occurrences:
            if o.component.name in names:
                print(o.name,o.transform2.asArray(),[f.name for f in o.component.features.extrudeFeatures])
    print('suppressed',[(i,guard().timeline.item(i).name) for i in range(guard().timeline.count) if guard().timeline.item(i).isSuppressed])

def shoulder():
    guard()
    import mounts
    importlib.reload(mounts).shoulder()
    checkpoint('I_support')

def clamp():
    guard()
    import mounts
    importlib.reload(mounts).clamp()
    checkpoint('I_clamp')

def clearances():
    guard()
    import mounts
    importlib.reload(mounts).clearances()
    checkpoint('clearances')

def pen():
    guard()
    import travel
    importlib.reload(travel).pen()
    checkpoint('K')

def pen_joint():
    guard()
    import travel
    importlib.reload(travel).pen_joint()
    checkpoint('K_slider')

def stops():
    guard()
    import travel
    importlib.reload(travel).stops()
    checkpoint('L')

def audit():
    d=guard()
    robot=runpy.run_path(str(HERE.parent/'joint_limits_fusion.py'))['Robot']()
    out=[]
    try:
        for q in [[0,80,0,0],[0,115,-42,0],[0,130,45,0],[0,0,0,0],[0,141,0,0]]:
            collisions=robot.collisions(q)
            out.append(dict(q=q,collisions=collisions))
            (HERE/'assembly_check.json').write_text(json.dumps(out,indent=2),encoding='utf8')
    finally:
        robot.restore()
    print(out[0])

def cloud_context():
    app=core.Application.get()
    print('projects',[(p.name,p.rootFolder.name) for p in app.data.dataProjects])
    for d in app.documents:
        f=d.dataFile
        print(d.name,(f.parentProject.name,f.parentFolder.name) if f else '(local)')

def export_bodies():
    guard()
    p=runpy.run_path(str(HERE.parent/'step7_export_bodies.py'))['run']
    p.__globals__.update(OUT=str(HERE/'step7_bodies'),SPIN=p.__globals__['SPIN']+('D_Q4_Transfer_pulley','D_Q1_Stage1_idler','D_Q3_First_stage_idler','D_Q4_Long_idler'))
    p('')
    check=runpy.run_path(str(HERE.parent/'step7_pose_check.py'))['run']
    check.__globals__['OUT']=str(HERE/'step7_bodies'/'pose_check.json')
    check('')

def finish():
    d=guard()
    import mechanism as m
    import mounts as n
    # Pockets on the outboard web, between the fixed screw supports.
    plate=L.comp('D_Q2_Rail_plate')
    body=plate.bRepBodies.item(0)
    slots=[]
    for x in (22,65,108):
        for y in (-83,-74,-64,-12,0,10):
            if all(body.pointContainment(core.Point3D.create((x+i)/10,(y+j)/10,.5))==fusion.PointContainment.PointInsidePointContainment for i in (-16,0,16) for j in (-6,0,6)):
                slots.append((x,y))
                break
    assert len(slots)>=2,slots
    sk=m.sketch(plate,'Web_lightening_pockets')
    for x,y in slots:
        m.slot(sk,x,y,22,4)
    m.solid(plate,sk,0,6,'Web_lightening_pockets',m.CUT,True)
    # Chamfer both fork tips away from the drum floor at full downward pitch.
    n.cut('D_04','D_Q3_Link_40x40x3','Fork_floor_relief',2,-20,20,[('poly',(304,-20.1),(320.1,-20.1),(320.1,-6.5))])
    for i in reversed(range(d.timeline.count)):
        item=d.timeline.item(i)
        if item.isSuppressed:
            item.entity.deleteMe()
    checkpoint('M')
    print('Web pocket centres',slots)

def ranges():
    guard()
    r=runpy.run_path(str(HERE.parent/'joint_limits_fusion.py'))['Robot']()
    out=[]
    drum=next(o for o in r.root.occurrences if o.component.name.startswith('Wave_Drum')).bRepBodies.item(0)
    try:
        for slide in (0,80,115,122.5,130,141):
            for angle in (-43,-42,-30,0,30,45,46):
                for wr in ((-90,0,90) if slide in (115,130) and angle in (-42,45) else (0,)):
                    pose=[0,slide,angle,wr]
                    pairs=r.collisions(pose)
                    r.place(pose)
                    relevant=[b for b in r.bodies if b.parentComponent.name in ('D_Q3_Link_40x40x3','D_GT3M_40T_W9_D8','D_Q4_Pen')]
                    clearance={b.parentComponent.name:round(core.Application.get().measureManager.measureMinimumDistance(b,drum).value*10,3) for b in relevant}
                    out.append(dict(q=pose,pairs=pairs,drum_mm=clearance))
                    (HERE/'joint_grid.json').write_text(json.dumps(out,indent=1),encoding='utf8')
        for pose in ([0,-.1,0,0],[0,141.1,0,0],*[ [0,80,0,v] for v in (-133,-132,-131,131,132,133)],*[ [v,122,-42,0] for v in (-180,-126,-90,90,126,180)]):
            pairs=r.collisions(pose)
            out.append(dict(q=pose,pairs=pairs))
        (HERE/'joint_grid.json').write_text(json.dumps(out,indent=1),encoding='utf8')
    finally:
        r.restore()
    print('Joint combinations',len(out))

def edge_check():
    guard()
    r=runpy.run_path(str(HERE.parent/'joint_limits_fusion.py'))['Robot']()
    cases=[('q2_lo',[0,0,0,0],1,0,-.2),('q2_hi',[0,141,0,0],1,141,141.2),
           ('q3_lo',[0,122.5,-42,80],2,-42,-44),('q3_hi',[0,122.5,45,-70],2,45,47),
           ('q4_lo',[0,80,0,-120],3,-120,-150),('q4_hi',[0,80,0,120],3,120,150)]
    report={}
    try:
        for label,pose,idx,free,hit in cases:
            pose[idx]=free
            assert not r.collisions(pose),(label,pose)
            pose[idx]=hit
            pair=r.collisions(pose)
            if not pair:
                report[label]={'last_clear':hit,'range_test_only':True}
                continue
            while abs(hit-free)>.05:
                mid=(hit+free)/2
                pose[idx]=mid
                c=r.collisions(pose)
                if c: hit,pair=mid,c
                else: free=mid
            report[label]=dict(last_clear=free,first_hit=hit,parts=pair)
            (HERE/'mechanical_edges.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    finally:
        r.restore()
    print(report)

def archive():
    d=guard()
    assert not L.timeline_problems()
    path=HERE/'E_Writing.f3d'
    assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(path)))
    wheel=L.comp('D_GT3M_120T_W9_D8')
    part_path=HERE/(wheel.name+'.f3d')
    assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(part_path),wheel))
    print(path,d.timeline.count)

def update_motion():
    d=guard()
    path=HERE/'writing_track.json'
    data=json.loads(path.read_text(encoding='utf8'))
    assert len(data['q'])==len(data['t']) and len(data['q'])>100
    d.rootComponent.attributes.add('RPRR','WritingTrackV3',json.dumps(data,separators=(',',':')))
    d.rootComponent.attributes.add('RPRR','Home',json.dumps(data['q'][0]))
    print('Embedded third edition trajectory, samples',len(data['t']))
    archive()

def track_pen():
    d=guard()
    import mounts as n
    r=runpy.run_path(str(HERE.parent/'joint_limits_fusion.py'))['Robot']()
    head=L.group('D_05')
    pen=n.instance('D_05','D_Q4_Pen')
    pin=n.instance('D_05','D_Q4_Pen_travel_pin')
    base=r.saved
    outputs=[]
    def err():
        expected=core.Point3D.create(5,0,0)
        expected.transformBy(head.transform2)
        tip=core.Point3D.create(0,0,3.4)
        tip.transformBy(pen.createForAssemblyContext(head).transform2)
        return round(expected.distanceTo(tip)*10,5)
    try:
        for fixed in (False,True):
            if fixed:
                pen.isGroundToParent=True
                pin.isGroundToParent=True
            errors=[]
            for q in [[0,80,0,0],[10,60,-20,25],[0,120,45,0]]:
                r.place(q)
                errors.append(err())
            outputs.append((fixed,errors))
            r.restore()
            if max(errors)<.001:
                break
        assert max(errors)<.001,outputs
    finally:
        r.restore()
    print('Nominal tip tracking errors mm',outputs)
    checkpoint('pen_tracking')

def placements():
    guard()
    for g in ('D_01','D_02','D_03','D_04','D_05'):
        for o in L.group(g).component.occurrences:
            if any(k in o.component.name for k in ('KR0','KR13','Bearing_626','XGT2','flange','Spacer_28','Pen','Motor_bracket','stub')):
                print(g,o.name,[round(t*10,2) for t in o.transform2.translation.asArray()],o.isGroundToParent)

def joint_state():
    d=guard()
    r=runpy.run_path(str(HERE.parent/'joint_limits_fusion.py'))['Robot']()
    try:
        for pose in ([0,80,0,0],[10,90,10,10]):
            r.place(pose)
            print('Pose',pose)
            for j in d.rootComponent.asBuiltJoints:
                motion=j.jointMotion
                if hasattr(motion,'rotationValue'):
                    print(j.name,math.degrees(motion.rotationValue))
                elif hasattr(motion,'slideValue'):
                    print(j.name,motion.slideValue*10)
    finally:
        r.restore()

def test_jog():
    guard()
    r=runpy.run_path(str(HERE.parent/'joint_limits_fusion.py'))['Robot']()
    api=runpy.run_path(str(HERE.parent/'MoveWithinDrum'/'MoveWithinDrum.py'))
    try:
        q=api['move']([0,122.5,0,0])
        assert 0<q<122.5
        print('Requested 122.5, constrained to',q,'mm')
        (HERE/'jog_check.json').write_text(json.dumps(dict(q=[0,122.5,0,0],actual_q2_mm=q),indent=2),encoding='utf8')
    finally:
        r.restore()

def deck():
    guard()
    for c in (L.comp('D_Q2_Rail_plate'),L.comp('D_Q2_Saddle')):
        for sk in c.sketches:
            pts=[[round(p.geometry.x*10,1),round(p.geometry.y*10,1)] for p in sk.sketchPoints if p.connectedEntities and p.connectedEntities.count]
            if 3<len(pts)<20:
                print(c.name,sk.name,pts)
    for c in guard().allComponents:
        if 'buffer' in c.name:
            print('Buffer material',c.name,c.material.name)

def belt_channel():
    guard()
    sk=L.comp('D_Q2_Rail_plate').sketches.itemByName('Q4_belt_and_pulley_clearance_profile')
    assert L.move_points(sk,lambda x,y:abs(x-132)<.02,dx=-94)==1
    assert L.move_points(sk,lambda x,y:abs(x-142)<.02,dx=-104)==1
    # Keep 0.5 mm of nut clearance when the carriage first touches the front buffer.
    for name in ('D_EF08_End_support:1','D_Q2_Support_riser:2'):
        L.shift(L.native_child('D_02',name),dx=.5)
    screw=L.comp('D_Q2_Ballscrew_12x10').features.extrudeFeatures.itemByName('Support_journal')
    L.set_distance(screw,17.5)
    checkpoint('belt_channel')
    import mechanism as m
    br=L.comp('D_Q4_Motor_bracket')
    print('Bracket bottom',br.bRepBodies.item(0).boundingBox.minPoint.y)

def restore_mounts():
    d=guard()
    import mechanism as m
    app=core.Application.get()
    work=app.activeDocument
    baseline=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(HERE/'E_Writing_v3_build.f3d')))
    baseline.name='V2_mount_reference'
    targets={}
    try:
        for g in ('D_02','D_03','D_04','D_05'):
            for o in L.group(g).component.occurrences:
                name=o.name.replace('D_Motor_slotted_flange','D_Motor_flange')
                t=o.transform2.copy()
                shift=t.translation
                if (g,name) in (('D_02','D_EF08_End_support:1'),('D_02','D_Q2_Support_riser:2'),('D_03','D_R12_10K3_Nut:1'),('D_03','D_Q2_Nut_bracket:1')):
                    shift.x+=1
                if (g=='D_03' and o.component.name in ('D_GT3M_24T_W15_D8','D_Belt_Q3_stage2','D_Belt_idler_15','D_Idler_axle_6','D_Q3_Idler_bracket')) or (g=='D_04' and o.component.name=='D_GT3M_120T_W15_D15'):
                    shift.z+=.7
                if g=='D_05' and o.component.name=='D_GT3M_40T_W9_D8':
                    shift.z-=.2
                t.translation=shift
                targets[g,name]=t
    finally:
        work.activate()
        baseline.close(False)
    x,y=-22.5,45*math.sqrt(3)/2
    replacement=[('D_HG_KR053:5',(x,y,109.5),m.reverse_axes),('D_XGT2_19C_8x6:5',(x,y,96.5),m.reverse_axes),('D_Bearing_626:10',(0,0,48),m.identity_axes),('D_Bearing_626:12',(x,y,56),m.identity_axes),('D_Bearing_626:14',(x,y,64),m.identity_axes)]
    entries=[]
    for g in ('D_02','D_03','D_04','D_05'):
        for o in L.group(g).component.occurrences:
            if (g,o.name) in targets:
                entries.append((o.timelineObject.index,g,o,targets[g,o.name]))
    def diff(o,t):
        return max(abs(a-b) for a,b in zip(o.transform2.asArray(),t.asArray()))
    changed=[]
    for i,g,o,t in sorted(entries,key=lambda e:e[0]):
        if diff(o,t)>1e-6:
            changed.append([g,o.name])
            m.position(o,t)
    for name,pos,axes in replacement:
        old=L.native_child('D_03',name)
        c=old.component
        old.deleteMe()
        m.place('D_03',c,pos,axes)
    assert all(diff(o,t)<1e-6 for _,_,o,t in entries)
    checkpoint('mounts_verified')
    print('Rechecked',len(entries),'occurrences; corrected',changed)

def fasteners():
    guard()
    import mounts
    importlib.reload(mounts).fasteners()
    checkpoint('assembled')

def mount_hardware():
    guard()
    import mounts
    importlib.reload(mounts).mount_hardware()
    checkpoint('hardware')

def flush_foot():
    guard()
    import mechanism as m
    screw=L.comp('D_Q4_Foot_M4').features.extrudeFeatures
    L.set_distance(screw.itemByName('Grip'),1.5)
    L.set_start_offset(screw.itemByName('Socket_head'),6.5)
    L.set_distance(screw.itemByName('Socket_head'),2.5)
    L.set_start_offset(screw.itemByName('Socket_recess'),7.5)
    L.set_distance(screw.itemByName('Socket_recess'),1.5)
    br=L.comp('D_Q4_Motor_bracket')
    sk=br.sketches.add(br.xZConstructionPlane)
    sk.name='Sliding_clamp_head_recesses_profile'
    p=lambda xx,yy:sk.modelToSketchSpace(core.Point3D.create((xx+22.5)/10,(yy-45*math.sqrt(3)/2)/10,9.7))
    for x in (-42,-9):
        a=p(x,10)
        m.slot(sk,a.x*10,a.y*10,6,3.6)
    a,b=p(0,7.5).z*10,p(0,10).z*10
    m.solid(br,sk,a,b-a,'Sliding_clamp_head_recesses',m.CUT,True)
    checkpoint('flush')

def recess_heads():
    guard()
    import mounts
    importlib.reload(mounts).recess_stub_fasteners()
    checkpoint('recessed')

def recheck_heads():
    guard()
    import trajectory_verify
    importlib.reload(trajectory_verify).recheck_heads()

def verify_delta():
    import hashlib
    app=core.Application.get()
    work=app.activeDocument
    final=guard()
    def geometry(d):
        parts={}
        instances={}
        for g in d.rootComponent.occurrences:
            if not g.component.name.startswith('D_0'): continue
            for o in g.component.occurrences:
                c=o.component
                instances[g.component.name+'/'+o.name]=[round(v,8) for v in o.transform2.asArray()]
                if c.name in parts: continue
                rows=[]
                for b in c.bRepBodies:
                    corners=sorted(tuple(round(t,6) or 0.0 for t in v.geometry.asArray()) for v in b.vertices)
                    rows.append([b.faces.count,b.edges.count,round(b.volume,6),round(b.area,6),corners])
                parts[c.name]=hashlib.sha256(json.dumps(sorted(rows),separators=(',',':')).encode()).hexdigest()
        return parts,instances
    f,p=geometry(final)
    doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(HERE/'E_Writing_flush.f3d')))
    try:
        old,loc=geometry(L.design())
    finally:
        work.activate()
        doc.close(False)
    changed=sorted(k for k in f if old.get(k)!=f[k])
    shifted=sorted(k for k in p if loc.get(k)!=p[k])
    assert set(f)==set(old) and set(p)==set(loc)
    assert set(changed)=={'D_Q4_Stub_M3','D_Q4_Transfer_stub'} and not shifted,(changed,shifted)
    result=dict(unique_robot_parts=len(f),part_instances=len(p),changed_geometry=changed,moved_instances=shifted)
    (HERE/'release_geometry_delta.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(result)

def preview_path():
    guard()
    import trajectory_verify
    importlib.reload(trajectory_verify).run(True)

def full_path():
    app=core.Application.get()
    doc=next(d for d in app.documents if d.name=='E_Writing_v3_build')
    design=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    assert design and design.rootComponent.attributes.itemByName('RPRR','WritingTrackV3')
    import trajectory_verify
    importlib.reload(trajectory_verify).run(False,design)

def drawing_source():
    app=core.Application.get()
    project=next(p for p in app.data.dataProjects if p.name=='Default Project')
    folders=project.rootFolder.dataFolders
    f=folders.itemByName('RPRR - Third Edition') or folders.add('RPRR - Third Edition')
    options=app.importManager.createFusionArchiveImportOptions(str(HERE/'E_Writing.f3d'))
    source=app.importManager.importToNewDocument(options)
    drawing_design=L.design()
    for o in drawing_design.rootComponent.occurrences:
        if o.component.name.startswith('Wave_Drum'):
            o.isLightBulbOn=False
    for sk in drawing_design.rootComponent.sketches:
        sk.isVisible=False
    if not source.saveAs('E_Writing_V3',f,'Third edition drawing source',''):
        raise RuntimeError('Fusion saveAs did not complete')
    print(source.name)

def drawings():
    import adsk.drawing as dr
    app=core.Application.get()
    project=next(p for p in app.data.dataProjects if p.name=='Default Project')
    folder=project.rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    source=next(f for f in folder.dataFiles if f.name=='E_Writing_V3')
    assert source.isComplete,'Source still uploading'
    manager=dr.DrawingManager.get()
    spec=manager.createDrawingInput(source,dr.DrawingCreationModes.AutomaticDrawingCreationMode)
    spec.content=dr.DrawingContentTypes.VisibleOnlyDrawingContentType
    spec.sheetCreationType=dr.SheetCreationTypes.AllLevelsSheetCreationType
    settings=spec.automationPreferences.globalPreferences
    settings.isMainAssemblySheetGenerated=True
    settings.isSubAssemblySheetGenerated=True
    settings.isComponentSheetGenerated=True
    settings.isAnimationSheetGenerated=True
    settings.isAutoDimensionEnabled=True
    settings.isDetectAndOmitFasteners=False
    file=manager.createDrawing(spec)
    assert file and 'Exploded' not in file.name,'Expected main design drawing'
    (HERE/'drawing_source.json').write_text(json.dumps(dict(source=source.id,drawing=file.id,folder=folder.name),indent=2),encoding='utf8')
    print(file.name,file.id)

def drawing_progress():
    import adsk.drawing as dr
    app=core.Application.get()
    project=next(p for p in app.data.dataProjects if p.name=='Default Project')
    folder=project.rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    print('Cloud files',[(f.name,f.fileExtension,f.isComplete) for f in folder.dataFiles])
    for d in app.documents:
        art=dr.DrawingDocument.cast(d)
        if art:
            print('Drawing',d.name,'sheets',art.drawing.sheets.count,'examples',[s.name for s in list(art.drawing.sheets)[:3]],'active',art.drawing.activeSheet.name if art.drawing.activeSheet else None)

def export_drawing():
    import adsk.drawing as dr
    app=core.Application.get()
    meta=json.loads((HERE/'drawing_source.json').read_text(encoding='utf8'))
    data=app.data.findFileById(meta['drawing'])
    assert 'Exploded' not in data.name, data.name
    doc=next((d for d in app.documents if d.dataFile and d.dataFile.id==data.id),None) or app.documents.open(data)
    doc.activate()
    drawing=dr.DrawingDocument.cast(doc).drawing
    if any(ref.isOutOfDate for ref in doc.documentReferences):
        assert doc.updateAllReferences()
        assert doc.save('Update views to released shoulder flange')
    empty=[s for s in drawing.sheets if s.views.count==0 and s.name in ('Sheet1','\u56fe\u7eb81')]   # Chinese-language Fusion names: Sheet1
    if empty and drawing.sheets.count>10:
        for sheet in empty:
            assert sheet.deleteMe()
        assert doc.save('Remove the unused template sheet')
    meta['references']=[dict(name=r.dataFile.name,version=r.version,out_of_date=r.isOutOfDate) for r in doc.documentReferences]
    assert not any(r['out_of_date'] for r in meta['references']),meta['references']
    meta['drawing_url']=data.fusionWebURL
    meta['source_url']=app.data.findFileById(meta['source']).fusionWebURL
    (HERE/'drawing_source.json').write_text(json.dumps(meta,indent=2),encoding='utf8')
    sheets=[dict(name=s.name,views=s.views.count,dimensions=s.drawingDimensions.count) for s in getattr(drawing,'sheets',[drawing.activeSheet]) if s]
    (HERE/'drawing_sheets.json').write_text(json.dumps(sheets,ensure_ascii=False,indent=2),encoding='utf8')
    print('Sheets',len(sheets),'examples',sheets[:4])
    mgr=drawing.exportManager
    spec=mgr.createPDFExportOptions(str(HERE/'E_Writing_v3_drawings.pdf'))
    spec.sheetsToExport=dr.PDFSheetsExport.AllPDFSheetsExport
    spec.openPDF=False
    assert mgr.execute(spec)
    print('PDF exported')

def update_cloud_source():
    import mounts
    app=core.Application.get()
    src=next(d for d in app.documents if d.dataFile and d.dataFile.fileExtension=='f3d' and d.dataFile.name=='E_Writing_V3')
    src.activate()
    comp=L.comp('D_Q4_Transfer_stub')
    if not comp.features.extrudeFeatures.itemByName('Flush_M3_counterbores'):
        importlib.reload(mounts).recess_stub_fasteners()
        assert not L.timeline_problems()
        if not src.save('Recess the shoulder transfer flange mounting screws'):
            raise RuntimeError('Fusion source revision could not be saved')
    print('Drawing source version',src.dataFile.versionNumber)

def resume_build():
    app=core.Application.get()
    next(d for d in app.documents if d.name=='E_Writing_v3_build').activate()
    guard()

def verify_archive():
    app=core.Application.get()
    doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(HERE/'E_Writing.f3d')))
    doc.name='V3_archive_readback'
    try:
        d=L.design()
        wheel=L.comp('D_GT3M_120T_W9_D8')
        sk=wheel.sketches.itemByName('Six_web_lightening_windows_profile')
        assert all(c.isFullyConstrained for c in sk.sketchCurves.sketchCircles)
        p=d.userParameters.itemByName('q1MidWindowDia')
        wheel_body=wheel.bRepBodies.item(0)
        before=wheel_body.volume
        p.expression='23 mm'
        d.computeAll()
        delta=wheel_body.volume-before
        expected=6*math.pi*(12**2-11.5**2)*11/1000
        assert abs(delta-expected)<.005,(delta,expected)
        p.expression='24 mm'
        d.computeAll()
        assert abs(wheel_body.volume-before)<1e-5 and not L.timeline_problems()
        track=json.loads(d.rootComponent.attributes.itemByName('RPRR','WritingTrackV3').value)
        result=dict(timeline=d.timeline.count,components=d.allComponents.count,assembly_joints=d.rootComponent.asBuiltJoints.count,
                    sketches=sum(c.sketches.count for c in d.allComponents),motion_samples=len(track['t']),duration_s=track['t'][-1],
                    suppressed=sum(d.timeline.item(i).isSuppressed for i in range(d.timeline.count)),
                    pen_joints=L.group('D_05').component.asBuiltJoints.count,
                    q1_window_dimensions=sk.sketchDimensions.count,q1_window_resize_test_delta_cm3=delta,health='OK')
        assert result['suppressed']==0 and result['assembly_joints']==4
        (HERE/'archive_readback.json').write_text(json.dumps(result,indent=2),encoding='utf8')
        print(result)
    finally:
        next(d for d in app.documents if d.name=='E_Writing_v3_build').activate()
        doc.close(False)

def exploded_source():
    # Separate presentation document. Preserve all joints in the working assembly.
    import mechanism as m
    app=core.Application.get()
    opt=app.importManager.createFusionArchiveImportOptions(str(HERE/'E_Writing.f3d'))
    doc=app.importManager.importToNewDocument(opt)
    d=L.design()
    root=d.rootComponent
    for j in list(root.asBuiltJoints):
        j.deleteMe()
    for o in root.occurrences:
        if o.component.name.startswith('Wave_Drum'): o.isLightBulbOn=False
    for sk in root.sketches: sk.isVisible=False
    poses={'D_01':(0,0,0),'D_02':(0,160,0),'D_03':(80,510,0),
           'D_04':(190,540,120),'D_05':(585,540,180)}
    for g,p in poses.items():
        o=L.group(g)
        if g=='D_01': continue
        o.isGroundToParent=False
        o.transform2=m.matrix(p)
    if d.snapshots.hasPendingSnapshot: d.snapshots.add()
    for g,p in poses.items():
        actual=L.group(g).transform2.translation.asArray()
        assert math.dist(p,[10*x for x in actual])<.01,(g,p,actual)
    project=next(p for p in app.data.dataProjects if p.name=='Default Project')
    folder=project.rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    assert doc.saveAs('E_Writing_V3_Exploded',folder,'Third-edition assembly sequence','')
    print('Exploded assembly saved',poses)

def exploded_drawing():
    import adsk.drawing as dr
    app=core.Application.get()
    project=next(p for p in app.data.dataProjects if p.name=='Default Project')
    folder=project.rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    src=next(d for d in folder.dataFiles if d.name=='E_Writing_V3_Exploded')
    assert src.isComplete
    mgr=dr.DrawingManager.get()
    spec=mgr.createDrawingInput(src,dr.DrawingCreationModes.AutomaticDrawingCreationMode)
    spec.content=dr.DrawingContentTypes.VisibleOnlyDrawingContentType
    prefs=spec.automationPreferences.globalPreferences
    prefs.isSubAssemblySheetGenerated=False
    prefs.isComponentSheetGenerated=False
    prefs.isAnimationSheetGenerated=False
    data=mgr.createDrawing(spec)
    assert data
    (HERE/'exploded_drawing.json').write_text(json.dumps(dict(id=data.id,url=data.fusionWebURL)),encoding='utf8')
    print('Exploded drawing',data.name)


def final_slide():
    verify_slide('final_slide_check.json')

def recover_drawing():
    import adsk.drawing as dr
    app=core.Application.get()
    project=next(p for p in app.data.dataProjects if p.name=='Default Project')
    folder=project.rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    source=next(f for f in folder.dataFiles if f.name=='E_Writing_V3' and f.fileExtension=='f3d')
    file=next((f for f in folder.dataFiles if f.fileExtension=='f2d' and 'Exploded' not in f.name),None)
    if not file:
        candidates=[dr.DrawingDocument.cast(d) for d in app.documents if dr.DrawingDocument.cast(d)]
        doc=next((d for d in candidates if d.drawing.sheets.count>10 and any(s.name=='D_GT3M_120T_W9_D8' for s in d.drawing.sheets)),None)
        if not doc:
            raise RuntimeError('No populated Fusion drawing found')
        if not doc.dataFile:
            assert doc.saveAs('E_Writing_V3_Drawings',folder,'Third edition assembly and part drawings','')
        file=doc.dataFile
    (HERE/'drawing_source.json').write_text(json.dumps(dict(source=source.id,drawing=file.id,folder=folder.name),indent=2),encoding='utf8')
    print('Drawing recovered',file.name)

def export_exploded():
    import adsk.drawing as dr
    app=core.Application.get()
    meta=json.loads((HERE/'exploded_drawing.json').read_text(encoding='utf8'))
    file=app.data.findFileById(meta['id'])
    doc=next((d for d in app.documents if d.dataFile and d.dataFile.id==file.id),None) or app.documents.open(file)
    doc.activate()
    if any(r.isOutOfDate for r in doc.documentReferences):
        assert doc.updateAllReferences() and doc.save('Refresh released hardware materials')
    drawing=dr.DrawingDocument.cast(doc).drawing
    mgr=drawing.exportManager
    opt=mgr.createPDFExportOptions(str(HERE/'E_Writing_v3_exploded.pdf'))
    opt.sheetsToExport=dr.PDFSheetsExport.AllPDFSheetsExport
    opt.openPDF=False
    assert mgr.execute(opt)
    print('Exploded sheets',[(s.name,s.views.count) for s in getattr(drawing,'sheets',[drawing.activeSheet]) if s])


def sheet_status():
    import adsk.drawing as dr
    docs=[dr.DrawingDocument.cast(d) for d in core.Application.get().documents]
    doc=next(d for d in docs if d and d.drawing.sheets.count>20)
    rows=[(s.name,s.views.count,s.drawingDimensions.count) for s in doc.drawing.sheets]
    name=doc.drawing.activeSheet.name
    ix=next(i for i,r in enumerate(rows) if r[0]==name)
    print(doc.name,len(rows),'active',ix+1,name,'viewless',sum(not v for n,v,k in rows),'dimensioned',sum(k>0 for n,v,k in rows),'last six',rows[-6:])


def part_number_status():
    import adsk.drawing as dr
    app=core.Application.get()
    for doc in app.documents:
        if doc.dataFile and doc.dataFile.name in ('E_Writing_V3','E_Writing_V3_Exploded') and doc.dataFile.fileExtension=='f3d':
            d=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
            print(doc.name,[(c.name,c.partNumber) for c in list(d.allComponents)[:7]])
        drawing=dr.DrawingDocument.cast(doc)
        if drawing and doc.dataFile:
            sh=drawing.drawing.sheets.item(0)
            print('Table API',[n for n in dir(sh) if 'art' in n or 'able' in n])


def drawing_coverage():
    import adsk.drawing as dr
    app=core.Application.get()
    work=next(d for d in app.documents if d.name=='E_Writing_v3_build')
    d=fusion.Design.cast(work.products.itemByProductType('DesignProductType'))
    docs=[dr.DrawingDocument.cast(doc) for doc in app.documents]
    document=next(doc for doc in docs if doc and doc.drawing.sheets.count>20)
    sheets={sh.name for sh in document.drawing.sheets if sh.views.count}
    groups=[o for o in d.rootComponent.occurrences if o.component.name.startswith('D_0')]
    parts={o.component.name for g in groups for o in g.component.occurrences}
    missing=sorted(parts-sheets)
    print('Additional sheet names',sorted(sheets-parts))
    print('Missing statuses',[(o.component.name,o.isLightBulbOn,o.component.bRepBodies.count) for g in groups for o in g.component.occurrences if o.component.name in missing])
    absent_groups=sorted(g.component.name for g in groups if g.component.name not in sheets)
    report=dict(unique_parts=len(parts),represented_parts=len(parts)-len(missing),missing_parts=missing,missing_groups=absent_groups)
    (HERE/'drawing_coverage.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(report)


def detail_source():
    app=core.Application.get()
    names=set(json.loads((HERE/'drawing_coverage.json').read_text(encoding='utf8'))['missing_parts'])
    assert names
    options=app.importManager.createFusionArchiveImportOptions(str(HERE/'E_Writing.f3d'))
    doc=app.importManager.importToNewDocument(options)
    root=L.design().rootComponent
    visible=set()
    for group in root.occurrences:
        if not group.component.name.startswith('D_0'):
            group.isLightBulbOn=False
            continue
        for o in group.component.occurrences:
            show=o.component.name in names
            o.isLightBulbOn=show
            if show: visible.add(o.component.name)
        group.isLightBulbOn=any(o.component.name in names for o in group.component.occurrences)
    for sk in root.sketches: sk.isVisible=False
    assert visible==names
    project=next(p for p in app.data.dataProjects if p.name=='Default Project')
    folder=project.rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    assert doc.saveAs('E_Writing_V3_Details',folder,'Additional detail sheets extracted from the released assembly','')
    print('Supplementary detail source',sorted(visible))

def detail_drawings():
    import adsk.drawing as dr
    app=core.Application.get()
    project=next(p for p in app.data.dataProjects if p.name=='Default Project')
    folder=project.rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    source=next(f for f in folder.dataFiles if f.name=='E_Writing_V3_Supplement')
    assert source.isComplete
    mgr=dr.DrawingManager.get()
    spec=mgr.createDrawingInput(source,dr.DrawingCreationModes.AutomaticDrawingCreationMode)
    spec.content=dr.DrawingContentTypes.VisibleOnlyDrawingContentType
    spec.sheetCreationType=dr.SheetCreationTypes.AllLevelsSheetCreationType
    prefs=spec.automationPreferences.globalPreferences
    prefs.isMainAssemblySheetGenerated=True
    prefs.isSubAssemblySheetGenerated=True
    prefs.isComponentSheetGenerated=True
    prefs.isAutoDimensionEnabled=True
    prefs.isAnimationSheetGenerated=False
    prefs.isDetectAndOmitFasteners=False
    prefs.omitComponentsWithKeywords='__omit_none__'
    assert prefs.omitComponentsWithKeywords=='__omit_none__'
    data=mgr.createDrawing(spec)
    assert data and 'Supplement' in data.name
    (HERE/'detail_drawing.json').write_text(json.dumps(dict(id=data.id,url=data.fusionWebURL)),encoding='utf8')
    print(data.name)

def export_detail():
    import adsk.drawing as dr
    app=core.Application.get()
    meta=json.loads((HERE/'detail_drawing.json').read_text(encoding='utf8'))
    file=app.data.findFileById(meta['id'])
    doc=next((d for d in app.documents if d.dataFile and d.dataFile.id==file.id),None) or app.documents.open(file)
    doc.activate()
    drawing=dr.DrawingDocument.cast(doc).drawing
    needed=set(json.loads((HERE/'drawing_coverage.json').read_text(encoding='utf8'))['missing_parts'])
    mapping=json.loads((HERE/'detail_numbers.json').read_text(encoding='utf8'))
    for sh in drawing.sheets:
        if sh.name in mapping:
            sh.name=mapping[sh.name]
    if needed.issubset(sh.name for sh in drawing.sheets if sh.views.count):
        for sh in list(drawing.sheets):
            if sh.name not in needed:
                assert sh.deleteMe()
    sheets=[dict(name=s.name,views=s.views.count,dimensions=s.drawingDimensions.count) for s in drawing.sheets]
    assert needed.issubset(s['name'] for s in sheets if s['views']),(needed,sheets)
    if doc.isModified: assert doc.save('Verified detail sheets')
    (HERE/'detail_sheets.json').write_text(json.dumps(sheets,ensure_ascii=False,indent=2),encoding='utf8')
    mgr=drawing.exportManager
    opt=mgr.createPDFExportOptions(str(HERE/'E_Writing_v3_details.pdf'))
    opt.sheetsToExport=dr.PDFSheetsExport.AllPDFSheetsExport
    opt.openPDF=False
    assert mgr.execute(opt)
    print('Verified supplementary parts',sorted(needed),'sheets',sheets)


def main_drawing_doc():
    app=core.Application.get()
    meta=json.loads((HERE/'drawing_source.json').read_text(encoding='utf8'))
    data=app.data.findFileById(meta['drawing'])
    return next((d for d in app.documents if d.dataFile and d.dataFile.id==data.id),None) or app.documents.open(data)

def wheel_curves():
    import adsk.drawing as dr
    doc=main_drawing_doc()
    doc.activate()
    drawing=dr.DrawingDocument.cast(doc).drawing
    for name in ('D_GT3M_120T_W9_D8','D_Q4_Transfer_stub'):
        s=drawing.sheets.itemByName(name)
        print(name,'sheet',s.width,s.height,[k for k in dir(s) if 'activ' in k.lower()])
        for view in s.views:
            groups={}
            curves=view.viewCurves
            for j in range(min(100,curves.count)):
                c=curves.item(j)
                if c.type!=dr.ViewCurveTypes.CircleViewCurveType: continue
                center=c.centerPoint
                edge=c.startPoint
                if center:
                    xy=tuple(round(t,3) for t in center.coordinate.asArray())
                    radius=round(center.coordinate.distanceTo(edge.coordinate),3) if edge else None
                    groups.setdefault(xy,[]).append(radius)
            print('view',curves.count,groups)


def detail_pack():
    import tempfile
    import mechanism as m
    app=core.Application.get()
    work=next(d for d in app.documents if d.name=='E_Writing_v3_build')
    design=fusion.Design.cast(work.products.itemByProductType('DesignProductType'))
    names=json.loads((HERE/'drawing_coverage.json').read_text(encoding='utf8'))['missing_parts']
    components={c.name:c for c in design.allComponents if c.name in names}
    assert len(components)==len(names)
    with tempfile.TemporaryDirectory(prefix='Fusion_RPRR_details_') as staging:
        for name,c in components.items():
            path=str(Path(staging)/(name+'.f3d'))
            mgr=design.exportManager
            assert mgr.execute(mgr.createFusionArchiveExportOptions(path,c))
        doc=app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
        root=L.design().rootComponent
        for i,name in enumerate(names):
            path=str(Path(staging)/(name+'.f3d'))
            opt=app.importManager.createFusionArchiveImportOptions(path)
            result=app.importManager.importToTarget2(opt,root)
            assert result and result.count
            occ=root.occurrences.item(root.occurrences.count-1)
            if occ.component.name!=name: occ.component.name=name
            assert occ.component.bRepBodies.count==components[name].bRepBodies.count
            occ.transform2=m.matrix((150*(i%3),150*(i//3),0))
    if L.design().snapshots.hasPendingSnapshot: L.design().snapshots.add()
    assert {o.component.name for o in root.occurrences}==set(names)
    project=next(p for p in app.data.dataProjects if p.name=='Default Project')
    folder=project.rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    assert doc.saveAs('E_Writing_V3_Parts_2',folder,'Seven remaining detail drawings, native part copies','')
    print('Saved standalone details',names)


def drawing_parts():
    from collections import Counter
    app=core.Application.get()
    work=next(d for d in app.documents if d.name=='E_Writing_v3_build')
    design=fusion.Design.cast(work.products.itemByProductType('DesignProductType'))
    rows=[]
    for g in design.rootComponent.occurrences:
        if not g.component.name.startswith('D_0'): continue
        children=list(g.component.occurrences)
        counts=Counter(o.component.name for o in children)
        for name,n in counts.items():
            c=next(o.component for o in children if o.component.name==name)
            materials=sorted({b.material.name for b in c.bRepBodies if b.material})
            rows.append(dict(group=g.component.name,part=name,quantity=n,materials=materials))
    (HERE/'drawing_parts.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf8')
    print('Materials and quantities for',sum(r['quantity'] for r in rows),'instances')


def dimension_q1_drawing():
    import adsk.drawing as dr
    doc=main_drawing_doc()
    doc.activate()
    drawing=dr.DrawingDocument.cast(doc).drawing
    sheet=drawing.sheets.itemByName('D_GT3M_120T_W9_D8')
    dims=sheet.drawingDimensions
    if dims.count>=5: return
    assert dims.count==2
    face=next(v for v in sheet.views if v.viewCurves.count==8)
    circles=[]
    for c in face.viewCurves:
        center=c.centerPoint
        edge=c.startPoint
        if center and edge: circles.append((center.coordinate.distanceTo(edge.coordinate),c))
    windows=[c for r,c in circles if abs(r-12)<.01]
    bore=next(c for r,c in circles if abs(r-4)<.01)
    assert len(windows)==6
    xx,yy=bore.centerPoint.coordinate.asArray()
    place=lambda x,y: dr.DrawingPoint.create(core.Point2D.create(xx+x,yy+y))
    upper=max(windows,key=lambda c:c.centerPoint.coordinate.y)
    left=min(windows,key=lambda c:c.centerPoint.coordinate.x)
    right=max(windows,key=lambda c:c.centerPoint.coordinate.x)
    assert abs(left.centerPoint.coordinate.distanceTo(right.centerPoint.coordinate)-68)<.01
    assert dims.addDiameterDimension(upper,place(55,63))
    assert dims.addDiameterDimension(bore,place(-52,59))
    assert dims.addLinearDimension(left.centerPoint,right.centerPoint,place(0,-67),True)
    assert dims.count==5 and doc.save('Dimension the six-window q1 pulley, pitch circle and shaft bore')
    print(sheet.name,'native detail dimensions',dims.count)


def detail_visibility():
    app=core.Application.get()
    wk=next(d for d in app.documents if d.name=='E_Writing_v3_build')
    names=set(json.loads((HERE/'drawing_coverage.json').read_text(encoding='utf8'))['missing_parts'])
    names.add('D_GT3M_120T_W9_D8')
    d=fusion.Design.cast(wk.products.itemByProductType('DesignProductType'))
    for c in d.allComponents:
        if c.name in names:
            print(c.name,c.isBodiesFolderLightBulbOn,[(b.isLightBulbOn,b.isVisible) for b in c.bRepBodies])
    doc=next(d for d in app.documents if d.dataFile and d.name=='E_Writing_V3_Parts_2')
    d=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    for o in d.rootComponent.occurrences:
        print(o.name,o.isLightBulbOn,o.component.isBodiesFolderLightBulbOn,[(b.isLightBulbOn,b.isVisible) for b in o.bRepBodies])


def set_nominal_hardware(design):
    steel=next(m for m in design.materials if m.name.startswith('RPRR Bearing steel') and '(7850.00 kg/m3)' in m.name)
    changed=[]
    for c in design.allComponents:
        if c.name=='D_MGN12H_Block': continue
        bodies=[b for b in c.bRepBodies if b.material and '4415.88 kg/m3' in b.material.name]
        if not bodies: continue
        c.material=steel
        for b in bodies:
            if b.material.name!=steel.name: b.material=steel
            assert b.material.name==steel.name
        changed.append(c.name)
    return sorted(changed)

def final_materials():
    d=guard()
    changed=set_nominal_hardware(d)
    assert len(changed) in (0,18),changed
    if changed:
        (HERE/'steel_material_update.json').write_text(json.dumps(changed,indent=2),encoding='utf8')
    assert d.computeAll() and not L.timeline_problems()
    archive()
    print('Nominal solid steel',changed)

def drawing_materials():
    app=core.Application.get()
    project=next(p for p in app.data.dataProjects if p.name=='Default Project')
    folder=project.rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    for name in ['E_Writing_V3','E_Writing_V3_Exploded','E_Writing_V3_Parts_2']:
        data=next(f for f in folder.dataFiles if f.name==name and f.fileExtension=='f3d')
        doc=app.documents.open(data)
        doc.activate()
        design=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
        changed=set_nominal_hardware(design)
        if changed:
            assert doc.save('Nominal bearing steel density for solid hardware')
        print(name,len(changed))


def details_status():
    import adsk.drawing as dr
    app=core.Application.get()
    project=next(p for p in app.data.dataProjects if p.name=='Default Project')
    folder=project.rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    print('Files',[(f.name,f.id,f.versionNumber) for f in folder.dataFiles if f.fileExtension=='f2d'])
    for doc in app.documents:
        draw=dr.DrawingDocument.cast(doc)
        if draw:
            ss=draw.drawing.sheets
            print(doc.name,doc.dataFile.id if doc.dataFile else '',ss.count,[sh.name for sh in ss] if ss.count<15 else '')


def numbered_pack():
    import tempfile
    app=core.Application.get()
    folder=next(p for p in app.data.dataProjects if p.name=='Default Project').rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    src=next(f for f in folder.dataFiles if f.name=='E_Writing_V3_Parts_2' and f.fileExtension=='f3d')
    old=app.documents.open(src)
    old.activate()
    with tempfile.TemporaryDirectory(prefix='RPRR_drawing_') as staging:
        path=str(Path(staging)/'Details.f3d')
        mgr=L.design().exportManager
        assert mgr.execute(mgr.createFusionArchiveExportOptions(path))
        doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(path))
    d=L.design()
    set_nominal_hardware(d)
    mapping={}
    children=sorted(d.rootComponent.occurrences,key=lambda o:o.component.name)
    assert len(children)==7
    for i,o in enumerate(children,1):
        c=o.component
        name=c.name
        code='RPRR_Detail_'+str(i).zfill(2)
        c.name=code
        c.partNumber=code
        c.description=name
        for b in c.bRepBodies: b.name=code
        mapping[code]=name
    assert doc.saveAs('E_Writing_V3_Supplement',folder,'Detail sheet source with drawing numbers','')
    (HERE/'detail_numbers.json').write_text(json.dumps(mapping,indent=2),encoding='utf8')
    print(mapping)


def open_master():
    app=core.Application.get()
    folder=next(p for p in app.data.dataProjects if p.name=='Default Project').rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    name='E_Writing_V3_\u542b\u6876\u603b\u88c5'   # Chinese-language Fusion names: assembly with bucket
    file=next((f for f in folder.dataFiles if f.name==name and f.fileExtension=='f3d'),None)
    if file:
        doc=app.documents.open(file)
    else:
        doc=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(HERE/'E_Writing.f3d')))
        doc.activate()
        d=L.design()
        drum=next(o for o in d.rootComponent.occurrences if o.component.name.startswith('Wave_Drum'))
        drum.isLightBulbOn=True
        assert d.rootComponent.attributes.itemByName('RPRR','WritingTrackV3')
        assert d.timeline.count==1010 and not L.timeline_problems()
        assert doc.saveAs(name,folder,'Editable third-edition master with drum, joints and writing track','')
    doc.activate()
    file=doc.dataFile
    (HERE/'master_cloud.json').write_text(json.dumps(dict(name=name,folder=folder.name,url=file.fusionWebURL,version=file.versionNumber),ensure_ascii=False,indent=2),encoding='utf8')
    print('Cloud master',name,file.versionNumber)


def master_status():
    app=core.Application.get()
    folder=next(p for p in app.data.dataProjects if p.name=='Default Project').rootFolder.dataFolders.itemByName('RPRR - Third Edition')
    file=next(f for f in folder.dataFiles if f.name=='E_Writing_V3_\u542b\u6876\u603b\u88c5' and f.fileExtension=='f3d')   # Chinese-language Fusion names: assembly with bucket
    assert file.isComplete
    doc=next(d for d in app.documents if d.dataFile and d.dataFile.id==file.id)
    doc.activate()
    design=fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    assert design.rootComponent.attributes.itemByName('RPRR','WritingTrackV3') and design.timeline.count==1010
    assert next(o for o in design.rootComponent.occurrences if o.component.name.startswith('Wave_Drum')).isLightBulbOn
    (HERE/'master_cloud.json').write_text(json.dumps(dict(name=file.name,folder=folder.name,id=file.id,url=file.fusionWebURL,version=file.versionNumber),ensure_ascii=False,indent=2),encoding='utf8')
    print('Verified uploaded master',file.name,file.versionNumber)
