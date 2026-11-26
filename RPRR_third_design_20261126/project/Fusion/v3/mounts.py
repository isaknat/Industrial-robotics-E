"""Machining features specified in the parent assembly's millimetre coordinates."""
import math
import adsk.core as core
import adsk.fusion as fusion
import mechanism as M
import cadlib as L

def instance(group,name):
    return next(o for o in L.group(group).component.occurrences if o.component.name==name)

def cut(group,name,label,axis,start,end,shapes):
    occ=instance(group,name)
    c=occ.component
    inverse=occ.transform2.copy()
    inverse.invert()
    direction=core.Vector3D.create(*(int(i==axis) for i in range(3)))
    direction.transformBy(inverse)
    planes=(c.yZConstructionPlane,c.xZConstructionPlane,c.xYConstructionPlane)
    plane=planes[max(range(3),key=lambda i:abs(direction.asArray()[i]))]
    sk=c.sketches.add(plane)
    sk.name=label+'_profile'
    def point(u,v,d):
        p=[u,v]
        p.insert(axis,d)
        pt=core.Point3D.create(*(t/10 for t in p))
        pt.transformBy(inverse)
        return sk.modelToSketchSpace(pt)
    for shape,*args in shapes:
        if shape=='circle':
            u,v,r=args
            p=point(u,v,0)
            sk.sketchCurves.sketchCircles.addByCenterRadius(core.Point3D.create(p.x,p.y,0),r/10)
        else:
            points=[]
            for u,v in args:
                p=point(u,v,0)
                points.append(core.Point3D.create(p.x,p.y,0))
            for a,b in zip(points,points[1:]+points[:1]):
                sk.sketchCurves.sketchLines.addByTwoPoints(a,b)
    a,b=point(0,0,start).z*10,point(0,0,end).z*10
    assert abs(abs(b-a)-abs(end-start))<.001
    return M.solid(c,sk,a,b-a,label,M.CUT,True)

def slot_y(sk,x,y,travel,r):
    a,b=y-travel/2,y+travel/2
    arc=sk.sketchCurves.sketchArcs
    line=sk.sketchCurves.sketchLines
    arc.addByThreePoints(M.P(x-r,a),M.P(x,a-r),M.P(x+r,a))
    arc.addByThreePoints(M.P(x+r,b),M.P(x,b+r),M.P(x-r,b))
    line.addByTwoPoints(M.P(x+r,a),M.P(x+r,b))
    line.addByTwoPoints(M.P(x-r,a),M.P(x-r,b))

def long_idler():
    x=100
    k=-10*3/math.pi/300
    y=(20*3/math.tau+1.1+8-k*x)/-math.sqrt(1-k*k)
    g='D_04'
    plate=M.part(g,'D_Q4_Long_idler_bracket',(x,0,20))
    M.rect(plate,'Adjuster_plate',0,5,(-13,y-8,13,0))
    M.holes(plate,'Tube_mounts',0,5,[(-8,-7),(8,-7)],1.7)
    sk=M.sketch(plate,'Tension_slot')
    slot_y(sk,0,y,6,2.6)
    M.solid(plate,sk,0,5,'Tension_slot',M.CUT)
    cut(g,'D_Q3_Link_40x40x3','Idler_M3_mounts',2,17,20,[('circle',x+i,-7,1.25) for i in (-8,8)])
    pin=M.part(g,'D_Q4_Long_idler_pin',(x,y,17),M.SHAFT)
    M.disk(pin,'Pin_head',0,3,4.5)
    M.disk(pin,'Clamp_stem',3,5,2.5)
    M.disk(pin,'Idler_pedestal',8,11,4.5)
    M.disk(pin,'Journal',19,9,3)
    M.disk(pin,'Retainer',28,1,4)
    idler=M.part(g,'D_Q4_Long_idler',(x,y,36),M.STEEL)
    M.disk(idler,'Roller',0,9,8,3)
    print('Long belt roller centre',x,y)

def shoulder():
    g='D_03'
    cheek=L.comp('D_Q3_Drive_cheek')
    body=cheek.bRepBodies.item(0)
    test=core.Point3D.create(-7,4.8,.45)
    assert body.pointContainment(test)==fusion.PointContainment.PointInsidePointContainment,'Third post has no landing'
    for name in ('D_GT3M_24T_W15_D8','D_Belt_Q3_stage2','D_Belt_idler_15','D_Idler_axle_6','D_Q3_Idler_bracket'):
        L.shift(instance(g,name),dz=7)
    L.shift(instance('D_04','D_GT3M_120T_W15_D15'),dz=7)
    bridge=L.comp('D_Q3_Outboard_bridge')
    M.rect(bridge,'Third_standoff_and_idler_landing',0,7,(-99,37,-62,69))
    for c,n in [(bridge,'Third_standoff_clearance'),(cheek,'Third_standoff_mount')]:
        M.holes(c,n,0,7 if c==bridge else 9,[(-70,48)],2.2)
    M.place(g,L.comp('D_Spacer_53'),(-70,48,-83))
    k=(16-80)*3/math.tau/61.4
    x=-90
    y=78.29+(16*3/math.tau+1.1+8-k*(x+106.6))/-math.sqrt(1-k*k)
    sk=M.sketch(bridge,'First_stage_tension_slot')
    slot_y(sk,x,y,6,2.6)
    M.solid(bridge,sk,0,7,'First_stage_tension_slot',M.CUT)
    roller=M.part(g,'D_Q3_First_stage_idler',(x,y,-102),M.STEEL)
    M.disk(roller,'Roller',0,9,8,3)
    axle=M.part(g,'D_Q3_First_stage_idler_axle',(x,y,-104),M.SHAFT)
    M.disk(axle,'Captive_head',0,2,4.3)
    M.disk(axle,'Idler_journal',2,9,3)
    M.disk(axle,'Shoulder',11,3,4.3)
    M.disk(axle,'Sliding_shank',14,7,2.5)
    M.disk(axle,'Locknut_envelope',21,4,4)
    bracket=L.comp('D_Q3_Idler_bracket')
    sk=M.sketch(bracket,'Angular_tension_slot')
    a,b=-45.2,78.29
    r,half,end=16,2.2,math.radians(13)
    def pt(radius,theta):
        return M.P(a+radius*math.cos(theta),b+radius*math.sin(theta))
    arcs=sk.sketchCurves.sketchArcs
    for rad in (r+half,r-half):
        arcs.addByThreePoints(pt(rad,-end),pt(rad,0),pt(rad,end))
    for angle,s in [(-end,-1),(end,1)]:
        middle=M.P(a+r*math.cos(angle)-s*half*math.sin(angle),b+r*math.sin(angle)+s*half*math.cos(angle))
        arcs.addByThreePoints(pt(r+half,angle),middle,pt(r-half,angle))
    M.solid(bracket,sk,0,7,'Angular_tension_slot',M.CUT)
    x,y=a+r,b
    M.holes(cheek,'Second_stage_lock_M4_tap',0,9,[(x,y)],1.65)
    spacer=M.part(g,'D_Q3_Tension_lock_spacer',(x,y,-41))
    M.disk(spacer,'Lock_spacer',0,11,3.7,2.2)
    screw=M.part(g,'D_Q3_Tension_lock_M4',(x,y,-51.5),M.STEEL)
    M.disk(screw,'Screw_head',0,3.5,3.5)
    M.disk(screw,'Clearance_shank',3.5,18,2)
    M.disk(screw,'Thread_minor_diameter',21.5,7,1.65)

def clamp():
    g='D_04'
    name='D_Q3_Arm_clamp_hub'
    hub=M.part(g,name,(0,0,0))
    M.rect(hub,'Clamp_block',-10,20,(-17,-15,17,15))
    for s in [-1,1]:
        M.rect(hub,'Tube_landing_'+str(s),-10,20,(-17,min(s*15,s*17),17,max(s*15,s*17)))
    M.disk(hub,'Shoulder_bore',-10,20,7.5,op=M.CUT)
    M.rect(hub,'Split',-10,20,(7.4,-.6,18,.6),M.CUT)
    cut(g,name,'Clamp_screw_clearance',1,.6,17,[('circle',12,0,2.2)])
    cut(g,name,'Clamp_screw_tap',1,-.6,-13,[('circle',12,0,1.65)])
    cut(g,name,'Clamp_screw_head_recess',1,10,17,[('circle',12,0,3.5)])
    shapes=[('circle',-11,-5,2.2),('circle',3,5,2.2)]
    for side in (-1,1):
        cut(g,'D_Q3_Link_40x40x3','Hub_mounts_'+str(side),1,side*17,side*20,shapes)
        cut(g,name,'Tube_taps_'+str(side),1,side*17,side*10,[('circle',a,b,1.65) for _,a,b,_ in shapes])
    wheel=L.comp('D_GT3M_120T_W15_D15')
    M.disk(wheel,'Set_screw_hub',-11,10,13,7.5)
    for axis in (0,1):
        cut(g,wheel.name,'Set_screw_tap_'+str(axis),axis,6.7,13,[('circle',0,-77,1.65)])
        cut(g,'D_Q3_Shaft_15','Set_screw_flat_'+str(axis),axis,6.8,8,[('poly',(-4,-80),(4,-80),(4,-74),(-4,-74))])
        axes=((0,1,0),(0,0,1)) if axis==0 else ((0,0,1),(1,0,0))
        p=[0,0,-77]
        p[axis]=6.8
        screw=M.part(g,'D_Q3_Hub_set_screw_'+str(axis),p,M.STEEL,axes)
        M.disk(screw,'Thread_minor_diameter',0,6,1.65)

def clearances():
    tray=L.comp('D_Q1_Round_base')
    M.disk(tray,'Belt_rim_clearance',22,13,160,157,op=M.CUT)
    group=L.group('D_01').component
    bears=[o for o in group.occurrences if o.component.name=='D_Bearing_626' and abs(o.transform2.translation.y*10-47.9)<.01]
    assert len(bears)==2,[(o.name,o.transform2.translation.asArray()) for o in bears]
    bears[-1].deleteMe()
    ang=math.atan2(-80.33126861277884,-40.5)
    co,si=math.cos(ang),math.sin(ang)
    for name in ('D_GT3M_18T_W9_D6','D_GT3M_120T_W9_D8'):
        o=instance('D_01',name)
        t=o.transform2
        orig,xa,ya,za=t.getAsCoordinateSystem()
        def spin(v):
            x,y,z=v.asArray()
            return (co*x-si*z,y,si*x+co*z)
        M.position(o,M.matrix([10*v for v in orig.asArray()],(spin(xa),spin(ya))))
    ret=L.comp('D_Q4_Transfer_retainer')
    M.disk(ret,'Journal_counterbore',0,1,3,op=M.CUT)
    foot=L.comp('D_Q4_Motor_bracket_foot')
    sk=foot.sketches.itemByName('Slotted_foot_profile')
    assert L.move_points(sk,lambda x,y:abs(y+108)<.01,dy=4)==2
    plate=L.comp('D_Q4_Motor_bracket')
    sk=plate.sketches.itemByName('Bearing_plate_profile')
    assert L.move_points(sk,lambda x,y:abs(y+33)<.01,dy=.12)==2
    cut('D_03',foot.name,'Plate_M3_clearances',2,70,79,[('circle',-22.5+i,8,1.7) for i in (-19,19)])
    tube=L.comp('D_Q3_Link_40x40x3')
    sk=M.sketch(tube,'Long_idler_head_travel')
    slot_y(sk,100,-21.84346428,6,4.6)
    M.solid(tube,sk,16.9,3.1,'Long_idler_head_travel',M.CUT)

def fasteners():
    g='D_04'
    along_y=((0,0,1),(1,0,0))
    neg_y=((1,0,0),(0,0,1))
    bolt=M.part(g,'D_Q3_Split_clamp_M4',(12,-9,0),M.STEEL,along_y)
    M.disk(bolt,'Thread_core',0,9,1.65)
    M.disk(bolt,'Grip',9,10,2)
    M.disk(bolt,'Socket_head',19,4,3.4)
    M.disk(bolt,'Socket_recess',21,2,1.5,op=M.CUT)
    # Countersunk below the tube exterior to preserve the swept envelope.
    for side in (-1,1):
        axes=along_y if side==1 else neg_y
        for j,(x,z) in enumerate([(-11,-5),(3,5)]):
            if side==-1 and j==0:
                screw=M.part(g,'D_Q3_Tube_mount_M4',(x,side*11,z),M.STEEL,axes)
                M.disk(screw,'Tap_core',0,6,1.65)
                M.disk(screw,'Grip',6,1,2)
                M.disk(screw,'Low_head',7,2,3.4)
                M.disk(screw,'Socket_recess',8,1,1.4,op=M.CUT)
            else:
                M.place(g,screw,(x,side*11,z),axes)
        cut(g,'D_Q3_Link_40x40x3','Recess_tube_heads_'+str(side),1,side*18,side*20,[('circle',-11,-5,3.5),('circle',3,5,3.5)])
    # Local clearance for the stationary lower stop and its mounting ear.
    cut('D_03','D_Q2_Saddle','Shoulder_stop_pocket',1,-24,-16,[('poly',(6,-39),(17,-39),(17,-25),(6,-25))])

def mount_hardware():
    g='D_03'
    x,y=-22.5,45*math.sqrt(3)/2
    br=L.comp('D_Q4_Motor_bracket')
    # Make the mounting foot and the bearing plate a one-piece machined angle.
    # The two earlier screw pilots disappear with the temporary separate foot.
    instance(g,'D_Q4_Motor_bracket_foot').deleteMe()
    br.features.extrudeFeatures.itemByName('Foot_M3_taps').deleteMe()
    M.rect(br,'Integral_sliding_foot',70,34,(-51-x,6-y,5-x,10-y))
    sk=br.sketches.add(br.xZConstructionPlane)
    sk.name='Foot_adjustment_slots_profile'
    p=lambda xx,yy,zz: sk.modelToSketchSpace(core.Point3D.create((xx-x)/10,(yy-y)/10,zz/10))
    for xx in (-42,-9):
        pt=p(xx,6,97)
        M.slot(sk,pt.x*10,pt.y*10,6,2.25)
    a,b=p(0,6,97).z*10,p(0,10,97).z*10
    M.solid(br,sk,a,b-a,'Foot_adjustment_slots',M.CUT,True)
    faxes=((1,0,0),(0,0,-1))
    bolt=M.part(g,'D_Q4_Foot_M4',(-42,1,97),M.STEEL,faxes)
    M.disk(bolt,'Tap_core',0,5,1.65)
    M.disk(bolt,'Grip',5,4,2)
    M.disk(bolt,'Socket_head',9,3.5,3.5)
    M.disk(bolt,'Socket_recess',10.5,2,1.4,op=M.CUT)
    M.place(g,bolt,(-9,1,97),faxes)
    for i,(a,b) in enumerate([(a,b) for a in (-23,23) for b in (-24,24)]):
        pos=(x+a,y+b,70)
        if i:
            M.place(g,post_bolt,pos)
        else:
            post_bolt=M.part(g,'D_Q4_Standoff_M4',pos,M.STEEL)
            M.disk(post_bolt,'Tap_core',0,6.5,1.65)
            M.disk(post_bolt,'Grip',6.5,33,2)
            M.disk(post_bolt,'Socket_head',39.5,3.5,3.5)
            M.disk(post_bolt,'Socket_recess',41,2,1.4,op=M.CUT)
    for i,xx in enumerate((-13,13)):
        pos=(xx,15,25)
        if i:
            M.place(g,ret,pos)
        else:
            ret=M.part(g,'D_Q4_Stub_M3',pos,M.STEEL)
            M.disk(ret,'Tap_core',0,5,1.25)
            M.disk(ret,'Grip',5,4,1.5)
            M.disk(ret,'Low_head',9,2.5,2.7)
            M.disk(ret,'Socket_recess',10,1.5,1.1,op=M.CUT)

def recess_stub_fasteners():
    # Sink 1.5 mm of each head into the fixed flange. The top of the head
    # ends at z=35 mm for every joint angle; the long belt begins at z=36.
    c=L.comp('D_Q4_Stub_M3')
    ef=c.features.extrudeFeatures
    L.set_distance(ef.itemByName('Grip'),2.5)
    L.set_start_offset(ef.itemByName('Low_head'),7.5)
    L.set_start_offset(ef.itemByName('Socket_recess'),8.5)
    M.holes(L.comp('D_Q4_Transfer_stub'),'Flush_M3_counterbores',32.5,1.5,[(-13,15),(13,15)],2.8)
    for o in L.group('D_03').component.occurrences:
        if o.component.name==c.name:
            bb=o.bRepBodies.item(0).boundingBox
            assert bb.maxPoint.z*10<=35.001,(o.name,bb.maxPoint.z*10)
