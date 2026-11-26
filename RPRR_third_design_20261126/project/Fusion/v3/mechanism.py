"""Parametric sketches and local placements for the third-edition transmission."""
from pathlib import Path
import json
import math
import runpy
import adsk.core as core
import adsk.fusion as fusion
import cadlib as L

HERE=Path(__file__).resolve().parent
OP=fusion.FeatureOperations
NEW,JOIN,CUT=OP.NewBodyFeatureOperation,OP.JoinFeatureOperation,OP.CutFeatureOperation
AL='RPRR 6061-T6'
STEEL='RPRR Bearing steel'
SHAFT='RPRR 7075-T6'
BELT='RPRR 3MGT belt'
identity_axes=((1,0,0),(0,1,0))
reverse_axes=((-1,0,0),(0,1,0))

def matrix(pos,axes=identity_axes):
    a,b=(core.Vector3D.create(*v) for v in axes)
    mat=core.Matrix3D.create()
    mat.setWithCoordinateSystem(core.Point3D.create(*(v/10 for v in pos)),a,b,a.crossProduct(b))
    return mat

def position(o,mat):
    err=lambda:max(abs(a-b) for a,b in zip(o.transform2.asArray(),mat.asArray()))
    if err()>1e-7:
        o.timelineObject.rollTo(False)
        try:
            inverse=o.transform2.copy()
            inverse.invert()
            corrected=mat.copy()
            corrected.transformBy(inverse)
            corrected.transformBy(o.initialTransform)
            o.initialTransform=corrected
        finally:
            L.design().timeline.moveToEnd()
    assert err()<1e-6,(o.name,o.transform2.asArray(),mat.asArray())
    o.isGroundToParent=True
    return o

def place(group,comp,pos,axes=identity_axes):
    mat=matrix(pos,axes)
    children=L.group(group).component.occurrences
    o=children.addExistingComponent(comp,mat)
    if max(abs(a-b) for a,b in zip(o.transform2.asArray(),mat.asArray()))>1e-7:
        inverse=o.transform2.copy()
        inverse.invert()
        corrected=mat.copy()
        corrected.transformBy(inverse)
        corrected.transformBy(mat)
        o.deleteMe()
        o=children.addExistingComponent(comp,corrected)
    return position(o,mat)

def part(group,name,pos,material=AL,axes=identity_axes):
    mat=matrix(pos,axes)
    o=L.group(group).component.occurrences.addNewComponent(mat)
    c=o.component
    c.name=name
    c.material=L.material(material)
    position(o,mat)
    return c

def P(x,y):
    return core.Point3D.create(x/10,y/10,0)

def sketch(c,label):
    return L.sketch_xy(c,label+'_profile')

def solid(c,sk,z,depth,label,op=None,all_profiles=False):
    if op is None:
        op=JOIN if c.bRepBodies.count else NEW
    if all_profiles:
        ps=core.ObjectCollection.create()
        for p in sk.profiles:
            ps.add(p)
    else:
        ps=max(sk.profiles,key=lambda p:p.areaProperties().area)
    f=L.extrude(c,ps,depth,z,op,label)
    sk.isVisible=False
    return f

def disk(c,label,z,depth,r,bore=0,xy=(0,0),op=None):
    sk=sketch(c,label)
    L.circle(sk,*xy,r)
    if bore:
        L.circle(sk,*xy,bore)
    if bore:
        ps=next(p for p in sk.profiles if p.profileLoops.count==2)
        f=L.extrude(c,ps,depth,z,op if op is not None else (JOIN if c.bRepBodies.count else NEW),label)
        sk.isVisible=False
        return f
    return solid(c,sk,z,depth,label,op)

def rect(c,label,z,depth,extents,op=None):
    sk=sketch(c,label)
    L.rect(sk,*extents)
    return solid(c,sk,z,depth,label,op)

def holes(c,label,z,depth,centres,r):
    sk=sketch(c,label)
    for x,y in centres:
        L.circle(sk,x,y,r)
    return solid(c,sk,z,depth,label,CUT,True)

def slot(sk,x,y,travel,r):
    a,b=x-travel/2,x+travel/2
    ar=sk.sketchCurves.sketchArcs
    lines=sk.sketchCurves.sketchLines
    ar.addByThreePoints(P(a,y+r),P(a-r,y),P(a,y-r))
    ar.addByThreePoints(P(b,y-r),P(b+r,y),P(b,y+r))
    lines.addByTwoPoints(P(a,y+r),P(b,y+r))
    lines.addByTwoPoints(P(a,y-r),P(b,y-r))

def wheel(c,teeth,z,width,bore,label):
    pitch_radius=teeth*3/(2*math.pi)
    radius=pitch_radius-.38
    disk(c,label+'_blank',z,width,radius,bore)
    centres=[((radius+.1)*math.cos(i*math.tau/teeth),(radius+.1)*math.sin(i*math.tau/teeth)) for i in range(teeth)]
    holes(c,label+'_tooth_gaps',z,width,centres,1.31)
    disk(c,label+'_flange_A',z-1,1,radius+1.5,bore)
    disk(c,label+'_flange_B',z+width,1,radius+1.5,bore)

def belt(c,label,distance,teeth1,teeth2,z,width):
    a=teeth1*3/math.tau
    b=teeth2*3/math.tau
    nx=(a-b)/distance
    ny=math.sqrt(1-nx*nx)
    sk=sketch(c,label)
    for delta in (-.38,1.10):
        r,s=a+delta,b+delta
        ltop,lbot=P(nx*r,ny*r),P(nx*r,-ny*r)
        rtop,rbot=P(distance+nx*s,ny*s),P(distance+nx*s,-ny*s)
        sk.sketchCurves.sketchLines.addByTwoPoints(ltop,rtop)
        sk.sketchCurves.sketchLines.addByTwoPoints(lbot,rbot)
        sk.sketchCurves.sketchArcs.addByThreePoints(ltop,P(-r,0),lbot)
        sk.sketchCurves.sketchArcs.addByThreePoints(rbot,P(distance+s,0),rtop)
    p=next(p for p in sk.profiles if p.profileLoops.count==2)
    f=L.extrude(c,p,width,z,NEW,label)
    sk.isVisible=False
    return f

def fittings():
    origin=(0,0,0)
    stub=part('D_03','D_Q4_Transfer_stub',origin,SHAFT)
    rect(stub,'Fixed_flange',30,4,(-21,-13,21,20))
    holes(stub,'Flange_M3_clearance',30,4,[(-13,15),(13,15)],1.7)
    holes(L.comp('D_Q3_Idle_cheek'),'Transfer_flange_M3_taps',2,7,[(-13,15),(13,15)],1.25)
    disk(stub,'Inner_race_shoulder',34,4,4.3)
    disk(stub,'Bearing_journal',38,17,3)
    disk(stub,'Retaining_thread_envelope',55,2,2.5)
    wheelset=part('D_03','D_Q4_Transfer_pulley_60T20T',origin)
    wheel(wheelset,20,36,9,4.8,'Long_belt_20T')
    disk(wheelset,'Connecting_hub',46,1,7.5,4.8)
    wheel(wheelset,60,48,6,4.8,'First_stage_60T')
    disk(wheelset,'686_counterbore',35,8,6.5,op=CUT)
    disk(wheelset,'626_counterbore',48,7,9.5,op=CUT)
    race=part('D_03','D_Bearing_686',(0,0,38),STEEL)
    disk(race,'Outer_race',0,5,6.5,5.2)
    disk(race,'Inner_race',0,5,4.1,3,op=NEW)
    place('D_03',L.comp('D_Bearing_626'),(0,0,48))
    collar=part('D_03','D_Q4_Inner_race_spacer',(0,0,43),SHAFT)
    disk(collar,'Hollow_spacer',0,5,4,3)
    retainer=part('D_03','D_Q4_Transfer_retainer',(0,0,54),STEEL)
    disk(retainer,'Locknut',0,3,4.2,2.5)

def drive():
    x,y=-22.5,45*math.sqrt(3)/2
    pos=(x,y,0)
    br=part('D_03','D_Q4_Motor_bracket',pos)
    rect(br,'Bearing_plate',64,6,(-29,-33,29,31))
    disk(br,'Bearing_boss',56,8,12)
    disk(br,'Front_626_seat',56,6,9.5,op=CUT)
    disk(br,'Back_626_seat',64,6,9.5,op=CUT)
    disk(br,'Shaft_clearance',62,2,4.6,op=CUT)
    for i,(a,b) in enumerate([(a,b) for a in (-23,23) for b in (-24,24)]):
        disk(br,'Post_landing_'+str(i+1),70,6.5,4.5,xy=(a,b))
    holes(br,'Tapped_post_mounts',64,12.5,[(a,b) for a in (-23,23) for b in (-24,24)],1.65)
    # A horizontal foot uses its own sketch, with thickness along group Y.
    foot=part('D_03','D_Q4_Motor_bracket_foot',(0,6,0),AL,((1,0,0),(0,0,-1)))
    rect(foot,'Slotted_foot',0,4,(-51,-108,5,-70))
    sk=sketch(foot,'Foot_adjustment')
    for a,b in [(-42,-97),(-9,-97)]:
        slot(sk,a,b,6,2.25)
    solid(foot,sk,0,4,'Foot_adjustment',CUT,True)
    holes(L.comp('D_Q2_Saddle'),'Q4_foot_M4_taps',31,6,[(-42,-97),(-9,-97)],1.65)
    # Bolt the foot to the vertical plate through their adjoining faces at z=70.
    for c,label,z,depth,r in [(br,'Foot_M3_taps',64,6,1.25)]:
        holes(c,label,z,depth,[(-19,8-y),(19,8-y)],r)
    w=part('D_03','D_GT3M_20T_W6_D6',pos)
    wheel(w,20,48,6,3,'Motor_pulley')
    sh=part('D_03','D_Q4_Input_shaft_v3',pos,SHAFT)
    disk(sh,'Input_shaft',47,36.2,3)
    for z in (56,64):
        place('D_03',L.comp('D_Bearing_626'),(x,y,z))
    place('D_03',L.comp('D_HG_KR053'),(x,y,109.5),reverse_axes)
    place('D_03',L.comp('D_Motor_flange'),(x,y,109.5),reverse_axes)
    place('D_03',L.comp('D_XGT2_19C_8x6'),(x,y,96.5),reverse_axes)
    posts=part('D_03','D_Spacer_28',(x-23,y-24,76.5))
    disk(posts,'Post',0,28,4,2.2)
    for a,b in [(-23,24),(23,-24),(23,24)]:
        place('D_03',posts,(x+a,y+b,76.5))
    u,v=-x/45,-y/45
    belt1=part('D_03','D_Belt_Q4_stage1_W6',pos,BELT,((u,v,0),(-v,u,0)))
    belt(belt1,'3MGT_73T_backing',45,20,60,48,6)

def long_drive():
    group=L.group('D_04').component
    keep={'D_Q3_Link_40x40x3','D_GT3M_120T_W15_D15','D_Q3_Shaft_15','D_Bearing_608','D_Q4_Fork_bush'}
    for o in list(group.occurrences):
        if o.component.name not in keep:
            o.deleteMe()
    tube=L.comp('D_Q3_Link_40x40x3')
    for n in ['Raised_drive_foot_clearances','Motor_backplate_M3_wall_tap_drills']:
        tube.features.extrudeFeatures.itemByName(n).deleteMe()
    wrist=next(o for o in L.group('D_05').component.occurrences if o.component.name=='D_GT3M_40T_W9_D8')
    L.shift(wrist,dz=-2)
    band=part('D_04','D_Belt_Q4_long_690',(0,0,0),BELT)
    belt(band,'3MGT_230T_backing_schematic',300,20,40,36,9)
