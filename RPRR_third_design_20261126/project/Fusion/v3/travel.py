"""Pen overtravel, spring geometry and end stops in assembly frames."""
import math
import adsk.core as core
import adsk.fusion as fusion
import cadlib as L
import mechanism as M
from mounts import cut, instance

def pen():
    g='D_05'
    carrier='D_Q4_Pen_carrier'
    cut(g,carrier,'Spring_seat',0,5,12.9,[('circle',0,0,3.75)])
    cut(g,carrier,'Sliding_barrel_bore',0,12.9,26.1,[('circle',0,0,6.08)])
    holder=L.comp(carrier)
    sk=M.sketch(holder,'Travel_pin_slots')
    M.slot(sk,20,0,4,1.02)
    M.solid(holder,sk,-8,16,'Travel_pin_slots',M.CUT)
    cut(g,'D_Q4_Pen','Retaining_pin_bore',2,-6,6,[('circle',21,0,1)])
    pin=M.part(g,'D_Q4_Pen_travel_pin',(21,0,-7.7),M.STEEL)
    M.disk(pin,'Travel_pin',0,15.4,1)
    spring=M.part(g,'D_Q4_Pen_return_spring',(5,0,0),M.STEEL,((0,0,-1),(0,1,0)))
    sk=M.sketch(spring,'Installed_spring_helix')
    pts=core.ObjectCollection.create()
    for i in range(97):
        a=i*math.tau/16
        pts.add(core.Point3D.create(.3*math.cos(a),.3*math.sin(a),(.325+10.35*i/96)/10))
    curve=sk.sketchCurves.sketchFittedSplines.add(pts)
    path=spring.features.createPath(curve,False)
    pipes=spring.features.pipeFeatures
    spec=pipes.createInput(path,M.NEW)
    spec.sectionSize=core.ValueInput.createByString('0.65 mm')
    pipes.add(spec).name='Six_active_coils'
    sk.isVisible=False
    print('Spring rate N/mm',79000*.65**4/(8*6**3*6))

def pen_joint():
    group=L.group('D_05').component
    body=instance('D_05','D_Q4_Pen')
    pin=instance('D_05','D_Q4_Pen_travel_pin')
    carrier=instance('D_05','D_Q4_Pen_carrier')
    body.isGroundToParent=False
    pin.isGroundToParent=False
    joints=group.asBuiltJoints
    spec=joints.createInput(pin,body,None)
    spec.setAsRigidJointMotion()
    joints.add(spec).name='Pen_pin_press_fit'
    origin=carrier.component.originConstructionPoint.createForAssemblyContext(carrier)
    spec=joints.createInput(body,carrier,fusion.JointGeometry.createByPoint(origin))
    spec.setAsSliderJointMotion(fusion.JointDirections.XAxisJointDirection)
    joint=joints.add(spec)
    joint.name='Pen_compliance'
    proxy=joint.createForAssemblyContext(L.group('D_05'))
    motion=proxy.jointMotion
    limits=motion.slideLimits
    limits.minimumValue=-.3
    limits.maximumValue=.1
    limits.isMinimumValueEnabled=True
    limits.isMaximumValueEnabled=True
    values=[]
    for displ in (-.3,.1,0):
        motion.slideValue=displ
        import adsk
        adsk.doEvents()
        tip=core.Point3D.create(0,0,3.4)
        tip.transformBy(body.createForAssemblyContext(L.group('D_05')).transform2)
        values.append((displ,motion.slideValue,tip.asArray()))
        assert abs(motion.slideValue-displ)<1e-6,values
    print('Slider command, readback, absolute pen-tip positions',values)

def stops():
    floor=((1,0,0),(0,0,-1))
    g='D_02'
    for side,edge in [(-1,-48.7),(1,189.7)]:
        label='Rear' if side<0 else 'Front'
        x=edge+side*4
        block=M.part(g,'D_Q2_'+label+'_stop',(x,181,-44.5),axes=floor)
        M.rect(block,'Stop_block',0,7,(-2.5,-3,2.5,3))
        M.holes(block,'Stop_mount',0,7,[(0,0)],1.7)
        cut(g,'D_Q2_Rail_plate',label+'_stop_M3_tap',1,174,181,[('circle',x,-44.5,1.25)])
        pad=M.part(g,'D_Q2_'+label+'_buffer',(edge+side*.75,181,-44.5),axes=floor)
        M.rect(pad,'Bonded_buffer',0,7,(-.75,-3,.75,3))
        rub=next((m for lib in core.Application.get().materialLibraries for m in lib.materials if m.name in ('Rubber','Neoprene','\u6a61\u80f6','\u6c2f\u4e01\u6a61\u80f6')),None)   # Chinese-language Fusion names: neoprene; rubber
        if rub: pad.material=rub
    # Rotating flag is integral to the shoulder shaft. Stops screw into ears
    # on the fixed cheek, between the bearing and the second belt plane.
    shaft=L.comp('D_Q3_Shaft_15')
    M.rect(shaft,'Stop_flag',61,4,(-2,-3,23,3))
    cheek=L.comp('D_Q3_Drive_cheek')
    for side,angle in [(-1,-42.6),(1,45.6)]:
        # Contact between the flag's +/-3 mm side and a 3 mm pin at r=22.
        a=math.radians(angle)+side*math.asin(6/22)
        x,y=22*math.cos(a),22*math.sin(a)
        label='Lower' if side<0 else 'Upper'
        M.rect(cheek,label+'_stop_ear',0,4,(x-4,min(side*14,y)-3,x+4,max(side*14,y)+3))
        M.holes(cheek,label+'_stop_thread',0,4,[(x,y)],1.65)
        pin=M.part('D_03','D_Q3_'+label+'_stop',(x,y,-38),M.STEEL)
        M.disk(pin,'Stop_journal',0,8,3)
        M.disk(pin,'Thread_minor_diameter',8,4,1.65)
    M.disk(cheek,'Restore_bearing_seat_at_stop_ears',0,4,16,op=M.CUT)
