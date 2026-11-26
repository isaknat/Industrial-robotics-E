"""Round q1 enclosure; coordinates of sketches are (column x, minus column z)."""
import math
import mechanism as M
import cadlib as L

floor_axes=((1,0,0),(0,0,-1))
def run():
    mx=-59.5
    dist=89.96311864835738
    mz=math.sqrt(dist**2-(mx+100)**2)
    center=(mx,-mz)
    group=L.group('D_01').component
    old=list(group.occurrences)
    for o in old:
        name=o.component.name
        if name in ('D_Q1_Base_tray','D_Q1_Base_lid','D_Idler_axle_6'):
            o.deleteMe()
            continue
        tr=o.transform2
        x,y,z=[v*10 for v in tr.translation.asArray()]
        if name=='D_Belt_Q1_stage1':
            u,v=(-100-mx)/dist,-mz/dist
            M.position(o,M.matrix((mx,y,mz),((u,0,v),(v,0,-u))))
        elif x<-150:
            if name in ('D_HG_KR053','D_M4x12_motor_socket_screw'):
                axes=tuple(tuple(t[k]*(-1 if k!=1 else 1) for k in range(3)) for t in (tr.getAsCoordinateSystem()[1].asArray(),tr.getAsCoordinateSystem()[2].asArray()))
                M.position(o,M.matrix((mx-(x+100+dist),y,mz-z),axes))
            else:
                L.shift(o,dx=mx+100+dist,dz=mz)
    tray=M.part('D_01','D_Q1_Round_base',(0,0,0),axes=floor_axes)
    M.disk(tray,'Floor',0,6,165,22)
    M.disk(tray,'Rim',6,69,165,158)
    M.disk(tray,'Main_bearing_housing',6,15,42,34)
    M.disk(tray,'Intermediate_bearing_housing',6,9,16.5,11,(-100,0))
    M.disk(tray,'Input_bearing_housing',6,9,14.5,9.5,center)
    boltpts=[(161.5*math.cos(i*math.tau/8),161.5*math.sin(i*math.tau/8)) for i in range(8)]
    M.holes(tray,'Cover_M4_taps',64,11,boltpts,1.65)
    M.holes(tray,'Floor_mounts',0,6,[(125,55),(125,-55),(-15,135),(-15,-135)],3.3)
    lid=M.part('D_01','D_Q1_Round_lid',(0,0,0),axes=floor_axes)
    M.disk(lid,'Access_cover',75,8,165,34)
    M.disk(lid,'Main_6008_seat',68,7,41.5,34)
    M.holes(lid,'Intermediate_608_seat',75,8,[(-100,0)],11)
    M.holes(lid,'Cover_M4_clearance',75,8,boltpts,2.2)
    # The upper input bearing is recessed below the cover. The integral housing
    # opens toward the coupling and has an independent shoulder for the 626.
    M.disk(lid,'Upper_bearing_tower',46.4,28.6,18,7.5,center)
    M.disk(lid,'Upper_626_seat',47.9,6,9.5,xy=center,op=M.CUT)
    M.disk(lid,'Coupling_access',53.9,29.1,13,xy=center,op=M.CUT)
    M.rect(lid,'Motor_landing',83,9,(mx-21,center[1]-21,mx+21,center[1]+21))
    M.disk(lid,'Rotating_shaft_clearance',83,9,13,xy=center,op=M.CUT)
    M.disk(lid,'Motor_register',89.25,2.75,15.1,xy=center,op=M.CUT)
    M.place('D_01',L.comp('D_Bearing_626'),(mx,47.9,mz),floor_axes)
    d=23/math.sqrt(2)
    M.holes(lid,'Motor_M4_mounts',80,12,[(mx-d,center[1]+d),(mx+d,center[1]-d)],1.65)
    sk=M.sketch(lid,'Stage2_tension_slot')
    M.slot(sk,-70.69,36.17,6,3.1)
    M.solid(lid,sk,75,8,'Stage2_tension_slot',M.CUT)
    axle=M.part('D_01','D_Q1_Adjustable_idler_axle',(-70.69,46,-36.17),M.SHAFT,floor_axes)
    M.disk(axle,'Idler_journal',0,15,3)
    M.disk(axle,'Shoulder',15,14,4.8)
    M.disk(axle,'M6_thread_envelope',29,13,3)
    nut=M.part('D_01','D_Q1_Idler_locknut',(-70.69,83,-36.17),M.STEEL,floor_axes)
    M.disk(nut,'M6_locknut',0,5,5.3,3)
    # First-stage idler tangent to the outside of the belt; its slot is in
    # the stationary lower pedestal and leaves the motor and shaft coaxial.
    dx,dy=-100-mx,-center[1]
    ux,uy=dx/dist,dy/dist
    rs,rl=18*3/math.tau,120*3/math.tau
    k=(rs-rl)/dist
    h=math.sqrt(1-k*k)
    nx,ny=k*ux-h*uy,k*uy+h*ux
    t=.28
    x=mx+t*dx+nx*(rs*(1-t)+rl*t+1.1+8)
    v=center[1]+t*dy+ny*(rs*(1-t)+rl*t+1.1+8)
    M.rect(tray,'Stage1_slider_landing',6,15,(x-10,v-7,x+10,v+7))
    sk=M.sketch(tray,'Stage1_tension_slot')
    M.slot(sk,x,v,6,2.7)
    M.solid(tray,sk,0,21,'Stage1_tension_slot',M.CUT)
    idler=M.part('D_01','D_Q1_Stage1_idler',(x,24,-v),M.STEEL,floor_axes)
    M.disk(idler,'Smooth_idler',0,9,8,3)
    pin=M.part('D_01','D_Q1_Stage1_adjuster',(x,0,-v),M.SHAFT,floor_axes)
    M.disk(pin,'Clamp_shank',0,21,2.5)
    M.disk(pin,'Idler_shoulder',21,3,4.5)
    M.disk(pin,'Idler_journal',24,9,3)
    M.disk(pin,'Retainer',33,2,4)
    print('Round base, input axis (x,z)',mx,mz,'centre distance',dist)
