"""Third round of fixes from the structure review (docs/02_mechanical_design.md, items A, B, C, D). after CAD edition 3, 2026-11-26.

Run inside Fusion on the working document E_Writing_q3_50W_build, one stage at a time:
  seats()        A  a shoulder behind each small bearing whose plate is thicker than the bearing, a cap where it is not
  fasteners()    B  bolts and nuts through the two sets of standoffs of the q3 drive, screws of the q3 motor
  wrist()        C  pen carrier set-screwed on the wrist shaft; the wrist shaft located by a spacer and a retaining ring
  q3_support()   D  the outboard bridge reaches the q3 axis and carries a third bearing for the q3 shaft
Only new parts are placed (mechanism.place); no existing part is moved, because moving one instance
drags the later instances of the same component (see struct_fix2.q1_idlers_along_slots).
Coordinates in mm, in the frame of the sub-assembly named in each call.
"""
import importlib
import math
import sys
from pathlib import Path
import adsk.core as core
import adsk.fusion as fusion

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cadlib as L
import mechanism as M
import mounts as N
import belt_path as B
for _m in (L, M, N, B):
    importlib.reload(_m)

DOC = 'E_Writing_q3_50W_build'
FLOOR = ((1, 0, 0), (0, 0, -1))
MID, INPUT = B.MID, B.INPUT
IN = fusion.PointContainment.PointInsidePointContainment


def guard():
    app = core.Application.get()
    if app.activeDocument.name != DOC:
        raise RuntimeError('Expected the working document, got ' + app.activeDocument.name)
    return L.design()


def volume(name):
    c = L.comp(name)
    return sum(b.volume for b in c.bRepBodies) * 1000


def ring(comp_name, label, z, depth, r_out, r_in, xy, sign):
    """Join (+1) or cut (-1) a ring in a part and check that the volume changed by the ring."""
    before = volume(comp_name)
    M.disk(L.comp(comp_name), label, z, depth, r_out, r_in, xy=xy, op=M.JOIN if sign > 0 else M.CUT)
    change = volume(comp_name) - before
    ideal = sign * math.pi * (r_out ** 2 - r_in ** 2) * depth
    assert L.comp(comp_name).bRepBodies.count == 1 and abs(change - ideal) < 0.02 * abs(ideal) + 0.5, (comp_name, label, change, ideal)


def plain_cut(c, sk, start, depth, label):
    """Cut without naming participant bodies (cadlib.extrude names them, which fails on some small parts)."""
    ef = c.features.extrudeFeatures
    coll = core.ObjectCollection.create()
    for p in sk.profiles:
        coll.add(p)
    inp = ef.createInput(coll, M.CUT)
    inp.setDistanceExtent(False, core.ValueInput.createByString(f'{depth} mm'))
    if abs(start) > 1e-9:
        inp.startExtent = fusion.OffsetStartDefinition.create(core.ValueInput.createByString(f'{start} mm'))
    f = ef.add(inp)
    f.name = label
    sk.isVisible = False
    return f


def report(d, names):
    d.computeAll()
    for n in names:
        c = L.comp(n)
        print(n, 'bodies', c.bRepBodies.count, 'volume', round(volume(n), 1))
    bad = [(i, d.timeline.item(i).name) for i in range(d.timeline.count)
           if d.timeline.item(i).healthState != fusion.FeatureHealthStates.HealthyFeatureHealthState]
    print('timeline', d.timeline.count, 'not healthy', bad)
    assert not bad, bad


# ---------------------------------------------------------------- A. bearing seats
def seats():
    d = guard()
    # 626 (6 wide) in the 7 mm outboard bridge and in the 9 mm drive cheek: a 1 mm shoulder on the inner side
    ring('D_Q3_Outboard_bridge', 'Input_626_shoulder', 6, 1, 9.5, 8.0, INPUT, +1)
    ring('D_Q3_Drive_cheek', 'Input_626_shoulder', 8, 1, 9.5, 8.0, INPUT, +1)
    ring('D_Q3_Drive_cheek', 'Intermediate_626_shoulder', 8, 1, 9.5, 8.0, MID, +1)
    # q1: the lower bearings of the two small shafts stood over open bores; the upper 626 of the intermediate shaft too
    ring('D_Q1_Round_base', 'Input_626_shoulder', 6, 3, 9.5, 8.0, (-59.5, -80.33126861277884), +1)
    ring('D_Q1_Round_base', 'Intermediate_608_shoulder', 6, 2, 11.0, 9.5, (-100, 0), +1)
    ring('D_Q1_Round_lid', 'Intermediate_626_shoulder', 81, 2, 9.5, 8.0, (-100, 0), +1)
    # q1 main bearing: the floor reached under the turning inner ring (hole 44 mm, inner ring 49 mm): 0.5 mm relief
    ring('D_Q1_Round_base', 'Main_bearing_inner_ring_relief', 5.5, 0.5, 28.0, 22.0, (0, 0), -1)
    # 608 (7 wide) fills the 7 mm bridge: a cap on the outside holds its outer ring
    cap = M.part('D_03', 'D_Bearing_cap_608', (*MID, -91.2))
    M.disk(cap, 'Cap_ring', 0, 1.2, 15.0, 9.6)
    pts = [(12.3 * math.cos(a), 12.3 * math.sin(a)) for a in (math.radians(200), math.radians(340))]
    M.holes(cap, 'M3_clearance', 0, 1.2, pts, 1.7)
    M.holes(L.comp('D_Q3_Outboard_bridge'), 'Bearing_cap_M3_taps', 0, 7, [(MID[0] + x, MID[1] + y) for x, y in pts], 1.25)
    report(d, ['D_Q3_Outboard_bridge', 'D_Q3_Drive_cheek', 'D_Q1_Round_base', 'D_Q1_Round_lid', 'D_Bearing_cap_608'])


# ---------------------------------------------------------------- B. fasteners
def bolt(group, name, pos, axes, head_r, head_h, length, thread_len=8.0):
    """Screw whose own z runs from the top of the head (z = 0) into the thread."""
    c = M.part(group, name, pos, M.STEEL, axes)
    M.disk(c, 'Head', 0, head_h, head_r)
    M.disk(c, 'Shank', head_h, length - thread_len, 2.0)
    M.disk(c, 'Thread_root', head_h + length - thread_len, thread_len, 1.65)
    sk = M.sketch(c, 'Hex_socket')
    L.circle(sk, 0, 0, 1.5)
    plain_cut(c, sk, 0, min(2.2, head_h - 0.6), 'Hex_socket')
    return c


def fasteners():
    d = guard()
    plus_z, minus_z = M.identity_axes, M.reverse_axes
    # (1) four 41 mm standoffs between the drive cheek (z -30..-21) and the q3 motor plate (z 20..25):
    #     M4x60 from the outside of the cheek, nut on the motor side of the plate
    c41 = [(INPUT[0] + a, INPUT[1] + b) for a in (-23, 23) for b in (-24, 24)]
    long_bolt = nut = None
    for x, y in c41:
        if long_bolt is None:
            long_bolt = bolt('D_03', 'D_M4x60_socket_screw', (x, y, -34), plus_z, 3.5, 4.0, 60.0)
            nut = M.part('D_03', 'D_M4_nut', (x, y, 25), M.STEEL)
            M.disk(nut, 'Nut_envelope', 0, 3.2, 3.5, 1.65)
        else:
            M.place('D_03', long_bolt, (x, y, -34), plus_z)
            M.place('D_03', nut, (x, y, 25))
    # (2) three 53 mm standoffs between the outboard bridge (z -90..-83) and the drive cheek: the 90-tooth pulley
    #     runs 3 mm outside the bridge, so a button head (2.2 mm) goes there and the nut inside the cheek
    button = None
    for x, y in ((-75, 92), (-100, 92), (-70, 48)):
        if button is None:
            button = bolt('D_03', 'D_M4x75_button_screw', (x, y, -92.2), plus_z, 3.8, 2.2, 75.0)
        else:
            M.place('D_03', button, (x, y, -92.2), plus_z)
        M.place('D_03', nut, (x, y, -21))
    # (3) the q3 motor on its plate: two M4x12 in the motor's diagonal holes, as on the q1 motor
    plate = L.comp('D_Motor_flange')
    holes = [(16.263, 16.263), (-16.263, -16.263)]
    if plate.bRepBodies.item(0).pointContainment(core.Point3D.create(holes[0][0] / 10, holes[0][1] / 10, 0.25)) == IN:
        M.holes(plate, 'Motor_M4_taps', 0, 5, holes, 1.65)
        print('motor plate: two M4 taps added (it is the same part under the q2, q3 and q4 motors)')
    screw = L.comp('D_M4x12_motor_socket_screw')
    for a, b in holes:                       # the motor is turned so that its x points along -x of the slider
        M.place('D_03', screw, (INPUT[0] - a, INPUT[1] + b, 30), minus_z)
    report(d, ['D_M4x60_socket_screw', 'D_M4x75_button_screw', 'D_M4_nut', 'D_Motor_flange'])


# ---------------------------------------------------------------- C. pen carrier and wrist shaft
def wrist():
    d = guard()
    car = L.comp('D_Q4_Pen_carrier')
    assert car.bRepBodies.item(0).pointContainment(core.Point3D.create(-0.6, 0, 0)) == IN
    sk = car.sketches.add(car.yZConstructionPlane)
    sk.name = 'Shaft_set_screw_tap_profile'
    p = sk.modelToSketchSpace(core.Point3D.create(0, 0, 0))
    sk.sketchCurves.sketchCircles.addByCenterRadius(core.Point3D.create(p.x, p.y, 0), 0.125)
    before = volume('D_Q4_Pen_carrier')
    ef = car.features.extrudeFeatures
    inp = ef.createInput(sk.profiles.item(0), M.CUT)
    inp.setDistanceExtent(False, core.ValueInput.createByString('4 mm'))
    inp.startExtent = fusion.OffsetStartDefinition.create(core.ValueInput.createByString('-8 mm'))
    f = ef.add(inp)
    f.name = 'Shaft_set_screw_tap'
    sk.isVisible = False
    removed = before - volume('D_Q4_Pen_carrier')
    assert 15 < removed < 21, removed                       # a 2.5 mm hole, 4 mm deep, minus the curved end at the bore
    shaft = L.comp('D_Q4_Wrist_shaft')                      # own z = z + 27 in the pen-head frame
    M.rect(shaft, 'Carrier_flat', 24.5, 5.0, (-5.0, -4.0, -3.7, 4.0), M.CUT)
    grub = M.part('D_05', 'D_Set_screw_M3x4', (-7.7, 0, 0), M.STEEL, ((0, 1, 0), (0, 0, 1)))     # own z along +x
    M.disk(grub, 'Thread_minor_diameter', 0, 4.0, 1.25)
    # axial location: a spacer between the pulley hub and the inner ring of the bearing at z 20..27,
    # and a retaining ring outside the bearing at z -27..-20 (the shaft is lengthened 1.5 mm for it)
    spacer = M.part('D_05', 'D_Q4_Wrist_spacer', (0, 0, 27), M.SHAFT)
    M.disk(spacer, 'Spacer_ring', 0, 1.0, 5.5, 4.0)
    M.disk(shaft, 'Ring_end_extension', -1.5, 1.5, 4.0, op=M.JOIN)
    M.disk(shaft, 'Ring_groove_end', -0.8, 0.8, 4.6, 3.8, op=M.CUT)
    M.place('D_05', L.comp('D_Retaining_ring_8'), (0, 0, -27.8))
    assert shaft.bRepBodies.count == 1
    report(d, ['D_Q4_Pen_carrier', 'D_Q4_Wrist_shaft', 'D_Set_screw_M3x4', 'D_Q4_Wrist_spacer'])


# ---------------------------------------------------------------- D. third bearing for the q3 shaft
def q3_support():
    """The 120-tooth pulley of q3 hangs 25 to 52 mm outside the drive-side bearing. The outboard bridge
    (z -90..-83, so far only around the two small shafts) gets an arm to the q3 axis with a 6002 bearing
    in a 9 mm boss (z -92..-83)."""
    d = guard()
    bridge = L.comp('D_Q3_Outboard_bridge')
    before = volume('D_Q3_Outboard_bridge')
    px, py, half = -42.0, 58.0, 15.0
    l = math.hypot(px, py)
    nx, ny = -py / l, px / l                      # unit normal of the line from the q3 axis to (px, py)
    sk = M.sketch(bridge, 'Q3_outboard_arm')
    L.poly(sk, [(px + half * nx, py + half * ny), (px - half * nx, py - half * ny), (-half * nx, -half * ny), (half * nx, half * ny)])
    M.solid(bridge, sk, 0, 7, 'Q3_outboard_arm', M.JOIN, True)
    M.disk(bridge, 'Q3_bearing_boss', -2, 9, 22.0, op=M.JOIN)
    M.disk(bridge, 'Q3_6002_seat', -2, 9, 16.0, op=M.CUT)
    assert bridge.bRepBodies.count == 1 and volume('D_Q3_Outboard_bridge') > before + 5000, (bridge.bRepBodies.count, volume('D_Q3_Outboard_bridge') - before)
    M.place('D_03', L.comp('D_Bearing_6002'), (0, 0, -92))
    report(d, ['D_Q3_Outboard_bridge'])
    print('bridge grew by', round(volume('D_Q3_Outboard_bridge') - before), 'mm3')
