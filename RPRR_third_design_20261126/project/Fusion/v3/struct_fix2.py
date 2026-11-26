"""Second round of fixes from the structure review (docs/02_mechanical_design.md). after CAD edition 3, 2026-11-26.

Run inside Fusion on the working document E_Writing_q3_50W_build, one stage at a time:
  flanges()        large pulleys (90 and 120 teeth) lose their flanges, as stock ones have none
  q1_carrier()     the q1 output pulley is bolted to a collar that is set-screwed on the 40 mm spindle
  screws_flats()   set screws in every pulley hub and a flat under each on the shaft
  rings()          retaining rings that fix the small shafts along their axis
  small_fixes()    q1 idlers brought onto their belts, gap in the q2 coupling, second screw for the q3 idler bracket
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


def guard():
    app = core.Application.get()
    if app.activeDocument.name != DOC:
        raise RuntimeError('Expected the working document, got ' + app.activeDocument.name)
    return L.design()


def frame(o):
    """Origin and axes of a part in its sub-assembly, in mm."""
    p, x, y, z = o.transform2.getAsCoordinateSystem()
    return tuple(v * 10 for v in p.asArray()), x.asArray(), y.asArray(), z.asArray()


def add(a, b, k=1.0):
    return tuple(u + k * v for u, v in zip(a, b))


def dot(a, b):
    return sum(u * v for u, v in zip(a, b))


def instances(group, name):
    return [o for o in L.group(group).component.occurrences if o.component.name == name]


def put(group, o, mat):
    """Place an existing part (see q3_50w.put: local matrix first, root coordinates if that misses)."""
    d = L.design()
    err = lambda: max(abs(a - b) for a, b in zip(o.transform2.asArray(), mat.asArray()))

    def write(m):
        o.timelineObject.rollTo(False)
        try:
            o.initialTransform = m
        finally:
            d.timeline.moveToEnd()
    write(mat)
    if err() > 1e-6:
        in_root = mat.copy()
        in_root.transformBy(L.group(group).transform2)
        write(in_root)
    assert err() < 1e-6, (o.name, o.transform2.asArray(), mat.asArray())
    o.isGroundToParent = True


def report(d, names):
    d.computeAll()
    for n in names:
        c = L.comp(n)
        bb = c.bRepBodies.item(0).boundingBox
        print(n, 'bodies', c.bRepBodies.count, 'z', round(bb.minPoint.z * 10, 2), round(bb.maxPoint.z * 10, 2),
              'volume', round(sum(b.volume for b in c.bRepBodies) * 1000, 1))
    print('timeline', d.timeline.count, 'problems', L.timeline_problems())


# ---------------------------------------------------------------- 1. flanges
def flanges():
    d = guard()
    for name, fa, fb, hub_start, hub_len in (
            ('D_GT3M_120T_W9_D8', 'Retaining_flange_A', 'Retaining_flange_B', -7.5, 7.5),
            ('D_GT3M_90T_W9_D8', 'Stage1_90T_flange_A', 'Stage1_90T_flange_B', -10.5, 10.5),
            ('D_GT3M_120T_W15_D15', 'Retaining_flange_A', 'Retaining_flange_B', -11.0, 11.0),
            ('D_GT3M_120T_W15_D40', 'Retaining_flange_A', 'Retaining_flange_B', None, None)):
        c = L.comp(name)
        ef = c.features.extrudeFeatures
        if hub_start is not None:               # the hub used to end on flange A: bring it to the pulley face first
            h = ef.itemByName('Set_screw_hub')
            L.set_start_offset(h, hub_start)
            L.set_distance(h, hub_len)
        for f in (fa, fb):
            ef.itemByName(f).deleteMe()
        assert c.bRepBodies.count == 1, (name, c.bRepBodies.count)
    report(d, ['D_GT3M_120T_W9_D8', 'D_GT3M_90T_W9_D8', 'D_GT3M_120T_W15_D15', 'D_GT3M_120T_W15_D40'])


# ---------------------------------------------------------------- 2. q1 output pulley on the spindle
BOLT_CIRCLE = 25.0


def q1_carrier():
    """The stock 120-tooth pulley can be bored to 25.4 mm at most and its hub is 31.75 mm, so on the
    40 mm spindle it is a reworked part: bored to 40 mm without hub and bolted with six M4 screws to
    a collar that is fixed on the spindle with two M5 set screws. D_02 frame: y up, spindle on the y axis;
    pulley from y 46 to 61; collar below it."""
    d = guard()
    pts = [(BOLT_CIRCLE * math.cos(math.radians(30 + 60 * i)), BOLT_CIRCLE * math.sin(math.radians(30 + 60 * i))) for i in range(6)]
    pulley = L.comp('D_GT3M_120T_W15_D40')
    M.holes(pulley, 'Carrier_M4_clearance', -1, 17, pts, 2.2)            # between the six windows
    car = M.part('D_02', 'D_Q1_Pulley_carrier', (0, 29, 0), M.AL, FLOOR)
    M.disk(car, 'Clamp_collar', 0, 12, 27.0, 20.0)                        # y 29..41
    M.disk(car, 'Pulley_flange', 12, 5, 31.0, 20.0)                       # y 41..46
    M.holes(car, 'Pulley_M4_taps', 12, 5, pts, 1.65)
    sk = car.sketches.add(car.yZConstructionPlane)
    sk.name = 'Spindle_M5_set_screw_taps_profile'
    p = sk.modelToSketchSpace(core.Point3D.create(0, 0, 0.6))
    sk.sketchCurves.sketchCircles.addByCenterRadius(core.Point3D.create(p.x, p.y, 0), 0.21)
    L.extrude(car, sk.profiles.item(0), 2 * 28, 0, M.CUT, 'Spindle_M5_set_screw_taps', symmetric=True)
    sk.isVisible = False
    assert car.bRepBodies.count == 1
    bolt = M.part('D_02', 'D_M4x20_socket_screw', (pts[0][0], 65, -pts[0][1]), M.STEEL, ((1, 0, 0), (0, 0, 1)))   # own z points down
    M.disk(bolt, 'Socket_head', 0, 4, 3.5)                                # y 65..61, on the pulley
    M.disk(bolt, 'Shank', 4, 15, 2.0)                                     # through the pulley
    M.disk(bolt, 'Thread_root', 19, 5, 1.65)                              # in the carrier flange
    M.disk(bolt, 'Hex_socket', 0, 2.2, 1.5, op=M.CUT)
    for x, v in pts[1:]:
        M.place('D_02', bolt, (x, 65, -v), ((1, 0, 0), (0, 0, 1)))
    grub = M.part('D_02', 'D_Set_screw_M5x6', (21.0, 35, 0), M.STEEL, ((0, 1, 0), (0, 0, 1)))       # own z along +x
    M.disk(grub, 'Thread_minor_diameter', 0, 6, 2.1)
    M.place('D_02', grub, (-21.0, 35, 0), ((0, 0, 1), (0, 1, 0)))                                   # own z along -x
    spindle = L.comp('D_Q1_Spindle')                                      # spindle own z = y - 6; flats at y 35
    for s in (1, -1):
        M.rect(spindle, f'Set_screw_flat_{0 if s > 0 else 1}', 26.0, 6.0, (min(s * 19.6, s * 21), -8, max(s * 19.6, s * 21), 8), M.CUT)
    report(d, ['D_GT3M_120T_W15_D40', 'D_Q1_Pulley_carrier', 'D_Q1_Spindle', 'D_M4x20_socket_screw'])


# ---------------------------------------------------------------- 3. set screws and flats
HUBS = [  # group, pulley, shaft, z of the tapped holes (pulley frame), hub radius, shaft radius there, screw radius, screw length
    ('D_03', 'D_GT3M_16T_W9_D6', 'D_Q3_Input_shaft', -3.5, 8.76, 3.0, 1.25, 5.0),
    ('D_03', 'D_GT3M_90T_W9_D8', 'D_Q3_Intermediate_shaft', -5.75, 15.875, 4.0, 1.65, 8.0),
    ('D_03', 'D_GT3M_16T_W15_D6', 'D_Q3_Intermediate_shaft', -4.25, 8.76, 3.0, 1.25, 5.0),
    ('D_03', 'D_GT3M_20T_W6_D6', 'D_Q4_Input_shaft_v3', 43.5, 6.35, 3.0, 1.25, 3.0),
    ('D_01', 'D_GT3M_18T_W9_D6', 'D_Q1_Input_shaft', -4.5, 5.6, 3.0, 1.25, 2.5),
    ('D_01', 'D_GT3M_120T_W9_D8', 'D_Q1_Intermediate_shaft', -4.75, 15.875, 4.0, 1.65, 8.0),
    ('D_01', 'D_GT3M_16T_W15_D6', 'D_Q1_Intermediate_shaft', -4.25, 8.76, 3.0, 1.25, 5.0),
    ('D_05', 'D_GT3M_40T_W9_D8', 'D_Q4_Wrist_shaft', -4.5, 14.6, 4.0, 1.65, 8.0)]


def screws_flats():
    d = guard()
    made = {}
    n = 0
    for g, pulley, shaft, zh, hub_r, shaft_r, sr, sl in HUBS:
        po = instances(g, pulley)
        so = instances(g, shaft)
        assert len(po) == 1 and len(so) == 1, (g, pulley, len(po), shaft, len(so))
        o, X, Y, Z = frame(po[0])
        centre = add(o, Z, zh)
        key = f'D_Set_screw_M{3 if sr < 1.5 else 4}x{sl:g}'.replace('.', '_')
        for side in (1, -1):
            pos = add(centre, X, side * (hub_r - sl))
            axes = (Y, Z) if side > 0 else (Z, Y)             # own z = +X or -X of the pulley
            if key not in made:
                c = M.part(g, key, pos, M.STEEL, axes)
                M.disk(c, 'Thread_minor_diameter', 0, sl, sr)
                made[key] = c
            else:
                M.place(g, made[key], pos, axes)
            n += 1
        # flats on the shaft, 0.3 mm deep, 5 mm long, under both screws
        s0, SX, SY, SZ = frame(so[0])
        rel = add(centre, s0, -1)
        zs = dot(rel, SZ)
        u = (dot(X, SX), dot(X, SY))
        v = (-u[1], u[0])
        sc = L.comp(shaft)
        tag = pulley.split('_')[2]
        for side in (1, -1):
            a, b = side * (shaft_r - 0.3), side * (shaft_r + 1.0)
            sk = M.sketch(sc, f'Flat_{tag}_{0 if side > 0 else 1}')
            L.poly(sk, [(u[0] * a + v[0] * shaft_r, u[1] * a + v[1] * shaft_r), (u[0] * b + v[0] * shaft_r, u[1] * b + v[1] * shaft_r),
                        (u[0] * b - v[0] * shaft_r, u[1] * b - v[1] * shaft_r), (u[0] * a - v[0] * shaft_r, u[1] * a - v[1] * shaft_r)])
            before = sc.bRepBodies.item(0).volume
            M.solid(sc, sk, zs - 2.5, 5.0, f'Flat_{tag}_{0 if side > 0 else 1}', M.CUT, True)
            assert sc.bRepBodies.count == 1 and sc.bRepBodies.item(0).volume < before, (shaft, tag)
    print('set screws placed', n, 'kinds', sorted(made))
    report(d, sorted({h[2] for h in HUBS}))


# ---------------------------------------------------------------- 4. retaining rings
RINGS = [  # group, shaft, shaft radius there, position along the group axis of the ring face next to the bearing, +1 if the ring lies on the + side
    ('D_03', 'D_Q3_Input_shaft', 3.0, -90.0, -1), ('D_03', 'D_Q3_Input_shaft', 3.0, -84.0, +1),
    ('D_03', 'D_Q3_Intermediate_shaft', 4.0, -90.0, -1), ('D_03', 'D_Q3_Intermediate_shaft', 4.0, -83.0, +1),
    ('D_03', 'D_Q4_Input_shaft_v3', 3.0, 56.0, -1), ('D_03', 'D_Q4_Input_shaft_v3', 3.0, 62.0, +1),
    ('D_01', 'D_Q1_Input_shaft', 3.0, 15.0, +1), ('D_01', 'D_Q1_Input_shaft', 3.0, 47.9, -1),
    ('D_01', 'D_Q1_Intermediate_shaft', 4.0, 15.0, +1), ('D_01', 'D_Q1_Intermediate_shaft', 3.0, 75.0, -1)]
RING = {3.0: dict(name='D_Retaining_ring_6', t=0.7, groove=2.85, outer=4.3), 4.0: dict(name='D_Retaining_ring_8', t=0.8, groove=3.8, outer=5.6)}


def rings():
    """One ring groove and ring next to a bearing face. q3 and q4 shafts: both sides of one bearing
    (the other bearing is free to slide); the two vertical q1 shafts: above the lower bearing and
    below the upper one."""
    d = guard()
    made = {}
    for k, (g, shaft, r, at, side) in enumerate(RINGS):
        spec = RING[r]
        so = instances(g, shaft)[0]
        s0, SX, SY, SZ = frame(so)
        axis = max(range(3), key=lambda i: abs(SZ[i]))
        assert abs(abs(SZ[axis]) - 1) < 1e-9 and SZ[axis] > 0, (shaft, SZ)
        lo = at if side > 0 else at - spec['t']                 # ring from lo to lo + t along the group axis
        z_local = lo - s0[axis]
        M.disk(L.comp(shaft), f'Ring_groove_{k}', z_local, spec['t'], r + 0.6, spec['groove'], op=M.CUT)
        pos = list(s0)
        pos[axis] = lo
        axes = (SX, SY)
        if spec['name'] not in made:
            c = M.part(g, spec['name'], tuple(pos), M.STEEL, axes)
            M.disk(c, 'Ring', 0, spec['t'], spec['outer'], spec['groove'])
            made[spec['name']] = c
        else:
            M.place(g, made[spec['name']], tuple(pos), axes)
    print('rings placed', len(RINGS))
    report(d, sorted({x[1] for x in RINGS}))


# ---------------------------------------------------------------- 5. small fixes
def small_fixes():
    d = guard()
    app = core.Application.get()
    mm = app.measureManager
    root = d.rootComponent
    # (a) the two q1 idlers: move each onto the back of its belt (D_01 is fixed at the origin: group = world)
    for roller, belt in (('D_Q1_Stage1_idler', 'D_Belt_Q1_stage1'), ('D_Belt_idler_15', 'D_Belt_Q1_stage2')):
        ro = next(o for o in root.allOccurrences if o.fullPathName.startswith('D_01') and o.component.name == roller)
        bo = next(o for o in root.allOccurrences if o.fullPathName.startswith('D_01') and o.component.name == belt)
        res = mm.measureMinimumDistance(ro.bRepBodies.item(0), bo.bRepBodies.item(0))
        gap = res.value * 10
        a, b = res.positionOne, res.positionTwo
        step = ((b.x - a.x) * 10, 0.0, (b.z - a.z) * 10)
        if gap < 1e-4:
            print(roller, 'already touches')
            continue
        x0, _, z0 = frame(instances('D_01', roller)[0])[0]
        moved = []
        same_axis = [o for o in L.group('D_01').component.occurrences
                     if abs(o.transform2.translation.x * 10 - x0) < 0.01 and abs(o.transform2.translation.z * 10 - z0) < 0.01]
        for o in same_axis:                      # listed first: moving a part rolls the timeline
            if True:
                m = o.transform2.copy()
                t = m.translation
                t.add(core.Vector3D.create(step[0] / 10, 0, step[2] / 10))
                m.translation = t
                put('D_01', o, m)
                moved.append(o.name)
        after = mm.measureMinimumDistance(ro.bRepBodies.item(0), bo.bRepBodies.item(0)).value * 10
        print(roller, 'gap', round(gap, 3), '-> ', round(after, 4), 'moved', moved)
    # (b) q2: 1 mm between the motor shaft and the ball screw inside the coupling
    M.disk(L.comp('D_Q2_Ballscrew_12x10'), 'Coupling_end_gap', 0, 1.0, 7.0, op=M.CUT)
    # (c) q3 stage-2 idler bracket: second lock screw, 28 mm from the intermediate shaft
    mid = B.MID
    lay = __import__('json').loads((HERE / 'q3_50w_layout.json').read_text(encoding='utf8'))
    swing = math.radians(lay['swing'])
    sx, sy = mid[0] + 28.0, mid[1]
    cheek = L.comp('D_Q3_Drive_cheek')
    inside = cheek.bRepBodies.item(0).pointContainment(core.Point3D.create(sx / 10, sy / 10, 0.45))
    assert inside == fusion.PointContainment.PointInsidePointContainment, 'no material in the side plate for the second lock screw'
    M.holes(cheek, 'Second_lock_M4_tap', 0, 9, [(sx, sy)], 1.65)
    bracket = L.comp('D_Q3_Idler_bracket')
    sk = M.sketch(bracket, 'Second_angular_slot')
    r, half, span, c0 = 28.0, 2.2, math.radians(10), -swing        # the bracket is turned by swing, so the screw sits at -swing in its frame
    pt = lambda rad, th: M.P(mid[0] + rad * math.cos(th), mid[1] + rad * math.sin(th))
    arcs = sk.sketchCurves.sketchArcs
    for rad in (r + half, r - half):
        arcs.addByThreePoints(pt(rad, c0 - span), pt(rad, c0), pt(rad, c0 + span))
    for ang, s in ((c0 - span, -1), (c0 + span, 1)):
        middle = M.P(mid[0] + r * math.cos(ang) - s * half * math.sin(ang), mid[1] + r * math.sin(ang) + s * half * math.cos(ang))
        arcs.addByThreePoints(pt(r + half, ang), middle, pt(r - half, ang))
    before = bracket.bRepBodies.item(0).volume
    M.solid(bracket, sk, 0, 7, 'Second_angular_slot', M.CUT)
    removed = (before - bracket.bRepBodies.item(0).volume) * 1000
    ideal = (2 * span * r * 2 * half + math.pi * half ** 2) * 7
    assert bracket.bRepBodies.count == 1 and abs(removed / ideal - 1) < 0.02, (removed, ideal)
    M.place('D_03', L.comp('D_Q3_Tension_lock_spacer'), (sx, sy, -41))
    M.place('D_03', L.comp('D_Q3_Tension_lock_M4'), (sx, sy, -51.5))
    report(d, ['D_Q2_Ballscrew_12x10', 'D_Q3_Idler_bracket', 'D_Q3_Drive_cheek'])


def q1_idlers_along_slots():
    """The q1 idlers can only slide along x in their slots, so they are brought onto the belts by moving
    along x only (small_fixes had moved them straight towards the belt, which pushed the stage-2 axle
    0.2 mm sideways into the edge of its slot).
    Found on 2026-11-26: writing initialTransform of one instance, with the timeline rolled back, also
    shifts the later instances of the same component by the same amount. So every position here is
    written as an absolute target, in the order the instances were made, and checked at the end."""
    d = guard()
    mm = core.Application.get().measureManager
    root = d.rootComponent
    base = L.group('D_01').component
    lay = __import__('json').loads((HERE / 'q3_50w_layout.json').read_text(encoding='utf8'))
    sets = [dict(roller='D_Q1_Stage1_idler', belt='D_Belt_Q1_stage1', x0=-86.727, z0=84.843,
                 members=[('D_Q1_Stage1_idler', None), ('D_Q1_Stage1_adjuster', None), ('D_Bearing_MR126', 24.0), ('D_Bearing_MR126', 29.0)]),
            dict(roller='D_Belt_idler_15', belt='D_Belt_Q1_stage2', x0=-70.69, z0=-36.17,
                 members=[('D_Belt_idler_15', None), ('D_Q1_Adjustable_idler_axle', None), ('D_Q1_Idler_locknut', None),
                          ('D_Bearing_MR126', 46.0), ('D_Bearing_MR126', 57.0)])]

    def member(name, y):
        return next(o for o in base.occurrences if o.component.name == name and (y is None or abs(o.transform2.translation.y * 10 - y) < 0.01))

    def place_set(s, x):
        for name, y in s['members']:
            o = member(name, y)
            m = o.transform2.copy()
            t = m.translation
            t.x, t.z = x / 10, s['z0'] / 10
            m.translation = t
            put('D_01', o, m)

    def gap(s):
        ro = next(o for o in root.allOccurrences if o.fullPathName.startswith('D_01') and o.component.name == s['roller'])
        bo = next(o for o in root.allOccurrences if o.fullPathName.startswith('D_01') and o.component.name == s['belt'])
        r = mm.measureMinimumDistance(ro.bRepBodies.item(0), bo.bRepBodies.item(0))
        a, b = r.positionOne, r.positionTwo
        return r.value * 10, (b.x - a.x) * 10

    for s in sets:
        x = s['x0']
        place_set(s, x)
        for _ in range(8):
            g, gx = gap(s)
            if g < 0.003:
                break
            x += 0.98 * g * g / gx          # along x: gap / cos(angle between x and the direction to the belt)
            place_set(s, x)
        s['x'] = x
        print(s['roller'], 'x', s['x0'], '->', round(x, 4), 'gap now', round(gap(s)[0], 4))
    # put right what the moves above dragged along, then check every instance against its target
    for _ in range(3):
        for s in sets:
            place_set(s, s['x'])
        q3 = next(o for o in L.group('D_03').component.occurrences if o.component.name == 'D_Belt_idler_15')
        put('D_03', q3, M.matrix((lay['idler2'][0], lay['idler2'][1], -71)))
        bad = []
        for s in sets:
            for name, y in s['members']:
                t = member(name, y).transform2.translation
                if abs(t.x * 10 - s['x']) > 1e-6 or abs(t.z * 10 - s['z0']) > 1e-6:
                    bad.append((name, y))
        t = q3.transform2.translation
        if abs(t.x * 10 - lay['idler2'][0]) > 1e-6 or abs(t.y * 10 - lay['idler2'][1]) > 1e-6:
            bad.append(('q3 roller', None))
        if not bad:
            break
    assert not bad, bad
    d.computeAll()
    print('timeline', d.timeline.count, 'problems', L.timeline_problems())


def seat_set_screws():
    """The set screws were drawn flush with the hub surface, so their inner ends stopped short of the
    flats on the shafts. Each kind is pushed in until it stands on the flat (0.3 or 0.4 mm below the
    shaft surface). The wrist pulley has a thinner hub than the other pulleys with M4x8 screws, so its
    two screws (the last instances made) are moved back out by the difference."""
    d = guard()
    for name, length, r, inward in (('D_Set_screw_M3x5', 5.0, 1.25, 8.76 - 5.0 - 2.7), ('D_Set_screw_M3x3', 3.0, 1.25, 6.35 - 3.0 - 2.7),
                                    ('D_Set_screw_M3x2_5', 2.5, 1.25, 5.6 - 2.5 - 2.7), ('D_Set_screw_M4x8', 8.0, 1.65, 15.875 - 8.0 - 3.7),
                                    ('D_Set_screw_M5x6', 6.0, 2.1, 21.0 - 19.6)):
        c = L.comp(name)
        M.disk(c, 'Seat_on_flat', -inward, inward, r, op=M.JOIN)
        sk = M.sketch(c, 'Trim_outer_end')              # plain cut: with participant bodies set (cadlib.extrude)
        L.circle(sk, 0, 0, r + 0.2)                     # Fusion found no target body in these small parts
        ef = c.features.extrudeFeatures
        inp = ef.createInput(sk.profiles.item(0), M.CUT)
        inp.setDistanceExtent(False, core.ValueInput.createByString(f'{inward} mm'))
        inp.startExtent = fusion.OffsetStartDefinition.create(core.ValueInput.createByString(f'{length - inward} mm'))
        ef.add(inp).name = 'Trim_outer_end'
        sk.isVisible = False
        bb = c.bRepBodies.item(0).boundingBox
        assert c.bRepBodies.count == 1 and abs((bb.maxPoint.z - bb.minPoint.z) * 10 - length) < 0.05, (name, c.bRepBodies.count, (bb.maxPoint.z - bb.minPoint.z) * 10)   # boxes are loose by about 0.02 mm
    wrist = [o for o in L.group('D_05').component.occurrences if o.component.name == 'D_Set_screw_M4x8']
    assert len(wrist) == 2
    back = (15.875 - 8.0 - 3.7) - (14.6 - 8.0 - 3.7)
    for o in wrist:
        p, X, Y, Z = frame(o)
        m = o.transform2.copy()
        m.translation = core.Vector3D.create(*[(p[i] + Z[i] * back) / 10 for i in range(3)])
        put('D_05', o, m)
    d.computeAll()
    print('wrist screws moved out by', round(back, 3), 'mm; timeline', d.timeline.count, 'problems', L.timeline_problems())
