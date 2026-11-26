"""Ninth round (after CAD edition 3, 2026-11-26): lightening holes, ball-screw detail, real-material colours.

Run inside Fusion on the working document E_Writing_q3_50W_build, one stage at a time:
  arm_holes()     22 mm lightening holes in the four walls of the arm tube
  screw_thread()  the ball screw gets its helical ball groove (lead 10 mm)
  rail_screws()   counterbores in the rail and the ten M3 screws that hold it
  screw_joint()   revolute joint for the screw and a motion link to q2: 10 mm of travel = one turn
  colours()       every part gets the appearance of the material it is made of
Coordinates in mm. D_04 frame: origin on the q3 axis, x along the arm, y up, z sideways.
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
import struct_fix6 as S6
for _m in (L, M, N, S6):
    importlib.reload(_m)

DOC = 'E_Writing_q3_50W_build'
LOG = HERE / 'struct_fix9_log.txt'


def guard():
    app = core.Application.get()
    if app.activeDocument.name != DOC:
        raise RuntimeError('Expected the working document, got ' + app.activeDocument.name)
    return L.design()


def say(text):
    with open(LOG, 'a', encoding='utf8') as f:
        f.write(text + '\n')
    print(text)


def healthy(d):
    return [(i, d.timeline.item(i).name) for i in range(d.timeline.count)
            if d.timeline.item(i).healthState != fusion.FeatureHealthStates.HealthyFeatureHealthState]


def volume(c):
    return sum(b.volume for b in c.bRepBodies if b.isSolid) * 1000


def done(d, names):
    d.computeAll()
    for n in names:
        c = L.comp(n)
        say(f'{n}: bodies {c.bRepBodies.count}, volume {volume(c):.1f} mm3, mass {c.physicalProperties.mass * 1000:.1f} g')
    bad = healthy(d)
    say(f'timeline {d.timeline.count}, not healthy {bad}')
    assert not bad, bad


def interference_pairs(d):
    """One interference analysis over all robot bodies (reference pose); {pair of part names: mm3}."""
    coll = core.ObjectCollection.create()

    def walk(occ):
        for b in occ.bRepBodies:
            coll.add(b)
        for c in occ.childOccurrences:
            walk(c)
    for o in d.rootComponent.occurrences:
        if not o.component.name.startswith('Wave_Drum'):
            walk(o)
    inp = d.createInterferenceInput(coll)
    inp.areCoincidentFacesIncluded = False
    res = d.analyzeInterference(inp)
    out = {}
    for i in range(res.count):
        r = res.item(i)
        key = ' | '.join(sorted((r.entityOne.parentComponent.name, r.entityTwo.parentComponent.name)))
        try:
            vol = r.interferenceBody.volume * 1000
        except RuntimeError:
            vol = 0.0                                      # an overlap too thin to measure still counts
        out[key] = round(out.get(key, 0.0) + vol, 3)
    return out


# ---------------------------------------------------------------- arm
HOLE_R = 11.0
TOP_X = [40 + 32 * k for k in range(7)]          # 40 ... 232: between the clamp hub (x < 17) and the end of the webs (x 274)
SIDE_X = [152 + 32 * k for k in range(4)]        # 152 ... 248: half a pitch off the top holes; clear of the idler
                                                 # bracket and its slot (x < 113) and of the fork bushes (x > 285.5)


def arm_holes():
    d = guard()
    tube = L.comp('D_Q3_Link_40x40x3')
    v0, m0 = volume(tube), tube.physicalProperties.mass * 1000
    N.cut('D_04', 'D_Q3_Link_40x40x3', 'Lightening_holes_top_bottom', 1, -20, 20, [('circle', x, 0, HOLE_R) for x in TOP_X])
    N.cut('D_04', 'D_Q3_Link_40x40x3', 'Lightening_holes_sides', 2, -20, 20, [('circle', x, 0, HOLE_R) for x in SIDE_X])
    removed = v0 - volume(tube)
    ideal = 2 * (len(TOP_X) + len(SIDE_X)) * math.pi * HOLE_R ** 2 * 3
    assert tube.bRepBodies.count == 1 and abs(removed / ideal - 1) < 0.01, (tube.bRepBodies.count, removed, ideal)
    hits = S6.interference_with(d, tube.name)
    assert not hits, hits
    say(f'arm tube: {2 * (len(TOP_X) + len(SIDE_X))} holes, removed {removed:.0f} mm3 (ideal {ideal:.0f}); '
        f'mass {m0:.1f} -> {tube.physicalProperties.mass * 1000:.1f} g')
    done(d, ['D_Q3_Link_40x40x3'])


# ---------------------------------------------------------------- ball screw
LEAD = 10.0            # mm per turn (R12-10)
BALL_CIRCLE_R = 6.15   # mm, radius on which the ball centres run (shown, not from the catalogue)
GROOVE_R = 1.25        # mm, groove radius (ball 2.381 mm); groove depth 6 - (6.15 - 1.25) = 1.1 mm
THREAD_Z = (38.0, 222.0)   # the 12 mm section of the screw in its own frame


def screw_thread(points_per_turn=16, overrun=2.0):
    """Right-hand helical ball groove over the whole 12 mm section. The path is a 3D spline through
    points of the helix; it starts and ends 2 mm beyond the section, in the air round the journals."""
    d = guard()
    sc = L.comp('D_Q2_Ballscrew_12x10')
    v0, m0 = volume(sc), sc.physicalProperties.mass * 1000
    z0, z1 = THREAD_Z[0] - overrun, THREAD_Z[1] + overrun
    n = int(round((z1 - z0) / LEAD * points_per_turn))
    sk = sc.sketches.add(sc.xYConstructionPlane)
    sk.name = 'Ball_groove_helix'
    sk.isComputeDeferred = True
    pts = core.ObjectCollection.create()
    for i in range(n + 1):
        z = z0 + (z1 - z0) * i / n
        a = 2 * math.pi * (z - z0) / LEAD
        pts.add(core.Point3D.create(BALL_CIRCLE_R * math.cos(a) / 10, BALL_CIRCLE_R * math.sin(a) / 10, z / 10))
    spline = sk.sketchCurves.sketchFittedSplines.add(pts)
    sk.isComputeDeferred = False
    path = sc.features.createPath(spline, False)
    pf = sc.features.pipeFeatures
    inp = pf.createInput(path, M.CUT)
    inp.sectionType = fusion.PipeSectionTypes.CircularPipeSectionType
    inp.sectionSize = core.ValueInput.createByString(f'{2 * GROOVE_R} mm')
    # without this the cut also goes into every other body it meets: the first run cut the groove
    # into the nut and into both supports as well (found afterwards from their masses)
    inp.participantBodies = [sc.bRepBodies.item(0)]
    f = pf.add(inp)
    f.name = 'Ball_groove_lead10'
    sk.isVisible = False
    removed = v0 - volume(sc)
    say(f'ball screw: helix of {n + 1} points, {(z1 - z0) / LEAD:.1f} turns; removed {removed:.0f} mm3; '
        f'mass {m0:.1f} -> {sc.physicalProperties.mass * 1000:.1f} g; bodies {sc.bRepBodies.count}, faces {sc.bRepBodies.item(0).faces.count}')
    assert sc.bRepBodies.count == 1 and 600 < removed < 3000, (sc.bRepBodies.count, removed)
    hits = S6.interference_with(d, sc.name)
    assert not hits, hits
    done(d, ['D_Q2_Ballscrew_12x10'])


# ---------------------------------------------------------------- rail
FLOOR = ((1, 0, 0), (0, 0, -1))        # part z = group +y (up)


def keep_mass(comp, mass_g):
    """Catalogue parts carry a density chosen so that the drawn volume has the catalogue mass.
    After a change of shape the density of that material is set again."""
    d = L.design()
    mat = comp.material
    users = [c.name for c in d.allComponents if c.bRepBodies.count and c.material and c.material.name == mat.name]
    assert users == [comp.name], users
    rho = mass_g / volume(comp) * 1e6                      # g/mm3 -> kg/m3
    mat.materialProperties.itemById('structural_Density').value = rho
    mat.name = mat.name[:mat.name.rindex('(')] + f'({rho:.2f} kg/m3)'
    return rho


def rail_screws():
    """MGN12 rail: 6 x 4.5 mm counterbores over the ten 3.4 mm holes, and ten M3 x 10 socket screws.
    Rail frame: x along, y across, z up (0 = plate, 8 = top)."""
    d = guard()
    rail = L.comp('D_MGN12_Rail')
    m0 = rail.physicalProperties.mass * 1000
    xs = [-55 + 25 * k for k in range(10)]
    v0 = volume(rail)
    M.holes(rail, 'Counterbores_6x4_5', 3.5, 4.5, [(x, 0) for x in xs], 3.0)
    removed = v0 - volume(rail)
    ideal = 10 * math.pi * (3.0 ** 2 - 1.7 ** 2) * 4.5
    assert rail.bRepBodies.count == 1 and abs(removed / ideal - 1) < 0.02, (removed, ideal)
    rho = keep_mass(rail, m0)
    say(f'rail: counterbores removed {removed:.0f} mm3; density set to {rho:.1f} kg/m3 so that the mass stays {m0:.1f} g '
        f'(now {rail.physicalProperties.mass * 1000:.1f} g)')
    # screw: own z up, origin on the counterbore floor (rail z 3.5 -> D_02 y 184.5), rail centre line D_02 z -34
    sc = M.part('D_02', 'D_M3x10_rail_screw', (xs[0], 184.5, -34.0), M.STEEL, FLOOR)
    M.disk(sc, 'Shank_M3', -10, 10, 1.5)
    M.disk(sc, 'Head', 0, 3, 2.75)
    sk = M.sketch(sc, 'Hex_socket_2_5')
    L.poly(sk, [(1.443 * math.cos(math.radians(60 * k)), 1.443 * math.sin(math.radians(60 * k))) for k in range(6)])
    ef = sc.features.extrudeFeatures
    inp = ef.createInput(sk.profiles.item(0), M.CUT)
    inp.setDistanceExtent(False, core.ValueInput.createByString('1.6 mm'))
    inp.startExtent = fusion.OffsetStartDefinition.create(core.ValueInput.createByString('1.4 mm'))
    ef.add(inp).name = 'Hex_socket_2_5'
    sk.isVisible = False
    assert sc.bRepBodies.count == 1
    for x in xs[1:]:
        M.place('D_02', sc, (x, 184.5, -34.0), FLOOR)
    hits = S6.interference_with(d, sc.name)
    assert not hits, hits
    done(d, ['D_MGN12_Rail', 'D_M3x10_rail_screw'])


# ---------------------------------------------------------------- screw turns with the slider
def _box(occ):
    b = occ.bRepBodies.item(0).boundingBox
    return [b.minPoint.x, b.minPoint.y, b.minPoint.z, b.maxPoint.x, b.maxPoint.y, b.maxPoint.z]


def screw_joint():
    d = guard()
    root = d.rootComponent
    g2 = L.group('D_02')
    c2 = g2.component
    so = next(o for o in c2.occurrences if o.component.name == 'D_Q2_Ballscrew_12x10')
    po = next(o for o in c2.occurrences if o.component.name == 'D_Q2_Rail_plate')
    sp = next(o for o in g2.childOccurrences if o.component.name == 'D_Q2_Ballscrew_12x10')    # as seen from the root
    home = so.transform2.copy()
    box0 = _box(sp)
    so.isGroundToParent = False
    # once it is no longer pinned to its parent the screw jumps to an old stored position (seen: 21 mm
    # off and turned 11.9 deg). It is put back before the joint is made, because an as-built joint
    # takes the position it finds as its zero.
    say(f'screw box after unpinning: {[round(v * 10, 2) for v in _box(sp)]}')
    target = home.copy()
    target.transformBy(g2.transform2)
    assert root.transformOccurrences([sp], [target], True)
    # a position set like this is only temporary (the next joint solve went back to the old one):
    # it is captured, so that it becomes the stored position
    say(f'pending position to capture: {d.snapshots.hasPendingSnapshot}')
    if d.snapshots.hasPendingSnapshot:
        d.snapshots.add()
    assert max(abs(a - b) for a, b in zip(_box(sp), box0)) < 1e-6, (_box(sp), box0)
    face = next(f for f in so.component.bRepBodies.item(0).faces
                if f.geometry.objectType == core.Cylinder.classType() and abs(f.geometry.radius - 0.4) < 1e-5)
    geo = fusion.JointGeometry.createByNonPlanarFace(face.createForAssemblyContext(so), fusion.JointKeyPointTypes.StartKeyPoint)
    inp = c2.asBuiltJoints.createInput(so, po, geo)
    inp.setAsRevoluteJointMotion(fusion.JointDirections.ZAxisJointDirection)
    j = c2.asBuiltJoints.add(inp)
    j.name = 'q2_Screw_spin'
    q2 = root.asBuiltJoints.itemByName('q2_Radial_slide')
    ml = root.motionLinks
    inp = ml.createInput(q2, j.createForAssemblyContext(g2))
    inp.motionOne = fusion.JointMotionTypes.SliderJointSlideMotionType
    inp.valueOne = core.ValueInput.createByString(f'{LEAD} mm')
    inp.motionTwo = fusion.JointMotionTypes.RevoluteJointRotateMotionType
    inp.valueTwo = core.ValueInput.createByString('360 deg')
    inp.isReversed = True          # right-hand thread: slider towards +x <-> screw turns negative about +x
    link = ml.add(inp)
    link.name = 'q2_Lead_10mm_per_turn'
    say(f'joint {j.name}; link {link.name} health {link.healthState}, reversed {link.isReversed}')
    done(d, [])


def screw_state():
    """(turn of the screw about its axis in degrees, distance of its journal axis from the bore axis of
    the fixed support in mm, angle between the two axes in degrees), from the positions seen at the root."""
    g2 = L.group('D_02')
    sp = next(o for o in g2.childOccurrences if o.component.name == 'D_Q2_Ballscrew_12x10')
    ek = next(o for o in g2.childOccurrences if o.component.name == 'D_EK08_Fixed_support')
    a, g = sp.transform2.asArray(), g2.transform2.copy()
    g.invert()
    x_axis = core.Vector3D.create(a[0], a[4], a[8])        # screw's own x axis, in the root frame
    x_axis.transformBy(g)                                  # ... in the D_02 frame: unturned it is (0, 0, -1)
    turn = math.degrees(math.atan2(x_axis.y, -x_axis.z))
    cyl = lambda o, r: next(f.geometry for f in o.bRepBodies.item(0).faces
                            if f.geometry.objectType == core.Cylinder.classType() and abs(f.geometry.radius - r) < 1e-5)
    s, e = cyl(sp, 0.4), cyl(ek, 0.4)
    v = e.origin.vectorTo(s.origin)
    off = v.crossProduct(e.axis).length * 10
    tilt = math.degrees(math.acos(min(1.0, abs(s.axis.dotProduct(e.axis)))))
    return turn, off, tilt


def screw_test():
    d = guard()
    root = d.rootComponent
    q2 = root.asBuiltJoints.itemByName('q2_Radial_slide')
    link = root.motionLinks.itemByName('q2_Lead_10mm_per_turn')
    occ3 = L.group('D_03')
    x0 = occ3.transform2.translation.x
    rows = []
    for dmm in (0.0, 2.5, 5.0, -2.5, 61.0, -80.0, 0.0):
        q2.jointMotion.slideValue = dmm / 10
        moved = (occ3.transform2.translation.x - x0) * 10
        turn, off, tilt = screw_state()
        rows.append((dmm, moved, turn, off, tilt))
        say(f'q2 joint {dmm:+.1f} mm: slider moved {moved:+.3f} mm, screw turned {turn:+.2f} deg, '
            f'axis off the support bore by {off:.4f} mm and {tilt:.4f} deg')
    assert all(abs(r[1] - r[0]) < 1e-6 and r[3] < 1e-4 and r[4] < 1e-4 for r in rows), rows
    # right-hand thread, nut fixed against turning: slider +x by one lead <-> screw -360 deg about +x
    want = lambda dmm: (-36.0 * dmm + 180) % 360 - 180
    ok = all(abs((r[2] - want(r[0]) + 180) % 360 - 180) < 0.01 for r in rows)
    say(f'direction right for a right-hand thread: {ok} (link reversed: {link.isReversed})')
    return ok


def screw_angle():
    """Rotation of the screw about its own axis, read from its position inside D_02 (degrees)."""
    c2 = L.group('D_02').component
    so = next(o for o in c2.occurrences if o.component.name == 'D_Q2_Ballscrew_12x10')
    a = so.transform2.asArray()            # local x axis in D_02 = (a[0], a[4], a[8]); screw axis = D_02 +x
    # unturned: local x -> D_02 (0, 0, -1), local y -> D_02 (0, 1, 0)
    return math.degrees(math.atan2(a[4], -a[8]))


# ---------------------------------------------------------------- screw supports EK08 / EF08
# THK catalogue A15-330 (EK) and A15-316 (EF), same cross-section for both: width B 52, height H 32,
# bore centre h1 17 above the mounting face, raised block B1 25 wide, wings H1 26 high, two 6.6 mm
# holes at pitch P 38 with 11 x 12 mm counterbores from the top of the wings. Length 23 (EK08) / 14 (EF08).
# The model had the bore 16 mm above the foot and the EF08 only 42 mm wide.
ALONG_X = ((0, 0, -1), (0, 1, 0))       # part z = D_02 +x (the screw axis), part y = up
SUPPORTS = {'D_EK08_Fixed_support': dict(x=-26.5, length=23.0, mass=240.0),
            'D_EF08_End_support': dict(x=177.5, length=14.0, mass=130.0)}
AXIS_Y, AXIS_Z = 201.0, 86.0            # screw axis in D_02


def supports():
    d = guard()
    # riser plates 4 -> 3 mm, so that the bore centre is 17 mm above the foot of the support
    riser = L.comp('D_Q2_Support_riser')
    L.set_distance(riser.features.extrudeFeatures.itemByName('Foot'), 3)
    d.computeAll()
    assert not healthy(d), healthy(d)
    for name, p in SUPPORTS.items():
        c = L.comp(name)
        n = p['length']
        M.rect(c, 'Base_with_wings', 0, n, (-26, -17, 26, 9), M.JOIN)          # also closes the bore for a moment
        sk = M.sketch(c, 'Shoulders_and_top')
        L.rect(sk, -26.1, 9, -12.5, 16.1)
        L.rect(sk, 12.5, 9, 26.1, 16.1)
        L.rect(sk, -12.5, 15, 12.5, 16.1)
        M.solid(c, sk, 0, n, 'Shoulders_and_top', M.CUT, True)
        if name.startswith('D_EK08'):
            M.disk(c, 'Journal_bore_8', 0, n, 4.0, op=M.CUT)
        else:
            M.disk(c, 'Shaft_clearance_10', 0, n, 5.0, op=M.CUT)
            M.disk(c, 'Seat_606ZZ', 7, 7, 8.5, op=M.CUT)                       # bearing z 7..13, open to the outer face
        holes = [('circle', p['x'], AXIS_Z - 19, 3.3), ('circle', p['x'], AXIS_Z + 19, 3.3)]
        N.cut('D_02', name, 'Mounting_holes_6_6', 1, 184, 210, holes)
        N.cut('D_02', name, 'Counterbores_11x12', 1, 198, 210, [(s, x, z, 5.5) for s, x, z, _ in holes])
        b = c.bRepBodies.item(0).boundingBox
        box = [round(v * 10, 2) for v in (b.minPoint.x, b.minPoint.y, b.minPoint.z, b.maxPoint.x, b.maxPoint.y, b.maxPoint.z)]
        assert c.bRepBodies.count == 1 and box == [-26.0, -17.0, 0.0, 26.0, 15.0, n], (name, c.bRepBodies.count, box)
    # riser, plate
    ek = SUPPORTS['D_EK08_Fixed_support']['x']
    ef = SUPPORTS['D_EF08_End_support']['x']
    N.cut('D_02', 'D_Q2_Support_riser', 'M6_clearance_6_6', 1, 181, 184,
          [('circle', ek, AXIS_Z - 19, 3.3), ('circle', ek, AXIS_Z + 19, 3.3)])
    plate = L.comp('D_Q2_Rail_plate')
    v0 = volume(plate)
    spots = [(x, AXIS_Z + s * 19) for x in (ek, ef) for s in (-1, 1)]
    N.cut('D_02', 'D_Q2_Rail_plate', 'Support_M6_threads', 1, 171, 181, [('circle', x, z, 3.0) for x, z in spots])
    removed, ideal = v0 - volume(plate), 4 * math.pi * 9 * 10
    assert plate.bRepBodies.count == 1 and abs(removed / ideal - 1) < 0.01, (removed, ideal)     # all four in solid plate
    # screws M6 x 25: own z up, origin under the head (counterbore floor, D_02 y 198)
    sc = M.part('D_02', 'D_M6x25_socket_screw', (spots[0][0], 198.0, spots[0][1]), M.STEEL, FLOOR)
    M.disk(sc, 'Shank_M6', -25, 25, 3.0)
    M.disk(sc, 'Head', 0, 6, 5.0)
    sk = M.sketch(sc, 'Hex_socket_5')
    L.poly(sk, [(2.887 * math.cos(math.radians(60 * k)), 2.887 * math.sin(math.radians(60 * k))) for k in range(6)])
    ef_ = sc.features.extrudeFeatures
    inp = ef_.createInput(sk.profiles.item(0), M.CUT)
    inp.setDistanceExtent(False, core.ValueInput.createByString('3 mm'))
    inp.startExtent = fusion.OffsetStartDefinition.create(core.ValueInput.createByString('3 mm'))
    inp.participantBodies = [sc.bRepBodies.item(0)]
    ef_.add(inp).name = 'Hex_socket_5'
    sk.isVisible = False
    for x, z in spots[1:]:
        M.place('D_02', sc, (x, 198.0, z), FLOOR)
    # EF08: its 606ZZ bearing (6 x 17 x 6, shields) and the C6 ring on the journal
    bx = ef - 7 + 7                                   # support starts at ef - 7; bearing 7..13 from there
    br = M.part('D_02', 'D_Bearing_606ZZ', (bx, AXIS_Y, AXIS_Z), M.STEEL, ALONG_X)
    M.disk(br, 'Rings_and_shields', 0, 6, 8.5, 3.0)
    M.disk(br, 'Shield_recess_inner', 0, 0.4, 7.0, 4.2, op=M.CUT)
    M.disk(br, 'Shield_recess_outer', 5.6, 0.4, 7.0, 4.2, op=M.CUT)
    assert br.bRepBodies.count == 1
    screw = L.comp('D_Q2_Ballscrew_12x10')
    z_ring = bx + 6 + 52.0                            # screw frame: z = D_02 x + 52
    M.disk(screw, 'End_ring_groove', z_ring, 0.7, 3.6, 2.85, op=M.CUT)
    assert screw.bRepBodies.count == 1
    M.place('D_02', L.comp('D_Retaining_ring_6'), (bx + 6, AXIS_Y, AXIS_Z), ALONG_X)
    # catalogue masses: 240 g (EK08, unit) and 130 g (EF08 unit = housing + bearing)
    rho1 = keep_mass(L.comp('D_EK08_Fixed_support'), 240.0)
    m_b = br.physicalProperties.mass * 1000
    rho2 = keep_mass(L.comp('D_EF08_End_support'), 130.0 - m_b)
    say(f'supports: EK08 density {rho1:.0f} kg/m3 -> 240 g; EF08 housing density {rho2:.0f} kg/m3 -> {130 - m_b:.1f} g + bearing {m_b:.1f} g')
    hits = interference_pairs(d)
    assert not hits, hits
    done(d, list(SUPPORTS) + ['D_M6x25_socket_screw', 'D_Bearing_606ZZ', 'D_Q2_Support_riser', 'D_Q2_Ballscrew_12x10'])


# ---------------------------------------------------------------- colours
APPEARANCE_LIB = 'BA5EE55E-9982-449B-9D66-9F036540E140'      # Fusion appearance library
ALU, STEEL_SATIN, STEEL_GROUND = '\u94dd - \u7f0e\u5149', '\u94a2 - \u7f0e\u5149', '\u4e0d\u9508\u94a2 - \u629b\u5149'   # Chinese-language Fusion names: Stainless Steel - Polished; Aluminum - Satin; Steel - Satin
BLACK_OXIDE, ZINC, BLACK_PAINT, RUBBER, ABS_WHITE = '\u6d82\u5c42 - \u9ed1\u8272\u6c27\u5316\u7269', '\u94a2 - \u9540\u950c(\u5c0f)', '\u6d82\u6599 - \u74f7\u91c9\u6709\u5149\u6cfd(\u9ed1\u8272)', '\u6a61\u80f6 - \u8f6f', 'ABS(\u767d\u8272)'   # Chinese-language Fusion names: Paint - Enamel Glossy (Black); Coating - Black Oxide; Steel - Galvanized (small); ABS (White); Rubber - Soft; rubber


def appearance_for(name, material):
    """Appearance of the material the part is made of (and, for steel, of its usual finish)."""
    if name.startswith('Wave_Drum'):
        return None                                              # the bucket is not a robot part: left as it is
    if 'belt' in material or name.startswith('D_Belt_Q') or material.startswith('\u6a61\u80f6'):   # Chinese-language Fusion names: rubber
        return RUBBER
    if 'ABS' in material:
        return ABS_WHITE
    if name.startswith('D_HG_KR'):
        return BLACK_PAINT                                       # painted motor housing; the shaft is done by face
    if name.startswith(('D_EK08', 'D_EF08')):
        return BLACK_OXIDE                                       # TBI data sheets: black oxide housings
    if '6061' in material or '7075' in material or name.startswith('D_XGT2'):
        return ALU                                               # machined aluminium; coupling hubs are A2017
    low = name.lower()
    if name.startswith('D_Bearing_') and not name.startswith('D_Bearing_cap'):
        return STEEL_GROUND
    if name.startswith(('D_Q2_Ballscrew', 'D_MGN12_Rail')):
        return STEEL_GROUND                                      # ground bearing steel
    if name.startswith('D_RN8'):
        return BLACK_OXIDE                                       # lock nut of the fixed support
    if low.endswith('nut') and not name.startswith('D_R12'):
        return ZINC                                              # M4, M5 nuts and the idler lock nut
    if 'screw' in low or 'retaining_ring' in low or name in ('D_Q3_Split_clamp_M4', 'D_Q3_Tube_mount_M4', 'D_Q4_Foot_M4',
                                                              'D_Q4_Stub_M3', 'D_Q4_Transfer_retainer'):
        return BLACK_OXIDE                                       # socket screws, set screws, retaining rings
    return STEEL_SATIN                                           # ball nut, rail blocks, rollers, pins, spring, standoffs


def colours():
    d = guard()
    lib = core.Application.get().materialLibraries.itemById(APPEARANCE_LIB)

    def get(app_name):
        a = d.appearances.itemByName(app_name)
        return a if a else d.appearances.addByCopy(lib.appearances.itemByName(app_name), app_name)
    count, face_overrides, occ_overrides = {}, 0, 0
    for c in d.allComponents:
        if c.bRepBodies.count == 0:
            continue
        want = appearance_for(c.name, c.material.name if c.material else '')
        if want is None:
            continue
        a = get(want)
        for b in c.bRepBodies:
            for f in b.faces:
                if f.appearanceSourceType == core.AppearanceSourceTypes.FaceAppearanceSource:
                    f.appearance = None
                    face_overrides += 1
            b.appearance = a
        count.setdefault(want, []).append(c.name)
    for o in d.rootComponent.allOccurrences:
        if o.appearance is not None and not o.component.name.startswith('Wave_Drum'):
            o.appearance = None
            occ_overrides += 1
    # motor: bare steel shaft (everything in front of the 2.5 mm pilot)
    motor = L.comp('D_HG_KR053')
    steel, n_shaft = get(STEEL_SATIN), 0
    for f in motor.bRepBodies.item(0).faces:
        if f.boundingBox.minPoint.z * 10 > 2.5 - 0.01:
            f.appearance = steel
            n_shaft += 1
    for k in sorted(count):
        say(f'{k}: {len(count[k])} kinds - {", ".join(sorted(count[k]))}')
    say(f'motor shaft faces set to steel: {n_shaft}; face overrides cleared {face_overrides}, occurrence overrides cleared {occ_overrides}')
    unused = [a for a in d.appearances if not a.isUsed]
    for a in unused:
        try:
            a.deleteMe()
        except RuntimeError:
            pass
    say(f'appearances in the design now: {[a.name for a in d.appearances]}')
    core.Application.get().activeViewport.refresh()


# ---------------------------------------------------------------- final check
def check(tag='struct_fix9'):
    import json
    import runpy
    d = guard()
    root = d.rootComponent
    app = core.Application.get()
    out = dict(poses=[])
    poses = [p['q'] for p in json.loads((HERE / 'struct_fix8_check.json').read_text(encoding='utf8'))['poses']]
    poses += [[25, 33.5, -30, 100], [-25, 127.5, 30, -60]]            # q2 not a multiple of the lead
    R = runpy.run_path(str(HERE.parent / 'joint_limits_fusion.py'))['Robot'](d)
    try:
        for q in poses:
            c = R.collisions(q)
            turn, off, tilt = screw_state()
            out['poses'].append(dict(q=q, collisions=c, screw_turn_deg=round(turn, 2), screw_off_axis_mm=round(off, 5)))
            say(f'pose {q}: collisions {c}; screw turn {turn:.1f} deg, off axis {off:.5f} mm')
    finally:
        R.restore()
    out['screw_link_ok'] = screw_test()
    # every new part touches the part it sits on
    mm = app.measureManager
    g2 = L.group('D_02')
    by = {}
    for o in g2.childOccurrences:
        by.setdefault(o.component.name, []).append(o)
    seats = {'D_M3x10_rail_screw': 'D_MGN12_Rail', 'D_M6x25_socket_screw': None, 'D_Bearing_606ZZ': 'D_EF08_End_support',
             'D_EK08_Fixed_support': 'D_Q2_Support_riser', 'D_EF08_End_support': 'D_Q2_Support_riser',
             'D_Q2_Support_riser': 'D_Q2_Rail_plate', 'D_Q2_Ballscrew_12x10': 'D_Bearing_606ZZ'}
    gaps = {}
    for name, seat in seats.items():
        for o in by[name]:
            others = by[seat] if seat else by['D_EK08_Fixed_support'] + by['D_EF08_End_support']
            gaps[o.name] = round(min(mm.measureMinimumDistance(o.bRepBodies.item(0), s.bRepBodies.item(0)).value for s in others) * 10, 5)
    ring = [o for o in by['D_Retaining_ring_6']]
    for o in ring:
        gaps[o.name] = round(mm.measureMinimumDistance(o.bRepBodies.item(0), by['D_Bearing_606ZZ'][0].bRepBodies.item(0)).value * 10, 5)
    # second batch (lock nut round): parts anywhere under D_02, found from the root
    under = {}
    for o in root.allOccurrences:
        if o.fullPathName.startswith('D_02') and o.bRepBodies.count:
            under.setdefault(o.component.name, []).append(o)
    if 'D_RN8_Lock_nut' in under:
        for a, b in (('D_RN8_Lock_nut', 'D_EK08_Fixed_support'), ('D_RN8_Lock_nut', 'D_Q2_Ballscrew_12x10'),
                     ('D_XGT2_19C_8x6', 'D_Q2_Ballscrew_12x10'), ('D_XGT2_19C_8x6', 'D_HG_KR053'),
                     ('D_HG_KR053', 'D_Motor_flange'), ('D_Motor_flange', 'D_Q2_Motor_foot'), ('D_Q2_Motor_foot', 'D_Q2_Rail_plate')):
            gaps[a + ' - ' + b] = round(mm.measureMinimumDistance(under[a][0].bRepBodies.item(0), under[b][0].bRepBodies.item(0)).value * 10, 5)
    out['gap_to_seat_mm'] = gaps
    assert all(v < 1e-3 for v in gaps.values()), gaps
    d.computeAll()
    out['timeline'] = d.timeline.count
    out['not_healthy'] = healthy(d)
    kinds, inst = set(), 0
    for o in root.allOccurrences:
        if o.bRepBodies.count and not o.component.name.startswith('Wave_Drum'):
            kinds.add(o.component.name)
            inst += 1
    out['kinds'], out['instances'] = len(kinds), inst
    out['masses_kg'] = {o.component.name[:4]: round(o.getPhysicalProperties(fusion.CalculationAccuracy.HighCalculationAccuracy).mass, 4)
                        for o in root.occurrences}
    out['part_masses_g'] = {n: round(L.comp(n).physicalProperties.mass * 1000, 2) for n in
                            ('D_Q3_Link_40x40x3', 'D_Q2_Ballscrew_12x10', 'D_MGN12_Rail', 'D_EK08_Fixed_support', 'D_EF08_End_support',
                             'D_Bearing_606ZZ', 'D_Q2_Support_riser', 'D_Q2_Rail_plate', 'D_M3x10_rail_screw', 'D_M6x25_socket_screw')}
    em = d.exportManager
    out['export'] = em.execute(em.createFusionArchiveExportOptions(str(HERE / 'E_Writing.f3d')))
    (HERE / (tag + '_check.json')).write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding='utf8')
    say(f'CHECK DONE: timeline {out["timeline"]}, not healthy {out["not_healthy"]}, kinds {out["kinds"]}, instances {out["instances"]}, '
        f'masses {out["masses_kg"]}, export {out["export"]}')
