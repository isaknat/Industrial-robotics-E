"""Eleventh round (after CAD edition 3, 2026-11-26): pulleys and shafts turn with the joints.

The belts stay as they are (a belt running round its pulleys is not a rigid motion, so a joint cannot
show it). Six groups of parts ("rotors") are released inside their sub-assembly, each gets a revolute
joint about its shaft axis, its parts are tied together in a rigid group, and a motion link ties the
joint to the robot joint that drives it:

  rotor            carried by   turns by (about its own axis, same sense as the joint)
  q1_input         D_01         50      x q1     (18 -> 120, then 16 -> 120)
  q1_intermediate  D_01         7.5     x q1
  q3_input         D_03         42.1875 x q3     (16 -> 90, then 16 -> 120)
  q3_intermediate  D_03         7.5     x q3
  q4_transfer      D_03         q3 + 2 x q4      (60/20 pulley on the q3 axis; 20 -> 40 along the arm)
  q4_input         D_03         3 x q3 + 6 x q4  (20 -> 60)
The table is also stored in the design (attribute RPRR / SpinningParts) for the scripts that place the
robot directly (joint_limits_fusion.py, PlayWriting.py): parts that are free inside a sub-assembly do
not follow it there, so those scripts place every one of them.
Run inside Fusion on E_Writing_q3_50W_build: build([...keys]), test(), store().
"""
import importlib
import json
import math
import sys
from pathlib import Path
import adsk.core as core
import adsk.fusion as fusion

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cadlib as L
import struct_fix9 as S9
for _m in (L, S9):
    importlib.reload(_m)
say, healthy = S9.say, S9.healthy

# coef: turn of the rotor in rad per (rad of q1, m of q2, rad of q3, rad of q4), from the reference pose
ROTORS = {
    'q1_input': dict(group='D_01', point=(-59.5, 0.0, 80.33), axis=(0, 1, 0), fixed='D_Q1_Round_base:1', coef=(50.0, 0, 0, 0),
                     parts=['D_Q1_Input_shaft:1', 'D_GT3M_18T_W9_D6:1', 'D_Set_screw_M3x2_5:1', 'D_Set_screw_M3x2_5:2']),
    'q1_intermediate': dict(group='D_01', point=(-100.0, 0.0, 0.0), axis=(0, 1, 0), fixed='D_Q1_Round_base:1', coef=(7.5, 0, 0, 0),
                            parts=['D_Q1_Intermediate_shaft:1', 'D_GT3M_120T_W9_D8:1', 'D_GT3M_16T_W15_D6:1',
                                   'D_Set_screw_M4x8:4', 'D_Set_screw_M4x8:5', 'D_Set_screw_M3x5:8', 'D_Set_screw_M3x5:9']),
    'q3_input': dict(group='D_03', point=(-106.6, 78.29, 0.0), axis=(0, 0, 1), fixed='D_Q2_Saddle:1', coef=(0, 0, 90 / 16 * 120 / 16, 0),
                     parts=['D_Q3_Input_shaft:1', 'D_GT3M_16T_W9_D6:1', 'D_Set_screw_M3x5:1', 'D_Set_screw_M3x5:3']),
    'q3_intermediate': dict(group='D_03', point=(-45.2, 78.29, 0.0), axis=(0, 0, 1), fixed='D_Q2_Saddle:1', coef=(0, 0, 7.5, 0),
                            parts=['D_Q3_Intermediate_shaft:1', 'D_GT3M_90T_W9_D8:1', 'D_GT3M_16T_W15_D6:3',
                                   'D_Set_screw_M4x8:1', 'D_Set_screw_M4x8:3', 'D_Set_screw_M3x5:5', 'D_Set_screw_M3x5:7']),
    'q4_transfer': dict(group='D_03', point=(0.0, 0.0, 0.0), axis=(0, 0, 1), fixed='D_Q2_Saddle:1', coef=(0, 0, 1.0, 2.0),
                        parts=['D_Q4_Transfer_pulley_60T20T:1']),
    # the q4 coupling (a plain cylinder on the q4 input axis) is used as a carrier that turns by 3 x q3;
    # the q4 input parts turn on it by 6 x q4, which gives 3 q3 + 6 q4 in all (see q4_chain)
    'q4_carrier': dict(group='D_03', point=(-22.5, 38.97, 0.0), axis=(0, 0, 1), fixed='D_Q2_Saddle:1', coef=(0, 0, 3.0, 0),
                       parts=['D_XGT2_19C_8x6:7']),
    'q4_input': dict(group='D_03', point=(-22.5, 38.97, 0.0), axis=(0, 0, 1), fixed='D_XGT2_19C_8x6:7', coef=(0, 0, 3.0, 6.0),
                     parts=['D_Q4_Input_shaft_v3:1', 'D_GT3M_20T_W6_D6:1', 'D_Set_screw_M3x3:1', 'D_Set_screw_M3x3:3']),
}
DRIVE = {0: 'q1_Base_yaw', 2: 'q3_Shoulder_pitch', 3: 'q4_Wrist_pitch'}


def natives(key):
    r = ROTORS[key]
    c = L.group(r['group']).component
    by = {o.name: o for o in c.occurrences}
    return [by[n] for n in r['parts']], by[r['fixed']]


def proxies(key):
    r = ROTORS[key]
    by = {o.name: o for o in L.group(r['group']).childOccurrences}
    return [by[n] for n in r['parts']]


def coaxial_face(occ, rotor):
    """A cylindrical face of the part whose axis is the rotor axis (in the sub-assembly frame)."""
    t = occ.transform2
    px, py, pz = t.translation.x, t.translation.y, t.translation.z      # the first part has its origin on the axis
    ax = core.Vector3D.create(*rotor['axis'])
    for f in occ.component.bRepBodies.item(0).faces:
        g = f.geometry
        if g.objectType != core.Cylinder.classType():
            continue
        o, a = g.origin.copy(), g.axis.copy()
        o.transformBy(t)
        a.transformBy(t)
        if abs(abs(a.dotProduct(ax)) - 1) > 1e-6:
            continue
        v = core.Vector3D.create(o.x - px, o.y - py, o.z - pz)
        if v.crossProduct(ax).length < 1e-5:
            return f
    raise RuntimeError('no face on the rotor axis: ' + occ.name)


def box(o):
    b = o.bRepBodies.item(0).boundingBox
    return [b.minPoint.x, b.minPoint.y, b.minPoint.z, b.maxPoint.x, b.maxPoint.y, b.maxPoint.z]


def build(keys, joint=True):
    """Release, put back, capture, joint, rigid group, link - in that order (see struct_fix9.screw_joint:
    a released part jumps to an old stored position, and an as-built joint takes what it finds as zero)."""
    d = S9.guard()
    root = d.rootComponent
    for key in keys:
        r = ROTORS[key]
        g = L.group(r['group'])
        c = g.component
        parts, fixed = natives(key)
        seen = proxies(key)
        homes = []
        for n, p in zip(parts, seen):
            h = n.transform2.copy()
            h.transformBy(g.transform2)
            homes.append(h)
        box0 = [box(p) for p in seen]
        for n in parts:
            n.isGroundToParent = False
        jumped = sum(max(abs(a - b) for a, b in zip(box(p), b0)) > 1e-6 for p, b0 in zip(seen, box0))
        assert root.transformOccurrences(seen, homes, True)
        if d.snapshots.hasPendingSnapshot:
            d.snapshots.add()
        err = max(max(abs(a - b) for a, b in zip(box(p), b0)) for p, b0 in zip(seen, box0))
        assert err < 1e-6, (key, err)
        if not joint:
            say(f'{key}: {len(parts)} parts released ({jumped} had jumped, put back), no joint here')
            continue
        face = coaxial_face(parts[0], r)
        geo = fusion.JointGeometry.createByNonPlanarFace(face.createForAssemblyContext(parts[0]), fusion.JointKeyPointTypes.StartKeyPoint)
        inp = c.asBuiltJoints.createInput(parts[0], fixed, geo)
        inp.setAsRevoluteJointMotion(fusion.JointDirections.ZAxisJointDirection)
        j = c.asBuiltJoints.add(inp)
        j.name = 'spin_' + key
        if len(parts) > 1:
            coll = core.ObjectCollection.create()
            for n in parts:
                coll.add(n)
            rg = c.rigidGroups.add(coll, False)
            rg.name = 'rotor_' + key
        # the captured position must hold when the joint is driven (on the very first run it did not:
        # the parts went back to the old stored places; the same steps worked on every later run)
        j.jointMotion.rotationValue = math.radians(30)
        t1, d1 = state(key)
        j.jointMotion.rotationValue = 0.0
        t0, d0 = state(key)
        assert abs(abs(t1) - 30) < 1e-6 and d1 < 1e-4 and d0 < 1e-4, (key, t1, d1, t0, d0)
        say(f'{key}: {len(parts)} parts released ({jumped} had jumped, put back), joint {j.name}; '
            f'driven to 30 deg: turn {t1:+.2f}, deviation {d1:.5f} mm; back: {t0:+.2f}, {d0:.5f} mm')
    d.computeAll()
    bad = healthy(d)
    say(f'build {keys}: timeline {d.timeline.count}, not healthy {bad}')
    assert not bad, bad


def link(key, drive_index, ratio, reverse=False, name=None):
    """Motion link: the robot joint turning by 10 deg turns the rotor joint by 10 * ratio deg."""
    d = S9.guard()
    root = d.rootComponent
    r = ROTORS[key]
    g = L.group(r['group'])
    j = g.component.asBuiltJoints.itemByName('spin_' + key)
    inp = root.motionLinks.createInput(root.asBuiltJoints.itemByName(DRIVE[drive_index]), j.createForAssemblyContext(g))
    inp.motionOne = fusion.JointMotionTypes.RevoluteJointRotateMotionType
    inp.valueOne = core.ValueInput.createByString('10 deg')
    inp.motionTwo = fusion.JointMotionTypes.RevoluteJointRotateMotionType
    inp.valueTwo = core.ValueInput.createByString(f'{10 * ratio} deg')
    inp.isReversed = reverse
    ml = root.motionLinks.add(inp)
    ml.name = name or f'link_{key}'
    return ml


def state(key):
    """(turn of the rotor in degrees, largest deviation of any of its parts from a pure rotation about
    the rotor axis by that turn, in mm at 50 mm from the part origin). Read from the positions seen at the root."""
    r = ROTORS[key]
    g = L.group(r['group'])
    gi = g.transform2.copy()
    gi.invert()
    parts, _ = natives(key)
    seen = proxies(key)
    ax = core.Vector3D.create(*r['axis'])
    pt = parts[0].transform2.translation.asPoint()                      # exact; r['point'] is rounded to 0.01 mm
    u = core.Vector3D.create(0, 0, 1) if abs(ax.z) < 0.5 else core.Vector3D.create(1, 0, 0)
    turn, worst = None, 0.0
    for n, p in zip(parts, seen):
        now = p.transform2.copy()
        now.transformBy(gi)                                   # in the sub-assembly frame
        rel = n.transform2.copy()                             # home
        rel.invert()
        rel.transformBy(now)                                  # now * home^-1  (applied to points in the group frame)
        v = u.copy()
        v.transformBy(rel)
        a = math.atan2(u.crossProduct(v).dotProduct(ax), u.dotProduct(v))
        if turn is None:
            turn = a
        ideal = core.Matrix3D.create()
        ideal.setToRotation(turn, ax, pt)
        dev = max(abs(x - y) for x, y in zip(rel.asArray(), ideal.asArray()))
        worst = max(worst, dev)
    return math.degrees(turn), worst * 10


def expected(key, q):
    """q = (q1 deg, q2 mm, q3 deg, q4 deg) relative to the reference pose -> turn in degrees, wrapped."""
    c = ROTORS[key]['coef']
    a = c[0] * q[0] + c[2] * q[2] + c[3] * q[3]
    return (a + 180) % 360 - 180


def test(keys, moves):
    """moves: list of (q1, q3, q4) in degrees given to the robot joints."""
    d = S9.guard()
    root = d.rootComponent
    jq = [root.asBuiltJoints.itemByName(DRIVE[i]) for i in (0, 2, 3)]
    ok = True
    for mv in list(moves) + [(0, 0, 0)]:
        for j, v in zip(jq, mv):
            j.jointMotion.rotationValue = math.radians(v)
        hits = S9.interference_pairs(d)
        line = f'joints q1 {mv[0]:g}, q3 {mv[1]:g}, q4 {mv[2]:g} deg:'
        for key in keys:
            turn, dev = state(key)
            want = expected(key, (mv[0], 0, mv[1], mv[2]))
            good = abs((turn - want + 180) % 360 - 180) < 0.01 and dev < 1e-3
            ok = ok and good
            line += f' {key} {turn:+.2f} (want {want:+.2f}, dev {dev:.5f}){"" if good else " WRONG"};'
        say(line + f' interference {hits}')
        ok = ok and not hits
    say(f'test {keys}: {"all right" if ok else "NOT RIGHT"}')
    return ok


def link_checked(key, drive_index, ratio, name=None):
    """link(), then the robot joint is moved by 1 degree: if the rotor turned the other way, the link is reversed."""
    d = S9.guard()
    ml = link(key, drive_index, ratio, name=name)
    j = d.rootComponent.asBuiltJoints.itemByName(DRIVE[drive_index])
    j.jointMotion.rotationValue = math.radians(1.0)
    turn, dev = state(key)
    if turn * ratio < 0:
        j.jointMotion.rotationValue = 0.0
        ml.isReversed = not ml.isReversed
        j.jointMotion.rotationValue = math.radians(1.0)
        turn, dev = state(key)
    j.jointMotion.rotationValue = 0.0
    say(f'{ml.name}: 1 deg of {DRIVE[drive_index]} -> {turn:+.4f} deg of {key} (ratio {ratio:g}), reversed {ml.isReversed}, deviation {dev:.5f} mm')
    assert abs(turn - ratio) < 1e-4 and dev < 1e-4, (key, turn, ratio, dev)
    return ml


def q4_chain():
    """q4 motor pulley: 3 x q3 + 6 x q4; 60/20 transfer pulley on the q3 axis: q3 + 2 x q4 (both relative to
    the slider). A motion link ties two joints only, so each of the two sums is built from two joints in series:
      transfer pulley - ARM (joint at the root, spin_q4_transfer_on_arm), linked to q4 x 2:
          the pulley goes round with the arm (q3) and turns on it by 2 q4
      coupling - saddle (spin_q4_carrier), linked to q3 x 3; q4 input shaft - coupling (spin_q4_input), linked to q4 x 6
    (first tried: two joints on the transfer pulley and a link from one of them to the motor pulley. With both
    links present the q4 joint no longer turned the pulley.)"""
    d = S9.guard()
    root = d.rootComponent
    build(['q4_transfer'], joint=False)
    build(['q4_carrier', 'q4_input'])
    g3, g4 = L.group('D_03'), L.group('D_04')
    c3 = g3.component
    pulley_n = next(o for o in c3.occurrences if o.name == 'D_Q4_Transfer_pulley_60T20T:1')
    pulley = pulley_n.createForAssemblyContext(g3)
    arm = next(o for o in g4.childOccurrences if o.name == 'D_Q3_Shaft_15:1')
    face = coaxial_face(pulley_n, ROTORS['q4_transfer']).createForAssemblyContext(pulley)
    geo = fusion.JointGeometry.createByNonPlanarFace(face, fusion.JointKeyPointTypes.StartKeyPoint)
    inp = root.asBuiltJoints.createInput(pulley, arm, geo)
    inp.setAsRevoluteJointMotion(fusion.JointDirections.ZAxisJointDirection)
    ja = root.asBuiltJoints.add(inp)
    ja.name = 'spin_q4_transfer_on_arm'
    q3 = root.asBuiltJoints.itemByName(DRIVE[2])
    q4 = root.asBuiltJoints.itemByName(DRIVE[3])
    keys = ('q4_transfer', 'q4_carrier', 'q4_input')

    def probe(tag):
        out = []
        for a3, a4 in ((1.0, 0.0), (0.0, 1.0), (0.0, 0.0)):
            q3.jointMotion.rotationValue = math.radians(a3)
            q4.jointMotion.rotationValue = math.radians(a4)
            out.append([state(k) for k in keys])
        say(tag + ': ' + '; '.join(f'{n} -> ' + ', '.join(f'{k[3:]} {o[i][0]:+.3f}' for i, k in enumerate(keys))
                                   for n, o in zip(('q3 1 deg', 'q4 1 deg', 'back'), out))
            + f'; largest deviation {max(x[1] for o in out for x in o):.5f} mm')
        return out

    def add(j1, j2, ratio, name):
        inp = root.motionLinks.createInput(j1, j2)
        inp.motionOne = fusion.JointMotionTypes.RevoluteJointRotateMotionType
        inp.valueOne = core.ValueInput.createByString('10 deg')
        inp.motionTwo = fusion.JointMotionTypes.RevoluteJointRotateMotionType
        inp.valueTwo = core.ValueInput.createByString(f'{10 * ratio} deg')
        ml = root.motionLinks.add(inp)
        ml.name = name
        return ml
    probe('joints made, no links yet')
    jc = c3.asBuiltJoints.itemByName('spin_q4_carrier').createForAssemblyContext(g3)
    ji = c3.asBuiltJoints.itemByName('spin_q4_input').createForAssemblyContext(g3)
    for ml_args, row, col, want in (((q4, ja, 2.0, 'link_q4_transfer'), 1, 0, 2.0),
                                    ((q3, jc, 3.0, 'link_q4_carrier'), 0, 1, 3.0),
                                    ((q4, ji, 6.0, 'link_q4_input'), 1, 2, 6.0)):
        ml = add(*ml_args)
        out = probe(ml.name + ' made')
        if out[row][col][0] * want < -1e-6:
            ml.isReversed = not ml.isReversed
            out = probe(ml.name + ' reversed')
        assert abs(out[row][col][0] - want) < 1e-4, (ml.name, out[row][col], want)
    want = ((1.0, 3.0, 3.0), (2.0, 0.0, 6.0), (0.0, 0.0, 0.0))
    for o, w in zip(out, want):
        assert all(abs(o[i][0] - w[i]) < 1e-4 and o[i][1] < 1e-4 for i in range(3)), (o, w)
    d.computeAll()
    bad = healthy(d)
    say(f'q4 chain: timeline {d.timeline.count}, not healthy {bad}')
    assert not bad, bad


def store():
    """Table of the parts that are free inside their sub-assembly, for the scripts that place the robot
    directly: attribute RPRR / SpinningParts of the root component (travels with the .f3d file)."""
    d = S9.guard()
    rows = []
    for key, r in ROTORS.items():
        parts, _ = natives(key)
        t = parts[0].transform2.translation
        rows.append(dict(key=key, group=r['group'], point_cm=[t.x, t.y, t.z], axis=list(r['axis']), coef=list(r['coef']), parts=r['parts']))
    c2 = L.group('D_02').component
    screw = next(o for o in c2.occurrences if o.component.name == 'D_Q2_Ballscrew_12x10')
    t = screw.transform2.translation
    rows.append(dict(key='q2_screw', group='D_02', point_cm=[t.x, t.y, t.z], axis=[1, 0, 0],
                     coef=[0, -2 * math.pi / (S9.LEAD / 1000.0), 0, 0], parts=[screw.name]))
    data = dict(note='turn of each rotor in rad = sum(coef[i] * (q[i] - q0[i])), q = (q1 rad, q2 m, q3 rad, q4 rad); '
                     'point and axis in the frame of the sub-assembly', q0=[0, 0.080, 0, 0], rotors=rows)
    d.rootComponent.attributes.add('RPRR', 'SpinningParts', json.dumps(data))
    say(f'stored RPRR/SpinningParts: {len(rows)} rotors, {sum(len(r["parts"]) for r in rows)} parts')


def placed_check(poses):
    """The robot is placed the way the checks and the playback do it (joint_limits_fusion.Robot);
    every rotor must then stand where the gear ratios say. poses: (q1 deg, q2 mm, q3 deg, q4 deg)."""
    import runpy
    d = S9.guard()
    R = runpy.run_path(str(HERE.parent / 'joint_limits_fusion.py'))['Robot'](d)
    ok = True
    try:
        for q in poses:
            hits = R.collisions(q)
            line = f'placed {q}:'
            for key in ROTORS:
                turn, dev = state(key)
                want = expected(key, (q[0], 0, q[2], q[3]))
                good = abs((turn - want + 180) % 360 - 180) < 0.01 and dev < 1e-3
                ok = ok and good
                line += f' {key[:2]}{key[3:6]} {turn:+.1f}{"" if good else " WRONG (want %+.1f, dev %.4f)" % (want, dev)};'
            sc_turn, off, tilt = S9.screw_state()
            ok = ok and off < 1e-4 and not hits
            say(line + f' screw {sc_turn:+.1f}, off axis {off:.5f}; collisions {hits}')
    finally:
        R.restore()
    for key in ROTORS:
        turn, dev = state(key)
        ok = ok and abs(turn) < 1e-6 and dev < 1e-4
    say(f'placed_check: {"all right" if ok else "NOT RIGHT"}; pending position {d.snapshots.hasPendingSnapshot}')
    return ok


def final():
    """Joint-driven test of all rotors, the playback script's own pose function, then the general check."""
    import runpy
    d = S9.guard()
    root = d.rootComponent
    ok = test(list(ROTORS), [(3, 0, 0), (-7.7, 5, 0), (0, -10.3, 20), (15, 12, -33)])
    assert ok
    pw = runpy.run_path(str(HERE.parent / 'PlayWriting' / 'PlayWriting.py'))
    moving = sorted((o for o in root.occurrences if o.component.name[:4] in ('D_02', 'D_03', 'D_04', 'D_05')), key=lambda o: o.component.name)
    spin = pw['spinning_parts'](root)
    home = [o.transform2.copy() for o in moving]
    for q in ((0.21, 0.0937, -0.31, 0.52), (-0.4, 0.031, 0.6, -1.1)):
        pw['drive'](root, moving, q, spin)
        deg = (math.degrees(q[0]), 0, math.degrees(q[2]), math.degrees(q[3]))
        line, good = f'PlayWriting.drive {q}: tip error {pw["tip_error_mm"](moving[-1], q):.5f} mm;', True
        for key in ROTORS:
            turn, dev = state(key)
            want = expected(key, deg)
            g = abs((turn - want + 180) % 360 - 180) < 0.01 and dev < 1e-3
            good = good and g
            line += f' {key[:2]}{key[3:6]} {turn:+.1f}{"" if g else " WRONG"};'
        sc_turn, off, tilt = S9.screw_state()
        want_sc = (-36000.0 * (q[1] - 0.080) + 180) % 360 - 180
        good = good and off < 1e-4 and abs((sc_turn - want_sc + 180) % 360 - 180) < 0.01
        say(line + f' screw {sc_turn:+.1f} (want {want_sc:+.1f}), off axis {off:.5f} -> {"right" if good else "NOT RIGHT"}')
        assert good
    pw['drive'](root, moving, (0.0, 0.080, 0.0, 0.0), spin)
    for o, h in zip(moving, home):
        assert max(abs(a - b) for a, b in zip(o.transform2.asArray(), h.asArray())) < 1e-9
    say(f'back at the reference pose; pending position {d.snapshots.hasPendingSnapshot}')
    S9.check('struct_fix11')
    say('FINAL DONE')
