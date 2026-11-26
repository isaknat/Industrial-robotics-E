"""Fifth round (after CAD edition 3, 2026-11-26): changes decided after looking at the model.

Run inside Fusion on the working document E_Writing_q3_50W_build:
  cheek()    1. the separate bracket of the q3 stage-2 idler goes: the drive cheek is made larger at the
                front and the idler stands on an axle that slides in a slot of the cheek itself.
             4. the cheek stood on the saddle on a 40 mm base; its slanted rear edge now starts 28 mm
                further back, so the base is 68 mm long.
  saddle()   3. the bridge that carries the ball-screw nut hung on a 4 mm wall standing on the last 10 mm
                of the lower plate: the plate now runs under the whole wall, the wall is 10 mm thick and
                a gusset supports the bridge.
Coordinates in mm in the slider frame (D_03: origin on the q3 axis, x along the rail, y up, z sideways).
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
import mechanism as M
import mounts as N
import belt_path as B
for _m in (L, M, N, B):
    importlib.reload(_m)

DOC = 'E_Writing_q3_50W_build'
IN = fusion.PointContainment.PointInsidePointContainment


def guard():
    app = core.Application.get()
    if app.activeDocument.name != DOC:
        raise RuntimeError('Expected the working document, got ' + app.activeDocument.name)
    return L.design()


def healthy(d):
    return [(i, d.timeline.item(i).name) for i in range(d.timeline.count)
            if d.timeline.item(i).healthState != fusion.FeatureHealthStates.HealthyFeatureHealthState]


def volume(c):
    return sum(b.volume for b in c.bRepBodies) * 1000


def plain_cut(c, sk, start, depth, label):
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


def cheek():
    d = guard()
    ck = L.comp('D_Q3_Drive_cheek')
    lay = json.loads((HERE / 'q3_50w_layout.json').read_text(encoding='utf8'))
    x2, y2 = lay['idler2']
    # --- take away what belonged to the bracket
    ff = ck.features.filletFeatures
    for name in [f.name for f in ff][::-1]:
        ff.itemByName(name).deleteMe()
    ef = ck.features.extrudeFeatures
    for name in ('Second_lock_ear', 'Second_lock_M4_tap', 'Second_stage_lock_M4_tap'):
        ef.itemByName(name).deleteMe()
    group = L.group('D_03').component
    gone = [o for o in group.occurrences if o.component.name in
            ('D_Q3_Idler_bracket', 'D_Idler_axle_6', 'D_Q3_Tension_lock_spacer', 'D_Q3_Tension_lock_M4')]
    assert len(gone) == 6, [o.name for o in gone]
    for o in gone:
        o.deleteMe()
    # --- larger cheek: longer base at the back, a lobe at the front for the idler
    v0 = volume(ck)
    sk = M.sketch(ck, 'Rear_base_extension')
    L.poly(sk, [(-22, -17), (-50, -17), (-127, 58)])
    M.solid(ck, sk, 0, 9, 'Rear_base_extension', M.JOIN, True)
    sk = M.sketch(ck, 'Idler_lobe')
    L.poly(sk, [(18, 17), (18, 64), (-2, 98), (-28, 98)])
    M.solid(ck, sk, 0, 9, 'Idler_lobe', M.JOIN, True)
    # --- slot for the idler axle, along the direction in which the idler pushes on the belt
    c2 = B.pitch_circles([(*B.Q3, 120), (*B.MID, 16)], (x2, y2, 8.0))
    arcs = [s for s in B.loop(c2) if s[0] == 'arc']
    a_mid = arcs[1][3] + arcs[1][4] / 2                       # middle of the belt's wrap on the idler
    ux, uy = math.cos(a_mid), math.sin(a_mid)                 # from the idler centre towards the belt
    sk = M.sketch(ck, 'Second_stage_idler_slot')
    r, half = 2.7, 3.0
    p = lambda s, t: M.P(x2 + s * ux - t * uy, y2 + s * uy + t * ux)
    sk.sketchCurves.sketchLines.addByTwoPoints(p(-half, r), p(half, r))
    sk.sketchCurves.sketchLines.addByTwoPoints(p(-half, -r), p(half, -r))
    sk.sketchCurves.sketchArcs.addByThreePoints(p(half, r), p(half + r, 0), p(half, -r))
    sk.sketchCurves.sketchArcs.addByThreePoints(p(-half, -r), p(-half - r, 0), p(-half, r))
    before = volume(ck)
    plain_cut(ck, sk, 0, 9, 'Second_stage_idler_slot')
    removed = before - volume(ck)
    ideal = (4 * half * r + math.pi * r * r) * 9
    assert ck.bRepBodies.count == 1 and abs(removed / ideal - 1) < 0.02, (ck.bRepBodies.count, removed, ideal)
    body = ck.bRepBodies.item(0)
    for rad in (6.0, 9.0):                                    # the whole seat of the axle flange must be on the plate
        on = sum(body.pointContainment(core.Point3D.create((x2 + rad * math.cos(a)) / 10, (y2 + rad * math.sin(a)) / 10, 0.45)) == IN
                 for a in [i * math.pi / 12 for i in range(24)])
        assert on == 24, (rad, on)
    # --- axle: journal for the two bearings of the roller, a 10 mm post, a flange on the cheek, M5 through the slot
    ax = M.part('D_03', 'D_Q3_Second_stage_idler_axle', (x2, y2, -72), M.SHAFT)        # own z = z + 72
    M.disk(ax, 'Idler_journal', 0, 16, 3.0)                    # z -72..-56
    M.disk(ax, 'Post', 16, 24, 5.0)                            # z -56..-32
    M.disk(ax, 'Seat_flange', 40, 2, 8.0)                      # z -32..-30, on the outer face of the cheek
    M.disk(ax, 'M5_shank', 42, 14, 2.5)                        # z -30..-16: 9 mm in the slot, 5 mm for the nut
    M.disk(ax, 'Ring_groove', 0.2, 0.7, 3.6, 2.85, op=M.CUT)   # z -71.8..-71.1, outside the outer bearing
    assert ax.bRepBodies.count == 1
    M.place('D_03', L.comp('D_Retaining_ring_6'), (x2, y2, -71.8))
    nut = M.part('D_03', 'D_M5_nut', (x2, y2, -21), M.STEEL)
    M.disk(nut, 'Nut_envelope', 0, 4.0, 4.6, 2.5)              # z -21..-17, on the inner face of the cheek
    d.computeAll()
    print('cheek volume', round(v0), '->', round(volume(ck)), 'mm3; slot direction', (round(ux, 3), round(uy, 3)), 'not healthy', healthy(d))
    # --- corners again (the stop ears stay sharp, see struct_fix4)
    import struct_fix4 as F4
    importlib.reload(F4)
    F4.round_corners(only=('D_Q3_Drive_cheek',))


def saddle():
    """Saddle own frame: x = slider x, y = -(slider z), z = slider y + 31."""
    d = guard()
    sd = L.comp('D_Q2_Saddle')
    v0 = volume(sd)
    # lower plate 12 mm longer at the back, so that it runs under the whole bridge wall (x -62..-40)
    M.rect(sd, 'Rear_plate_extension', 0, 14, (-62, -31, -50, 51), M.JOIN)          # slider z -51..31
    # the wall between the lower plate and the nut bridge: 4 mm -> 10 mm (towards the arm, slider z 25..31)
    M.rect(sd, 'Bridge_wall_thickening', 14, 17, (-62, -31, -40, -25), M.JOIN)
    # gusset under the bridge: a triangle in the (z, y) plane, slider z 35..62, y -17..0, through x -62..-40
    sk = sd.sketches.add(sd.yZConstructionPlane)
    sk.name = 'Bridge_gusset_profile'
    pts = [sk.modelToSketchSpace(core.Point3D.create(0, yo / 10, zo / 10)) for yo, zo in ((-35, 14), (-35, 31), (-62, 31))]
    ln = sk.sketchCurves.sketchLines
    for a, b in zip(pts, pts[1:] + pts[:1]):
        ln.addByTwoPoints(core.Point3D.create(a.x, a.y, 0), core.Point3D.create(b.x, b.y, 0))
    ef = sd.features.extrudeFeatures
    made = False
    for start in ('-62 mm', '40 mm'):                          # the sketch normal may point along +x or -x
        inp = ef.createInput(sk.profiles.item(0), M.JOIN)
        inp.setDistanceExtent(False, core.ValueInput.createByString('22 mm'))
        inp.startExtent = fusion.OffsetStartDefinition.create(core.ValueInput.createByString(start))
        f = ef.add(inp)
        f.name = 'Bridge_gusset'
        inside = sd.bRepBodies.item(0).pointContainment(core.Point3D.create(-5.1, -4.0, 2.6)) == IN     # x -51, z 40, y -5
        if inside and sd.bRepBodies.count == 1:
            made = True
            break
        f.deleteMe()
    assert made, 'gusset not where it should be'
    sk.isVisible = False
    d.computeAll()
    print('saddle volume', round(v0), '->', round(volume(sd)), 'mm3; bodies', sd.bRepBodies.count, 'not healthy', healthy(d))
    assert not healthy(d)
