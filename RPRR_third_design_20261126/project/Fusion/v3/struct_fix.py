"""Fixes from the structure review of 2026-11-26 (docs/02_mechanical_design.md). after CAD edition 3.

Run inside Fusion on the working document E_Writing_q3_50W_build, one stage at a time:
  belts()         q3 and q4 belts redrawn with teeth; all six belts black; new pulleys coloured like the old ones
  idler_bearings()  two small ball bearings in each of the five idler rollers
  stepped_shafts()  q1 and q3 intermediate shafts stepped to 6 mm where the 16-tooth pulley sits
  hubs()          hubs and set screws on the pulleys, small gaps between turning and fixed faces
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
RP = lambda z: z * 3 / (2 * math.pi)


def guard():
    app = core.Application.get()
    if app.activeDocument.name != DOC:
        raise RuntimeError('Expected the working document, got ' + app.activeDocument.name)
    return L.design()


def xyz(o):
    t = o.transform2.translation
    return t.x * 10, t.y * 10, t.z * 10


def draw_loop(sk, circles):
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    for seg in B.loop(circles):
        if seg[0] == 'line':
            lines.addByTwoPoints(M.P(*seg[1]), M.P(*seg[2]))
        else:
            (cx, cy), r, a0, sweep = seg[1:]
            pt = lambda a: M.P(cx + r * math.cos(a), cy + r * math.sin(a))
            arcs.addByThreePoints(pt(a0), pt(a0 + sweep / 2), pt(a0 + sweep))


def toothed_belt(group, name, circles, teeth, z, width, label):
    """Belt as one body: the band between the tooth-root line and the back, plus the tooth bumps
    on the free runs (belt_path.tooth_centres). circles = pitch loop in the group frame."""
    c = M.part(group, name, (0, 0, z), M.BELT)
    sk = M.sketch(c, label)
    draw_loop(sk, B.offset(circles, -B.PITCH_TO_ROOT))
    draw_loop(sk, B.offset(circles, B.PITCH_TO_BACK))
    bumps = B.tooth_centres(circles, teeth)
    for x, y in bumps:
        L.circle(sk, x, y, B.TOOTH_R)
    profs = sorted(sk.profiles, key=lambda p: p.areaProperties().area)
    inside = profs.pop()                      # the open space inside the loop is the largest region
    coll = core.ObjectCollection.create()
    for p in profs:
        coll.add(p)
    L.extrude(c, coll, width, 0, M.NEW, label)
    sk.isVisible = False
    assert c.bRepBodies.count == 1, (name, c.bRepBodies.count)
    area = c.bRepBodies.item(0).volume * 1000 / width
    band = 1.48 * (B.length(circles) + 2 * math.pi * 0.36)
    cap = B.TOOTH_R ** 2 * math.acos(B.TOOTH_SINK / B.TOOTH_R) + B.TOOTH_SINK * math.sqrt(B.TOOTH_R ** 2 - B.TOOTH_SINK ** 2)
    print(f'{name}: {teeth} teeth, pitch {B.length(circles) / teeth:.4f} mm, bumps drawn {len(bumps)}; '
          f'section {area:.1f} mm2, expected about {band + len(bumps) * cap:.1f}')
    return c


def belts():
    d = guard()
    # --- q3: the two belts made in q3_50w.py, now with teeth
    lay = __import__('json').loads((HERE / 'q3_50w_layout.json').read_text(encoding='utf8'))
    c1 = B.pitch_circles([(*B.INPUT, 16), (*B.MID, 90)], (*lay['idler1'], 8.0))
    c2 = B.pitch_circles([(*B.Q3, 120), (*B.MID, 16)], (*lay['idler2'], 8.0))
    assert abs(B.length(c1) - 303) < 1e-6 and abs(B.length(c2) - 414) < 1e-6, (B.length(c1), B.length(c2))
    # --- q4 stage 1: motor pulley (20) to the transfer pulley (60), 73 teeth, no idler (the motor unit slides)
    motor = xyz(N.instance('D_03', 'D_GT3M_20T_W6_D6'))
    c3 = [(motor[0], motor[1], RP(20), 1), (0.0, 0.0, RP(60), 1)]
    # --- q4 long belt: transfer pulley (20) on the q3 axis to the wrist pulley (40), 230 teeth; its idler only touches
    c4 = [(0.0, 0.0, RP(20), 1), (300.0, 0.0, RP(40), 1)]
    for g, n in (('D_03', 'D_Belt_Q3_stage1'), ('D_03', 'D_Belt_Q3_stage2'), ('D_03', 'D_Belt_Q4_stage1_W6'), ('D_04', 'D_Belt_Q4_long_690')):
        N.instance(g, n).deleteMe()
    gone = [c.name for c in d.allComponents if c.name in ('D_Belt_Q3_stage1', 'D_Belt_Q3_stage2', 'D_Belt_Q4_stage1_W6', 'D_Belt_Q4_long_690')]
    assert not gone, gone
    toothed_belt('D_03', 'D_Belt_Q3_stage1', c1, 101, -102, 9, '3MGT_101T_belt')
    toothed_belt('D_03', 'D_Belt_Q3_stage2', c2, 138, -71, 15, '3MGT_138T_belt')
    toothed_belt('D_03', 'D_Belt_Q4_stage1_W6', c3, 73, 48, 6, '3MGT_73T_belt')
    toothed_belt('D_04', 'D_Belt_Q4_long_690', c4, 230, 36, 9, '3MGT_230T_belt')
    # --- one look for all belts (black rubber) and for all pulleys (the light blue of the older pulleys)
    black = next(a for a in d.appearances if a.name == '\u6a61\u80f6 - \u8f6f')   # Chinese-language Fusion names: Rubber - Soft; rubber
    blue = next(a for a in d.appearances if a.name == 'SW RGB 221 232 255')
    for c in d.allComponents:
        if c.name.startswith('D_Belt_Q'):
            for b in c.bRepBodies:
                b.appearance = black
        if c.name.startswith('D_GT3M_') or c.name == 'D_Q4_Transfer_pulley_60T20T':
            for b in c.bRepBodies:
                b.appearance = blue
    d.computeAll()
    for c in d.allComponents:
        if c.name.startswith('D_Belt_Q') or c.name.startswith('D_GT3M_') or c.name == 'D_Q4_Transfer_pulley_60T20T':
            print('  ', c.name, c.bRepBodies.item(0).appearance.name)
    print('timeline', d.timeline.count, 'problems', L.timeline_problems())


FLOOR = ((1, 0, 0), (0, 0, -1))        # parts of the base whose own z points up (group y)


def idler_bearings():
    """Each idler roller turns on a fixed axle, so it gets two MR126 ball bearings (6 x 12 x 4 mm),
    one at each end of its bore. Rollers stay 16 mm outside, so no belt path changes."""
    d = guard()
    rollers = [  # group, roller part, width, axes of the roller in its group
        ('D_03', 'D_Q3_First_stage_idler', 9, M.identity_axes),
        ('D_03', 'D_Belt_idler_15', 15, M.identity_axes),
        ('D_04', 'D_Q4_Long_idler', 9, M.identity_axes),
        ('D_01', 'D_Q1_Stage1_idler', 9, FLOOR),
        ('D_01', 'D_Belt_idler_15', 15, FLOOR)]
    bored = set()
    bearing = None
    count = 0
    for g, name, width, axes in rollers:
        comp = L.comp(name)
        if name not in bored:
            before = comp.bRepBodies.item(0).volume
            M.disk(comp, 'Bearing_bore_D12', 0, width, 6.0, op=M.CUT)
            removed = (before - comp.bRepBodies.item(0).volume) * 1000
            ideal = math.pi * (6.0 ** 2 - 3.0 ** 2) * width
            assert abs(removed / ideal - 1) < 0.01, (name, removed, ideal)
            bored.add(name)
        x, y, z = xyz(N.instance(g, name))
        up = (0, 0, 1) if axes == M.identity_axes else (0, 1, 0)
        for off in (0, width - 4):
            pos = (x + up[0] * off, y + up[1] * off, z + up[2] * off)
            if bearing is None:
                bearing = M.part(g, 'D_Bearing_MR126', pos, M.STEEL, axes)
                M.disk(bearing, 'Outer_race', 0, 4, 6.0, 4.9)
                M.disk(bearing, 'Inner_race', 0, 4, 4.1, 3.0, op=M.NEW)
            else:
                M.place(g, bearing, pos, axes)
            count += 1
    d.computeAll()
    print('bearings placed', count, 'bearing bodies', bearing.bRepBodies.count)
    print('timeline', d.timeline.count, 'problems', L.timeline_problems())


def set_circle(comp_name, sketch_name, centre, old_r, new_r):
    """Change the radius of the sketch circle at centre (mm, sketch coordinates); through its
    driving dimension if it has one, directly otherwise."""
    sk = L.comp(comp_name).sketches.itemByName(sketch_name)
    hit = [c for c in sk.sketchCurves.sketchCircles
           if abs(c.centerSketchPoint.geometry.x * 10 - centre[0]) < 0.01 and abs(c.centerSketchPoint.geometry.y * 10 - centre[1]) < 0.01
           and abs(c.radius * 10 - old_r) < 0.01]
    assert len(hit) == 1, (comp_name, sketch_name, len(hit))
    circle = hit[0]
    for dim in sk.sketchDimensions:
        ent = getattr(dim, 'entity', None)
        if ent is not None and ent == circle:
            is_dia = dim.objectType.endswith('SketchDiameterDimension')
            dim.parameter.expression = f'{2 * new_r if is_dia else new_r} mm'
            break
    else:
        circle.radius = new_r / 10
    assert abs(circle.radius * 10 - new_r) < 1e-6, (comp_name, sketch_name, circle.radius * 10)


def stepped_shafts():
    """A stock 16-tooth pulley can be bored to 6.35 mm at most (Gates manual p.33-34), so the two
    intermediate shafts are 8 mm only where the large pulley and the first bearing sit and 6 mm
    from there on; the second bearing of each shaft becomes a 626."""
    d = guard()
    # the 16-tooth, 15 mm wide pulley (used on both intermediate shafts): bore 8 -> 6
    for skn in ('Timing_rim_envelope_profile', 'Retaining_flange_A_profile', 'Retaining_flange_B_profile'):
        set_circle('D_GT3M_16T_W15_D8', skn, (0, 0), 4.0, 3.0)
    L.comp('D_GT3M_16T_W15_D8').name = 'D_GT3M_16T_W15_D6'
    # q3: shaft from z -103 to 2; 8 mm up to z -81 (90-tooth pulley, bearing in the outboard bridge), then 6 mm
    M.disk(L.comp('D_Q3_Intermediate_shaft'), 'Stepped_journal_D6', 22, 83, 4.6, 3.0, op=M.CUT)
    old = next(o for o in L.group('D_03').component.occurrences
               if o.component.name == 'D_Bearing_608' and abs(o.transform2.translation.z * 10 + 29) < 0.01)
    assert abs(old.transform2.translation.x * 10 + 45.2) < 0.01
    old.deleteMe()
    M.place('D_03', L.comp('D_Bearing_626'), (*B.MID, -28))
    set_circle('D_Q3_Drive_cheek', 'Belt_shaft_bearing_plate_profile', B.MID, 11.0, 9.5)
    set_circle('D_Q3_Idler_bracket', 'Idler_seat_profile', B.MID, 4.25, 3.25)
    # q1: shaft from y 8 to 83; 8 mm up to y 36 (bearing in the base, 120-tooth pulley), then 6 mm
    M.disk(L.comp('D_Q1_Intermediate_shaft'), 'Stepped_journal_D6', 28, 47, 4.6, 3.0, op=M.CUT)
    old = next(o for o in L.group('D_01').component.occurrences
               if o.component.name == 'D_Bearing_608' and abs(o.transform2.translation.y * 10 - 75) < 0.01)
    old.deleteMe()
    M.place('D_01', L.comp('D_Bearing_626'), (-100, 75, 0), FLOOR)
    set_circle('D_Q1_Round_lid', 'Intermediate_608_seat_profile', (-100, 0), 11.0, 9.5)
    d.computeAll()
    for n in ('D_GT3M_16T_W15_D6', 'D_Q3_Intermediate_shaft', 'D_Q1_Intermediate_shaft', 'D_Q3_Drive_cheek', 'D_Q3_Idler_bracket', 'D_Q1_Round_lid'):
        c = L.comp(n)
        print(n, 'bodies', c.bRepBodies.count, 'volume mm3', round(sum(b.volume for b in c.bRepBodies) * 1000, 1))
    print('timeline', d.timeline.count, 'problems', L.timeline_problems())


def hub(comp_name, z0, z1, radius, bore, screw_r):
    """Hub on one side of a pulley (own frame, mm) with two opposite radial tapped holes for set screws."""
    c = L.comp(comp_name)
    M.disk(c, 'Set_screw_hub', min(z0, z1), abs(z1 - z0), radius, bore, op=M.JOIN)
    sk = c.sketches.add(c.yZConstructionPlane)
    sk.name = 'Set_screw_taps_profile'
    p = sk.modelToSketchSpace(core.Point3D.create(0, 0, (z0 + z1) / 20))
    sk.sketchCurves.sketchCircles.addByCenterRadius(core.Point3D.create(p.x, p.y, 0), screw_r / 10)
    before = c.bRepBodies.item(0).volume
    L.extrude(c, sk.profiles.item(0), 2 * (radius + 1), 0, M.CUT, 'Set_screw_taps', symmetric=True)
    sk.isVisible = False
    removed = (before - c.bRepBodies.item(0).volume) * 1000
    assert c.bRepBodies.count == 1 and removed > 0, (comp_name, c.bRepBodies.count, removed)
    return removed


def shorten(comp_name, feature, by):
    ext = L.comp(comp_name).features.extrudeFeatures.itemByName(feature)
    v = ext.extentOne.distance.value * 10
    L.set_distance(ext, v - by if v > 0 else v + by)
    return v


def hubs():
    """Stock pulleys are fixed with set screws in a hub (Gates manual p.31-34). Hub diameters follow the
    stock sizes; lengths are what fits next to each pulley. Also: 1 mm between each motor shaft and the
    shaft it drives inside the coupling, and a 0.5 mm relief where the fixed transfer stub faced the
    turning q3 shaft and bearing."""
    d = guard()
    done = {}
    # name, hub from z0 to z1 in the pulley's own frame, hub radius, bore radius, tap drill radius
    for name, z0, z1, r, bore, tap in (
            ('D_GT3M_16T_W9_D6', -6.0, -1.0, 8.76, 3.0, 1.25),      # q3 input: hub outboard, to the shaft end
            ('D_GT3M_90T_W9_D8', -10.5, -1.0, 15.875, 4.0, 1.65),   # q3 stage 1: hub outboard, shaft lengthened below
            ('D_GT3M_16T_W15_D6', -7.5, -1.0, 8.76, 3.0, 1.25),     # q1 and q3 stage 2
            ('D_GT3M_20T_W6_D6', 40.0, 47.0, 6.35, 3.0, 1.25),      # q4 motor pulley: hub towards the slider, shaft lengthened
            ('D_GT3M_18T_W9_D6', -8.0, -1.0, 5.6, 3.0, 1.25),       # q1 input
            ('D_GT3M_120T_W9_D8', -8.5, -1.0, 15.875, 4.0, 1.65),   # q1 stage 1
            ('D_GT3M_40T_W9_D8', -8.0, -1.0, 14.6, 4.0, 1.65)):     # wrist
        done[name] = round(hub(name, z0, z1, r, bore, tap), 1)
    M.disk(L.comp('D_Q3_Intermediate_shaft'), 'Hub_extension', -9.5, 9.5, 4.0, op=M.JOIN)
    M.disk(L.comp('D_Q4_Input_shaft_v3'), 'Hub_extension', 40.0, 7.0, 3.0, op=M.JOIN)
    M.disk(L.comp('D_Q4_Transfer_stub'), 'Running_clearance_relief', 30.0, 0.5, 16.5, op=M.CUT)
    was = {n: shorten(n, 'Axial_length', 1.0) for n in ('D_Q3_Input_shaft', 'D_Q1_Input_shaft')}
    d.computeAll()
    print('set screw holes removed mm3', done)
    print('input shaft lengths before', was)
    for n in ('D_Q3_Intermediate_shaft', 'D_Q4_Input_shaft_v3', 'D_Q4_Transfer_stub', 'D_Q3_Input_shaft', 'D_Q1_Input_shaft'):
        c = L.comp(n)
        bb = c.bRepBodies.item(0).boundingBox
        print(n, 'bodies', c.bRepBodies.count, 'local z', round(bb.minPoint.z * 10, 2), round(bb.maxPoint.z * 10, 2))
    print('timeline', d.timeline.count, 'problems', L.timeline_problems())
