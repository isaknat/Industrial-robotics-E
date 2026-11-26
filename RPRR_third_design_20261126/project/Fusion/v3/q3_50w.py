"""q3 drive changed to the 50 W motor with 1:42.19. after CAD edition 3 (2026-11-26).

Run inside Fusion on the separate working document E_Writing_q3_50W_build (imported from
E_Writing.f3d), one stage at a time: pulleys_and_motor(), belts_and_idlers(), report().

  motor    HG-KR13 (100 W) -> HG-KR053 (50 W), same flange and shaft, 16 mm shorter
  stage 1  16 -> 80 teeth becomes 16 -> 90 (9 mm belt, 101 teeth instead of 94)
  stage 2  24 -> 120 teeth becomes 16 -> 120 (15 mm belt, 138 teeth instead of 140)
  total    1:25 becomes 90/16 * 120/16 = 1:42.19
The shafts stay where they are. All tooth counts are stock sizes (Gates manual p.31-33).
Both belts are a little longer than the path around the two pulleys (0.24 and 1.17 mm); each
backside idler is placed where it makes the path exactly as long as the belt (belt_path.py).

Coordinates: mm in the slider frame (D_03: origin on the q3 axis, x along the rail, y up).
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
G = 'D_03'
MID, INPUT, Q3 = B.MID, B.INPUT, B.Q3
BIG1, SMALL2 = 90, 16                      # new stage-1 driven pulley, new stage-2 driving pulley
BELT1, BELT2 = 101, 138                    # belt teeth
IDLER1_X, ROLLER1, ROLLER2 = -94.0, 8.0, 8.0
WINDOW_DIA, WINDOW_RADIUS = 16, 27         # six lightening windows of the 90-tooth pulley


def guard():
    app = core.Application.get()
    if app.activeDocument.name != DOC:
        raise RuntimeError('Expected the working document, got ' + app.activeDocument.name)
    return L.design()


def xy(o):
    t = o.transform2.translation
    return t.x * 10, t.y * 10, t.z * 10


def put(o, mat):
    """Give a part of the sub-assembly a new place (mat in the sub-assembly frame).
    With the timeline rolled back to before a part made earlier, Fusion takes the value written to
    initialTransform in root coordinates (found by trial on 2026-11-26: writing the local matrix
    moved the part by minus the position of the sub-assembly). So the local matrix is tried first
    and, if the part does not land on it, the matrix is written again in root coordinates."""
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
        in_root.transformBy(L.group(G).transform2)
        write(in_root)
    assert err() < 1e-6, (o.name, o.transform2.asArray(), mat.asArray())
    o.isGroundToParent = True


def idler_positions():
    """Where the two idlers must sit for belts of BELT1 and BELT2 teeth."""
    p1 = [(*INPUT, 16), (*MID, BIG1)]
    y1 = B.solve(p1, lambda y: (IDLER1_X, y), ROLLER1, BELT1, 36.0, 58.0)
    c1 = B.pitch_circles(p1, (IDLER1_X, y1, ROLLER1))
    old = xy(N.instance(G, 'D_Belt_idler_15'))
    arm = math.hypot(old[0] - MID[0], old[1] - MID[1])
    old_deg = math.degrees(math.atan2(old[1] - MID[1], old[0] - MID[0]))
    at = lambda deg: (MID[0] + arm * math.cos(math.radians(deg)), MID[1] + arm * math.sin(math.radians(deg)))
    p2 = [(*Q3, 120), (*MID, SMALL2)]
    new_deg = -B.solve(p2, lambda u: at(-u), ROLLER2, BELT2, -old_deg - 2.0, -old_deg + 50.0)
    x2, y2 = at(new_deg)
    c2 = B.pitch_circles(p2, (x2, y2, ROLLER2))
    return dict(idler1=(IDLER1_X, y1), circles1=c1, idler2=(x2, y2), circles2=c2, arm=arm,
                old_deg=old_deg, new_deg=new_deg, swing=new_deg - old_deg)


def pulleys_and_motor():
    d = guard()
    group = L.group(G).component
    old = {o.component.name: o for o in group.occurrences
           if o.component.name in ('D_GT3M_80T_W9_D8', 'D_GT3M_24T_W15_D8', 'D_HG_KR13')}
    assert len(old) == 3, list(old)
    place = {n: xy(o) for n, o in old.items()}
    assert max(abs(a - b) for a, b in zip(place['D_GT3M_80T_W9_D8'], (*MID, -102))) < 1e-6, place
    assert max(abs(a - b) for a, b in zip(place['D_GT3M_24T_W15_D8'], (*MID, -71))) < 1e-6, place
    assert max(abs(a - b) for a, b in zip(place['D_HG_KR13'], (*INPUT, 25))) < 1e-6, place
    for o in old.values():
        o.deleteMe()
    # stage-1 driven pulley, drawn like the other new pulleys of the third edition
    for name, value in (('q3MidWindowDia', WINDOW_DIA), ('q3MidWindowRadius', WINDOW_RADIUS)):
        d.userParameters.itemByName(name).expression = f'{value} mm'
    wheel = M.part(G, f'D_GT3M_{BIG1}T_W9_D8', (*MID, -102))
    M.wheel(wheel, BIG1, 0, 9, 4, f'Stage1_{BIG1}T')
    centres = [(WINDOW_RADIUS * math.cos(i * math.tau / 6), WINDOW_RADIUS * math.sin(i * math.tau / 6)) for i in range(6)]
    before = wheel.bRepBodies.item(0).volume
    M.holes(wheel, 'Six_web_lightening_windows', -2, 13, centres, WINDOW_DIA / 2)
    removed = (before - wheel.bRepBodies.item(0).volume) * 1000
    ideal = 6 * math.pi * (WINDOW_DIA / 2) ** 2 * 11
    assert abs(removed / ideal - 1) < 0.01, (removed, ideal)
    sk = wheel.sketches.itemByName('Six_web_lightening_windows_profile')
    sk.timelineObject.rollTo(False)
    dims = sk.sketchDimensions
    H = fusion.DimensionOrientations.HorizontalDimensionOrientation
    V = fusion.DimensionOrientations.VerticalDimensionOrientation
    for circle in sk.sketchCurves.sketchCircles:
        c = circle.centerSketchPoint
        x, y = c.geometry.x * 10, c.geometry.y * 10
        dim = dims.addDiameterDimension(circle, M.P(x + WINDOW_DIA / 2 + 1, y + WINDOW_DIA / 2 + 1))
        dim.parameter.expression = 'q3MidWindowDia'
        for val, orient in ((x, H), (y, V)):
            if abs(val) < 1e-5:
                if orient == H:
                    sk.geometricConstraints.addVerticalPoints(sk.originPoint, c)
                else:
                    sk.geometricConstraints.addHorizontalPoints(sk.originPoint, c)
            else:
                dim = dims.addDistanceDimension(sk.originPoint, c, orient, M.P(x * .5 + 1, y * .5 + 1))
                dim.parameter.expression = f'q3MidWindowRadius * {abs(val) / WINDOW_RADIUS:.12g}'
        assert circle.isFullyConstrained, (x, y)
    d.timeline.moveToEnd()
    # stage-2 driving pulley: the same stock part as in the q1 drive; motor: the same 50 W motor as q1, q2, q4
    M.place(G, L.comp('D_GT3M_16T_W15_D8'), (*MID, -71))
    M.place(G, L.comp('D_HG_KR053'), (*INPUT, 25), M.reverse_axes)
    d.computeAll()
    names = [c.name for c in d.allComponents]
    print('removed components still listed:', [n for n in ('D_GT3M_80T_W9_D8', 'D_GT3M_24T_W15_D8', 'D_HG_KR13') if n in names])
    print('90T pulley volume mm3', round(wheel.bRepBodies.item(0).volume * 1000), 'windows removed', round(removed))
    print('timeline', d.timeline.count, 'problems', L.timeline_problems())


def draw_loop(sk, circles):
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    for seg in B.loop(circles):
        if seg[0] == 'line':
            lines.addByTwoPoints(M.P(*seg[1]), M.P(*seg[2]))
        else:
            (cx, cy), r, a0, sweep = seg[1:]
            pt = lambda a: M.P(cx + r * math.cos(a), cy + r * math.sin(a))
            arcs.addByThreePoints(pt(a0), pt(a0 + sweep / 2), pt(a0 + sweep))


def belt(name, circles, z, width, label):
    c = M.part(G, name, (0, 0, z), M.BELT)
    sk = M.sketch(c, label)
    draw_loop(sk, B.offset(circles, -B.PITCH_TO_ROOT))
    draw_loop(sk, B.offset(circles, B.PITCH_TO_BACK))
    band = [p for p in sk.profiles if p.profileLoops.count == 2]
    assert len(band) == 1, [p.profileLoops.count for p in sk.profiles]
    L.extrude(c, band[0], width, 0, M.NEW, label)
    sk.isVisible = False
    area = band[0].areaProperties().area * 100
    return c, area


def belts_and_idlers():
    d = guard()
    pos = idler_positions()
    group = L.group(G).component
    for n in ('D_Belt_Q3_stage1', 'D_Belt_Q3_stage2'):
        N.instance(G, n).deleteMe()
    left = [c.name for c in d.allComponents if c.name in ('D_Belt_Q3_stage1', 'D_Belt_Q3_stage2')]
    assert not left, left
    b1, a1 = belt('D_Belt_Q3_stage1', pos['circles1'], -102, 9, f'3MGT_{BELT1}T_backing')
    b2, a2 = belt('D_Belt_Q3_stage2', pos['circles2'], -71, 15, f'3MGT_{BELT2}T_backing')
    for label, area, teeth in (('stage 1', a1, BELT1), ('stage 2', a2, BELT2)):
        # band area = thickness times the length of its centre line, which is 0.36 mm outside the pitch line
        print(label, 'band area', round(area, 1), 'mm2; thickness 1.48 x pitch length would be', round(1.48 * 3 * teeth, 1))
    # stage-1 idler: roller, axle and the slot in the outboard bridge
    x1, y1 = pos['idler1']
    roller = N.instance(G, 'D_Q3_First_stage_idler')
    axle = N.instance(G, 'D_Q3_First_stage_idler_axle')
    ox, oy, _ = xy(roller)
    put(roller, M.matrix((x1, y1, -102)))
    put(axle, M.matrix((x1, y1, -104)))
    sk = L.comp('D_Q3_Outboard_bridge').sketches.itemByName('First_stage_tension_slot_profile')
    curves = core.ObjectCollection.create()
    for cv in sk.sketchCurves:
        curves.add(cv)
    move = core.Matrix3D.create()
    move.translation = core.Vector3D.create((x1 - ox) / 10, (y1 - oy) / 10, 0)
    assert sk.move(curves, move)
    # stage-2 idler: bracket, roller and axle swing about the intermediate shaft
    swing = math.radians(pos['swing'])
    co, si = math.cos(swing), math.sin(swing)
    bracket = N.instance(G, 'D_Q3_Idler_bracket')
    bx, by, bz = xy(bracket)
    rot = lambda x, y: (MID[0] + co * (x - MID[0]) - si * (y - MID[1]), MID[1] + si * (x - MID[0]) + co * (y - MID[1]))
    put(bracket, M.matrix((*rot(bx, by), bz), ((co, si, 0), (-si, co, 0))))
    x2, y2 = pos['idler2']
    for n in ('D_Belt_idler_15', 'D_Idler_axle_6'):
        o = N.instance(G, n)
        put(o, M.matrix((x2, y2, xy(o)[2])))
    # the lock screw stays in the side plate: turn the bracket's slot back so that it is centred on the screw
    sk = L.comp('D_Q3_Idler_bracket').sketches.itemByName('Angular_tension_slot_profile')
    curves = core.ObjectCollection.create()
    for cv in sk.sketchCurves:
        curves.add(cv)
    back = core.Matrix3D.create()
    back.setToRotation(-swing, core.Vector3D.create(0, 0, 1), core.Point3D.create(MID[0] / 10, MID[1] / 10, 0))
    assert sk.move(curves, back)
    d.computeAll()
    out = {k: (list(v) if isinstance(v, tuple) else v) for k, v in pos.items() if not k.startswith('circles')}
    out.update(stage1_wrap_16T_deg=B.wrap_deg(pos['circles1'], 0), stage1_idler_wrap_deg=B.wrap_deg(pos['circles1'], 1),
               stage2_wrap_16T_deg=B.wrap_deg(pos['circles2'], 2), stage2_idler_wrap_deg=B.wrap_deg(pos['circles2'], 1),
               stage1_length_mm=B.length(pos['circles1']), stage2_length_mm=B.length(pos['circles2']),
               ratio=BIG1 / 16 * 120 / SMALL2)
    (HERE / 'q3_50w_layout.json').write_text(json.dumps(out, indent=1), encoding='utf8')
    print(json.dumps(out, indent=1))
    for n in ('D_Q3_Outboard_bridge', 'D_Q3_Idler_bracket', 'D_Belt_Q3_stage1', 'D_Belt_Q3_stage2'):
        c = L.comp(n)
        print(n, 'bodies', c.bRepBodies.count, 'volume mm3', round(sum(b.volume for b in c.bRepBodies) * 1000, 1))
    print('timeline', d.timeline.count, 'problems', L.timeline_problems())
