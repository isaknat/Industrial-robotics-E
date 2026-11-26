"""Fourth round (after CAD edition 3, 2026-11-26): landing for the second lock screw, rounded plate corners.

Run inside Fusion on the working document E_Writing_q3_50W_build:
  lock_screw_ear()   the second lock screw of the q3 idler bracket (struct_fix2.small_fixes) stood on the
                     slanted edge of the drive cheek, with part of its spacer beyond the plate. The check
                     made then looked only at the centre of the hole. The cheek gets an ear under it.
  round_corners()    plates and flanges were plain polygons: outer corners get a 4 mm radius (2 mm where
                     4 does not fit), inner corners 3 mm (1.5 mm). Only corners that run through the
                     thickness of a plate are touched; holes, slots and bores stay as they are.
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
import belt_path as B
for _m in (L, M, B):
    importlib.reload(_m)

DOC = 'E_Writing_q3_50W_build'
IN = fusion.PointContainment.PointInsidePointContainment
OUT = fusion.PointContainment.PointOutsidePointContainment


def guard():
    app = core.Application.get()
    if app.activeDocument.name != DOC:
        raise RuntimeError('Expected the working document, got ' + app.activeDocument.name)
    return L.design()


def healthy(d):
    return [(i, d.timeline.item(i).name) for i in range(d.timeline.count)
            if d.timeline.item(i).healthState != fusion.FeatureHealthStates.HealthyFeatureHealthState]


def lock_screw_ear():
    d = guard()
    x, y = B.MID[0] + 28.0, B.MID[1]
    cheek = L.comp('D_Q3_Drive_cheek')
    body = cheek.bRepBodies.item(0)
    rim = lambda r: sum(body.pointContainment(core.Point3D.create((x + r * math.cos(a)) / 10, (y + r * math.sin(a)) / 10, 0.45)) == IN
                        for a in [i * math.pi / 12 for i in range(24)])
    before = rim(3.7)
    M.disk(cheek, 'Second_lock_ear', 0, 9, 8.0, 1.65, xy=(x, y), op=M.JOIN)
    d.computeAll()
    body = cheek.bRepBodies.item(0)
    assert cheek.bRepBodies.count == 1 and rim(3.7) == 24 and rim(7.5) == 24, (cheek.bRepBodies.count, rim(3.7), rim(7.5))
    print('points on the spacer rim that lie on the plate:', before, '-> 24 of 24; not healthy:', healthy(d))


PLATES = [  # component, also round the inner corners
    ('D_Motor_flange', True), ('D_Q3_Drive_cheek', True), ('D_Q3_Idle_cheek', True), ('D_Q3_Outboard_bridge', True),
    ('D_Q3_Idler_bracket', True), ('D_Q4_Long_idler_bracket', True), ('D_Q4_Transfer_stub', False),
    ('D_Q2_Motor_foot', False), ('D_Q2_Rail_plate', False), ('D_Q2_Nut_bracket', False), ('D_Q2_Support_riser', False),
    ('D_Bearing_cap_608', False)]


THICKNESS = {'D_Q3_Outboard_bridge': 0.7, 'D_Q4_Transfer_stub': 0.4}     # cm; parts with a boss, where the box is thicker than the plate


KEEP_SHARP = {'D_Q3_Drive_cheek': [(11.519, -18.743, 14.0), (10.522, 19.321, 14.0)]}   # x, y, radius in mm: the two small ears
                                                                                    # that carry the q3 stop pins

def corner_edges(comp):
    """Straight edges between two flat faces that run through the plate thickness, split into outer and inner corners."""
    body = comp.bRepBodies.item(0)
    bb = body.boundingBox
    thick = THICKNESS.get(comp.name) or min(bb.maxPoint.x - bb.minPoint.x, bb.maxPoint.y - bb.minPoint.y, bb.maxPoint.z - bb.minPoint.z)
    outer, inner = [], []
    for e in body.edges:
        if e.geometry.objectType != core.Line3D.classType() or e.faces.count != 2:
            continue
        if not (0.8 * thick <= e.length <= 1.05 * thick):
            continue
        f1, f2 = e.faces.item(0), e.faces.item(1)
        if f1.geometry.objectType != core.Plane.classType() or f2.geometry.objectType != core.Plane.classType():
            continue
        a, b = e.startVertex.geometry, e.endVertex.geometry
        mid = core.Point3D.create((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2)
        if any(math.hypot(mid.x * 10 - x, mid.y * 10 - y) < r for x, y, r in KEEP_SHARP.get(comp.name, [])):
            continue
        ok1, n1 = f1.evaluator.getNormalAtPoint(mid)
        ok2, n2 = f2.evaluator.getNormalAtPoint(mid)
        if not (ok1 and ok2) or abs(n1.dotProduct(n2)) > 0.9:          # flush or opposite faces: not a corner
            continue
        eps = 0.002                                                    # 0.02 mm
        side = []
        for s in (1, -1):
            p = core.Point3D.create(mid.x + s * eps * (n1.x - n2.x), mid.y + s * eps * (n1.y - n2.y), mid.z + s * eps * (n1.z - n2.z))
            side.append(body.pointContainment(p))
        if side[0] == OUT and side[1] == OUT:
            outer.append(e)
        elif side[0] == IN and side[1] == IN:
            inner.append(e)
    return outer, inner


def fillet(comp, edges, radius, label):
    ff = comp.features.filletFeatures
    coll = core.ObjectCollection.create()
    for e in edges:
        coll.add(e)
    inp = ff.createInput()
    inp.addConstantRadiusEdgeSet(coll, core.ValueInput.createByString(f'{radius} mm'), False)
    f = ff.add(inp)
    f.name = label
    return f


def round_set(comp, which, radii, label):
    """All corners of one kind in one feature if possible, otherwise one by one with the smaller radius."""
    pick = lambda: corner_edges(comp)[0 if which == 'outer' else 1]
    n = len(pick())
    if n == 0:
        return 0, 0
    try:
        fillet(comp, pick(), radii[0], f'{label}_R{radii[0]:g}')
        return n, 0
    except RuntimeError:
        pass
    done = small = 0
    guard_count = 0
    while guard_count < 80:
        guard_count += 1
        todo = pick()
        progressed = False
        for e in todo:
            for r in radii:
                try:
                    fillet(comp, [e], r, f'{label}_R{r:g}_{done + 1}')
                    done += 1
                    small += r != radii[0]
                    progressed = True
                    break
                except RuntimeError:
                    continue
            if progressed:
                break                              # the body changed: collect the edges again
        if not progressed:
            break
    return done, small


def round_corners(only=None):
    d = guard()
    lines = []
    def say(t):
        lines.append(t)
        print(t)
        (HERE / 'round_corners_log.txt').write_text(chr(10).join(lines), encoding='utf8')
    for name, inner_too in PLATES:
        if only and name not in only:
            continue
        comp = L.comp(name)
        v0 = comp.bRepBodies.item(0).volume
        o, osmall = round_set(comp, 'outer', (4.0, 2.0), 'Outer_corners')
        i, ismall = round_set(comp, 'inner', (3.0, 1.5), 'Inner_corners') if inner_too else (0, 0)
        left = corner_edges(comp)
        v1 = comp.bRepBodies.item(0).volume
        assert comp.bRepBodies.count == 1, name
        say(f'{name}: outer corners rounded {o} ({osmall} with the small radius), inner {i} ({ismall} small); '
              f'still sharp: {len(left[0])} outer, {len(left[1]) if inner_too else "-"} inner; volume {v0 * 1000:.0f} -> {v1 * 1000:.0f} mm3')
    d.computeAll()
    bad = healthy(d)
    say(f'timeline {d.timeline.count} not healthy {bad}')
    assert not bad, bad


def redo_cheek():
    """The first pass rounded the corners of the small ear that holds the lower q3 stop pin and cut the
    ear away (found by the contact check: the pin touched nothing). The two fillet features of the
    drive cheek are removed and made again, leaving the two stop ears alone."""
    d = guard()
    cheek = L.comp('D_Q3_Drive_cheek')
    ff = cheek.features.filletFeatures
    for name in [f.name for f in ff][::-1]:
        ff.itemByName(name).deleteMe()
    d.computeAll()
    v_plain = cheek.bRepBodies.item(0).volume * 1000
    round_corners(only=('D_Q3_Drive_cheek',))
    mm = core.Application.get().measureManager
    root = d.rootComponent
    co = next(o for o in root.allOccurrences if o.component.name == 'D_Q3_Drive_cheek')
    for s in ('D_Q3_Lower_stop', 'D_Q3_Upper_stop'):
        so = next(o for o in root.allOccurrences if o.component.name == s)
        gap = mm.measureMinimumDistance(so.bRepBodies.item(0), co.bRepBodies.item(0)).value * 10
        print(s, 'distance to the cheek', round(gap, 4))
        assert gap < 1e-4, (s, gap)
    print('cheek volume without fillets', round(v_plain), 'with', round(cheek.bRepBodies.item(0).volume * 1000))
