"""Sixth round (after CAD edition 3, 2026-11-26): soft edges on the parts we make ourselves.

soften(names) rounds, on each named part,
  - every straight outer edge between two flat faces with 1 mm (edge break; it follows the corner
    radii made in struct_fix4 through tangent edges),
  - every straight inner edge between two flat faces with 2 mm (where a boss, wall or rib meets a plate).
Round edges (holes, bores, bosses) are not touched: bearings and screws sit there.
An inner radius adds material, so after each part the whole robot is checked for interference; if the
part now cuts into a neighbour, its inner radii are removed again.
Progress goes to soften_log.txt, because the run is longer than one MCP call.
"""
import importlib
import math
import sys
import time
from pathlib import Path
import adsk.core as core
import adsk.fusion as fusion

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cadlib as L
importlib.reload(L)

DOC = 'E_Writing_q3_50W_build'
IN = fusion.PointContainment.PointInsidePointContainment
OUT = fusion.PointContainment.PointOutsidePointContainment
KEEP_SHARP = {'D_Q3_Drive_cheek': [(11.519, -18.743, 14.0), (10.522, 19.321, 14.0)]}     # the q3 stop ears (x, y, radius)
LOG = HERE / 'soften_log.txt'


def guard():
    app = core.Application.get()
    if app.activeDocument.name != DOC:
        raise RuntimeError('Expected the working document, got ' + app.activeDocument.name)
    return L.design()


def say(text):
    with open(LOG, 'a', encoding='utf8') as f:
        f.write(text + '\n')
    print(text)


def key(e):
    a, b = e.startVertex.geometry, e.endVertex.geometry
    return (round((a.x + b.x) * 50, 1), round((a.y + b.y) * 50, 1), round((a.z + b.z) * 50, 1))


def straight_edges(comp, kind, min_len=0.3):
    body = comp.bRepBodies.item(0)
    out = []
    for e in body.edges:
        if e.geometry.objectType != core.Line3D.classType() or e.faces.count != 2 or e.length < min_len:
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
        if not (ok1 and ok2) or abs(n1.dotProduct(n2)) > 0.9:
            continue
        eps = 0.002
        side = [body.pointContainment(core.Point3D.create(mid.x + s * eps * (n1.x - n2.x), mid.y + s * eps * (n1.y - n2.y),
                                                          mid.z + s * eps * (n1.z - n2.z))) for s in (1, -1)]
        if (kind == 'outer' and side[0] == OUT and side[1] == OUT) or (kind == 'inner' and side[0] == IN and side[1] == IN):
            out.append(e)
    return out


def fillet(comp, edges, radius, label):
    ff = comp.features.filletFeatures
    coll = core.ObjectCollection.create()
    for e in edges:
        coll.add(e)
    inp = ff.createInput()
    inp.addConstantRadiusEdgeSet(coll, core.ValueInput.createByString(f'{radius} mm'), True)
    f = ff.add(inp)
    if f.healthState != fusion.FeatureHealthStates.HealthyFeatureHealthState:      # made, but with a warning:
        f.deleteMe()                                                              # treat it like a failure
        raise RuntimeError('fillet made with a warning: ' + label)
    f.name = label
    return f


def round_all(comp, kind, radius, label, budget=14):
    """Largest batches first; an edge that cannot take the radius even alone is left sharp."""
    bad, made, n_done, tries = set(), [], 0, 0
    while tries < budget:
        edges = [e for e in straight_edges(comp, kind) if key(e) not in bad]
        if not edges:
            break
        size = len(edges)
        ok = False
        while size >= 1 and tries < budget:
            tries += 1
            try:
                f = fillet(comp, edges[:size], radius, f'{label}_{len(made) + 1}')
                made.append(f.name)
                n_done += size
                ok = True
                break
            except RuntimeError:
                size //= 2
        if not ok and size == 0:
            bad.add(key(edges[0]))
    left = len([e for e in straight_edges(comp, kind) if key(e) not in bad]) + len(bad)
    return n_done, left, made


def interference_with(design, name):
    root = design.rootComponent
    coll = core.ObjectCollection.create()

    def walk(occ):
        for b in occ.bRepBodies:
            coll.add(b)
        for c in occ.childOccurrences:
            walk(c)
    for o in root.occurrences:
        if not o.component.name.startswith('Wave_Drum'):
            walk(o)
    inp = design.createInterferenceInput(coll)
    inp.areCoincidentFacesIncluded = False
    res = design.analyzeInterference(inp)
    hits = {}
    for i in range(res.count):
        r = res.item(i)
        a, b = r.entityOne.parentComponent.name, r.entityTwo.parentComponent.name
        if name in (a, b):
            other = b if a == name else a
            hits[other] = hits.get(other, 0.0) + r.interferenceBody.volume * 1000
    return hits


def soften(names, outer_r=1.0, inner_r=2.0):
    d = guard()
    for name in names:
        t0 = time.time()
        comp = L.comp(name)
        v0 = comp.bRepBodies.item(0).volume * 1000
        o_done, o_left, _ = round_all(comp, 'outer', outer_r, f'Edge_break_R{outer_r:g}')
        i_done, i_left, i_made = round_all(comp, 'inner', inner_r, f'Junction_R{inner_r:g}') if inner_r else (0, 0, [])
        note = ''
        if i_made:
            hits = interference_with(d, name)
            if hits:
                ff = comp.features.filletFeatures
                for fn in i_made[::-1]:
                    ff.itemByName(fn).deleteMe()
                note = f'; inner radii removed again, they cut into {sorted(hits)}'
                i_done = 0
        assert comp.bRepBodies.count == 1, name
        v1 = comp.bRepBodies.item(0).volume * 1000
        say(f'{name}: outer edges rounded {o_done}, left sharp {o_left}; inner {i_done}, left {i_left}{note}; '
            f'volume {v0:.0f} -> {v1:.0f} mm3; {time.time() - t0:.0f} s')
    d.computeAll()
    # a fillet can be fine when made and get a warning when the whole design is computed again:
    # such a fillet is taken out (its edges stay sharp) instead of leaving a warning in the timeline
    for _ in range(6):
        sick = [d.timeline.item(i) for i in range(d.timeline.count)
                if d.timeline.item(i).healthState != fusion.FeatureHealthStates.HealthyFeatureHealthState
                and d.timeline.item(i).name.startswith(('Edge_break_R', 'Junction_R'))]
        if not sick:
            break
        say(f'removed after recompute: {sick[-1].name} ({sick[-1].errorOrWarningMessage})')
        sick[-1].entity.deleteMe()
        d.computeAll()
    bad = [(i, d.timeline.item(i).name) for i in range(d.timeline.count)
           if d.timeline.item(i).healthState != fusion.FeatureHealthStates.HealthyFeatureHealthState]
    say(f'batch done: timeline {d.timeline.count}, not healthy {bad}')


def round_tube(outer_r=5.0, inner_r=2.0):
    """The arm stays a 40 x 40 x 3 square tube; its four long edges get a 5 mm radius outside and
    2 mm inside, so the wall stays 3 mm round the corner. 5 mm is the largest radius that leaves the
    side faces flat over the full 29 mm of the wrist bearing seats (flat width 40 - 2 x 5 = 30 mm).
    The short edges are then broken with 1 mm like the other parts."""
    d = guard()
    comp = L.comp('D_Q3_Link_40x40x3')
    v0 = comp.bRepBodies.item(0).volume * 1000

    def long_edges(kind):
        out = []
        for e in straight_edges(comp, kind):
            a, b = e.startVertex.geometry, e.endVertex.geometry
            if e.length >= 5.0 and abs(b.x - a.x) > 0.999 * e.length:      # at least 50 mm, along the arm
                out.append(e)
        return out
    done = {}
    for kind, radius in (('outer', outer_r), ('inner', inner_r)):
        n = 0
        for _ in range(12):
            edges = long_edges(kind)
            if not edges:
                break
            try:
                f = fillet(comp, edges, radius, f'Long_{kind}_corners_R{radius:g}')
                n += len(edges)
            except RuntimeError:
                made_one = False
                for e in edges:
                    try:
                        fillet(comp, [e], radius, f'Long_{kind}_corner_R{radius:g}_{n + 1}')
                        n += 1
                        made_one = True
                        break
                    except RuntimeError:
                        continue
                if not made_one:
                    break
        done[kind] = (n, len(long_edges(kind)))
    assert comp.bRepBodies.count == 1
    hits = interference_with(d, comp.name)
    v1 = comp.bRepBodies.item(0).volume * 1000
    say(f'D_Q3_Link_40x40x3: long outer corners R{outer_r:g}: {done["outer"][0]} done, {done["outer"][1]} left; '
        f'long inner corners R{inner_r:g}: {done["inner"][0]} done, {done["inner"][1]} left; volume {v0:.0f} -> {v1:.0f} mm3; '
        f'interference with {hits}')
    assert not hits, hits
    soften(['D_Q3_Link_40x40x3'], outer_r=1.0, inner_r=0)
