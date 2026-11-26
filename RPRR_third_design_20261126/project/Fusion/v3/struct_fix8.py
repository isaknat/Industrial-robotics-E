"""Eighth round (after CAD edition 3, 2026-11-26): four open points of the review and the arm tip.

Run inside Fusion on the working document E_Writing_q3_50W_build, one stage at a time:
  arm_tip()       the fork end of the arm was cut at an angle only underneath (to clear the drum floor);
                  the same cut on top, so the tip is symmetric
  idler_axles()   three idler axles had a head AND a shoulder larger than the roller bore, so the roller
                  could not be put on; the head becomes a retaining ring. The fourth gets a ring too.
  input_shaft()   the q3 input shaft is 8 mm between its two bearings (6 mm before); the steps replace
                  the inner retaining ring
  lanterns()      q3 and q4 motors: one U-shaped support each instead of four thin standoffs
  base_column()   rounded rim on the base cover and base, rounded long edges on the column
Only new parts are placed and existing ones deleted; no existing part is moved.
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
import struct_fix6 as S6
for _m in (L, M, N, B, S6):
    importlib.reload(_m)

DOC = 'E_Writing_q3_50W_build'
FLOOR = ((1, 0, 0), (0, 0, -1))
INPUT = B.INPUT


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


def done(d, names):
    d.computeAll()
    for n in names:
        c = L.comp(n)
        print(n, 'bodies', c.bRepBodies.count, 'volume', round(volume(c), 1))
    bad = healthy(d)
    print('timeline', d.timeline.count, 'not healthy', bad)
    assert not bad, bad


# ---------------------------------------------------------------- arm tip
def arm_tip():
    d = guard()
    tube = L.comp('D_Q3_Link_40x40x3')
    v0 = volume(tube)
    # mirror image of Fork_floor_relief (304,-20.1) (320.1,-20.1) (320.1,-6.5)
    N.cut('D_04', 'D_Q3_Link_40x40x3', 'Fork_top_relief', 2, -20, 20, [('poly', (304, 20.1), (320.1, 20.1), (320.1, 6.5))])
    assert tube.bRepBodies.count == 1 and volume(tube) < v0 - 50, (tube.bRepBodies.count, v0 - volume(tube))
    print('removed', round(v0 - volume(tube)), 'mm3')
    S6.soften(['D_Q3_Link_40x40x3'], outer_r=1.0, inner_r=0)
    done(d, ['D_Q3_Link_40x40x3'])


# ---------------------------------------------------------------- idler axles
def resize(comp_name, sketch_name, old_r, new_r):
    sk = L.comp(comp_name).sketches.itemByName(sketch_name)
    hit = [c for c in sk.sketchCurves.sketchCircles if abs(c.radius * 10 - old_r) < 0.01]
    assert len(hit) == 1, (comp_name, sketch_name, [c.radius * 10 for c in sk.sketchCurves.sketchCircles])
    hit[0].radius = new_r / 10


def idler_axles():
    d = guard()
    ring = L.comp('D_Retaining_ring_6')

    def pos(group, name):
        o = next(x for x in L.group(group).component.occurrences if x.component.name == name)
        t = o.transform2.translation
        return t.x * 10, t.y * 10, t.z * 10
    # q3 stage 1 (axis z, axle from z -104): the 4.3 mm head at the free end becomes a 6 mm journal end with a ring
    resize('D_Q3_First_stage_idler_axle', 'Captive_head_profile', 4.3, 3.0)
    M.disk(L.comp('D_Q3_First_stage_idler_axle'), 'Ring_groove', 1.2, 0.7, 3.6, 2.85, op=M.CUT)
    x, y, z = pos('D_03', 'D_Q3_First_stage_idler_axle')
    M.place('D_03', ring, (x, y, z + 1.2))
    # q4 long belt (axis z, pin from z 17): retainer at the free end
    resize('D_Q4_Long_idler_pin', 'Retainer_profile', 4.0, 3.0)
    M.disk(L.comp('D_Q4_Long_idler_pin'), 'Ring_groove', 28.1, 0.7, 3.6, 2.85, op=M.CUT)
    x, y, z = pos('D_04', 'D_Q4_Long_idler_pin')
    M.place('D_04', ring, (x, y, z + 28.1))
    # q1 stage 1 (axis y, adjuster from y 0): retainer at the free end
    c = L.comp('D_Q1_Stage1_adjuster')
    sk = c.sketches.itemByName('Retainer_profile')
    old = [round(ci.radius * 10, 2) for ci in sk.sketchCurves.sketchCircles]
    resize('D_Q1_Stage1_adjuster', 'Retainer_profile', old[0], 3.0)
    M.disk(c, 'Ring_groove', 33.2, 0.7, 3.6, 2.85, op=M.CUT)
    x, y, z = pos('D_01', 'D_Q1_Stage1_adjuster')
    M.place('D_01', ring, (x, y + 33.2, z), FLOOR)
    # q1 stage 2 (axis y, journal starts at y 46, free end): 1 mm more journal and a ring
    c = L.comp('D_Q1_Adjustable_idler_axle')
    M.disk(c, 'Ring_end', -1.0, 1.0, 3.0, op=M.JOIN)
    M.disk(c, 'Ring_groove', -0.8, 0.7, 3.6, 2.85, op=M.CUT)
    x, y, z = pos('D_01', 'D_Q1_Adjustable_idler_axle')
    M.place('D_01', ring, (x, y - 0.8, z), FLOOR)
    for n in ('D_Q3_First_stage_idler_axle', 'D_Q4_Long_idler_pin', 'D_Q1_Stage1_adjuster', 'D_Q1_Adjustable_idler_axle'):
        assert L.comp(n).bRepBodies.count == 1, n
    print('q1 stage-1 retainer radius was', old)
    done(d, ['D_Q3_First_stage_idler_axle', 'D_Q4_Long_idler_pin', 'D_Q1_Stage1_adjuster', 'D_Q1_Adjustable_idler_axle'])


# ---------------------------------------------------------------- q3 input shaft
def input_shaft():
    d = guard()
    sh = L.comp('D_Q3_Input_shaft')                 # own z = slider z + 108; bearings at z -90..-84 and -28..-22
    v0 = volume(sh)
    M.disk(sh, 'Centre_section_D8', 24.0, 56.0, 4.0, op=M.JOIN)        # z -84..-28
    assert sh.bRepBodies.count == 1 and volume(sh) > v0 + 1000, volume(sh) - v0
    inner = [o for o in L.group('D_03').component.occurrences if o.component.name == 'D_Retaining_ring_6'
             and abs(o.transform2.translation.x * 10 - INPUT[0]) < 0.01 and abs(o.transform2.translation.z * 10 + 84.0) < 0.01]
    assert len(inner) == 1, len(inner)
    inner[0].deleteMe()
    done(d, ['D_Q3_Input_shaft'])


# ---------------------------------------------------------------- motor supports
def lantern(name, pos, length, bottom, relief=None):
    """U-shaped support between a plate and the motor plate (own z along the motor axis, open towards -y).
    Side walls 6 mm, top wall 6 mm, four bosses with 4.4 mm holes at (+-23, +-24) for the through bolts,
    a window in each side wall for the coupling screws."""
    c = M.part('D_03', name, pos)
    sk = M.sketch(c, 'U_section')
    L.poly(sk, [(-32, bottom), (-26, bottom), (-26, 24), (26, 24), (26, bottom), (32, bottom), (32, 30), (-32, 30)])
    M.solid(c, sk, 0, length, 'U_section', M.NEW, True)
    for i, (a, b) in enumerate([(a, b) for a in (-23, 23) for b in (-24, 24)]):
        M.disk(c, f'Bolt_boss_{i + 1}', 0, length, 4.0, xy=(a, b), op=M.JOIN)
    M.holes(c, 'Bolt_holes', 0, length, [(a, b) for a in (-23, 23) for b in (-24, 24)], 2.2)
    wsk = c.sketches.add(c.yZConstructionPlane)
    wsk.name = 'Side_windows_profile'
    pts = [wsk.modelToSketchSpace(core.Point3D.create(0, y / 10, z / 10)) for y, z in ((-12, 7), (12, 7), (12, length - 7), (-12, length - 7))]
    ln = wsk.sketchCurves.sketchLines
    for p, q in zip(pts, pts[1:] + pts[:1]):
        ln.addByTwoPoints(core.Point3D.create(p.x, p.y, 0), core.Point3D.create(q.x, q.y, 0))
    L.extrude(c, wsk.profiles.item(0), 70, 0, M.CUT, 'Side_windows', symmetric=True)
    wsk.isVisible = False
    if relief:
        M.disk(c, 'Nut_relief', 0, 6.0, 5.0, xy=relief, op=M.CUT)
    assert c.bRepBodies.count == 1, (name, c.bRepBodies.count)
    return c


def lanterns():
    d = guard()
    group = L.group('D_03').component
    # q3: between the drive cheek (z -21) and the motor plate (z 20); a nut of the outboard standoffs sits under one wall
    old = [o for o in group.occurrences if o.component.name == 'D_Spacer_41']
    assert len(old) == 4
    for o in old:
        o.deleteMe()
    lantern('D_Q3_Motor_support', (*INPUT, -21), 41.0, -30, relief=(-75 - INPUT[0], 92 - INPUT[1]))
    # q4: between the four landings of the motor bracket (z 76.5) and the motor plate (z 104.5);
    # shorter side walls, because the sliding foot of the bracket and its two screws lie underneath
    old = [o for o in group.occurrences if o.component.name == 'D_Spacer_28']
    assert len(old) == 4
    x4, y4 = old[0].transform2.translation.x * 10 + 23, old[0].transform2.translation.y * 10 + 24
    for o in old:
        o.deleteMe()
    lantern('D_Q4_Motor_support', (x4, y4, 76.5), 28.0, -28)
    left = [c.name for c in d.allComponents if c.name in ('D_Spacer_41', 'D_Spacer_28')]
    print('q4 motor axis', (round(x4, 3), round(y4, 3)), '; standoff parts still listed:', left)
    S6.soften(['D_Q3_Motor_support', 'D_Q4_Motor_support'], outer_r=1.0, inner_r=0)
    done(d, ['D_Q3_Motor_support', 'D_Q4_Motor_support'])


# ---------------------------------------------------------------- base and column
def circle_edges(comp, radius_mm, z_mm, tol=0.05):
    out = []
    for e in comp.bRepBodies.item(0).edges:
        g = e.geometry
        if g.objectType == core.Circle3D.classType() and abs(g.radius * 10 - radius_mm) < tol and abs(g.center.z * 10 - z_mm) < tol:
            out.append(e)
    return out


def base_column():
    d = guard()
    # cover: top outer edge (own z 83, radius 165) 5 mm; base: bottom outer edge (own z 0) 3 mm
    lid, tray = L.comp('D_Q1_Round_lid'), L.comp('D_Q1_Round_base')
    e = circle_edges(lid, 165.0, 83.0)
    assert len(e) == 1, len(e)
    S6.fillet(lid, e, 5.0, 'Top_rim_R5')
    e = circle_edges(tray, 165.0, 0.0)
    assert len(e) == 1, len(e)
    S6.fillet(tray, e, 3.0, 'Bottom_rim_R3')
    # column: the four long edges of the 70 x 60 tube, 8 mm outside; inside as much as keeps the wall
    col = L.comp('D_Q1_Column')

    def long_edges(kind):
        out = []
        for ed in S6.straight_edges(col, kind):
            a, b = ed.startVertex.geometry, ed.endVertex.geometry
            if ed.length >= 5.0 and abs(b.z - a.z) > 0.999 * ed.length:
                out.append(ed)
        return out
    n_out = len(long_edges('outer'))
    if n_out:
        S6.fillet(col, long_edges('outer'), 8.0, 'Long_outer_corners_R8')
    n_in = len(long_edges('inner'))
    r_in = None
    for r_in in (4.0, 3.0, 2.0):
        if not n_in:
            break
        try:
            S6.fillet(col, long_edges('inner'), r_in, f'Long_inner_corners_R{r_in:g}')
            break
        except RuntimeError:
            continue
    hits = {n: S6.interference_with(d, n) for n in ('D_Q1_Round_lid', 'D_Q1_Round_base', 'D_Q1_Column')}
    print('column long corners: outer', n_out, 'inner', n_in, 'inner radius', r_in, '; interference', hits)
    assert not any(hits.values()), hits
    done(d, ['D_Q1_Round_lid', 'D_Q1_Round_base', 'D_Q1_Column'])
