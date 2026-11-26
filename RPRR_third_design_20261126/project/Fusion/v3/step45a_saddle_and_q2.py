"""Third design, steps 4-5 part A: slider saddle, nut bracket and the front of the q2 axis.
after CAD edition 2 (2026-11-24). Run inside Fusion with E_Writing_v3 active.

Coordinates in the comments: mm, slider frame (origin on the q3 axis, x along the rail, y up,
z sideways); the column frame for D_02 parts is the same with x measured from the q1 axis.

1. D_Q2_Saddle: the three plates that carry the ball-screw nut (Bridge_wall, Nut_side_bridge,
   Extended_nut_mounting_bridge) sat at x -49..-9, right behind the q3 axis, where the new q4
   transfer pulleys (z 35-55) and belts must go. They are moved back to x -74..-40; the plate
   that carries the nut bracket (z 64-109) now runs from x -74 to +6, under the moved bracket.
   The two M3 holes become four 4.5 mm holes for M4 screws at x -2.5, z 70/80/92/102.
2. D_Q2_Nut_bracket and the nut move 10 mm forward (q2 lower limit 9.5 -> about 0 mm); the
   bracket's two M3 tap drills become four M4 tap drills (3.3 mm) matching the saddle.
3. D_02: EF08 and its riser move 10 mm forward and the ball screw gets 10 mm longer (q2 upper
   limit stays about 141 mm). The MGN12 rail and the rail plate get 8 mm longer at the front, so
   that the front carriage stays on the rail at q2 = 141 (before: 5.7 mm off the rail end).
   The rail plate's -z edge moves in by 7 mm (z -58 -> -51) to make room for the q3 output
   pulley, which moves 7 mm towards the bearings in a later step.
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import importlib
import adsk.core as core
import adsk.fusion as fusion
import cadlib as L
importlib.reload(L)

near = lambda a, b: abs(a - b) < 0.05


def run(_context: str):
    log = []
    # ---- 1. saddle -------------------------------------------------------------------------
    sad = L.comp("D_Q2_Saddle")
    for skn in ("Bridge_wall_profile", "Nut_side_bridge_profile"):
        sk = sad.sketches.itemByName(skn)
        L.strip(sk)
        a = L.move_points(sk, lambda x, y: near(x, -49), dx=-25)
        b = L.move_points(sk, lambda x, y: near(x, -9), dx=-31)
        log.append(f"{skn}: moved {a}+{b} points")
    sk = sad.sketches.itemByName("Extended_nut_mounting_bridge_profile")
    L.strip(sk)
    a = L.move_points(sk, lambda x, y: near(x, -49), dx=-25)
    b = L.move_points(sk, lambda x, y: near(x, -9), dx=15)
    c = L.move_points(sk, lambda x, y: near(y, -84.8), dy=20.8)
    log.append(f"Extended_nut_mounting_bridge_profile: moved {a}+{b}+{c} points")
    sk = sad.sketches.itemByName("Relocated_nut_bracket_M3_pair_profile")
    cut = sad.features.extrudeFeatures.itemByName("Relocated_nut_bracket_M3_pair")
    L.strip(sk)
    old = sorted(sk.sketchCurves.sketchCircles, key=lambda c: c.centerSketchPoint.geometry.y)
    L.move_circle(old[0], -2.5, -102, 2.25)       # was (-12.5, -101)
    L.move_circle(old[1], -2.5, -70, 2.25)        # was (-12.5, -71)
    for x, y in ((-2.5, -80), (-2.5, -92)):
        L.circle(sk, x, y, 2.25)
    cut.name = "Nut_bracket_M4_holes_outer"
    L.extrude_like(cut, "Relocated_nut_bracket_M3_pair_profile", [(-2.5, -80), (-2.5, -92)],
                   "Nut_bracket_M4_holes_inner")
    log.append("saddle nut holes -> 4 x 4.5 mm for M4")

    # ---- 2. nut bracket and nut ---------------------------------------------------------------
    for inst in ("D_Q2_Nut_bracket:1", "D_R12_10K3_Nut:1"):
        L.shift(L.native_child("D_03", inst), dx=10)
    nb = L.comp("D_Q2_Nut_bracket")
    sk = nb.sketches.itemByName("Saddle_pair_M3_tap_drills_profile")
    cut = nb.features.extrudeFeatures.itemByName("Saddle_pair_M3_tap_drills")
    L.strip(sk)
    old = sorted(sk.sketchCurves.sketchCircles, key=lambda c: c.centerSketchPoint.geometry.x)
    L.move_circle(old[0], -16, -3.5, 1.65)
    L.move_circle(old[1], 16, -3.5, 1.65)
    for x, y in ((-6, -3.5), (6, -3.5)):
        L.circle(sk, x, y, 1.65)
    cut.name = "Saddle_M4_tap_drills_outer"
    L.extrude_like(cut, "Saddle_pair_M3_tap_drills_profile", [(-6, -3.5), (6, -3.5)], "Saddle_M4_tap_drills_inner")
    log.append("nut bracket and nut +10 mm; 4 x M4 tap drills")

    # ---- 3. front of the q2 axis ----------------------------------------------------------------
    for inst in ("D_EF08_End_support:1", "D_Q2_Support_riser:2"):
        L.shift(L.native_child("D_02", inst), dx=10)
    scr = L.comp("D_Q2_Ballscrew_12x10")
    L.set_distance(scr.features.extrudeFeatures.itemByName("Lead10_envelope"), 184)
    L.set_start_offset(scr.features.extrudeFeatures.itemByName("Support_journal"), 222)
    rail = L.comp("D_MGN12_Rail")
    sk = rail.sketches.itemByName("Rail_section_profile")
    L.strip(sk)
    a = L.move_points(sk, lambda x, y: near(x, 184), dx=8)
    log.append(f"EF08 +10, screw 229 -> 239 mm, rail +8 ({a} points)")
    plate = L.comp("D_Q2_Rail_plate")
    sk = plate.sketches.itemByName("Rail_deck_profile")
    L.strip(sk)
    a = L.move_points(sk, lambda x, y: near(x, 190), dx=8)
    b = L.move_points(sk, lambda x, y: near(y, 58), dy=-7)
    sk2 = plate.sketches.itemByName("Outboard_screw_support_extension_profile")
    L.strip(sk2)
    c = L.move_points(sk2, lambda x, y: near(x, 190), dx=8)
    d = 0
    for skn in ("\u8349\u56fe1", "Q4_belt_and_pulley_clearance_profile"):   # Chinese-language Fusion names: Sketch1
        s = plate.sketches.itemByName(skn)
        L.strip(s)
        d += L.move_points(s, lambda x, y: near(x, 193), dx=8)
    log.append(f"rail plate front +8 ({a}+{c}+{d} points), -z edge -7 ({b} points)")

    probs = L.timeline_problems()
    log.append(f"timeline problems: {probs}")
    for n in ("D_Q2_Saddle", "D_Q2_Nut_bracket", "D_Q2_Ballscrew_12x10", "D_MGN12_Rail", "D_Q2_Rail_plate"):
        c = L.comp(n)
        v = sum(b.volume for b in c.bRepBodies) * 1000
        bb = c.bRepBodies.item(0).boundingBox
        log.append(f"{n}: bodies {c.bRepBodies.count}, volume {v:.0f} mm3, box "
                   f"({bb.minPoint.x*10:.1f},{bb.minPoint.y*10:.1f},{bb.minPoint.z*10:.1f})-"
                   f"({bb.maxPoint.x*10:.1f},{bb.maxPoint.y*10:.1f},{bb.maxPoint.z*10:.1f})")
    print("\n".join(log))
