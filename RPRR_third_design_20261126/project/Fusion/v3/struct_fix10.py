"""Tenth round (after CAD edition 3, 2026-11-26): lock nut on the fixed end of the ball screw.

THK catalogue (A15-330 support unit EK, A15-322 recommended shaft end H1 for EK8, A15-353 lock nut RN 8):
  - the screw shoulder stands K3 = 3.5 mm in front of the housing      (model: 1 mm)
  - behind the shoulder: 8 mm journal with M8x1 thread at its end, E = 35 mm in all, then the
    drive end 6 mm x F = 9 mm for the coupling                            (model: 8 mm x 37 mm, no thread)
  - the lock nut stands L1 = 7 mm out of the housing on the motor side    (model: 1 mm between housing and coupling)
  - lock nut RN 8: M8x1, 17 mm round with 14 mm flats, 6.5 mm wide
So, along D_02 x (shoulder at -14):
  housing -40.5 ... -17.5 (was -38 ... -15), lock nut -47.5 ... -40.5, journal ends at -49,
  drive end -58 ... -49, coupling 8x6 -75.5 ... -49.5, motor shaft tip -62.5, motor face -87.5 (was -77).
The motor foot stays where it is (its two M5 holes match the rail plate); its upright and deck move back.
Run inside Fusion on E_Writing_q3_50W_build:  locknut()
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
import struct_fix9 as S9
for _m in (L, M, N, S9):
    importlib.reload(_m)

say, done, healthy, volume = S9.say, S9.done, S9.healthy, S9.volume
FLOOR, ALONG_X = S9.FLOOR, S9.ALONG_X
AXIS_Y, AXIS_Z = S9.AXIS_Y, S9.AXIS_Z
HOUSING_BACK = 2.5        # K3 3.5 instead of 1
MOTOR_BACK = 10.5         # motor face -77 -> -87.5


def at(name, x_mm):
    """Parts of that kind in D_02 whose origin is at the given x."""
    return [o for o in L.group('D_02').component.occurrences
            if o.component.name == name and abs(o.transform2.translation.x * 10 - x_mm) < 0.05]


def replace(name, old_x, new_pos, axes, count=1, new_name=None):
    """A new instance at the new place, then the old one is deleted (nothing is moved)."""
    old = at(name, old_x)
    assert len(old) == count, (name, old_x, len(old))
    comp = L.comp(new_name or name)
    for o in old:
        t = o.transform2.translation
        pos = new_pos(t.x * 10, t.y * 10, t.z * 10)
        M.place('D_02', comp, pos, axes)
    for o in old:
        o.deleteMe()


def locknut():
    d = S9.guard()
    # ---- housing 2.5 mm back, with its riser plate and its two screws; the two threads in the plate follow
    back = lambda dx: (lambda x, y, z: (x - dx, y, z))
    replace('D_EK08_Fixed_support', -38.0, back(HOUSING_BACK), ALONG_X)
    replace('D_Q2_Support_riser', -26.5, back(HOUSING_BACK), FLOOR)
    replace('D_M6x25_socket_screw', -26.5, back(HOUSING_BACK), FLOOR, count=2)
    plate = L.comp('D_Q2_Rail_plate')
    sk = plate.sketches.itemByName('Support_M6_threads_profile')
    moved = 0
    for c in sk.sketchCurves.sketchCircles:
        p = sk.sketchToModelSpace(c.centerSketchPoint.geometry)
        if abs(p.x * 10 + 26.5) < 0.01:
            q = sk.modelToSketchSpace(core.Point3D.create(p.x - HOUSING_BACK / 10, p.y, p.z))
            g = c.centerSketchPoint.geometry
            c.centerSketchPoint.move(core.Vector3D.create(q.x - g.x, q.y - g.y, 0))
            moved += 1
    assert moved == 2, moved
    # ---- screw: drive end 6 x 9 behind the 35 mm journal (screw frame: z = D_02 x + 52)
    sc = L.comp('D_Q2_Ballscrew_12x10')
    M.disk(sc, 'Drive_end_6', -6, 9, 3.0, op=M.JOIN)                 # z -6..3
    M.disk(sc, 'Drive_end_step', 0, 3, 4.5, 3.0, op=M.CUT)           # journal now ends at z 3 (x -49)
    b = sc.bRepBodies.item(0).boundingBox
    assert sc.bRepBodies.count == 1 and abs(b.minPoint.z * 10 + 6) < 0.01, (sc.bRepBodies.count, b.minPoint.z)
    # ---- motor foot: stays; upright and the rear edge of its deck 10.5 mm back
    foot = L.comp('D_Q2_Motor_foot')
    L.set_start_offset(foot.features.extrudeFeatures.itemByName('Motor_flange_upright'), 2.0 - MOTOR_BACK)
    for name in ('Foot_profile', 'Deck_mount_foot_profile'):
        s = foot.sketches.itemByName(name)
        L.strip(s)
        n = L.move_points(s, lambda x, y: abs(x + 5.0) < 0.01, dx=-MOTOR_BACK)
        assert n == 2, (name, n)
    d.computeAll()
    for _ in range(6):                                               # edge breaks that lost their edges
        sick = [d.timeline.item(i) for i in range(d.timeline.count)
                if d.timeline.item(i).healthState != fusion.FeatureHealthStates.HealthyFeatureHealthState
                and d.timeline.item(i).name.startswith(('Edge_break_R', 'Junction_R'))]
        if not sick:
            break
        say(f'removed after recompute: {sick[-1].name} ({sick[-1].errorOrWarningMessage})')
        sick[-1].entity.deleteMe()
        d.computeAll()
    assert not healthy(d), healthy(d)
    fo = at('D_Q2_Motor_foot', -74.0)[0]
    fb = fo.bRepBodies.item(0).boundingBox
    say(f'motor foot now x {fb.minPoint.x * 10:.1f} .. {fb.maxPoint.x * 10:.1f}')
    # ---- flange plate, motor, coupling (8x6 instead of 8x8: the drive end is 6 mm)
    replace('D_Motor_flange', -77.0, back(MOTOR_BACK), ALONG_X)
    replace('D_HG_KR053', -77.0, back(MOTOR_BACK), ALONG_X)
    replace('D_XGT2_19C_8x8', -65.0, lambda x, y, z: (-75.5, y, z), ALONG_X, new_name='D_XGT2_19C_8x6')
    # ---- lock nut RN 8, as a part inside the screw so that it turns with it
    m = core.Matrix3D.create()
    m.translation = core.Vector3D.create(0, 0, 0.45)                 # screw z 4.5..11.5 = D_02 x -47.5..-40.5
    no = sc.occurrences.addNewComponent(m)
    nut = no.component
    nut.name = 'D_RN8_Lock_nut'
    nut.material = L.material(M.STEEL)
    s = M.sketch(nut, 'Flats_14_round_17')
    h = math.sqrt(8.5 ** 2 - 7.0 ** 2)
    ln, ar = s.sketchCurves.sketchLines, s.sketchCurves.sketchArcs
    for k in range(4):
        a = math.radians(90 * k)
        rot = lambda x, y: M.P(x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a))
        ln.addByTwoPoints(rot(7.0, -h), rot(7.0, h))
        ar.addByCenterStartSweep(M.P(0, 0), rot(7.0, h), 2 * (math.pi / 4 - math.atan2(h, 7.0)))
    L.circle(s, 0, 0, 4.0)
    ring = [p for p in s.profiles if p.profileLoops.count == 2]
    assert len(ring) == 1, [p.profileLoops.count for p in s.profiles]
    L.extrude(nut, ring[0], 4.0, 0.0, M.NEW, 'Body_with_flats')     # motor side
    s.isVisible = False
    M.disk(nut, 'Step_13', 4, 3, 6.5, 4.0, op=M.JOIN)                # towards the housing (stands for the 0.5 mm collar gap too)
    assert nut.bRepBodies.count == 1
    no.isGroundToParent = True
    nut.bRepBodies.item(0).appearance = d.appearances.itemByName(S9.BLACK_OXIDE)
    m_nut = nut.physicalProperties.mass * 1000
    rho = S9.keep_mass(L.comp('D_EK08_Fixed_support'), 240.0 - m_nut)
    say(f'lock nut {m_nut:.1f} g; EK08 housing density {rho:.0f} kg/m3 -> {240 - m_nut:.1f} g (unit with nut 240 g)')
    d.computeAll()
    hits = S9.interference_pairs(d)
    say(f'interference at the reference pose: {hits}')
    assert not hits, hits
    turn, off, tilt = S9.screw_state()
    assert off < 1e-4 and tilt < 1e-4, (off, tilt)
    done(d, ['D_Q2_Ballscrew_12x10', 'D_RN8_Lock_nut', 'D_Q2_Motor_foot', 'D_EK08_Fixed_support', 'D_Q2_Rail_plate'])
    say('LOCKNUT DONE')
