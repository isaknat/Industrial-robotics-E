"""Third design, step 1: measure where every part sits. after CAD edition 2 (2026-11-24).

Run inside Fusion with the E_Writing model active (it is imported from
Parametric/E_Writing.f3d as the unsaved document "E_Writing_v3").

The robot is put in the reference pose q = (q1 0 deg, q2 80 mm, q3 0 deg, q4 0 deg), where the
frames of the sub-assemblies D_02 to D_05 are parallel to the world frame (Y up, X along the
rail towards the E, Z sideways), exactly as in joint_limits_fusion.py. For every part instance
the axis-aligned bounding box of its bodies is read and written to measure_layers.json:
  - for D_02 to D_05 relative to the q3 axis  (x along the rail, y up, z sideways, in mm);
  - for D_01 (the fixed base) relative to the q1 axis and the bucket floor (world, in mm).
The robot is put back where it was; nothing is saved.
"""
import json, os, sys, time
import adsk.core as core
import adsk.fusion as fusion

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import joint_limits_fusion as jl                  # Robot.place(), same pose code as PlayWriting

Q0 = [0.0, 80.0, 0.0, 0.0]
OUT = os.path.join(HERE, "measure_layers.json")


def box_mm(bodies, origin_cm):
    lo = [1e9] * 3
    hi = [-1e9] * 3
    for b in bodies:
        bb = b.boundingBox
        for i, (a, c) in enumerate(zip(bb.minPoint.asArray(), bb.maxPoint.asArray())):
            lo[i] = min(lo[i], a)
            hi[i] = max(hi[i], c)
    return [[round((lo[i] - origin_cm[i]) * 10, 2), round((hi[i] - origin_cm[i]) * 10, 2)] for i in range(3)]


def run(_context: str):
    r = jl.Robot()
    try:
        r.place(Q0)
        q3_axis = [Q0[1] / 10.0, jl.H_CM, 0.0]
        result = dict(note="after CAD edition 2; pose q = (0 deg, 80 mm, 0 deg, 0 deg); "
                           "x, y, z ranges in mm; D_01 relative to the q1 axis at the bucket floor, "
                           "D_02..D_05 relative to the q3 axis",
                      time=time.strftime("%Y-%m-%d %H:%M:%S"), parts=[])
        for grp in r.robot:
            g = grp.component.name[:4]
            origin = [0.0, 0.0, 0.0] if g == "D_01" else q3_axis
            for occ in grp.childOccurrences:
                bodies = []

                def walk(o):
                    bodies.extend(list(o.bRepBodies))
                    for c in o.childOccurrences:
                        walk(c)
                walk(occ)
                if not bodies:
                    continue
                x, y, z = box_mm(bodies, origin)
                result["parts"].append(dict(group=g, part=occ.component.name, instance=occ.name,
                                            x=x, y=y, z=z))
        with open(OUT, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=1)
        print(f"{len(result['parts'])} part instances written to {OUT}")
    finally:
        r.restore()
