"""Step 7, second version: reference numbers from Fusion for checking the Simscape model. after CAD edition 1 (2026-11-24).

Run inside Fusion with E_Writing.f3d active. For a few test poses the four moving sub-assemblies are
put in place exactly as in PlayWriting.py / joint_limits_fusion.py; Fusion then reports, in the
assembly frame (Y up), each sub-assembly's mass, centre of mass and inertia tensor about the assembly
origin, plus where the q3 and q4 axes are. The robot is put back where it was; nothing is saved.
Output: step7_bodies/pose_check.json (SI units).

step7b_simscape_cad.m turns these into the gravity torques at q3 and q4 and the moment of inertia
about the q1 axis, and compares them with what the Simscape model computes in the same poses.
"""
import json, os, sys
import adsk.core as core
import adsk.fusion as fusion

sys.path.insert(0, os.path.dirname(__file__))
import joint_limits_fusion as jl          # same pose code (targets) as the joint-limit sweep

POSES = [[20.0, 60.0, 30.0, -60.0], [-45.0, 120.0, -40.0, 90.0], [100.0, 20.0, 50.0, 30.0]]   # [deg, mm, deg, deg]
OUT = os.path.join(os.path.dirname(__file__), "step7_bodies", "pose_check.json")


def tensor_m(p):
    _, xx, yy, zz, xy, yz, xz = p.getXYZMomentsOfInertia()       # about the assembly origin, kg cm^2
    return [[xx * 1e-4, xy * 1e-4, xz * 1e-4], [xy * 1e-4, yy * 1e-4, yz * 1e-4], [xz * 1e-4, yz * 1e-4, zz * 1e-4]]


def frame(o):
    a = o.transform2.asArray()
    return dict(origin_m=[a[3] / 100.0, a[7] / 100.0, a[11] / 100.0], z_axis=[a[2], a[6], a[10]])


def run(_context: str):
    r = jl.Robot()
    res = dict(note="assembly frame, Y up; inertia about the assembly origin, tensor convention", poses=[])
    try:
        for q in POSES:
            r.place(q)
            bodies = {}
            for o in r.moving:
                p = o.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
                bodies[o.component.name[:4]] = dict(mass=p.mass, com_m=[v / 100.0 for v in p.centerOfMass.asArray()],
                                                     I_origin=tensor_m(p))
            res["poses"].append(dict(q=q, bodies=bodies, q3_axis=frame(r.moving[1]), q4_axis=frame(r.moving[3])))
            print("pose", q, "done")
    finally:
        r.restore()
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1)
    print("written", OUT, "- robot put back")
