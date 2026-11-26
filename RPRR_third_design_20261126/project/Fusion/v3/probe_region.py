"""Third design, step 1: which parts fill a region of space. after CAD edition 2 (2026-11-24).

Run inside Fusion with E_Writing_v3 active. The robot is put in a pose (default: the reference
pose q = 0 deg, 80 mm, 0 deg, 0 deg) and a grid of points around the q3 axis is tested against the
bodies of the chosen parts with BRepBody.pointContainment. For every z slice and every x column the
occupied y intervals are listed, per part. Coordinates in mm relative to the q3 axis (x along the
rail, y up, z sideways), the same frame as measure_layers.json. Writes probe_region.json.
The robot is put back; nothing is saved.
"""
import json, os, sys
import adsk.core as core
import adsk.fusion as fusion

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import joint_limits_fusion as jl

Q = [0.0, 80.0, 0.0, 0.0]
PARTS = None          # None = every part whose box reaches the region
X = (-44, 44, 2)      # mm, inclusive range and step
Y = (-46, 44, 2)
Z = [26, 30, 33, 36, 40, 44, 48, 52, 56, 59, 62]
OUT = os.path.join(HERE, "probe_region.json")


def rng(a, b, s):
    v = a
    while v <= b + 1e-9:
        yield v
        v += s


def run(_context: str):
    r = jl.Robot()
    try:
        r.place(Q)
        cx, cy = Q[1] / 10.0, jl.H_CM
        cand = []
        for grp in r.robot:
            for occ in grp.childOccurrences:
                bodies = []

                def walk(o):
                    bodies.extend(list(o.bRepBodies))
                    for c in o.childOccurrences:
                        walk(c)
                walk(occ)
                for b in bodies:
                    bb = b.boundingBox
                    lo = [(bb.minPoint.x - cx) * 10, (bb.minPoint.y - cy) * 10, bb.minPoint.z * 10]
                    hi = [(bb.maxPoint.x - cx) * 10, (bb.maxPoint.y - cy) * 10, bb.maxPoint.z * 10]
                    if hi[0] < X[0] or lo[0] > X[1] or hi[1] < Y[0] or lo[1] > Y[1] or hi[2] < Z[0] or lo[2] > Z[-1]:
                        continue
                    if PARTS and occ.component.name not in PARTS:
                        continue
                    cand.append((grp.component.name[:4] + ":" + occ.name, b, lo, hi))
        out = dict(pose=Q, note="after CAD edition 2; mm relative to the q3 axis", slices={})
        inside = adsk_inside = fusion.PointContainment.PointInsidePointContainment
        for z in Z:
            sl = {}
            for name, b, lo, hi in cand:
                if not lo[2] <= z <= hi[2]:
                    continue
                cols = {}
                for x in rng(*X):
                    if not lo[0] <= x <= hi[0]:
                        continue
                    ys = [y for y in rng(*Y) if lo[1] <= y <= hi[1] and
                          b.pointContainment(core.Point3D.create(cx + x / 10, cy + y / 10, z / 10)) == inside]
                    if ys:
                        # compress to intervals
                        iv, s0, prev = [], ys[0], ys[0]
                        for y in ys[1:]:
                            if y - prev > Y[2] + 1e-9:
                                iv.append([s0, prev]); s0 = y
                            prev = y
                        iv.append([s0, prev])
                        cols[x] = iv
                if cols:
                    sl[name] = cols
            out["slices"][str(z)] = sl
        with open(OUT, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False)
        print("candidates:", sorted(set(n for n, *_ in cand)))
    finally:
        r.restore()
