"""Play the four measured writing drives on the joints of the active Fusion model."""
from bisect import bisect_right
import csv
import json
import math
from pathlib import Path
import time

import adsk
import adsk.core as core
import adsk.fusion as fusion

HERE = Path(__file__).resolve().parent


def trajectory(home):
    curves = []
    for filename in ["q1_yaw_rad.csv", "q2_slide_m.csv", "q3_shoulder_rad.csv", "q4_wrist_rad.csv"]:
        with (HERE / "curves" / filename).open(newline="") as stream:
            curves.append([tuple(map(float, row)) for row in csv.reader(stream)])
    times = [t for t, _ in curves[0]]
    if any([t for t, _ in curve] != times for curve in curves[1:]) or any(
            b <= a for a, b in zip(times, times[1:])):
        raise ValueError("The four curves must share one strictly increasing time axis.")
    poses = [tuple(h + entry[1] for h, entry in zip(home, sample)) for sample in zip(*curves)]
    return times, poses


def pose_at(t, times, poses):
    i = min(len(times)-2, max(0, bisect_right(times, t)-1))
    u = (t-times[i]) / (times[i+1]-times[i])
    return tuple(a+(b-a)*u for a,b in zip(poses[i], poses[i+1]))


def spinning_parts(root):
    """The ball screw, the pulleys and the shafts can turn inside their sub-assembly (joints and motion
    links). A part that is free inside a sub-assembly does not follow it when the sub-assembly is placed
    with transformOccurrences, so each of them is placed as well, turned by its gear ratio. The table is
    the attribute RPRR / SpinningParts of the design; without it nothing changes."""
    attr = root.attributes.itemByName("RPRR", "SpinningParts")
    if not attr:
        return None
    data = json.loads(attr.value)
    groups = {o.component.name[:4]: o for o in root.occurrences}
    parts = []
    for r in data["rotors"]:
        g = groups[r["group"]]
        native = {o.name: o for o in g.component.occurrences}
        seen = {o.name: o for o in g.childOccurrences}
        for n in r["parts"]:
            parts.append((seen[n], native[n].transform2.copy(), r["group"], r["point_cm"], r["axis"], r["coef"]))
    return parts, data["q0"], groups["D_01"]


def drive(root, moving, q, spin=None):
    yaw, slide, shoulder, wrist = q
    c, s = math.cos(yaw), math.sin(yaw)
    radius = slide*100
    arm_end = radius + 30*math.cos(shoulder)
    wrist_height = 22.5 + 30*math.sin(shoulder)
    targets = []
    for pitch, r, h in [(0, 0, 0), (0, radius, 22.5), (shoulder, radius, 22.5),
                         (shoulder+wrist, arm_end, wrist_height)]:
        cp, sp = math.cos(pitch), math.sin(pitch)
        matrix = core.Matrix3D.create()
        matrix.setWithArray([c*cp, -c*sp, s, r*c, sp, cp, 0, h,
                             -s*cp, s*sp, c, -r*s, 0, 0, 0, 1])
        targets.append(matrix)
    if spin:
        # Two calls: the four sub-assemblies first, then everything. When a sub-assembly and the free
        # parts inside it are moved in one call from a captured state, the free parts end up displaced
        # by the move of their sub-assembly (found by trial, 2026-11-26).
        if not root.transformOccurrences(moving, targets, False):
            raise RuntimeError("The four-axis pose could not be solved.")
        parts, q0, base = spin
        where = {"D_01": base.transform2, "D_02": targets[0], "D_03": targets[1], "D_04": targets[2],
                 "D_05": targets[3]}
        moving, targets = list(moving), list(targets)
        for occ, home, group, point, axis, coef in parts:
            angle = sum(k*(v-z) for k, v, z in zip(coef, q, q0))
            turn = core.Matrix3D.create()
            turn.setToRotation(angle, core.Vector3D.create(*axis), core.Point3D.create(*point))
            m = home.copy()
            m.transformBy(turn)
            m.transformBy(where[group])
            moving.append(occ)
            targets.append(m)
    if not root.transformOccurrences(moving, targets, False):
        raise RuntimeError("The four-axis pose could not be solved.")


def tip_error_mm(pen_head, q):
    point = core.Point3D.create(5, 0, 0)
    point.transformBy(pen_head.transform2)
    radius = q[1]*100 + 30*math.cos(q[2]) + 5*math.cos(q[2]+q[3])
    height = 22.5 + 30*math.sin(q[2]) + 5*math.sin(q[2]+q[3])
    expected = (radius*math.cos(q[0]), height, -radius*math.sin(q[0]))
    return 10*math.dist(point.asArray(), expected)


def play(render_dir=None, fps=20):
    app = core.Application.get()
    design = fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    attr = root.attributes.itemByName("RPRR", "Home")
    if not attr:
        raise ValueError("Open E_Writing.f3d before starting the writing demonstration.")
    track = root.attributes.itemByName("RPRR", "WritingTrackV3")
    if track:
        data = json.loads(track.value)
        times, poses = data["t"], data["q"]
    else:
        times, poses = trajectory(json.loads(attr.value))
    moving = sorted((o for o in root.occurrences if o.component.name[:4] in
                     ("D_02", "D_03", "D_04", "D_05")), key=lambda o: o.component.name)
    head = moving[-1]
    spin = spinning_parts(root)
    max_error = 0.0
    count = 0
    start = time.perf_counter()
    t = 0.0
    while True:
        q = pose_at(t, times, poses)
        drive(root, moving, q, spin)
        adsk.doEvents()
        err = tip_error_mm(head, q)
        if err > 0.05:
            raise ValueError(f"Pen tip differs from forward kinematics by {err:.4f} mm at {t:.4f} s")
        max_error = max(err, max_error)
        if render_dir is not None:
            path = Path(render_dir) / f"frame_{count:04}.png"
            if not app.activeViewport.saveAsImageFile(str(path), 960, 640):
                raise RuntimeError("Could not render " + str(path))
        else:
            app.activeViewport.refresh()
        count += 1
        if t >= times[-1]:
            break
        t = min(times[-1], count*times[-1]/round(times[-1]*fps)
                if render_dir is not None else time.perf_counter()-start)
    return dict(source_samples=len(times), duration_s=times[-1], displayed_frames=count,
                max_tcp_error_mm=max_error)


def run(context):
    play()
