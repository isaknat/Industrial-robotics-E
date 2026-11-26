"""Model-level checks of the current working model (after CAD edition 3, 2026-11-26), before the MATLAB steps.

Run inside Fusion on E_Writing_q3_50W_build:
  static()        timeline, interference at the reference pose, every part touches another one, masses
  export()        step7_bodies/ for MATLAB (same code as edition3.export_bodies) and the pose check
  trajectory(preview)   the writing path against the robot itself and the bucket (trajectory_verify.py);
                  the files of 2026-11-24 are kept under *_20261124.jsonl
  limits()        single-joint sweeps as in joint_limits_fusion.py, written to joint_limits_20261126.jsonl
Everything outlasts one MCP call, so progress goes to model_checks_log.txt.
"""
import importlib
import json
import math
import runpy
import shutil
import sys
import time
from pathlib import Path
import adsk.core as core
import adsk.fusion as fusion

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cadlib as L
import struct_fix9 as S9
import struct_fix11 as S11
for _m in (L, S9, S11):
    importlib.reload(_m)
LOG = HERE / 'model_checks_log.txt'


def say(text):
    with open(LOG, 'a', encoding='utf8') as f:
        f.write(time.strftime('%H:%M:%S ') + text + '\n')
    print(text)


def at_reference(d):
    """Judged from where the parts are, not from the joint read-outs: after the robot has been placed
    directly (playback, checks) a joint can show an old value until the design is computed again
    (seen: q2 showed -11.17 mm with the slider exactly at its reference place)."""
    root = d.rootComponent
    want = {'D_02': (0, 0, 0), 'D_03': (8.0, 22.5, 0), 'D_04': (8.0, 22.5, 0), 'D_05': (38.0, 22.5, 0)}
    err = 0.0
    for o in root.occurrences:
        k = o.component.name[:4]
        if k in want:
            a = o.transform2.asArray()
            ideal = [1, 0, 0, want[k][0], 0, 1, 0, want[k][1], 0, 0, 1, want[k][2]]
            err = max(err, max(abs(x - y) for x, y in zip(a[:12], ideal)))
    rot = {k: S11.state(k) for k in S11.ROTORS}
    sc = S9.screw_state()
    ok = err < 1e-9 and all(abs(t) < 1e-6 and dev < 1e-4 for t, dev in rot.values()) and abs(sc[0]) < 1e-6 and sc[1] < 1e-4         and not d.snapshots.hasPendingSnapshot
    vals = {j.name: (j.jointMotion.slideValue * 10 if j.jointMotion.jointType == 2 else math.degrees(j.jointMotion.rotationValue))
            for j in root.asBuiltJoints}
    return ok, vals


def static():
    d = S9.guard()
    root = d.rootComponent
    out = {}
    ok, vals = at_reference(d)
    out['at_reference'] = ok
    out['joint_values'] = vals
    d.computeAll()
    out['timeline'] = d.timeline.count
    out['not_healthy'] = S9.healthy(d)
    out['interference_reference'] = S9.interference_pairs(d)
    # every part touches at least one other part
    items = []
    for o in root.allOccurrences:
        if o.bRepBodies.count and not o.component.name.startswith('Wave_Drum'):
            bodies = [b for b in o.bRepBodies if b.isSolid]
            lo, hi = [1e9] * 3, [-1e9] * 3
            for b in bodies:
                bb = b.boundingBox
                for i, v in enumerate(bb.minPoint.asArray()):
                    lo[i] = min(lo[i], v)
                for i, v in enumerate(bb.maxPoint.asArray()):
                    hi[i] = max(hi[i], v)
            items.append((o.fullPathName, o.component.name, bodies, lo, hi))
    mm = core.Application.get().measureManager
    eps = 0.005                       # cm
    alone, n_meas = [], 0
    for i, (path, name, bodies, lo, hi) in enumerate(items):
        touching = False
        for j, (path2, name2, bodies2, lo2, hi2) in enumerate(items):
            if i == j or any(lo[k] > hi2[k] + eps or lo2[k] > hi[k] + eps for k in range(3)):
                continue
            for b in bodies:
                for b2 in bodies2:
                    n_meas += 1
                    if mm.measureMinimumDistance(b, b2).value < 1e-5:
                        touching = True
                        break
                if touching:
                    break
            if touching:
                break
        if not touching:
            alone.append(path)
    out['parts_touching_nothing'] = alone
    out['instances'] = len(items)
    out['kinds'] = len({n for _, n, _, _, _ in items})
    out['masses_kg'] = {o.component.name[:4]: round(o.getPhysicalProperties(fusion.CalculationAccuracy.HighCalculationAccuracy).mass, 4)
                        for o in root.occurrences}
    out['spinning_table'] = bool(root.attributes.itemByName('RPRR', 'SpinningParts'))
    out['joints'] = [j.name for j in root.asBuiltJoints] + [j.name for g in root.occurrences for j in g.component.asBuiltJoints]
    out['motion_links'] = [(m.name, m.healthState) for m in root.motionLinks]
    (HERE / 'model_static.json').write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding='utf8')
    say(f'static: at reference {ok}; timeline {out["timeline"]}, not healthy {out["not_healthy"]}; interference {out["interference_reference"]}; '
        f'touching nothing {alone} ({n_meas} distance measurements); {out["kinds"]} kinds, {out["instances"]} instances; masses {out["masses_kg"]}')
    say('STATIC DONE')


def export():
    d = S9.guard()
    ok, vals = at_reference(d)
    assert ok, vals
    p = runpy.run_path(str(HERE.parent / 'step7_export_bodies.py'))['run']
    p.__globals__.update(OUT=str(HERE / 'step7_bodies'),
                         SPIN=p.__globals__['SPIN'] + ('D_Q4_Transfer_pulley', 'D_Q1_Stage1_idler', 'D_Q3_First_stage_idler', 'D_Q4_Long_idler'))
    p('')
    say('export: step7_bodies written')
    check = runpy.run_path(str(HERE.parent / 'step7_pose_check.py'))['run']
    check.__globals__['OUT'] = str(HERE / 'step7_bodies' / 'pose_check.json')
    check('')
    ok, vals = at_reference(d)
    say(f'export: pose_check written; back at reference {ok}')
    say('EXPORT DONE')


def trajectory(preview=True):
    d = S9.guard()
    for name in ('trajectory_check.jsonl', 'trajectory_preview.jsonl', 'trajectory_verified.jsonl'):
        keep = HERE / name.replace('.jsonl', '_20261124.jsonl')
        if (HERE / name).exists() and not keep.exists():
            shutil.copy2(HERE / name, keep)
    t0 = time.time()
    run = runpy.run_path(str(HERE / 'trajectory_verify.py'))['run']
    run(preview, d)
    ok, vals = at_reference(d)
    rows = [json.loads(s) for s in (HERE / ('trajectory_preview.jsonl' if preview else 'trajectory_check.jsonl')).read_text(encoding='utf8').splitlines()]
    own = [r['index'] for r in rows if r['robot_interferences_mm3']]
    drum = {}
    for r in rows:
        for k in r['drum_contact']:
            drum[k] = drum.get(k, 0) + 1
    say(f'trajectory ({"preview" if preview else "full"}): {len(rows)} poses in {time.time() - t0:.0f} s; poses with robot interference {own}; '
        f'parts at zero distance from the bucket (number of poses) {drum}; smallest clearance bound of the moving groups '
        f'{min(r["moving_clearance_lower_bound_mm"] for r in rows)} mm; back at reference {ok}')
    say('TRAJECTORY %s DONE' % ('PREVIEW' if preview else 'FULL'))


def limits(jobs=((1, -1), (1, 1), (2, -1), (2, 1), (3, -1), (3, 1), (0, -1), (0, 1)), q1_step=4.0):
    d = S9.guard()
    jl = runpy.run_path(str(HERE.parent / 'joint_limits_fusion.py'))
    jl['STEP'][0] = q1_step
    log = HERE / 'joint_limits_20261126.jsonl'
    r = jl['Robot'](d)
    try:
        for j, sign in jobs:
            t0 = time.time()
            try:
                res = jl['sweep'](r, j, sign)
            except Exception:
                import traceback
                side = 'lower' if sign < 0 else 'upper'
                say('limit q%d %s FAILED after %.0f s: %s' % (j + 1, side, time.time() - t0, traceback.format_exc()))
                raise
            res.update(joint=f'q{j + 1}', side='lower' if sign < 0 else 'upper', seconds=round(time.time() - t0, 1))
            with open(log, 'a', encoding='utf8') as f:
                f.write(json.dumps(res, ensure_ascii=False) + '\n')
            say(f'limit {res["joint"]} {res["side"]}: {res["limit"]} (first collision {res.get("first_collision")}, {res.get("hit")}, {res["checks"]} checks, {res["seconds"]} s)')
    finally:
        r.restore()
    ok, vals = at_reference(d)
    say(f'limits done; back at reference {ok}; poses analysed in pieces because the whole-robot analysis gave no result: {getattr(r, "fallbacks", [])}')
    say('LIMITS DONE')


def probe_q1(values=(52, 56, 60, 64, 68, 60.01, 60)):
    """Does the interference analysis give a result at these q1 angles? (It returned nothing once, in the upper q1 sweep.)"""
    d = S9.guard()
    jl = runpy.run_path(str(HERE.parent / 'joint_limits_fusion.py'))
    R = jl['Robot'](d)

    def analyse():
        coll = core.ObjectCollection.create()
        for b in R.bodies:
            coll.add(b)
        inp = d.createInterferenceInput(coll)
        inp.areCoincidentFacesIncluded = False
        return d.analyzeInterference(inp)
    try:
        for q1 in values:
            R.place([q1, 80, 0, 0])
            t0 = time.time()
            res = analyse()
            line = f'q1 {q1}: result {None if res is None else res.count} in {time.time() - t0:.1f} s'
            if res is None:
                res2 = analyse()
                line += f'; second try {None if res2 is None else res2.count}'
            say(line)
    finally:
        R.restore()
    say('PROBE DONE')


def probe_pairs(q=(56, 80, 0, 0)):
    """At a pose where the whole-robot analysis returns nothing: which parts are involved?
    Each turning part is analysed against its sub-assembly on its own; a pair that fails is then measured."""
    d = S9.guard()
    root = d.rootComponent
    jl = runpy.run_path(str(HERE.parent / 'joint_limits_fusion.py'))
    R = jl['Robot'](d)
    mm = core.Application.get().measureManager

    def analyse(bodies):
        coll = core.ObjectCollection.create()
        for b in bodies:
            coll.add(b)
        inp = d.createInterferenceInput(coll)
        inp.areCoincidentFacesIncluded = False
        return d.analyzeInterference(inp)
    try:
        R.place(list(q))
        groups = {o.component.name[:4]: o for o in root.occurrences}
        by_group = {}
        for k, g in groups.items():
            if k.startswith('D_0'):
                by_group[k] = [(o.name, b) for o in g.childOccurrences for b in o.bRepBodies]
        for k, items in by_group.items():
            res = analyse([b for _, b in items])
            say(f'{k} alone ({len(items)} bodies): {None if res is None else res.count}')
        res = analyse([b for k in ('D_01', 'D_02') for _, b in by_group[k]])
        say(f'D_01 + D_02: {None if res is None else res.count}')
        bad = []
        for key, r in S11.ROTORS.items():
            if r['group'] != 'D_01':
                continue
            for name in r['parts']:
                mine = [b for n, b in by_group['D_01'] if n == name]
                for other_name, ob in by_group['D_01'] + by_group['D_02']:
                    if other_name == name:
                        continue
                    bb1, bb2 = mine[0].boundingBox, ob.boundingBox
                    if any(bb1.minPoint.asArray()[i] > bb2.maxPoint.asArray()[i] + 0.01 or bb2.minPoint.asArray()[i] > bb1.maxPoint.asArray()[i] + 0.01 for i in range(3)):
                        continue
                    res = analyse(mine + [ob])
                    if res is None:
                        dist = mm.measureMinimumDistance(mine[0], ob).value * 10
                        bad.append((name, other_name, dist))
                        say(f'   no result for {name} | {other_name}; smallest distance {dist:.5f} mm')
        say(f'pairs without a result at {list(q)}: {len(bad)}')
    finally:
        R.restore()
    say('PAIR PROBE DONE')


def probe_groups(q=(56, 80, 0, 0)):
    d = S9.guard()
    root = d.rootComponent
    jl = runpy.run_path(str(HERE.parent / 'joint_limits_fusion.py'))
    R = jl['Robot'](d)
    mm = core.Application.get().measureManager

    def analyse(bodies):
        coll = core.ObjectCollection.create()
        for b in bodies:
            coll.add(b)
        inp = d.createInterferenceInput(coll)
        inp.areCoincidentFacesIncluded = False
        return d.analyzeInterference(inp)

    def walk(occ, out):
        for b in occ.bRepBodies:
            out.append((occ.fullPathName.split('+', 1)[-1], b))
        for c in occ.childOccurrences:
            walk(c, out)
    try:
        R.place(list(q))
        by = {}
        for o in root.occurrences:
            k = o.component.name[:4]
            if k.startswith('D_0'):
                by[k] = []
                walk(o, by[k])
        keys = sorted(by)
        failing = []
        for i, a in enumerate(keys):
            for b in keys[i + 1:]:
                res = analyse([x for _, x in by[a] + by[b]])
                say(f'{a} + {b}: {None if res is None else res.count}')
                if res is None:
                    failing.append((a, b))
        for a, b in failing:
            # narrow down: each body of the smaller group against the whole other group
            small, large = (a, b) if len(by[a]) <= len(by[b]) else (b, a)
            for name, body in by[small]:
                res = analyse([body] + [x for _, x in by[large]])
                if res is None:
                    say(f'   {small}:{name} against all of {large}: no result')
                    for name2, body2 in by[large]:
                        r2 = analyse([body, body2])
                        if r2 is None:
                            say(f'      with {name2}: no result; smallest distance {mm.measureMinimumDistance(body, body2).value * 10:.5f} mm')
    finally:
        R.restore()
    say('GROUP PROBE DONE')


def probe_pose(poses=((168, 80, 0, 0), (164, 80, 0, 0), (172, 80, 0, 0), (-168, 80, 0, 0))):
    """Is everything where it should be at these poses? Group transforms against the targets, turning parts,
    and the distance between the q2 motor foot and each motor flange (rigid with it or with the slider)."""
    d = S9.guard()
    root = d.rootComponent
    jl = runpy.run_path(str(HERE.parent / 'joint_limits_fusion.py'))
    R = jl['Robot'](d)
    mm = core.Application.get().measureManager
    g2, g3 = L.group('D_02'), L.group('D_03')
    foot = next(o for o in g2.childOccurrences if o.component.name == 'D_Q2_Motor_foot')
    flanges = [o for g in (g2, g3) for o in g.childOccurrences if o.component.name == 'D_Motor_flange']

    def report(tag, q):
        tg = jl['targets'](list(q))
        err = [max(abs(a - b) for a, b in zip(o.transform2.asArray(), t.asArray())) for o, t in zip(R.moving, tg)]
        dist = {o.fullPathName[:4] + ':' + o.name: round(mm.measureMinimumDistance(foot.bRepBodies.item(0), o.bRepBodies.item(0)).value * 10, 4) for o in flanges}
        rot = max(S11.state(k)[1] for k in S11.ROTORS)
        sc = S9.screw_state()
        say(f'{tag} {list(q)}: groups off target {[round(e, 6) for e in err]}; rotors deviation {rot:.5f} mm; screw off axis {sc[1]:.5f}; foot to flanges {dist}')
    report('reference', (0, 80, 0, 0))
    try:
        for q in poses:
            c = R.collisions(list(q))
            report(f'collisions {c} at', q)
    finally:
        R.restore()
    say('POSE PROBE DONE')


def q1_rest(values=(168, 170, 174, 176, 178, 180)):
    """The upper q1 sweep stopped at 168 deg on a hit between the q2 motor foot and a motor flange. Both
    are fixed in the same sub-assembly or in the slider, which does not move against D_02 when only q1
    turns, so the hit cannot be real. Shown here: their relative position at 168 deg is the reference one,
    the two bodies alone give no interference, and the remaining angles up to 180 deg are free."""
    d = S9.guard()
    jl = runpy.run_path(str(HERE.parent / 'joint_limits_fusion.py'))
    R = jl['Robot'](d)
    g2, g3 = L.group('D_02'), L.group('D_03')
    foot = next(o for o in g2.childOccurrences if o.component.name == 'D_Q2_Motor_foot')
    flanges = [o for g in (g2, g3) for o in g.childOccurrences if o.component.name == 'D_Motor_flange']

    def relative():
        out = []
        inv = foot.transform2.copy()
        inv.invert()
        for o in flanges:
            m = o.transform2.copy()
            m.transformBy(inv)
            out.append(m.asArray())
        return out
    ref = relative()
    res = {}
    try:
        for q1 in values:
            c = R.collisions([q1, 80, 0, 0])
            now = relative()
            drift = max(abs(a - b) for x, y in zip(ref, now) for a, b in zip(x, y))
            alone = {o.fullPathName[:4]: R._analyse([foot.bRepBodies.item(0), o.bRepBodies.item(0)]) for o in flanges} if c else None
            res[q1] = c
            say(f'q1 {q1}: collisions {c}; flanges moved against the foot by {drift:.2e} (matrix entries, cm); the foot with each flange alone: {alone}')
    finally:
        R.restore()
    with open(HERE / 'joint_limits_20261126.jsonl', 'a', encoding='utf8') as f:
        f.write(json.dumps(dict(joint='q1', side='upper', note='168 deg hit is between two parts that are rigid with each other; rest checked one by one',
                                poses={str(k): v for k, v in res.items()})) + '\n')
    say('Q1 REST DONE')
