"""Joint limits from the Fusion model, option A (chosen 2026-11-24). after CAD edition 1.

Run inside Fusion (through the Fusion MCP "script" tool, or Scripts and Add-Ins) with
E_Writing.f3d or RPRR_Robot.f3d active. Each joint is moved alone from the reference pose
q0 = (q1 0 deg, q2 80 mm, q3 0 deg, q4 0 deg) in steps of 2 deg / 2 mm until two bodies of
different sub-assemblies interfere; the edge is then found by bisection to 0.1 deg / 0.1 mm.
Combinations of several joints are not checked (that would be option B).

The poses are set the same way as PlayWriting.py (Root.transformOccurrences, same frames:
Y up, q1 about +Y with zero towards +X = middle of the E, q3 and q4 positive = up). The four
moving sub-assemblies are put back where they were at the end; nothing is saved.
Ignored: pairs inside one sub-assembly and the three belts drawn overlapping the teeth of the
pulley they drive in the next sub-assembly (present in every pose).

Set JOBS (list of (joint index 0..3, sign)) and TEST before running. Prints one JSON line per job.
"""
import json, math, time
import adsk.core as core
import adsk.fusion as fusion

H_CM, L3_CM = 22.5, 30.0
Q0 = [0.0, 80.0, 0.0, 0.0]                     # [deg, mm, deg, deg]
STEP = [2.0, 2.0, 2.0, 2.0]
TOL = [0.1, 0.1, 0.1, 0.1]
BOUND = [(-180, 180), (0, 260), (-180, 180), (-180, 180)]
FUNCTIONAL = {frozenset(("D_Belt_Q1_stage2", "D_GT3M_120T_W15_D40")),
              frozenset(("D_Belt_Q3_stage2", "D_GT3M_120T_W15_D15")),
              frozenset(("D_Belt_Q4_stage2", "D_GT3M_40T_W9_D8"))}
JOBS = []          # e.g. [(1, -1), (1, +1)]
TEST = True
TRAJ = None        # path of a JSON file {"t": [...], "q": [[deg, mm, deg, deg], ...]} to check pose by pose


def targets(q):
    yaw, radius = math.radians(q[0]), q[1] / 10.0
    sh, wr = math.radians(q[2]), math.radians(q[3])
    c, s = math.cos(yaw), math.sin(yaw)
    arm_end = radius + L3_CM * math.cos(sh)
    wrist_h = H_CM + L3_CM * math.sin(sh)
    out = []
    for pitch, r, h in [(0, 0, 0), (0, radius, H_CM), (sh, radius, H_CM), (sh + wr, arm_end, wrist_h)]:
        cp, sp = math.cos(pitch), math.sin(pitch)
        m = core.Matrix3D.create()
        m.setWithArray([c * cp, -c * sp, s, r * c, sp, cp, 0, h, -s * cp, s * sp, c, -r * s, 0, 0, 0, 1])
        out.append(m)
    return out


def spinning_parts(root):
    """Since 2026-11-26 the ball screw, the pulleys and the shafts can turn inside their sub-assembly
    (joints and motion links, see v3/struct_fix9.py and v3/struct_fix11.py). A part that is free inside
    a sub-assembly does not follow it when the sub-assembly is placed with transformOccurrences, so
    each of them is placed as well. The table is the attribute RPRR / SpinningParts of the design.
    Returns (list of parts, q0); the list is empty for a model without that attribute."""
    attr = root.attributes.itemByName("RPRR", "SpinningParts")
    if not attr:
        return [], None
    data = json.loads(attr.value)
    groups = {o.component.name[:4]: o for o in root.occurrences}
    out = []
    for r in data["rotors"]:
        g = groups[r["group"]]
        native = {o.name: o for o in g.component.occurrences}
        seen = {o.name: o for o in g.childOccurrences}
        for n in r["parts"]:
            out.append(dict(occ=seen[n], home=native[n].transform2.copy(), group=r["group"],
                            point=r["point_cm"], axis=r["axis"], coef=r["coef"]))
    return out, data["q0"]


def spin_target(part, q0, group_target, q_si):
    """Home position in the sub-assembly, turned about the rotor axis, then carried by the sub-assembly.
    q_si = (q1 rad, q2 m, q3 rad, q4 rad)."""
    angle = sum(c * (q - z) for c, q, z in zip(part["coef"], q_si, q0))
    turn = core.Matrix3D.create()
    turn.setToRotation(angle, core.Vector3D.create(*part["axis"]), core.Point3D.create(*part["point"]))
    m = part["home"].copy()
    m.transformBy(turn)
    m.transformBy(group_target)
    return m


class Robot:
    def __init__(self, design=None):
        app = core.Application.get()
        self.design = design if design is not None else fusion.Design.cast(app.activeProduct)
        self.root = self.design.rootComponent
        occs = {o.component.name[:4]: o for o in self.root.occurrences}
        self.moving = [occs[k] for k in ("D_02", "D_03", "D_04", "D_05")]
        self.robot = [occs[k] for k in ("D_01", "D_02", "D_03", "D_04", "D_05")]
        self.saved = [o.transform2.copy() for o in self.moving]
        self.base = occs["D_01"]
        self.spin, self.spin_q0 = spinning_parts(self.root)         # kept apart: moving and saved stay four long
        self.spin_saved = [p["occ"].transform2.copy() for p in self.spin]
        self.bodies, groups = [], {}

        self.body_group = []                       # sub-assembly of each body, same order as self.bodies

        def walk(occ, g):
            for b in occ.bRepBodies:
                self.bodies.append(b)
                self.body_group.append(g)
                groups.setdefault(b.parentComponent.name, set()).add(g)
            for c in occ.childOccurrences:
                walk(c, g)
        for o in self.robot:
            walk(o, o.component.name[:4])
        self.group_of = {k: "/".join(sorted(v)) for k, v in groups.items()}

    def place(self, q):
        occs, t = list(self.moving), targets(q)
        if self.spin:
            # Two calls: the four sub-assemblies first, then everything. Found by trial (2026-11-26):
            # when a sub-assembly and the free parts inside it are moved in ONE call from a captured
            # (not pending) state, the free parts end up displaced by the move of their sub-assembly;
            # with the sub-assembly already in place they land exactly.
            if not self.root.transformOccurrences(occs, t, False):
                raise RuntimeError(f"pose {q} could not be set")
            where = {"D_01": self.base.transform2, "D_02": t[0], "D_03": t[1], "D_04": t[2], "D_05": t[3]}
            q_si = (math.radians(q[0]), q[1] / 1000.0, math.radians(q[2]), math.radians(q[3]))
            for p in self.spin:
                occs.append(p["occ"])
                t.append(spin_target(p, self.spin_q0, where[p["group"]], q_si))
        if not self.root.transformOccurrences(occs, t, False):
            raise RuntimeError(f"pose {q} could not be set")

    def collisions(self, q):
        # The bodies (not the occurrences) must be given: with occurrences Fusion reports nothing.
        # Inside one sub-assembly nothing interferes in the reference pose, and the sub-assemblies
        # are rigid, so every interference found is between two sub-assemblies. The results come
        # back as the parts' own bodies (no assembly context), so the sub-assembly is looked up by
        # part name, only for the report; parts used in several sub-assemblies list all of them.
        self.place(q)
        pairs = self._analyse(self.bodies)
        if pairs is None:
            # Fusion gave no result for the whole robot (seen 2026-11-26 at q1 = 56 deg, every time, while
            # 52, 60 and 56.01 deg were fine; no two parts are close there). First the same question is
            # asked in pieces; if a piece gives no result either, the pose is moved by 0.01 deg / 0.01 mm.
            self.fallbacks = getattr(self, "fallbacks", [])
            pairs = self._pieces()
            how = "pieces"
            if pairs is None:
                for dq in ((0.01, 0, 0, 0), (0, 0.01, 0, 0), (0, 0, 0.01, 0), (0, 0, 0, 0.01), (-0.01, 0, 0, 0), (0.01, 0.01, 0.01, 0.01)):
                    self.place([x + y for x, y in zip(q, dq)])
                    pairs = self._analyse(self.bodies)
                    if pairs is not None:
                        how = f"moved by {dq}"
                        break
                else:
                    raise RuntimeError(f"no interference result at {q}, also not nearby")
            self.fallbacks.append((list(q), how))
        return pairs

    def _pieces(self):
        """Each sub-assembly alone and each pair of sub-assemblies; what a pair adds to its two members
        is between them. None if one of these analyses gives no result."""
        names = sorted(set(self.body_group))
        part = {g: [b for b, k in zip(self.bodies, self.body_group) if k == g] for g in names}
        single = {g: self._analyse(part[g]) for g in names}
        if any(v is None for v in single.values()):
            return None
        pairs = {}
        for v in single.values():
            for k, x in v.items():
                pairs[k] = pairs.get(k, 0.0) + x
        for i, ga in enumerate(names):
            for gb in names[i + 1:]:
                both = self._analyse(part[ga] + part[gb])
                if both is None:
                    return None
                for k, x in both.items():
                    cross = x - single[ga].get(k, 0.0) - single[gb].get(k, 0.0)
                    if cross > 1e-9 or (x == 0.0 and k not in single[ga] and k not in single[gb]):
                        pairs[k] = pairs.get(k, 0.0) + max(cross, 0.0)
        return pairs

    def _analyse(self, bodies):
        """{pair of parts: overlap in mm^3} for these bodies, or None if Fusion returns no result."""
        coll = core.ObjectCollection.create()
        for b in bodies:
            coll.add(b)
        inp = self.design.createInterferenceInput(coll)
        inp.areCoincidentFacesIncluded = False
        res = self.design.analyzeInterference(inp)
        if res is None:
            return None
        pairs = {}
        for i in range(res.count):
            r = res.item(i)
            na, nb = r.entityOne.parentComponent.name, r.entityTwo.parentComponent.name
            if frozenset((na, nb)) in FUNCTIONAL:
                continue
            key = " | ".join(sorted((f"{self.group_of.get(na, '?')}:{na}", f"{self.group_of.get(nb, '?')}:{nb}")))
            try:
                vol = r.interferenceBody.volume * 1000.0                       # cm^3 -> mm^3
            except RuntimeError:
                vol = 0.0       # two parts just touching: Fusion reports an overlap so thin that it cannot
                                # give its volume (seen 2026-11-26 at the q4 limit). It still counts as a hit.
            pairs[key] = pairs.get(key, 0.0) + vol
        return pairs

    def restore(self):
        self.root.transformOccurrences(self.moving, self.saved, False)
        if self.spin:                                               # second call: see place()
            occs = list(self.moving) + [p["occ"] for p in self.spin]
            self.root.transformOccurrences(occs, list(self.saved) + list(self.spin_saved), False)


def sweep(r, j, sign):
    q = list(Q0); last_free = x = Q0[j]; n = 0
    while True:
        x_new = x + sign * STEP[j]
        if not BOUND[j][0] <= x_new <= BOUND[j][1]:
            return dict(limit=last_free, hit=None, checks=n, note="no collision up to the search bound")
        q[j] = x_new; c = r.collisions(q); n += 1
        if c:
            lo, hi = last_free, x_new
            while abs(hi - lo) > TOL[j]:
                mid = 0.5 * (lo + hi); q[j] = mid; cm = r.collisions(q); n += 1
                if cm:
                    hi, c = mid, cm
                else:
                    lo = mid
            return dict(limit=round(lo, 3), first_collision=round(hi, 3), checks=n,
                        hit={k: round(v, 3) for k, v in c.items()})
        last_free = x = x_new


LOG = __file__.replace(".py", ".jsonl")      # one JSON line per result, appended


def out(d):
    line = json.dumps(d, ensure_ascii=False)
    print(line)
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(line + "\n")


def run(_context: str):
    # A long run outlasts the MCP call (it times out after about a minute) but goes on in
    # Fusion; the results are therefore also appended to joint_limits_fusion.jsonl.
    r = Robot()
    try:
        out(dict(event="start", jobs=JOBS, test=TEST, time=time.strftime("%Y-%m-%d %H:%M:%S")))
        if TEST:
            for q in (Q0, [0, 0, 0, 0], [0, 80, -100, 0], [-11.86, 41.671, -10.941, -63.059]):
                t0 = time.time(); c = r.collisions(q)
                out(dict(q=q, collisions=c, seconds=round(time.time() - t0, 2)))
        for j, sign in JOBS:
            t0 = time.time()
            res = sweep(r, j, sign)
            res.update(joint=f"q{j + 1}", side="lower" if sign < 0 else "upper", seconds=round(time.time() - t0, 1))
            out(res)
        if TRAJ:                                   # check given poses, e.g. the step 4 trajectory
            with open(TRAJ, encoding="utf-8") as f:
                T = json.load(f)
            t0 = time.time(); bad = []
            for tk, q in zip(T["t"], T["q"]):
                c = r.collisions(q)
                if c:
                    bad.append(dict(t=tk, q=[round(v, 3) for v in q], hit={k: round(v, 3) for k, v in c.items()}))
            out(dict(event="trajectory check", file=TRAJ, poses=len(T["q"]), colliding=len(bad),
                     first_collisions=bad[:20], seconds=round(time.time() - t0, 1)))
    finally:
        r.restore()
        out(dict(event="done, robot put back", time=time.strftime("%Y-%m-%d %H:%M:%S")))
