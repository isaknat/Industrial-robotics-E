"""Belt loop around pulleys and one backside idler: geometry only (no Fusion). after CAD edition 3 (2026-11-26).

Lengths in mm. A loop is a list of circles in travel order, each (x, y, radius, sign):
sign +1 = the belt wraps the circle counter-clockwise (a pulley inside the loop),
sign -1 = clockwise (an idler pressing on the back of the belt from outside).
"""
import math


def tangent(c1, c2):
    """Tangent points (on c1, on c2) of the straight run from circle c1 to circle c2."""
    x1, y1, r1, s1 = c1
    x2, y2, r2, s2 = c2
    dx, dy = x2 - x1, y2 - y1
    dist = math.hypot(dx, dy)
    ex, ey = dx / dist, dy / dist
    a = (s2 * r2 - s1 * r1) / dist          # left normal n: n.(c2 - c1) = s2 r2 - s1 r1
    b = math.sqrt(1 - a * a)          # ValueError if the circles are too close for this run
    nx, ny = a * ex - b * ey, a * ey + b * ex
    return (x1 - s1 * r1 * nx, y1 - s1 * r1 * ny), (x2 - s2 * r2 * nx, y2 - s2 * r2 * ny)


def loop(circles):
    """Segments of the closed path: ('line', p, q) and ('arc', centre, r, a_in, sweep) with signed sweep."""
    n = len(circles)
    runs = [tangent(circles[i], circles[(i + 1) % n]) for i in range(n)]
    out = []
    for i in range(n):
        x, y, r, s = circles[i]
        p_in = runs[i - 1][1]
        p_out = runs[i][0]
        a_in = math.atan2(p_in[1] - y, p_in[0] - x)
        a_out = math.atan2(p_out[1] - y, p_out[0] - x)
        sweep = (s * (a_out - a_in)) % (2 * math.pi)
        out.append(('arc', (x, y), r, a_in, s * sweep))
        out.append(('line', runs[i][0], runs[i][1]))
    return out


def length(circles):
    total = 0.0
    for seg in loop(circles):
        if seg[0] == 'arc':
            total += seg[2] * abs(seg[4])
        else:
            total += math.hypot(seg[2][0] - seg[1][0], seg[2][1] - seg[1][1])
    return total


def wrap_deg(circles, i):
    arcs = [s for s in loop(circles) if s[0] == 'arc']
    return math.degrees(abs(arcs[i][4]))


PITCH_TO_BACK = 1.10     # pitch line to the flat back of a 3 mm GT belt (same numbers as mechanism.belt)
PITCH_TO_ROOT = 0.38     # pitch line to the tooth-root side that is drawn


TOOTH_R, TOOTH_SINK = 0.9, 0.24      # a tooth is drawn as a round bump: radius, and how far its centre lies
                                     # inside the root line; it stands 1.14 mm proud and is 1.7 mm wide at the root


def tooth_centres(circles, n):
    """Centres of the n tooth bumps, evenly spaced along the pitch loop (pitch = length / n).
    Teeth that would cut into a pulley are left out: there they sit in the pulley's grooves,
    between its flanges. The loop must run counter-clockwise (pulleys with sign +1)."""
    pitch = length(circles) / n
    depth = PITCH_TO_ROOT + TOOTH_SINK
    out, s, nxt = [], 0.0, pitch / 2
    for seg in loop(circles):
        if seg[0] == 'line':
            (px, py), (qx, qy) = seg[1], seg[2]
            l = math.hypot(qx - px, qy - py)
            dx, dy = (qx - px) / l, (qy - py) / l
            while nxt < s + l:
                u = nxt - s
                out.append((px + dx * u - dy * depth, py + dy * u + dx * depth))
                nxt += pitch
        else:
            (cx, cy), r, a0, sweep = seg[1:]
            l = r * abs(sweep)
            turn = 1.0 if sweep > 0 else -1.0
            while nxt < s + l:
                a = a0 + turn * (nxt - s) / r
                rr = r - turn * depth            # towards the centre on a pulley, away from it on a backside idler
                out.append((cx + rr * math.cos(a), cy + rr * math.sin(a)))
                nxt += pitch
        s += l
    keep = []
    for x, y in out:
        if all(math.hypot(x - cx, y - cy) >= (r - PITCH_TO_ROOT) + TOOTH_R + 0.05 for cx, cy, r, sgn in circles if sgn > 0):
            keep.append((x, y))
    return keep


def pitch_circles(pulleys, idler):
    """pulleys: [(x, y, teeth)], idler: (x, y, roller radius). Travel order: pulley 0, idler, pulley 1."""
    (xa, ya, za), (xb, yb, zb) = pulleys
    xi, yi, ri = idler
    return [(xa, ya, za * 3 / (2 * math.pi), +1), (xi, yi, ri + PITCH_TO_BACK, -1), (xb, yb, zb * 3 / (2 * math.pi), +1)]


def offset(circles, d):
    """The same loop moved by d towards the outside (back) of the belt."""
    return [(x, y, r + s * d, s) for x, y, r, s in circles]


def solve(pulleys, idler_at, roller, belt_teeth, lo, hi):
    """Find the idler parameter u in [lo, hi] that gives a pitch length of belt_teeth*3 mm.
    idler_at(u) -> (x, y). The path is shortest where the idler just touches the straight span;
    the search starts there and moves towards hi (idler pushed into the belt)."""
    def f(u):
        try:
            return length(pitch_circles(pulleys, (*idler_at(u), roller))) - 3 * belt_teeth
        except ValueError:          # idler overlaps a pulley: no such path
            return float('inf')
    grid = [lo + (hi - lo) * i / 2000 for i in range(2001)]
    vals = [f(u) for u in grid]
    k = min(range(len(grid)), key=lambda i: vals[i])
    assert vals[k] < 0, ('belt too short even without the idler', vals[k])
    lo = grid[k]
    reach = [(u, v) for u, v in zip(grid[k:], vals[k:]) if v > 0]
    assert reach and reach[0][1] < float('inf'), 'the idler meets a pulley before the belt is tight'
    hi = reach[0][0]
    for _ in range(80):
        mid = (lo + hi) / 2
        if f(mid) < 0:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


# ---- third edition, q3 drive with the 50 W motor: 16 -> 96 (9 mm belt), 16 -> 120 (15 mm belt); slider frame
INPUT = (-106.6, 78.29)        # q3 input shaft (motor axis)
MID = (-45.2, 78.29)           # q3 intermediate shaft
Q3 = (0.0, 0.0)                # q3 axis
STAGE1 = dict(pulleys=[(*INPUT, 16), (*MID, 96)], roller=8.0, teeth=105, x=-93.0)      # idler slides in y at x
STAGE2 = dict(pulleys=[(*Q3, 120), (*MID, 16)], roller=7.5, teeth=138, arm=None)        # idler swings about MID


def stage1():
    s = STAGE1
    at = lambda y: (s['x'], y)
    # tangent position (no push) is the lowest; pushing up (+y) lengthens the path
    y = solve(s['pulleys'], at, s['roller'], s['teeth'], 36.0, 52.0)
    return at(y)


def stage2(arm, start_deg):
    s = STAGE2
    at = lambda deg: (MID[0] + arm * math.cos(math.radians(deg)), MID[1] + arm * math.sin(math.radians(deg)))
    # swinging clockwise (angle decreasing) pushes the idler into the span: use u = -angle
    u = solve(s['pulleys'], lambda u: at(-u), s['roller'], s['teeth'], -start_deg - 15.0, -start_deg + 40.0)
    return at(-u), -u


if __name__ == '__main__':
    # self-test 1: two pulleys only must reproduce the textbook length
    for c, z1, z2 in ((61.4, 16, 80), (90.4, 24, 120)):
        d1, d2 = 3 * z1 / math.pi, 3 * z2 / math.pi
        book = 2 * c * math.sqrt(1 - ((d2 - d1) / (2 * c)) ** 2) + math.pi * (d1 + d2) / 2 + (d2 - d1) * math.asin((d2 - d1) / (2 * c))
        mine = length([(0, 0, d1 / 2, 1), (c, 0, d2 / 2, 1)])
        print(f'two pulleys {z1}/{z2} at {c}: exact formula {book:.4f}, loop() {mine:.4f}')
    # self-test 2: every line must be tangent to both circles it joins
    for name, circles in (('stage 1', pitch_circles(STAGE1['pulleys'], (*stage1(), STAGE1['roller']))),):
        worst = 0
        segs = loop(circles)
        for k, seg in enumerate(segs):
            if seg[0] != 'line':
                continue
            (px, py), (qx, qy) = seg[1], seg[2]
            for cx, cy, r, s in (circles[(k // 2) % 3], circles[(k // 2 + 1) % 3]):
                dline = abs((qx - px) * (py - cy) - (px - cx) * (qy - py)) / math.hypot(qx - px, qy - py)
                worst = max(worst, abs(dline - r))
        print(name, 'tangency error', f'{worst:.2e}', 'length', f'{length(circles):.4f}')
    x, y = stage1()
    c1 = pitch_circles(STAGE1['pulleys'], (x, y, STAGE1['roller']))
    print(f'stage 1: idler at ({x:.3f}, {y:.3f}); wraps: 16T {wrap_deg(c1, 0):.1f} deg = {wrap_deg(c1, 0) * 16 / 360:.2f} teeth, idler {wrap_deg(c1, 1):.1f}, 96T {wrap_deg(c1, 2):.1f}')
    print('   idler to 96T flange (r 46.96): gap', round(math.hypot(x - MID[0], y - MID[1]) - 46.96 - 8, 2), 'mm; to 16T flange (r 8.76):', round(math.hypot(x - INPUT[0], y - INPUT[1]) - 8.76 - 8, 2))
    for arm in (44.1,):
        (x, y), deg = stage2(arm, 0.0)
        c2 = pitch_circles(STAGE2['pulleys'], (x, y, STAGE2['roller']))
        print(f'stage 2: idler at ({x:.3f}, {y:.3f}), arm angle {deg:.2f} deg; length {length(c2):.4f}; wraps: 120T {wrap_deg(c2, 0):.1f}, idler {wrap_deg(c2, 1):.1f}, 16T {wrap_deg(c2, 2):.1f} deg = {wrap_deg(c2, 2) * 16 / 360:.2f} teeth')
        print('   idler to 120T flange (r 58.42):', round(math.hypot(x, y) - 58.42 - 7.5, 2), 'mm; to 16T flange (r 8.76):', round(math.hypot(x - MID[0], y - MID[1]) - 8.76 - 7.5, 2))
