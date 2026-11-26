"""Run the writing playback once (PlayWriting.play, real time) and check the turning parts along the way.
Run inside Fusion on E_Writing_q3_50W_build:  run_play()
"""
import importlib
import json
import math
import runpy
import sys
from pathlib import Path
import adsk
import adsk.core as core
import adsk.fusion as fusion

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import struct_fix9 as S9
import struct_fix11 as S11
for _m in (S9, S11):
    importlib.reload(_m)
OUT = HERE / 'play_check.json'


def run_play(samples=24):
    d = S9.guard()
    root = d.rootComponent
    pw = runpy.run_path(str(HERE.parent / 'PlayWriting' / 'PlayWriting.py'))
    out = dict(started=True)
    OUT.write_text(json.dumps(out), encoding='utf8')
    res = pw['play']()                                   # the real thing: real time, raises if the pen tip is off by > 0.05 mm
    out['play'] = res
    S9.say(f'playback: {res}')
    # the same pose function at evenly spaced times: where every turning part stands
    track = json.loads(root.attributes.itemByName('RPRR', 'WritingTrackV3').value)
    times, poses = track['t'], track['q']
    moving = sorted((o for o in root.occurrences if o.component.name[:4] in ('D_02', 'D_03', 'D_04', 'D_05')), key=lambda o: o.component.name)
    spin = pw['spinning_parts'](root)
    rows, worst_turn, worst_dev, worst_off, worst_tip = [], 0.0, 0.0, 0.0, 0.0
    for k in range(samples + 1):
        t = times[-1] * k / samples
        q = pw['pose_at'](t, times, poses)
        pw['drive'](root, moving, q, spin)
        deg = (math.degrees(q[0]), 0, math.degrees(q[2]), math.degrees(q[3]))
        row = dict(t=round(t, 3), q=[round(v, 5) for v in q], turn_deg={})
        for key in S11.ROTORS:
            turn, dev = S11.state(key)
            want = S11.expected(key, deg)
            worst_turn = max(worst_turn, abs((turn - want + 180) % 360 - 180))
            worst_dev = max(worst_dev, dev)
            row['turn_deg'][key] = round(turn, 2)
        sc, off, tilt = S9.screw_state()
        want_sc = (-36000.0 * (q[1] - 0.080) + 180) % 360 - 180
        worst_turn = max(worst_turn, abs((sc - want_sc + 180) % 360 - 180))
        worst_off = max(worst_off, off)
        worst_tip = max(worst_tip, pw['tip_error_mm'](moving[-1], q))
        row['turn_deg']['q2_screw'] = round(sc, 2)
        rows.append(row)
    out['samples'] = rows
    out['largest_angle_error_deg'] = worst_turn
    out['largest_deviation_from_pure_rotation_mm'] = worst_dev
    out['largest_screw_off_axis_mm'] = worst_off
    out['largest_tip_error_mm'] = worst_tip
    q_span = [[min(p[i] for p in poses), max(p[i] for p in poses)] for i in range(4)]
    out['trajectory_range'] = q_span
    pw['drive'](root, moving, (0.0, 0.080, 0.0, 0.0), spin)          # back to the reference pose
    back = max(S11.state(k)[1] for k in S11.ROTORS)
    out['back_at_reference'] = dict(deviation_mm=back, pending_position=d.snapshots.hasPendingSnapshot)
    OUT.write_text(json.dumps(out, indent=1), encoding='utf8')
    S9.say(f'playback check: {samples + 1} times; largest angle error {worst_turn:.4f} deg, deviation {worst_dev:.5f} mm, '
           f'screw off axis {worst_off:.5f} mm, tip error {worst_tip:.5f} mm; back at reference (pending {d.snapshots.hasPendingSnapshot})')
    S9.say('PLAY CHECK DONE')


def close_cloud_copies():
    """The cloud copies and drawings opened while publishing are saved; they are closed so that only the working document is open."""
    a = core.Application.get()
    closed = []
    for d in list(a.documents):
        if d.name != 'E_Writing_q3_50W_build' and d.dataFile and not d.isModified:
            closed.append(d.name)
            d.close(False)
    next(d for d in a.documents if d.name == 'E_Writing_q3_50W_build').activate()
    S9.say(f'closed {closed}; open now {[d.name for d in a.documents]}')


def video_view(test_png=None, show_bucket=True):
    """Bucket shown, sketches hidden, camera on the whole bucket. Returns what was changed, for video_restore()."""
    d = S9.guard()
    a = core.Application.get()
    root = d.rootComponent
    drum = next(o for o in root.occurrences if o.component.name.startswith('Wave_Drum'))
    before = dict(drum=drum.isLightBulbOn, sketches=[(sk.name, sk.isVisible) for sk in root.sketches])
    drum.isLightBulbOn = show_bucket
    before['joints'] = []
    for c in [root] + [o.component for o in root.occurrences]:       # the joint symbols would show in every frame
        before['joints'].append((c.name, c.isJointsFolderLightBulbOn))
        c.isJointsFolderLightBulbOn = False
    vp = a.activeViewport
    cam = vp.camera
    cam.isSmoothTransition = False
    cam.cameraType = core.CameraTypes.OrthographicCameraType
    cam.target = core.Point3D.create(0, 20, 0)
    cam.eye = core.Point3D.create(95, 80, 120)
    cam.upVector = core.Vector3D.create(0, 1, 0)
    cam.viewExtents = 70
    vp.camera = cam
    vp.refresh()
    adsk.doEvents()
    if test_png:
        vp.saveAsImageFile(str(test_png), 960, 640)
    (HERE / 'video_view_before.json').write_text(json.dumps(before, ensure_ascii=False), encoding='utf8')
    return before


def render(frames_dir, fps=25):
    """All frames of the writing cycle (PlayWriting.play with a frame folder), then back to the reference pose."""
    d = S9.guard()
    root = d.rootComponent
    frames_dir = Path(frames_dir)
    frames_dir.mkdir(parents=True, exist_ok=True)
    video_view()
    pw = runpy.run_path(str(HERE.parent / 'PlayWriting' / 'PlayWriting.py'))
    res = pw['play'](str(frames_dir), fps)
    moving = sorted((o for o in root.occurrences if o.component.name[:4] in ('D_02', 'D_03', 'D_04', 'D_05')), key=lambda o: o.component.name)
    pw['drive'](root, moving, (0.0, 0.080, 0.0, 0.0), pw['spinning_parts'](root))
    before = json.loads((HERE / 'video_view_before.json').read_text(encoding='utf8'))
    video_restore()
    S9.say(f'video frames: {res}; {len(list(frames_dir.glob("frame_*.png")))} files; back at reference, pending {d.snapshots.hasPendingSnapshot}')
    S9.say('RENDER DONE')


def video_restore():
    d = S9.guard()
    root = d.rootComponent
    before = json.loads((HERE / 'video_view_before.json').read_text(encoding='utf8'))
    next(o for o in root.occurrences if o.component.name.startswith('Wave_Drum')).isLightBulbOn = before['drum']
    comps = {c.name: c for c in [root] + [o.component for o in root.occurrences]}
    for name, on in before.get('joints', []):
        comps[name].isJointsFolderLightBulbOn = on
