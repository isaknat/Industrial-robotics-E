"""Third design, step 3: motor flange slots -> round holes. after CAD edition 2 (2026-11-24).

D_Motor_slotted_flange (used for the q2, q3 and q4 motors) had four 4.4 x 8.4 mm slots for its
fixing screws. The pulleys and the ball screw sit on their own bearings and are driven through a
coupling, so moving the motor in these slots cannot tension anything; it only misaligns the
coupling. The slots are replaced by plain 4.4 mm holes at the slot centres (+-23, +-24) mm; the
motor stays located by its 30 mm spigot in the 30.2 mm bore. The extrusion is pointed at the new
plate profile and the timeline health is checked. Run inside Fusion with E_Writing_v3 active.
"""
import adsk.core as core
import adsk.fusion as fusion


def run(_context: str):
    design = fusion.Design.cast(core.Application.get().activeProduct)
    comp = next(c for c in design.allComponents if c.name == "D_Motor_slotted_flange")
    sk = comp.sketches.itemByName("Motor_mount_profile")
    ext = comp.features.extrudeFeatures.itemByName("Motor_mount")
    before_area = ext.bodies.item(0).area
    # slot pieces: the eight r = 2.2 mm arcs and the eight 4 mm lines between them
    arcs = [a for a in sk.sketchCurves.sketchArcs if abs(a.radius - 0.22) < 1e-6]
    lines = [l for l in sk.sketchCurves.sketchLines if abs(l.length - 0.4) < 1e-6]
    assert len(arcs) == 8 and len(lines) == 8, (len(arcs), len(lines))
    centres = sorted({(round(a.centerSketchPoint.geometry.x * 10), round(a.centerSketchPoint.geometry.y * 10))
                      for a in arcs})
    holes = sorted({(round((x + (4 if x < 0 else -4) * (1 if abs(x) == 25 else 0)) / 1), y) for x, y in centres})
    holes = sorted({(23 if x > 0 else -23, y) for x, y in centres})
    assert len(holes) == 4, holes
    for c in arcs + lines:
        c.deleteMe()
    for x, y in holes:
        circ = sk.sketchCurves.sketchCircles.addByCenterRadius(core.Point3D.create(x / 10, y / 10, 0), 0.22)
        sk.sketchDimensions.addDiameterDimension(circ, core.Point3D.create(x / 10 + 0.5, y / 10 + 0.5, 0))
    # the plate profile = the profile with the largest area
    prof = max((sk.profiles.item(i) for i in range(sk.profiles.count)),
               key=lambda p: p.areaProperties().area)
    ext.timelineObject.rollTo(True)
    ext.profile = prof
    design.timeline.moveToEnd()
    bad = [design.timeline.item(i).name for i in range(design.timeline.count)
           if design.timeline.item(i).healthState != fusion.FeatureHealthStates.HealthyFeatureHealthState]
    body = ext.bodies.item(0)
    print("holes at", holes, "| profiles now", sk.profiles.count,
          "| body volume mm3", round(body.volume * 1000, 1), "| area before/after cm2",
          round(before_area, 3), round(body.area, 3), "| unhealthy timeline items:", bad)
    comp.name = "D_Motor_flange"
    print("renamed to", comp.name)
