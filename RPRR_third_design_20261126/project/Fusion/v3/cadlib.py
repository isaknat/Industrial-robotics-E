"""Small helpers for editing the RPRR Fusion model from scripts. after CAD edition 2 (2026-11-24).

All lengths in mm at the interface (Fusion works in cm internally).
"""
import adsk.core as core
import adsk.fusion as fusion

CM = 0.1


def design():
    return fusion.Design.cast(core.Application.get().activeProduct)


def comp(name):
    return next(c for c in design().allComponents if c.name == name)


def group(prefix):
    """Top-level occurrence of a sub-assembly, e.g. group('D_03')."""
    return next(o for o in design().rootComponent.occurrences if o.component.name.startswith(prefix))


def native_child(group_prefix, instance_name):
    """The native occurrence (inside the sub-assembly component) of a part instance, e.g. 'D_R12_10K3_Nut:1'."""
    g = group(group_prefix).component
    return next(o for o in g.occurrences if o.name == instance_name)


def shift(occ, dx=0.0, dy=0.0, dz=0.0):
    old = occ.transform2.translation.asArray()
    m = occ.initialTransform.copy()
    t = m.translation
    t.add(core.Vector3D.create(dx * CM, dy * CM, dz * CM))
    m.translation = t
    occ.timelineObject.rollTo(False)
    try:
        occ.initialTransform = m
    finally:
        design().timeline.moveToEnd()
    actual = occ.transform2.translation.asArray()
    expected = [v + CM * d for v, d in zip(old, (dx, dy, dz))]
    if max(abs(a-b) for a,b in zip(actual, expected)) > 1e-6:
        raise ValueError(f"{occ.name}: initial placement {actual} vs {expected}")


def strip(sketch):
    """Remove dimensions and geometric constraints so points can be moved freely."""
    for d in list(sketch.sketchDimensions):
        d.deleteMe()
    for c in list(sketch.geometricConstraints):
        if c.isDeletable:
            c.deleteMe()


def move_points(sketch, pred, dx=0.0, dy=0.0):
    """Move every sketch point whose (x, y) in mm satisfies pred by (dx, dy) mm. Returns the count."""
    pts = [p for p in sketch.sketchPoints
           if p.connectedEntities and p.connectedEntities.count and pred(p.geometry.x / CM, p.geometry.y / CM)]
    for p in pts:
        p.move(core.Vector3D.create(dx * CM, dy * CM, 0))
    return len(pts)


def set_distance(ext, mm):
    ext.extentOne.distance.expression = f"{mm} mm"


def set_start_offset(ext, mm):
    ext.startExtent.offset.expression = f"{mm} mm"


def timeline_problems():
    tl = design().timeline
    out = []
    for i in range(tl.count):
        it = tl.item(i)
        if it.isSuppressed:
            continue
        if it.healthState != fusion.FeatureHealthStates.HealthyFeatureHealthState:
            out.append((i, it.name, it.errorOrWarningMessage))
    return out


def material(name_start):
    matches=[m for m in design().materials if m.name.startswith(name_start)]
    if name_start=='RPRR Bearing steel':
        return next(m for m in matches if '(7850.00 kg/m3)' in m.name)
    return next(iter(matches))


def new_part(group_prefix, name, origin_mm, x_axis=(1, 0, 0), y_axis=(0, 1, 0), mat="RPRR 6061-T6"):
    """New empty component placed in a sub-assembly. origin and axes are in the sub-assembly frame."""
    g = group(group_prefix).component
    m = core.Matrix3D.create()
    xa = core.Vector3D.create(*x_axis)
    ya = core.Vector3D.create(*y_axis)
    za = xa.crossProduct(ya)
    m.setWithCoordinateSystem(core.Point3D.create(*[v * CM for v in origin_mm]), xa, ya, za)
    occ = g.occurrences.addNewComponent(m)
    occ.component.name = name
    occ.component.material = material(mat)
    return occ


def sketch_xy(component, name, z_mm=0.0):
    if abs(z_mm) < 1e-9:
        plane = component.xYConstructionPlane
    else:
        pi = component.constructionPlanes.createInput()
        pi.setByOffset(component.xYConstructionPlane, core.ValueInput.createByString(f"{z_mm} mm"))
        plane = component.constructionPlanes.add(pi)
    sk = component.sketches.add(plane)
    sk.name = name
    return sk


def rect(sk, x0, y0, x1, y1):
    sk.sketchCurves.sketchLines.addTwoPointRectangle(core.Point3D.create(x0 * CM, y0 * CM, 0),
                                                     core.Point3D.create(x1 * CM, y1 * CM, 0))


def circle(sk, x, y, r):
    return sk.sketchCurves.sketchCircles.addByCenterRadius(core.Point3D.create(x * CM, y * CM, 0), r * CM)


def poly(sk, pts):
    lines = sk.sketchCurves.sketchLines
    P = [core.Point3D.create(x * CM, y * CM, 0) for x, y in pts]
    for a, b in zip(P, P[1:] + P[:1]):
        lines.addByTwoPoints(a, b)


def profiles_near(sk, points_mm, tol_mm=0.5):
    """Profiles whose area centroid lies within tol of one of the points (mm). Use for holes."""
    out = core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        c = p.areaProperties().centroid
        if any(abs(c.x / CM - x) < tol_mm and abs(c.y / CM - y) < tol_mm for x, y in points_mm):
            out.add(p)
    return out


def set_profile_at(feature, sketch_name, points_mm, tol_mm=0.5):
    """Roll the timeline to the feature, point it at the profiles near the points, roll back to the end."""
    feature.timelineObject.rollTo(True)
    sk = feature.parentComponent.sketches.itemByName(sketch_name)
    profs = profiles_near(sk, points_mm, tol_mm)
    if profs.count != len(points_mm):
        design().timeline.moveToEnd()
        raise RuntimeError(f"{sketch_name}: found {profs.count} profiles for {len(points_mm)} points")
    feature.profile = profs if profs.count > 1 else profs.item(0)
    design().timeline.moveToEnd()
    return profs.count


def move_circle(circ, x, y, r=None):
    c = circ.centerSketchPoint.geometry
    circ.centerSketchPoint.move(core.Vector3D.create(x * CM - c.x, y * CM - c.y, 0))
    if r is not None:
        circ.radius = r * CM


def extrude_like(model, sketch_name, points_mm, name, tol_mm=0.5):
    """New extrude in the same component, with the same operation, start offset and signed
    distance as an existing extrude, on the profiles of a sketch near the given points."""
    compo = model.parentComponent
    sk = compo.sketches.itemByName(sketch_name)
    profs = profiles_near(sk, points_mm, tol_mm)
    if profs.count != len(points_mm):
        raise RuntimeError(f"{sketch_name}: found {profs.count} profiles for {len(points_mm)} points")
    ef = compo.features.extrudeFeatures
    inp = ef.createInput(profs, model.operation)
    if model.operation == fusion.FeatureOperations.CutFeatureOperation:
        inp.participantBodies = list(compo.bRepBodies)
    dist = model.extentOne.distance.value            # cm, signed
    direction = (fusion.ExtentDirections.NegativeExtentDirection if dist < 0
                 else fusion.ExtentDirections.PositiveExtentDirection)
    inp.setOneSideExtent(fusion.DistanceExtentDefinition.create(core.ValueInput.createByReal(abs(dist))), direction)
    st = model.startExtent
    if hasattr(st, "offset"):
        inp.startExtent = fusion.OffsetStartDefinition.create(core.ValueInput.createByReal(st.offset.value))
    f = ef.add(inp)
    f.name = name
    return f


def profiles_containing(sk, points_mm, exclude_points_mm=()):
    """Profiles whose area contains any of the given points (mm) and none of the excluded points."""
    out = core.ObjectCollection.create()
    for i in range(sk.profiles.count):
        p = sk.profiles.item(i)
        loops = p.profileLoops

        def inside(x, y):
            # outer loop bounding box test, then exclusion by inner loops' boxes
            outer = next(l for l in loops if l.isOuter)
            bb = outer.profileCurves.item(0).boundingBox
            for c in outer.profileCurves:
                bb.combine(c.boundingBox)
            if not (bb.minPoint.x <= x * CM <= bb.maxPoint.x and bb.minPoint.y <= y * CM <= bb.maxPoint.y):
                return False
            for l in loops:
                if l.isOuter:
                    continue
                ib = l.profileCurves.item(0).boundingBox
                for c in l.profileCurves:
                    ib.combine(c.boundingBox)
                if ib.minPoint.x <= x * CM <= ib.maxPoint.x and ib.minPoint.y <= y * CM <= ib.maxPoint.y:
                    return False
            return True
        if any(inside(x, y) for x, y in points_mm) and not any(inside(x, y) for x, y in exclude_points_mm):
            out.add(p)
    return out


def extrude(component, profiles, dist_mm, start_mm=0.0, op=fusion.FeatureOperations.NewBodyFeatureOperation,
            name=None, symmetric=False):
    ef = component.features.extrudeFeatures
    inp = ef.createInput(profiles, op)
    if op == fusion.FeatureOperations.CutFeatureOperation:
        inp.participantBodies = list(component.bRepBodies)
    if symmetric:
        inp.setSymmetricExtent(core.ValueInput.createByString(f"{dist_mm} mm"), True)
    else:
        inp.setDistanceExtent(False, core.ValueInput.createByString(f"{dist_mm} mm"))
        if abs(start_mm) > 1e-9:
            inp.startExtent = fusion.OffsetStartDefinition.create(core.ValueInput.createByString(f"{start_mm} mm"))
    f = ef.add(inp)
    if name:
        f.name = name
    return f
