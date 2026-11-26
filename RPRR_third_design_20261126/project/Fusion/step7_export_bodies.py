"""Step 7, second version: what the Simscape model needs from the Fusion model. after CAD edition 1 (2026-11-24).

Run inside Fusion (Fusion MCP "script" tool, or Scripts and Add-Ins) with E_Writing.f3d active.
Nothing in the design is changed and nothing is saved; the only output is files in step7_bodies/:

  D_01.step ... D_05.step, Wave_Drum.step   shape of each sub-assembly in its own frame (for display)
  mass_properties.json                      for each sub-assembly, in its own frame (SI units):
                                            mass, centre of mass, inertia tensor about the centre of
                                            mass (tensor convention: off-diagonal = -integral of x*y dm);
                                            where each motor sits and which way its shaft points;
                                            mass and spin inertia of every part of the transmissions.

Frames of the sub-assemblies (the same as PlayWriting.py and joint_limits_fusion.py): Y up; D_01 and
the bucket fixed; D_02 origin on the q1 axis; D_03 and D_04 origin on the q3 axis, Z = q3 axis;
D_05 origin on the q4 axis, pen tip at (50, 0, 0) mm.

The inertia tensor is built from Fusion's principal moments and principal axes, which do not depend
on any sign convention. Fusion's own "XYZ" moments are also saved (raw, kg cm^2) so that their
reference point and sign convention can be checked against the tensor.
"""
import json, math, os
import adsk.core as core
import adsk.fusion as fusion

OUT = os.path.join(os.path.dirname(__file__), "step7_bodies")
SUBS = ("D_01", "D_02", "D_03", "D_04", "D_05", "Wave_Drum")
MOTORS = ("D_HG_KR053", "D_HG_KR13")
SPIN = ("D_GT3M_", "D_Belt_", "D_Q1_Input_shaft", "D_Q1_Intermediate_shaft", "D_Q3_Input_shaft",
        "D_Q3_Intermediate_shaft", "D_Q4_Input_shaft", "D_Q4_Intermediate_shaft", "D_Q4_Wrist_shaft",
        "D_Q2_Ballscrew_12x10", "D_XGT2_19C_", "D_Belt_idler_15", "D_Idler_axle_6", "D_Q3_Shaft_15")


def props(comp):
    p = comp.getPhysicalProperties(fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
    c = [v / 100.0 for v in p.centerOfMass.asArray()]                      # cm -> m
    _, i1, i2, i3 = p.getPrincipalMomentsOfInertia()
    _, a1, a2, a3 = p.getPrincipalAxes()
    I = [[0.0] * 3 for _ in range(3)]
    for ik, a in zip((i1, i2, i3), (a1, a2, a3)):
        v = a.asArray(); n = math.sqrt(sum(x * x for x in v)); v = [x / n for x in v]
        for r in range(3):
            for s in range(3):
                I[r][s] += ik * 1e-4 * v[r] * v[s]                           # kg cm^2 -> kg m^2
    _, xx, yy, zz, xy, yz, xz = p.getXYZMomentsOfInertia()
    return dict(mass=p.mass, com_m=c, I_com=I, principal=[i1 * 1e-4, i2 * 1e-4, i3 * 1e-4],
                fusion_xyz_raw_kgcm2=dict(xx=xx, yy=yy, zz=zz, xy=xy, yz=yz, xz=xz))


def matrix(m):
    a = m.asArray()                                    # row-major 4x4, translation in cm
    return dict(R=[[a[0], a[1], a[2]], [a[4], a[5], a[6]], [a[8], a[9], a[10]]],
                t_m=[a[3] / 100.0, a[7] / 100.0, a[11] / 100.0])


def run(_context: str):
    app = core.Application.get()
    design = fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    os.makedirs(OUT, exist_ok=True)
    tops = {}
    for o in root.occurrences:
        for k in SUBS:
            if o.component.name.startswith(k):
                tops[k] = o
    res = dict(document=app.activeDocument.name, units="SI (kg, m, kg m^2)", subassemblies={})
    em = design.exportManager
    for k in SUBS:
        o = tops[k]; comp = o.component
        d = props(comp)
        d.update(component=comp.name, occurrence_transform_now=matrix(o.transform2))
        motors, spin = [], []
        for c in comp.allOccurrences:             # occurrences in the sub-assembly's own frame
            n = c.component.name
            if n in MOTORS:
                m = matrix(c.transform2)
                motors.append(dict(part=n, origin_m=m["t_m"], shaft_axis=[row[2] for row in m["R"]]))
            if any(n.startswith(s) for s in SPIN):
                pc = props(c.component)
                spin.append(dict(part=n, mass=pc["mass"], I_about_own_z=pc["I_com"][2][2],
                                 principal=pc["principal"]))
        d.update(motors=motors, transmission_parts=spin)
        path = os.path.join(OUT, k + ".step")
        opts = em.createSTEPExportOptions(path, comp)
        if not em.execute(opts):
            raise RuntimeError("STEP export failed for " + k)
        d["step_file"] = os.path.basename(path)
        res["subassemblies"][k] = d
        print(k, comp.name, "m = %.4f kg" % d["mass"], "com =", [round(v * 1000, 2) for v in d["com_m"]], "mm")
    with open(os.path.join(OUT, "mass_properties.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1, ensure_ascii=False)
    print("written", OUT)
