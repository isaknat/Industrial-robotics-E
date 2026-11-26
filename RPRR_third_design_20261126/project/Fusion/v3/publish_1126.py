"""Cloud documents and drawings of the model after the structure review (after CAD edition 3, 2026-11-26).

The third-edition cloud files (folder "RPRR - Third Edition") are left as they are. A new folder
"RPRR - 2026-11-26" gets:
  E_Writing_1126_\u542b\u6876\u603b\u88c5   the whole model with the bucket shown (joints and motion links work)   # Chinese-language Fusion names: assembly with bucket
  E_Writing_1126            the same with the bucket and the sketches hidden: source of the drawings
  E_Writing_1126_Exploded   the five sub-assemblies pulled apart
and the automatic drawings made from them. Same method as edition3.py (drawing_source, drawings, ...).
The working document E_Writing_q3_50W_build stays local and unsaved; the cloud files are made by importing
the exported E_Writing.f3d, so that the working document keeps its name.
Stages, run inside Fusion one at a time: export_local, demo_doc, drawing_source, drawings, ...
"""
import json
import math
import sys
import time
from pathlib import Path
import adsk.core as core
import adsk.fusion as fusion

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
WORK = 'E_Writing_q3_50W_build'
FOLDER = 'RPRR - 2026-11-26'
DEMO, SOURCE, EXPLODED = 'E_Writing_1126_\u542b\u6876\u603b\u88c5', 'E_Writing_1126', 'E_Writing_1126_Exploded'   # Chinese-language Fusion names: assembly with bucket
LOG = HERE / 'publish_1126_log.txt'
STATE = HERE / 'publish_1126.json'


def say(text):
    with open(LOG, 'a', encoding='utf8') as f:
        f.write(time.strftime('%H:%M:%S ') + text + '\n')
    print(text)


def state(**kw):
    data = json.loads(STATE.read_text(encoding='utf8')) if STATE.exists() else {}
    if kw:
        data.update(kw)
        STATE.write_text(json.dumps(data, indent=1, ensure_ascii=False), encoding='utf8')
    return data


def app():
    return core.Application.get()


def work_doc():
    return next(d for d in app().documents if d.name == WORK)


def folder(create=False):
    project = next(p for p in app().data.dataProjects if p.name == 'Default Project')
    folders = project.rootFolder.dataFolders
    f = folders.itemByName(FOLDER)
    if not f and create:
        f = folders.add(FOLDER)
    return f


def design_of(doc):
    return fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))


def health(d):
    return [(i, d.timeline.item(i).name) for i in range(d.timeline.count)
            if d.timeline.item(i).healthState != fusion.FeatureHealthStates.HealthyFeatureHealthState]


def summary(d):
    root = d.rootComponent
    kinds, inst = set(), 0
    for o in root.allOccurrences:
        if o.bRepBodies.count and not o.component.name.startswith('Wave_Drum'):
            kinds.add(o.component.name)
            inst += 1
    return dict(timeline=d.timeline.count, not_healthy=health(d), kinds=len(kinds), instances=inst,
                root_joints=[j.name for j in root.asBuiltJoints], links=[m.name for m in root.motionLinks],
                spinning_table=bool(root.attributes.itemByName('RPRR', 'SpinningParts')),
                track=bool(root.attributes.itemByName('RPRR', 'WritingTrackV3')),
                masses={o.component.name[:4]: round(o.getPhysicalProperties(fusion.CalculationAccuracy.MediumCalculationAccuracy).mass, 4)
                        for o in root.occurrences})


def export_local():
    """The local archive is written again from the working document, and its summary kept for comparison."""
    doc = work_doc()
    doc.activate()
    d = design_of(doc)
    assert not d.snapshots.hasPendingSnapshot, 'the working model is not at its captured position'
    em = d.exportManager
    assert em.execute(em.createFusionArchiveExportOptions(str(HERE / 'E_Writing.f3d')))
    s = summary(d)
    state(work=s)
    say(f'local archive written; working model: {s}')


def import_copy():
    a = app()
    doc = a.importManager.importToNewDocument(a.importManager.createFusionArchiveImportOptions(str(HERE / 'E_Writing.f3d')))
    d = design_of(doc)
    s = summary(d)
    w = state()['work']
    same = all(s[k] == w[k] for k in ('timeline', 'kinds', 'instances', 'root_joints', 'links', 'spinning_table', 'track', 'masses')) and not s['not_healthy']
    assert same, (s, w)
    return doc, d


def drive_test(d):
    """q3 joint by 4 deg: the q3 intermediate pulley must turn 30 deg about its axis. Then back."""
    root = d.rootComponent
    g3 = next(o for o in root.occurrences if o.component.name.startswith('D_03'))
    p = next(o for o in g3.childOccurrences if o.component.name == 'D_GT3M_90T_W9_D8')
    q3 = root.asBuiltJoints.itemByName('q3_Shoulder_pitch')

    def angle():
        a = p.transform2.asArray()
        return math.degrees(math.atan2(a[4], a[0]))
    a0 = angle()
    q3.jointMotion.rotationValue = math.radians(4)
    a1 = angle()
    q3.jointMotion.rotationValue = 0.0
    a2 = angle()
    assert abs(a1 - a0 - 30) < 1e-6 and abs(a2 - a0) < 1e-9, (a0, a1, a2)
    assert not d.snapshots.hasPendingSnapshot
    return a1 - a0


def demo_doc():
    f = folder(create=True)
    assert not any(x.name == DEMO for x in f.dataFiles), DEMO + ' exists already'
    doc, d = import_copy()
    turned = drive_test(d)
    root = d.rootComponent
    for o in root.occurrences:
        if o.component.name.startswith('Wave_Drum'):
            o.isLightBulbOn = True
    for sk in root.sketches:
        sk.isVisible = False
    app().activeViewport.fit()
    assert doc.saveAs(DEMO, f, 'Model after the structure review of 2026-11-26, with the bucket; joints and motion links', '')
    say(f'{DEMO}: imported copy equals the working model (timeline, parts, joints, links, masses); q3 4 deg -> pulley {turned:.2f} deg; saved to "{FOLDER}"')
    state(demo_saved=True)
    work_doc().activate()


def drawing_source():
    f = folder()
    assert not any(x.name == SOURCE for x in f.dataFiles), SOURCE + ' exists already'
    doc, d = import_copy()
    root = d.rootComponent
    for o in root.occurrences:
        if o.component.name.startswith('Wave_Drum'):
            o.isLightBulbOn = False
    for sk in root.sketches:
        sk.isVisible = False
    app().activeViewport.fit()
    assert doc.saveAs(SOURCE, f, 'Drawing source: model after the structure review of 2026-11-26, bucket hidden', '')
    say(f'{SOURCE}: saved to "{FOLDER}"')
    state(source_saved=True)
    work_doc().activate()


def cloud_status():
    f = folder()
    rows = [(x.name, x.fileExtension, x.versionNumber, x.isComplete) for x in f.dataFiles] if f else None
    say(f'cloud folder "{FOLDER}": {rows}')
    say('open documents: ' + str([(d.name, bool(d.dataFile)) for d in app().documents]))
    return rows


# ---------------------------------------------------------------- drawings (same settings as edition3.drawings)
def data_file(name, ext='f3d'):
    return next((x for x in folder().dataFiles if x.name == name and x.fileExtension == ext), None)


def drawings():
    import adsk.drawing as dr
    source = data_file(SOURCE)
    assert source and source.isComplete, 'the drawing source is still uploading'
    manager = dr.DrawingManager.get()
    spec = manager.createDrawingInput(source, dr.DrawingCreationModes.AutomaticDrawingCreationMode)
    spec.content = dr.DrawingContentTypes.VisibleOnlyDrawingContentType
    spec.sheetCreationType = dr.SheetCreationTypes.AllLevelsSheetCreationType
    settings = spec.automationPreferences.globalPreferences
    settings.isMainAssemblySheetGenerated = True
    settings.isSubAssemblySheetGenerated = True
    settings.isComponentSheetGenerated = True
    settings.isAnimationSheetGenerated = True
    settings.isAutoDimensionEnabled = True
    settings.isDetectAndOmitFasteners = False
    settings.omitComponentsWithKeywords = '__omit_none__'
    say('automatic drawing requested ...')
    t0 = time.time()
    file = manager.createDrawing(spec)
    say(f'createDrawing returned {file.name if file else None} after {time.time() - t0:.0f} s')
    if file:
        state(drawing=file.id, source=source.id)
    drawing_status()


def drawing_docs():
    import adsk.drawing as dr
    out = []
    for d in app().documents:
        art = dr.DrawingDocument.cast(d)
        if art:
            out.append((d, art))
    return out


def drawing_status():
    rows = [(x.name, x.fileExtension, x.versionNumber, x.isComplete) for x in folder().dataFiles]
    say(f'cloud files: {rows}')
    for d, art in drawing_docs():
        sheets = art.drawing.sheets
        names = [s.name for s in sheets]
        say(f'drawing document {d.name}: saved {bool(d.dataFile)}, {sheets.count} sheets, without views {sum(1 for s in sheets if s.views.count == 0)}, '
            f'first {names[:4]}, last {names[-3:]}')


def open_data(file):
    a = app()
    return next((d for d in a.documents if d.dataFile and d.dataFile.id == file.id), None) or a.documents.open(file)


def export_pdf(file, pdf_name, sheets_json=None, rename=None):
    import adsk.drawing as dr
    doc = open_data(file)
    doc.activate()
    if any(r.isOutOfDate for r in doc.documentReferences):
        assert doc.updateAllReferences()
    drawing = dr.DrawingDocument.cast(doc).drawing
    empty = [s for s in drawing.sheets if s.views.count == 0 and s.name in ('Sheet1', '\u56fe\u7eb81')]   # Chinese-language Fusion names: Sheet1
    if empty and drawing.sheets.count > 3:
        for s in empty:
            s.deleteMe()
    sheets = [dict(name=s.name, views=s.views.count, dimensions=s.drawingDimensions.count) for s in drawing.sheets]
    if doc.isModified:
        doc.save('Checked sheets')
    if sheets_json:
        (HERE / sheets_json).write_text(json.dumps(sheets, ensure_ascii=False, indent=1), encoding='utf8')
    mgr = drawing.exportManager
    opt = mgr.createPDFExportOptions(str(HERE / pdf_name))
    opt.sheetsToExport = dr.PDFSheetsExport.AllPDFSheetsExport
    opt.openPDF = False
    assert mgr.execute(opt)
    say(f'{pdf_name}: {len(sheets)} sheets, without views {[s["name"] for s in sheets if not s["views"]]}, with dimensions {sum(1 for s in sheets if s["dimensions"])}')
    return sheets


def main_drawing_file():
    st = state()
    if st.get('drawing'):
        return app().data.findFileById(st['drawing'])
    return next(x for x in folder().dataFiles if x.fileExtension == 'f2d' and x.name.startswith(SOURCE) and 'Exploded' not in x.name)


def export_main():
    file = main_drawing_file()
    sheets = export_pdf(file, 'E_Writing_1126_drawings.pdf', 'drawing_sheets_1126.json')
    state(drawing=file.id, drawing_url=file.fusionWebURL, source_url=data_file(SOURCE).fusionWebURL, demo_url=data_file(DEMO).fusionWebURL)
    work_doc().activate()
    coverage()


def parts_table():
    """Quantity and material of every part, from the working document."""
    from collections import Counter
    d = design_of(work_doc())
    rows = []

    def walk(comp, group):
        children = list(comp.occurrences)
        counts = Counter(o.component.name for o in children)
        for name, n in counts.items():
            c = next(o.component for o in children if o.component.name == name)
            rows.append(dict(group=group, part=name, quantity=n, material=c.material.name if c.material else '',
                             mass_g=round(c.physicalProperties.mass * 1000, 2)))
            if c.occurrences.count:
                walk(c, group)
    for g in d.rootComponent.occurrences:
        if g.component.name.startswith('D_0'):
            walk(g.component, g.component.name)
    (HERE / 'drawing_parts_1126.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding='utf8')
    return rows


def coverage():
    rows = parts_table()
    sheets = json.loads((HERE / 'drawing_sheets_1126.json').read_text(encoding='utf8'))
    have = {s['name'] for s in sheets if s['views']}
    parts = {r['part'] for r in rows}
    groups = {r['group'] for r in rows}
    missing = sorted(parts - have)
    report = dict(unique_parts=len(parts), with_sheet=len(parts) - len(missing), missing_parts=missing,
                  missing_groups=sorted(groups - have), other_sheets=sorted(have - parts - groups))
    (HERE / 'drawing_coverage_1126.json').write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding='utf8')
    say(f'coverage: {report}')
    return report


# ---------------------------------------------------------------- exploded view
POSES = {'D_02': (0, 160, 0), 'D_03': (80, 510, 0), 'D_04': (190, 540, 120), 'D_05': (585, 540, 180)}     # mm, as in edition3


def exploded_source():
    f = folder()
    assert not any(x.name == EXPLODED for x in f.dataFiles), EXPLODED + ' exists already'
    doc, d = import_copy()
    root = d.rootComponent
    table = json.loads(root.attributes.itemByName('RPRR', 'SpinningParts').value)
    for m in list(root.motionLinks):
        m.deleteMe()
    for j in list(root.asBuiltJoints):
        j.deleteMe()
    groups = {o.component.name[:4]: o for o in root.occurrences}
    released = 0
    for r in table['rotors']:
        c = groups[r['group']].component
        for rg in list(c.rigidGroups):
            rg.deleteMe()
        for j in list(c.asBuiltJoints):
            if j.name.startswith('spin_') or j.name == 'q2_Screw_spin':
                j.deleteMe()
        by = {o.name: o for o in c.occurrences}
        for n in r['parts']:
            if not by[n].isGroundToParent:
                by[n].isGroundToParent = True
                released += 1
    # every pinned part must be where its sub-assembly says
    worst = 0.0
    for k, g in groups.items():
        if not k.startswith('D_0'):
            continue
        natives = {o.name: o for o in g.component.occurrences}
        for o in g.childOccurrences:
            m = natives[o.name].transform2.copy()
            m.transformBy(g.transform2)
            worst = max(worst, max(abs(a - b) for a, b in zip(m.asArray(), o.transform2.asArray())))
    assert worst < 1e-9, worst
    for o in root.occurrences:
        if o.component.name.startswith('Wave_Drum'):
            o.isLightBulbOn = False
    for sk in root.sketches:
        sk.isVisible = False
    for k, p in POSES.items():
        o = groups[k]
        o.isGroundToParent = False
        m = core.Matrix3D.create()
        m.translation = core.Vector3D.create(*(v / 10 for v in p))
        o.transform2 = m
    if d.snapshots.hasPendingSnapshot:
        d.snapshots.add()
    for k, p in POSES.items():
        actual = groups[k].transform2.translation.asArray()
        assert math.dist(p, [10 * x for x in actual]) < 0.01, (k, p, actual)
    worst2 = 0.0
    for k, g in groups.items():
        if k.startswith('D_0'):
            natives = {o.name: o for o in g.component.occurrences}
            for o in g.childOccurrences:
                m = natives[o.name].transform2.copy()
                m.transformBy(g.transform2)
                worst2 = max(worst2, max(abs(a - b) for a, b in zip(m.asArray(), o.transform2.asArray())))
    assert worst2 < 1e-9, worst2
    app().activeViewport.fit()
    assert doc.saveAs(EXPLODED, f, 'Sub-assemblies pulled apart; model after the structure review of 2026-11-26', '')
    say(f'{EXPLODED}: {released} turning parts pinned again, links and joints removed, five sub-assemblies moved apart; saved')
    work_doc().activate()


def exploded_drawing():
    import adsk.drawing as dr
    src = data_file(EXPLODED)
    assert src and src.isComplete, 'the exploded source is still uploading'
    mgr = dr.DrawingManager.get()
    spec = mgr.createDrawingInput(src, dr.DrawingCreationModes.AutomaticDrawingCreationMode)
    spec.content = dr.DrawingContentTypes.VisibleOnlyDrawingContentType
    prefs = spec.automationPreferences.globalPreferences
    prefs.isSubAssemblySheetGenerated = False
    prefs.isComponentSheetGenerated = False
    prefs.isAnimationSheetGenerated = False
    say('exploded drawing requested ...')
    data = mgr.createDrawing(spec)
    say(f'exploded drawing: {data.name if data else None}')
    if data:
        state(exploded_drawing=data.id, exploded_url=data.fusionWebURL)


def export_exploded():
    st = state()
    file = app().data.findFileById(st['exploded_drawing'])
    export_pdf(file, 'E_Writing_1126_exploded.pdf', 'exploded_sheets_1126.json')
    work_doc().activate()


# ---------------------------------------------------------------- parts the automatic drawing leaves out
DETAILS = 'E_Writing_1126_Details'


def detail_source():
    """The automatic drawing skips every component whose name looks like a fastener (screw, nut ...), whatever the
    settings say; that also hits the ball screw, its nut and the nut bracket. As in edition3.detail_pack these
    parts are copied into a document of their own under neutral names (P01, P02 ...); the sheets get the real
    names back afterwards (detail_numbers_1126.json)."""
    import tempfile
    a = app()
    f = folder()
    assert not any(x.name == DETAILS for x in f.dataFiles), DETAILS + ' exists already'
    names = json.loads((HERE / 'drawing_coverage_1126.json').read_text(encoding='utf8'))['missing_parts']
    work_doc().activate()                               # the export of a component works only from the active document
    work = fusion.Design.cast(a.activeProduct)
    comps = {c.name: c for c in work.allComponents if c.name in names}
    assert len(comps) == len(names), sorted(set(names) - set(comps))
    mapping = {}
    with tempfile.TemporaryDirectory(prefix='Fusion_RPRR_details_') as staging:
        for name, c in comps.items():
            em = work.exportManager
            target = Path(staging) / (name + '.f3d')
            ok = em.execute(em.createFusionArchiveExportOptions(str(target), c))
            if not ok:
                # seen for D_Set_screw_M3x4: execute() answers False although the archive is written;
                # the copy is checked after the import (same number of bodies, same volume)
                ok = em.execute(em.createFusionArchiveExportOptions(str(target), c))
                say(f'export of {name}: first answer False, second {ok}, file {target.stat().st_size if target.exists() else None} bytes')
            assert target.exists() and target.stat().st_size > 0, name
        doc = a.documents.add(core.DocumentTypes.FusionDesignDocumentType)
        d = design_of(doc)
        root = d.rootComponent
        for i, name in enumerate(names):
            opt = a.importManager.createFusionArchiveImportOptions(str(Path(staging) / (name + '.f3d')))
            result = a.importManager.importToTarget2(opt, root)
            assert result and result.count, name
            occ = root.occurrences.item(root.occurrences.count - 1)
            comp = occ.component
            for child in list(comp.occurrences):          # the lock nut inside the ball screw: it has its own sheet
                child.deleteMe()
            assert comp.bRepBodies.count == comps[name].bRepBodies.count, name
            v_new = sum(b.volume for b in comp.bRepBodies)
            v_old = sum(b.volume for b in comps[name].bRepBodies)
            assert abs(v_new - v_old) < 1e-6 * max(v_old, 1e-6), (name, v_new, v_old)
            neutral = 'P%02d' % (i + 1)
            comp.name = neutral
            mapping[neutral] = name
            m = core.Matrix3D.create()
            m.translation = core.Vector3D.create(30.0 * (i % 5), 0, 30.0 * (i // 5))
            occ.transform2 = m
        if d.snapshots.hasPendingSnapshot:
            d.snapshots.add()
        assert {o.component.name for o in root.occurrences} == set(mapping)
        a.activeViewport.fit()
        assert doc.saveAs(DETAILS, f, 'Parts the automatic drawing leaves out (names like fasteners), under neutral names', '')
    (HERE / 'detail_numbers_1126.json').write_text(json.dumps(mapping, ensure_ascii=False, indent=1), encoding='utf8')
    say(f'{DETAILS}: {len(mapping)} parts saved under neutral names')
    work_doc().activate()


def detail_drawing():
    import adsk.drawing as dr
    src = data_file(DETAILS)
    assert src and src.isComplete, 'the detail source is still uploading'
    mgr = dr.DrawingManager.get()
    spec = mgr.createDrawingInput(src, dr.DrawingCreationModes.AutomaticDrawingCreationMode)
    spec.content = dr.DrawingContentTypes.VisibleOnlyDrawingContentType
    spec.sheetCreationType = dr.SheetCreationTypes.AllLevelsSheetCreationType
    prefs = spec.automationPreferences.globalPreferences
    prefs.isMainAssemblySheetGenerated = False
    prefs.isSubAssemblySheetGenerated = False
    prefs.isComponentSheetGenerated = True
    prefs.isAutoDimensionEnabled = True
    prefs.isAnimationSheetGenerated = False
    prefs.isDetectAndOmitFasteners = False
    say('detail drawing requested ...')
    data = mgr.createDrawing(spec)
    say(f'detail drawing: {data.name if data else None}')
    if data:
        state(detail_drawing=data.id, detail_url=data.fusionWebURL)


def export_detail():
    import adsk.drawing as dr
    st = state()
    file = app().data.findFileById(st['detail_drawing']) if st.get('detail_drawing') else data_file(DETAILS + ' \u5de5\u7a0b\u56fe', 'f2d')   # Chinese-language Fusion names: drawing
    doc = open_data(file)
    doc.activate()
    drawing = dr.DrawingDocument.cast(doc).drawing
    mapping = json.loads((HERE / 'detail_numbers_1126.json').read_text(encoding='utf8'))
    for sh in list(drawing.sheets):
        if sh.name in mapping:
            sh.name = mapping[sh.name]
    wanted = set(mapping.values())
    have = {sh.name for sh in drawing.sheets if sh.views.count}
    if wanted.issubset(have):
        for sh in list(drawing.sheets):
            if sh.name not in wanted:
                sh.deleteMe()
    if doc.isModified:
        doc.save('Sheets named after the parts')
    state(detail_drawing=file.id, detail_url=file.fusionWebURL)
    sheets = export_pdf(file, 'E_Writing_1126_details.pdf', 'detail_sheets_1126.json')
    say(f'detail sheets missing: {sorted(wanted - {s["name"] for s in sheets if s["views"]})}')
    work_doc().activate()
