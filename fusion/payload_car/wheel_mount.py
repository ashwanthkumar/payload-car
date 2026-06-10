# -*- coding: utf-8 -*-
"""Parametric hub-motor wheel-mount assembly (Fusion API).

Sized from the Robokits 10" hub-motor drawing (RKI 9051):
  axle Ø11.62, total axle length 236.30, hub flange Ø142.30, hub body width 91.0.
Axle flat-to-flat and tyre OD/width are NOT on that drawing -> placeholders (measure & edit).

Coordinates: wheel axis = Y, wheel centre at origin; Z up; X = chassis-rail direction.
Build by calling ``build(app)``. Every dimension is a Fusion User Parameter.
"""

import math

try:  # adsk only exists inside Fusion
    import adsk.core
    import adsk.fusion
except ImportError:  # pragma: no cover
    adsk = None

PARAMS = {
    # --- hub motor (from RKI 9051 drawing) ---
    'axleRound':      [16.0,   'Shaft round-side diameter (datasheet)'],
    'axleFlat':       [13.5,   'Shaft flat-side width / flat-to-flat (datasheet)'],
    'shaftLen':       [57.0,   'Shaft length protruding (single drive side, datasheet)'],
    'threadDia':      [12.0,   'M12 threaded tip diameter (axle nut; measured ~11.5)'],
    'hubBodyWidth':   [91.0,   'Hub shell width, flange to flange (RKI 9051 drawing)'],
    'flangeDia':      [142.30, 'Hub flange diameter (drawing)'],
    'centreBoss':     [44.05,  'Hub centre boss width (drawing)'],
    'wheelOD':        [254.0,  'Tyre outer diameter, 10in (MEASURE)'],
    'wheelWidth':     [85.0,   'Tyre width (MEASURE)'],
    'tyreInner':      [148.0,  'Tyre inner diameter - exposes the hub face'],
    'tyreShoulder':   [34.0,   'Tyre shoulder round-over (fat balloon profile)'],
    'bossDia':        [55.0,   'Hub centre boss diameter'],
    'bossProtrude':   [6.0,    'Hub centre boss protrusion'],
    'boltCircleDia':  [105.0,  'Hub-face bolt circle diameter'],
    'boltHeadDia':    [9.0,    'Hub-face cap-screw head diameter'],
    'boltHeadH':      [3.0,    'Hub-face cap-screw head height'],
    'wireDia':        [5.0,    'Motor phase wire diameter'],
    'wireLen':        [90.0,   'Motor phase wire visible length'],
    # --- dropout plate (the mounting interface: the rover's rocker arm bolts onto it) ---
    'plateThk':       [8.0,    'Dropout plate thickness'],
    'plateW':         [90.0,   'Dropout plate width (X)'],
    'plateH':         [175.0,  'Dropout plate height (Z) - covers the rocker-arm bolt pattern'],
    'plateDrop':      [45.0,   'Plate length below axle (open slot end)'],
    'plateClear':     [12.0,   'Clearance from tyre face to plate inner face'],
    'slotClear':      [0.2,    'Slot/seat clearance on the axle'],
    # --- torque arm ---
    'torqueThk':      [6.0,    'Torque arm thickness'],
    'torqueLen':      [95.0,   'Torque arm length - stays below the rocker-arm band on the plate'],
    'torqueW':        [40.0,   'Torque arm width'],
    'boltDia':        [9.0,    'M8 clearance hole (through arm + plate)'],
    'boltSpacing':    [60.0,   'Torque-arm bolt hole spacing'],
    'm8Dia':          [8.0,    'M8 cap-screw shaft diameter'],
    'm8HeadDia':      [13.0,   'M8 socket-head cap-screw head diameter'],
    'm8HeadH':        [8.0,    'M8 cap-screw head height'],
    'm8WasherOD':     [16.0,   'M8 washer outer diameter'],
    'm8WasherThk':    [1.6,    'M8 washer thickness'],
    'm8NutAF':        [13.0,   'M8 hex nut across-flats (modelled round)'],
    'm8NutH':         [6.5,    'M8 hex nut height'],
    # --- fasteners ---
    'nutAF':          [18.0,   'M12 axle nut across-flats (DIN 934)'],
    'nutW':           [10.0,   'M12 axle nut height'],
    'washerOD':       [24.0,   'M12 axle washer outer diameter'],
    'washerThk':      [2.5,    'M12 axle washer thickness'],
}

P = {k: v[0] for k, v in PARAMS.items()}

APP_STEEL = 'Stainless Steel - Satin'
APP_CHROME = 'Stainless Steel - Polished'
APP_TYRE = 'Rubber - Hard'
APP_ALU = 'Aluminum - Anodized Glossy (Grey)'
APP_HUB = 'Plastic - Glossy (Black)'
APP_ARM = 'Paint - Enamel Glossy (Red)'
APP_BOLT = 'Coating - Black Oxide'

_app = _design = _root = None
P3 = vS = None
H = V = NEWBODY = CUTOP = None
_APP_CACHE = {}


def _v(expr):
    return float(eval(expr, {'__builtins__': {}}, P))


def _pos(expr):
    return expr if _v(expr) >= 0 else '-(' + expr + ')'


def _appearance(name):
    if name in _APP_CACHE:
        return _APP_CACHE[name]
    appr = _design.appearances.itemByName(name)
    if appr is None:
        lib = _app.materialLibraries.itemByName('Fusion Appearance Library')
        appr = _design.appearances.addByCopy(lib.appearances.itemByName(name), name)
    _APP_CACHE[name] = appr
    return appr


def _paint(body, name):
    if name:
        body.appearance = _appearance(name)


def _plane(comp, base, expr):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(base, vS(expr))
    return comp.constructionPlanes.add(pin)


def _comp(name):
    occ = _root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


def _locate(sk, pt, uexpr, vexpr):
    o = sk.originPoint
    g = pt.geometry
    if abs(_v(uexpr)) < 1e-6:
        sk.geometricConstraints.addVerticalPoints(o, pt)
    else:
        sk.sketchDimensions.addDistanceDimension(
            o, pt, H, P3(g.x / 2.0, g.y - 1.0, 0)).parameter.expression = _pos(uexpr)
    if abs(_v(vexpr)) < 1e-6:
        sk.geometricConstraints.addHorizontalPoints(o, pt)
    else:
        sk.sketchDimensions.addDistanceDimension(
            o, pt, V, P3(g.x - 1.0, g.y / 2.0, 0)).parameter.expression = _pos(vexpr)


def _circle(sk, u_mm, v_mm, dia_mm, uexpr, vexpr, dexpr):
    c = sk.sketchCurves.sketchCircles.addByCenterRadius(
        P3(u_mm / 10.0, v_mm / 10.0, 0), dia_mm / 2.0 / 10.0)
    sk.sketchDimensions.addDiameterDimension(
        c, P3(u_mm / 10.0 + dia_mm / 20.0, v_mm / 10.0, 0)).parameter.expression = dexpr
    _locate(sk, c.centerSketchPoint, uexpr, vexpr)
    return sk.profiles.item(0)


def _rect_profile(sk, cx, cy, l, w, cxe, cye, lexpr, wexpr):
    """Draw + fully constrain a rectangle; return its profile (fetched LAST, after all dims)."""
    x0, y0 = (cx - l / 2) / 10.0, (cy - w / 2) / 10.0
    x1, y1 = (cx + l / 2) / 10.0, (cy + w / 2) / 10.0
    lines = sk.sketchCurves.sketchLines
    a = lines.addByTwoPoints(P3(x0, y0, 0), P3(x1, y0, 0))
    b = lines.addByTwoPoints(a.endSketchPoint, P3(x1, y1, 0))
    c = lines.addByTwoPoints(b.endSketchPoint, P3(x0, y1, 0))
    lines.addByTwoPoints(c.endSketchPoint, a.startSketchPoint)
    sk.geometricConstraints.addHorizontal(a)
    sk.geometricConstraints.addHorizontal(c)
    sk.geometricConstraints.addVertical(b)
    sk.sketchDimensions.addDistanceDimension(
        a.startSketchPoint, a.endSketchPoint, H, P3((x0 + x1) / 2, y0 - 0.5, 0)).parameter.expression = lexpr
    sk.sketchDimensions.addDistanceDimension(
        b.startSketchPoint, b.endSketchPoint, V, P3(x1 + 0.5, (y0 + y1) / 2, 0)).parameter.expression = wexpr
    _locate(sk, a.startSketchPoint, '(' + cxe + ') - (' + lexpr + ')/2', '(' + cye + ') - (' + wexpr + ')/2')
    return sk.profiles.item(0)


def box(comp, name, cx, cy, zbase, l, w, h, appr=None):
    sk = comp.sketches.add(_plane(comp, comp.xYConstructionPlane, zbase))
    prof = _rect_profile(sk, _v(cx), _v(cy), _v(l), _v(w), cx, cy, l, w)
    ext = comp.features.extrudeFeatures.addSimple(prof, vS(h), NEWBODY)
    ext.bodies.item(0).name = name
    sk.name = name + '_sk'
    _paint(ext.bodies.item(0), appr)
    return ext.bodies.item(0)


def cut_box(comp, body, cx, cy, zbase, l, w, h):
    sk = comp.sketches.add(_plane(comp, comp.xYConstructionPlane, zbase))
    prof = _rect_profile(sk, _v(cx), _v(cy), _v(l), _v(w), cx, cy, l, w)
    ef = comp.features.extrudeFeatures
    inp = ef.createInput(prof, CUTOP)
    inp.setDistanceExtent(False, vS(h))
    inp.participantBodies = [body]
    ef.add(inp)
    sk.name = 'cut_sk'


def cyl_y(comp, name, cx, cz, yc, dia, length, appr=None):
    yb = '(' + yc + ') - (' + length + ')/2'
    sk = comp.sketches.add(_plane(comp, comp.xZConstructionPlane, yb))
    prof = _circle(sk, _v(cx), -_v(cz), _v(dia), cx, '-(' + cz + ')', dia)
    ext = comp.features.extrudeFeatures.addSimple(prof, vS(length), NEWBODY)
    ext.bodies.item(0).name = name
    sk.name = name + '_sk'
    _paint(ext.bodies.item(0), appr)
    return ext.bodies.item(0)


def bore_y(comp, body, cx, cz, yc, dia, length):
    yb = '(' + yc + ') - (' + length + ')/2'
    sk = comp.sketches.add(_plane(comp, comp.xZConstructionPlane, yb))
    prof = _circle(sk, _v(cx), -_v(cz), _v(dia), cx, '-(' + cz + ')', dia)
    ef = comp.features.extrudeFeatures
    inp = ef.createInput(prof, CUTOP)
    inp.setDistanceExtent(False, vS(length))
    inp.participantBodies = [body]
    ef.add(inp)
    sk.name = 'bore_sk'


def dhole_y(comp, body, cx_mm, cz_mm, yc, round_dia_mm, flat_mm, length):
    """Cut a double-flat (D) hole, axis Y, sized to the axle round+flats."""
    yb = '(' + yc + ') - (' + length + ')/2'
    sk = comp.sketches.add(_plane(comp, comp.xZConstructionPlane, yb))
    r = (round_dia_mm + 0.2) / 2.0 / 10.0
    hf = (flat_mm + 0.2) / 2.0 / 10.0
    u0, v0 = cx_mm / 10.0, -cz_mm / 10.0
    xf = math.sqrt(max(r * r - hf * hf, 0.0))
    tl, tr = P3(u0 - xf, v0 + hf, 0), P3(u0 + xf, v0 + hf, 0)
    bl, br = P3(u0 - xf, v0 - hf, 0), P3(u0 + xf, v0 - hf, 0)
    rt, lt = P3(u0 + r, v0, 0), P3(u0 - r, v0, 0)
    lines, arcs = sk.sketchCurves.sketchLines, sk.sketchCurves.sketchArcs
    lines.addByTwoPoints(tl, tr)
    arcs.addByThreePoints(tr, rt, br)
    lines.addByTwoPoints(br, bl)
    arcs.addByThreePoints(bl, lt, tl)
    ef = comp.features.extrudeFeatures
    inp = ef.createInput(sk.profiles.item(0), CUTOP)
    inp.setDistanceExtent(False, vS(length))
    inp.participantBodies = [body]
    ef.add(inp)
    sk.name = 'dhole_sk'


def _fillet_radius(body, radius_mm, radius_expr):
    """Round circular edges matching radius_mm (cosmetic; failures ignored)."""
    try:
        r = radius_mm / 10.0
        edges = adsk.core.ObjectCollection.create()
        circ = adsk.core.Circle3D.classType()
        for e in body.edges:
            g = e.geometry
            if g.objectType == circ and abs(g.radius - r) < 0.05:
                edges.add(e)
        if edges.count:
            ff = body.parentComponent.features.filletFeatures
            fin = ff.createInput()
            fin.addConstantRadiusEdgeSet(edges, vS(radius_expr), True)
            ff.add(fin)
    except Exception:  # noqa: BLE001
        pass


def _flat(comp, axle, y_from, y_to):
    """Mill top & bottom flats onto the axle across a Y range."""
    midy = '(' + y_from + ' + ' + y_to + ')/2'
    span = '(' + y_to + ') - (' + y_from + ')'
    cut_box(comp, axle, '0', midy, 'axleFlat/2', 'axleRound + 2', span, 'axleRound')
    cut_box(comp, axle, '0', midy, '-axleFlat/2 - axleRound', 'axleRound + 2', span, 'axleRound')


def build(app):
    global _app, _design, _root, P3, vS, H, V, NEWBODY, CUTOP, _APP_CACHE
    _app = app
    _APP_CACHE = {}
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    _design = adsk.fusion.Design.cast(app.activeProduct)
    _design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    _design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    _root = _design.rootComponent

    P3 = adsk.core.Point3D.create
    vS = adsk.core.ValueInput.createByString
    H = adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation
    V = adsk.fusion.DimensionOrientations.VerticalDimensionOrientation
    NEWBODY = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
    CUTOP = adsk.fusion.FeatureOperations.CutFeatureOperation

    for name, (val, comment) in PARAMS.items():
        _design.userParameters.add(name, vS('{} mm'.format(val)), 'mm', comment)
    _design.userParameters.add('magnetPolePairs', vS('25'), '', '25 magnet pole pairs (electrical)')

    PLATE_Y = 'wheelWidth/2 + plateClear + plateThk/2'   # plate centre offset from wheel centre

    motor = _comp('HubMotor')
    mount = _comp('Mount')
    arm = _comp('TorqueArm')
    fast = _comp('Fasteners')

    # ---- hub motor (reference, from drawing) ----
    # single-sided (cantilever) hub motor, +Y drive side only. Stepped axle:
    # Ø16 round body -> 13.5 flats (dropout + torque arm) -> M12 threaded tip (axle nut).
    thread_start = '(' + PLATE_Y + ') + plateThk/2 + torqueThk'
    axle = cyl_y(motor, 'Axle', '0', '0', '(-hubBodyWidth/2 + (' + thread_start + '))/2',
                 'axleRound', '(' + thread_start + ') + hubBodyWidth/2', APP_CHROME)
    _flat(motor, axle, 'hubBodyWidth/2', thread_start)                      # 13.5 flats
    thread = cyl_y(motor, 'Axle_Thread', '0', '0',
                   '((' + thread_start + ') + (hubBodyWidth/2 + shaftLen))/2', 'threadDia',
                   '(hubBodyWidth/2 + shaftLen) - (' + thread_start + ')', APP_CHROME)
    hub = cyl_y(motor, 'Hub_Shell', '0', '0', '0', 'flangeDia', 'hubBodyWidth', APP_ALU)
    tyre = cyl_y(motor, 'Tyre', '0', '0', '0', 'wheelOD', 'wheelWidth', APP_TYRE)
    bore_y(motor, tyre, '0', '0', '0', 'tyreInner', 'wheelWidth + 2')   # ring -> hub face shows
    _fillet_radius(tyre, _v('wheelOD') / 2.0, 'tyreShoulder')           # fat rounded tread
    # aluminium side covers: raised centre boss + 6-screw bolt circle each side (per photos)
    for side, sgn in (('L', '-'), ('R', '+')):
        cyl_y(motor, 'Boss_' + side, '0', '0', sgn + '(hubBodyWidth/2 + bossProtrude/2)',
              'bossDia', 'bossProtrude', APP_ALU)
        yhead = sgn + '(hubBodyWidth/2 + boltHeadH/2)'
        for i in range(6):
            th = math.radians(60 * i + 30)
            cyl_y(motor, 'Bolt_%s%d' % (side, i),
                  'boltCircleDia/2 * (%.5f)' % math.cos(th),
                  'boltCircleDia/2 * (%.5f)' % math.sin(th),
                  yhead, 'boltHeadDia', 'boltHeadH', APP_CHROME)
    # 3 phase wires exiting the hollow drive-side axle end (beyond the nut)
    wstart = '(hubBodyWidth/2 + shaftLen) + wireLen/2'
    wires = (('Plastic - Glossy (Green)', '0', 'wireDia*0.7'),
             ('Plastic - Glossy (Blue)', '-wireDia*0.6', '-wireDia*0.4'),
             ('Plastic - Glossy (Red)', 'wireDia*0.6', '-wireDia*0.4'))
    for i, (col, ox, oz) in enumerate(wires):
        cyl_y(motor, 'Wire_%d' % (i + 1), ox, oz, wstart, 'wireDia', 'wireLen', col)

    # ---- single-sided cantilever mount on the drive side (slot opens downward) ----
    cy = '(' + PLATE_Y + ')'
    plate = box(mount, 'Dropout', '0', cy, '-plateDrop', 'plateW', 'plateThk', 'plateH', APP_STEEL)
    cut_box(mount, plate, '0', cy, '-plateDrop - 1', 'axleFlat + slotClear', 'plateThk + 2', 'plateDrop + 1')
    bore_y(mount, plate, '0', '0', cy, 'axleRound + slotClear', 'plateThk + 2')
    # NOTE: no chassis tie here. The dropout plate IS the mounting interface - on the rover it
    # bolts to the rocker-suspension arm (M10 pattern), so the wheel articulates with the arm.

    # ---- torque arm: bottom D-hole clamps the axle flats; upper bolts fasten it to the
    #      dropout plate. It sits flush against the plate's outboard face. ----
    arm_y = '(' + PLATE_Y + ') + plateThk/2 + torqueThk/2'   # flush on the plate outer face
    bar = box(arm, 'Torque_Arm', '0', arm_y, '-axleRound', 'torqueW', 'torqueThk', 'torqueLen', APP_ARM)
    dhole_y(arm, bar, 0.0, 0.0, arm_y, P['axleRound'], P['axleFlat'], 'torqueThk + 2')  # axle clamp
    arm_outer = '(' + PLATE_Y + ') + plateThk/2 + torqueThk'   # +Y outboard face of the arm
    plate_inner = '(' + PLATE_Y + ') - plateThk/2'             # -Y back face of the plate
    for k, zc in enumerate(('torqueLen*0.45', 'torqueLen*0.78'), 1):   # 2 M8 bolts
        bore_y(arm, bar, '0', zc, arm_y, 'boltDia', 'torqueThk + 2')
        bore_y(mount, plate, '0', zc, '(' + PLATE_Y + ')', 'boltDia', 'plateThk + 2')
        # explicit M8 socket-head cap screw + washer + hex nut (through arm and plate)
        cyl_y(fast, 'M8_Screw_Head_%d' % k, '0', zc, '(' + arm_outer + ') + m8HeadH/2',
              'm8HeadDia', 'm8HeadH', APP_BOLT)
        sh_ctr = '((' + arm_outer + ') + (' + plate_inner + ') - m8WasherThk - m8NutH)/2'
        sh_len = '(' + arm_outer + ') - ((' + plate_inner + ') - m8WasherThk - m8NutH)'
        cyl_y(fast, 'M8_Screw_%d' % k, '0', zc, sh_ctr, 'm8Dia', sh_len, APP_BOLT)
        cyl_y(fast, 'M8_Washer_%d' % k, '0', zc, '(' + plate_inner + ') - m8WasherThk/2',
              'm8WasherOD', 'm8WasherThk', APP_CHROME)
        cyl_y(fast, 'M8_Nut_%d' % k, '0', zc, '(' + plate_inner + ') - m8WasherThk - m8NutH/2',
              'm8NutAF', 'm8NutH', APP_CHROME)

    # ---- M12 axle washer + nut on the threaded tip, bearing against the torque arm ----
    stack = '(' + PLATE_Y + ') + plateThk/2 + torqueThk'
    cyl_y(fast, 'M12_Axle_Washer', '0', '0', stack + ' + washerThk/2', 'washerOD', 'washerThk', APP_CHROME)
    cyl_y(fast, 'M12_Axle_Nut', '0', '0', stack + ' + washerThk + nutW/2', 'nutAF', 'nutW', APP_CHROME)

    # ---- motor metadata + datasheet mass (2.30 kg applied to the motor metal) ----
    for k, val in (('weight_kg', '2.30'), ('magnetPolePairs', '25'), ('shaftRound_mm', '16.0'),
                   ('shaftFlat_mm', '13.5'), ('shaftThread', 'M12'), ('shaftLen_mm', '57.0'),
                   ('ipRating', 'IP44')):
        motor.attributes.add('HubMotor', k, val)
    try:
        lib = app.materialLibraries.itemByName('Fusion Material Library')
        mat = _design.materials.addByCopy(lib.materials.itemByName('Steel'), 'HubMotor 2.30kg')
        vol = (axle.physicalProperties.volume + hub.physicalProperties.volume
               + thread.physicalProperties.volume)  # cm^3
        dprop = mat.materialProperties.itemById('PhysicalMaterialAssetPropertyDensity')
        if dprop is None:
            for i in range(mat.materialProperties.count):
                pp = mat.materialProperties.item(i)
                if 'densit' in pp.name.lower():
                    dprop = pp
                    break
        dprop.value = 2.30 / (vol * 1e-6)  # density FloatProperty is SI kg/m^3; vol is cm^3
        axle.material = mat
        hub.material = mat
        thread.material = mat
        print('motor mass set: %.3f kg over %.1f cm^3' % (
            axle.physicalProperties.mass + hub.physicalProperties.mass
            + thread.physicalProperties.mass, vol))
    except Exception as e:  # noqa: BLE001 - mass override is optional metadata
        print('mass override skipped:', e)

    app.activeViewport.fit()
    return _design


def export_f3d(design, path):
    em = design.exportManager
    em.execute(em.createFusionArchiveExportOptions(path))
    return path
