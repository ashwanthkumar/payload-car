# -*- coding: utf-8 -*-
"""Parametric ESP32 control PCB for the payload rover (Fusion API).

Mechanical PCB model - board outline, mounting holes and the component stack - built as
its own document and xref-inserted into the rover by ``assemble.py`` (vertical, on the
rear-cabinet standoffs, components facing the access doors). The electrical design
(schematic/copper) is done later in ECAD; this model owns the form factor.

Architecture: a CARRIER BOARD. The ESP32 module and the four BLDC driver modules sit on
it as daughterboards; power enters via XT60 + fuse, a buck drops 36 V to 5 V logic, and
each hub motor gets a 3-pin phase terminal + 5-pin hall connector along the bottom edge.

Local coordinates: board centred on origin, XY plane, +Z = component side.
Mounting holes match the cabinet standoffs in builder.py: inset (holeInset) from the
edges of a boardW x boardH board.
"""

try:  # adsk only exists inside Fusion
    import adsk.core
    import adsk.fusion
except ImportError:  # pragma: no cover
    adsk = None

PARAMS = {
    'boardW':     [240.0, 'Board width (X) - matches builder pcbWidth'],
    'boardH':     [140.0, 'Board height (Y) - matches builder pcbHeight'],
    'boardThk':   [1.6,   'FR4 thickness - matches builder pcbThickness'],
    'holeInset':  [7.0,   'Mounting-hole inset from board edges (= builder standoffDiameter)'],
    'holeDia':    [3.2,   'M3 mounting hole diameter'],
    'esp32W':     [28.0,  'ESP32 dev-module width (X)'],
    'esp32L':     [55.0,  'ESP32 dev-module length (Y)'],
    'drvSize':    [46.0,  'Motor-driver daughterboard size (square)'],
    'drvBodyH':   [12.0,  'Motor-driver body height above its board'],
    'finH':       [8.0,   'Driver heatsink fin height'],
    'termW':      [20.0,  '3-pin motor phase terminal block width'],
    'jstW':       [12.0,  '5-pin hall JST connector width'],
}

P = {k: v[0] for k, v in PARAMS.items()}

APP_PCB = 'Plastic - Matte (Green)'
APP_MODULE = 'Plastic - Glossy (Blue)'
APP_BLACK = 'Plastic - Glossy (Black)'
APP_WHITE = 'Plastic - Glossy (White)'
APP_CHROME = 'Stainless Steel - Polished'
APP_ALU = 'Aluminum - Anodized Glossy (Grey)'
APP_YELLOW = 'Paint - Enamel Glossy (Yellow)'
APP_RED = 'Plastic - Glossy (Red)'

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


def _plane(comp, expr):
    pin = comp.constructionPlanes.createInput()
    pin.setByOffset(comp.xYConstructionPlane, vS(expr))
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


def _rect_profile(sk, cx, cy, l, w, cxe, cye, lexpr, wexpr):
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
    sk = comp.sketches.add(_plane(comp, zbase))
    prof = _rect_profile(sk, _v(cx), _v(cy), _v(l), _v(w), cx, cy, l, w)
    ext = comp.features.extrudeFeatures.addSimple(prof, vS(h), NEWBODY)
    ext.bodies.item(0).name = name
    sk.name = name + '_sk'
    _paint(ext.bodies.item(0), appr)
    return ext.bodies.item(0)


def cyl_z(comp, name, cx, cy, zbase, dia, h, appr=None):
    sk = comp.sketches.add(_plane(comp, zbase))
    c = sk.sketchCurves.sketchCircles.addByCenterRadius(
        P3(_v(cx) / 10.0, _v(cy) / 10.0, 0), _v(dia) / 20.0)
    sk.sketchDimensions.addDiameterDimension(
        c, P3(_v(cx) / 10.0 + _v(dia) / 20.0, _v(cy) / 10.0, 0)).parameter.expression = dia
    _locate(sk, c.centerSketchPoint, cx, cy)
    prof = sk.profiles.item(0)
    ext = comp.features.extrudeFeatures.addSimple(prof, vS(h), NEWBODY)
    ext.bodies.item(0).name = name
    sk.name = name + '_sk'
    _paint(ext.bodies.item(0), appr)
    return ext.bodies.item(0)


def hole(comp, body, cx, cy, dia):
    """Through-hole in the board."""
    sk = comp.sketches.add(_plane(comp, '-1 mm'))
    c = sk.sketchCurves.sketchCircles.addByCenterRadius(
        P3(_v(cx) / 10.0, _v(cy) / 10.0, 0), _v(dia) / 20.0)
    sk.sketchDimensions.addDiameterDimension(
        c, P3(_v(cx) / 10.0 + _v(dia) / 20.0, _v(cy) / 10.0, 0)).parameter.expression = dia
    _locate(sk, c.centerSketchPoint, cx, cy)
    prof = sk.profiles.item(0)
    ef = comp.features.extrudeFeatures
    inp = ef.createInput(prof, CUTOP)
    inp.setDistanceExtent(False, vS('boardThk + 2'))
    inp.participantBodies = [body]
    ef.add(inp)
    sk.name = 'hole_sk'


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

    TOP = 'boardThk'                      # component-side surface
    board_c = _comp('Board')
    esp = _comp('ESP32_Module')
    drv = _comp('Motor_Drivers')
    pwr = _comp('Power')
    io = _comp('Motor_IO')

    # ---- FR4 carrier board with 4x M3 mounting holes (match the cabinet standoffs) ----
    board = box(board_c, 'PCB', '0', '0', '0', 'boardW', 'boardH', 'boardThk', APP_PCB)
    for sx in ('+', '-'):
        for sy in ('+', '-'):
            hole(board_c, board, sx + '(boardW/2 - holeInset)', sy + '(boardH/2 - holeInset)', 'holeDia')

    # ---- ESP32 dev module, top centre: carrier + RF can + micro-USB + boot/reset buttons ----
    box(esp, 'ESP32_Carrier', '0', '30', TOP, 'esp32W', 'esp32L', '1.6', APP_MODULE)
    box(esp, 'ESP32_RF_Can', '0', '38', '(' + TOP + ') + 1.6', '18', '26', '3.2', APP_CHROME)
    box(esp, 'ESP32_USB', '0', '6', '(' + TOP + ') + 1.6', '8', '6', '3', APP_CHROME)
    box(esp, 'Btn_Boot', '-9', '10', '(' + TOP + ') + 1.6', '4', '3', '2', APP_BLACK)
    box(esp, 'Btn_Reset', '9', '10', '(' + TOP + ') + 1.6', '4', '3', '2', APP_BLACK)
    for i, (led, col) in enumerate((('LED_PWR', APP_RED), ('LED_STAT', APP_WHITE))):
        cyl_z(esp, led, str(22 + 8 * i), '60', TOP, '3', '1.5', col)

    # ---- 4x BLDC driver daughterboards in a row, finned heatsinks on top ----
    for nm, dx in (('FL', '-86'), ('FR', '-29'), ('RL', '29'), ('RR', '86')):
        box(drv, 'Drv_%s_Board' % nm, dx, '-22', TOP, 'drvSize', 'drvSize', '1.6', APP_MODULE)
        box(drv, 'Drv_%s_Body' % nm, dx, '-22', '(' + TOP + ') + 1.6',
            'drvSize - 6', 'drvSize - 6', 'drvBodyH', APP_BLACK)
        for f in range(4):
            box(drv, 'Drv_%s_Fin%d' % (nm, f), dx, str(-22 - 15 + 10 * f),
                '(' + TOP + ') + 1.6 + drvBodyH', 'drvSize - 6', '3', 'finH', APP_ALU)

    # ---- power: XT60 input, blade fuse, 36V->5V buck module, E-stop loop terminal ----
    box(pwr, 'XT60_In', '-105', '52', TOP, '16', '16', '8', APP_YELLOW)
    box(pwr, 'Fuse_Holder', '-105', '30', TOP, '22', '10', '10', APP_BLACK)
    box(pwr, 'Buck_Board', '62', '52', TOP, '43', '21', '1.6', APP_MODULE)
    cyl_z(pwr, 'Buck_Inductor', '52', '52', '(' + TOP + ') + 1.6', '12', '7', APP_ALU)
    cyl_z(pwr, 'Buck_Cap', '74', '52', '(' + TOP + ') + 1.6', '8', '10', APP_BLACK)
    box(pwr, 'Estop_Term', '105', '52', TOP, '14', '12', '10', APP_RED)

    # ---- per-motor IO along the bottom edge: 3-pin phase terminal + 5-pin hall JST ----
    for nm, dx in (('FL', '-86'), ('FR', '-29'), ('RL', '29'), ('RR', '86')):
        t = box(io, 'Phase_%s' % nm, dx, '-62', TOP, 'termW', '12', '10', APP_PCB)
        _paint(t, APP_PCB)
        for k in range(3):
            cyl_z(io, 'Phase_%s_Screw%d' % (nm, k), str(int(dx) - 6 + 6 * k), '-62',
                  '(' + TOP + ') + 10', '4', '1.5', APP_CHROME)
        box(io, 'Hall_%s' % nm, dx, '-48', TOP, 'jstW', '6', '6', APP_WHITE)

    app.activeViewport.fit()
    return _design


def export_f3d(design, path):
    em = design.exportManager
    em.execute(em.createFusionArchiveExportOptions(path))
    return path
