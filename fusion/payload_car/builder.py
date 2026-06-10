# -*- coding: utf-8 -*-
"""Parametric RC payload-car builder (Fusion API).

Pure builder: no exception swallowing (except cosmetic fillets), no UI. Call ``build(app)``
from a Fusion script, an add-in, or the Fusion MCP. Every dimension is a Fusion User Parameter
so the model regenerates from Modify -> Change Parameters. Defaults target a ~200 kg payload.

Layout: skid-steer 4WD rover on four single-sided hub motors (no steering linkage), welded
square-tube payload bay, rear electronics cabinet (battery / ESP32 PCB / 4 motor drivers /
E-stop), and an optional walking-beam rocker suspension (its own deletable component). The
hub-motor wheel modules are a separate component (components/hub_motor.f3d) inserted by
``assemble.py``; this file builds everything that is welded to the car itself.
Coordinates: +X forward, +Y left, +Z up; ground at z=0; model centred on origin.
"""

try:  # adsk only exists inside Fusion's interpreter; allow offline import for validation/CLI
    import adsk.core
    import adsk.fusion
except ImportError:  # pragma: no cover - offline (uv) path
    adsk = None

# name -> [millimetre default, comment]  (each becomes an editable Fusion User Parameter)
PARAMS = {
    # --- chassis ---
    'chassisLength':        [900.0,  'Overall chassis length (X)'],
    'chassisWidth':         [605.0,  'Overall chassis width (Y) - flush with the wheel outer faces'],
    'basePlateThickness':   [12.0,   'Load-bearing base plate (sized for 200 kg payload)'],
    'frameRailThickness':   [30.0,   'Side rail wall thickness'],
    'frameRailHeight':      [80.0,   'Side rail / cross member height'],
    'crossMemberWidth':     [40.0,   'Front/rear cross member width (X)'],
    'groundClearance':      [320.0,  'Base plate to ground - high enough for rocker travel (tyre top 254 + ~43 swing + margin)'],
    'bedThickness':         [10.0,   'Payload bed deck thickness'],
    'bumperDiameter':       [22.0,   'Tubular bumper diameter'],
    # --- payload bay fabrication (4 MS square-tube columns + steel sheet sides) ---
    'tubeSize':             [40.0,   'MS square-tube corner column size'],
    'sheetThk':             [12.0,   'Steel sheet panel thickness (bay/cabinet walls)'],
    'bayWallHeight':        [180.0,  'Payload bay / cabinet wall height'],
    # --- hub-motor wheel placement (the wheels are the separate hub_motor component) ---
    'wheelbase':            [620.0,  'Front-to-rear hub-motor spacing (X)'],
    'trackWidth':           [520.0,  'Left-to-right hub-motor spacing (Y)'],
    # --- rear electronics cabinet (same width/height & construction as the payload bay) ---
    'cabDepth':             [200.0,  'Cabinet depth - how far it projects behind the rover (X)'],
    'doorThk':              [6.0,    'Access door panel thickness'],
    'batteryLength':        [130.0,  'Battery pack depth (X)'],
    'batteryWidth':         [220.0,  'Battery pack width (Y)'],
    'batteryHeight':        [95.0,   'Battery pack height (Z)'],
    # --- control PCB + 4 motor drivers, mounted vertically facing the door ---
    'pcbWidth':             [240.0,  'Control PCB width (Y)'],
    'pcbHeight':            [140.0,  'Control PCB height (Z)'],
    'pcbThickness':         [1.6,    'PCB thickness (X)'],
    'standoffDiameter':     [7.0,    'PCB standoff diameter'],
    'standoffHeight':       [14.0,   'PCB standoff height off the back wall'],
    'esp32Width':           [28.0,   'ESP32 module width (Y)'],
    'esp32Height':          [55.0,   'ESP32 module height (Z)'],
    'esp32Depth':           [15.0,   'ESP32 module depth off the PCB (X)'],
    'driverWidth':          [46.0,   'Motor driver width (Y)'],
    'driverHeight':         [46.0,   'Motor driver height (Z)'],
    'driverDepth':          [22.0,   'Motor driver depth off the PCB (X)'],
    # --- connections ---
    'glandDia':             [14.0,   'Cable gland diameter (motor cables)'],
    'glandLen':             [22.0,   'Cable gland length'],
    'switchSize':           [18.0,   'Power switch size'],
    # --- emergency stop (red mushroom, latching, cuts motor power via a contactor) ---
    'estopBaseDia':         [30.0,   'E-stop base/collar diameter'],
    'estopBaseH':           [22.0,   'E-stop base height'],
    'estopHeadDia':         [42.0,   'E-stop mushroom head diameter'],
    'estopHeadH':           [18.0,   'E-stop mushroom head height'],
}

# --- rocker suspension (optional add-on; built as its own deletable 'Rocker_Suspension'
#     component). A passive walking-beam rocker per side links that side's front & rear wheel
#     and pivots on a pin bracketed to the chassis base plate; a central differential cross-bar
#     couples the two rockers so the body holds the mean pitch. Everything sits in the gap
#     between the wheel axles (z=axleHeight) and the base plate (z=groundClearance). ---
WITH_ROCKER = True
ROCKER_PARAMS = {
    # arm: MS rectangular hollow section, runs fore-aft inboard of the tyres; its outer face
    # CONTACTS the hub-motor dropout inner face (y=197.5) - the M10s clamp the two flush so
    # friction carries shear. 172.5 = 197.5 (dropout inner face) - 50/2 (half arm), no gap.
    # Heights leave swing room: at +-8.5 deg the arm tip rises ~52 mm, under the plate (320).
    'rockerArmY':     [172.5, 'Rocker arm centreline (Y) - outer face flush on the dropout inner face'],
    'rockerArmW':     [50.0,  'Rocker arm MS RHS width (Y)'],
    'rockerArmH':     [50.0,  'Rocker arm MS RHS height (Z)'],
    'rockerArmTopZ':  [265.0, 'Rocker arm top (Z) - leaves swing clearance under the base plate'],
    'rockerArmLen':   [700.0, 'Rocker arm length (X) - spans the front & rear dropout plates'],
    'dropoutThk':     [8.0,   'Hub-motor dropout plate thickness (= wheel_mount plateThk)'],
    'mountBoltDia':   [10.0,  'M10 through-bolt, rocker arm to hub-motor dropout plate'],
    'm10HeadDia':     [16.0,  'M10 socket-head cap-screw head diameter'],
    'm10HeadH':       [7.0,   'M10 cap-screw head height'],
    'm10WasherOD':    [21.0,  'M10 washer outer diameter'],
    'm10WasherThk':   [2.0,   'M10 washer thickness'],
    'm10NutAF':       [17.0,  'M10 nyloc nut across-flats (modelled round)'],
    'm10NutH':        [8.0,   'M10 nyloc nut height'],
    # central pivot: ground steel shaft in two bearings, carried by cheeks off the base plate
    'pivotZ':         [240.0, 'Central pivot axis height (Z) = rocker arm centroid'],
    'pivotShaftDia':  [20.0,  'Central pivot shaft diameter (EN8 ground shaft)'],
    'pivotBracketT':  [12.0,  'Pivot bracket cheek thickness (Y)'],
    'bearingDia':     [34.0,  'Pivot bearing / boss outer diameter'],
    'bearingW':       [12.0,  'Pivot bearing width (Y)'],
    # differential: cross-bar on a centre pivot post, drop-linked (rod ends) to each rocker arm
    'diffOffsetX':    [130.0, 'Differential cross-bar offset forward of the pivot (X)'],
    'diffBarZ':       [185.0, 'Differential cross-bar height (Z), below the arms'],
    'diffBarDia':     [20.0,  'Differential cross-bar diameter'],
    'diffTrunnionDia': [24.0, 'Differential pivot pin diameter (EN8, clamped in the clevis cheeks)'],
    'diffHubOD':      [48.0,  'Differential hub outer diameter (bar welds into it; rotates on the pin)'],
    'diffHubLen':     [50.0,  'Differential hub length (X)'],
    'diffBushFlangeOD': [38.0, 'Bronze bushing flange OD (thrust face between hub and cheek)'],
    'diffBushFlangeThk': [3.0, 'Bronze bushing flange thickness'],
    'clevisThk':      [10.0,  'Clevis cheek plate thickness (welded under the base plate)'],
    'diffPinHeadDia': [36.0,  'Differential pin head diameter'],
    'diffPinHeadH':   [10.0,  'Differential pin head height'],
    'diffLinkDia':    [12.0,  'Differential link-rod diameter'],
    'rodEndDia':      [18.0,  'Spherical rod-end (heim joint) body diameter'],
}

P = {k: v[0] for k, v in PARAMS.items()}  # mm lookup for stable initial sketch geometry
P.update({k: v[0] for k, v in ROCKER_PARAMS.items()})

# appearance names (validated against the Fusion Appearance Library)
APP_STEEL = 'Paint - Enamel Glossy (Dark Grey)'   # dark frame: tube columns, base plate, rails
APP_DECK = 'Paint - Enamel Glossy (Yellow)'       # yellow body panels: bay + cabinet sheets
APP_FLOOR = 'Steel - Galvanized (Small)'          # galvanised cargo deck floor
APP_ACCENT = 'Paint - Enamel Glossy (Red)'
APP_TYRE = 'Rubber - Hard'
APP_RIM = 'Aluminum - Anodized Glossy (Grey)'
APP_BLACK = 'Plastic - Glossy (Black)'
APP_WHITE = 'Plastic - Glossy (White)'
APP_CHROME = 'Stainless Steel - Polished'
APP_PCB = 'Plastic - Matte (Green)'
APP_BLUE = 'Plastic - Glossy (Blue)'
APP_RED = 'Plastic - Glossy (Red)'
APP_BRASS = 'Brass - Matte'
APP_BATT = 'Paint - Enamel Glossy (Black)'

# handles populated in build()
_app = _design = _root = None
P3 = vS = None
H = V = NEWBODY = CUTOP = None
_APP_CACHE = {}


def _v(expr):
    return float(eval(expr, {'__builtins__': {}}, P))


def _pos(expr):
    """Distance dimensions are unsigned; return a positive-valued expression matching |expr|."""
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


def _subcomp(parent, name):
    """Child component (identity transform) - used for parts that move in joints/motion."""
    occ = parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())
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


def box(comp, name, cx, cy, zbase, l, w, h, appr=None):
    """Box centred at (cx, cy) in XY, sitting on plane z=zbase, extruded up by h."""
    sk = comp.sketches.add(_plane(comp, comp.xYConstructionPlane, zbase))
    cxx, cyy, ll, ww = _v(cx), _v(cy), _v(l), _v(w)
    x0, y0 = (cxx - ll / 2) / 10.0, (cyy - ww / 2) / 10.0
    x1, y1 = (cxx + ll / 2) / 10.0, (cyy + ww / 2) / 10.0
    lines = sk.sketchCurves.sketchLines
    a = lines.addByTwoPoints(P3(x0, y0, 0), P3(x1, y0, 0))
    b = lines.addByTwoPoints(a.endSketchPoint, P3(x1, y1, 0))
    c = lines.addByTwoPoints(b.endSketchPoint, P3(x0, y1, 0))
    lines.addByTwoPoints(c.endSketchPoint, a.startSketchPoint)
    gc = sk.geometricConstraints
    gc.addHorizontal(a)
    gc.addHorizontal(c)
    gc.addVertical(b)
    sk.sketchDimensions.addDistanceDimension(
        a.startSketchPoint, a.endSketchPoint, H, P3((x0 + x1) / 2, y0 - 0.5, 0)).parameter.expression = l
    sk.sketchDimensions.addDistanceDimension(
        b.startSketchPoint, b.endSketchPoint, V, P3(x1 + 0.5, (y0 + y1) / 2, 0)).parameter.expression = w
    _locate(sk, a.startSketchPoint, '(' + cx + ') - (' + l + ')/2', '(' + cy + ') - (' + w + ')/2')
    ext = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), vS(h), NEWBODY)
    body = ext.bodies.item(0)
    body.name = name
    sk.name = name + '_sk'
    _paint(body, appr)
    return body


def cyl_z(comp, name, cx, cy, zbase, dia, h, appr=None):
    sk = comp.sketches.add(_plane(comp, comp.xYConstructionPlane, zbase))
    prof = _circle(sk, _v(cx), _v(cy), _v(dia), cx, cy, dia)
    ext = comp.features.extrudeFeatures.addSimple(prof, vS(h), NEWBODY)
    ext.bodies.item(0).name = name
    sk.name = name + '_sk'
    _paint(ext.bodies.item(0), appr)
    return ext.bodies.item(0)


def cyl_y(comp, name, cx, cz, yc, dia, length, appr=None):
    # XZ-plane sketch: local +V maps to global -Z, so the cz expression is negated.
    yb = '(' + yc + ') - (' + length + ')/2'
    sk = comp.sketches.add(_plane(comp, comp.xZConstructionPlane, yb))
    prof = _circle(sk, _v(cx), -_v(cz), _v(dia), cx, '-(' + cz + ')', dia)
    ext = comp.features.extrudeFeatures.addSimple(prof, vS(length), NEWBODY)
    ext.bodies.item(0).name = name
    sk.name = name + '_sk'
    _paint(ext.bodies.item(0), appr)
    return ext.bodies.item(0)


def cyl_x(comp, name, cy, cz, xc, dia, length, appr=None):
    # YZ-plane sketch: local u -> global Z, local v -> global Y. So u = -cz (gives global Z=cz),
    # v = cy (gives global Y=cy).
    xb = '(' + xc + ') - (' + length + ')/2'
    sk = comp.sketches.add(_plane(comp, comp.yZConstructionPlane, xb))
    prof = _circle(sk, -_v(cz), _v(cy), _v(dia), '-(' + cz + ')', cy, dia)
    ext = comp.features.extrudeFeatures.addSimple(prof, vS(length), NEWBODY)
    ext.bodies.item(0).name = name
    sk.name = name + '_sk'
    _paint(ext.bodies.item(0), appr)
    return ext.bodies.item(0)


def sphere(comp, name, cx, cz, yc, dia, appr=None):
    """Sphere centred at (cx, yc, cz): semicircle on an XZ plane at y=yc, revolved 360 deg."""
    sk = comp.sketches.add(_plane(comp, comp.xZConstructionPlane, yc))
    u, v, r = _v(cx) / 10.0, -_v(cz) / 10.0, _v(dia) / 20.0
    axis = sk.sketchCurves.sketchLines.addByTwoPoints(P3(u, v - r, 0), P3(u, v + r, 0))
    arc = sk.sketchCurves.sketchArcs.addByThreePoints(
        axis.startSketchPoint, P3(u + r, v, 0), axis.endSketchPoint)
    sk.sketchDimensions.addRadialDimension(
        arc, P3(u + r / 2, v, 0)).parameter.expression = '(' + dia + ')/2'
    _locate(sk, axis.startSketchPoint, cx, '-(' + cz + ') - (' + dia + ')/2')
    rin = comp.features.revolveFeatures.createInput(sk.profiles.item(0), axis, NEWBODY)
    rin.setAngleExtent(False, vS('360 deg'))
    body = comp.features.revolveFeatures.add(rin).bodies.item(0)
    body.name = name
    sk.name = name + '_sk'
    _paint(body, appr)
    return body


def _bore_y(comp, body, cx, cz, yc, dia, length):
    """Cut a coaxial through-hole (axis Y) in an existing body."""
    yb = '(' + yc + ') - (' + length + ')/2'
    sk = comp.sketches.add(_plane(comp, comp.xZConstructionPlane, yb))
    prof = _circle(sk, _v(cx), -_v(cz), _v(dia), cx, '-(' + cz + ')', dia)
    ef = comp.features.extrudeFeatures
    inp = ef.createInput(prof, CUTOP)
    inp.setDistanceExtent(False, vS(length))
    inp.participantBodies = [body]
    ef.add(inp)
    sk.name = 'bore_sk'


def hole(comp, body, cx, cy, zbase, dia, depth):
    sk = comp.sketches.add(_plane(comp, comp.xYConstructionPlane, zbase))
    prof = _circle(sk, _v(cx), _v(cy), _v(dia), cx, cy, dia)
    ef = comp.features.extrudeFeatures
    inp = ef.createInput(prof, CUTOP)
    inp.setDistanceExtent(False, vS(depth))
    inp.participantBodies = [body]
    ef.add(inp)
    sk.name = 'PCB_hole_sk'


def _fillet_radius(body, radius_mm, radius_expr):
    """Round all circular edges whose radius matches radius_mm (cosmetic; failures ignored)."""
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
    except Exception:
        pass


def wheel(comp, name, cx, cz, yc):
    """Tyre (rubber ring) + rim + protruding hub cap, with a rounded tread shoulder."""
    tyre = cyl_y(comp, name + '_Tyre', cx, cz, yc, 'wheelDiameter', 'wheelWidth', APP_TYRE)
    _bore_y(comp, tyre, cx, cz, yc, 'rimDiameter', 'wheelWidth')
    cyl_y(comp, name + '_Rim', cx, cz, yc, 'rimDiameter', 'wheelWidth', APP_RIM)
    cyl_y(comp, name + '_Hub', cx, cz, yc, 'hubDiameter', 'wheelWidth + hubProtrude', APP_BLACK)
    _fillet_radius(tyre, _v('wheelDiameter') / 2.0, 'tyreFillet')
    return tyre


def framed_box(comp, pre, cx, cy, zbase, length, width, height, skip=None):
    """A welded box: 4 MS square-tube corner columns + steel-sheet side panels between them.
    `skip` ('front'|'rear'|'left'|'right') omits one panel (e.g. for a door opening)."""
    X, Y = '(' + cx + ')', '(' + cy + ')'
    for sx, fx in (('F', '+'), ('R', '-')):
        for sy, fy in (('L', '+'), ('R', '-')):
            box(comp, '%s_Col_%s%s' % (pre, sx, sy),
                X + ' ' + fx + ' (' + length + '/2 - tubeSize/2)',
                Y + ' ' + fy + ' (' + width + '/2 - tubeSize/2)',
                zbase, 'tubeSize', 'tubeSize', height, APP_STEEL)
    panels = {
        'front': (X + ' + (' + length + '/2 - sheetThk/2)', cy, 'sheetThk', '(' + width + ') - tubeSize*2'),
        'rear':  (X + ' - (' + length + '/2 - sheetThk/2)', cy, 'sheetThk', '(' + width + ') - tubeSize*2'),
        'left':  (cx, Y + ' + (' + width + '/2 - sheetThk/2)', '(' + length + ') - tubeSize*2', 'sheetThk'),
        'right': (cx, Y + ' - (' + width + '/2 - sheetThk/2)', '(' + length + ') - tubeSize*2', 'sheetThk'),
    }
    for side, (px, py, pl, pw) in panels.items():
        if side != skip:
            box(comp, '%s_Sheet_%s' % (pre, side.capitalize()), px, py, zbase, pl, pw, height, APP_DECK)


def build_rocker(design=None):
    """Build the deletable 'Rocker_Suspension' component: a passive walking-beam rocker per side
    plus a central differential. Assumes build() already ran (globals + car params set).

    Integration with the hub_motor module: each wheel's own dropout plate + torque arm + M12 axle
    nut already fully carry its axle; the dropout plate is the module's mounting face (it has no
    chassis tie of its own). The rocker arm runs fore-aft inboard of the tyres, OVERLAPPING both
    dropout plates (y~201) and bolts to them with M10s, so each side's two wheels hang off one arm
    that see-saws about a central Ø20 shaft in bearings on cheeks off the base plate. A
    differential cross-bar (rod-end drop-links) ties the two arms so the body holds the mean
    pitch. Heights are set so tyres, plates and arm tips all clear the base plate at full travel.
    Delete the component to remove it all."""
    d = design or _design
    for name, (val, comment) in ROCKER_PARAMS.items():
        if d.userParameters.itemByName(name) is None:
            d.userParameters.add(name, vS('{} mm'.format(val)), 'mm', comment)
        P[name] = val
    rk = _comp('Rocker_Suspension')

    # Moving parts are CHILD COMPONENTS so assemble.py can put real joints on them
    # (revolute at each pivot, rigid wheel-to-arm) and motion.py can animate them.
    # Fixed parts (shafts, bearings, bracket cheeks, diff post) stay as parent bodies.
    arm_comps = {'L': _subcomp(rk, 'Rocker_Arm_L'), 'R': _subcomp(rk, 'Rocker_Arm_R')}
    diff = _subcomp(rk, 'Differential')

    ARM_ZB = 'rockerArmTopZ - rockerArmH'        # arm underside
    ARM_BOT = ARM_ZB
    for s, Y, OUT in (('L', 'rockerArmY', '+'), ('R', '-rockerArmY', '-')):
        arm = arm_comps[s]
        # MS RHS walking-beam arm: spans both dropout plates on this side, just under the base
        # plate; the outer face lands flush on each dropout's inner face for the bolted joint
        box(arm, 'Rocker_Arm_' + s, '0', Y, ARM_ZB,
            'rockerArmLen', 'rockerArmW', 'rockerArmH', APP_BLUE)
        # welded pivot boss: its cylindrical face IS the revolute-joint axis for the arm
        cyl_y(arm, 'Pivot_Boss_' + s, '0', 'pivotZ', Y, 'bearingDia', 'rockerArmW', APP_BLUE)
        # 2x M10 through-bolts at each end fasten the arm to that wheel's dropout plate:
        # head on the arm inner face, shaft through arm + plate, washer + nyloc on the plate
        # outer face (nut tip clears the tyre inner face at 217.5).
        IN = '-' if OUT == '+' else '+'
        for fx, sx in (('wheelbase/2', 'F'), ('-wheelbase/2', 'R')):
            for dz, lvl in (('+ 14', 'U'), ('- 14', 'L')):
                nm = sx + s + lvl
                zc = 'pivotZ ' + dz
                cyl_y(arm, 'Mount_Bolt_' + nm, fx, zc,
                      '(' + Y + ') ' + OUT + ' (dropoutThk/2 + 2)',
                      'mountBoltDia', 'rockerArmW + dropoutThk + 14', APP_BLACK)
                cyl_y(arm, 'Mount_Bolt_Head_' + nm, fx, zc,
                      '(' + Y + ') ' + IN + ' (rockerArmW/2 + m10HeadH/2)',
                      'm10HeadDia', 'm10HeadH', APP_BLACK)
                cyl_y(arm, 'Mount_Washer_' + nm, fx, zc,
                      '(' + Y + ') ' + OUT + ' (rockerArmW/2 + dropoutThk + m10WasherThk/2)',
                      'm10WasherOD', 'm10WasherThk', APP_CHROME)
                cyl_y(arm, 'Mount_Nut_' + nm, fx, zc,
                      '(' + Y + ') ' + OUT + ' (rockerArmW/2 + dropoutThk + m10WasherThk + m10NutH/2)',
                      'm10NutAF', 'm10NutH', APP_CHROME)
        # fixed side: ground shaft through the boss, two bearings, two bracket cheeks off the plate
        cyl_y(rk, 'Pivot_Shaft_' + s, '0', 'pivotZ', Y,
              'pivotShaftDia', 'rockerArmW + 2*bearingW + 2*pivotBracketT + 16', APP_CHROME)
        for sgn, tag in (('+', 'out'), ('-', 'in')):
            cyl_y(rk, 'Bearing_%s_%s' % (s, tag), '0', 'pivotZ',
                  '(' + Y + ') ' + sgn + ' (rockerArmW/2 + bearingW/2)',
                  'bearingDia', 'bearingW', APP_CHROME)
            box(rk, 'Pivot_Bracket_%s_%s' % (s, tag), '0',
                '(' + Y + ') ' + sgn + ' (rockerArmW/2 + bearingW + pivotBracketT/2)',
                'pivotZ - bearingDia/2 - 6', 'bearingDia + 24', 'pivotBracketT',
                'groundClearance - (pivotZ - bearingDia/2 - 6)', APP_BLACK)

    # central differential, built as a real clevis joint so the pivot is fabricable/readable:
    #   FIXED side: two clevis cheek plates welded under the base plate; the Ø24 PIN is clamped
    #   across them (head one side, washer + nyloc the other) - the pin does NOT rotate.
    #   ROTATING side: the cross-bar welds into a HUB; bronze flanged bushings press into the
    #   hub bore and ride on the pin (the actual bearing surface), flanges acting as thrust
    #   washers against the cheeks. Rod-end drop-links go out to the two arms.
    cyl_y(diff, 'Diff_Bar', 'diffOffsetX', 'diffBarZ', '0', 'diffBarDia', '2*rockerArmY + 60', APP_CHROME)
    cyl_x(diff, 'Diff_Hub', '0', 'diffBarZ', 'diffOffsetX',
          'diffHubOD', 'diffHubLen', APP_BLUE)                # its cyl face = diff revolute axis
    cyl_x(diff, 'Diff_Bushing_Sleeve', '0', 'diffBarZ', 'diffOffsetX',
          'diffTrunnionDia + 8', 'diffHubLen', APP_BRASS)     # bronze sleeve inside the hub bore
    for sgn, tag in (('+', 'F'), ('-', 'R')):
        cyl_x(diff, 'Diff_Bushing_Flange_' + tag, '0', 'diffBarZ',
              '(diffOffsetX) ' + sgn + ' (diffHubLen/2 + diffBushFlangeThk/2)',
              'diffBushFlangeOD', 'diffBushFlangeThk', APP_BRASS)
    # drop-links: BALL JOINT at both ends (bolted, never welded - the link skews in two planes
    # as the mechanism articulates), with the steel ball explicitly modelled in each eye.
    #   top:    heim rod end - eye + ball on an M8 pin through a lug welded under the arm
    #   bottom: GE-style spherical plain bearing - the link eye's ball rides directly on the
    #           cross-bar (the bar is its pin), retained axially by two welded collars
    for s, Y, OUT, IN in (('L', 'rockerArmY', '+', '-'), ('R', '-rockerArmY', '-', '+')):
        arm = arm_comps[s]
        TOPZ = '(' + ARM_BOT + ') - 12'
        box(arm, 'Diff_Lug_' + s, 'diffOffsetX', '(' + Y + ') ' + OUT + ' 8',
            '(' + ARM_BOT + ') - 20', '24', '8', '21', APP_BLUE)        # welded tab under the arm
        cyl_y(arm, 'Diff_Lug_Pin_' + s, 'diffOffsetX', TOPZ, Y,
              '8', '26', APP_CHROME)                                    # M8 bolt through lug + eye
        cyl_z(diff, 'Diff_Link_' + s, 'diffOffsetX', Y, 'diffBarZ',
              'diffLinkDia', '(' + TOPZ + ') - diffBarZ', APP_BLUE)
        # top heim joint: ball sits proud of the eye faces so the sphere reads in the model
        sphere(diff, 'Diff_Ball_Top_' + s, 'diffOffsetX', TOPZ,
               '(' + Y + ') ' + IN + ' 3', 'rodEndDia*0.78', APP_CHROME)
        cyl_y(diff, 'Rod_End_Top_' + s, 'diffOffsetX', TOPZ,
              '(' + Y + ') ' + IN + ' 3', 'rodEndDia', 'rodEndDia*0.45', APP_BLACK)
        # bottom spherical bearing: ball around the bar, housed in the link's eye ring
        sphere(diff, 'Diff_Ball_Bot_' + s, 'diffOffsetX', 'diffBarZ', Y,
               'diffBarDia + 6', APP_CHROME)
        cyl_y(diff, 'Diff_Eye_Bot_' + s, 'diffOffsetX', 'diffBarZ', Y,
              'diffBarDia + 12', 'diffBarDia*0.7', APP_BLACK)
        for sgn2, tag2 in (('+', 'out'), ('-', 'in')):
            cyl_y(diff, 'Diff_Collar_%s_%s' % (s, tag2), 'diffOffsetX', 'diffBarZ',
                  '(' + Y + ') ' + sgn2 + ' 10', 'diffBarDia + 6', '6', APP_CHROME)
    # fixed side of the clevis (parent bodies; the chassis carries these)
    CHEEK_OFF = 'diffHubLen/2 + diffBushFlangeThk + 1 + clevisThk/2'
    for sgn, tag in (('+', 'F'), ('-', 'R')):
        box(rk, 'Clevis_Cheek_' + tag, '(diffOffsetX) ' + sgn + ' (' + CHEEK_OFF + ')', '0',
            'diffBarZ - 28', 'clevisThk', 'diffHubOD + 16',
            'groundClearance - (diffBarZ - 28)', APP_STEEL)
    cyl_x(rk, 'Diff_Pin', '0', 'diffBarZ', 'diffOffsetX', 'diffTrunnionDia',
          'diffHubLen + 2*diffBushFlangeThk + 2 + 2*clevisThk', APP_CHROME)
    cyl_x(rk, 'Diff_Pin_Head', '0', 'diffBarZ',
          '(diffOffsetX) - (diffHubLen/2 + diffBushFlangeThk + 1 + clevisThk + diffPinHeadH/2)',
          'diffPinHeadDia', 'diffPinHeadH', APP_BLACK)
    cyl_x(rk, 'Diff_Pin_Washer', '0', 'diffBarZ',
          '(diffOffsetX) + (diffHubLen/2 + diffBushFlangeThk + 1 + clevisThk + 1.5)',
          'diffPinHeadDia + 4', '3', APP_CHROME)
    cyl_x(rk, 'Diff_Pin_Nut', '0', 'diffBarZ',
          '(diffOffsetX) + (diffHubLen/2 + diffBushFlangeThk + 1 + clevisThk + 3 + 6)',
          'diffPinHeadDia', '12', APP_CHROME)
    return rk


def build(app):
    """Create a new parametric RC-payload-car design in the given Application. Returns the Design."""
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

    PLATE_TOP = 'groundClearance + basePlateThickness'
    RAIL_TOP = 'groundClearance + basePlateThickness + frameRailHeight'
    BED_TOP = 'groundClearance + basePlateThickness + frameRailHeight + bedThickness'
    BUMPER_CZ = 'groundClearance + frameRailHeight/2'

    core = _comp('Core_Chassis')
    bay = _comp('Payload_Bay')
    cab = _comp('Rear_Cabinet')

    # ---- core chassis: load-bearing frame (base plate, side rails, cross members, bumpers) ----
    box(core, 'Base_Plate', '0', '0', 'groundClearance',
        'chassisLength', 'chassisWidth', 'basePlateThickness', APP_STEEL)
    box(core, 'Rail_Left', '0', 'chassisWidth/2 - frameRailThickness/2', PLATE_TOP,
        'chassisLength', 'frameRailThickness', 'frameRailHeight', APP_STEEL)
    box(core, 'Rail_Right', '0', '-chassisWidth/2 + frameRailThickness/2', PLATE_TOP,
        'chassisLength', 'frameRailThickness', 'frameRailHeight', APP_STEEL)
    box(core, 'Cross_Front', 'chassisLength/2 - crossMemberWidth/2', '0', PLATE_TOP,
        'crossMemberWidth', 'chassisWidth - frameRailThickness*2', 'frameRailHeight', APP_STEEL)
    box(core, 'Cross_Rear', '-chassisLength/2 + crossMemberWidth/2', '0', PLATE_TOP,
        'crossMemberWidth', 'chassisWidth - frameRailThickness*2', 'frameRailHeight', APP_STEEL)
    cyl_y(core, 'Bumper_Front', 'chassisLength/2', BUMPER_CZ, '0',
          'bumperDiameter', 'chassisWidth*0.8', APP_CHROME)
    cyl_y(core, 'Bumper_Rear', '-chassisLength/2', BUMPER_CZ, '0',
          'bumperDiameter', 'chassisWidth*0.8', APP_CHROME)

    # ---- payload bay: deck floor + welded cargo box (4 MS square-tube columns + steel sheets) ----
    box(bay, 'Payload_Bed', '0', '0', RAIL_TOP,
        'chassisLength', 'chassisWidth', 'bedThickness', APP_FLOOR)
    framed_box(bay, 'Bay', '0', '0', BED_TOP, 'chassisLength', 'chassisWidth', 'bayWallHeight')
    # ---- drivetrain & steering: none in the chassis. The rover runs on four hub_motor wheel
    #      modules (separate component) and is skid-steered by driving them independently. ----

    # ---- rear electronics cabinet: same welded construction and width/height as the payload
    #      bay, mounted on the back; the rear face is a pair of hinged access doors ----
    CX = '-chassisLength/2 - cabDepth/2'              # cabinet centre (behind the chassis)
    REAR = '-chassisLength/2 - cabDepth'              # rear (door) face
    BOT = BED_TOP                                     # floor = bay floor, so its top is flush with the bay top
    framed_box(cab, 'Cab', CX, '0', BOT, 'cabDepth', 'chassisWidth', 'bayWallHeight', skip='rear')
    box(cab, 'Cab_Bottom', CX, '0', BOT, 'cabDepth', 'chassisWidth', 'sheetThk', APP_DECK)
    box(cab, 'Cab_Top', CX, '0', '(' + BOT + ') + bayWallHeight - sheetThk', 'cabDepth', 'chassisWidth', 'sheetThk', APP_DECK)
    # twin rear access doors (each half the opening), hinged on the outer columns, shown open 90 deg
    half = '(chassisWidth - tubeSize*2)/2'
    for side, sgn in (('Left', '+'), ('Right', '-')):
        box(cab, 'Door_' + side, '(' + REAR + ') - (' + half + ')/2',
            sgn + '(chassisWidth/2 - tubeSize/2)', BOT, half, 'doorThk', 'bayWallHeight', APP_ACCENT)
        cyl_z(cab, 'Hinge_L_' + side, REAR, sgn + '(chassisWidth/2 - tubeSize)', '(' + BOT + ') + tubeSize',
              'doorThk*2', 'bayWallHeight/6', APP_CHROME)
        cyl_z(cab, 'Hinge_U_' + side, REAR, sgn + '(chassisWidth/2 - tubeSize)',
              '(' + BOT + ') + bayWallHeight - tubeSize - bayWallHeight/6', 'doorThk*2', 'bayWallHeight/6', APP_CHROME)
    # battery low on the floor (left side)
    box(cab, 'Battery_Pack', CX, 'chassisWidth/4', '(' + BOT + ') + sheetThk',
        'batteryLength', 'batteryWidth', 'batteryHeight', APP_BATT)
    # vertical control PCB on standoffs off the front wall (right side), facing the doors
    pcby = '-chassisWidth/5'
    pcbx = '-chassisLength/2 - sheetThk - standoffHeight - pcbThickness/2'
    pcbz = '(' + BOT + ') + sheetThk + 12'
    si = 0
    for cyo in ['pcbWidth/2 - standoffDiameter', '-(pcbWidth/2 - standoffDiameter)']:
        for czo in ['(' + pcbz + ') + standoffDiameter', '(' + pcbz + ') + pcbHeight - standoffDiameter']:
            si += 1
            cyl_x(cab, 'PCB_Standoff_%d' % si, '(' + pcby + ') + (' + cyo + ')', czo,
                  '-chassisLength/2 - sheetThk - standoffHeight/2', 'standoffDiameter', 'standoffHeight', APP_BRASS)
    box(cab, 'PCB', pcbx, pcby, pcbz, 'pcbThickness', 'pcbWidth', 'pcbHeight', APP_PCB)
    compx = '(' + pcbx + ') - pcbThickness/2'
    box(cab, 'ESP32', '(' + compx + ') - esp32Depth/2', pcby,
        '(' + pcbz + ') + pcbHeight - esp32Height - 12', 'esp32Depth', 'esp32Width', 'esp32Height', APP_BLACK)
    for nm, dyo in zip(['Driver_FL', 'Driver_FR', 'Driver_RL', 'Driver_RR'],
                       ['pcbWidth*0.36', 'pcbWidth*0.12', '-pcbWidth*0.12', '-pcbWidth*0.36']):
        box(cab, nm, '(' + compx + ') - driverDepth/2', '(' + pcby + ') + (' + dyo + ')',
            '(' + pcbz + ') + 14', 'driverDepth', 'driverWidth', 'driverHeight', APP_BLUE)
    # connections: four motor-cable glands through the front wall + a power switch on the side
    for i, gyo in enumerate(['chassisWidth*0.30', 'chassisWidth*0.10', '-chassisWidth*0.10', '-chassisWidth*0.30']):
        cyl_x(cab, 'Gland_%d' % (i + 1), gyo, '(' + BOT + ') + 40', '-chassisLength/2 - sheetThk',
              'glandDia', 'glandLen', APP_BLACK)
    box(cab, 'Power_Switch', CX, '-chassisWidth/2 - switchSize/2', '(' + BOT + ') + bayWallHeight*0.5',
        'switchSize', 'switchSize', 'switchSize', APP_BLACK)
    # emergency stop: red mushroom latching button on top of the cabinet, slap-accessible from the rear
    estop_x = '(' + REAR + ') + 60'
    estop_z = '(' + BOT + ') + bayWallHeight'
    cyl_z(cab, 'Estop_Base', estop_x, '0', estop_z, 'estopBaseDia', 'estopBaseH', APP_BLACK)
    cyl_z(cab, 'Estop_Button', estop_x, '0', '(' + estop_z + ') + estopBaseH',
          'estopHeadDia', 'estopHeadH', APP_RED)

    # ---- optional rocker suspension (its own deletable component) ----
    if WITH_ROCKER:
        build_rocker(_design)

    app.activeViewport.fit()
    return _design


def export_f3d(design, path):
    """Export the design to a local .f3d archive."""
    em = design.exportManager
    em.execute(em.createFusionArchiveExportOptions(path))
    return path
