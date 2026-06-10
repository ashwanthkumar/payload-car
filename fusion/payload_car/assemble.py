# -*- coding: utf-8 -*-
"""Assemble the complete rover (Fusion API).

Builds the car from ``builder.py``, cloud-saves it as ``payload_car_4wd``, inserts the four
referenced ``hub_motor`` wheel modules at the corners (grouped under a 'Motors' component),
adds the 173 cm scale human, then exports ``components/payload_car_4wd.f3d``.

This is the single source of truth for the assembly step (wheel transforms, grouping, save
and export targets); run it via the Fusion MCP / a Fusion script (note ``__file__`` must be
seeded, since plain ``exec`` does not define it):

    ns = {'__file__': path}
    with open(path) as f:
        exec(compile(f.read(), path, 'exec'), ns)
    ns['assemble'](adsk.core.Application.get())

Wheel placement: corners at (+-wheelbase/2, +-trackWidth/2); axle height = hub wheelOD/2 read
from ``wheel_mount.PARAMS`` (no magic numbers). Left-side wheels are rotated 180 deg about Z so
their single-sided drive shafts face outboard on both sides.
"""

import math
import os

try:  # adsk only exists inside Fusion's interpreter
    import adsk.core
    import adsk.fusion
except ImportError:  # pragma: no cover - offline (uv) path
    adsk = None

HERE = os.path.dirname(os.path.abspath(__file__))
BUILDER_PY = os.path.join(HERE, 'builder.py')
WHEEL_PY = os.path.join(HERE, 'wheel_mount.py')
EXPORT_F3D = os.path.normpath(os.path.join(HERE, '..', '..', 'components', 'payload_car_4wd.f3d'))
CLOUD_DOC = 'payload_car_4wd'
HUB_DOC = 'hub_motor'                      # cloud doc holding the wheel module (xref-inserted)
HUMAN_HEIGHT_NOTE = 'Scale_Human_173cm'


def _exec_module(path):
    """Exec a sibling module from disk (the canonical MCP pattern) and return its namespace."""
    ns = {}
    with open(path) as f:
        exec(compile(f.read(), path, 'exec'), ns)
    return ns


def _project_folder(app):
    data = app.data
    for i in range(data.dataProjects.count):
        if data.dataProjects.item(i).name == 'Default Project':
            return data.dataProjects.item(i).rootFolder
    return data.dataProjects.item(0).rootFolder


def add_human(app, design, root):
    """A blocky 173 cm reference man standing beside the rover (+Y side)."""
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = HUMAN_HEIGHT_NOTE
    comp = occ.component
    lib = app.materialLibraries.itemByName('Fusion Appearance Library')
    appr = (design.appearances.itemByName('Human Blue')
            or design.appearances.addByCopy(lib.appearances.itemByName('Plastic - Matte (Blue)'),
                                            'Human Blue'))
    P3 = adsk.core.Point3D.create
    rv = adsk.core.ValueInput.createByReal
    NB = adsk.fusion.FeatureOperations.NewBodyFeatureOperation

    def plane(z):
        pin = comp.constructionPlanes.createInput()
        pin.setByOffset(comp.xYConstructionPlane, rv(z / 10.0))
        return comp.constructionPlanes.add(pin)

    def box(n, cx, cy, zb, lx, ly, lz):
        sk = comp.sketches.add(plane(zb))
        sk.sketchCurves.sketchLines.addTwoPointRectangle(
            P3((cx - lx / 2) / 10.0, (cy - ly / 2) / 10.0, 0),
            P3((cx + lx / 2) / 10.0, (cy + ly / 2) / 10.0, 0))
        e = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), rv(lz / 10.0), NB)
        e.bodies.item(0).name = n
        e.bodies.item(0).appearance = appr

    def cyl(n, cx, cy, zb, dia, h):
        sk = comp.sketches.add(plane(zb))
        sk.sketchCurves.sketchCircles.addByCenterRadius(P3(cx / 10.0, cy / 10.0, 0), dia / 2 / 10.0)
        e = comp.features.extrudeFeatures.addSimple(sk.profiles.item(0), rv(h / 10.0), NB)
        e.bodies.item(0).name = n
        e.bodies.item(0).appearance = appr

    hx, hy = 200.0, 720.0
    box('Foot_L', hx - 95, hy + 70, 0, 120, 250, 50)
    box('Foot_R', hx + 95, hy + 70, 0, 120, 250, 50)
    cyl('Leg_L', hx - 95, hy, 50, 130, 820)
    cyl('Leg_R', hx + 95, hy, 50, 130, 820)
    box('Hips', hx, hy, 820, 330, 200, 130)
    box('Torso', hx, hy, 950, 360, 210, 540)
    cyl('Arm_L', hx - 215, hy, 790, 95, 700)
    cyl('Arm_R', hx + 215, hy, 790, 95, 700)
    cyl('Neck', hx, hy, 1490, 85, 80)
    cyl('Head', hx, hy, 1550, 185, 180)


def _cyl_face(occ, body_name):
    """First cylindrical face of a named body in an occurrence (joint axis geometry)."""
    cyl = adsk.core.Cylinder.classType()
    for b in occ.bRepBodies:
        if b.name == body_name:
            for f in b.faces:
                if f.geometry.objectType == cyl:
                    return f
    raise RuntimeError('no cylindrical face found on %r' % body_name)


def add_joints(design, root):
    """Make the rocker suspension a real mechanism:
    - ground Core_Chassis,
    - revolute joints: each rocker arm about its pivot boss, the differential about its trunnion,
    - rigid joints: each hub_motor wheel onto its side's arm (so wheels swing with the arms).
    After this the arms can be dragged in Fusion, and motion.py can drive them."""
    JG = adsk.fusion.JointGeometry
    KP = adsk.fusion.JointKeyPointTypes.MiddleKeyPoint
    ZDIR = adsk.fusion.JointDirections.ZAxisJointDirection

    occs = {o.component.name: o for o in root.occurrences}
    occs['Core_Chassis'].isGrounded = True
    arms, diff_occ = {}, None
    for ch in occs['Rocker_Suspension'].childOccurrences:
        if ch.component.name == 'Rocker_Arm_L':
            arms['L'] = ch
        elif ch.component.name == 'Rocker_Arm_R':
            arms['R'] = ch
        elif ch.component.name == 'Differential':
            diff_occ = ch

    abj = root.asBuiltJoints

    def revolute(name, occ, boss_body, limit_deg):
        inp = abj.createInput(occ, occs['Core_Chassis'],
                              JG.createByNonPlanarFace(_cyl_face(occ, boss_body), KP))
        inp.setAsRevoluteJointMotion(ZDIR)
        j = abj.add(inp)
        j.name = name
        lim = j.jointMotion.rotationLimits
        lim.isMinimumValueEnabled = True
        lim.minimumValue = -math.radians(limit_deg)
        lim.isMaximumValueEnabled = True
        lim.maximumValue = math.radians(limit_deg)
        return j

    # +-8.5 deg keeps tyre crowns, dropout plates and arm tips clear of the base plate (z=320)
    revolute('Rev_Arm_L', arms['L'], 'Pivot_Boss_L', 8.5)
    revolute('Rev_Arm_R', arms['R'], 'Pivot_Boss_R', 8.5)
    revolute('Rev_Diff', diff_occ, 'Diff_Hub', 7.0)

    # wheels follow their arm: rigid-group each side's arm with that side's two wheel modules,
    # includeChildren=True so EVERYTHING inside the xref (hub, dropout plate, torque arm,
    # fasteners) moves as one body. A plain rigid joint on the wheel's top occurrence is NOT
    # enough: the xref's children are independent free bodies in the solve and get left behind
    # (the hub moved while the dropout plate stayed - caught in motion review).
    wheels = list(occs['Motors'].childOccurrences)   # insertion order: FL, FR, RL, RR
    for side, idxs in (('L', (0, 2)), ('R', (1, 3))):
        coll = adsk.core.ObjectCollection.create()
        coll.add(arms[side])
        for i in idxs:
            coll.add(wheels[i])
        root.rigidGroups.add(coll, True).name = 'RG_Side_%s' % side


def assemble(app):
    """Build car -> cloud-save -> insert 4 wheels under 'Motors' -> human -> export. Returns Design."""
    while app.documents.count:                       # start from a clean slate
        app.documents.item(0).close(False)

    design = _exec_module(BUILDER_PY)['build'](app)
    root = design.rootComponent

    folder = _project_folder(app)
    hub_file = None
    for i in range(folder.dataFiles.count):
        df = folder.dataFiles.item(i)
        if df.name == HUB_DOC:
            hub_file = df
        if df.name == CLOUD_DOC:
            df.deleteMe()                            # replace the previous saved assembly
    if hub_file is None:
        raise RuntimeError('cloud doc %r not found - build/save wheel_mount.py first' % HUB_DOC)
    app.activeDocument.saveAs(CLOUD_DOC, folder, 'Skid-steer payload rover (assembled)', '')

    # wheel transforms: axle height comes from the hub module itself, corners from the car params
    wheel_od = _exec_module(WHEEL_PY)['PARAMS']['wheelOD'][0]      # mm
    axle_z = wheel_od / 2.0 / 10.0                                 # cm (Fusion internal units)
    up = design.userParameters
    wb, tw = up.itemByName('wheelbase').value, up.itemByName('trackWidth').value

    def xf(x, y, flip):
        m = adsk.core.Matrix3D.create()
        if flip:                                     # left side: drive shaft faces outboard
            m.setToRotation(math.pi, adsk.core.Vector3D.create(0, 0, 1),
                            adsk.core.Point3D.create(0, 0, 0))
        m.translation = adsk.core.Vector3D.create(x, y, axle_z)
        return m

    motors = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    motors.component.name = 'Motors'
    for x, y, flip in ((wb / 2, tw / 2, True), (wb / 2, -tw / 2, False),
                       (-wb / 2, tw / 2, True), (-wb / 2, -tw / 2, False)):
        motors.component.occurrences.addByInsert(hub_file, xf(x, y, flip), True)

    add_human(app, design, root)
    add_joints(design, root)

    em = design.exportManager
    em.execute(em.createFusionArchiveExportOptions(EXPORT_F3D))
    app.activeDocument.save('assembled via assemble.py')
    app.activeViewport.fit()
    print('assembled %s: %s' % (CLOUD_DOC, [o.component.name for o in root.occurrences]))
    print('exported %s' % EXPORT_F3D)
    return design
