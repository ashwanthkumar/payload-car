# -*- coding: utf-8 -*-
"""Rocker-suspension motion study: drive the joints through an articulation cycle.

The Fusion API does not expose Motion Studies, so this does it properly with the real
joints that ``assemble.py`` creates (``Rev_Arm_L`` / ``Rev_Arm_R`` / ``Rev_Diff`` plus the
wheel-to-arm rigid joints): it sweeps the arm joints in opposition - left front wheel up
while right front wheel down, the differential rocking between them - renders each pose,
and the frames become ``media/rocker_motion.gif``.

In Fusion (via the MCP):

    ns = {'__file__': path}
    with open(path) as f:
        exec(compile(f.read(), path, 'exec'), ns)
    ns['animate'](adsk.core.Application.get())

Offline (uv) to assemble the GIF from the captured frames:

    uv run python -c "from payload_car.motion import make_gif; make_gif()"
"""

import math
import os

try:  # adsk only exists inside Fusion's interpreter
    import adsk.core
    import adsk.fusion
except ImportError:  # pragma: no cover - offline (uv) path
    adsk = None

HERE = os.path.dirname(os.path.abspath(__file__))
MEDIA = os.path.normpath(os.path.join(HERE, '..', '..', 'media'))
FRAMES_DIR = os.path.join(MEDIA, 'frames')
GIF = os.path.join(MEDIA, 'rocker_motion.gif')
DIFF_FRAMES_DIR = os.path.join(MEDIA, 'frames_diff')
DIFF_GIF = os.path.join(MEDIA, 'diff_pivot_motion.gif')

ARM_SWING_DEG = 8.0          # arm articulation amplitude (joint limits are 9 deg)
FRAMES = 36
FRAME_W, FRAME_H = 1100, 650


def animate(app, frames=FRAMES, width=FRAME_W, height=FRAME_H, diff_sign=-1.0):
    # diff_sign=-1 verified in-model: with +1 the link tops moved away from the arms.
    """Sweep one full articulation cycle and save a PNG per pose into media/frames/."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    joints = {j.name: j for j in root.asBuiltJoints}
    rev_l, rev_r, rev_d = joints['Rev_Arm_L'], joints['Rev_Arm_R'], joints['Rev_Diff']

    # diff bar end must drop as far as the arm's link point: ratio = diffOffsetX / rockerArmY
    up = design.userParameters
    ratio = up.itemByName('diffOffsetX').value / up.itemByName('rockerArmY').value

    os.makedirs(FRAMES_DIR, exist_ok=True)
    root.isJointsFolderLightBulbOn = False    # keep joint glyphs out of the frames
    human = next((o for o in root.occurrences if o.component.name == 'Scale_Human_173cm'), None)
    if human:                                 # the rover is the star of this movie
        human.isLightBulbOn = False
    vp = app.activeViewport
    cam = vp.camera                       # fixed 3/4 view, low enough to read the see-saw
    cam.isSmoothTransition = False
    cam.eye = adsk.core.Point3D.create(150.0, -120.0, 70.0)
    cam.target = adsk.core.Point3D.create(0.0, 0.0, 25.0)
    cam.upVector = adsk.core.Vector3D.create(0.0, 0.0, 1.0)
    cam.viewExtents = 100.0               # whole rover with a little air
    vp.camera = cam
    vp.refresh()

    amp = math.radians(ARM_SWING_DEG)
    for i in range(frames):
        th = amp * math.sin(2.0 * math.pi * i / frames)
        rev_l.jointMotion.rotationValue = th
        rev_r.jointMotion.rotationValue = -th
        rev_d.jointMotion.rotationValue = diff_sign * ratio * th
        adsk.doEvents()
        vp.refresh()
        vp.saveAsImageFile(os.path.join(FRAMES_DIR, 'frame_%03d.png' % i), width, height)

    for j in (rev_l, rev_r, rev_d):       # park the mechanism level again
        j.jointMotion.rotationValue = 0.0
    root.isJointsFolderLightBulbOn = True
    if human:
        human.isLightBulbOn = True
    adsk.doEvents()
    vp.refresh()
    print('captured %d frames -> %s' % (frames, FRAMES_DIR))


def animate_diff(app, frames=FRAMES, width=1100, height=700, diff_sign=-1.0):
    """Close-up motion sequence of the differential clevis pivot (media/diff_pivot_motion.gif).

    Teaching view: bay/cabinet/wheels/human hidden, the near clevis cheek and the hub ghosted
    (opacity) so you can see the FIXED pin and the bronze bushings inside while the hub + bar
    rotate around them."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    joints = {j.name: j for j in root.asBuiltJoints}
    rev_l, rev_r, rev_d = joints['Rev_Arm_L'], joints['Rev_Arm_R'], joints['Rev_Diff']
    up = design.userParameters
    ratio = up.itemByName('diffOffsetX').value / up.itemByName('rockerArmY').value

    hidden = [o for o in root.occurrences
              if o.component.name in ('Core_Chassis', 'Payload_Bay', 'Rear_Cabinet',
                                      'Scale_Human_173cm', 'Motors')]
    for o in hidden:
        o.isLightBulbOn = False
    root.isJointsFolderLightBulbOn = False
    rk = next(o for o in root.occurrences if o.component.name == 'Rocker_Suspension')
    ghosts = []
    for b in rk.bRepBodies:                       # near cheek: camera sits on the +X side
        if b.name == 'Clevis_Cheek_F':
            ghosts.append((b, 0.35))
    for ch in rk.childOccurrences:
        if ch.component.name == 'Differential':
            for b in ch.bRepBodies:
                if b.name == 'Diff_Hub':
                    ghosts.append((b, 0.40))
    for b, op in ghosts:
        b.opacity = op

    os.makedirs(DIFF_FRAMES_DIR, exist_ok=True)
    vp = app.activeViewport
    cam = vp.camera
    cam.isSmoothTransition = False
    cam.eye = adsk.core.Point3D.create(40.0, -24.0, 14.0)     # cm; low, looking slightly up
    cam.target = adsk.core.Point3D.create(13.0, 0.0, 18.5)    # pivot at (130, 0, 185) mm
    cam.upVector = adsk.core.Vector3D.create(0.0, 0.0, 1.0)
    cam.viewExtents = 13.0
    vp.camera = cam
    vp.refresh()

    amp = math.radians(ARM_SWING_DEG)
    for i in range(frames):
        th = amp * math.sin(2.0 * math.pi * i / frames)
        rev_l.jointMotion.rotationValue = th
        rev_r.jointMotion.rotationValue = -th
        rev_d.jointMotion.rotationValue = diff_sign * ratio * th
        adsk.doEvents()
        vp.refresh()
        vp.saveAsImageFile(os.path.join(DIFF_FRAMES_DIR, 'frame_%03d.png' % i), width, height)

    for j in (rev_l, rev_r, rev_d):
        j.jointMotion.rotationValue = 0.0
    for b, _op in ghosts:
        b.opacity = 1.0
    for o in hidden:
        o.isLightBulbOn = True
    root.isJointsFolderLightBulbOn = True
    adsk.doEvents()
    vp.refresh()
    print('captured %d frames -> %s' % (frames, DIFF_FRAMES_DIR))


def play(app, cycles=3, steps=72, diff_sign=-1.0):
    """Play the rocker articulation LIVE in the Fusion viewport (no frames, no files).

    Drives the real joints (Rev_Arm_L/R + Rev_Diff) through `cycles` full see-saw cycles
    for direct on-screen review. NOTE: Fusion's Animation-workspace storyboards cannot be
    used for this - the API exposes no action/keyframe authoring, and that workspace
    ignores joints entirely - so this, joint dragging, and right-click 'Drive Joint' on a
    revolute joint are the ways to watch the mechanism inside Fusion."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    joints = {j.name: j for j in root.asBuiltJoints}
    rev_l, rev_r, rev_d = joints['Rev_Arm_L'], joints['Rev_Arm_R'], joints['Rev_Diff']
    up = design.userParameters
    ratio = up.itemByName('diffOffsetX').value / up.itemByName('rockerArmY').value
    vp = app.activeViewport
    amp = math.radians(ARM_SWING_DEG)
    for _c in range(cycles):
        for i in range(steps):
            th = amp * math.sin(2.0 * math.pi * i / steps)
            rev_l.jointMotion.rotationValue = th
            rev_r.jointMotion.rotationValue = -th
            rev_d.jointMotion.rotationValue = diff_sign * ratio * th
            adsk.doEvents()
            vp.refresh()
    for j in (rev_l, rev_r, rev_d):
        j.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    vp.refresh()
    print('played %d cycles' % cycles)


def pose(app, arm_deg, diff_sign=-1.0):
    """Hold a single articulation pose (degrees on the left arm); for checks and stills."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    joints = {j.name: j for j in root.asBuiltJoints}
    up = design.userParameters
    ratio = up.itemByName('diffOffsetX').value / up.itemByName('rockerArmY').value
    th = math.radians(arm_deg)
    joints['Rev_Arm_L'].jointMotion.rotationValue = th
    joints['Rev_Arm_R'].jointMotion.rotationValue = -th
    joints['Rev_Diff'].jointMotion.rotationValue = diff_sign * ratio * th
    adsk.doEvents()
    app.activeViewport.refresh()
    print('pose: arms %+.1f/%+.1f deg, diff %+.2f deg' % (arm_deg, -arm_deg,
                                                          diff_sign * ratio * arm_deg))


def make_gif(frames_dir=FRAMES_DIR, out=GIF, frame_ms=70):
    """Assemble the captured frames into a looping GIF (offline; needs pillow)."""
    from PIL import Image
    files = sorted(f for f in os.listdir(frames_dir) if f.endswith('.png'))
    if not files:
        raise RuntimeError('no frames in %s - run animate() in Fusion first' % frames_dir)
    imgs = [Image.open(os.path.join(frames_dir, f)).convert(
        'P', palette=Image.Palette.ADAPTIVE, colors=128) for f in files]
    imgs[0].save(out, save_all=True, append_images=imgs[1:], duration=frame_ms,
                 loop=0, optimize=True)
    print('wrote %s (%d frames, %.1f KB)' % (out, len(imgs), os.path.getsize(out) / 1024.0))
