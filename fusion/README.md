# Payload Rover — Parametric Fusion Model

A fully parametric Autodesk Fusion model of a **skid-steer, payload-carrying rover**
controlled by an **ESP32-based PCB**. Designed for a **~200 kg payload**, riding on
**four single-sided hub motors** (Robokits RKI 9051-class, 10", 2.30 kg) driven
independently — no steering linkage; the rover skid-steers.

Everything is generated **directly in Fusion via the Fusion MCP**, which executes the
modules below from disk (they stay the single source of truth):

| Module | Builds |
|---|---|
| `payload_car/builder.py` | the car: chassis, payload bay, rear cabinet, rocker suspension |
| `payload_car/wheel_mount.py` | the hub-motor wheel module → saved as `components/hub_motor.f3d` |
| `payload_car/assemble.py` | the full assembly: car + 4 wheel xrefs + scale human → cloud doc `payload_car_4wd` + `components/payload_car_4wd.f3d` |

## Project layout (uv)

```
fusion/
├── pyproject.toml            uv project + console script `payload-car`
├── README.md
└── payload_car/              importable package + Fusion script folder
    ├── builder.py            car model: PARAMS/ROCKER_PARAMS + build(app) + build_rocker()
    ├── wheel_mount.py        hub-motor wheel module (RKI 9051 drawing) + dropout/torque-arm mount
    ├── assemble.py           assembly step: wheel transforms, Motors group, joints, save + export
    ├── motion.py             rocker motion study: drives the joints, renders media/rocker_motion.gif
    ├── cli.py                `payload-car` CLI (validate / params / bootstrap)
    ├── payload_car.py        Fusion script entry
    └── payload_car.manifest
```

## Run it with uv

`adsk` (the Fusion API) only exists inside Fusion; the CLI covers the offline parts:

```bash
cd fusion
uv run payload-car validate     # offline-check every parameter expression
uv run payload-car params       # print the car + rocker parameter tables
uv run payload-car bootstrap    # emit the Fusion/MCP runner that assembles the rover
```

## Build the model in Fusion

With Fusion open and the MCP connected, run the snippet `uv run payload-car bootstrap`
prints — it execs `assemble.py`, which builds the car, inserts the four `hub_motor`
xrefs, adds the scale human, cloud-saves `payload_car_4wd` and exports the `.f3d`.
(`hub_motor` must exist as a cloud doc first — run `wheel_mount.py`'s build once.)

## What gets built

```
payload_car_4wd
├── Core_Chassis        base plate + side rails + cross members + tubular bumpers
├── Payload_Bay         galvanised deck + welded box: 4 MS square-tube columns + steel sheets
├── Rear_Cabinet        same welded construction; twin rear doors, battery, vertical ESP32
│                       PCB + 4 motor drivers, cable glands, power switch, E-stop on top
├── Rocker_Suspension   DELETABLE walking-beam rocker (see below)
├── Motors              4× hub_motor xrefs at (±wheelbase/2, ±trackWidth/2), axle z = wheelOD/2
└── Scale_Human_173cm   blocky reference man
```

### Rocker suspension (optional, deletable)

One MS 50×50 RHS walking-beam arm per side runs inboard of the tyres; its outer face sits
flush on each hub motor's **dropout plate** (the module's only mounting face — the old
chassis-tie gusset is gone from `wheel_mount.py`) and is through-bolted with M10s. Each arm
see-saws on a central **Ø20 EN8 shaft in two bearings** carried by cheeks off the base
plate; a **differential cross-bar** with rod-end drop-links ties the two arms so the body
holds the mean pitch. The wheels hang off the arms only — nothing else touches the chassis —
and the chassis is raised (`groundClearance = 320`) so everything clears at full travel.
Delete the `Rocker_Suspension` component (or set `WITH_ROCKER = False`) to remove the whole
subsystem (then re-add a rigid dropout-to-chassis tie).

## Editing parameters

Every dimension is a Fusion **User Parameter** (54 of them: `PARAMS` + `ROCKER_PARAMS` in
`builder.py`, plus the wheel module's own table in `wheel_mount.py`). Change anything in
**Modify → Change Parameters** and the model regenerates.

## Coordinate system & key levels

- **+X** forward, **+Y** left, **+Z** up, ground at `z = 0`, model centred on origin.
- Wheel axles at `z = wheelOD/2 = 127`; base plate bottom at `groundClearance = 320`
  (raised so tyre crowns, dropout plates and arm tips clear it at full ±8.5° travel);
  the rocker lives in the gap between them.
- Hub-motor dropout plates sit at `|y| ≈ 197.5–205.5`; the rocker arms land flush on them.
- Fusion gotcha: XZ/YZ sketch planes don't map local axes to global X/Y/Z directly —
  `cyl_y` negates Z, `cyl_x` swaps/negates (see helpers in `builder.py`).

### Motion study

`assemble.py` creates real joints: revolute `Rev_Arm_L` / `Rev_Arm_R` (about each arm's
pivot boss, ±9°) and `Rev_Diff` (about the differential trunnion, ±8°), plus rigid
`Fix_Wheel_*` joints so each wheel swings with its arm; `Core_Chassis` is grounded.
**Drag any arm or wheel in Fusion** to feel the mechanism, or run `motion.py`'s
`animate(app)` (via the MCP) to sweep a full articulation cycle and dump frames, then:

```bash
uv run python -c "from payload_car.motion import make_gif; make_gif()"   # media/rocker_motion.gif
```

(The Fusion API does not expose its Motion Study timeline, so the study is joint-driven.)

There is also a close-up teaching sequence of the **differential clevis pivot** — fixed pin
clamped in two cheek plates, bronze flanged bushings in the rotating hub — rendered with the
hub and near cheek ghosted: `animate_diff(app)` → `media/diff_pivot_motion.gif`.

## Notes / next steps

- Bodies are representative volumes for packaging/layout. For real 200 kg structural
  validation, assign materials and run a Simulation study.
- The diff drop-links are rigid with the bar (real rod ends articulate); the closed loop
  is enforced by the scripted ratio `diffOffsetX / rockerArmY`, not by the solver.
