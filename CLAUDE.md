# Payload Car — Project Conventions

## Python: ALWAYS use uv — never bare `python` / `python3` / `pip`
The uv project lives in `fusion/` (`pyproject.toml`, `.venv`, `uv.lock`).

- Compile / syntax-check: `uv run python -m py_compile <file>`
- Run anything: `uv run python ...`, `uv run payload-car <cmd>`
- Add dependencies: `uv add <pkg>` (dev tools: `uv add --dev <pkg>`). Do **not** pip install or hand-edit deps.
- Create the project with `uv init` if it doesn't exist; the project venv (`.venv`) is managed by `uv run`.

If a uv project doesn't exist for new Python work, create one first.

## CAD / Fusion
- Build models **directly in Fusion via the Fusion MCP** (`fusion_mcp_execute`, `featureType: "script"`, entry `def run(_context):`). Don't just hand over a script to run manually.
- Model logic is the single source of truth under `fusion/payload_car/`:
  - `builder.py` — the payload car (chassis, bay, cabinet, rocker suspension)
  - `wheel_mount.py` — the single-sided hub-motor wheel mount
  - `assemble.py` — full assembly: car + 4 wheel xrefs + scale human + suspension joints, cloud-save + f3d export
  - `motion.py` — joint-driven rocker motion study (frames -> `media/rocker_motion.gif` via `make_gif`)
  The MCP execs these from disk (`exec(compile(open(path).read()...))`) so the file stays canonical.
  Assembly changes go in `assemble.py`, never in throwaway MCP snippets.
- Do **not** catch exceptions in the MCP `run()` entry — let them surface for debugging (cosmetic helpers like fillets may guard).
- Reusable parts are exported as `.f3d` into `components/` (e.g. `components/hub_motor.f3d`) for later assembly into the car.
- Fusion gotcha: XZ/YZ sketch planes map local +V to global **−Z** — negate the Z expression in `cyl_y`/`cyl_x` or parts build mirrored below ground.
