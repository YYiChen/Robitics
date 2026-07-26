# Pi Service Agent Rules

These rules apply only to `pi_service/` and its descendants. They deliberately
do not define policy for the rest of the Robitics repository.

## Start Here

Before editing:

1. Read `pi_service/README.md`.
2. Run `git status --short --branch` from the repository root.
3. State the bounded target, owned files, validation commands, and whether
   hardware access is required.
4. Inspect only the formal module or experiment involved in that target. Do not
   recursively read unrelated experiments, runtime logs, caches, vendor files,
   archives, sibling repositories, or hardware assets.

Keep planning in the task conversation and Git history. Do not create a new
project-management directory unless the user explicitly requests one.

## Project Boundaries

- `robot_web/` is the formal Raspberry Pi web/control service.
- `robot_web/api/` owns HTTP endpoint registration; `app.py` only assembles
  services, CLI arguments, lifecycle, and route selection.
- `robot_web/control/` contains independently testable controller helpers;
  top-level `controller.py` remains the only serial and M1/M2 owner.
- `robot_web/media/` contains camera profiles, persistence, image processing,
  and metrics; camera lifecycle remains in the two camera facade modules.
- `robot_web/routes/` contains formal route implementations. Production route
  code must not import implementation from `experiments/`.
- `tests/` contains regression tests for the formal service.
- `experiments/<target>/` contains isolated validation work. An experiment is
  not a production entry point merely because its tests pass.
- Root `start_*.sh`, `run_*.sh`, `install_*.sh`, and `verify_*.sh` files are
  operator entry points. `start_robot.sh` is the normal MJPEG control entry.
- Runtime output belongs in `logs/`, `run/`, `runtime_logs/`, or another ignored
  runtime directory. It must not be staged as source.
- Persisted vehicle and camera configuration is deployment state. Do not
  overwrite, migrate, or stage it unless the target explicitly owns that
  configuration contract.
- Formal configuration sections belong in `config/defaults.json`; deployment
  overrides and UI writes belong in ignored `config/local.json`. Do not add a
  new production JSON store without documenting why it cannot use that
  sectioned contract.

If work requires changing a file outside `pi_service/`, stop and tell the user
why the local boundary is insufficient before making that change.

## Change Discipline

- One target per change set and commit.
- Declare the files or directories owned by the target before editing.
- Do not mix feature work, tuning, cleanup, and directory reorganization.
- Do not move shared modules merely to improve appearance. First verify imports,
  launch scripts, Pi deployment paths, and tests.
- Put reusable production behavior in `robot_web/`; keep probes, sandboxes,
  one-off calibration, and hardware trials under `experiments/`.
- A new experiment needs its own folder and `README.md` describing purpose,
  safety boundary, entry command, outputs, validation, and promotion criteria.
- Update `pi_service/README.md` when the formal entry point, active route mode,
  directory contract, or validation command changes.
- Do not add long prompt documents. Repository-wide maintenance rules belong in
  this file; experiment instructions belong in that experiment's README.

## Validation

Choose the smallest deterministic validation that covers the target.

From the repository root on Windows:

```powershell
py -3 pi_service/verify_paths.py
py -3 -m unittest discover -s pi_service/tests -v
node --check pi_service/robot_web/static/app.js
git diff --check -- pi_service
git status --short --branch
```

For an experiment:

```powershell
py -3 -m unittest discover -s pi_service/experiments/<target> -p "test_*.py" -v
```

Run Bash syntax checks and Pi-only dependency checks on the Raspberry Pi when
Windows does not provide the required environment. Record skipped checks and
their reason; do not describe them as passing.

Passing unit tests, HTTP responses, or serial acknowledgements do not prove
physical vehicle motion. Report software validation, Pi deployment validation,
and physical-car validation separately.

## Git and Deployment

- Inspect staged and unstaged changes before committing.
- Stage only paths owned by the current `pi_service` target.
- After a requested `pi_service` code change passes its declared checks, create
  the scoped local Git commit before deploying it to the Raspberry Pi.
- After that commit, synchronize only its relevant changed files, preserving
  their repository-relative paths under `/home/g11/Desktop`. Do not copy the
  entire repository when a bounded file sync is sufficient.
- Keep `main` deployable; use a target branch for unfinished or experimental
  work when publication is involved.
- Do not rewrite shared history or force-push to make the branch look tidy.
- A Pi file sync is a deployment step, not authorization to start services,
  flash firmware, move motors, or run autonomous control.
- Never store Pi credentials, tokens, passwords, private keys, or local
  authentication helper scripts in this directory.

## Hardware Safety

- Exactly one process may own the camera and M1/M2 motor path during a live run.
- Motor-capable experiments must start disabled or require explicit operator
  arming.
- Preserve browser/Pi/Arduino timeout stopping and explicit `STOP` behavior.
- Never start motors, autonomous driving, card motors, or firmware flashing
  without explicit user authorization for that physical action.

## Completion

A target is complete only when:

1. the owned diff is reviewed;
2. the declared validation has been run;
3. runtime artifacts and deployment configuration are not staged;
4. remaining software, Pi, and physical-validation gaps are reported;
5. the intended files are committed or the uncommitted state is stated clearly.
