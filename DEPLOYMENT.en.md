# Physics Replay

This page explains how to run Isaac Sim / PhysX again from the trajectory reference and how to interpret a new run. Ordinary playback reads the accepted actual-state trace; a new simulation starts only when physics mode is explicitly enabled. Chinese: [DEPLOYMENT.md](DEPLOYMENT.md).

## Contents

- [Prerequisites](#prerequisites)
- [Run a new PhysX replay](#run-a-new-physx-replay)
- [Interpreting results](#interpreting-results)

## Prerequisites

- Download runtime assets and verify them with `python3 scripts/verify_bundle.py` as described in [README.en.md](README.en.md).
- Install Isaac Sim 5.1 and confirm its Python interpreter starts on this host; a compatible GPU is required. Fresh physics also requires NumPy, SciPy, and Pinocchio in that same Python environment. The verified environment used NumPy 1.26.0, SciPy 1.15.3, and Pinocchio (`pin`) 2.7.0.
- The replay tool creates a new output directory outside the repository. Keep its outputs and logs for inspection; do not replace accepted results with a new run.

## Run a new PhysX replay

Run from the repository root and replace the path with your local Isaac Sim Python interpreter:

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python --physics
```

`--physics` creates a new PhysX run and actual-state trace from the reference trajectory, using 480 Hz and the published solver, drive, and official assembly-collision settings. All 56 axes use acceleration drives; arm force drive and RNEA feedforward are disabled. It needs significant compute; runtime depends on the host. Inspect the new report, trace, and logs. A successful command or a normal-looking video does not mean the new result passed acceptance.

Default mode does not step physics; it plays the recorded path bound to the accepted actual-state trace. Both modes write to a run directory outside the repository and do not modify release assets. Do not apply another 5× time scaling to the trajectory.

## Interpreting results

- The accepted trajectory covers the 344.718 s source interval with body feedback, corresponding to 68.944 s of 5× execution time. The first approximately 2.494 s of the source bag is outside this coverage.
- Aggregate acceptance applies to the specified USD, Isaac / PhysX settings, model, and geometry-pair population. Recheck input bindings, actual states, speed, wrist gap, thumb motion, contacts, and geometry for every new run; earlier acceptance does not carry over.
- Dense mesh checks are sampled evidence, not a continuous collision detection (CCD) certificate. The tabletop-to-physical-base-origin gap is fixed at 0.45 m.
- The offline RNEA effort proxy is 1.251 of the URDF effort limit and exceeds that limit. The real body-to-actuator mapping is also unconfirmed. These tools and results do not provide hardware control, hardware admission, or torque acceptance.

For the current accepted run, see `evidence/imported/acceptance_155.json`, `evidence/imported/actual_mesh_133.json`, `evidence/imported/physics_report.json`, and the binding records in `metadata/`.
