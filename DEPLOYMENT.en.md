# Hardware deployment preparation

This repository provides reference display and physical tracking. **It has no hardware controller and has not passed hardware admission.** 中文：[DEPLOYMENT.md](DEPLOYMENT.md)。

## Contents

- [Offline checks](#offline-checks)
- [Required information](#required-information)
- [Commissioning sequence](#commissioning-sequence)

## Offline checks

Download assets using [README.en.md](README.en.md), then run from the repository root. Output directories must be new and outside the repository:

```bash
python3 scripts/verify_bundle.py
python3.12 -m venv ../tron2-analysis-env
../tron2-analysis-env/bin/pip install -r requirements-analysis.txt
../tron2-analysis-env/bin/python scripts/audit_retimed_trajectory.py --limits-only --out ../tron2-limits-check
../tron2-analysis-env/bin/python scripts/audit_target_velocity.py --out ../tron2-target-check
python3 scripts/check_hardware_admission.py
../tron2-analysis-env/bin/python -m unittest discover -s tests -v
```

Integrity verification, target speed checks and eight tests should pass. Strict position checks should return `2` because original5x tiny excess up to approximately 0.001635° is retained; hardware information checks also return `2`. These scripts do not connect to devices. Physical actual tracking still has maximum deviation around28.10° and cannot inherit reference fidelity admission.

## Required information

| File | Confirm |
| --- | --- |
| `configs/hardware_inventory.json` | TRON2 and left/right Wuji model, firmware, identifiers and evidence |
| `configs/joint_mapping.csv` | SDK names/indices, directions, zero offsets and units for56 axes |
| `configs/hardware_limits.csv` | Revision-bound per-axis position, velocity, acceleration, current, torque, temperature and load limits |

The16-axis body order remains unconfirmed, with a conflict between candidate head yaw/pitch order and manual indices. The40 named hand channels do not confirm the installed devices' SDK mapping. Leave unknown fields blank; model values do not substitute for device ratings.

## Commissioning sequence

1. Confirm devices, mappings, trajectory hash and initial pose; keep the mounting gap fixed at0.45m.
2. Prepare a trajectory/controller meeting hardware limits. Resolve tiny angle excess, starts/stops, braking, acceleration/jerk and physical tracking errors, then recheck collisions/limits. The file's288Hz does not define hardware control frequency.
3. Implement per-axis limits, feedback timeouts, tracking-error/fault/temperature monitoring and emergency stops; validate read-only feedback and stopping first.
4. After device-owner approval, progress through single axes, single hands/arms, slow segments and slow full runs, increasing speed only after each stage passes.
5. Follow manufacturer stop procedures for mapping errors, lost feedback, limit violations, abnormal heating or unexpected contact. Never resume automatically; resolve causes and revalidate. Store runtime records outside the repository.
