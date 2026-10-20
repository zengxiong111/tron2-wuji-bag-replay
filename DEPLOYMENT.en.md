# Offline checks and hardware preparation

This repository provides Isaac target display and physical tracking; it has no hardware control entry and has not passed hardware admission. 中文：[DEPLOYMENT.md](DEPLOYMENT.md).

## Contents

- [Offline checks](#offline-checks)
- [Information to confirm](#information-to-confirm)
- [Commissioning sequence](#commissioning-sequence)

## Offline checks

Download assets following [README.en.md](README.en.md), then run at repository root. Example output and environment directories must be new and outside the repository:

```bash
python3 scripts/verify_bundle.py
python3.12 -m venv ../tron2-analysis-env
../tron2-analysis-env/bin/pip install -r requirements-analysis.txt
../tron2-analysis-env/bin/python scripts/audit_retimed_trajectory.py --limits-only --out ../tron2-limits-check
../tron2-analysis-env/bin/python scripts/audit_target_velocity.py --out ../tron2-target-check
python3 scripts/check_hardware_admission.py
../tron2-analysis-env/bin/python -m unittest discover -s tests -v
```

Integrity verification, target speed checks and eight tests should pass. Strict position checks return `2` for the inherited tiny excess around 0.001067°; hardware inventory checks also return `2`. These commands do not connect to devices. Finger targets match the old version; see README for physical tracking after arm expansion, local gain exceptions and temporal metrics. Inspect the full motion; a reference video alone does not establish physical quality.

## Information to confirm

| File | Required information |
| --- | --- |
| `configs/hardware_inventory.json` | TRON2 and both Wuji model, firmware, identity and evidence |
| `configs/joint_mapping.csv` | 56-axis SDK names/IDs, directions, offsets and units |
| `configs/hardware_limits.csv` | Per-axis position, velocity, acceleration, current, torque, temperature and load limits with source revisions |

The 16-axis body order is unconfirmed. Named hand source channels do not prove hardware SDK mapping. Do not replace unknown device ratings with model values.

## Commissioning sequence

1. Confirm devices, mapping, current trajectory hash and initial pose; base stays 0.45 m above the tabletop.
2. Handle position excess, starts/stops, braking, acceleration/jerk, physical tracking and contacts; recheck collisions and limits. 288 Hz is the file sampling rate.
3. Implement feedback timeouts, limits, fault/temperature monitoring and emergency stop; first verify read-only feedback and stopping.
4. After device-owner approval, test single axes, individual hands/arms, slow segments and slow full runs in order; store run records outside the repository.
