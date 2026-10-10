# Hardware deployment preparation

This repository provides Isaac Sim playback. **It includes no hardware controller and has not passed hardware admission.** These steps support further preparation after handoff. Chinese: [DEPLOYMENT.md](DEPLOYMENT.md).

## Contents

- [Offline checks](#offline-checks)
- [Device information](#device-information)
- [Deployment sequence](#deployment-sequence)

## Offline checks

Download assets using [README.en.md](README.en.md), then run from the repository root:

```bash
python3 scripts/verify_bundle.py
python3.12 -m venv ../tron2-analysis-env
../tron2-analysis-env/bin/pip install -r requirements-analysis.txt
../tron2-analysis-env/bin/python scripts/audit_retimed_trajectory.py --limits-only --out ../tron2-limits-check
python3 scripts/check_hardware_admission.py
../tron2-analysis-env/bin/python -m unittest discover -s tests -v
```

`../tron2-limits-check` must be a new directory outside the repository. The angle check currently returns `2` (3 axes have minor overshoot); hardware admission also returns `2` (incomplete information). Integrity verification and unit tests should pass. Hardware admission checks offline information without connecting to devices.

To recompute the complete discrete collision check, use the same environment and the command below. It takes longer and writes results to a new external directory:

```bash
../tron2-analysis-env/bin/python scripts/audit_retimed_trajectory.py --out ../tron2-full-check
```

## Device information

The device owner must complete these files. Leave unknown fields blank; model values cannot substitute for physical-device ratings.

| File | Required confirmation |
| --- | --- |
| `configs/hardware_inventory.json` | TRON2, left-hand and right-hand models, hardware revisions, firmware, identifiers and evidence |
| `configs/joint_mapping.csv` | SDK names/IDs, directions, zero offsets and units for 56 axes |
| `configs/hardware_limits.csv` | Per-axis position, speed, acceleration, current, torque, temperature and sustained-load limits, with source revisions |

The source bag has 16 unnamed body-state channels. Their order remains unconfirmed, with conflicting head yaw/pitch order: the candidate order is yaw, pitch, while the manual lists 14=pitch and 15=yaw. The hands have 40 named position channels, but name matching does not confirm the installed devices' SDK mapping. `bag_hand_index` / `bag_pos_index` are nested hand / position indices; `bag_raw_index` is the flattened 40-channel index.

## Deployment sequence

1. **Confirm devices and mappings.** Verify the information above and resolve body-channel order and hand-actuator assignments.
2. **Prepare an executable trajectory.** Fix angle overshoot, add smooth starts/stops and acceleration/jerk limits, then resample at the actual control period. The file's 192 Hz sample rate does not define hardware control frequency. Recheck limits, collisions and physical tracking; update trajectory and report hashes.
3. **Check the site and initial pose.** Keep the tabletop-to-physical-base gap at 0.45 m; do not change mounting height to eliminate collisions. Check adapters, cables, loads, obstacles, the first-frame transition and braking space.
4. **Implement a hardware controller.** Use the confirmed SDK with per-axis limits, feedback timeouts, tracking-error monitoring, fault stops and emergency stops. Verify read-only feedback and fault handling first. Parameters and gains must be confirmed against the physical devices.
5. **Commission in stages.** After device-owner approval, check individual axes, hands, arms, slow segments and the full trajectory, then validate the target motion speed. Record actual states, current/torque, temperatures, fault codes and timestamps outside the repository.

Follow the manufacturer's stop procedure for mapping errors, lost feedback, limit violations, abnormal temperatures or unexpected contact. Resolve the cause before continuing. Revalidate affected checks after changes to parameters, loads, assembly, controllers or trajectories.

Hardware acceptance requires device-owner confirmation of information, mappings, limits, starts/stops, braking and complete-run results. Isaac videos and offline tables cannot substitute for physical commissioning. Acceleration/jerk remains unbounded for the current piecewise-linear path; slowing it alone does not resolve this.
