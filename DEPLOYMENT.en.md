# TRON2 + Wuji hardware deployment guide

Prepare the current trajectory for eventual hardware execution. **Hardware admission has not passed, and this repository has no hardware player. Existing Isaac playback commands do not control the robot.** Chinese: [DEPLOYMENT.md](DEPLOYMENT.md).

## Contents

- [Current status](#current-status)
- [Offline preparation](#offline-preparation)
- [Deployment steps](#deployment-steps)
- [Stopping and acceptance](#stopping-and-acceptance)
- [References](#references)

## Current status

| Item | Conclusion |
| --- | --- |
| Trajectory | `trajectories/retimed_actual_4x.npz`, 56 axes, 72.4375 seconds; execute at 1× file time, without another 4× multiplier |
| Model speeds | TRON2 / Wuji peaks are 34.86% / 80.72% of model speed limits |
| Angles | Minor overshoot on 3 axes, maximum 0.001635°; strict checking failed |
| Collision | No intersections at 45,194 discrete poses; continuous motion, braking and hardware collision safety remain unproven |
| Hardware evidence | Device revisions, joint mappings and per-axis limits are pending; current, torque and temperatures are unverified |

The current piecewise-linear trajectory has velocity jumps. Continuous acceleration, jerk and start/stop transitions have not passed acceptance. **Slowing down does not establish hardware safety.**

## Offline preparation

Requires private repository access, Git, GitHub CLI and Python 3.12. Download instructions are in [README.en.md](README.en.md#download-and-upgrade). Run these commands from the repository root; they do not connect to devices:

```bash
python3 scripts/verify_bundle.py
python3.12 -m venv .venv-analysis
.venv-analysis/bin/pip install -r requirements-analysis.txt
.venv-analysis/bin/python scripts/audit_retimed_trajectory.py --out ../tron2-deploy-check
python3 scripts/check_hardware_admission.py
```

`../tron2-deploy-check` must be a new directory outside the repository. The trajectory and hardware admission checks currently both return exit code `2`, for angle violations and missing hardware evidence respectively. The last command only checks offline evidence completeness; passing never authorizes hardware motion.

## Deployment steps

Complete these steps in order. Resolve any failed step before proceeding.

1. **Confirm devices and every joint.** The device owner verifies the model, hardware revision, firmware and identifier for TRON2 and each hand. Confirm every SDK axis, direction, zero offset and unit. Resolve the TRON2 16-channel order and head yaw/pitch conflict; confirm the Wuji joint-to-S1–S4/actuator mapping.
2. **Supply physical limits.** Record revision-bound evidence and per-axis position, speed, acceleration, current, torque, temperature and sustained-load limits in the files below. Do not substitute model values for unknowns or merely change statuses to `confirmed`.
3. **Generate an executable trajectory.** Repair angle violations, add smooth starts/stops and bounded acceleration/jerk, and resample to the confirmed controller period. 192 Hz is the current file sample rate, not a confirmed hardware control rate. After changes, recheck angles, speeds, collisions and physical tracking; record new trajectory and report hashes.
4. **Check the installation and starting pose.** Inspect adapters, cables, payload, table and surrounding obstacles. The tabletop-to-physical-base-origin gap stays fixed at 0.45 m (current `base_Link` Z=0, tabletop Z=-0.45 m). Verify the transition from the actual starting pose to the first frame, and clearance for tracking error, communication delay and braking.
5. **Then prepare the hardware player.** Implement control using the confirmed SDK, with per-axis limits, feedback timeouts, tracking-error monitoring, fault stops, emergency stopping and recording. Derive thresholds and control gains from device parameters and commissioning results; do not copy simulation gains. First verify read-only feedback and fault handling, then obtain device-owner approval for controlled motion commissioning.
6. **Commission progressively.** Start with controlled single-axis direction/zero checks, then one hand, one arm, slow segments and the full path. Advance only after each stage passes and the device owner approves; validate the target 4× motion speed last. Record actual joint states, current/torque, temperatures, fault codes and timestamps for every run outside the repository.

| File | Required information |
| --- | --- |
| `configs/hardware_inventory.json` | Identities of all three devices, evidence sources and trajectory admission report binding |
| `configs/joint_mapping.csv` | Hardware SDK mappings, directions, zero offsets and units for 56 axes |
| `configs/hardware_limits.csv` | Physical limits, applicable revisions and evidence sources for 56 axes |

## Stopping and acceptance

| Situation | Action |
| --- | --- |
| Wrong mapping, lost feedback, device fault, exceeded limits, abnormal heating, tracking error beyond approved bounds or unexpected contact | Follow the manufacturer's stop/emergency-stop procedure; investigate before resuming |
| Changed parameters, payload, assembly, controller or trajectory | Revalidate affected checks and update hashes and admission records |

Hardware acceptance requires all of the following: confirmed evidence and mappings; passing limits and collision checks for the corrected trajectory; passing start/stop and braking checks; a complete run without exceeded limits or unexpected contact; feedback, temperatures and tracking errors within approved bounds; and device-owner sign-off. **An Isaac video, passing offline checks or completed tables cannot replace hardware acceptance.**

## References

- [Verification report](REPORT.en.md); [hardware parameters and mapping](HARDWARE.en.md)
- [Official Wuji documentation](https://docs.wuji.tech/docs/zh/wuji-hand/latest/)
- [TRON2 SDK guide](https://www.limxdynamics.com/zh/documents/847884267345285120); [TRON2 user manual](https://www.limxdynamics.com/zh/documents/844648486841487360)
