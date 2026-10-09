# Four-times trajectory inspection report

**Conclusion: model speed and target-angle checks pass; the full hardware motor envelope remains unverified.** This report covers the current four-times, head-forward trajectory. Checked on 2026-10-09.

Chinese: [REPORT.md](REPORT.md).

## Contents

- [Passed checks](#passed-checks)
- [Unverified checks](#unverified-checks)
- [Recheck](#recheck)
- [Official sources](#official-sources)

## Passed checks

A speed ratio approaching 100% leaves less margin. Per-axis limits below come from the delivered URDF/USD models and do not automatically establish actual device ratings.

| Check | Result | Assessment |
| --- | --- | --- |
| TRON2 arm target speed | Fastest axis 6.182 rad/s, limit 17.8 rad/s, ratio 34.73% | Within model limit |
| Wuji target speed | Tightest axis: left little-finger PIP, 11.888 rad/s against 12.86 rad/s, ratio 92.44% | Within model limit, approximately 7.56% margin |
| Actual physical-state speed | All 56 axes within model limits, maximum ratio 88.33% | Pass; estimated from 240 Hz positions |
| Target angle | Complete cubic interpolation for all 56 axes stays within model and documented angle ranges | Pass |
| TRON2 wrist reference target speed | Left/right 1.000 / 0.969 m/s, published endpoint specification 5 m/s | Reference measurement only; no endpoint admission |
| TRON2 wrist reference target acceleration | Left/right 14.025 / 14.507 m/s², published endpoint specification 36 m/s² | Reference measurement only; no endpoint admission |
| Actual-state penetration | Full 17,386 states × 2,717 geometry pairs, none >0.1 mm | Discrete check passed |
| Jitter improvement | Joint high-frequency RMS down 78.98%, wrist down 67.01%, acceleration P95 down 84.44% | Improvement over original four-times replay under matched conditions |

Target speed uses analytical extrema in every cubic interval, including peaks between samples. Wrist values use 240 Hz FK differences at `wrist_roll_L_Link` / `wrist_roll_R_Link` origins; the actual tool endpoint may differ, and continuous-time peaks are not proven.

## Unverified checks

| Item | Why no admission is possible |
| --- | --- |
| Hardware per-axis rated/peak speed and acceleration | Complete parameters bound to the specific device version are unavailable; model limits cannot replace ratings |
| Current, torque and payload | No four-times hardware current/torque/load measurements exist for this new trajectory; original-speed bag feedback cannot establish accelerated behavior |
| Wuji current | The 2.2 Beta control guide specifies `effort` in A and an allowed hard ceiling of 2.0 A; there is no comparable four-times current recording, and simulation N·m cannot be treated as A |
| Temperature and continuous duty | Wuji documents state that long full-load thermal behavior and load capability are not committed Beta ratings; no temperatures were measured here |
| Joint mapping | The bag's 16 body axes have no names. Manual indices 14=pitch,15=yaw contradict the tentative head order; both head targets are zero here. Wuji SDK S1–S4 to model actuator mapping also requires device confirmation |
| Contact-free targets and continuous collision freedom | Intrahand reference targets overlap by up to 13.301 mm. Accepted actual states are constrained by physical contacts. This is not CCD evidence or a collision-free kinematic reference |

This release is therefore an **Isaac simulation trajectory**. Hardware admission needs device identity, per-axis ratings and confirmed mappings, plus speed, current/torque, temperature and error-code recordings for this trajectory. Simulation drive gains must not be copied directly to hardware.

## Recheck

1. Download runtime inputs and verify hashes as described in [README.en.md](README.en.md).
2. Run in the analysis environment; outputs go to new directories outside the repository:

```bash
.venv-analysis/bin/python scripts/check_motion_limits.py --out ../tron2-4x-checks
.venv-analysis/bin/python scripts/122_audit_replay_motor_velocity.py --out ../tron2-4x-speeds
```

This file is the plain-language report. Machine-verifiable results: [motor checks](evidence/motor_checks/report.json), [per-axis angle/speed table](evidence/motor_checks/joint_limits.csv), [detailed speeds](evidence/final/motor_limit_audit/joint_speeds.csv), [collision result](evidence/final/audit/gate.json), [jitter metrics](evidence/final/temporal_quality.json).

## Official sources

- [Wuji control guide](https://docs.wuji.tech/docs/zh/wuji-hand/latest/control-guide/): joint units, current ceiling and SDK mapping boundaries; 1000 Hz is a communication/control frequency, not a mechanical speed limit.
- [Wuji user notice](https://docs.wuji.tech/docs/zh/wuji-hand/latest/user-notice/): Beta load and thermal capability boundaries.
- [TRON2 manual](https://www.limxdynamics.com/zh/documents/844648486841487360): endpoint performance in section 1.2, joint angles and numbering in section 1.5.
- [TRON2 SDK guide](https://www.limxdynamics.com/zh/documents/847884267345285120): q in rad, dq in rad/s, tau as estimated N·m. Example control frequency is not a motor speed limit.
