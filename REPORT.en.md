# Slowed trajectory verification report

Checked on 2026-10-09. **Model speeds pass; strict discrete collision checks pass; small angle violations remain; complete physical motor limits are unverified and direct hardware execution is not admitted.** Chinese: [REPORT.md](REPORT.md).

## Contents

- [Results](#results)
- [What this playback represents](#what-this-playback-represents)
- [What hardware deployment still requires](#what-hardware-deployment-still-requires)
- [Files and verification](#files-and-verification)
- [Official references](#official-references)

## Results

| Item | Measured result | Decision |
| --- | --- | --- |
| Duration | Old recording 57.9500 s → new file 72.4375 s | Time stretched 1.25×, equivalent to 4× bag speed |
| TRON2 arm speed | Tightest axis: left wrist yaw, 6.2047 rad/s, 34.86% of the model's 17.8 rad/s | Model speed passed |
| Wuji speed | Tightest axis: left ring PIP, 10.3810 rad/s, 80.72% of the model's 12.86 rad/s | Model speed passed |
| Head speed | Maximum about 0.000179 rad/s | Model speed passed |
| Joint angles | Left wrist pitch exceeds by 0.001473°; right wrist pitch by 0.000152°; left middle DIP by 0.001635° | **3 axes fail strict admission**; not an all-pass result |
| Collisions/penetration | 45,194 poses × 2717 geometry pairs, strict check finds 0 intersections and reports 0 mm depth | Discrete check passed; not continuous CCD |
| Isaac path playback | Full 72.4375 s, 2175 playback instants; readback error <0.000001 rad, rendered-link changes checked | Recorded-path replay verified; not new physics execution |

Speed checks use exact interval slopes of the piecewise-linear position path over the entire interval, rather than the file's averaged gradient. Linear interpolation has velocity jumps at knots, so **it does not establish finite continuous acceleration/jerk limits**; smooth start/stop admission is also incomplete.

Collision checks include all 13,909 original poses, every interval midpoint and the 240 Hz execution grid including the endpoint. There are 75 robot collision geometries; 2642 robot pairs and 75 table pairs are checked. The 133 same-body/direct-parent structural exclusions are explicitly recorded, with **no Wuji assembly exemptions**. Discrete triangle intersections do not establish exact solid clearance, continuous avoidance between samples or real contact response. Physical base `base_Link` has world Z=0 and tabletop Z=-0.45 m, maintaining the fixed 0.45 m gap.

## What this playback represents

This slows the previous 5× PhysX actual-joint recording; it is distinct from the older reference targets that still had intrahand overlap and is not video speed editing. All joint positions remain bitwise identical, while native sample frequency changes from 240 Hz to 192 Hz. `time` is already execution seconds, so play at **1× file time**.

The default player reproduces this path with position replay and no new dynamics integration. Executing it again with a controller at 4× speed may change tracking errors and contacts, requiring new actual-state recording and admission. These results do not prove task success or physical motor safety.

## What hardware deployment still requires

| Order | Required work | Current status |
| --- | --- | --- |
| 1 | Confirm TRON2 and both hand models/hardware revisions/firmware, SDK per-axis mapping, signs, zero offsets and units; resolve head yaw/pitch numbering conflict | Pending |
| 2 | Correct minor angle violations against the installed device ranges; produce speed/acceleration-limited motion with smooth start/stop, then recheck interpolated collisions | Incomplete |
| 3 | Obtain revision-bound per-axis speed, acceleration, rated/peak current or torque, temperature and sustained-load limits | Pending |
| 4 | Calibrate physical base, adapter, hands and environment; check collisions under tracking errors and braking conditions | Incomplete |
| 5 | Implement and test emergency stop, feedback timeouts, fault stops, limits, tracking-error protection and complete state logging | Hardware player not prepared |
| 6 | Supervised single-axis/hand/arm checks, followed by staged low-speed, low-load trials; record speed, current/torque, temperature and error codes | Not performed |

**Passing model speeds cannot establish that all real motor limits are respected.** Wuji's documented 2.0 A is an allowed control hard cap, not an installed device's rated current; 1000 Hz is control/communication frequency, not mechanical speed. Model `effort` or simulation torque cannot directly be converted into Wuji current. Actual ratings, revisions and mappings are covered by [HARDWARE.md](HARDWARE.md) / [HARDWARE.en.md](HARDWARE.en.md), with unknown fields pending.

## Files and verification

- Current trajectory: `trajectories/retimed_actual_4x.npz`, SHA-256 `23b555d6a4bf6df33fa4da4e5905a10f9fe375f6447784edd44079e412bdfcd1`.
- [Per-axis checks](evidence/motor_limits/joint_limits.csv), [speed/angle results](evidence/motor_limits/report.json), [collision result](evidence/collision/gate.json), [coverage](evidence/collision/coverage.json), [complete path playback verification](evidence/isaac_playback.json).
- Retiming lineage: [configs/retiming.json](configs/retiming.json); playback binding: [configs/trajectory.json](configs/trajectory.json). The source bag and old trajectory copies are not in the current runtime bundle.

Install the analysis environment and download runtime assets as described in README, then run from the repository root; output must not already exist:

```bash
.venv-analysis/bin/python scripts/audit_retimed_trajectory.py --out ../tron2-retimed-check
python3 scripts/check_hardware_admission.py
python3 -m unittest discover -s tests -v
```

The first two commands currently return 2 for minor angle violations and missing physical-hardware admission evidence, respectively. Tests should pass. Do not interpret nonzero admission as success. `scripts/retime_recorded_trajectory.py --tracking /path/to/tracking.npz --out ../reproduced.npz` rebuilds the file from the hash-identical old recording, which remains an external source and is not distributed in the current runtime bundle.

## Official references

- [Wuji control guide](https://docs.wuji.tech/docs/zh/wuji-hand/latest/control-guide/): angle/speed units, current in A and the 2.0 A control hard cap; SDK S1–S4 to model-actuator ordering is not yet provided.
- [Wuji user notice](https://docs.wuji.tech/docs/zh/wuji-hand/latest/user-notice/): Beta load and thermal boundaries.
- [TRON2 user manual](https://www.limxdynamics.com/zh/documents/844648486841487360), [TRON2 SDK guide](https://www.limxdynamics.com/zh/documents/847884267345285120): device performance, joint numbering and state units. Endpoint performance is not a per-axis motor limit.
