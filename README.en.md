# TRON2 + Wuji repaired 6x bag replay

This branch publishes the physically verified **6x trajectory**: **48.295833 simulation seconds**, 56 joints, 240 Hz, 11,591 actual states, with the head facing forward. Main and the 5x branch remain unchanged. 中文：[README.md](README.md)。

## Contents

- [Install and download](#install-and-download)
- [Run in Isaac](#run-in-isaac)
- [Repository structure](#repository-structure)
- [Verified results](#verified-results)
- [Motion changes and limitations](#motion-changes-and-limitations)

## Install and download

1. Prepare Git, GitHub CLI and Python 3.12. Display and physics execution also require Isaac Sim 5.1 and a compatible GPU; its Python needs NumPy. Offline analysis dependencies are in `requirements-analysis.txt`.
2. Run:

```bash
gh auth login
git clone --branch replay-6x-collision-smooth https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

The model reuses the pinned `v1.1.0` Release asset; 6x trajectory data comes from this branch's `6x-collision-smooth-20261014` prerelease. Verification should print `Verified 85 runtime files`.

For an existing clone, switch to this branch, run `python3 scripts/download_assets.py --upgrade`, then verify. Upgrade replaces only hash-matched managed references and removes hash-matched retired 4x/5x actual paths and the 5x video. Modified files are not overwritten.

## Run in Isaac

Run from the repository root, replacing `/path/to/isaac/python.sh` with your Isaac Python entry.

1. **Display the verified actual recorded path**:

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
```

File time is already 6x execution seconds: display at **1x file time**, without another factor of 6. This mode sets recorded poses and does not rerun physics.

2. **Execute physics again at 6x action speed**:

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

This mode integrates at 240 Hz with dense linear position/velocity targets, table/self-collision, 32/8 solver iterations, SDF512 and authored velocity caps tightened to 98%. Authored stiffness and effort bounds are preserved. State is set only at initialization, then drives control motion. Do not substitute cubic interpolation: it overshoots between dense contact-constrained poses.

Both entries accept `--headless`, `--max-seconds 1` and `--out`; records go to a new directory beside the checkout. Physics `--wall-speed 10` changes only waiting/render cadence, while action speed stays 6x; computation may be slower than real time. A new physics execution is diagnostic and requires new checks; it does not automatically inherit this branch's acceptance.

## Repository structure

The directory layout follows main:

| Path | Content |
| --- | --- |
| `scripts/replay.py` | Shared entry; `--physics` selects physical execution |
| `scripts/download_assets.py`, `scripts/verify_bundle.py` | Hashed Release downloads and integrity verification |
| `scripts/audit_retimed_trajectory.py`, `scripts/audit_target_velocity.py` | Default actual-path limits/discrete collisions and physical target speed checks |
| `trajectories/recorded_actual_6x.npz` | Actual poses; `time` in execution seconds, `source_time` in source seconds, `q` in radians, `joint_names` in axis order |
| `trajectories/repaired_source.npz` | Dense linear physics reference in source seconds |
| `assets/`, `outputs/robot/` | Same robot, meshes and adapter as main |
| `configs/trajectory.json`, `configs/physics.json`, `configs/lineage.json` | Timing, controller settings, input hashes and provenance |
| `evidence/collision/`, `evidence/motor_limits/`, `evidence/target_motor_limits/` | Delivered-file collision, position/speed and target speed checks |
| `evidence/physics/`, `evidence/physics_collision/`, `evidence/target_collision/` | Original full-run and target/actual-pose evidence; portable-entry short check |
| `evidence/isaac_playback.json` | First-second check of portable display entry |
| [DEPLOYMENT.en.md](DEPLOYMENT.en.md) | Hardware preparation and unmet admission conditions |
| `tests/`, `requirements-analysis.txt`, `licenses/` | Offline tests, analysis dependencies and upstream licenses |

Trajectory/model binaries live in Release assets; Git contains code, configuration and necessary evidence. Raw bag, old trajectories, logs, internal instructions and a full 6x video are not included; this delivery is the trajectory.

## Verified results

| Item | Verified result |
| --- | --- |
| Source interval / physics duration | 0–289.766667 s / 48.295833 s |
| Default actual-path last knot | 48.291667 s; hold final pose for the remaining single step if needed |
| Delivered-path discrete collision | 34,741 poses × 2,717 geometry pairs, zero intersections |
| Original full-run target / actual mesh checks | 23,183 / 23,181 poses, zero intersections |
| Original full-run PhysX contacts | Zero negative-separation pairs; minimum +0.05417 mm |
| Jitter improvement | Versus directly accelerated 6x baseline: joint/wrist HF RMS down 65.43% / 56.46%; absolute acceleration P95 down 37.74% |
| Model position limits | No target or actual-record excess |
| Model speed ratios | TRON2 target/actual 34.57% / 33.77%; Wuji 93.68% / 93.63%, all pass |
| Tracking RMS / maximum | 0.023342 / 0.453704 rad over all physics steps |
| Portable physics entry | First-second 241 target/actual states bitwise equal to original full-run prefix |
| Hardware and continuous CCD | Unverified |

See [actual-path limits](evidence/motor_limits/report.json), [target speed](evidence/target_motor_limits/report.json), [collision](evidence/collision/gate.json), [physics acceptance](evidence/physics/acceptance.json), and [jitter](evidence/physics/temporal_quality.json). HF metrics use the residual of a 5 Hz order-3 zero-phase low-pass filter. Discrete collision retains same-rigid/direct-parent structural exclusions without additional assembly exemptions; positive-separation contact events still occur.

## Motion changes and limitations

1. The repaired 5x reference was smoothed at 0.6 Hz in source time, projected through contact-constrained physics, then given up to 25 mm additional separation per wrist. Against 5x actual motion, fingertip position mean/P95 changes are 21.27/34.81 mm; minimum opposite-hand fingertip-frame spacing changes from 6.47 to 27.42 mm. Maximum spacing change is 95.62 mm at source 92.30 s; joint RMS/maximum changes are 0.048654/0.801485 rad. Local gestures changed; original surface-contact/interaction preservation is unverified. See [motion changes](evidence/physics/fidelity.json).
2. Smoother does not mean zero jitter. Linear interpolation does not enforce acceleration/jerk bounds. Discrete meshes and PhysX contacts do not prove continuous CCD, exact solid clearance or hardware safety.
3. Physical `base_Link` world Z=0 and tabletop Z=−0.45 m retain the fixed **0.45 m** gap. Physical base, URDF root, wrist and flange are distinct frames. The 16-axis body mapping remains `candidate_unconfirmed`; hardware identity, SDK mapping, current/torque and thermal envelopes remain unresolved.
4. Default actual positions are bitwise equal to the accepted 6x physical trace; publication did not accelerate again or change joint values. Trajectory SHA-256: `460463385a43a8df54604d84b01ea389b60b56152366f32487a3e846410894fc`; original physical trace SHA-256: `7078b8dda74e1384efba22b2575f4067cd43b6ffee75341d88e6956b6d6db45c`. Path-normalized reports retain their source-report hashes; see [provenance](configs/lineage.json).
5. Model sources are TRON2 commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0` (Apache 2.0) and Wuji commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95` (MIT), under `licenses/`. Those licenses do not automatically cover user trajectories or the adapter.
