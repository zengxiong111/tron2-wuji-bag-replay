# TRON2 + Wuji: original trajectories, 5x replay and repair

This independent private runtime bundle reads TRON2 and dual Wuji Hand2 joint feedback from an original ROS1 bag, replays it in Isaac Sim at 5x action speed, and repairs collisions and jitter. Chinese: [README.md](README.md).

## Contents

- [Download and play](#download-and-play)
- [Where the files are](#where-the-files-are)
- [Repair workflow](#repair-workflow)
- [Results and caveats](#results-and-caveats)

## Download and play

Prerequisites: Git, GitHub CLI and Python 3.12. Simulation also requires an existing Isaac Sim 5.1 installation and compatible GPU. Downloading private files requires a GitHub account with access to this repository.

1. Clone the code and download models, bag, trajectories and video:

```bash
gh auth login
gh auth setup-git
git clone https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

The downloader verifies SHA-256 before installation and refuses to overwrite modified inputs. Large files are in the same repository's [v1.0.0 Release](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/v1.0.0).

2. Watch the complete video: open `media/head_forward_full_5x.mp4`. Duration 57.97s,30 fps, with the head facing forward.

3. Play the final repaired trajectory in an Isaac window:

```bash
ISAAC_PYTHON=/your/isaac/installation/python.sh
python3 scripts/replay.py --isaac-python "$ISAAC_PYTHON"
```

Replace `ISAAC_PYTHON` with your machine's Isaac Python entry point. The default uses the repaired reference at 5x action speed with 240 Hz physics, saving new results beside the repository in `tron2-wuji-bag-replay-runs/`.

| What to view | Additional argument |
| --- | --- |
| Original targets,1x speed | `--mode original` |
| Original targets,5x speed | `--mode accelerated` |
| Repaired targets,5x speed, retain source head motion | `--mode repaired` |
| Repaired targets,5x speed, head facing forward (default) | `--mode head-forward` |

These modes share the repaired collision model. Unrepaired targets are not guaranteed collision-free.5x refers to faster simulated motion, not postproduction video acceleration or a guarantee of real-time5x computation.

## Where the files are

| File | Meaning |
| --- | --- |
| `data/2026-10-08-17-54-13.bag` | Original real-robot ROS1 recording,289.797s |
| `trajectories/original.npz` | Original feedback resampled to8694 frames,56 joints |
| `trajectories/accelerated_5x.npz` | Identical positions with an explicit5x time axis, unrepaired |
| `trajectories/repaired_source.npz` / `repaired_5x.npz` | Clearance- and filter-repaired references in source/5x time |
| `trajectories/head_forward_source.npz` / `head_forward_5x.npz` | Final repaired reference with head yaw/pitch fixed at zero |
| `outputs/robot/tron2_wuji_hand2.usd` | Original canonical USD with the adapter assembled |
| `outputs/robot/robot_collision_fixed.usd` | SDF512 collision-repaired model for physical replay |
| `outputs/robot/tron2_wuji_hand2.urdf`, `assets/` | Portable URDF, required meshes and adapter STP/STL |
| `evidence/final/validation/tracking.npz` | Actual joint states and targets from the complete final physical replay |
| `evidence/`, `configs/` | Collision, smoothness, provenance and hash records |

The original recording is a runtime input explicitly requested by the user. The canonical and collision models serve source geometry and physical replay respectively; they are not backups. Old debug outputs are excluded.

In NPZ files, `q` is in radians and columns are named by `joint_names`. For explicit5x files, `time` is simulation seconds, `source_time` is original seconds and `q_velocity` is target rad/s. These are targets, not actual physical states. The launcher uses source-time references and applies5x in the controller to avoid double acceleration.

## Repair workflow

ROS1 installation is unnecessary; `rosbags` reads the bag directly. Keep analysis/repair and Isaac environments separate to avoid replacing Isaac's built-in USD libraries.

```bash
python3.12 -m venv .venv-analysis
.venv-analysis/bin/pip install -r requirements-analysis.txt
.venv-analysis/bin/python scripts/repair_pipeline.py
```

The flow is: **original bag → 30 Hz named feedback →continuous wrist separation/IK →low-pass filtering →clearance repair again →5x export →head facing forward**.

- Model: apply collision APIs to actual Mesh prims, enable self-collision and use SDF512 on the hands; adapter and visual meshes stay unchanged.
- Clearance: adjust bilateral wrist separation continuously and synchronize the arms with full SE(3) IK; safety margin 0.03 m.
- Jitter: source-time 1 Hz fourth-order zero-phase low-pass,0.005rad joint-limit margin; use C2 cubic positions and matching continuous velocities during replay.
- Fixed conditions: physical base 0.45 m above the tabletop; retain the same-rigid/direct-parent-child exclusions.

Rebuild the collision model:

```bash
.venv-analysis/bin/python scripts/116_build_collision_fixed_usd.py \
  --source outputs/robot/tron2_wuji_hand2.usd \
  --out ../tron2-wuji-bag-replay-runs/model/robot_collision_fixed.usd \
  --hand-approximation sdf --sdf-resolution 512
```

Add `--reference-only` to rebuild quickly from delivered extracted feedback. Each rebuild uses a new directory and does not overwrite delivered references. New physical runs need fresh collision checks; old acceptance cannot be inherited.

## Results and caveats

- Complete joint reference approximately 289.77s;5x simulation approximately 57.95s; complete video57.97s after frame rounding.
- All 13909 physical states and 2717 geometry pairs per step: no contact-feature penetration above 0.1 mm.
- Compared with the physical replay before smoothing: joint high-frequency jitter reduced approximately 61%, wrist high-frequency jitter 51%, and acceleration p95 by 82%.
- 40 hand joints have explicit names. The bag's16-dimensional body order remains the unconfirmed assumption “left7, right7, head yaw/pitch”.
- Filtering attenuates fast gesture details; the original bag is unchanged. Discrete collision checks do not establish CCD or hardware safety.

Check the delivered complete physical states:

```bash
.venv-analysis/bin/python scripts/115_audit_collision_fixed_replay.py \
  --input evidence/final/validation/tracking.npz \
  --out ../tron2-wuji-bag-replay-runs/audit
```

Asset sources: TRON2 [limxdynamics/tron2-robot-description](https://github.com/limxdynamics/tron2-robot-description), commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0`, Apache2.0; Wuji [wuji-technology/wuji-description](https://github.com/wuji-technology/wuji-description), commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95`, MIT. Original licenses are in `licenses/`; the user-provided bag and adapter do not thereby inherit third-party model licenses. Exact input hashes are in `release_manifest.json` and `configs/origin.json`.
