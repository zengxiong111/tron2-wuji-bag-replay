# TRON2 + Wuji: four-times Isaac trajectory replay

The current default is **four-times action speed, head forward, approximately 72.44 seconds**. Model speed and target-angle checks pass; the complete hardware motor envelope remains unverified. Read the [plain-language report](REPORT.en.md). Chinese: [README.md](README.md).

## Contents

- [Download and replay](#download-and-replay)
- [Files](#files)
- [Repair and recheck](#repair-and-recheck)
- [Limitations and provenance](#limitations-and-provenance)

## Download and replay

Requires Git, GitHub CLI and Python 3.12; simulation requires installed Isaac Sim 5.1 and a compatible GPU. Private files require repository access.

1. Download code and runtime inputs:

```bash
gh auth login
gh auth setup-git
git clone https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

Existing clones: run `git pull --ff-only`, then `python3 scripts/download_assets.py --upgrade` and the verifier. Upgrades replace managed files only when their hashes match the previous delivery; modified inputs are not overwritten. Retired five-times files and the old video are removed only by known hash.

Binary runtime inputs are in the [v1.1.0 Release](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/v1.1.0). Code and plain-language reports are stored in Git. No new video is included; the old five-times video does not represent this version.

2. Execute in the Isaac window:

```bash
ISAAC_PYTHON=/path/to/isaac/python.sh
python3 scripts/replay.py --isaac-python "$ISAAC_PYTHON"
```

Replace `/path/to/isaac/python.sh` with your local Isaac Python entry. Source-time references are executed at four-times action speed with a 240 Hz physics step; this is not video acceleration. Wall-clock runtime depends on computer performance. New records go beside the repository in `tron2-wuji-bag-replay-runs/`.

| Mode | Meaning |
| --- | --- |
| Default / `--mode head-forward` | Repaired four-times trajectory, head forward |
| `--mode repaired` | Compatibility alias for the default, using the same trajectory |
| `--mode original` | Original targets at original speed, unrepaired |
| `--mode accelerated` | Original targets at four-times speed, unrepaired |

## Files

| File | Purpose |
| --- | --- |
| `data/2026-10-08-17-54-13.bag` | Original hardware ROS1 input, 289.797 seconds |
| `trajectories/original.npz` | Original feedback reference at 30 Hz, 8694 frames and 56 axes |
| `trajectories/head_forward_source.npz` | Current repaired reference, source time 0–289.7667 seconds |
| `trajectories/head_forward_4x.npz` | Same targets exported on a four-times timeline with velocities, approximately 72.4417 seconds |
| `evidence/final/validation/tracking.npz` | Complete four-times physical replay targets and actual states |
| `evidence/comparison/` | Original four-times recording and zero-head targets required for this jitter comparison |
| `outputs/robot/`, `assets/` | Canonical USD, SDF512 collision USD, portable URDF, required meshes and adapter |
| `REPORT.md`, `REPORT.en.md` | Plain-language inspection reports |
| `evidence/`, `configs/`, `release_manifest.json` | Reproducible numbers, settings and SHA-256 bindings |

`q` is in rad; `joint_names` defines columns. In the four-times export, `time` is simulation seconds, `source_time` is original seconds, and `q_velocity` is rad/s. The launcher uses the source-time file to avoid accelerating an already compressed timeline again. Targets differ from actual physical states.

## Repair and recheck

Keep analysis and Isaac environments separate. ROS1 installation is unnecessary.

```bash
python3.12 -m venv .venv-analysis
.venv-analysis/bin/pip install -r requirements-analysis.txt
.venv-analysis/bin/python scripts/repair_pipeline.py
.venv-analysis/bin/python scripts/check_motion_limits.py --out ../tron2-4x-checks
.venv-analysis/bin/python scripts/115_audit_collision_fixed_replay.py \
  --input evidence/final/validation/tracking.npz --out ../tron2-4x-collision
```

Pipeline: original bag → named 30 Hz feedback → fourth-order zero-phase 1 Hz source-time filtering and head-forward targets → continuous wrist clearance/SE(3) IK with 0.04 m margin → C2 interpolation and four-times export → full physical checks. Use `--reference-only` to reuse extracted feedback. Rebuilds create new directories; new physical runs need fresh admission.

## Limitations and provenance

- Discrete collision checks pass for all 17,386 actual states; joint high-frequency jitter falls 78.98%. Full results and unverified items are in the [report](REPORT.en.md). Intrahand reference targets still overlap and cannot be used for collision-free kinematic playback; physical contacts introduce tracking deviations.
- Body axis order remains unconfirmed; both head targets are zero. Hardware current, torque, temperature, load and continuous CCD are unverified. Filtering attenuates some fast gesture details. The original bag is retained unchanged.
- The physical base is `base_Link`, Z=0; tabletop Z=-0.45 m, a fixed 0.45 m gap. Mounting height must not change to pass checks. Canonical and collision USDs share visual and mounting geometry.
- Models: [TRON2](https://github.com/limxdynamics/tron2-robot-description) commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0` (Apache 2.0), [Wuji](https://github.com/wuji-technology/wuji-description) commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95` (MIT). Licenses are in `licenses/`; they do not automatically apply to the user's bag or adapter. Input identities are in `configs/origin.json` and `release_manifest.json`.
