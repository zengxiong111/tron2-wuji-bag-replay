# TRON2 + Wuji repaired5x bag replay

This branch restores the original repaired **5x** trajectory: **57.95 seconds**,56 joints,13,909 recorded states at240Hz, head facing forward. Main's4x record is a slower timing of the same positions; this branch preserves the original5x timing. Chinese: [README.md](README.md).

## Contents

- [Install and download](#install-and-download)
- [Playback and physical execution](#playback-and-physical-execution)
- [Files](#files)
- [Results and limitations](#results-and-limitations)

## Install and download

Requires Git, authenticated GitHub CLI and Python3.12. Playback/physics requires Isaac Sim5.1 and a compatible GPU. The physical entry also requires SciPy in the same Isaac Python environment; if missing, install it with `/path/to/isaac/python.sh -m pip install scipy`. Run:

```bash
gh auth login
git clone --branch replay-5x-collision-smooth https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

Models and trajectories are installed from checksummed GitHub Release assets. Successful verification prints `Verified 86 runtime files`. The robot asset is reused from the pinnedv1.1.0 release; the5x trajectory/video asset belongs to this branch's prerelease. Existing clones must switch to this branch and run `python3 scripts/download_assets.py --upgrade`, then verify. The downloader refuses to overwrite modified inputs and removes only hash-matched retired4x managed data. Main and its release are unchanged.

## Playback and physical execution

From the repository root, replace `/path/to/isaac/python.sh` with your Isaac Sim Python entry.

1. **Replay the verified recorded path** (default):

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
```

This sets recorded actual joint positions; it does not execute dynamics again. File time is already5x source time: **do not apply another5x multiplier**. `--headless` hides the window; `--max-seconds 1` checks the first second.

2. **Execute the repaired targets again in PhysX**:

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

This integrates physics at240Hz, sends continuous C2 position/velocity targets at5x action speed, enables table/self-collision and keeps solver32/8 and SDF512. After a single initialization, joints are controlled through drives. It creates a fresh diagnostic trace, which needs fresh validation; the included full-run reports apply to the original run. `--max-seconds 1` limits simulation time, starting from source zero. `--wall-speed` changes wall pacing only; computation may be slower than real time and never multiplies action speed again. Outputs go beside the checkout in `<checkout-name>-runs/`; no hardware I/O is included.

[Full video:57.97seconds,30fps](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/download/5x-collision-smooth-20261014/head_forward_full_5x.mp4). After asset download, the local file is `media/head_forward_full_5x.mp4`. The video renders the recorded5x PhysX states at fixed frame rate; it is not a1x recording sped up afterward. Raw bag duration289.797s, synchronized reference289.767s and frame rounding explain the small timing differences.

## Files

| Path | Purpose |
| --- | --- |
| `scripts/replay.py` | Shared entry; `--physics` selects new dynamics execution |
| `scripts/download_assets.py`, `scripts/verify_bundle.py` | Asset download/integrity checks |
| `trajectories/recorded_actual_5x.npz` | Default actual-state path; `time`=execution seconds, `source_time`=bag seconds, `q`=radians, `joint_names`=axis order |
| `trajectories/repaired_source.npz` | Smoothed/clearance-repaired targets in source seconds, head yaw/pitch targets0 |
| `assets/`, `outputs/robot/` | Same robot, meshes and adapter as main |
| `configs/trajectory.json`, `configs/physics.json`, `configs/lineage.json` | Timing, model/input hashes and provenance |
| `evidence/collision/`, `evidence/motor_limits/`, `evidence/target_motor_limits/` | Fresh checks of the published actual path and target-speed curve |
| `evidence/physics/` | Original full5x physical-run, contact and smoothness evidence; separately labeled portable-entry smoke test |
| `evidence/isaac_playback.json` | First-second portable recorded-path entry test |
| [DEPLOYMENT.en.md](DEPLOYMENT.en.md) | Hardware preparation and blocked admission |
| `tests/`, `requirements-analysis.txt`, `licenses/` | Offline tests/dependencies and upstream licenses |

The actual path uses linear position interpolation and omits unused velocity arrays. The physical reference uses source-time C2 interpolation and analytic velocity feedforward. Default actual positions are bit-identical to the original recorded run; only file packaging/time units are normalized. Original raw bag, baseline traces, logs, internal instructions and historical assets are excluded.

## Results and limitations

| Item | Current result |
| --- | --- |
| Recorded-path discrete collisions | Zero intersections in41,692 poses ×2,717 geometry pairs, using structural exclusions only |
| Original physical-run collisions | All13,909 recorded states passed the0.1mm threshold; minimum PhysX contact separation+0.016693mm |
| Original-run jitter | Joint/wrist high-frequency RMS reduced61.21% /51.24%; acceleration absolute p95 reduced82.37% against the preceding collision-only run |
| Model speed of published actual path | TRON2 peak43.57% of limit; Wuji100.90%,2 axes slightly exceed; failed |
| Model speed of physical targets |8 Wuji joints exceed limits, worst15.55%; failed |
| Joint angles |3 axes slightly exceed limits, maximum0.001635°; strict check failed |
| Hardware / continuous CCD | Unverified; no hardware authorization |

Evidence: [actual limits](evidence/motor_limits/report.json), [target speed](evidence/target_motor_limits/report.json), [discrete collision](evidence/collision/gate.json), [original physics](evidence/physics/report.json), [smoothness](evidence/physics/temporal_quality.json). Improved smoothness does not mean zero jitter; linear recorded-path interpolation is not an acceleration/jerk guarantee. Discrete samples do not certify continuous CCD.

Physical-base/tabletop gap remains **0.45m**: `base_Link` worldZ=0, table topZ=-0.45. Physical base, URDF root, wrist and flange are distinct. Body mapping remains `candidate_unconfirmed`; hardware identity, SDK mapping, current/torque and thermal limits are unresolved.

Actual trajectory SHA-256: `4e086ddf442ddded028b0af4824c519de4a004ba421b9df4662c11c8e498fc06`. Source tracking SHA-256: `8634e73e49bbc0b769ea86c21ef918df92962da55e43305784c7005ffc6b505c`. [Lineage](configs/lineage.json) retains source hashes and array identity. Normalized report paths are identified explicitly. New physics execution may produce different results and does not inherit acceptance automatically.

Model sources remain TRON2 commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0` (Apache2.0) and Wuji commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95` (MIT); see `licenses/`. These licenses do not automatically cover the user trajectory or adapter.
