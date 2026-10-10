# TRON2 + Wuji trajectory playback

Play a recorded trajectory for TRON2 arms and Wuji hands in Isaac Sim. The current trajectory lasts **72.44 seconds**, has 56 joints, and corresponds to 4× the source bag motion speed. Chinese: [README.md](README.md).

## Contents

- [Install and download](#install-and-download)
- [Playback](#playback)
- [Files](#files)
- [Results and limitations](#results-and-limitations)

## Install and download

Requires private repository access, Git, GitHub CLI and Python 3.12. Playback also requires Isaac Sim 5.1 and a compatible GPU. Run in a terminal:

```bash
gh auth login
gh auth setup-git
git clone https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

Models and trajectories are downloaded from GitHub Releases; copying source code alone is insufficient. Successful verification prints `Verified 84 runtime files`. For an existing clone, run `git pull --ff-only`, then `python3 scripts/download_assets.py --upgrade` and the verification command. The downloader refuses to overwrite modified inputs.

## Playback

From the repository root, replace the path with your Isaac Sim `python.sh`:

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
```

Trajectory timing is already adjusted. Use default file time without another 4× multiplier. Add `--headless` to hide the window or `--max-seconds 1` to check the first second. Playback records are saved beside the repository in `tron2-wuji-bag-replay-runs/`.

Troubleshooting: check GitHub access and retry failed downloads; download again if verification reports missing files; check the Isaac Sim entry and GPU environment if the window cannot start. `python3` launches the script; the specified Isaac Python runs playback.

## Files

| Path | Purpose |
| --- | --- |
| `scripts/replay.py` | Playback entry |
| `scripts/download_assets.py`, `scripts/verify_bundle.py` | Download and integrity checks |
| `trajectories/retimed_actual_4x.npz` | Current trajectory, installed by download |
| `assets/`, `outputs/robot/` | Models, meshes and adapter, installed by download |
| `configs/`, `release_manifest.json` | Trajectory, provenance, hardware configuration and download manifest |
| `evidence/` | Current trajectory verification results |
| `tests/`, `requirements-analysis.txt` | Offline verification tools and dependencies |
| [DEPLOYMENT.en.md](DEPLOYMENT.en.md) | Preparation and checks before hardware deployment |
| `licenses/` | Upstream model licenses |

Trajectory fields: `time` is execution seconds, `source_time` is source bag seconds, and `q` is radians; `joint_names` defines joint order. There are 13,909 frames at a native sample rate of 192 Hz. Playback uses piecewise-linear position interpolation and does not use `q_velocity`.

## Results and limitations

| Item | Current result |
| --- | --- |
| Model speed | TRON2 / Wuji peaks are 34.86% / 80.72% of model limits; passed |
| Joint angles | 3 axes have minor overshoot, maximum 0.001635°; strict check failed |
| Discrete collisions | No intersections across 45,194 poses and 2717 geometry pairs per pose; this does not establish continuous collision clearance |
| Isaac playback | A complete path playback record is included; playback sets recorded positions without new dynamics execution |
| Hardware | Unverified; no hardware controller is included |

Details: [speed and angles](evidence/motor_limits/report.json), [collision check](evidence/collision/gate.json), and [Isaac playback](evidence/isaac_playback.json). See the [deployment guide](DEPLOYMENT.en.md#offline-checks) for offline verification commands.

The vertical gap from tabletop to physical-base origin is fixed at **0.45 m**: current `base_Link` world Z=0, tabletop Z=-0.45 m. Physical base, URDF root, wrist and flange origins are distinct.

Trajectory SHA-256: `23b555d6a4bf6df33fa4da4e5905a10f9fe375f6447784edd44079e412bdfcd1`. Provenance: `configs/retiming.json` and `configs/origin.json`. The current bundle excludes the original bag and old trajectories.

Model sources: TRON2 commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0` (Apache 2.0); Wuji commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95` (MIT). See `licenses/`; these licenses do not automatically cover the user trajectory or adapter.
