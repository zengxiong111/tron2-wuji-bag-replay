# TRON2 + Wuji: slowed 72.44-second trajectory

The current default is a slowed version of the previous 57.95-second PhysX **actual-motion trajectory**, lasting **72.4375 seconds**, equivalent to 4× motion speed relative to the bag. Positions are unchanged. Isaac plays the recorded path by default; **there is no new physics execution or hardware command**. Chinese: [README.md](README.md).

## Contents

- [Download and upgrade](#download-and-upgrade)
- [Playback in Isaac](#playback-in-isaac)
- [Results and files](#results-and-files)
- [Verification and compatibility](#verification-and-compatibility)

## Download and upgrade

Requires Git, GitHub CLI and Python 3.12; playback requires Isaac Sim 5.1 and a compatible GPU. This private repository requires access permission.

```bash
gh auth login
gh auth setup-git
git clone https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

Existing clone: run `git pull --ff-only`, then `python3 scripts/download_assets.py --upgrade`, then verification. Upgrade removes only hash-matching old managed files and refuses to overwrite modified inputs. Old trajectories, old verification data and old videos are removed from the current runtime; original inputs remain locally outside the repository.

The new trajectory is in [v1.2.0 Release](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/v1.2.0). The model is unchanged; the downloader reuses the hash-identical model asset from v1.1.0 according to the manifest. All runtime inputs are manifest-covered. The current bundle does not distribute the source bag or old trajectory copies.

## Playback in Isaac

```bash
ISAAC_PYTHON=/path/to/isaac/python.sh
python3 scripts/replay.py --isaac-python "$ISAAC_PYTHON"
```

Replace `/path/to/isaac/python.sh` with the local Isaac Python entry. **Time is already slowed: play at 1× file time, without another 4× multiplier.** The player uses piecewise-linear interpolation and position replay, displaying the complete path without new dynamics integration. Run records go beside the repository in `tron2-wuji-bag-replay-runs/`. Add `--headless` to hide the window; `--max-seconds 1` is a short check only.

## Results and files

| Check | Conclusion |
| --- | --- |
| TRON2 / Wuji model speeds | Peak ratios 34.86% / 80.72%, no over-speed axes |
| Collisions and penetration | 45,194 poses, 2717 geometry pairs per pose; no intersections in the strict discrete audit, without assembly exemptions |
| Angles | 3 axes have minor overshoot, maximum 0.001635°; strict angle admission failed |
| All physical motor limits | Unverified: hardware revisions, mappings, per-axis ratings, current/torque and temperatures remain pending |

| File | Purpose |
| --- | --- |
| `trajectories/retimed_actual_4x.npz` | The only current default trajectory: 13,909 frames, 56 axes, 192 Hz native samples |
| [REPORT.md](REPORT.md), [REPORT.en.md](REPORT.en.md) | Simple verification reports and pre-hardware deployment steps |
| [HARDWARE.md](HARDWARE.md), [HARDWARE.en.md](HARDWARE.en.md) | Parameter/mapping evidence tables, with unknown fields explicitly blank |
| `evidence/` | Per-axis results, collision coverage and full Isaac path playback verification |
| `configs/trajectory.json`, `configs/retiming.json` | Current bindings and retiming lineage |
| `outputs/robot/`, `assets/` | Current USD, portable URDF and required meshes/adapter |

Trajectory SHA-256: `23b555d6a4bf6df33fa4da4e5905a10f9fe375f6447784edd44079e412bdfcd1`. `time` is execution seconds, `source_time` is original bag seconds, and `q` is radians. `q_velocity` is a numerical gradient not used by the player; speed verification uses the actual interval position slopes.

## Verification and compatibility

```bash
python3.12 -m venv .venv-analysis
.venv-analysis/bin/pip install -r requirements-analysis.txt
.venv-analysis/bin/python scripts/audit_retimed_trajectory.py --out ../tron2-retimed-check
python3 scripts/check_hardware_admission.py
```

Output must be a new directory outside the repository. The trajectory checker currently returns 2 because of minor angle violations. Hardware admission also returns 2 because physical-device evidence is missing. Neither is an all-pass result.

Old `--mode` and `--max-source-seconds` options were removed; use `--max-seconds` for execution time. Previous physical-target replay is distinct from the current recorded-path playback. Passing model speeds and discrete intersections does not prove continuous CCD, new physics execution or hardware safety.

Physical base `base_Link` has world Z=0 and tabletop Z=-0.45 m, maintaining the fixed 0.45 m gap. Model sources are TRON2 commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0` (Apache 2.0) and Wuji commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95` (MIT); licenses are in `licenses/`. These licenses do not automatically cover the user trajectory or adapter.
