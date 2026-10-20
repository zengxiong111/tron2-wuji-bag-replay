# TRON2 + Wuji: 6x with unchanged fingers and quicker arms

This branch changes only arm timing relative to the previous 6x: **finger targets stay unchanged, 22 opening/closing phases move faster with short pauses, and duration remains 48.29 seconds**. 中文：[README.md](README.md).

## Contents

- [What changed](#what-changed)
- [Download and run](#download-and-run)
- [Files and videos](#files-and-videos)
- [Verification and limitations](#verification-and-limitations)

## What changed

| Item | New behavior |
| --- | --- |
| 40 finger axes | Identical to the previous 6x at every sample; target speed, amplitude and time unchanged |
| 14 arm axes | Original pose path; 22 larger opening/closing phases move faster locally; other phases keep their timing |
| Acceleration and pauses | Local path rate up to 1.5x; average 1.103x while moving; pauses near phase endpoints total 0.781 s |
| Timing and head | 48.291667 s, 13,909 poses, 288 Hz; head and endpoint poses unchanged |
| Collision fallback | A phase around 35.1 s retains original timing to avoid interhand intersections from early motion |

“Unchanged fingers” refers to target joint trajectories. During physical tracking, arms and contacts can still affect actual finger motion. The parent is `2d0d503` on `replay-6x-collision-smooth`; that branch and `main` are preserved.

## Download and run

Requires Git, authenticated GitHub CLI (`gh`) and Python 3.12; display and physics additionally require Isaac Sim 5.1 and a compatible GPU.

```bash
git clone --branch codex/replay-6x-arm-fast-hand-fixed https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

Expect `Verified 89 runtime files`. Models remain pinned to `v1.1.0`; trajectory and videos use prerelease `6x-arm-fast-hand-fixed-20261020`. When switching from the old version with assets already installed, run `python3 scripts/download_assets.py --upgrade` to replace only hash-matched managed inputs; modified files are not overwritten.

Run at repository root, replacing `/path/to/isaac/python.sh` with the Isaac Python entry point. The first line displays targets; the second reruns physical tracking:

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

The target file already uses 6x execution seconds and displays at 1x file time. Physics uses 240 Hz, 6x source-time progression, linear targets and 98% model velocity caps. Both entries support `--headless`, `--max-seconds 1` and `--out`; outputs must be outside the repository. Physical `--wall-speed` changes waiting/rendering pace only.

## Files and videos

| Path | Purpose |
| --- | --- |
| `trajectories/trajectory_6x.npz` | Default targets: execution `time`, `source_time`, `q` in radians and `joint_names` |
| `trajectories/repaired_source.npz` | Same targets in source seconds for physics |
| `trajectories/physical_tracking_6x.npz` | Actual states and targets from this verified physical run |
| `media/new_6x_full.mp4` | Full new target-motion video, 48.3 s |
| `media/old_vs_new_6x.mp4` | Full synchronized comparison, old left/new right, 48.3 s |
| `evidence/reference/` | Arm clock, finger identity, temporal metrics and acceptance |
| `evidence/reference_collision/`, `evidence/physics_collision/` | Discrete mesh checks of targets and actual physical states |
| `configs/lineage.json`, `release_manifest.json` | Provenance, current file hashes and download configuration |

Videos visualize Isaac targets with the same camera and frame times: 250 changed-pose frames freshly rendered, 1,199 identical-pose frames reused from the old video. They are not actual physical-run recordings. Both contain 1,449 frames at 30 fps and cover the full action timeline to frame-rounding precision. The [release page](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/6x-arm-fast-hand-fixed-20261020) provides trajectory and video archives.

## Verification and limitations

| Check | Result |
| --- | --- |
| Target / actual meshes | 27,817 / 23,181 native and midpoint poses; 2,717 pairs per pose, zero intersections |
| PhysX | Full 48.295833 s; zero negative-separation pairs, minimum reported separation +0.05417 mm |
| Model velocity | Targets and sampled actual velocities pass; finger target peak is close to the model cap (99.9903%) |
| Physical tracking error | Whole-robot RMS 0.025509 rad, maximum 0.597680 rad (34.24°) |
| Actual high-frequency RMS above 6 Hz | Relative to old 6x: arms +24.0%, fingers +1.5%; does not establish further jitter reduction |

Mount geometry is unchanged: physical `base_Link` world Z=0, tabletop Z=−0.45 m, fixed gap **0.45 m**. A tiny position excess up to about 0.001473° remains; body mapping is still `candidate_unconfirmed`. Discrete collisions, model speed and sampled physics do not prove continuous CCD, hardware motor capability or hardware execution readiness. See [DEPLOYMENT.en.md](DEPLOYMENT.en.md) for checks and preparation.

Current target SHA-256: `5467cff16fcf08f2aba6fada0bce74d914ba8bc69e141a99e3c08531915db2e2`. New runs require new validation. Upstream model licenses are in `licenses/`.
