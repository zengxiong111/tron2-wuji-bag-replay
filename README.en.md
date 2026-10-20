# TRON2 + Wuji: wider arm opening with unchanged fingers at 6x

Finger targets are unchanged. Arm opening strokes increase by approximately **20%**, using the same **22 phases with local arm rate up to 2x**. Total duration remains **48.29 seconds**. 中文：[README.md](README.md).

## Contents

- [What changed](#what-changed)
- [Download and run](#download-and-run)
- [Files and videos](#files-and-videos)
- [Verification and limits](#verification-and-limits)

## What changed

| Item | Current behavior |
| --- | --- |
| 40 finger axes | Pointwise identical to the original 6x targets; joint amplitude, target speed and timing unchanged |
| Arm opening | Expand travel from each closed position; wrist separation increases by up to about 4.23 cm, preserving wrist orientation and closing keyframes |
| Local repair | Of 132 opening peaks, 122 retain about +20%; 10 reduce expansion near three measured problem windows |
| Arm timing | Preserve the previous 22 phases with local rate up to 2x, using pauses and delayed closing to coordinate gestures |
| Duration | 48.291667 s, 13,909 poses, 288 Hz; head and endpoint poses unchanged |

20% measures opening travel gain. Local 2x uses the original 6x arm clock as its baseline. Unchanged fingers refers to joint targets; actual physical tracking can still be affected by arm motion and contacts.

## Download and run

Requires Git, installed and authenticated GitHub CLI (`gh`), and Python 3.12. First-time users should run `gh auth login`. Isaac display and physical execution also require Isaac Sim 5.1 and a compatible GPU.

```bash
git clone --branch codex/replay-6x-arm-fast-hand-fixed https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

Expect `Verified 94 runtime files`. Models stay pinned to `v1.1.0`; new trajectories and videos use prerelease [6x-arm-open20-hand-fixed-20261020](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/6x-arm-open20-hand-fixed-20261020).

If the previous assets from this branch are installed, update code and then upgrade assets:

```bash
git pull --ff-only
python3 scripts/download_assets.py --upgrade
python3 scripts/verify_bundle.py
```

Upgrade replaces only files matching the previous managed hashes; modified files are not overwritten. Run from the repository root, replacing `/path/to/isaac/python.sh` with the Isaac Python entry point:

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

The first command displays reference targets; the second reruns physical tracking. Target file time already uses 6x execution seconds, so display uses file time at 1x. Physics uses 240 Hz, 6x source-time progression, linear targets and 98% model velocity caps. Both support `--headless`, `--max-seconds 1` and `--out`; output directories must be outside the repository. Physical `--wall-speed` changes waiting/render pacing only.

## Files and videos

| Path | Purpose |
| --- | --- |
| `trajectories/trajectory_6x.npz` | Default targets: execution seconds, source seconds, joint angles in radians and joint names |
| `trajectories/repaired_source.npz` | Same targets in source time, used by the physics entry point |
| `trajectories/physical_tracking_6x.npz` | Actual states and targets from the final full physical run |
| `media/new_6x_full.mp4` | Full new reference video, 48.3 s |
| `media/old_vs_new_6x.mp4` | Full synchronized comparison, 48.3 s: previous 2x arm timing with original amplitude left; new approximately +20% expansion right |
| `evidence/reference/` | Expansion, local windows, arm timing, finger identity and acceptance evidence |
| `evidence/reference_collision/`, `evidence/physics_collision/` | Discrete target and actual-state mesh checks |
| `configs/lineage.json`, `release_manifest.json` | Lineage bindings, current hashes and download configuration |

Videos visualize Isaac reference targets with matched model, view and frame times: 1,449 frames at 30 fps. Changed poses are freshly rendered; exactly identical poses reuse bound frames. The video left panel uses the local-2x arm reference; this branch previously published a local-1.5x default.

## Verification and limits

| Check | Result |
| --- | --- |
| Target / actual mesh | 27,817 / 23,181 native and midpoint poses; 2,717 pairs each; zero intersections |
| PhysX | Full 48.295833 s; zero negative-separation pairs, minimum reported separation +0.05417 mm |
| Model velocity | Targets and actual sampled speeds pass; arm target peak 66.37%, finger target peak 99.9903% |
| Physical tracking error (all 240 Hz states) | Whole-robot RMS 0.026655 rad, maximum 0.633004 rad (36.27 degrees) |
| Actual high-frequency RMS above 6 Hz | Relative to the video left-panel 2x arm version: arms about +7.1%, fingers about +1.0%; does not establish jitter reduction |

The physical report’s `max_error_rad` samples render cadence only. The table and `full_trace_tracking_error` use all 240 Hz states, so their maximum can be larger.

Physical `base_Link` world Z=0, tabletop Z=−0.45 m: fixed gap **0.45 m**. A tiny position excess up to about 0.001067 degrees remains; body mapping is `candidate_unconfirmed`. These results do not prove continuous CCD, hardware motor capability or hardware readiness. See [DEPLOYMENT.en.md](DEPLOYMENT.en.md) for offline checks and hardware preparation.

Current target SHA-256: `c29efc597857dc3193ab68b629d1b4c6b8309eaea9bf7b44b5dc03303ec98a25`. New runs require new validation. Upstream model licenses are in `licenses/`.
