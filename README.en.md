# TRON2 + Wuji: 6x trajectory with finger clearance

This version repairs finger contact while retaining the previous arms, head and clocks, with a **48.29 s** duration. 中文：[README.md](README.md). The legacy branch name remains; finger targets have been locally modified.

## Contents

- [Current motion](#current-motion)
- [Download and run](#download-and-run)
- [Files and video](#files-and-video)
- [Verification and limits](#verification-and-limits)

## Current motion

| Item | Current result |
| --- | --- |
| Arms, head and clocks | Pointwise numerical identity to `27ff86b`; retains approximately +20% opening, three local expansion exceptions and 22 phases with local peak 2x timing |
| Fingers | Locally adjusted for clearance; joint changes RMS 0.523°, maximum 8.276° |
| Relationship between hands | Wrist targets unchanged; fingertip relative vector changes RMS 2.579 mm, maximum 20.660 mm; gestures are not identical |
| Duration and sampling | 48.291667 s; 13,909 poses at 288 Hz |
| Physics configuration | Model, drives, collision settings, 240 Hz, 6x clock, linear targets and 98% model velocity limit retained |

## Download and run

Requires Git, authenticated GitHub CLI (`gh auth login`) and Python 3. Isaac entry points also require Isaac Sim 5.1 and a compatible GPU.

```bash
git clone --branch codex/replay-6x-arm-fast-hand-fixed https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

Expect `Verified 93 runtime files`, followed by acceptance binding confirmation. Models remain pinned to `v1.1.0`; trajectory and video come from [6x-no-finger-contact-20261020](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/6x-no-finger-contact-20261020).

For an installed immediately previous branch version, run from the repository root:

```bash
git pull --ff-only
python3 scripts/download_assets.py --upgrade
python3 scripts/verify_bundle.py
```

Upgrade replaces only managed files matching previous hashes, refuses modified files, and removes matching obsolete comparison video and retired evidence. Use a fresh clone for older versions.

Replace `/path/to/isaac/python.sh` with the Isaac Python entry point:

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

The first displays targets; the second executes fresh physics tracking. Target files already use 6x execution seconds and display at 1x file time; physics advances source time at 6x. Supports `--headless`, `--max-seconds 1` and `--out`; output directories must be outside the repository. Physics `--wall-speed` only changes waiting and render pacing.

## Files and video

| Path | Purpose |
| --- | --- |
| `trajectories/trajectory_6x.npz` | Default repaired target: execution seconds, source seconds, radians and joint names |
| `trajectories/repaired_source.npz` | Same target on source time for physics |
| `trajectories/physical_tracking_6x.npz` | Accepted full 240 Hz actual states and targets |
| `media/new_6x_full.mp4` | Isaac rendering of those actual states, 48.3 s, 30 fps, 1,449 frames |
| `evidence/reference/` | Acceptance, changes, provenance and model equivalence |
| `evidence/reference_collision/`, `evidence/physics_collision/` | Whole-robot mesh checks for target and actual states |
| `evidence/finger_*`, `evidence/physics/` | Finger clearance, linear path certificates and physical contact reports |
| `configs/lineage.json`, `release_manifest.json` | Source commit, file hashes and download configuration |

Video uses actual states executed at 6x in physics; capture redraws those states without rerunning physics or post-production acceleration. Default display shows targets and therefore differs from the actual-state video by tracking error. The obsolete comparison video does not represent this motion and was removed.

JSON evidence is reformatted and its path labels relocated; `evidence/reference/import_binding.json` records accepted source and published byte hashes, and the verifier checks the published hashes.

## Verification and limits

| Check | Accepted result |
| --- | --- |
| Finger collision response | Full 48.295833 s run including initialization: zero pairs/points with nonzero impulse |
| Predictive proximity reports | 3,602 positive-separation, zero-impulse points remain; this does not mean no contact callbacks |
| Sampled finger mesh clearance | Target minimum 2.334 mm; actual minimum 0.363 mm |
| Whole-robot meshes | Target 27,817 / actual 23,181 native and midpoint poses, 2,717 pairs per pose, zero intersections |
| Linear path clearance certificates | Target and linear interpolation of recorded actual states pass; conservative interval lower bound approximately 0.010 mm |
| Model speed | Finger target peak 99.9900%, arm target 66.3724%, actual finite-difference peak 98.6036% |
| Full 240 Hz tracking error | RMS 0.026866 rad, maximum 0.632980 rad |
| Actual finger high-frequency RMS | Above 6 Hz, approximately −2.00% versus the previous version; only indicates no increase in this metric |

Physical `base_Link` world Z=0; tabletop Z=−0.45 m; the gap stays **0.45 m**. The inherited left wrist position excess of approximately **0.001067°** remains; body ordering is still `candidate_unconfirmed`. Report `max_error_rad` uses render cadence; see `full_trace_tracking_error` for full-state metrics.

Linear path certificates do not cover unknown true within-step PhysX curves or prove convex collision shape equivalence, hardware CCD or motor safety. There is no hardware control entry point. See [DEPLOYMENT.en.md](DEPLOYMENT.en.md) for offline checks and hardware preparation. Model licenses are in `licenses/`.

Current target SHA-256: `bedc266d4339cd038e35dcd0ec555cb41696a004fb018c8437573cc94a728246`. Acceptance binds only the listed files; new runs require fresh checks.
