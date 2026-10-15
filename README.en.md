# TRON2 + Wuji: 6x trajectory preserving the 5x motion

This branch's default trajectory is replaced by a reference preserving the original 5x motion, lasting approximately **48.29 seconds**. Arms, head and endpoints are unchanged; only measured hand speed/collision windows receive small corrections. 中文：[README.md](README.md)。

## Contents

- [Install and update](#install-and-update)
- [Run in Isaac](#run-in-isaac)
- [Results and files](#results-and-files)
- [Limitations](#limitations)

## Install and update

Requires Git, authenticated GitHub CLI and Python 3.12. Display/physics also require Isaac Sim 5.1 and a compatible GPU. First use:

```bash
git clone --branch replay-6x-collision-smooth https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

For an existing clone of this branch, run from the repository root:

```bash
git pull --ff-only
python3 scripts/download_assets.py --upgrade
python3 scripts/verify_bundle.py
```

The downloader replaces hash-matched old references and removes retired managed trajectories, refusing to overwrite modified inputs. The model remains pinned to `v1.1.0`; new trajectory data comes from prerelease `6x-preserve5x-20261015`. Verification should print `Verified 87 runtime files`.

## Run in Isaac

Run from the repository root, replacing `/path/to/isaac/python.sh` with your Isaac Python entry.

1. **Display the preserved reference path** (default):

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
```

This sets reference poses without new physics. File time is already 6x execution seconds; display at 1x file time.

2. **Execute physical tracking again**:

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

Uses 240 Hz, 6x action speed, dense linear targets, 98% model velocity caps and local contact margins. New actual motion has tracking deviations and needs new checks. Both entries accept `--headless`, `--max-seconds 1` and `--out`; outputs go to a new directory beside the checkout. Physics `--wall-speed` changes only waiting/render cadence.

## Results and files

The small-deviation metrics below describe the **reference path**:

| Check | Result |
| --- | --- |
| Reference duration / samples |48.291667 s;56 axes,13,909 poses,288 Hz |
| Unchanged joint samples |98.41%; arms, head, endpoints and all cells outside repair windows unchanged |
| Maximum / RMS joint correction |3.80° /0.0775° |
| Fingertip mean / maximum deviation |0.019 /4.24 mm |
| Reference / physical actual discrete meshes |Both zero intersections; structural adjacent/same-rigid exclusions only |
| Final PhysX contact report |Zero negative-separation pairs |
| Model velocity |Reference and sampled physical actual speeds pass |

Repair windows are distributed across the motion: about 50.90% of frames contain at least one modified finger value. The 98.41% refers to joint-by-sample cells, not whole frames. No global smoothing or wrist displacement to separate the hands was applied.

| File | Purpose |
| --- | --- |
| `trajectories/trajectory_6x.npz` | Default preserved reference: `time` in execution seconds, `source_time` in source seconds, `q` in radians, `joint_names` in axis order |
| `trajectories/repaired_source.npz` | Same targets in source time for physics execution |
| `trajectories/physical_tracking_6x.npz` | Verified physical run's actual states and targets, saved separately |
| `configs/`, `release_manifest.json` | Timing, model/input hashes, provenance and download configuration |
| `evidence/reference/`, `evidence/collision/`, `evidence/physics/` | Repair scope, fidelity, collision and physical results |
| [DEPLOYMENT.en.md](DEPLOYMENT.en.md) | Offline checks and hardware preparation |

## Limitations

- **Physical actual motion still deviates**: against 5x, fingertip mean/P95/maximum differences are approximately 7.94/18.37/72.74 mm and maximum joint difference is 28.10°. Small reference changes do not establish the same physical fidelity; see [comparison](evidence/reference/fidelity_temporal.json).
- Original 5x tiny position-limit excess is retained, up to approximately 0.001635°. Linear interpolation does not enforce hardware acceleration/jerk bounds; discrete collision does not prove continuous CCD or hardware safety.
- Physical `base_Link` world Z=0 and tabletop Z=−0.45 m preserve the fixed **0.45 m** gap. Body mapping remains `candidate_unconfirmed`; device identity, SDK mapping and current/torque/thermal limits remain unresolved.
- New reference SHA-256: `841c29aefb3a3bfb2813bd59698ffe60ecdf20b712205227cb5ba05f77890253`. Repair masks are in `evidence/reference/local_masks.npz`; see [lineage](configs/lineage.json). New executions do not automatically inherit original acceptance.
- Upstream models/licenses are identified in `licenses/` and `configs/origin.json`; model licenses do not automatically cover user trajectories or the adapter.
