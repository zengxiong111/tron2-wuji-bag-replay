# TRON2 + Wuji 5× Trajectory Playback

This repository provides a TRON2 dual-arm and Wuji dual-hand trajectory that passed acceptance for a specified Isaac Sim / PhysX run, along with its runtime models and playback tools. Here, 5× means execution speed relative to the source bag. The default playback already uses the 5× execution timeline; **do not multiply it by 5 again**. Chinese: [README.md](README.md).

## Contents

- [Install and download](#install-and-download)
- [Play the trajectory](#play-the-trajectory)
- [Results and boundaries](#results-and-boundaries)

## Install and download

You need access to the private GitHub repository, Git, GitHub CLI, and Python 3. Playback also requires Isaac Sim 5.1 and a compatible GPU. Run these commands in a terminal:

```bash
gh auth login
gh auth setup-git
git clone https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

The trajectory and models are downloaded from a GitHub Release; cloning source code alone is insufficient. The current release tag is `bag-20261027-5x-validated-20261028`. The downloader checks archive and file hashes and refuses to overwrite locally modified files. For an existing clone, run `git pull --ff-only`, then, if needed, run `python3 scripts/download_assets.py --upgrade` and verify the bundle again.

## Play the trajectory

Launch from the repository root. Replace the path with your local Isaac Sim Python interpreter:

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python
```

Default mode plays the recorded path bound to the accepted PhysX actual-state trace; it does not step physics again. Time is measured in execution seconds, so do not apply another 5× multiplier. Logs and results are written outside the repository under `../tron2-wuji-bag-replay-runs/<timestamp>/`. Add `--headless` to hide the window or `--max-seconds 1` to play only the first second.

You can also watch the [accepted actual-state replay video](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/download/bag-20261027-5x-validated-20261028/actual_replay_5x.mp4). The video visualizes saved PhysX states; watching it does not run physics again.

To start a new physics replay, explicitly add `--physics`. It creates new diagnostic results and does not replace the accepted trajectory or establish acceptance for that new run. See [Physics replay](DEPLOYMENT.en.md) for prerequisites, the full command, and checks.

## Results and boundaries

| Item | Verified result |
| --- | --- |
| Time coverage | The source bag is 347.212 s; body feedback is missing for about the first 2.494 s. The valid 344.718 s source interval maps to a 68.944 s execution trajectory at 5×. |
| Dense geometry checks | The source trajectory has 186,407 samples, the execution export 325,400, and the PhysX actual-state trace 66,187. Each was checked against 2,707 retained geometry pairs with no collision events. |
| Actual tracking and motion | Actual wrist gap versus raw C2 is 1.891 mm maximum / 0.927 mm RMS; peak position finite-difference speed is 0.993 of original URDF limits; left thumb CMC flex range fraction / correlation is 99.4% / 0.9965. |
| High-frequency motion | Hand RMS above 30 Hz is 0.411 of the same-phase raw C2 baseline. |
| Aggregate acceptance | All 52 working thresholds in the acceptance record passed; see `validation/acceptance_155.json`. |

The geometry population retains 2,632 robot pairs and 75 table pairs. The official Wuji Beta2 assembly policy excludes 20 internal body pairs; 10 are already among 133 structural exclusions, for 143 unique exclusions overall. The tabletop-to-physical-base-origin gap is fixed at **0.45 m**.

These results describe discrete sampling checks for the specified model and Isaac / PhysX run. They do not certify continuous collision detection (CCD) or real hardware safety. The offline Pinocchio RNEA effort proxy reaches 1.251 of the URDF effort limit and did not pass that limit; do not claim torque acceptance. The original body mapping remains unconfirmed, and hardware admission and Sim2Real are unverified. See `validation/`, `metadata/`, and [Physics replay](DEPLOYMENT.en.md) for detailed evidence and definitions.
