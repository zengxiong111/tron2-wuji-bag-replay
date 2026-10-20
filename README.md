# TRON2 + Wuji：手指不变、双臂开合更大的 6x 轨迹

新版保持手指目标不变，双臂张开行程约增加 **20%**，沿用 **22 段局部最高 2 倍**的臂部节奏，总时长仍为 **48.29 秒**。English: [README.en.md](README.en.md).

## 目录

- [新版改了什么](#新版改了什么)
- [下载和运行](#下载和运行)
- [文件和视频](#文件和视频)
- [验证与限制](#验证与限制)

## 新版改了什么

| 项目 | 新版行为 |
| --- | --- |
| 手指 40 轴 | 与原版 6x 目标逐点一致，关节幅度、目标速度和时间不变 |
| 双臂开合 | 相对每次合拢位置扩大张开行程；手腕间距最多增加约 4.23 cm，保持手腕朝向和合拢关键姿态 |
| 局部修复 | 132 个张开峰值中，122 个保持约 +20%；10 个在三个实测异常窗口附近减弱扩幅 |
| 臂部节奏 | 保留上一版 22 段局部最高 2 倍的时间映射，结合停顿与延后合拢协调手势 |
| 时长 | 48.291667 秒、13,909 姿态、288 Hz；头部和首尾姿态不变 |

20% 表示张开行程增量。局部 2 倍以原版 6x 臂部时间推进为基准。这里的“手指不变”指关节目标，实际物理跟踪仍会受到双臂和接触影响。

## 下载和运行

需要 Git、已安装并登录的 GitHub CLI（`gh`）和 Python 3.12。首次使用先运行 `gh auth login`。Isaac 显示与物理执行另需 Isaac Sim 5.1 和兼容 GPU。

```bash
git clone --branch codex/replay-6x-arm-fast-hand-fixed https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

预期输出 `Verified 94 runtime files`。模型固定使用 `v1.1.0`；新轨迹与视频来自预发布 [6x-arm-open20-hand-fixed-20261020](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/6x-arm-open20-hand-fixed-20261020)。

已安装本分支上一版资源时，先更新代码，再升级资源：

```bash
git pull --ff-only
python3 scripts/download_assets.py --upgrade
python3 scripts/verify_bundle.py
```

升级仅替换与上一版受管哈希匹配的文件，自行修改的文件不会被覆盖。在仓库根目录运行，将 `/path/to/isaac/python.sh` 替换为 Isaac 的 Python 入口：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

第一条显示目标轨迹，第二条重新执行物理跟踪。目标文件已使用 6x 执行秒，显示按 1x 文件时间播放；物理入口使用 240 Hz、6x 源时间推进、线性目标和 98% 模型速度上限。支持 `--headless`、`--max-seconds 1` 和 `--out`，输出目录必须在仓库外；物理参数 `--wall-speed` 只调整等待和渲染节奏。

## 文件和视频

| 路径 | 用途 |
| --- | --- |
| `trajectories/trajectory_6x.npz` | 默认目标：执行秒、源秒、关节角（弧度）和轴名 |
| `trajectories/repaired_source.npz` | 同一目标的源时间版本，供物理入口使用 |
| `trajectories/physical_tracking_6x.npz` | 最终完整物理运行的实际状态与目标 |
| `media/new_6x_full.mp4` | 新版完整目标动作视频，48.3 秒 |
| `media/old_vs_new_6x.mp4` | 完整同步对比，48.3 秒：左侧上一版 2 倍节奏、原幅度；右侧新版约 +20% 扩幅 |
| `evidence/reference/` | 扩幅、局部窗口、臂部时序、手指一致性和验收证据 |
| `evidence/reference_collision/`、`evidence/physics_collision/` | 目标与实际状态的离散网格检查 |
| `configs/lineage.json`、`release_manifest.json` | 来源绑定、当前文件哈希和下载配置 |

视频是 Isaac 目标轨迹可视化，模型、视角和帧时间一致；30 fps 共 1,449 帧。变化姿态重新渲染，完全一致姿态复用绑定画面。视频中的左侧参照是局部 2 倍臂部版；本分支此前发布的默认版本是 1.5 倍臂部版。

## 验证与限制

| 检查 | 结果 |
| --- | --- |
| 目标 / 实际网格 | 27,817 / 23,181 个原采样及中点姿态；每姿态 2,717 对，均零相交 |
| PhysX | 完整运行 48.295833 秒；零负间距对，最小报告间距 +0.05417 mm |
| 模型速度 | 目标与实际采样速度均通过；臂部目标峰值 66.37%，手指目标峰值 99.9903% |
| 物理跟踪误差（全部 240 Hz 状态） | 全机器人 RMS 0.026655 rad，最大 0.633004 rad（36.27°） |
| 实际高频 RMS（6 Hz 以上） | 相对视频左侧 2 倍臂部版：臂部约 +7.1%，手指约 +1.0%；不能据此称抖动减少 |

物理报告中的 `max_error_rad` 只在渲染节拍统计；上表和 `full_trace_tracking_error` 使用全部 240 Hz 状态，因此峰值可能更大。

物理 `base_Link` 世界 Z=0，桌面 Z=−0.45 m，高度差固定 **0.45 m**。仍有最大约 0.001067° 的微小位置越限；机体映射为 `candidate_unconfirmed`。这些结果不证明连续 CCD、真机电机能力或真机可执行性。离线检查和真机准备见 [DEPLOYMENT.md](DEPLOYMENT.md)。

当前目标 SHA-256：`c29efc597857dc3193ab68b629d1b4c6b8309eaea9bf7b44b5dc03303ec98a25`。新运行须重新验证。上游模型许可证见 `licenses/`。
