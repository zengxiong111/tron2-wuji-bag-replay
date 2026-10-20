# TRON2 + Wuji：手指不变、双臂加快的 6x 轨迹

本分支在旧版 6x 基础上，仅调整双臂节奏：**手指目标不变，22 个开合段加快并短暂停顿，总时长仍为 48.29 秒**。English: [README.en.md](README.en.md).

## 目录

- [新版改了什么](#新版改了什么)
- [下载和运行](#下载和运行)
- [文件和视频](#文件和视频)
- [验证与限制](#验证与限制)

## 新版改了什么

| 项目 | 新版行为 |
| --- | --- |
| 手指 40 轴 | 与旧版 6x 逐点一致，目标速度、幅度和时间均不变 |
| 双臂 14 轴 | 沿原姿态路径，22 个较明显开合段局部加快；其余区间保留原节奏 |
| 加快和停顿 | 路径推进局部最高 1.5 倍；运动期间平均 1.103 倍；接近开合终点时停顿，累计 0.781 秒 |
| 时间与头部 | 48.291667 秒、13,909 姿态、288 Hz；头部和首尾姿态不变 |
| 碰撞回退 | 约 35.1 秒处的一段保持原节奏，避免提前动作造成双手相交 |

这里的“手指不变”指目标关节轨迹。物理跟踪时，双臂和接触仍可能影响实际手指运动。来源为旧分支 `replay-6x-collision-smooth` 的 `2d0d503`；原分支和 `main` 保留。

## 下载和运行

需要 Git、已登录的 GitHub CLI（`gh`）、Python 3.12；显示和物理执行另需 Isaac Sim 5.1 与兼容 GPU。

```bash
git clone --branch codex/replay-6x-arm-fast-hand-fixed https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

预期输出 `Verified 89 runtime files`。模型固定使用 `v1.1.0`，轨迹与视频来自预发布 `6x-arm-fast-hand-fixed-20261020`。切换自旧版且已下载资源时，执行 `python3 scripts/download_assets.py --upgrade`，仅替换哈希匹配的受管文件；自行修改的文件不会被覆盖。

在仓库根目录运行，将 `/path/to/isaac/python.sh` 替换为 Isaac 的 Python 入口。第一行显示目标动作，第二行重新执行物理跟踪：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

目标文件已使用 6x 执行秒，显示按 1x 文件时间播放；物理入口使用 240 Hz、6x 源时间推进、线性目标和 98% 模型速度上限。两个入口支持 `--headless`、`--max-seconds 1` 和 `--out`；输出目录必须在仓库外。物理参数 `--wall-speed` 只调整等待和渲染节奏。

## 文件和视频

| 路径 | 用途 |
| --- | --- |
| `trajectories/trajectory_6x.npz` | 默认目标：`time` 执行秒、`source_time` 源秒、`q` 弧度、`joint_names` 轴名 |
| `trajectories/repaired_source.npz` | 同一目标的源时间版本，供物理入口使用 |
| `trajectories/physical_tracking_6x.npz` | 本次已验证物理运行的实际状态与目标 |
| `media/new_6x_full.mp4` | 新版完整目标动作视频，48.3 秒 |
| `media/old_vs_new_6x.mp4` | 完整同步对比视频，左旧右新，48.3 秒 |
| `evidence/reference/` | 臂部时间映射、手指一致性、时间质量和验收结果 |
| `evidence/reference_collision/`、`evidence/physics_collision/` | 目标及物理实际状态的离散网格检查 |
| `configs/lineage.json`、`release_manifest.json` | 来源、当前文件哈希和下载配置 |

视频为 Isaac 目标轨迹可视化，使用相同视角和帧时间；250 帧变化姿态重新渲染，1,199 帧完全相同姿态复用旧画面。它们不是实际物理运行录像。30 fps 共 1,449 帧，完整覆盖动作时间轴至帧取整精度。[发布页](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/6x-arm-fast-hand-fixed-20261020)提供轨迹和视频压缩包。

## 验证与限制

| 检查 | 结果 |
| --- | --- |
| 目标 / 实际网格 | 27,817 / 23,181 个原采样及中点姿态；每姿态 2,717 对，均零相交 |
| PhysX | 完整运行 48.295833 秒；零负间距对，最小报告间距 +0.05417 mm |
| 模型速度 | 目标与实际采样速度均通过；手指目标峰值接近模型上限（99.9903%） |
| 物理跟踪误差 | 全机器人 RMS 0.025509 rad，最大 0.597680 rad（34.24°） |
| 实际高频 RMS（6 Hz 以上） | 相对旧版 6x：臂部 +24.0%，手指 +1.5%；不代表抖动进一步减少 |

安装几何不变：物理 `base_Link` 世界 Z=0，桌面 Z=−0.45 m，高度差固定 **0.45 m**。仍有最大约 0.001473° 的微小位置越限；身体映射仍为 `candidate_unconfirmed`。离散碰撞、模型速度和物理采样结果不证明连续 CCD、真机电机能力或真机可执行性。检查命令和准备要求见 [DEPLOYMENT.md](DEPLOYMENT.md)。

当前目标 SHA-256：`5467cff16fcf08f2aba6fada0bce74d914ba8bc69e141a99e3c08531915db2e2`。新运行须重新验证。上游模型许可证见 `licenses/`。
