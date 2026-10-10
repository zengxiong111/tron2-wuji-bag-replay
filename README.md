# TRON2 + Wuji 轨迹回放

在 Isaac Sim 中播放 TRON2 双臂和 Wuji 双手的记录轨迹。当前轨迹长 **72.44 秒**，共 56 个关节，动作速度相当于源 bag 的 4 倍。英文：[README.en.md](README.en.md)。

## 目录

- [安装与下载](#安装与下载)
- [播放](#播放)
- [文件说明](#文件说明)
- [检查结果与限制](#检查结果与限制)

## 安装与下载

需要私有仓库访问权限、Git、GitHub CLI、Python 3.12；播放还需要 Isaac Sim 5.1 和兼容 GPU。以下命令在终端执行：

```bash
gh auth login
gh auth setup-git
git clone https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

模型和轨迹从 GitHub Release 下载，不能只复制源码。校验成功会显示 `Verified 84 runtime files`。已有克隆先运行 `git pull --ff-only`，再运行 `python3 scripts/download_assets.py --upgrade` 和校验命令；下载器不会覆盖用户修改过的输入。

## 播放

在仓库根目录执行，将路径替换成本机 Isaac Sim 的 `python.sh`：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
```

轨迹时间已调整，按默认文件时间播放即可，不要再乘 4。加 `--headless` 可隐藏窗口，加 `--max-seconds 1` 可检查前 1 秒。播放记录自动保存在仓库旁的 `tron2-wuji-bag-replay-runs/`。

常见问题：下载失败时检查 GitHub 访问权限后重试；校验缺文件时重新下载；无法启动窗口时检查 Isaac Sim 入口与 GPU 环境。`python3` 用于启动脚本，实际回放由指定的 Isaac Python 执行。

## 文件说明

| 路径 | 用途 |
| --- | --- |
| `scripts/replay.py` | 回放入口 |
| `scripts/download_assets.py`、`scripts/verify_bundle.py` | 下载与完整性校验 |
| `trajectories/retimed_actual_4x.npz` | 当前轨迹，下载后生成 |
| `assets/`、`outputs/robot/` | 模型、网格和转接件，下载后生成 |
| `configs/`、`release_manifest.json` | 轨迹、来源、硬件配置与下载清单 |
| `evidence/` | 当前轨迹的检查结果 |
| `tests/`、`requirements-analysis.txt` | 离线复核工具与依赖 |
| [DEPLOYMENT.md](DEPLOYMENT.md) | 真机部署前的准备与检查 |
| `licenses/` | 上游模型许可证 |

轨迹文件的 `time` 单位为执行秒，`source_time` 为源 bag 秒，`q` 为弧度；关节顺序见 `joint_names`。共 13,909 帧，原生采样率 192 Hz。播放器使用分段线性位置插值，不使用 `q_velocity`。

## 检查结果与限制

| 项目 | 当前结果 |
| --- | --- |
| 模型速度 | TRON2 / Wuji 峰值分别为模型限速的 34.86% / 80.72%，通过 |
| 关节角度 | 3 轴轻微越界，最大 0.001635°，严格检查未通过 |
| 离散碰撞 | 45,194 个姿态、每姿态 2717 对几何，未检出交叉；不代表连续碰撞检查通过 |
| Isaac 回放 | 已有完整路径播放记录；当前播放记录位置，不重新执行动力学 |
| 真机 | 尚未验证，仓库没有真机控制程序 |

详细结果见 [速度与角度](evidence/motor_limits/report.json)、[碰撞检查](evidence/collision/gate.json) 和 [Isaac 回放](evidence/isaac_playback.json)。离线复核命令见 [部署指南](DEPLOYMENT.md#离线检查)。

桌面到物理基座原点的垂直间距固定为 **0.45 m**：当前 `base_Link` 世界 Z=0、桌面 Z=-0.45 m。物理基座与 URDF 根、腕部和法兰原点不可混用。

轨迹 SHA-256：`23b555d6a4bf6df33fa4da4e5905a10f9fe375f6447784edd44079e412bdfcd1`。来源见 `configs/retiming.json` 与 `configs/origin.json`。当前包不包含原始 bag 或旧轨迹。

模型来源：TRON2 commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0`（Apache 2.0）；Wuji commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95`（MIT）。许可证见 `licenses/`，不自动覆盖用户轨迹或转接件。
