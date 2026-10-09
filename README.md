# TRON2 + Wuji：4 倍 Isaac 轨迹重放

当前默认轨迹为 **4 倍动作速度、头部朝前，约 72.44 秒**。模型速度与目标角度检查通过；真机电机全部极限尚未验证。先看 [简明检验报告](REPORT.md)。英文：[README.en.md](README.en.md)。

## 目录

- [下载和播放](#下载和播放)
- [文件说明](#文件说明)
- [修复和复核](#修复和复核)
- [限制与来源](#限制与来源)

## 下载和播放

需要 Git、GitHub CLI、Python 3.12；仿真需要已安装的 Isaac Sim 5.1 及兼容 GPU。私有文件需要仓库访问权限。

1. 下载代码和运行输入：

```bash
gh auth login
gh auth setup-git
git clone https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

已有旧克隆：先 `git pull --ff-only`，再运行 `python3 scripts/download_assets.py --upgrade` 和校验命令。升级只替换哈希匹配旧交付的受管文件；修改过的输入不会被覆盖。旧版 5 倍文件及旧视频按已知哈希清理。

二进制运行输入在 [v1.1.0 Release](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/v1.1.0)，代码与简明报告直接保存在 Git。当前版本未附新视频，旧 5 倍视频不能代表本版。

2. 打开 Isaac 窗口执行：

```bash
ISAAC_PYTHON=/path/to/isaac/python.sh
python3 scripts/replay.py --isaac-python "$ISAAC_PYTHON"
```

将 `/path/to/isaac/python.sh` 替换为本机 Isaac Python 入口。默认使用源时间参考，由物理控制器实施 4 倍动作速度、240 Hz 步长；不是对视频加速。计算机运行耗时取决于性能。新运行记录放在仓库旁的 `tron2-wuji-bag-replay-runs/`。

| 模式 | 含义 |
| --- | --- |
| 默认 / `--mode head-forward` | 修复后 4 倍，头部朝前 |
| `--mode repaired` | 默认模式的兼容别名，同一条轨迹 |
| `--mode original` | 原始目标，1 倍，未修复 |
| `--mode accelerated` | 原始目标，4 倍，未修复 |

## 文件说明

| 文件 | 用途 |
| --- | --- |
| `data/2026-10-08-17-54-13.bag` | 原始真机 ROS1 输入，289.797 秒 |
| `trajectories/original.npz` | 原始反馈 30 Hz 参考，8694 帧、56 轴 |
| `trajectories/head_forward_source.npz` | 当前修复参考，源时间 0–289.7667 秒 |
| `trajectories/head_forward_4x.npz` | 同一目标的 4 倍时间轴和速度导出，约 72.4417 秒 |
| `evidence/final/validation/tracking.npz` | 完整 4 倍物理重放的目标与实际状态 |
| `evidence/comparison/` | 本次去抖对照所需原始 4 倍记录与零头部目标 |
| `outputs/robot/`、`assets/` | 标准 USD、SDF512 碰撞 USD、可移植 URDF、必要网格和转接件 |
| `REPORT.md`、`REPORT.en.md` | 简明检验报告 |
| `evidence/`、`configs/`、`release_manifest.json` | 可复核数值、参数及 SHA-256 |

`q` 是 rad，列顺序由 `joint_names` 给出。4 倍导出的 `time` 是仿真秒，`source_time` 是原始秒，`q_velocity` 是 rad/s。启动器使用源时间文件，避免再次压缩已加速时间轴。目标不等于实际物理状态。

## 修复和复核

分析环境与 Isaac 环境分开；无需 ROS1 安装。

```bash
python3.12 -m venv .venv-analysis
.venv-analysis/bin/pip install -r requirements-analysis.txt
.venv-analysis/bin/python scripts/repair_pipeline.py
.venv-analysis/bin/python scripts/check_motion_limits.py --out ../tron2-4x-checks
.venv-analysis/bin/python scripts/115_audit_collision_fixed_replay.py \
  --input evidence/final/validation/tracking.npz --out ../tron2-4x-collision
```

流程：原 bag → 30 Hz 具名反馈 → 源时间 1 Hz 四阶零相位低通和头部朝前 → 连续双腕避碰/SE(3) IK（0.04 m 裕量）→ C2 插值和 4 倍导出 → 完整物理检查。`--reference-only` 可复用已提取参考。每次重建写新目录，新物理运行需重新验收。

## 限制与来源

- 当前 17,386 个实际状态的离散碰撞检查通过，关节高频抖动下降 78.98%；完整结果及未验证项见 [报告](REPORT.md)。参考目标仍有手内重叠，不能用于无碰撞运动学播放；仿真接触会造成跟踪偏差。
- 机身轴序仍未确认；头部两个目标为零。未验证真机电流、力矩、温升、负载或连续 CCD。滤波削弱部分快速手势。原 bag 保留不变。
- 物理基座为 `base_Link`，Z=0；桌面 Z=-0.45 m，固定相差 0.45 m。不能移动安装高度来通过检查。标准 USD 与碰撞 USD 的视觉/安装几何一致。
- 模型来自 [TRON2](https://github.com/limxdynamics/tron2-robot-description) commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0`（Apache 2.0）与 [Wuji](https://github.com/wuji-technology/wuji-description) commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95`（MIT）。许可证在 `licenses/`；这些许可证不自动适用于用户 bag 和转接件。输入身份见 `configs/origin.json` 和 `release_manifest.json`。
