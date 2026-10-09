# TRON2 + Wuji：72.44 秒减速轨迹

当前默认是此前 57.95 秒 PhysX **实际运动轨迹**的减速版，时长 **72.4375 秒**，相当于原 bag 的 4 倍动作速度。位置保持不变。默认在 Isaac 按记录路径播放，**没有重新计算物理运动，也没有真机下发**。英文：[README.en.md](README.en.md)。

## 目录

- [下载和升级](#下载和升级)
- [在 Isaac 播放](#在-isaac-播放)
- [检验结果与文件](#检验结果与文件)
- [复核与兼容性](#复核与兼容性)

## 下载和升级

需要 Git、GitHub CLI、Python 3.12；播放需要 Isaac Sim 5.1 和兼容 GPU。仓库为私有，需要访问权限。

```bash
gh auth login
gh auth setup-git
git clone https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

已有克隆：`git pull --ff-only` 后运行 `python3 scripts/download_assets.py --upgrade`，再校验。升级只清理哈希匹配的旧受管文件；修改过的文件不会被覆盖。旧轨迹、旧检验数据和旧视频从当前运行版本移除，原始输入在本地仓库外保留。

新轨迹在 [v1.2.0 Release](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/v1.2.0)。模型未改变，下载器按清单复用 v1.1.0 的同哈希模型资产；运行所需文件全部由清单覆盖。当前包不分发源 bag 或旧轨迹副本。

## 在 Isaac 播放

```bash
ISAAC_PYTHON=/path/to/isaac/python.sh
python3 scripts/replay.py --isaac-python "$ISAAC_PYTHON"
```

把 `/path/to/isaac/python.sh` 换成本机 Isaac Python 入口。**时间轴已经减速，按 1 倍文件时间播放，不要再次加速 4 倍。** 播放器使用分段线性插值和位置回放；窗口可显示完整路径，不做新的动力学积分。运行记录写到仓库旁的 `tron2-wuji-bag-replay-runs/`。可加 `--headless` 隐藏窗口；`--max-seconds 1` 仅作短检查。

## 检验结果与文件

| 检查 | 结论 |
| --- | --- |
| TRON2 / Wuji 模型速度 | 最高分别占限速 34.86% / 80.72%，无超速轴 |
| 碰撞与穿透 | 45,194 个姿态、每姿态 2717 对几何，严格离散检查未检出交叉；没有装配碰撞豁免 |
| 角度 | 3 个轴微小越界，最大 0.001635°，严格角度检查未通过 |
| 真机全部电机极限 | 未验证；硬件版本、映射、逐轴额定参数、电流/力矩和温度仍待确认 |

| 文件 | 用途 |
| --- | --- |
| `trajectories/retimed_actual_4x.npz` | 当前唯一默认轨迹：13,909 帧、56 轴、192 Hz 原生采样 |
| [REPORT.md](REPORT.md)、[REPORT.en.md](REPORT.en.md) | 简明检验报告及真机部署前步骤 |
| [HARDWARE.md](HARDWARE.md)、[HARDWARE.en.md](HARDWARE.en.md) | 参数与映射证据表，未知项明确留空 |
| [DEPLOYMENT.md](DEPLOYMENT.md)、[DEPLOYMENT.en.md](DEPLOYMENT.en.md) | 简明真机部署指南：准备、逐级调试、停机与验收 |
| `evidence/` | 逐轴结果、碰撞覆盖、完整 Isaac 路径播放验证 |
| `configs/trajectory.json`、`configs/retiming.json` | 当前输入绑定及减速来源 |
| `outputs/robot/`、`assets/` | 当前 USD、可移植 URDF 和必要网格/转接件 |

轨迹 SHA-256：`23b555d6a4bf6df33fa4da4e5905a10f9fe375f6447784edd44079e412bdfcd1`。`time` 是执行秒，`source_time` 是原 bag 秒，`q` 为 rad。`q_velocity` 是数值梯度，播放器不使用它；限速检验使用真正的逐段位置斜率。

## 复核与兼容性

```bash
python3.12 -m venv .venv-analysis
.venv-analysis/bin/pip install -r requirements-analysis.txt
.venv-analysis/bin/python scripts/audit_retimed_trajectory.py --out ../tron2-retimed-check
python3 scripts/check_hardware_admission.py
```

输出目录必须为仓库外的新目录。轨迹检查当前预期退出码 2，因为存在微小角度越界；硬件准入也预期退出码 2，因为实机证据缺失。这些不能写成全部通过。

旧 `--mode` 和 `--max-source-seconds` 选项已移除；使用 `--max-seconds` 指定执行时间。旧物理目标重放不能与当前记录路径播放混为一谈。模型速度通过及离散无交叉，不等于连续 CCD、新物理执行或真机安全。

物理基座 `base_Link` 的世界 Z=0，桌面 Z=-0.45 m，固定间距 0.45 m。模型源自 TRON2 commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0`（Apache 2.0）和 Wuji commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95`（MIT）；许可证在 `licenses/`。这些许可证不自动适用于用户轨迹或转接件。
