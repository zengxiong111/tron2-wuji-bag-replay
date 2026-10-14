# TRON2 + Wuji 修复后的 6x bag 重放

本分支发布已完成物理验证的 **6x 轨迹**：模拟时长 **48.295833 秒**，56 关节、240 Hz，11,591 个实际状态，头部朝前。原始 main 和 5x 分支保持不变。English: [README.en.md](README.en.md).

## 目录

- [安装与下载](#安装与下载)
- [在 Isaac 中运行](#在-isaac-中运行)
- [仓库结构](#仓库结构)
- [验证结果](#验证结果)
- [动作变化与限制](#动作变化与限制)

## 安装与下载

1. 准备 Git、GitHub CLI 和 Python 3.12。显示与物理执行还需要 Isaac Sim 5.1 及兼容 GPU；其 Python 环境需要 NumPy。离线分析环境依赖见 `requirements-analysis.txt`。
2. 执行以下指令：

```bash
gh auth login
git clone --branch replay-6x-collision-smooth https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

模型复用固定的 `v1.1.0` Release 资产；6x 轨迹从本分支预发布 `6x-collision-smooth-20261014` 下载。完整性检查应输出 `Verified 85 runtime files`。

已有克隆请先切换本分支，再执行 `python3 scripts/download_assets.py --upgrade` 和完整性检查。升级仅替换哈希匹配的受管参考文件、删除哈希匹配的旧 4x/5x 实际轨迹及 5x 视频；修改过的文件不会被覆盖。

## 在 Isaac 中运行

在仓库根目录运行，将 `/path/to/isaac/python.sh` 替换成你的 Isaac Python 入口。

1. **显示已验证的实际记录路径**：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
```

文件时间已经是 6x 执行秒，按 **1x 文件速度**显示，不要再乘 6。该模式设置记录姿态，不重新执行物理。

2. **以 6x 动作速度重新执行物理仿真**：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

该模式以 240 Hz 积分，发送密集线性位置/速度目标，启用桌面和自碰撞，使用 32/8 求解迭代、SDF512，并将模型速度上限收紧到 98%。保持原刚度和力矩上限，仅初始化时设置状态，之后由驱动器控制。不要改用三次插值，它会在密集接触姿态间超调。

两个入口均支持 `--headless`、`--max-seconds 1`、`--out`；结果写入仓库旁的新目录。物理入口的 `--wall-speed 10` 仅改变等待和渲染节奏，动作倍率仍为 6x；计算可能慢于实时。新的物理运行是诊断结果，需要重新检查，不自动继承本分支验收。

## 仓库结构

沿用 main 的目录布局：

| 路径 | 内容 |
| --- | --- |
| `scripts/replay.py` | 共用入口；`--physics` 选择物理执行 |
| `scripts/download_assets.py`、`scripts/verify_bundle.py` | 带哈希的 Release 下载和完整性校验 |
| `scripts/audit_retimed_trajectory.py`、`scripts/audit_target_velocity.py` | 默认实际路径的限制/离散碰撞检查及物理目标速度检查 |
| `trajectories/recorded_actual_6x.npz` | 实际姿态；`time` 为执行秒，`source_time` 为源秒，`q` 为弧度，`joint_names` 为轴序 |
| `trajectories/repaired_source.npz` | 密集线性物理参考，源时间单位 |
| `assets/`、`outputs/robot/` | 与 main 相同的机器人、网格和连接件 |
| `configs/trajectory.json`、`configs/physics.json`、`configs/lineage.json` | 时序、控制设置、输入哈希及来源 |
| `evidence/collision/`、`evidence/motor_limits/`、`evidence/target_motor_limits/` | 发布文件的碰撞、限位和目标速度复查 |
| `evidence/physics/`、`evidence/physics_collision/`、`evidence/target_collision/` | 完整原运行和目标/实际姿态证据，可移植入口短段检查 |
| `evidence/isaac_playback.json` | 可移植显示入口第一秒检查 |
| [DEPLOYMENT.md](DEPLOYMENT.md) | 真机准备及尚未满足的准入条件 |
| `tests/`、`requirements-analysis.txt`、`licenses/` | 离线测试、分析依赖和上游许可 |

轨迹和模型存放于 Release 资产，代码/配置/必要证据存放于 Git。没有原始 bag、旧轨迹、日志、内部指令或完整 6x 视频；本次交付对象为轨迹。

## 验证结果

| 项目 | 已验证结果 |
| --- | --- |
| 源区间 / 物理时长 | 0–289.766667 秒 / 48.295833 秒 |
| 默认实际路径末节点 | 48.291667 秒；剩余一个物理步可保持末姿态 |
| 发布路径离散碰撞 | 34,741 姿态 × 2,717 对几何体，零相交 |
| 原完整运行目标 / 实际网格检查 | 23,183 / 23,181 姿态，零相交 |
| 原完整 PhysX 接触 | 零负间距对；最小分离 +0.05417 mm |
| 抖动改善 | 相对直接提速的 6x 基线，关节/腕部高频 RMS 下降 65.43% / 56.46%，加速度绝对值 P95 下降 37.74% |
| 模型位置限制 | 目标和实际记录均无越限 |
| 模型速度占比 | TRON2 目标/实际 34.57% / 33.77%；Wuji 93.68% / 93.63%，均通过 |
| 跟踪 RMS / 最大值 | 0.023342 / 0.453704 rad，全部物理步统计 |
| 可移植物理入口 | 第一秒 241 状态的目标/实际数组与原整段运行前缀逐位一致 |
| 真机与连续 CCD | 未验证 |

查看[实际路径限位](evidence/motor_limits/report.json)、[目标速度](evidence/target_motor_limits/report.json)、[碰撞](evidence/collision/gate.json)、[物理验收](evidence/physics/acceptance.json)、[抖动](evidence/physics/temporal_quality.json)。高频指标采用 5 Hz 三阶零相位低通残差。离散碰撞保留同刚体/直接父子结构排除，不新增装配豁免；正间距处仍有接触事件。

## 动作变化与限制

1. 在 5x 修复参考上以源时间 0.6 Hz 平滑，使用接触约束物理姿态修正参考，并额外增加最多每只手腕 25 mm 间距。相对 5x 实际轨迹，指尖位置变化均值/P95 为 21.27/34.81 mm；最小双手指尖坐标原点间距由 6.47 变为 27.42 mm。最大间距变化为 95.62 mm，位于源时间 92.30 秒；关节 RMS/最大变化为 0.048654/0.801485 rad。局部手势已改变，未证明原表面接触与交互关系保持，详见[动作变化](evidence/physics/fidelity.json)。
2. 平滑改善不代表零抖动；线性插值不保证加速度/jerk 限制。离散网格和 PhysX 接触检查不证明连续 CCD、精确实体间隙或真机安全。
3. 物理基座 `base_Link` 世界 Z=0，桌面 Z=−0.45 m，固定高度差 **0.45 m**。物理基座、URDF 根、手腕及法兰是不同坐标系。16 维身体映射仍为 `candidate_unconfirmed`；真机身份、SDK 映射、电流/力矩与温度包络待确认。
4. 默认实际位置与验收的 6x 物理记录逐位相同，没有在发布时再次加速或改关节值。轨迹 SHA-256：`460463385a43a8df54604d84b01ea389b60b56152366f32487a3e846410894fc`；原物理记录 SHA-256：`7078b8dda74e1384efba22b2575f4067cd43b6ffee75341d88e6956b6d6db45c`。路径规范化的报告保留源报告哈希，见[来源](configs/lineage.json)。
5. 模型来源为 TRON2 commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0`（Apache 2.0）和 Wuji commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95`（MIT），见 `licenses/`；不自动覆盖用户轨迹或连接件。
