# TRON2 + Wuji 修复后的5倍bag重放

本分支恢复原始修复后的 **5倍** 轨迹：**57.95秒**、56关节、240Hz下13,909个记录状态，头部朝前。Main的4倍记录是同一位置序列的放慢版；本分支保留原5倍时序。英文：[README.en.md](README.en.md)。

## 目录

- [安装与下载](#安装与下载)
- [路径回放与物理执行](#路径回放与物理执行)
- [文件](#文件)
- [结果与限制](#结果与限制)

## 安装与下载

依赖Git、已登录的GitHub CLI及Python3.12。播放/物理执行另需Isaac Sim5.1及兼容GPU。物理入口还要求同一个Isaac Python环境安装SciPy；若缺失，执行 `/path/to/isaac/python.sh -m pip install scipy`。运行：

```bash
gh auth login
git clone --branch replay-5x-collision-smooth https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

模型和轨迹通过带校验和的GitHub Release资产安装。校验成功输出 `Verified 86 runtime files`。机器人资产复用固定的v1.1.0发布；5倍轨迹/视频资产属于本分支预发布。已有克隆需切换到本分支，运行 `python3 scripts/download_assets.py --upgrade` 并校验。下载器拒绝覆盖被修改的输入，仅删除哈希匹配的已退役4倍受管数据。Main及其发布不变。

## 路径回放与物理执行

在仓库根目录运行，将 `/path/to/isaac/python.sh` 替换为你的Isaac Sim Python入口。

1. **回放已验证的记录路径**（默认）：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
```

该模式直接设置记录的实际关节位置，不重新执行动力学。文件时间已经是源时间的5倍动作速度：**不要再乘5倍**。`--headless` 隐藏窗口，`--max-seconds 1` 检查第一秒。

2. **在PhysX中重新执行修复后的目标**：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

该模式以240Hz积分物理，按5倍动作速度发送连续C2位置/速度目标，启用桌面/自碰撞，保留32/8求解迭代及SDF512。仅初始化时设置关节状态，之后通过驱动器控制。它生成新的诊断记录，需要重新验证；已附整段报告属于原始物理运行。`--max-seconds 1` 限制仿真时间，从源起点开始。`--wall-speed` 只改变墙钟节奏，计算可能慢于实时，不会再次倍乘动作速度。输出保存在仓库旁的 `<checkout-name>-runs/`，不包含硬件I/O。

[完整视频：57.97秒、30fps](media/head_forward_full_5x.mp4) · [直接下载视频](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/download/5x-collision-smooth-20261014/head_forward_full_5x.mp4)。视频按固定帧率渲染记录的5倍PhysX实际状态，不是1倍视频后期提速。原始bag289.797秒、共同参考289.767秒及视频帧取整解释了小幅时长差异。

## 文件

| 路径 | 用途 |
| --- | --- |
| `scripts/replay.py` | 共用入口；`--physics` 选择新的动力学执行 |
| `scripts/download_assets.py`、`scripts/verify_bundle.py` | 资产下载/完整性检查 |
| `trajectories/recorded_actual_5x.npz` | 默认实际状态路径；`time`为执行秒，`source_time`为bag秒，`q`为弧度，`joint_names`为轴序 |
| `trajectories/repaired_source.npz` | 源时间下平滑/间距修正后的目标，头部yaw/pitch目标为0 |
| `assets/`、`outputs/robot/` | 与main相同的机器人、网格及连接件 |
| `configs/trajectory.json`、`configs/physics.json`、`configs/lineage.json` | 时序、模型/输入哈希及来源 |
| `evidence/collision/`、`evidence/motor_limits/`、`evidence/target_motor_limits/` | 发布的实际路径及目标速度曲线的最新检查 |
| `evidence/physics/` | 原始完整5倍物理运行、接触和平滑证据；独立标注可移植入口短段测试 |
| `evidence/isaac_playback.json` | 可移植记录路径入口第一秒测试 |
| [DEPLOYMENT.md](DEPLOYMENT.md) | 真机准备及未通过的准入 |
| `tests/`、`requirements-analysis.txt`、`licenses/` | 离线测试/依赖及上游许可 |

实际路径采用位置线性插值，不保留未使用的速度数组；物理参考采用源时间C2插值及解析速度前馈。默认实际位置与原物理记录逐位相同，仅规范化文件打包/时间单位。不包含原始bag、基线记录、日志、内部指令或历史资产。

## 结果与限制

| 项目 | 当前结果 |
| --- | --- |
| 记录路径的离散碰撞 | 41,692个姿态 ×2,717对几何，零相交，仅排除结构性相邻/同刚体对 |
| 原物理运行碰撞 | 全部13,909个记录状态通过0.1mm阈值；PhysX最小接触分离距离+0.016693mm |
| 原运行抖动 | 相对先前仅避碰修正的版本，关节/腕部高频RMS下降61.21% /51.24%，加速度绝对值p95下降82.37% |
| 发布实际路径的模型速度 | TRON2峰值为限速43.57%；Wuji为100.90%，2轴略超，未通过 |
| 物理目标的模型速度 | 8个Wuji关节超限，最严重超15.55%，未通过 |
| 关节角度 | 3轴轻微越界，最大0.001635°，严格检查未通过 |
| 真机/连续CCD | 未验证，无真机执行授权 |

证据：[实际路径限位](evidence/motor_limits/report.json)、[目标速度](evidence/target_motor_limits/report.json)、[离散碰撞](evidence/collision/gate.json)、[原始物理运行](evidence/physics/report.json)、[平滑度](evidence/physics/temporal_quality.json)。平滑改善不代表零抖动，线性记录路径插值不保证加速度/jerk；离散采样不认证连续CCD。

物理基座/桌面间距仍为 **0.45m**：`base_Link` 世界Z=0、桌面Z=-0.45。物理基座、URDF根、手腕和法兰是不同坐标系。机身映射仍为 `candidate_unconfirmed`；真机身份、SDK映射、电流/力矩及温度限值未解决。

实际轨迹SHA-256：`4e086ddf442ddded028b0af4824c519de4a004ba421b9df4662c11c8e498fc06`。源物理记录SHA-256：`8634e73e49bbc0b769ea86c21ef918df92962da55e43305784c7005ffc6b505c`。[来源](configs/lineage.json)保留源哈希及数组身份，路径规范化的报告有明确标注。新的物理执行可能产生不同结果，不自动继承验收。

模型来源仍为TRON2 commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0`（Apache2.0）及Wuji commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95`（MIT），见 `licenses/`。上述许可不自动覆盖用户轨迹或连接件。
