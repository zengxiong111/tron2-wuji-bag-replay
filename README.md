# TRON2 + Wuji：保留 5x 动作的 6x 轨迹

本分支的默认轨迹已替换为保留原 5x 动作的参考版，约 **48.29 秒**。双臂、头部和首尾姿态不变，仅在测得的手部超速/碰撞窗口小幅修正。English: [README.en.md](README.en.md).

## 目录

- [安装和更新](#安装和更新)
- [在 Isaac 中运行](#在-isaac-中运行)
- [结果和文件](#结果和文件)
- [使用限制](#使用限制)

## 安装和更新

需要 Git、已登录的 GitHub CLI、Python 3.12；显示/物理执行还需要 Isaac Sim 5.1 及兼容 GPU。首次使用：

```bash
git clone --branch replay-6x-collision-smooth https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

已有本分支克隆，在仓库根目录执行：

```bash
git pull --ff-only
python3 scripts/download_assets.py --upgrade
python3 scripts/verify_bundle.py
```

下载器会替换哈希匹配的旧参考文件并移除旧受管轨迹，拒绝覆盖自行修改的输入。模型仍使用固定的 `v1.1.0` 资产；新轨迹来自 `6x-preserve5x-20261015` 预发布。校验应输出 `Verified 87 runtime files`。

## 在 Isaac 中运行

在仓库根目录执行，将 `/path/to/isaac/python.sh` 换成你的 Isaac Python 入口。

1. **查看保留动作的参考路径**（默认）：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
```

直接设置参考姿态，不重新执行物理。文件时间已是 6x 执行秒，按 1x 文件时间显示。

2. **重新执行物理跟踪**：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

使用 240 Hz、6x 动作速度、密集线性目标、98% 模型速度上限及局部接触裕量。新的实际运动会有跟踪偏差，需重新检查。两个入口均支持 `--headless`、`--max-seconds 1`、`--out`；输出写入仓库旁的新目录。物理入口的 `--wall-speed` 只调整等待/渲染节奏。

## 结果和文件

以下小幅偏差指标属于**参考路径**：

| 检查 | 结果 |
| --- | --- |
| 参考时长 / 采样 |48.291667秒；56轴、13,909姿态、288 Hz |
| 原样保留的关节采样值 |98.41%；双臂、头部、首尾姿态及修复窗口外全部不变 |
| 最大 / RMS 关节修正 |3.80° /0.0775° |
| 指尖平均 / 最大偏差 |0.019 /4.24 mm |
| 参考 / 物理实际离散网格 |均零相交；只排除结构性相邻/同刚体对 |
| 最终 PhysX 接触报告 |零负接触间距对 |
| 模型速度 |参考与物理实际采样速度均通过 |

修复窗口分散在全段，约 50.90% 的帧至少有一个手指值被微调；98.41% 指的是“关节×采样点”，并非整帧比例。没有全段平滑或移动手腕分离双手。

| 文件 | 用途 |
| --- | --- |
| `trajectories/trajectory_6x.npz` | 默认保真参考：`time`为执行秒，`source_time`为源秒，`q`为弧度，`joint_names`为轴序 |
| `trajectories/repaired_source.npz` | 同一目标的源时间版本，用于物理执行 |
| `trajectories/physical_tracking_6x.npz` | 已验证物理运行的实际状态与目标，单独保存 |
| `configs/`、`release_manifest.json` | 时序、模型/输入哈希、来源和下载配置 |
| `evidence/reference/`、`evidence/collision/`、`evidence/physics/` | 修改范围、保真度、碰撞及物理结果 |
| [DEPLOYMENT.md](DEPLOYMENT.md) | 离线检查和真机准备 |

## 使用限制

- **物理实际运动仍有偏差**：相对 5x，指尖平均/P95/最大偏差约 7.94/18.37/72.74 mm，最大关节偏差约 28.10°。小幅参考修正不代表物理实际同样保真，详见[对比结果](evidence/reference/fidelity_temporal.json)。
- 保留了原 5x 的微小位置越限，最大约 0.001635°。线性插值尚未约束真机加速度/jerk；离散碰撞不证明连续 CCD 或真机安全。
- 物理基座 `base_Link` 世界 Z=0、桌面 Z=−0.45 m，高度差固定 **0.45 m**。身体映射仍为 `candidate_unconfirmed`；设备身份、SDK映射、电流/力矩/温度限制待确认。
- 新参考 SHA-256：`841c29aefb3a3bfb2813bd59698ffe60ecdf20b712205227cb5ba05f77890253`。修改掩码见 `evidence/reference/local_masks.npz`；来源见[lineage](configs/lineage.json)。新运行不自动继承原报告验收。
- 上游模型及许可见 `licenses/` 和 `configs/origin.json`；模型许可不自动覆盖用户轨迹或连接件。
