# TRON2 + Wuji：原始轨迹、5倍重放与修复

这是一个独立的私有运行包：从原始 ROS1 bag 读取 TRON2 与双 Wuji Hand2 的关节反馈，在 Isaac Sim 中以5倍动作速度重放，并修复碰撞与抖动。英文：[README.en.md](README.en.md)。

## 目录

- [下载和播放](#下载和播放)
- [文件在哪里](#文件在哪里)
- [修复流程](#修复流程)
- [结果和注意事项](#结果和注意事项)

## 下载和播放

需要 Git、GitHub CLI、Python3.12；运行仿真还需要已安装的 Isaac Sim5.1及兼容GPU。下载私有文件需要具有仓库访问权限的 GitHub账号。

1. 克隆代码并下载模型、bag、轨迹和视频：

```bash
gh auth login
gh auth setup-git
git clone https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

下载器先检查SHA-256，再安装文件，不覆盖你修改过的输入。大文件存放在同仓库的[v1.0.0 Release](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/v1.0.0)。

2. 直接看完整视频：打开 `media/head_forward_full_5x.mp4`。时长57.97秒，30fps，头部朝前。

3. 在 Isaac 窗口播放最终修复轨迹：

```bash
ISAAC_PYTHON=/你的Isaac安装目录/python.sh
python3 scripts/replay.py --isaac-python "$ISAAC_PYTHON"
```

`ISAAC_PYTHON`需要替换成你机器的Isaac Python入口。默认使用修复后的轨迹、5倍动作速度、240Hz物理仿真，并将新运行结果保存到仓库旁的 `tron2-wuji-bag-replay-runs/`。

| 想看什么 | 增加的参数 |
| --- | --- |
| 原始目标、1倍速度 | `--mode original` |
| 原始目标、5倍速度 | `--mode accelerated` |
| 修复目标、5倍速度，保留源头部动作 | `--mode repaired` |
| 修复目标、5倍速度，头部朝前（默认） | `--mode head-forward` |

这些模式共用修好的碰撞模型。未修复目标不保证避碰。5倍指仿真动作变快，不是视频后期加速，也不保证计算机能实时跑5倍。

## 文件在哪里

| 文件 | 含义 |
| --- | --- |
| `data/2026-10-08-17-54-13.bag` | 原始真机ROS1记录，289.797秒 |
| `trajectories/original.npz` | 原始反馈重采样，8694帧、56关节 |
| `trajectories/accelerated_5x.npz` | 相同关节位置，显式5倍时间轴，未做修复 |
| `trajectories/repaired_source.npz` / `repaired_5x.npz` | 避碰和滤波后的参考，分别使用源时间/5倍时间 |
| `trajectories/head_forward_source.npz` / `head_forward_5x.npz` | 最终修复参考，头部yaw/pitch固定为零 |
| `outputs/robot/tron2_wuji_hand2.usd` | 原始标准USD，已装配转接件 |
| `outputs/robot/robot_collision_fixed.usd` | 物理重放用的SDF512碰撞修复模型 |
| `outputs/robot/tron2_wuji_hand2.urdf`、`assets/` | 可移植URDF、必要网格与转接件STP/STL |
| `evidence/final/validation/tracking.npz` | 最终完整物理重放的实际关节状态与目标 |
| `evidence/`、`configs/` | 碰撞、平滑度、来源及哈希记录 |

原始记录是用户明确要求保留的运行输入，标准模型和碰撞模型分别用于源几何与物理重放，不是备份。旧调试输出未上传。

NPZ的 `q` 单位是rad，列顺序由 `joint_names`给出。显式5倍文件的 `time` 是仿真秒，`source_time` 是原始秒，`q_velocity` 是目标rad/s。它们是目标轨迹，不等于实际物理状态；启动器使用源时间参考，再由控制器实施5倍速度，避免重复加速。

## 修复流程

无需安装ROS1；用 `rosbags`直接读bag。分析/修复环境与Isaac环境分开，避免更换Isaac内置USD库。

```bash
python3.12 -m venv .venv-analysis
.venv-analysis/bin/pip install -r requirements-analysis.txt
.venv-analysis/bin/python scripts/repair_pipeline.py
```

流程很简单：**原始bag → 30Hz具名反馈 → 连续腕部间距/IK修正 → 低通滤波 → 再次避碰 → 导出5倍轨迹 → 头部朝前**。

- 模型：把实际碰撞API放到Mesh，开启自碰撞，手部使用SDF512；不改变转接件或视觉网格。
- 避碰：连续调整双腕间距，通过完整SE(3) IK同步修正手臂；安全裕量0.03m。
- 去抖：源时间1Hz、四阶零相位低通，限位裕量0.005rad；重放时使用C2三次位置插值及对应连续速度。
- 固定条件：物理基座高于桌面0.45m，保留既有同刚体/直接父子关节排除规则。

重建碰撞模型：

```bash
.venv-analysis/bin/python scripts/116_build_collision_fixed_usd.py \
  --source outputs/robot/tron2_wuji_hand2.usd \
  --out ../tron2-wuji-bag-replay-runs/model/robot_collision_fixed.usd \
  --hand-approximation sdf --sdf-resolution 512
```

从已提取反馈快速重跑可加 `--reference-only`。每次重建都写新目录，不覆盖交付轨迹；新的物理运行需要重新检查碰撞，不能沿用旧验收结论。

## 结果和注意事项

- 完整轨迹约289.77秒；5倍仿真约57.95秒；完整视频57.97秒（帧取整）。
- 整段13909个物理状态、每步2717对几何，未发现超过0.1mm的接触特征穿透。
- 与修复前的物理重放比较：关节高频抖动降低约61%，腕部高频抖动降低约51%，加速度p95降低约82%。
- 40个手部关节有明确名称；bag的16维机身顺序仍按“左7、右7、头yaw/pitch”假设，未确认。
- 滤波会削弱快速手势细节；原始bag未修改。离散碰撞检查不等于CCD或真机安全。

检查交付的完整物理状态：

```bash
.venv-analysis/bin/python scripts/115_audit_collision_fixed_replay.py \
  --input evidence/final/validation/tracking.npz \
  --out ../tron2-wuji-bag-replay-runs/audit
```

资产来源：TRON2 [limxdynamics/tron2-robot-description](https://github.com/limxdynamics/tron2-robot-description)，commit `f547f5bc949f2a4c98e076e61cf6d3ca73d179a0`，Apache2.0；Wuji [wuji-technology/wuji-description](https://github.com/wuji-technology/wuji-description)，commit `c2cd7f8d1ef8b6dc8cb907c17daa5a88b4442d95`，MIT。原始许可证在 `licenses/`；用户提供的bag与转接件不因此获得第三方模型许可证。精确输入哈希在 `release_manifest.json`、`configs/origin.json`。
