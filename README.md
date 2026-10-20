# TRON2 + Wuji：6x 手指避碰轨迹

当前版本修复指间接触，保留上一版双臂、头部及时间轴，总时长 **48.29 秒**。English: [README.en.md](README.en.md)。分支名沿用旧名称，手指目标已做局部修改。

## 目录

- [当前动作](#当前动作)
- [下载和运行](#下载和运行)
- [文件和视频](#文件和视频)
- [验证与限制](#验证与限制)

## 当前动作

| 项目 | 当前结果 |
| --- | --- |
| 双臂、头部、时间轴 | 相对 `27ff86b` 逐点数值不变；保留约 +20% 开合、三个局部扩幅例外及 22 段局部最高 2 倍节奏 |
| 手指 | 为保持间隙局部调整；关节变化 RMS 0.523°、最大 8.276° |
| 左右手关系 | 手腕目标不变；指尖相对向量变化 RMS 2.579 mm、最大 20.660 mm，不是完全相同的手势 |
| 时长与采样 | 48.291667 秒；13,909 姿态，288 Hz |
| 物理配置 | 模型、驱动、碰撞参数、240 Hz、6x 时间推进、线性目标及 98% 模型限速均保留 |

## 下载和运行

需要 Git、已登录的 GitHub CLI（`gh auth login`）及 Python 3。Isaac 入口另需 Isaac Sim 5.1 和兼容 GPU。

```bash
git clone --branch codex/replay-6x-arm-fast-hand-fixed https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

预期 `Verified 93 runtime files`，随后显示验收绑定通过。模型固定来自 `v1.1.0`，轨迹和视频来自 [6x-no-finger-contact-20261020](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/tag/6x-no-finger-contact-20261020)。

已安装本分支上一版时，在仓库根目录运行：

```bash
git pull --ff-only
python3 scripts/download_assets.py --upgrade
python3 scripts/verify_bundle.py
```

升级仅替换上一版哈希匹配的受管文件，拒绝覆盖自行修改的文件，并删除匹配的旧对比视频及已退役证据。更早版本建议使用全新克隆。

将 `/path/to/isaac/python.sh` 替换为 Isaac Python 入口：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh
python3 scripts/replay.py --isaac-python /path/to/isaac/python.sh --physics
```

第一条显示目标，第二条重新执行物理跟踪。目标文件已采用 6x 执行秒，按 1x 文件时间显示；物理入口按 6x 源时间推进。支持 `--headless`、`--max-seconds 1` 和 `--out`，输出目录必须位于仓库外。物理参数 `--wall-speed` 仅调整等待和渲染节奏。

## 文件和视频

| 路径 | 用途 |
| --- | --- |
| `trajectories/trajectory_6x.npz` | 默认修复后目标：执行秒、源秒、弧度、轴名 |
| `trajectories/repaired_source.npz` | 同一目标的源时间版本，供物理入口使用 |
| `trajectories/physical_tracking_6x.npz` | 验收的完整 240 Hz 实际状态与目标 |
| `media/new_6x_full.mp4` | 上述实际状态在 Isaac 中渲染的视频，48.3 秒、30 fps、1,449 帧 |
| `evidence/reference/` | 验收、修改量、来源及模型等价性 |
| `evidence/reference_collision/`、`evidence/physics_collision/` | 目标与实际状态的整机网格检查 |
| `evidence/finger_*`、`evidence/physics/` | 指间间隙、线性路径证书及物理接触报告 |
| `configs/lineage.json`、`release_manifest.json` | 来源提交、文件哈希及下载配置 |

视频来自实际执行的 6x 物理状态；录制时按状态重绘，没有再次运行物理，也没有后期加速。默认显示命令展示目标，因此与实际状态视频有跟踪误差。旧版对比视频不代表本次动作，已移除。

JSON 证据仅重排格式及迁移路径标签；`evidence/reference/import_binding.json` 记录验收源字节与发布字节的哈希，校验器检查发布哈希。

## 验证与限制

| 检查 | 验收结果 |
| --- | --- |
| 指间碰撞响应 | 完整运行 48.295833 秒，含初始化：非零冲量对/点均为 0 |
| 预测接近报告 | 仍有 3,602 个正间距、零冲量点；不等于没有接触回调 |
| 指间采样网格间隙 | 目标最小 2.334 mm；实际最小 0.363 mm |
| 整机网格 | 目标 27,817 / 实际 23,181 个原采样及中点姿态，每姿态 2,717 对，零相交 |
| 线性路径间隙证书 | 目标和记录实际状态的线性插值路径均通过；保守区间下界约 0.010 mm |
| 模型速度 | 手指目标峰值 99.9900%，臂目标 66.3724%，实际有限差分峰值 98.6036% |
| 全部 240 Hz 跟踪误差 | RMS 0.026866 rad，最大 0.632980 rad |
| 手指实际高频 RMS | 6 Hz 以上相对上一版约 −2.00%；仅表示该指标未增加 |

物理 `base_Link` 世界 Z=0，桌面 Z=−0.45 m，差值固定 **0.45 m**。继承的左腕位置越限约 **0.001067°** 未改变；机体轴序仍为 `candidate_unconfirmed`。报告中的 `max_error_rad` 使用渲染节拍，完整跟踪指标见 `full_trace_tracking_error`。

线性路径证书不涵盖未知的 PhysX 步内真实曲线，也不证明凸碰撞形状等价、真机 CCD 或电机安全。当前无真机控制入口。离线检查与真机准备见 [DEPLOYMENT.md](DEPLOYMENT.md)。模型许可证见 `licenses/`。

当前目标 SHA-256：`bedc266d4339cd038e35dcd0ec555cb41696a004fb018c8437573cc94a728246`。验收仅绑定所列文件，新运行需重新检查。
