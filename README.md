# TRON2 + Wuji 5× 轨迹回放

本仓库提供一段已通过指定 Isaac Sim / PhysX 验收的 TRON2 双臂和 Wuji 双手轨迹，以及其运行模型与回放工具。这里的 5× 指相对原始 bag 的执行速度；默认回放已经使用 5× 执行时间，**不要再乘 5**。英文版：[README.en.md](README.en.md)。

## 目录

- [安装与下载](#安装与下载)
- [播放轨迹](#播放轨迹)
- [验证结果与边界](#验证结果与边界)

## 安装与下载

需要有权访问 GitHub 仓库，并安装 Git、GitHub CLI 和 Python 3。播放还需要 Isaac Sim 5.1 与兼容的 GPU。以下命令在终端执行：

```bash
gh auth login
gh auth setup-git
git clone --branch codex/bag-20261027-5x-validated https://github.com/zengxiong111/tron2-wuji-bag-replay.git
cd tron2-wuji-bag-replay
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

轨迹与模型从 GitHub Release 下载；仅克隆源码不足以回放。当前发布标签为 `bag-20261027-5x-validated-20261028`。下载器会校验归档和文件哈希，并拒绝覆盖本地修改过的文件。已有副本在仓库根目录执行：

```bash
git switch codex/bag-20261027-5x-validated
git pull --ff-only
python3 scripts/download_assets.py
python3 scripts/verify_bundle.py
```

## 播放轨迹

从仓库根目录启动。将路径替换为本机 Isaac Sim Python 解释器：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python
```

默认模式播放与已验收 PhysX 实际状态轨迹绑定的录制路径，不重新推进物理。时间单位为执行秒；不需要也不能再乘 5。回放日志和结果写入仓库外的 `../tron2-wuji-bag-replay-runs/<时间戳>/`。可加 `--headless` 隐藏窗口，或加 `--max-seconds 1` 只播放前 1 秒。

也可直接观看 [已验收实际状态回放视频](https://github.com/zengxiong111/tron2-wuji-bag-replay/releases/download/bag-20261027-5x-validated-20261028/actual_replay_5x.mp4)。视频是已保存 PhysX 状态的可视化，不会在播放视频时重新运行物理。

如需从零执行新的物理回放，必须显式加 `--physics`。它会生成新的诊断结果，不会替换已验收轨迹或证明这次新运行通过验收。前置条件、完整命令和复核要求见[物理复播说明](DEPLOYMENT.md)。

## 验证结果与边界

| 项目 | 已验证结果 |
| --- | --- |
| 时间范围 | 原 bag 长 347.212 s；开头约 2.494 s 缺少 body feedback。有效源区间 344.718 s，对应 68.944 s 的 5× 执行轨迹。 |
| 稠密几何检查 | 源轨迹 186,407 点、执行导出 325,400 点、PhysX 实际状态 66,187 点；各自对 2,707 个保留几何对检查，未发现碰撞事件。 |
| 实际跟踪与运动 | 实际腕间距相对原始 C2 最大 / RMS 为 1.891 / 0.927 mm；最大位置差分速度为原始 URDF 限值的 0.993；左拇指 CMC flex 行程比例 / 相关系数为 99.4% / 0.9965。 |
| 高频波动 | 手部 30 Hz 以上 RMS 为同相位原始 C2 基准的 0.411 倍。 |
| 综合验收 | 验收文件中的 52 项工作门限均通过，详见 `evidence/imported/acceptance_155.json`。 |

几何口径保留了 2,632 个机器人几何对和 75 个桌面几何对。官方 Wuji Beta2 装配规则中的 20 个内部 body pair 按来源规则排除，其中 10 个已属于 133 个结构排除；最终唯一排除数为 143。桌面到物理基座原点固定为 **0.45 m**。56 个轴均使用 acceleration drive；实际 PhysX 验收运行未启用手臂 force drive 或 RNEA 前馈。

这些结果代表指定模型和 Isaac / PhysX 离散采样检查，不构成连续碰撞检测（CCD）认证或真实硬件安全证明。离线 Pinocchio RNEA 力矩代理最大比值为 1.251，未通过 URDF effort 限值；不得据此声称力矩验收通过。原始 body mapping 仍未确认，硬件准入和 Sim2Real 均未验证。完整证据及逐项口径见 `evidence/imported/`、`metadata/` 和[物理复播说明](DEPLOYMENT.md)。
