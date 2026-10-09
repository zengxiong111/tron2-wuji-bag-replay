# 减速轨迹检验报告

核查日期：2026-10-09。**模型速度通过；严格离散碰撞检查通过；角度有微小越界；尚不能确认真机电机全部极限，也不能直接下发真机。** 英文：[REPORT.en.md](REPORT.en.md)。

## 目录

- [检查结果](#检查结果)
- [这次播放代表什么](#这次播放代表什么)
- [真机部署前还需要什么](#真机部署前还需要什么)
- [文件与复核](#文件与复核)
- [官方依据](#官方依据)

## 检查结果

| 项目 | 实测结果 | 判断 |
| --- | --- | --- |
| 时长 | 旧记录 57.9500 秒 → 新文件 72.4375 秒 | 时间拉长 1.25 倍，等效 bag 4 倍速度 |
| TRON2 手臂速度 | 左腕 yaw 最紧：6.2047 rad/s，占模型 17.8 rad/s 的 34.86% | 通过模型限速 |
| Wuji 速度 | 左无名指 PIP 最紧：10.3810 rad/s，占模型 12.86 rad/s 的 80.72% | 通过模型限速 |
| 头部速度 | 最大约 0.000179 rad/s | 通过模型限速 |
| 关节角度 | 左腕 pitch 超 0.001473°；右腕 pitch 超 0.000152°；左中指 DIP 超 0.001635° | **3 轴严格检查未通过**，不能写成全通过 |
| 碰撞/穿透 | 45,194 个姿态 × 2717 对几何，严格检查 0 交叉、报告深度 0 mm | 通过离散检查，不是连续 CCD |
| Isaac 路径播放 | 全段 72.4375 秒、2175 个播放时刻；读回误差 <0.000001 rad，已检查渲染连杆随运动改变 | 通过记录路径回放验证，非新物理执行 |

速度检查使用分段线性位置轨迹的逐段精确斜率，覆盖整个插值段，而非文件内的平均梯度。线性插值在节点有速度跳变，因此**不能据此给出有限的连续加速度/jerk 上限**，平滑起停也未验收。

碰撞检查包含全部 13,909 个原始姿态、每段中点和 240 Hz 执行时间网格（含末端）。机器人有 75 个碰撞几何体；检查 2642 对机器人几何和 75 对桌面几何，同刚体/直接相邻关节的 133 对结构排除明确记录，**没有采用 Wuji 装配碰撞豁免**。三角网格离散交叉检查不证明精确实体间隙、采样间连续避碰或真实接触响应。基座 `base_Link` 世界 Z=0，桌面 Z=-0.45 m，间距固定 0.45 m。

## 这次播放代表什么

这是此前 5 倍 PhysX 实际关节记录的减速，不是之前仍有手内重叠的参考目标，也不是视频变速。新文件所有关节位置逐位保留，原生采样从 240 Hz 变为 192 Hz；`time` 已是执行秒，所以按 **1 倍文件时间**播放。

目前默认播放器用位置回放复现该路径，没有重新做动力学积分。若用控制器在 4 倍速度重新执行，跟踪误差和接触可能改变，必须重新记录实际状态并验收。现有结果也不证明任务成功或真实电机安全。

## 真机部署前还需要什么

| 顺序 | 要完成的工作 | 当前状态 |
| --- | --- | --- |
| 1 | 确认 TRON2、左右手型号/硬件版本/固件及 SDK 逐轴映射、方向、零点和单位；解决头部 yaw/pitch 编号冲突 | 待确认 |
| 2 | 按真实设备范围修正微小角度越界，生成速度/加速度受限且平滑起停的轨迹；重新检查插值碰撞 | 未完成 |
| 3 | 获得逐轴速度、加速度、额定/峰值电流或力矩、温度、持续负载限制和来源版本 | 待确认 |
| 4 | 标定实物基座、转接件、双手与环境；在跟踪误差和制动条件下复核碰撞 | 未完成 |
| 5 | 实现并测试急停、反馈超时、故障停机、限幅、跟踪误差保护及完整状态记录 | 未准备真机播放器 |
| 6 | 有人员监护，先单轴/单手/单臂，再低速低负载逐步验证；记录速度、电流/力矩、温度和错误码 | 未进行 |

**不能仅凭模型速度通过，就判断没有突破真实电机极限。** Wuji 文档的 2.0 A 是允许的控制硬上限，不能替代安装设备的额定电流；1000 Hz 是控制/通信频率，不是机械速度限制。模型 `effort` 或仿真力矩也不能直接换算成 Wuji 电流。真实额定参数、设备版本和映射见 [HARDWARE.md](HARDWARE.md) / [HARDWARE.en.md](HARDWARE.en.md)，未知项均保留待确认。

## 文件与复核

- 当前轨迹：`trajectories/retimed_actual_4x.npz`，SHA-256 `23b555d6a4bf6df33fa4da4e5905a10f9fe375f6447784edd44079e412bdfcd1`。
- [逐轴检查](evidence/motor_limits/joint_limits.csv)、[速度/角度结论](evidence/motor_limits/report.json)、[碰撞结论](evidence/collision/gate.json)、[覆盖范围](evidence/collision/coverage.json)、[完整路径播放验证](evidence/isaac_playback.json)。
- 来源与时间缩放：[configs/retiming.json](configs/retiming.json)；播放绑定：[configs/trajectory.json](configs/trajectory.json)。源 bag 和旧轨迹副本不在当前运行包中。

按 README 安装分析环境、下载运行资产后，在仓库根目录执行，输出目录必须未存在：

```bash
.venv-analysis/bin/python scripts/audit_retimed_trajectory.py --out ../tron2-retimed-check
python3 scripts/check_hardware_admission.py
python3 -m unittest discover -s tests -v
```

前两条命令当前预期退出码为 2：分别表示微小角度越界和缺少真实硬件准入证据。测试命令应通过。不要将非零准入结果忽略为成功。可用 `scripts/retime_recorded_trajectory.py --tracking /path/to/tracking.npz --out ../reproduced.npz` 从同哈希旧记录重建文件；旧记录只作为外部来源，不随当前运行包分发。

## 官方依据

- [Wuji 控制指南](https://docs.wuji.tech/docs/zh/wuji-hand/latest/control-guide/)：角度/速度单位、A 电流单位、2.0 A 控制硬上限；文档尚未提供 SDK S1–S4 与模型执行器的轴序对应。
- [Wuji 用户须知](https://docs.wuji.tech/docs/zh/wuji-hand/latest/user-notice/)：Beta 设备的负载和温升边界。
- [TRON2 用户手册](https://www.limxdynamics.com/zh/documents/844648486841487360)、[TRON2 SDK 指南](https://www.limxdynamics.com/zh/documents/847884267345285120)：设备性能、关节编号和状态单位。末端指标不能替代逐轴电机限制。
