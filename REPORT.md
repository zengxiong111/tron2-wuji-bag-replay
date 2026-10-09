# 4 倍轨迹检验报告

**结论：模型速度和目标角度通过；真实电机全部极限尚未验证。** 这份报告对应当前仓库的 4 倍、头部朝前轨迹，核查日期为 2026-10-09。

英文：[REPORT.en.md](REPORT.en.md)。

## 目录

- [已通过](#已通过)
- [尚未验证](#尚未验证)
- [如何复核](#如何复核)
- [官方依据](#官方依据)

## 已通过

“占上限比例”越接近 100%，速度余量越小。下面的逐轴上限来自交付 URDF/USD 模型，不能自动当成实际设备的额定值。

| 检查 | 结果 | 判断 |
| --- | --- | --- |
| TRON2 手臂目标速度 | 最快轴 6.182 rad/s，上限 17.8 rad/s，占 34.73% | 通过模型限速 |
| Wuji 目标速度 | 最紧轴为左小指 PIP：11.888 rad/s，上限 12.86 rad/s，占 92.44% | 通过模型限速，余量约 7.56% |
| 实际物理状态速度 | 全部 56 轴未超模型限速，最大占 88.33% | 通过；由 240 Hz 位置差分估计 |
| 目标角度 | 全部 56 轴的完整三次插值未超模型及官方文档角度范围 | 通过 |
| TRON2 腕部参考点目标速度 | 左/右 1.000 / 0.969 m/s，手册末端指标 5 m/s | 仅参考点测量，不作末端验收 |
| TRON2 腕部参考点目标加速度 | 左/右 14.025 / 14.507 m/s²，手册末端指标 36 m/s² | 仅参考点测量，不作末端验收 |
| 实际状态穿透 | 全段 17,386 状态 × 2,717 对几何，未发现 >0.1 mm 穿透 | 通过离散检查 |
| 去抖改善 | 关节高频 RMS 降低 78.98%，腕部降低 67.01%，加速度 P95 降低 84.44% | 相对同条件原始 4 倍重放改善 |

目标速度使用三次插值每段的解析极值，包含采样点之间的峰值。腕部指标为 `wrist_roll_L_Link` / `wrist_roll_R_Link` 原点的 240 Hz FK 差分，实际工具末端可能不同，且未证明连续时间峰值。

## 尚未验证

| 项目 | 为什么不能判断 |
| --- | --- |
| 真机逐轴额定/峰值速度、加速度 | 尚未取得与具体设备版本绑定的完整参数；模型限速不能代替额定值 |
| 电流、力矩、带载能力 | 新轨迹记录没有 4 倍真机电流/力矩/负载测量；原 bag 的原速反馈不能证明加速后的表现 |
| Wuji 电流 | 2.2 Beta 控制文档写明 `effort` 单位为 A，允许硬上限 2.0 A；本次没有可比的 4 倍电流记录，不能把仿真 N·m 当作 A |
| 温升和连续运行 | Wuji 文档说明 Beta 样机长期满载温升与负载能力尚未形成承诺；本次没有温度测量 |
| 关节映射 | bag 的 16 维机身没有名称。手册为 14=pitch、15=yaw，与此前暂定头部顺序相反；本版两轴目标均为零。Wuji SDK S1–S4 与模型执行器对应关系也需设备确认 |
| 无接触目标与连续避碰 | 参考目标仍有手内重叠，最大 13.301 mm；通过的是物理接触约束后的实际状态。不是 CCD 证明，不能把参考直接用于无碰撞运动学播放 |

因此当前发布定位为 **Isaac 仿真轨迹**。要确认真机可执行，还需要设备版本、逐轴参数和已确认映射，以及该轨迹下的速度、电流/力矩、温度和错误码记录。仿真驱动增益不能直接复制到真机。

## 如何复核

1. 先按 [README.md](README.md) 下载运行文件并校验哈希。
2. 在分析环境运行以下命令，报告写入仓库外的新目录：

```bash
.venv-analysis/bin/python scripts/check_motion_limits.py --out ../tron2-4x-checks
.venv-analysis/bin/python scripts/122_audit_replay_motor_velocity.py --out ../tron2-4x-speeds
```

简明报告是本文件；可供程序复核的结果在 [电机检查](evidence/motor_checks/report.json)、[逐轴角度/速度表](evidence/motor_checks/joint_limits.csv)、[速度详细表](evidence/final/motor_limit_audit/joint_speeds.csv)、[碰撞结论](evidence/final/audit/gate.json)、[去抖指标](evidence/final/temporal_quality.json)。

## 官方依据

- [Wuji 控制指南](https://docs.wuji.tech/docs/zh/wuji-hand/latest/control-guide/)：关节单位、电流上限、SDK 轴序边界；1000 Hz 是通信/控制频率，不是机械速度上限。
- [Wuji 用户须知](https://docs.wuji.tech/docs/zh/wuji-hand/latest/user-notice/)：负载和温升的 Beta 能力边界。
- [TRON2 用户手册](https://www.limxdynamics.com/zh/documents/844648486841487360)：1.2 的末端性能、1.5 的关节角度和编号。
- [TRON2 SDK 指南](https://www.limxdynamics.com/zh/documents/847884267345285120)：q 为 rad、dq 为 rad/s、tau 为估计 N·m。示例控制频率不是电机速度上限。
