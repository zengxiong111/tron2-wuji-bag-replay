# 硬件身份、关节映射与限值

English: [HARDWARE.en.md](HARDWARE.en.md)。

本文定义 bag-replay 工具读取的硬件参数清单。当前清单用于阻止未经确认的真机播放；它不是设备配置或调试记录。

## 目录

- [状态与结论](#状态与结论)
- [本地证据确认项](#本地证据确认项)
- [必须补齐的设备信息](#必须补齐的设备信息)
- [配置文件](#配置文件)
- [准入规则](#准入规则)
- [官方参考](#官方参考)

## 状态与结论

**当前不能确认实际设备版本、固件、序列号、完整真机关节映射或逐轴硬件限值。** 本次交付只有减速轨迹、模型文件和源 bag 的元数据（原始 bag 不在当前运行包中）；没有设备身份记录、版本绑定参数表或操作员确认。不得把模型 URDF/USD 限值写成真机额定参数。

| 设备信息 | 当前状态 | 证据边界 |
| --- | --- | --- |
| TRON2 机身 | bag 有 16 个 `/motor/state` 的 `q` 通道；消息未声明名称 | 顺序与模型关节的关联仍是候选映射 |
| 头部通道 | 本地模型/bag候选顺序是 yaw、pitch；TRON2 手册编号为 14=pitch、15=yaw | 有冲突；此前目标两通道均为零；当前文件来自其实际反馈，仍不能确认映射 |
| Wuji Hand2 | bag 有 40 个命名位置通道；`left_`/`right_` 前缀对应模型的 `l_`/`r_` 名称 | 只确认 bag 名称归一化；与当前实物及 SDK 的 S1–S4/执行器编号映射未确认 |
| 位置/速度限值 | 配置包含交付模型的角度和速度限值 | 这些是模型/文档值，不是已确认的实物限值 |
| 加速度、额定电流、力矩、温升 | 未取得与实物版本绑定的逐轴值 | 不可用于真机准入 |

公开 Wuji 2.2 Beta 控制资料中的 `effort` 单位为 A，允许硬上限为 2.0 A。该值是控制边界，不是默认指令，也不是每轴额定电流。1000 Hz 是控制/通信频率，不是机械速度上限。TRON2 文档中的 `q`、`dq`、`tau` 分别为 rad、rad/s、估计 N·m；手册末端速度/加速度指标不能替代逐轴电机限制。

## 本地证据确认项

| 项目 | 已知事实 | 来源 |
| --- | --- | --- |
| 输入 bag | 源文件 `2026-10-08-17-54-13.bag`（当前包不分发）；SHA-256 `2af6c090b36e26231aff0b2b102768c7224044cd260b8bc07486aa4167fe55bd` | `configs/bag_info.json` |
| 机身通道 | 16 维 `/motor/state q`，无消息关节名 | `configs/bag_info.json` |
| 手部通道 | 40 维 `/wuji/hand/state hand_state.pos`，有源名称 | `configs/bag_info.json` |
| bag 设备标签 | 全部 44,058 条手部状态的 `hand_type` 均为 `wuji/hand2`，嵌套 frame 为 left/right | 原始 bag；`configs/hardware_inventory.json`。仅确认记录标签，未确认 Beta 版本、固件或当前实物 |
| 模型限值 | 按 56 个模型关节列出的角度与速度数据 | `evidence/motor_limits/joint_limits.csv` |

## 必须补齐的设备信息

在准入任何硬件播放器前，由设备负责人提供并复核以下信息：

1. TRON2 与每只 Wuji 手的准确产品变体、硬件修订、固件版本和资产编号/序列号。
2. 将 bag 通道及模型关节逐一映射到真机 SDK 关节名/执行器编号；TRON2 机身需确认 16 维顺序，头部 14/15 冲突须单独解决；手部需确认 40 个命名关节与 SDK S1–S4/执行器的关系。
3. 提供适用于该版本的逐轴位置、速度、加速度、电流及力矩限制，注明单位、持续/峰值定义、来源版本和文档页码。
4. 由有权限的设备负责人签署或在受控配置库登记确认记录；随后重新计算目标轨迹对硬件限制的准入结果。

本工作没有连接设备，也没有执行 SDK 查询、使能或运动。仅凭现有 bag 与公开资料不能完成上述确认。

## 配置文件

| 文件 | 用途 |
| --- | --- |
| `configs/hardware_inventory.json` | 设备身份、证据状态、公开资料边界及准入所需字段；`admission_ready` 当前为 `false` |
| `configs/joint_mapping.csv` | 56 个模型关节到 bag 原始通道、设备组及待确认的实物/SDK 映射。未确认字段为空，状态列明确标为候选或未确认 |
| `configs/hardware_limits.csv` | 56 个关节的模型角度/速度值与空白真机限值列。`hardware_limit_status` 当前全部为 `unconfirmed` |

`bag_hand_index` 与 `bag_pos_index` 标出 bag 内嵌套的手序号和位置序号；`bag_raw_index` 为展开的 40 维序号。真机方向、零偏、单位和证据来源单独留空，不能由名称匹配推断。左右手按独立设备记录身份；温度与持续负载条件也须补齐。

离线检查命令（仓库根目录；当前预期退出码为 `2`，因为设备证据尚未确认）：

```bash
python3 scripts/check_hardware_admission.py
```

该命令只核对离线参数表和轨迹证据完整性，不连接设备，也不会授权硬件执行。实机调试另列为 `live_commissioning_verified=false`。可运行 `python3 -m unittest discover -s tests -v` 检查未知值、非法数值、错误关节集合、设备错绑和通道索引是否被拒绝。

空白真机限值表示未知，不表示无限制或零限制。播放器准入必须将空值视为拒绝条件；不得回退到模型限值、通用手册性能或 2.0 A 控制硬上限。

## 准入规则

播放器只可在以下条件全部满足后进入单独授权的硬件流程：设备身份与映射已确认；所有实际命令通道都有版本绑定的逐轴限制和单位；目标轨迹按硬件限制重新检查；静态检查与受控低速调试记录分别通过；设备负责人批准运行。任何缺项都必须阻止使能和播放。

本配置仅建立未确认字段和准入边界；它不证明碰撞连续性、负载能力、温升、闭环任务成功或 Sim2Real。

## 官方参考

- [Wuji 控制指南](https://docs.wuji.tech/docs/zh/wuji-hand/latest/control-guide/)
- [Wuji SDK 参考](https://docs.wuji.tech/docs/zh/wuji-hand/latest/sdk-reference/)
- [Wuji 用户须知](https://docs.wuji.tech/docs/zh/wuji-hand/latest/user-notice/)
- [TRON2 SDK 指南](https://www.limxdynamics.com/zh/documents/847884267345285120)
- [TRON2 用户手册](https://www.limxdynamics.com/zh/documents/844648486841487360)
