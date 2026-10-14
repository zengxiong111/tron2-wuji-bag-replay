# 真机部署准备

本分支模型速度检查未通过：记录路径2个手部关节略超限，物理目标8个手部关节超限。角度也有3轴轻微越界；以上离线检查及真机准入均预期返回退出码2。

本5倍分支提供记录路径回放和显式PhysX重新执行，**没有真机控制程序，也未通过真机准入**。以下步骤用于交接后继续准备。英文：[DEPLOYMENT.en.md](DEPLOYMENT.en.md)。

## 目录

- [离线检查](#离线检查)
- [设备资料](#设备资料)
- [部署顺序](#部署顺序)

## 离线检查

先按 [README.md](README.md) 下载资产。在仓库根目录执行：

```bash
python3 scripts/verify_bundle.py
python3.12 -m venv ../tron2-analysis-env
../tron2-analysis-env/bin/pip install -r requirements-analysis.txt
../tron2-analysis-env/bin/python scripts/audit_retimed_trajectory.py --limits-only --out ../tron2-limits-check
../tron2-analysis-env/bin/python scripts/audit_target_velocity.py --out ../tron2-target-speed-check
python3 scripts/check_hardware_admission.py
../tron2-analysis-env/bin/python -m unittest discover -s tests -v
```

`../tron2-limits-check` 必须是仓库外的新目录。当前路径检查预期退出码为 `2`（2个手部关节速度略超限，3轴角度轻微越界）；目标速度检查也为 `2`（8个手部关节超限），硬件检查为 `2`（资料不完整）。完整性校验与单元测试应通过。硬件检查只核对离线资料，不连接设备。

如需重新计算完整离散碰撞检查，使用同一环境执行以下命令；计算耗时较长，结果写入新的外部目录：

```bash
../tron2-analysis-env/bin/python scripts/audit_retimed_trajectory.py --out ../tron2-full-check
```

## 设备资料

由设备负责人补齐以下文件。未知项保持空白，不要用模型值冒充实机参数。

| 文件 | 需要确认 |
| --- | --- |
| `configs/hardware_inventory.json` | TRON2、左手、右手的型号、硬件版本、固件、编号及证据 |
| `configs/joint_mapping.csv` | 56 轴的 SDK 名称/编号、方向、零偏和单位 |
| `configs/hardware_limits.csv` | 逐轴位置、速度、加速度、电流、力矩、温度和持续负载限制，以及来源版本 |

源 bag 的机身状态有 16 个无名称通道，顺序仍待确认，头部 yaw/pitch 顺序存在冲突：候选顺序为 yaw、pitch，手册编号为 14=pitch、15=yaw。手部有 40 个命名位置通道，但名称匹配不能证明当前实物的 SDK 映射正确。`bag_hand_index` / `bag_pos_index` 是嵌套手序号 / 位置序号；`bag_raw_index` 是展开后的 40 维序号。

## 部署顺序

1. **确认设备和映射。** 核实上表资料，解决机身通道顺序和手部执行器对应关系。
2. **准备可执行轨迹。** 修复模型速度及角度越界，加入平滑起停和加速度/jerk 限制，再按真实控制周期重采样。文件的 240 Hz 采样率不代表硬件控制频率。重新检查限制、碰撞及物理跟踪，并更新轨迹和报告哈希。
3. **检查现场和初始姿态。** 桌面到物理基座原点固定为 0.45 m，不能移动安装高度来消除碰撞。检查转接件、线缆、负载、障碍，以及首帧过渡和制动空间。
4. **实现真机控制程序。** 使用确认的 SDK，加入逐轴限幅、反馈超时、跟踪误差监测、故障停机与急停。先验证只读反馈和故障处理；参数及增益必须依据真实设备确认。
5. **逐级试运行。** 设备负责人批准后，依次检查单轴、单手、单臂、低速分段及完整轨迹，最后才验证目标动作速度。记录实际状态、电流/力矩、温度、故障码和时间戳，保存到仓库外。

映射错误、反馈丢失、越限、异常温升或意外接触时，按厂家流程停机；解决原因后再继续。参数、负载、装配、控制器或轨迹变化后重新验证受影响项。

真机验收需要设备负责人确认资料、映射、限制、起停、制动和完整运行结果。Isaac 视频和离线表格不能代替实机验收；当前分段线性路径的加速度/jerk 尚未受限，减速本身不能解决这些问题。
