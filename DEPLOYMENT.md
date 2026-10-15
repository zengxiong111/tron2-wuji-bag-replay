# 真机部署准备

本仓库提供参考显示和物理跟踪，**没有真机控制程序，未通过真机准入**。English: [DEPLOYMENT.en.md](DEPLOYMENT.en.md).

## 目录

- [离线检查](#离线检查)
- [需要补齐的资料](#需要补齐的资料)
- [试运行顺序](#试运行顺序)

## 离线检查

先按 [README.md](README.md) 下载资产，再在仓库根目录执行。输出目录必须是仓库外的新目录：

```bash
python3 scripts/verify_bundle.py
python3.12 -m venv ../tron2-analysis-env
../tron2-analysis-env/bin/pip install -r requirements-analysis.txt
../tron2-analysis-env/bin/python scripts/audit_retimed_trajectory.py --limits-only --out ../tron2-limits-check
../tron2-analysis-env/bin/python scripts/audit_target_velocity.py --out ../tron2-target-check
python3 scripts/check_hardware_admission.py
../tron2-analysis-env/bin/python -m unittest discover -s tests -v
```

完整性校验、目标速度检查和 8 项测试应通过。严格位置检查预期退出 `2`，因为保留原 5x 最大约 0.001635° 的微小越限；硬件资料检查也预期退出 `2`。这些脚本不连接设备。物理实际仍有最大约 28.10° 的跟踪偏差，不能按参考保真度直接准入。

## 需要补齐的资料

| 文件 | 需要确认 |
| --- | --- |
| `configs/hardware_inventory.json` | TRON2、左右 Wuji 的型号、固件、编号及证据 |
| `configs/joint_mapping.csv` | 56轴 SDK 名称/编号、方向、零偏和单位 |
| `configs/hardware_limits.csv` | 逐轴位置、速度、加速度、电流、力矩、温度和负载限制及来源版本 |

16维身体顺序仍未确认，头部候选 yaw/pitch 与手册编号存在冲突；40个手部命名通道也不能证明当前实物 SDK 映射正确。未知项保持空白，不用模型值代替设备额定值。

## 试运行顺序

1. 确认设备、映射、轨迹哈希及初始姿态；安装高度差固定0.45m。
2. 准备满足真机限制的轨迹和控制器，处理微小位置越限、起停、制动、加速度/jerk及物理跟踪偏差，重新验算碰撞和限位。文件288 Hz不代表硬件控制频率。
3. 实现逐轴限幅、反馈超时、跟踪误差、故障/温度监测及急停，先验证只读反馈和停止流程。
4. 设备负责人批准后，依次进行单轴、单手/单臂、低速分段、低速全段测试，每级通过后再加速。
5. 映射错误、反馈丢失、越限、异常温升或意外接触时按厂家流程停机，不自动恢复；解决原因后重新验证。运行记录保存在仓库外。
