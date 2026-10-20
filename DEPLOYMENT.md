# 离线检查与真机准备

本仓库提供 Isaac 目标显示和物理跟踪；没有真机控制入口，未通过真机准入。 English: [DEPLOYMENT.en.md](DEPLOYMENT.en.md).

## 目录

- [离线检查](#离线检查)
- [需要确认的资料](#需要确认的资料)
- [试运行顺序](#试运行顺序)

## 离线检查

先按 [README.md](README.md) 下载资源，再在仓库根目录运行。示例输出和环境目录必须尚不存在，且位于仓库外：

```bash
python3 scripts/verify_bundle.py
python3.12 -m venv ../tron2-analysis-env
../tron2-analysis-env/bin/pip install -r requirements-analysis.txt
../tron2-analysis-env/bin/python scripts/audit_retimed_trajectory.py --limits-only --out ../tron2-limits-check
../tron2-analysis-env/bin/python scripts/audit_target_velocity.py --out ../tron2-target-check
python3 scripts/check_hardware_admission.py
../tron2-analysis-env/bin/python -m unittest discover -s tests -v
```

完整性校验、目标速度检查和 8 项测试应通过。严格位置检查预期退出 `2`（继承的微小越限约 0.001473°）；硬件资料检查也预期退出 `2`。这些命令不连接设备。手指目标与旧版一致，但本次实际跟踪最大误差为 34.24°，臂部高频指标增加约 24%，需要检查节奏和跟踪质量。

## 需要确认的资料

| 文件 | 待确认内容 |
| --- | --- |
| `configs/hardware_inventory.json` | TRON2、左右 Wuji 的型号、固件、编号及证据 |
| `configs/joint_mapping.csv` | 56 轴 SDK 名称/编号、方向、零偏和单位 |
| `configs/hardware_limits.csv` | 逐轴位置、速度、加速度、电流、力矩、温度和负载限制及来源版本 |

16 维机体轴序尚未确认；手部源消息有名称，但不证明真机 SDK 映射。未知项不使用模型值代替设备额定值。

## 试运行顺序

1. 确认设备、轴序、当前轨迹哈希和初始姿态；基座比桌面高 0.45 m。
2. 处理位置越限、起停、制动、加速度/jerk、物理跟踪偏差和接触；重新检查碰撞与限位。288 Hz 是文件采样率。
3. 实现反馈超时、限幅、故障/温度监测及急停；先验证只读反馈和停止流程。
4. 设备负责人批准后，从单轴、单手/单臂、低速分段到低速全段逐级测试；运行记录写在仓库外。
