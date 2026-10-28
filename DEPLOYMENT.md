# 物理复播说明

本页说明如何从轨迹参考重新运行 Isaac Sim / PhysX，以及如何理解新运行结果。普通播放会读取已验收的实际状态轨迹；只有显式启用物理模式才会开始新的仿真。英文版：[DEPLOYMENT.en.md](DEPLOYMENT.en.md)。

## 前置条件

- 已按 [README.md](README.md) 下载并通过 `python3 scripts/verify_bundle.py` 校验运行资产。
- 已安装 Isaac Sim 5.1，并能在当前主机启动其 Python 解释器；需使用兼容 GPU。
- 输出目录由回放工具在仓库外新建。保留输出和日志以便检查，不要把新运行结果覆盖成已验收结果。

## 运行新的 PhysX 回放

在仓库根目录执行，将路径替换为本机 Isaac Sim Python 解释器：

```bash
python3 scripts/replay.py --isaac-python /path/to/isaac/python --physics
```

`--physics` 会从参考轨迹产生新的 PhysX 运行与实际状态记录，使用 480 Hz 和发布配置中的求解器、驱动器及官方装配碰撞规则。它需要较多计算资源，耗时取决于设备。查看新生成报告、轨迹和日志；命令完成或视频看起来正常，都不表示新结果已验收。

默认模式不推进物理，只回放绑定到已验收实际状态的记录路径。新物理模式与默认模式输出到仓库外的运行目录，不修改发布资产，也不应再对轨迹施加额外 5× 时间缩放。

## 结果解释

- 本次已验收轨迹覆盖源 bag 中有 body feedback 的 344.718 s，对应 68.944 s 的 5× 执行时间；原 bag 开头约 2.494 s 不在覆盖范围内。
- 已有综合验收针对指定 USD、Isaac / PhysX 参数、模型和几何对口径。新运行必须重新检查输入绑定、实际状态、速度、腕间距、拇指运动、接触与几何结果，不能继承旧验收结论。
- dense mesh 检查是采样证据，不是连续碰撞检测（CCD）证明。桌面到物理基座原点固定为 0.45 m。
- 离线 RNEA 力矩代理比值为 1.251，超过 URDF effort 限值。当前资料也没有确认真实 body-to-actuator mapping。因此，这些工具和结果不提供真机控制、硬件准入或扭矩通过结论。

若需逐项审阅当前已验收运行，查看 `validation/acceptance_155.json`、`validation/actual_mesh_133.json`、`validation/physics_report.json` 与 `metadata/` 中的绑定记录。
