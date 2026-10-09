# 仓库指令

维护可移植、可复现的 ROS1 bag 与 Isaac 重放包。英文：[AGENTS.md](AGENTS.md)。

## 规则

- Markdown 简洁易懂，并提供完整英文与简体中文版本。
- 遵守 [TABLETOP_FRAME_CONTRACT.zh-CN.md](TABLETOP_FRAME_CONTRACT.zh-CN.md)：物理基座高于桌面恰好0.45m，不通过移动安装几何来通过碰撞检查。
- 运行二进制文件放在私有 GitHub Release。只保留当前必要资产，不向 Git 提交日志、备份、changelog或中间输出目录。
- 使用克隆目录相对路径，发布前核对资产 SHA-256绑定。
- 机身关节顺序未确认，离散碰撞、物理重放、CCD与真机结论必须分别表述。
