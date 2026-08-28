# 历史仓库状态

## 用途

本仓库保留早期 Go2 控制框架、Gazebo 接口、硬件接口和旧版 ACLF/RBF 尝试，仅用于追溯和参考。

当前 MyRBF 工作已经迁移到：

[`ACLF_MPC_MyRBF_Legged_Robot`](https://github.com/Geezer565/ACLF_MPC_MyRBF_Legged_Robot)

## 2026-08-28 清理

清理前状态已保存为标签：

```text
legacy-before-cleanup-20260828
```

本次从主分支移除了以下不应长期提交的内容：

- `.catkin_tools/` 本机编译配置与包索引；
- `.catkin_workspace` 本机工作区标记；
- `.disabled_pkgs/` 停用的重复 OCS2/RaiSim 包；
- 两个隐藏的论文文本缓存；
- `CLAUDE.md` 本机助手说明。

旧版 Go2 主体源码、接口、脚本和原有技术文档暂时保留，避免清理时误删可用于后续 Go2 迁移的历史信息。

## 使用边界

- 不要从本仓库复制同名自适应包覆盖当前仓库。
- 本仓库中的 Go2 框架不证明当前 MyRBF 已经完成 Go2 迁移。
- 新实验、新结果和 MyRBF 修改只进入当前仓库。
- 如需恢复清理前文件，从 `legacy-before-cleanup-20260828` 标签读取。
