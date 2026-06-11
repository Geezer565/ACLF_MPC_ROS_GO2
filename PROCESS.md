# 迁移进度

> Go2 ACLF-MPC → VAN-MPC | 分支: `feature/van-mpc-migration`

**当前状态**: Phase 1 — 双模式架构已实现，待编译测试

## 已完成

- 策略模式: `AdaptiveEstimatorBase` → `Legacy` (论文A) / `Rbf` (论文B)
- 三种模式切换: `task.info` → `adaptiveMode "off" | "legacy" | "rbf"`
- 向后兼容: 旧 `useAclf=true` 路径保留

## 当前架构

```
task.info: adaptiveMode
  "off"    → 名义 MPC (无自适应)
  "legacy" → AdaptiveEstimatorLegacy (16参数 Slotine-Li)
  "rbf"    → AdaptiveEstimatorRbf (Dual-RBFNN 骨架)
```

## Phase 路线

| Phase | 内容 | 状态 |
|-------|------|------|
| 1 | Dual-RBFNN 替换 Y_u 回归矩阵 (固定Γ) | 架构✓ 待编译 |
| 2 | 复合误差 E_c = γ·E_e + (1−γ)·σ | 待实现 |
| 3 | 变步长 Γ(χ) | 待实现 |
| 4 | 多地形仿真验证 | 待实现 |

## 关键决策

1. **修改基础**: 基于旧版 ACLF 封装 (已集成, 零风险)，非新版独立包
2. **复合误差**: Phase 1 用纯 σ, Phase 2 引入 E_c
3. **RBF 规模**: m=10 (21 centers), inputDim=12

## 下一步

1. 编译验证: `catkin build legged_interface legged_controllers`
2. 修复编译错误
3. 仿真测试三种模式
4. 建立性能基准数据

## 参考

- [TARGET.md](TARGET.md) — 完整迁移方案 (数学推导 + 路线图)
- [ARCHITECTURE.md](ARCHITECTURE.md) — 代码架构
- [ALGORITHM.md](ALGORITHM.md) — ACLF 算法
