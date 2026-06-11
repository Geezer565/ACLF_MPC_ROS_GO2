# PROCESS — VAN-MPC 迁移过程记录

> 项目: Go2 四足机器人 ACLF-MPC → VAN-MPC 迁移
> 开始日期: 2026-06-10
> 当前状态: **Phase 1 — 双模式架构已实现，待编译测试**

---

## 会话记录

### 2026-06-10 — 初始方案设计

**背景**:
- 当前项目使用论文 A (Minniti et al., 2021) 的 ACLF-MPC 框架控制 Go2 四足机器人
- 论文 B (Liu et al., 2025) 提出了更先进的自适应 MPC 框架 (VAN-MPC)，应用于球形机器人
- 目标: 将论文 B 的核心算法迁移到四足平台

**已完成**:
1. 提取并保存两篇论文的文本内容:
   - `.paper1_ACLF_MPC_Quadruped.txt` — 论文 A 全文
   - `.paper2_VAN_MPC_Spherical.txt` — 论文 B 全文
2. 分析当前项目架构（详见 `01_OVERVIEW.md` ~ `05_TUNING_GUIDE.md`）
3. 撰写迁移方案 `TARGET.md`，包含:
   - 两篇论文的数学框架对比
   - 四个关键创新点的四足适配推导
   - 分阶段实现路线图
   - 代码架构变更总览
   - 数学一致性检查清单

**关键发现**:
- 当前项目已有两套 ACLF 实现:
  - **旧版**: 嵌入式（ocs2_legged_robot/adaptive/），16维物理参数+6×16回归矩阵，通过 legged_interface 桥接
  - **新版**: 独立包（ocs2_legged_robot_adaptive/），6维wrench直接估计，外挂式模块，**尚未接入运行**
- 论文 B 的创新可以直接应用于四足，核心是:
  1. 不确定性转换: 16维物理参数×回归矩阵 → 6维 RBFNN 直接输出
  2. 复合误差: E_c = γ·E_e + (1−γ)·σ
  3. 变步长: Γ 由 RBFNN 自身状态自适应决定
- 论文 B 的球形机器人 Δu 是 2 维的（速度和横滚角补偿），四足上是 6 维的（基座 wrench 补偿），但框架完全一致

**下一步**:
- Phase 1: 实现 Dual-RBFNN 自适应估计器，替换 Y_u 回归矩阵
- 需确认: 新版 ocs2_legged_robot_adaptive 是否可作为更好的修改基础（比旧版嵌入式的耦合更少）

---

### 2026-06-11 — 双模式共存架构实现

**目标**: 封装旧版 ACLF 代码，设计策略模式支持三种模式切换（off / legacy / rbf），确保新旧代码可以对比实验。

**已完成**:

1. **Git 保存与分支**:
   - `main` 分支: commit `e4e082a` 保存迁移前快照
   - 创建 `feature/van-mpc-migration` 分支
   - commit `5a1615c`: 双模式架构实现 (14 files, +1370 lines)

2. **新增 9 个文件** (策略模式架构):
   - `AdaptiveEstimatorBase.h/.cpp` — 抽象接口 + computeCompositeError 共用实现
   - `AdaptiveEstimatorLegacy.h/.cpp` — 包装现有 16 参数 Slotine-Li 代码
   - `AdaptiveEstimatorRbf.h/.cpp` — Dual-RBFNN 在线学习骨架
   - `RbfClfConstraint.h/.cpp` — 使用 RBFNN wrench 估计的 OCS2 CLF 约束

3. **修改 5 个文件**:
   - `LeggedInterface.h/.cpp` — 添加 estimatorPtr_ + 三种模式 setup ("off"/"legacy"/"rbf")
   - `LeggedController.h/.cpp` — updateAdaptiveEstimator() 统一入口，向后兼容旧 useAclf_ 路径
   - `task.info` — 添加 adaptiveMode 选项 + rbf/rbfSoftConstraint 配置段
   - `CMakeLists.txt` — 添加新的 adaptive/ 源文件

4. **架构设计要点**:
   - 策略模式: `AdaptiveEstimatorBase` → `Legacy` / `Rbf` 两种实现
   - 统一接口: `update()` → `getOutput()` → WBC 补偿
   - 约束分离: Legacy 用旧 `AdaptiveClfConstraint`，RBF 用新 `RbfClfConstraint`
   - 向后兼容: `useAclf_=true` 原始路径保留，`adaptiveMode_` 新路径优先

5. **配置切换方法**:
   ```ini
   # task.info — 三种模式:
   adaptiveMode  "off"      # 原始名义 MPC
   adaptiveMode  "legacy"   # 论文 A: 16参数 Slotine-Li (封装版)
   adaptiveMode  "rbf"      # 论文 B: Dual-RBFNN (新增)
   ```

**待做**:
- 编译验证 (`catkin build legged_interface legged_controllers`)
- 修复编译错误 (预计: 命名空间、头文件路径、API 不匹配)
- 仿真测试: 三种模式切换 + 性能对比
- 记录性能基准数据

**关键决策**:

### 决策 1 (已确定): 修改基础 — 使用旧版 ACLF 代码封装

**结论**: 采用选项 A — 基于旧版 ACLF (ocs2_legged_robot/adaptive/) 代码封装。

**理由**:
- 旧版已集成在 legged_interface/legged_controllers 中，可直接运行
- 新版 ocs2_legged_robot_adaptive 尚未接入 Go2 (需额外工作)
- 封装方式 (AdaptiveEstimatorLegacy wrapper) 不影响旧代码，零风险
- 后续可逐步迁移到新版独立包

### 决策 2 (已确定): 复合误差 γ 初值

**结论**: Phase 1 使用纯 σ (γ=0)，Phase 2 引入复合误差 E_c。

### 决策 3 (已确定): RBF 基函数数量 m

**结论**: m=10 (21 centers), inputDim=12。待实验调优。

---

## 决策记录

### 决策 1: 修改基础 — 使用旧版 ACLF 还是新版 ocs2_legged_robot_adaptive?

**状态**: 待决定 (Phase 1 开始时确认)

**选项 A** — 基于旧版 ACLF (ocs2_legged_robot/adaptive/)
- 优点: 已在运行，与 legged_interface/legged_controllers 耦合已完成
- 缺点: 代码侵入性强，修改分散在多个文件中

**选项 B** — 基于新版 ocs2_legged_robot_adaptive
- 优点: 独立包，SolverSynchronizedModule 接口更干净，机器人无关
- 缺点: 尚未接入 Go2，需先完成基础接入

**倾向**: 选项 B — 新版架构的 SolverSynchronizedModule 回调机制天然适合嵌入 RBFNN 前向推理，且不污染原有代码。

---

### 决策 2: 复合误差中 γ 的初值

**状态**: 待实验确定

**候选值**: γ = 0.7 (偏向模型学习) 或 γ = 0.5 (等权重)

**依据**: 论文 B 强调 γ > 0.5，且在实验中 γ=0.7 取得了最好效果

---

### 决策 3: RBF 基函数数量 m

**状态**: 待实验确定

**候选值**: m = 10 (21 个中心), m = 15 (31 个中心), m = 20 (41 个中心)

**考虑因素**: 计算复杂度 vs 表示能力。WBC 以 400Hz 运行，RBFNN 推理需在 2.5ms 内完成。

---

## 遇到的问题 & 解决方案

| 日期 | 问题 | 状态 | 解决方案 |
|------|------|------|----------|
| 2026-06-10 | 方案设计完成，待开始实现 | 进行中 | 见 TARGET.md Phase 1 |

---

## Git 分支

| 分支 | 说明 |
|------|------|
| `main` | 原始代码 (e4e082a 快照) |
| `feature/van-mpc-migration` | **当前工作分支** (5a1615c 双模式架构) |

---

## 文件变更记录

| 日期 | 文件 | 操作 | 说明 |
|------|------|------|------|
| 2026-06-10 | `TARGET.md` | 新建 | 迁移方案文档 |
| 2026-06-10 | `PROCESS.md` | 新建 | 本文件 |
| 2026-06-10 | `.paper1_ACLF_MPC_Quadruped.txt` | 新建 | 论文 A 全文文本 |
| 2026-06-10 | `.paper2_VAN_MPC_Spherical.txt` | 新建 | 论文 B 全文文本 |
| 2026-06-11 | `legged_interface/.../adaptive/AdaptiveEstimatorBase.h` | 新建 | 抽象估计器接口 |
| 2026-06-11 | `legged_interface/.../adaptive/AdaptiveEstimatorLegacy.h` | 新建 | Legacy 估计器 (16参数) |
| 2026-06-11 | `legged_interface/.../adaptive/AdaptiveEstimatorRbf.h` | 新建 | RBF 估计器 (Dual-RBFNN) |
| 2026-06-11 | `legged_interface/.../adaptive/RbfClfConstraint.h` | 新建 | RBF CLF 约束 (OCS2) |
| 2026-06-11 | `legged_interface/src/adaptive/AdaptiveEstimatorBase.cpp` | 新建 | computeCompositeError 实现 |
| 2026-06-11 | `legged_interface/src/adaptive/AdaptiveEstimatorLegacy.cpp` | 新建 | Legacy 估计器实现 |
| 2026-06-11 | `legged_interface/src/adaptive/AdaptiveEstimatorRbf.cpp` | 新建 | RBFNN 前向+权值更新 |
| 2026-06-11 | `legged_interface/src/adaptive/RbfClfConstraint.cpp` | 新建 | RBF CLF 约束实现 |
| 2026-06-11 | `legged_interface/include/.../LeggedInterface.h` | 修改 | +estimatorPtr_ +getAdaptiveEstimator |
| 2026-06-11 | `legged_interface/src/LeggedInterface.cpp` | 修改 | +rbf/legacy mode setup |
| 2026-06-11 | `legged_controllers/.../LeggedController.h` | 修改 | +estimatorPtr_ +updateAdaptiveEstimator |
| 2026-06-11 | `legged_controllers/src/LeggedController.cpp` | 修改 | +策略模式分支 +updateAdaptiveEstimator |
| 2026-06-11 | `legged_interface/CMakeLists.txt` | 修改 | +4 个 adaptive/ 源文件 |
| 2026-06-11 | `legged_controllers/config/go2/task.info` | 修改 | +adaptiveMode +rbf 配置段 |

---

## 编译 & 测试记录

_暂无 — 代码实现尚未开始_

---

## 性能基准 (迁移前)

_待测量 — Phase 1 前需建立以下基准数据:_

- [ ] 名义 MPC (useAclf=false) 在平坦地面的位姿跟踪 RMSE
- [ ] 旧版 ACLF (useAclf=true) 在平坦地面的位姿跟踪 RMSE
- [ ] 携带 5kg 额外负载时的跟踪性能
- [ ] MPC 求解时间 (avg/max)
- [ ] WBC 求解时间 (avg/max)
- [ ] 单次 updateAdaptiveParams() 耗时
