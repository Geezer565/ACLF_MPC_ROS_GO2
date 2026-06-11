# 代码架构

## 控制流程

```
cmd_vel / MoveBase Goal
        │
        ▼
┌──────────────────────────────┐
│ NMPC (OCS2 SQP, 100Hz)       │
│ SRBD 单刚体模型               │
│ + 摩擦锥 + 步态接触            │
│ + [ACLF] CLF 软约束           │
│ → optimizedState, optimizedInput
└──────────────┬───────────────┘
               ▼
┌──────────────────────────────┐
│ WBC 层级 QP (400Hz)           │
│ 优先级: swingLeg > baseAccel  │
│         > contactForce        │
└──────────────┬───────────────┘
               ▼
┌──────────────────────────────┐
│ Kalman Filter (100Hz)         │
│ IMU + 足端接触 → base位姿速度  │
└──────────────┬───────────────┘
               ▼
         关节力矩 → Gazebo/真机
```

## 核心包职能

| 包 | 作用 | 核心文件 |
|----|------|----------|
| `legged_controllers` | 主控制循环 + ACLF 更新 | `LeggedController.cpp` |
| `legged_wbc` | 层级 QP → 12维关节力矩 | `HierarchicalWbc.cpp` |
| `legged_estimation` | Kalman Filter 状态估计 | `LinearKalmanFilter.cpp` |
| `legged_gazebo` | 仿真 HW 接口 | `LeggedHWSim.cpp`, `default.yaml` |
| `legged_interface` | OCS2 ↔ ros_control 桥接 | `LeggedInterface.cpp` |
| `legged_interface/adaptive/` | ★ 自适应估计器 (策略模式) | 见下方 |

## ACLF 自适应估计器架构

```
AdaptiveEstimatorBase  (抽象接口)
├── update(state, stateDes, dt) → void
├── getOutput() → {adaptiveForce, adaptiveTorque, sigma}
└── reset()

    ├── AdaptiveEstimatorLegacy  (论文 A)
    │   ├── pi_hat[16] + Gamma[16×16] + Yu[6×16]
    │   └── pi_hat += Gamma·Yu^T·sigma·dt

    └── AdaptiveEstimatorRbf     (论文 B)
        ├── Dual-RBFNN: centers_ + widths_ + weights_
        └── W += -Gamma·sigma·h(chi)^T·dt
```

**模式切换**: `task.info` → `adaptiveMode "off" | "legacy" | "rbf"`

## ACLF 数据流

```
LeggedController::update()
  │
  ├─1. updateStateEstimation()  → measuredRbdState_
  ├─2. mpcMrtInterface_->evaluatePolicy()
  │     └─ [OCS2] CLF 约束读取 estimator 输出的 wrench 估计
  ├─3. estimatorPtr_->update(state, stateDes, dt)
  │     ├─ Legacy: pi_hat += Gamma·Yu^T·sigma·dt → f_u, t_u
  │     └─ RBF:    chi→RBFNN→wrench, W += -Gamma·sigma·h^T·dt
  ├─4. setAdaptiveWrench(f_u, t_u) → LeggedRobotPreComputation
  ├─5. wbc_->update() → 12维关节力矩
  └─6. 下发力矩
```

**注意**: estimator 在 evaluatePolicy() **之后**更新 — MPC 求解用的是上一周期的估计值，这是标准的自适应MPC做法。

## Go2 物理参数

| 参数 | 值 | 参数 | 值 |
|------|-----|------|-----|
| 躯干质量 | 6.921 kg | 大腿/小腿长 | 0.213 m |
| 髋扭矩极限 | 23.7 Nm | 膝扭矩极限 | 35.55 Nm |
| 髋角度极限 | ±1.047 rad | 膝角度极限 | [-2.723, -0.838] rad |

关节命名: `{L/R}{F/H}_{HAA/HFE/KFE}`，足端 `{L/R}{F/H}_FOOT`

## ACLF 新增/修改文件

### 新增 (策略模式 + RBF)

| 文件 | 说明 |
|------|------|
| `legged_interface/.../adaptive/AdaptiveEstimatorBase.h/.cpp` | 抽象接口 + 共用 σ 计算 |
| `legged_interface/.../adaptive/AdaptiveEstimatorLegacy.h/.cpp` | Legacy 16参数包装器 |
| `legged_interface/.../adaptive/AdaptiveEstimatorRbf.h/.cpp` | RBFNN 估计器 |
| `legged_interface/.../adaptive/RbfClfConstraint.h/.cpp` | RBF CLF OCS2约束 |

### 修改

| 文件 | 改动 |
|------|------|
| `LeggedInterface.h/.cpp` | +estimatorPtr_ +模式setup |
| `LeggedController.h/.cpp` | +updateAdaptiveEstimator() |
| `task.info` | +adaptiveMode +rbf配置段 |
| `CMakeLists.txt` | +adaptive/源文件 |

## 关键 ROS 话题

| 话题 | 用途 |
|------|------|
| `/cmd_vel` | 速度指令 (键盘→控制器) |
| `/joint_states` | 关节状态 (控制器→监控) |
| `/legged_robot_mpc_observation` | MPC 观测值 (调试) |
| `/controller_manager/switch_controller` | 控制器启停 |
