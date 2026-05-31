# 02 — 代码架构

## 一、工作空间总览

```
go2_ros1_ws/
├── src/
│   ├── leggedcontrol_go2/        # ★ 主控制框架 (NMPC + WBC)
│   │   ├── legged_common/        #   共享硬件接口 (HybridJoint, ContactSensor)
│   │   ├── legged_controllers/   #   NMPC 控制器 + 轨迹发布器
│   │   ├── legged_estimation/    #   Kalman Filter 状态估计
│   │   ├── legged_gazebo/        #   Gazebo 仿真硬件接口
│   │   ├── legged_hw/            #   硬件抽象层
│   │   ├── legged_interface/     #   OCS2 ↔ ros_control 桥接
│   │   ├── legged_wbc/           #   全身 QP 控制器
│   │   ├── qpoases_catkin/       #   qpOASES 求解器
│   │   └── legged_examples/legged_unitree/
│   │       ├── legged_unitree_description/  # URDF/XACRO 模型
│   │       ├── legged_unitree_hw/           # 真机硬件接口
│   │       └── unitree_sdk2-main/           # Unitree SDK v2
│   ├── ocs2-main/                # ETH OCS2 最优控制库
│   ├── ocs2_robotic_assets/      # OCS2 模型资产
│   ├── pinocchio/                # 刚体动力学库
│   ├── hpp-fcl/                  # 碰撞检测
│   ├── grid_map/                 # 栅格地图
│   └── unitree_ros/              # Unitree 官方 ROS 包
├── build/  devel/  logs/
```

---

## 二、核心包职能

| 包 | 作用 | 核心文件 |
|----|------|----------|
| `legged_controllers` | NMPC 主控制器 + cmd_vel 轨迹发布 | `LeggedController.cpp`, `TargetTrajectoriesPublisher.cpp` |
| `legged_wbc` | 全身 QP 优化 → 12 维关节力矩 | 层级 QP 求解器 |
| `legged_estimation` | Kalman Filter 状态估计（位姿+速度） | `LinearKalmanFilter.cpp` |
| `legged_gazebo` | Gazebo 仿真 HW 接口 | `LeggedHWSim.cpp`, `default.yaml`, `empty_world.world` |
| `legged_interface` | OCS2 ↔ ros_control 双向桥接 | `LeggedInterface.h/.cpp` |
| `legged_hw` | 硬件抽象（真机/仿真共用） | `LeggedHW.cpp` |
| `legged_common` | 共享接口定义 | `HybridJointInterface.h`, `ContactSensorInterface.h` |

---

## 三、控制流程

```
cmd_vel / MoveBase Goal
        │
        ▼
┌─────────────────────────────────┐
│ TargetTrajectoriesPublisher     │  速度指令 → 状态轨迹
└───────────────┬─────────────────┘
                │
                ▼
┌─────────────────────────────────┐
│ NMPC (OCS2 SQP, 100Hz)          │
│ • 质心动力学 (SRBD)              │
│ • 摩擦锥约束                     │
│ • 步态调度接触                   │
│ • [ACLF] CLF 不等式约束          │  ← ★ 新增
│ • QP 子问题 → HPIPM             │
└───────────────┬─────────────────┘
                │ optimized state + forces
                ▼
┌─────────────────────────────────┐
│ WBC (层级 QP, 400Hz)            │
│ 优先级: swingLeg > baseAccel    │
│         > contactForce          │
└───────────────┬─────────────────┘
                │ joint torques (feed-forward + PD)
                ▼
┌─────────────────────────────────┐
│ Kalman Filter (100Hz)           │
│ 估计 base pos/vel                │
│ 输入: IMU + 足端接触              │
└───────────────┬─────────────────┘
                │
                ▼
┌─────────────────────────────────┐
│ Gazebo / Go2 真机               │
└─────────────────────────────────┘
```

---

## 四、ACLF-MPC 数据流

```
┌──────────────────────────────────────────┐
│        LeggedInterface (共享状态)          │
│  ┌────────────────────────────────────┐  │
│  │ AdaptiveParams                     │  │
│  │  ├── pi_hat[16]    ← 控制器更新     │  │
│  │  ├── Gamma[16×16]                  │  │
│  │  ├── Lambda_l/o                    │  │
│  │  └── KD[6×6]                       │  │
│  └──────────┬─────────────────────────┘  │
│             │ pointer                      │
│             ▼                              │
│  ┌────────────────────────────────────┐  │
│  │ AdaptiveClfConstraint (OCS2 侧)    │  │
│  │  读取 pi_hat → 计算 CLF 约束值      │  │
│  └────────────────────────────────────┘  │
│             │                              │
│  ┌────────────────────────────────────┐  │
│  │ LeggedRobotPreComputation          │  │
│  │  ├── adaptiveForce_[3]             │  │
│  │  └── adaptiveTorque_[3]            │  │
│  └────────────────────────────────────┘  │
└──────────────────────────────────────────┘
         ▲                    ▲
         │ update             │ compute
         │ pi_hat             │ wrench
         │                    │
┌────────┴────────────────────┴───────────┐
│        LeggedController                 │
│  updateAdaptiveParams():                │
│    ├── compute σ (滑模面)               │
│    ├── compute Y_u (回归矩阵)            │
│    ├── pi_hat += Γ·Y_u^T·σ·dt          │
│    └── setAdaptiveWrench(f_u, t_u)      │
└─────────────────────────────────────────┘
```

---

## 五、ACLF 新增/修改文件清单

### 新增 (4 个)

| # | 路径 | 行数 | 说明 |
|---|------|------|------|
| 1 | `ocs2_legged_robot/include/.../adaptive/AdaptiveParams.h` | ~130 | π̂_u 结构 + 回归矩阵声明 |
| 2 | `ocs2_legged_robot/src/adaptive/AdaptiveWrenchEstimator.cpp` | ~120 | 回归矩阵实现 + 参数更新律 |
| 3 | `ocs2_legged_robot/include/.../constraint/AdaptiveClfConstraint.h` | ~80 | CLF 约束类 |
| 4 | `ocs2_legged_robot/src/constraint/AdaptiveClfConstraint.cpp` | ~270 | CLF 约束实现 |

### 修改 (10 个)

| # | 文件 | 修改 |
|---|------|------|
| 5 | `ocs2_legged_robot/.../LeggedRobotPreComputation.h` | +adaptiveForce/Torque +adaptiveParams |
| 6 | `ocs2_legged_robot/.../LeggedRobotInterface.h` | +ACLF 公共接口 |
| 7 | `ocs2_legged_robot/src/LeggedRobotInterface.cpp` | +约束注册 |
| 8 | `ocs2_legged_robot/CMakeLists.txt` | +新增源文件 |
| 9 | `legged_interface/.../LeggedInterface.h` | +ACLF 接口 + 成员 |
| 10 | `legged_interface/.../LeggedRobotPreComputation.h` | +adaptiveForce/Torque |
| 11 | `legged_interface/src/LeggedInterface.cpp` | +约束注册 |
| 12 | `legged_controllers/.../LeggedController.h` | +useAclf_ + updateAdaptiveParams |
| 13 | `legged_controllers/src/LeggedController.cpp` | +参数更新律实现 |
| 14 | `legged_controllers/config/go2/task.info` | +ACLF 配置段 |

---

## 六、Go2 关节命名约定

```
      前 (Front)
  LF ──────── RF      命名: {L/R}{F/H}_{关节}
  │           │      HAA = Hip Abduction/Adduction (髋横滚)
  │   trunk   │      HFE = Hip Flexion/Extension (髋俯仰)
  │           │      KFE = Knee Flexion/Extension (膝俯仰)
  LH ──────── RH
      后 (Hind)       足端: {L/R}{F/H}_FOOT
```

### 默认关节角度（站立姿态, reference.info）

| 关节 | 角度 (rad) |
|------|-----------|
| LF_HAA, LH_HAA | -0.00 |
| LF_HFE, LH_HFE | 0.72 |
| LF_KFE, LH_KFE | -1.44 |
| RF_HAA, RH_HAA | 0.25 |
| RF_HFE, RH_HFE | 0.72 |
| RF_KFE, RH_KFE | -1.44 |

### Go2 物理参数 (const.xacro)

| 参数 | 值 |
|------|-----|
| 躯干质量 | 6.921 kg |
| 大腿长度 | 0.213 m |
| 小腿长度 | 0.213 m |
| 腿偏移 | x=±0.1934, y=±0.0465 |
| 髋极限 | ±1.047 rad, 23.7 Nm |
| 膝极限 | [-2.723, -0.838] rad, 35.55 Nm |

---

## 七、关键 ROS 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/cmd_vel` | Twist | 输入 | 速度指令 |
| `/move_base_simple/goal` | PoseStamped | 输入 | 目标点指令 |
| `/joint_states` | JointState | 输出 | 关节状态 |
| `/legged_robot_mpc_observation` | mpc_observation | 输出 | MPC 观测值 |
| `/controller_manager/switch_controller` | Service | 控制 | 启动/停止控制器 |
| `/legged_robot_gait_command` | 终端 | 交互 | 步态选择 |

---

## 八、两种 Go2 URDF 模型

1. **`go2_description`** (`unitree_ros/robots/go2_description/`)
   - SolidWorks 导出，含详细 mesh
   - 用于 RViz 可视化

2. **`legged_unitree_description`** (XACRO 生成)
   - 使用基本几何体碰撞
   - 用于 Gazebo 仿真 + NMPC 控制
   - 通过 `export ROBOT_TYPE=go2` 选择

---

## 九、新包 `ocs2_legged_robot_adaptive` 架构分析

已复制到 `src/` 下。这是独立于旧版 ACLF 的新框架，包装 OCS2 的 LeggedRobotInterface 并注入自适应组件。纯控制器包，不含仿真，机器人无关（模型由 URDF + 配置文件决定）。

### 与旧版 ACLF 的核心区别

- 旧版估计 16 维物理参数（质量、质心、惯量、外力），需计算 6×16 回归矩阵 Y_u
- 新版只估计 6 维外部扰动 wrench（力 3 维 + 力矩 3 维），用滑模面适应律 d̂̇ = −Γ·σ
- 旧版手动通过 shared_ptr 共享状态；新版用 OCS2 原生 SolverSynchronizedModule 接口
- 旧版直接修改 ocs2_legged_robot 源码；新版是外挂模块

### 三个核心模块

**AdaptiveDisturbanceEstimator**（SolverSynchronizedModule）

在每个 MPC 周期前在线更新扰动估计：
```
sigma_p = e_v + lambdaPosition * e_p
sigma_o = e_w + lambdaOrientation * e_o
f_ext -= forceAdaptationGain * dt * sigma_p
t_ext -= torqueAdaptationGain * dt * sigma_o
```

**AdaptiveClfConstraint**（StateInputConstraint，dfdu=0）

CLF 不等式：`h = epsilon - sigma^T * d_hat - clfRate * ||sigma||² >= 0`，作为 relaxed barrier 软约束加入 MPC。

**AdaptiveInputBiasCost**（QuadraticStateInputCost）

将估计外力按支撑腿数均匀分配到各足端名义接触力，在 MPC 求解时偏置最优输入。

### AdaptiveLeggedRobotInterface 包装方式

```
内部创建 LeggedRobotInterface → 复制 OptimalControlProblem
→ 注入 AdaptiveDisturbanceEstimator
→ 注入 AdaptiveClfConstraint（软约束）
→ 注入 AdaptiveInputBiasCost（代价偏置）
```

### 顶层组织对比

- 旧 ACLF：`ocs2_legged_robot/adaptive/`（嵌入原包）+ `legged_interface`（桥接）+ `legged_controllers`（更新律）= 三层耦合
- 新包：`ocs2_legged_robot_adaptive`（自包含核心库）+ `ocs2_legged_robot_adaptive_ros`（瘦 ROS 节点）= 两层独立

### 估计算法对比

- 旧版 16 维物理参数 π̂_u，回归矩阵 Y_u ∈ R^{6×16}，更新律 π̂_u += Γ·Y_u^T·σ·dt。理论依据 Slotine-Li 自适应控制，可解释性强（物理量有明确含义）
- 新版 6 维外部扰动 d̂，无回归矩阵直接 σ 驱动，更新律 d̂ -= Γ·σ·dt。理论依据滑模扰动观测器，每步计算量低

### 约束形式对比

- 旧约束：h = -σ^T·[-Sτ + Y_n·π_n + Y_u·π̂_u] - 0.5·σ^T·K_D·σ，dfdu ≠ 0，需访问动力学
- 新约束：h = epsilon - σ^T·d̂ - clfRate·‖σ‖²，dfdu = 0，只用 σ 和 d̂

### 同步机制对比

- 旧：legged_interface 持有 adaptiveParams_ 共享状态，LeggedController 直接修改，OCS2 侧通过裸指针读取。更新顺序依赖人工保证
- 新：AdaptiveDisturbanceEstimator 继承 SolverSynchronizedModule，OCS2 求解器自动在每个 MPC 周期调用 preSolverRun()。数据流单向：estimator → constraint → solver

### 与 legged_controllers 的耦合

- 旧：LeggedController::update() 显式调用 updateAdaptiveParams()，必须知道 ACLF 存在
- 新：扰动估计在 MPC solver 内部回调中运行，LeggedController 完全透明——它只看到 MPC 输出已被补偿

### 机器人无关性

源码中零机器人特定硬编码。状态访问用 OCS2 质心模型标准约定（state[0:3]=速度, [3:6]=角动量, [6:9]=位置, [9:12]=姿态），接触力维度从 CentroidalModelInfo 动态获取，支撑腿数运行时计算。Go2 的 URDF 遵循相同约定，可直接适配。

### 配置管理对比

- 旧：参数散落在 task.info 的 `acl.*` 段，用 `loadData::loadCppDataType()` 逐个读取，读取失败时崩溃
- 新：独立 `adaptive_settings.info` 含内置默认值，用 `boost::property_tree::get_optional<>()` 读取。支持从 task.info 覆盖（先读默认，再读 task.info，后者覆盖前者）
