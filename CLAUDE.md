# CLAUDE.md — Go2 ACLF-MPC 四足机器人控制

## 项目概述

基于 ETH OCS2 的四足机器人自适应 MPC 控制框架，为 Unitree Go2 提供仿真 + 真机部署能力。
核心算法来自两篇论文的融合：
- Paper A: Minniti et al., "Adaptive CLF-MPC", IEEE RA-L 2021（当前运行版本）
- Paper B: Liu et al., "VAN-MPC", IEEE/ASME TMECH 2025（迁移目标）

所有开发通过 Docker 完成（ROS Noetic + Gazebo 11），宿主机只用来编辑代码。

## 目录结构

```
go2_ros1_ws/
├── src/
│   ├── leggedcontrol_go2/          # ★ 主控制框架
│   │   ├── legged_interface/       #   OCS2桥接 + 自适应估计器(策略模式)
│   │   │   ├── adaptive/           #     AdaptiveEstimatorBase → Legacy | RBF
│   │   │   └── constraint/         #     SwingTrajectoryPlanner 等
│   │   ├── legged_controllers/     #   主控制循环 + 轨迹发布
│   │   ├── legged_wbc/             #   层级QP全身控制
│   │   ├── legged_estimation/      #   Kalman Filter
│   │   ├── legged_gazebo/          #   仿真HW接口
│   │   └── legged_examples/.../    #   Go2 URDF + 真机HW
│   ├── ocs2-main/                  # ETH OCS2 (git submodule)
│   ├── pinocchio/                  # 刚体动力学 (git submodule)
│   └── hpp-fcl/                    # 碰撞检测 (git submodule)
├── build/ devel/ logs/             # ROS编译产物 (gitignored)
├── TARGET.md                       # VAN-MPC迁移方案（当前任务蓝图）
├── ARCHITECTURE.md                 # 代码架构 + 数据流
├── ALGORITHM.md                    # 数学公式 + 参数速查
└── PROCESS.md                      # 迁移进度
```

## 技术栈

| 层 | 技术 | 运行时 |
|----|------|--------|
| 最优控制 | OCS2 (SQP + HPIPM) | 100Hz, Docker内 |
| 全身控制 | 层级QP (qpOASES) | 400Hz |
| 状态估计 | Kalman Filter | 100Hz |
| 动力学 | Pinocchio (Centroidal Model) | — |
| 仿真 | Gazebo 11 | Docker内 |
| 编译 | catkin_tools | Docker内 |

## 关键设计决策

1. **自适应策略模式**: `AdaptiveEstimatorBase` 抽象接口 → `Legacy`(16参数) / `Rbf`(神经网络)。通过 `task.info` 中 `adaptiveMode` 切换，无需重编译。
2. **SRBD 模型**: MPC 使用单刚体+质心动力学（非全关节模型），接触力为控制输入，由 WBC 转为关节力矩。
3. **CLF 约束**: OCS2 中以 relaxed barrier 软约束嵌入，权重 10.0，μ=0.1, δ=5.0。
4. **状态布局**: `[v_com(3), L/m(3), p(3), eulerZYX(3)]` — 注意 L/m 不是 ω。
5. **Docker 绑定挂载**: 宿主机 `~/go2_ros1_ws` ↔ Docker `/root/catkin_ws`，源码共享但 build/ 权限归 root。

## 开发环境

```bash
# 编译（必须在 Docker 内）
docker exec unitree_ros1_go2 bash -c \
  "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin build"

# 只编译改动的包
docker exec unitree_ros1_go2 bash -c \
  "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin build legged_interface legged_controllers --no-deps"

# 启动仿真
docker exec -it unitree_ros1_go2 /root/start_go2_sim.sh
```

**重要**: 绝不在宿主机上 `catkin build`，权限不一致会导致 `.catkin_tools/` 元数据损坏。

## 自适应模式切换

在 `src/leggedcontrol_go2/legged_controllers/config/go2/task.info` 中：

```ini
legged_robot_interface.adaptiveMode "off"     # 名义MPC
legged_robot_interface.adaptiveMode "legacy"  # 论文A
legged_robot_interface.adaptiveMode "rbf"     # 论文B
```

## 文件约定

- **C++**: 4空格缩进，120字符行宽，成员变量 `trailing_`，类名 `PascalCase`
- **配置**: YAML/INFO 文件，用 ROS param 机制，不硬编码
- **文档**: markdown，简洁优先，数学公式用 Unicode（不上 MathJax 依赖）
- **Commit**: Conventional Commits 格式，英文 subject，如 `feat(aclrf): add RBF estimator`

## 已知问题 / 历史包袱

1. `SwitchedModelReferenceManager` 命名空间冲突：OCS2 自带一个版本（`reference_manager/`），legged_interface 有一个增强版（加了 SwingTrajectoryPlanner）。曾被迁到 `legged::legged_robot_ref` 又迁回 `ocs2::legged_robot`。**两个文件定义同名同命名空间类，不要同时 include**。
2. `AdaptiveNewHelper.cpp` 中的 `AdaptiveInputBias` 是实验代码，未接入主循环。
3. `ocs2_legged_robot_adaptive/` 是独立包（新版外挂式 ACLF），未接入 Go2，仅作参考。
4. OCS2 只有 `matrix_t`（动态尺寸），没有 `matrix3_t`/`vector3_t`。新增代码用 `Eigen::Matrix<scalar_t, 3, 3>` 或在自己命名空间定义别名。

## 仿真实验模块

| 模块 | 路径 | 用途 |
|------|------|------|
| 地形生成 | `scripts/terrain_generator/` | 6种地形：flat/slope/stairs/rough/varied |
| 负载注入 | `scripts/payload/` | 7种预设：5.43~21.6kg + CoM偏移 |
| 实验入口 | `scripts/run_experiment.py` | 模式 × 地形 × 负载 一键对比 |

地形需要安装依赖后在宿主机运行（用 numpy + PIL 生成 heightmap）。
负载在 Docker 内通过 Gazebo ROS service 动态注入。

## 当前任务

通过完成 `PROCESS.md` 中 Phase 1-4 的 VAN-MPC 迁移。
Phase 1 (Dual-RBFNN 自适应估计器) 架构已完成，**编译已通过** (`catkin build` clean)。

## 参考文件索引

- `TARGET.md` — 完整迁移方案（数学推导 + 代码变更）
- `ALGORITHM.md` — ACLF 公式 + 参数表
- `ARCHITECTURE.md` — 数据流 + 文件职责
- `.paper1_ACLF_MPC_Quadruped.txt` — 论文A全文
- `.paper2_VAN_MPC_Spherical.txt` — 论文B全文
