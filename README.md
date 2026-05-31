# ACLF-MPC ROS Workspace for Go2 Quadruped Robot

> 基于 Adaptive CLF-MPC 的四足机器人控制框架 — 仿真 + 真机部署

**论文基础**：Minniti, Grandia, Farshidian, Hutter — *Adaptive CLF-MPC With Application To Quadrupedal Robots*, IEEE RA-L 2021

**控制架构**：NMPC (OCS2) → WBC (Hierarchical QP) → Kalman Filter → [待开发] RNN 模型补偿

---

## 目录

- [项目结构](#项目结构)
- [环境依赖](#环境依赖)
- [快速开始](#快速开始)
- [步态控制](#步态控制)
- [调试指南](#调试指南)
- [开发者公约](#开发者公约)
- [参考资料](#参考资料)

---

## 项目结构

```
go2_ros1_ws/
├── src/
│   ├── leggedcontrol_go2/              # ★ 主控制框架 (NMPC + WBC)
│   │   ├── legged_controllers/         #   NMPC 控制器 + 轨迹发布器
│   │   ├── legged_wbc/                 #   全身 QP 控制器 (层级优化)
│   │   ├── legged_estimation/          #   Kalman Filter 状态估计
│   │   ├── legged_gazebo/              #   Gazebo 仿真硬件接口
│   │   ├── legged_interface/           #   OCS2 ↔ ros_control 桥接
│   │   ├── legged_hw/                  #   硬件抽象层
│   │   ├── legged_common/              #   共享接口 (HybridJoint, ContactSensor)
│   │   └── legged_examples/legged_unitree/
│   │       ├── legged_unitree_description/  #   URDF/XACRO 机器人模型
│   │       └── legged_unitree_hw/           #   真机硬件接口
│   ├── ocs2-main/                      # ETH OCS2 最优控制库
│   ├── ocs2_robotic_assets/            # OCS2 模型资产
│   ├── pinocchio/                      # 刚体动力学库 (Centroidal Model)
│   ├── hpp-fcl/                        # 碰撞检测库
│   ├── grid_map/                       # 栅格地图
│   └── unitree_ros/                    # Unitree 官方 ROS 包
├── 01_OVERVIEW.md                      # 总览与研究路线
├── 02_CODE_ARCHITECTURE.md             # 代码架构详解
├── 03_ACLF_ALGORITHM.md                # ACLF 算法数学推导
├── 04_RUN_GUIDE.md                     # 仿真启动与操控
├── 05_TUNING_GUIDE.md                  # 参数调优手册
├── build/                              # (gitignored) ROS 编译产物
├── devel/                              # (gitignored) ROS 开发空间
└── logs/                               # (gitignored) 运行日志
```

### 控制流程

```
cmd_vel / MoveBase Goal
        │
        ▼
┌──────────────────────────────┐
│ TargetTrajectoriesPublisher  │  速度指令 → 状态轨迹
└──────────────┬───────────────┘
               ▼
┌──────────────────────────────┐
│ NMPC (OCS2 SQP, ~100Hz)      │
│ • 单刚体质心动力学 (SRBD)      │
│ • 摩擦锥约束                   │
│ • 步态调度 + 接触序列          │
│ • [ACLF] CLF 不等式约束       │
│ • QP 子问题 → HPIPM 求解      │
└──────────────┬───────────────┘
               ▼
┌──────────────────────────────┐
│ WBC (层级 QP, ~400Hz)        │
│ 优先级: swingLeg > baseAccel │
│         > contactForce       │
└──────────────┬───────────────┘
               ▼
┌──────────────────────────────┐
│ Kalman Filter (~100Hz)       │
│ 估计 base 位姿/速度           │
│ 输入: IMU + 足端接触          │
└──────────────┬───────────────┘
               ▼
         关节力矩指令 → 仿真/真机
```

---

## 环境依赖

| 组件 | 版本 | 说明 |
|------|------|------|
| Ubuntu | 20.04 | 宿主机系统 |
| ROS | Noetic | 机器人框架 |
| Gazebo | 11 | 物理仿真引擎 |
| OCS2 | main | ETH 最优控制库 |
| Pinocchio | latest | 刚体动力学 |
| hpp-fcl | latest | 碰撞检测 |
| qpOASES | - | QP 求解器 |
| HPIPM | - | 高性能内点法求解器 |
| Docker | 20.10+ | 容器化运行 (推荐) |

### 编译

```bash
cd ~/go2_ros1_ws
catkin build
source devel/setup.bash
```

---

## 快速开始

### 1. 启动仿真

```bash
# 终端1: 启动 Gazebo + 控制器
docker exec -it unitree_ros1_go2 /root/start_go2_sim.sh
```

### 2. 键盘遥控

```bash
# 终端2: 键盘控制 (需保持前台窗口)
docker exec -it unitree_ros1_go2 bash -c \
  'source /opt/ros/noetic/setup.bash && rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel'
```

**操控按键**：

| 按键 | 动作 |
|------|------|
| `i` | 前进 |
| `,` | 后退 |
| `j` | 左转 |
| `l` | 右转 |
| `k` | 停止 |
| `q` | 退出 |

---

## 步态控制

启动后在提示符输入步态名称，常用步态：

| 步态 | 说明 |
|------|------|
| `stance` | 站立 (建议启动后先发此步态稳住) |
| `trot` | 对角小跑 (默认) |
| `dynamic_walk` | 动态行走 |
| `static_walk` | 静态行走 |
| `pace` | 同侧步 |
| `bound` | 跳跃步 |

> 完整步态列表（12种）请参考 `05_TUNING_GUIDE.md`

---

## 调试指南

按以下顺序排查：

**① Gazebo 是否正常启动？**

```bash
docker exec unitree_ros1_go2 bash -c 'fuser -k 11345/tcp'  # 先杀端口残留
```

看到 `SpawnModel: Successfully spawned` 表示启动成功。

**② 控制器是否运行？**

```bash
rosservice call /controller_manager/list_controllers
```

`legged_controller` 必须显示 `state: running`。

**③ 键盘是否发送指令？**

```bash
rostopic echo /cmd_vel -n 1
```

按住 `i` 键不放，应有数据输出。

**④ 关节是否响应？**

```bash
rostopic echo /joint_states -n 1
```

关节值持续变化 = 控制器正常输出力矩。

---

## 开发者公约

### 一、分支策略

本项目采用 **Git Flow** 简化版分支模型：

| 分支 | 用途 | 说明 |
|------|------|------|
| `main` | 稳定发布分支 | 只接受经过 Review 的 PR 合并，保证可编译可运行 |
| `dev` | 开发主线 | 日常开发集成，所有功能分支从此拉出 |
| `feature/<name>` | 功能开发 | 例如 `feature/rnn-compensator`, `feature/multi-terrain` |
| `fix/<name>` | Bug 修复 | 例如 `fix/safety-check-threshold`, `fix/compiler-warning` |
| `experiment/<name>` | 实验性分支 | 用于论文实验、参数探索，不保证合并回 dev |

**规则**：
- **永远不要在 `main` 上直接提交**
- 开发前从 `dev` 拉出功能分支
- 功能完成后提 PR 合并回 `dev`
- `dev` 稳定后合并到 `main` 并打 tag

### 二、Commit 提交规范

使用 **Conventional Commits** 格式：

```
<type>(<scope>): <subject>

[optional body]

[optional footer]
```

**Type 类型**：

| Type | 用途 |
|------|------|
| `feat` | 新功能 |
| `fix` | Bug 修复 |
| `docs` | 文档变更 |
| `style` | 代码格式（不影响逻辑） |
| `refactor` | 重构（非新功能、非修 bug） |
| `perf` | 性能优化 |
| `test` | 添加或修改测试 |
| `chore` | 构建/工具/依赖变更 |
| `revert` | 回滚之前的提交 |

**Scope 范围**（本项目推荐）：

| Scope | 含义 |
|-------|------|
| `mpc` | MPC 求解器相关 |
| `wbc` | 全身控制相关 |
| `aclrf` | ACLF 自适应律 |
| `estimation` | 状态估计 / Kalman Filter |
| `gazebo` | 仿真环境 / world 文件 |
| `hw` | 硬件接口 / 真机部署 |
| `controller` | 主控制器逻辑 |
| `config` | 配置 / 参数变更 |
| `docs` | 文档 |

**提交示例**：

```bash
# 好 ✅
git commit -m "feat(aclrf): add CLF-guided integral term in WBC layer"
git commit -m "fix(gazebo): resolve ODE physics iterator type mismatch"
git commit -m "docs: update run guide with multi-terrain instructions"
git commit -m "refactor(controller): extract RNN inference to separate class"

# 不好 ❌
git commit -m "update"
git commit -m "fix bug"
git commit -m "修改了一些东西"
```

**额外规则**：
- Subject 使用**英文**，首字母小写，不加句号，不超过 72 字符
- Body 解释 **为什么** 做这个改动（非必须，复杂改动建议写）
- 关联 Issue 时在 footer 写 `Closes #<issue-number>`
- 每次提交应该是**逻辑独立**的最小变更单元

### 三、代码风格

#### C++（本项目主要语言）

- 遵循 [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide)
- 缩进：**4 空格**（禁止 Tab）
- 行宽：最大 **120** 字符
- 命名约定：
  - 类名：`PascalCase` — `LeggedController`, `KalmanFilter`
  - 函数/方法：`camelCase` — `updateAdaptiveParams()`, `computeTorques()`
  - 变量：`snake_case` — `joint_position_`, `contact_force_`
  - 成员变量：以 `_` 结尾 — `base_pose_`, `kp_`
  - 常量/枚举：`UPPER_SNAKE_CASE` — `MAX_JOINTS`, `STATE_SIZE`
  - 命名空间：`snake_case` — `legged`, `ocs2`

#### Python

- 遵循 [PEP 8](https://peps.python.org/pep-0008/)
- 缩进：**4 空格**
- 函数/变量：`snake_case`
- 类名：`PascalCase`

#### CMake

- 命令：小写 — `add_library`, `target_link_libraries`
- 变量：`UPPER_SNAKE_CASE` 或 `snake_case`（保持项目内一致）

#### 通用规则

- **注释写英文**，复杂算法可以加中文注释便于团队理解
- 禁止提交注释掉的代码块（用 git history 回溯）
- 禁止提交调试用的 `printf` / `ROS_WARN` 刷屏日志
- 新增参数使用 ROS param 机制，**不要硬编码**在代码中

### 四、Pull Request 流程

**提交 PR 前自查**：

- [ ] 代码在本地编译通过 (`catkin build`)
- [ ] 仿真环境测试通过（机器人不摔倒、步态正常）
- [ ] 删除调试日志和注释代码
- [ ] 遵循代码风格约定
- [ ] Commit message 符合规范
- [ ] 新功能/参数有文档说明

**PR 标题格式**：`<type>(<scope>): <description>`

**PR 描述模板**：

```markdown
## 变更说明
简要描述做了什么改动。

## 动机
为什么需要这个改动？解决什么问题？

## 测试
- [ ] 本地编译通过
- [ ] Gazebo 仿真测试通过
- [ ] 步态 trot/dynamic_walk 正常运行

## 关联 Issue
Closes #X

## 截图/日志 (如有)
```

**Review 规则**：

- 至少 **1 人 Review + Approve** 后才能合并
- 合并使用 **Squash and Merge**（将 PR 内所有 commit 压缩为 1 个）
- 合并前确保 CI 通过（如配置了 CI）

### 五、Issue 规范

**Bug Report 模板**：

```markdown
## 现象
描述 bug 的具体表现。

## 复现步骤
1. 启动仿真...
2. 切换到 trot 步态...
3. 观察到...

## 预期行为
应该发生什么。

## 环境
- ROS 版本:
- Gazebo 版本:
- 分支/Commit:
- 步态/参数:

## 日志 / 截图
```

**Feature Request 模板**：

```markdown
## 需求描述
想要实现什么功能。

## 技术方案
初步的技术思路（可选）。

## 优先级
紧急 / 高 / 中 / 低
```

### 六、版本号与 Tag

使用语义化版本 **Semantic Versioning 2.0**：`MAJOR.MINOR.PATCH`

- **MAJOR**：不兼容的 API/接口变更
- **MINOR**：向后兼容的新功能
- **PATCH**：向后兼容的 Bug 修复

```bash
# 发版时打 tag
git tag -a v0.1.0 -m "v0.1.0: ACLF-MPC basic framework, trot gait verified"
git push origin v0.1.0
```

### 七、禁止事项

- ❌ 不要 `push --force` 到 `main` 或 `dev` 分支
- ❌ 不要在 PR Review 通过前自行合并
- ❌ 不要提交大文件（>10MB 的二进制文件优先用 Git LFS）
- ❌ 不要提交编译产物（`build/`, `devel/`, `logs/`, `*.o`, `*.so`）
- ❌ 不要提交包含密码、Token、IP 地址等敏感信息的配置文件
- ❌ 不要在代码中硬编码路径（使用 ROS param 或相对路径）

---

## 常见问题

| 问题 | 解决方法 |
|------|----------|
| Gazebo 闪退 | `fuser -k 11345/tcp` 杀掉端口残留后重试 |
| "Authorization required" | 宿主机执行 `xhost +local:root` |
| 机器人不走 | 键盘终端保持前台，按住 `i` 键不放 |
| 机器人摔倒 | 先发 `stance` 站稳，再切 `trot`；增大 Q 矩阵 pitch/roll 权重 |
| 端口冲突 | `docker exec unitree_ros1_go2 bash -c 'fuser -k 11345/tcp'` |
| 关节角度刷屏 | 删除 `LeggedController.cpp` 第 112 行 `ROS_WARN` |
| 强制清理 | `docker exec unitree_ros1_go2 bash -c 'pkill -9 gzserver; pkill -9 gzclient; pkill -9 roslaunch'` |

---

## 参考资料

- [Adaptive CLF-MPC 论文](https://arxiv.org/abs/2112.04536) — Minniti et al., IEEE RA-L 2021
- [OCS2 最优控制库](https://github.com/leggedrobotics/ocs2) — ETH Zurich
- [Legged Control 框架](https://github.com/qiayuanliao/legged_control) — Qiayuan Liao
- [Pinocchio 动力学库](https://stack-of-tasks.github.io/pinocchio/)
- [hpp-fcl 碰撞检测](https://github.com/humanoid-path-planner/hpp-fcl)
- [ROS Noetic 文档](http://wiki.ros.org/noetic)
- [Gazebo 11 文档](https://classic.gazebosim.org/tutorials)
- 项目内详细文档：`01_OVERVIEW.md` / `02_CODE_ARCHITECTURE.md` / `03_ACLF_ALGORITHM.md` / `04_RUN_GUIDE.md` / `05_TUNING_GUIDE.md`

---

## 致谢

本项目基于 ETH Zurich 机器人系统实验室 (RSL) 的开源工作：
- OCS2 最优控制框架
- Adaptive CLF-MPC 算法
- Pinocchio 刚体动力学库

---

## License

MIT
