# Go2 ACLF-MPC 控制框架

> 四足机器人自适应模型预测控制 — 仿真 + 真机部署

**论文基础**: Minniti et al., *Adaptive CLF-MPC With Application To Quadrupedal Robots*, IEEE RA-L 2021

**控制链路**: NMPC (OCS2, 100Hz) → WBC (层级QP, 400Hz) → Kalman Filter → 关节力矩

## 项目结构

```
src/leggedcontrol_go2/
├── legged_controllers/    # NMPC 主控制器 + cmd_vel 轨迹发布
├── legged_wbc/            # 全身层级 QP → 12 维关节力矩
├── legged_estimation/     # Kalman Filter 状态估计 (位姿+速度)
├── legged_gazebo/         # Gazebo 仿真硬件接口
├── legged_interface/      # OCS2 ↔ ros_control 桥接 + ACLF 自适应估计器
│   └── adaptive/          #   ★ 自适应策略模式 (legacy + RBF)
├── legged_hw/             # 硬件抽象层 (仿真/真机共用)
├── legged_common/         # 共享接口 (HybridJoint, ContactSensor)
└── legged_examples/legged_unitree/
    ├── legged_unitree_description/  # URDF/XACRO 模型
    └── legged_unitree_hw/           # 真机硬件接口

外部依赖: ocs2-main/  pinocchio/  hpp-fcl/  grid_map/  unitree_ros/
```

详见 [ARCHITECTURE.md](ARCHITECTURE.md) 和 [ALGORITHM.md](ALGORITHM.md)。

## 环境与编译

| 组件 | 版本 | 组件 | 版本 |
|------|------|------|------|
| Ubuntu | 20.04 | ROS | Noetic |
| Gazebo | 11 | OCS2 | main |
| Pinocchio | latest | Docker | 20.10+ |

```bash
cd ~/go2_ros1_ws && catkin build && source devel/setup.bash
```

## 快速开始

```bash
# 终端1: 启动仿真
docker exec -it unitree_ros1_go2 /root/start_go2_sim.sh

# 终端2: 键盘遥控 (保持前台)
docker exec -it unitree_ros1_go2 bash -c \
  'source /opt/ros/noetic/setup.bash && rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel'
```

**操控**: `i`前进 `,`后退 `j`左转 `l`右转 `k`停 `q`退出

**步态**: 提示符输入 `stance`(站稳) / `trot`(对角小跑) / `dynamic_walk` / `static_walk`

## 自适应模式切换

```ini
# legged_controllers/config/go2/task.info
legged_robot_interface {
  adaptiveMode  "off"       # 原始名义 MPC (无自适应)
  adaptiveMode  "legacy"    # 论文 A: 16参数 Slotine-Li 回归
  adaptiveMode  "rbf"       # 论文 B: Dual-RBFNN 在线学习
}
```
改完需重新编译 `legged_interface`，再重启仿真：
```bash
docker exec unitree_ros1_go2 bash -c \
  "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin build legged_interface legged_controllers --no-deps"
```

## 仿真实验 — 地形 × 控制器 × 负载对比

### 1. 地形
```bash
# 生成6种地形 world 文件（宿主机运行）
pip install numpy pillow
python scripts/terrain_generator/terrains.py scripts/terrain_generator/output/

# 启动指定地形
docker exec -it unitree_ros1_go2 /root/start_go2_sim.sh \
  /root/catkin_ws/scripts/terrain_generator/output/slope10.world
```

地形类型：`flat` / `slope10`(10°) / `slope20`(20°) / `stairs`(4cm台阶) / `rough`(随机起伏) / `varied`(复合过渡)

### 2. 负载
```bash
# 仿真运行后新开终端
docker exec unitree_ros1_go2 bash -c \
  "source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash && \
   python3 /root/catkin_ws/scripts/payload/spawn_payload.py brick_5kg"

# 可用: none / brick_5kg / brick_10kg / box_16kg / box_21kg / offset_com / offset_com_left
# 查看: python3 scripts/payload/spawn_payload.py --list
# 移除: python3 scripts/payload/spawn_payload.py none
```

### 3. 采集数据
```bash
# 录制 bag
docker exec unitree_ros1_go2 bash -c \
  "source /opt/ros/noetic/setup.bash && \
   rosbag record -O /root/catkin_ws/gazebo_results/exp.bag \
   /legged_robot_mpc_observation /joint_states /cmd_vel --duration=30"

# 或单话题采集
rostopic echo /legged_robot_mpc_observation -n 500 > mpc_data.csv
```

### 4. 一键对比
```bash
python scripts/run_experiment.py --list          # 查看可用组合
python scripts/run_experiment.py --mode rbf --terrain slope10 --payload brick_5kg
python scripts/run_experiment.py --compare       # 全矩阵对比
```

### 实验矩阵

| 目的 | 模式 | 地形 | 负载 |
|------|------|------|------|
| 基准 | off / legacy / rbf | flat | none |
| 负重适应 | off / legacy / rbf | flat | brick_5kg, box_16kg |
| 爬坡 | off / legacy / rbf | slope10, slope20 | none |
| 崎岖 | legacy / rbf | rough, stairs | none |
| CoM偏移 | legacy / rbf | flat | offset_com |
| 综合 | legacy / rbf | varied | box_16kg |

## 调试

按顺序排查：

**① Gazebo 起得来吗？**
```bash
docker exec unitree_ros1_go2 bash -c 'fuser -k 11345/tcp'  # 杀端口残留
```
看到 `SpawnModel: Successfully spawned` 才行。

**② 控制器跑了吗？** `rosservice call /controller_manager/list_controllers` → `legged_controller: running`

**③ 键盘发数据吗？** 按住 `i`，`rostopic echo /cmd_vel -n 1` 应有输出。

**④ 关节响应吗？** `rostopic echo /joint_states -n 1` 值持续变化 = 正常。

| 常见问题 | 解决 |
|----------|------|
| Gazebo 闪退 | `fuser -k 11345/tcp` |
| 机器人摔倒 | 先 `stance` 再 `trot`；增大 pitch/roll 权重 |
| 强制清理 | `pkill -9 gzserver gzclient roslaunch` |
| 关节角度打印刷屏 | 删 `LeggedController.cpp:112` 的 `ROS_WARN` |

## 开发者公约（精简版）

- **分支**: `main`(稳定) / `dev`(开发) / `feature/<name>`(功能) / `fix/<name>`(修复)
- **Commit**: [Conventional Commits](https://www.conventionalcommits.org/) 格式，英文 subject
  - `feat(aclrf):`, `fix(gazebo):`, `refactor(controller):`, `docs:` ...
- **代码**: 4空格缩进，120字符行宽，C++ 用 ROS Style Guide，提交前 `catkin build` 通过
- **PR**: 至少1人 Review + Approve，用 Squash and Merge
- **禁止**: `push --force` 到 main/dev，提交编译产物，硬编码路径和密码

## 参考资料

- [ACLF-MPC 论文](https://arxiv.org/abs/2112.04536) — Minniti et al., IEEE RA-L 2021
- [VAN-MPC 论文 (球形机器人)](https://doi.org/10.1109/TMECH.2025.3528106) — Liu et al., IEEE/ASME TMECH 2025
- [OCS2](https://github.com/leggedrobotics/ocs2) | [Legged Control](https://github.com/qiayuanliao/legged_control) | [Pinocchio](https://stack-of-tasks.github.io/pinocchio/)

├── [ARCHITECTURE.md](ARCHITECTURE.md) — 代码架构 + 数据流
├── [ALGORITHM.md](ALGORITHM.md) — ACLF 算法数学推导
├── [TARGET.md](TARGET.md) — VAN-MPC 迁移方案 (当前任务)
└── [PROCESS.md](PROCESS.md) — 迁移进度记录
