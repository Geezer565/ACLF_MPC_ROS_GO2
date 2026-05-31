# 04 — 运行与调试

> 当前可用：原始 MPC + 旧版 ACLF（16 参数）
> 新包 `ocs2_legged_robot_adaptive` 尚未接入，不能使用

---

## 一、一键启动

```bash
# 终端1: 仿真+控制器
docker exec -it unitree_ros1_go2 /root/start_go2_sim.sh

# 终端2: 键盘（前台，长按才走）
docker exec -it unitree_ros1_go2 bash -c \
  'source /opt/ros/noetic/setup.bash && rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel'
```

按键：`i`前进 `,`后退 `j`左转 `l`右转 `k`停

---

## 二、调试流程

按顺序排查，上一步不通就别往下走。

**1. Gazebo 起得来吗？**

`docker exec unitree_ros1_go2 bash -c 'fuser -k 11345/tcp'` 然后重试终端 1。看到 `SpawnModel: Successfully spawned` 才行。

**2. 控制器跑起来了吗？**

```bash
rosservice call /controller_manager/list_controllers
```

`legged_controller` 必须 `state: running`。如果是 `stopped`，看终端 1 有没有报 `Safety check failed`——有就说明机器人摔了，重启终端 1。

**3. 键盘发数据了吗？**

终端 2 保持前台，按住 `i` 不放，然后新开终端：

```bash
rostopic echo /cmd_vel -n 1
```

没数据 = 键盘没工作。确认终端 2 窗口在最前面。

**4. 机器人受控吗？**

```bash
rostopic echo /joint_states -n 1
```

关节值在变 = 控制器在输出力矩。不变 = 控制器挂了。

---

## 三、步态

终端 1 启动后在提示符输入步态名。常用：`stance`（站稳）、`trot`（走）、`dynamic_walk`（慢走）。

---

## 四、切换 ACLF 模式（当前仅支持旧版）

编辑 `legged_controllers/config/go2/task.info`：

```ini
legged_robot_interface {
  useAclf    false    ; false=原始MPC, true=旧版ACLF(16参数)
}
```

改完重启终端 1。启用 ACLF 后终端每秒打印 `[ACLF] m_u=...`。

---

## 五、新包接入状态（还不能用）

两包已复制到 `src/` 但：

- 未编译（依赖 ocs2_legged_robot，需 `catkin build ocs2_legged_robot_adaptive ocs2_legged_robot_adaptive_ros`）
- launch 指向 ANYmal 配置，未改为 Go2
- legged_interface 未添加 `adaptive_mode` 参数
- 接入计划见 `01_OVERVIEW.md` 第四章

---

## 六、强制清理

```bash
docker exec unitree_ros1_go2 bash -c 'pkill -9 gzserver; pkill -9 gzclient; pkill -9 roslaunch'
```

端口冲突：`docker exec unitree_ros1_go2 bash -c 'fuser -k 11345/tcp'`
