# 05 — 参数调优指南

所有参数位于：
- `legged_controllers/config/go2/task.info` — MPC + WBC + Kalman + ACLF
- `legged_controllers/config/go2/reference.info` — 默认姿态 + 速度限制
- `legged_controllers/config/go2/gait.info` — 步态定义

> `.info` 文件修改后无需重新编译，重新 launch 即可生效。

---

## 一、MPC Q 矩阵 — 状态跟踪权重

值越大 = 跟踪越紧。位于 `task.info` 的 `Q { }` 段。

```
(0,0)   15.0     ; vcom_x — 前进速度
(1,1)   15.0     ; vcom_y — 侧向速度
(2,2)   100.0    ; vcom_z — 垂直速度
(6,6)   1000.0   ; p_base_x — 基座 X 位置
(7,7)   1000.0   ; p_base_y — 基座 Y 位置
(8,8)   1500.0   ; p_base_z — 基座高度 ★ 最重要
(9,9)   100.0    ; theta_base_z — 偏航
(10,10) 300.0    ; theta_base_y — 俯仰 (保持稳定!)
(11,11) 300.0    ; theta_base_x — 横滚 (保持稳定!)
(12-23) 0.5~2.0  ; 各关节位置权重
```

### 常见调参场景

| 症状 | 修改 |
|------|------|
| 机器人摔倒 | **增大 (10,10) 俯仰 和 (11,11) 横滚** |
| 高度下降/下蹲 | **增大 (8,8) 基座 Z** |
| 机器人漂移 | 增大 (6,6) 和 (7,7) 位置权重 |
| 动作抖动 | 减小关节权重 (12,12)–(23,23) |
| 速度跟踪差 | 增大 (0,0) 和 (1,1) 速度权重 |
| 转向慢 | 增大 (3,3) 偏航角速度权重 |

---

## 二、MPC R 矩阵 — 控制输入惩罚

值越大 = 越平滑但越保守。位于 `task.info` 的 `R { }` 段。

```
(0,0)–(11,11)   1.0     ; 接触力惩罚 (scaling=1e-3)
(12,12)–(23,23) 5000.0  ; 足端速度惩罚 (影响摆腿速度!)
```

| 场景 | 修改 |
|------|------|
| 摆腿太慢 | **减小 (12,12)–(23,23)** |
| 着地太硬 | **增大 (0,0)–(11,11)** |
| 控制太激进 | 整体增大 R |
| 响应太迟钝 | 整体减小 R |

---

## 三、MPC 时序

```
mpc {
  timeHorizon          1.0    ; [s] 预测窗口
  mpcDesiredFrequency  100    ; [Hz] 控制频率
}

ddp {
  maxNumIterations     1      ; DDP 每次迭代数
  constraintTolerance  5e-3   ; 约束松弛
  timeStep             0.015  ; [s] 积分步长
}
```

| 参数 | 调大效果 | 调小效果 |
|------|---------|---------|
| timeHorizon | 更好预判，更多计算 | 更快响应 |
| mpcDesiredFrequency | 更快反应，更多 CPU | 省 CPU |
| maxNumIterations | 更精确解 | 更快 |
| timeStep | 更快但粗糙 | 更精确但慢 |
| constraintTolerance | 更宽松 | 更严格 |

---

## 四、WBC 权重

```
weight {
  swingLeg      100     ; 摆腿跟踪 (最高优先级)
  baseAccel     1       ; 基座加速度跟踪
  contactForce  0.01    ; 接触力分配 (最低优先级)
}
```

- `swingLeg=100` → WBC 优先保证足端轨迹
- `baseAccel` 增大 → 躯干更僵硬
- `contactForce=0.01` → 力分布软，避免冲击

---

## 五、摆动轨迹

```
swing_trajectory_config {
  liftOffVelocity    0.05    ; 抬脚速度
  touchDownVelocity  -0.1    ; 落脚速度 (负=向下)
  swingHeight        0.08    ; [m] 足端最大离地高度
  swingTimeScale     0.15    ; 摆动时长比例
}
```

| 场景 | 修改 |
|------|------|
| 过障碍 | 增大 swingHeight (如 0.12) |
| 落脚太慢 | 更负的 touchDownVelocity (如 -0.2) |
| 摆腿太快/太慢 | 调 swingTimeScale |

---

## 六、步态 (gait.info)

每个步态由 `modeSequence`（接触模式序列）和 `switchingTimes`（切换时间）定义。

### 示例：修改 trot 步态

```
trot {
  modeSequence { LF_RH, RF_LH }
  switchingTimes {
    [0] 0.0    ; LF+RH 触地
    [1] 0.3    ; 切换到 RF+LH
    [2] 0.6    ; 周期重复 (0.6s → ~1.67Hz)
  }
}
```

| 场景 | 修改 |
|------|------|
| 更稳 | 增大切换时间（减频） |
| 更快 | 减小切换时间（增频） |
| 加站立相 | 插入 STANCE 模式 |

---

## 七、参考姿态 (reference.info)

```
targetDisplacementVelocity   0.5    ; 最大平移速度 [m/s]
targetRotationVelocity       1.57   ; 最大旋转速度 [rad/s]
comHeight                    0.33   ; 期望质心高度 [m]
```

| 场景 | 修改 |
|------|------|
| 走路不稳 | 降低 targetDisplacementVelocity (如 0.3) |
| 转弯太快 | 降低 targetRotationVelocity |
| 机器人太高/太低 | 调整 comHeight |

---

## 八、Kalman Filter

```
kalmanFilter {
  footRadius               0.02    ; 足端接触半径 [m]
  imuProcessNoisePosition  0.02    ; IMU 位置过程噪声
  imuProcessNoiseVelocity  0.02    ; IMU 速度过程噪声
  footProcessNoisePosition 0.002   ; 足端过程噪声
  footSensorNoisePosition  0.005   ; 足端传感器噪声
  footSensorNoiseVelocity  0.1     ; 足端速度传感器噪声
  footHeightSensorNoise    0.01    ; 足端高度传感器噪声
}
```

| 症状 | 修改 |
|------|------|
| 状态估计漂移 | **减小** IMU 过程噪声 |
| 状态估计震荡 | **增大** IMU 过程噪声 |
| 足端打滑 | **减小** 足端传感器噪声 |

---

## 九、ACLF (task.info)

```
acl {
  lambdaGain      5.0      ; 滑模面增益 Λ = lambdaGain * I₃
  gammaMass       5.0      ; 质量自适应速度
  gammaCom        1.0      ; 质心自适应速度
  gammaInertia    0.01     ; 惯量自适应速度
  gammaWrench     0.1      ; 外力自适应速度
  KDDiag { 50,50,50, 80,80,80 }  ; CLF 耗散矩阵
}

aclSoftConstraint {
  clfWeight      10.0      ; CLF 约束在 cost 中权重
  mu             0.1       ; 松弛障碍参数
  delta          5.0       ; 松弛障碍 delta
}
```

| 参数 | 调大 | 调小 |
|------|------|------|
| lambdaGain | 位姿收敛更快 | 更平滑 |
| gammaMass | 质量估计更快 | 更稳定 |
| gammaCom | 质心适应更快 | 噪音更小 |
| gammaInertia | 惯量适应更快 | 防发散 |
| gammaWrench | 扰动补偿更快 | 减少震荡 |
| clfWeight | 稳定性优先 | 最优性优先 |

### 关闭 ACLF

```ini
legged_robot_interface { useAclf false }
```
重启控制器即恢复原始名义 MPC。

---

## 十、Gazebo 物理

`legged_gazebo/worlds/empty_world.world`：

```xml
<physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_update_rate>1000</real_time_update_rate>
</physics>
```

| 场景 | 修改 |
|------|------|
| 仿真太慢 | 增大 max_step_size (如 0.002)，减小 update_rate |
| 物理不准 | 减小 max_step_size |
| 接触不稳定 | 增大接触刚度（gazebo.xacro 中的 kp/kd） |

---

## 十一、Gazebo 接触参数 (gazebo.xacro)

```xml
<gazebo reference="LF_FOOT">
    <kp value="1000000.0"/>   <!-- 接触刚度 -->
    <kd value="100.0"/>        <!-- 接触阻尼 -->
    <mu1 value="0.6"/>         <!-- 静摩擦 -->
    <mu2 value="0.6"/>         <!-- 动摩擦 -->
</gazebo>
```

| 场景 | 修改 |
|------|------|
| 脚陷入地面 | 增大 kp |
| 脚弹跳 | 增大 kd |
| 打滑 | 增大 mu1/mu2 |
| 太黏 | 减小 mu1/mu2 |

---

## 十二、参考值速查

### Go2 物理参数

| 参数 | 值 |
|------|-----|
| 躯干质量 | 6.921 kg |
| 大腿长 | 0.213 m |
| 小腿长 | 0.213 m |
| 髋扭矩极限 | 23.7 Nm |
| 膝扭矩极限 | 35.55 Nm |
| 髋角度极限 | ±1.047 rad |
| 膝角度极限 | [-2.723, -0.838] rad |

### 默认站立关节角

| 关节 | 角度 |
|------|------|
| LF/LH_HAA | -0.00 rad |
| LF/LH_HFE | +0.72 rad |
| LF/LH_KFE | -1.44 rad |
| RF/RH_HAA | +0.25 rad |
| RF/RH_HFE | +0.72 rad |
| RF/RH_KFE | -1.44 rad |

### 默认姿态

| 参数 | 值 |
|------|-----|
| comHeight | 0.33 m |
| targetDisplacementVelocity | 0.5 m/s |
| targetRotationVelocity | 1.57 rad/s |
