# 03 — ACLF-MPC 算法

> 基于 Minniti et al. — *Adaptive CLF-MPC With Application To Quadrupedal Robots*, IEEE RA-L 2021
> 完整实现位于 ocs2_legged_robot/adaptive/ 和 ocs2_legged_robot/constraint/

---

## 一、核心思想

将 **Slotine-Li 自适应控制**的 Lyapunov 稳定性条件作为不等式约束嵌入 MPC，同时在 MPC 外部运行参数更新律。使四足机器人携带未知负载、拖拽重物时保持稳定跟踪。

---

## 二、自适应动力学

标准 SRBD（单刚体动力学）被修改为：

```
v̇_p = g + (1/m) * (Σ R_WB * λ_EEi - R_WB * f_u)     // 线加速度
ω̇   = I⁻¹ * (−ω × Iω + Σ r_EEi × λ_EEi − t_u)      // 角加速度
```

其中 `(f_u, t_u) = Y_u(q, v, v̇) * π̂_u` 是自适应 wrench。

---

## 三、自适应参数 π̂_u ∈ R^16

```
索引      参数             维度   物理含义
────────────────────────────────────────────
[0]       m_u              1     负载质量偏移
[1-3]     h_u = m_u*c_u    3     一阶质量矩 (CoM偏移 × 质量)
[4-9]     vec(I_u)         6     负载惯量 (Ixx,Ixy,Ixz,Iyy,Iyz,Izz)
[10-12]   f_const          3     恒定外力 (世界坐标系)
[13-15]   t_const          3     恒定外力矩 (世界坐标系)
────────────────────────────────────────────
Total                      16
```

---

## 四、回归矩阵 Y_u ∈ R^{6×16}

```
Y_u = [Y_force; Y_torque]

Y_force (3×16):  [v̇_pr−g |  0₃  |   0₆   | I₃ | 0₃]
                   ↑mass    ↑h_u   ↑inertia ↑f_c  ↑t_c

Y_torque (3×16): [0 | −S(v̇_pr−g) | L(ω̇_r)+S(ω)·L(ω_r) | 0₃ | I₃]
                   ↑mass  ↑h_u          ↑inertia                ↑f_c ↑t_c
```

其中：
- `S(v)`: 反对称矩阵（cross-product matrix）
- `L(ω)`: 将 6-向量 `vec(I)` 映射到 `I·ω` 的 3×6 矩阵

---

## 五、CLF 不等式约束

### 滑模面（复合误差）

```
σ_l = (v − v_des) + Λ_l * (p − p_des)     // 线运动
σ_o = (ω − ω_des) + Λ_o * (θ − θ_des)    // 角运动 (欧拉角近似)
σ   = [σ_l; σ_o] ∈ R^6
```

### 约束残差

```
appliedForce  = Σ λ_EEi                          // 足端力求和
appliedTorque = Σ r_EEi × λ_EEi                  // 足端力矩求和
nomForce      = m * g                             // 名义重力补偿
nomTorque     = ω × I_n * ω                       // 名义陀螺力矩
adapForce     = m_u*g + f_const                   // 自适应力
adapTorque    = ω × I_u * ω + t_const             // 自适应力矩

RHS = −[appliedForce; I⁻¹·appliedTorque]
      + [nomForce; I⁻¹·nomTorque]
      + [adapForce; I⁻¹·adapTorque]
```

### 约束值

```
h_clf = −σ^T * RHS − 0.5*σ^T*K_D*σ ≥ 0
```

当 `h_clf ≥ 0` 满足时，Lyapunov 函数 V 的导数 ≤ −W，保证 σ → 0。

---

## 六、参数更新律

```
π̂̇_u = Γ * Y_u^T * σ
```

欧拉积分：
```cpp
pi_hat += Gamma * Y_u.transpose() * sigma * dt;
```

更新后对质量/惯量做正值截断防止发散。

---

## 七、Lyapunov 稳定性分析

**候选函数**:
```
V(σ, π̃_u) = 0.5 * σ^T * M * σ + 0.5 * π̃_u^T * Γ⁻¹ * π̃_u
```

**导数**（沿闭环轨迹，利用 Ṁ−2C 反对称性）:
```
V̇ = σ^T * [−Sτ + Y_n*π_n + Y_u*π̂_u]
```

CLF 约束 h_clf ≥ 0 等价于 V̇ ≤ −W:
```
h_clf = −V̇ − W = −σ^T*[−Sτ + Y_n*π_n + Y_u*π̂_u] − 0.5*σ^T*K_D*σ ≥ 0
```

当满足时：V̇ ≤ −0.5*σ^T*K_D*σ < 0（对 σ ≠ 0），从而 σ → 0，q̃ → 0。

---

## 八、MPC 中软约束处理

CLF 约束作为 **relaxed barrier** 软约束加入 MPC 目标函数：

```
min Σ[l(x,u) + B(h_clf)]   subject to: dynamics + hard constraints
```

其中 `B(h) = −μ * ln(h + δ)` 是松弛对数障碍（OCS2 `RelaxedBarrierPenalty`）。

默认参数: μ = 0.1, δ = 5.0

---

## 九、关键算法伪代码

```
LeggedController::update():
  1. updateStateEstimation()
     ├── IMU → 加速度 + 角速度 + 姿态
     ├── 足端接触 → 接触标志
     └── Kalman Filter → measuredRbdState_

  2. mpcMrtInterface_->updatePolicy()           // 获取最新 MPC 策略

  3. mpcMrtInterface_->evaluatePolicy()         // 求解最优状态/输入
     │  [OCS2 SQP Solver, 100Hz, 1.0s horizon]
     │  ┌─ Dynamics: SRBD
     │  ├─ Cost: Q,R 二次型跟踪
     │  ├─ Equality: 零速度、零力、法向速度
     │  ├─ Soft: 摩擦锥 (relaxed barrier)
     │  └─ Soft: [ACLF] AdaptiveClfConstraint   ← ★ 读取 adaptiveParams_
     │

  4. [ACLF] updateAdaptiveParams()               // 参数更新律
     ├── σ = ṽ + Λ*q̃
     ├── Y_u = AdaptiveRegressor::computeRegressor()
     ├── π̂_u += Γ * Y_u^T * σ * dt
     ├── f_u = params.computeAdaptiveForce()
     ├── t_u = params.computeAdaptiveTorque()
     └── setAdaptiveWrench(f_u, t_u) → WBC

  5. wbc_->update()                              // 全身 QP
     └── 12维关节力矩

  6. 力矩 → Gazebo / 真机
```

---

## 十、已知限制

- **欧拉角近似**（非 SO(3) 四元数误差）：大角度旋转误差。论文使用四元数误差 `e_o := η ε_d − η_d ε − ε_d × ε`，但当前代码用的欧拉角差
- **ω̇_r = 0 准静态假设**：回归矩阵中参考角加速度设为 0
- **不区分摆动/支撑腿**：CLF 约束在所有接触状态下活跃
- **固定 Γ 增益**：无法在线调整适应速度
- **仅在仿真测试**：未在 Go2 真机验证

---

## 十一、配置参数

```ini
# task.info

legged_robot_interface {
  useAclf    true          ; 开启/关闭 ACLF-MPC
}

acl {
  lambdaGain      5.0      ; Λ_l = Λ_o = lambdaGain * I₃
  gammaMass       5.0      ; 负载质量自适应增益
  gammaCom        1.0      ; CoM偏移自适应增益
  gammaInertia    0.01     ; 负载惯量自适应增益
  gammaWrench     0.1      ; 恒定wrench自适应增益
  KDDiag { 50,50,50, 80,80,80 }   ; 线运动50, 角运动80
}

aclSoftConstraint {
  clfWeight      10.0      ; CLF约束在cost中权重
  mu             0.1       ; 松弛障碍参数
  delta          5.0       ; 松弛障碍delta
}
```

---

## 十二、代码-论文对应关系

本节分析当前 workspace 中两套 ACLF 实现与论文公式的对应关系。

### 12.1 旧版 ACLF（ocs2_legged_robot/adaptive/，已嵌入 legged_interface）

对应论文 Sec. III-IV 的完整实现。

**参数化（论文式 5, 6, 23）**

- 文件：`AdaptiveParams.h`，π̂_u 16 维布局见上第三章
- 对应论文：`π_u = [π_u^in (10维); π_u^f (6维)] = [m_u, h_u, vec(I_u), f_const, t_const]`
- 匹配度：完全对应。论文式 23 的 π_u^in 和 π_u^f 与代码一致

**回归矩阵（论文式 9）**

- 文件：`AdaptiveWrenchEstimator.cpp`，`AdaptiveRegressor::computeRegressor()`
- 对应论文：Y_u ∈ R^{6×16}，v̇_pr − g 对应质量项，S(v̇_pr−g) 对应 h_u 项，L(ω̇_r) + S(ω)·L(ω_r) 对应惯量项
- 差异：论文中回归矩阵使用参考速度 v_r = σ + v 和参考加速度 v̇_r，代码中使用了 v̇_pr 和 ω̇_r（当前设为 0，即准静态假设）

**滑模面（论文式 24 + 文本）**

- 文件：`AdaptiveClfConstraint.cpp`，`computeCompositeError()`
- 对应论文：σ_l = ṽ_p + Λ_l p̃（完全一致），σ_o = ω̃ + Λ_o θ̃（代码用欧拉角差，论文用四元数误差式 24）
- 差异：这是已知限制，论文明确说应该用四元数误差

**CLF 约束 h_clf（论文式 11）**

- 文件：`AdaptiveClfConstraint.cpp`，`computeConstraintResidual()` + `getValue()`
- 对应论文：RHS 计算 = −appliedForce/I−1 torque + nominal + adaptive，h_clf = −σ^T·RHS − 0.5·σ^T·K_D·σ
- 匹配度：完全对应。appliedForce = Σ λ_EEi，nomForce = m*g，adapForce = m_u*g + f_const
- 差异：论文的 RHS 中 Y_n π_n 包含了完整的 M_n v̇_r + C_n v_r + g_n，而代码用重力 + 陀螺项的简化形式（因为 SRBD 模型下 v̇_r = 0 和 ω̇_r = 0）

**参数更新律（论文式 8）**

- 文件：`LeggedController.cpp`，`updateAdaptiveParams()`
- 对应论文：`π̂̇_u = Γ Y_u^T σ`
- 离散实现：`pi_hat += Gamma * Y_u^T * sigma * dt`（我们的修复：将 5 参数神秘调用改为内联计算）
- 匹配度：完全对应

**自适应动力学补偿（论文式 10, 16, 17）**

- 文件：`LeggedRobotPreComputation.h`，`setAdaptiveWrench()` + `getAdaptiveForce/Torque()`
- 对应论文：MPC 动力学中的 `−Y_u π̂_u` 项。代码通过 PreComputation 传递 f_u, t_u 给 WBC 层，而非直接修改 MPC 动力学
- 差异：论文要求 MPC 内部动力学包含 `−Y_u π̂_u`（式 10），代码在 WBC 层做补偿。这是架构层面的简化，等价于将自适应项从 MPC 约束移到 WBC 力矩输出

**MPC 中作为软约束（论文 Sec. III-A）**

- 文件：`LeggedInterface.cpp`，`setupOptimalControlProblem()`
- 对应论文：h_clf 作为 relaxed log-barrier 软约束（式 7f）
- 实现：`problemPtr_->softConstraintPtr->add("adaptiveCLF", ...)`，μ=0.1, δ=5.0
- 匹配度：完全一致

### 12.2 新版 ACLF（ocs2_legged_robot_adaptive/，独立包）

简化版本，对应论文 Sec. III 的思路但采用了不同的估计策略。

**扰动估计（不同于论文式 8）**

- 文件：`AdaptiveDisturbanceEstimator.cpp`
- 更新律：`d̂ -= Γ·σ·dt`（6D wrench，非 16D 物理参数）
- 对比论文：论文更新的是物理参数 π̂_u（式 8），新包直接更新扰动 wrench d̂ = [f_ext; t_ext]
- 数学关系：如果 Y_u 已知，则 `Y_u π̂_u = d̂`。新包跳过了显式参数化，直接估计合力/力矩

**CLF 约束（论文式 11 的简化版）**

- 文件：`AdaptiveClfConstraint.cpp`
- 公式：`h = ε − σ^T·d̂ − c·‖σ‖² ≥ 0`
- 对比论文式 11：`h_clf = −σ^T[−Sτ + Y_n π_n + Y_u π̂_u] − ½ σ^T K_D σ`
- 差异：论文的 RHS 包含控制输入 τ（dfdu ≠ 0），新包去掉动力学项只保留 σ 和 d̂（dfdu = 0）

**输入偏置（论文式 16, 17 的显式实现）**

- 文件：`AdaptiveInputCost.h`，`AdaptiveInputBiasCost`
- 对应论文：将自适应估计分配到名义接触力 `Sτ = S w + Y_u π̂_u`
- 实现：`nominalInput += compensationPerFoot`，均分估计外力到各支撑腿

### 12.3 对应关系总表

```
论文公式                旧版 ACLF 代码位置                    新版 ACLF 代码位置
──────────────────────────────────────────────────────────────────────────────
式 5  τ_u = Y_u π_u     AdaptiveParams.h (16维)              无（直接估计 d̂）
式 6  Slotine-Li 性质   AdaptiveParams (M>0, Ṁ−2C 反对称)   无（不使用此性质）
式 8  π̂̇_u = Γ Y_u^T σ  LeggedController::updateAdaptive    AdaptiveDisturbanceEstimator
                         Params()                            (d̂ -= Γ·σ·dt)
式 9  Y_u 定义           AdaptiveRegressor::computeRegressor 无回归矩阵
式 10 自适应动力学       LeggedRobotPreComputation (WBC层)   无（在代价中偏置）
式 11 h_clf 不等式       AdaptiveClfConstraint (ocs2版)       AdaptiveClfConstraint (新包版)
式 12 V, W 定义          未显式实现（隐含在 h_clf 中）        未显式实现
式 15 V̇ 不依赖 π_u       由 h_clf 公式隐式保证               未使用此推导路径
式 16 Sτ = Sw + Y_u π̂_u PreComputation (WBC侧)              AdaptiveInputBiasCost (MPC侧)
式 24 四元数误差         computeCompositeError (欧拉角近似)   computeCompositeError (欧拉角)
```

### 12.4 架构层面的关键差异

**论文要求**：MPC 内部动力学（式 7b, 10）包含自适应项 −Y_u π̂_u，使得 MPC 求解时就已知自适应补偿。

**旧版代码**：自适应补偿发生在 WBC 层（通过 `setAdaptiveWrench` 传递到 PreComputation），MPC 求解时仍使用名义动力学。这在数学上等价于先求名义解再加自适应校正，但 MPC 本身不知道补偿的存在。

**新版代码**：自适应补偿发生在 MPC 层（通过 `AdaptiveInputBiasCost` 偏置名义输入），更接近论文的式 16-17 架构。

---

## 十三、参考资料

- **论文**: Minniti, Grandia, Farshidian, Hutter. *"Adaptive CLF-MPC With Application To Quadrupedal Robots"*, IEEE RA-L, 2021. [arXiv:2112.04536](https://arxiv.org/abs/2112.04536)
- **OCS2**: [github.com/leggedrobotics/ocs2](https://github.com/leggedrobotics/ocs2)
- **Legged Control**: [github.com/qiayuanliao/legged_control](https://github.com/qiayuanliao/legged_control)
- **视频**: [youtu.be/Gu2mfAAvT0A](https://youtu.be/Gu2mfAAvT0A)
