# TARGET — 将 VAN-MPC (球形机器人) 自适应框架迁移到 Go2 四足机器人

> **源论文 A** (当前架构): Minniti, Grandia, Farshidian, Hutter — *Adaptive CLF-MPC With Application To Quadrupedal Robots*, IEEE RA-L 2021
> **源论文 B** (目标能力): Liu, Hu, Guan, Wang, Zhang, Wang, Li — *Adaptive MPC-Based Multi-Terrain Trajectory Tracking Framework for Mobile Spherical Robots*, IEEE/ASME TMECH 2025
> **迁移目标**: 在 Go2 四足机器人上，将论文 A 的 **ACLF-MPC** 框架升级为论文 B 的 **VAN-MPC** 风格的自适应 MPC，核心是引入 **Dual-RBFNN 在线学习不确定性** + **复合误差驱动** + **变步长自适应**

---

## 一、两篇论文的数学框架对比

### 1.1 系统动力学模型

| | 论文 A (ACLF-MPC, 四足) | 论文 B (VAN-MPC, 球形) |
|---|---|---|
| **机器人** | 四足 ANYmal/Go2 | 球形机器人 |
| **控制模型** | 浮基单刚体动力学 (SRBD) | 简化运动学 + 动力学 |
| **状态 x** | [p, θ, v_p, ω] ∈ R^12 | [X, Y, φ] ∈ R^3 |
| **控制输入 u** | [λ_EEi (接触力), ξ_j (关节速度)] | [v_des, qr_des] (速度+横滚角指令) |
| **不确定性来源** | 负载质量/CoM/惯量, 外部力/力矩 | 地面坡度, 摩擦, 外部冲击, 运动学+动力学耦合 |
| **不确定性参数化** | τ_u = Y_u(q,v,v̇)·π_u, π_u ∈ R^16 | 所有不确定性 → 指令补偿 Δu ∈ R^2 |

**论文 A 的 SRBD 动力学** (Eq. 18-21):

```
ṗ   = v_p
θ̇   = T(θ)·ω
v̇_p = g + (1/m)·[Σ_i R_WB·λ_EEi − R_WB·f_u]
ω̇   = I⁻¹·[−ω×Iω + Σ_i r_EEi×λ_EEi − t_u]
```

其中不确定性 wrench `(f_u, t_u) = Y_u(q, v, v̇)·π_u`。

**论文 B 的系统模型** (Eq. 7, 最终形式):

```
ẋ = f(x, u) = f̂(x, u) − f̂'_u·Δu
```

其中 f̂ 是名义模型，f̂'_u = ∂f̂/∂u 是输入雅可比矩阵，Δu 是所有不确定性的等效指令补偿。

### 1.2 不确定性表达方式

| | 论文 A | 论文 B |
|---|---|---|
| **表达形式** | π̂_u ∈ R^16 物理参数 | Δu ∈ R^2 指令补偿 (Dual-RBFNN 输出) |
| **学习器** | 物理回归矩阵 Y_u ∈ R^{6×16} | Dual-RBF 神经网络: Δu ≈ Γ·d2m(h(χ)·W) |
| **更新律** | π̂̇_u = Γ·Y_u^T·σ | Ẇ = −Γ·[E_c^T·f̂'_u·h_1^T·O; E_c^T·f̂'_u·h_2^T·O] |
| **步长 Γ** | **固定常数** | **自适应变量** (由 RBFNN 自身状态决定) |

### 1.3 驱动信号

| | 论文 A | 论文 B |
|---|---|---|
| **主要信号** | σ = ṽ + Λ·q̃ (滑模面, 6维) | E_c = γ·E_e + (1−γ)·E_r (复合误差, 3维) |
| **E_e (模型预测误差)** | 无 | x_k − x_predicted(dt\|k-1) |
| **E_r (轨迹跟踪误差)** | 隐含在 σ 中 | x_k − x_ref |
| **γ (复合系数)** | 无 | γ ∈ (0.5, 1.0)，平衡模型学习 vs 跟踪收敛 |

### 1.4 CLF 约束

| | 论文 A | 论文 B |
|---|---|---|
| **Lyapunov 函数 V** | ½σᵀMσ + ½π̃ᵀΓ⁻¹π̃ | γ/2·E_eᵀE_e + (1−γ)/2·E_rᵀE_r + ½tr(W̃ᵀW̃) |
| **约束形式** | h = −σᵀ[−Sτ+Y_nπ_n+Y_uπ̂_u] − ½σᵀK_Dσ | H = −E_cᵀ[f̂−f̂'_u·Δû] + γE_eᵀx̂̇[-1] + (1−γ)E_rᵀẋ_ref − ½E_cᵀKE_c |
| **dfdu** | ≠ 0 (含控制输入 τ) | = 0 (不直接含 u) |
| **MPC 中处理** | relaxed barrier 软约束 | relaxed barrier 软约束 |

---

## 二、核心迁移策略：四个关键创新点的四足适配

### 创新 1: 不确定性从"物理参数空间"迁移到"指令补偿空间"

**论文 B 的思想**: 将所有复杂不确定性（运动学+动力学，多个未知变量）全部转换为**输入指令的等效补偿量**，用一个低维向量统一表达。

**四足适配**:

在论文 A 中，不确定性由 16 维物理参数通过 6×16 回归矩阵映射为 6 维基座 wrench:
```
[f_u; t_u] = Y_u(q, v, v̇)·π_u,   Y_u ∈ R^{6×16}, π_u ∈ R^{16}
```

迁移后，用一个 **Dual-RBFNN 直接输出 6 维扰动 wrench**:
```
Δw = [Δf; Δt] = DualRBFNN(χ; Ŵ),   Δw ∈ R^6
```

**优势**:
- 不再需要预知不确定性的物理结构（负载?地面?摩擦?关节耦合?）
- 维度降低: 16 维 → 6 维，且无需计算复杂的 6×16 回归矩阵
- RBFNN 可以学习任意非线性不确定性映射

**注意**: 论文 B 的 Δu 是 2 维的（球形机器人只有速度和横滚角两个指令），四足上是 6 维的（基座力+力矩），但框架结构完全相同。

### 创新 2: 引入复合误差 E_c，平衡模型学习与轨迹跟踪

**论文 B 的思想**: 仅靠跟踪误差 E_r 驱动自适应会导致两个问题:
1. 跟踪误差变小时，学习停止（不确定性还未完全获取）
2. 跟踪误差变化与模型误差变化不同步，容易超调和振荡

**四足适配**:

定义四足系统的两种误差:

**模型预测误差 E_e** (6维，基座位姿):
```
E_e(k) = x_base(k) − x̂_base(dt|k−1)
```
其中:
- x_base = [p; θ] ∈ R^6 （基座位置 + 欧拉角）
- x̂_base(dt|k−1) = 上一周期 MPC 预测的当前时刻状态

**轨迹跟踪误差 E_r** (6维，用滑模面 σ 表达):
```
E_r ≡ σ = [ṽ_p + Λ_l·p̃; ω̃ + Λ_o·θ̃] ∈ R^6
```
（论文 B 中 E_r 直接用位姿误差 x − x_ref，但在四足上我们沿用 σ，因为它同时编码了位置和速度误差，具有相对阶1的性质）

**复合误差** (6维):
```
E_c = γ·E_e + (1−γ)·σ,    γ ∈ (0.5, 1.0)
```

**优势**:
- 当跟踪误差 σ → 0 时，E_e 仍能驱动模型向真实动力学收敛
- 当初始距离大时，E_r 分量加速跟踪收敛
- γ 靠近 1（如 γ=0.7~0.9）强调模型学习，避免振荡

**与论文 A 的关键区别**: 论文 A 只有 σ（纯跟踪信号），论文 B 的框架增加了 E_e（模型学习信号），两者在 E_c 中融合。

### 创新 3: Dual-RBF 神经网络替代物理回归矩阵

**论文 B 的思想**: 用神经网络而非手工推导的回归矩阵来表达不确定性，使其具有通用性。

**四足适配的 Dual-RBFNN 设计**:

**网络结构**:
```
输入 χ ∈ R^N: 系统状态特征
隐藏层: 2m+1 个高斯 RBF 基函数，2 个子网络
输出: Δw = [Δf (3维); Δt (3维)] ∈ R^6

Δw = Γ·d2m(h(χ)·Ŵ)

其中:
- h_kj(χ) = exp(−||χ − c_j||²_{Ok} / b_j²), k=0,1; j=0,...,2m
- Ŵ ∈ R^{(2m+1)×2}: 网络权值 (使用权值分解到 6 维输出)
- Γ = diag(Γ1, Γ2): 步长缩放系数
- c_j: RBF 中心向量, b_j: 宽度参数
```

**输入特征 χ 设计** (针对四足):

| 特征 | 维度 | 说明 |
|------|------|------|
| 基座位置误差 p̃ | 3 | p − p_des |
| 基座姿态误差 θ̃ | 3 | θ − θ_des (欧拉角差) |
| 基座速度误差 ṽ_p | 3 | v_p − v_des |
| 基座角速度误差 ω̃ | 3 | ω − ω_des |
| 模型预测位置误差 | 3 | p_current − p_predicted |
| 模型预测姿态误差 | 3 | θ_current − θ_predicted |
| MPC 预测的优化加速度 v̇_pr | 3 | 来自 OCS2 optimizedState |
| 接触状态标志 | 4 | 各足是否触地 (0/1) |
| **总计** | **~25** | 可调整 |

**输出**: 6 维扰动 wrench Δŵ = [Δf_x, Δf_y, Δf_z, Δt_x, Δt_y, Δt_z]

**自适应权值更新律** (替换论文A的 π̂̇_u = Γ·Y_u^T·σ):

对于四足系统，RBFNN 输出直接进入 SRBD 动力学作为补偿项。定义修改后的动力学为:
```
v̇_p = g + (1/m)·[Σ R_WB·λ_EEi − R_WB·Δf̂]
ω̇   = I⁻¹·[−ω×Iω + Σ r_EEi×λ_EEi − Δt̂]
```

其中 Δf̂ = Γ_1·h_1(χ)·Ŵ_1, Δt̂ = Γ_2·h_2(χ)·Ŵ_2。

权值更新被设计为使 Lyapunov 函数递减:
```
Ŵ̇_k = −Γ_k · E_c^T · B_k · h_k(χ)^T · O_k,   k=1,2
```

其中 B_k 是将复合误差映射到对应力/力矩通道的矩阵（对于四足 SRBD，B_1 为加速度通道的映射，B_2 为角加速度通道的映射）。

**初始化**:
- c_j 在状态空间均匀采样 (j=0,...,2m)
- b_j 设为覆盖相邻中心的距离
- Ŵ 初始化接近零（零不确定性先验）
- m 建议值: 10-20 (RBF 基函数数量)

### 创新 4: 变步长算法

**论文 B 的思想**: 学习步长 Γ 不应是常数。在未补偿不确定性大时→大步长加速收敛，近真实值时→小步长维持稳定。

**四足适配** (直接适用):

```
定义未补偿不确定性水平指标:
ζ_k = argmin_j ||χ − c_j||²_O − m        (j=0,...,2m)

用 replay buffer B 平滑:
ζ̄_k = E[ζ_i | ζ_i ∈ B]

变步长:
Γ_1 = min(a_v·(ζ̄_k^v)² + b_v, c_v)     // 力通道步长
Γ_2 = min(a_q·(ζ̄_k^qr)² + b_q, c_q)    // 力矩通道步长
```

**参数建议**:
- a_v, a_q: 二次系数，控制步长增长速率
- b_v, b_q: 基础步长（最小步长）
- c_v, c_q: 步长上限（防止过大导致不稳定）

---

## 三、迁移后的完整数学框架

### 3.1 自适应 SRBD 动力学

```
ṗ   = v_p
θ̇   = T(θ)·ω
v̇_p = g + (1/m_n)·[Σ_i R_WB·λ_EEi − R_WB·Δf̂]
ω̇   = I_n⁻¹·[−ω×I_n·ω + Σ_i r_EEi×λ_EEi − Δt̂]

Δŵ = [Δf̂; Δt̂] = DualRBFNN(χ; Ŵ, Γ)
```

### 3.2 复合误差

```
σ(k) = ṽ(k) + Λ·q̃(k)                                    // 滑模面(跟踪误差)
E_e(k) = x_base(k) − x̂_base(dt|k−1)                     // 模型预测误差
E_c(k) = γ·E_e(k) + (1−γ)·σ(k)                          // 复合误差, γ∈(0.5,1.0)
```

### 3.3 自适应更新律

```
Ŵ̇ = −Γ(χ)·E_c^T·B(χ)·h(χ)^T·O        // RBFNN 权值更新
Γ = f_var_stepsize(||χ−c_j||)          // 变步长
```

离散实现 (dt 为控制周期):
```
Ŵ(k+1) = Ŵ(k) − Γ_k·E_c(k)^T·B_k·h(χ_k)^T·O·dt
Γ_k = min(a·ζ̄_k² + b, c)
```

### 3.4 CLF 约束 (MPC 内)

```
h_clf = −σᵀ·[−appliedWrench + nominalWrench + Δŵ(χ,Ŵ)] − ½σᵀ·K_D·σ ≥ 0
```

其中:
- appliedWrench = [Σλ_EEi; I⁻¹·Σ(r_EEi×λ_EEi)] (6维)
- nominalWrench = [m·g; I⁻¹·(−ω×Iω)] (6维)
- Δŵ(χ,Ŵ) = RBFNN 输出的自适应补偿 (6维)

### 3.5 Lyapunov 稳定性

**候选函数**:
```
V(σ, E_e, W̃) = ½σᵀ·M(q)·σ + γ/2·E_eᵀ·E_e + ½tr(W̃ᵀ·W̃)
```

**导数** (沿系统轨迹):
```
V̇ = σᵀ[−Sτ+Y_nπ_n+Y_uπ̂_u] + γE_eᵀĖ_e + tr(W̃ᵀŴ̇)
```

通过选择自适应律使 tr(W̃ᵀŴ̇) 项抵消不确定性残差，CLF 约束确保 V̇ ≤ −½σᵀK_Dσ < 0 (σ≠0)。

---

## 四、分阶段实现路线图

### Phase 1: Dual-RBFNN 自适应估计器（替换 Y_u 回归矩阵）

**目标**: 用 RBFNN 直接输出 6 维扰动 wrench，替代 16 维物理参数 + 6×16 回归矩阵的计算链路。

**改动范围**:

| 操作 | 文件 | 说明 |
|------|------|------|
| 新增 | `ocs2_legged_robot/include/.../adaptive/RbfAdaptiveEstimator.h` | RBFNN 类定义: 网络结构、前向推理、权值更新 |
| 新增 | `ocs2_legged_robot/src/adaptive/RbfAdaptiveEstimator.cpp` | RBFNN 实现: h(χ) 计算、权值更新、变步长 |
| 修改 | `ocs2_legged_robot/include/.../adaptive/AdaptiveParams.h` | 添加 RBFNN 成员，保留原 π̂_u 用于对比实验 |
| 修改 | `legged_interface/include/.../SwitchedModelReferenceManager.h` | 暴露 RBFNN 接口 |
| 修改 | `legged_controllers/src/LeggedController.cpp` | `updateAdaptiveParams()` 调用 RBFNN 替代 Y_u 回归 |

**关键实现细节**:

```
RbfAdaptiveEstimator 类:
  // 构造函数
  RbfAdaptiveEstimator(int numCenters, int inputDim, int outputDim);
  
  // 前向传播: χ → Δŵ
  Vector6d forward(const VectorXd& chi);
  
  // 权值更新 (自适应律)
  void updateWeights(const Vector6d& E_c, const VectorXd& chi, double dt);
  
  // 变步长计算
  void updateStepSize(const VectorXd& chi);
  
  // RBF 核函数
  double rbfKernel(const VectorXd& chi, const VectorXd& center, double width);
  
  // 成员
  MatrixXd centers_;      // RBF 中心 (2m+1)×inputDim
  VectorXd widths_;       // RBF 宽度 (2m+1)
  MatrixXd weights_1_;    // 子网络1权值 (力通道)
  MatrixXd weights_2_;    // 子网络2权值 (力矩通道)
  VectorXd gamma_;        // 自适应步长 (变步长)
  VectorXd chi_recent_;   // 最近的输入 (用于变步长)
```

**Phase 1 成功标准**:
- RBFNN 输出的 Δŵ 能在仿真中收敛到注入的已知扰动
- 携带负载（10kg 质量偏移）时位姿跟踪误差与论文 A ACLF 方法相当或更好
- 回归矩阵 Y_u 的物理参数路径与 RBFNN 路径可通过开关切换

### Phase 2: 复合误差 E_c 集成

**目标**: 引入模型预测误差 E_e，构造复合误差 E_c = γ·E_e + (1−γ)·σ。

**改动范围**:

| 操作 | 文件 | 说明 |
|------|------|------|
| 修改 | `legged_controllers/src/LeggedController.cpp` | 计算 E_e = x_current − x_predicted |
| 修改 | `ocs2_legged_robot/src/adaptive/RbfAdaptiveEstimator.cpp` | 用 E_c 替代 σ 驱动权值更新 |
| 修改 | `ocs2_legged_robot/src/constraint/AdaptiveClfConstraint.cpp` | CLF 约束用 E_c 的对应项 |
| 修改 | `legged_interface/src/LeggedInterface.cpp` | 传递 x_predicted 给控制器 |

**关键实现**:
```
// 在 LeggedController::update() 中:
// 1. 获取 MPC 预测状态
Eigen::VectorXd x_predicted = mpcMrtInterface_->getPredictedState(dt);
Eigen::VectorXd x_current = measuredRbdState_;

// 2. 计算误差
E_e = x_current.head(6) − x_predicted.head(6);  // 基座位姿误差
sigma = computeCompositeError(x_current, x_des);  // 现有滑模面

// 3. 复合误差
E_c = gamma_ * E_e + (1.0 - gamma_) * sigma;       // γ∈(0.5, 1.0)

// 4. 用 E_c 更新 RBFNN
rbfEstimator_->updateWeights(E_c, chi, dt);
```

**Phase 2 成功标准**:
- E_e 收敛速度 > σ 收敛速度（表明模型在学习）
- γ=0.7 时跟踪性能与收敛速度平衡最优
- 突加扰动后 E_c 响应比纯 σ 驱动快 30%+

### Phase 3: 变步长算法

**目标**: 实现由 RBFNN 自身状态决定的变步长 Γ，消除收敛速度与超调的矛盾。

**改动范围**:

| 操作 | 文件 | 说明 |
|------|------|------|
| 修改 | `ocs2_legged_robot/src/adaptive/RbfAdaptiveEstimator.cpp` | 实现 `updateStepSize()` |
| 新增 | `ocs2_legged_robot/include/.../adaptive/ReplayBuffer.h` | 平滑缓冲区 |

**关键实现**:
```
void RbfAdaptiveEstimator::updateStepSize(const VectorXd& chi) {
    // 1. 计算距离指标
    double zeta_v = 0, zeta_qr = 0;
    double minDistV = 1e10, minDistQr = 1e10;
    for (int j = 0; j < nCenters_; j++) {
        double dist_v = (chi - centers_.col(j)).squaredNorm() * pow(o0, j-m);
        double dist_qr = (chi - centers_.col(j)).squaredNorm() * pow(o1, j-m);
        if (dist_v < minDistV) { minDistV = dist_v; zeta_v = (j - m) / (double)m; }
        if (dist_qr < minDistQr) { minDistQr = dist_qr; zeta_qr = (j - m) / (double)m; }
    }
    
    // 2. 缓冲区平滑
    buffer_v_.push_back(zeta_v);
    buffer_qr_.push_back(zeta_qr);
    double zeta_v_smooth = mean(buffer_v_);
    double zeta_qr_smooth = mean(buffer_qr_);
    
    // 3. 变步长计算
    gamma_(0) = std::min(a_v_ * zeta_v_smooth * zeta_v_smooth + b_v_, c_v_);
    gamma_(1) = std::min(a_q_ * zeta_qr_smooth * zeta_qr_smooth + b_q_, c_q_);
}
```

**Phase 3 成功标准**:
- 切换地形（平面→斜坡）时步长自动增大，跟踪误差回落快于固定步长
- 稳态下步长自动缩小，无超调和振荡
- 与固定步长 ACLF 对比，收敛时间减少 40%+

### Phase 4: 多地形仿真验证

**目标**: 在多种地形下验证 VAN-MPC 框架的有效性。

**测试场景**:
| 地形 | 不确定性特征 | 测试内容 |
|------|-------------|----------|
| 平地板 | 基准(无不确定性) | 验证名义性能不退化 |
| 斜坡 (10°, 20°) | 重力方向偏差 | 基座姿态保持 |
| 碎石地面 | 随机接触力扰动 | 步态稳定性 |
| 软地面 | 接触刚度变化 | 足端下陷补偿 |
| 负载扰动 | 未知 CoM 偏移 | 姿态跟踪 |
| 地形过渡 (平地→斜坡) | 不确定性突变 | 自适应响应速度 |

**评估指标**:
- 基座位姿 RMSE (位置/姿态)
- 不确定性估计收敛时间
- 步态成功率
- 能耗
- 与原始 MPC、论文 A ACLF 的对比

---

## 五、代码架构变更总览

### 迁移前的数据流 (当前)

```
LeggedController::update():
  1. updateStateEstimation()          → measuredRbdState_
  2. mpcMrtInterface_->evaluatePolicy() → optimizedState, optimizedInput
     └─ [OCS2] AdaptiveClfConstraint::getValue()
        └─ 读取 adaptiveParams_->pi_hat → 计算 Y_u·π̂_u → h_clf
  3. updateAdaptiveParams():
     ├─ σ = computeCompositeError()
     ├─ Y_u = AdaptiveRegressor::computeRegressor()    ← 6×16 矩阵
     ├─ π̂_u += Γ·Y_u^T·σ·dt                            ← 固定Γ
     └─ setAdaptiveWrench(f_u, t_u)
  4. wbc_->update(optimizedState, adaptiveWrench)
  5. 力矩下发
```

### 迁移后的数据流 (目标)

```
LeggedController::update():
  1. updateStateEstimation()          → measuredRbdState_
  2. mpcMrtInterface_->evaluatePolicy() → optimizedState, x_predicted
     └─ [OCS2] AdaptiveClfConstraint::getValue()
        └─ 读取 rbfEstimator_->forward(χ) → Δŵ → h_clf
  3. updateRbfAdaptiveEstimator():
     ├─ σ = computeCompositeError()
     ├─ E_e = x_current − x_predicted
     ├─ E_c = γ·E_e + (1−γ)·σ
     ├─ χ = buildInputFeatures(E_c, state, contactFlags, ...)
     ├─ Ŵ += Γ(χ)·E_c^T·B(χ)·h(χ)^T·O·dt              ← 变步长Γ
     ├─ Δŵ = rbfEstimator_->forward(χ)
     └─ setAdaptiveWrench(Δf, Δt)
  4. wbc_->update(optimizedState, adaptiveWrench)
  5. 力矩下发
```

### 新增文件清单

```
ocs2_legged_robot/include/ocs2_legged_robot/adaptive/
├── RbfAdaptiveEstimator.h          # Dual-RBFNN 类声明 (~100行)
├── ReplayBuffer.h                  # 变步长平滑缓冲区 (~40行)
└── RbfConfig.h                     # RBFNN 超参数配置 (~50行)

ocs2_legged_robot/src/adaptive/
├── RbfAdaptiveEstimator.cpp        # Dual-RBFNN 实现 (~250行)
└── ReplayBuffer.cpp                # 缓冲区实现 (~50行)

legged_controllers/config/go2/
└── rbf_config.info                 # RBFNN 参数配置文件

legged_interface/include/legged_interface/
└── RbfAdaptiveInterface.h          # RBF 与 MPC 的桥接接口 (~60行)
```

### 修改文件清单

```
legged_controllers/src/LeggedController.cpp    # updateRbfAdaptiveEstimator()
legged_controllers/include/.../LeggedController.h  # RBF 成员变量
legged_interface/src/LeggedInterface.cpp            # RBFNN 注册到 OCS2
ocs2_legged_robot/src/constraint/AdaptiveClfConstraint.cpp  # 用 RBFNN 输出
ocs2_legged_robot/CMakeLists.txt                    # 新增源文件
```

---

## 六、关键参数设计

### RBFNN 参数 (rbf_config.info)

```ini
rbf {
  numCenters        21           # 2m+1, m=10
  inputDim          25           # χ 特征维度
  outputDim         6            # [Δf(3); Δt(3)]
  
  # 中心初始化范围
  centerRange {
    posError        [-0.5, 0.5]  # m
    oriError        [-0.5, 0.5]  # rad
    velError        [-2.0, 2.0]  # m/s
    angVelError     [-2.0, 2.0]  # rad/s
    predPosError    [-0.3, 0.3]  # m
    predOriError    [-0.3, 0.3]  # rad
  }
  
  # 宽度参数
  widthScale        0.5          # b_j 缩放因子
  
  # 子网络权重
  o0                0.7          # 力通道权重
  o1                0.3          # 力矩通道权重
}
```

### 复合误差参数

```ini
compositeError {
  gamma             0.7          # E_c = γ·E_e + (1-γ)·σ
                                 # γ↑ = 更强调模型学习
                                 # γ↓ = 更强调跟踪收敛
}
```

### 变步长参数

```ini
variableStepSize {
  # 力通道 (Δf_x, Δf_y, Δf_z)
  a_v               8.0          # 二次系数
  b_v               0.3          # 基础步长
  c_v               1.5          # 步长上限
  
  # 力矩通道 (Δt_x, Δt_y, Δt_z)
  a_q               0.3          # 二次系数
  b_q               0.06         # 基础步长
  c_q               0.15         # 步长上限
  
  # 缓冲区
  bufferSize        10           # 平滑窗口大小
}
```

### CLF 约束参数 (保留论文 A 配置)

```ini
acl {
  lambdaGain        5.0          # Λ_l = Λ_o = 5·I₃
  KDDiag { 50, 50, 50, 80, 80, 80 }  # K_D 对角阵
}

aclSoftConstraint {
  clfWeight         10.0
  mu                0.1
  delta             5.0
}
```

---

## 七、数学推导一致性检查清单

迁移过程中必须保持以下数学一致性:

- [ ] **Lyapunov 函数正定性**: V > 0 for all (σ ≠ 0, W̃ ≠ 0)
- [ ] **V̇ 负定性**: V̇ ≤ −½σᵀK_Dσ 由 CLF 约束保证
- [ ] **滑模面收敛**: σ → 0 ⇒ q̃ → 0, ṽ̃ → 0
- [ ] **RBFNN 更新方向**: 权值更新使 V̇ 中的 tr(W̃ᵀŴ̇) 项不增加 V̇
- [ ] **Δŵ 的有界性**: RBFNN 输出有界（高斯核性质保证）
- [ ] **CLF 约束可行性**: 对所有可达状态存在可行 u (递归可行性)
- [ ] **与 MPC 代价函数的协调**: RBFNN 补偿不影响 MPC 的最优性
- [ ] **离散化一致性**: 连续时间推导 → 离散时间实现，Euler 积分 dt ≤ 10ms

---

## 八、与论文 A 旧版 ACLF 的共存策略

为保证实验可对比、可回退:

1. **所有修改用编译开关控制**:
   ```cpp
   #ifdef USE_RBF_ADAPTIVE
       rbfEstimator_->updateWeights(E_c, chi, dt);
   #else
       updateAdaptiveParamsLegacy();  // 原 16 参数更新律
   #endif
   ```

2. **配置文件中选择模式**:
   ```ini
   legged_robot_interface {
     adaptiveMode    "rbf"     # "off" | "legacy" | "rbf"
   }
   ```

3. **实验对比矩阵**:
   | 模式 | 不确定性表达 | 驱动信号 | 步长 |
   |------|-------------|----------|------|
   | off | 无 | 无 | − |
   | legacy (论文 A) | Y_u·π̂_u (16维) | σ | 固定 Γ |
   | rbf (Phase 1) | RBFNN (6维) | σ | 固定 Γ |
   | rbf+ec (Phase 2) | RBFNN (6维) | E_c | 固定 Γ |
   | rbf+ec+vs (Phase 3) | RBFNN (6维) | E_c | 变步长 Γ |
   | van-mpc (Phase 4) | RBFNN (6维) | E_c | 变步长 Γ + 多地形 |

---

## 九、参考文献

- **论文 A**: Minniti, Grandia, Farshidian, Hutter. "Adaptive CLF-MPC With Application To Quadrupedal Robots." IEEE RA-L, 2021. [arXiv:2112.04536](https://arxiv.org/abs/2112.04536)
- **论文 B**: Liu, Hu, Guan, Wang, Zhang, Wang, Li. "Adaptive MPC-Based Multi-Terrain Trajectory Tracking Framework for Mobile Spherical Robots." IEEE/ASME TMECH, 2025. DOI: [10.1109/TMECH.2025.3528106](https://doi.org/10.1109/TMECH.2025.3528106)
- **Slotine-Li**: Slotine, Li. "On the Adaptive Control of Robot Manipulators." IJRR, 1987.
- **OCS2**: [github.com/leggedrobotics/ocs2](https://github.com/leggedrobotics/ocs2)
- **CLF-MPC**: Grandia, Taylor, Singletary, Hutter, Ames. "Nonlinear MPC of Robotic Systems with Control Lyapunov Functions." RSS, 2020.
