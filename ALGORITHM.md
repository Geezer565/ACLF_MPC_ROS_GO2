# ACLF-MPC 算法

> 基于 Minniti et al. 2021 + Liu et al. 2025 (迁移中)

## 一、自适应动力学 (SRBD)

```
ṗ   = v_p
θ̇   = T(θ)·ω
v̇_p = g + (1/m)·[Σ R_WB·λ_EEi − R_WB·f_u]
ω̇   = I⁻¹·[−ω×Iω + Σ r_EEi×λ_EEi − t_u]
```

不确定性 wrench `(f_u, t_u)` 有两种估计方式:

| | Legacy (论文 A) | RBF (论文 B) |
|---|---|---|
| 参数化 | `Y_u·π̂_u`, π̂_u ∈ R^16 | Dual-RBFNN 直接输出 |
| 更新律 | `π̂̇_u = Γ·Y_u^T·σ` | `Ẇ = −Γ·σ·h(χ)^T` |
| 步长 Γ | 固定 | 固定 (Phase 1) → 变步长 (Phase 3) |

## 二、Legacy: 16维参数 + 回归矩阵

```
π̂_u = [m_u(1), h_u(3), vec(I_u)(6), f_const(3), t_const(3)]
```

| 索引 | 参数 | 物理含义 |
|------|------|----------|
| [0] | m_u | 负载质量偏移 |
| [1-3] | h_u = m_u·c_u | 一阶质量矩 |
| [4-9] | vec(I_u) | 负载惯量 (6个独立分量) |
| [10-12] | f_const | 恒定外力 |
| [13-15] | t_const | 恒定外力矩 |

回归矩阵 `Y_u ∈ R^{6×16}`:
```
Y_f = [v̇_pr−g | 0₃ | 0₆ | I₃ | 0₃]   (力通道)
Y_t = [0 | −S(v̇_pr−g) | L(ω̇_r)+S(ω)·L(ω_r) | 0₃ | I₃]   (力矩通道)
```

## 三、RBF: Dual-RBFNN 直接估计

```
输入 χ ∈ R^12: 位姿/速度误差
隐藏层: N 个高斯核 h_j(χ) = exp(−||χ−c_j||²/b_j²)
输出: Δŵ ∈ R^6 = W·h(χ)  →  [f_u; t_u]

权值更新: Ẇ = −Γ·σ·h(χ)^T − λ·W   (梯度下降 + L2正则)
```

## 四、CLF 不等式约束

**滑模面** (6维): `σ_l = ṽ_p + Λ_l·p̃`, `σ_o = ω̃ + Λ_o·θ̃`

**Legacy 约束** (论文 A Eq.11):
```
h_clf = −σ^T·[−appliedWrench + nominalWrench + Y_u·π̂_u] − ½σ^T·K_D·σ ≥ 0
```

**RBF 约束** (论文 B 适配):
```
h_clf = −σ^T·[−appliedWrench + nominalWrench + RBFNN(χ)] − ½σ^T·K_D·σ ≥ 0
```

约束在 MPC 中作为 **relaxed barrier 软约束** (μ=0.1, δ=5.0)，加入代价函数。

## 五、Lyapunov 稳定性

**候选函数**: `V = ½σᵀ·M·σ + ½π̃ᵀ·Γ⁻¹·π̃` (Legacy) / `+½tr(W̃ᵀ·W̃)` (RBF)

**导数**: `V̇ = σᵀ·[−Sτ + Y_n·π_n + adaptiveWrench]` ≤ −½σᵀ·K_D·σ < 0 (σ≠0)

CLF 约束 h_clf ≥ 0 等价于 V̇ ≤ −½σᵀ·K_D·σ，保证 σ → 0 ⇒ q̃ → 0。

## 六、关键参数

```ini
acl {
  lambdaGain   5.0    # Λ_l = Λ_o = 5·I₃ (滑模面增益)
  gammaMass    5.0    # 质量自适应速率
  gammaCom     1.0    # CoM 自适应速率
  gammaInertia 0.01   # 惯量自适应速率
  gammaWrench  0.1    # 恒力自适应速率
  KDDiag { 50,50,50, 80,80,80 }  # CLF 阻尼
}

rbf {
  nCenters          21     # RBF 中心数 (2m+1, m=10)
  learningRateForce 0.5    # 力通道学习率
  learningRateTorque 0.1   # 力矩通道学习率
  weightDecay       1e-4   # L2 正则
}
```

## 七、已知限制

- **欧拉角近似** (非四元数误差): 大角度下 σ_o 不准确
- **ω̇_r = 0 假设**: Legacy 回归矩阵中参考角加速度设为 0
- **固定 Γ**: RBF Phase 1 仍用固定步长 (Phase 3 引入变步长)
- **仅仿真验证**: 未在 Go2 真机上测试 ACLF/RBF

## 八、参考资料

- [论文 A (ACLF-MPC)](https://arxiv.org/abs/2112.04536) — Minniti et al., IEEE RA-L 2021
- [论文 B (VAN-MPC)](https://doi.org/10.1109/TMECH.2025.3528106) — Liu et al., IEEE/ASME TMECH 2025
- [CLF-MPC 基础](https://github.com/leggedrobotics/ocs2) — Grandia et al., RSS 2020
- [Slotine-Li 自适应控制] — IJRR 1987
