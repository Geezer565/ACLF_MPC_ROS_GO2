# 01 — 总览：研究路线 + 任务 + 进度

> 项目：Go2 四足机器人 ACLF-MPC 控制框架
> 基础：Minniti, Grandia, Farshidian, Hutter — *Adaptive CLF-MPC With Application To Quadrupedal Robots*, IEEE RA-L 2021
> 控制架构：NMPC (OCS2) + WBC (Hierarchical QP) + Kalman Filter + [待加] RNN 模型补偿

---

## 一、项目总体目标

在现有 NMPC+WBC 框架基础上，完成两个方向的研究：

- **方向 A — 仿真环境验证与多地形测试**：构建多地形仿真环境，验证控制器在不同地面条件下的鲁棒性，优化地面对机器人的反馈力建模。
- **方向 B — 控制器设计增强**：在 WBC 层引入 RNN 网络在线拟合 Pinocchio 质心模型与真实动力学的误差，将传统 PID 积分升级为 CLF 函数引导的可控积分项。

---

## 二、方向 A：仿真环境与多地形验证

### A.1 目标

验证 MPC+WBC 控制器在多种地形下的稳定性和鲁棒性，量化地面对机器人的反作用力反馈，为控制器优化提供数据支撑。

### A.2 技术路线

**第一步 — 地形生成与导入**

- 使用多平台生成多样化地形场景（平面、斜坡、台阶、碎石、草地等）
- 将地形以 URDF/SDF 格式导入 Gazebo，作为 world 文件的一部分
- 目标平台优先级：
  - Gazebo（当前已有，优先扩展 world 文件）
  - Isaac Sim（NVIDIA 生态，GPU 加速物理，ROS2 桥接）
  - GIM（Generalizable Imitation Model 相关工具）

**第二步 — 地面反作用力（GRF）监测与优化**

- 在 Gazebo 接触模型中提取足端接触力（已有 contact sensor: `LF_FOOT, LH_FOOT, RF_FOOT, RH_FOOT`）
- 对比 MPC 预测的接触力 vs Gazebo 物理引擎计算的真实接触力
- 分析不同地形下的力误差分布，优化接触模型参数（kp/kd/mu）

**第三步 — 多地形控制器验证**

- 每种地形上运行标准 gait（trot, dynamic_walk, static_walk）
- 记录指标：姿态跟踪误差、接触力误差、能耗、成功率
- 对比 flat ground vs rough terrain 的 ACLF 自适应效果

### A.3 涉及文件

- `legged_gazebo/worlds/empty_world.world` — 当前平原 world，需扩展为多地形
- `legged_gazebo/config/default.yaml` — 接触传感器配置
- `legged_gazebo/src/LeggedHWSim.cpp` — 接触力读取逻辑 (ContactManager)
- `urdf/common/gazebo.xacro` — 足端接触刚度/阻尼/摩擦

### A.4 多地形 world 创建方法

在 Gazebo 中创建多地形 world 的常用方式：

- **高度图 (Heightmap)**：用灰度图定义地形高度，Gazebo 原生支持 `<heightmap>` 标签
- **模型拼接**：用多个 box/ramp 模型拼接台阶、斜坡
- **外部导入**：从 Isaac Sim 或 Blender 导出 DAE/STL 网格，通过 `<mesh>` 加载
- **程序化生成**：ROS 节点运行时动态生成 SDF 模型

---

## 三、方向 B：控制器设计增强

### B.1 目标

现有 NMPC 使用 Pinocchio 刚体动力学库建立质心模型（Centroidal Model），但该模型与真实物理存在固有误差。目标是在 WBC 层引入 RNN 网络，在线学习并补偿这一模型差异，同时将误差积分机制从传统 PID 升级为 CLF 引导的积分。

### B.2 问题分析

当前控制链路中的误差来源：

- Pinocchio 质心模型假设单刚体 + 无质量腿，忽略腿惯量、关节摩擦、传动间隙
- MPC 求解的 feed-forward 力矩基于该简化模型，与 Gazebo 物理引擎（ODE）的真实响应存在偏差
- 现有 ACLF 框架用 16 维 π̂_u 补偿负载差异，但无法补偿模型结构误差（腿惯量耦合、非线性摩擦等）
- 传统 PID 积分项简单累加历史误差，缺乏方向引导，容易 windup

### B.3 技术路线

**第一步 — RNN 模型差异学习器**

- 在 WBC 层插入一个轻量级 RNN（如 GRU 或 LSTM，2-3 层，隐藏维度 64-128）
- 输入特征：
  - 当前关节状态 (q, q̇) — 12+12 维
  - MPC 预测的优化状态和输入 (optimizedState, optimizedInput)
  - ACLF 自适应参数 π̂_u（当前 16 维估计值）
  - 上一时刻的模型误差（反馈）
- 输出：6 维 wrench 修正量 Δw = [Δf; Δt]，补偿 Pinocchio 模型与真实动力学的差异
- 训练数据：Gazebo 仿真中采集的 (状态, MPC预测力矩, 真实所需力矩) 三元组
- 推理频率：与 WBC 同步（~400Hz），需保证 < 2ms 推理延迟

**第二步 — CLF 引导的积分项设计**

传统 PID 积分：

```
u_int(t) = K_i * ∫ e(τ) dτ      （被动累加，无方向引导）
```

CLF 引导的积分：

```
u_int(t) = K_i * ∫ [ Y_u^T(q, v, v̇) * σ ] dτ
```

其中：
- σ = ṽ + Λ·q̃ 是滑模面（复合误差），直接编码了收敛方向
- Y_u^T * σ 将滑模面映射回参数空间，给出"哪个参数需要怎么调"的方向信息
- 积分不再是盲目的误差累加，而是沿 CLF 梯度下降方向的可控积累

优势：
- 有明确收敛方向（沿滑模面），不会盲目累加
- 天然的 anti-windup：当 σ → 0 时积分自动停止
- 与 ACLF 框架自然衔接：σ 和 Y_u 已在 ACLF 代码中计算

**第三步 — 将 RNN 补偿与 CLF 积分融合**

```
u_final = u_MPC(ff) + u_PD(kp, kd) + u_RNN(Δw) + u_CLF_int(∫Y^Tσ)
           ↑nominal       ↑stabilize     ↑model comp    ↑adaptive integral
```

融合策略：
- `u_MPC`：NMPC 求解的 feed-forward 力矩（基于 Pinocchio 名义模型）
- `u_PD`：低增益 PD 稳定项（当前 kp=0, kd=3，建议加小 kp）
- `u_RNN`：RNN 在线推断的模型差异补偿（高频，~400Hz）
- `u_CLF_int`：CLF 引导的积分项，补偿残余稳态误差（低频，自适应速率）

### B.4 实现位置

在 `LeggedController::update()` 的主循环中，位于 WBC 计算之后、力矩下发之前：

```
现有流程:
  MPC evaluatePolicy → WBC update → setCommand → 下发

改进后:
  MPC evaluatePolicy → WBC update → [RNN infer Δw] → [CLF integral ∫Y^Tσ]
  → setCommand(加总) → 下发
```

### B.5 涉及文件

- `legged_controllers/src/LeggedController.cpp` — 主控制循环，插入 RNN 推理 + CLF 积分
- `legged_wbc/` — WBC QP 求解器，可能需要修改以接受外部 wrench 修正
- `legged_interface/` — 可能需要增加 RNN 模型加载/推理接口
- `ocs2_legged_robot/include/.../adaptive/AdaptiveParams.h` — σ 和 Y_u 已在此计算，复用现有代码
- 新增文件：`legged_controllers/src/RnnModelCompensator.cpp` — RNN 推理模块（LibTorch 或 ONNX Runtime）

### B.6 关键技术细节

**Pinocchio Centroidal Model 与真实误差**

- Pinocchio 计算的是理想刚体动力学：M(q)q̈ + C(q,q̇)q̇ + g(q) = τ
- 真实系统额外包含：腿惯量效应、关节摩擦、传动误差、接触力非线性
- 模型误差 e_model = τ_real − τ_pinocchio，RNN 学习映射 (q,q̇,optimizedState) → e_model

**PI 误差系数的单积分设计**

- 不用传统的 P+I+D 三个独立通道，而是只用一个积分通道
- 但积分的"输入"不是裸误差 e，而是 CLF 梯度投影 Y_u^T*σ
- 这等价于：在 Lyapunov 下降方向上做积分积累，数学上保证 V̇ ≤ −W
- 避免传统 PID 在非线性系统中的 windup 和方向错误问题

**与 ACLF 参数更新律的区别**

- ACLF 更新律 `π̂_u += Γ·Y_u^T·σ·dt` 更新的是物理参数估计（16 维）
- CLF 积分项 `u_int = K_i * ∫ Y_u^T*σ dτ` 输出的是力矩补偿（12 维）
- 两者互补：ACLF 处理参数不确定性（慢变），CLF 积分处理结构误差（需要力矩直接补偿）

---

## 四、当前进度与任务

### 已完成

- 基础仿真环境搭建（Docker + ROS Noetic + Gazebo 11 + GPU 穿透）
- ACLF-MPC 代码框架写入（4 新增文件 + 10 修改文件），待编译
- 一键启动脚本 `/root/start_go2_sim.sh`（Gazebo + 控制器 + 步态）
- 键盘遥控 + cmd_vel 速度控制
- 步态库（12 种步态）可用

### 已修复 Bug

- transmission.xacro 参数 bug（name → prefix）
- ODE physics iters 类型不兼容（已删除）
- Docker GPU 穿透（重启容器加 --device /dev/dri）
- 控制器只 load 不 start（controller_manager → spawner）
- 安全检查阈值（90° → 162°）
- LeggedController.cpp 编译错误（updateAdaptiveParams 内联修复）
- 生成高度冲击（z: 0.5 → 0.35）

### 待做 — 方向 A

- 创建多地形 world：斜坡、台阶、碎石、草地（优先级：高）
- 地面反作用力数据采集与对比分析（优先级：高）
- 多地形控制器稳定性验证（优先级：中）
- Isaac Sim 环境调研与桥接（优先级：低）

### 待做 — 方向 B

- ACLF 代码编译验证 + useAclf 开关测试（优先级：高）
- RNN 网络结构设计与训练数据采集（优先级：高）
- CLF 积分项公式推导与代码实现（优先级：高）
- RNN + CLF 积分与 WBC 融合调试（优先级：中）
- 对比实验：原始 vs ACLF vs ACLF+RNN vs ACLF+RNN+CLF-Int（优先级：中）

### 待做 — 代码优化

- 删除 ROS_WARN 调试刷屏（LeggedController.cpp:112）
- 关节 kp/kd 可配置化（当前硬编码 kp=0, kd=3）
- 安全检查阈值做成 ROS param
- RViz 可视化 + rqt 控制器管理 GUI

---

## 五、常见问题

- **Gazebo 闪退**：端口 11345 被占，`fuser -k 11345/tcp` 杀掉后重试
- **"Authorization required"**：宿主机 `xhost +local:root` 放开 X11 权限
- **机器人不走**：键盘终端需保持前台，长按 i 键不放；同时确认控制器 state=running
- **机器人摔倒**：先发 stance 步态让机器人站稳，再切 trot；调大 Q 矩阵 (10,10) 俯仰和 (11,11) 横滚权重
- **关节角度打印刷屏**：LeggedController.cpp 第 112 行 ROS_WARN 待删除
- **编译 pinocchio 报错**：`ocs2_self_collision/src/PinocchioGeometryInterface.cpp` 第 73 行 API 兼容问题，需手动修改

---

## 六、参考资源

- ACLF-MPC 论文：Minniti et al., IEEE RA-L 2021, [arXiv:2112.04536](https://arxiv.org/abs/2112.04536)
- OCS2：[github.com/leggedrobotics/ocs2](https://github.com/leggedrobotics/ocs2)
- Legged Control：[github.com/qiayuanliao/legged_control](https://github.com/qiayuanliao/legged_control)
- Pinocchio 动力学：[stack-of-tasks.github.io/pinocchio](https://stack-of-tasks.github.io/pinocchio/)
- 同目录详细文档：
  - `02_CODE_ARCHITECTURE.md` — 代码架构与数据流
  - `03_ACLF_ALGORITHM.md` — ACLF 数学推导与算法
  - `04_RUN_GUIDE.md` — 仿真启动与操控
  - `05_TUNING_GUIDE.md` — 全部参数调优
