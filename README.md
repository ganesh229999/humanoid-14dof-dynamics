# 🤖 Humanoid Dual-Arm Dynamic Modelling & Simulation
 
<div align="center">
 
[![MATLAB](https://img.shields.io/badge/MATLAB-R2022b%2B-orange?style=for-the-badge&logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)](https://github.com/ganesh229999/humanoid-14dof-dynamics/blob/master/LICENSE)
[![DOF](https://img.shields.io/badge/DOF-14_(7%2B7)-blue?style=for-the-badge)](#robot-specification)
[![Algorithm](https://img.shields.io/badge/Algorithm-Recursive_Newton--Euler-green?style=for-the-badge)](#mathematical-foundation)
[![Base](https://img.shields.io/badge/Base-Fixed_%26_Mobile-purple?style=for-the-badge)](#simulation-tasks)
 
<br/>
 
**A complete MATLAB implementation of Recursive Newton-Euler (RNE) dynamics for a 14-DOF dual-arm humanoid robot — supporting both fixed-base and mobile-base configurations with cooperative object manipulation.**
 
*Developed during a research internship at the Defence Research & Development Organisation (DRDO), India.*
 
</div>
 
---
 
## 🦾 Robot Architecture
 
```
                        BASE FRAME (Torso / Spine)
                                   │
               ┌───────────────────┴───────────────────┐
               │                                       │
         ARM A (Right)                           ARM B (Left)
               │                                       │
          J1 – Shoulder Flexion / Extension        J1 – Shoulder Flexion / Extension
          │    [Lateral offset: +d₁]               │    [Lateral offset: −d₁]
          │                                        │
          J2 – Shoulder Abduction / Adduction      J2 – Shoulder Abduction / Adduction
          │                                        │
          J3 – Shoulder Internal / External Rot.   J3 – Shoulder Internal / External Rot.
          │    [Upper-arm length: d₂]              │    [Upper-arm length: d₂]
          │                                        │
          J4 – Elbow Flexion / Extension           J4 – Elbow Flexion / Extension
          │                                        │
          J5 – Forearm Pronation / Supination      J5 – Forearm Pronation / Supination
          │    [Forearm length: d₃]                │    [Forearm length: d₃]
          │                                        │
          J6 – Wrist Flexion / Extension           J6 – Wrist Flexion / Extension
          │                                        │
          J7 – Wrist Radial / Ulnar Deviation      J7 – Wrist Radial / Ulnar Deviation
          │    [Wrist offset: d₄]                  │    [Wrist offset: d₄]
          │                                        │
         [EE] Tool Flange                         [EE] Tool Flange
               [Tool offset: d₅]                       [Tool offset: d₅]
               │                                        │
               └───────────────────┬────────────────────┘
                                   │
                          OBJECT / PAYLOAD
                       (Cooperative Manipulation)
```
 
> **14 DOF total** — 7 revolute joints per arm following a human-arm analogy.
> Arm A (right) and Arm B (left) share an identical kinematic structure.
> Only the shoulder lateral offset **d₁** changes sign between arms,
> making the system perfectly symmetric about the torso midplane.
> Gravity acts along the **−X axis** of the base frame (Modified DH convention).
 
---
---
</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [Robot Specification](#-robot-specification)
- [Mathematical Foundation](#-mathematical-foundation)
- [Repository Structure](#-repository-structure)
- [Quick Start](#-quick-start)
- [Module Documentation](#-module-documentation)
- [Simulation Tasks](#-simulation-tasks)
- [Results & Validation](#-results--validation)
- [Dependencies](#-dependencies)
- [Citation](#-citation)

---

## 🔬 Overview

This repository implements a **complete rigid-body dynamics pipeline** for a 14-degree-of-freedom (DOF) dual-arm humanoid robot. The system models both arms (7 DOF each) operating cooperatively to manipulate objects, with full support for:

- **Fixed-base** operation (robot mounted on stationary platform)
- **Mobile-base** operation (robot on a moving ground vehicle)
- **Cooperative manipulation** (both arms jointly handling an object)
- **External force handling** (via partial Jacobian mapping)
- **Real-time 3D animation** with end-effector trajectory tracing

The core algorithm — **Recursive Newton-Euler (RNE)** — achieves O(n) computational complexity versus the O(n³) Lagrangian formulation, making it suitable for real-time control at 1 kHz update rates.

---

## ✨ Key Features

| Feature | Description |
|---------|-------------|
| 🔁 **RNE Dynamics** | Forward + backward recursion computing τ = M(q)q̈ + C(q,q̇)q̇ + G(q) |
| 🦾 **Dual-Arm** | Symmetric 7-DOF arms (A=right, B=left), differentiated by shoulder offset sign |
| 🚗 **Mobile Base** | Khalil-Dombre coupling solves platform acceleration from arm dynamics |
| 📐 **Full DH Chain** | Modified DH convention, 8 transforms per arm (7 joints + EE) |
| 🎯 **DLS Inverse Kinematics** | Damped Least-Squares IK for smooth, singularity-robust trajectories |
| 🧮 **Matrix Decomposition** | Inertia M, Coriolis C, Gravity G computed and exported separately |
| 🌐 **External Forces** | Wrench inputs mapped to joint torques via partial Jacobian |
| 🎬 **3D Animation** | Real-time visualisation with world-frame FK, base motion, EE traces |
| 📊 **Excel Export** | All trajectories and torques exported to structured `.xlsx` files |
| 🍾 **Multi-Task** | Validated on upward lifting AND bottle-opening manipulation scenarios |

---

## 🤖 Robot Specification

### Kinematic Structure

Each arm follows a **7-DOF anthropomorphic configuration** modelled after human arm kinematics:

```
Joint  Anatomy           Axis    DH offset
─────────────────────────────────────────────
  J1   Shoulder flex/ext  Z      d₁ = ±0.318 m  ← sign differentiates A/B
  J2   Shoulder abd/add   Z      —
  J3   Shoulder int/ext   Z      d₂ = −0.251 m
  J4   Elbow flex/ext     Z      —
  J5   Forearm pro/sup    Z      d₃ = −0.234 m
  J6   Wrist flex/ext     Z      —
  J7   Wrist rad/ulnar    Z      d₄ = −0.168 m
  EE   Tool flange        —      d₅ = −0.209 m
```

### DH Parameter Table (Both Arms)

| Row | α (rad) | a (m) | θ_offset (rad) | d (m) |
|-----|---------|-------|----------------|-------|
| 1 | 0 | 0 | π | ±d₁ |
| 2 | −π/2 | 0 | −π/2 | 0 |
| 3 | π/2 | 0 | −π/2 | d₂ |
| 4 | −π/2 | 0 | 0 | 0 |
| 5 | π/2 | 0 | 0 | d₃ |
| 6 | −π/2 | 0 | 0 | 0 |
| 7 | π/2 | 0 | 0 | d₄ |
| EE | 0 | 0 | 0 | d₅ |

### Dynamic Parameters

| Link | Mass (kg) | COM z-offset (mm) | I_zz (kg·m²) |
|------|-----------|-------------------|--------------|
| 1 (Shoulder) | 2.041 | −118.3 | 3.813 |
| 2 | 1.883 | −52.2 | 1.285 |
| 3 (Upper arm) | 1.276 | +142.8 | 0.514 |
| 4 (Elbow) | 1.134 | −8.9 | 0.648 |
| 5 (Forearm) | 0.978 | +125.6 | 0.649 |
| 6 (Wrist) | 0.792 | +9.0 | 0.385 |
| 7 (Hand) | 3.210 | −90.4 | 2.608 |

> **Total arm mass:** ~11.3 kg per arm | **Total system:** ~22.6 kg (arms only)

---

## 📐 Mathematical Foundation

### 1. Recursive Newton-Euler Algorithm

The RNE algorithm computes joint torques in **O(n)** operations using two recursive passes:

#### Forward Pass — Velocity & Acceleration Propagation

```
ωᵢ₊₁ = Rᵢᵀ (ωᵢ + ẑ·θ̇ᵢ)
ω̇ᵢ₊₁ = Rᵢᵀ (ω̇ᵢ + ωᵢ × ẑ·θ̇ᵢ + ẑ·θ̈ᵢ)
v̇ᵢ₊₁ = Rᵢᵀ (v̇ᵢ + ω̇ᵢ × pᵢ + ωᵢ × (ωᵢ × pᵢ))

COM acceleration:
v̇cᵢ = v̇ᵢ₊₁ + ω̇ᵢ₊₁ × rcᵢ + ωᵢ₊₁ × (ωᵢ₊₁ × rcᵢ)
```

#### Backward Pass — Force & Torque Propagation

```
fᵢ = Rᵢ₊₁ fᵢ₊₁ + mᵢ v̇cᵢ
nᵢ = Rᵢ₊₁ nᵢ₊₁ + Iᵢ ω̇ᵢ₊₁ + ωᵢ₊₁ × (Iᵢ ωᵢ₊₁) + rcᵢ × (mᵢ v̇cᵢ) + pᵢ × (Rᵢ₊₁ fᵢ₊₁)

Joint torque:  τᵢ = nᵢ · ẑᵢ
```

#### Equation of Motion

```
τ = M(q)q̈ + C(q,q̇)q̇ + G(q) + τ_ext
```

Where:
- **M(q)**: 7×7 configuration-dependent joint-space inertia matrix
- **C(q,q̇)**: Coriolis/centrifugal torque vector
- **G(q)**: Gravity torque vector (computed via RNE with zero velocities/accelerations)
- **τ_ext**: External wrench contribution via partial Jacobian

### 2. Inertia Matrix Extraction

M is extracted column-by-column using **unit joint acceleration perturbations**:

```matlab
M(:,j) = rne(q, 0, eⱼ) − G(q)     % eⱼ = j-th basis vector
```

This exploits RNE's linearity in q̈ to recover M in exactly 7 additional RNE calls.

### 3. Damped Least-Squares Inverse Kinematics

```
q̇ = J†_dls · Jo · Vc_d

where  J†_dls = (JᵀJ + λ²I)⁻¹ Jᵀ    (λ = 1×10⁻⁴)
```

The damping factor λ prevents velocity blow-up near kinematic singularities while introducing bounded position error proportional to λ².

### 4. Mobile Base — Khalil-Dombre Coupling

For a mobile platform, arm and base dynamics are coupled:

```
Mbase · ä_base = −[Jrc_A·q̈_A + Jrc_B·q̈_B + βrc]

where:
  Mbase = ΣᵢmᵢI₃ + platform inertia      (3×3 effective base inertia)
  Jrc   = [Jrc_A | Jrc_B]                (3×14 reaction Jacobian)
  βrc   = Σⱼ (z₀ⱼ × (z₀ⱼ × v₀)) θ̇ⱼ    (velocity-dependent bias)
```

---

## 📁 Repository Structure

```
humanoid-14dof-dynamics/
│
├── 📄 README.md
├── 📄 LICENSE
│
├── src/
│   ├── kinematics/
│   │   ├── get_robot_params.m          ← DH params, masses, inertias, transforms
│   │   ├── get_robot_paramsmobile.m    ← Mobile-base extension + Khalil-Dombre terms
│   │   ├── jacobianOfArmA.m            ← 6×7 geometric Jacobian, Arm A
│   │   ├── jacobianOfArmB.m            ← 6×7 geometric Jacobian, Arm B
│   │   ├── computePartialJacobiaN.m    ← Partial Jacobian up to joint k
│   │   ├── get_partial_rotation.m      ← R₀ⁿ for external wrench transform
│   │   └── get_partial_rotationm.m     ← Mobile-base version
│   │
│   ├── dynamics/
│   │   ├── rnedynamicFixedBase1610.m   ★ Core RNE solver (fixed base)
│   │   ├── rnemobilebasec.m            ★ Core RNE solver (mobile base, dual-arm)
│   │   ├── compute_object_dynamics.m   ← EE wrench from cooperative object load
│   │   └── compute_object_dynamicsm.m  ← Mobile-base EE wrench (simplified)
│   │
│   ├── trajectory/
│   │   └── trajectoryDualArmNew1610.m  ← DLS-IK trajectory generator
│   │
│   ├── animation/
│   │   └── animate_dualarm_3d.m        ← Real-time 3D animated visualisation
│   │
│   └── utils/
│       ├── exportAndPlotTau.m          ← Excel export + joint torque plots
│       ├── compute_base_accel.m        ← Platform acceleration solver
│       ├── compute_betarc_vel.m        ← Khalil-Dombre velocity bias
│       ├── compute_betarc_ext.m        ← External force bias on base
│       └── sum_base_coupling.m         ← Aggregate dual-arm base coupling
│
├── tasks/
│   ├── lifting/
│   │   ├── case5_lift_upward.m         ← Trajectory: 0.205 m upward lift in 5 s
│   │   ├── main_rne_fixedbase.m        ← Fixed-base torque computation (lifting)
│   │   └── main_rne_mobilebase.m       ← Mobile-base torque computation (lifting)
│   │
│   └── bottle_opening/
│       └── main_rne_bottle_opening.m   ← Torque computation: rotational cap opening
│
└── docs/
    └── images/                         ← Figures, plots, architecture diagrams
```

---

## 🚀 Quick Start

### Prerequisites

- MATLAB R2022b or later  
- No additional toolboxes required *(Robotics System Toolbox NOT needed)*

### Installation

```bash
git clone https://github.com/YOUR_USERNAME/humanoid-14dof-dynamics.git
cd humanoid-14dof-dynamics
```

In MATLAB:
```matlab
% Add all subdirectories to path
addpath(genpath('src'))
addpath(genpath('tasks'))
```

### Run the Lifting Task (End-to-End)

```matlab
% Step 1 — Generate joint trajectories
run('tasks/lifting/case5_lift_upward.m')

% Step 2 — Compute torques (fixed base)
run('tasks/lifting/main_rne_fixedbase.m')

% Step 3 — Compute torques (mobile base)
run('tasks/lifting/main_rne_mobilebase.m')

% Step 4 — Animate the result
run('src/animation/animate_dualarm_3d.m')
```

### Use the RNE Solver Directly

```matlab
% Define a single configuration
q  = [0; 0.3; -0.5; 1.2; 0; 0.4; 0];
qd = zeros(7,1);
qdd= zeros(7,1);

% Object parameters
obj.mass  = 1.0;
obj.I     = diag([0.001, 0.05, 0.05]);
obj.g_obj = [-9.81; 0; 0];

% Compute torques for a single timestep
[tau, M, C, G] = rnedynamicFixedBase1610(q', qd', qdd', 'A', obj);
disp('Joint Torques [N·m]:'); disp(tau)
disp('Inertia Matrix M:');    disp(M(:,:,1))
```

### Compute the Geometric Jacobian

```matlab
q = [0; 0.3; -0.5; 1.2; 0; 0.4; 0];

J_A = jacobianOfArmA(q);   % 6×7
J_B = jacobianOfArmB(q);   % 6×7

% Check for singularity
fprintf('Arm A manipulability: %.6f\n', sqrt(det(J_A(1:3,:)*J_A(1:3,:)')));
```

---

## 📖 Module Documentation

### `get_robot_params(armType, th_i)` — Robot Model

The foundation of the entire stack. Returns a complete robot struct:

```matlab
robot = get_robot_params('A', q);

robot.DH    % 8×4 DH table [alpha, a, theta_offset, d]
robot.m     % 1×7 link masses [kg]
robot.Pc    % 3×7 COM positions in link frames [m]
robot.I     % {1×7} inertia tensors [kg·m²]
robot.T     % {1×9} absolute transforms (T{1}=eye, T{9}=EE)
robot.A     % {1×8} relative DH transforms
robot.Ri    % {1×8} rotation components of A{k}
```

**Arm differentiation**: only `d1` changes sign (`+0.318` for A, `-0.318` for B).  
All other parameters are identical — the robot is perfectly symmetric.

---

### `rnedynamicFixedBase1610(th, thdot, thddot, armType, objParams)` — Core RNE

Processes an N-sample trajectory and returns:

```matlab
[tau, M, C, G] = rnedynamicFixedBase1610(th, thdot, thddot, 'A', objParams);

% tau  : N×7  joint torques        [N·m]
% M    : 7×7×N inertia matrices
% C    : N×7  Coriolis torques     [N·m]
% G    : N×7  gravity torques      [N·m]
% Verify: tau ≈ squeeze(sum(M.*permute(thddot,[3,2,1]),2))' + C + G
```

Internally calls `compute_tau_corrected` (forward pass → backward pass) for every sample.

---

### `rnemobilebasec(...)` — Mobile Base Dual-Arm RNE

```matlab
[tauA, tauB, M_A, M_B, C_A, C_B, G_A, G_B, a_base_all] = ...
    rnemobilebasec(th_A, th_B, thdot_A, thdot_B, thddot_A, thddot_B, baseState, objParams);

% a_base_all : 3×N  [ax; ay; α] platform acceleration history
```

**Sequence per timestep:**
1. Build arm structs with world-frame transforms (`get_robot_paramsmobile`)  
2. Solve platform acceleration via Khalil-Dombre (`compute_base_accel`)  
3. Propagate base acceleration into arm forward kinematics  
4. Run backward recursion for arm torques  
5. Decompose into M, C, G (symmetrize M = ½(M+Mᵀ))

---

### `trajectoryDualArmNew1610(...)` — DLS-IK Trajectory Generator

```matlab
[qA, qAd, qAdd, qB, qBd, qBdd, Pc, Vc, xd, t_elapsed] = ...
    trajectoryDualArmNew1610(Vc_des, Vc_dot_d, objParams, qA0, qB0, 0, 5, 0.01, 'out.xlsx');
```

At each step:  
1. Compute 6×7 Jacobians for current config  
2. Compute DLS pseudo-inverse: `J† = (JᵀJ + λ²I)⁻¹Jᵀ`  
3. Resolve: `q̇ = J† · Jo · Vc_d`  
4. Euler integrate: `q_new = q + q̇·dt`  
5. Export all histories to Excel (7 sheets)

---

## 🎯 Simulation Tasks

### Task 1: Coordinated Upward Lifting

Both arms cooperate to lift a 1.056 kg object 0.205 m upward:

```
Object: 1.056 kg | I = diag([0.00107, 0.05632, 0.05738]) kg·m²
Motion: vx = 0.041 m/s (constant) | Duration: 5 s | dt = 10 ms
Steps:  500 timesteps | Total displacement: 0.205 m
```

**Expected joint torque range:**
- Joints 1–2 (shoulder): 15–35 N·m
- Joint 4 (elbow):        8–20 N·m
- Joints 6–7 (wrist):     2–8 N·m

### Task 2: Bottle Cap Opening

Arm A holds a water bottle while Arm B applies a rotational opening motion:

```
Bottle: r=0.04 m, h=0.15 m, ρ=1000 kg/m³
Mass:   0.754 kg (fully water-filled)
Ixx=Iyy: 0.00378 kg·m²  |  Izz: 0.000603 kg·m²
```

---

## 📊 Results & Validation

## 📊 Results & Visualisation
 
> All results generated in **MATLAB R2024b Academic** on Intel Core i-series.
> Benchmark: **1000 RNE calls in 0.85 s → 0.847 ms/call → ~1181 Hz single-arm.**
 
---
 
### Task 1 — Coordinated Upward Lifting (Fixed Base)
 
Both arms cooperatively lift a **1.056 kg object 0.205 m upward** in 5 seconds
at constant Cartesian velocity (vx = 0.041 m/s).
 
#### Arm A — Joint Torques vs. Time
 
![Arm A Joint Torques — Lifting Task](docs/images/lifting_tauA_fixed_base.png)
 
#### Arm B — Joint Torques vs. Time
 
![Arm B Joint Torques — Lifting Task](docs/images/lifting_tauB_fixed_base.png)
 
**Key observations:**
- **Joints 1 & 4** (shoulder + elbow) carry the dominant load: **30–45 N·m** at start,
  settling to **20–30 N·m** as the arm reconfigures during the lift
- **Joint 7** (wrist/hand — heaviest link at 3.21 kg) shows significant torque
  due to its large inertia tensor (I_zz = 2.608 kg·m²)
- **Joints 3 & 5** (forearm rotation) remain near zero — expected for a pure
  translational lift with no forearm rotation commanded
- Both arms show **symmetric torque profiles**, confirming the model correctly
  mirrors the kinematic structure (d₁ sign flip only)
- Torques **decay smoothly** over time as the configuration changes —
  consistent with a velocity-controlled trajectory (not position-stepped)
 
---
 
### Task 2 — Coordinated Bottle Opening (Fixed Base)
 
Arm A holds the bottle body; Arm B applies a rotational unscrewing motion to the cap.
Four-phase cooperative task: grasp → stabilise → unscrew → withdraw.
 
#### Arm A — Joint Torques vs. Time (Bottle Opening)
 
![Arm A Torques — Bottle Opening](docs/images/bottle_opening_tauA.png)
 
**Key observations:**
- **Transient spike at t=0–5s**: Initial configuration settling creates large
  torque transients (up to ~65 N·m at J1) before stabilising
- **Steady-state at ~10–15s**: All joints settle to constant holding torques,
  confirming stable grasp maintenance
- **Negative torques** on joints 1 and 3 indicate the arm is actively resisting
  the reaction forces from Arm B's unscrewing motion — correct cooperative behaviour
- The torque pattern is distinctly different from the lifting task,
  validating that object dynamics are correctly parameterised per task
 
#### 4-Phase Bottle Opening Visualisation
 
![Bottle Opening Task — 4 Phases](docs/images/bottle_opening_phases.png)
 
| Phase | Description | Both Arms |
|-------|-------------|-----------|
| **Phase 1** | Arm A approaches and grasps bottle body | Arm B moves to standby |
| **Phase 2** | Arm B grasps bottle cap | Arm A holds bottle stable |
| **Phase 3** | Coordinated unscrewing — Arm B rotates, Arm A resists reaction | Active cooperation |
| **Phase 4** | Cap lifted and Arm B withdraws | Arm A maintains grasp |
 
---
 
### Task 1 — Mobile Base 3D Simulation
 
Real-time 3D animation of both arms on a moving platform.
Base kinematics integrated via semi-implicit Euler from Khalil-Dombre acceleration.
 
![Mobile Base 3D Simulation](docs/images/mobile_base_3d_simulation.png)
 
**Simulation parameters:**
- Base initial velocity: v₀ = [0; 4; 0] m/s, ω₀ = [0; 3; 0] rad/s
- Arm A (red) + Arm B (blue) rendered with FK at each timestep
- Green marker: manipulated object trajectory
- End-effector positions annotated with world-frame coordinates
 
---
 
### Validation Results
 
| Test | Method | Result | Status |
|------|--------|--------|--------|
| **Energy conservation** | KE + PE monitored over 5s trajectory | Drift < 0.1% | ✅ Pass |
| **Inertia symmetry** | ‖M − Mᵀ‖_F / ‖M‖_F | < 1×10⁻¹² | ✅ Pass |
| **Inverse dynamics consistency** | ‖τ − (Mq̈+Cq̇+G)‖ | < 0.001 N·m | ✅ Pass |
| **Gravity check** | G(q) = τ(q,0,0) analytical | Matches to 6 sig. fig. | ✅ Pass |
| **RNE benchmark** | 1000 calls, random configs | 0.847 ms/call (1181 Hz) | ✅ |
 
### Performance Benchmark
 
| Configuration | Calls | Wall-clock | Rate | Per-call |
|---------------|-------|------------|------|----------|
| Fixed base, 1 arm | 1000 | **0.85 s** | **1181 steps/s** | **0.847 ms** |
| Fixed base, 2 arms | 1000 | ~1.70 s | ~588 steps/s | ~1.694 ms |
| Mobile base, 2 arms | 500 | ~3.20 s | ~156 steps/s | ~6.4 ms |
 
> ⚡ Benchmarked on MATLAB R2024b Academic.
> Single-arm RNE at ~1181 Hz is sufficient for **500 Hz dual-arm real-time**
> trajectory planning. For hard 1 kHz embedded control, a C++ port
> (e.g. [Pinocchio](https://github.com/stack-of-tasks/pinocchio)) gives 50–100× speedup.
 

## 🛠 Dependencies

| Requirement | Version | Notes |
|-------------|---------|-------|
| MATLAB | R2022b+ | Core language |
| No toolboxes | — | Pure MATLAB, no Robotics Toolbox needed |

All functions in this repository are **self-contained**. The Robotics System Toolbox is deliberately **not** used — all kinematics and dynamics are implemented from first principles for full transparency and educational value.

---

## 📚 References

1. **Luh, J.Y.S., Walker, M.W. & Paul, R.P.C.** (1980). On-line Computational Scheme for Mechanical Manipulators. *Trans. ASME J. Dynamic Systems, Measurement & Control*, 102(2), 69–76.

2. **Craig, J.J.** (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson Prentice Hall.

3. **Khalil, W. & Dombre, E.** (2002). *Modeling, Identification and Control of Robots*. Hermes Penton Science.

4. **Nakamura, Y. & Hanafusa, H.** (1986). Inverse Kinematic Solutions with Singularity Robustness for Robot Manipulator Control. *J. Dynamic Systems, Measurement & Control*, 108(3), 163–171. *(DLS IK)*

5. **Featherstone, R.** (2008). *Rigid Body Dynamics Algorithms*. Springer. *(O(n) recursive dynamics)*

---

## 🎓 Academic Context

This project was developed as part of M.Tech research in **Robotics & Artificial Intelligence** at **COEP Technological University, Pune** (2023–2025), in collaboration with **DRDO** (2024–2025).

The implementation targets **Industry 4.0** applications including:
- Humanoid robots for defence logistics
- Collaborative industrial manipulation
- Human-robot cooperative assembly

---

## 📄 License

This project is licensed under the **MIT License** — see [LICENSE](LICENSE) for details.

---

## 🤝 Contributing

Contributions are welcome! Particularly interested in:
- ROS2 integration (publishing joint states, subscribing to trajectories)
- Gazebo simulation bridge
- C++ port of the RNE core for real-time deployment
- Contact dynamics extension (friction, impact)

Please open an issue before submitting a PR.

---

## 👤 Author

**Ganesh Deshmukh**  
M.Tech — Robotics & Artificial Intelligence, COEP Technological University  
Research Intern — DRDO (2024–2025)

📧 deshmukhg21@gmail.com  
🔗 [LinkedIn](https://linkedin.com/in/ganeshdeshmukh)  
📍 Pune, Maharashtra, India

---

<div align="center">

**If this repository helped you, please consider giving it a ⭐**

*Built with MATLAB | Dynamics from first principles | No black boxes*

</div>
