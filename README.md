# 🚀 Nonlinear System Control & Trajectory Tracking

A high-fidelity numerical simulation developed as a **Freelance Project (2021)**. This repository provides the complete MATLAB implementation for a tracking algorithm applied to a **Nonlinear Differential System**.

---

## 🏗️ Technical Overview

The primary objective of this project was to engineer a control law that ensures the system states ($x_1, x_2$) accurately follow a desired reference trajectory ($\hat{x}_1, \hat{x}_2$). This involves solving complex nonlinear dynamics in real-time.

### Key Technical Components:
* **Trajectory Tracking**: Implementation of target paths ($\hat{x}_1, \hat{x}_2$) that serve as the system's "ideal" reference.
* **Control Input Derivation**: Calculation of optimized control variables $u_1$ and $u_2$ to drive the system and minimize error.
* **Advanced Numerical Integration**: Leveraged the **ODE45 (Runge-Kutta)** solver to handle the nonlinear state-space equations with high precision.

---

## 🛠️ Implementation Details

### 1. System Parameters (Freelance 2021 Specs)
The simulation was configured with precise engineering constants defined during the freelance engagement:
* **Time Span**: $T = 30$ seconds with $m = 80$ discrete steps.
* **Control Constants**: Initialized parameters ($p_1, p_2, q_1, q_2, c_1, c_2$) to model the specific nonlinear environment.

### 2. Nonlinear Dynamics & Solver
The system's behavior is governed by nonlinear feedback loops. I utilized the **ODE45** solver, which is a versatile numerical integrator, to compute the state evolution of the system over time.

### 3. Symbolic Math Integration
Used MATLAB's **Symbolic Math Toolbox** to derive exact derivatives of the desired trajectories, ensuring the control inputs respond accurately to changes in the target path.

---

## 📈 Visualizing Results

The project generates comprehensive plots to validate tracking performance:
* **State Accuracy**: Actual state ($x_1, x_2$) vs. Target trajectory ($\hat{x}_1, \hat{x}_2$) comparison.
* **Control Effort**: Visualizing the $u_1$ and $u_2$ inputs required to maintain system stability.

---

## 📂 Project Structure
* `main.m`: The core MATLAB script featuring the solver and logic.
* `Mathematical_Derivations_and_System_Dynamics_for_Nonlinear_Control.pdf`: Original mathematical derivations documentation.

---

### **Contact & Professional Links**
**Marwa Mahmoud Mohamed**
* **Email**: marwa.sw.eng@outlook.com
* **LinkedIn**: marwa-mahmoud123
* **Portfolio**: marwa-mahmoud-sw-eng.vercel.app
