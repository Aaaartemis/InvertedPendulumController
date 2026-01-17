# Inverted Pendulum Control: PID vs MPC

This repository contains the implementation, simulation models, and supporting materials for a Control Systems project focused on stabilising an inverted pendulum on a rotating base.

The project compares a classical PID controller with a Model Predictive Controller (MPC), analysing performance, stability, constraint handling, and tuning trade-offs.

The full theoretical derivation, experimental design, and results are documented in the accompanying project report.

---

## Project Overview

## Controllers Implemented

### PID Controller
- Manually tuned using trial-and-error
- Gains:
  - Kp = 40
  - Ki = 1e-5
  - Kd = 2
- Simple to implement but does not handle constraints
- Can become unstable for certain gain values

### Model Predictive Controller (MPC)
- Discrete-time state-space formulation
- Quadratic cost function with:
  - State weighting matrix Q
  - Input weighting R
  - Terminal cost P (computed from the DARE)
- Supports:
  - Input magnitude constraints
  - Input rate constraints
  - State and velocity constraints
- Implemented using YALMIP with a quadratic programming solver

An augmented state-space model is used to optimise over input increments (Δu), improving smoothness and feasibility.

---

## Experiments Conducted

The following investigations are included:
- PID vs MPC performance comparison
- Effect of varying Q/R weightings
- Effect of prediction horizon length (N)
- Impact of terminal cost inclusion
- Impact of state and input constraints

All experiments, plots, and quantitative results are described and analysed in detail in the project report.

---

## Key Findings

- PID provides faster settling and lower overshoot when constraints are ignored
- MPC ensures:
  - Explicit constraint handling
  - Improved closed-loop stability
  - Lower RMS control effort
- Terminal cost is critical for feasibility at short prediction horizons
- Very large prediction horizons significantly increase computation time with minimal performance gains

---

## Requirements

To run the simulations and controllers, the following are required:
- MATLAB
- Simulink
- YALMIP toolbox
- A supported quadratic programming solver (e.g. quadprog or OSQP)
- QLabs (for the full simulation environment)


---

## Repository Structure

The goal of this project is to stabilise an inverted pendulum in the upright position while respecting physical constraints such as:
- Input voltage limits
- Base and pendulum angular limits
- Angular velocity limits

Two control approaches are implemented and compared:
- PID control (baseline classical approach)
- MPC control (constraint-aware optimisation-based approach)

Simulations are performed in Simulink / QLabs, and controller performance is evaluated using metrics such as rise time, settling time, overshoot, RMS input, and steady-state error.

---
## Repository Structure
 ```
├── code/ 
| ├── quarc_mpc.m # mpc code 
| └── xxx.m # pid code
│
├── model/
│ └── pendulum.slx # System model (state-space / Simulink)
│
├── images/
│ ├── pid_analytics.png
│ ├── mpc_analytics.png
│ └── QLabs.png
│
└── README.md
 ```

