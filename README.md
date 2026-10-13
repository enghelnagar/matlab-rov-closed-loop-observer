# ROV Closed-Loop Control and Observer Design

## Project Overview
This repository implements a complete closed-loop control architecture for a Remotely Operated Vehicle (ROV). The project integrates a full-state feedback controller with a continuous-time Luenberger observer to ensure asymptotic stability and accurate state tracking, even when starting with significant initial estimation errors.

## Mathematical Architecture

### System Dynamics
The ROV is modeled as a 6-dimensional linear time-invariant (LTI) system:
$$\dot{X} = A X + B U$$
$$Y = C X$$
Where the measurement matrix $C$ only captures the spatial positions and yaw angle $(x, y, \psi)$, requiring an observer to estimate the unmeasured linear and angular velocities.

### Closed-Loop Control (Pole Placement)
The state feedback gain matrix $K_c \in \mathbb{R}^{3 \times 6}$ is designed to force the closed-loop system matrix $(A - B K_c)$ to be asymptotically stable. The gains are computed by matching the closed-loop characteristic equation to a desired second-order response:
$$s^2 + 2\xi\omega_n s + \omega_n^2 = 0$$
For the controller, the natural frequency is set to $\omega_n = 1$ rad/s with critical damping $\xi = 1$.

### Luenberger Observer
To reconstruct the full state vector from partial measurements, a Luenberger observer is implemented:
$$\dot{\hat{X}} = A \hat{X} + B U + K_o (Y - C \hat{X})$$
The observer gain matrix $K_o \in \mathbb{R}^{6 \times 3}$ is designed using the same pole placement methodology. To ensure the observer dynamics converge faster than the controller dynamics, the observer's natural frequency is strictly set to $\omega_n = 10$ rad/s.

## Simulation Results

The simulation evaluates the system under a "Closed Loop with Observer" configuration. The observer is initialized with a deliberate state error $e = [1, -1, 1.5, 0.5, -0.5, -1.5]^T$. 

**Performance Metrics:**
- The observer successfully estimates the true state rapidly without inducing instability.
- The closed-loop controller effectively drives the ROV to the target reference state despite the initial estimation offsets.

*- Trajectory closed loop with observer n*
<img width="700" height="525" alt="Trajectory closed loop with observer n" src="https://github.com/user-attachments/assets/ddc29d70-1e28-467d-8790-32592b6767f2" />

*- State closed loop with observer n*
<img width="1902" height="968" alt="State closed loop with observer n" src="https://github.com/user-attachments/assets/b127f6c2-f8f1-4b1c-8b0d-2ea815f13893" />


*- State esimaion error closed loop with observer n*
<img width="1902" height="968" alt="State esimaion error closed loop with observer n" src="https://github.com/user-attachments/assets/5b36ad54-91ae-438d-b93e-4bcd4380bec0" />

*- Input closed loop with observer n*
<img width="700" height="525" alt="input closed loop with observer n" src="https://github.com/user-attachments/assets/90bffef6-0270-45d2-b143-8a78e86782ab" />



## How to Run

**Prerequisites:**
- MATLAB (Symbolic Math Toolbox required for Gramian integration)

**Execution:**
1. Clone the repository.
2. Open `lab4_ex1_template_Sol.m` in MATLAB.
3. Run the script to execute the simulation. The script will automatically generate plots for state evolution, estimation error, control inputs, and the 2D spatial trajectory.

---
*Developed by Hussein Elnaggar.*
