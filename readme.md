# Lunar Lander Simulator (MPC-Driven)

An interactive, dual-mode Lunar Lander simulation environment developed in MATLAB. The system simulates continuous 2D rigid-body flight dynamics under lunar gravity, utilizing discrete Model Predictive Control (MPC) with constraint handling for automated soft touchdown, alongside manual keyboard piloting modes.

## Technical Architecture & Modules

- **Physics Engine (`physics/`):** Rigid-body 2D translational and rotational equations of motion integrated via Euler and RK2 schemes, with contact detection against surface mesh geometry.
- **MPC Guidance (`mpc/`):** Discrete-time state-space linearization, cost matrix optimization, and constraint enforcement for terminal descent velocity and attitude.
- **Numerical Methods (`numerical/`):** Root-finding (Bisection, False Position, Secant, Newton-Raphson), numerical differentiation, composite numerical integration (Trapezoidal, Simpson's 1/3, 3/8), and Lagrange surface interpolation.
- **Terrain Generation (`terrain/`):** Procedural lunar surface modeling and obstacle/landing zone generation.
- **Flight Analysis (`analysis/`):** Real-time touchdown telemetry, descent profile validation, and fuel efficiency metrics.
- **Test Suites (`tests/`):** Automated verification scripts for physics, controller response, landing geometry, and numerical convergence.

## Detailed Engineering Documentation

For deep technical derivations and design notes, see:
- [Physics & Dynamic Model](documentation/PHYSICS_ENGINE.md)
- [MPC Formulation & Optimization](documentation/MPC_ENGINE.md)
- [Numerical Methods & Algorithms](documentation/NUMERICAL_METHODS.md)
- Complete visual overview: `LunarLanderMPC_Study_Manual.html`

## Getting Started

### Prerequisites
- MATLAB (R2020b or later)
- Control System Toolbox
- Optimization Toolbox

### Running the Simulator
1. Open MATLAB and set your current working directory to this folder.
2. In the MATLAB Command Window, launch the application:
   ```matlab
   startLander
Or launch directly via the class definition:
app = LunarLanderMPC;

Running Validation Tests
To run the automated test suite:
run('tests/testPhysicsEngine.m')
run('tests/testMPCController.m')
run('tests/testNumericalMethods.m')
run('tests/testFlightAnalysis.m')