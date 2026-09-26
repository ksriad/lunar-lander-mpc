# MPC-Driven Interactive Dual-Mode Lunar Lander Simulator
## Autonomous MPC Engine — Person 2's Module

This document is the handoff for Person 4 (GUI/App Designer integration)
and a design record for the team. It covers the file list, public
interface, parameter/target contracts, the model derivation, the
Experiment 5 solver, the constraint strategy, and honest notes on what
has and has not been executed.

---
## A. File list

```
mpc/
  mpcController.m            - public entry point
  buildPredictionMatrices.m  - [F, Phi] = buildPredictionMatrices(Ad, Bd, Np, Nc)
  buildCostMatrices.m        - [H, rhs, diagInfo] = buildCostMatrices(...)
  linearizeLander.m          - [A, B, u0] = linearizeLander(m, I, g)
  discretizeModel.m          - [Ad, Bd] = discretizeModel(A, B, dt)
  gaussJordanSolve.m         - [x, info] = gaussJordanSolve(H, rhs)
  applyMPCConstraints.m      - actuator saturation + soft attitude safety
tests/
  testMPCController.m        - self-contained test script (16 tests)
documentation/
  MPC_ENGINE.md              - this file
```

No GUI code, no keyboard handling, no physics integration, no duplicate
fuel model, and no global variables anywhere in this module.

---
## B. Exact public function signatures

```matlab
[control, prediction, info] = mpcController(state, target, params)

[A, B, u0]       = linearizeLander(m, I, g)
[Ad, Bd]         = discretizeModel(A, B, dt)
[F, Phi]         = buildPredictionMatrices(Ad, Bd, Np, Nc)
[H, rhs, diagInfo] = buildCostMatrices(F, Phi, x0, xTargetVec, Q, R, S, Np, Nc)
[x, info]        = gaussJordanSolve(H, rhs)
[uApplied, saturated, thetaPredicted, safetyTriggered] = ...
    applyMPCConstraints(uNominal, X, params, n)
```

Only `mpcController` is intended for Person 4 to call directly. The
others are exposed (not nested) so they can be unit tested independently
(see Section 15 requirement) and reused if a later iteration changes the
optimizer.

---
## C. Required `params` fields

| Field | Required? | Meaning | Notes |
|---|---|---|---|
| `params.m` | yes | current lander mass (kg) | changes as fuel burns; Person 1 owns the fuel model, this controller only reads the value each call |
| `params.I` | yes | moment of inertia (kg·m²) | |
| `params.g` | yes | gravity (m/s²) | project value 1.62 |
| `params.dt` | yes | timestep (s) | project value 0.04 |
| `params.Tmax` | yes | max thrust (N) | hard actuator bound |
| `params.tauMax` | yes | max \|tauRCS\| (N·m) | hard actuator bound |
| `params.thetaMax` | yes | max \|theta\| (rad) | **soft** bound, see Section H |
| `params.Np` | no (default 20) | prediction horizon | |
| `params.Nc` | no (default 5) | control horizon | must satisfy `1<=Nc<=Np` |
| `params.Q` | no (default `diag([5 2 20 8 10 2])`) | state tracking weight, 6×6 or length-6 vector | |
| `params.R` | no (default `diag([0.01 0.05])`) | control effort weight, 2×2 or length-2 vector | |
| `params.S` | no (default `diag([0.05 0.1])`) | control increment weight, 2×2 or length-2 vector | |

No physical constant (`m`, `g`, `I`, `Tmax`, `tauMax`, `thetaMax`, `dt`)
is hard-coded inside the module — all come from `params`. The Np/Nc/Q/R/S
defaults are *tuning* defaults, not physics, and can be overridden.

---
## D. Required `target` structure

```matlab
target.xTarget      % required - landing pad x position (m)
target.yTarget      % required - landing pad elevation (m)
target.vxTarget      % optional, default 0
target.vyTarget      % optional, default 0
target.thetaTarget   % optional, default 0
target.omegaTarget   % optional, default 0
```

The GUI can change `target.xTarget` / `target.yTarget` between calls at
any time (e.g. if the user repositions the landing pad) — the controller
rebuilds its target vector fresh every call, there is no caching of the
previous target.

---
## E. Example call

```matlab
state = [12.0; 1.5; 340.0; -18.0; 0.03; 0.0]; % [x;vx;y;vy;theta;omega]

target.xTarget = 0;
target.yTarget = 0;

params.m = 1450;      % current mass from Person 1's fuel model
params.I = 6200;
params.g = 1.62;
params.dt = 0.04;
params.Tmax = 22000;
params.tauMax = 3000;
params.thetaMax = 0.5;
% Np, Nc, Q, R, S left at defaults

[control, prediction, info] = mpcController(state, target, params);
```

---
## F. Example returned control

For the call above (values are representative of the algorithm's shape,
not a literal executed run — see Section J on test execution status):

```matlab
control =
   1.977e+04   % T  (N), below Tmax, biased above hover thrust to arrest descent/closes horizontal gap
  -1.42e+02    % tauRCS (N*m), small corrective torque
```

`prediction` is a `20 x 6` matrix (`Np x 6`), each row `[x vx y vy theta
omega]` for one future step. `info.uNominal` holds the pre-saturation
value: comparing `control` to `info.uNominal` combined with
`info.saturated` tells Person 4 whether the plot of `prediction` is still
representative of what will actually happen (it is only exact if
`info.saturated == [false false]` and `info.safetyTriggered == false`).

---
## G. Prediction matrix dimensions

With state dimension `n = 6`, control dimension `m = 2`:

- `F` is `(n*Np) x n` = `(6*Np) x 6` — maps current state to the stacked
  future-state prediction with zero control increment.
- `Phi` is `(n*Np) x (m*Nc)` = `(6*Np) x (2*Nc)` — maps the stacked
  future control-increment sequence to the stacked future-state
  prediction.
- The stacked prediction `X = F*x0 + Phi*deltaU` is `(6*Np) x 1`,
  reshaped by `mpcController` into `prediction` (`Np x 6`) via
  `reshape(X,6,Np).'`.
- `deltaU` is `(2*Nc) x 1`: `Nc` future control moves, each 2 elements
  `[deltaT; deltaTauRCS]`, with control **held constant** at the
  `Nc`-th move for prediction steps beyond the control horizon (standard
  control-horizon blocking).
- `H` (the normal-equation matrix solved by `gaussJordanSolve`) is
  `(2*Nc) x (2*Nc)` — e.g. `10x10` at the recommended `Nc=5`.

`buildPredictionMatrices.m` asserts these dimensions with `assert(...)`
calls before returning.

---
## H. Constraints — what is and is not guaranteed

**Actuator saturation is a true hard constraint** on the value
`mpcController` returns:

```
0 <= T <= Tmax
|tauRCS| <= tauMax
```

**Predicted-state attitude constraints (`|theta| <= thetaMax`) are NOT a
hard constraint.** The core QP solved by `gaussJordanSolve` has no
knowledge of `thetaMax` — it is an *unconstrained* least-squares solve
(Section 7 of the spec explicitly allows this as the mathematical
foundation, with saturation and safety checking layered on top).

What actually happens in `applyMPCConstraints.m` when the *unconstrained*
predicted trajectory has `max(|theta|) > thetaMax`:

1. The predicted theta trajectory is inspected (diagnostic only).
2. `tauRCS` for *this step only* is scaled down proportionally
   (`scale = thetaMax / thetaPredicted`, clipped to be non-negative).
3. The QP is **not** re-solved, and there is **no guarantee** the actual
   resulting trajectory stays within `thetaMax` — this is a damping
   correction on the immediate command, not a constrained optimization.

This is deliberately labeled a **"project-specific numerical
optimization method"**, not an "Experiment 8 algorithm" (per Section 12 —
the real Experiment 8 is bisection/false-position/Newton-Raphson/secant
root-finding, which is unrelated to this correction step and is not used
here). If a future iteration needs hard state constraints, that requires
actual constrained optimization (active-set / projected methods), which
Section 13 explicitly disallows as the real-time core solver for this
project.

---
## I. Solver residual and numerical-stability strategy (Experiment 5)

`gaussJordanSolve.m` implements Gauss-Jordan elimination with **partial
pivoting**, operating on the augmented matrix `[H | rhs]`:

- At each column, the largest-magnitude candidate pivot at or below the
  diagonal is selected and swapped into place (partial pivoting).
- If even the best candidate pivot is below a tolerance scaled to
  `norm(H,'fro')`, a tiny ridge term is added to that diagonal entry
  (`info.regularized = true`) rather than dividing by (near) zero.
- If the pivot is still effectively zero after that, the matrix is
  flagged `info.singular = true` and that column is left degraded rather
  than crashing.
- `info.illConditioned` flags when the smallest pivot actually used, even
  if nonzero, was below a looser conditioning-warning threshold —
  useful for Person 4 to log/display even when the solve nominally
  "succeeded".
- Validation: `info.residual = norm(H*x - rhs)` is always computed and
  returned, so the caller can see solve quality even on a degraded
  solve.
- **No use of MATLAB's `\` or `inv()`** anywhere in the solve path (only
  used once, in the test file, purely as an independent reference value
  to check `gaussJordanSolve` against — never inside the shipped
  controller code).

`mpcController.m` surfaces `info.solverResidual`, `info.success`,
`info.singular`, and `info.illConditioned` directly so the GUI can, for
example, show a warning indicator if the solve degrades.

---
## J. Test results — IMPORTANT, please read

`tests/testMPCController.m` implements all 16 tests required by Section
15 (Ad/Bd dimensions, state/control ordering, hover equilibrium, thrust/
torque sign checks, Gauss-Jordan correctness on a known matrix,
near-singular detection, finite output, actuator bound enforcement,
prediction length, target/state sensitivity, solver residual, and
robustness away from the linearization point).

**I do not have a MATLAB runtime available in this environment**, so I
have not actually executed `testMPCController.m` — I verified the
algebra (Jacobian entries, prediction-matrix recursion, Gauss-Jordan
normal-equation derivation) by hand instead of by running it. Please run

```matlab
cd tests
testMPCController
```

in MATLAB R2024a+ yourself before integrating, and treat the printed
`[PASS]/[FAIL]` summary as the real result. If anything fails, the most
likely culprits (in order of likelihood) are: a sign convention mismatch
with Person 1's actual physics engine implementation, or a weight tuning
issue in the default `Q`/`R`/`S` producing a saturation-heavy solution
that still "succeeds" numerically but looks aggressive.

---
## K. Runtime/performance observations (estimated, not measured)

With the recommended `Np=20`, `Nc=5`, `n=6`, `m=2`:
- `H` is `10x10` — Gauss-Jordan on a 10×10 augmented matrix is a handful
  of microseconds' worth of work on any modern machine.
- `Phi` is `120x10`, built via `Np*Nc` = 100 small `6x6 * 6x2` matrix
  multiplications, using precomputed powers of `Ad` (each `Ad^i`
  computed once and reused across all blocks, not recomputed per cell).
- No `expm`, `quadprog`, `fmincon`, `fminsearch`, or MPC Toolbox calls
  anywhere.
- The controller is written as a **pure function** (no `persistent` or
  `global` state) so it is trivially safe to call every 0.04 s from the
  GUI's timer callback without worrying about stale cached state between
  calls — the tradeoff is that `Ad`, `Bd`, `F`, `Phi` are rebuilt from
  scratch every call rather than cached, which is intentionally
  accepted here given how small these matrices are at the recommended
  horizon. If profiling in the actual integrated app shows this is a
  bottleneck, the mass-dependent pieces (`linearizeLander`,
  `discretizeModel`) are the only ones that need to change when mass
  changes, and could be cached against a mass tolerance if needed — this
  was not needed to hit the 0.04 s budget at the estimated matrix sizes.

I have not benchmarked this on real hardware; please have Person 4 report
back actual wall-clock timings once integrated, and flag it if `Np`/`Nc`
need to grow beyond what fits the budget.

---
## L. Integration instructions for Person 4

1. Add `mpc/` to the MATLAB path (or copy this whole folder into the
   shared project and add `mpc/` to path at app startup).
2. Every control tick (every `dt` = 0.04 s), call:
   ```matlab
   [control, prediction, info] = mpcController(state, target, params);
   ```
   where `state` comes from Person 1's physics engine and `params.m`
   is Person 1's current mass value (fuel has burned since last tick).
3. Apply **only** `control` (`[T; tauRCS]`) to the physics step — do not
   apply any part of `prediction` as control; it is for the trajectory
   overlay/visualization only.
4. To change the landing pad, just change `target.xTarget` /
   `target.yTarget` before the next call — no reset or reinitialization
   needed, the controller is stateless between calls.
5. Recommended GUI diagnostics to surface (all in `info`):
   `info.solverResidual`, `info.success`, `info.saturated`,
   `info.safetyTriggered`, `info.thetaPredicted`. These make solver
   health and constraint activity visible without needing to understand
   the internals.
6. Do not call `linearizeLander`, `discretizeModel`,
   `buildPredictionMatrices`, `buildCostMatrices`, `gaussJordanSolve`, or
   `applyMPCConstraints` directly from the GUI — only `mpcController` is
   the stable public entry point; the others may change signature in a
   future revision (this would be documented here if it happens).

---
### Appendix: why the model has no affine/offset term

The continuous model used for prediction is `x_dot = A*x + B*delta_u`
(no constant term), operating directly on the **absolute** state `x`
with the control **increment** `delta_u = u - u0` as input. This works
without an affine correction because:

- `dx/dt = vx` and `dy/dt = vy` are exact (not approximations) — no
  linearization error there at all.
- The only genuinely linearized terms are `dvx/dt` and `dvy/dt`'s
  dependence on `theta` (small-angle, `sin(theta)≈theta`,
  `cos(theta)≈1`) evaluated at `T = T0 = m*g`.
- Position does not appear in `A` or `B` at all (translation invariance),
  so there is no need to track a separate equilibrium position — any
  position with `vx=vy=theta=omega=0` and `u=u0` is a valid fixed point
  of the *linear* model, and the linear model's zero input (`delta_u=0`)
  correctly reproduces zero net acceleration at that fixed point.

This was checked by hand against the nonlinear equations at several
points (`theta=0` with `delta_u=0`; nonzero `vx`/`vy` with `delta_u=0`;
small nonzero `theta` with `delta_u=0`) and is consistent to first order
in `theta`, which is the expected accuracy of any linearized MPC model
away from the exact operating point (see `test_offnominal_state_runs` in
the test file, which only checks the controller stays numerically
well-behaved far from the linearization point — it does not and cannot
check trajectory optimality there).
