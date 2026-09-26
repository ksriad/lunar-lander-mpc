# Numerical Methods & Flight Analysis — Person 3 Module

**Project:** MPC-Driven Interactive Dual-Mode Lunar Lander Simulator
**Person 3 scope:** EEE 212 numerical-method implementations, telemetry processing, post-flight numerical analysis, performance calculations.
**Target:** MATLAB R2024a+, macOS.

This module does **not** implement the GUI, keyboard input, physics simulation, or MPC control — those belong to Persons 1, 2, and 4. Person 3 provides hand-coded numerical methods, telemetry validation/processing, and the analysis pipeline that turns raw flight telemetry into the metrics and plot-ready data the rest of the team needs.

---

## 1. File list

```
numerical/
  lagrangeInterpolation.m
  gaussJordanElimination.m      <- see "Note on Experiment 5" below
  linearRegression.m
  polynomialRegression.m
  finiteDifference.m
  richardsonDerivative.m
  compositeTrapezoidal.m
  compositeSimpson13.m
  compositeSimpson38.m
  adaptiveIntegration.m
  bisection.m
  falsePosition.m
  newtonRaphson.m
  secant.m

analysis/
  analyzeFlight.m
  calculateFlightMetrics.m
  prepareResponseData.m

tests/
  testNumericalMethods.m
  testFlightAnalysis.m

documentation/
  NUMERICAL_METHODS.md          <- this file
```

**Note on `gaussJordanElimination.m`:** the spec's file list (section 13) does not list a standalone Gauss-Jordan file, since Person 2 owns the shared solver used inside the MPC. This file was added because (a) `linearRegression.m` / `polynomialRegression.m` need a hand-coded solver for their normal equations, and (b) Experiment 5 needs an independently testable implementation for `testNumericalMethods.m`. **At integration time, reconcile this with Person 2's solver** so the shipped project does not run two different Gauss-Jordan implementations inside the MPC loop — this file should only be used for regression's normal equations and for the Experiment 5 demonstration/test, not duplicated into the controller.

---

## 2. Public function signatures

```matlab
% Experiment 3 — Interpolation
yQuery = lagrangeInterpolation(xData, yData, xQuery)

% Experiment 4 — Curve fitting
coeff = linearRegression(x, y)                    % coeff = [a0; a1]
coeff = polynomialRegression(x, y, degree)         % coeff = [a0; a1; ...; a_degree]

% Experiment 5 — Linear systems
[x, info] = gaussJordanElimination(A, b)
% info.residual, info.singular, info.pivotSequence

% Experiment 6 — Numerical differentiation
d = finiteDifference(f, x, h, method)              % function-handle mode
d = finiteDifference(yData, xData)                 % discrete telemetry mode
[d, err] = richardsonDerivative(f, x, h, method)

% Experiment 7 — Numerical integration
I = compositeTrapezoidal(x, y)
I = compositeSimpson13(x, y)                       % needs even # subintervals, uniform spacing
I = compositeSimpson38(x, y)                        % needs subintervals multiple of 3, uniform spacing
[I, info] = adaptiveIntegration(f, a, b, tol, maxDepth)

% Experiment 8 — Root finding
[root, info] = bisection(f, a, b, tol, maxIter)
[root, info] = falsePosition(f, a, b, tol, maxIter)
[root, info] = newtonRaphson(f, df, x0, tol, maxIter)
[root, info] = secant(f, x0, x1, tol, maxIter)

% Analysis
results  = analyzeFlight(telemetry, params)
results  = calculateFlightMetrics(telemetry, params)
plotData = prepareResponseData(telemetry, results, params)
```

All root-finding / integration `info` structs include `.iterations` (or `.evaluations`), `.error`, `.converged`, `.history` where applicable — used for the validation-test error/iteration reporting required by the spec (section 11).

---

## 3. Dependency list

- `linearRegression.m`, `polynomialRegression.m` → call `gaussJordanElimination.m`
- `richardsonDerivative.m` → calls `finiteDifference.m` (function-handle mode)
- `calculateFlightMetrics.m` → calls `finiteDifference.m` (discrete mode) and `compositeTrapezoidal.m`
- `analyzeFlight.m` → calls `calculateFlightMetrics.m`
- `prepareResponseData.m` → depends on the `results` struct from `calculateFlightMetrics.m` (specifically `results.numericalAx`)
- `tests/testNumericalMethods.m` → exercises every function in `numerical/`
- `tests/testFlightAnalysis.m` → exercises `analyzeFlight.m` → `calculateFlightMetrics.m` → `prepareResponseData.m`, using a self-contained synthetic telemetry generator (no external file needed)

No file in `numerical/` or `analysis/` uses `interp1`, `interp2`, `polyfit`, `A\b`, `inv`, `gradient`, `trapz`, `integral`, `quadgk`, or `fzero` as its core implementation.

---

## 4. Telemetry structure

Standardized `telemetry` struct (all fields are numeric vectors of equal length, one entry per sample):

```
telemetry.time
telemetry.x        telemetry.vx
telemetry.y        telemetry.vy
telemetry.theta    telemetry.omega
telemetry.T        telemetry.tauRCS
telemetry.ax       telemetry.ay
telemetry.mass     telemetry.fuelMass
```

Optional (used by `prepareResponseData.m` if present; safely omitted otherwise):

```
telemetry.targetX          % scalar target/reference trajectory, same length as time
telemetry.predictedX       % MPC-predicted trajectory; may be a DIFFERENT length (per horizon) - passed through as-is, not forced into the core struct
telemetry.predictedTheta   % same variable-length caveat
telemetry.referenceAx
telemetry.referenceTheta
```

`analyzeFlight.m` validates that all 13 required fields are present and equal-length to `telemetry.time` before doing anything else, and raises a clear error naming the missing/mismatched field if not.

---

## 5. Analysis-result structure

`results = analyzeFlight(telemetry, params)` returns:

```
results.maxAltitude
results.maxHorizontalDisplacement
results.maxAbsAcceleration
results.maxAbsAttitude
results.totalFlightTime
results.landingVx / landingVy / landingTheta
results.fuelConsumed              % = fuelMass(1) - fuelMass(end)  (dimensionally correct)
results.fuelConsumedNumerical     % = integral(T/ve dt), NaN if params.ve not supplied — independent cross-check only
results.totalPathLength           % = integral(speed dt) via compositeTrapezoidal
results.landingStatus             % 'safe' | 'hard' | 'crash'
results.numericalVx / numericalVy % finite-difference estimate from x(t)/y(t)
results.numericalAx / numericalAy % finite-difference estimate from vx(t)/vy(t)
results.accelerationSource        % struct labeling which field is analytical vs. numerically estimated
```

`params` (optional fields): `.ve` (exhaust velocity, for the fuel cross-check), `.Tmax` (thrust reference line), `.safeLandingVySpeed`, `.safeLandingTheta` (landing-classification thresholds; defaults 2 m/s and 10°).

**Physics note honored from the spec:** `results.ax`/`ay` (the physics engine's analytical values, unchanged from telemetry) and `results.numericalAx`/`numericalAy` (Experiment 6 finite-difference estimates) are kept and labeled separately everywhere — they are never conflated.

`plotData = prepareResponseData(telemetry, results, params)` returns four groups (`.position`, `.acceleration`, `.attitude`, `.thrust`), matching the four required performance plots. It only prepares data; it does not create or own any axes.

---

## 6. Running the tests

```matlab
addpath('numerical'); addpath('analysis'); addpath('tests');
report1 = testNumericalMethods();   % prints a table: Test | Expected | Computed | AbsError | RelError | Iterations/Extra
report2 = testFlightAnalysis();     % prints a table for the analysis pipeline, using synthetic telemetry with a known closed-form solution
```

Every numerical-method function was manually cross-checked (via an Octave port used only for verification during development — the delivered `.m` files use `cell2table`, which is MATLAB-only and not available in Octave, so run the tests in MATLAB to see the formatted report). Results obtained during that verification pass:

| Test | Expected | Computed | Notes |
|---|---|---|---|
| Lagrange interpolation, (x-1)² at x=1.5 | 0.25 | 0.25 | exact (polynomial degree matches node count) |
| Linear regression [a0,a1] | [2, 3] | [2.000000, 3.000000] | exact fit, no noise |
| Polynomial regression [a0,a1,a2] | [1,-3,2] | [1.000000,-3.000000,2.000000] | exact fit, no noise |
| Gauss-Jordan solution | [2,3,-1] | [2.000000,3.000000,-1.000000] | residual 0.0 |
| Central finite difference, sin at π/4 | 0.70710678 | 0.70710678 | h = 1e-4 |
| Richardson derivative, sin at π/4 | 0.7071067812 | 0.7071067812 | h = 1e-2; **see bug note below** |
| Composite Trapezoidal, ∫sin, 0..π | 2 | 1.99983550 | n=100, O(h²) error as expected |
| Composite Simpson 1/3, ∫sin, 0..π | 2 | 2.00000001 | n=100, O(h⁴) |
| Composite Simpson 3/8, ∫sin, 0..π | 2 | 2.00000003 | n=99 |
| Adaptive Simpson, ∫sin, 0..π | 2 | 2.0000000000 | tol=1e-8, 177 evaluations |
| Bisection root of x²-2 | 1.4142135624 | 1.4142135623 | 35 iterations |
| False Position root of x²-2 | 1.4142135624 | 1.4142135624 | 15 iterations |
| Newton-Raphson root of x²-2 | 1.4142135624 | 1.4142135624 | 6 iterations |
| Secant root of x²-2 | 1.4142135624 | 1.4142135624 | 8 iterations |

**Bug found and fixed during verification:** the first draft of `richardsonDerivative.m` used the `(16·D(h/2) − D(h))/15` combination, which is only correct when combining two estimates that are *already* 4th-order accurate (e.g. the second column of a Romberg integration table). A plain central difference has leading error `O(h²)`, so the correct single-step Richardson combination is `(4·D(h/2) − D(h))/3`. Using the wrong coefficient silently left most of the `O(h²)` error uncancelled (error ≈ 2.4×10⁻⁶ instead of ≈1.5×10⁻¹¹ at h=1e-2). This was only caught by actually running the code against a known derivative — the corrected formula is what ships in `richardsonDerivative.m`, with the derivation left in a code comment so it is easy to check.

Full pipeline check (`testFlightAnalysis.m`, synthetic free-fall with known closed-form solution, y₀=100 m, g=1.62 m/s², landing at t≈11.2 s): `maxAltitude`, `totalFlightTime`, `landingVy`, `fuelConsumed`, and `totalPathLength` all matched their independently-computed expected values exactly; the finite-difference-estimated `numericalAy` matched −g exactly at interior points (expected, since `vy(t)` is exactly linear in this synthetic case, and central differences are exact for linear data).

---

## 7. Which function corresponds to which EEE 212 experiment

| Experiment | Topic | Function(s) |
|---|---|---|
| 3 | Interpolation | `lagrangeInterpolation.m` — terrain/pad elevation |
| 4 | Curve fitting | `linearRegression.m`, `polynomialRegression.m` — thruster calibration / synthetic fits |
| 5 | Simultaneous linear equations | `gaussJordanElimination.m` — used internally by regression, and standalone in `testNumericalMethods.m` |
| 6 | Numerical differentiation | `finiteDifference.m`, `richardsonDerivative.m` — telemetry-derived velocity/acceleration |
| 7 | Numerical integration | `compositeTrapezoidal.m`, `compositeSimpson13.m`, `compositeSimpson38.m`, `adaptiveIntegration.m` — path length, fuel cross-check |
| 8 | Nonlinear equations | `bisection.m`, `falsePosition.m`, `newtonRaphson.m`, `secant.m` — auxiliary root-finding, not the MPC optimizer |

Root-finding is used as an **auxiliary** calculation (demonstrated in `testNumericalMethods.m` against a known nonlinear equation), not as the MPC optimizer — Newton-Raphson is not forced into the MPC.

---

## 8. Instructions for Person 4

1. After each simulation run, populate the `telemetry` struct (section 4) and call:
   ```matlab
   results  = analyzeFlight(telemetry, params);
   plotData = prepareResponseData(telemetry, results, params);
   ```
2. Use `plotData.position`, `plotData.acceleration`, `plotData.attitude`, `plotData.thrust` to drive the four required response plots in App Designer. Each optional series (`targetX`, `predictedX`, `referenceAx`, `predictedTheta`, `referenceTheta`, `TmaxLine`) is `[]` when the corresponding telemetry/params field wasn't supplied — check `isempty(...)` before plotting that series.
3. `telemetry.predictedX` / `telemetry.predictedTheta` are passed through unmodified if present (their length may differ from `telemetry.time` because of the MPC horizon) — don't assume they're the same length as everything else.
4. `results.landingStatus` ('safe'/'hard'/'crash') and the scalar metrics in `results` are ready to display directly in a summary panel; no further processing needed.
5. Person 3's module never touches App Designer axes, keyboard input, physics stepping, or the MPC solver — if something in those areas breaks, it isn't in `numerical/` or `analysis/`.
