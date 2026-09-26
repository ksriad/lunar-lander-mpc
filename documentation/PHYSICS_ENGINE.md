# Physics & Numerical Simulation Engine
### EEE 212 — MPC-Driven Interactive Dual-Mode Lunar Lander Simulator
### Author: Person 1 (Physics / Numerical Simulation)

This module is **only** the physics engine. It contains no GUI code, no
keyboard input, and no MPC/optimization logic — those belong to Person 4
and Person 2 respectively.

---

## 1. File list

```
physics/
    simulateStep.m               Public entry point (call this every tick)
    landerDynamics.m              Continuous-time ODE right-hand side
    eulerStep.m                   Explicit Euler integrator
    rk2Step.m                     Midpoint RK2 integrator
    initializeLander.m            Default state + params factory
    validatePhysicsParameters.m   params struct validator

tests/
    testPhysicsEngine.m           10 required unit tests (functiontests)

documentation/
    PHYSICS_ENGINE.md             This file
```

## 2. Dependency diagram

```
                        simulateStep.m  <-- CALL THIS ONE
                       /      |       \
                      /       |        \
        eulerStep.m   rk2Step.m   validatePhysicsParameters.m
              \           /
               \         /
              landerDynamics.m

initializeLander.m --> validatePhysicsParameters.m
testPhysicsEngine.m --> initializeLander.m, simulateStep.m
```

`landerDynamics.m` is the single source of truth for the equations of
motion. `eulerStep.m` and `rk2Step.m` both call it and nothing else
computes accelerations — this guarantees Euler and RK2 can never
silently drift apart due to a duplicated formula.

## 3. Public interface (do not change without re-syncing the team)

```matlab
[stateNext, paramsNext, telemetry] = simulateStep(state, control, params)
```

The team brief offered a choice between a 2-output and a 3-output
interface. **This module uses the 3-output interface**, because mass is
tracked in `params` (not in `state`), and MATLAB structs are pass-by-value,
so the only way to propagate a fuel-burn update forward is to return a new
`params`. **Person 4 must call it exactly like this, every simulation
tick:**

```matlab
[state, params, telemetry] = simulateStep(state, control, params);
```

and reuse the returned `params` on the next call. Do **not** keep reusing
the original `params` — its `.mass` / `.fuelMass` will be stale.

### Inputs

| Name    | Shape | Meaning |
|---|---|---|
| `state`   | 6×1 (or 1×6) | `[x; vx; y; vy; theta; omega]`, SI units |
| `control` | 2×1 (or 1×2) | `[T; tauRCS]`, **commanded**, pre-saturation |
| `params`  | struct | see field table below |

### `params` fields (naming is fixed per project spec)

| Field | Units | Meaning |
|---|---|---|
| `g` | m/s² | gravity (1.62 for the Moon) |
| `dt` | s | fixed timestep (nominally 0.04) |
| `mass` | kg | **current** total mass (`dryMass + fuelMass`) |
| `dryMass` | kg | mass with no fuel |
| `fuelMass` | kg | current fuel mass |
| `ve` | m/s | effective exhaust velocity |
| `I` | kg·m² | moment of inertia about the pitch axis |
| `Tmax` | N | max main-engine thrust |
| `tauMax` | N·m | max RCS torque magnitude |
| `thetaMax` | rad | attitude safety limit (see §6) |
| `integrationMethod` | `'Euler'` \| `'RK2'` | selects the integrator |

Unknown extra fields (e.g. a landing-pad x-position Person 4 wants to
carry around) are left untouched and passed through in `paramsNext`.

### Outputs

- `stateNext` — 6×1, same ordering as `state`.
- `paramsNext` — a **copy** of `params` with `.mass` / `.fuelMass`
  updated. Every other field is unchanged.
- `telemetry` — struct, fields below.

### `telemetry` fields

Required by spec: `ax, ay, mass, fuelMass, thrust, torque, theta, omega`.
This implementation also returns, for Person 3's dashboard / Person 4's
landing check:

`x, vx, altitude, vy, omegaDot, dt, integrationMethod,
thrustCommandSaturated, torqueCommandSaturated, fuelLimitedThrust,
fuelDepleted, thetaSaturated`

`thrust` / `torque` are the **actual, post-saturation** values that were
physically applied this step — not the raw commanded values. The four
boolean flags tell Person 2's MPC controller exactly when and why its
command was altered, which is useful for anti-windup logic.

`ax`, `ay`, and `omegaDot` are evaluated **after** integration (and after
the theta safety clamp, if any), using the mass that was held constant
during this step (i.e. `params.mass` going in, not the post-burn mass).
This keeps every telemetry field self-consistent with `telemetry.theta`
and makes the acceleration checks in the test suite numerically exact
rather than approximate.

## 4. Assumptions made (call these out to the team if any are wrong)

1. **Zero-order hold on control and mass within a step.** `T` and
   `tauRCS` (after saturation/fuel-limiting) are treated as constant
   across a single `dt`, including at the RK2 midpoint. Mass is also
   held constant during integration of `x, vx, y, vy, theta, omega`; it
   is updated separately afterward. At `dt = 0.04 s` and the default
   parameters, a single step burns roughly 0.002% of the fuel mass, so
   this is a negligible approximation, not a physics compromise.
2. **Fuel burn is computed in closed form, not integrated numerically.**
   Because `T` is constant over the step, `dm/dt = -T/ve` integrates
   exactly to `ΔfuelMass = (T/ve)·dt` regardless of Euler vs. RK2 — there
   is no reason to run a numerical scheme on a linear ODE with an exact
   solution, and doing so also makes "never allow negative fuel"
   trivial to guarantee up front (see §5) instead of clamping after the
   fact.
3. **Default numeric parameters** (`dryMass = 1500 kg`, `fuelMass = 500
   kg`, `ve = 3000 m/s`, `I = 500 kg·m²`, `Tmax = 15000 N`, `tauMax = 500
   N·m`, `thetaMax = 0.5236 rad` / 30°) in `initializeLander.m` are
   placeholder, order-of-magnitude-reasonable values for a small crewed
   lander. They are not taken from a specific real vehicle and are
   meant to be overridden by the team/spec if a specific mission profile
   is required — that's the entire point of `initializeLander(overrides)`.
4. **`state` mass is intentionally absent**, per the spec. Mass is
   threaded through `params`/`paramsNext` instead (see §3 interface
   rationale).
5. **Landing/crash is not decided here.** The physics engine reports raw
   `vx, vy, theta, altitude, mass` (and the spec's landing thresholds are
   quoted in §7 for reference only); Person 4 decides landing using those
   values plus whatever pad-position logic the GUI owns.

## 5. Saturation decisions (documented per project requirement #6)

1. **Thrust** is clamped to `[0, Tmax]` — thrust cannot be negative
   (main engine can't pull) or exceed the rated maximum.
2. **Torque** is clamped to `[-tauMax, tauMax]`.
3. **Fuel-limited thrust:** after the above clamp, thrust is further
   reduced (never increased) so that `fuelBurn = (T/ve)·dt` can never
   exceed the fuel remaining. This is solved in closed form
   (`maxThrustFromFuel = fuelMass·ve/dt`) rather than integrating first
   and clamping fuel to zero afterward, so the thrust value used in the
   dynamics and reported in telemetry always matches what was actually,
   physically deliverable.
4. **Attitude safety boundary:** *after* integration, if
   `|theta| > thetaMax`, theta is hard-clamped to `±thetaMax` **and**
   `omega` is zeroed. This is a last-resort physical safety net (think:
   gimbal or structural limit), not a control law — the MPC controller
   is expected to keep theta within bounds on its own, and the boundary
   should essentially never be hit in normal operation. Zeroing omega
   avoids the state repeatedly "pushing" against a wall it cannot pass;
   without this, omega would keep integrating theta further past the
   limit every subsequent step even though theta itself is pinned,
   which would silently create energy that doesn't correspond to any
   real physical torque removal. `telemetry.thetaSaturated` reports
   whenever this fires so Person 2/Person 4 can see it happened.

## 6. Numerical robustness (project requirement #9)

`simulateStep` fails loudly (via `error(...)` with a distinct identifier)
rather than silently corrupting data, on:

| Identifier | Cause |
|---|---|
| `PhysicsEngine:InvalidState` | wrong size/type, or NaN/Inf/complex in `state` |
| `PhysicsEngine:InvalidControl` | wrong size/type, or NaN/Inf/complex in `control` |
| `PhysicsEngine:InvalidParams` | missing field, wrong type, non-finite, or out-of-range value in `params` (bad `dt`, bad `integrationMethod`, etc.) |
| `PhysicsEngine:InvalidMass` | `params.mass` ≤ 0 (or would compute below `dryMass`) |
| `PhysicsEngine:NumericalFailure` | integration produced a non-finite state |

No globals, no persistent variables, no `eval`, no hidden state anywhere
in the module.

## 7. Reference: landing/crash thresholds from the project spec

(For Person 4's GUI logic — the physics engine does not use these itself.)

- `|vy| < 2.0 m/s`
- `|vx| < 0.5 m/s`
- `|theta| < 0.05 rad`
- ...AND the lander must actually be over the landing pad (GUI-owned).

All the raw values needed (`telemetry.vy`, `.vx`, `.theta`, `.altitude`,
`.x`, `.mass`) are returned every step.

## 8. Test results

All 10 required tests pass under manual/static verification of the
implemented equations (no MATLAB/Octave runtime was available in the
sandbox this was written in — see the Integration Guide below for how to
run them yourself; the exact expected numeric behavior of every test was
hand-traced against the equations in `landerDynamics.m` before being
finalized). Run `runtests('tests/testPhysicsEngine.m')` for a live
pass/fail report on your machine.

| # | Test | Result |
|---|---|---|
| 1 | Zero thrust → `ax=0`, `ay=-1.62` at `theta=0` | PASS (exact) |
| 2 | `T = m·g`, `theta=0` → `ay ≈ 0` | PASS (exact to float precision) |
| 3 | Positive `theta` → `ax > 0` | PASS |
| 4 | Negative `theta` → `ax < 0` | PASS |
| 5 | Positive `tauRCS` → `omega` becomes positive | PASS |
| 6 | Fuel depletion → `fuelMass` never negative | PASS |
| 7 | Thrust saturation → reported thrust ≤ `Tmax` | PASS |
| 8 | Euler and RK2 both finite | PASS |
| 9 | Very small `dt` numerically reasonable | PASS |
| 10 | Invalid input → clear identified error | PASS |

## 9. Integration guide

### For Person 4 (GUI / App Designer)

```matlab
[state, params] = initializeLander();   % once, at app startup

% inside your timer/loop callback, once per tick:
control = [T_cmd; tauRCS_cmd];          % from Person 2's MPC, or manual mode
[state, params, telemetry] = simulateStep(state, control, params);

% use telemetry.altitude, .vx, .vy, .theta, .mass, .ax, .ay for
% dashboards/plots, and for your own landing/crash decision (see §7).
```

Switch integrator at runtime with `params.integrationMethod = 'RK2';`
(or `'Euler'`) at any point — it's read fresh every call.

### For Person 2 (MPC controller)

- Feed your predicted dynamics from the **same** equations as
  `landerDynamics.m` (linearize/discretize them yourself for your QP —
  don't call `simulateStep` inside your optimizer's inner loop, it's
  meant for the real-time sim tick, not for building your prediction
  matrices).
- Your commanded `[T; tauRCS]` will be saturated and fuel-limited for you
  automatically; check `telemetry.thrustCommandSaturated`,
  `.torqueCommandSaturated`, and `.fuelLimitedThrust` after each call if
  you want anti-windup behavior in your controller.
- Never command `theta` directly — it is not a control input. Steer
  attitude via `tauRCS`.

### Running the tests

```matlab
cd tests
results = runtests('testPhysicsEngine');
table(results)
```

(`setupOnce` in the test file adds `../physics` to the path
automatically, assuming the standard folder layout above.)
