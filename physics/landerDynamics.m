function dstate = landerDynamics(state, control, mass, params)
%LANDERDYNAMICS Continuous-time nonlinear equations of motion for the lander.
%
%   dstate = landerDynamics(state, control, mass, params)
%
%   Evaluates the right-hand side of the ODE:
%       dx/dt     = vx
%       dy/dt     = vy
%       dvx/dt    = (T/m) * sin(theta)
%       dvy/dt    = (T/m) * cos(theta) - g
%       dtheta/dt = omega
%       domega/dt = tauRCS / I
%
%   COORDINATE CONVENTION (fixed for the whole project - do not change):
%       theta = 0   -> thrust vector points straight up (+y)
%       theta > 0   -> thrust tilts toward +x
%       omega > 0   -> increases theta
%
%   INPUTS
%       state   6x1 [x; vx; y; vy; theta; omega]
%       control 2x1 [T; tauRCS] - ALREADY saturated/fuel-limited by the
%               caller (simulateStep). This function applies no limits.
%       mass    scalar, current vehicle mass in kg (must be > 0)
%       params  struct, must contain params.g and params.I
%
%   OUTPUT
%       dstate  6x1 time derivative of state, same field ordering as state
%
%   DESIGN NOTE: This function is intentionally "pure" - no saturation,
%   no validation, no side effects. All limiting logic lives in one place
%   (simulateStep.m) so there is exactly one implementation of each rule
%   and eulerStep/rk2Step/tests can all call the identical dynamics.

    vx    = state(2);
    theta = state(5);
    omega = state(6);
    vy    = state(4);

    T      = control(1);
    tauRCS = control(2);

    ax = (T / mass) * sin(theta);
    ay = (T / mass) * cos(theta) - params.g;

    dstate = zeros(6, 1);
    dstate(1) = vx;      % dx/dt
    dstate(2) = ax;      % dvx/dt
    dstate(3) = vy;      % dy/dt
    dstate(4) = ay;      % dvy/dt
    dstate(5) = omega;   % dtheta/dt
    dstate(6) = tauRCS / params.I;  % domega/dt

end
