function [Ad, Bd] = discretizeModel(A, B, dt)
%DISCRETIZEMODEL First-order (explicit Euler) discretization of the
%   continuous linear model x_dot = A*x + B*delta_u.
%
%   [Ad, Bd] = discretizeModel(A, B, dt)
%
%       Ad = I + A*dt
%       Bd = B*dt
%
%   JUSTIFICATION: dt = 0.04 s is small relative to the lander's
%   attitude/translational time constants for the thrust/torque
%   authority expected in this project, so the first-order approximation
%   of the true zero-order-hold discretization (Ad = expm(A*dt), with a
%   matching integral for Bd) introduces negligible additional error
%   while being essentially free to compute every control step (no
%   matrix exponential), which matters given the 40 ms real-time budget
%   (Section 13 of the spec).
%
%   If a different discretization scheme is substituted, it MUST be
%   documented here and in documentation/MPC_ENGINE.md, and the
%   dimension checks below must still hold.

    n = size(A,1);
    if size(A,2) ~= n
        error('discretizeModel:badA', 'A must be square.');
    end
    if size(B,1) ~= n
        error('discretizeModel:badB', 'B row count must match A.');
    end

    Ad = eye(n) + A*dt;
    Bd = B*dt;
end
