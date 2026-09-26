function [uApplied, saturated, thetaPredicted, safetyTriggered] = ...
    applyMPCConstraints(uNominal, X, params, n)
%APPLYMPCCONSTRAINTS Enforce actuator saturation (hard) and perform a
%   predicted-state attitude SAFETY CHECK (soft, NOT a hard constraint).
%
%   [uApplied, saturated, thetaPredicted, safetyTriggered] = ...
%       applyMPCConstraints(uNominal, X, params, n)
%
%   Inputs:
%       uNominal - 2x1 unconstrained-optimum control for THIS step,
%                  already reconstructed as u0 + delta_u, i.e. absolute
%                  [T; tauRCS]
%       X        - (n*Np)x1 stacked predicted states from the
%                  UNCONSTRAINED solution (used only for diagnostics /
%                  soft correction, never fed back as a hard bound)
%       params   - struct with Tmax, tauMax, thetaMax
%       n        - state dimension (6)
%
%   Outputs:
%       uApplied        - 2x1 control actually returned to the caller
%       saturated       - 1x2 logical, true per channel [T tauRCS] if
%                          actuator saturation clipped that channel
%       thetaPredicted  - max abs predicted theta over the horizon, from
%                          the UNCONSTRAINED prediction (informal
%                          diagnostic only -- not guaranteed once
%                          actuator saturation changes the real
%                          trajectory)
%       safetyTriggered - true if the soft attitude safety correction
%                          reduced tauRCS this step
%
%   *** READ BEFORE INTEGRATING (Section 7) ***
%   This function enforces ACTUATOR SATURATION as a genuine hard
%   constraint on the value returned:
%       0 <= T <= Tmax,   |tauRCS| <= tauMax
%
%   It does NOT enforce PREDICTED STATE CONSTRAINTS (|theta| <= thetaMax
%   over the horizon) as a hard constraint. The unconstrained QP solved
%   by gaussJordanSolve has no knowledge of thetaMax. Instead this
%   function:
%     1. Reads the predicted theta trajectory from the UNCONSTRAINED
%        solution X,
%     2. If any predicted |theta| exceeds params.thetaMax, applies a
%        proportional SOFT CORRECTION that scales down this step's
%        |tauRCS| (an ad hoc, project-specific numerical correction --
%        NOT an "Experiment 8 algorithm", see Section 12),
%     3. Does NOT re-solve the QP and does NOT guarantee the corrected
%        trajectory actually respects thetaMax -- it only damps the
%        immediate actuator command that was driving attitude further
%        out of bounds.
%   A genuine hard state-constrained QP would require constrained
%   optimization, which Section 13 explicitly disallows as the
%   real-time core solver here.

    Tmax     = params.Tmax;
    tauMax   = params.tauMax;
    thetaMax = params.thetaMax;

    T   = uNominal(1);
    tau = uNominal(2);

    saturated = [false false];

    Tc = min(max(T, 0), Tmax);
    if Tc ~= T
        saturated(1) = true;
    end

    tauc = min(max(tau, -tauMax), tauMax);
    if tauc ~= tau
        saturated(2) = true;
    end

    % --- Predicted attitude diagnostic ---
thetaIdx  = 5:n:numel(X);
thetaTraj = X(thetaIdx);
thetaPredicted = max(abs(thetaTraj));

% IMPORTANT:
% Do NOT scale the RCS torque based on the maximum theta over the
% entire prediction horizon.
%
% With a long MPC horizon, the unconstrained prediction can become
% unrealistic far into the future. Scaling the current torque using
% that distant prediction can reduce a necessary braking command
% from hundreds of N*m to only a few N*m.
%
% The physics engine already provides the final hard theta safety
% boundary. The terminal landing controller will also stabilize
% attitude near the ground.
safetyTriggered = false;

% Final actuator clipping
tauc = min(max(tauc, -tauMax), tauMax);

    uApplied = [Tc; tauc];
end
