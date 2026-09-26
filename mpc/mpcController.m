function [control, prediction, info] = mpcController(state, target, params)
%MPCCONTROLLER Autonomous receding-horizon MPC controller for the lunar
%   lander (EEE 212 project, Person 2's module ONLY -- no GUI, no
%   keyboard control, no physics integration). See
%   documentation/MPC_ENGINE.md for the full design write-up.
%
%   [control, prediction, info] = mpcController(state, target, params)
%
%   ---------------------------------------------------------------
%   STATE (input, from Person 1's physics engine), 6x1:
%       state = [x; vx; y; vy; theta; omega]
%           x,y   (m)     vx,vy (m/s)     theta (rad)     omega (rad/s)
%
%   TARGET (input), struct with REQUIRED fields:
%       target.xTarget  - landing pad x position (m)
%       target.yTarget  - landing pad elevation / y position (m)
%     Optional fields (default 0 if absent):
%       target.vxTarget, target.vyTarget, target.thetaTarget, target.omegaTarget
%
%   PARAMS (input), struct with REQUIRED fields:
%       params.mass        - CURRENT lander mass (kg) [changes as fuel burns;
%                          Person 1 owns the fuel model dm/dt = -T/ve --
%                          this controller only reads the current value]
%       params.I        - moment of inertia (kg*m^2)
%       params.g        - gravitational acceleration (m/s^2)
%       params.dt       - control/physics timestep (s); project value 0.04
%       params.Tmax     - max thrust magnitude (N)
%       params.tauMax   - max |tauRCS| (N*m)
%       params.thetaMax - max allowed |theta| (rad), SOFT safety bound
%     Optional tuning fields (defaults applied if absent):
%       params.Np  (default 20)                       - prediction horizon
%       params.Nc  (default 5)                        - control horizon
%       params.Q   (default diag([5 2 20 8 10 2]))    - state tracking weight
%       params.R   (default diag([0.01 0.05]))        - control effort weight
%       params.S   (default diag([0.05 0.1]))         - control increment weight
%
%   OUTPUTS:
%       control    - 2x1 = [T; tauRCS]. This is the ONLY control input
%                    meant to be applied this step (first element of the
%                    optimized sequence, per the receding-horizon
%                    principle, Section 8).
%       prediction - Np x 6 matrix of predicted FUTURE states (rows are
%                    k+1 .. k+Np), for visualization ONLY. Computed from
%                    the UNCONSTRAINED solution -- see info.saturated /
%                    info.safetyTriggered for whether the applied
%                    control differs from what produced this trajectory.
%       info       - struct with fields:
%           .cost           - scalar cost J at the unconstrained optimum
%           .deltaU         - (m*Nc)x1 raw solved control increments
%           .uNominal       - 2x1 unconstrained optimal control BEFORE
%                             saturation/safety correction
%           .iterations     - always 1 (direct linear solve, not an
%                             iterative optimizer; field kept for
%                             interface stability)
%           .solverResidual - norm(H*deltaU - rhs) from gaussJordanSolve
%           .saturated      - 1x2 logical, per-actuator saturation flag
%           .thetaPredicted - max abs predicted theta over the horizon
%           .success        - true if the solve succeeded numerically
%                             with no unrecoverable singularity
%           .singular       - true if gaussJordanSolve hit an
%                             unresolvable singular pivot
%           .illConditioned - true if the smallest usable pivot was
%                             below the conditioning warning threshold
%                             (diagnostic only)
%           .safetyTriggered - true if the soft attitude safety
%                             correction reduced tauRCS this step
%
%   This function does NOT advance the physics and does NOT own the
%   fuel model -- it only reads the current mass from params.mass.

    % ---- 1. validate required fields (no hard-coded PHYSICS values) ----
    requiredParamFields = {'mass','I','g','dt','Tmax','tauMax','thetaMax'};
    for k = 1:numel(requiredParamFields)
        if ~isfield(params, requiredParamFields{k})
            error('mpcController:missingParam', ...
                'params.%s is required.', requiredParamFields{k});
        end
    end
    requiredTargetFields = {'xTarget','yTarget'};
    for k = 1:numel(requiredTargetFields)
        if ~isfield(target, requiredTargetFields{k})
            error('mpcController:missingTarget', ...
                'target.%s is required.', requiredTargetFields{k});
        end
    end
    if numel(state) ~= 6
        error('mpcController:badState', ...
            'state must be a 6x1 vector [x;vx;y;vy;theta;omega].');
    end
    state = state(:);

    Np = getOr(params, 'Np', 70);
Nc = getOr(params, 'Nc', 12);

% Strong terminal tracking: x and vx must converge before touchdown,
% while attitude/rate are heavily penalized to keep the three feet level.
Q = getOr(params, 'Q', diag([30 220 60 260 180 35]));
R = getOr(params, 'R', diag([0.015 0.04]));
S = getOr(params, 'S', diag([0.02 0.04]));

% Tighten terminal tracking as the physical leg tips approach the pad.
% This preserves the same Gauss-Jordan MPC solver while increasing the
% priority of x, vx, theta and omega near touchdown.
if isfield(params, 'landerGeometry') && isfield(params.landerGeometry, 'footY')
    terminalAltitude = state(3) + params.landerGeometry.footY - ...
                       getOr(params.target, 'y', 0);
else
    terminalAltitude = state(3);
end
if terminalAltitude < 35
    terminalFactor = 1.8;
    if terminalAltitude < 15, terminalFactor = 2.5; end
    if terminalAltitude < 8,  terminalFactor = 3.5; end
    Q(1,1) = Q(1,1)*terminalFactor;
    Q(2,2) = Q(2,2)*terminalFactor;
    Q(5,5) = Q(5,5)*terminalFactor;
    Q(6,6) = Q(6,6)*terminalFactor;
end

    vxT    = getOr(target, 'vxTarget', 0);
    vyT    = getOr(target, 'vyTarget', 0);
    thetaT = getOr(target, 'thetaTarget', 0);
    omegaT = getOr(target, 'omegaTarget', 0);
    % -------------------------------------------------------------

    xTargetVec = [target.xTarget; vxT; target.yTarget; vyT; thetaT; omegaT];

    % ---- 2. linear model (recomputed each call since mass changes as
    %          fuel burns -- Section 13: only what depends on mass needs
    %          to be recomputed, and everything here is cheap regardless) ----
    [A, B, u0] = linearizeLander(params.mass, params.I, params.g);
    [Ad, Bd]   = discretizeModel(A, B, params.dt);

    % ---- 3. prediction + cost matrices ----
    [F, Phi] = buildPredictionMatrices(Ad, Bd, Np, Nc);
    [H, rhs, diagInfo] = buildCostMatrices(F, Phi, state, xTargetVec, Q, R, S, Np, Nc);

    % ---- 4. Experiment-5 solve: Gauss-Jordan with partial pivoting ----
    [deltaU, solveInfo] = gaussJordanSolve(H, rhs);

    % ---- 5. reconstruct absolute control + predicted trajectory ----
    deltaU1  = deltaU(1:2);
    uNominal = u0 + deltaU1;

    X = F*state + Phi*deltaU;          % unconstrained predicted states
    prediction = reshape(X, 6, Np).';  % Np x 6, rows = k+1 .. k+Np

    e = X - repmat(xTargetVec, Np, 1);
    J = e.'*diagInfo.Qbar*e + deltaU.'*diagInfo.Rbar*deltaU ...
        + (diagInfo.D*deltaU).'*diagInfo.Sbar*(diagInfo.D*deltaU);

    % ---- 6. actuator saturation + soft attitude safety check ----
    [control, saturated, thetaPredicted, safetyTriggered] = ...
        applyMPCConstraints(uNominal, X, params, 6);

    % ---- 7. info struct ----
    info.cost            = J;
    info.deltaU          = deltaU;
    info.uNominal        = uNominal;
    info.iterations      = 1;
    info.solverResidual  = solveInfo.residual;
    info.saturated       = saturated;
    info.thetaPredicted  = thetaPredicted;
    info.success         = solveInfo.success;
    info.singular        = solveInfo.singular;
    info.illConditioned  = solveInfo.illConditioned;
    info.safetyTriggered = safetyTriggered;
end

function v = getOr(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        v = s.(field);
    else
        v = default;
    end
end
