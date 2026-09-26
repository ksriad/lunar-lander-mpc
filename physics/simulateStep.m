function [stateNext, paramsNext, telemetry] = simulateStep(state, control, params)
%SIMULATESTEP Advance the lunar-lander physics by one fixed timestep.
%
%   [stateNext, paramsNext, telemetry] = simulateStep(state, control, params)
%
%   THIS IS THE PUBLIC INTERFACE FOR THE WHOLE TEAM. Do not change the
%   number/order of inputs or outputs without re-syncing with Person 2
%   (MPC controller) and Person 4 (App Designer GUI).
%
%   INPUTS
%       state   6x1 (or 1x6) [x; vx; y; vy; theta; omega]   (SI units)
%       control 2x1 (or 1x2) [T; tauRCS]  - the COMMANDED control, before
%               any saturation. This function performs all saturation.
%       params  struct with at least the fields:
%               g, dt, mass, dryMass, fuelMass, ve, I, Tmax, tauMax,
%               thetaMax, integrationMethod ('Euler' | 'RK2')
%               (see validatePhysicsParameters.m for the exact contract)
%
%   OUTPUTS
%       stateNext   6x1, same field ordering as state
%       paramsNext  a COPY of params with .mass and .fuelMass updated to
%                   reflect the fuel burned this step. Every other field
%                   is passed through unchanged. Because MATLAB structs
%                   are passed by value, the ORIGINAL params is never
%                   modified - the caller MUST capture and reuse
%                   paramsNext on the next call, e.g.:
%
%                   [state, params, telemetry] = simulateStep(state, control, params);
%                   % ... next control cycle ...
%                   [state, params, telemetry] = simulateStep(state, control, params);
%
%       telemetry   struct with (at minimum) the fields required by the
%                   project spec, plus a few extras useful to Person 3's
%                   dashboard/plots. Full field list in
%                   documentation/PHYSICS_ENGINE.md.
%
%   WHY THREE OUTPUTS: the project's state vector is fixed at 6 elements
%   (x, vx, y, vy, theta, omega) and explicitly does NOT include mass.
%   Mass still evolves over time (fuel burn), so it is carried in params
%   instead of state. Since params is passed by value, the only way to
%   propagate the updated mass forward is to return a new params struct.
%
%   SATURATION ORDER (see documentation/PHYSICS_ENGINE.md for the full
%   rationale of every decision made here):
%       1. Commanded T is clamped to [0, Tmax]; commanded tauRCS is
%          clamped to [-tauMax, tauMax].
%       2. If the remaining fuel cannot sustain that thrust for a full
%          dt, thrust is further reduced so fuel can never go negative
%          (an exact, closed-form limit - see "Fuel-limited thrust"
%          below - not a post-hoc clamp).
%       3. After integration, theta is clamped to [-thetaMax, thetaMax]
%          as a hard physical safety boundary (e.g. gimbal/structural
%          limit). This is NOT a substitute for the MPC controller
%          keeping theta in bounds - it is a last-resort safety net, and
%          it also zeros omega so the state does not keep driving into
%          the boundary. See "Attitude safety boundary" below.

    %% 1-2. Validate state and control first - fail loudly, never
    %% silently propagate corrupted data (project spec, section 9).
    state   = localValidateState(state);
    control = localValidateControl(control);
    validatePhysicsParameters(params);

    dt       = params.dt;
    dryMass  = params.dryMass;
    fuelMass = params.fuelMass;
    ve       = params.ve;
    Tmax     = params.Tmax;
    tauMax   = params.tauMax;
    thetaMax = params.thetaMax;

    %% 3. Read the current mass from params (never hard-coded).
    mass = params.mass;
    if ~(mass > 1e-9)
        error('PhysicsEngine:InvalidMass', ...
            'params.mass is zero, near-zero, or negative (%.6g kg); cannot integrate.', mass);
    end

    %% 4. Saturate the commanded control to actuator limits.
    Traw      = control(1);
    tauRCSraw = control(2);

    T_sat   = min(max(Traw, 0), Tmax);
    tau_sat = min(max(tauRCSraw, -tauMax), tauMax);

    thrustCommandSaturated = (T_sat ~= Traw);
    torqueCommandSaturated = (tau_sat ~= tauRCSraw);

    %% Fuel-limited thrust.
    % With T held constant over dt, dm/dt = -T/ve integrates EXACTLY to
    % fuelBurn = (T/ve)*dt (no discretization error, regardless of Euler
    % vs RK2). Solving fuelBurn <= fuelMass for T gives the largest
    % thrust the remaining fuel can sustain for one full step. Clamping
    % to this value up front - rather than integrating first and
    % clamping fuel to zero afterwards - means the thrust actually used
    % in the dynamics (and reported in telemetry) is the thrust that was
    % truly deliverable, so ax/ay/telemetry.thrust are never "wrong" by
    % silently ignoring a fuel shortfall.
    fuelAlreadyEmpty = (fuelMass <= 0);
    if fuelAlreadyEmpty
        T_eff = 0;
    else
        maxThrustFromFuel = fuelMass * ve / dt;
        T_eff = min(T_sat, maxThrustFromFuel);
    end
    fuelLimitedThrust = (T_eff < T_sat);

    controlEff = [T_eff; tau_sat];

    %% 5-6. Compute acceleration and integrate the state.
    method = lower(char(params.integrationMethod));
    switch method
        case 'euler'
            stateNext = eulerStep(state, controlEff, mass, params);
        case 'rk2'
            stateNext = rk2Step(state, controlEff, mass, params);
        otherwise
            % Unreachable in practice (validatePhysicsParameters already
            % checked this) but kept so this function still fails loudly
            % even if that check is ever bypassed.
            error('PhysicsEngine:InvalidParams', ...
                'Unknown integrationMethod "%s".', params.integrationMethod);
    end

    if ~all(isfinite(stateNext))
        error('PhysicsEngine:NumericalFailure', ...
            ['Integration produced a non-finite state (NaN/Inf). ' ...
             'Check dt, control magnitude, and params for extreme values.']);
    end

    %% 7. Landing-pad collision / touchdown.
    % The bottom tips of the three landing legs are the physical contact
    % points.  A valid touchdown is resolved immediately when all three
    % pads are simultaneously on the landing surface and level.
    [stateNext, padContact, safeTouchdown] = enforceLandingPadCollision(stateNext, params);

    %% 8. Attitude safety boundary (physical clamp, not a control law).
    % If theta exceeds the physical limit we clamp it AND zero omega, so
    % the state does not remain "pressed against" a wall it cannot
    % physically cross. This is a deliberate, documented safety net for
    % e.g. a gimbal or structural limit - the MPC controller (Person 2)
    % is expected to keep theta inside [-thetaMax, thetaMax] on its own;
    % hitting this clamp in normal operation indicates a controller or
    % tuning problem, not expected behavior.
    thetaSaturated = false;
    if abs(stateNext(5)) > thetaMax
        stateNext(5) = sign(stateNext(5)) * thetaMax;
        stateNext(6) = 0;
        thetaSaturated = true;
    end

    %% 9. Update fuel mass (never negative, mass never below dryMass).
    fuelBurn    = T_eff * dt / ve;
    newFuelMass = max(0, fuelMass - fuelBurn);
    newMass     = dryMass + newFuelMass;

    if newMass < dryMass - 1e-9
        error('PhysicsEngine:InvalidMass', ...
            'Computed mass (%.6g) fell below dryMass (%.6g); this should be impossible.', ...
            newMass, dryMass);
    end

    paramsNext          = params;   % copy-through; params passed by value
    paramsNext.mass     = newMass;
    paramsNext.fuelMass = newFuelMass;

    %% 10. Telemetry.
    % ax/ay/omegaDot are evaluated at the FINAL (post-integration,
    % post-clamp) attitude, using the mass that was actually held
    % constant during this step's integration (params.mass, i.e. `mass`
    % above) - NOT the post-burn mass. This keeps telemetry internally
    % consistent with what the integrator actually computed, and keeps
    % TEST 1 / TEST 2 style checks numerically exact rather than
    % approximate. The post-burn mass is reported separately as
    % telemetry.mass / paramsNext.mass for use starting next step.
    dstateFinal = landerDynamics(stateNext, controlEff, mass, params);

    telemetry = struct();
    telemetry.ax        = dstateFinal(2);
    telemetry.ay        = dstateFinal(4);
    telemetry.omegaDot  = dstateFinal(6);   % rad/s^2, bonus for Person 3
    telemetry.mass      = newMass;
    telemetry.fuelMass  = newFuelMass;
    telemetry.thrust    = T_eff;            % actual (post-saturation) thrust
    telemetry.torque    = tau_sat;          % actual (post-saturation) torque
    telemetry.theta     = stateNext(5);
    telemetry.omega     = stateNext(6);
    telemetry.x         = stateNext(1);
    telemetry.vx        = stateNext(2);
    telemetry.altitude  = stateNext(3);     % = y, named for clarity downstream
    telemetry.vy        = stateNext(4);
    telemetry.dt        = dt;
    telemetry.integrationMethod   = params.integrationMethod;
    telemetry.thrustCommandSaturated = thrustCommandSaturated;
    telemetry.torqueCommandSaturated = torqueCommandSaturated;
    telemetry.fuelLimitedThrust       = fuelLimitedThrust;
    telemetry.fuelDepleted            = (newFuelMass <= 0);
    telemetry.thetaSaturated          = thetaSaturated;
    telemetry.padContact              = padContact;
    telemetry.touchdown               = safeTouchdown;
    telemetry.engineCutoff            = safeTouchdown;

    % Once a valid touchdown is detected, the engine is considered cut
    % immediately.  The final integration step used the commanded thrust
    % up to the instant of contact; subsequent commands are zero.
    if safeTouchdown
        telemetry.thrust = 0;
        telemetry.torque = 0;
    end

end


%% ==================================================================
function [state, contact, safeTouchdown] = enforceLandingPadCollision(state, params)
%ENFORCELANDINGPADC OLLISION Resolve the three-foot touchdown.
%   Contact is evaluated with the same geometry used by the GUI and by
%   checkLandingStatus.  Only a simultaneous, balanced, safe-kinematic
%   three-pad touchdown is snapped to the pad surface.

    contact = false;
    safeTouchdown = false;

    if ~isfield(params, 'target') || ~isfield(params.target, 'x') || ...
            ~isfield(params.target, 'y') || ~isfield(params.target, 'halfWidth')
        return
    end

    c = getLandingContact(state, params);
    contact = c.contactDetected;

    if ~contact
        return
    end

    if c.safeTouchdown
        geom = getLandingGeometry(params);

        % Snap the common foot plane exactly onto the pad.  In the planar
        % three-leg model this is the physically meaningful equilibrium.
        state(5) = 0;
        state(6) = 0;
        state(3) = params.target.y - geom.footY;

        % A successful touchdown must also be nearly settled horizontally.
        % The status checker applies the same vx limit.
        safeTouchdown = true;
    end
end

%% ==================================================================
function state = localValidateState(state)
%LOCALVALIDATESTATE Check size/type/finiteness and force a 6x1 column.
    if ~isnumeric(state) || ~isvector(state) || numel(state) ~= 6
        error('PhysicsEngine:InvalidState', ...
            'state must be a 6-element numeric vector [x;vx;y;vy;theta;omega] (got %s of size %s).', ...
            class(state), mat2str(size(state)));
    end
    state = double(state(:));
    if ~all(isfinite(state)) || ~isreal(state)
        error('PhysicsEngine:InvalidState', ...
            'state contains NaN, Inf, or complex values.');
    end
end

%% ==================================================================
function control = localValidateControl(control)
%LOCALVALIDATECONTROL Check size/type/finiteness and force a 2x1 column.
    if ~isnumeric(control) || ~isvector(control) || numel(control) ~= 2
        error('PhysicsEngine:InvalidControl', ...
            'control must be a 2-element numeric vector [T; tauRCS] (got %s of size %s).', ...
            class(control), mat2str(size(control)));
    end
    control = double(control(:));
    if ~all(isfinite(control)) || ~isreal(control)
        error('PhysicsEngine:InvalidControl', ...
            'control contains NaN, Inf, or complex values.');
    end
end
