function results = calculateFlightMetrics(telemetry, params)
%CALCULATEFLIGHTMETRICS Compute summary flight metrics from telemetry.
%
%   results = calculateFlightMetrics(telemetry, params)
%
% Uses Person 3's hand-coded numerical methods (finiteDifference,
% compositeTrapezoidal) to compute quantities that are not directly
% logged, and reports both raw telemetry-derived metrics and
% numerically-estimated velocity/acceleration for comparison against
% the physics engine's analytical values.
%
% Output fields:
%   maxAltitude, maxHorizontalDisplacement, maxAbsAcceleration,
%   maxAbsAttitude, totalFlightTime, landingVx, landingVy, landingTheta,
%   fuelConsumed, fuelConsumedNumerical, totalPathLength, landingStatus,
%   numericalVx, numericalVy, numericalAx, numericalAy, accelerationSource

    t  = telemetry.time(:);
    x  = telemetry.x(:);
    y  = telemetry.y(:);
    vx = telemetry.vx(:);
    vy = telemetry.vy(:);
    ax = telemetry.ax(:);
    ay = telemetry.ay(:);
    theta = telemetry.theta(:);
    T  = telemetry.T(:);
    fuelMass = telemetry.fuelMass(:);

    % --- basic extrema / totals -------------------------------------------
    results.maxAltitude               = max(y);
    results.maxHorizontalDisplacement = max(abs(x - x(1)));
    results.maxAbsAcceleration        = max(sqrt(ax.^2 + ay.^2));
    results.maxAbsAttitude            = max(abs(theta));
    results.totalFlightTime           = t(end) - t(1);

    % --- landing state (final telemetry sample) -----------------------------
    results.landingVx    = vx(end);
    results.landingVy    = vy(end);
    results.landingTheta = theta(end);

    % --- fuel consumed: dimensionally correct physical quantity -------------
    % fuelUsed = initialFuelMass - finalFuelMass  (NOT an integral of mass)
    results.fuelConsumed = fuelMass(1) - fuelMass(end);

    % Optional independent numerical cross-check: integrate mass-flow
    % rate mdot = T/ve over time (Experiment 7 usage). Only computed if
    % an exhaust velocity is supplied, since mdot = T/ve requires it.
    if isfield(params, 've') && ~isempty(params.ve) && params.ve ~= 0
        mdot = abs(T) / params.ve;
        results.fuelConsumedNumerical = compositeTrapezoidal(t, mdot);
    else
        results.fuelConsumedNumerical = NaN;
    end

    % --- path length via hand-coded composite numerical integration ---------
    speed = sqrt(vx.^2 + vy.^2);
    results.totalPathLength = compositeTrapezoidal(t, speed);

    % --- landing status -------------------------------------------------------
    results.landingStatus = classifyLanding(telemetry, params);

    % --- numerically estimated velocity/acceleration from telemetry ---------
    % (Experiment 6, finite differences). These are DISTINCT from the
    % physics engine's analytical ax/ay - see accelerationSource below.
    results.numericalVx = finiteDifference(x, t);
    results.numericalVy = finiteDifference(y, t);
    results.numericalAx = finiteDifference(vx, t);
    results.numericalAy = finiteDifference(vy, t);

    results.accelerationSource = struct( ...
        'modelPhysics', 'telemetry.ax / telemetry.ay (analytical, from the physics engine)', ...
        'numericalEstimate', 'results.numericalAx / results.numericalAy (finite-difference estimate from telemetry.vx/vy)');
end

function status = classifyLanding(telemetry, params)

    finalState = [ ...
        telemetry.x(end); ...
        telemetry.vx(end); ...
        telemetry.y(end); ...
        telemetry.vy(end); ...
        telemetry.theta(end); ...
        telemetry.omega(end)];

    [status, ~, ~] = checkLandingStatus(finalState, params);

end
