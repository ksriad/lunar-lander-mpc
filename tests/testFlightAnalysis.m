function report = testFlightAnalysis()
%TESTFLIGHTANALYSIS Validation test for the flight-analysis pipeline.
%
%   report = testFlightAnalysis()
%
% Builds a synthetic telemetry dataset with a KNOWN closed-form motion
% (constant lunar gravity free-fall with constant horizontal velocity,
% zero thrust/attitude) so analyzeFlight's outputs can be checked
% against analytically known results. This is a pipeline/integration
% test, not a substitute for Person 4's physics engine.

    telemetry = generateSyntheticTelemetry();

params.ve = 3000;
params.Tmax = 5000;

% Landing-pad definition required by the shared landing-status checker.
% The synthetic trajectory is only testing the analysis pipeline, so place
% the pad at the final synthetic x-position.
params.target.x = telemetry.x(end);
params.target.y = 0;
params.target.halfWidth = 1.0;

% Project landing limits.
params.landing.vxMax = 0.5;
params.landing.vyMax = 2.0;
params.landing.thetaMax = 0.05;

results = analyzeFlight(telemetry, params);

    g = 1.62; % lunar gravity used to build the synthetic data
    t = telemetry.time;

    rows = {};
    rows = addRow(rows, 'maxAltitude matches telemetry peak', max(telemetry.y), results.maxAltitude);
    rows = addRow(rows, 'totalFlightTime matches telemetry span', t(end) - t(1), results.totalFlightTime);
    rows = addRow(rows, 'landingVy matches final telemetry sample', telemetry.vy(end), results.landingVy);

    % vy(t) is exactly linear in this synthetic dataset, so a central
    % finite difference of vy should recover -g with (near) zero error
    % at interior points - a clean check of Experiment 6 in this pipeline.
    meanNumericalAy = mean(results.numericalAy(2:end-1));
    rows = addRow(rows, 'numerical ay approx -g (finite-difference check, Exp.6)', -g, meanNumericalAy);

    % Independent manual trapezoid check of the path-length integration (Exp.7)
    speed = sqrt(telemetry.vx.^2 + telemetry.vy.^2);
    independentPathLength = sum(0.5*(speed(1:end-1) + speed(2:end)) .* diff(t));
    rows = addRow(rows, 'totalPathLength matches independent trapezoid check', ...
        independentPathLength, results.totalPathLength);

    fuelUsedExpected = telemetry.fuelMass(1) - telemetry.fuelMass(end);
    rows = addRow(rows, 'fuelConsumed matches initial-minus-final fuel mass', ...
        fuelUsedExpected, results.fuelConsumed);

    report = cell2table(rows, 'VariableNames', {'Test','Expected','Computed','AbsError','RelError'});
    disp(report);

    % Also sanity-check that prepareResponseData runs without error and
    % produces the four expected top-level plot groups.
    plotData = prepareResponseData(telemetry, results, params); %#ok<NASGU>
    requiredGroups = {'position','acceleration','attitude','thrust'};
    for i = 1:numel(requiredGroups)
        assert(isfield(plotData, requiredGroups{i}), ...
            'prepareResponseData is missing plot group "%s".', requiredGroups{i});
    end
    fprintf('prepareResponseData: all 4 required plot-data groups present.\n');
end

function telemetry = generateSyntheticTelemetry()
    % Synthetic descent with a known closed-form solution: constant
    % horizontal velocity, constant lunar gravitational acceleration,
    % zero thrust/attitude. Used purely to validate the analysis
    % pipeline - NOT a substitute for Person 4's physics engine.
    g = 1.62;
    vx0 = 5;
    y0 = 100;
    vy0 = 0;

    % y0=100, g=1.62 => y(t)=0 at t = sqrt(2*y0/g) ~= 11.11 s, so the
    % sample range must extend past that for the ground-clip below to
    % find a real landing sample (rather than clipping to empty).
    t = (0:0.1:15)';
    n = numel(t);

    x  = vx0 * t;
    vx = vx0 * ones(n, 1);
    y  = y0 + vy0*t - 0.5*g*t.^2;
    vy = vy0 - g*t;

    % Clip at "ground" (y = 0) for a physically sensible landing sample.
    landedIdx = find(y <= 0, 1, 'first');
    if isempty(landedIdx)
        error('testFlightAnalysis:noLanding', ...
            'Synthetic trajectory never reaches y<=0 within the sampled time range.');
    end
    t = t(1:landedIdx); x = x(1:landedIdx); vx = vx(1:landedIdx);
    y = y(1:landedIdx); vy = vy(1:landedIdx);
    n = landedIdx;

    telemetry.time = t;
    telemetry.x = x;
    telemetry.vx = vx;
    telemetry.y = y;
    telemetry.vy = vy;
    telemetry.theta = zeros(n, 1);
    telemetry.omega = zeros(n, 1);
    telemetry.T = zeros(n, 1);
    telemetry.tauRCS = zeros(n, 1);
    telemetry.ax = zeros(n, 1);
    telemetry.ay = -g * ones(n, 1);
    telemetry.mass = 1000 * ones(n, 1);
    telemetry.fuelMass = 300 - 0.05*t;   % arbitrary slow synthetic burn, for the fuel-consumed check
end

function rows = addRow(rows, name, expected, computed)
    absErr = abs(expected - computed);
    if expected ~= 0
        relErr = absErr / abs(expected);
    else
        relErr = NaN;
    end
    rows(end+1, :) = {name, expected, computed, absErr, relErr};
end
