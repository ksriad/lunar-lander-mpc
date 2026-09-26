function results = analyzeFlight(telemetry, params)
%ANALYZEFLIGHT Top-level post-flight numerical analysis (Person 3 module).
%
%   results = analyzeFlight(telemetry, params)
%
% Validates the standardized telemetry structure and delegates to
% calculateFlightMetrics. This is the single entry point Person 4
% should call after a run to get all performance metrics.
%
% Inputs:
%   telemetry - struct with the required fields listed in
%               documentation/NUMERICAL_METHODS.md (time, x, vx, y, vy,
%               theta, omega, T, tauRCS, ax, ay, mass, fuelMass, and
%               optionally predictedX/predictedY/predictedTheta,
%               targetX, referenceAx, referenceTheta).
%   params    - struct, may be empty. Recognized optional fields:
%               .ve (exhaust velocity, m/s, for the numerical
%               fuel-consumption cross-check), .Tmax (for thrust plot
%               reference line), .safeLandingVySpeed,
%               .safeLandingTheta (landing-status thresholds).
%
% Output:
%   results - struct, see calculateFlightMetrics.m for the full field
%             list (max altitude, max horizontal displacement, max
%             |accel|, max |attitude|, total flight time, landing
%             vx/vy/theta, fuel consumed, path length, landing status,
%             plus numerically-estimated velocity/acceleration).

    if nargin < 2
        params = struct();
    end

    validateTelemetry(telemetry);

    results = calculateFlightMetrics(telemetry, params);
end

function validateTelemetry(telemetry)
    requiredFields = {'time','x','vx','y','vy','theta','omega', ...
                       'T','tauRCS','ax','ay','mass','fuelMass'};
    for i = 1:numel(requiredFields)
        if ~isfield(telemetry, requiredFields{i})
            error('analyzeFlight:missingField', ...
                'telemetry is missing required field "%s".', requiredFields{i});
        end
    end
    n = numel(telemetry.time);
    for i = 1:numel(requiredFields)
        if numel(telemetry.(requiredFields{i})) ~= n
            error('analyzeFlight:inconsistentLength', ...
                'telemetry.%s must have the same length as telemetry.time.', requiredFields{i});
        end
    end
end
