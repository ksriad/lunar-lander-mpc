function validatePhysicsParameters(params)
%VALIDATEPHYSICSPARAMETERS Validate the shared physics parameter struct.
%
%   validatePhysicsParameters(params)
%
%   Throws a clearly identified error (see error identifiers below) if
%   params is missing a required field or contains a physically invalid
%   value (NaN, Inf, wrong sign, wrong type, etc). Returns nothing on
%   success - MATLAB structs are passed by value, so this function only
%   checks; it never modifies params.
%
%   REQUIRED FIELDS (see documentation/PHYSICS_ENGINE.md, section 5):
%       g, dt, mass, dryMass, fuelMass, ve, I, Tmax, tauMax, thetaMax,
%       integrationMethod ('Euler' or 'RK2', case-insensitive)
%
%   Additional/unknown fields on params are allowed and ignored - Person
%   2 and Person 4 may add their own fields (e.g. a landing-pad position)
%   without this function rejecting the struct.

    if ~isstruct(params) || ~isscalar(params)
        error('PhysicsEngine:InvalidParams', 'params must be a scalar struct.');
    end

    requiredFields = {'g','dt','mass','dryMass','fuelMass','ve','I', ...
                       'Tmax','tauMax','thetaMax','integrationMethod'};

    for k = 1:numel(requiredFields)
        f = requiredFields{k};
        if ~isfield(params, f)
            error('PhysicsEngine:InvalidParams', ...
                'params is missing required field "%s".', f);
        end
    end

    scalarNumericFields = {'g','dt','mass','dryMass','fuelMass','ve','I', ...
                            'Tmax','tauMax','thetaMax'};
    for k = 1:numel(scalarNumericFields)
        f = scalarNumericFields{k};
        v = params.(f);
        if ~isnumeric(v) || ~isscalar(v) || ~isreal(v) || ~isfinite(v)
            error('PhysicsEngine:InvalidParams', ...
                'params.%s must be a finite real scalar (got class %s).', f, class(v));
        end
    end

    if params.dt <= 0
        error('PhysicsEngine:InvalidParams', 'params.dt must be > 0 (got %.6g).', params.dt);
    end

    if params.dryMass <= 0
        error('PhysicsEngine:InvalidParams', 'params.dryMass must be > 0.');
    end

    if params.fuelMass < 0
        error('PhysicsEngine:InvalidParams', 'params.fuelMass cannot be negative.');
    end

    if params.mass < params.dryMass - 1e-9
        error('PhysicsEngine:InvalidParams', ...
            'params.mass (%.6g) cannot be less than params.dryMass (%.6g).', ...
            params.mass, params.dryMass);
    end

    if params.ve <= 0
        error('PhysicsEngine:InvalidParams', 'params.ve (exhaust velocity) must be > 0.');
    end

    if params.I <= 0
        error('PhysicsEngine:InvalidParams', 'params.I (moment of inertia) must be > 0.');
    end

    if params.Tmax <= 0
        error('PhysicsEngine:InvalidParams', 'params.Tmax must be > 0.');
    end

    if params.tauMax <= 0
        error('PhysicsEngine:InvalidParams', 'params.tauMax must be > 0.');
    end

    if params.thetaMax <= 0 || params.thetaMax > pi
        error('PhysicsEngine:InvalidParams', ...
            'params.thetaMax must be in the range (0, pi] radians (got %.6g).', params.thetaMax);
    end

    if ~(ischar(params.integrationMethod) || isstring(params.integrationMethod))
        error('PhysicsEngine:InvalidParams', ...
            'params.integrationMethod must be a char or string: "Euler" or "RK2".');
    end

    method = char(params.integrationMethod);
    if ~any(strcmpi(method, {'Euler', 'RK2'}))
        error('PhysicsEngine:InvalidParams', ...
            'params.integrationMethod must be "Euler" or "RK2" (got "%s").', method);
    end

end
