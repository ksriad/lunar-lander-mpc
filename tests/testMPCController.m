function testMPCController()
%TESTMPCCONTROLLER Self-contained test script for the MPC engine
%   (Person 2's module). Run with: testMPCController()
%
%   Requires ONLY base MATLAB (no toolboxes). Does not require Person 1
%   or Person 4's code -- exercises mpcController.m and its helpers
%   directly with synthetic states/targets/params.

    addRelativePaths();

    tests = {
        'T01_AdBd_dimensions',              @test_AdBd_dimensions
        'T02_state_ordering',                @test_state_ordering
        'T03_control_ordering',              @test_control_ordering
        'T04_hover_equilibrium',             @test_hover_equilibrium
        'T05_thrust_upward_accel',           @test_thrust_upward_accel
        'T06_torque_attitude_response',      @test_torque_attitude_response
        'T07_gaussJordan_known_matrix',      @test_gaussJordan_known_matrix
        'T08_near_singular_detected',        @test_near_singular_detected
        'T09_finite_control',                @test_finite_control
        'T10_thrust_within_Tmax',            @test_thrust_within_Tmax
        'T11_torque_within_tauMax',          @test_torque_within_tauMax
        'T12_prediction_has_Np_points',      @test_prediction_has_Np_points
        'T13_target_influences_control',     @test_target_influences_control
        'T14_different_states_diff_control', @test_different_states_diff_control
        'T15_solver_residual_small',         @test_solver_residual_small
        'T16_offnominal_state_runs',         @test_offnominal_state_runs
    };

    results = cell(size(tests,1),1);
    for i = 1:size(tests,1)
        results{i} = runTest(tests{i,1}, tests{i,2});
    end

    printSummary(results);
end

function addRelativePaths()
    here = fileparts(mfilename('fullpath'));
    addpath(fullfile(here, '..', 'mpc'));
end

function r = runTest(name, fn)
    r.name = name;
    try
        fn();
        r.pass = true;
        r.err  = '';
    catch ME
        r.pass = false;
        r.err  = ME.message;
    end
end

function printSummary(results)
    fprintf('\n==== testMPCController results ====\n');
    nPass = 0;
    for i = 1:numel(results)
        r = results{i};
        if r.pass
            fprintf('[PASS] %s\n', r.name);
            nPass = nPass + 1;
        else
            fprintf('[FAIL] %s -- %s\n', r.name, r.err);
        end
    end
    fprintf('%d / %d passed\n', nPass, numel(results));
end

%% ---------------- individual tests ----------------

function test_AdBd_dimensions()
    [A,B,~] = linearizeLander(1000, 5000, 1.62);
    [Ad,Bd] = discretizeModel(A,B,0.04);
    assert(isequal(size(Ad), [6 6]), 'Ad must be 6x6');
    assert(isequal(size(Bd), [6 2]), 'Bd must be 6x2');
end

function test_state_ordering()
    % Position (states 1,3) must not appear in A or B at all
    % (translation invariance -- see linearizeLander.m).
    [A,~,~] = linearizeLander(1000, 5000, 1.62);
    assert(all(A(:,1)==0) && all(A(:,3)==0), ...
        'Position columns of A must be zero (state order check).');
    % theta must be state index 5: A(2,5) must equal g.
    assert(A(2,5) == 1.62, 'A(2,5) must equal g (theta is state index 5).');
end

function test_control_ordering()
    % Column 1 of B is the thrust channel: affects vy (state 4) only.
    % Column 2 of B is the torque channel: affects omega (state 6) only.
    [~,B,~] = linearizeLander(1000, 5000, 1.62);
    assert(B(4,1) ~= 0 && B(6,1) == 0, ...
        'Column 1 of B must be the thrust channel (affects vy only).');
    assert(B(6,2) ~= 0 && B(4,2) == 0, ...
        'Column 2 of B must be the torque channel (affects omega only).');
end

function test_hover_equilibrium()
    params = defaultTestParams();
    state  = [0;0;100;0;0;0]; % hovering at rest, y = 100
    target = struct('xTarget', 0, 'yTarget', 100);
    [control, ~, info] = mpcController(state, target, params);
    T0 = params.mass*params.g;
    assert(abs(control(1) - T0) < 1e-6, ...
        'At exact hover with target = current position, thrust should equal m*g (got %.4f vs %.4f).', control(1), T0);
    assert(abs(control(2)) < 1e-6, 'Torque should be ~0 at hover equilibrium.');
    assert(info.success, 'Solver should succeed at hover equilibrium.');
end

function test_thrust_upward_accel()
    [~,B,~] = linearizeLander(1000, 5000, 1.62);
    assert(B(4,1) > 0, ...
        'B(4,1) must be positive: more thrust must give upward accel at theta=0.');
end

function test_torque_attitude_response()
    [~,B,~] = linearizeLander(1000, 5000, 1.62);
    assert(B(6,2) > 0, ...
        'B(6,2) must be positive: positive tauRCS must give positive angular accel.');
end

function test_gaussJordan_known_matrix()
    H = [4 1; 1 3];
    rhs = [1; 2];
    xExact = H\rhs; % reference only -- NOT used inside gaussJordanSolve.m
    [x, info] = gaussJordanSolve(H, rhs);
    assert(norm(x - xExact) < 1e-8, 'gaussJordanSolve result does not match known solution.');
    assert(info.residual < 1e-8, 'Residual should be near zero for a well-conditioned system.');
    assert(info.success, 'Should report success for a well-conditioned system.');
end

function test_near_singular_detected()
    H = [1 1; 1 1.0000000001]; % nearly singular
    rhs = [2; 2];
    [~, info] = gaussJordanSolve(H, rhs);
    assert(info.illConditioned || info.singular, ...
        'Near-singular matrix should be flagged as ill-conditioned or singular.');
end

function test_finite_control()
    params = defaultTestParams();
    state  = [5; 1; 80; -2; 0.05; 0.01];
    target = struct('xTarget', 0, 'yTarget', 0);
    [control, prediction, info] = mpcController(state, target, params);
    assert(all(isfinite(control)), 'Control must be finite.');
    assert(all(isfinite(prediction(:))), 'Prediction must be finite.');
    assert(info.success, 'Solver should succeed for a well-posed problem.');
end

function test_thrust_within_Tmax()
    params = defaultTestParams();
    params.Tmax = 5000; % tight, to force saturation
    state  = [0; 0; 500; -40; 0; 0]; % falling fast -> would demand huge thrust
    target = struct('xTarget', 0, 'yTarget', 0);
    control = mpcController(state, target, params);
    assert(control(1) >= -1e-9 && control(1) <= params.Tmax + 1e-9, ...
        'Thrust must respect [0, Tmax].');
end

function test_torque_within_tauMax()
    params = defaultTestParams();
    params.tauMax = 10; % tight
    state  = [0; 0; 100; 0; 1.0; 0]; % large attitude error
    target = struct('xTarget', 0, 'yTarget', 100);
    control = mpcController(state, target, params);
    assert(abs(control(2)) <= params.tauMax + 1e-9, ...
        'Torque must respect |tauRCS| <= tauMax.');
end

function test_prediction_has_Np_points()
    params = defaultTestParams();
    state  = [0;0;100;0;0;0];
    target = struct('xTarget', 0, 'yTarget', 100);
    [~, prediction, ~] = mpcController(state, target, params);
    assert(size(prediction,1) == params.Np, 'Prediction must have Np rows.');
    assert(size(prediction,2) == 6, 'Prediction must have 6 columns (state dimension).');
end

function test_target_influences_control()
    params  = defaultTestParams();
    state   = [0;0;100;0;0;0];
    targetA = struct('xTarget', 0,  'yTarget', 100);
    targetB = struct('xTarget', 50, 'yTarget', 100);
    controlA = mpcController(state, targetA, params);
    controlB = mpcController(state, targetB, params);
    assert(any(abs(controlA - controlB) > 1e-6), ...
        'Different targets should produce different control for the same state.');
end

function test_different_states_diff_control()
    params = defaultTestParams();
    target = struct('xTarget', 0, 'yTarget', 100);
    stateA = [0;0;100;0;0;0];
    stateB = [0;5;100;-10;0.1;0];
    controlA = mpcController(stateA, target, params);
    controlB = mpcController(stateB, target, params);
    assert(any(abs(controlA - controlB) > 1e-6), ...
        'Different states should produce different control for the same target.');
end

function test_solver_residual_small()
    params = defaultTestParams();
    state  = [3; -1; 90; 2; -0.02; 0.01];
    target = struct('xTarget', 0, 'yTarget', 100);
    [~, ~, info] = mpcController(state, target, params);
    assert(info.solverResidual < 1e-6, ...
        'Solver residual should be small for this well-conditioned QP (got %.3e).', info.solverResidual);
end

function test_offnominal_state_runs()
    params = defaultTestParams();
    state  = [100; 20; 300; -30; 0.3; 0.2]; % far from hover / linearization point
    target = struct('xTarget', 0, 'yTarget', 0);
    [control, prediction, info] = mpcController(state, target, params);
    assert(all(isfinite(control)) && all(isfinite(prediction(:))), ...
        'Controller must still return finite outputs far from the linearization point.');
    assert(info.success, 'Solver should still succeed (model is just less accurate there, not broken).');
end

function params = defaultTestParams()
    params.mass = 1000;
    params.I = 5000;
    params.g = 1.62;
    params.dt = 0.04;
    params.Tmax = 20000;
    params.tauMax = 2000;
    params.thetaMax = 0.6;
    params.Np = 20;
    params.Nc = 5;
end
