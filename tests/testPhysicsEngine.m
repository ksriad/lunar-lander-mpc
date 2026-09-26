function tests = testPhysicsEngine
%TESTPHYSICSENGINE Unit tests for the lunar-lander physics engine.
%
%   Run with:
%       results = runtests('testPhysicsEngine.m');
%   or, from the tests/ folder:
%       results = runtests('testPhysicsEngine');
%
%   This uses MATLAB's function-based unit test framework (functiontests /
%   matlab.unittest). No toolboxes beyond base MATLAB are required.
tests = functiontests(localfunctions);
end

% ======================================================================
% Fixtures
% ======================================================================

function setupOnce(testCase)
% Add ../physics to the path so the tests can find the functions,
% assuming the standard project layout (physics/, tests/, documentation/).
testFileDir = fileparts(mfilename('fullpath'));
physicsDir  = fullfile(testFileDir, '..', 'physics');
addpath(physicsDir);
testCase.TestData.physicsDir = physicsDir;
end

function teardownOnce(testCase)
rmpath(testCase.TestData.physicsDir);
end

function setup(testCase)
[state0, params0] = initializeLander();
testCase.TestData.state0  = state0;
testCase.TestData.params0 = params0;
end

% ======================================================================
% TEST 1: Zero thrust -> ax = 0, ay = -g_lunar when theta = 0
% ======================================================================
function testZeroThrust(testCase)
state   = testCase.TestData.state0;     % theta = 0
params  = testCase.TestData.params0;
control = [0; 0];

[~, ~, telemetry] = simulateStep(state, control, params);

testCase.verifyEqual(telemetry.ax, 0, 'AbsTol', 1e-9);
testCase.verifyEqual(telemetry.ay, -params.g, 'AbsTol', 1e-9);
end

% ======================================================================
% TEST 2: Thrust = m*g, theta = 0 -> ay ~ 0
% ======================================================================
function testHoverThrust(testCase)
state   = testCase.TestData.state0;
params  = testCase.TestData.params0;
T       = params.mass * params.g;
control = [T; 0];

[~, ~, telemetry] = simulateStep(state, control, params);

testCase.verifyEqual(telemetry.ay, 0, 'AbsTol', 1e-6);
end

% ======================================================================
% TEST 3: Positive theta -> positive horizontal acceleration
% ======================================================================
function testPositiveTheta(testCase)
state       = testCase.TestData.state0;
state(5)    = 0.1;    % theta, rad
params      = testCase.TestData.params0;
control     = [5000; 0];

[~, ~, telemetry] = simulateStep(state, control, params);

testCase.verifyGreaterThan(telemetry.ax, 0);
end

% ======================================================================
% TEST 4: Negative theta -> negative horizontal acceleration
% ======================================================================
function testNegativeTheta(testCase)
state       = testCase.TestData.state0;
state(5)    = -0.1;   % theta, rad
params      = testCase.TestData.params0;
control     = [5000; 0];

[~, ~, telemetry] = simulateStep(state, control, params);

testCase.verifyLessThan(telemetry.ax, 0);
end

% ======================================================================
% TEST 5: Positive torque -> omega changes in the positive direction
% ======================================================================
function testPositiveTorque(testCase)
state   = testCase.TestData.state0;    % omega = 0
params  = testCase.TestData.params0;
control = [0; 50];                     % tauRCS > 0

[stateNext, ~, ~] = simulateStep(state, control, params);

testCase.verifyGreaterThan(stateNext(6), 0);
end

% ======================================================================
% TEST 6: Fuel depletion -> fuel mass never goes negative
% ======================================================================
function testFuelNeverNegative(testCase)
params           = testCase.TestData.params0;
params.fuelMass  = 0.05;                          % almost-empty tank
params.mass      = params.dryMass + params.fuelMass;
state            = testCase.TestData.state0;
control          = [params.Tmax; 0];              % command full thrust

for k = 1:50
    [state, params, telemetry] = simulateStep(state, control, params);
    testCase.verifyGreaterThanOrEqual(telemetry.fuelMass, 0);
    testCase.verifyGreaterThanOrEqual(params.mass, params.dryMass - 1e-9);
end

testCase.verifyEqual(params.fuelMass, 0, 'AbsTol', 1e-9);
end

% ======================================================================
% TEST 7: Thrust saturation -> reported thrust never exceeds Tmax
% ======================================================================
function testThrustSaturation(testCase)
state   = testCase.TestData.state0;
params  = testCase.TestData.params0;
control = [params.Tmax * 5; 0];        % command way more than Tmax

[~, ~, telemetry] = simulateStep(state, control, params);

testCase.verifyLessThanOrEqual(telemetry.thrust, params.Tmax);
testCase.verifyTrue(telemetry.thrustCommandSaturated);
end

% ======================================================================
% TEST 8: Euler and RK2 both produce finite states
% ======================================================================
function testEulerAndRK2Finite(testCase)
state   = testCase.TestData.state0;
control = [5000; 10];

paramsEuler = testCase.TestData.params0;
paramsEuler.integrationMethod = 'Euler';
[stateE, ~, ~] = simulateStep(state, control, paramsEuler);
testCase.verifyTrue(all(isfinite(stateE)));

paramsRK2 = testCase.TestData.params0;
paramsRK2.integrationMethod = 'RK2';
[stateR, ~, ~] = simulateStep(state, control, paramsRK2);
testCase.verifyTrue(all(isfinite(stateR)));
end

% ======================================================================
% TEST 9: Very small timestep behaves numerically reasonably
% ======================================================================
function testSmallTimestep(testCase)
state    = testCase.TestData.state0;
state(5) = 0.05;                 % nonzero theta so ax is nonzero
params   = testCase.TestData.params0;
params.dt = 1e-4;
control  = [5000; 5];

[stateNext, ~, telemetry] = simulateStep(state, control, params);

testCase.verifyTrue(all(isfinite(stateNext)));
testCase.verifyTrue(isfinite(telemetry.ax));
% With such a tiny dt, the state should barely have moved in one step.
testCase.verifyLessThan(abs(stateNext(2) - state(2)), 1);
end

% ======================================================================
% TEST 10: Invalid input produces a clear, identified error
% ======================================================================
function testInvalidInputErrors(testCase)
params   = testCase.TestData.params0;
badState = [0; 0; 100; 0; NaN; 0];

testCase.verifyError(@() simulateStep(badState, [0; 0], params), ...
    'PhysicsEngine:InvalidState');

goodState = testCase.TestData.state0;
badControl = [0; 0; 0];   % wrong size (3 elements instead of 2)

testCase.verifyError(@() simulateStep(goodState, badControl, params), ...
    'PhysicsEngine:InvalidControl');
end
