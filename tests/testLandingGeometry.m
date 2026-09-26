function tests = testLandingGeometry
%TESTLANDINGGEOMETRY Verify the three-leg contact definition.
tests = functiontests(localfunctions);
end

function setup(testCase)
    [~, params] = initializeLander();
    testCase.TestData.params = params;
end

function testThreeFeetExist(testCase)
    geom = getLandingGeometry(testCase.TestData.params);
    testCase.verifyEqual(geom.footCount, 3);
    testCase.verifyEqual(size(geom.feetLocal), [3 2]);
    testCase.verifyEqual(geom.feetLocal(:,2), repmat(geom.footY,3,1));
    testCase.verifyEqual(geom.feetLocal(1,1), -geom.feetLocal(3,1), 'AbsTol', 1e-12);
    testCase.verifyEqual(geom.feetLocal(2,1), 0, 'AbsTol', 1e-12);
end

function testLevelThreeFootTouchdownIsSafe(testCase)
    params = testCase.TestData.params;
    geom = getLandingGeometry(params);
    state = [params.target.x; 0; params.target.y - geom.footY; 0; 0; 0];

    [status, isLanded, isCrash] = checkLandingStatus(state, params);
    testCase.verifyEqual(status, 'safe');
    testCase.verifyTrue(isLanded);
    testCase.verifyFalse(isCrash);
end

function testTiltedTouchdownIsCrash(testCase)
    params = testCase.TestData.params;
    geom = getLandingGeometry(params);
    state = [params.target.x; 0; params.target.y - geom.footY; 0; 0.05; 0];

    [status, isLanded, isCrash] = checkLandingStatus(state, params);
    testCase.verifyEqual(status, 'hard');
    testCase.verifyFalse(isLanded);
    testCase.verifyTrue(isCrash);
end

function testAllFeetMustFitOnPad(testCase)
    params = testCase.TestData.params;
    geom = getLandingGeometry(params);
    state = [params.target.x + 5; 0; params.target.y - geom.footY; 0; 0; 0];

    [status, isLanded, isCrash] = checkLandingStatus(state, params);
    testCase.verifyEqual(status, 'hard');
    testCase.verifyFalse(isLanded);
    testCase.verifyTrue(isCrash);
end

function testContactAtLegTips(testCase)
    params = testCase.TestData.params;
    geom = getLandingGeometry(params);
    state = [params.target.x; 0; params.target.y - geom.footY; 0; 0; 0];
    c = getLandingContact(state, params);
    testCase.verifyTrue(c.contactDetected);
    testCase.verifyTrue(c.simultaneous);
    testCase.verifyTrue(c.safeTouchdown);
    testCase.verifyEqual(c.altitude, 0, 'AbsTol', 1e-12);
end

function testFastLateralTouchdownIsNotSafe(testCase)
    params = testCase.TestData.params;
    geom = getLandingGeometry(params);
    state = [params.target.x; 0.5; params.target.y - geom.footY; -0.2; 0; 0];
    [status, isLanded, isCrash] = checkLandingStatus(state, params);
    testCase.verifyEqual(status, 'hard');
    testCase.verifyFalse(isLanded);
    testCase.verifyTrue(isCrash);
end
