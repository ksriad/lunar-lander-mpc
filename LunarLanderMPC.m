classdef LunarLanderMPC < matlab.apps.AppBase
    % LUNARLANDERMPC  MPC-Driven Interactive Dual-Mode Lunar Lander Simulator
    % Final integration application (Person 4). Combines:
    %   Person 1 - physics/simulateStep.m
    %   Person 2 - mpc/mpcController.m
    %   Person 3 - numerical/lagrangeInterp.m, numerical/generateTerrain.m,
    %              analysis/analyzeFlight.m
    %   Person 4 - this file (GUI, keyboard architecture, timer loop)
    %
    % This is a plain .m classdef built with the same uifigure/uiXXXXX
    % component API that MATLAB App Designer generates, so it can be run
    % directly as a function/script OR opened in App Designer via
    % File > Open (App Designer will import a programmatically-built
    % UIFigure App and let you keep editing it visually). A binary
    % .mlapp could not be produced in this text-only deliverable, so
    % this .m file IS the App Designer implementation referred to in
    % the project brief; nothing further needs to be pasted in.
    %
    % Run with:   app = LunarLanderMPC;

    % ---------------------------------------------------------------
    % Public UI component properties
    % ---------------------------------------------------------------
    properties (Access = public)
        UIFigure            matlab.ui.Figure
        MainAxes             matlab.ui.control.UIAxes

        ModeDropDownLabel    matlab.ui.control.Label
        ModeDropDown          matlab.ui.control.DropDown

        StartButton           matlab.ui.control.Button
        PauseButton           matlab.ui.control.Button
        ResumeButton          matlab.ui.control.Button
        ResetButton           matlab.ui.control.Button
        StopButton            matlab.ui.control.Button
        AnalysisButton        matlab.ui.control.Button

        TelemetryPanel        matlab.ui.container.Panel
        AltitudeLabel         matlab.ui.control.Label
        VxLabel               matlab.ui.control.Label
        VyLabel               matlab.ui.control.Label
        PitchLabel            matlab.ui.control.Label
        OmegaLabel            matlab.ui.control.Label
        ThrustLabel           matlab.ui.control.Label
        FuelLabel             matlab.ui.control.Label
        FlightTimeLabel       matlab.ui.control.Label
        StatusLabel           matlab.ui.control.Label

        KeyboardPanel         matlab.ui.container.Panel
        KeyboardLamp          matlab.ui.control.Lamp
        KeyboardStatusLabel   matlab.ui.control.Label
        LastKeyLabel          matlab.ui.control.Label
        UpStateLabel          matlab.ui.control.Label
        DownStateLabel        matlab.ui.control.Label
        LeftStateLabel        matlab.ui.control.Label
        RightStateLabel       matlab.ui.control.Label
        CurrentTLabel         matlab.ui.control.Label
        CurrentTauLabel       matlab.ui.control.Label
    end

    % ---------------------------------------------------------------
    % Private application state
    % ---------------------------------------------------------------
    properties (Access = private)
        Params              % shared parameter struct (defaultParams.m)
        State               % [x; vx; y; vy; theta; omega]
        Control             % [T; tauRCS]  (current commanded control)

        Mode = 'Autonomous' % 'Manual' | 'Autonomous'
        FlightActive = false
        FlightPaused = false
        FlightTime  = 0

        FlightTimer         % matlab.timer.Timer

        % --- Section 4/5/9/10: persistent held-key state ---
        KeyState = struct('up', false, 'down', false, 'left', false, 'right', false)

        % --- Manual control tuning ---
        ManualThrustRate = 1800   % N per second while UP held
        Fired = struct('T', 0)    % current manually-held thrust value

        % --- Telemetry log (preallocated growable arrays) ---
        Telemetry

        % --- Graphics handles (created once, updated via XData/YData) ---
        TerrainLine
        PadLine
        LanderPatch
        LanderBodyPatch
        LanderNosePatch
        LanderStripePatch
        LanderLegLines
        LanderFootLines
        TrailLine
        PredictedLine

        InitialState
        InitialParams

        AnalysisFig         % handle to post-flight analysis figure
    end

    methods (Access = public)

        function app = LunarLanderMPC()
    createComponents(app)
    registerApp(app, app.UIFigure)
    startupFcn(app)
end

        function delete(app)
            stopAndDeleteTimer(app)
            delete(app.UIFigure)
        end
    end

    % ===============================================================
    % STARTUP / INITIALIZATION
    % ===============================================================
    methods (Access = private)

        function startupFcn(app)
            if exist('initializeLander', 'file')
                [app.InitialState, app.Params] = initializeLander();
            elseif exist('defaultParams', 'file')
                app.Params = defaultParams();
                app.InitialState = [-40; 3.0; 120; -1.0; 0.05; 0];
            end
            % Ensure target fields exist
            if ~isfield(app.Params, 'target')
                app.Params.target.x = 0;
                app.Params.target.y = 0;
                app.Params.target.halfWidth = 7;
                app.Params.target.centerHeight = -5.3;
            end
            if ~isfield(app.Params, 'landerBody')
                app.Params.landerBody = [-2.9 -2.8; 2.9 -2.8; 2.9 2.8; -2.9 2.8];
            end
            if ~isfield(app.Params, 'landerGeometry')
                app.Params.landerGeometry = struct( ...
                    'bodyWidth', 5.8, 'bodyBottom', -2.8, 'bodyTop', 2.8, ...
                    'noseHeight', 2.0, 'legAttachY', -2.35, ...
                    'legAttachX', [-2.0 0 2.0], 'footX', [-3.4 0 3.4], ...
                    'footY', -5.3, 'footPadHalfWidth', 0.75, ...
                    'legThickness', 7.0, 'footThickness', 8.0);
            end
            app.Params.target.centerHeight = -app.Params.landerGeometry.footY;
            app.InitialParams = app.Params;
            app.State   = app.InitialState;
            app.Control = [0; 0];

            initTelemetry(app)
            drawStaticScene(app)
            drawDynamicScene(app)
            updateTelemetryLabels(app)
            updateKeyboardDiagnostics(app)

            app.StatusLabel.Text = 'READY';
            app.KeyboardStatusLabel.Text = 'KEYBOARD CONTROL: INACTIVE';
            app.KeyboardLamp.Color = [0.7 0.7 0.7];
        end

        function initTelemetry(app)
            app.Telemetry = struct('time', [], 'x', [], 'vx', [], 'y', [], ...
                'vy', [], 'theta', [], 'omega', [], 'T', [], 'tauRCS', [], ...
                'ax', [], 'ay', [], 'mass', [], 'fuelMass', []);
        end
        function logInitialTelemetry(app)

    app.Telemetry.time(1)     = 0;
    app.Telemetry.x(1)        = app.State(1);
    app.Telemetry.vx(1)       = app.State(2);
    app.Telemetry.y(1)        = app.State(3);
    app.Telemetry.vy(1)       = app.State(4);
    app.Telemetry.theta(1)    = app.State(5);
    app.Telemetry.omega(1)    = app.State(6);

    app.Telemetry.T(1)        = 0;
    app.Telemetry.tauRCS(1)   = 0;

    dstate = landerDynamics( ...
        app.State, [0;0], app.Params.mass, app.Params);

    app.Telemetry.ax(1)       = dstate(2);
    app.Telemetry.ay(1)       = dstate(4);

    app.Telemetry.mass(1)     = app.Params.mass;
    app.Telemetry.fuelMass(1) = app.Params.fuelMass;
end
    end

    % ===============================================================
    % UI CONSTRUCTION
    % ===============================================================
    methods (Access = private)

        function createComponents(app)
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [80 60 1180 720];
            app.UIFigure.Name = 'MPC-Driven Interactive Dual-Mode Lunar Lander Simulator';
            app.UIFigure.CloseRequestFcn = @(src,evt) uiFigureCloseRequest(app, src, evt);

            % --- Section 4: UIFigure owns the keyboard callbacks ---
            app.UIFigure.WindowKeyPressFcn   = @(src,evt) windowKeyPressed(app, evt);
            app.UIFigure.WindowKeyReleaseFcn = @(src,evt) windowKeyReleased(app, evt);

            % --- Main simulation axes (left, wide) ---
            app.MainAxes = uiaxes(app.UIFigure);
            app.MainAxes.Position = [20 20 780 680];
            app.MainAxes.XLabel.String = 'x  [m]';
            app.MainAxes.YLabel.String = 'y  [m]';
            app.MainAxes.Title.String = 'Lunar Descent';
app.MainAxes.Title.Color = [0.9 0.9 0.95];
            axis(app.MainAxes, 'equal');
            xlim(app.MainAxes, [-140 140]);
            ylim(app.MainAxes, [-10 160]);
            hold(app.MainAxes, 'on');
grid(app.MainAxes, 'off');

app.MainAxes.Color = [0.025 0.03 0.06];
app.MainAxes.XColor = [0.85 0.85 0.9];
app.MainAxes.YColor = [0.85 0.85 0.9];

app.MainAxes.GridColor = [0.3 0.3 0.35];
app.MainAxes.MinorGridColor = [0.2 0.2 0.25];

            xRight = 820;
            panelWidth = 340;

            % --- Mode selector + flight buttons ---
            app.ModeDropDownLabel = uilabel(app.UIFigure);
            app.ModeDropDownLabel.Position = [xRight 675 100 22];
            app.ModeDropDownLabel.Text = 'Flight Mode';

            app.ModeDropDown = uidropdown(app.UIFigure);
            app.ModeDropDown.Items = {'Manual / Interactive Pilot', 'Autonomous MPC'};
            app.ModeDropDown.Value = 'Autonomous MPC';
            app.ModeDropDown.Position = [xRight+100 675 panelWidth-100 22];
            app.ModeDropDown.ValueChangedFcn = @(src,evt) modeDropDownChanged(app, evt);

            btnY = 635; btnH = 30; btnW = 105; gap = 8;
            app.StartButton = uibutton(app.UIFigure, 'push');
            app.StartButton.Position = [xRight btnY btnW btnH];
            app.StartButton.Text = 'Start';
            app.StartButton.BackgroundColor = [0.72 0.90 0.72];
            app.StartButton.ButtonPushedFcn = @(src,evt) startButtonPushed(app);

            app.PauseButton = uibutton(app.UIFigure, 'push');
            app.PauseButton.Position = [xRight+(btnW+gap) btnY btnW btnH];
            app.PauseButton.Text = 'Pause';
            app.PauseButton.ButtonPushedFcn = @(src,evt) pauseButtonPushed(app);

            app.ResumeButton = uibutton(app.UIFigure, 'push');
            app.ResumeButton.Position = [xRight+2*(btnW+gap) btnY btnW btnH];
            app.ResumeButton.Text = 'Resume';
            app.ResumeButton.ButtonPushedFcn = @(src,evt) resumeButtonPushed(app);

            btnY2 = 600;
            app.ResetButton = uibutton(app.UIFigure, 'push');
            app.ResetButton.Position = [xRight btnY2 btnW btnH];
            app.ResetButton.Text = 'Reset';
            app.ResetButton.ButtonPushedFcn = @(src,evt) resetButtonPushed(app);

            app.StopButton = uibutton(app.UIFigure, 'push');
            app.StopButton.Position = [xRight+(btnW+gap) btnY2 btnW btnH];
            app.StopButton.Text = 'Stop / Abort';
            app.StopButton.BackgroundColor = [0.95 0.75 0.75];
            app.StopButton.ButtonPushedFcn = @(src,evt) stopButtonPushed(app);

            app.AnalysisButton = uibutton(app.UIFigure, 'push');
            app.AnalysisButton.Position = [xRight+2*(btnW+gap) btnY2 btnW btnH];
            app.AnalysisButton.Text = 'Analysis';
            app.AnalysisButton.ButtonPushedFcn = @(src,evt) analysisButtonPushed(app);

            app.StatusLabel = uilabel(app.UIFigure);
            app.StatusLabel.Position = [xRight 565 panelWidth 26];
            app.StatusLabel.Text = 'READY';
            app.StatusLabel.FontWeight = 'bold';
            app.StatusLabel.FontSize = 14;
            app.StatusLabel.HorizontalAlignment = 'center';

            % --- Telemetry panel ---
            app.TelemetryPanel = uipanel(app.UIFigure);
            app.TelemetryPanel.Title = 'Telemetry';
            app.TelemetryPanel.Position = [xRight 355 panelWidth 200];

            labels = {'AltitudeLabel','Altitude:  -- m'; ...
                      'VxLabel','Horiz. Vel:  -- m/s'; ...
                      'VyLabel','Vert. Vel:  -- m/s'; ...
                      'PitchLabel','Pitch:  -- rad'; ...
                      'OmegaLabel','Ang. Vel:  -- rad/s'; ...
                      'ThrustLabel','Thrust:  -- N'; ...
                      'FuelLabel','Fuel:  -- kg'; ...
                      'FlightTimeLabel','Flight Time:  -- s'};
            for k = 1:size(labels,1)
                lbl = uilabel(app.TelemetryPanel);
                lbl.Position = [10 168-22*(k-1) 310 20];
                lbl.Text = labels{k,2};
                app.(labels{k,1}) = lbl;
            end

            % --- Keyboard diagnostic panel (Section 12) ---
            app.KeyboardPanel = uipanel(app.UIFigure);
            app.KeyboardPanel.Title = 'Manual Keyboard Diagnostics (Mac-safe)';
            app.KeyboardPanel.Position = [xRight 60 panelWidth 290];

            app.KeyboardLamp = uilamp(app.KeyboardPanel);
            app.KeyboardLamp.Position = [10 250 20 20];
            app.KeyboardLamp.Color = [0.7 0.7 0.7];

            app.KeyboardStatusLabel = uilabel(app.KeyboardPanel);
            app.KeyboardStatusLabel.Position = [38 248 290 22];
            app.KeyboardStatusLabel.Text = 'KEYBOARD CONTROL: INACTIVE';
            app.KeyboardStatusLabel.FontWeight = 'bold';

            app.LastKeyLabel = uilabel(app.KeyboardPanel);
            app.LastKeyLabel.Position = [10 220 300 20];
            app.LastKeyLabel.Text = 'Last Key: (none)';

            app.UpStateLabel = uilabel(app.KeyboardPanel);
            app.UpStateLabel.Position = [10 195 300 20];
            app.UpStateLabel.Text = 'UP:    RELEASED';

            app.DownStateLabel = uilabel(app.KeyboardPanel);
            app.DownStateLabel.Position = [10 172 300 20];
            app.DownStateLabel.Text = 'DOWN:  RELEASED';

            app.LeftStateLabel = uilabel(app.KeyboardPanel);
            app.LeftStateLabel.Position = [10 149 300 20];
            app.LeftStateLabel.Text = 'LEFT:  RELEASED';

            app.RightStateLabel = uilabel(app.KeyboardPanel);
            app.RightStateLabel.Position = [10 126 300 20];
            app.RightStateLabel.Text = 'RIGHT: RELEASED';

            app.CurrentTLabel = uilabel(app.KeyboardPanel);
            app.CurrentTLabel.Position = [10 92 300 20];
            app.CurrentTLabel.Text = 'Current T:      0.0 N';

            app.CurrentTauLabel = uilabel(app.KeyboardPanel);
            app.CurrentTauLabel.Position = [10 68 300 20];
            app.CurrentTauLabel.Text = 'Current tauRCS: 0.0 N*m';

            note = uilabel(app.KeyboardPanel);
            note.Position = [10 8 320 48];
            note.WordWrap = 'on';
            note.FontColor = [0.4 0.4 0.4];
            note.Text = ['Arrow keys control the lander only while a flight ' ...
                'is active in Manual mode. Keys are read by the UIFigure ' ...
                'itself, so clicking buttons/dropdowns will not break control.'];

            app.UIFigure.Visible = 'on';
        end
    end

    % ===============================================================
    % KEYBOARD ARCHITECTURE (Sections 4-10) - lightweight callbacks only
    % ===============================================================
    methods (Access = private)

        function windowKeyPressed(app, evt)
            key = evt.Key;
            app.LastKeyLabel.Text = ['Last Key: ' key];

            switch key
                case 'uparrow'
                    app.KeyState.up = true;
                case 'downarrow'
                    app.KeyState.down = true;
                case 'leftarrow'
                    app.KeyState.left = true;
                case 'rightarrow'
                    app.KeyState.right = true;
                otherwise
                    % ignore all non-arrow keys; do nothing else here.
            end
            updateKeyboardDiagnostics(app)
            % NOTE: no physics, no MPC calls here (Section 14).
        end

        function windowKeyReleased(app, evt)
            key = evt.Key;
            switch key
                case 'uparrow'
                    app.KeyState.up = false;
                case 'downarrow'
                    app.KeyState.down = false;
                case 'leftarrow'
                    app.KeyState.left = false;
                case 'rightarrow'
                    app.KeyState.right = false;
                otherwise
            end
            updateKeyboardDiagnostics(app)
        end

        function clearKeyState(app)
            app.KeyState.up    = false;
            app.KeyState.down  = false;
            app.KeyState.left  = false;
            app.KeyState.right = false;
            updateKeyboardDiagnostics(app)
        end

        function updateKeyboardDiagnostics(app)
            app.UpStateLabel.Text    = sprintf('UP:    %s', tern(app.KeyState.up,    'PRESSED', 'RELEASED'));
            app.DownStateLabel.Text  = sprintf('DOWN:  %s', tern(app.KeyState.down,  'PRESSED', 'RELEASED'));
            app.LeftStateLabel.Text  = sprintf('LEFT:  %s', tern(app.KeyState.left,  'PRESSED', 'RELEASED'));
            app.RightStateLabel.Text = sprintf('RIGHT: %s', tern(app.KeyState.right, 'PRESSED', 'RELEASED'));
        end

        function control = getManualControl(app)
            % Section 11: dedicated manual-control command generation.
            dt = app.Params.dt;

            T = app.Fired.T;
            if app.KeyState.up
                T = T + app.ManualThrustRate * dt;
            end
            if app.KeyState.down
                T = T - app.ManualThrustRate * dt;
            end
            T = max(0, min(app.Params.Tmax, T));
            app.Fired.T = T;

            if app.KeyState.left && ~app.KeyState.right
                tauRCS = -app.Params.tauMax;
            elseif app.KeyState.right && ~app.KeyState.left
                tauRCS = app.Params.tauMax;
            else
                tauRCS = 0;
            end

            if app.Params.fuelMass <= 0
                T = 0;
            end

            control = [T; tauRCS];
        end
    end

    % ===============================================================
    % FLIGHT CONTROL BUTTONS
    % ===============================================================
    methods (Access = private)

        function modeDropDownChanged(app, evt)
            if strcmp(evt.Value, 'Manual / Interactive Pilot')
                app.Mode = 'Manual';
            else
                app.Mode = 'Autonomous';
            end
        end

        function startButtonPushed(app)
            if app.FlightActive
                return
            end

            app.State  = app.InitialState;
            app.Params = app.InitialParams;
            app.Control = [0;0];
            app.Fired.T = 0;
            app.FlightTime = 0;
            initTelemetry(app)
logInitialTelemetry(app)
clearKeyState(app)

            app.FlightActive = true;
            app.FlightPaused = false;
            app.StatusLabel.Text = 'IN FLIGHT';

            if strcmp(app.Mode, 'Manual')
                app.KeyboardStatusLabel.Text = 'KEYBOARD CONTROL: ACTIVE';
                app.KeyboardLamp.Color = [0.2 0.8 0.2];
            else
                app.KeyboardStatusLabel.Text = 'KEYBOARD CONTROL: INACTIVE (Autonomous)';
                app.KeyboardLamp.Color = [0.7 0.7 0.7];
            end

            startFlightTimer(app)
            focus(app.UIFigure) % ensure the UIFigure itself owns key focus
        end

        function pauseButtonPushed(app)
            if ~app.FlightActive || app.FlightPaused
                return
            end
            if ~isempty(app.FlightTimer) && isvalid(app.FlightTimer)
                stop(app.FlightTimer)
            end
            clearKeyState(app)
            app.FlightPaused = true;
            app.StatusLabel.Text = 'PAUSED';
            app.KeyboardStatusLabel.Text = 'KEYBOARD CONTROL: INACTIVE (Paused)';
            app.KeyboardLamp.Color = [0.9 0.6 0.1];
        end

        function resumeButtonPushed(app)
            if ~app.FlightActive || ~app.FlightPaused
                return
            end
            clearKeyState(app)
            app.FlightPaused = false;
            app.StatusLabel.Text = 'IN FLIGHT';
            if strcmp(app.Mode, 'Manual')
                app.KeyboardStatusLabel.Text = 'KEYBOARD CONTROL: ACTIVE';
                app.KeyboardLamp.Color = [0.2 0.8 0.2];
            end
            if isempty(app.FlightTimer) || ~isvalid(app.FlightTimer)
                startFlightTimer(app)
            else
                start(app.FlightTimer)
            end
            focus(app.UIFigure)
        end

        function resetButtonPushed(app)
            stopAndDeleteTimer(app)
            clearKeyState(app)

            app.FlightActive = false;
            app.FlightPaused = false;
            app.FlightTime = 0;
            app.Control = [0;0];
            app.Fired.T = 0;
            app.State  = app.InitialState;
            app.Params = app.InitialParams;
            initTelemetry(app)

            drawDynamicScene(app)
            updateTelemetryLabels(app)

            app.StatusLabel.Text = 'READY';
            app.KeyboardStatusLabel.Text = 'KEYBOARD CONTROL: INACTIVE';
            app.KeyboardLamp.Color = [0.7 0.7 0.7];
        end

        function stopButtonPushed(app)
            if ~app.FlightActive
                return
            end
            stopAndDeleteTimer(app)
            clearKeyState(app)
            app.FlightActive = false;
            app.FlightPaused = false;
            app.StatusLabel.Text = 'ABORTED';
            app.KeyboardStatusLabel.Text = 'KEYBOARD CONTROL: INACTIVE';
            app.KeyboardLamp.Color = [0.7 0.7 0.7];
        end

        function analysisButtonPushed(app)
            if isempty(app.Telemetry.time)
                uialert(app.UIFigure, 'No flight data yet - run a flight first.', 'No Data');
                return
            end
            results = analyzeFlight(app.Telemetry, app.Params);
            showAnalysisWindow(app, results)
        end
    end

    % ===============================================================
    % TIMER ARCHITECTURE (Sections 13-14)
    % ===============================================================
    methods (Access = private)

        function startFlightTimer(app)
            stopAndDeleteTimer(app)
            app.FlightTimer = timer( ...
                'ExecutionMode', 'fixedRate', ...
                'Period', max(0.02, app.Params.dt), ...
                'TimerFcn', @(~,~) timerCallback(app), ...
                'ErrorFcn', @(~,~) []);
            start(app.FlightTimer)
        end

        function stopAndDeleteTimer(app)
            if ~isempty(app.FlightTimer) && isvalid(app.FlightTimer)
                stop(app.FlightTimer)
                delete(app.FlightTimer)
            end
            app.FlightTimer = [];
        end
        function target = makeMPCTarget(app)
            s = app.State;
            p = app.Params;
            geom = getLandingGeometry(p);

            x = s(1);
            vx = s(2);
            xPad = p.target.x;
            yPad = p.target.y;

            % Target the COM height that places the bottom leg tips on the pad.
            yTarget = yPad - geom.footY;

            % Measure altitude from the actual lowest foot.
            R = [cos(s(5)) -sin(s(5)); sin(s(5)) cos(s(5))];
            feetWorld = (R * geom.feetLocal')';
            feetWorld(:,1) = feetWorld(:,1) + x;
            feetWorld(:,2) = feetWorld(:,2) + s(3);
            altitude = max(min(feetWorld(:,2)) - yPad, 0);

            distance = abs(xPad - x);
            aBrake = 1.8;

            if distance > 0.25
                desiredVx = sign(xPad-x)*min(sqrt(2*aBrake*distance), 2.5);
            else
                desiredVx = 0;
            end

            % Do not carry a large lateral velocity into the terminal phase.
            if altitude < 35, desiredVx = max(min(desiredVx, 1.5), -1.5); end
            if altitude < 20, desiredVx = max(min(desiredVx, 0.75), -0.75); end
            if altitude < 10, desiredVx = max(min(desiredVx, 0.30), -0.30); end

            if altitude > 50
                desiredVy = -7.0;
            elseif altitude > 30
                desiredVy = -4.0;
            elseif altitude > 15
                desiredVy = -2.2;
            elseif altitude > 8
                desiredVy = -1.2;
            elseif altitude > 4
                desiredVy = -0.65;
            else
                desiredVy = -0.30;
            end

            target = struct( ...
                'xTarget',     xPad, ...
                'yTarget',     yTarget, ...
                'vxTarget',    desiredVx, ...
                'vyTarget',    desiredVy, ...
                'thetaTarget', 0, ...
                'omegaTarget', 0);
        end

function control = applyTerminalLandingGuidance(app, state, mpcControl, params)
%APPLYTERMINALLANDINGGUIDANCE Tight terminal guidance for centered, upright touchdown.
%   Below the terminal altitude this controller brakes lateral velocity early,
%   drives x to the pad centre, then removes the temporary pitch needed for
%   braking before the three leg pads reach the surface.

    %#ok<INUSD>
    state = state(:);
    x = state(1); vx = state(2); y = state(3); vy = state(4);
    theta = state(5); omega = state(6);

    m = params.mass; g = params.g; I = params.I;
    Tmax = params.Tmax; tauMax = params.tauMax;
    xPad = params.target.x; yPad = params.target.y;
    geom = getLandingGeometry(params);

    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    feetWorld = (R * geom.feetLocal')';
    feetWorld(:,1) = feetWorld(:,1) + x;
    feetWorld(:,2) = feetWorld(:,2) + y;
    altitude = min(feetWorld(:,2)) - yPad;

    % Above 45 m the original MPC remains in charge.
    if altitude > 45
        control = mpcControl;
        return
    end

    xError = xPad - x;

    % Horizontal position plus velocity braking.  The derivative term is
    % intentionally strong so vx is near zero before the final descent.
    if altitude > 30
        KpX = 0.22; KdX = 1.65; aMax = 2.40;
    elseif altitude > 20
        KpX = 0.30; KdX = 2.00; aMax = 2.20;
    elseif altitude > 12
        KpX = 0.42; KdX = 2.35; aMax = 1.90;
    elseif altitude > 8
        KpX = 0.55; KdX = 2.60; aMax = 1.55;
    elseif altitude > 5
        KpX = 0.70; KdX = 2.80; aMax = 1.15;
    elseif altitude > 2.5
        KpX = 0.85; KdX = 3.00; aMax = 0.70;
    else
        KpX = 0.95; KdX = 3.20; aMax = 0.35;
    end

    axCommand = KpX*xError - KdX*vx;
    if abs(xError) < 0.20 && abs(vx) < 0.12
        axCommand = -1.5*vx;
    end
    axCommand = min(max(axCommand, -aMax), aMax);

    % Horizontal braking is produced by a temporary pitch.  The allowed
    % pitch is reduced to zero before touchdown so all three feet share the
    % same contact height.
    thetaForBraking = asin(min(max(axCommand/max(g,eps), -0.22), 0.22));
    if altitude > 15
        thetaCommand = thetaForBraking;
    elseif altitude > 9
        thetaCommand = 0.75*thetaForBraking;
    elseif altitude > 5
        thetaCommand = 0.45*thetaForBraking;
    elseif altitude > 2.5
        thetaCommand = 0.20*thetaForBraking;
    else
        % Below 2.5 m, allow only the small amount of tilt required to
        % brake residual vx.  Once vx is settled, the attitude command
        % automatically returns to exactly zero before contact.
        thetaCommand = 0.35*thetaForBraking;
        if abs(vx) < 0.08 && abs(xError) < 0.20
            thetaCommand = 0;
        end
    end
    thetaCommand = min(max(thetaCommand, -0.10), 0.10);

    if altitude < 10
        Ktheta = 11.0; Komega = 6.5;
    else
        Ktheta = 8.0; Komega = 5.0;
    end
    tau = I*(Ktheta*(thetaCommand-theta) - Komega*omega);
    tau = min(max(tau, -tauMax), tauMax);

    % Vertical descent is slowed whenever lateral or attitude settling is
    % incomplete.  This is the key anti-drift gate near the pad.
    if altitude > 30
        desiredVy = -2.8;
    elseif altitude > 20
        desiredVy = -2.0;
    elseif altitude > 12
        desiredVy = -1.35;
    elseif altitude > 8
        desiredVy = -0.90;
    elseif altitude > 5
        desiredVy = -0.55;
    elseif altitude > 2.5
        desiredVy = -0.32;
    else
        desiredVy = -0.18;
    end

    if altitude < 12 && (abs(vx) > 0.30 || abs(theta) > 0.035 || abs(omega) > 0.10)
        desiredVy = max(desiredVy, -0.45);
    end
    if altitude < 5 && (abs(vx) > 0.20 || abs(theta) > 0.020 || abs(omega) > 0.08)
        desiredVy = max(desiredVy, -0.22);
    end

    vyError = desiredVy - vy;
    ayCommand = min(max(4.5*vyError, -1.0), 7.0);
    cosTheta = max(cos(theta), 0.92);
    T = min(max(m*(g + ayCommand)/cosTheta, 0), Tmax);

    % Last 2.5 m: do not allow touchdown while lateral motion or attitude
    % is outside the safe envelope.  A small hover margin buys the controller
    % time to settle instead of accepting an off-centre/tilted landing.
    if altitude < 2.5 && (abs(vx) > 0.20 || abs(theta) > 0.020 || abs(omega) > 0.08)
        T = max(T, m*g + m*0.35);
    end

    control = [T; tau];
end

function updatePrediction(app, prediction)
    if isempty(prediction)
        app.PredictedLine.XData = [];
        app.PredictedLine.YData = [];
    elseif isstruct(prediction) && isfield(prediction, 'X')
        app.PredictedLine.XData = prediction.X;
        app.PredictedLine.YData = prediction.Y;
    elseif isnumeric(prediction)
        % prediction matrix: col 1 is x, col 3 is y
        app.PredictedLine.XData = prediction(:, 1);
        app.PredictedLine.YData = prediction(:, 3);
    end
end
       function timerCallback(app)
    if ~app.FlightActive || app.FlightPaused
        return
    end

    % 0. Resolve any touchdown that was already reached at the end of the
    % previous timer tick before asking the controller for another command.
    % This prevents a vehicle resting on its feet from remaining IN FLIGHT.
    [preStatus, preLanded, preCrash] = checkLandingStatus(app.State, app.Params);
    if ~strcmp(preStatus, 'flying')
        if preLanded || preCrash
            checkTermination(app);
        end
        return
    end

    % 1. Get control
    if strcmp(app.Mode, 'Manual')
        control = getManualControl(app);
        updatePrediction(app, []);
    else
    target = makeMPCTarget(app);

    % Main autonomous MPC controller
    [control, prediction, ~] = ...
        mpcController(app.State, target, app.Params);

    % Terminal landing guidance
    % Below 30 m, this stabilizes horizontal position,
    % horizontal velocity, vertical velocity and attitude.
    control = applyTerminalLandingGuidance( ...
        app, app.State, control, app.Params);

    updatePrediction(app, prediction);
end

    % 2. Advance REAL physics
    [stateNext, paramsNext, telemetry] = simulateStep(app.State, control, app.Params);

    % 3. Update state & controls
    app.State   = stateNext;
    app.Params  = paramsNext;
    app.Control = control;

    % The physics engine marks a safe three-foot touchdown explicitly and
    % cuts the engine at that instant. Keep the GUI command state consistent.
    if isfield(telemetry, 'touchdown') && telemetry.touchdown
        app.Control = [0; 0];
    end

    app.FlightTime = app.FlightTime + app.Params.dt;

    % 4. Log telemetry
    logTelemetry(app, telemetry);

    % 5. Draw dynamic objects
    drawDynamicScene(app);

    % 6. Update GUI telemetry labels & diagnostics
    updateTelemetryLabels(app);
    updateKeyboardDiagnostics(app);
    app.CurrentTLabel.Text = ...
    sprintf('Current T:      %.1f N', telemetry.thrust);

app.CurrentTauLabel.Text = ...
    sprintf('Current tauRCS: %.1f N*m', telemetry.torque);

    % 7. Check landing/crash termination
    checkTermination(app);

    % 8. Refresh UI
    drawnow limitrate;
end

        function logTelemetry(app, telemetry)
    t = app.Telemetry;
    t.time(end+1)     = app.FlightTime;
    t.x(end+1)        = app.State(1);
    t.vx(end+1)       = app.State(2);
    t.y(end+1)        = app.State(3);
    t.vy(end+1)       = app.State(4);
    t.theta(end+1)    = app.State(5);
    t.omega(end+1)    = app.State(6);

    % Log ACTUAL applied forces and accelerations from Person 1
    if isstruct(telemetry)
        t.T(end+1)      = telemetry.thrust;
        t.tauRCS(end+1) = telemetry.torque;
        t.ax(end+1)     = telemetry.ax;
        t.ay(end+1)     = telemetry.ay;
    else
        t.T(end+1)      = app.Control(1);
        t.tauRCS(end+1) = app.Control(2);
        t.ax(end+1)     = 0;
        t.ay(end+1)     = 0;
    end

    t.mass(end+1)     = app.Params.mass;
    t.fuelMass(end+1) = app.Params.fuelMass;
    app.Telemetry = t;
end
    end

    % ===============================================================
    % LANDING / CRASH DETECTION (Section 20)
    % ===============================================================
    methods (Access = private)

        function checkTermination(app)

    [status, isLanded, isCrash] = ...
        checkLandingStatus(app.State, app.Params);

    if strcmp(status, 'flying')
        return
    end

    stopAndDeleteTimer(app);
    clearKeyState(app);

    app.FlightActive = false;
    app.FlightPaused = false;

    app.KeyboardStatusLabel.Text = 'KEYBOARD CONTROL: INACTIVE';
    app.KeyboardLamp.Color = [0.7 0.7 0.7];

    if isLanded
        app.StatusLabel.Text = 'LANDED / TOUCHDOWN';
        app.StatusLabel.FontColor = [0.1 0.6 0.1];
    elseif isCrash
        app.StatusLabel.Text = 'CRASH / HARD LANDING';
        app.StatusLabel.FontColor = [0.8 0.1 0.1];
    end
end
    end

    % ===============================================================
    % GRAPHICS (Sections 19, 26)
    % ===============================================================
    methods (Access = private)

        function drawStaticScene(app)
            % --- Space background ---
rng(12);

starX = -140 + 280*rand(1,80);
starY = 15 + 140*rand(1,80);

scatter(app.MainAxes, starX, starY, 8, ...
    'MarkerFaceColor', [0.85 0.85 0.9], ...
    'MarkerEdgeColor', 'none');

% --- Decorative moon ---
moonX = 105;
moonY = 125;

scatter(app.MainAxes, moonX, moonY, 2600, ...
    'MarkerFaceColor', [0.55 0.55 0.58], ...
    'MarkerEdgeColor', [0.75 0.75 0.78], ...
    'LineWidth', 1.2);

% Moon craters
scatter(app.MainAxes, ...
    [96 113 108 118], ...
    [131 119 138 132], ...
    [120 90 70 100], ...
    'MarkerFaceColor', [0.42 0.42 0.45], ...
    'MarkerEdgeColor', 'none');
% ===============================================================
% --- DECORATIVE LUNAR GROUND ---
% Visual only. Does NOT affect flight physics or landing logic.
% ===============================================================

padX = app.Params.target.x;
padY = app.Params.target.y;

% Keep decorative ground safely below the landing pad.
clearance = 3.0;
floorBottom = -10;

% Smooth horizontal ground profile.
groundX = linspace(-140, 140, 400);

% Natural rolling baseline.
groundY = (padY - 7.0) ...
    + 0.65*sin(groundX/9.0) ...
    + 0.30*sin(groundX/4.5) ...
    + 0.12*sin(groundX/2.2);

% ===============================================================
% Crater depressions
%
% NO crater centers inside [-30, 30].
% This leaves a clean final approach corridor.
% ===============================================================

craterCenters = [-125 -103 -82 -61 -42 42 61 83 106 128];

craterDepths = [ ...
    1.8 2.4 1.2 2.0 1.4 ...
    1.5 2.1 1.3 2.5 1.6];

craterWidths = [ ...
    9 11 7 10 8 ...
    8 10 7 11 8];

for ci = 1:numel(craterCenters)

    craterShape = exp( ...
        -((groundX - craterCenters(ci)) ...
        ./ craterWidths(ci)).^2);

    groundY = groundY - ...
        craterDepths(ci).*craterShape;

end

% ===============================================================
% Smooth landing-area bowl
%
% Broad Gaussian window centered at the landing pad.
% This prevents sharp terrain walls near the pad.
% ===============================================================

approachWidth = 32;

approachWindow = exp( ...
    -((groundX - padX)./approachWidth).^4);

localGroundLevel = padY - clearance - 1.5;

groundY = ...
    groundY.*(1 - approachWindow) ...
    + localGroundLevel.*approachWindow;

% ===============================================================
% Guarantee clearance below landing pad
% ===============================================================

maximumGroundY = padY - clearance;

groundY = min(groundY, maximumGroundY);

% ===============================================================
% Additional smooth flattening immediately around pad
% ===============================================================

smoothWidth = 12;

padSmooth = exp( ...
    -((groundX - padX)./smoothWidth).^6);

smoothTarget = padY - clearance - 0.8;

groundY = ...
    groundY.*(1 - padSmooth) ...
    + smoothTarget.*padSmooth;

% Final clearance check.
groundY = min(groundY, maximumGroundY);

% ===============================================================
% Ground polygon
%
% IMPORTANT:
% First row = irregular upper surface.
% Second row = bottom edge.
% This keeps the polygon closed correctly.
% ===============================================================

groundPolyX = [ ...
    groundX, ...
    fliplr(groundX)];

groundPolyY = [ ...
    groundY, ...
    floorBottom*ones(size(groundX))];

patch(app.MainAxes, ...
    groundPolyX, ...
    groundPolyY, ...
    [0.20 0.20 0.23], ...
    'EdgeColor', [0.34 0.34 0.37], ...
    'LineWidth', 1.0);

% ===============================================================
% Lunar surface rim
% ===============================================================

plot(app.MainAxes, ...
    groundX, ...
    groundY, ...
    'Color', [0.43 0.43 0.46], ...
    'LineWidth', 1.5);

% ===============================================================
% Decorative crater rings
% ===============================================================

ringCenters = [-122 -99 -76 -54 -39 39 55 78 101 123];

ringRadii = [5 4 3.5 4.5 3 3.5 4.5 3.5 5 3.5];

for ci = 1:numel(ringCenters)

    cx = ringCenters(ci);

    cy = interp1( ...
        groundX, ...
        groundY, ...
        cx);

    theta = linspace(0, 2*pi, 50);

    rx = ringRadii(ci);
    ry = rx*0.28;

    plot(app.MainAxes, ...
        cx + rx*cos(theta), ...
        cy + ry*sin(theta), ...
        'Color', [0.30 0.30 0.33], ...
        'LineWidth', 0.9);

end

% ===============================================================
% Small lunar rocks
% ===============================================================

rockX = [-130 -113 -91 -69 -51 -40 ...
          43 58 76 94 116 133];

rockH = [ ...
    0.35 0.22 0.30 0.42 0.25 0.32 ...
    0.30 0.20 0.38 0.25 0.35 0.22];

for ri = 1:numel(rockX)

    rx = rockX(ri);

    ry = interp1( ...
        groundX, ...
        groundY, ...
        rx);

    rockWidth = 0.6 + 0.2*mod(ri,3);

    patch(app.MainAxes, ...
        [rx-rockWidth rx rx+rockWidth], ...
        [ry ry+rockH(ri) ry], ...
        [0.27 0.27 0.30], ...
        'EdgeColor', 'none');

end

% ===============================================================
% Preserve existing terrain handle.
% Old mountain terrain is intentionally not drawn.
% ===============================================================

app.TerrainLine = plot(app.MainAxes, ...
    NaN, NaN, ...
    'Visible', 'off');

            padX = app.Params.target.x;
            padY = app.Params.target.y;
            padHW = app.Params.target.halfWidth;
            app.PadLine = plot(app.MainAxes, [padX-padHW padX+padHW], [padY padY], ...
                'Color', [0.1 0.5 0.9], 'LineWidth', 4);

app.TrailLine = plot(app.MainAxes, NaN, NaN, ...
    'Color', [0.55 0.75 1.0], ...
    'LineStyle', '-', ...
    'LineWidth', 1.5);
app.PredictedLine = plot(app.MainAxes, NaN, NaN, 'Color', [0.9 0.3 0.1], 'LineStyle', '--', 'LineWidth', 1.2);

            % --- Polished lander visual model (2-D flight-plane view) ---
            geom = getLandingGeometry(app.Params);
            app.LanderBodyPatch = patch(app.MainAxes, ...
                'XData', NaN, 'YData', NaN, ...
                'FaceColor', [0.94 0.94 0.96], ...
                'EdgeColor', [0.18 0.18 0.20], 'LineWidth', 1.8);

            app.LanderNosePatch = patch(app.MainAxes, ...
                'XData', NaN, 'YData', NaN, ...
                'FaceColor', [0.82 0.06 0.08], ...
                'EdgeColor', [0.18 0.18 0.20], 'LineWidth', 1.5);

            app.LanderStripePatch = patch(app.MainAxes, ...
                'XData', NaN, 'YData', NaN, ...
                'FaceColor', [0.82 0.06 0.08], ...
                'EdgeColor', 'none');

            app.LanderLegLines = gobjects(1,3);
            app.LanderFootLines = gobjects(1,3);
            for li = 1:3
                app.LanderLegLines(li) = plot(app.MainAxes, NaN, NaN, ...
                    'Color', [1.0 0.82 0.05], ...
                    'LineWidth', geom.legThickness);
                app.LanderFootLines(li) = plot(app.MainAxes, NaN, NaN, ...
                    'Color', [1.0 0.82 0.05], ...
                    'LineWidth', geom.footThickness);
            end

            % Keep legacy handle valid but hidden for compatibility.
            app.LanderPatch = patch(app.MainAxes, 'XData', NaN, 'YData', NaN, ...
                'FaceColor', 'none', 'EdgeColor', 'none', 'Visible', 'off');
            % --- Landing pad ---
padX = app.Params.target.x;
padY = app.Params.target.y;
padHW = app.Params.target.halfWidth;

% Main landing platform
patch(app.MainAxes, ...
    [padX-padHW padX+padHW padX+padHW padX-padHW], ...
    [padY padY padY-1.5 padY-1.5], ...
    [0.15 0.55 0.9], ...
    'EdgeColor', [0.8 0.9 1.0], ...
    'LineWidth', 1.5);

% Landing strip
plot(app.MainAxes, ...
    [padX-padHW padX+padHW], ...
    [padY padY], ...
    'LineWidth', 6, ...
    'Color', [0.2 0.8 1.0]);

% Pad lights
scatter(app.MainAxes, ...
    [padX-padHW padX+padHW], ...
    [padY+0.7 padY+0.7], ...
    55, ...
    'MarkerFaceColor', [1.0 0.85 0.2], ...
    'MarkerEdgeColor', 'none');

% Pad center marker
plot(app.MainAxes, ...
    padX, padY+0.2, ...
    'p', ...
    'MarkerSize', 10, ...
    'MarkerFaceColor', [1.0 0.85 0.2], ...
    'MarkerEdgeColor', 'none');
        end

        function drawDynamicScene(app)
            x = app.State(1);
            y = app.State(3);
            theta = app.State(5);
            geom = getLandingGeometry(app.Params);

            R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

            % Main cylindrical fuselage, shown in side projection.
            bodyLocal = [ ...
                -geom.bodyWidth/2, geom.bodyBottom; ...
                 geom.bodyWidth/2, geom.bodyBottom; ...
                 geom.bodyWidth/2, geom.bodyTop; ...
                -geom.bodyWidth/2, geom.bodyTop];
            bodyWorld = (R * bodyLocal')';
            bodyWorld(:,1) = bodyWorld(:,1) + x;
            bodyWorld(:,2) = bodyWorld(:,2) + y;
            app.LanderBodyPatch.XData = bodyWorld(:,1);
            app.LanderBodyPatch.YData = bodyWorld(:,2);

            % Rounded red dome nose cone.
            a = linspace(0, pi, 40)';
            noseLocal = [ ...
                geom.bodyWidth/2*cos(a), ...
                geom.bodyTop + geom.noseHeight*sin(a)];
            noseWorld = (R * noseLocal')';
            noseWorld(:,1) = noseWorld(:,1) + x;
            noseWorld(:,2) = noseWorld(:,2) + y;
            app.LanderNosePatch.XData = noseWorld(:,1);
            app.LanderNosePatch.YData = noseWorld(:,2);

            % Red accent band around the fuselage.
            bandHalfHeight = 0.38;
            stripeLocal = [ ...
                -geom.bodyWidth/2, -bandHalfHeight; ...
                 geom.bodyWidth/2, -bandHalfHeight; ...
                 geom.bodyWidth/2,  bandHalfHeight; ...
                -geom.bodyWidth/2,  bandHalfHeight];
            stripeLocal(:,2) = stripeLocal(:,2) + 0.9;
            stripeWorld = (R * stripeLocal')';
            stripeWorld(:,1) = stripeWorld(:,1) + x;
            stripeWorld(:,2) = stripeWorld(:,2) + y;
            app.LanderStripePatch.XData = stripeWorld(:,1);
            app.LanderStripePatch.YData = stripeWorld(:,2);

            % Three sturdy yellow legs and their landing pads.
            attachWorld = (R * geom.attachLocal')';
            footWorld = (R * geom.feetLocal')';
            attachWorld(:,1) = attachWorld(:,1) + x;
            attachWorld(:,2) = attachWorld(:,2) + y;
            footWorld(:,1) = footWorld(:,1) + x;
            footWorld(:,2) = footWorld(:,2) + y;

            for li = 1:geom.footCount
                app.LanderLegLines(li).XData = [attachWorld(li,1), footWorld(li,1)];
                app.LanderLegLines(li).YData = [attachWorld(li,2), footWorld(li,2)];

                footHalf = geom.footPadHalfWidth;
                % Foot pad is kept perpendicular to the local vertical leg
                % direction in the planar rendering.
                footVec = R * [footHalf; 0];
                app.LanderFootLines(li).XData = [footWorld(li,1)-footVec(1), footWorld(li,1)+footVec(1)];
                app.LanderFootLines(li).YData = [footWorld(li,2)-footVec(2), footWorld(li,2)+footVec(2)];
            end

            if ~isempty(app.Telemetry.x)
                app.TrailLine.XData = app.Telemetry.x;
                app.TrailLine.YData = app.Telemetry.y;
            else
                app.TrailLine.XData = NaN;
                app.TrailLine.YData = NaN;
            end
        end

        function updateTelemetryLabels(app)
            s = app.State;
            p = app.Params;
            geom = getLandingGeometry(p);

            R = [cos(s(5)) -sin(s(5)); sin(s(5)) cos(s(5))];
            feetWorld = (R * geom.feetLocal')';
            feetWorld(:,1) = feetWorld(:,1) + s(1);
            feetWorld(:,2) = feetWorld(:,2) + s(3);
            lowestFootY = min(feetWorld(:,2));
            altitudeAbovePad = max(lowestFootY - p.target.y, 0);

            app.AltitudeLabel.Text    = sprintf('Altitude:  %.1f m', altitudeAbovePad);
            app.VxLabel.Text         = sprintf('Horiz. Vel:  %.2f m/s', s(2));
            app.VyLabel.Text         = sprintf('Vert. Vel:  %.2f m/s', s(4));
            app.PitchLabel.Text      = sprintf('Pitch:  %.3f rad', s(5));
            app.OmegaLabel.Text      = sprintf('Ang. Vel:  %.3f rad/s', s(6));
            if ~isempty(app.Telemetry.T)
                app.ThrustLabel.Text = sprintf('Thrust:  %.0f N', app.Telemetry.T(end));
            else
                app.ThrustLabel.Text = 'Thrust:  0 N';
            end
            app.FuelLabel.Text       = sprintf('Fuel:  %.1f kg', p.fuelMass);
            app.FlightTimeLabel.Text = sprintf('Flight Time:  %.2f s', app.FlightTime);
        end

    end

    % ===============================================================
    % POST-FLIGHT ANALYSIS WINDOW (Sections 23-24)
    % ===============================================================
    methods (Access = private)

        function showAnalysisWindow(app, results)
            if ~isempty(app.AnalysisFig) && isvalid(app.AnalysisFig)
                close(app.AnalysisFig)
            end
            app.AnalysisFig = uifigure('Name', 'Post-Flight Analysis', 'Position', [200 100 950 650]);

            t = app.Telemetry;
            tl = uigridlayout(app.AnalysisFig, [1 2]);
            tl.ColumnWidth = {'2x', '1x'};

            plotGrid = uigridlayout(tl, [2 2]);

            ax1 = uiaxes(plotGrid); ax1.Layout.Row = 1; ax1.Layout.Column = 1;
            plot(ax1, t.time, t.x, 'DisplayName', 'x(t)'); hold(ax1, 'on');
            yline(ax1, app.Params.target.x, '--', 'target x');
            title(ax1, 'Position Response'); xlabel(ax1,'t [s]'); ylabel(ax1,'x [m]'); legend(ax1);

            ax2 = uiaxes(plotGrid); ax2.Layout.Row = 1; ax2.Layout.Column = 2;
            plot(ax2, t.time, t.ax, 'DisplayName', 'a_x'); hold(ax2, 'on');
            plot(ax2, t.time, t.ay, 'DisplayName', 'a_y');
            title(ax2, 'Acceleration Response'); xlabel(ax2,'t [s]'); ylabel(ax2,'a [m/s^2]'); legend(ax2);

            ax3 = uiaxes(plotGrid); ax3.Layout.Row = 2; ax3.Layout.Column = 1;
            plot(ax3, t.time, t.theta, 'DisplayName', '\theta'); hold(ax3,'on');
            yline(ax3, 0, '--', 'reference \theta');
            title(ax3, 'Attitude Response'); xlabel(ax3,'t [s]'); ylabel(ax3,'\theta [rad]'); legend(ax3);

            ax4 = uiaxes(plotGrid); ax4.Layout.Row = 2; ax4.Layout.Column = 2;
            plot(ax4, t.time, t.T, 'DisplayName', 'T'); hold(ax4,'on');
            yline(ax4, 0, '--', '0');
            yline(ax4, app.Params.Tmax, '--', 'T_{max}');
            title(ax4, 'Thrust Response'); xlabel(ax4,'t [s]'); ylabel(ax4,'T [N]'); legend(ax4);

            summaryPanel = uipanel(tl);
            summaryPanel.Title = 'Summary';
            summaryText = uitextarea(summaryPanel);
            summaryText.Position = [10 10 280 590];
            summaryText.Editable = 'off';
            summaryText.Value = {
                sprintf('Landing Status:      %s', results.landingStatus)
                sprintf('Max Altitude:        %.2f m', results.maxAltitude)
                sprintf('Flight Time:         %.2f s', results.totalFlightTime)
sprintf('Fuel Used:           %.2f kg', results.fuelConsumed)
sprintf('Max Acceleration:    %.2f m/s^2', results.maxAbsAcceleration)
sprintf('Max Attitude:        %.3f rad', results.maxAbsAttitude)
                sprintf('Landing Vx:          %.2f m/s', results.landingVx)
                sprintf('Landing Vy:          %.2f m/s', results.landingVy)
                sprintf('Landing Theta:       %.3f rad', results.landingTheta)
                sprintf('Total Path Distance: %.2f m', results.totalPathLength)
                };
        end
    end

    % ===============================================================
    % APP CLOSING SAFETY (Section 25)
    % ===============================================================
    methods (Access = private)
        function uiFigureCloseRequest(app, ~, ~)
            stopAndDeleteTimer(app)
            clearKeyState(app)
            delete(app)
        end
    end
end

% ---------------------------------------------------------------------
function out = tern(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end
