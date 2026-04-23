classdef IntegratedRadarDirector < handle
    % =====================================================================
    % INTEGRATED RADAR SEARCH, TRACK & EFFECTOR DIRECTOR SIMULATION
    % =====================================================================
    % SYSTEM OF SYSTEMS ARCHITECTURE (V2.0 - Optimized):
    % 1. Target Environment: Orbital evasive maneuvers & linear tracks.
    % 2. Sensor (Radar): Physical link budget, SNR, Phased Array emulation.
    % 3. C2 Estimator: Time-Centered OLS Polynomial tracking with Predictive 
    %    Feed-Forward Lead-Time to counteract mechanical latency.
    % 4. Effector Domain: 
    %    -> Hexapod (Stewart Platform): Newton-Euler w/ Optical Lever.
    %    -> Azimuth/Elevation Gimbal: Euler-Lagrange w/ Zenith Keyhole Fix.
    %
    % OPTIMIZATIONS: Block Memory Allocation, Matrix Preconditioning, 
    % Singularity Avoidance, and Render-Culling.
    % =====================================================================

    properties (Access = private)
        % --- UI Components ---
        UIFigure, TargetTable
        BtnAddTarget, BtnRemTarget
        DropTargetSelect, DropEffectorType
        CheckManeuvers, CheckPredictive
        BtnRun, BtnStop, TxtOptLog, EstTable
        TabGroup, TabSensor, TabOperator, TabHexapod, TabAzEl
        
        % Radar Settings UI
        DropRadarMode, CheckMIMO, CheckMF, CheckDF, CheckAF, CheckFD  
        EditChirpBW, CheckRealTime
        
        % --- Axes & Graphics ---
        Ax3D, AxMeas3D, AxPPI, AxRangeDoppler2D, AxRangeDoppler   
        AxRangeError, AxVelError, AxPol, AxCrossSection
        beamConeSurf, hPolBars, hCrossShape
        hErrRange, hErrVel, hMeasRaw, hMeasLine, hMeasEst
        sweepLine, blipsScatter, rdScatter2D, rdScatter3D
        targetTransforms
        
        % Hexapod Graphics
        AxHexapod, HexTopPlate, HexLegs, HexLaser, HexWarningText, HexTelemetryText
        
        % AzEl Graphics
        AxAzEl, TAz, TEl, AzElLaser, AzElWarningText, AzElTelemetryText
        
        % --- Simulation States & Data Buffers ---
        simRunning = false;
        c = 299792458; 
        kT0 = 1.38e-23 * 290;
        
        % Modular States
        simState      % Time, dt, limits
        targets       % Struct array of targets
        radarParams   % SNR, BW, Link Budget parameters
        radarState    % Current azimuth, tracked flags
        trackHistory  % Block-allocated memory buffers
        visData       % Fading PPI and RD scatter data
        
        hexParams, hexState, hexRenderCounter
        azelParams, azelState, azelRenderCounter
        
        % Memory Constraints
        MAX_TRACK_MEM = 10000;
        trackCount = 0;
    end
    
    methods
        function obj = IntegratedRadarDirector()
            obj.initMemoryBuffers();
            obj.setupHexapodParams();
            obj.setupAzElParams();
            obj.buildUI();
        end
    end
    
    methods (Access = private)
        % =========================================================
        % --- MODULE 1: INITIALIZATION & SETUP ---
        % =========================================================
        function initMemoryBuffers(obj)
            obj.simState.time = 0;
            obj.simState.dt = 0.1; % 10Hz Radar Frame
            
            % Block Allocation: Pre-allocate maximum memory blocks to prevent 
            % 'end+1' memory heap fragmentation during high-speed loops.
            N = obj.MAX_TRACK_MEM;
            obj.trackCount = 0;
            obj.trackHistory.time = zeros(N, 1); obj.trackHistory.R_err = zeros(N, 1); obj.trackHistory.V_err = zeros(N, 1);
            obj.trackHistory.T_idx = zeros(N, 1); obj.trackHistory.meas_X = zeros(N, 1); obj.trackHistory.meas_Y = zeros(N, 1); 
            obj.trackHistory.meas_Z = zeros(N, 1); obj.trackHistory.SNR = zeros(N, 1); obj.trackHistory.HH = zeros(N, 1); 
            obj.trackHistory.VV = zeros(N, 1); obj.trackHistory.HV = zeros(N, 1);
            
            obj.visData.maxPPI = 100; obj.visData.maxRD = 500;
            obj.visData.blipsX = zeros(obj.visData.maxPPI, 1); obj.visData.blipsY = zeros(obj.visData.maxPPI, 1); obj.visData.blipsAge = ones(obj.visData.maxPPI, 1) * 100; obj.visData.blipIdx = 1;
            obj.visData.rdX = nan(obj.visData.maxRD, 1); obj.visData.rdY = nan(obj.visData.maxRD, 1); obj.visData.rdZ = nan(obj.visData.maxRD, 1);
            obj.visData.rdColor = repmat([0 0 0], obj.visData.maxRD, 1); obj.visData.rdAge = ones(obj.visData.maxRD, 1) * 100; obj.visData.rdIdx = 1;
        end
        
        function setupHexapodParams(obj)
            obj.hexParams.R_base = 0.5; obj.hexParams.R_top = 0.3; obj.hexParams.Z_nom = 0.6;    
            obj.hexParams.m_top = 25.0; 
            obj.hexParams.I_local = diag([ (1/4)*obj.hexParams.m_top*obj.hexParams.R_top^2, ...
                                           (1/4)*obj.hexParams.m_top*obj.hexParams.R_top^2, ...
                                           (1/2)*obj.hexParams.m_top*obj.hexParams.R_top^2 ]);
            obj.hexParams.g_vec = [0; 0; -9.81]; 
            theta_b = deg2rad([15, 105, 135, 225, 255, 345]); theta_t = deg2rad([45, 75, 165, 195, 285, 315]);
            obj.hexParams.B = zeros(3, 6); obj.hexParams.P = zeros(3, 6);
            for i = 1:6
                obj.hexParams.B(:, i) = [obj.hexParams.R_base * cos(theta_b(i)); obj.hexParams.R_base * sin(theta_b(i)); 0];
                obj.hexParams.P(:, i) = [obj.hexParams.R_top * cos(theta_t(i)); obj.hexParams.R_top * sin(theta_t(i)); 0];
            end
            obj.hexParams.F_limit = 2500; obj.hexParams.L_min = 0.35; obj.hexParams.L_max = 0.95; obj.hexParams.Cond_Limit = 500;       
            obj.hexState.Pt = [0; 0; 1000]; obj.hexState.V_t = zeros(3,1); obj.hexRenderCounter = 0;
        end
        
        function setupAzElParams(obj)
            obj.azelParams.m_az = 50.0; obj.azelParams.J_z = 4.0;  
            obj.azelParams.m_p = 30.0; r_p = 0.2; L_p = 0.8;
            obj.azelParams.I_x = 0.5 * obj.azelParams.m_p * r_p^2;
            obj.azelParams.I_y = (1/12) * obj.azelParams.m_p * (3*r_p^2 + L_p^2);
            obj.azelParams.I_z = obj.azelParams.I_y; 
            obj.azelParams.Tau_max_az = 500; obj.azelParams.Tau_max_el = 300; obj.azelParams.Max_Vel = 3.0;     
            obj.azelState.q = [0; 0]; obj.azelState.q_dot = [0; 0]; obj.azelRenderCounter = 0;
        end
        
        function buildUI(obj)
            obj.UIFigure = figure('Name', 'System of Systems: Closed-Loop Sensor to Effector', ...
                                  'Position', [50, 50, 1600, 900], 'MenuBar', 'none', 'NumberTitle', 'off', ...
                                  'Color', 'w', 'CloseRequestFcn', @obj.onClose);
                              
            % --- CONTROLS PANEL ---
            pnlControls = uipanel(obj.UIFigure, 'Title', '1. Scenario & Parameters', ...
                                  'Position', [0.01, 0.01, 0.28, 0.98], 'FontWeight', 'bold');
            
            % Target Data Table
            initialData = {'UAV (Stealth)', 0.1, 30, 20, 0.5, -30, -20, 0; 'Airliner', 50.0, -60, 50, 10.0, 200, -150, 0; 'Missile (Fast)', 0.5, 10, 70, 5.0, -100, -800, -50};
            colNames = {'Name', 'RCS', 'X (km)', 'Y (km)', 'Z (km)', 'Vx (m/s)', 'Vy (m/s)', 'Vz (m/s)'};
            colFmt = {'char', 'numeric', 'numeric', 'numeric', 'numeric', 'numeric', 'numeric', 'numeric'};
            obj.TargetTable = uitable('Parent', pnlControls, 'Data', initialData, 'ColumnName', colNames, 'ColumnFormat', colFmt, ...
                                      'ColumnEditable', true, 'Units', 'normalized', 'Position', [0.02, 0.81, 0.96, 0.14], 'RowName', []);
            
            obj.BtnAddTarget = uicontrol('Parent', pnlControls, 'Style', 'pushbutton', 'String', '+ Target', 'Units', 'normalized', 'Position', [0.02, 0.77, 0.45, 0.03], 'Callback', @(s,e) obj.addTarget());
            obj.BtnRemTarget = uicontrol('Parent', pnlControls, 'Style', 'pushbutton', 'String', '- Target', 'Units', 'normalized', 'Position', [0.53, 0.77, 0.45, 0.03], 'Callback', @(s,e) obj.remTarget());

            % Scenario Toggles
            obj.CheckManeuvers = uicontrol('Parent', pnlControls, 'Style', 'checkbox', 'String', 'Enable Orbital Maneuvers (Evasive)', 'Units', 'normalized', 'Position', [0.05, 0.73, 0.90, 0.03], 'FontWeight', 'bold', 'Value', 0);
            obj.CheckPredictive = uicontrol('Parent', pnlControls, 'Style', 'checkbox', 'String', 'Predictive Lead (Counter Latency)', 'Units', 'normalized', 'Position', [0.05, 0.70, 0.90, 0.03], 'FontWeight', 'bold', 'Value', 1, 'ForegroundColor', 'b');

            % Selection Dropdowns
            uicontrol('Parent', pnlControls, 'Style', 'text', 'String', 'Effector Target:', 'Units', 'normalized', 'Position', [0.02, 0.65, 0.35, 0.03], 'HorizontalAlignment', 'right', 'FontSize', 10);
            obj.DropTargetSelect = uicontrol('Parent', pnlControls, 'Style', 'popupmenu', 'String', {'Auto (Lowest TTI)'}, 'Units', 'normalized', 'Position', [0.38, 0.65, 0.60, 0.03], 'FontWeight', 'bold', 'Value', 1);
            obj.updateTargetDropdown(); 
            
            uicontrol('Parent', pnlControls, 'Style', 'text', 'String', 'Active Effector:', 'Units', 'normalized', 'Position', [0.02, 0.61, 0.35, 0.03], 'HorizontalAlignment', 'right', 'FontSize', 10);
            obj.DropEffectorType = uicontrol('Parent', pnlControls, 'Style', 'popupmenu', 'String', {'Stewart Platform (Hexapod)', 'Azimuth/Elevation Gimbal'}, 'Units', 'normalized', 'Position', [0.38, 0.61, 0.60, 0.03], 'FontWeight', 'bold', 'Value', 1);
                                  
            uicontrol('Parent', pnlControls, 'Style', 'text', 'String', 'Radar Mode:', 'Units', 'normalized', 'Position', [0.02, 0.57, 0.35, 0.03], 'HorizontalAlignment', 'right', 'FontSize', 10);
            obj.DropRadarMode = uicontrol('Parent', pnlControls, 'Style', 'popupmenu', 'String', {'Mechanical Sweep', 'AESA (Sequential)', 'AESA (MPAR Track)'}, 'Units', 'normalized', 'Position', [0.38, 0.57, 0.60, 0.03], 'FontWeight', 'bold', 'Value', 3);
            
            % DSP Panel
            pnlDSP = uipanel(pnlControls, 'Title', 'Radar DSP Enhancements', 'Position', [0.02, 0.40, 0.96, 0.16], 'FontWeight', 'bold');
            obj.CheckMF = uicontrol('Parent', pnlDSP, 'Style', 'checkbox', 'String', 'Matched Filter', 'Units', 'normalized', 'Position', [0.05, 0.75, 0.45, 0.2], 'Value', 1);
            obj.CheckDF = uicontrol('Parent', pnlDSP, 'Style', 'checkbox', 'String', 'Doppler MTI', 'Units', 'normalized', 'Position', [0.55, 0.75, 0.45, 0.2], 'Value', 1);
            obj.CheckAF = uicontrol('Parent', pnlDSP, 'Style', 'checkbox', 'String', 'STAP Adapt Filter', 'Units', 'normalized', 'Position', [0.05, 0.50, 0.45, 0.2], 'Value', 1);
            obj.CheckFD = uicontrol('Parent', pnlDSP, 'Style', 'checkbox', 'String', 'Freq Domain Int', 'Units', 'normalized', 'Position', [0.55, 0.50, 0.45, 0.2], 'Value', 1);
            obj.CheckMIMO = uicontrol('Parent', pnlDSP, 'Style', 'checkbox', 'String', 'MIMO Virtual Arr', 'Units', 'normalized', 'Position', [0.05, 0.25, 0.45, 0.2], 'Value', 1);
            uicontrol('Parent', pnlDSP, 'Style', 'text', 'String', 'LFM BW (MHz):', 'Units', 'normalized', 'Position', [0.55, 0.25, 0.25, 0.2], 'HorizontalAlignment', 'left');
            obj.EditChirpBW = uicontrol('Parent', pnlDSP, 'Style', 'edit', 'String', '5', 'Units', 'normalized', 'Position', [0.80, 0.25, 0.15, 0.25]);

            % Main Buttons
            obj.CheckRealTime = uicontrol('Parent', pnlControls, 'Style', 'checkbox', 'String', 'Sync Sim to 1x Real-Time', 'Units', 'normalized', 'Position', [0.05, 0.36, 0.9, 0.03], 'FontWeight', 'bold', 'Value', 1);
            obj.BtnRun = uicontrol('Parent', pnlControls, 'Style', 'pushbutton', 'String', '▶ Optimize & Run System', 'Units', 'normalized', 'Position', [0.02, 0.30, 0.65, 0.05], 'BackgroundColor', [0 0.5 0], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'FontSize', 11, 'Callback', @(src,event) obj.startSimulation());
            obj.BtnStop = uicontrol('Parent', pnlControls, 'Style', 'pushbutton', 'String', '■ Stop', 'Units', 'normalized', 'Position', [0.69, 0.30, 0.29, 0.05], 'BackgroundColor', [0.8 0 0], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'FontSize', 11, 'Callback', @(src,event) obj.stopSimulation());
            
            % Log Area
            pnlLog = uipanel(pnlControls, 'Title', 'Link Budget Optimizer Log', 'Position', [0.02, 0.01, 0.96, 0.28], 'FontWeight', 'bold');
            obj.TxtOptLog = uicontrol('Parent', pnlLog, 'Style', 'edit', 'String', 'Awaiting initialization...', 'Max', 2, 'Min', 0, 'Enable', 'inactive', 'Units', 'normalized', 'Position', [0.02, 0.02, 0.96, 0.96], 'HorizontalAlignment', 'left', 'FontSize', 9, 'FontName', 'Courier New');
                                  
            % --- RIGHT PANELS: TABBED VISUALIZATIONS ---
            obj.TabGroup = uitabgroup(obj.UIFigure, 'Position', [0.30, 0.01, 0.69, 0.98]);
            
            % TAB 1: RADAR SENSOR
            obj.TabSensor = uitab(obj.TabGroup, 'Title', '1. Radar Sensor View');
            obj.Ax3D = axes('Parent', obj.TabSensor, 'Position', [0.05, 0.55, 0.40, 0.40]); title(obj.Ax3D, 'True 3D Operational Airspace'); grid(obj.Ax3D, 'on'); view(obj.Ax3D, 3); xlabel(obj.Ax3D, 'East (m)'); ylabel(obj.Ax3D, 'North (m)'); zlabel(obj.Ax3D, 'Alt (m)');
            obj.AxRangeDoppler = axes('Parent', obj.TabSensor, 'Position', [0.55, 0.55, 0.40, 0.40]); title(obj.AxRangeDoppler, '3D RD-Time Map (Waterfall)'); grid(obj.AxRangeDoppler, 'on'); view(obj.AxRangeDoppler, 3); set(obj.AxRangeDoppler, 'Color', [0 0.1 0.1], 'GridColor', [1 1 1], 'GridAlpha', 0.2); xlabel(obj.AxRangeDoppler, 'Range (km)'); ylabel(obj.AxRangeDoppler, 'Velocity'); zlabel(obj.AxRangeDoppler, 'Time');
            obj.AxPPI = axes('Parent', obj.TabSensor, 'Position', [0.05, 0.05, 0.40, 0.40]); title(obj.AxPPI, 'PPI Radar Display'); set(obj.AxPPI, 'Color', [0 0.1 0], 'XColor', 'none', 'YColor', 'none'); 
            obj.AxRangeDoppler2D = axes('Parent', obj.TabSensor, 'Position', [0.55, 0.05, 0.40, 0.40]); title(obj.AxRangeDoppler2D, '2D Range-Doppler Map'); grid(obj.AxRangeDoppler2D, 'on'); set(obj.AxRangeDoppler2D, 'Color', [0 0.1 0.1], 'GridColor', [1 1 1], 'GridAlpha', 0.2); xlabel(obj.AxRangeDoppler2D, 'Range (km)'); ylabel(obj.AxRangeDoppler2D, 'Radial Vel (m/s)');
            
            % TAB 2: C2 ESTIMATOR
            obj.TabOperator = uitab(obj.TabGroup, 'Title', '2. C2 Estimator View');
            obj.AxMeas3D = axes('Parent', obj.TabOperator, 'Position', [0.02, 0.35, 0.45, 0.60]); title(obj.AxMeas3D, 'Measured 3D Space & Poly Estimations'); grid(obj.AxMeas3D, 'on'); view(obj.AxMeas3D, 3); xlabel(obj.AxMeas3D, 'East (m)'); ylabel(obj.AxMeas3D, 'North (m)'); zlabel(obj.AxMeas3D, 'Alt (m)');
            obj.AxPol = axes('Parent', obj.TabOperator, 'Position', [0.52, 0.68, 0.20, 0.27]); title(obj.AxPol, 'Dual-Pol Signature'); grid(obj.AxPol, 'on'); set(obj.AxPol, 'XTick', 1:3, 'XTickLabel', {'HH', 'VV', 'HV'}); ylabel(obj.AxPol, 'Measured RCS (m^2)');
            obj.AxCrossSection = axes('Parent', obj.TabOperator, 'Position', [0.76, 0.68, 0.20, 0.27]); title(obj.AxCrossSection, 'Estimated Silhouette'); axis(obj.AxCrossSection, 'equal'); set(obj.AxCrossSection, 'XTick', [], 'YTick', []); xlim(obj.AxCrossSection, [-10 10]); ylim(obj.AxCrossSection, [-10 10]);
            obj.AxRangeError = axes('Parent', obj.TabOperator, 'Position', [0.52, 0.35, 0.20, 0.25]); title(obj.AxRangeError, 'Range Error (m)'); grid(obj.AxRangeError, 'on');
            obj.AxVelError = axes('Parent', obj.TabOperator, 'Position', [0.76, 0.35, 0.20, 0.25]); title(obj.AxVelError, 'Velocity Error (m/s)'); grid(obj.AxVelError, 'on');
            estColNames = {'Track ID', 'Rng (km)', 'Alt (m)', 'Spd (m/s)', 'Est RCS', 'Shape', 'Class', 'TTI (s)'};
            obj.EstTable = uitable('Parent', obj.TabOperator, 'ColumnName', estColNames, 'ColumnWidth', {60, 60, 60, 60, 60, 100, 130, 60}, 'Units', 'normalized', 'Position', [0.02, 0.02, 0.96, 0.28], 'RowName', []);
            
            % Polymorphic Effector Tabs
            obj.buildHexapodTab();
            obj.buildAzElTab();
        end
        
        function buildHexapodTab(obj)
            obj.TabHexapod = uitab(obj.TabGroup, 'Title', '3A. Effector: Hexapod');
            obj.AxHexapod = axes('Parent', obj.TabHexapod, 'Position', [0.05, 0.1, 0.55, 0.8]); hold(obj.AxHexapod, 'on'); grid(obj.AxHexapod, 'on'); view(obj.AxHexapod, 3);
            xlabel(obj.AxHexapod, 'X (m)'); ylabel(obj.AxHexapod, 'Y (m)'); zlabel(obj.AxHexapod, 'Z (m)'); title(obj.AxHexapod, 'Hexapod Joint & Actuator Dynamics', 'FontSize', 14); axis(obj.AxHexapod, 'equal');
            xlim(obj.AxHexapod, [-0.8 0.8]); ylim(obj.AxHexapod, [-0.8 0.8]); zlim(obj.AxHexapod, [0 1.5]); 
            patch('Parent', obj.AxHexapod, 'XData', obj.hexParams.B(1,:), 'YData', obj.hexParams.B(2,:), 'ZData', obj.hexParams.B(3,:), 'FaceColor', [0.3 0.3 0.3], 'FaceAlpha', 0.5, 'EdgeColor', 'k', 'LineWidth', 2);
            obj.HexTopPlate = patch('Parent', obj.AxHexapod, 'XData', obj.hexParams.P(1,:), 'YData', obj.hexParams.P(2,:), 'ZData', obj.hexParams.P(3,:)+obj.hexParams.Z_nom, 'FaceColor', [0.2 0.6 0.8], 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'LineWidth', 1.5);
            obj.HexLegs = gobjects(1,6);
            for i = 1:6
                obj.HexLegs(i) = plot3(obj.AxHexapod, [obj.hexParams.B(1,i), obj.hexParams.P(1,i)], [obj.hexParams.B(2,i), obj.hexParams.P(2,i)], [obj.hexParams.B(3,i), obj.hexParams.P(3,i)+obj.hexParams.Z_nom], 'Color', [0.8 0.4 0.1], 'LineWidth', 4);
            end
            obj.HexLaser = plot3(obj.AxHexapod, [0, 0], [0, 0], [obj.hexParams.Z_nom, 1.5], 'r-', 'LineWidth', 3);
            obj.HexWarningText = uicontrol('Parent', obj.TabHexapod, 'Style', 'text', 'Position', [650, 750, 300, 30], 'String', 'SYSTEM NOMINAL', 'BackgroundColor', 'g', 'FontWeight', 'bold', 'FontSize', 14);
            obj.HexTelemetryText = uicontrol('Parent', obj.TabHexapod, 'Style', 'text', 'Position', [650, 400, 350, 300], 'String', 'Waiting for telemetry...', 'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontName', 'Courier New', 'FontSize', 11);
        end
        
        function buildAzElTab(obj)
            obj.TabAzEl = uitab(obj.TabGroup, 'Title', '3B. Effector: Az/El Gimbal');
            obj.AxAzEl = axes('Parent', obj.TabAzEl, 'Position', [0.05, 0.1, 0.55, 0.8]); hold(obj.AxAzEl, 'on'); grid(obj.AxAzEl, 'on'); view(obj.AxAzEl, 3);
            xlabel(obj.AxAzEl, 'X (m)'); ylabel(obj.AxAzEl, 'Y (m)'); zlabel(obj.AxAzEl, 'Z (m)'); title(obj.AxAzEl, 'Exact Euler-Lagrange Az/El Dynamics', 'FontSize', 14); axis(obj.AxAzEl, 'equal');
            xlim(obj.AxAzEl, [-4 4]); ylim(obj.AxAzEl, [-4 4]); zlim(obj.AxAzEl, [0 6]); 
            
            obj.TAz = hgtransform('Parent', obj.AxAzEl);
            obj.TEl = hgtransform('Parent', obj.TAz);
            
            [Xb, Yb, Zb] = cylinder([1.5, 1.5], 20); Zb = Zb * 1.5; 
            surface('Parent', obj.AxAzEl, 'XData', Xb, 'YData', Yb, 'ZData', Zb, 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none');
            
            [Xc, Yc, Zc] = cylinder([0.8, 0.8], 20); Zc = Zc * 1.0 + 1.5; 
            surface('Parent', obj.TAz, 'XData', Xc, 'YData', Yc, 'ZData', Zc, 'FaceColor', [0.2 0.5 0.8], 'EdgeColor', 'none');
            patch('Parent', obj.TAz, 'XData', [-0.5 -0.5 0.5 0.5], 'YData', [-1.2 -0.8 -0.8 -1.2], 'ZData', [2.5 2.5 2.5 2.5], 'FaceColor', [0.2 0.5 0.8]);
            patch('Parent', obj.TAz, 'XData', [-0.5 -0.5 0.5 0.5], 'YData', [-1.2 -0.8 -0.8 -1.2], 'ZData', [4.0 4.0 4.0 4.0], 'FaceColor', [0.2 0.5 0.8]);
            fill3(obj.TAz, [-0.5 -0.5 0.5 0.5], [-1.2 -0.8 -0.8 -1.2], [2.5 4.0 4.0 2.5], [0.2 0.5 0.8]);
            fill3(obj.TAz, [-0.5 -0.5 0.5 0.5], [0.8 1.2 1.2 0.8], [2.5 4.0 4.0 2.5], [0.2 0.5 0.8]);
            
            set(obj.TEl, 'Matrix', makehgtform('translate', [0, 0, 3.25]));
            [Xp, Yp, Zp] = cylinder([0.6, 0.6], 20); p_mat = makehgtform('yrotate', pi/2);
            pts = [Xp(:)'; Yp(:)'; Zp(:)'; ones(1, numel(Xp))]; pts = p_mat * pts;
            Xp(:) = pts(1,:); Yp(:) = pts(2,:); Zp(:) = pts(3,:); Xp = (Xp - 0.5) * 3.0; 
            surface('Parent', obj.TEl, 'XData', Xp, 'YData', Yp, 'ZData', Zp, 'FaceColor', [0.8 0.6 0.1], 'EdgeColor', 'k');
            
            obj.AzElLaser = plot3(obj.TEl, [0, 50], [0, 0], [0, 0], 'r-', 'LineWidth', 3);
            obj.AzElWarningText = uicontrol('Parent', obj.TabAzEl, 'Style', 'text', 'Position', [650, 750, 300, 30], 'String', 'SYSTEM NOMINAL', 'BackgroundColor', 'g', 'FontWeight', 'bold', 'FontSize', 14);
            obj.AzElTelemetryText = uicontrol('Parent', obj.TabAzEl, 'Style', 'text', 'Position', [650, 400, 320, 250], 'String', 'Waiting for telemetry...', 'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontName', 'Courier New', 'FontSize', 11);
        end
        
        % --- UI Callbacks ---
        function updateTargetDropdown(obj)
            data = obj.TargetTable.Data;
            strList = {'Auto (Lowest TTI)'};
            for i = 1:size(data, 1)
                if ~isempty(data{i,1}), strList{end+1} = sprintf('Trk-%d: %s', i, data{i,1}); end %#ok<AGROW>
            end
            obj.DropTargetSelect.String = strList;
            obj.DropTargetSelect.Value = min(obj.DropTargetSelect.Value, length(strList));
        end
        function addTarget(obj), obj.TargetTable.Data = [obj.TargetTable.Data; {'New Threat', 1.0, 40, 40, 5, -150, -150, 0}]; obj.updateTargetDropdown(); end
        function remTarget(obj), if size(obj.TargetTable.Data, 1) > 1, obj.TargetTable.Data = obj.TargetTable.Data(1:end-1, :); obj.updateTargetDropdown(); end; end
        function stopSimulation(obj), obj.simRunning = false; end
        function onClose(obj, ~, ~), obj.simRunning = false; delete(obj.UIFigure); end
        
        % =========================================================
        % --- MODULE 2: MASTER SIMULATION CONTROLLER ---
        % =========================================================
        function startSimulation(obj)
            if obj.simRunning, return; end
            obj.simRunning = true; obj.BtnRun.Enable = 'off'; 
            
            obj.initMemoryBuffers();
            if ~obj.parseAndOptimizeTargets(), return; end
            obj.buildGeometricEnvironment();
            
            % Modular System Loop
            while obj.simRunning && isvalid(obj.UIFigure)
                loop_start = tic;
                obj.simState.time = obj.simState.time + obj.simState.dt;
                
                % 1. Target Step (Includes auto-orbital maneuvers)
                obj.stepTargets();
                
                % 2. Sensor Step (Radar detection & measurements)
                obj.stepRadarSensor();
                
                % 3. C2 Estimator Step (OLS Polynomial & Predictive Lead)
                effector_cmd_pos = obj.stepC2Estimator();
                
                % 4. Effector Step (Polymorphic tracking)
                obj.stepEffector(effector_cmd_pos);
                
                drawnow limitrate;
                if obj.CheckRealTime.Value
                    pause_time = obj.simState.dt - toc(loop_start);
                    if pause_time > 0, pause(pause_time); end
                end
            end
            if isvalid(obj.UIFigure), obj.BtnRun.Enable = 'on'; end
        end
        
        function success = parseAndOptimizeTargets(obj)
            data = obj.TargetTable.Data;
            tIdx = 1; obj.targets = [];
            for i = 1:size(data, 1)
                if isempty(data{i,1}) || (isstring(data{i,1}) && data{i,1} == ""), continue; end
                obj.targets(tIdx).name = data{i,1};
                obj.targets(tIdx).rcs  = data{i,2};
                obj.targets(tIdx).pos  = [data{i,3}, data{i,4}, data{i,5}] * 1000;
                obj.targets(tIdx).vel  = [data{i,6}, data{i,7}, data{i,8}];
                
                nameLower = lower(obj.targets(tIdx).name);
                if contains(nameLower, 'airliner') || contains(nameLower, 'wing'), obj.targets(tIdx).shapePol = 3.0; 
                elseif contains(nameLower, 'missile') || contains(nameLower, 'rocket'), obj.targets(tIdx).shapePol = 0.33; 
                else, obj.targets(tIdx).shapePol = 1.0; 
                end
                tIdx = tIdx + 1;
            end
            
            if isempty(obj.targets)
                obj.TxtOptLog.String = 'Error: No valid targets.'; obj.stopSimulation(); obj.BtnRun.Enable = 'on'; 
                success = false; return;
            end
            
            % Radar Link Budget Optimizer
            maxR = 0; maxV = 0;
            for i = 1:length(obj.targets)
                if norm(obj.targets(i).pos) > maxR, maxR = norm(obj.targets(i).pos); end
                if norm(obj.targets(i).vel) > maxV, maxV = norm(obj.targets(i).vel); end
            end
            obj.radarParams.R_max = maxR * 1.2; 
            obj.radarParams.PRF = obj.c / (2 * obj.radarParams.R_max);
            BW_MHz = str2double(obj.EditChirpBW.String); if isnan(BW_MHz), BW_MHz = 5; end
            obj.radarParams.BW = BW_MHz * 1e6;
            
            obj.radarParams.scan_rate = 360 / 2.0; % 2s revisit
            obj.radarParams.beamwidth = 5;
            obj.radarParams.T_dwell = obj.radarParams.beamwidth / obj.radarParams.scan_rate; 
            
            obj.radarParams.req_SNR = 10^(13/10);
            obj.radarParams.lambda = obj.c/3e9; obj.radarParams.G = 10^(35/10); obj.radarParams.NoiseF = 10^(3/10); obj.radarParams.tau = 10e-6;
            
            obj.radarParams.PG = 1;
            if obj.CheckMF.Value, obj.radarParams.PG = obj.radarParams.PG * (obj.radarParams.BW * obj.radarParams.tau); end
            if obj.CheckDF.Value, obj.radarParams.PG = obj.radarParams.PG * 16; end 
            if obj.CheckAF.Value, obj.radarParams.PG = obj.radarParams.PG * 10; end 
            if obj.CheckFD.Value, obj.radarParams.PG = obj.radarParams.PG * 16; end 
            
            worstP = 0;
            for i = 1:length(obj.targets)
                Pt_req = (obj.radarParams.req_SNR * ((4*pi)^3) * (norm(obj.targets(i).pos)^4) * obj.kT0 * obj.radarParams.BW * obj.radarParams.NoiseF) / (obj.radarParams.G^2 * obj.radarParams.lambda^2 * obj.targets(i).rcs * obj.radarParams.PG);
                if Pt_req > worstP, worstP = Pt_req; end
            end
            obj.radarParams.Pt = worstP * 1.5; 
            
            logStr = sprintf('--- SYSTEM CAPABILITY LIMITS ---\n');
            logStr = [logStr, sprintf('-> Required Peak Power: %.1f kW.\n', obj.radarParams.Pt/1000)];
            min_rcs = (obj.radarParams.req_SNR * ((4*pi)^3) * (obj.radarParams.R_max^4) * obj.kT0 * obj.radarParams.BW * obj.radarParams.NoiseF) / (obj.radarParams.Pt * obj.radarParams.G^2 * obj.radarParams.lambda^2 * obj.radarParams.PG);
            logStr = [logStr, sprintf('-> Evade RCS Threshold: < %.4f m^2 at %.1f km\n', min_rcs, obj.radarParams.R_max/1000)];
            obj.TxtOptLog.String = logStr;
            
            obj.radarState.last_az = 0; obj.radarState.tracked = false(length(obj.targets), 1); obj.radarState.mpar_tick = 0;
            success = true;
        end
        
        function buildGeometricEnvironment(obj)
            cla(obj.Ax3D); cla(obj.AxMeas3D); cla(obj.AxPPI); cla(obj.AxRangeDoppler2D); cla(obj.AxRangeDoppler); 
            cla(obj.AxRangeError); cla(obj.AxVelError); cla(obj.AxPol); cla(obj.AxCrossSection);
            
            hold(obj.Ax3D, 'on'); hold(obj.AxMeas3D, 'on'); hold(obj.AxPPI, 'on'); hold(obj.AxRangeDoppler2D, 'on'); hold(obj.AxRangeDoppler, 'on');
            hold(obj.AxRangeError, 'on'); hold(obj.AxVelError, 'on'); hold(obj.AxPol, 'on'); hold(obj.AxCrossSection, 'on');
            
            obj.hPolBars = bar(obj.AxPol, categorical({'HH', 'VV', 'HV'}), [0 0 0], 'FaceColor', [0.2 0.6 0.8]);
            obj.hCrossShape = rectangle('Parent', obj.AxCrossSection, 'Position', [-1 -1 2 2], 'Curvature', [1 1], 'FaceColor', 'y', 'EdgeColor', 'k');
            
            lim = obj.radarParams.R_max;
            axis(obj.Ax3D, 'equal'); xlim(obj.Ax3D, [-lim lim]); ylim(obj.Ax3D, [-lim lim]); zlim(obj.Ax3D, [0 lim/2]);
            axis(obj.AxMeas3D, 'equal'); xlim(obj.AxMeas3D, [-lim lim]); ylim(obj.AxMeas3D, [-lim lim]); zlim(obj.AxMeas3D, [0 lim/2]);
            axis(obj.AxPPI, 'equal'); xlim(obj.AxPPI, [-lim lim]); ylim(obj.AxPPI, [-lim lim]);
            
            for ring_R = [0.25, 0.5, 0.75, 1.0] * lim
                plot(obj.AxPPI, ring_R*cos(linspace(0, 2*pi, 100)), ring_R*sin(linspace(0, 2*pi, 100)), 'Color', [0 0.4 0], 'LineStyle', '--');
            end
            
            xlim(obj.AxRangeDoppler2D, [0, lim/1000]); xlim(obj.AxRangeDoppler, [0, lim/1000]);
            ylim(obj.AxRangeDoppler2D, [-300, 300]); ylim(obj.AxRangeDoppler, [-300, 300]); zlim(obj.AxRangeDoppler, [0, 15]); 
            
            radarBase = hgtransform('Parent', obj.Ax3D);
            patch('Parent', radarBase, 'XData', [0 0 0 0], 'YData', [-500 500 500 -500], 'ZData', [-500 -500 500 500], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k');
            [bX, bY, bZ] = cylinder([0, lim * tand(obj.radarParams.beamwidth/2)], 20);
            obj.beamConeSurf = surface('XData', bZ * lim, 'YData', bX, 'ZData', bY, 'FaceColor', 'c', 'EdgeColor', 'none', 'FaceAlpha', 0.15, 'Parent', radarBase);
                
            numT = length(obj.targets);
            obj.targetTransforms = gobjects(numT, 1);
            obj.hErrRange = gobjects(numT, 1); obj.hErrVel = gobjects(numT, 1);
            obj.hMeasRaw = gobjects(numT, 1); obj.hMeasLine = gobjects(numT, 1); obj.hMeasEst = gobjects(numT, 1);
            
            colorPalette = lines(numT); 
            for i = 1:numT
                obj.targetTransforms(i) = hgtransform('Parent', obj.Ax3D);
                scale = log10(obj.targets(i).rcs + 1) * 800 + 800; 
                V = [scale 0 0; -scale -scale -scale; -scale scale -scale; -scale 0 scale]; 
                patch('Parent', obj.targetTransforms(i), 'Vertices', V, 'Faces', [1 2 3; 1 3 4; 1 4 2; 2 3 4], 'FaceColor', colorPalette(i,:), 'EdgeColor', 'k', 'FaceAlpha', 0.8);
                obj.targets(i).textObj = text(obj.Ax3D, 0, 0, 0, obj.targets(i).name, 'FontSize', 8, 'Color', 'k', 'FontWeight', 'bold');
                
                obj.hErrRange(i) = scatter(obj.AxRangeError, NaN, NaN, 36, colorPalette(i,:), 'filled');
                obj.hErrVel(i) = scatter(obj.AxVelError, NaN, NaN, 36, colorPalette(i,:), 'filled');
                obj.hMeasRaw(i) = scatter3(obj.AxMeas3D, NaN, NaN, NaN, 15, colorPalette(i,:), 'filled');
                obj.hMeasLine(i) = plot3(obj.AxMeas3D, NaN, NaN, NaN, '--', 'Color', colorPalette(i,:), 'LineWidth', 2);
                obj.hMeasEst(i) = scatter3(obj.AxMeas3D, NaN, NaN, NaN, 80, 's', 'MarkerEdgeColor', colorPalette(i,:), 'LineWidth', 2);
            end
            obj.sweepLine = plot(obj.AxPPI, [0, lim], [0, 0], 'Color', [0 1 0], 'LineWidth', 2);
            obj.blipsScatter = scatter(obj.AxPPI, obj.visData.blipsX, obj.visData.blipsY, 60, 'filled', 'MarkerFaceColor', [0 1 0]);
            obj.rdScatter2D = scatter(obj.AxRangeDoppler2D, obj.visData.rdX, obj.visData.rdY, 40, obj.visData.rdColor, 'filled');
            obj.rdScatter3D = scatter3(obj.AxRangeDoppler, obj.visData.rdX, obj.visData.rdY, obj.visData.rdZ, 40, obj.visData.rdColor, 'filled');
        end
        
        % =========================================================
        % --- MODULE 3: TARGET STEP (ORBITAL MANEUVERS) ---
        % =========================================================
        function stepTargets(obj)
            doManeuver = obj.CheckManeuvers.Value;
            dt = obj.simState.dt;
            t = obj.simState.time;
            
            for i = 1:length(obj.targets)
                pos = obj.targets(i).pos;
                vel = obj.targets(i).vel;
                
                if doManeuver
                    % Generate orthogonal vector for turning
                    orth_vel = [-vel(2), vel(1), 0];
                    if norm(orth_vel) > 0, orth_vel = orth_vel / norm(orth_vel); end
                    
                    % Evasive Maneuver: Turn Accel + Vertical Porpoise
                    turn_accel = orth_vel * 25 * sin(0.4 * t + i*2); % ~2.5G lateral turn
                    vert_accel = [0, 0, 10 * cos(0.3 * t + i)];      % ~1G climb/dive
                    vel = vel + (turn_accel + vert_accel) * dt;
                end
                
                pos = pos + vel * dt;
                obj.targets(i).pos = pos;
                obj.targets(i).vel = vel;
                
                % Safe Geometry update
                if isvalid(obj.targetTransforms(i))
                    heading = atan2(vel(2), vel(1)); pitch = atan2(vel(3), norm(vel(1:2)));
                    set(obj.targetTransforms(i), 'Matrix', makehgtform('translate', pos, 'zrotate', heading, 'yrotate', -pitch));
                    set(obj.targets(i).textObj, 'Position', pos + [0 0 2000]);
                end
            end
        end
        
        % =========================================================
        % --- MODULE 4: SENSOR STEP (RADAR EQUATION) ---
        % =========================================================
        function stepRadarSensor(obj)
            t = obj.simState.time; dt = obj.simState.dt;
            if t > 15, zlim(obj.AxRangeDoppler, [t - 15, t]); end
            
            is_track_frame = false; curr_idx = -1;
            if obj.DropRadarMode.Value == 3 && any(obj.radarState.tracked)
                obj.radarState.mpar_tick = obj.radarState.mpar_tick + 1;
                if mod(obj.radarState.mpar_tick, 4) == 0 
                    is_track_frame = true;
                    actives = find(obj.radarState.tracked);
                    curr_idx = actives(mod(obj.radarState.mpar_tick/4 - 1, length(actives)) + 1);
                end
            end
            
            if is_track_frame
                tgt = obj.targets(curr_idx).pos; az_deg = rad2deg(atan2(tgt(2), tgt(1)));
                if az_deg < 0, az_deg = az_deg + 360; end
                set(obj.beamConeSurf, 'FaceColor', 'm', 'FaceAlpha', 0.4); 
                set(obj.sweepLine, 'Color', [1 0 1]);
            else
                set(obj.beamConeSurf, 'FaceColor', 'c', 'FaceAlpha', 0.15); 
                set(obj.sweepLine, 'Color', [0 1 0]);
                if obj.DropRadarMode.Value == 1, obj.radarState.last_az = obj.radarState.last_az + (obj.radarParams.scan_rate * dt);
                else, obj.radarState.last_az = floor(t / obj.radarParams.T_dwell) * obj.radarParams.beamwidth; end
                az_deg = mod(obj.radarState.last_az, 360);
            end
            
            pMat = get(obj.beamConeSurf, 'Parent');
            set(pMat, 'Matrix', makehgtform('zrotate', deg2rad(az_deg)));
            set(obj.sweepLine, 'XData', [0, obj.radarParams.R_max * cosd(az_deg)], 'YData', [0, obj.radarParams.R_max * sind(az_deg)]);
            
            colorPalette = lines(length(obj.targets));
            
            for i = 1:length(obj.targets)
                pos = obj.targets(i).pos; vel = obj.targets(i).vel;
                rng = norm(pos); az = mod(rad2deg(atan2(pos(2), pos(1))), 360); el = rad2deg(asin(pos(3) / rng)); 
                v_r = -dot(vel, pos / norm(pos)); 
                
                angle_diff = abs(mod(deg2rad(az_deg) - deg2rad(az) + pi, 2*pi) - pi);
                
                if angle_diff <= deg2rad(obj.radarParams.beamwidth/2)
                    snr_lin = (obj.radarParams.Pt * obj.radarParams.G^2 * obj.radarParams.lambda^2 * obj.targets(i).rcs) / (((4*pi)^3) * (rng^4) * obj.kT0 * obj.radarParams.BW * obj.radarParams.NoiseF) * obj.radarParams.PG;
                    snr_dB = 10 * log10(snr_lin);
                    
                    if snr_dB >= 13
                        obj.radarState.tracked(i) = true;
                        
                        eff_dwell = obj.radarParams.T_dwell; if is_track_frame, eff_dwell = eff_dwell * 2; end
                        sigR = (obj.c / (2 * obj.radarParams.BW)) / sqrt(2 * snr_lin); if ~obj.CheckMF.Value, sigR = sigR * 5; end
                        sigV = (obj.radarParams.lambda / (2 * eff_dwell)) / sqrt(2 * snr_lin); if ~obj.CheckDF.Value, sigV = sigV * 20; end
                        sigA = obj.radarParams.beamwidth / sqrt(2 * snr_lin); if obj.CheckMIMO.Value, sigA = sigA / 4; end
                        
                        m_rng = rng + randn()*sigR; m_vr = v_r + randn()*sigV; m_az = az + randn()*sigA; m_el = el + randn()*sigA;
                        m_x = m_rng * cosd(m_el) * cosd(m_az); m_y = m_rng * cosd(m_el) * sind(m_az); m_z = m_rng * sind(m_el);
                        
                        pol = obj.targets(i).shapePol; trueHH = obj.targets(i).rcs * (2*pol/(pol+1)); trueVV = obj.targets(i).rcs * (2*1/(pol+1));
                        sigRCS = obj.targets(i).rcs / sqrt(snr_lin);
                        m_HH = max(0, trueHH + randn()*sigRCS); m_VV = max(0, trueVV + randn()*sigRCS); m_HV = max(0, obj.targets(i).rcs*0.1 + randn()*sigRCS);
                        
                        % --- OPTIMIZATION: Block Allocation Hit Storage ---
                        obj.trackCount = obj.trackCount + 1;
                        if obj.trackCount > obj.MAX_TRACK_MEM
                            % Perform a single vectorized shift of the buffer block
                            shiftN = floor(obj.MAX_TRACK_MEM / 2);
                            keep_idx = (shiftN+1):obj.MAX_TRACK_MEM;
                            obj.trackHistory.time(1:shiftN) = obj.trackHistory.time(keep_idx);
                            obj.trackHistory.R_err(1:shiftN) = obj.trackHistory.R_err(keep_idx);
                            obj.trackHistory.V_err(1:shiftN) = obj.trackHistory.V_err(keep_idx);
                            obj.trackHistory.T_idx(1:shiftN) = obj.trackHistory.T_idx(keep_idx);
                            obj.trackHistory.meas_X(1:shiftN) = obj.trackHistory.meas_X(keep_idx);
                            obj.trackHistory.meas_Y(1:shiftN) = obj.trackHistory.meas_Y(keep_idx);
                            obj.trackHistory.meas_Z(1:shiftN) = obj.trackHistory.meas_Z(keep_idx);
                            obj.trackHistory.SNR(1:shiftN) = obj.trackHistory.SNR(keep_idx);
                            obj.trackHistory.HH(1:shiftN) = obj.trackHistory.HH(keep_idx);
                            obj.trackHistory.VV(1:shiftN) = obj.trackHistory.VV(keep_idx);
                            obj.trackHistory.HV(1:shiftN) = obj.trackHistory.HV(keep_idx);
                            obj.trackCount = shiftN + 1;
                        end
                        
                        cIdx = obj.trackCount;
                        obj.trackHistory.time(cIdx) = t; obj.trackHistory.R_err(cIdx) = m_rng - rng; obj.trackHistory.V_err(cIdx) = m_vr - v_r;
                        obj.trackHistory.T_idx(cIdx) = i; obj.trackHistory.meas_X(cIdx) = m_x; obj.trackHistory.meas_Y(cIdx) = m_y; obj.trackHistory.meas_Z(cIdx) = m_z;
                        obj.trackHistory.SNR(cIdx) = snr_dB; obj.trackHistory.HH(cIdx) = m_HH; obj.trackHistory.VV(cIdx) = m_VV; obj.trackHistory.HV(cIdx) = m_HV;
                        
                        % Update Viz Buffers
                        idxP = obj.visData.blipIdx; obj.visData.blipsX(idxP) = m_rng * cosd(m_az); obj.visData.blipsY(idxP) = m_rng * sind(m_az); obj.visData.blipsAge(idxP) = 0; obj.visData.blipIdx = mod(idxP, obj.visData.maxPPI) + 1;
                        idxR = obj.visData.rdIdx; obj.visData.rdX(idxR) = m_rng/1000; obj.visData.rdY(idxR) = m_vr; obj.visData.rdZ(idxR) = t; obj.visData.rdColor(idxR,:) = colorPalette(i,:); obj.visData.rdAge(idxR) = 0; obj.visData.rdIdx = mod(idxR, obj.visData.maxRD) + 1;
                        
                        if isvalid(obj.targetTransforms(i)), pObj = findobj(obj.targetTransforms(i), 'Type', 'Patch'); if ~isempty(pObj), set(pObj, 'FaceColor', 'y'); end; end
                    else
                        if is_track_frame && i == curr_idx, obj.radarState.tracked(i) = false; end
                    end
                else
                    if isvalid(obj.targetTransforms(i)), pObj = findobj(obj.targetTransforms(i), 'Type', 'Patch'); if ~isempty(pObj), set(pObj, 'FaceColor', colorPalette(i,:)); end; end
                end
            end
        end
        
        % =========================================================
        % --- MODULE 5: C2 ESTIMATOR (PREDICTIVE LEAD TIME) ---
        % =========================================================
        function effector_cmd = stepC2Estimator(obj)
            t = obj.simState.time;
            numT = length(obj.targets);
            estData = cell(numT, 8);
            effector_cmd = [0; 0; 1000]; % Default Zenith Safety state
            best_tti = Inf;
            dyn_target_mode = obj.DropTargetSelect.Value; 
            colorPalette = lines(numT);
            
            if obj.trackCount > 0
                active_T_idx = obj.trackHistory.T_idx(1:obj.trackCount);
                active_Time = obj.trackHistory.time(1:obj.trackCount);
                
                for i = 1:numT
                    idx = (active_T_idx == i);
                    if any(idx)
                        set(obj.hErrRange(i), 'XData', active_Time(idx), 'YData', obj.trackHistory.R_err(idx));
                        set(obj.hErrVel(i), 'XData', active_Time(idx), 'YData', obj.trackHistory.V_err(idx));
                        
                        % Rolling window for Maneuver tracking
                        fit_idx = idx & (active_Time >= t - 20);
                        if sum(fit_idx) >= 3
                            tv = active_Time(fit_idx);
                            xv = obj.trackHistory.meas_X(fit_idx); yv = obj.trackHistory.meas_Y(fit_idx); zv = obj.trackHistory.meas_Z(fit_idx);
                            set(obj.hMeasRaw(i), 'XData', xv, 'YData', yv, 'ZData', zv);
                            
                            % --- OPTIMIZATION: Matrix Preconditioning via Time Shifting ---
                            % Ensures the Vandermonde matrix in OLS does not become rank 
                            % deficient as simulation time grows into thousands of seconds.
                            tv_shift = tv - t; 
                            
                            p_deg = min(2, sum(fit_idx)-1);
                            pX = polyfit(tv_shift, xv, p_deg); pY = polyfit(tv_shift, yv, p_deg); pZ = polyfit(tv_shift, zv, p_deg);
                            
                            t_ext = [min(tv_shift), max(tv_shift) + 10]; 
                            set(obj.hMeasLine(i), 'XData', polyval(pX, t_ext), 'YData', polyval(pY, t_ext), 'ZData', polyval(pZ, t_ext));
                            
                            % Predictive Feed-Forward Calculation
                            t_eval_shift = 0; % Default to current time (t - t = 0)
                            if obj.CheckPredictive.Value
                                t_eval_shift = 0.15; % 150 ms system lag compensation lead
                            end
                            
                            curr_est = [polyval(pX, t_eval_shift); polyval(pY, t_eval_shift); polyval(pZ, t_eval_shift)];
                            set(obj.hMeasEst(i), 'XData', curr_est(1), 'YData', curr_est(2), 'ZData', curr_est(3));
                            
                            % Kinematics extract
                            if p_deg == 2, v_est = [pX(2), pY(2), pZ(2)]; else, v_est = [pX(1), pY(1), pZ(1)]; end
                            spd = norm(v_est); rng = norm(curr_est);
                            v_r = -dot(v_est, curr_est / rng);
                            
                            if v_r > 0
                                tti = rng / v_r; 
                                tti_str = sprintf('%.1f', tti);
                                if dyn_target_mode == 1 && tti < best_tti
                                    best_tti = tti; effector_cmd = curr_est;
                                end
                            else
                                tti = Inf; 
                                tti_str = 'N/A';
                            end
                            
                            if dyn_target_mode == (i + 1), effector_cmd = curr_est; best_tti = -1; end
                            
                            last_idx = find(idx, 1, 'last');
                            snr = obj.trackHistory.SNR(last_idx); HH = obj.trackHistory.HH(last_idx); VV = obj.trackHistory.VV(last_idx); HV = obj.trackHistory.HV(last_idx);
                            estRcs = (10^(snr/10) * ((4*pi)^3) * (rng^4) * obj.kT0 * obj.radarParams.BW * obj.radarParams.NoiseF) / (obj.radarParams.Pt * obj.radarParams.G^2 * obj.radarParams.lambda^2 * obj.radarParams.PG);
                            ratio = HH / (VV + 1e-6);
                            if ratio > 2.0, shp = 'Horizontal'; elseif ratio < 0.5, shp = 'Vertical'; else, shp = 'Symmetric'; end
                            if spd > 343, cls = 'Supersonic Missile'; elseif spd > 100 && ratio > 2.0, cls = 'Airliner'; elseif ratio > 0.5 && ratio < 2.0, cls = 'Stealth/UAV'; else, cls = 'Unknown'; end
                            
                            estData{i,1}=sprintf('Trk-%d',i); estData{i,2}=sprintf('%.2f',rng/1000); estData{i,3}=sprintf('%.0f',curr_est(3)); estData{i,4}=sprintf('%.1f',spd); estData{i,5}=sprintf('%.2f',estRcs); estData{i,6}=shp; estData{i,7}=cls; estData{i,8}=tti_str;
                            
                            if best_tti == -1 || (dyn_target_mode == 1 && tti == best_tti)
                                set(obj.hPolBars, 'YData', [HH, VV, HV]);
                                set(obj.hCrossShape, 'Position', [-max(0.5, sqrt(HH)*2), -max(0.5, sqrt(VV)*2), max(0.5, sqrt(HH)*4), max(0.5, sqrt(VV)*4)], 'FaceColor', colorPalette(i,:));
                            end
                        else
                            set(obj.hMeasRaw(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN); set(obj.hMeasLine(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN); set(obj.hMeasEst(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                        end
                    else
                        set(obj.hErrRange(i), 'XData', NaN, 'YData', NaN); set(obj.hErrVel(i), 'XData', NaN, 'YData', NaN);
                        set(obj.hMeasRaw(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN); set(obj.hMeasLine(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN); set(obj.hMeasEst(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                    end
                end
            end
            
            validRows = ~cellfun('isempty', estData(:,1));
            if any(validRows), obj.EstTable.Data = estData(validRows, :); end
            
            obj.visData.blipsAge = obj.visData.blipsAge + obj.simState.dt; 
            set(obj.blipsScatter, 'XData', obj.visData.blipsX, 'YData', obj.visData.blipsY, 'CData', [zeros(obj.visData.maxPPI,1), max(0, 1 - obj.visData.blipsAge/3.0), zeros(obj.visData.maxPPI,1)]);
            
            obj.visData.rdAge = obj.visData.rdAge + obj.simState.dt; fadColor = obj.visData.rdColor .* max(0, 1 - obj.visData.rdAge/3.0);
            set(obj.rdScatter2D, 'XData', obj.visData.rdX, 'YData', obj.visData.rdY, 'CData', fadColor);
            set(obj.rdScatter3D, 'XData', obj.visData.rdX, 'YData', obj.visData.rdY, 'ZData', obj.visData.rdZ, 'CData', fadColor);
        end
        
        % =========================================================
        % --- MODULE 6: POLYMORPHIC EFFECTOR DOMAIN ---
        % =========================================================
        function stepEffector(obj, target_cmd)
            if obj.DropEffectorType.Value == 1
                obj.stepHexapodEffector(target_cmd, obj.simState.dt);
            else
                obj.stepAzElEffector(target_cmd, obj.simState.dt);
            end
        end
        
        % --- EFFECTOR A: EXACT HEXAPOD ---
        function stepHexapodEffector(obj, target_cmd, sim_dt)
            hex_dt = 0.01; steps = round(sim_dt / hex_dt);
            for k = 1:steps
                omega_n = 4.0; zeta = 1.0;
                A_cmd = omega_n^2 * (target_cmd - obj.hexState.Pt) - 2 * zeta * omega_n * obj.hexState.V_t;
                [F_cmd, Top_Points_cmd, l_cmd_cmd, u_opt_cmd, cond_J] = obj.compute_hexapod_state(obj.hexState.Pt, obj.hexState.V_t, A_cmd);
                
                warn_str = 'SYSTEM NOMINAL'; warn_col = 'g'; A_act = A_cmd; F_act = F_cmd; Top_Points = Top_Points_cmd; l_cmd = l_cmd_cmd; u_opt = u_opt_cmd;
                if cond_J > obj.hexParams.Cond_Limit, A_act = -10.0 * obj.hexState.V_t; warn_str = 'ERR: SINGULARITY'; warn_col = 'r'; [F_act, Top_Points, ~, u_opt, ~] = obj.compute_hexapod_state(obj.hexState.Pt, obj.hexState.V_t, A_act);
                elseif any(l_cmd < obj.hexParams.L_min) || any(l_cmd > obj.hexParams.L_max), A_act = -10.0 * obj.hexState.V_t; warn_str = 'ERR: STROKE LIMIT'; warn_col = 'r'; [F_act, Top_Points, ~, u_opt, ~] = obj.compute_hexapod_state(obj.hexState.Pt, obj.hexState.V_t, A_act);
                elseif max(abs(F_cmd)) > obj.hexParams.F_limit
                    [F_base, ~, ~, ~, ~] = obj.compute_hexapod_state(obj.hexState.Pt, obj.hexState.V_t, zeros(3,1));
                    if max(abs(F_base)) > obj.hexParams.F_limit, A_act = -5.0 * obj.hexState.V_t; warn_str = 'ERR: OVERLOAD'; warn_col = 'r'; [F_act, Top_Points, ~, u_opt, ~] = obj.compute_hexapod_state(obj.hexState.Pt, obj.hexState.V_t, A_act);
                    else
                        alpha = 1.0; dF = F_cmd - F_base;
                        for j = 1:6, if abs(dF(j))>1e-6, if dF(j)>0, alpha = min(alpha, (obj.hexParams.F_limit - F_base(j))/dF(j)); else, alpha = min(alpha, (-obj.hexParams.F_limit - F_base(j))/dF(j)); end; end; end
                        A_act = max(0, alpha) * A_cmd; warn_str = sprintf('GOVERNOR ACTIVE (Scale: %.2f)', alpha); warn_col = 'y'; [F_act, Top_Points, ~, u_opt, cond_J] = obj.compute_hexapod_state(obj.hexState.Pt, obj.hexState.V_t, A_act);
                    end
                end
                
                obj.hexState.V_t = obj.hexState.V_t + A_act * hex_dt;
                obj.hexState.Pt = obj.hexState.Pt + obj.hexState.V_t * hex_dt;
            end
            
            if obj.TabGroup.SelectedTab == obj.TabHexapod
                obj.hexRenderCounter = obj.hexRenderCounter + 1;
                if mod(obj.hexRenderCounter, 2) == 0 && isvalid(obj.UIFigure)
                    set(obj.HexTopPlate, 'XData', Top_Points(1,:), 'YData', Top_Points(2,:), 'ZData', Top_Points(3,:));
                    for j = 1:6, set(obj.HexLegs(j), 'XData', [obj.hexParams.B(1,j), Top_Points(1,j)], 'YData', [obj.hexParams.B(2,j), Top_Points(2,j)], 'ZData', [obj.hexParams.B(3,j), Top_Points(3,j)]); end
                    laser_end = [0;0;obj.hexParams.Z_nom] + u_opt * 2.0; 
                    set(obj.HexLaser, 'XData', [0, laser_end(1)], 'YData', [0, laser_end(2)], 'ZData', [obj.hexParams.Z_nom, laser_end(3)]);
                    set(obj.HexWarningText, 'String', warn_str, 'BackgroundColor', warn_col);
                    set(obj.HexTelemetryText, 'String', sprintf('Jacobian Cond #: %.1f\nActuator Forces (N) [Lim: %d]:\nL1: %8.1f   L4: %8.1f\nL2: %8.1f   L5: %8.1f\nL3: %8.1f   L6: %8.1f', cond_J, obj.hexParams.F_limit, F_act(1), F_act(4), F_act(2), F_act(5), F_act(3), F_act(6)));
                end
            end
        end
        
        function [F_act, Top_Points, l_cmd, u_opt, cond_J] = compute_hexapod_state(obj, Pt, V_t, A_t)
            T = [0; 0; obj.hexParams.Z_nom]; p = obj.hexParams;
            v = Pt - T; v_mag = max(0.1, norm(v)); u_opt = v / v_mag;
            
            % Mathematical Safeguard: Clamp downward vectoring to prevent horizon flip singularity
            if u_opt(3) < 0.05, u_opt(3) = 0.05; u_opt = u_opt / norm(u_opt); end
            
            v_dot = dot(u_opt, V_t); u_dot = (V_t - u_opt * v_dot) / v_mag;
            v_ddot = dot(u_dot, V_t) + dot(u_opt, A_t); u_ddot = ((A_t - (u_dot * v_dot + u_opt * v_ddot)) * v_mag - (V_t - u_opt * v_dot) * v_dot) / (v_mag^2);

            K_amp = 4.0; omega_mech = cross(u_opt, u_dot) / K_amp; omega_dot_mech = cross(u_opt, u_ddot) / K_amp; 
            axis_opt = cross([0;0;1], u_opt);
            if norm(axis_opt) < 1e-6, axis_opt = [1;0;0]; theta_opt = 0; else, axis_opt = axis_opt / norm(axis_opt); theta_opt = acos(max(-1, min(1, u_opt(3)))); end
            theta_mech = theta_opt / K_amp;
            K_mat = [0, -axis_opt(3), axis_opt(2); axis_opt(3), 0, -axis_opt(1); -axis_opt(2), axis_opt(1), 0];
            R_mech = eye(3) + sin(theta_mech) * K_mat + (1 - cos(theta_mech)) * (K_mat^2);

            I_in = R_mech * p.I_local * R_mech';
            Wrench = [p.m_top * -p.g_vec; I_in * omega_dot_mech + cross(omega_mech, I_in * omega_mech)];

            J = zeros(6, 6); Top_Points = zeros(3, 6); l_cmd = zeros(6, 1);
            for i = 1:6
                P_in = R_mech * p.P(:, i); Top_Points(:, i) = T + P_in; L_i = Top_Points(:, i) - p.B(:, i);
                l_cmd(i) = norm(L_i); s_i = L_i / l_cmd(i); J(i, 1:3) = s_i'; J(i, 4:6) = cross(P_in, s_i)'; 
            end
            cond_J = cond(J);
            if cond_J > 1e4, F_act = zeros(6,1); else, F_act = J' \ Wrench; end
        end
        
        % --- EFFECTOR B: EXACT AZ/EL GIMBAL ---
        function stepAzElEffector(obj, target_cmd, sim_dt)
            azel_dt = 0.01; steps = round(sim_dt / azel_dt);
            
            for k = 1:steps
                v_target = target_cmd - [0; 0; 3.25];
                q_az_cmd = atan2(v_target(2), v_target(1));
                q_el_cmd = atan2(v_target(3), sqrt(v_target(1)^2 + v_target(2)^2));
                
                q_az = obj.azelState.q(1); q_el = obj.azelState.q(2);
                q_dot_az = obj.azelState.q_dot(1); q_dot_el = obj.azelState.q_dot(2);
                
                err_az = mod(q_az_cmd - q_az + pi, 2*pi) - pi; 
                err_el = q_el_cmd - q_el; 
                
                % --- OPTIMIZATION: Zenith Keyhole Gimbal Lock Avoidance ---
                % Dramatically scales down azimuthal error response if target flies 
                % directly overhead (near 90 deg elevation) to prevent infinite torque.
                cos_el = cos(q_el);
                az_dampen = min(1.0, abs(cos_el) / 0.1); 
                err_az = err_az * az_dampen;
                
                q_ddot_az_req = 5.0^2 * err_az - 2 * 1.0 * 5.0 * q_dot_az;
                q_ddot_el_req = 5.0^2 * err_el - 2 * 1.0 * 5.0 * q_dot_el;
                
                p = obj.azelParams;
                I_eff_az = p.J_z + p.I_x * sin(q_el)^2 + p.I_z * cos(q_el)^2;
                C_az = (p.I_x - p.I_z) * sin(2 * q_el) * q_dot_az * q_dot_el;
                C_el = 0.5 * (p.I_x - p.I_z) * sin(2 * q_el) * q_dot_az^2;
                
                tau_req_az = I_eff_az * q_ddot_az_req + C_az;
                tau_req_el = p.I_y * q_ddot_el_req - C_el;
                
                warn_str = 'SYSTEM NOMINAL'; warn_col = 'g';
                tau_act_az = max(-p.Tau_max_az, min(p.Tau_max_az, tau_req_az));
                tau_act_el = max(-p.Tau_max_el, min(p.Tau_max_el, tau_req_el));
                if abs(tau_req_az) > p.Tau_max_az || abs(tau_req_el) > p.Tau_max_el, warn_str = 'ERR: TORQUE SATURATION'; warn_col = 'y'; end
                
                q_ddot_az_act = (tau_act_az - C_az) / I_eff_az;
                q_ddot_el_act = (tau_act_el + C_el) / p.I_y;
                
                if abs(q_dot_az) >= p.Max_Vel && sign(q_ddot_az_act) == sign(q_dot_az), q_ddot_az_act = 0; warn_str = 'ERR: MAX PAN VELOCITY'; warn_col = 'r'; end
                if abs(q_dot_el) >= p.Max_Vel && sign(q_ddot_el_act) == sign(q_dot_el), q_ddot_el_act = 0; warn_str = 'ERR: MAX TILT VELOCITY'; warn_col = 'r'; end
                
                obj.azelState.q_dot(1) = obj.azelState.q_dot(1) + q_ddot_az_act * azel_dt; obj.azelState.q_dot(2) = obj.azelState.q_dot(2) + q_ddot_el_act * azel_dt;
                obj.azelState.q(1) = mod(obj.azelState.q(1) + obj.azelState.q_dot(1) * azel_dt + pi, 2*pi) - pi;
                obj.azelState.q(2) = obj.azelState.q(2) + obj.azelState.q_dot(2) * azel_dt;
            end
            
            if obj.TabGroup.SelectedTab == obj.TabAzEl
                obj.azelRenderCounter = obj.azelRenderCounter + 1;
                if mod(obj.azelRenderCounter, 2) == 0 && isvalid(obj.UIFigure)
                    set(obj.TAz, 'Matrix', makehgtform('zrotate', obj.azelState.q(1)));
                    set(obj.TEl, 'Matrix', makehgtform('translate', [0, 0, 3.25], 'yrotate', -obj.azelState.q(2)));
                    set(obj.AzElWarningText, 'String', warn_str, 'BackgroundColor', warn_col);
                    set(obj.AzElTelemetryText, 'String', sprintf('Eff Inertia:  %6.2f kgm^2\nCoriolis C_az: %+6.1f Nm\nCentrif  C_el: %+6.1f Nm\n\nTau Azimuth:   %+6.1f Nm\nTau Elevation: %+6.1f Nm', I_eff_az, C_az, C_el, tau_act_az, tau_act_el));
                end
            end
        end
    end
end