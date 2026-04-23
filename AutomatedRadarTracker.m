classdef AutomatedRadarTracker < handle
    % AutomatedRadarTracker - Interactive GUI for Radar System Optimization,
    % 3D Search & Track, MPAR, MIMO, Dual-Polarization, and Data Analysis.
    %
    % Save this file as 'AutomatedRadarTracker.m' and run it from the Command Window.

    properties (Access = private)
        % UI Components
        UIFigure
        TargetTable
        BtnAddTarget
        BtnRemTarget
        BtnRun
        BtnStop
        TxtOptLog
        EstTable       
        
        % Tabs for Visualizations
        TabGroup
        TabSensor
        TabOperator
        
        % Radar & DSP Settings
        DropRadarMode
        CheckMIMO
        CheckMF  
        CheckDF  
        CheckAF  
        CheckFD  
        EditChirpBW
        CheckRealTime
        
        % Axes
        Ax3D
        AxMeas3D       
        AxPPI
        AxRangeDoppler2D 
        AxRangeDoppler   
        AxRangeError
        AxVelError
        AxPol          % NEW: Dual-Pol Signature Bar Chart
        AxCrossSection % NEW: Estimated 2D Cross-Section Shape
        
        % Graphics Objects
        beamConeSurf
        hPolBars
        hCrossShape
        
        % Persistent Graphics Handles (Memory Optimization)
        hErrRange
        hErrVel
        hMeasRaw
        hMeasLine
        hMeasEst
        
        % Simulation State
        simRunning = false;
        
        % Constants
        c = 299792458; % Speed of light (m/s)
        kT0 = 1.38e-23 * 290; % Boltzmann * Temp
    end
    
    methods
        function obj = AutomatedRadarTracker()
            obj.buildUI();
        end
    end
    
    methods (Access = private)
        function buildUI(obj)
            % Create main figure window
            obj.UIFigure = figure('Name', 'Advanced Search & Track Radar Simulation', ...
                                  'Position', [50, 50, 1600, 900], ...
                                  'MenuBar', 'none', 'NumberTitle', 'off', ...
                                  'Color', 'w', 'CloseRequestFcn', @obj.onClose);
                              
            % =========================================================
            % --- LEFT PANEL: CONTROLS & INPUTS ---
            % =========================================================
            pnlControls = uipanel(obj.UIFigure, 'Title', '1. Scenario Setup & Targets', ...
                                  'Position', [0.01, 0.01, 0.28, 0.98], 'FontWeight', 'bold');
                              
            uicontrol('Parent', pnlControls, 'Style', 'text', ...
                      'String', 'Define incoming threats. The optimizer will design the radar to handle the worst-case parameters automatically.', ...
                      'Units', 'normalized', 'Position', [0.02, 0.92, 0.96, 0.06], 'HorizontalAlignment', 'left', 'FontSize', 10);
            
            % Target Data Table
            initialData = {
                'UAV (Stealth)',   0.1,  30,  20,  0.5,  -30,  -20,   0;
                'Airliner',       50.0, -60,  50, 10.0,  200, -150,   0;
                'Missile (Fast)',  0.5,  10,  70,  5.0, -100, -800, -50
            };
            colNames = {'Name', 'RCS (m^2)', 'X (km)', 'Y (km)', 'Z (km)', 'Vx (m/s)', 'Vy (m/s)', 'Vz (m/s)'};
            colFmt = {'char', 'numeric', 'numeric', 'numeric', 'numeric', 'numeric', 'numeric', 'numeric'};
            
            obj.TargetTable = uitable('Parent', pnlControls, 'Data', initialData, ...
                                      'ColumnName', colNames, 'ColumnFormat', colFmt, ...
                                      'ColumnEditable', true, ...
                                      'Units', 'normalized', 'Position', [0.02, 0.76, 0.96, 0.16], 'RowName', []);
                                  
            % Add/Remove Target Buttons
            obj.BtnAddTarget = uicontrol('Parent', pnlControls, 'Style', 'pushbutton', 'String', '+ Target', ...
                                         'Units', 'normalized', 'Position', [0.02, 0.72, 0.45, 0.03], 'Callback', @(s,e) obj.addTarget());
            obj.BtnRemTarget = uicontrol('Parent', pnlControls, 'Style', 'pushbutton', 'String', '- Target', ...
                                         'Units', 'normalized', 'Position', [0.53, 0.72, 0.45, 0.03], 'Callback', @(s,e) obj.remTarget());
                                  
            % Radar Architecture Options
            uicontrol('Parent', pnlControls, 'Style', 'text', 'String', 'Radar Operating Mode:', ...
                      'Units', 'normalized', 'Position', [0.02, 0.67, 0.4, 0.03], 'HorizontalAlignment', 'right', 'FontSize', 10);
            obj.DropRadarMode = uicontrol('Parent', pnlControls, 'Style', 'popupmenu', ...
                                      'String', {'Mechanical Sweep', 'AESA (Sequential Scan)', 'AESA (MPAR Search & Track)'}, ...
                                      'Units', 'normalized', 'Position', [0.45, 0.67, 0.53, 0.03], 'FontWeight', 'bold', 'Value', 3);
                                  
            obj.CheckMIMO = uicontrol('Parent', pnlControls, 'Style', 'checkbox', 'String', 'Enable MIMO Virtual Array Processing', ...
                                      'Units', 'normalized', 'Position', [0.05, 0.63, 0.9, 0.03], 'FontWeight', 'bold', 'ForegroundColor', [0 0.4 0.8]);
                                  
            % Signal Processing Suite
            pnlDSP = uipanel(pnlControls, 'Title', 'Signal Processing Techniques', ...
                             'Position', [0.02, 0.44, 0.96, 0.18], 'FontWeight', 'bold');
            obj.CheckMF = uicontrol('Parent', pnlDSP, 'Style', 'checkbox', 'String', 'Matched Filter (Pulse Compression)', 'Units', 'normalized', 'Position', [0.05, 0.75, 0.9, 0.2], 'Value', 1);
            obj.CheckDF = uicontrol('Parent', pnlDSP, 'Style', 'checkbox', 'String', 'Doppler Filter (MTI/MTD)', 'Units', 'normalized', 'Position', [0.05, 0.50, 0.9, 0.2], 'Value', 1);
            obj.CheckAF = uicontrol('Parent', pnlDSP, 'Style', 'checkbox', 'String', 'Adaptive Filter (STAP Clutter Suppression)', 'Units', 'normalized', 'Position', [0.05, 0.25, 0.9, 0.2], 'Value', 1);
            obj.CheckFD = uicontrol('Parent', pnlDSP, 'Style', 'checkbox', 'String', 'Frequency Domain (Coherent Pulse Integration)', 'Units', 'normalized', 'Position', [0.05, 0.00, 0.9, 0.2], 'Value', 1);
            
            uicontrol('Parent', pnlControls, 'Style', 'text', 'String', 'LFM Chirp BW (MHz):', ...
                      'Units', 'normalized', 'Position', [0.02, 0.40, 0.45, 0.03], 'HorizontalAlignment', 'right', 'FontSize', 10);
            obj.EditChirpBW = uicontrol('Parent', pnlControls, 'Style', 'edit', 'String', '5', ...
                                      'Units', 'normalized', 'Position', [0.5, 0.40, 0.2, 0.03], 'FontSize', 10);
                                  
            obj.CheckRealTime = uicontrol('Parent', pnlControls, 'Style', 'checkbox', 'String', 'Sync Sim to 1x Real-Time', ...
                                      'Units', 'normalized', 'Position', [0.05, 0.36, 0.9, 0.03], 'FontWeight', 'bold', 'Value', 1);

            % Run / Stop Buttons
            obj.BtnRun = uicontrol('Parent', pnlControls, 'Style', 'pushbutton', 'String', '▶ Optimize & Run', ...
                                   'Units', 'normalized', 'Position', [0.02, 0.30, 0.65, 0.05], ...
                                   'BackgroundColor', [0 0.5 0], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'FontSize', 11, ...
                                   'Callback', @(src,event) obj.startSimulation());
            obj.BtnStop = uicontrol('Parent', pnlControls, 'Style', 'pushbutton', 'String', '■ Stop', ...
                                   'Units', 'normalized', 'Position', [0.69, 0.30, 0.29, 0.05], ...
                                   'BackgroundColor', [0.8 0 0], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'FontSize', 11, ...
                                   'Callback', @(src,event) obj.stopSimulation());
            
            % Log Area
            pnlLog = uipanel(pnlControls, 'Title', '2. Optimizer & Capability Limits', ...
                             'Position', [0.02, 0.01, 0.96, 0.28], 'FontWeight', 'bold');
            obj.TxtOptLog = uicontrol('Parent', pnlLog, 'Style', 'edit', 'String', 'Awaiting run command...', ...
                                      'Max', 2, 'Min', 0, 'Enable', 'inactive', ... % Allows scrolling!
                                      'Units', 'normalized', 'Position', [0.02, 0.02, 0.96, 0.96], ...
                                      'HorizontalAlignment', 'left', 'FontSize', 9, 'FontName', 'Courier New');
                                  
            % =========================================================
            % --- RIGHT PANELS: TABBED VISUALIZATIONS ---
            % =========================================================
            obj.TabGroup = uitabgroup(obj.UIFigure, 'Position', [0.30, 0.01, 0.69, 0.98]);
            
            % --- TAB 1: SENSOR & SIGNAL VIEW ---
            obj.TabSensor = uitab(obj.TabGroup, 'Title', '1. Sensor & Signal Processing View');
            
            obj.Ax3D = axes('Parent', obj.TabSensor, 'Position', [0.05, 0.55, 0.40, 0.40]);
            title(obj.Ax3D, 'True 3D Operational Airspace (God''s Eye)');
            xlabel(obj.Ax3D, 'East (m)'); ylabel(obj.Ax3D, 'North (m)'); zlabel(obj.Ax3D, 'Alt (m)');
            grid(obj.Ax3D, 'on'); view(obj.Ax3D, 3);
            
            obj.AxRangeDoppler = axes('Parent', obj.TabSensor, 'Position', [0.55, 0.55, 0.40, 0.40]);
            title(obj.AxRangeDoppler, '3D RD-Time Map (Waterfall)');
            xlabel(obj.AxRangeDoppler, 'Range (km)'); ylabel(obj.AxRangeDoppler, 'Velocity'); zlabel(obj.AxRangeDoppler, 'Time');
            grid(obj.AxRangeDoppler, 'on'); view(obj.AxRangeDoppler, 3);
            set(obj.AxRangeDoppler, 'Color', [0 0.1 0.1], 'GridColor', [1 1 1], 'GridAlpha', 0.2); 
            
            obj.AxPPI = axes('Parent', obj.TabSensor, 'Position', [0.05, 0.05, 0.40, 0.40]);
            title(obj.AxPPI, 'PPI Radar Display (Raw Hits)');
            set(obj.AxPPI, 'Color', [0 0.1 0], 'XColor', 'none', 'YColor', 'none'); 
            
            obj.AxRangeDoppler2D = axes('Parent', obj.TabSensor, 'Position', [0.55, 0.05, 0.40, 0.40]);
            title(obj.AxRangeDoppler2D, '2D Range-Doppler Map');
            xlabel(obj.AxRangeDoppler2D, 'Range (km)'); ylabel(obj.AxRangeDoppler2D, 'Radial Vel (m/s)');
            grid(obj.AxRangeDoppler2D, 'on');
            set(obj.AxRangeDoppler2D, 'Color', [0 0.1 0.1], 'GridColor', [1 1 1], 'GridAlpha', 0.2);
            
            % --- TAB 2: OPERATOR C2 VIEW ---
            obj.TabOperator = uitab(obj.TabGroup, 'Title', '2. Operator C2 (Command & Control) View');
            
            % Measured 3D Plot
            obj.AxMeas3D = axes('Parent', obj.TabOperator, 'Position', [0.02, 0.35, 0.45, 0.60]);
            title(obj.AxMeas3D, 'Measured 3D Space & Trajectory Estimations');
            xlabel(obj.AxMeas3D, 'East (m)'); ylabel(obj.AxMeas3D, 'North (m)'); zlabel(obj.AxMeas3D, 'Alt (m)');
            grid(obj.AxMeas3D, 'on'); view(obj.AxMeas3D, 3);
            
            % Dual Pol Analysis Plots
            obj.AxPol = axes('Parent', obj.TabOperator, 'Position', [0.52, 0.68, 0.20, 0.27]);
            title(obj.AxPol, 'Dual-Pol Signature'); ylabel(obj.AxPol, 'Measured RCS (m^2)'); grid(obj.AxPol, 'on');
            set(obj.AxPol, 'XTick', 1:3, 'XTickLabel', {'HH', 'VV', 'HV'});
            
            obj.AxCrossSection = axes('Parent', obj.TabOperator, 'Position', [0.76, 0.68, 0.20, 0.27]);
            title(obj.AxCrossSection, 'Estimated Target Silhouette');
            axis(obj.AxCrossSection, 'equal'); set(obj.AxCrossSection, 'XTick', [], 'YTick', []);
            xlim(obj.AxCrossSection, [-10 10]); ylim(obj.AxCrossSection, [-10 10]);
            
            % Error Analysis Plots
            obj.AxRangeError = axes('Parent', obj.TabOperator, 'Position', [0.52, 0.35, 0.20, 0.25]);
            title(obj.AxRangeError, 'Range Error (m)'); grid(obj.AxRangeError, 'on');
            
            obj.AxVelError = axes('Parent', obj.TabOperator, 'Position', [0.76, 0.35, 0.20, 0.25]);
            title(obj.AxVelError, 'Velocity Error (m/s)'); grid(obj.AxVelError, 'on');
            
            % Target Estimation Table
            estColNames = {'Track ID', 'Rng (km)', 'Alt (m)', 'Spd (m/s)', 'Hdg (deg)', 'Est RCS (m^2)', 'Shape Profile', 'Class', 'TTI (s)'};
            obj.EstTable = uitable('Parent', obj.TabOperator, 'ColumnName', estColNames, ...
                                   'ColumnWidth', {60, 60, 60, 70, 60, 85, 120, 130, 60}, ...
                                   'Units', 'normalized', 'Position', [0.02, 0.02, 0.96, 0.28], 'RowName', []);
        end
        
        function addTarget(obj)
            data = obj.TargetTable.Data;
            newRow = {'New Threat', 1.0, 40, 40, 5, -150, -150, 0};
            obj.TargetTable.Data = [data; newRow];
        end
        
        function remTarget(obj)
            data = obj.TargetTable.Data;
            if size(data, 1) > 1
                obj.TargetTable.Data = data(1:end-1, :);
            end
        end
        
        function stopSimulation(obj)
            obj.simRunning = false;
        end
        
        function onClose(obj, ~, ~)
            obj.simRunning = false;
            delete(obj.UIFigure);
        end
        
        function startSimulation(obj)
            if obj.simRunning
                return; 
            end
            
            obj.simRunning = true;
            obj.BtnRun.Enable = 'off'; 
            
            %% --- 1. PARSE TARGET DATA ---
            data = obj.TargetTable.Data;
            numRows = size(data, 1);
            targets = struct('name', {}, 'rcs', {}, 'pos', {}, 'vel', {}, 'shapePol', {});
            
            tIdx = 1;
            for i = 1:numRows
                if isempty(data{i,1}) || (isstring(data{i,1}) && data{i,1} == "")
                    continue; 
                end
                targets(tIdx).name = data{i,1};
                targets(tIdx).rcs  = data{i,2};
                targets(tIdx).pos  = [data{i,3}, data{i,4}, data{i,5}] * 1000;
                targets(tIdx).vel  = [data{i,6}, data{i,7}, data{i,8}];
                
                % Deduce Physical Shape for Polarization from Name
                nameLower = lower(targets(tIdx).name);
                if contains(nameLower, 'airliner') || contains(nameLower, 'wing')
                    targets(tIdx).shapePol = 3.0; % Horizontal (HH dominant)
                elseif contains(nameLower, 'missile') || contains(nameLower, 'rocket')
                    targets(tIdx).shapePol = 0.33; % Vertical (VV dominant)
                else
                    targets(tIdx).shapePol = 1.0; % Symmetric (UAV/Sphere)
                end
                
                tIdx = tIdx + 1;
            end
            numTargets = length(targets);
            
            if numTargets == 0
                obj.TxtOptLog.String = 'Error: No valid targets.';
                obj.stopSimulation();
                obj.BtnRun.Enable = 'on'; 
                return;
            end
            
            %% --- 2. AUTOMATED OPTIMIZATION & LIMITS ---
            logStr = sprintf('--- RADAR SYSTEM OPTIMIZER ---\n\n');
            
            maxRangeNeeded = 0; maxSpeedNeeded = 0;
            for i = 1:numTargets
                r = norm(targets(i).pos);
                v = norm(targets(i).vel);
                if r > maxRangeNeeded, maxRangeNeeded = r; end
                if v > maxSpeedNeeded, maxSpeedNeeded = v; end
            end
            R_max_design = maxRangeNeeded * 1.2; 
            
            PRF_opt = obj.c / (2 * R_max_design);
            BW_MHz = str2double(obj.EditChirpBW.String);
            if isnan(BW_MHz), BW_MHz = 5; end
            BW = BW_MHz * 1e6;
            
            logStr = [logStr, sprintf('1. Target Geometry limits:\n')];
            logStr = [logStr, sprintf('   -> R_max set to %.1f km.\n', R_max_design/1000)];
            logStr = [logStr, sprintf('   -> Optimal PRF: %.2f kHz.\n\n', PRF_opt/1000)];
            
            revisit_time = 2.0; 
            scan_rate_deg_per_sec = 360 / revisit_time;
            RPM_opt = scan_rate_deg_per_sec / 6;
            beamwidth_deg = 5;
            T_dwell = beamwidth_deg / scan_rate_deg_per_sec; 
            
            req_SNR_dB = 13; 
            req_SNR = 10^(req_SNR_dB/10);
            f = 3e9; lambda = obj.c/f; 
            G_dB = 35; G = 10^(G_dB/10); 
            NoiseF = 10^(3/10); 
            tau = 10e-6;
            
            % DSP Gain Modifiers
            PG_total = 1;
            if obj.CheckMF.Value, PG_total = PG_total * (BW * tau); end
            if obj.CheckFD.Value, PG_total = PG_total * 16; end 
            if obj.CheckAF.Value, PG_total = PG_total * 10; end 
            
            worst_power_needed = 0;
            hardest_target = 'Unknown';
            for i = 1:numTargets
                r = norm(targets(i).pos);
                sigma = targets(i).rcs;
                Pt_req = (req_SNR * ((4*pi)^3) * (r^4) * obj.kT0 * BW * NoiseF) / (G^2 * lambda^2 * sigma * PG_total);
                if Pt_req > worst_power_needed
                    worst_power_needed = Pt_req;
                    hardest_target = targets(i).name;
                end
            end
            Pt_opt = worst_power_needed * 1.5; 
            
            logStr = [logStr, sprintf('2. Power & Link Budget:\n')];
            logStr = [logStr, sprintf('   -> Hardest target: %s\n', hardest_target)];
            logStr = [logStr, sprintf('   -> Required Peak Power: %.1f kW.\n\n', Pt_opt/1000)];
            
            min_rcs = (req_SNR * ((4*pi)^3) * (R_max_design^4) * obj.kT0 * BW * NoiseF) / (Pt_opt * G^2 * lambda^2 * PG_total);
            v_blind = (lambda * PRF_opt) / 4; 
            
            logStr = [logStr, sprintf('3. CAPABILITY LIMITS (Evasion Profile):\n')];
            logStr = [logStr, sprintf('   Target parameters OUTSIDE radar capabilities:\n')];
            logStr = [logStr, sprintf('   -> RCS < %.4f m^2 at %.1f km will EVADE detection.\n', min_rcs, R_max_design/1000)];
            logStr = [logStr, sprintf('   -> Velocity > %.1f m/s (Mach %.1f) causes Doppler Aliasing blinding.\n', v_blind, v_blind/343)];
            
            obj.TxtOptLog.String = logStr;
            
            %% --- 3. BUILD 3D GEOMETRIC ENVIRONMENT ---
            cla(obj.Ax3D); cla(obj.AxMeas3D); cla(obj.AxPPI); 
            cla(obj.AxRangeDoppler2D); cla(obj.AxRangeDoppler); 
            cla(obj.AxRangeError); cla(obj.AxVelError); 
            cla(obj.AxPol); cla(obj.AxCrossSection);
            
            hold(obj.Ax3D, 'on'); hold(obj.AxMeas3D, 'on'); hold(obj.AxPPI, 'on'); 
            hold(obj.AxRangeDoppler2D, 'on'); hold(obj.AxRangeDoppler, 'on');
            hold(obj.AxRangeError, 'on'); hold(obj.AxVelError, 'on'); 
            hold(obj.AxPol, 'on'); hold(obj.AxCrossSection, 'on');
            
            % Setup Dual-Pol Visualization Handles
            obj.hPolBars = bar(obj.AxPol, categorical({'HH', 'VV', 'HV'}), [0 0 0], 'FaceColor', [0.2 0.6 0.8]);
            obj.hCrossShape = rectangle('Parent', obj.AxCrossSection, 'Position', [-1 -1 2 2], 'Curvature', [1 1], 'FaceColor', 'y', 'EdgeColor', 'k');
            axis(obj.AxCrossSection, 'equal'); xlim(obj.AxCrossSection, [-10 10]); ylim(obj.AxCrossSection, [-10 10]);
            
            % 3D View Limits (True & Measured)
            lim = R_max_design;
            axis(obj.Ax3D, 'equal'); xlim(obj.Ax3D, [-lim lim]); ylim(obj.Ax3D, [-lim lim]); zlim(obj.Ax3D, [0 lim/2]);
            axis(obj.AxMeas3D, 'equal'); xlim(obj.AxMeas3D, [-lim lim]); ylim(obj.AxMeas3D, [-lim lim]); zlim(obj.AxMeas3D, [0 lim/2]);
            
            % PPI Limits & Grid
            axis(obj.AxPPI, 'equal');
            xlim(obj.AxPPI, [-lim lim]); ylim(obj.AxPPI, [-lim lim]);
            
            theta_circle = linspace(0, 2*pi, 100);
            for ring_R = [0.25, 0.5, 0.75, 1.0] * R_max_design
                plot(obj.AxPPI, ring_R*cos(theta_circle), ring_R*sin(theta_circle), 'Color', [0 0.4 0], 'LineStyle', '--');
                text(obj.AxPPI, ring_R*cos(pi/4), ring_R*sin(pi/4), sprintf('%.0fkm', ring_R/1000), 'Color', [0 0.6 0], 'FontSize', 8, 'FontWeight', 'bold');
            end
            
            % Range-Doppler Limits
            xlim(obj.AxRangeDoppler2D, [0, lim/1000]); xlim(obj.AxRangeDoppler, [0, lim/1000]);
            v_lim = max(100, maxSpeedNeeded * 1.5);
            ylim(obj.AxRangeDoppler2D, [-v_lim, v_lim]); ylim(obj.AxRangeDoppler, [-v_lim, v_lim]);
            yline(obj.AxRangeDoppler2D, 0, 'Color', [0.4 0.4 0.4], 'LineStyle', '--');
            zlim(obj.AxRangeDoppler, [0, 15]); 
            
            % Radar Base
            radarBase = hgtransform('Parent', obj.Ax3D);
            patch('Parent', radarBase, 'XData', [0 0 0 0], 'YData', [-500 500 500 -500], 'ZData', [-500 -500 500 500], ...
                  'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k');
              
            beamRadius = lim * tand(beamwidth_deg/2);
            [bX, bY, bZ] = cylinder([0, beamRadius], 20);
            obj.beamConeSurf = surface('XData', bZ * lim, 'YData', bX, 'ZData', bY, ...
                    'FaceColor', 'c', 'EdgeColor', 'none', 'FaceAlpha', 0.15, 'Parent', radarBase);
                
            targetTransforms = gobjects(numTargets, 1);
            colorPalette = lines(numTargets); 
            
            for i = 1:numTargets
                targetTransforms(i) = hgtransform('Parent', obj.Ax3D);
                scale = log10(targets(i).rcs + 1) * 800 + 800; 
                V = [scale 0 0; -scale -scale -scale; -scale scale -scale; -scale 0 scale]; 
                PatchFaces = [1 2 3; 1 3 4; 1 4 2; 2 3 4]; 
                patch('Parent', targetTransforms(i), 'Vertices', V, 'Faces', PatchFaces, ...
                      'FaceColor', colorPalette(i,:), 'EdgeColor', 'k', 'FaceAlpha', 0.8);
                targets(i).textObj = text(obj.Ax3D, 0, 0, 0, targets(i).name, 'FontSize', 8, 'Color', 'k', 'FontWeight', 'bold');
            end
            
            % Setup PPI & RD Fading Phosphor 
            sweepLine = plot(obj.AxPPI, [0, lim], [0, 0], 'Color', [0 1 0], 'LineWidth', 2);
            maxBlipsPPI = 100;
            blipsX = zeros(maxBlipsPPI, 1); blipsY = zeros(maxBlipsPPI, 1); blipsAge = ones(maxBlipsPPI, 1) * 100; 
            blipsScatter = scatter(obj.AxPPI, blipsX, blipsY, 60, 'filled', 'MarkerFaceColor', [0 1 0]);
            
            maxBlipsRD = 500;
            rdX = nan(maxBlipsRD, 1); rdY = nan(maxBlipsRD, 1); rdZ = nan(maxBlipsRD, 1);
            rdColor = repmat([0 0 0], maxBlipsRD, 1); 
            rdAge = ones(maxBlipsRD, 1) * 100; 
            
            rdScatter2D = scatter(obj.AxRangeDoppler2D, rdX, rdY, 40, rdColor, 'filled');
            rdScatter3D = scatter3(obj.AxRangeDoppler, rdX, rdY, rdZ, 40, rdColor, 'filled');
            
            blipIdx = 1;
            rdIdx = 1;
            
            % History Arrays for Tracking & Dual-Pol
            hist_time = []; hist_R_err = []; hist_V_err = []; hist_T_idx = [];
            hist_meas_X = []; hist_meas_Y = []; hist_meas_Z = []; hist_SNR = [];
            hist_meas_HH = []; hist_meas_VV = []; hist_meas_HV = [];

            % --- PERSISTENT GRAPHICS INITIALIZATION ---
            obj.hErrRange = gobjects(numTargets, 1);
            obj.hErrVel = gobjects(numTargets, 1);
            obj.hMeasRaw = gobjects(numTargets, 1);
            obj.hMeasLine = gobjects(numTargets, 1);
            obj.hMeasEst = gobjects(numTargets, 1);
            
            % Draw static base patch for AxMeas3D once
            patch('Parent', obj.AxMeas3D, 'XData', [0 0 0 0], 'YData', [-500 500 500 -500], 'ZData', [-500 -500 500 500], 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k');
            
            for i = 1:numTargets
                obj.hErrRange(i) = scatter(obj.AxRangeError, NaN, NaN, 36, colorPalette(i,:), 'filled', 'DisplayName', targets(i).name);
                obj.hErrVel(i) = scatter(obj.AxVelError, NaN, NaN, 36, colorPalette(i,:), 'filled', 'DisplayName', targets(i).name);
                obj.hMeasRaw(i) = scatter3(obj.AxMeas3D, NaN, NaN, NaN, 15, colorPalette(i,:), 'filled');
                obj.hMeasLine(i) = plot3(obj.AxMeas3D, NaN, NaN, NaN, '--', 'Color', colorPalette(i,:), 'LineWidth', 2);
                obj.hMeasEst(i) = scatter3(obj.AxMeas3D, NaN, NaN, NaN, 80, 's', 'MarkerEdgeColor', colorPalette(i,:), 'LineWidth', 2);
            end
            legend(obj.AxRangeError, 'Location', 'best');
            legend(obj.AxVelError, 'Location', 'best');

            %% --- 4. MAIN SIMULATION LOOP ---
            sim_dt = 0.1; sim_time = 0;
            last_search_az_deg = 0;
            tracked_targets = false(numTargets, 1);
            mpar_tick = 0;
            radar_mode = obj.DropRadarMode.Value;
            useMIMO = obj.CheckMIMO.Value;
            
            while obj.simRunning && isvalid(obj.UIFigure)
                loop_start = tic;
                sim_time = sim_time + sim_dt;
                
                if ~isvalid(sweepLine) || ~isvalid(radarBase), break; end
                
                if sim_time > 15
                    zlim(obj.AxRangeDoppler, [sim_time - 15, sim_time]);
                else
                    zlim(obj.AxRangeDoppler, [0, 15]);
                end
                
                % MPAR & Beam Steering
                is_tracking_frame = false;
                curr_tr_idx = -1;
                if radar_mode == 3 && any(tracked_targets)
                    mpar_tick = mpar_tick + 1;
                    if mod(mpar_tick, 4) == 0 
                        is_tracking_frame = true;
                        active_tracks = find(tracked_targets);
                        curr_tr_idx = active_tracks(mod(mpar_tick/4 - 1, length(active_tracks)) + 1);
                    end
                end
                
                if is_tracking_frame
                    tgt_pos = targets(curr_tr_idx).pos;
                    current_azimuth_deg = rad2deg(atan2(tgt_pos(2), tgt_pos(1)));
                    if current_azimuth_deg < 0, current_azimuth_deg = current_azimuth_deg + 360; end
                    obj.beamConeSurf.FaceColor = 'm'; obj.beamConeSurf.FaceAlpha = 0.4; 
                    sweepLine.Color = [1 0 1];
                else
                    obj.beamConeSurf.FaceColor = 'c'; obj.beamConeSurf.FaceAlpha = 0.15;
                    if radar_mode == 1 
                        last_search_az_deg = last_search_az_deg + (scan_rate_deg_per_sec * sim_dt);
                    else
                        last_search_az_deg = floor(sim_time / T_dwell) * beamwidth_deg;
                    end
                    current_azimuth_deg = mod(last_search_az_deg, 360);
                    sweepLine.Color = [0 1 0];
                end
                
                radarBase.Matrix = makehgtform('zrotate', deg2rad(current_azimuth_deg));
                sweepLine.XData = [0, lim * cosd(current_azimuth_deg)];
                sweepLine.YData = [0, lim * sind(current_azimuth_deg)];
                
                % Active track rendering for C2 Tab
                last_illuminated_idx = -1;
                
                for i = 1:numTargets
                    targets(i).pos = targets(i).pos + (targets(i).vel * sim_dt);
                    pos = targets(i).pos; vel = targets(i).vel;
                    
                    heading_rad = atan2(vel(2), vel(1));
                    pitch_rad = atan2(vel(3), norm(vel(1:2)));
                    
                    if isvalid(targetTransforms(i))
                        targetTransforms(i).Matrix = makehgtform('translate', pos, 'zrotate', heading_rad, 'yrotate', -pitch_rad);
                    end
                    if isvalid(targets(i).textObj)
                        targets(i).textObj.Position = pos + [0 0 2000];
                    end
                    
                    target_range = norm(pos);
                    target_az_deg = mod(rad2deg(atan2(pos(2), pos(1))), 360);
                    target_el_deg = rad2deg(asin(pos(3) / target_range)); 
                    true_v_r = -dot(vel, pos / norm(pos)); 
                    
                    d = deg2rad(current_azimuth_deg) - deg2rad(target_az_deg);
                    angle_diff = abs(mod(d + pi, 2*pi) - pi);
                    
                    if angle_diff <= deg2rad(beamwidth_deg/2)
                        snr_linear = (Pt_opt * (G^2) * (lambda^2) * targets(i).rcs) / (((4*pi)^3) * (target_range^4) * obj.kT0 * BW * NoiseF);
                        if obj.CheckMF.Value, snr_linear = snr_linear * (BW * tau); end
                        if obj.CheckFD.Value, snr_linear = snr_linear * 16; end
                        if obj.CheckAF.Value, snr_linear = snr_linear * 10; end 
                        snr_dB = 10 * log10(snr_linear);
                        
                        if snr_dB >= req_SNR_dB
                            tracked_targets(i) = true;
                            last_illuminated_idx = i;
                            
                            eff_dwell = T_dwell;
                            if is_tracking_frame, eff_dwell = T_dwell * 2; end
                            
                            % Measurement Errors
                            if obj.CheckMF.Value, sigma_R = (obj.c / (2 * BW)) / sqrt(2 * snr_linear);
                            else, sigma_R = (obj.c * tau / 2) / sqrt(2 * snr_linear) * 5; end
                            
                            if obj.CheckDF.Value, sigma_V = (lambda / (2 * eff_dwell)) / sqrt(2 * snr_linear);
                            else, sigma_V = (lambda / (2 * eff_dwell)) / sqrt(2 * snr_linear) * 20; end
                            
                            if useMIMO, eff_bw_deg = beamwidth_deg / 4; 
                            else, eff_bw_deg = beamwidth_deg; end
                            
                            sigma_Az = eff_bw_deg / sqrt(2 * snr_linear);
                            sigma_El = eff_bw_deg / sqrt(2 * snr_linear);
                            
                            meas_range = target_range + randn() * sigma_R;
                            meas_vel   = true_v_r + randn() * sigma_V;
                            meas_az    = target_az_deg + randn() * sigma_Az;
                            meas_el    = target_el_deg + randn() * sigma_El;
                            
                            meas_x = meas_range * cosd(meas_el) * cosd(meas_az);
                            meas_y = meas_range * cosd(meas_el) * sind(meas_az);
                            meas_z = meas_range * sind(meas_el);
                            
                            % --- DUAL POLARIZATION MEASUREMENT ---
                            % Calculate True Polarization scattering based on physical shape
                            polRatio = targets(i).shapePol;
                            true_HH = targets(i).rcs * (2 * polRatio / (polRatio + 1));
                            true_VV = targets(i).rcs * (2 * 1 / (polRatio + 1));
                            true_HV = targets(i).rcs * 0.1; % Cross-pol is generally weak
                            
                            % Add thermal noise to the RCS measurements
                            sigma_RCS = targets(i).rcs / sqrt(snr_linear);
                            meas_HH = max(0, true_HH + randn() * sigma_RCS);
                            meas_VV = max(0, true_VV + randn() * sigma_RCS);
                            meas_HV = max(0, true_HV + randn() * sigma_RCS);
                            
                            hist_time(end+1) = sim_time; %#ok<AGROW>
                            hist_R_err(end+1) = meas_range - target_range; %#ok<AGROW>
                            hist_V_err(end+1) = meas_vel - true_v_r; %#ok<AGROW>
                            hist_T_idx(end+1) = i; %#ok<AGROW>
                            hist_meas_X(end+1) = meas_x; %#ok<AGROW>
                            hist_meas_Y(end+1) = meas_y; %#ok<AGROW>
                            hist_meas_Z(end+1) = meas_z; %#ok<AGROW>
                            hist_SNR(end+1) = snr_dB; %#ok<AGROW>
                            hist_meas_HH(end+1) = meas_HH; %#ok<AGROW>
                            hist_meas_VV(end+1) = meas_VV; %#ok<AGROW>
                            hist_meas_HV(end+1) = meas_HV; %#ok<AGROW>
                            
                            % Spawn Blips
                            blipsX(blipIdx) = meas_range * cosd(meas_az);
                            blipsY(blipIdx) = meas_range * sind(meas_az);
                            blipsAge(blipIdx) = 0; 
                            blipIdx = mod(blipIdx, maxBlipsPPI) + 1; 
                            if blipIdx == 0; blipIdx = 1; end
                            
                            rdX(rdIdx) = meas_range / 1000;
                            rdY(rdIdx) = meas_vel;
                            rdZ(rdIdx) = sim_time;
                            rdColor(rdIdx, :) = colorPalette(i, :);
                            rdAge(rdIdx) = 0; 
                            rdIdx = mod(rdIdx, maxBlipsRD) + 1;
                            if rdIdx == 0; rdIdx = 1; end
                            
                            if isvalid(targetTransforms(i))
                                patchObj = findobj(targetTransforms(i), 'Type', 'Patch');
                                patchObj.FaceColor = 'y'; 
                            end
                        else
                            if is_tracking_frame && i == curr_tr_idx
                                tracked_targets(i) = false;
                            end
                        end
                    else
                        if isvalid(targetTransforms(i))
                            patchObj = findobj(targetTransforms(i), 'Type', 'Patch');
                            patchObj.FaceColor = colorPalette(i,:);
                        end
                    end
                end
                
                % --- PRUNE HISTORY (15s Rolling Window) ---
                if ~isempty(hist_time)
                    keep_idx = hist_time >= (sim_time - 15);
                    hist_time = hist_time(keep_idx);
                    hist_R_err = hist_R_err(keep_idx);
                    hist_V_err = hist_V_err(keep_idx);
                    hist_T_idx = hist_T_idx(keep_idx);
                    hist_meas_X = hist_meas_X(keep_idx);
                    hist_meas_Y = hist_meas_Y(keep_idx);
                    hist_meas_Z = hist_meas_Z(keep_idx);
                    hist_SNR = hist_SNR(keep_idx);
                    hist_meas_HH = hist_meas_HH(keep_idx);
                    hist_meas_VV = hist_meas_VV(keep_idx);
                    hist_meas_HV = hist_meas_HV(keep_idx);
                end
                
                % Update Graphs & Extrapolations (Operator C2 View)
                estData = cell(numTargets, 9);
                
                if ~isempty(hist_time)
                    for i = 1:numTargets
                        idx = (hist_T_idx == i);
                        if any(idx)
                            % Errors - Safely update persistent handle data
                            set(obj.hErrRange(i), 'XData', hist_time(idx), 'YData', hist_R_err(idx));
                            set(obj.hErrVel(i), 'XData', hist_time(idx), 'YData', hist_V_err(idx));
                            
                            % Extrapolation & Measurements
                            if sum(idx) > 2
                                t_vals = hist_time(idx);
                                if max(t_vals) - min(t_vals) > 0 
                                    x_vals = hist_meas_X(idx);
                                    y_vals = hist_meas_Y(idx);
                                    z_vals = hist_meas_Z(idx);
                                    
                                    set(obj.hMeasRaw(i), 'XData', x_vals, 'YData', y_vals, 'ZData', z_vals);
                                    
                                    pX = polyfit(t_vals, x_vals, 1);
                                    pY = polyfit(t_vals, y_vals, 1);
                                    pZ = polyfit(t_vals, z_vals, 1);
                                    
                                    t_ext = [min(t_vals), max(t_vals) + 10]; 
                                    x_ext = polyval(pX, t_ext); y_ext = polyval(pY, t_ext); z_ext = polyval(pZ, t_ext);
                                    set(obj.hMeasLine(i), 'XData', x_ext, 'YData', y_ext, 'ZData', z_ext);
                                    
                                    curr_est = [polyval(pX, sim_time), polyval(pY, sim_time), polyval(pZ, sim_time)];
                                    set(obj.hMeasEst(i), 'XData', curr_est(1), 'YData', curr_est(2), 'ZData', curr_est(3));
                                    
                                    % Current Kinematic Estimations
                                    est_vx = pX(1); est_vy = pY(1); est_vz = pZ(1);
                                    est_speed = norm([est_vx, est_vy, est_vz]);
                                    est_heading = mod(rad2deg(atan2(est_vy, est_vx)), 360);
                                    est_rng = norm(curr_est);
                                    est_alt = curr_est(3);
                                    
                                    % Time to Intercept (TTI)
                                    est_v_r = -dot([est_vx, est_vy, est_vz], curr_est / norm(curr_est));
                                    if est_v_r > 0
                                        tti = est_rng / est_v_r;
                                        tti_str = sprintf('%.1f', tti);
                                    else
                                        tti_str = 'N/A'; % Moving away
                                    end
                                    
                                    % Dual Pol Classification Logic & Est RCS
                                    last_snr = hist_SNR(find(idx, 1, 'last'));
                                    est_rcs = (10^(last_snr/10) * ((4*pi)^3) * (est_rng^4) * obj.kT0 * BW * NoiseF) / (Pt_opt * G^2 * lambda^2 * PG_total);
                                    
                                    last_HH = hist_meas_HH(find(idx, 1, 'last'));
                                    last_VV = hist_meas_VV(find(idx, 1, 'last'));
                                    ratio = last_HH / (last_VV + 1e-6);
                                    
                                    if ratio > 2.0, shape_str = 'Horizontal (Winged)';
                                    elseif ratio < 0.5, shape_str = 'Vertical (Cylinder)';
                                    else, shape_str = 'Symmetrical';
                                    end
                                    
                                    if est_speed > 343, class_str = 'Supersonic Missile';
                                    elseif est_speed > 100 && ratio > 2.0, class_str = 'Large Aircraft';
                                    elseif ratio > 0.5 && ratio < 2.0, class_str = 'Stealth/UAV';
                                    else, class_str = 'Unknown';
                                    end
                                    
                                    estData{i, 1} = sprintf('Trk-%d', i);
                                    estData{i, 2} = sprintf('%.2f', est_rng / 1000);
                                    estData{i, 3} = sprintf('%.0f', est_alt);
                                    estData{i, 4} = sprintf('%.1f', est_speed);
                                    estData{i, 5} = sprintf('%.0f', est_heading);
                                    estData{i, 6} = sprintf('%.2f', est_rcs);
                                    estData{i, 7} = shape_str;
                                    estData{i, 8} = class_str;
                                    estData{i, 9} = tti_str;
                                    
                                    % Update Live Dual-Pol Visuals for Currently Illuminated Target
                                    if i == last_illuminated_idx
                                        last_HV = hist_meas_HV(find(idx, 1, 'last'));
                                        set(obj.hPolBars, 'YData', [last_HH, last_VV, last_HV]);
                                        
                                        % Update estimated Cross Section Ellipse
                                        rX = max(0.5, sqrt(last_HH)*2);
                                        rY = max(0.5, sqrt(last_VV)*2);
                                        set(obj.hCrossShape, 'Position', [-rX, -rY, 2*rX, 2*rY], 'FaceColor', colorPalette(i,:));
                                        
                                        xlim(obj.AxCrossSection, [-10 10]); 
                                        ylim(obj.AxCrossSection, [-10 10]);
                                    end
                                else
                                    % Missing Time Spread (Hide Graphics)
                                    set(obj.hMeasRaw(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                                    set(obj.hMeasLine(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                                    set(obj.hMeasEst(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                                end
                            else
                                % Not enough hits (Hide Graphics)
                                set(obj.hMeasRaw(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                                set(obj.hMeasLine(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                                set(obj.hMeasEst(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                            end
                        else
                            % Target Lost/Not tracked (Hide Graphics)
                            set(obj.hErrRange(i), 'XData', NaN, 'YData', NaN);
                            set(obj.hErrVel(i), 'XData', NaN, 'YData', NaN);
                            set(obj.hMeasRaw(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                            set(obj.hMeasLine(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                            set(obj.hMeasEst(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                        end
                    end
                end
                
                % Update C2 Table
                validRows = ~cellfun('isempty', estData(:,1));
                if any(validRows)
                    obj.EstTable.Data = estData(validRows, :);
                end
                
                % Fading effects
                blipsAge = blipsAge + sim_dt;
                alphas = max(0, 1 - (blipsAge / 3.0)); 
                if isvalid(blipsScatter)
                    blipsScatter.XData = blipsX; blipsScatter.YData = blipsY;
                    blipsScatter.CData = [zeros(maxBlipsPPI, 1), alphas, zeros(maxBlipsPPI, 1)];
                end
                
                rdAge = rdAge + sim_dt;
                rdAlphas = max(0, 1 - (rdAge / 3.0));
                fadedRdColor = rdColor .* rdAlphas;
                if isvalid(rdScatter2D)
                    rdScatter2D.XData = rdX; rdScatter2D.YData = rdY; rdScatter2D.CData = fadedRdColor;
                end
                if isvalid(rdScatter3D)
                    rdScatter3D.XData = rdX; rdScatter3D.YData = rdY; rdScatter3D.ZData = rdZ; rdScatter3D.CData = fadedRdColor;
                end
                
                drawnow limitrate;
                
                if obj.CheckRealTime.Value
                    loop_time = toc(loop_start);
                    pause_time = sim_dt - loop_time;
                    if pause_time > 0, pause(pause_time); end
                end
            end
            
            if isvalid(obj.UIFigure), obj.BtnRun.Enable = 'on'; end
        end
    end
end