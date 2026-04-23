classdef IntegratedRadarTool < matlab.apps.AppBase
    
    properties (Access = public)
        UIFigure      matlab.ui.Figure
        MainGrid      matlab.ui.container.GridLayout
        
        % Left Pane (Inputs)
        TabGroupInputs matlab.ui.container.TabGroup
        TabRadar       matlab.ui.container.Tab
        TabEnv         matlab.ui.container.Tab
        TabOpt         matlab.ui.container.Tab
        
        % Input Fields - Radar
        EditRange, EditVel, EditRCS
        EditPt, EditFreq, EditTau, EditPRF
        EditTxN, EditRxN, DropTaper
        
        % Input Fields - Env & Jammer
        EditScanAz, EditScanEl
        EditJamAz, EditJamEl, EditJSR
        CheckMVDR
        
        % Input Fields - Optimizer
        EditReqSNR, EditMaxPt, BtnOpt, TxtOptLog
        
        % Dashboard Labels
        LblSNR, LblMaxR, LblRes, LblAvgPwr
        
        % Right Pane (Visuals)
        TabGroupVisuals matlab.ui.container.TabGroup
        Tab3DWorld      matlab.ui.container.Tab
        TabPattern      matlab.ui.container.Tab
        TabSignal       matlab.ui.container.Tab
        
        % Axes
        Ax3DWorld, AxPat3D, AxCutAz
        AxTime, AxMF, AxRDMap
        
        BtnSimulate     matlab.ui.control.Button
    end
    
    properties (Access = private)
        RadarEngine
        SimTimer
        
        hTarget, hJammer, hScanBeam
        hPatSurf, hCutLine
        hTimeLine, hMFLine, hRDImage
    end
    
    methods (Access = private)
        function startupFcn(app)
            app.RadarEngine = MasterRadarSystem();
            
            % Initialize UI values
            app.EditRange.Value = app.RadarEngine.TargetState.Range / 1000;
            app.EditVel.Value = app.RadarEngine.TargetState.Vel;
            app.EditRCS.Value = app.RadarEngine.TargetState.RCS;
            app.EditPt.Value = app.RadarEngine.HardwareSpecs.Pt / 1000;
            app.EditFreq.Value = app.RadarEngine.HardwareSpecs.Freq / 1e9;
            
            app.SimTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.033, 'TimerFcn', @(~,~) app.stepSimulation());
            
            app.updateCalculations();
        end
        
        function updateCalculations(app)
            %% 1. Metrics Update
            metrics = app.RadarEngine.calcRadarFundamentals();
            app.LblSNR.Text = sprintf('%.1f dB', metrics.SNR_dB);
            app.LblMaxR.Text = sprintf('%.1f km', metrics.R_max / 1000);
            app.LblRes.Text = sprintf('%.1f m', metrics.R_res_comp);
            app.LblAvgPwr.Text = sprintf('%.1f W', metrics.AvgPower);
            
            if metrics.SNR_dB < 13, app.LblSNR.FontColor = [0.8 0 0]; else, app.LblSNR.FontColor = [0 0.6 0]; end
            
            %% 2. Live 3D World 
            R = app.RadarEngine.TargetState.Range;
            Az = app.RadarEngine.TargetState.Az; El = app.RadarEngine.TargetState.El;
            x = R * cosd(El) * cosd(Az); y = R * cosd(El) * sind(Az); z = R * sind(El);
            
            jamAz = app.RadarEngine.EnvironmentState.JammerAz; jamEl = app.RadarEngine.EnvironmentState.JammerEl;
            jx = R * cosd(jamEl) * cosd(jamAz); jy = R * cosd(jamEl) * sind(jamAz); jz = R * sind(jamEl);
            
            scanAz = app.RadarEngine.EnvironmentState.ScanAz; scanEl = app.RadarEngine.EnvironmentState.ScanEl;
            bx = R*0.9 * cosd(scanEl) * cosd(scanAz); by = R*0.9 * cosd(scanEl) * sind(scanAz); bz = R*0.9 * sind(scanEl);
            
            if isempty(app.hTarget) || ~isvalid(app.hTarget)
                app.hTarget = scatter3(app.Ax3DWorld, x, y, z, 100, 'g', 'filled', 'DisplayName', 'Target');
                app.hJammer = scatter3(app.Ax3DWorld, jx, jy, jz, 80, 'r', 'x', 'LineWidth', 2, 'DisplayName', 'Jammer');
                app.hScanBeam = plot3(app.Ax3DWorld, [0 bx], [0 by], [0 bz], '--b', 'LineWidth', 2, 'DisplayName', 'Look Dir');
                legend(app.Ax3DWorld, 'Location', 'best');
            else
                set(app.hTarget, 'XData', x, 'YData', y, 'ZData', z);
                set(app.hJammer, 'XData', jx, 'YData', jy, 'ZData', jz);
                set(app.hScanBeam, 'XData', [0 bx], 'YData', [0 by], 'ZData', [0 bz]);
            end
            
            %% 3. Array Pattern
            if app.TabGroupVisuals.SelectedTab == app.TabPattern
                [AZ, EL, Pat_dB, az_cut, ~] = app.RadarEngine.calcArrayPattern();
                if isempty(app.hPatSurf) || ~isvalid(app.hPatSurf)
                    app.hPatSurf = surf(app.AxPat3D, AZ, EL, Pat_dB, 'EdgeColor', 'none'); colormap(app.AxPat3D, 'jet');
                    app.hCutLine = plot(app.AxCutAz, AZ(1,:), az_cut, 'b', 'LineWidth', 2);
                else
                    set(app.hPatSurf, 'ZData', Pat_dB, 'CData', Pat_dB);
                    set(app.hCutLine, 'YData', az_cut);
                end
            end
            
            %% 4. Full DSP Pipeline
            if app.TabGroupVisuals.SelectedTab == app.TabSignal
                [t_rx, rx_raw, mf_out_dB, r_axis, RD_Map, v_axis] = app.RadarEngine.processCPI();
                
                if isempty(app.hRDImage) || ~isvalid(app.hRDImage)
                    app.hTimeLine = plot(app.AxTime, t_rx*1e6, rx_raw, 'Color', [0.5 0.5 0.5]);
                    app.hMFLine = plot(app.AxMF, r_axis/1000, mf_out_dB, 'b', 'LineWidth', 1.5);
                    app.hRDImage = imagesc(app.AxRDMap, 'XData', r_axis/1000, 'YData', v_axis, 'CData', RD_Map);
                    app.AxRDMap.YDir = 'normal'; colormap(app.AxRDMap, 'jet'); caxis(app.AxRDMap, [-50 0]);
                else
                    set(app.hTimeLine, 'YData', rx_raw);
                    set(app.hMFLine, 'YData', mf_out_dB);
                    set(app.hRDImage, 'CData', RD_Map);
                end
            end
        end
        
        function InputChanged(app, ~)
            app.RadarEngine.TargetState.Range = app.EditRange.Value * 1000;
            app.RadarEngine.TargetState.Vel = app.EditVel.Value;
            app.RadarEngine.TargetState.RCS = app.EditRCS.Value;
            app.RadarEngine.HardwareSpecs.Pt = app.EditPt.Value * 1000;
            app.RadarEngine.HardwareSpecs.Freq = app.EditFreq.Value * 1e9;
            app.RadarEngine.HardwareSpecs.Tau = app.EditTau.Value * 1e-6;
            app.RadarEngine.HardwareSpecs.PRF = app.EditPRF.Value * 1000;
            
            app.RadarEngine.ArrayConfig.Tx_Ny = app.EditTxN.Value; app.RadarEngine.ArrayConfig.Tx_Nz = app.EditTxN.Value;
            app.RadarEngine.ArrayConfig.Rx_Ny = app.EditRxN.Value; app.RadarEngine.ArrayConfig.Rx_Nz = app.EditRxN.Value;
            if strcmp(app.DropTaper.Value, 'Uniform'), app.RadarEngine.ArrayConfig.Taper = 1; else, app.RadarEngine.ArrayConfig.Taper = 2; end
            
            app.RadarEngine.EnvironmentState.ScanAz = app.EditScanAz.Value;
            app.RadarEngine.EnvironmentState.ScanEl = app.EditScanEl.Value;
            app.RadarEngine.EnvironmentState.JammerAz = app.EditJamAz.Value;
            app.RadarEngine.EnvironmentState.JammerEl = app.EditJamEl.Value;
            app.RadarEngine.EnvironmentState.JSR_dB = app.EditJSR.Value;
            app.RadarEngine.EnvironmentState.UseMVDR = app.CheckMVDR.Value;
            
            app.updateCalculations();
        end
        
        function runOptimization(app, ~)
            [opt_Pt, opt_N, log] = app.RadarEngine.runOptimizer(app.EditReqSNR.Value, app.EditMaxPt.Value);
            app.TxtOptLog.Value = log;
            
            % Apply optimized results to UI
            app.EditPt.Value = opt_Pt / 1000;
            app.EditTxN.Value = opt_N; app.EditRxN.Value = opt_N;
            app.InputChanged();
        end
        
        function toggleSimulation(app, ~)
            if strcmp(app.SimTimer.Running, 'off')
                app.BtnSimulate.Text = 'Stop Kinematics'; app.BtnSimulate.BackgroundColor = [0.8 0.2 0.2]; start(app.SimTimer);
            else
                app.BtnSimulate.Text = 'Run Kinematics'; app.BtnSimulate.BackgroundColor = [0.2 0.6 0.2]; stop(app.SimTimer);
            end
        end
        
        function stepSimulation(app)
            dt = app.SimTimer.Period;
            app.RadarEngine.TargetState.Range = app.RadarEngine.TargetState.Range + (app.RadarEngine.TargetState.Vel * dt);
            app.EditRange.Value = app.RadarEngine.TargetState.Range / 1000;
            app.updateCalculations();
        end
        
        function createComponents(app)
            app.UIFigure = uifigure('Name', 'System of Systems: Advanced Radar', 'Position', [50, 50, 1500, 850]);
            
            masterGrid = uigridlayout(app.UIFigure, [2 1]); masterGrid.RowHeight = {90, '1x'}; 
            
            % --- Dashboard ---
            dashPanel = uipanel(masterGrid, 'Title', 'Live Telemetry Dashboard', 'FontWeight', 'bold');
            dashGrid = uigridlayout(dashPanel, [1 8]);
            uilabel(dashGrid, 'Text', 'SNR:', 'HorizontalAlignment', 'right', 'FontWeight', 'bold'); app.LblSNR = uilabel(dashGrid, 'Text', '-');
            uilabel(dashGrid, 'Text', 'Max Rng:', 'HorizontalAlignment', 'right', 'FontWeight', 'bold'); app.LblMaxR = uilabel(dashGrid, 'Text', '-');
            uilabel(dashGrid, 'Text', 'Res (Comp):', 'HorizontalAlignment', 'right', 'FontWeight', 'bold'); app.LblRes = uilabel(dashGrid, 'Text', '-');
            uilabel(dashGrid, 'Text', 'Avg Pwr:', 'HorizontalAlignment', 'right', 'FontWeight', 'bold'); app.LblAvgPwr = uilabel(dashGrid, 'Text', '-');
            
            app.MainGrid = uigridlayout(masterGrid, [1 2]); app.MainGrid.ColumnWidth = {380, '1x'}; 
            
            % --- LEFT PANE (Inputs) ---
            leftLayout = uigridlayout(app.MainGrid, [2 1]); leftLayout.RowHeight = {'1x', 50};
            app.TabGroupInputs = uitabgroup(leftLayout);
            
            % Tab: Radar & Target
            app.TabRadar = uitab(app.TabGroupInputs, 'Title', 'Hardware');
            rGrid = uigridlayout(app.TabRadar, [7 2]);
            uilabel(rGrid, 'Text', 'Target Rng (km):'); app.EditRange = uieditfield(rGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'Target Vel (m/s):'); app.EditVel = uieditfield(rGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'Target RCS (m^2):'); app.EditRCS = uieditfield(rGrid, 'numeric', 'Value', 5, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'Peak Pwr (kW):'); app.EditPt = uieditfield(rGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'Freq (GHz):'); app.EditFreq = uieditfield(rGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'Pulse Width (us):'); app.EditTau = uieditfield(rGrid, 'numeric', 'Value', 10, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'PRF (kHz):'); app.EditPRF = uieditfield(rGrid, 'numeric', 'Value', 2, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            
            % Tab: Array & Env
            app.TabEnv = uitab(app.TabGroupInputs, 'Title', 'Array & Env');
            eGrid = uigridlayout(app.TabEnv, [8 2]);
            uilabel(eGrid, 'Text', 'Tx/Rx Elements (NxN):'); app.EditTxN = uieditfield(eGrid, 'numeric', 'Value', 8, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            app.EditRxN = uieditfield(eGrid, 'numeric', 'Value', 8, 'Visible', 'off'); % Hidden mirror
            uilabel(eGrid, 'Text', 'Amplitude Taper:'); app.DropTaper = uidropdown(eGrid, 'Items', {'Uniform', 'Hamming'}, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(eGrid, 'Text', 'Scan Azimuth:'); app.EditScanAz = uieditfield(eGrid, 'numeric', 'Value', 20, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(eGrid, 'Text', 'Scan Elevation:'); app.EditScanEl = uieditfield(eGrid, 'numeric', 'Value', 10, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(eGrid, 'Text', 'Jammer Azimuth:'); app.EditJamAz = uieditfield(eGrid, 'numeric', 'Value', -30, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(eGrid, 'Text', 'Jammer JSR (dB):'); app.EditJSR = uieditfield(eGrid, 'numeric', 'Value', 30, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            app.CheckMVDR = uicheckbox(eGrid, 'Text', 'Enable MVDR Null Steering', 'FontWeight', 'bold', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            app.CheckMVDR.Layout.Column = [1 2];
            
            % Tab: Optimizer
            app.TabOpt = uitab(app.TabGroupInputs, 'Title', 'Optimizer');
            oGrid = uigridlayout(app.TabOpt, [4 2]); oGrid.RowHeight = {22, 22, 30, '1x'};
            uilabel(oGrid, 'Text', 'Req. SINR (dB):'); app.EditReqSNR = uieditfield(oGrid, 'numeric', 'Value', 15);
            uilabel(oGrid, 'Text', 'Max Allowable Pt (kW):'); app.EditMaxPt = uieditfield(oGrid, 'numeric', 'Value', 500);
            app.BtnOpt = uibutton(oGrid, 'push', 'Text', 'Synthesize Optimal System', 'BackgroundColor', [0.85 0.33 0.1], 'FontColor', 'w', 'ButtonPushedFcn', @(s,e) app.runOptimization(e));
            app.BtnOpt.Layout.Column = [1 2];
            app.TxtOptLog = uitextarea(oGrid, 'Value', 'Awaiting optimization...', 'Editable', 'off', 'FontName', 'Courier');
            app.TxtOptLog.Layout.Column = [1 2];
            
            app.BtnSimulate = uibutton(leftLayout, 'push', 'Text', 'Run Kinematics', 'BackgroundColor', [0.2 0.6 0.2], 'FontColor', 'w', 'FontWeight', 'bold', 'ButtonPushedFcn', @(s,e) app.toggleSimulation(e));
            
            % --- RIGHT PANE (Visuals) ---
            app.TabGroupVisuals = uitabgroup(app.MainGrid);
            app.TabGroupVisuals.SelectionChangedFcn = @(s,e) app.updateCalculations();
            
            app.Tab3DWorld = uitab(app.TabGroupVisuals, 'Title', '1. Live Kinematics');
            tab1Grid = uigridlayout(app.Tab3DWorld, [1 1]);
            app.Ax3DWorld = uiaxes(tab1Grid); title(app.Ax3DWorld, '3D Tracking Volume'); grid(app.Ax3DWorld, 'on'); hold(app.Ax3DWorld, 'on'); view(app.Ax3DWorld, 3);
            app.Ax3DWorld.XLim = [-60000 60000]; app.Ax3DWorld.YLim = [-60000 60000]; app.Ax3DWorld.ZLim = [0 60000];
            
            app.TabPattern = uitab(app.TabGroupVisuals, 'Title', '2. Antenna Pattern');
            tab2Grid = uigridlayout(app.TabPattern, [2 1]); tab2Grid.RowHeight = {'1x', 220};
            app.AxPat3D = uiaxes(tab2Grid); title(app.AxPat3D, '3D Radiation Pattern'); view(app.AxPat3D, [30 30]); grid(app.AxPat3D, 'on'); app.AxPat3D.ZLim = [-50 0];
            app.AxCutAz = uiaxes(tab2Grid); title(app.AxCutAz, 'Azimuth Cut'); grid(app.AxCutAz, 'on'); app.AxCutAz.YLim = [-50 0];
            
            app.TabSignal = uitab(app.TabGroupVisuals, 'Title', '3. Signal Processing');
            tab3Grid = uigridlayout(app.TabSignal, [3 1]);
            app.AxTime = uiaxes(tab3Grid); title(app.AxTime, 'Raw Receive (Time Domain)');
            app.AxMF = uiaxes(tab3Grid); title(app.AxMF, 'Matched Filter Output'); app.AxMF.YLim = [-60 5];
            app.AxRDMap = uiaxes(tab3Grid); title(app.AxRDMap, 'Range-Doppler Map'); ylabel(app.AxRDMap, 'Velocity (m/s)'); xlabel(app.AxRDMap, 'Range (km)');
        end
        
        function UIFigureCloseRequest(app, ~)
            if isvalid(app.SimTimer), stop(app.SimTimer); delete(app.SimTimer); end
            delete(app);
        end
    end
    
    methods (Access = public)
        function app = IntegratedRadarTool()
            app.createComponents();
            app.UIFigure.CloseRequestFcn = @(s,e) app.UIFigureCloseRequest(e);
            app.startupFcn();
            if nargout == 0, clear app; end
        end
    end
end
classdef IntegratedRadarTool < matlab.apps.AppBase
    
    properties (Access = public)
        UIFigure      matlab.ui.Figure
        MainGrid      matlab.ui.container.GridLayout
        
        % Left Pane (Inputs)
        TabGroupInputs matlab.ui.container.TabGroup
        TabRadar       matlab.ui.container.Tab
        TabEnv         matlab.ui.container.Tab
        TabOpt         matlab.ui.container.Tab
        
        % Input Fields - Radar
        EditRange, EditVel, EditRCS
        EditPt, EditFreq, EditTau, EditPRF
        EditTxN, EditRxN, DropTaper
        
        % Input Fields - Env & Jammer
        EditScanAz, EditScanEl
        EditJamAz, EditJamEl, EditJSR
        CheckMVDR
        
        % Input Fields - Optimizer
        EditReqSNR, EditMaxPt, BtnOpt, TxtOptLog
        
        % Dashboard Labels
        LblSNR, LblMaxR, LblRes, LblAvgPwr
        
        % Right Pane (Visuals)
        TabGroupVisuals matlab.ui.container.TabGroup
        Tab3DWorld      matlab.ui.container.Tab
        TabPattern      matlab.ui.container.Tab
        TabSignal       matlab.ui.container.Tab
        
        % Axes
        Ax3DWorld, AxPat3D, AxCutAz
        AxTime, AxMF, AxRDMap
        
        BtnSimulate     matlab.ui.control.Button
    end
    
    properties (Access = private)
        RadarEngine
        SimTimer
        
        hTarget, hJammer, hScanBeam
        hPatSurf, hCutLine
        hTimeLine, hMFLine, hRDImage
    end
    
    methods (Access = private)
        function startupFcn(app)
            app.RadarEngine = MasterRadarSystem();
            
            % Initialize UI values
            app.EditRange.Value = app.RadarEngine.TargetState.Range / 1000;
            app.EditVel.Value = app.RadarEngine.TargetState.Vel;
            app.EditRCS.Value = app.RadarEngine.TargetState.RCS;
            app.EditPt.Value = app.RadarEngine.HardwareSpecs.Pt / 1000;
            app.EditFreq.Value = app.RadarEngine.HardwareSpecs.Freq / 1e9;
            
            app.SimTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.033, 'TimerFcn', @(~,~) app.stepSimulation());
            
            app.updateCalculations();
        end
        
        function updateCalculations(app)
            %% 1. Metrics Update
            metrics = app.RadarEngine.calcRadarFundamentals();
            app.LblSNR.Text = sprintf('%.1f dB', metrics.SNR_dB);
            app.LblMaxR.Text = sprintf('%.1f km', metrics.R_max / 1000);
            app.LblRes.Text = sprintf('%.1f m', metrics.R_res_comp);
            app.LblAvgPwr.Text = sprintf('%.1f W', metrics.AvgPower);
            
            if metrics.SNR_dB < 13, app.LblSNR.FontColor = [0.8 0 0]; else, app.LblSNR.FontColor = [0 0.6 0]; end
            
            %% 2. Live 3D World 
            R = app.RadarEngine.TargetState.Range;
            Az = app.RadarEngine.TargetState.Az; El = app.RadarEngine.TargetState.El;
            x = R * cosd(El) * cosd(Az); y = R * cosd(El) * sind(Az); z = R * sind(El);
            
            jamAz = app.RadarEngine.EnvironmentState.JammerAz; jamEl = app.RadarEngine.EnvironmentState.JammerEl;
            jx = R * cosd(jamEl) * cosd(jamAz); jy = R * cosd(jamEl) * sind(jamAz); jz = R * sind(jamEl);
            
            scanAz = app.RadarEngine.EnvironmentState.ScanAz; scanEl = app.RadarEngine.EnvironmentState.ScanEl;
            bx = R*0.9 * cosd(scanEl) * cosd(scanAz); by = R*0.9 * cosd(scanEl) * sind(scanAz); bz = R*0.9 * sind(scanEl);
            
            if isempty(app.hTarget) || ~isvalid(app.hTarget)
                app.hTarget = scatter3(app.Ax3DWorld, x, y, z, 100, 'g', 'filled', 'DisplayName', 'Target');
                app.hJammer = scatter3(app.Ax3DWorld, jx, jy, jz, 80, 'r', 'x', 'LineWidth', 2, 'DisplayName', 'Jammer');
                app.hScanBeam = plot3(app.Ax3DWorld, [0 bx], [0 by], [0 bz], '--b', 'LineWidth', 2, 'DisplayName', 'Look Dir');
                legend(app.Ax3DWorld, 'Location', 'best');
            else
                set(app.hTarget, 'XData', x, 'YData', y, 'ZData', z);
                set(app.hJammer, 'XData', jx, 'YData', jy, 'ZData', jz);
                set(app.hScanBeam, 'XData', [0 bx], 'YData', [0 by], 'ZData', [0 bz]);
            end
            
            %% 3. Array Pattern
            if app.TabGroupVisuals.SelectedTab == app.TabPattern
                [AZ, EL, Pat_dB, az_cut, ~] = app.RadarEngine.calcArrayPattern();
                if isempty(app.hPatSurf) || ~isvalid(app.hPatSurf)
                    app.hPatSurf = surf(app.AxPat3D, AZ, EL, Pat_dB, 'EdgeColor', 'none'); colormap(app.AxPat3D, 'jet');
                    app.hCutLine = plot(app.AxCutAz, AZ(1,:), az_cut, 'b', 'LineWidth', 2);
                else
                    set(app.hPatSurf, 'ZData', Pat_dB, 'CData', Pat_dB);
                    set(app.hCutLine, 'YData', az_cut);
                end
            end
            
            %% 4. Full DSP Pipeline
            if app.TabGroupVisuals.SelectedTab == app.TabSignal
                [t_rx, rx_raw, mf_out_dB, r_axis, RD_Map, v_axis] = app.RadarEngine.processCPI();
                
                if isempty(app.hRDImage) || ~isvalid(app.hRDImage)
                    app.hTimeLine = plot(app.AxTime, t_rx*1e6, rx_raw, 'Color', [0.5 0.5 0.5]);
                    app.hMFLine = plot(app.AxMF, r_axis/1000, mf_out_dB, 'b', 'LineWidth', 1.5);
                    app.hRDImage = imagesc(app.AxRDMap, 'XData', r_axis/1000, 'YData', v_axis, 'CData', RD_Map);
                    app.AxRDMap.YDir = 'normal'; colormap(app.AxRDMap, 'jet'); caxis(app.AxRDMap, [-50 0]);
                else
                    set(app.hTimeLine, 'YData', rx_raw);
                    set(app.hMFLine, 'YData', mf_out_dB);
                    set(app.hRDImage, 'CData', RD_Map);
                end
            end
        end
        
        function InputChanged(app, ~)
            app.RadarEngine.TargetState.Range = app.EditRange.Value * 1000;
            app.RadarEngine.TargetState.Vel = app.EditVel.Value;
            app.RadarEngine.TargetState.RCS = app.EditRCS.Value;
            app.RadarEngine.HardwareSpecs.Pt = app.EditPt.Value * 1000;
            app.RadarEngine.HardwareSpecs.Freq = app.EditFreq.Value * 1e9;
            app.RadarEngine.HardwareSpecs.Tau = app.EditTau.Value * 1e-6;
            app.RadarEngine.HardwareSpecs.PRF = app.EditPRF.Value * 1000;
            
            app.RadarEngine.ArrayConfig.Tx_Ny = app.EditTxN.Value; app.RadarEngine.ArrayConfig.Tx_Nz = app.EditTxN.Value;
            app.RadarEngine.ArrayConfig.Rx_Ny = app.EditRxN.Value; app.RadarEngine.ArrayConfig.Rx_Nz = app.EditRxN.Value;
            if strcmp(app.DropTaper.Value, 'Uniform'), app.RadarEngine.ArrayConfig.Taper = 1; else, app.RadarEngine.ArrayConfig.Taper = 2; end
            
            app.RadarEngine.EnvironmentState.ScanAz = app.EditScanAz.Value;
            app.RadarEngine.EnvironmentState.ScanEl = app.EditScanEl.Value;
            app.RadarEngine.EnvironmentState.JammerAz = app.EditJamAz.Value;
            app.RadarEngine.EnvironmentState.JammerEl = app.EditJamEl.Value;
            app.RadarEngine.EnvironmentState.JSR_dB = app.EditJSR.Value;
            app.RadarEngine.EnvironmentState.UseMVDR = app.CheckMVDR.Value;
            
            app.updateCalculations();
        end
        
        function runOptimization(app, ~)
            [opt_Pt, opt_N, log] = app.RadarEngine.runOptimizer(app.EditReqSNR.Value, app.EditMaxPt.Value);
            app.TxtOptLog.Value = log;
            
            % Apply optimized results to UI
            app.EditPt.Value = opt_Pt / 1000;
            app.EditTxN.Value = opt_N; app.EditRxN.Value = opt_N;
            app.InputChanged();
        end
        
        function toggleSimulation(app, ~)
            if strcmp(app.SimTimer.Running, 'off')
                app.BtnSimulate.Text = 'Stop Kinematics'; app.BtnSimulate.BackgroundColor = [0.8 0.2 0.2]; start(app.SimTimer);
            else
                app.BtnSimulate.Text = 'Run Kinematics'; app.BtnSimulate.BackgroundColor = [0.2 0.6 0.2]; stop(app.SimTimer);
            end
        end
        
        function stepSimulation(app)
            dt = app.SimTimer.Period;
            app.RadarEngine.TargetState.Range = app.RadarEngine.TargetState.Range + (app.RadarEngine.TargetState.Vel * dt);
            app.EditRange.Value = app.RadarEngine.TargetState.Range / 1000;
            app.updateCalculations();
        end
        
        function createComponents(app)
            app.UIFigure = uifigure('Name', 'System of Systems: Advanced Radar', 'Position', [50, 50, 1500, 850]);
            
            masterGrid = uigridlayout(app.UIFigure, [2 1]); masterGrid.RowHeight = {90, '1x'}; 
            
            % --- Dashboard ---
            dashPanel = uipanel(masterGrid, 'Title', 'Live Telemetry Dashboard', 'FontWeight', 'bold');
            dashGrid = uigridlayout(dashPanel, [1 8]);
            uilabel(dashGrid, 'Text', 'SNR:', 'HorizontalAlignment', 'right', 'FontWeight', 'bold'); app.LblSNR = uilabel(dashGrid, 'Text', '-');
            uilabel(dashGrid, 'Text', 'Max Rng:', 'HorizontalAlignment', 'right', 'FontWeight', 'bold'); app.LblMaxR = uilabel(dashGrid, 'Text', '-');
            uilabel(dashGrid, 'Text', 'Res (Comp):', 'HorizontalAlignment', 'right', 'FontWeight', 'bold'); app.LblRes = uilabel(dashGrid, 'Text', '-');
            uilabel(dashGrid, 'Text', 'Avg Pwr:', 'HorizontalAlignment', 'right', 'FontWeight', 'bold'); app.LblAvgPwr = uilabel(dashGrid, 'Text', '-');
            
            app.MainGrid = uigridlayout(masterGrid, [1 2]); app.MainGrid.ColumnWidth = {380, '1x'}; 
            
            % --- LEFT PANE (Inputs) ---
            leftLayout = uigridlayout(app.MainGrid, [2 1]); leftLayout.RowHeight = {'1x', 50};
            app.TabGroupInputs = uitabgroup(leftLayout);
            
            % Tab: Radar & Target
            app.TabRadar = uitab(app.TabGroupInputs, 'Title', 'Hardware');
            rGrid = uigridlayout(app.TabRadar, [7 2]);
            uilabel(rGrid, 'Text', 'Target Rng (km):'); app.EditRange = uieditfield(rGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'Target Vel (m/s):'); app.EditVel = uieditfield(rGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'Target RCS (m^2):'); app.EditRCS = uieditfield(rGrid, 'numeric', 'Value', 5, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'Peak Pwr (kW):'); app.EditPt = uieditfield(rGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'Freq (GHz):'); app.EditFreq = uieditfield(rGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'Pulse Width (us):'); app.EditTau = uieditfield(rGrid, 'numeric', 'Value', 10, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(rGrid, 'Text', 'PRF (kHz):'); app.EditPRF = uieditfield(rGrid, 'numeric', 'Value', 2, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            
            % Tab: Array & Env
            app.TabEnv = uitab(app.TabGroupInputs, 'Title', 'Array & Env');
            eGrid = uigridlayout(app.TabEnv, [8 2]);
            uilabel(eGrid, 'Text', 'Tx/Rx Elements (NxN):'); app.EditTxN = uieditfield(eGrid, 'numeric', 'Value', 8, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            app.EditRxN = uieditfield(eGrid, 'numeric', 'Value', 8, 'Visible', 'off'); % Hidden mirror
            uilabel(eGrid, 'Text', 'Amplitude Taper:'); app.DropTaper = uidropdown(eGrid, 'Items', {'Uniform', 'Hamming'}, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(eGrid, 'Text', 'Scan Azimuth:'); app.EditScanAz = uieditfield(eGrid, 'numeric', 'Value', 20, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(eGrid, 'Text', 'Scan Elevation:'); app.EditScanEl = uieditfield(eGrid, 'numeric', 'Value', 10, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(eGrid, 'Text', 'Jammer Azimuth:'); app.EditJamAz = uieditfield(eGrid, 'numeric', 'Value', -30, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(eGrid, 'Text', 'Jammer JSR (dB):'); app.EditJSR = uieditfield(eGrid, 'numeric', 'Value', 30, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            app.CheckMVDR = uicheckbox(eGrid, 'Text', 'Enable MVDR Null Steering', 'FontWeight', 'bold', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            app.CheckMVDR.Layout.Column = [1 2];
            
            % Tab: Optimizer
            app.TabOpt = uitab(app.TabGroupInputs, 'Title', 'Optimizer');
            oGrid = uigridlayout(app.TabOpt, [4 2]); oGrid.RowHeight = {22, 22, 30, '1x'};
            uilabel(oGrid, 'Text', 'Req. SINR (dB):'); app.EditReqSNR = uieditfield(oGrid, 'numeric', 'Value', 15);
            uilabel(oGrid, 'Text', 'Max Allowable Pt (kW):'); app.EditMaxPt = uieditfield(oGrid, 'numeric', 'Value', 500);
            app.BtnOpt = uibutton(oGrid, 'push', 'Text', 'Synthesize Optimal System', 'BackgroundColor', [0.85 0.33 0.1], 'FontColor', 'w', 'ButtonPushedFcn', @(s,e) app.runOptimization(e));
            app.BtnOpt.Layout.Column = [1 2];
            app.TxtOptLog = uitextarea(oGrid, 'Value', 'Awaiting optimization...', 'Editable', 'off', 'FontName', 'Courier');
            app.TxtOptLog.Layout.Column = [1 2];
            
            app.BtnSimulate = uibutton(leftLayout, 'push', 'Text', 'Run Kinematics', 'BackgroundColor', [0.2 0.6 0.2], 'FontColor', 'w', 'FontWeight', 'bold', 'ButtonPushedFcn', @(s,e) app.toggleSimulation(e));
            
            % --- RIGHT PANE (Visuals) ---
            app.TabGroupVisuals = uitabgroup(app.MainGrid);
            app.TabGroupVisuals.SelectionChangedFcn = @(s,e) app.updateCalculations();
            
            app.Tab3DWorld = uitab(app.TabGroupVisuals, 'Title', '1. Live Kinematics');
            tab1Grid = uigridlayout(app.Tab3DWorld, [1 1]);
            app.Ax3DWorld = uiaxes(tab1Grid); title(app.Ax3DWorld, '3D Tracking Volume'); grid(app.Ax3DWorld, 'on'); hold(app.Ax3DWorld, 'on'); view(app.Ax3DWorld, 3);
            app.Ax3DWorld.XLim = [-60000 60000]; app.Ax3DWorld.YLim = [-60000 60000]; app.Ax3DWorld.ZLim = [0 60000];
            
            app.TabPattern = uitab(app.TabGroupVisuals, 'Title', '2. Antenna Pattern');
            tab2Grid = uigridlayout(app.TabPattern, [2 1]); tab2Grid.RowHeight = {'1x', 220};
            app.AxPat3D = uiaxes(tab2Grid); title(app.AxPat3D, '3D Radiation Pattern'); view(app.AxPat3D, [30 30]); grid(app.AxPat3D, 'on'); app.AxPat3D.ZLim = [-50 0];
            app.AxCutAz = uiaxes(tab2Grid); title(app.AxCutAz, 'Azimuth Cut'); grid(app.AxCutAz, 'on'); app.AxCutAz.YLim = [-50 0];
            
            app.TabSignal = uitab(app.TabGroupVisuals, 'Title', '3. Signal Processing');
            tab3Grid = uigridlayout(app.TabSignal, [3 1]);
            app.AxTime = uiaxes(tab3Grid); title(app.AxTime, 'Raw Receive (Time Domain)');
            app.AxMF = uiaxes(tab3Grid); title(app.AxMF, 'Matched Filter Output'); app.AxMF.YLim = [-60 5];
            app.AxRDMap = uiaxes(tab3Grid); title(app.AxRDMap, 'Range-Doppler Map'); ylabel(app.AxRDMap, 'Velocity (m/s)'); xlabel(app.AxRDMap, 'Range (km)');
        end
        
        function UIFigureCloseRequest(app, ~)
            if isvalid(app.SimTimer), stop(app.SimTimer); delete(app.SimTimer); end
            delete(app);
        end
    end
    
    methods (Access = public)
        function app = IntegratedRadarTool()
            app.createComponents();
            app.UIFigure.CloseRequestFcn = @(s,e) app.UIFigureCloseRequest(e);
            app.startupFcn();
            if nargout == 0, clear app; end
        end
    end
end