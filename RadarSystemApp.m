classdef RadarSystemApp < matlab.apps.AppBase
    
    properties (Access = public)
        UIFigure
        MainGrid
        
        % Left Pane (Inputs)
        TabGroupInputs
        TabHardware
        TabArray
        
        % Input Fields
        EditRange, EditPower, EditFreq, EditTargetVel
        EditTxNy, EditTxNz, DropTaper
        
        % Dashboard Labels
        LabelSNR, LabelMaxRange, LabelRes
        
        % Right Pane (Visuals)
        TabGroupVisuals
        Tab3DWorld
        TabPattern
        TabSignal
        
        % Axes
        Ax3DWorld
        AxPat3D, AxCutAz
        AxRDMap
        
        BtnSimulate
    end
    
    properties (Access = private)
        RadarEngine
        SimTimer
        CurrentTime
        
        % Graphics Handles for high-speed updating
        hTarget, hFlightPath
        hPatSurf, hCutLine
        hRDImage
    end
    
    methods (Access = private)
        function startupFcn(app)
            app.RadarEngine = MasterRadarSystem();
            
            % Sync Inputs
            app.EditRange.Value = app.RadarEngine.TargetState.Range / 1000;
            app.EditTargetVel.Value = app.RadarEngine.TargetState.Vel;
            app.EditPower.Value = app.RadarEngine.HardwareSpecs.Pt / 1000;
            app.EditFreq.Value = app.RadarEngine.HardwareSpecs.Freq / 1e9;
            
            app.SimTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.033, 'TimerFcn', @(~,~) app.stepSimulation());
            app.CurrentTime = 0;
            
            app.updateCalculations();
        end
        
        function updateCalculations(app)
            %% 1. Fundamentals & Dashboard
            [snr, r_max, r_res] = app.RadarEngine.calcRadarFundamentals();
            app.LabelSNR.Text = sprintf('%.2f dB', snr);
            app.LabelMaxRange.Text = sprintf('%.2f km', r_max / 1000);
            app.LabelRes.Text = sprintf('%.2f m', r_res);
            
            if snr < 13, app.LabelSNR.FontColor = [0.8 0 0]; else, app.LabelSNR.FontColor = [0 0.6 0]; end
            
            %% 2. Live 3D World (Tab 1)
            R = app.RadarEngine.TargetState.Range;
            Az = app.RadarEngine.TargetState.Az;
            El = app.RadarEngine.TargetState.El;
            x = R * cosd(El) * cosd(Az); y = R * cosd(El) * sind(Az); z = R * sind(El);
            
            if isempty(app.hTarget) || ~isvalid(app.hTarget)
                app.hTarget = scatter3(app.Ax3DWorld, x, y, z, 100, 'g', 'filled');
            else
                set(app.hTarget, 'XData', x, 'YData', y, 'ZData', z);
            end
            
            %% 3. Array Pattern (Tab 2)
            [AZ, EL, Pat_dB, az_cut, ~] = app.RadarEngine.calcArrayPattern();
            if isempty(app.hPatSurf) || ~isvalid(app.hPatSurf)
                app.hPatSurf = surf(app.AxPat3D, AZ, EL, Pat_dB, 'EdgeColor', 'none');
                app.hCutLine = plot(app.AxCutAz, AZ(1,:), az_cut, 'b', 'LineWidth', 2);
            else
                set(app.hPatSurf, 'ZData', Pat_dB);
                set(app.hCutLine, 'YData', az_cut);
            end
            
            %% 4. Range Doppler Map (Tab 3)
            % Only update if the tab is visible to save CPU cycles
            if app.TabGroupVisuals.SelectedTab == app.TabSignal
                [RD_Map, r_axis, v_axis] = app.RadarEngine.processCPI();
                if isempty(app.hRDImage) || ~isvalid(app.hRDImage)
                    app.hRDImage = imagesc(app.AxRDMap, 'XData', r_axis/1000, 'YData', v_axis, 'CData', RD_Map);
                    app.AxRDMap.YDir = 'normal'; colormap(app.AxRDMap, 'jet'); clim(app.AxRDMap, [-50 0]);
                else
                    set(app.hRDImage, 'CData', RD_Map);
                end
            end
        end
        
        function InputChanged(app, ~)
            % Push UI updates to Model
            app.RadarEngine.TargetState.Range = app.EditRange.Value * 1000;
            app.RadarEngine.TargetState.Vel = app.EditTargetVel.Value;
            app.RadarEngine.HardwareSpecs.Pt = app.EditPower.Value * 1000;
            app.RadarEngine.HardwareSpecs.Freq = app.EditFreq.Value * 1e9;
            app.RadarEngine.ArrayConfig.Tx_Ny = app.EditTxNy.Value;
            app.RadarEngine.ArrayConfig.Tx_Nz = app.EditTxNz.Value;
            
            if strcmp(app.DropTaper.Value, 'Uniform'), app.RadarEngine.ArrayConfig.Taper = 1; else, app.RadarEngine.ArrayConfig.Taper = 2; end
            
            app.updateCalculations();
        end
        
        function toggleSimulation(app, ~)
            if strcmp(app.SimTimer.Running, 'off')
                app.BtnSimulate.Text = 'Stop Tracking';
                app.BtnSimulate.BackgroundColor = [0.8 0.2 0.2];
                start(app.SimTimer);
            else
                app.BtnSimulate.Text = 'Run Kinematic Tracking';
                app.BtnSimulate.BackgroundColor = [0.2 0.6 0.2];
                stop(app.SimTimer);
            end
        end
        
        function stepSimulation(app)
            dt = app.SimTimer.Period;
            app.CurrentTime = app.CurrentTime + dt;
            
            % Update kinematic position
            vel = app.RadarEngine.TargetState.Vel;
            app.RadarEngine.TargetState.Range = app.RadarEngine.TargetState.Range + (vel * dt);
            
            % Sync UI
            app.EditRange.Value = app.RadarEngine.TargetState.Range / 1000;
            app.updateCalculations();
        end
        
        function createComponents(app)
            app.UIFigure = uifigure('Name', 'System of Systems: Radar Architecture', 'Position', [50, 50, 1400, 800]);
            
            % Dashboard Panel (Top)
            dashPanel = uipanel(app.UIFigure, 'Position', [10, 700, 1380, 90], 'Title', 'Live Telemetry Dashboard', 'FontWeight', 'bold');
            dashGrid = uigridlayout(dashPanel, [1 6]);
            uilabel(dashGrid, 'Text', 'SNR:', 'HorizontalAlignment', 'right', 'FontWeight', 'bold');
            app.LabelSNR = uilabel(dashGrid, 'Text', '-');
            uilabel(dashGrid, 'Text', 'Max Range:', 'HorizontalAlignment', 'right', 'FontWeight', 'bold');
            app.LabelMaxRange = uilabel(dashGrid, 'Text', '-');
            uilabel(dashGrid, 'Text', 'Range Res:', 'HorizontalAlignment', 'right', 'FontWeight', 'bold');
            app.LabelRes = uilabel(dashGrid, 'Text', '-');
            
            % Main Split Layout
            app.MainGrid = uigridlayout(app.UIFigure, [1 2]);
            app.MainGrid.Position = [10, 10, 1380, 680];
            app.MainGrid.ColumnWidth = {350, '1x'}; 
            
            %% LEFT PANE: Input Tabs
            app.TabGroupInputs = uitabgroup(app.MainGrid);
            app.TabHardware = uitab(app.TabGroupInputs, 'Title', 'Target & RF');
            app.TabArray = uitab(app.TabGroupInputs, 'Title', 'Array Config');
            
            % Hardware Tab Elements
            hwGrid = uigridlayout(app.TabHardware, [6 2]);
            uilabel(hwGrid, 'Text', 'Target Range (km):'); app.EditRange = uieditfield(hwGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(hwGrid, 'Text', 'Radial Velocity (m/s):'); app.EditTargetVel = uieditfield(hwGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(hwGrid, 'Text', 'Peak Power (kW):'); app.EditPower = uieditfield(hwGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(hwGrid, 'Text', 'Frequency (GHz):'); app.EditFreq = uieditfield(hwGrid, 'numeric', 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            
            % Array Tab Elements
            arrGrid = uigridlayout(app.TabArray, [4 2]);
            uilabel(arrGrid, 'Text', 'Tx Elements (Az):'); app.EditTxNy = uieditfield(arrGrid, 'numeric', 'Value', 8, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(arrGrid, 'Text', 'Tx Elements (El):'); app.EditTxNz = uieditfield(arrGrid, 'numeric', 'Value', 8, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            uilabel(arrGrid, 'Text', 'Amplitude Taper:'); app.DropTaper = uidropdown(arrGrid, 'Items', {'Uniform', 'Hamming'}, 'ValueChangedFcn', @(s,e) app.InputChanged(e));
            
            app.BtnSimulate = uibutton(app.TabGroupInputs.Parent, 'push', 'Text', 'Run Kinematic Tracking', ...
                'BackgroundColor', [0.2 0.6 0.2], 'FontColor', 'w', 'FontWeight', 'bold', 'ButtonPushedFcn', @(s,e) app.toggleSimulation(e));
            app.BtnSimulate.Layout.Row = 2; % Places button below the tab group
            
            %% RIGHT PANE: Visualization Tabs
            app.TabGroupVisuals = uitabgroup(app.MainGrid);
            app.TabGroupVisuals.SelectionChangedFcn = @(s,e) app.updateCalculations(); % Force redraw when tab changes
            
            % Tab 1: 3D World
            app.Tab3DWorld = uitab(app.TabGroupVisuals, 'Title', '1. Live Kinematics');
            app.Ax3DWorld = uiaxes(app.Tab3DWorld, 'Position', [10 10 950 600]);
            title(app.Ax3DWorld, '3D Tracking Environment'); xlabel(app.Ax3DWorld, 'X'); ylabel(app.Ax3DWorld, 'Y'); zlabel(app.Ax3DWorld, 'Z');
            grid(app.Ax3DWorld, 'on'); hold(app.Ax3DWorld, 'on'); view(app.Ax3DWorld, 3);
            app.Ax3DWorld.XLim = [-60000 60000]; app.Ax3DWorld.YLim = [-60000 60000]; app.Ax3DWorld.ZLim = [0 60000];
            
            % Tab 2: Pattern
            app.TabPattern = uitab(app.TabGroupVisuals, 'Title', '2. Antenna Pattern');
            app.AxPat3D = uiaxes(app.TabPattern, 'Position', [10 250 950 350]);
            title(app.AxPat3D, '2-Way 3D Radiation Pattern'); xlabel(app.AxPat3D, 'Azimuth'); ylabel(app.AxPat3D, 'Elevation'); zlabel(app.AxPat3D, 'Gain (dB)');
            view(app.AxPat3D, [30 30]); grid(app.AxPat3D, 'on');
            app.AxCutAz = uiaxes(app.TabPattern, 'Position', [10 10 950 220]);
            title(app.AxCutAz, 'Azimuth Principal Cut'); xlabel(app.AxCutAz, 'Angle (deg)'); ylabel(app.AxCutAz, 'Gain (dB)'); grid(app.AxCutAz, 'on');
            app.AxCutAz.YLim = [-50 0];
            
            % Tab 3: Signal Processing
            app.TabSignal = uitab(app.TabGroupVisuals, 'Title', '3. Range-Doppler DSP');
            app.AxRDMap = uiaxes(app.TabSignal, 'Position', [10 10 950 600]);
            title(app.AxRDMap, 'Coherent Processing Interval: Range-Doppler Map');
            xlabel(app.AxRDMap, 'Range (km)'); ylabel(app.AxRDMap, 'Radial Velocity (m/s)');
        end
        
        function UIFigureCloseRequest(app, ~)
            if isvalid(app.SimTimer), stop(app.SimTimer); delete(app.SimTimer); end
            delete(app);
        end
    end
    
    methods (Access = public)
        function app = RadarSystemApp()
            app.createComponents();
            app.UIFigure.CloseRequestFcn = @(s,e) app.UIFigureCloseRequest(e);
            app.startupFcn();
            if nargout == 0, clear app; end
        end
    end
end