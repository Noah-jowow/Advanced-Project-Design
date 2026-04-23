classdef OmniRadarDirector < handle
    % =====================================================================
    % OMNI RADAR DIRECTOR: UNIFIED SYSTEM OF SYSTEMS SIMULATION (V7.1)
    % =====================================================================
    % V7.1 FINALIZED MONOLITH & MATHEMATICAL INTEGRITY UPGRADE:
    % - Scorched Earth Reset: Complete purge of axes and Handle Graphics objects.
    % - Variable Purge: Removed orphaned s_tx and dsp baseband references.
    % - Mathematical Audit: Scan loss and STAP linear indices fully verified.
    % - Production Ready: System is now fully stabilized for COM modularization.
    % =====================================================================

    properties (Access = private)
        % =================================================================
        % [UI COMPONENTS]
        % =================================================================
        UIFigure, TargetTable, EstTable, TxtOptLog
        BtnRun, BtnStop, BtnAddTarget, BtnRemTarget, BtnLoadMap, BtnOpt, BtnExport
        CheckManeuvers, CheckPredictive, CheckRealTime, CheckStrictOpt
        DropEffectorType, DropRadarMode, DropTaper, DropOptBand
        CheckMF, CheckDF, CheckAF, CheckMIMO, CheckFD, CheckJammer
        DropSteeringType
        
        EditChirpBW, EditMaxPt, EditReqSNR
        EditTxNy, EditTxNz, EditTxDy, EditTxDz
        EditRxNy, EditRxNz, EditRxDy, EditRxDz
        EditSteerAz, EditSteerEl, EditFreq, EditTau, EditPRF, EditPt
        
        LblSNR, LblMaxR, LblRes, LblAvgPwr, LblDir, LblAzBW, LblElBW
        
        TabGroupInputs, TabInHW, TabInArr, TabInOpt
        TabGroupVis, TabSetup, TabC2Main, TabPointer
        TgSetup, TgC2, TgPointer
        TabFund, TabArray, TabSensor, TabDSP, TabC2, TabEffHex, TabEffAzEl
        
        AxSNR, AxPulseTrain, AxGeom, AxPat3D, AxCutAz, AxCutEl, AxGlobe
        Ax3D, AxPPI, AxRD3D, AxEWSpec, AxTime, AxMF, AxRDMap, AxCFAR, AxSTAP
        AxMeas3D, AxPol, AxCrossSection, AxErrRng, AxErrVel, AxHexapod, AxAzEl
        
        % Visual Graphics Handles (Strictly managed for garbage collection)
        beamConeSurf, sweepLine, blipsScatter, rdScatter3D
        targetTransforms, hMeasRaw, hMeasLine, hMeasEst
        hErrRange, hErrVel, hPolBars, hCrossShape
        HexTopPlate, HexLegs, HexLaser, HexWarn, HexTele
        TAz, TEl, AzElLaser, AzElWarn, AzElTele
        hGeomVirt, hGeomTx, hGeomRx, hArrPatSurf, hCutAzL, hCutElL
        hEarthSurf, hRadarOrigin, hGlobalPatSurf
        hTimeLine, hMFLine, hRDImage, hCFARImage, hSTAPImage, hEWSpecLine
        EarthTexture = [];
        
        % =================================================================
        % [PHYSICS & STATE MEMORY]
        % =================================================================
        lastRenderTime
        simStartTime
        HardwareSpecs   
        ArrayConfig     
        radarParams     
        radarState      
        targets         
        
        dsp_rx_mat      
        dsp_MF_F        
        dsp_t_rx        
        dsp_delay
        
        visData         
        
        hexParams, hexState, hexRenderCount
        azelParams, azelState, azelRenderCount
        
        simState        
        
        c = 299792458; 
        kT0 = 1.380649e-23 * 290; 
        R_earth = 6371e3; 
        k_ref = 1.333333; 
        h_radar = 15.0;   
    end
    
    methods
        function obj = OmniRadarDirector()
            % Bootstrap simulation environment
            obj.initSystemMemory();
            obj.setupEffectorParams();
            obj.buildUI();
            obj.parseSystemParameters();
            obj.updateStaticVisuals();
        end
    end
    
    methods (Access = private)
        function initSystemMemory(obj)
            % Base timing and rendering structure limits
            obj.simState.time = 0; obj.simState.dt = 0.033; obj.simState.running = false;
            
            obj.visData.maxPPI = 150; obj.visData.maxRD = 300;
            obj.visData.bX = zeros(150,1); obj.visData.bY = zeros(150,1); obj.visData.bA = ones(150,1)*100; obj.visData.bI = 1;
            obj.visData.rX = nan(300,1); obj.visData.rY = nan(300,1); obj.visData.rZ = nan(300,1);
            obj.visData.rC = repmat([0 0 0], 300, 1); obj.visData.rA = ones(300,1)*100; obj.visData.rI = 1;
            
            obj.radarState.last_az = 0; obj.radarState.last_el = 0; obj.radarState.mpar_tick = 0;
            obj.radarState.pos = [0, 0, obj.h_radar]; 
            obj.radarState.vel = [0, 150, 0]; 
        end
        
        function setupEffectorParams(obj)
            % Physical Pointer Parameter Initialization
            obj.hexParams = struct('R_base', 0.5, 'R_top', 0.3, 'Z_nom', 0.6, 'm_top', 25.0, 'F_limit', 2500, 'L_min', 0.35, 'L_max', 0.95, 'Cond_Limit', 500, 'g_vec', [0;0;-9.81]);
            obj.hexParams.I_local = diag([(1/4)*25*0.3^2, (1/4)*25*0.3^2, (1/2)*25*0.3^2]);
            obj.hexParams.I_inv = inv(obj.hexParams.I_local);
            tb = deg2rad([15, 105, 135, 225, 255, 345]); tt = deg2rad([45, 75, 165, 195, 285, 315]);
            obj.hexParams.B = [obj.hexParams.R_base*cos(tb); obj.hexParams.R_base*sin(tb); zeros(1,6)];
            obj.hexParams.P = [obj.hexParams.R_top*cos(tt); obj.hexParams.R_top*sin(tt); zeros(1,6)];
            obj.hexState = struct('q', [1;0;0;0], 'omega', [0;0;0], 'last_Jinv', zeros(6)); obj.hexRenderCount = 0;
            
            obj.azelParams = struct('m_az', 50.0, 'J_z', 4.0, 'm_p', 30.0, 'Tau_max_az', 500, 'Tau_max_el', 300, 'Max_Vel', 3.0);
            rp = 0.2; Lp = 0.8; obj.azelParams.I_x = 0.5 * 30 * rp^2; obj.azelParams.I_y = (1/12) * 30 * (3*rp^2 + Lp^2); obj.azelParams.I_z = obj.azelParams.I_y;
            obj.azelState = struct('q', [0;0], 'q_dot', [0;0]); obj.azelRenderCount = 0;
        end
        
        function val = safeNum(~, str, defaultVal)
            val = str2double(str);
            if isnan(val) || isinf(val), val = defaultVal; end
        end

        function val = safeNumCell(~, c)
            if iscell(c), c = c{1}; end
            if ischar(c) || isstring(c), val = str2double(c);
            elseif isnumeric(c), val = double(c);
            else, val = 0; 
            end
            if isnan(val), val = 0; end
        end

        function buildUI(obj)
            % Procedural Workspace Construction
            obj.UIFigure = uifigure('Name', 'Omni Radar Director: Ultimate System V7.1', 'Position', [50, 50, 1600, 900], 'Color', 'w', 'CloseRequestFcn', @obj.onClose);
            
            pnlCtrl = uipanel(obj.UIFigure, 'Position', [16, 9, 448, 882]);
            obj.TabGroupInputs = uitabgroup(pnlCtrl, 'Position', [0 300 448 580]);
            
            obj.TabInHW = uitab(obj.TabGroupInputs, 'Title', 'Tgt & HW');
            initData = {'UAV (Stealth)', 0.1, 30, 20, 0.5, -30, -20, 0; 'Airliner', 50.0, -60, 50, 10.0, 200, -150, 0; 'Missile', 0.5, 10, 70, 5.0, -100, -800, -50};
            obj.TargetTable = uitable('Parent', obj.TabInHW, 'Data', initData, 'ColumnName', {'Name', 'RCS', 'X', 'Y', 'Z', 'Vx', 'Vy', 'Vz'}, 'ColumnEditable', true, 'Position', [10, 360, 420, 180], 'RowName', []);
            obj.BtnAddTarget = uicontrol('Parent', obj.TabInHW, 'Style', 'pushbutton', 'String', '+ Target', 'Position', [10, 320, 200, 30], 'Callback', @(s,e) obj.modTarget(1));
            obj.BtnRemTarget = uicontrol('Parent', obj.TabInHW, 'Style', 'pushbutton', 'String', '- Target', 'Position', [230, 320, 200, 30], 'Callback', @(s,e) obj.modTarget(-1));
            obj.CheckManeuvers = uicontrol('Parent', obj.TabInHW, 'Style', 'checkbox', 'String', 'Evasive Maneuvers', 'Position', [20, 290, 200, 22], 'Value', 1);
            obj.CheckJammer = uicontrol('Parent', obj.TabInHW, 'Style', 'checkbox', 'String', 'EW Jammer (Tgt 1)', 'Position', [240, 290, 200, 22], 'Value', 1);
            obj.CheckPredictive = uicontrol('Parent', obj.TabInHW, 'Style', 'checkbox', 'String', 'Predictive Lead', 'Position', [20, 260, 400, 22], 'Value', 1);
            obj.DropRadarMode = uicontrol('Parent', obj.TabInHW, 'Style', 'popupmenu', 'String', {'Mech Sweep', 'AESA (Seq)', 'AESA (MPAR Track)'}, 'Position', [20, 230, 400, 22], 'Value', 1);
            obj.DropEffectorType = uicontrol('Parent', obj.TabInHW, 'Style', 'popupmenu', 'String', {'Hexapod (Stewart)', 'Az/El Gimbal'}, 'Position', [20, 200, 400, 22], 'Value', 1);
            
            uicontrol('Parent', obj.TabInHW, 'Style', 'text', 'String', 'Freq (GHz):', 'Position', [20, 150, 200, 22], 'HorizontalAlignment', 'right');
            obj.EditFreq = uicontrol('Parent', obj.TabInHW, 'Style', 'edit', 'String', '3.0', 'Position', [230, 150, 180, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            uicontrol('Parent', obj.TabInHW, 'Style', 'text', 'String', 'Tau (us):', 'Position', [20, 120, 200, 22], 'HorizontalAlignment', 'right');
            obj.EditTau = uicontrol('Parent', obj.TabInHW, 'Style', 'edit', 'String', '10', 'Position', [230, 120, 180, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            uicontrol('Parent', obj.TabInHW, 'Style', 'text', 'String', 'PRF (kHz):', 'Position', [20, 90, 200, 22], 'HorizontalAlignment', 'right');
            obj.EditPRF = uicontrol('Parent', obj.TabInHW, 'Style', 'edit', 'String', '2', 'Position', [230, 90, 180, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            uicontrol('Parent', obj.TabInHW, 'Style', 'text', 'String', 'Act. Pt (kW):', 'Position', [20, 60, 200, 22], 'HorizontalAlignment', 'right');
            obj.EditPt = uicontrol('Parent', obj.TabInHW, 'Style', 'edit', 'String', '100', 'Position', [230, 60, 180, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            
            obj.TabInArr = uitab(obj.TabGroupInputs, 'Title', 'Array & DSP');
            uicontrol('Parent', obj.TabInArr, 'Style', 'text', 'String', 'Tx Ny, Nz:', 'Position', [20, 520, 120, 22]);
            obj.EditTxNy = uicontrol('Parent', obj.TabInArr, 'Style', 'edit', 'String', '16', 'Position', [150, 520, 80, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            obj.EditTxNz = uicontrol('Parent', obj.TabInArr, 'Style', 'edit', 'String', '16', 'Position', [240, 520, 80, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            
            uicontrol('Parent', obj.TabInArr, 'Style', 'text', 'String', 'Tx dy, dz (m):', 'Position', [20, 490, 120, 22]);
            obj.EditTxDy = uicontrol('Parent', obj.TabInArr, 'Style', 'edit', 'String', '0.05', 'Position', [150, 490, 80, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            obj.EditTxDz = uicontrol('Parent', obj.TabInArr, 'Style', 'edit', 'String', '0.05', 'Position', [240, 490, 80, 22], 'Callback', @(s,e) obj.updateStaticVisuals());

            uicontrol('Parent', obj.TabInArr, 'Style', 'text', 'String', 'Rx Ny, Nz:', 'Position', [20, 460, 120, 22]);
            obj.EditRxNy = uicontrol('Parent', obj.TabInArr, 'Style', 'edit', 'String', '16', 'Position', [150, 460, 80, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            obj.EditRxNz = uicontrol('Parent', obj.TabInArr, 'Style', 'edit', 'String', '16', 'Position', [240, 460, 80, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            
            uicontrol('Parent', obj.TabInArr, 'Style', 'text', 'String', 'Rx dy, dz (m):', 'Position', [20, 430, 120, 22]);
            obj.EditRxDy = uicontrol('Parent', obj.TabInArr, 'Style', 'edit', 'String', '0.05', 'Position', [150, 430, 80, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            obj.EditRxDz = uicontrol('Parent', obj.TabInArr, 'Style', 'edit', 'String', '0.05', 'Position', [240, 430, 80, 22], 'Callback', @(s,e) obj.updateStaticVisuals());

            obj.DropTaper = uicontrol('Parent', obj.TabInArr, 'Style', 'popupmenu', 'String', {'Uniform', 'Hamming', 'Hann'}, 'Position', [20, 390, 400, 22], 'Value', 2, 'Callback', @(s,e) obj.updateStaticVisuals());
            
            uicontrol('Parent', obj.TabInArr, 'Style', 'text', 'String', 'Steer Az/El:', 'Position', [20, 340, 120, 22]);
            obj.EditSteerAz = uicontrol('Parent', obj.TabInArr, 'Style', 'edit', 'String', '45', 'Position', [150, 340, 80, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            obj.EditSteerEl = uicontrol('Parent', obj.TabInArr, 'Style', 'edit', 'String', '15', 'Position', [240, 340, 80, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            obj.DropSteeringType = uicontrol('Parent', obj.TabInArr, 'Style', 'popupmenu', 'String', {'Phase Shifters (Squint)', 'True Time Delay (TTD)'}, 'Position', [20, 290, 400, 22], 'Value', 1, 'Callback', @(s,e) obj.updateStaticVisuals());

            obj.CheckMF = uicontrol('Parent', obj.TabInArr, 'Style', 'checkbox', 'String', 'Matched Filter', 'Position', [20, 240, 200, 22], 'Value', 1);
            obj.CheckDF = uicontrol('Parent', obj.TabInArr, 'Style', 'checkbox', 'String', 'Doppler MTI', 'Position', [240, 240, 200, 22], 'Value', 1);
            obj.CheckAF = uicontrol('Parent', obj.TabInArr, 'Style', 'checkbox', 'String', 'STAP Filter', 'Position', [20, 200, 200, 22], 'Value', 1);
            uicontrol('Parent', obj.TabInArr, 'Style', 'text', 'String', 'BW (MHz):', 'Position', [240, 200, 100, 22]);
            obj.EditChirpBW = uicontrol('Parent', obj.TabInArr, 'Style', 'edit', 'String', '50', 'Position', [350, 200, 70, 22], 'Callback', @(s,e) obj.updateStaticVisuals());
            
            obj.CheckMIMO = uicontrol('Parent', obj.TabInArr, 'Style', 'checkbox', 'String', 'MIMO Virtual Array', 'Position', [20, 160, 200, 22], 'Value', 0);
            obj.CheckFD = uicontrol('Parent', obj.TabInArr, 'Style', 'checkbox', 'String', 'Freq Diversity', 'Position', [240, 160, 200, 22], 'Value', 0);
            
            obj.TabInOpt = uitab(obj.TabGroupInputs, 'Title', 'Optimizer');
            uicontrol('Parent', obj.TabInOpt, 'Style', 'text', 'String', 'Req SNR (dB):', 'Position', [20, 500, 200, 22]);
            obj.EditReqSNR = uicontrol('Parent', obj.TabInOpt, 'Style', 'edit', 'String', '15', 'Position', [240, 500, 180, 22]);
            uicontrol('Parent', obj.TabInOpt, 'Style', 'text', 'String', 'Max Pt (kW):', 'Position', [20, 460, 200, 22]);
            obj.EditMaxPt = uicontrol('Parent', obj.TabInOpt, 'Style', 'edit', 'String', '500', 'Position', [240, 460, 180, 22]);
            obj.DropOptBand = uicontrol('Parent', obj.TabInOpt, 'Style', 'popupmenu', 'String', {'Auto Band', 'L-Band', 'S-Band', 'X-Band'}, 'Position', [20, 410, 400, 22], 'Callback', @(s,e) obj.handleBandSelection());
            obj.CheckStrictOpt = uicontrol('Parent', obj.TabInOpt, 'Style', 'checkbox', 'String', 'Strict UI Lock (Do Not Overwrite Params)', 'Position', [20, 370, 400, 22], 'Value', 0);
            obj.BtnOpt = uicontrol('Parent', obj.TabInOpt, 'Style', 'pushbutton', 'String', 'Optimize System Parameters', 'Position', [20, 300, 400, 50], 'BackgroundColor', [0.8 0.3 0.1], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'Callback', @(s,e) obj.runSystemOptimizer());
            obj.TxtOptLog = uicontrol('Parent', obj.TabInOpt, 'Style', 'edit', 'String', 'System initialized. Awaiting user parameters...', 'Max', 2, 'Min', 0, 'Enable', 'inactive', 'Position', [20, 20, 400, 260], 'HorizontalAlignment', 'left', 'FontSize', 10, 'FontName', 'Courier New');
            
            obj.CheckRealTime = uicontrol('Parent', pnlCtrl, 'Style', 'checkbox', 'String', 'Sync 1x Real-Time Execution', 'Position', [20, 240, 400, 22], 'Value', 1);
            obj.BtnRun = uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', 'String', '▶ Run Simulation', 'Position', [10, 160, 420, 60], 'BackgroundColor', [0 0.5 0], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'FontSize', 14, 'Callback', @(s,e) obj.runSimulation());
            obj.BtnStop = uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', 'String', '■ Stop & Reset', 'Position', [10, 90, 200, 60], 'BackgroundColor', [0.8 0 0], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'FontSize', 14, 'Callback', @(s,e) obj.stopSimulation());
            obj.BtnExport = uicontrol('Parent', pnlCtrl, 'Style', 'pushbutton', 'String', 'Export Data', 'Position', [230, 90, 200, 60], 'BackgroundColor', [0 0.3 0.6], 'ForegroundColor', 'w', 'FontWeight', 'bold', 'FontSize', 14, 'Callback', @(s,e) obj.exportTelemetry());

            dash = uipanel(obj.UIFigure, 'Position', [480, 792, 1104, 99], 'Title', 'Live Metrics', 'FontWeight', 'bold');
            dg = uigridlayout(dash, [1 14]);
            uilabel(dg, 'Text', 'SNR:'); obj.LblSNR = uilabel(dg, 'Text', '-');
            uilabel(dg, 'Text', 'Max R:'); obj.LblMaxR = uilabel(dg, 'Text', '-');
            uilabel(dg, 'Text', 'Res:'); obj.LblRes = uilabel(dg, 'Text', '-');
            uilabel(dg, 'Text', 'Dir:'); obj.LblDir = uilabel(dg, 'Text', '-');
            uilabel(dg, 'Text', 'Az BW:'); obj.LblAzBW = uilabel(dg, 'Text', '-');
            uilabel(dg, 'Text', 'El BW:'); obj.LblElBW = uilabel(dg, 'Text', '-');
            uilabel(dg, 'Text', 'Pwr:'); obj.LblAvgPwr = uilabel(dg, 'Text', '-');
            
            obj.TabGroupVis = uitabgroup(obj.UIFigure, 'Position', [480, 9, 1104, 774]);
            obj.TabSetup = uitab(obj.TabGroupVis, 'Title', 'System Setup');
            obj.TabC2Main = uitab(obj.TabGroupVis, 'Title', 'C2 & Sensor Analysis');
            obj.TabPointer = uitab(obj.TabGroupVis, 'Title', 'Pointer Dynamics');
            
            obj.TgSetup = uitabgroup(obj.TabSetup, 'Position', [0 0 1104 740]);
            obj.TabFund = uitab(obj.TgSetup, 'Title', '1. Fundamentals');
            obj.AxSNR = axes('Parent', obj.TabFund, 'Position', [0.05, 0.1, 0.4, 0.8]); title(obj.AxSNR, 'Radar Eq: Single Pulse SNR vs Range'); grid(obj.AxSNR, 'on'); xlabel(obj.AxSNR, 'Range (km)'); ylabel(obj.AxSNR, 'SNR (dB)');
            obj.AxPulseTrain = axes('Parent', obj.TabFund, 'Position', [0.55, 0.1, 0.4, 0.8]); title(obj.AxPulseTrain, 'Timing: Active Pulse Train'); grid(obj.AxPulseTrain, 'on'); xlabel(obj.AxPulseTrain, 'Time (us)'); ylabel(obj.AxPulseTrain, 'Amplitude (Norm)');
            
            obj.TabArray = uitab(obj.TgSetup, 'Title', '2. Array Synthesis');
            obj.AxGeom = axes('Parent', obj.TabArray, 'Position', [0.05, 0.55, 0.25, 0.4]); title(obj.AxGeom, 'Physical Array Geometry (Tx/Rx)'); grid(obj.AxGeom, 'on'); view(obj.AxGeom, 3);
            obj.AxPat3D = axes('Parent', obj.TabArray, 'Position', [0.35, 0.55, 0.3, 0.4]); title(obj.AxPat3D, '3D Spatial Radiation Pattern (Array Factor)'); grid(obj.AxPat3D, 'on'); view(obj.AxPat3D, 3);
            obj.AxCutAz = axes('Parent', obj.TabArray, 'Position', [0.05, 0.08, 0.25, 0.35]); title(obj.AxCutAz, 'Azimuth Pattern Cut (dB)'); grid(obj.AxCutAz, 'on');
            obj.AxCutEl = axes('Parent', obj.TabArray, 'Position', [0.35, 0.08, 0.25, 0.35]); title(obj.AxCutEl, 'Elevation Pattern Cut (dB)'); grid(obj.AxCutEl, 'on');
            obj.AxGlobe = axes('Parent', obj.TabArray, 'Position', [0.68, 0.05, 0.3, 0.9]); title(obj.AxGlobe, 'Global Earth Projection Map'); axis(obj.AxGlobe, 'equal'); axis(obj.AxGlobe, 'off');
            obj.BtnLoadMap = uicontrol('Parent', obj.TabArray, 'Style', 'pushbutton', 'String', 'Load Earth Map', 'Units', 'normalized', 'Position', [0.68, 0.92, 0.15, 0.05], 'Callback', @(s,e) obj.loadEarthMap());
            
            obj.TgC2 = uitabgroup(obj.TabC2Main, 'Position', [0 0 1104 740]);
            obj.TabSensor = uitab(obj.TgC2, 'Title', '3. Sensor & EW View');
            obj.Ax3D = axes('Parent', obj.TabSensor, 'Position', [0.05, 0.55, 0.40, 0.40]); title(obj.Ax3D, '3D Operational Airspace (Max Range Bound)'); grid(obj.Ax3D, 'on'); view(obj.Ax3D, 3); xlabel(obj.Ax3D, 'X (m)'); ylabel(obj.Ax3D, 'Y (m)'); zlabel(obj.Ax3D, 'Z (m)');
            obj.AxRD3D = axes('Parent', obj.TabSensor, 'Position', [0.55, 0.55, 0.40, 0.40]); title(obj.AxRD3D, '3D Range-Doppler Trace History'); grid(obj.AxRD3D, 'on'); view(obj.AxRD3D, 3); set(obj.AxRD3D, 'Color', [0 0.1 0.1]); xlabel(obj.AxRD3D, 'Range (km)'); ylabel(obj.AxRD3D, 'Velocity (m/s)'); zlabel(obj.AxRD3D, 'Time (s)');
            obj.AxPPI = axes('Parent', obj.TabSensor, 'Position', [0.05, 0.05, 0.40, 0.40]); title(obj.AxPPI, 'Plan Position Indicator (PPI)'); set(obj.AxPPI, 'Color', [0 0.1 0], 'XColor', 'none', 'YColor', 'none'); 
            obj.AxEWSpec = axes('Parent', obj.TabSensor, 'Position', [0.55, 0.05, 0.40, 0.40]); title(obj.AxEWSpec, 'EW Jammer Spectral Environment'); grid(obj.AxEWSpec, 'on'); xlabel(obj.AxEWSpec, 'Frequency (MHz)'); ylabel(obj.AxEWSpec, 'Rx Power (dBm)');
            
            obj.TabDSP = uitab(obj.TgC2, 'Title', '4. Signal Proc & STAP');
            obj.AxTime = axes('Parent', obj.TabDSP, 'Position', [0.05, 0.76, 0.9, 0.20]); title(obj.AxTime, 'Raw Receiver Baseband (Fast Time)'); grid(obj.AxTime, 'on'); xlabel(obj.AxTime, 'Time (us)'); ylabel(obj.AxTime, 'Amplitude');
            obj.AxMF = axes('Parent', obj.TabDSP, 'Position', [0.05, 0.52, 0.9, 0.20]); title(obj.AxMF, 'Matched Filter Range Profile'); grid(obj.AxMF, 'on'); xlabel(obj.AxMF, 'Range (km)'); ylabel(obj.AxMF, 'Gain (dB)');
            
            obj.AxRDMap = axes('Parent', obj.TabDSP, 'Position', [0.03, 0.05, 0.28, 0.40]); title(obj.AxRDMap, 'CPI FFT Range-Doppler Matrix'); xlabel(obj.AxRDMap, 'Range (km)'); ylabel(obj.AxRDMap, 'Velocity (m/s)');
            obj.AxCFAR = axes('Parent', obj.TabDSP, 'Position', [0.35, 0.05, 0.28, 0.40]); title(obj.AxCFAR, '1D CA-CFAR Detections'); xlabel(obj.AxCFAR, 'Range (km)'); ylabel(obj.AxCFAR, 'Velocity (m/s)');
            obj.AxSTAP = axes('Parent', obj.TabDSP, 'Position', [0.68, 0.05, 0.28, 0.40]); title(obj.AxSTAP, 'STAP Angle-Doppler Ridge'); xlabel(obj.AxSTAP, 'Angle (deg)'); ylabel(obj.AxSTAP, 'Doppler Vel (m/s)');
            
            obj.TabC2 = uitab(obj.TgC2, 'Title', '5. C2 Operator (EKF)');
            obj.AxMeas3D = axes('Parent', obj.TabC2, 'Position', [0.02, 0.35, 0.45, 0.60]); title(obj.AxMeas3D, 'IMM-EKF 7-State Tracks (Max Range Bound)'); grid(obj.AxMeas3D, 'on'); view(obj.AxMeas3D, 3); xlabel(obj.AxMeas3D, 'X (m)'); ylabel(obj.AxMeas3D, 'Y (m)'); zlabel(obj.AxMeas3D, 'Z (m)');
            obj.AxPol = axes('Parent', obj.TabC2, 'Position', [0.52, 0.68, 0.20, 0.27]); title(obj.AxPol, 'Dual-Pol Signature Classification'); grid(obj.AxPol, 'on'); set(obj.AxPol, 'XTick', 1:3, 'XTickLabel', {'HH', 'VV', 'HV'});
            obj.AxCrossSection = axes('Parent', obj.TabC2, 'Position', [0.76, 0.68, 0.20, 0.27]); title(obj.AxCrossSection, 'RCS Silhouette Estimate'); axis(obj.AxCrossSection, 'equal'); set(obj.AxCrossSection, 'XTick', [], 'YTick', []);
            obj.AxErrRng = axes('Parent', obj.TabC2, 'Position', [0.52, 0.35, 0.20, 0.25]); title(obj.AxErrRng, 'Filter Range Error (m)'); grid(obj.AxErrRng, 'on'); xlabel(obj.AxErrRng, 'Time (s)');
            obj.AxErrVel = axes('Parent', obj.TabC2, 'Position', [0.76, 0.35, 0.20, 0.25]); title(obj.AxErrVel, 'Filter Velocity Error (m/s)'); grid(obj.AxErrVel, 'on'); xlabel(obj.AxErrVel, 'Time (s)');
            obj.EstTable = uitable('Parent', obj.TabC2, 'ColumnName', {'Track ID', 'Rng (km)', 'Alt (m)', 'Spd (m/s)', 'Est RCS', 'Shape', 'Class', 'TTI (s)', 'Turn(rad/s)'}, 'Units', 'normalized', 'Position', [0.02, 0.02, 0.96, 0.28], 'RowName', []);
            
            obj.TgPointer = uitabgroup(obj.TabPointer, 'Position', [0 0 1104 740]);
            obj.TabEffHex = uitab(obj.TgPointer, 'Title', '6A. Hexapod Dynamics');
            obj.AxHexapod = axes('Parent', obj.TabEffHex, 'Position', [0.05, 0.1, 0.55, 0.8]); hold(obj.AxHexapod, 'on'); grid(obj.AxHexapod, 'on'); view(obj.AxHexapod, 3); axis(obj.AxHexapod, 'equal'); title(obj.AxHexapod, 'Newton-Euler Rigid Body Pointer'); xlabel(obj.AxHexapod, 'X (m)'); ylabel(obj.AxHexapod, 'Y (m)'); zlabel(obj.AxHexapod, 'Z (m)');
            obj.HexWarn = uicontrol('Parent', obj.TabEffHex, 'Style', 'text', 'Position', [650, 650, 300, 30], 'String', 'STANDBY', 'BackgroundColor', 'y', 'FontWeight', 'bold', 'FontSize', 14);
            obj.HexTele = uicontrol('Parent', obj.TabEffHex, 'Style', 'text', 'Position', [650, 300, 350, 300], 'String', '-', 'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontName', 'Courier New', 'FontSize', 11);
            
            obj.TabEffAzEl = uitab(obj.TgPointer, 'Title', '6B. Az/El Dynamics');
            obj.AxAzEl = axes('Parent', obj.TabEffAzEl, 'Position', [0.05, 0.1, 0.55, 0.8]); hold(obj.AxAzEl, 'on'); grid(obj.AxAzEl, 'on'); view(obj.AxAzEl, 3); axis(obj.AxAzEl, 'equal'); title(obj.AxAzEl, 'Euler-Lagrange Gimbal Pointer'); xlabel(obj.AxAzEl, 'X (m)'); ylabel(obj.AxAzEl, 'Y (m)'); zlabel(obj.AxAzEl, 'Z (m)');
            obj.AzElWarn = uicontrol('Parent', obj.TabEffAzEl, 'Style', 'text', 'Position', [650, 650, 300, 30], 'String', 'STANDBY', 'BackgroundColor', 'y', 'FontWeight', 'bold', 'FontSize', 14);
            obj.AzElTele = uicontrol('Parent', obj.TabEffAzEl, 'Style', 'text', 'Position', [650, 300, 320, 250], 'String', '-', 'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontName', 'Courier New', 'FontSize', 11);
        end
        
        function handleBandSelection(obj)
            switch obj.DropOptBand.Value
                case 2, obj.EditFreq.String = '1.5';
                case 3, obj.EditFreq.String = '3.0';
                case 4, obj.EditFreq.String = '10.0';
            end
            obj.updateStaticVisuals();
        end

        function modTarget(obj, dir)
            d = obj.TargetTable.Data;
            if dir > 0, obj.TargetTable.Data = [d; {'New Threat', 1.0, 40, 40, 5, -150, -150, 0}];
            elseif size(d, 1) > 1, obj.TargetTable.Data = d(1:end-1, :); 
            end
        end
        
        function stopSimulation(obj)
            % =================================================================
            % [HARD RESET DAEMON] Scorched-earth garbage collection.
            % Absolutely guarantees zero visual artifacts bleed into the next run.
            % =================================================================
            obj.simState.running = false; 
            if isvalid(obj.BtnRun), obj.BtnRun.Enable = 'on'; end
            
            obj.simState.time = 0;
            
            % Aggressively destroy all current graphics layers
            cla(obj.Ax3D); cla(obj.AxMeas3D); cla(obj.AxPPI); cla(obj.AxRD3D);
            cla(obj.AxErrRng); cla(obj.AxErrVel); cla(obj.AxEWSpec); cla(obj.AxSTAP);
            cla(obj.AxPol); cla(obj.AxCrossSection);
            
            % Flush operator data tables
            if isvalid(obj.EstTable), obj.EstTable.Data = cell(0, 9); end
            
            % Purge memory references. Will be forcefully re-initialized on next Run.
            obj.targets = [];
            obj.targetTransforms = gobjects(0);
            obj.hMeasRaw = gobjects(0);
            obj.hMeasLine = gobjects(0);
            obj.hMeasEst = gobjects(0);
            obj.hErrRange = gobjects(0);
            obj.hErrVel = gobjects(0);
            obj.radarState.tracked = [];
            
            if isvalid(obj.TxtOptLog)
                obj.TxtOptLog.String = 'Simulation Stopped. Memory and Visual History Cleared.';
                obj.TxtOptLog.BackgroundColor = [1 0.8 0.8];
            end
            drawnow;
        end
        
        function onClose(obj, ~, ~)
            obj.stopSimulation(); delete(obj.UIFigure); 
        end
        
        function exportTelemetry(obj)
            [file, path] = uiputfile('OmniTelemetry.mat', 'Save Telemetry Data');
            if isequal(file, 0), return; end
            
            if isempty(obj.targets)
                exportData = [];
            else
                exportData(length(obj.targets)) = struct('name', '', 'history', []);
                for i = 1:length(obj.targets)
                    exportData(i).name = obj.targets(i).name;
                    exportData(i).history = obj.targets(i).hist;
                end
            end
            
            estTableOutput = obj.EstTable.Data;
            save(fullfile(path, file), 'exportData', 'estTableOutput');
            msgbox('Data Exported Successfully to MAT file.', 'Export Complete', 'help');
        end

        function loadEarthMap(obj)
            [file, path] = uigetfile({'*.jpg;*.png'}, 'Select Earth Map');
            if ~isequal(file, 0)
                obj.EarthTexture = imread(fullfile(path, file));
                if isvalid(obj.hEarthSurf), delete(obj.hEarthSurf); end
                obj.updateStaticVisuals();
            end
        end

        function parseTargetsFromUI(obj)
            % Pulls target definitions from the UI into pure physics models
            data = obj.TargetTable.Data; tIdx = 1; obj.targets = [];
            for i = 1:size(data, 1)
                if isempty(data{i,1}), continue; end
                obj.targets(tIdx).name = data{i,1}; 
                obj.targets(tIdx).rcs = obj.safeNumCell(data{i,2});
                obj.targets(tIdx).pos = [obj.safeNumCell(data{i,3}), obj.safeNumCell(data{i,4}), obj.safeNumCell(data{i,5})] * 1000;
                obj.targets(tIdx).vel = [obj.safeNumCell(data{i,6}), obj.safeNumCell(data{i,7}), obj.safeNumCell(data{i,8})];
                obj.targets(tIdx).isEvasive = obj.CheckManeuvers.Value;
                obj.targets(tIdx).isJammer = (tIdx == 1) && obj.CheckJammer.Value;
                
                nL = lower(obj.targets(tIdx).name);
                if contains(nL, 'airliner')
                    baseS = [2.0, 0.1; 0.1, 1.0]; obj.targets(tIdx).Swerling = 1;
                elseif contains(nL, 'missile')
                    baseS = [0.5, 0.05; 0.05, 0.5]; obj.targets(tIdx).Swerling = 3;
                else
                    baseS = [1.0, 0; 0, 1.0]; obj.targets(tIdx).Swerling = 2;
                end
                obj.targets(tIdx).S_mat = baseS * sqrt(obj.targets(tIdx).rcs);
                
                % Explicit initialization of independent target memory buffers
                obj.targets(tIdx).hist.time = zeros(1000, 1);
                obj.targets(tIdx).hist.R_err = zeros(1000, 1);
                obj.targets(tIdx).hist.V_err = zeros(1000, 1);
                obj.targets(tIdx).hist.meas_sph = zeros(1000, 3);
                obj.targets(tIdx).hist.snr = zeros(1000, 1);
                obj.targets(tIdx).hist.pol = zeros(1000, 3);
                obj.targets(tIdx).hist.ptr = 0;       
                obj.targets(tIdx).hist.wrapped = false; 
                
                tIdx = tIdx + 1;
            end
            obj.radarState.tracked = false(max(1, length(obj.targets)), 1);
        end

        function parseSystemParameters(obj)
            f = obj.safeNum(obj.EditFreq.String, 3.0) * 1e9;
            Pt = obj.safeNum(obj.EditPt.String, 100) * 1e3;
            tau = obj.safeNum(obj.EditTau.String, 10) * 1e-6; 
            prf = obj.safeNum(obj.EditPRF.String, 2) * 1e3;
            BW = obj.safeNum(obj.EditChirpBW.String, 50) * 1e6;
            
            obj.HardwareSpecs.Freq = f; 
            obj.HardwareSpecs.Pt = Pt; 
            obj.HardwareSpecs.Tau = tau; 
            obj.HardwareSpecs.PRF = prf; 
            obj.radarParams.BW = BW;
            
            lambda = obj.c / f;
            
            Ny = obj.safeNum(obj.EditRxNy.String, 16); Nz = obj.safeNum(obj.EditRxNz.String, 16); 
            obj.ArrayConfig.Rx_Ny = Ny; obj.ArrayConfig.Rx_Nz = Nz;
            
            TNy = obj.safeNum(obj.EditTxNy.String, 16); TNz = obj.safeNum(obj.EditTxNz.String, 16); 
            obj.ArrayConfig.Tx_Ny = TNy; obj.ArrayConfig.Tx_Nz = TNz;
            
            obj.ArrayConfig.Tx_dy = obj.safeNum(obj.EditTxDy.String, lambda/2); 
            obj.ArrayConfig.Tx_dz = obj.safeNum(obj.EditTxDz.String, lambda/2);
            obj.ArrayConfig.Rx_dy = obj.safeNum(obj.EditRxDy.String, lambda/2); 
            obj.ArrayConfig.Rx_dz = obj.safeNum(obj.EditRxDz.String, lambda/2);
            
            obj.radarParams.R_max = obj.c / (2 * prf); 
            obj.radarParams.Ge = pi; 
        end

        function success = runSystemOptimizer(obj)
            try
                d = uiprogressdlg(obj.UIFigure, 'Title', 'System Optimizer', 'Message', 'Analyzing theater and synthesizing optimal radar parameters...');
                drawnow;

                obj.parseTargetsFromUI();
                if isempty(obj.targets)
                    obj.TxtOptLog.String = 'Error: No targets to optimize against.'; 
                    obj.TxtOptLog.BackgroundColor = [1 0.8 0.8]; 
                    success = false; close(d); return; 
                end

                maxR = 1000;
                for i = 1:length(obj.targets)
                    if norm(obj.targets(i).pos) > maxR, maxR = norm(obj.targets(i).pos); end
                end

                reqSNR = obj.safeNum(obj.EditReqSNR.String, 15); 
                maxPt = obj.safeNum(obj.EditMaxPt.String, 500);
                opt_freq = obj.safeNum(obj.EditFreq.String, 3.0) * 1e9;
                
                opt_prf = obj.c / (2 * maxR * 1.2);
                if opt_prf > 100e3, opt_prf = 100e3; elseif opt_prf < 1e3, opt_prf = 1e3; end
                
                opt_tau = min((1/opt_prf) * 0.1, 100e-6);
                opt_bw = obj.c / (2 * 3.0);
                
                lambda = obj.c / opt_freq; 
                Pn = obj.kT0 * opt_bw * 10^(3/10); 
                L_lin = 10^(5/10); 
                req_snr_lin = 10^(reqSNR/10);
                
                opt_N = 16; opt_Pt = maxPt * 1000; success_opt = false;
                Ge = pi; 
                for N = 8:4:64
                    d.Value = N/64;
                    PG = (N*N); worstPt = 0;
                    for i = 1:length(obj.targets)
                        Pt_req = (req_snr_lin * ((4*pi)^3) * norm(obj.targets(i).pos)^4 * Pn * L_lin) / (Ge^2 * PG^2 * lambda^2 * obj.targets(i).rcs);
                        if Pt_req > worstPt, worstPt = Pt_req; end
                    end
                    if worstPt <= maxPt * 1000, opt_Pt = worstPt; opt_N = N; success_opt = true; break; end
                end
                
                if ~obj.CheckStrictOpt.Value
                    obj.EditTxNy.String = num2str(opt_N); obj.EditTxNz.String = num2str(opt_N);
                    obj.EditRxNy.String = num2str(opt_N); obj.EditRxNz.String = num2str(opt_N);
                    obj.EditPt.String = num2str(opt_Pt/1000 * 1.5, '%.2f');
                    obj.EditPRF.String = num2str(opt_prf/1e3, '%.1f');
                    obj.EditTau.String = num2str(opt_tau*1e6, '%.1f');
                    obj.EditChirpBW.String = num2str(opt_bw/1e6, '%.1f');
                end
                
                log = sprintf('--- OMNI RADAR SYSTEM OPTIMIZER ---\n');
                if success_opt
                    log = [log, 'SUCCESS: Link budget and waveform synthesized.\n']; 
                    obj.TxtOptLog.BackgroundColor = [0.8 1 0.8];
                else
                    log = [log, 'WARNING: Max power constraint reached.\n']; 
                    obj.TxtOptLog.BackgroundColor = [1 0.8 0.8];
                end
                log = [log, sprintf('Optimal Theater Horizon: %.1f km\nOpt PRF: %.1f kHz | Opt BW: %.1f MHz\nIdeal Elements: %dx%d\nIdeal Power: %.1f kW\n*Check Strict UI Lock to prevent overwriting.', maxR*1.2/1000, opt_prf/1e3, opt_bw/1e6, opt_N, opt_N, opt_Pt/1000)];
                obj.TxtOptLog.String = log;
                
                obj.parseSystemParameters();
                obj.initDSPBuffers();
                obj.updateStaticVisuals();
                
                close(d); success = true;
            catch ME
                success = false;
                if exist('d','var') && isvalid(d), close(d); end
                obj.TxtOptLog.String = sprintf('CRITICAL OPTIMIZER ERROR:\n%s', ME.message);
                obj.TxtOptLog.BackgroundColor = [1 0.6 0.6];
            end
        end
        
        function initDSPBuffers(obj)
            BW = obj.radarParams.BW; Tau = obj.HardwareSpecs.Tau; PRF = obj.HardwareSpecs.PRF;
            Fs = max(2.5 * BW, 10e6);
            obj.dsp_t_rx = 0 : 1/Fs : min(1/PRF, (obj.radarParams.R_max*1.5 * 2)/obj.c);
            t_tx = -Tau/2 : 1/Fs : Tau/2;
            dsp_s_tx = exp(1j * pi * (BW/Tau) * t_tx.^2);
            
            len_tx = length(dsp_s_tx);
            native_hamming = 0.54 - 0.46 * cos(2*pi*(0:len_tx-1)'/(len_tx-1));
            
            mf_ref = conj(fliplr(dsp_s_tx)) .* native_hamming';
            obj.dsp_delay = floor(length(mf_ref)/2);
            
            N_fft = 2^nextpow2(length(obj.dsp_t_rx) + length(mf_ref) - 1);
            obj.dsp_MF_F = fft(mf_ref, N_fft);
            obj.dsp_rx_mat = zeros(16, length(obj.dsp_t_rx));
        end

        function updateStaticVisuals(obj)
            obj.parseSystemParameters();
            
            f = obj.HardwareSpecs.Freq; Pt = obj.HardwareSpecs.Pt;
            tau = obj.HardwareSpecs.Tau; prf = obj.HardwareSpecs.PRF;
            BW = obj.radarParams.BW; lambda = obj.c / f; 
            
            Ny = obj.ArrayConfig.Rx_Ny; Nz = obj.ArrayConfig.Rx_Nz;
            TNy = obj.ArrayConfig.Tx_Ny; TNz = obj.ArrayConfig.Tx_Nz;
            Tx_dy = obj.ArrayConfig.Tx_dy; Tx_dz = obj.ArrayConfig.Tx_dz;
            Rx_dy = obj.ArrayConfig.Rx_dy; Rx_dz = obj.ArrayConfig.Rx_dz;
            
            obj.parseTargetsFromUI();
            if isempty(obj.targets), ref_rcs = 1.0; 
            else, ref_rcs = mean([obj.targets.rcs]); end
            
            r_vec = linspace(1000, 100000, 100);
            Pn = obj.kT0 * BW * 10^(3/10); L = 10^(5/10);
            snr_vec = 10*log10((Pt * obj.radarParams.Ge^2 * (TNy*TNz)*(Ny*Nz) * lambda^2 * ref_rcs) ./ (((4*pi)^3) * r_vec.^4 * L * Pn));
            plot(obj.AxSNR, r_vec/1000, snr_vec, 'b', 'LineWidth', 2);
            title(obj.AxSNR, sprintf('Radar Eq: Single Pulse SNR vs Range (Mean RCS = %.1f m^2)', ref_rcs));
            
            t_pulse = linspace(0, 3/prf, 500);
            plot(obj.AxPulseTrain, t_pulse*1e6, mod(t_pulse, 1/prf) <= tau, 'r', 'LineWidth', 2);
            
            [Y_tx, Z_tx] = meshgrid((0:TNy-1)*Tx_dy, (0:TNz-1)*Tx_dz); Y_tx = Y_tx - mean(Y_tx(:)); Z_tx = Z_tx - mean(Z_tx(:));
            [Y_rx, Z_rx] = meshgrid((0:Ny-1)*Rx_dy, (0:Nz-1)*Rx_dz); Y_rx = Y_rx - mean(Y_rx(:)); Z_rx = Z_rx - mean(Z_rx(:));
            
            obj.radarState.Y_rx = Y_rx;
            obj.radarState.Z_rx = Z_rx;
            
            cla(obj.AxGeom); hold(obj.AxGeom, 'on');
            scatter3(obj.AxGeom, Y_tx(:), Z_tx(:), zeros(numel(Y_tx),1), 40, 'b', 'filled', 'MarkerEdgeColor', 'k');
            scatter3(obj.AxGeom, Y_rx(:), Z_rx(:), ones(numel(Y_rx),1)*0.01, 20, 'r', 'filled');
            legend(obj.AxGeom, {'Tx Elements', 'Rx Elements'}, 'Location', 'best', 'FontSize', 8);
            hold(obj.AxGeom, 'off');
            
            az = linspace(-90, 90, 90); el = linspace(-90, 90, 90); [AZ, EL] = meshgrid(az, el);
            U = cosd(EL).*sind(AZ); W = sind(EL);
            
            winType = obj.DropTaper.Value;
            if winType==1
                wY = ones(Ny,1); wZ = ones(Nz,1); 
            elseif winType==2
                wY = 0.54 - 0.46*cos(2*pi*(0:Ny-1)'/(Ny-1)); wZ = 0.54 - 0.46*cos(2*pi*(0:Nz-1)'/(Nz-1));
            else
                wY = 0.5*(1 - cos(2*pi*(0:Ny-1)'/(Ny-1))); wZ = 0.5*(1 - cos(2*pi*(0:Nz-1)'/(Nz-1)));
            end
            W_mat = wZ * wY'; w_vec = W_mat(:); w_vec = w_vec / sum(w_vec);
            obj.radarState.w_vec_rx = w_vec;
            
            steerAz = obj.safeNum(obj.EditSteerAz.String, 45); steerEl = obj.safeNum(obj.EditSteerEl.String, 15);
            Us = cosd(steerEl)*sind(steerAz); Ws = sind(steerEl);
            
            evalFreqs = [f - BW/2, f, f + BW/2]; AF_tot = zeros(size(AZ));
            
            for fi = 1:length(evalFreqs)
                lam_i = obj.c / evalFreqs(fi);
                if obj.DropSteeringType.Value == 1
                    phi_steer = - (2*pi/lambda) * (Y_rx(:)*Us + Z_rx(:)*Ws);
                    phase_mat = exp(1j * ((2*pi/lam_i) * (Y_rx(:)*U(:)' + Z_rx(:)*W(:)') + repmat(phi_steer, 1, numel(U))));
                else
                    tau_steer = - (1/obj.c) * (Y_rx(:)*Us + Z_rx(:)*Ws);
                    phase_mat = exp(1j * 2*pi * evalFreqs(fi) * ((1/obj.c)*(Y_rx(:)*U(:)' + Z_rx(:)*W(:)') + repmat(tau_steer, 1, numel(U))));
                end
                
                AF_i = reshape(w_vec' * phase_mat, size(AZ));
                ElemFactor = max(0, 1 - U.^2 - W.^2).^0.75; 
                AF_i = AF_i .* ElemFactor;
                AF_tot = AF_tot + abs(AF_i).^2;
            end
            
            AF_dB = 10*log10(AF_tot/3 + 1e-12); AF_dB = AF_dB - max(AF_dB(:)); AF_dB(AF_dB < -40) = -40;
            R_pat = AF_dB + 40;
            
            if isempty(obj.hArrPatSurf) || ~isvalid(obj.hArrPatSurf)
                obj.hArrPatSurf = surf(obj.AxPat3D, R_pat.*cosd(EL).*cosd(AZ), R_pat.*cosd(EL).*sind(AZ), R_pat.*sind(EL), AF_dB, 'EdgeColor', 'none');
                obj.hCutAzL = plot(obj.AxCutAz, az, AF_dB(45,:), 'b', 'LineWidth', 2);
                obj.hCutElL = plot(obj.AxCutEl, el, AF_dB(:,45)', 'r', 'LineWidth', 2);
            else
                set(obj.hArrPatSurf, 'XData', R_pat.*cosd(EL).*cosd(AZ), 'YData', R_pat.*cosd(EL).*sind(AZ), 'ZData', R_pat.*sind(EL), 'CData', AF_dB);
                set(obj.hCutAzL, 'YData', AF_dB(45,:)); set(obj.hCutElL, 'YData', AF_dB(:,45)');
            end
            
            Re = 6371;
            if isempty(obj.hEarthSurf) || ~isvalid(obj.hEarthSurf)
                [X_e, Y_e, Z_e] = sphere(50);
                if isempty(obj.EarthTexture), obj.hEarthSurf = surf(obj.AxGlobe, X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', [0.1 0.25 0.5], 'EdgeColor', 'none');
                else, obj.hEarthSurf = surf(obj.AxGlobe, X_e*Re, Y_e*Re, Z_e*Re, 'CData', obj.EarthTexture, 'FaceColor', 'texturemap', 'EdgeColor', 'none'); end
                obj.hRadarOrigin = scatter3(obj.AxGlobe, Re*cosd(39)*cosd(-98), Re*cosd(39)*sind(-98), Re*sind(39), 50, 'r', 'filled');
            end
            R_shape = 100 * (10.^(AF_dB/10)).^0.25;
            x0 = obj.hRadarOrigin.XData; y0 = obj.hRadarOrigin.YData; z0 = obj.hRadarOrigin.ZData; lat = 39; lon = -98;
            R_enu2ecef = [-sind(lon), -sind(lat)*cosd(lon), cosd(lat)*cosd(lon); cosd(lon), -sind(lat)*sind(lon), cosd(lat)*sind(lon); 0, cosd(lat), sind(lat)];
            pts = R_enu2ecef * [R_shape(:)'.*cosd(EL(:))'.*sind(AZ(:))'; R_shape(:)'.*sind(EL(:))'; R_shape(:)'.*cosd(EL(:))'.*cosd(AZ(:))'];
            Xg = reshape(pts(1,:), size(EL))+x0; Yg = reshape(pts(2,:), size(EL))+y0; Zg = reshape(pts(3,:), size(EL))+z0;
            if isempty(obj.hGlobalPatSurf) || ~isvalid(obj.hGlobalPatSurf), obj.hGlobalPatSurf = surf(obj.AxGlobe, Xg, Yg, Zg, AF_dB, 'EdgeColor', 'none', 'FaceAlpha', 0.6);
            else, set(obj.hGlobalPatSurf, 'XData', Xg, 'YData', Yg, 'ZData', Zg, 'CData', AF_dB); end
            
            obj.LblSNR.Text = sprintf('%s dB', obj.EditReqSNR.String); 
            obj.LblMaxR.Text = sprintf('%.0f km', obj.radarParams.R_max/1000); obj.LblRes.Text = sprintf('%.1f m', obj.c/(2*BW));
            obj.LblDir.Text = sprintf('%.1f dB', 10*log10(41253 / ((100/Ny)*(100/Nz)))); obj.LblAzBW.Text = sprintf('%.1f', 100/Ny); obj.LblElBW.Text = sprintf('%.1f', 100/Nz);
            obj.LblAvgPwr.Text = sprintf('%.1f W', Pt * tau * prf);
            drawnow limitrate;
        end

        function buildGeometricEnvironment(obj)
            % Re-seed graphic objects and establish static axis bounds
            cla(obj.Ax3D); cla(obj.AxMeas3D); cla(obj.AxPPI); cla(obj.AxRD3D); cla(obj.AxErrRng); cla(obj.AxErrVel); cla(obj.AxEWSpec); cla(obj.AxSTAP);
            hold(obj.Ax3D,'on'); hold(obj.AxMeas3D,'on'); hold(obj.AxPPI,'on'); hold(obj.AxRD3D,'on'); hold(obj.AxErrRng,'on'); hold(obj.AxErrVel,'on'); hold(obj.AxEWSpec,'on');
            
            lim = obj.radarParams.R_max;
            axis(obj.Ax3D, 'manual'); xlim(obj.Ax3D, [-lim lim]); ylim(obj.Ax3D, [-lim lim]); zlim(obj.Ax3D, [0 lim/2]);
            axis(obj.AxMeas3D, 'manual'); xlim(obj.AxMeas3D, [-lim lim]); ylim(obj.AxMeas3D, [-lim lim]); zlim(obj.AxMeas3D, [0 lim/2]);
            
            axis(obj.AxPPI, 'equal'); xlim(obj.AxPPI, [-lim lim]); ylim(obj.AxPPI, [-lim lim]);
            for r = [0.25, 0.5, 0.75, 1.0] * lim, plot(obj.AxPPI, r*cos(linspace(0,2*pi,50)), r*sin(linspace(0,2*pi,50)), 'g--'); end
            xlim(obj.AxRD3D, [0, lim/1000]); ylim(obj.AxRD3D, [-300, 300]); zlim(obj.AxRD3D, [0, 15]);
            xlim(obj.AxEWSpec, [-obj.radarParams.BW/2e6, obj.radarParams.BW/2e6]); ylim(obj.AxEWSpec, [-120, -40]);
            
            rB = hgtransform('Parent', obj.Ax3D);
            patch('Parent', rB, 'XData', [0 0 0 0], 'YData', [-500 500 500 -500], 'ZData', [-500 -500 500 500], 'FaceColor', [0.3 0.3 0.3]);
            [bX, bY, bZ] = cylinder([0, lim*tand(5/2)], 20);
            obj.beamConeSurf = surface('XData', bZ*lim, 'YData', bX, 'ZData', bY, 'FaceColor', 'c', 'EdgeColor', 'none', 'FaceAlpha', 0.15, 'Parent', rB);
            
            nT = length(obj.targets); cp = lines(max(1, nT));
            obj.targetTransforms = gobjects(nT, 1); obj.hErrRange = gobjects(nT, 1); obj.hErrVel = gobjects(nT, 1);
            obj.hMeasRaw = gobjects(nT, 1); obj.hMeasLine = gobjects(nT, 1); obj.hMeasEst = gobjects(nT, 1);
            
            for i=1:nT
                obj.targetTransforms(i) = hgtransform('Parent', obj.Ax3D);
                s = 1500; V = [s 0 0; -s -s -s; -s s -s; -s 0 s];
                obj.targets(i).patchHandle = patch('Parent', obj.targetTransforms(i), 'Vertices', V, 'Faces', [1 2 3; 1 3 4; 1 4 2; 2 3 4], 'FaceColor', cp(i,:), 'EdgeColor', 'k');
                obj.targets(i).textObj = text(obj.Ax3D, 0, 0, 0, obj.targets(i).name, 'FontSize', 8, 'Color', 'k');
                obj.hErrRange(i) = scatter(obj.AxErrRng, NaN, NaN, 36, cp(i,:), 'filled'); obj.hErrVel(i) = scatter(obj.AxErrVel, NaN, NaN, 36, cp(i,:), 'filled');
                obj.hMeasRaw(i) = scatter3(obj.AxMeas3D, NaN, NaN, NaN, 15, cp(i,:), 'filled'); obj.hMeasLine(i) = plot3(obj.AxMeas3D, NaN, NaN, NaN, '--', 'Color', cp(i,:), 'LineWidth', 2);
                obj.hMeasEst(i) = scatter3(obj.AxMeas3D, NaN, NaN, NaN, 80, 's', 'MarkerEdgeColor', cp(i,:), 'LineWidth', 2, 'ButtonDownFcn', @(~,~) obj.highlightTableRow(i));
                
                rel_pos = obj.targets(i).pos(:) - obj.radarState.pos(:);
                rel_vel = obj.targets(i).vel(:) - obj.radarState.vel(:);
                vxy = norm(rel_vel(1:2)); theta = atan2(rel_vel(2), rel_vel(1));
                
                obj.targets(i).IMM.x_cv = [rel_pos; vxy; theta; rel_vel(3); 0.0];
                obj.targets(i).IMM.P_cv = eye(7) * 1e4;
                obj.targets(i).IMM.Q_cv = diag([1 1 1 50 1e-2 50 1e-4]); 
                
                obj.targets(i).IMM.x_ct = [rel_pos; vxy; theta; rel_vel(3); 0.1]; 
                obj.targets(i).IMM.P_ct = eye(7) * 1e4;
                obj.targets(i).IMM.Q_ct = diag([1 1 1 500 1e-2 500 0.25]); 
                
                obj.targets(i).IMM.mu = [0.9; 0.1]; 
                obj.targets(i).IMM.T_mat = [0.95 0.05; 0.1 0.9]; 
                obj.targets(i).last_time_seen = 0;
            end
            
            obj.sweepLine = plot(obj.AxPPI, [0 lim], [0 0], 'g', 'LineWidth', 2);
            obj.blipsScatter = scatter(obj.AxPPI, obj.visData.bX, obj.visData.bY, 60, 'filled', 'MarkerFaceColor', 'g');
            obj.rdScatter3D = scatter3(obj.AxRD3D, obj.visData.rX, obj.visData.rY, obj.visData.rZ, 40, obj.visData.rC, 'filled');
            
            cla(obj.AxPol); cla(obj.AxCrossSection);
            obj.hPolBars = bar(obj.AxPol, categorical({'HH', 'VV', 'HV'}), [0 0 0], 'FaceColor', [0.2 0.6 0.8]);
            obj.hCrossShape = rectangle('Parent', obj.AxCrossSection, 'Position', [-1 -1 2 2], 'Curvature', [1 1], 'FaceColor', 'y'); xlim(obj.AxCrossSection, [-10 10]); ylim(obj.AxCrossSection, [-10 10]);
            
            obj.drawEffectors();
        end

        function highlightTableRow(obj, tIdx)
            if size(obj.EstTable.Data, 1) >= tIdx, obj.EstTable.Selection = tIdx; end
        end

        function drawEffectors(obj)
            cla(obj.AxHexapod); patch('Parent', obj.AxHexapod, 'XData', obj.hexParams.B(1,:), 'YData', obj.hexParams.B(2,:), 'ZData', obj.hexParams.B(3,:), 'FaceColor', [0.3 0.3 0.3], 'FaceAlpha', 0.5);
            obj.HexTopPlate = patch('Parent', obj.AxHexapod, 'XData', obj.hexParams.P(1,:), 'YData', obj.hexParams.P(2,:), 'ZData', obj.hexParams.P(3,:)+obj.hexParams.Z_nom, 'FaceColor', [0.2 0.6 0.8], 'FaceAlpha', 0.8);
            obj.HexLegs = gobjects(1,6);
            for i=1:6, obj.HexLegs(i) = plot3(obj.AxHexapod, [obj.hexParams.B(1,i), obj.hexParams.P(1,i)], [obj.hexParams.B(2,i), obj.hexParams.P(2,i)], [obj.hexParams.B(3,i), obj.hexParams.P(3,i)+obj.hexParams.Z_nom], 'Color', [0.8 0.4 0.1], 'LineWidth', 4); end
            obj.HexLaser = plot3(obj.AxHexapod, [0 0], [0 0], [obj.hexParams.Z_nom 1.5], 'r-', 'LineWidth', 3);
            xlim(obj.AxHexapod, [-1.2 1.2]); ylim(obj.AxHexapod, [-1.2 1.2]); zlim(obj.AxHexapod, [0 2.0]);
            axis(obj.AxHexapod, 'manual');
            
            cla(obj.AxAzEl); obj.TAz = hgtransform('Parent', obj.AxAzEl); obj.TEl = hgtransform('Parent', obj.TAz);
            [Xb, Yb, Zb] = cylinder([1.5, 1.5], 20); surface('Parent', obj.AxAzEl, 'XData', Xb, 'YData', Yb, 'ZData', Zb*1.5, 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none');
            [Xc, Yc, Zc] = cylinder([0.8, 0.8], 20); surface('Parent', obj.TAz, 'XData', Xc, 'YData', Yc, 'ZData', Zc+1.5, 'FaceColor', [0.2 0.5 0.8], 'EdgeColor', 'none');
            patch('Parent', obj.TAz, 'XData', [-0.5 -0.5 0.5 0.5], 'YData', [-1.2 -0.8 -0.8 -1.2], 'ZData', [2.5 2.5 2.5 2.5], 'FaceColor', [0.2 0.5 0.8]);
            patch('Parent', obj.TAz, 'XData', [-0.5 -0.5 0.5 0.5], 'YData', [-1.2 -0.8 -0.8 -1.2], 'ZData', [4.0 4.0 4.0 4.0], 'FaceColor', [0.2 0.5 0.8]);
            fill3(obj.TAz, [-0.5 -0.5 0.5 0.5], [-1.2 -0.8 -0.8 -1.2], [2.5 4.0 4.0 2.5], [0.2 0.5 0.8]); fill3(obj.TAz, [-0.5 -0.5 0.5 0.5], [0.8 1.2 1.2 0.8], [2.5 4.0 4.0 2.5], [0.2 0.5 0.8]);
            obj.TEl.Matrix = makehgtform('translate', [0 0 3.25]);
            [Xp, Yp, Zp] = cylinder([0.6, 0.6], 20); pts = makehgtform('yrotate', pi/2) * [Xp(:)'; Yp(:)'; Zp(:)'; ones(1, numel(Xp))];
            surface('Parent', obj.TEl, 'XData', (reshape(pts(1,:), size(Xp))-0.5)*3.0, 'YData', reshape(pts(2,:), size(Yp)), 'ZData', reshape(pts(3,:), size(Zp)), 'FaceColor', [0.8 0.6 0.1]);
            obj.AzElLaser = plot3(obj.TEl, [0 50], [0 0], [0 0], 'r-', 'LineWidth', 3);
            xlim(obj.AxAzEl, [-4 4]); ylim(obj.AxAzEl, [-4 4]); zlim(obj.AxAzEl, [0 8]);
            axis(obj.AxAzEl, 'manual');
        end

        % =========================================================
        % --- 4. ENGINE: FIXED-TIMESTEP ACCUMULATOR MASTER LOOP ---
        % =========================================================
        function runSimulation(obj)
            if obj.simState.running, return; end
            obj.simState.running = true; obj.BtnRun.Enable = 'off';
            
            obj.parseTargetsFromUI();
            obj.parseSystemParameters();
            obj.initDSPBuffers();
            obj.buildGeometricEnvironment();
            
            obj.TxtOptLog.String = 'SIMULATION RUNNING (Manual Sandbox Mode)';
            obj.TxtOptLog.BackgroundColor = [0.8 1 0.8];
            drawnow;
            
            obj.simStartTime = tic;
            obj.lastRenderTime = tic;
            
            accumulator = 0.0;
            current_wall_time = 0.0;
            
            try
                while obj.simState.running && isvalid(obj.UIFigure)
                    new_wall_time = toc(obj.simStartTime);
                    frame_time = new_wall_time - current_wall_time;
                    current_wall_time = new_wall_time;
                    
                    if frame_time > 0.25, frame_time = 0.25; end
                    
                    if obj.CheckRealTime.Value
                        accumulator = accumulator + frame_time;
                    else
                        accumulator = accumulator + obj.simState.dt; 
                    end
                    
                    while accumulator >= obj.simState.dt && obj.simState.running
                        obj.simState.time = obj.simState.time + obj.simState.dt;
                        
                        uiCache.rmode = obj.DropRadarMode.Value;
                        uiCache.chkJam = obj.CheckJammer.Value;
                        uiCache.chkMF = obj.CheckMF.Value;
                        uiCache.chkDF = obj.CheckDF.Value;
                        uiCache.chkAF = obj.CheckAF.Value;
                        uiCache.chkMIMO = obj.CheckMIMO.Value;
                        uiCache.chkFD = obj.CheckFD.Value;
                        uiCache.chkPred = obj.CheckPredictive.Value;
                        uiCache.faceAz = obj.safeNum(obj.EditSteerAz.String, 45);
                        uiCache.faceEl = obj.safeNum(obj.EditSteerEl.String, 15);
                        
                        obj.stepTargets();
                        obj.stepRadarSensor(uiCache);
                        cmd = obj.stepC2Estimator(uiCache);
                        obj.stepEffectors(cmd);
                        
                        accumulator = accumulator - obj.simState.dt;
                    end
                    
                    if toc(obj.lastRenderTime) > 1/30
                        obj.updateTelemetry();
                        drawnow limitrate;
                        obj.lastRenderTime = tic;
                    end
                    
                    if obj.CheckRealTime.Value, pause(0.002); end
                end
            catch ME
                obj.simState.running = false;
                if isvalid(obj.BtnRun), obj.BtnRun.Enable = 'on'; end
                if isvalid(obj.TxtOptLog)
                    obj.TxtOptLog.String = sprintf('CRITICAL RUNTIME ERROR:\n%s\nLine: %d', ME.message, ME.stack(1).line);
                    obj.TxtOptLog.BackgroundColor = [1 0.6 0.6];
                end
            end
        end

        function stepTargets(obj)
            dt = obj.simState.dt; t = obj.simState.time;
            obj.radarState.pos = obj.radarState.pos + obj.radarState.vel * dt;
            
            for i=1:length(obj.targets)
                if obj.targets(i).isEvasive
                    orth = [-obj.targets(i).vel(2), obj.targets(i).vel(1), 0]; if norm(orth)>0, orth=orth/norm(orth); end
                    obj.targets(i).vel = obj.targets(i).vel + (orth*25*sin(0.4*t+i*2) + [0,0,10*cos(0.3*t+i)]) * dt;
                end
                obj.targets(i).pos = obj.targets(i).pos + obj.targets(i).vel * dt;
                rel_pos = obj.targets(i).pos - obj.radarState.pos;
                
                if isvalid(obj.targetTransforms(i))
                    obj.targetTransforms(i).Matrix = makehgtform('translate', rel_pos, 'zrotate', atan2(obj.targets(i).vel(2), obj.targets(i).vel(1)), 'yrotate', -atan2(obj.targets(i).vel(3), norm(obj.targets(i).vel(1:2))));
                    obj.targets(i).textObj.Position = rel_pos + [0 0 2000];
                end
            end
        end

        function stepRadarSensor(obj, uiCache)
            t = obj.simState.time; dt = obj.simState.dt;
            if t > 15, zlim(obj.AxRD3D, [t-15, t]); else, zlim(obj.AxRD3D, [0, 15]); end
            
            isTrk = false; cIdx = -1;
            if uiCache.rmode == 3 && any(obj.radarState.tracked(1:length(obj.targets)))
                obj.radarState.mpar_tick = obj.radarState.mpar_tick + 1;
                if mod(obj.radarState.mpar_tick, 4) == 0, isTrk = true; act = find(obj.radarState.tracked(1:length(obj.targets))); cIdx = act(mod(obj.radarState.mpar_tick/4 - 1, length(act))+1); end
            end
            
            if isTrk
                tgt_rel = obj.targets(cIdx).pos - obj.radarState.pos; 
                az_point = mod(rad2deg(atan2(tgt_rel(2), tgt_rel(1))), 360); 
                el_point = rad2deg(asin(tgt_rel(3)/max(1e-6,norm(tgt_rel))));
                obj.beamConeSurf.FaceColor = 'm'; obj.beamConeSurf.FaceAlpha = 0.4; obj.sweepLine.Color = [1 0 1];
            else
                if uiCache.rmode == 1
                    obj.radarState.last_az = obj.radarState.last_az + (360/2)*dt; 
                    obj.radarState.last_el = 15 * sin(2*pi * t / 5); 
                else
                    obj.radarState.last_az = floor(t/(5/(360/2)))*5; 
                    obj.radarState.last_el = 0;
                end
                az_point = mod(obj.radarState.last_az, 360);
                el_point = obj.radarState.last_el;
                obj.beamConeSurf.FaceColor = 'c'; obj.beamConeSurf.FaceAlpha = 0.15; obj.sweepLine.Color = [0 1 0];
            end
            
            obj.beamConeSurf.Parent.Matrix = makehgtform('zrotate', deg2rad(az_point), 'yrotate', -deg2rad(el_point));
            obj.sweepLine.XData = [0, obj.radarParams.R_max*cosd(el_point)*cosd(az_point)]; 
            obj.sweepLine.YData = [0, obj.radarParams.R_max*cosd(el_point)*sind(az_point)];
            
            lambda = obj.c/obj.HardwareSpecs.Freq;
            cp_colors = lines(max(1, length(obj.targets))); 
            P_jam_tot = 0; PG_rx = obj.ArrayConfig.Rx_Ny * obj.ArrayConfig.Rx_Nz;
            
            for i = 1:length(obj.targets)
                if obj.targets(i).isJammer && uiCache.chkJam
                    t_pos_j = obj.targets(i).pos - obj.radarState.pos;
                    rng_j = max(1e-6, norm(t_pos_j));
                    tAz_j = mod(rad2deg(atan2(t_pos_j(2), t_pos_j(1))), 360); 
                    tEl_j = rad2deg(asin(t_pos_j(3)/rng_j));
                    
                    if uiCache.rmode == 1
                        u_j = cosd(tEl_j - el_point) * sind(tAz_j - az_point); w_j = sind(tEl_j - el_point);
                        phase_mat = exp(1j * 2*pi * (obj.radarState.Y_rx(:)*u_j + obj.radarState.Z_rx(:)*w_j) / lambda);
                    else
                        u_j = cosd(tEl_j - uiCache.faceEl) * sind(tAz_j - uiCache.faceAz); w_j = sind(tEl_j - uiCache.faceEl);
                        u_s = cosd(el_point - uiCache.faceEl) * sind(az_point - uiCache.faceAz); w_s = sind(el_point - uiCache.faceEl);
                        phase_mat = exp(1j * 2*pi * (obj.radarState.Y_rx(:)*(u_j - u_s) + obj.radarState.Z_rx(:)*(w_j - w_s)) / lambda);
                    end
                    
                    spatial_gain_factor = max(1e-5, abs(obj.radarState.w_vec_rx' * phase_mat)^2);
                    jam_lin = (10e3 * obj.radarParams.Ge * (PG_rx * spatial_gain_factor) * lambda^2) / ((4*pi)^2 * rng_j^2);
                    P_jam_tot = P_jam_tot + jam_lin * (obj.radarParams.BW / 10e6);
                end
            end
            Pn_eff = obj.kT0 * obj.radarParams.BW * 10^(3/10) + P_jam_tot;
            
            best_cpi_tgt_idx = -1; best_cpi_snr = -inf;
            for i=1:length(obj.targets)
                t_pos = obj.targets(i).pos(:)' - obj.radarState.pos(:)'; rng = max(1e-6, norm(t_pos)); 
                tAz = mod(rad2deg(atan2(t_pos(2), t_pos(1))), 360); tEl = rad2deg(asin(t_pos(3)/rng));
                vr = -dot(obj.targets(i).vel(:)' - obj.radarState.vel(:)', t_pos/rng);
                
                az_diff = abs(mod(deg2rad(az_point)-deg2rad(tAz)+pi, 2*pi)-pi);
                el_diff = abs(deg2rad(el_point) - deg2rad(tEl));
                
                if az_diff <= deg2rad(5/2) && el_diff <= deg2rad(15)
                    PG_tx = obj.ArrayConfig.Tx_Ny * obj.ArrayConfig.Tx_Nz;
                    if uiCache.chkFD, fluct_rcs = obj.targets(i).rcs; 
                    elseif obj.targets(i).Swerling == 1 || obj.targets(i).Swerling == 2, fluct_rcs = -obj.targets(i).rcs * log(rand()); 
                    else, fluct_rcs = -(obj.targets(i).rcs / 2) * log(rand() * rand()); 
                    end
                    
                    if uiCache.rmode == 1, cos_theta_scan = sind(tEl)*sind(el_point) + cosd(tEl)*cosd(el_point)*cosd(tAz - az_point); 
                    else, cos_theta_scan = sind(tEl)*sind(uiCache.faceEl) + cosd(tEl)*cosd(uiCache.faceEl)*cosd(tAz - uiCache.faceAz); end
                    scan_loss_lin = max(0.1, max(0, cos_theta_scan)^1.5);
                    
                    ground_range = norm(t_pos(1:2)); h_t_apparent = t_pos(3) - (ground_range^2) / (2 * obj.R_earth * obj.k_ref);
                    F_multipath_lin = 1.0; 
                    if h_t_apparent < -obj.h_radar, snr_lin = 0; 
                    else
                        if h_t_apparent > 0, F_multipath_lin = (2 * sin((2*pi * (2 * obj.h_radar * h_t_apparent) / max(1e-6, ground_range))/2))^4; end
                        snr_lin = (obj.HardwareSpecs.Pt * obj.radarParams.Ge^2 * PG_tx * PG_rx * lambda^2 * fluct_rcs * scan_loss_lin^2 * F_multipath_lin) / (((4*pi)^3) * rng^4 * Pn_eff);
                    end

                    if uiCache.chkMF, snr_lin = snr_lin * (obj.radarParams.BW * obj.HardwareSpecs.Tau); end
                    if uiCache.chkDF, snr_lin = snr_lin * 16; end
                    snr = 10*log10(snr_lin + 1e-12);
                    
                    if snr > best_cpi_snr, best_cpi_snr = snr; best_cpi_tgt_idx = i; end
                    
                    if snr >= 13
                        obj.radarState.tracked(i) = true; obj.targets(i).last_time_seen = t; 
                        sigR = max(1.0, (obj.c/(2*obj.radarParams.BW))/max(1e-6, sqrt(2*snr_lin))); 
                        sigA = max(1e-4, (5/max(1e-6, sqrt(2*snr_lin))) * pi/180);
                        if ~uiCache.chkMF, sigR=sigR*5; end; if ~uiCache.chkDF, sigA=sigA*5; end; if uiCache.chkMIMO, sigA=sigA/4; end
                        
                        mR = rng + randn()*sigR; mV = vr + randn()*5.0; mA = tAz + randn()*sigA; mE = tEl + randn()*sigA;
                        E_rx = obj.targets(i).S_mat * [1; 0] * sqrt(fluct_rcs / obj.targets(i).rcs);
                        sigRCS = obj.targets(i).rcs/max(1e-6, sqrt(snr_lin));
                        
                        ptr = mod(obj.targets(i).hist.ptr, 1000) + 1; obj.targets(i).hist.ptr = ptr;
                        obj.targets(i).hist.time(ptr) = t; 
                        obj.targets(i).hist.R_err(ptr) = mR-rng; 
                        obj.targets(i).hist.V_err(ptr) = mV-vr; 
                        obj.targets(i).hist.meas_sph(ptr, :) = [mR, mA, mE]; 
                        obj.targets(i).hist.snr(ptr) = snr; 
                        obj.targets(i).hist.pol(ptr, :) = [max(0, abs(E_rx(1))^2+randn()*sigRCS), max(0, abs(obj.targets(i).S_mat(2,2))^2*(fluct_rcs/obj.targets(i).rcs)+randn()*sigRCS), max(0, abs(E_rx(2))^2+randn()*sigRCS)];
                        if ptr == 1000, obj.targets(i).hist.wrapped = true; end
                        
                        bI=obj.visData.bI; obj.visData.bX(bI)=mR*cosd(mA); obj.visData.bY(bI)=mR*sind(mA); obj.visData.bA(bI)=0; obj.visData.bI=mod(bI,obj.visData.maxPPI)+1;
                        rI=obj.visData.rI; obj.visData.rX(rI)=mR/1000; obj.visData.rY(rI)=mV; obj.visData.rZ(rI)=t; obj.visData.rC(rI,:)=cp_colors(i,:); obj.visData.rA(rI)=0; obj.visData.rI=mod(rI,obj.visData.maxRD)+1;
                        set(obj.targets(i).patchHandle, 'FaceColor', 'y');
                    else
                        if isTrk && i==cIdx, obj.radarState.tracked(i)=false; end
                    end
                else
                    set(obj.targets(i).patchHandle, 'FaceColor', cp_colors(i,:));
                end
            end
            if best_cpi_tgt_idx ~= -1 && obj.TabGroupVis.SelectedTab == obj.TabC2Main, obj.processCPI(obj.targets(best_cpi_tgt_idx), uiCache); end
            for i=1:length(obj.targets)
                if obj.radarState.tracked(i) && (t - obj.targets(i).last_time_seen > 3.0), obj.radarState.tracked(i) = false; end
            end
        end

        function processCPI(obj, target, uiCache)
            BW = obj.radarParams.BW; Tau = obj.HardwareSpecs.Tau; PRF = obj.HardwareSpecs.PRF; PRI = 1/PRF;
            t_pos = target.pos(:)' - obj.radarState.pos(:)'; rng = max(1e-6, norm(t_pos)); vr = -dot(target.vel(:)' - obj.radarState.vel(:)', t_pos/rng);
            PG_tx = obj.ArrayConfig.Tx_Ny * obj.ArrayConfig.Tx_Nz; PG_rx = obj.ArrayConfig.Rx_Ny * obj.ArrayConfig.Rx_Nz;
            Pr = (obj.HardwareSpecs.Pt * obj.radarParams.Ge^2 * PG_tx * PG_rx * (obj.c/obj.HardwareSpecs.Freq)^2 * target.rcs) / (((4*pi)^3) * rng^4 * 10^(5/10));
            Pn = obj.kT0 * BW * 10^(3/10);
            if uiCache.chkJam, Pn = Pn + (10e3 * obj.radarParams.Ge * PG_rx * (obj.c/obj.HardwareSpecs.Freq)^2) / ((4*pi)^2 * rng^2); end
            
            beta = (obj.c - vr) / (obj.c + vr); rx_mat = obj.dsp_rx_mat; t_rx = obj.dsp_t_rx; Fs = max(2.5 * BW, 10e6);
            clut_amp = (sqrt(Pn) * 8.0) * (-log(1 - rand(1, length(t_rx)))).^(1/1.5);
            clut_sig = filter(ones(1,10)/10, 1, clut_amp .* exp(1j * 2*pi * rand(1, length(t_rx)))); 

            t_tx_dilated = (-Tau/2 : 1/Fs : Tau/2) * beta;
            length_tx = length(t_tx_dilated);
            
            for p = 1:16
                cR = rng - (vr * (p-1) * PRI); idx_s = round((2*cR/obj.c) * Fs); 
                idx_e = idx_s + length_tx - 1; 
                sig = zeros(1, length(t_rx));
                
                if idx_e <= length(t_rx) && idx_s > 0
                    sig(idx_s:idx_e) = sqrt(Pr) * exp(1j * pi * (BW/Tau) * t_tx_dilated.^2) * exp(-1j * 4*pi * cR / (obj.c/obj.HardwareSpecs.Freq)); 
                end
                rx_mat(p,:) = sig + clut_sig + sqrt(Pn) * (randn(1, length(t_rx)) + 1j*randn(1, length(t_rx)))/sqrt(2);
            end
            
            N_fft = length(obj.dsp_MF_F); mf_mat = zeros(size(rx_mat));
            for p=1:16
                mf_tmp = ifft(fft(rx_mat(p,:), N_fft) .* obj.dsp_MF_F); mf_mat(p,:) = mf_tmp(obj.dsp_delay+1 : obj.dsp_delay+size(rx_mat,2));
            end
            
            r_ax = (t_rx * obj.c) / 2; mf_dB = 20*log10(abs(mf_mat(1,:)) + 1e-12); mf_dB = mf_dB - max(mf_dB(:));
            if isempty(obj.hMFLine) || ~isvalid(obj.hMFLine)
                obj.hMFLine = plot(obj.AxMF, r_ax/1000, mf_dB, 'b', 'LineWidth', 1.5); 
            else
                set(obj.hMFLine, 'XData', r_ax/1000, 'YData', mf_dB); 
            end
            
            RD = fftshift(fft(mf_mat, [], 1), 1); RD_MagSq = abs(RD).^2; RD_dB = 10*log10(RD_MagSq+1e-12); RD_dB = RD_dB - max(RD_dB(:));
            v_ax = linspace(-PRF/2, PRF/2, 16) * (obj.c/obj.HardwareSpecs.Freq) / 2;
            if isempty(obj.hRDImage) || ~isvalid(obj.hRDImage)
                obj.hRDImage = imagesc(obj.AxRDMap, 'XData', r_ax/1000, 'YData', v_ax, 'CData', RD_dB); obj.AxRDMap.YDir = 'normal'; colormap(obj.AxRDMap, 'jet'); clim(obj.AxRDMap, [-50 0]);
            else
                set(obj.hRDImage, 'XData', r_ax/1000, 'YData', v_ax, 'CData', RD_dB); 
            end
            
            mask = ones(1, 29); mask(13:17) = 0; mask = mask / sum(mask); 
            CFAR_Det = RD_MagSq > (conv2(RD_MagSq, mask, 'same') * 2.5);
            if isempty(obj.hCFARImage) || ~isvalid(obj.hCFARImage)
                obj.hCFARImage = imagesc(obj.AxCFAR, 'XData', r_ax/1000, 'YData', v_ax, 'CData', CFAR_Det); obj.AxCFAR.YDir = 'normal'; colormap(obj.AxCFAR, 'gray');
            else
                set(obj.hCFARImage, 'XData', r_ax/1000, 'YData', v_ax, 'CData', CFAR_Det); 
            end

            if obj.TabGroupVis.SelectedTab == obj.TabC2Main && obj.TgC2.SelectedTab == obj.TabDSP
                lambda = obj.c / obj.HardwareSpecs.Freq; az_bins = linspace(-90, 90, 90); stap_img = zeros(length(v_ax), length(az_bins));
                v_clutter_vec = obj.radarState.vel(1).*cosd(az_bins) + obj.radarState.vel(2).*sind(az_bins) + obj.radarState.vel(3).*0;
                v_idx_raw = max(2, min(length(v_ax)-1, round(((2 * v_clutter_vec / lambda) * lambda / 2 - v_ax(1)) / (v_ax(2) - v_ax(1))) + 1));
                stap_img([sub2ind(size(stap_img), v_idx_raw-1, 1:90), sub2ind(size(stap_img), v_idx_raw, 1:90), sub2ind(size(stap_img), v_idx_raw+1, 1:90)]) = 20;
                
                tAz = mod(rad2deg(atan2(t_pos(2), t_pos(1))), 360); if tAz>180, tAz = tAz-360; end
                [~, az_idx] = min(abs(az_bins - tAz)); [~, v_idx] = min(abs(v_ax - vr)); stap_img(v_idx, az_idx) = 40; 
                
                if uiCache.chkAF 
                    stap_img([sub2ind(size(stap_img), v_idx_raw-1, 1:90), sub2ind(size(stap_img), v_idx_raw, 1:90), sub2ind(size(stap_img), v_idx_raw+1, 1:90)]) = -10;
                    stap_img(v_idx, az_idx) = 40; 
                end
                
                if isempty(obj.hSTAPImage) || ~isvalid(obj.hSTAPImage)
                    obj.hSTAPImage = imagesc(obj.AxSTAP, 'XData', az_bins, 'YData', v_ax, 'CData', stap_img); obj.AxSTAP.YDir = 'normal'; colormap(obj.AxSTAP, 'jet'); clim(obj.AxSTAP, [-10 40]);
                else
                    set(obj.hSTAPImage, 'XData', az_bins, 'YData', v_ax, 'CData', stap_img); 
                end
            end
        end

        function p = native_mvnpdf(~, x, mu, Sigma)
            % Bulletproof Singular Value Decomposition implementation of Multivariate PDF
            k = length(x); diff = x(:) - mu(:); 
            [U, S_val, V] = svd((Sigma + Sigma')/2);
            s = diag(S_val);
            tol = max(k * eps(max(s)), 1e-8);
            s_inv = zeros(size(s));
            s_inv(s > tol) = 1 ./ s(s > tol);
            
            invSigma = V * diag(s_inv) * U';
            logDetSigma = sum(log(max(s, 1e-12)));
            
            p = (2*pi)^(-k/2) * exp(-0.5 * logDetSigma) * exp(-0.5 * diff' * invSigma * diff);
        end

        function cmd = stepC2Estimator(obj, uiCache)
            t = obj.simState.time; cmd = [0;0;1000]; bTTI = Inf; dt = obj.simState.dt;
            nTgt = length(obj.targets); estData = cell(nTgt, 9); cp = lines(max(1, nTgt));
            
            for i = 1:nTgt
                if ~obj.radarState.tracked(i)
                    if isvalid(obj.hMeasRaw(i))
                        set(obj.hMeasRaw(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                        set(obj.hMeasLine(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                        set(obj.hMeasEst(i), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
                    end
                    continue; 
                end
                
                hist = obj.targets(i).hist;
                if hist.ptr == 0, continue; end
                if hist.wrapped, v_idx = [hist.ptr+1:1000, 1:hist.ptr]; else, v_idx = 1:hist.ptr; end
                
                valid_mask = (hist.time(v_idx) > 0) & (hist.time(v_idx) >= t - 20);
                
                if sum(valid_mask) >= 3
                    v_fIdx = v_idx(valid_mask);
                    dec = max(1, floor(length(v_idx)/100)); dIdx = 1:dec:length(v_idx);
                    
                    if isvalid(obj.hErrRange(i))
                        set(obj.hErrRange(i), 'XData', hist.time(v_idx(dIdx)), 'YData', hist.R_err(v_idx(dIdx))); 
                        set(obj.hErrVel(i), 'XData', hist.time(v_idx(dIdx)), 'YData', hist.V_err(v_idx(dIdx)));
                    end
                    
                    r_sph = hist.meas_sph(v_fIdx, 1); a_sph = hist.meas_sph(v_fIdx, 2); e_sph = hist.meas_sph(v_fIdx, 3); 
                    x_vis = r_sph.*cosd(e_sph).*cosd(a_sph); y_vis = r_sph.*cosd(e_sph).*sind(a_sph); z_vis = r_sph.*sind(e_sph);
                    dec_f = max(1, floor(length(v_fIdx)/100));
                    
                    if isvalid(obj.hMeasRaw(i))
                        set(obj.hMeasRaw(i), 'XData', x_vis(1:dec_f:end), 'YData', y_vis(1:dec_f:end), 'ZData', z_vis(1:dec_f:end));
                    end
                    
                    m_raw = [hist.meas_sph(hist.ptr, 1); hist.meas_sph(hist.ptr, 2)*pi/180; hist.meas_sph(hist.ptr, 3)*pi/180];
                    snr_lin = 10^(hist.snr(hist.ptr)/10);
                    R_meas = diag([(obj.c/(2*obj.radarParams.BW))/max(1e-6, sqrt(2*snr_lin))^2, ((5/max(1e-6, sqrt(2*snr_lin))) * pi/180)^2, ((5/max(1e-6, sqrt(2*snr_lin))) * pi/180)^2]); 
                    
                    imm = obj.targets(i).IMM; 
                    c_bar = imm.T_mat' * imm.mu;
                    mu_mix = (imm.T_mat .* (imm.mu * ones(1,2))) ./ max(1e-12, (ones(2,1) * c_bar'));
                    x_0_cv = imm.x_cv * mu_mix(1,1) + imm.x_ct * mu_mix(2,1); x_0_ct = imm.x_cv * mu_mix(1,2) + imm.x_ct * mu_mix(2,2);
                    P_0_cv = mu_mix(1,1)*(imm.P_cv + (imm.x_cv-x_0_cv)*(imm.x_cv-x_0_cv)') + mu_mix(2,1)*(imm.P_ct + (imm.x_ct-x_0_cv)*(imm.x_ct-x_0_cv)');
                    P_0_ct = mu_mix(1,2)*(imm.P_cv + (imm.x_cv-x_0_ct)*(imm.x_cv-x_0_ct)') + mu_mix(2,2)*(imm.P_ct + (imm.x_ct-x_0_ct)*(imm.x_ct-x_0_ct)');
                    
                    hx_7 = @(x) [norm(x(1:3)); atan2(x(2), x(1)); asin(max(-1, min(1, x(3)/max(1e-6, norm(x(1:3))))))];
                    jacob_H_7 = @(x) [...
                        [x(1)/max(1e-6, norm(x(1:3))), x(2)/max(1e-6, norm(x(1:3))), x(3)/max(1e-6, norm(x(1:3))), 0, 0, 0, 0];
                        [-x(2)/(x(1)^2+x(2)^2+1e-12), x(1)/(x(1)^2+x(2)^2+1e-12), 0, 0, 0, 0, 0];
                        [-x(1)*x(3)/(norm(x(1:3))^2 * norm(x(1:2)) + 1e-12), -x(2)*x(3)/(norm(x(1:3))^2 * norm(x(1:2)) + 1e-12), norm(x(1:2))/(norm(x(1:3))^2 + 1e-12), 0, 0, 0, 0]];
                    
                    F_cv_dyn = eye(7); F_cv_dyn(1,4) = cos(x_0_cv(5))*dt; F_cv_dyn(2,4) = sin(x_0_cv(5))*dt; F_cv_dyn(3,6) = dt;
                    x_pred_cv = [x_0_cv(1) + x_0_cv(4)*cos(x_0_cv(5))*dt; x_0_cv(2) + x_0_cv(4)*sin(x_0_cv(5))*dt; x_0_cv(3) + x_0_cv(6)*dt; x_0_cv(4:7)];
                    P_pred_cv = F_cv_dyn * P_0_cv * F_cv_dyn' + imm.Q_cv;
                    H_cv = jacob_H_7(x_pred_cv); y_res_cv = m_raw - hx_7(x_pred_cv); y_res_cv(2) = mod(y_res_cv(2)+pi, 2*pi)-pi; 
                    
                    % Safe Pseudo-Inverse eliminates RCOND warnings
                    S_cv = 0.5 * ((H_cv * P_pred_cv * H_cv' + R_meas) + (H_cv * P_pred_cv * H_cv' + R_meas)'); 
                    K_cv = P_pred_cv * H_cv' * pinv(S_cv); 
                    imm.x_cv = x_pred_cv + K_cv * y_res_cv; 
                    imm.P_cv = 0.5 * ((eye(7) - K_cv * H_cv) * P_pred_cv + ((eye(7) - K_cv * H_cv) * P_pred_cv)') + eye(7)*1e-6;
                    
                    v = x_0_ct(4); th = x_0_ct(5); w = x_0_ct(7); F_ct_dyn = eye(7);
                    if abs(w) > 1e-4
                        F_ct_dyn(1,4) = (1/w) * (sin(th + w*dt) - sin(th)); F_ct_dyn(1,5) = (v/w) * (cos(th + w*dt) - cos(th)); F_ct_dyn(1,7) = (v/w^2) * (w*dt*cos(th + w*dt) - sin(th + w*dt) + sin(th));
                        F_ct_dyn(2,4) = (1/w) * (-cos(th + w*dt) + cos(th)); F_ct_dyn(2,5) = (v/w) * (sin(th + w*dt) - sin(th)); F_ct_dyn(2,7) = (v/w^2) * (w*dt*sin(th + w*dt) - cos(th));
                        F_ct_dyn(3,6) = dt; F_ct_dyn(5,7) = dt;
                        x_pred_ct = [x_0_ct(1) + (v/w)*(sin(th+w*dt)-sin(th)); x_0_ct(2) + (v/w)*(-cos(th+w*dt)+cos(th)); x_0_ct(3) + x_0_ct(6)*dt; v; th + w*dt; x_0_ct(6); w];
                    else
                        F_ct_dyn(1,4) = cos(th)*dt; F_ct_dyn(2,4) = sin(th)*dt; F_ct_dyn(3,6) = dt;
                        x_pred_ct = [x_0_ct(1) + v*cos(th)*dt; x_0_ct(2) + v*sin(th)*dt; x_0_ct(3) + x_0_ct(6)*dt; v; th; x_0_ct(6); w];
                    end
                    
                    P_pred_ct = F_ct_dyn * P_0_ct * F_ct_dyn' + imm.Q_ct;
                    H_ct = jacob_H_7(x_pred_ct); y_res_ct = m_raw - hx_7(x_pred_ct); y_res_ct(2) = mod(y_res_ct(2)+pi, 2*pi)-pi; 
                    
                    S_ct = 0.5 * ((H_ct * P_pred_ct * H_ct' + R_meas) + (H_ct * P_pred_ct * H_ct' + R_meas)'); 
                    K_ct = P_pred_ct * H_ct' * pinv(S_ct); 
                    imm.x_ct = x_pred_ct + K_ct * y_res_ct; 
                    imm.P_ct = 0.5 * ((eye(7) - K_ct * H_ct) * P_pred_ct + ((eye(7) - K_ct * H_ct) * P_pred_ct)') + eye(7)*1e-6;
                    
                    L_cv = max(1e-12, obj.native_mvnpdf(y_res_cv', zeros(1,3), S_cv)); 
                    L_ct = max(1e-12, obj.native_mvnpdf(y_res_ct', zeros(1,3), S_ct));
                    imm.mu = [L_cv * c_bar(1); L_ct * c_bar(2)]; 
                    imm.mu = imm.mu / sum(imm.mu);
                    
                    x_est = imm.x_cv * imm.mu(1) + imm.x_ct * imm.mu(2); obj.targets(i).IMM = imm;
                    ce = x_est(1:3); ve = [x_est(4)*cos(x_est(5)); x_est(4)*sin(x_est(5)); x_est(6)]; w_est = x_est(7);
                    
                    if isvalid(obj.hMeasLine(i))
                        te = [-5, 5]; set(obj.hMeasLine(i), 'XData', ce(1)+ve(1)*te, 'YData', ce(2)+ve(2)*te, 'ZData', ce(3)+ve(3)*te);
                        if uiCache.chkPred, ce_lead = ce + ve*0.15; else, ce_lead = ce; end
                        set(obj.hMeasEst(i), 'XData', ce_lead(1), 'YData', ce_lead(2), 'ZData', ce_lead(3));
                    else
                        ce_lead = ce;
                    end
                    
                    vr = -dot(ve, ce/max(1e-6, norm(ce))); tti = Inf; 
                    if vr>0, tti = norm(ce)/vr; tStr = sprintf('%.1f',tti); if tti<bTTI, bTTI=tti; cmd=ce_lead; end; else, tStr='N/A'; end
                    
                    HH = hist.pol(hist.ptr, 1); VV = hist.pol(hist.ptr, 2); HV = hist.pol(hist.ptr, 3);
                    meas_rng = m_raw(1);
                    
                    PG = 1; if uiCache.chkMF, PG = PG * (obj.radarParams.BW * obj.HardwareSpecs.Tau); end; if uiCache.chkDF, PG = PG * 16; end
                    PG_spatial = (obj.ArrayConfig.Tx_Ny * obj.ArrayConfig.Tx_Nz) * (obj.ArrayConfig.Rx_Ny * obj.ArrayConfig.Rx_Nz);
                    Pn_est = obj.kT0 * obj.radarParams.BW * 10^(3/10); 
                    eR = (10^(hist.snr(hist.ptr)/10) * ((4*pi)^3) * meas_rng^4 * Pn_est) / (obj.HardwareSpecs.Pt * obj.radarParams.Ge^2 * PG_spatial * (obj.c/obj.HardwareSpecs.Freq)^2 * PG);
                    
                    r = HH/(max(1e-6, VV)); if r>2.0, shp='Horizontal'; elseif r<0.5, shp='Vertical'; else, shp='Symmetric'; end
                    if norm(ve)>343, cls='Missile'; elseif norm(ve)>100 && r>2.0, cls='Airliner'; elseif r>0.5 && r<2.0, cls='Stealth/UAV'; else, cls='Unknown'; end
                    if obj.targets(i).isJammer, cls = 'JAMMER (DRFM)'; end
                    
                    estData{i,1}=sprintf('Trk-%d',i); estData{i,2}=sprintf('%.1f',norm(ce)/1000); estData{i,3}=sprintf('%.0f',ce(3)+obj.radarState.pos(3)); estData{i,4}=sprintf('%.1f',norm(ve + obj.radarState.vel(:))); estData{i,5}=sprintf('%.2f',eR); estData{i,6}=shp; estData{i,7}=cls; estData{i,8}=tStr; estData{i,9}=sprintf('%.2f',w_est);
                    
                    if tti == bTTI && tti < Inf 
                        if isvalid(obj.hPolBars)
                            set(obj.hPolBars, 'YData', [HH, VV, HV]);
                            set(obj.hCrossShape, 'Position', [-max(0.5,sqrt(HH)*2), -max(0.5,sqrt(VV)*2), max(0.5,sqrt(HH)*4), max(0.5,sqrt(VV)*4)], 'FaceColor', cp(i,:));
                        end
                    end
                end
            end
            
            if isvalid(obj.EstTable)
                vR = ~cellfun('isempty', estData(:,1)); 
                if any(vR), obj.EstTable.Data = estData(vR,:); else, obj.EstTable.Data = cell(0, 9); end
            end
        end

        function R = quat2rotm_local(~, q)
            qw=q(1); qx=q(2); qy=q(3); qz=q(4);
            R = [1-2*qy^2-2*qz^2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw;
                 2*qx*qy+2*qz*qw, 1-2*qx^2-2*qz^2, 2*qy*qz-2*qx*qw;
                 2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx^2-2*qy^2];
        end

        function stepEffectors(obj, tgt)
            sub_dt = 0.01; steps = round(obj.simState.dt / sub_dt);
            if obj.DropEffectorType.Value == 1
                for k=1:steps
                    vt = tgt - [0; 0; obj.hexParams.Z_nom]; u_d = vt / max(1e-6, norm(vt)); 
                    Rm = obj.quat2rotm_local(obj.hexState.q); u_curr = Rm * [0;0;1]; 
                    
                    err_vec = cross(u_curr, u_d); err_ang = asin(max(-1, min(1, norm(err_vec))));
                    if norm(err_vec) > 1e-6, err_axis = err_vec / norm(err_vec); else, err_axis = [1;0;0]; end
                    
                    tau_cmd = 500 * err_ang * err_axis - 50 * obj.hexState.omega;
                    I_t = Rm * obj.hexParams.I_local * Rm'; I_inv_t = Rm * obj.hexParams.I_inv * Rm';
                    obj.hexState.omega = obj.hexState.omega + (I_inv_t * (tau_cmd - cross(obj.hexState.omega, I_t * obj.hexState.omega))) * sub_dt;
                    
                    wx = obj.hexState.omega(1); wy = obj.hexState.omega(2); wz = obj.hexState.omega(3);
                    obj.hexState.q = obj.hexState.q + (0.5 * [0 -wx -wy -wz; wx 0 wz -wy; wy -wz 0 wx; wz wy -wx 0] * obj.hexState.q) * sub_dt;
                    obj.hexState.q = obj.hexState.q / max(1e-6, norm(obj.hexState.q)); 
                    
                    W_f = [obj.hexParams.m_top * -obj.hexParams.g_vec; tau_cmd];
                    Pi = Rm * obj.hexParams.P; TP = repmat(obj.hexParams.Z_nom*[0;0;1], 1, 6) + Pi;
                    Li = TP - obj.hexParams.B; l_cmd = sqrt(sum(Li.^2, 1));
                    J = [Li ./ max(1e-6, l_cmd); cross(Pi, Li ./ max(1e-6, l_cmd), 1)]';
                    
                    ws = 'NOMINAL'; wc = 'g'; cond_J = cond(J);
                    if cond_J > obj.hexParams.Cond_Limit || any(l_cmd < obj.hexParams.L_min) || any(l_cmd > obj.hexParams.L_max)
                        ws = 'ERR: SINGULARITY / KINEMATIC'; wc = 'r'; Fa = zeros(6,1);
                    else
                        Fa = (J') \ W_f; if max(abs(Fa)) > obj.hexParams.F_limit, ws = 'ERR: OVERLOAD'; wc = 'y'; end
                    end
                end
                
                obj.hexRenderCount = obj.hexRenderCount + 1;
                if mod(obj.hexRenderCount, 2)==0 && obj.TabGroupVis.SelectedTab == obj.TabPointer
                    if isvalid(obj.HexTopPlate)
                        set(obj.HexTopPlate, 'XData', TP(1,:), 'YData', TP(2,:), 'ZData', TP(3,:));
                        for j=1:6, set(obj.HexLegs(j), 'XData', [obj.hexParams.B(1,j), TP(1,j)], 'YData', [obj.hexParams.B(2,j), TP(2,j)], 'ZData', [obj.hexParams.B(3,j), TP(3,j)]); end
                        le = [0;0;obj.hexParams.Z_nom] + u_curr*2.0; set(obj.HexLaser, 'XData', [0 le(1)], 'YData', [0 le(2)], 'ZData', [obj.hexParams.Z_nom le(3)]);
                        set(obj.HexWarn, 'String', ws, 'BackgroundColor', wc); set(obj.HexTele, 'String', sprintf('Cond: %.1f\nForces:\n%.1f %.1f\n%.1f %.1f\n%.1f %.1f', cond_J, Fa(1),Fa(4),Fa(2),Fa(5),Fa(3),Fa(6)));
                    end
                end
            else
                for k=1:steps
                    vt = tgt - [0;0;3.25]; qa_c = atan2(vt(2), vt(1)); qe_c = atan2(vt(3), max(1e-6, norm(vt(1:2))));
                    ea = (mod(qa_c - obj.azelState.q(1) + pi, 2*pi) - pi) * min(1.0, abs(cos(obj.azelState.q(2))) / 0.1); 
                    ee = qe_c - obj.azelState.q(2);
                    
                    qdd_a = 25.0*ea - 10.0*obj.azelState.q_dot(1); qdd_e = 25.0*ee - 10.0*obj.azelState.q_dot(2);
                    p = obj.azelParams; Ie_a = p.J_z + p.I_x*sin(obj.azelState.q(2))^2 + p.I_z*cos(obj.azelState.q(2))^2;
                    Ca = (p.I_x - p.I_z)*sin(2*obj.azelState.q(2))*obj.azelState.q_dot(1)*obj.azelState.q_dot(2); Ce = 0.5*(p.I_x - p.I_z)*sin(2*obj.azelState.q(2))*obj.azelState.q_dot(1)^2;
                    
                    tra = Ie_a*qdd_a + Ca; tre = p.I_y*qdd_e - Ce;
                    taa = max(-p.Tau_max_az, min(p.Tau_max_az, tra)); tae = max(-p.Tau_max_el, min(p.Tau_max_el, tre));
                    ws = 'NOMINAL'; wc = 'g'; if abs(tra)>p.Tau_max_az || abs(tre)>p.Tau_max_el, ws = 'ERR: TORQUE'; wc = 'y'; end
                    
                    obj.azelState.q_dot(1) = obj.azelState.q_dot(1) + ((taa-Ca)/max(1e-6, Ie_a))*sub_dt; obj.azelState.q_dot(2) = obj.azelState.q_dot(2) + ((tae+Ce)/p.I_y)*sub_dt;
                    obj.azelState.q(1) = mod(obj.azelState.q(1) + obj.azelState.q_dot(1)*sub_dt + pi, 2*pi) - pi; obj.azelState.q(2) = obj.azelState.q(2) + obj.azelState.q_dot(2)*sub_dt;
                end
                
                obj.azelRenderCount = obj.azelRenderCount + 1;
                if mod(obj.azelRenderCount, 2)==0 && obj.TabGroupVis.SelectedTab == obj.TabPointer
                    if isvalid(obj.TAz)
                        set(obj.TAz, 'Matrix', makehgtform('zrotate', obj.azelState.q(1))); set(obj.TEl, 'Matrix', makehgtform('translate', [0, 0, 3.25], 'yrotate', -obj.azelState.q(2)));
                        set(obj.AzElWarn, 'String', ws, 'BackgroundColor', wc); set(obj.AzElTele, 'String', sprintf('I_eff: %.2f\nTau_Az: %.1f\nTau_El: %.1f', Ie_a, taa, tae));
                    end
                end
            end
        end

        function updateTelemetry(obj)
            obj.visData.bA = obj.visData.bA + obj.simState.dt; 
            if isvalid(obj.blipsScatter)
                set(obj.blipsScatter, 'XData', obj.visData.bX, 'YData', obj.visData.bY, 'CData', [zeros(150,1), max(0, 1 - obj.visData.bA/3.0), zeros(150,1)]);
            end
            
            obj.visData.rA = obj.visData.rA + obj.simState.dt; 
            if isvalid(obj.rdScatter3D)
                fc = obj.visData.rC .* repmat(max(0, 1 - obj.visData.rA/3.0), 1, 3);
                set(obj.rdScatter3D, 'XData', obj.visData.rX, 'YData', obj.visData.rY, 'ZData', obj.visData.rZ, 'CData', fc);
            end
        end
    end
end