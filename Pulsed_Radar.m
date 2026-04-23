classdef Pulsed_Radar < handle
    % PULSED_RADAR - An advanced interactive app for learning Pulsed Radar
    % design, Phased Arrays, Beamforming, Signal Processing, and Optimization.
    %
    % Save this file as 'Pulsed_Radar.m' and run it from the Command Window.
    
    properties (Access = private)
        % Main UI Figure & Tabs
        UIFigure
        Fig3DWorld
        Ax3DWorld
        
        TabGroup
        TabFundamentals
        TabPhasedArray
        TabSignalProc
        TabOptimizer
        TabArrayDesign
        
        % --- TAB 1: Fundamentals Inputs ---
        EditTargetRCS, EditTargetRange
        EditPeakPower, EditFrequency, EditAntennaGain
        EditPulseWidth, EditPRF, EditSystemLosses, EditNoiseFigure
        LblCurrentSNROut, LblRangeResOut, LblMaxRangeOut
        LblMinRangeOut, LblAvgPowerOut, LblDutyCycleOut
        AxSNR, AxPulseTrain
        
        % --- TAB 2: Phased Array & Kinematics Inputs ---
        EditTargetAz, EditTargetEl, EditTargetVel, EditTargetHeading
        EditSimTime
        EditJammerAz, EditJammerEl, EditJammerJSR
        EditScanAz, EditScanEl
        CheckMVDR
        AxBeamPattern
        
        % --- TAB 3: Signal Processing Inputs ---
        EditChirpBW
        AxTimeDomain, AxMatchedFilter, AxRangeDoppler
        
        % --- TAB 4: Optimizer Inputs ---
        EditOptReqSNR, EditOptReqRes, EditOptMaxDC
        TxtOptLog
        
        % --- TAB 5: Array Design Inputs ---
        EditTxNy, EditTxNz
        EditRxNy, EditRxNz
        AxArrayHeatmap, AxArrayCuts
    end
    
    % Physics Constants
    properties (Constant)
        c = 299792458; % Speed of light (m/s)
        k = 1.380649e-23; % Boltzmann constant (J/K)
        T0 = 290; % Standard noise temperature (K)
    end
    
    methods
        function obj = Pulsed_Radar()
            % Constructor: Create UI and initialize calculations
            obj.buildUI();
            obj.updateCalculations();
        end
    end
    
    methods (Access = private)
        function buildUI(obj)
            % Create main standard figure window
            obj.UIFigure = figure('Name', 'Advanced Radar Systems Laboratory', ...
                                  'Position', [50, 50, 1050, 750], ...
                                  'MenuBar', 'none', ...
                                  'NumberTitle', 'off');
                              
            obj.TabGroup = uitabgroup(obj.UIFigure, 'Position', [0 0 1 1]);
            
            % Build Individual Tabs
            obj.buildTabFundamentals();
            obj.buildTabArrayDesign();       % NEW: Array Design Tab
            obj.buildTabPhasedArray();       % Updated: Kinematics
            obj.buildTabSignalProcessing();
            obj.buildTabOptimizer();
            
            % Initialize 3D World pop-out window
            obj.init3DFigure();
        end
        
        function init3DFigure(obj)
            if isempty(obj.Fig3DWorld) || ~isvalid(obj.Fig3DWorld)
                obj.Fig3DWorld = figure('Name', 'Live 3D Radar Environment', ...
                                        'Position', [1120, 50, 750, 750], ...
                                        'NumberTitle', 'off');
                obj.Ax3DWorld = axes('Parent', obj.Fig3DWorld, 'Position', [0.1 0.1 0.8 0.8]);
                grid(obj.Ax3DWorld, 'on'); view(obj.Ax3DWorld, 3);
            end
        end
        
        function buildTabFundamentals(obj)
            obj.TabFundamentals = uitab(obj.TabGroup, 'Title', '1. Fundamentals');
            
            pnlInputs = uipanel(obj.TabFundamentals, 'Title', 'Hardware & Target Parameters', ...
                                'Position', [0.01 0.02 0.32 0.96], 'FontWeight', 'bold');
            
            dy = 1/14;
            function fld = createInput(parent, row, labelText, defaultVal)
                yPos = 1 - row*dy;
                uicontrol('Parent', parent, 'Style', 'text', 'String', labelText, ...
                          'Units', 'normalized', 'Position', [0.01, yPos-0.02, 0.65, dy*0.8], 'HorizontalAlignment', 'right', 'FontSize', 9);
                fld = uicontrol('Parent', parent, 'Style', 'edit', 'String', num2str(defaultVal), ...
                                'Units', 'normalized', 'Position', [0.68, yPos, 0.28, dy*0.7], 'Callback', @(src,event) obj.updateCalculations(), 'FontSize', 9);
            end
            
            uicontrol('Parent', pnlInputs, 'Style', 'text', 'String', '--- Target (Tab 1) ---', 'Units', 'normalized', 'Position', [0.05, 1-1*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditTargetRCS   = createInput(pnlInputs, 2, 'RCS (\sigma) (m^2)', 5);
            obj.EditTargetRange = createInput(pnlInputs, 3, 'Range (R) (km)', 50);
            
            uicontrol('Parent', pnlInputs, 'Style', 'text', 'String', '--- Hardware ---', 'Units', 'normalized', 'Position', [0.05, 1-5*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditPeakPower   = createInput(pnlInputs, 6, 'Peak Power (Pt) (kW)', 100);
            obj.EditFrequency   = createInput(pnlInputs, 7, 'Frequency (f) (GHz)', 3);
            obj.EditAntennaGain = createInput(pnlInputs, 8, 'Base Element Gain (dB)', 10);
            obj.EditPulseWidth  = createInput(pnlInputs, 9, 'Pulse Width (\tau) (\mus)', 10);
            obj.EditPRF         = createInput(pnlInputs, 10, 'PRF (kHz)', 2);
            obj.EditSystemLosses= createInput(pnlInputs, 11, 'System Losses (L) (dB)', 5);
            obj.EditNoiseFigure = createInput(pnlInputs, 12, 'Noise Figure (F) (dB)', 3);

            pnlOutputs = uipanel(obj.TabFundamentals, 'Title', 'System Performance Metrics', ...
                                 'Position', [0.34 0.52 0.30 0.46], 'FontWeight', 'bold');
            
            dyOut = 1/7;
            function lbl = createOutput(parent, row, labelText)
                yPos = 1 - row*dyOut;
                uicontrol('Parent', parent, 'Style', 'text', 'String', labelText, ...
                          'Units', 'normalized', 'Position', [0.02, yPos-0.02, 0.63, dyOut*0.8], 'HorizontalAlignment', 'right', 'FontWeight', 'bold', 'FontSize', 9);
                lbl = uicontrol('Parent', parent, 'Style', 'text', 'String', '-', ...
                                'Units', 'normalized', 'Position', [0.7, yPos-0.02, 0.28, dyOut*0.8], 'HorizontalAlignment', 'left', 'FontSize', 9);
            end
            
            obj.LblCurrentSNROut = createOutput(pnlOutputs, 1, 'Target SNR (Base):');
            obj.LblCurrentSNROut.ForegroundColor = [0 0.5 0]; obj.LblCurrentSNROut.FontSize = 12;
            obj.LblRangeResOut   = createOutput(pnlOutputs, 2, 'Raw Range Res (\DeltaR):');
            obj.LblMaxRangeOut   = createOutput(pnlOutputs, 3, 'Unambiguous Range:');
            obj.LblMinRangeOut   = createOutput(pnlOutputs, 4, 'Blind Zone:');
            obj.LblAvgPowerOut   = createOutput(pnlOutputs, 5, 'Average Power:');
            obj.LblDutyCycleOut  = createOutput(pnlOutputs, 6, 'Duty Cycle:');
            
            pnlPlots = uipanel(obj.TabFundamentals, 'Title', 'Radar Equation & Timing', ...
                               'Position', [0.65 0.02 0.34 0.96], 'FontWeight', 'bold');
            obj.AxSNR = axes('Parent', pnlPlots, 'Position', [0.15 0.58 0.75 0.35]);
            title(obj.AxSNR, 'SNR vs. Range'); xlabel(obj.AxSNR, 'Range (km)'); ylabel(obj.AxSNR, 'SNR (dB)'); grid(obj.AxSNR, 'on');
            
            obj.AxPulseTrain = axes('Parent', pnlPlots, 'Position', [0.15 0.1 0.75 0.35]);
            title(obj.AxPulseTrain, 'Pulse Train'); xlabel(obj.AxPulseTrain, 'Time (\mus)'); ylabel(obj.AxPulseTrain, 'Amplitude');
        end
        
        function buildTabArrayDesign(obj)
            obj.TabArrayDesign = uitab(obj.TabGroup, 'Title', '2. Array Design');
            
            pnlArray = uipanel(obj.TabArrayDesign, 'Title', 'Antenna Transceiver/Receiver Design', ...
                               'Position', [0.01 0.02 0.32 0.96], 'FontWeight', 'bold');
                           
            dy = 1/14;
            function fld = createArrayInput(row, labelText, defaultVal)
                yPos = 1 - row*dy;
                uicontrol('Parent', pnlArray, 'Style', 'text', 'String', labelText, ...
                          'Units', 'normalized', 'Position', [0.01, yPos-0.02, 0.65, dy*0.8], 'HorizontalAlignment', 'right', 'FontSize', 9);
                fld = uicontrol('Parent', pnlArray, 'Style', 'edit', 'String', num2str(defaultVal), ...
                                'Units', 'normalized', 'Position', [0.68, yPos, 0.28, dy*0.7], 'Callback', @(src,event) obj.updateCalculations(), 'FontSize', 9);
            end
            
            uicontrol('Parent', pnlArray, 'Style', 'text', 'String', '--- Transmitter Array (Tx) ---', 'Units', 'normalized', 'Position', [0.05, 1-1*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditTxNy = createArrayInput(2, 'Tx Elements Y (Azimuth)', 8);
            obj.EditTxNz = createArrayInput(3, 'Tx Elements Z (Elevation)', 8);
            
            uicontrol('Parent', pnlArray, 'Style', 'text', 'String', '--- Receiver Array (Rx) ---', 'Units', 'normalized', 'Position', [0.05, 1-5*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditRxNy = createArrayInput(6, 'Rx Elements Y (Azimuth)', 8);
            obj.EditRxNz = createArrayInput(7, 'Rx Elements Z (Elevation)', 8);
            
            uicontrol('Parent', pnlArray, 'Style', 'text', 'String', ...
                sprintf(['Designing separate Tx and Rx arrays allows you to optimize costs and beamwidths.\n\n',...
                         'The Two-Way Pattern is the product of the Tx and Rx patterns.']), ...
                'Units', 'normalized', 'Position', [0.05, 0.1, 0.9, 0.3], 'HorizontalAlignment', 'left', 'FontSize', 10);
            
            % Plots
            obj.AxArrayHeatmap = axes('Parent', obj.TabArrayDesign, 'Position', [0.4 0.55 0.55 0.4]);
            title(obj.AxArrayHeatmap, 'Two-Way Antenna Pattern (Azimuth vs. Elevation Heatmap)');
            xlabel(obj.AxArrayHeatmap, 'Azimuth (deg)'); ylabel(obj.AxArrayHeatmap, 'Elevation (deg)');
            
            obj.AxArrayCuts = axes('Parent', obj.TabArrayDesign, 'Position', [0.4 0.08 0.55 0.35]);
            title(obj.AxArrayCuts, 'Principal Cuts (Boresight)');
            xlabel(obj.AxArrayCuts, 'Angle (deg)'); ylabel(obj.AxArrayCuts, 'Two-Way Gain (dB)');
            grid(obj.AxArrayCuts, 'on');
        end
        
        function buildTabPhasedArray(obj)
            obj.TabPhasedArray = uitab(obj.TabGroup, 'Title', '3. Kinematics & Tracking');
            
            % Modified layout to prevent UI runoff completely
            pnlArray = uipanel(obj.TabPhasedArray, 'Title', 'Target Kinematics & Space Environment', ...
                               'Position', [0.01 0.02 0.32 0.96], 'FontWeight', 'bold');
            
            dy = 1/15; % Tighter spacing to fit everything perfectly
            function fld = createKinematicsInput(row, labelText, defaultVal)
                yPos = 1 - row*dy;
                uicontrol('Parent', pnlArray, 'Style', 'text', 'String', labelText, ...
                          'Units', 'normalized', 'Position', [0.01, yPos-0.01, 0.65, dy*0.8], 'HorizontalAlignment', 'right', 'FontSize', 9);
                fld = uicontrol('Parent', pnlArray, 'Style', 'edit', 'String', num2str(defaultVal), ...
                                'Units', 'normalized', 'Position', [0.68, yPos+0.01, 0.28, dy*0.7], 'Callback', @(src,event) obj.updateCalculations(), 'FontSize', 9);
            end
            
            uicontrol('Parent', pnlArray, 'Style', 'text', 'String', '--- Target Kinematics ---', 'Units', 'normalized', 'Position', [0.05, 1-1*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditTargetAz = createKinematicsInput(2, 'Target Azimuth (deg)', 20);
            obj.EditTargetEl = createKinematicsInput(3, 'Target Elevation (deg)', 10);
            obj.EditTargetVel = createKinematicsInput(4, 'Target Velocity (m/s)', -300);
            obj.EditTargetHeading = createKinematicsInput(5, 'Target Heading (deg)', 180);
            
            uicontrol('Parent', pnlArray, 'Style', 'text', 'String', '--- Simulation ---', 'Units', 'normalized', 'Position', [0.05, 1-6.5*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditSimTime = createKinematicsInput(7.5, 'Simulation Time (s)', 25);
            
            uicontrol('Parent', pnlArray, 'Style', 'text', 'String', '--- Interference (Jammer) ---', 'Units', 'normalized', 'Position', [0.05, 1-9*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditJammerAz  = createKinematicsInput(10, 'Jammer Azimuth (deg)', -30);
            obj.EditJammerEl  = createKinematicsInput(11, 'Jammer Elevation (deg)', 5);
            obj.EditJammerJSR = createKinematicsInput(12, 'Jammer-to-Signal Ratio (dB)', 30);
            
            uicontrol('Parent', pnlArray, 'Style', 'text', 'String', '--- Beam Steering ---', 'Units', 'normalized', 'Position', [0.05, 1-13.5*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditScanAz = createKinematicsInput(14.5, 'Scan Azimuth (deg)', 20);
            
            % Squeezed elements to prevent run-off
            obj.EditScanEl = uicontrol('Parent', pnlArray, 'Style', 'edit', 'String', '10', ...
                            'Units', 'normalized', 'Position', [0.68, 1-15.5*dy+0.01, 0.28, dy*0.7], 'Callback', @(src,event) obj.updateCalculations(), 'FontSize', 9);
            uicontrol('Parent', pnlArray, 'Style', 'text', 'String', 'Scan Elevation (deg)', ...
                      'Units', 'normalized', 'Position', [0.01, 1-15.5*dy-0.01, 0.65, dy*0.8], 'HorizontalAlignment', 'right', 'FontSize', 9);

            obj.CheckMVDR = uicontrol('Parent', pnlArray, 'Style', 'checkbox', 'String', 'Enable MVDR Null Steering', ...
                                      'Units', 'normalized', 'Position', [0.05, 0.01, 0.9, 0.05], 'FontWeight', 'bold', ...
                                      'Callback', @(src,event) obj.updateCalculations());
                                  
            obj.AxBeamPattern = axes('Parent', obj.TabPhasedArray, 'Position', [0.4 0.2 0.55 0.6]);
            title(obj.AxBeamPattern, 'Array Factor (Azimuth Cut at Scan Elevation)'); 
            xlabel(obj.AxBeamPattern, 'Azimuth (deg)'); 
            ylabel(obj.AxBeamPattern, 'Gain (dB)'); 
            grid(obj.AxBeamPattern, 'on');
            
            uicontrol('Parent', obj.TabPhasedArray, 'Style', 'text', ...
                'String', 'Note: The 3D view has been popped out to a separate dynamic window to track the target trajectory over Simulation Time.', ...
                'Units', 'normalized', 'Position', [0.4, 0.05, 0.55, 0.08], 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontAngle', 'italic');
        end
        
        function buildTabSignalProcessing(obj)
            obj.TabSignalProc = uitab(obj.TabGroup, 'Title', '4. Signal Proc (Pulse-by-Pulse)');
            
            pnlSig = uipanel(obj.TabSignalProc, 'Title', 'Waveform & Range-Doppler Parameters', ...
                             'Position', [0.01 0.02 0.32 0.96], 'FontWeight', 'bold');
            
            uicontrol('Parent', pnlSig, 'Style', 'text', 'String', 'LFM Chirp Bandwidth (MHz):', ...
                      'Units', 'normalized', 'Position', [0.05, 0.85, 0.65, 0.05], 'HorizontalAlignment', 'right', 'FontSize', 9);
            obj.EditChirpBW = uicontrol('Parent', pnlSig, 'Style', 'edit', 'String', '5', ...
                                        'Units', 'normalized', 'Position', [0.72, 0.85, 0.2, 0.06], 'Callback', @(src,event) obj.updateCalculations(), 'FontSize', 9);
                                    
            uicontrol('Parent', pnlSig, 'Style', 'text', 'String', ...
                sprintf(['This tab simulates 16 sequential radar pulses (A Coherent Processing Interval).\n\n', ...
                         '1. Pulse Compression compresses the chirp to achieve high Range Resolution.\n\n', ...
                         '2. Slow-Time FFT extracts the microscopic phase shifts caused by the target moving between pulses.\n\n', ...
                         '3. The Range-Doppler Map reveals exactly where the target is in space AND its velocity, completely isolated from stationary noise!']), ...
                'Units', 'normalized', 'Position', [0.05, 0.3, 0.9, 0.5], 'HorizontalAlignment', 'left', 'FontSize', 10);
            
            obj.AxTimeDomain = axes('Parent', obj.TabSignalProc, 'Position', [0.4 0.7 0.55 0.25]);
            title(obj.AxTimeDomain, 'Raw Received Signal (Pulse 1)'); xlabel(obj.AxTimeDomain, 'Time (\mus)'); ylabel(obj.AxTimeDomain, 'Amplitude (V)'); grid(obj.AxTimeDomain, 'on');
            
            obj.AxMatchedFilter = axes('Parent', obj.TabSignalProc, 'Position', [0.4 0.38 0.55 0.23]);
            title(obj.AxMatchedFilter, 'Matched Filter Output (Range Profile)'); xlabel(obj.AxMatchedFilter, 'Range (km)'); ylabel(obj.AxMatchedFilter, 'Magnitude (dB)'); grid(obj.AxMatchedFilter, 'on');
            
            obj.AxRangeDoppler = axes('Parent', obj.TabSignalProc, 'Position', [0.4 0.06 0.55 0.23]);
            title(obj.AxRangeDoppler, 'Range-Doppler Map (Velocity vs Range)'); xlabel(obj.AxRangeDoppler, 'Range (km)'); ylabel(obj.AxRangeDoppler, 'Radial Velocity (m/s)'); grid(obj.AxRangeDoppler, 'on');
        end
        
        function buildTabOptimizer(obj)
            obj.TabOptimizer = uitab(obj.TabGroup, 'Title', '5. Auto-Optimizer');
            
            pnlOpt = uipanel(obj.TabOptimizer, 'Title', 'System Design Optimizer', ...
                             'Position', [0.01 0.02 0.32 0.96], 'FontWeight', 'bold');
                         
            uicontrol('Parent', pnlOpt, 'Style', 'text', 'String', ...
                sprintf(['This tool analyzes your 3D spatial setup, array geometry, and current Jammer JSR, ', ...
                         'to mathematically reverse-engineer the required Radar parameters.']), ...
                'Units', 'normalized', 'Position', [0.05, 0.75, 0.9, 0.2], 'HorizontalAlignment', 'left', 'FontSize', 10);
            
            dy = 1/12;
            uicontrol('Parent', pnlOpt, 'Style', 'text', 'String', 'Req. Matched Filter SINR (dB):', 'Units', 'normalized', 'Position', [0.02, 1-4*dy, 0.65, dy*0.8], 'HorizontalAlignment', 'right', 'FontSize', 9);
            obj.EditOptReqSNR = uicontrol('Parent', pnlOpt, 'Style', 'edit', 'String', '15', 'Units', 'normalized', 'Position', [0.7, 1-4*dy, 0.25, dy*0.7], 'FontSize', 9);
            
            uicontrol('Parent', pnlOpt, 'Style', 'text', 'String', 'Req. Range Resolution (m):', 'Units', 'normalized', 'Position', [0.02, 1-5.5*dy, 0.65, dy*0.8], 'HorizontalAlignment', 'right', 'FontSize', 9);
            obj.EditOptReqRes = uicontrol('Parent', pnlOpt, 'Style', 'edit', 'String', '15', 'Units', 'normalized', 'Position', [0.7, 1-5.5*dy, 0.25, dy*0.7], 'FontSize', 9);
            
            uicontrol('Parent', pnlOpt, 'Style', 'text', 'String', 'Max Duty Cycle (%):', 'Units', 'normalized', 'Position', [0.02, 1-7*dy, 0.65, dy*0.8], 'HorizontalAlignment', 'right', 'FontSize', 9);
            obj.EditOptMaxDC = uicontrol('Parent', pnlOpt, 'Style', 'edit', 'String', '10', 'Units', 'normalized', 'Position', [0.7, 1-7*dy, 0.25, dy*0.7], 'FontSize', 9);
            
            uicontrol('Parent', pnlOpt, 'Style', 'pushbutton', 'String', 'Run 3D Optimization', ...
                      'Units', 'normalized', 'Position', [0.1, 0.1, 0.8, 0.1], ...
                      'BackgroundColor', [0.85 0.325 0.098], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
                      'Callback', @(src,event) obj.runOptimizer());
                  
            pnlLog = uipanel(obj.TabOptimizer, 'Title', 'Optimization Log', ...
                             'Position', [0.35 0.02 0.62 0.96], 'FontWeight', 'bold');
            obj.TxtOptLog = uicontrol('Parent', pnlLog, 'Style', 'text', 'String', 'Awaiting optimization command...', ...
                                      'Units', 'normalized', 'Position', [0.02 0.02 0.96 0.96], 'HorizontalAlignment', 'left', ...
                                      'FontSize', 10, 'FontName', 'Courier New');
        end
        
        function v = getNum(~, strVal, defaultVal)
            v = str2double(strVal);
            if isnan(v)
                v = defaultVal;
            end
        end

        function updateCalculations(obj)
            % Ensure 3D figure exists
            obj.init3DFigure();
            
            %% 1. FETCH ALL VARIABLES (With safe fallbacks)
            sigma = obj.getNum(obj.EditTargetRCS.String, 5);
            R_target = obj.getNum(obj.EditTargetRange.String, 50) * 1e3;
            Pt = obj.getNum(obj.EditPeakPower.String, 100) * 1e3;
            f = obj.getNum(obj.EditFrequency.String, 3) * 1e9;
            G_elem_dB = obj.getNum(obj.EditAntennaGain.String, 10);
            tau = obj.getNum(obj.EditPulseWidth.String, 10) * 1e-6;
            prf = obj.getNum(obj.EditPRF.String, 2) * 1e3;
            L_dB = obj.getNum(obj.EditSystemLosses.String, 5);
            F_dB = obj.getNum(obj.EditNoiseFigure.String, 3);
            
            % Array Size (Tab 5)
            Ny_tx = round(obj.getNum(obj.EditTxNy.String, 8)); Nz_tx = round(obj.getNum(obj.EditTxNz.String, 8));
            Ny_rx = round(obj.getNum(obj.EditRxNy.String, 8)); Nz_rx = round(obj.getNum(obj.EditRxNz.String, 8));
            
            % Kinematics (Tab 2)
            targAz = obj.getNum(obj.EditTargetAz.String, 20);
            targEl = obj.getNum(obj.EditTargetEl.String, 10);
            targVel = obj.getNum(obj.EditTargetVel.String, -300); % m/s
            targHeading = obj.getNum(obj.EditTargetHeading.String, 180); % deg
            simTime = obj.getNum(obj.EditSimTime.String, 25); % seconds
            
            jamAz = obj.getNum(obj.EditJammerAz.String, -30);
            jamEl = obj.getNum(obj.EditJammerEl.String, 5);
            jamJSR = obj.getNum(obj.EditJammerJSR.String, 30); 
            scanAz = obj.getNum(obj.EditScanAz.String, 20);
            scanEl = obj.getNum(obj.EditScanEl.String, 10);
            useMVDR = obj.CheckMVDR.Value;
            
            BW = obj.getNum(obj.EditChirpBW.String, 5) * 1e6;
            
            % Sanity checks to prevent plot crashes
            R_target = max(100, R_target);
            prf = max(10, prf);
            tau = max(1e-9, tau);
            BW = max(1e3, BW);
            
            %% 2. ARRAY MATHEMATICS (Separate Tx and Rx)
            lambda = obj.c / f;
            d = lambda / 2;
            
            % Element Gain
            G_elem = 10^(G_elem_dB / 10);
            
            % Tx Array Gain
            NumElements_Tx = Ny_tx * Nz_tx;
            G_tx = G_elem * NumElements_Tx;
            
            % Rx Array Gain
            NumElements_Rx = Ny_rx * Nz_rx;
            G_rx = G_elem * NumElements_Rx;
            
            % Total 2-Way Array Gain for Radar Equation
            G_total_2way_dB = 10*log10(G_tx * G_rx);
            
            %% 3. RADAR EQUATION & HARDWARE METRICS
            L = 10^(L_dB / 10);
            Fn = 10^(F_dB / 10);
            
            R_max = obj.c / (2 * prf);
            R_min = (obj.c * tau) / 2;
            Raw_Range_Res = (obj.c * tau) / 2;
            Compressed_Res = obj.c / (2 * BW); 
            Duty_Cycle = tau * prf;
            P_avg = Pt * Duty_Cycle;
            
            % Updated Radar Equation using G_tx and G_rx
            Pr_target = (Pt * G_tx * G_rx * (lambda^2) * sigma) / (((4 * pi)^3) * (R_target^4) * L);
            Pn = obj.k * obj.T0 * BW * Fn; 
            SNR_base_dB = 10 * log10(Pr_target / Pn);
            
            % Update Tab 1 UI
            obj.LblCurrentSNROut.String = sprintf('%.2f dB', SNR_base_dB);
            if SNR_base_dB < 10, obj.LblCurrentSNROut.ForegroundColor = 'r'; else, obj.LblCurrentSNROut.ForegroundColor = [0 0.5 0]; end
            obj.LblRangeResOut.String = sprintf('%.2f m (Raw) -> %.2f m (Comp)', Raw_Range_Res, Compressed_Res);
            obj.LblMaxRangeOut.String = sprintf('%.2f km', R_max / 1e3);
            obj.LblMinRangeOut.String = sprintf('%.2f m', R_min);
            obj.LblAvgPowerOut.String = sprintf('%.2f W', P_avg);
            obj.LblDutyCycleOut.String = sprintf('%.2f %%', Duty_Cycle * 100);
            
            % Plot 1: SNR
            r_vec = linspace(1000, max(R_target*1.5, R_max), 500);
            snr_vec_dB = 10 * log10(((Pt * G_tx * G_rx * (lambda^2) * sigma) ./ (((4 * pi)^3) .* (r_vec.^4) .* L)) ./ Pn);
            plot(obj.AxSNR, r_vec/1e3, snr_vec_dB, 'LineWidth', 2, 'Color', [0 0.4470 0.7410], 'DisplayName', '1/R^4 Curve');
            hold(obj.AxSNR, 'on'); 
            plot(obj.AxSNR, R_target/1e3, SNR_base_dB, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Target Location'); 
            hold(obj.AxSNR, 'off');
            legend(obj.AxSNR, 'Location', 'northeast');
            
            % Plot 2: Pulse Train
            PRI = 1 / prf; t_vec_pulse = linspace(0, 3 * PRI, 1000);
            plot(obj.AxPulseTrain, t_vec_pulse * 1e6, double(mod(t_vec_pulse, PRI) <= tau), 'LineWidth', 2, 'Color', [0.8500 0.3250 0.0980], 'DisplayName', 'Tx Power');
            ylim(obj.AxPulseTrain, [-0.2 1.2]);
            legend(obj.AxPulseTrain, 'Location', 'northeast');
            
            %% 4. ARRAY DESIGN (TAB 5) - 2D HEATMAP
            % Helper function for steering vector
            calcSteeringVec = @(az, el, Ny, Nz) exp(1j * (2*pi/lambda) * ...
                (kron(ones(Nz,1), ((0:Ny-1)*d - mean((0:Ny-1)*d))') * cosd(el)*sind(az) + ...
                 kron(((0:Nz-1)*d - mean((0:Nz-1)*d))', ones(Ny,1)) * sind(el)));
                 
            % Generate 2D grid for heatmap
            az_grid = linspace(-90, 90, 100);
            el_grid = linspace(-90, 90, 100);
            [AZ, EL] = meshgrid(az_grid, el_grid);
            AF_2way_map = zeros(size(AZ));
            
            % Assuming steering towards Boresight (0,0) for the pure array design view
            for i = 1:numel(AZ)
                v_tx = calcSteeringVec(AZ(i), EL(i), Ny_tx, Nz_tx);
                v_rx = calcSteeringVec(AZ(i), EL(i), Ny_rx, Nz_rx);
                % Uniform weighting for design view
                w_tx = ones(NumElements_Tx, 1) / NumElements_Tx;
                w_rx = ones(NumElements_Rx, 1) / NumElements_Rx;
                
                AF_tx_val = abs(w_tx' * v_tx)^2;
                AF_rx_val = abs(w_rx' * v_rx)^2;
                AF_2way_map(i) = AF_tx_val * AF_rx_val;
            end
            
            AF_2way_dB = 10*log10(AF_2way_map / max(AF_2way_map(:)));
            AF_2way_dB(AF_2way_dB < -40) = -40;
            
            imagesc(obj.AxArrayHeatmap, 'XData', az_grid, 'YData', el_grid, 'CData', AF_2way_dB);
            obj.AxArrayHeatmap.YDir = 'normal';
            colormap(obj.AxArrayHeatmap, 'jet');
            clim(obj.AxArrayHeatmap, [-40 0]);
            
            % Principal Cuts
            plot(obj.AxArrayCuts, az_grid, AF_2way_dB(50,:), 'b', 'LineWidth', 2, 'DisplayName', 'Azimuth Cut (El=0)');
            hold(obj.AxArrayCuts, 'on');
            plot(obj.AxArrayCuts, el_grid, AF_2way_dB(:,50), 'r', 'LineWidth', 2, 'DisplayName', 'Elevation Cut (Az=0)');
            hold(obj.AxArrayCuts, 'off');
            legend(obj.AxArrayCuts, 'Location', 'best');
            ylim(obj.AxArrayCuts, [-40 5]);
            
            %% 5. KINEMATICS, BEAMFORMING & 3D WORLD (TAB 2 & 3D FIG)
            % Use Rx Array for Beam Steering calculation in Tab 2
            v_targ = calcSteeringVec(targAz, targEl, Ny_rx, Nz_rx);
            v_jam  = calcSteeringVec(jamAz, jamEl, Ny_rx, Nz_rx);
            v_scan = calcSteeringVec(scanAz, scanEl, Ny_rx, Nz_rx);
            
            if useMVDR
                P_j_linear = Pr_target * 10^(jamJSR/10);
                sigma_n2 = Pn;
                R_in = sigma_n2 * eye(NumElements_Rx) + P_j_linear * (v_jam * v_jam');
                R_inv_v = R_in \ v_scan; 
                w = R_inv_v / (v_scan' * R_inv_v);
            else
                w = v_scan / NumElements_Rx; 
            end
            
            az_ang = linspace(-90, 90, 360);
            AF_mag = zeros(1, length(az_ang));
            for i = 1:length(az_ang)
                v_theta = calcSteeringVec(az_ang(i), scanEl, Ny_rx, Nz_rx);
                AF_mag(i) = abs(w' * v_theta)^2;
            end
            AF_dB = 10*log10(AF_mag / max(AF_mag));
            AF_dB(AF_dB < -50) = -50; 
            
            plot(obj.AxBeamPattern, az_ang, AF_dB, 'LineWidth', 2, 'Color', 'b', 'DisplayName', 'Rx Antenna Pattern'); hold(obj.AxBeamPattern, 'on');
            plot(obj.AxBeamPattern, targAz, 0, 'gp', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Target');
            plot(obj.AxBeamPattern, jamAz, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Jammer');
            xline(obj.AxBeamPattern, scanAz, '--k', 'Look Dir', 'HandleVisibility', 'off');
            hold(obj.AxBeamPattern, 'off');
            legend(obj.AxBeamPattern, 'Location', 'best');
            ylim(obj.AxBeamPattern, [-50 5]);
            
            % --- 3D Kinematics and Position ---
            % Target Start Position Vector
            T_start_x = R_target * cosd(targEl)*cosd(targAz);
            T_start_y = R_target * cosd(targEl)*sind(targAz);
            T_start_z = R_target * sind(targEl);
            T_start = [T_start_x, T_start_y, T_start_z];
            
            % Velocity Vector 
            Vel_vec = [targVel * cosd(targHeading), targVel * sind(targHeading), 0];
            
            % Target End Position (After Sim Time)
            T_end = T_start + Vel_vec * simTime;
            
            % Radial Velocity relative to radar (positive = towards radar)
            unit_radial = T_start / norm(T_start);
            v_r = -dot(Vel_vec, unit_radial); 
            
            % Update 3D World View
            cla(obj.Ax3DWorld); hold(obj.Ax3DWorld, 'on');
            
            % Target Trajectory
            plot3(obj.Ax3DWorld, T_start(1), T_start(2), T_start(3), 'gp', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'DisplayName', 'Target (Start)');
            plot3(obj.Ax3DWorld, T_end(1), T_end(2), T_end(3), 'go', 'MarkerSize', 8, 'DisplayName', 'Target (End)');
            plot3(obj.Ax3DWorld, [T_start(1) T_end(1)], [T_start(2) T_end(2)], [T_start(3) T_end(3)], '-g', 'LineWidth', 2, 'DisplayName', 'Flight Path');
            
            % Jammer
            J_x = R_target * cosd(jamEl)*cosd(jamAz);
            J_y = R_target * cosd(jamEl)*sind(jamAz);
            J_z = R_target * sind(jamEl);
            plot3(obj.Ax3DWorld, J_x, J_y, J_z, 'rx', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Jammer');
            plot3(obj.Ax3DWorld, [0 J_x], [0 J_y], [0 J_z], '--r', 'HandleVisibility', 'off');
            
            % Beam Steering direction Cone Visualization
            B_x = R_target * 0.9 * cosd(scanEl)*cosd(scanAz);
            B_y = R_target * 0.9 * cosd(scanEl)*sind(scanAz);
            B_z = R_target * 0.9 * sind(scanEl);
            plot3(obj.Ax3DWorld, [0 B_x], [0 B_y], [0 B_z], '-b', 'LineWidth', 3, 'DisplayName', 'Main Beam Center');
            
            hold(obj.Ax3DWorld, 'off');
            
            lim_val = max(1000, R_target * 1.1);
            xlim(obj.Ax3DWorld, [-0.1*lim_val, lim_val]);
            ylim(obj.Ax3DWorld, [-lim_val/1.5, lim_val/1.5]);
            zlim(obj.Ax3DWorld, [-lim_val/1.5, lim_val/1.5]);
            xlabel(obj.Ax3DWorld, 'X Range (m)');
            ylabel(obj.Ax3DWorld, 'Y Azimuth (m)');
            zlabel(obj.Ax3DWorld, 'Z Elevation (m)');
            legend(obj.Ax3DWorld, 'Location', 'northeast');
            
            %% 6. PULSE-BY-PULSE SIGNAL PROCESSING & DOPPLER
            N_pulses = 16; % Coherent Processing Interval (CPI)
            Fs = max(2.5 * BW, 10e6); 
            t_tx = -tau/2 : 1/Fs : tau/2;
            K = BW / tau; 
            s_tx = exp(1j * pi * K * t_tx.^2); 
            
            T_sim_sig = min(PRI, (R_target*1.5 * 2)/obj.c); 
            t_rx = 0 : 1/Fs : T_sim_sig;
            N_rx = length(t_rx);
            
            rx_matrix = zeros(N_pulses, N_rx);
            mf_ref = conj(fliplr(s_tx)) .* hamming(length(s_tx))';
            
            gain_jam = abs(w' * v_jam);
            P_j_linear = Pr_target * 10^(jamJSR/10);
            jam_amplitude = sqrt(P_j_linear) * gain_jam;
            
            for p = 1:N_pulses
                % Physical range moves slightly between each pulse
                current_R = R_target - (v_r * (p-1) * PRI);
                
                target_delay = (2 * current_R) / obj.c;
                idx_start = round(target_delay * Fs);
                idx_end = idx_start + length(s_tx) - 1;
                
                rx_sig = zeros(1, N_rx);
                if idx_end <= N_rx
                    gain_targ = abs(w' * v_targ);
                    sig_amplitude = sqrt(Pr_target) * gain_targ;
                    
                    % Crucial Step: The microscopic phase shift due to motion
                    phase_shift = exp(-1j * 4 * pi * current_R / lambda);
                    rx_sig(idx_start:idx_end) = sig_amplitude * s_tx * phase_shift;
                end
                
                jammer_sig = jam_amplitude * (randn(1, N_rx) + 1j*randn(1, N_rx)) / sqrt(2);
                noise_amplitude = sqrt(Pn);
                noise_sig = noise_amplitude * (randn(1, N_rx) + 1j*randn(1, N_rx)) / sqrt(2);
                
                rx_matrix(p, :) = rx_sig + jammer_sig + noise_sig;
            end
            
            % 1. Plot Raw Time Domain (Pulse 1)
            plot(obj.AxTimeDomain, t_rx * 1e6, real(rx_matrix(1,:)), 'Color', [0.5 0.5 0.5]);
            xlim(obj.AxTimeDomain, [0, max(1, T_sim_sig*1e6)]);
            
            % 2. Matched Filter (Fast-Time / Range)
            mf_matrix = zeros(N_pulses, N_rx);
            for p = 1:N_pulses
                mf_matrix(p,:) = conv(rx_matrix(p,:), mf_ref, 'same');
            end
            
            range_axis = (t_rx * obj.c) / 2;
            mf_out_dB = 20*log10(abs(mf_matrix(1,:)) + 1e-12);
            mf_out_dB = mf_out_dB - max(mf_out_dB); 
            
            plot(obj.AxMatchedFilter, range_axis / 1e3, mf_out_dB, 'LineWidth', 1.5, 'Color', 'b');
            hold(obj.AxMatchedFilter, 'on');
            xline(obj.AxMatchedFilter, R_target/1e3, '--r', 'True Target Range');
            hold(obj.AxMatchedFilter, 'off');
            ylim(obj.AxMatchedFilter, [-60 5]);
            xlim(obj.AxMatchedFilter, [0, max(1, max(range_axis)/1e3)]);
            
            % 3. Range-Doppler Map (Slow-Time FFT)
            rd_map = fftshift(fft(mf_matrix, [], 1), 1);
            rd_map_dB = 20*log10(abs(rd_map) + 1e-12);
            rd_map_dB = rd_map_dB - max(rd_map_dB(:)); % Normalize
            
            doppler_axis = linspace(-prf/2, prf/2, N_pulses);
            vel_axis = doppler_axis * lambda / 2; % v = f_d * lambda / 2
            
            imagesc(obj.AxRangeDoppler, 'XData', range_axis/1e3, 'YData', vel_axis, 'CData', rd_map_dB);
            obj.AxRangeDoppler.YDir = 'normal';
            colormap(obj.AxRangeDoppler, 'jet');
            xlim(obj.AxRangeDoppler, [0, max(1, max(range_axis)/1e3)]);
            ylim(obj.AxRangeDoppler, [min(vel_axis), max(vel_axis)]);
            caxis(obj.AxRangeDoppler, [-50 0]);
            
            hold(obj.AxRangeDoppler, 'on');
            plot(obj.AxRangeDoppler, R_target/1e3, v_r, 'w+', 'MarkerSize', 10, 'LineWidth', 2);
            hold(obj.AxRangeDoppler, 'off');
        end
        
        function runOptimizer(obj)
            req_snr_dB = obj.getNum(obj.EditOptReqSNR.String, 15);
            req_res = obj.getNum(obj.EditOptReqRes.String, 15);
            max_dc = obj.getNum(obj.EditOptMaxDC.String, 10) / 100;
            
            R_target = obj.getNum(obj.EditTargetRange.String, 50) * 1e3;
            sigma = obj.getNum(obj.EditTargetRCS.String, 5);
            f = obj.getNum(obj.EditFrequency.String, 3) * 1e9;
            lambda = obj.c / f;
            L = 10^(obj.getNum(obj.EditSystemLosses.String, 5) / 10);
            F_dB = obj.getNum(obj.EditNoiseFigure.String, 3);
            Fn = 10^(F_dB / 10);
            jamJSR = obj.getNum(obj.EditJammerJSR.String, 30);
            useMVDR = obj.CheckMVDR.Value;
            
            prf_opt = obj.c / (2 * R_target * 1.2); 
            BW_opt = obj.c / (2 * req_res);
            tau_opt = max_dc / prf_opt;
            
            obj.EditPRF.String = num2str(prf_opt/1e3, '%.2f');
            obj.EditChirpBW.String = num2str(BW_opt/1e6, '%.2f');
            obj.EditPulseWidth.String = num2str(tau_opt*1e6, '%.2f');
            
            Ny_tx = round(obj.getNum(obj.EditTxNy.String, 8)); Nz_tx = round(obj.getNum(obj.EditTxNz.String, 8));
            Ny_rx = round(obj.getNum(obj.EditRxNy.String, 8)); Nz_rx = round(obj.getNum(obj.EditRxNz.String, 8));
            NumElements_Tx = Ny_tx * Nz_tx;
            NumElements_Rx = Ny_rx * Nz_rx;
            
            G_elem_dB = obj.getNum(obj.EditAntennaGain.String, 10);
            G_elem = 10^(G_elem_dB / 10);
            G_tx = G_elem * NumElements_Tx;
            G_rx = G_elem * NumElements_Rx;
            
            targAz = obj.getNum(obj.EditTargetAz.String, 20); targEl = obj.getNum(obj.EditTargetEl.String, 10);
            jamAz = obj.getNum(obj.EditJammerAz.String, -30); jamEl = obj.getNum(obj.EditJammerEl.String, 5);
            scanAz = obj.getNum(obj.EditScanAz.String, 20); scanEl = obj.getNum(obj.EditScanEl.String, 10);
            
            d = lambda / 2;
            
            calcSteeringVec = @(az, el, Ny, Nz) exp(1j * (2*pi/lambda) * ...
                (kron(ones(Nz,1), ((0:Ny-1)*d - mean((0:Ny-1)*d))') * cosd(el)*sind(az) + ...
                 kron(((0:Nz-1)*d - mean((0:Nz-1)*d))', ones(Ny,1)) * sind(el)));

            v_targ = calcSteeringVec(targAz, targEl, Ny_rx, Nz_rx);
            v_jam  = calcSteeringVec(jamAz, jamEl, Ny_rx, Nz_rx);
            v_scan = calcSteeringVec(scanAz, scanEl, Ny_rx, Nz_rx);
            
            Pn = obj.k * obj.T0 * BW_opt * Fn;
            PG = tau_opt * BW_opt; 
            
            Pr_factor = (G_tx * G_rx * lambda^2 * sigma) / (((4 * pi)^3) * (R_target^4) * L);
            
            Pt_low = 1;      
            Pt_high = 1e9;   
            Pt_opt = 100e3;
            
            logText = sprintf('--- Starting 3D Optimization ---\n');
            logText = [logText, sprintf('Target: %d km, RCS: %.1f m^2\n', R_target/1e3, sigma)];
            logText = [logText, sprintf('Calculated PRF: %.2f kHz (for Ambiguity margin)\n', prf_opt/1e3)];
            logText = [logText, sprintf('Calculated BW: %.2f MHz (for %d m Res)\n', BW_opt/1e6, req_res)];
            logText = [logText, sprintf('Calculated Pulse Width: %.2f us (Duty Cycle limit)\n\n', tau_opt*1e6)];
            
            success = false;
            for iter = 1:50
                Pt_opt = (Pt_low + Pt_high) / 2;
                Pr = Pt_opt * Pr_factor;
                
                P_j_linear = Pr * 10^(jamJSR/10);
                if useMVDR
                    R_in = Pn * eye(NumElements_Rx) + P_j_linear * (v_jam * v_jam');
                    R_inv_v = R_in \ v_scan;
                    w = R_inv_v / (v_scan' * R_inv_v);
                else
                    w = v_scan / NumElements_Rx;
                end
                
                S_out = Pr * abs(w' * v_targ)^2;
                if useMVDR
                    IN_out = real(w' * R_in * w);
                else
                    IN_out = Pn * (w'*w) + P_j_linear * abs(w' * v_jam)^2;
                end
                
                SINR_array = S_out / IN_out;
                SINR_final_dB = 10*log10(SINR_array * PG);
                
                if abs(SINR_final_dB - req_snr_dB) < 0.1
                    success = true; break;
                elseif SINR_final_dB < req_snr_dB
                    Pt_low = Pt_opt;
                else
                    Pt_high = Pt_opt;
                end
            end
            
            if success || SINR_final_dB >= req_snr_dB
                logText = [logText, sprintf('SUCCESS: Target SINR reached.\n')];
                logText = [logText, sprintf('Required Peak Power (Pt): %.2f kW\n', Pt_opt/1e3)];
                logText = [logText, sprintf('Final SINR achieved: %.2f dB\n', SINR_final_dB)];
                obj.EditPeakPower.String = num2str(Pt_opt/1e3, '%.2f');
            else
                logText = [logText, sprintf('WARNING: Optimization Failed (Max Power Reached: 1 GW)\n')];
                logText = [logText, sprintf('Current Jammer is overpowering the signal.\n')];
                logText = [logText, sprintf('REASON: Jammer JSR is defined relative to target return.\nIncreasing Tx Power also increases Jammer power. \n\n')];
                logText = [logText, sprintf('SOLUTION: Enable MVDR Null Steering, increase Array Size, \nor increase Pulse Compression bandwidth.\n')];
            end
            
            obj.TxtOptLog.String = logText;
            obj.updateCalculations();
        end
    end
end