classdef CustomArrayDesigner < handle
    % CustomArrayDesigner - An interactive 3D tool for designing radar 
    % arrays, beamforming, beamshaping, and analyzing performance.
    % Optimized for memory efficiency and zero-leak persistent rendering.
    %
    % Save this file as 'CustomArrayDesigner.m' and run it from the Command Window.

    properties (Access = private)
        % Main UI Figure
        UIFigure
        
        % Left Panel Tabs
        TabGroupCtrl
        TabDesign
        TabOptimizer
        
        % Inputs - Auto Gen
        EditTotalMod, EditTxRatio, BtnAutoGen
        
        % Inputs - Geometry (Physical)
        EditTxNy, EditTxNz, EditTxDy, EditTxDz
        EditRxNy, EditRxNz, EditRxDy, EditRxDz
        
        % Inputs - Processing & Steering
        DropWindow
        EditSteerAz, EditSteerEl
        EditFreq, EditPt, EditBW
        
        % Inputs - Optimizer Constraints & Targets
        EditOptRCS, EditOptRange, EditOptSNR
        EditOptMaxMod, EditOptMaxPt, DropOptBand
        TxtOptLog, BtnRunOpt
        
        % Dashboard Labels
        LblDirectivity, LblAzBW, LblElBW
        LblPSLL, LblRangeRes, LblMaxRange, LblTargetSNR
        
        % Right Panel Tabs
        TabGroupVis
        TabLocal
        TabGlobal
        
        % Earth Map Customization
        BtnLoadMap
        EarthTexture = []; % Stores the loaded image array
        
        % Axes
        AxGeom
        AxPat3D
        AxCutAz
        AxCutEl
        AxGlobe
        
        % Persistent Graphics Handles (Memory Optimization)
        hGeomVirt
        hGeomTx
        hGeomRx
        hPatSurf
        hCutAzLine, hCutAzSteer
        hCutElLine, hCutElSteer
        hEarthSurf, hRadarOrigin, hGlobalPatSurf
    end
    
    properties (Constant)
        c = 299792458; % Speed of light (m/s)
        kT0 = 1.38e-23 * 290; % Boltzmann * Temp
    end
    
    methods
        function obj = CustomArrayDesigner()
            obj.buildUI();
            % Enable interactive 3D rotation on the figures
            rotate3d(obj.AxGeom, 'on');
            rotate3d(obj.AxPat3D, 'on');
            rotate3d(obj.AxGlobe, 'on');
            obj.updateCalculations();
        end
    end
    
    methods (Access = private)
        function buildUI(obj)
            % Main Figure
            obj.UIFigure = figure('Name', 'Custom Radar Array & Beamforming Designer', ...
                                  'Position', [50, 50, 1600, 850], ...
                                  'MenuBar', 'none', 'NumberTitle', 'off', 'Color', 'w', ...
                                  'CloseRequestFcn', @obj.onClose);
            
            % --- 1. CONTROL PANEL (LEFT) ---
            obj.TabGroupCtrl = uitabgroup(obj.UIFigure, 'Position', [0.01, 0.01, 0.24, 0.98]);
            obj.TabDesign = uitab(obj.TabGroupCtrl, 'Title', 'Array Design');
            obj.TabOptimizer = uitab(obj.TabGroupCtrl, 'Title', 'Auto-Optimizer');
                          
            % === DESIGN TAB ===
            dy = 1/25; 
            function fld = createInput(parent, row, label, defVal, unit)
                yPos = 1 - row*dy;
                uicontrol('Parent', parent, 'Style', 'text', 'String', [label ' ' unit], ...
                          'Units', 'normalized', 'Position', [0.02, yPos-0.01, 0.60, dy*0.8], 'HorizontalAlignment', 'right', 'FontSize', 9);
                fld = uicontrol('Parent', parent, 'Style', 'edit', 'String', num2str(defVal), ...
                                'Units', 'normalized', 'Position', [0.65, yPos, 0.30, dy*0.7], 'Callback', @(s,e) obj.updateCalculations(), 'FontSize', 9);
            end
            
            % Auto-Generate Section
            uicontrol('Parent', obj.TabDesign, 'Style', 'text', 'String', '--- Auto-Generate Array ---', 'Units', 'normalized', 'Position', [0.05, 1-1*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditTotalMod = createInput(obj.TabDesign, 2, 'Total Modules', 512, '');
            obj.EditTxRatio  = createInput(obj.TabDesign, 3, 'Tx Ratio', 50, '(%)');
            obj.BtnAutoGen   = uicontrol('Parent', obj.TabDesign, 'Style', 'pushbutton', 'String', 'Autofill Array Grid', ...
                                         'Units', 'normalized', 'Position', [0.15, 1-4.2*dy, 0.7, dy*1.0], ...
                                         'BackgroundColor', [0 0.4 0.8], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
                                         'Callback', @(s,e) obj.autoGenArray());
            
            % Physical Array Settings
            uicontrol('Parent', obj.TabDesign, 'Style', 'text', 'String', '--- Transmit Array (Tx) ---', 'Units', 'normalized', 'Position', [0.05, 1-5.5*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditTxNy = createInput(obj.TabDesign, 6.5, 'Elements Y (Azimuth)', 16, '');
            obj.EditTxNz = createInput(obj.TabDesign, 7.5, 'Elements Z (Elevation)', 16, '');
            obj.EditTxDy = createInput(obj.TabDesign, 8.5, 'Physical Spacing Y', 0.05, '(m)');
            obj.EditTxDz = createInput(obj.TabDesign, 9.5, 'Physical Spacing Z', 0.05, '(m)');
            
            uicontrol('Parent', obj.TabDesign, 'Style', 'text', 'String', '--- Receive Array (Rx) ---', 'Units', 'normalized', 'Position', [0.05, 1-11*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditRxNy = createInput(obj.TabDesign, 12, 'Elements Y (Azimuth)', 16, '');
            obj.EditRxNz = createInput(obj.TabDesign, 13, 'Elements Z (Elevation)', 16, '');
            obj.EditRxDy = createInput(obj.TabDesign, 14, 'Physical Spacing Y', 0.05, '(m)');
            obj.EditRxDz = createInput(obj.TabDesign, 15, 'Physical Spacing Z', 0.05, '(m)');
            
            uicontrol('Parent', obj.TabDesign, 'Style', 'text', 'String', '--- Beamshaping & Steering ---', 'Units', 'normalized', 'Position', [0.05, 1-16.5*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            uicontrol('Parent', obj.TabDesign, 'Style', 'text', 'String', 'Amplitude Taper:', 'Units', 'normalized', 'Position', [0.02, 1-17.5*dy-0.01, 0.40, dy*0.8], 'HorizontalAlignment', 'right', 'FontSize', 9);
            obj.DropWindow = uicontrol('Parent', obj.TabDesign, 'Style', 'popupmenu', 'String', {'Uniform (Max Gain)', 'Hamming (Low Sidelobes)', 'Hann (Deep Nulls)'}, ...
                                       'Units', 'normalized', 'Position', [0.45, 1-17.5*dy, 0.50, dy*0.8], 'Callback', @(s,e) obj.updateCalculations(), 'FontSize', 9);
            obj.EditSteerAz = createInput(obj.TabDesign, 18.5, 'Steer Azimuth', 0, '(deg)');
            obj.EditSteerEl = createInput(obj.TabDesign, 19.5, 'Steer Elevation', 0, '(deg)');
            
            uicontrol('Parent', obj.TabDesign, 'Style', 'text', 'String', '--- Hardware & Target ---', 'Units', 'normalized', 'Position', [0.05, 1-21*dy, 0.9, dy*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditFreq = createInput(obj.TabDesign, 22, 'Frequency', 3.0, '(GHz)');
            obj.EditPt   = createInput(obj.TabDesign, 23, 'Peak Power', 100, '(kW)');
            obj.EditBW   = createInput(obj.TabDesign, 24, 'Modulation BW', 10, '(MHz)');
            
            % === OPTIMIZER TAB ===
            dyOpt = 1/17;
            uicontrol('Parent', obj.TabOptimizer, 'Style', 'text', ...
                      'String', 'Define threat requirements and hardware limits. The solver synthesizes the optimal Array Size, Frequency, and Power for detection.', ...
                      'Units', 'normalized', 'Position', [0.05, 1-2*dyOpt, 0.9, 0.10], 'HorizontalAlignment', 'left', 'FontSize', 9);
            
            uicontrol('Parent', obj.TabOptimizer, 'Style', 'text', 'String', '--- Target Requirements ---', 'Units', 'normalized', 'Position', [0.05, 1-3.5*dyOpt, 0.9, dyOpt*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');      
            obj.EditOptRCS   = createInput(obj.TabOptimizer, 4.5, 'Target RCS', 0.5, '(m^2)');
            obj.EditOptRange = createInput(obj.TabOptimizer, 5.5, 'Target Range', 80, '(km)');
            obj.EditOptSNR   = createInput(obj.TabOptimizer, 6.5, 'Required SNR', 15, '(dB)');
            
            uicontrol('Parent', obj.TabOptimizer, 'Style', 'text', 'String', '--- Design Constraints ---', 'Units', 'normalized', 'Position', [0.05, 1-8.5*dyOpt, 0.9, dyOpt*0.8], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
            obj.EditOptMaxMod = createInput(obj.TabOptimizer, 9.5, 'Max Available Modules', 2048, '');
            obj.EditOptMaxPt  = createInput(obj.TabOptimizer, 10.5, 'Max Peak Power Limit', 500, '(kW)');
            
            uicontrol('Parent', obj.TabOptimizer, 'Style', 'text', 'String', 'Preferred Band:', 'Units', 'normalized', 'Position', [0.02, 1-11.5*dyOpt-0.01, 0.40, dyOpt*0.8], 'HorizontalAlignment', 'right', 'FontSize', 9);
            obj.DropOptBand = uicontrol('Parent', obj.TabOptimizer, 'Style', 'popupmenu', 'String', {'Auto-Select (Recommended)', 'L-Band (1.5 GHz) - Long Range', 'S-Band (3.0 GHz) - Volume Search', 'C-Band (6.0 GHz) - Multi-Role', 'X-Band (10.0 GHz) - Precision'}, ...
                                       'Units', 'normalized', 'Position', [0.45, 1-11.5*dyOpt, 0.50, dyOpt*0.8], 'FontSize', 9);
            
            obj.BtnRunOpt = uicontrol('Parent', obj.TabOptimizer, 'Style', 'pushbutton', 'String', 'Synthesize Optimal Radar System', ...
                                      'Units', 'normalized', 'Position', [0.1, 1-13.5*dyOpt, 0.8, 0.05], ...
                                      'BackgroundColor', [0.85 0.33 0.10], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
                                      'Callback', @(s,e) obj.runOptimizer());
                                  
            obj.TxtOptLog = uicontrol('Parent', obj.TabOptimizer, 'Style', 'edit', 'String', 'Awaiting optimization...', ...
                                      'Max', 2, 'Min', 0, 'Enable', 'inactive', ...
                                      'Units', 'normalized', 'Position', [0.05, 0.02, 0.9, 0.20], ...
                                      'HorizontalAlignment', 'left', 'FontSize', 9, 'FontName', 'Courier New');

            % --- 2. DASHBOARD PANEL (TOP RIGHT) ---
            pnlDash = uipanel(obj.UIFigure, 'Title', 'Live Radar Performance Metrics', ...
                              'Position', [0.26, 0.85, 0.73, 0.14], 'FontWeight', 'bold', 'BackgroundColor', 'w');
                          
            function lbl = createDashMetric(x, label)
                uicontrol('Parent', pnlDash, 'Style', 'text', 'String', label, ...
                          'Units', 'normalized', 'Position', [x, 0.55, 0.13, 0.3], 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'BackgroundColor', 'w', 'FontSize', 8);
                lbl = uicontrol('Parent', pnlDash, 'Style', 'text', 'String', '-', ...
                                'Units', 'normalized', 'Position', [x, 0.15, 0.13, 0.4], 'HorizontalAlignment', 'center', 'FontSize', 12, 'ForegroundColor', [0 0.4 0.8], 'BackgroundColor', 'w', 'FontWeight', 'bold');
            end
            
            obj.LblDirectivity = createDashMetric(0.01, 'Approx Directivity');
            obj.LblAzBW        = createDashMetric(0.15, 'Azimuth Beamwidth');
            obj.LblElBW        = createDashMetric(0.29, 'Elev Beamwidth');
            obj.LblPSLL        = createDashMetric(0.43, 'Peak Sidelobe Lvl');
            obj.LblRangeRes    = createDashMetric(0.57, 'Range Resolution');
            obj.LblMaxRange    = createDashMetric(0.71, 'Max Range (km)');
            obj.LblTargetSNR   = createDashMetric(0.85, 'Target SNR (Est)');
            
            % --- 3. VISUALIZATION PANEL (BOTTOM RIGHT TABS) ---
            obj.TabGroupVis = uitabgroup(obj.UIFigure, 'Position', [0.26, 0.01, 0.73, 0.83]);
            obj.TabLocal = uitab(obj.TabGroupVis, 'Title', 'Local Array Analysis');
            obj.TabGlobal = uitab(obj.TabGroupVis, 'Title', 'Global Macro Scale');
            
            % Local Analysis Tab
            obj.AxGeom = axes('Parent', obj.TabLocal, 'Position', [0.05, 0.55, 0.4, 0.4]);
            title(obj.AxGeom, '3D Array Geometry & Virtual Aperture');
            xlabel(obj.AxGeom, 'Y (Azimuth \lambda)'); ylabel(obj.AxGeom, 'Z (Elevation \lambda)'); zlabel(obj.AxGeom, 'X');
            grid(obj.AxGeom, 'on'); view(obj.AxGeom, [30, 30]); hold(obj.AxGeom, 'on');
            
            obj.AxPat3D = axes('Parent', obj.TabLocal, 'Position', [0.55, 0.55, 0.4, 0.4]);
            title(obj.AxPat3D, '3D Radiation Pattern (Interactive)');
            grid(obj.AxPat3D, 'on'); view(obj.AxPat3D, [30, 30]); hold(obj.AxPat3D, 'on');
            
            obj.AxCutAz = axes('Parent', obj.TabLocal, 'Position', [0.05, 0.08, 0.4, 0.35]);
            title(obj.AxCutAz, 'Azimuth Principal Cut');
            xlabel(obj.AxCutAz, 'Azimuth Angle (deg)'); ylabel(obj.AxCutAz, 'Normalized Gain (dB)');
            grid(obj.AxCutAz, 'on'); hold(obj.AxCutAz, 'on');
            
            obj.AxCutEl = axes('Parent', obj.TabLocal, 'Position', [0.55, 0.08, 0.4, 0.35]);
            title(obj.AxCutEl, 'Elevation Principal Cut');
            xlabel(obj.AxCutEl, 'Elevation Angle (deg)'); ylabel(obj.AxCutEl, 'Normalized Gain (dB)');
            grid(obj.AxCutEl, 'on'); hold(obj.AxCutEl, 'on');
            
            % Global Macro Scale Tab
            obj.BtnLoadMap = uicontrol('Parent', obj.TabGlobal, 'Style', 'pushbutton', 'String', 'Load Custom Earth Texture (JPG/PNG)', ...
                                       'Units', 'normalized', 'Position', [0.05, 0.93, 0.3, 0.05], 'FontWeight', 'bold', ...
                                       'Callback', @(s,e) obj.loadEarthMap());
            
            obj.AxGlobe = axes('Parent', obj.TabGlobal, 'Position', [0.05, 0.05, 0.9, 0.85]);
            title(obj.AxGlobe, 'Global Macro Scale Coverage (Earth View)');
            set(obj.AxGlobe, 'Color', [0.05 0.05 0.1]); 
            axis(obj.AxGlobe, 'equal'); grid(obj.AxGlobe, 'off'); axis(obj.AxGlobe, 'off');
            hold(obj.AxGlobe, 'on');
            view(obj.AxGlobe, [-98.5 + 65, 39.8 + 25]); % Default look at USA
        end
        
        function val = getNum(~, ctrl, def)
            val = str2double(ctrl.String);
            if isnan(val), val = def; end
        end
        
        function onClose(obj, ~, ~)
            delete(obj.UIFigure);
        end
        
        function loadEarthMap(obj)
            [file, path] = uigetfile({'*.jpg;*.jpeg;*.png;*.tif;*.tiff', 'Image Files (*.jpg, *.png)'}, 'Select Earth Texture Map');
            if isequal(file, 0), return; end
            try
                obj.EarthTexture = imread(fullfile(path, file));
                % Destroy old earth surface to force recreation with new texture
                if ~isempty(obj.hEarthSurf) && isvalid(obj.hEarthSurf)
                    delete(obj.hEarthSurf);
                end
                obj.updateCalculations(); 
            catch ME
                errordlg(['Error loading image: ', ME.message], 'Image Load Error');
            end
        end
        
        function autoGenArray(obj)
            total = obj.getNum(obj.EditTotalMod, 256);
            ratio = obj.getNum(obj.EditTxRatio, 50) / 100;
            
            if total < 2, total = 2; end
            if ratio <= 0 || ratio >= 1, ratio = 0.5; end

            num_tx = max(1, round(total * ratio));
            num_rx = max(1, total - num_tx);

            tx_y = round(sqrt(num_tx)); 
            tx_z = max(1, round(num_tx / tx_y));
            rx_y = round(sqrt(num_rx)); 
            rx_z = max(1, round(num_rx / rx_y));

            obj.EditTxNy.String = num2str(tx_y);
            obj.EditTxNz.String = num2str(tx_z);
            obj.EditRxNy.String = num2str(rx_y);
            obj.EditRxNz.String = num2str(rx_z);

            obj.updateCalculations();
        end
        
        function runOptimizer(obj)
            target_rcs = obj.getNum(obj.EditOptRCS, 1.0);
            target_rng = obj.getNum(obj.EditOptRange, 50) * 1e3;
            req_snr_dB = obj.getNum(obj.EditOptSNR, 15);
            
            max_mod = obj.getNum(obj.EditOptMaxMod, 1024);
            max_pt_kw = obj.getNum(obj.EditOptMaxPt, 500);
            max_pt = max_pt_kw * 1000;
            band_idx = obj.DropOptBand.Value;
            
            if band_idx == 1 % Auto Select
                if target_rng > 100e3, freq = 1.5e9; bandStr = 'L-Band (1.5 GHz)';
                elseif target_rng > 30e3, freq = 3.0e9; bandStr = 'S-Band (3.0 GHz)';
                else, freq = 10.0e9; bandStr = 'X-Band (10.0 GHz)';
                end
            elseif band_idx == 2, freq = 1.5e9; bandStr = 'L-Band (1.5 GHz)';
            elseif band_idx == 3, freq = 3.0e9; bandStr = 'S-Band (3.0 GHz)';
            elseif band_idx == 4, freq = 6.0e9; bandStr = 'C-Band (6.0 GHz)';
            elseif band_idx == 5, freq = 10.0e9; bandStr = 'X-Band (10.0 GHz)';
            end
            
            bw = obj.getNum(obj.EditBW, 10.0) * 1e6;
            lambda = obj.c / freq;
            
            Loss = 10^(5/10); 
            NoiseF = 10^(3/10); 
            req_snr_lin = 10^(req_snr_dB/10);
            
            logText = sprintf('--- ADVANCED DESIGN SYNTHESIS SOLVER ---\n');
            logText = [logText, sprintf('Target: %d km | RCS: %.1f m^2 | Target SNR: %.1f dB\n', target_rng/1000, target_rcs, req_snr_dB)];
            
            obj.DropWindow.Value = 2;
            win_eff = 0.54; 
            logText = [logText, sprintf('1. Tapering: Hamming window forced to reject interference.\n')];
            
            dy_opt = lambda / 2;
            dz_opt = lambda / 2;
            obj.EditTxDy.String = num2str(dy_opt, '%.3f');
            obj.EditTxDz.String = num2str(dz_opt, '%.3f'); 
            obj.EditRxDy.String = num2str(dy_opt, '%.3f');
            obj.EditRxDz.String = num2str(dz_opt, '%.3f');
            logText = [logText, sprintf('2. Freq & Spacing: %s selected. Spacing enforced to %.3f m (\\lambda/2).\n', bandStr, dy_opt)];
            
            tx_ratio = 0.50;
            G_elem = pi; 
            
            optimal_N = -1;
            optimal_Pt = -1;
            success = false;
            
            for N_tot = 4:4:max_mod
                N_tx = round(N_tot * tx_ratio); 
                N_rx = N_tot - N_tx;
                
                G_tx = G_elem * N_tx * win_eff;
                G_rx = G_elem * N_rx * win_eff;
                
                Pt_req = (req_snr_lin * (((4*pi)^3) * (target_rng^4) * obj.kT0 * bw * NoiseF * Loss)) / (G_tx * G_rx * (lambda^2) * target_rcs);
                
                if Pt_req <= max_pt
                    optimal_N = N_tot;
                    optimal_Pt = Pt_req;
                    success = true;
                    break;
                end
            end
            
            if success
                logText = [logText, sprintf('3. Array Sizing: Found valid configuration at %d total modules.\n', optimal_N)];
                logText = [logText, sprintf('4. Power Control: Required Peak Power is %.1f kW (Under %d kW limit).\n', optimal_Pt/1000, max_pt_kw)];
                logText = [logText, sprintf('\nSUCCESS: Radar optimized!\n')];
            else
                optimal_N = max_mod;
                optimal_Pt = max_pt;
                
                N_tx = round(optimal_N * tx_ratio); N_rx = optimal_N - N_tx;
                G_tx = G_elem * N_tx * win_eff; G_rx = G_elem * N_rx * win_eff;
                
                achieved_snr_lin = (optimal_Pt * G_tx * G_rx * (lambda^2) * target_rcs) / (((4*pi)^3) * (target_rng^4) * obj.kT0 * bw * NoiseF * Loss);
                achieved_snr_dB = 10*log10(achieved_snr_lin);
                
                logText = [logText, sprintf('3. Array Sizing: Hit limit of %d modules.\n', max_mod)];
                logText = [logText, sprintf('4. Power Control: Hit limit of %.1f kW.\n', max_pt_kw)];
                logText = [logText, sprintf('\nWARNING: CANNOT achieve required SNR. Max possible SNR is %.1f dB.\n', achieved_snr_dB)];
            end
            
            obj.EditFreq.String = num2str(freq/1e9, '%.2f');
            obj.EditTotalMod.String = num2str(optimal_N);
            obj.EditTxRatio.String = num2str(tx_ratio * 100);
            obj.EditPt.String = num2str(optimal_Pt/1000, '%.2f');
            obj.TxtOptLog.String = logText;
            
            obj.autoGenArray(); 
        end
        
        function updateCalculations(obj)
            %% 1. FETCH PARAMETERS
            tx_Ny = max(1, round(obj.getNum(obj.EditTxNy, 8)));
            tx_Nz = max(1, round(obj.getNum(obj.EditTxNz, 8)));
            tx_dy_m = obj.getNum(obj.EditTxDy, 0.05); 
            tx_dz_m = obj.getNum(obj.EditTxDz, 0.05);
            
            rx_Ny = max(1, round(obj.getNum(obj.EditRxNy, 8)));
            rx_Nz = max(1, round(obj.getNum(obj.EditRxNz, 8)));
            rx_dy_m = obj.getNum(obj.EditRxDy, 0.05);
            rx_dz_m = obj.getNum(obj.EditRxDz, 0.05);
            
            steerAz = obj.getNum(obj.EditSteerAz, 0);
            steerEl = obj.getNum(obj.EditSteerEl, 0);
            freq    = obj.getNum(obj.EditFreq, 3.0) * 1e9;
            bw      = obj.getNum(obj.EditBW, 10.0) * 1e6;
            Pt_kW   = obj.getNum(obj.EditPt, 100);
            winIdx  = obj.DropWindow.Value;
            
            lambda = obj.c / freq;
            tx_dy = tx_dy_m / lambda; tx_dz = tx_dz_m / lambda;
            rx_dy = rx_dy_m / lambda; rx_dz = rx_dz_m / lambda;
            
            %% 2. GENERATE ELEMENT POSITIONS
            [tx_Y, tx_Z] = meshgrid((0:tx_Ny-1)*tx_dy, (0:tx_Nz-1)*tx_dz);
            tx_Y = tx_Y - mean(tx_Y(:)); tx_Z = tx_Z - mean(tx_Z(:));
            tx_pos = [zeros(numel(tx_Y), 1), tx_Y(:), tx_Z(:)];
            
            [rx_Y, rx_Z] = meshgrid((0:rx_Ny-1)*rx_dy, (0:rx_Nz-1)*rx_dz);
            rx_Y = rx_Y - mean(rx_Y(:)); rx_Z = rx_Z - mean(rx_Z(:));
            rx_pos = [zeros(numel(rx_Y), 1), rx_Y(:), rx_Z(:)];
            
            % MEMORY OPTIMIZATION: Check if virtual array is too massive to compute
            num_tx = size(tx_pos, 1);
            num_rx = size(rx_pos, 1);
            if (num_tx * num_rx) <= 5e5
                virt_pos = zeros(num_tx * num_rx, 3); % Preallocate for speed
                for i = 1:num_tx
                    idx_start = (i-1)*num_rx + 1;
                    idx_end = i*num_rx;
                    virt_pos(idx_start:idx_end, :) = rx_pos + tx_pos(i,:);
                end
                virt_pos_unique = unique(round(virt_pos * 1e5) / 1e5, 'rows');
            else
                virt_pos_unique = [NaN NaN NaN]; % Skip render to prevent UI lag
                warning('Virtual array exceeds 500k elements. Rendering skipped to preserve memory.');
            end
            
            %% 3. UPDATE GEOMETRY PLOTS (Persistent Handles)
            if isempty(obj.hGeomVirt) || ~isvalid(obj.hGeomVirt)
                obj.hGeomVirt = scatter3(obj.AxGeom, virt_pos_unique(:,2), virt_pos_unique(:,3), virt_pos_unique(:,1), 10, 'g', 'filled', 'MarkerFaceAlpha', 0.3, 'DisplayName', 'Virtual Array (MIMO)');
                obj.hGeomTx = scatter3(obj.AxGeom, tx_pos(:,2), tx_pos(:,3), tx_pos(:,1), 60, 'b', 's', 'filled', 'DisplayName', 'Tx Elements');
                obj.hGeomRx = scatter3(obj.AxGeom, rx_pos(:,2), rx_pos(:,3), rx_pos(:,1), 40, 'r', '^', 'filled', 'DisplayName', 'Rx Elements');
                legend(obj.AxGeom, 'Location', 'northeast');
            else
                set(obj.hGeomVirt, 'XData', virt_pos_unique(:,2), 'YData', virt_pos_unique(:,3), 'ZData', virt_pos_unique(:,1));
                set(obj.hGeomTx, 'XData', tx_pos(:,2), 'YData', tx_pos(:,3), 'ZData', tx_pos(:,1));
                set(obj.hGeomRx, 'XData', rx_pos(:,2), 'YData', rx_pos(:,3), 'ZData', rx_pos(:,1));
            end
            
            %% 4. BEAMSHAPING & WEIGHTS
            function w = getWindow(N, type)
                if N == 1, w = 1;
                elseif type == 1, w = ones(N, 1);
                elseif type == 2, w = 0.54 - 0.46*cos(2*pi*(0:N-1)'/(N-1));
                elseif type == 3, w = 0.5 * (1 - cos(2*pi*(0:N-1)'/(N-1)));
                end
            end
            
            w_tx_y = getWindow(tx_Ny, winIdx); w_tx_z = getWindow(tx_Nz, winIdx);
            W_tx = w_tx_z * w_tx_y'; w_tx = W_tx(:) / sum(W_tx(:)); 
            
            w_rx_y = getWindow(rx_Ny, winIdx); w_rx_z = getWindow(rx_Nz, winIdx);
            W_rx = w_rx_z * w_rx_y'; w_rx = W_rx(:) / sum(W_rx(:));
            
            u_steer = cosd(steerEl) * sind(steerAz); w_steer = sind(steerEl);
            phase_tx = exp(-1j * 2*pi * (tx_pos(:,2)*u_steer + tx_pos(:,3)*w_steer));
            phase_rx = exp(-1j * 2*pi * (rx_pos(:,2)*u_steer + rx_pos(:,3)*w_steer));
            
            W_tx_c = w_tx .* phase_tx; W_rx_c = w_rx .* phase_rx;
            
            %% 5. CALCULATE RADIATION PATTERN
            az_vec = linspace(-90, 90, 181); el_vec = linspace(-90, 90, 181);
            [AZ, EL] = meshgrid(az_vec, el_vec);
            U = cosd(EL) .* sind(AZ); W = sind(EL);
            
            AF_tx = zeros(size(AZ)); AF_rx = zeros(size(AZ));
            for i = 1:numel(w_tx), AF_tx = AF_tx + W_tx_c(i) * exp(1j * 2*pi * (tx_pos(i,2)*U + tx_pos(i,3)*W)); end
            for i = 1:numel(w_rx), AF_rx = AF_rx + W_rx_c(i) * exp(1j * 2*pi * (rx_pos(i,2)*U + rx_pos(i,3)*W)); end
            
            Pat_2way_dB = 10 * log10(abs(AF_tx .* AF_rx).^2 + 1e-12);
            Pat_2way_dB_norm = Pat_2way_dB - max(Pat_2way_dB(:));
            floor_dB = -50;
            Pat_plot = max(Pat_2way_dB_norm, floor_dB);
            
            %% 6. PLOT LOCAL 3D PATTERN (Persistent Handles)
            R_plot = Pat_plot - floor_dB; 
            X_surf = R_plot .* cosd(EL) .* cosd(AZ); 
            Y_surf = R_plot .* cosd(EL) .* sind(AZ); Z_surf = R_plot .* sind(EL);
            
            if isempty(obj.hPatSurf) || ~isvalid(obj.hPatSurf)
                obj.hPatSurf = surf(obj.AxPat3D, X_surf, Y_surf, Z_surf, Pat_plot, 'EdgeColor', 'none');
                colormap(obj.AxPat3D, 'jet');
            else
                set(obj.hPatSurf, 'XData', X_surf, 'YData', Y_surf, 'ZData', Z_surf, 'CData', Pat_plot);
            end
            clim(obj.AxPat3D, [floor_dB 0]); 
            xlim(obj.AxPat3D, [0, max(R_plot(:))*1.1]);
            
            %% 7. PRINCIPAL CUTS & METRICS (Persistent Handles)
            [~, el_idx] = min(abs(el_vec - steerEl)); az_cut = Pat_2way_dB_norm(el_idx, :);
            [~, az_idx] = min(abs(az_vec - steerAz)); el_cut = Pat_2way_dB_norm(:, az_idx)';
            
            if isempty(obj.hCutAzLine) || ~isvalid(obj.hCutAzLine)
                obj.hCutAzLine = plot(obj.AxCutAz, az_vec, max(az_cut, floor_dB), 'LineWidth', 2, 'Color', 'b');
                obj.hCutAzSteer = xline(obj.AxCutAz, steerAz, '--k');
                obj.hCutElLine = plot(obj.AxCutEl, el_vec, max(el_cut, floor_dB), 'LineWidth', 2, 'Color', 'r');
                obj.hCutElSteer = xline(obj.AxCutEl, steerEl, '--k');
            else
                set(obj.hCutAzLine, 'YData', max(az_cut, floor_dB));
                set(obj.hCutAzSteer, 'Value', steerAz);
                set(obj.hCutElLine, 'YData', max(el_cut, floor_dB));
                set(obj.hCutElSteer, 'Value', steerEl);
            end
            ylim(obj.AxCutAz, [floor_dB, 5]); ylim(obj.AxCutEl, [floor_dB, 5]);
            
            %% 8. PERFORMANCE METRICS & MAX RANGE
            function bw_val = getHPBW(vec, cut, steer_ang)
                idx_main = find(vec >= steer_ang, 1);
                if isempty(idx_main), idx_main = round(length(vec)/2); end
                idx_R = find(cut(idx_main:end) <= -3, 1); idx_L = find(fliplr(cut(1:idx_main)) <= -3, 1);
                if ~isempty(idx_R) && ~isempty(idx_L), bw_val = vec(idx_main + idx_R - 1) - vec(idx_main - idx_L + 1);
                else, bw_val = NaN; end
            end
            
            hpbw_az = getHPBW(az_vec, az_cut, steerAz); hpbw_el = getHPBW(el_vec, el_cut, steerEl);
            
            function psll = getPSLL(cut)
                [pks, ~] = findpeaks(cut); pks = sort(pks, 'descend');
                if length(pks) > 1, psll = pks(2); else, psll = -Inf; end
            end
            psll_val = max(getPSLL(az_cut), getPSLL(el_cut));
            
            if ~isnan(hpbw_az) && ~isnan(hpbw_el), Dir_approx = 10*log10(41253 / (hpbw_az * hpbw_el));
            else, Dir_approx = 0; end
            
            res = obj.c / (2 * bw);
            G_linear = 10^(Dir_approx/10); Pt_watts = Pt_kW * 1000;
            target_rng = obj.getNum(obj.EditOptRange, 50) * 1e3; target_rcs = obj.getNum(obj.EditOptRCS, 1.0);
            Loss = 10^(5/10); NoiseF = 10^(3/10); req_snr_lin = 10^(13/10); 
            
            snr_linear = (Pt_watts * G_linear * lambda^2 * target_rcs) / (((4*pi)^3) * (target_rng^4) * obj.kT0 * bw * NoiseF * Loss);
            snr_db = 10*log10(snr_linear);
            R_max_m = ( (Pt_watts * G_linear * lambda^2 * target_rcs) / (((4*pi)^3) * obj.kT0 * bw * NoiseF * Loss * req_snr_lin) )^0.25;
            R_max_km = R_max_m / 1000;
            
            %% 9. UPDATE DASHBOARD
            obj.LblDirectivity.String = sprintf('%.1f dB', Dir_approx);
            obj.LblAzBW.String = sprintf('%.1f\\circ', hpbw_az);
            obj.LblElBW.String = sprintf('%.1f\\circ', hpbw_el);
            if psll_val < floor_dB, obj.LblPSLL.String = sprintf('< %.0f dB', floor_dB); else, obj.LblPSLL.String = sprintf('%.1f dB', psll_val); end
            obj.LblRangeRes.String = sprintf('%.1f m', res);
            obj.LblMaxRange.String = sprintf('%.1f', R_max_km);
            obj.LblTargetSNR.String = sprintf('%.1f dB', snr_db);
            if snr_db < 13, obj.LblTargetSNR.ForegroundColor = 'r'; else, obj.LblTargetSNR.ForegroundColor = [0 0.5 0]; end
            
            %% 10. GLOBAL MACRO SCALE VISUALIZATION (Persistent Handles)
            Re = 6371;
            
            if isempty(obj.hEarthSurf) || ~isvalid(obj.hEarthSurf)
                [X_e, Y_e, Z_e] = sphere(100);
                if isempty(obj.EarthTexture)
                    obj.hEarthSurf = surf(obj.AxGlobe, X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', [0.1 0.25 0.5], 'EdgeColor', [0.15 0.35 0.6], 'FaceAlpha', 0.9);
                else
                    obj.hEarthSurf = surf(obj.AxGlobe, X_e*Re, Y_e*Re, Z_e*Re, 'CData', obj.EarthTexture, 'FaceColor', 'texturemap', 'EdgeColor', 'none');
                end
                
                lat = 39.8; lon = -98.5;
                x0 = Re * cosd(lat) * cosd(lon); y0 = Re * cosd(lat) * sind(lon); z0 = Re * sind(lat);
                obj.hRadarOrigin = scatter3(obj.AxGlobe, x0, y0, z0, 80, 'r', 'filled', 'MarkerEdgeColor', 'w', 'DisplayName', 'Radar Location (USA)');
            else
                x0 = obj.hRadarOrigin.XData; y0 = obj.hRadarOrigin.YData; z0 = obj.hRadarOrigin.ZData;
                lat = asind(z0/Re); lon = atan2d(y0, x0);
            end
            
            Pat_2way_lin_norm = 10.^(Pat_plot / 10);
            R_shape_km = R_max_km .* (Pat_2way_lin_norm).^(0.25);
            X_local = R_shape_km .* cosd(EL) .* cosd(AZ); Y_local = R_shape_km .* cosd(EL) .* sind(AZ); Z_local = R_shape_km .* sind(EL);
            
            clat = cosd(lat); slat = sind(lat); clon = cosd(lon); slon = sind(lon);
            R_enu2ecef = [-slon, -slat*clon, clat*clon; clon, -slat*slon, clat*slon; 0, clat, slat];
            pts_enu = [Y_local(:)'; Z_local(:)'; X_local(:)'];
            pts_ecef = R_enu2ecef * pts_enu;
            
            X_glob = reshape(pts_ecef(1,:), size(EL)) + x0; Y_glob = reshape(pts_ecef(2,:), size(EL)) + y0; Z_glob = reshape(pts_ecef(3,:), size(EL)) + z0;
            
            if isempty(obj.hGlobalPatSurf) || ~isvalid(obj.hGlobalPatSurf)
                obj.hGlobalPatSurf = surf(obj.AxGlobe, X_glob, Y_glob, Z_glob, Pat_plot, 'EdgeColor', 'none', 'FaceAlpha', 0.6);
                colormap(obj.AxGlobe, 'jet');
            else
                set(obj.hGlobalPatSurf, 'XData', X_glob, 'YData', Y_glob, 'ZData', Z_glob, 'CData', Pat_plot);
            end
            clim(obj.AxGlobe, [floor_dB 0]);
        end
    end
end