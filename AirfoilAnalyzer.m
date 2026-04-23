classdef AirfoilAnalyzer < handle
    % AIRFOILANALYZER High-Fidelity Interactive App for Aerodynamic Flow
    % Features: True Unsteady Panel Method (UPM), Viscous-Inviscid Interaction (VII), Transonic Shock Capture
    
    properties
        % UI Components
        UIFigure
        GridLayout
        LeftPanel, CenterPanel, RightPanel
        
        % Controls
        ddShapeType, ddStandardID
        efAltitude, efMach
        efReCrit, efTripX
        slAoA, efAoA
        cbParticles, cbPressure, cbVelocity, cbPitching
        btnSolve, btnToggleAnim, btnSweep
        lblStatus
        lblCL, lblCD, lblLD
        
        % Axes
        axMain, cbMain
        axBL, axPressureGrad, axShearStress
        
        % Graphics Objects
        surfHeatmap, patchAirfoil, scatterParticles
        plotDPDX, plotShear, plotSep
        patchAirfoilZoom, plotBLEdgeZoom, plotBLProfile, plotBLNormal
        
        % State Variables
        Timer
        Particles
        AirfoilData
        FlowData
        AnimationState % Tracks animation progress
    end
    
    methods
        function app = AirfoilAnalyzer()
            % Constructor: Initialize UI and Physics
            app.AnimationState = struct('Frame', 0, 'MaxFrames', 300, 'IsSolved', false);
            app.createUI();
            
            % Setup Timer for High-Fidelity Particle/Vortex Animation
            app.Timer = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, ...
                'TimerFcn', @(~,~) app.animateFrame());
                
            app.updateGeometryPreview(); % Initial Preview
        end
        
        function delete(app)
            % Destructor: Clean up timer gracefully
            if isa(app.Timer, 'timer') && isvalid(app.Timer)
                stop(app.Timer);
                delete(app.Timer);
            end
            delete(app.UIFigure);
        end
    end
    
    methods (Access = private)
        function createUI(app)
            % Create main figure
            app.UIFigure = uifigure('Name', 'High-Fidelity Airfoil Boundary Layer Analyzer', ...
                'Position', [50, 50, 1500, 850], 'Color', 'w');
            
            app.GridLayout = uigridlayout(app.UIFigure, [1 3]);
            app.GridLayout.ColumnWidth = {280, '1x', 380};
            app.GridLayout.Padding = [10 10 10 10];
            
            %% Left Panel: Controls
            app.LeftPanel = uipanel(app.GridLayout, 'Title', 'Simulation Parameters', ...
                'BackgroundColor', 'w', 'FontWeight', 'bold');
            gridLeft = uigridlayout(app.LeftPanel, [28 1]);
            
            rowH = num2cell(repmat(22, 1, 28));
            rowH{25} = 28; % Sweep Btn
            rowH{26} = 35; % Solve Btn
            rowH{27} = 25; % Pause Btn
            rowH{28} = '1x'; % Spacer
            gridLeft.RowHeight = rowH;
            
            uilabel(gridLeft, 'Text', 'Airfoil Type:');
            app.ddShapeType = uidropdown(gridLeft, 'Items', {'Standard (NACA)', 'Diamond', 'Wedge', 'Flat Line'}, ...
                'ValueChangedFcn', @(~,~) app.onShapeTypeChanged());
            
            app.ddStandardID = uidropdown(gridLeft, 'Items', {'0012', '2412', '4412', '6409', '0024'}, ...
                'ValueChangedFcn', @(~,~) app.updateGeometryPreview());
            
            uilabel(gridLeft, 'Text', 'Altitude (Km):');
            app.efAltitude = uieditfield(gridLeft, 'numeric', 'Value', 10.0000, ...
                'Limits', [0 30], 'ValueDisplayFormat', '%.4f', ...
                'ValueChangedFcn', @(~,~) app.updateGeometryPreview());
            
            uilabel(gridLeft, 'Text', 'Mach Number:');
            app.efMach = uieditfield(gridLeft, 'numeric', 'Value', 0.3, ...
                'Limits', [0.01 10.0], 'ValueChangedFcn', @(~,~) app.updateGeometryPreview());
                
            uilabel(gridLeft, 'Text', 'Transition Re_crit:');
            app.efReCrit = uieditfield(gridLeft, 'numeric', 'Value', 5e5, ...
                'Limits', [1e5 3e6], 'ValueDisplayFormat', '%d');
                
            uilabel(gridLeft, 'Text', 'Trip Wire (x/c):');
            app.efTripX = uieditfield(gridLeft, 'numeric', 'Value', 1.0, ...
                'Limits', [0.0 1.0], 'ValueDisplayFormat', '%.2f');
            
            uilabel(gridLeft, 'Text', 'Angle of Attack (deg):');
            app.slAoA = uislider(gridLeft, 'Limits', [-20 20], 'Value', 5, ...
                'ValueChangedFcn', @(src, event) app.syncAoA(src));
            app.efAoA = uieditfield(gridLeft, 'numeric', 'Value', 5, ...
                'Limits', [-20 20], 'ValueChangedFcn', @(src, event) app.syncAoA(src));
                
            app.cbPitching = uicheckbox(gridLeft, 'Text', 'Dynamic Pitching (\pm 5^\circ)', 'Value', 0);
            app.cbParticles = uicheckbox(gridLeft, 'Text', 'Show Air Particles', 'Value', 1);
            app.cbPressure = uicheckbox(gridLeft, 'Text', 'Pressure Heatmap', 'Value', 0, ...
                'ValueChangedFcn', @(~,~) app.updateVisuals());
            app.cbVelocity = uicheckbox(gridLeft, 'Text', 'Velocity Heatmap', 'Value', 1, ...
                'ValueChangedFcn', @(~,~) app.updateVisuals());
                
            % Aerodynamic Readouts
            app.lblCL = uilabel(gridLeft, 'Text', 'C_L (Lift): --', 'FontWeight', 'bold', 'FontColor', [0 0.4 0]);
            app.lblCD = uilabel(gridLeft, 'Text', 'C_D (Drag): --', 'FontWeight', 'bold', 'FontColor', [0.6 0 0]);
            app.lblLD = uilabel(gridLeft, 'Text', 'L/D (Effic): --', 'FontWeight', 'bold', 'FontColor', [0 0.4 0.6]);
            app.lblStatus = uilabel(gridLeft, 'Text', 'Status: Preview Mode', 'FontColor', [0.4 0.4 0.4], 'FontWeight', 'bold');
            
            % Action Buttons
            app.btnSweep = uibutton(gridLeft, 'Text', 'Run Drag Polar Sweep', ...
                'BackgroundColor', [0.1 0.4 0.6], 'FontColor', 'w', 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.runAlphaSweep());
                
            app.btnSolve = uibutton(gridLeft, 'Text', 'Solve & Animate (15s)', ...
                'BackgroundColor', [0.6 0.1 0.1], 'FontColor', 'w', 'FontWeight', 'bold', ...
                'ButtonPushedFcn', @(~,~) app.runHighFidelitySimulation());
                
            app.btnToggleAnim = uibutton(gridLeft, 'Text', 'Pause Animation', ...
                'Enable', 'off', 'ButtonPushedFcn', @(~,~) app.toggleAnimation());
            
            %% Center Panel: Main Visualization
            app.CenterPanel = uipanel(app.GridLayout, 'BackgroundColor', 'w', 'BorderType', 'none');
            
            app.axMain = uiaxes(app.CenterPanel, 'Position', [10 10 750 800], ...
                'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.4 0.4 0.4]);
            app.axMain.XLim = [-0.5 1.5];
            app.axMain.YLim = [-0.6 0.6];
            app.axMain.DataAspectRatio = [1 1 1];
            title(app.axMain, 'Flow Field Visualization', 'FontSize', 14, 'Color', 'w');
            xlabel(app.axMain, 'x/c', 'Color', 'w'); ylabel(app.axMain, 'y/c', 'Color', 'w');
            hold(app.axMain, 'on'); grid(app.axMain, 'on');
            
            app.cbMain = colorbar(app.axMain);
            app.cbMain.Color = 'w';
            
            [X, Y] = meshgrid(linspace(-0.5, 1.5, 150), linspace(-0.6, 0.6, 150));
            app.surfHeatmap = surf(app.axMain, X, Y, -ones(size(X)), zeros(size(X)), ...
                'EdgeColor', 'none', 'FaceColor', 'interp');
            view(app.axMain, 2); 
            colormap(app.axMain, jet);
            
            app.patchAirfoil = patch(app.axMain, 'XData', [], 'YData', [], 'ZData', [], ...
                'FaceColor', 'w', 'EdgeColor', 'w', 'LineWidth', 1.5);
                
            app.plotSep = plot3(app.axMain, NaN, NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
            app.scatterParticles = scatter3(app.axMain, [], [], [], 3, [0.6 0.6 0.6], 'filled', 'MarkerFaceAlpha', 0.6);
            
            %% Right Panel: Plots
            app.RightPanel = uipanel(app.GridLayout, 'BackgroundColor', 'w', 'BorderType', 'none');
            gridRight = uigridlayout(app.RightPanel, [3 1]);
            
            app.axBL = uiaxes(gridRight);
            title(app.axBL, 'Boundary Layer Profile (Physical Zoom)');
            xlabel(app.axBL, 'Physical x/c'); ylabel(app.axBL, 'Physical y/c');
            grid(app.axBL, 'on'); hold(app.axBL, 'on');
            
            app.patchAirfoilZoom = patch(app.axBL, 'XData', [], 'YData', [], 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'k', 'LineWidth', 1.5);
            app.plotBLEdgeZoom = plot(app.axBL, NaN, NaN, 'c-', 'LineWidth', 2);
            app.plotBLProfile = plot(app.axBL, NaN, NaN, 'b-', 'LineWidth', 2);
            app.plotBLNormal = plot(app.axBL, NaN, NaN, 'k:', 'LineWidth', 1);
            
            app.axPressureGrad = uiaxes(gridRight);
            title(app.axPressureGrad, 'Pressure Gradient (dp/dx)');
            xlabel(app.axPressureGrad, 'x/c'); ylabel(app.axPressureGrad, 'dp/dx (Pa/m)');
            grid(app.axPressureGrad, 'on'); hold(app.axPressureGrad, 'on');
            app.plotDPDX = plot(app.axPressureGrad, NaN, NaN, 'r-', 'LineWidth', 2);
            
            app.axShearStress = uiaxes(gridRight);
            title(app.axShearStress, 'Wall Shear Stress (\tau_w)');
            xlabel(app.axShearStress, 'x/c'); ylabel(app.axShearStress, '\tau_w (Pa)');
            grid(app.axShearStress, 'on'); hold(app.axShearStress, 'on');
            app.plotShear = plot(app.axShearStress, NaN, NaN, 'k-', 'LineWidth', 2);
            yline(app.axShearStress, 0, 'r--', 'Separation Threshold', 'LabelHorizontalAlignment', 'left');
        end
        
        function syncAoA(app, src)
            if src == app.slAoA
                app.efAoA.Value = app.slAoA.Value;
            else
                app.slAoA.Value = app.efAoA.Value;
            end
            app.updateGeometryPreview();
        end
        
        function onShapeTypeChanged(app)
            type = app.ddShapeType.Value;
            app.ddStandardID.Enable = strcmp(type, 'Standard (NACA)');
            app.updateGeometryPreview();
        end
        
        function updateGeometryPreview(app)
            if isa(app.Timer, 'timer') && isvalid(app.Timer) && strcmp(app.Timer.Running, 'on')
                stop(app.Timer); 
            end
            
            app.AnimationState.IsSolved = false;
            app.btnToggleAnim.Enable = 'off';
            app.lblStatus.Text = 'Status: Preview Mode (Click Solve to Run CFD)';
            app.lblStatus.FontColor = [0.4 0.4 0.4];
            
            app.lblCL.Text = 'C_L (Lift): --';
            app.lblCD.Text = 'C_D (Drag): --';
            app.lblLD.Text = 'L/D (Effic): --';
            
            aoa = app.slAoA.Value * pi / 180;
            [x_c, yu, yl, dyu, dyl, x_node_base, y_node_base] = app.generateGeometry();
            
            coords = [x_c, yu; flipud(x_c), flipud(yl)];
            coords_shifted = [coords(:,1) - 0.5, coords(:,2)];
            R_mat = [cos(-aoa), -sin(-aoa); sin(-aoa), cos(-aoa)];
            coords_rot = (R_mat * coords_shifted')' + [0.5, 0];
            
            app.AirfoilData.x_rot = coords_rot(:,1);
            app.AirfoilData.y_rot = coords_rot(:,2);
            app.AirfoilData.x_c = x_c;
            app.AirfoilData.yu = yu;
            app.AirfoilData.yl = yl; 
            app.AirfoilData.dyu = dyu;
            app.AirfoilData.dyl = dyl;
            app.AirfoilData.x_node_base = x_node_base;
            app.AirfoilData.y_node_base = y_node_base;
            app.AirfoilData.coords_shifted = coords_shifted;
            
            nodes_shifted = [x_node_base - 0.5, y_node_base];
            nodes_rot = (R_mat * nodes_shifted')' + [0.5, 0];
            app.AirfoilData.x_node = nodes_rot(:,1);
            app.AirfoilData.y_node = nodes_rot(:,2);
            
            app.AirfoilData.theta_u = atan(dyu) - aoa;
            app.AirfoilData.theta_l = aoa - atan(dyl);
            
            app.patchAirfoil.XData = app.AirfoilData.x_rot;
            app.patchAirfoil.YData = app.AirfoilData.y_rot;
            app.patchAirfoil.ZData = zeros(size(app.AirfoilData.x_rot));
            
            app.patchAirfoilZoom.XData = app.AirfoilData.x_rot;
            app.patchAirfoilZoom.YData = app.AirfoilData.y_rot;
            app.plotBLEdgeZoom.XData = NaN; app.plotBLEdgeZoom.YData = NaN;
            app.plotBLProfile.XData = NaN;  app.plotBLProfile.YData = NaN;
            app.plotBLNormal.XData = NaN;   app.plotBLNormal.YData = NaN;
            app.plotDPDX.XData = NaN;       app.plotDPDX.YData = NaN;
            app.plotShear.XData = NaN;      app.plotShear.YData = NaN;
            
            app.surfHeatmap.Visible = 'off';
            app.cbMain.Visible = 'off';
            app.scatterParticles.Visible = 'off';
            app.plotSep.XData = NaN;
        end
        
        %% Alpha Sweep Routine (Drag Polar Feature)
        function runAlphaSweep(app)
            if isa(app.Timer, 'timer') && isvalid(app.Timer) && strcmp(app.Timer.Running, 'on')
                stop(app.Timer);
            end
            
            aoa_list = -10:1:15;
            CL_list = zeros(size(aoa_list));
            CD_list = zeros(size(aoa_list));
            
            d = uiprogressdlg(app.UIFigure, 'Title', 'Generating Drag Polar...', 'Indeterminate', 'off');
            old_aoa = app.slAoA.Value;
            
            try
                for i = 1:length(aoa_list)
                    d.Value = i / length(aoa_list);
                    d.Message = sprintf('Solving Viscous-Inviscid Interaction: AoA = %d deg...', aoa_list(i));
                    
                    app.slAoA.Value = aoa_list(i);
                    app.efAoA.Value = aoa_list(i);
                    app.updateGeometryPreview(); 
                    app.updatePhysics(); 
                    
                    CL_list(i) = app.FlowData.CL;
                    CD_list(i) = app.FlowData.CD;
                end
                
                % Restore UI
                app.slAoA.Value = old_aoa;
                app.efAoA.Value = old_aoa;
                app.updateGeometryPreview();
                close(d);
                
                figure('Name', 'Aerodynamic Performance (Drag Polar)', 'Color', 'w', 'Position', [100, 100, 900, 400]);
                subplot(1,2,1);
                plot(aoa_list, CL_list, 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b');
                xlabel('Angle of Attack (deg)', 'FontWeight', 'bold'); ylabel('C_L', 'FontWeight', 'bold'); 
                grid on; title('Lift Curve', 'FontSize', 12);
                
                subplot(1,2,2);
                plot(CD_list, CL_list, 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r');
                xlabel('C_D', 'FontWeight', 'bold'); ylabel('C_L', 'FontWeight', 'bold'); 
                grid on; title('Drag Polar', 'FontSize', 12);
                
            catch ME
                if exist('d', 'var') && isvalid(d), close(d); end
                app.lblStatus.Text = ['Status: Sweep Error - ' ME.message];
            end
        end
        
        function runHighFidelitySimulation(app)
            if isa(app.Timer, 'timer') && isvalid(app.Timer) && strcmp(app.Timer.Running, 'on')
                stop(app.Timer);
            end
            
            d = uiprogressdlg(app.UIFigure, 'Title', 'Running Advanced CFD Simulation', ...
                'Message', 'Initializing atmospheric and geometric properties...');
            
            try
                app.updatePhysics(d); 
                app.precomputeAnimation(d); 
                
                app.AnimationState.Frame = 1;
                app.AnimationState.IsSolved = true;
                
                app.updateVisuals(); 
                
                close(d);
                
                app.btnToggleAnim.Enable = 'on';
                app.btnToggleAnim.Text = 'Pause Animation';
                start(app.Timer);
            catch ME
                if exist('d', 'var') && isvalid(d), close(d); end
                app.lblStatus.Text = ['Status: Error - ' ME.message];
                app.lblStatus.FontColor = 'r';
            end
        end
        
        function toggleAnimation(app)
            if strcmp(app.Timer.Running, 'on')
                stop(app.Timer);
                app.btnToggleAnim.Text = 'Play Animation';
                app.lblStatus.Text = 'Status: Paused';
            else
                if app.AnimationState.Frame >= app.AnimationState.MaxFrames
                    app.AnimationState.Frame = 1; 
                end
                start(app.Timer);
                app.btnToggleAnim.Text = 'Pause Animation';
                app.lblStatus.Text = sprintf('Status: Animating Frame %d/%d', app.AnimationState.Frame, app.AnimationState.MaxFrames);
            end
        end
        
        function initializeParticles(app)
            numParticles = 5000;
            app.Particles.x = -0.5 + rand(numParticles, 1) * 2.0;
            app.Particles.y = -0.6 + rand(numParticles, 1) * 1.2;
        end
        
        %% Core Physics Engine (VII Implementation)
        function updatePhysics(app, d)
            if nargin > 1 && ~isempty(d), d.Value = 0.1; d.Message = 'Establishing Atmospheric Model...'; drawnow; end
            alt = max(0, min(app.efAltitude.Value * 1000, 30000));
            M_inf = max(0.01, app.efMach.Value);
            aoa = app.slAoA.Value * pi / 180;
            
            [T_inf, P_inf, rho_inf, a_inf, mu_inf] = app.standardAtmosphere(alt);
            V_inf = M_inf * a_inf;
            gamma = 1.4; R = 287.05;
            q_inf = 0.5 * gamma * P_inf * M_inf^2;
            
            app.FlowData = struct('rho', rho_inf, 'mu', mu_inf, 'V_inf', V_inf, ...
                'P_inf', P_inf, 'T_inf', T_inf, 'M_inf', M_inf, 'aoa', aoa, 'gamma', gamma);
            
            x_c = app.AirfoilData.x_c;
            yu = app.AirfoilData.yu;
            yl = app.AirfoilData.yl;
            theta_u = app.AirfoilData.theta_u;
            theta_l = app.AirfoilData.theta_l;
            x_node_rot = app.AirfoilData.x_node;
            y_node_rot = app.AirfoilData.y_node;
            x_node_base = app.AirfoilData.x_node_base;
            
            if M_inf >= 1.0  
                if nargin > 1 && ~isempty(d), d.Value = 0.25; d.Message = 'Running Supersonic Shock-Expansion...'; drawnow; end
                app.lblStatus.Text = sprintf('Status: Supersonic (Shock-Expansion), M=%.2f', M_inf);
                [P_u, ~, T_u, V_u] = app.solveSupersonic(theta_u, M_inf, P_inf, T_inf, gamma, R);
                [P_l, ~, T_l, V_l] = app.solveSupersonic(theta_l, M_inf, P_inf, T_inf, gamma, R);
                app.FlowData.CFD_PanelData = [];
                
                R_mat = [cos(-aoa), -sin(-aoa); sin(-aoa), cos(-aoa)];
                coords_u_rot = (R_mat * [x_c - 0.5, yu]')';
                coords_l_rot = (R_mat * [x_c - 0.5, yl]')';
                
                dx_u = diff(coords_u_rot(:,1)); dy_u = diff(coords_u_rot(:,2));
                dx_l = diff(coords_l_rot(:,1)); dy_l = diff(coords_l_rot(:,2));
                
                P_avg_u = (P_u(1:end-1) + P_u(2:end)) / 2;
                P_avg_l = (P_l(1:end-1) + P_l(2:end)) / 2;
                
                Fx_u = sum(P_avg_u .* dy_u);    Fy_u = sum(P_avg_u .* (-dx_u));
                Fx_l = sum(P_avg_l .* (-dy_l)); Fy_l = sum(P_avg_l .* dx_l);
                
                Fx_inf = sum(P_inf .* dy_u) + sum(P_inf .* (-dy_l));
                Fy_inf = sum(P_inf .* (-dx_u)) + sum(P_inf .* dx_l);
                
                CL = (Fy_u + Fy_l - Fy_inf) / q_inf;
                CD_p = (Fx_u + Fx_l - Fx_inf) / q_inf;
                CD_wave = CD_p; 
                
                % BL Integration
                rho_s_upper = P_u ./ (R .* T_u); rho_s_lower = P_l ./ (R .* T_l);
                [tau_w_u, delta_u, dpdx_u, sep_x_u, ~] = app.solveBoundaryLayer(x_c, V_u, rho_s_upper, mu_inf, P_u, app.efReCrit.Value, app.efTripX.Value);
                [tau_w_l, ~, ~, sep_x_l, ~] = app.solveBoundaryLayer(x_c, V_l, rho_s_lower, mu_inf, P_l, app.efReCrit.Value, app.efTripX.Value);
                
            else             
                % Viscous-Inviscid Interaction (VII) Loop & Transonic Capture
                if nargin > 1 && ~isempty(d), d.Value = 0.2; d.Message = 'VII Pass 1: Inviscid Potential Field...'; drawnow; end
                [P_u, V_u, P_l, V_l, ~, ~, ~, ~] = app.solveSubsonicCFD(x_node_rot, y_node_rot, x_node_base, M_inf, P_inf, gamma, x_c, []);
                
                T_u = T_inf .* (P_u ./ P_inf).^((gamma-1)/gamma);
                T_l = T_inf .* (P_l ./ P_inf).^((gamma-1)/gamma);
                rho_s_upper = P_u ./ (R .* T_u); rho_s_lower = P_l ./ (R .* T_l);
                
                if nargin > 1 && ~isempty(d), d.Value = 0.3; d.Message = 'VII Pass 1: Viscous Boundary Layer...'; drawnow; end
                [tau_w_u, delta_u, dpdx_u, sep_x_u, dstar_u] = app.solveBoundaryLayer(x_c, V_u, rho_s_upper, mu_inf, P_u, app.efReCrit.Value, app.efTripX.Value);
                [tau_w_l, ~, ~, sep_x_l, dstar_l] = app.solveBoundaryLayer(x_c, V_l, rho_s_lower, mu_inf, P_l, app.efReCrit.Value, app.efTripX.Value);
                
                % Compute Transpiration Blowing Velocity V_n = d(U_e * delta*)/ds
                if nargin > 1 && ~isempty(d), d.Value = 0.4; d.Message = 'VII Pass 2: Coupled Transpiration Matrix...'; drawnow; end
                flux_u = V_u .* dstar_u; flux_l = V_l .* dstar_l;
                Vn_u = gradient(flux_u, x_c); Vn_l = gradient(flux_l, x_c);
                
                % Map to panels
                N = length(x_node_rot) - 1;
                xc_base = (x_node_base(1:N) + x_node_base(2:N+1)) / 2;
                [~, LE_idx] = min(xc_base);
                V_trans = zeros(N, 1);
                
                % Lower surface mapping (panels 1 to LE)
                % FIX: Use `ic` index output from `unique` to safely broadcast unique mapping back to full original array
                x_l_cfd = xc_base(1:LE_idx);
                [x_l_uniq, ~, ic_l] = unique(flipud(x_l_cfd), 'stable');
                Vn_l_mapped_uniq = interp1(x_c, Vn_l, x_l_uniq, 'linear', 'extrap');
                V_trans(1:LE_idx) = flipud(Vn_l_mapped_uniq(ic_l));
                
                % Upper surface mapping (panels LE to end)
                x_u_cfd = xc_base(LE_idx:end);
                [x_u_uniq, ~, ic_u] = unique(x_u_cfd, 'stable');
                Vn_u_mapped_uniq = interp1(x_c, Vn_u, x_u_uniq, 'linear', 'extrap');
                V_trans(LE_idx:end) = Vn_u_mapped_uniq(ic_u);
                
                % Final Pass with Transonic Check
                [P_u, V_u, P_l, V_l, panel_q, panel_gamma, CL, CD_p, CD_wave] = app.solveSubsonicCFD(x_node_rot, y_node_rot, x_node_base, M_inf, P_inf, gamma, x_c, V_trans);
                app.FlowData.CFD_PanelData.q = panel_q;
                app.FlowData.CFD_PanelData.gamma = panel_gamma;
                
                if CD_wave > 0.001
                     app.lblStatus.Text = sprintf('Status: Transonic Flow Captured (CD_w = %.4f)', CD_wave);
                else
                     app.lblStatus.Text = sprintf('Status: Subsonic Fully Coupled VII, M=%.2f', M_inf);
                end
            end
            
            app.FlowData.V_s_upper = V_u;
            app.FlowData.V_s_lower = V_l;
            app.FlowData.P_s_upper = P_u;
            app.FlowData.tau_w = tau_w_u;
            app.FlowData.dpdx = dpdx_u;
            app.FlowData.delta = delta_u;
            app.FlowData.separation_x = sep_x_u;
            app.FlowData.separation_x_lower = sep_x_l;
            
            CD_f = (trapz(x_c, tau_w_u) + trapz(x_c, tau_w_l)) / q_inf;
            CD_total = CD_p + CD_f + CD_wave;
            LD_ratio = CL / max(CD_total, 1e-6);
            
            app.FlowData.CL = CL;
            app.FlowData.CD = CD_total;
            
            app.lblCL.Text = sprintf('C_L (Lift): %.3f', CL);
            app.lblCD.Text = sprintf('C_D (Drag): %.4f', CD_total);
            app.lblLD.Text = sprintf('L/D (Effic): %.1f', LD_ratio);
            
            if nargin > 1 && ~isempty(d), d.Value = 0.5; d.Message = 'Precomputing UPM Field Unit Matrices...'; drawnow; end
            app.precomputeBaseField(M_inf, aoa, x_c, V_u, P_u, V_l, P_l, x_node_rot, y_node_rot);
        end
        
        %% UPM Fast Field Reconstruction & Animation
        function precomputeAnimation(app, d)
            MaxFrames = app.AnimationState.MaxFrames;
            dt = 0.05;
            M_inf = app.FlowData.M_inf;
            V_inf = app.FlowData.V_inf;
            
            app.FlowData.Frames.V_field = zeros(150, 150, MaxFrames);
            app.FlowData.Frames.P_field = zeros(150, 150, MaxFrames);
            numParticles = 5000;
            app.FlowData.Frames.ParticlesX = zeros(numParticles, MaxFrames);
            app.FlowData.Frames.ParticlesY = zeros(numParticles, MaxFrames);
            
            app.FlowData.WakeVortices = struct('x', [], 'y', [], 'Gamma', []);
            app.initializeParticles();
            px = app.Particles.x;
            py = app.Particles.y;
            
            GridX = app.FlowData.GridX; GridY = app.FlowData.GridY;
            y_vec = linspace(-0.6, 0.6, 150); x_vec = linspace(-0.5, 1.5, 150);
            
            % Interpolants for particle advection
            U_f = app.FlowData.U_base_field; 
            V_f = app.FlowData.V_base_field;
            
            F_U = griddedInterpolant({y_vec, x_vec}, U_f, 'linear', 'nearest');
            F_V = griddedInterpolant({y_vec, x_vec}, V_f, 'linear', 'nearest');
            
            scale_factor = 1.0 / max(app.FlowData.V_inf, 0.01);
            dt_sub = dt / 3;
            
            curr_x_rot = app.AirfoilData.x_rot; curr_y_rot = app.AirfoilData.y_rot;
            
            if M_inf < 1
                U_inf_mat = app.FlowData.CFD_PanelData.U_inf_mat;
                V_inf_mat = app.FlowData.CFD_PanelData.V_inf_mat;
                L_mat = app.FlowData.CFD_PanelData.L_mat;
                U_mat = app.FlowData.CFD_PanelData.U_mat;
                P_mat = app.FlowData.CFD_PanelData.P_mat;
                b_base = app.FlowData.CFD_PanelData.b_base;
                xc = app.FlowData.CFD_PanelData.xc;
                yc = app.FlowData.CFD_PanelData.yc;
                nx = app.FlowData.CFD_PanelData.nx;
                ny = app.FlowData.CFD_PanelData.ny;
                N = length(xc);
            end
            
            for f = 1:MaxFrames
                if nargin > 1 && ~isempty(d) && mod(f, 15) == 0
                    d.Value = 0.5 + 0.5 * (f / MaxFrames);
                    d.Message = sprintf('True Unsteady Panel Tracking: Frame %d of %d...', f, MaxFrames);
                end
                
                if M_inf < 1 && ~app.cbPitching.Value
                    % True UPM (Unsteady Panel Method) Execution
                    % 1. Release continuous vortex sheet from separation points
                    if app.FlowData.separation_x > 0
                        idx_sep = find(app.AirfoilData.x_c >= app.FlowData.separation_x, 1);
                        if ~isempty(idx_sep)
                            app.FlowData.WakeVortices.x(end+1) = curr_x_rot(idx_sep) + 0.01;
                            app.FlowData.WakeVortices.y(end+1) = curr_y_rot(idx_sep) + 0.01;
                            U_edge = app.FlowData.V_s_upper(idx_sep);
                            app.FlowData.WakeVortices.Gamma(end+1) = -0.5 * U_edge^2 * dt;
                        end
                        app.FlowData.WakeVortices.x(end+1) = curr_x_rot(end) + 0.01;
                        app.FlowData.WakeVortices.y(end+1) = curr_y_rot(end) - 0.01;
                        U_te = app.FlowData.V_s_lower(end);
                        app.FlowData.WakeVortices.Gamma(end+1) = 0.5 * U_te^2 * dt;
                    end
                    
                    % 2. Calculate Wake Induced Velocity on the Airfoil Control Points
                    u_ind_panel = zeros(N, 1); v_ind_panel = zeros(N, 1);
                    U_wake_field = zeros(150*150, 1); V_wake_field = zeros(150*150, 1);
                    v_remove = false(size(app.FlowData.WakeVortices.x));
                    
                    for k = 1:length(app.FlowData.WakeVortices.x)
                        vx = app.FlowData.WakeVortices.x(k);
                        vy = app.FlowData.WakeVortices.y(k);
                        gam = app.FlowData.WakeVortices.Gamma(k);
                        
                        if vx > 2.0, v_remove(k) = true; continue; end
                        app.FlowData.WakeVortices.x(k) = vx + 0.8 * V_inf * dt;
                        
                        % Induced on panels (for Matrix B update)
                        dx_p = xc - vx; dy_p = yc - vy;
                        r2_p = max(dx_p.^2 + dy_p.^2, 1e-4);
                        u_ind_panel = u_ind_panel - (gam/(2*pi)) * dy_p ./ r2_p;
                        v_ind_panel = v_ind_panel + (gam/(2*pi)) * dx_p ./ r2_p;
                        
                        % Induced on global grid
                        dx_f = GridX(:) - vx; dy_f = GridY(:) - vy;
                        r2_f = max(dx_f.^2 + dy_f.^2, 1e-4);
                        core = 1 - exp(-r2_f / (0.15^2)); % Large diffusion for smooth pressure
                        U_wake_field = U_wake_field - (gam/(2*pi)) * dy_f ./ r2_f .* core;
                        V_wake_field = V_wake_field + (gam/(2*pi)) * dx_f ./ r2_f .* core;
                    end
                    
                    app.FlowData.WakeVortices.x(v_remove) = [];
                    app.FlowData.WakeVortices.y(v_remove) = [];
                    app.FlowData.WakeVortices.Gamma(v_remove) = [];
                    
                    % 3. Kelvin's Theorem Matrix Re-solve
                    b_upm = b_base;
                    b_upm(1:N) = b_upm(1:N) - (u_ind_panel .* nx + v_ind_panel .* ny);
                    sol_upm = U_mat \ (L_mat \ (P_mat * b_upm));
                    
                    % 4. Fast Field Reconstruction
                    U_f_vec = V_inf + U_inf_mat * sol_upm + U_wake_field;
                    V_f_vec = V_inf_mat * sol_upm + V_wake_field;
                    
                    U_f = reshape(U_f_vec, 150, 150);
                    V_f = reshape(V_f_vec, 150, 150);
                    
                    V_field = sqrt(U_f.^2 + V_f.^2);
                    Cp_f = 1 - (V_field ./ V_inf).^2;
                    beta = sqrt(max(1e-6, 1 - M_inf^2));
                    Cp_f = Cp_f ./ (beta + (M_inf^2 / (1 + beta)) .* (Cp_f / 2));
                    P_field = app.FlowData.P_inf .* (1 + (1.4/2) * M_inf^2 .* Cp_f);
                    
                    P_field = max(real(P_field), 0.01);
                    F_U.Values = U_f; F_V.Values = V_f;
                    
                elseif app.cbPitching.Value && M_inf < 1
                    % Dynamic Pitching (Geometry Rebuild necessary, cancels fast UPM)
                    aoa_t = app.FlowData.aoa + 5 * (pi/180) * sin(2*pi * f / MaxFrames);
                    R_mat = [cos(-aoa_t), -sin(-aoa_t); sin(-aoa_t), cos(-aoa_t)];
                    nodes_shifted = [app.AirfoilData.x_node_base - 0.5, app.AirfoilData.y_node_base];
                    nodes_rot = (R_mat * nodes_shifted')' + [0.5, 0];
                    curr_x_rot = nodes_rot(:,1); curr_y_rot = nodes_rot(:,2);
                    
                    [~, ~, ~, ~, q_t, gamma_t, ~, ~, ~] = app.solveSubsonicCFD(curr_x_rot, curr_y_rot, app.AirfoilData.x_node_base, M_inf, app.FlowData.P_inf, app.FlowData.gamma, app.AirfoilData.x_c, []);
                    
                    U_f = V_inf * ones(size(GridX)); V_f = zeros(size(GridX));
                    for j = 1:length(curr_x_rot)-1
                        [u_s, v_s] = app.panelVelocity(GridX, GridY, curr_x_rot(j), curr_y_rot(j), curr_x_rot(j+1), curr_y_rot(j+1), q_t(j), 0);
                        [u_v, v_v] = app.panelVelocity(GridX, GridY, curr_x_rot(j), curr_y_rot(j), curr_x_rot(j+1), curr_y_rot(j+1), 0, gamma_t);
                        U_f = U_f + u_s + u_v; V_f = V_f + v_s + v_v;
                    end
                    V_field = sqrt(U_f.^2 + V_f.^2);
                    P_field = app.FlowData.P_inf .* (1 + (1.4/2) * M_inf^2 .* (1 - (V_field./V_inf).^2));
                    F_U.Values = U_f; F_V.Values = V_f;
                else
                    % Supersonic 
                    V_field = U_f; P_field = app.FlowData.P_static_supersonic;
                end
                
                in_airfoil = inpolygon(GridX, GridY, curr_x_rot, curr_y_rot);
                V_field(in_airfoil) = 0; P_field(in_airfoil) = app.FlowData.P_inf;
                app.FlowData.Frames.V_field(:,:,f) = V_field;
                app.FlowData.Frames.P_field(:,:,f) = P_field;
                
                % RK2 Particle Advection
                for sub = 1:3
                    py_clamp = max(-0.6, min(0.6, py)); px_clamp = max(-0.5, min(1.5, px));
                    vx_1 = F_U([py_clamp, px_clamp]); vy_1 = F_V([py_clamp, px_clamp]);
                    
                    px_mid = px + (vx_1 * scale_factor) * (dt_sub/2); py_mid = py + (vy_1 * scale_factor) * (dt_sub/2);
                    py_mid_clamp = max(-0.6, min(0.6, py_mid)); px_mid_clamp = max(-0.5, min(1.5, px_mid));
                    
                    vx_2 = F_U([py_mid_clamp, px_mid_clamp]); vy_2 = F_V([py_mid_clamp, px_mid_clamp]);
                    px = px + (vx_2 * scale_factor) * dt_sub; py = py + (vy_2 * scale_factor) * dt_sub;
                end
                
                px = px + 0.002 * randn(length(px), 1); py = py + 0.002 * randn(length(py), 1);
                out_of_bounds = px > 1.5 | py > 0.6 | py < -0.6 | isnan(px) | inpolygon(px, py, curr_x_rot, curr_y_rot);
                
                if any(out_of_bounds)
                    px(out_of_bounds) = -0.5; py(out_of_bounds) = -0.6 + rand(sum(out_of_bounds), 1) * 1.2;
                end
                app.FlowData.Frames.ParticlesX(:,f) = px; app.FlowData.Frames.ParticlesY(:,f) = py;
            end
        end
        
        %% Transonic-Capable Hess-Smith CFD
        function [P_u, V_u, P_l, V_l, q, gamma_circ, CL, CD_p, CD_wave] = solveSubsonicCFD(app, x_node, y_node, x_node_base, M_inf, P_inf, gamma_air, x_c_target, V_trans)
            N = length(x_node) - 1;
            V_inf = app.FlowData.V_inf;
            if nargin < 9 || isempty(V_trans), V_trans = zeros(N,1); end
            
            xc = (x_node(1:N) + x_node(2:N+1)) / 2; yc = (y_node(1:N) + y_node(2:N+1)) / 2;
            dx = x_node(2:N+1) - x_node(1:N); dy = y_node(2:N+1) - y_node(1:N);
            S = sqrt(dx.^2 + dy.^2); theta = atan2(dy, dx);
            
            nx = -sin(theta); ny = cos(theta); tx = cos(theta);  ty = sin(theta);
            A = zeros(N+1, N+1); b = zeros(N+1, 1); At = zeros(N, N+1); 
            
            for j = 1:N
                [us, vs] = app.panelVelocity(xc, yc, x_node(j), y_node(j), x_node(j+1), y_node(j+1), 1, 0);
                [uv, vv] = app.panelVelocity(xc, yc, x_node(j), y_node(j), x_node(j+1), y_node(j+1), 0, 1);
                
                A(1:N, j) = us .* nx + vs .* ny;
                A(1:N, N+1) = A(1:N, N+1) + (uv .* nx + vv .* ny);
                At(1:N, j) = us .* tx + vs .* ty;
                At(1:N, N+1) = At(1:N, N+1) + (uv .* tx + vv .* ty);
            end
            
            for i = 1:N, b(i) = -V_inf * nx(i) + V_trans(i); end
            A(N+1, 1:N) = At(1, 1:N) + At(N, 1:N);
            A(N+1, N+1) = At(1, N+1) + At(N, N+1);
            b(N+1) = -V_inf * (tx(1) + tx(N));
            
            [L, U, P_mat] = lu(A);
            app.FlowData.CFD_PanelData.L_mat = L; app.FlowData.CFD_PanelData.U_mat = U;
            app.FlowData.CFD_PanelData.P_mat = P_mat; app.FlowData.CFD_PanelData.b_base = b;
            app.FlowData.CFD_PanelData.xc = xc; app.FlowData.CFD_PanelData.yc = yc;
            app.FlowData.CFD_PanelData.nx = nx; app.FlowData.CFD_PanelData.ny = ny;
            
            sol = U \ (L \ (P_mat * b));
            q = sol(1:N); gamma_circ = sol(N+1);
            
            V_t = zeros(N, 1);
            for i = 1:N, V_t(i) = At(i, 1:N) * q + At(i, N+1) * gamma_circ + V_inf * tx(i); end
            Cp_inc = 1 - (V_t / V_inf).^2;
            
            % Compressibility & Transonic Shock Capturing
            beta = sqrt(max(1e-6, 1 - M_inf^2));
            Cp_comp = Cp_inc ./ (beta + (M_inf^2 / (1 + beta)) .* (Cp_inc / 2));
            
            Cp_crit = (2/(gamma_air*M_inf^2)) * ( ((2+(gamma_air-1)*M_inf^2)/(gamma_air+1))^(gamma_air/(gamma_air-1)) - 1 );
            CD_wave = 0;
            
            % Identify Supersonic Pockets and Wave Drag via Rankine-Hugoniot
            if min(Cp_comp) < Cp_crit
                M_local = zeros(N,1);
                for i=1:N
                    if Cp_comp(i) < Cp_crit
                        ratio = max(1e-6, 1 - Cp_comp(i)*(gamma_air/2)*M_inf^2);
                        M_local(i) = sqrt((2/(gamma_air-1)) * ((1+0.5*(gamma_air-1)*M_inf^2) * ratio^(-(gamma_air-1)/gamma_air) - 1));
                    else
                        M_local(i) = sqrt(max(0, (2/(gamma_air-1)) * ((1+0.5*(gamma_air-1)*M_inf^2) / (1 + (gamma_air/2)*M_inf^2*Cp_comp(i))^((gamma_air-1)/gamma_air) - 1)));
                    end
                end
                
                % Crude shock detection: Sudden drop in supersonic Mach
                for i=2:N
                    if M_local(i-1) > 1.05 && M_local(i) < M_local(i-1) - 0.1
                        M1 = M_local(i-1);
                        % Normal shock pressure ratio
                        P2_P1 = (2*gamma_air*M1^2 - (gamma_air-1)) / (gamma_air+1);
                        shock_dp = (P2_P1 - 1) * (1 + (gamma_air/2)*M_inf^2*Cp_comp(i-1));
                        CD_wave = CD_wave + shock_dp * abs(nx(i)) * S(i);
                        Cp_comp(i:end) = max(Cp_comp(i:end), Cp_crit); % Cap downstream
                    end
                end
            end
            
            CL = sum(-Cp_comp .* ny .* S);
            CD_p = sum(-Cp_comp .* nx .* S);
            
            xc_base = (x_node_base(1:N) + x_node_base(2:N+1)) / 2; [~, LE_idx] = min(xc_base);
            
            x_u_CFD = xc_base(LE_idx:end); V_u_CFD = abs(V_t(LE_idx:end)); Cp_u_CFD = Cp_comp(LE_idx:end);
            [x_u_unique, uni_idx_u] = unique(x_u_CFD, 'stable');
            V_u = interp1(x_u_unique, V_u_CFD(uni_idx_u), x_c_target, 'linear', 'extrap');
            Cp_u = interp1(x_u_unique, Cp_u_CFD(uni_idx_u), x_c_target, 'linear', 'extrap');
            
            x_l_CFD = xc_base(1:LE_idx); V_l_CFD = flipud(abs(V_t(1:LE_idx))); Cp_l_CFD = flipud(Cp_comp(1:LE_idx));
            [x_l_unique, uni_idx_l] = unique(flipud(x_l_CFD), 'stable');
            V_l = interp1(x_l_unique, V_l_CFD(uni_idx_l), x_c_target, 'linear', 'extrap');
            Cp_l = interp1(x_l_unique, Cp_l_CFD(uni_idx_l), x_c_target, 'linear', 'extrap');
            
            P_u = max(real(P_inf .* (1 + (gamma_air/2) * M_inf^2 .* Cp_u)), 10);
            P_l = max(real(P_inf .* (1 + (gamma_air/2) * M_inf^2 .* Cp_l)), 10);
        end
        
        function [u, v] = panelVelocity(~, x, y, x1, y1, x2, y2, q, gamma)
            dx = x2 - x1; dy = y2 - y1; S = sqrt(dx.^2 + dy.^2); theta = atan2(dy, dx);
            xt = x - x1; yt = y - y1;
            xl = xt .* cos(theta) + yt .* sin(theta); yl = -xt .* sin(theta) + yt .* cos(theta);
            
            tol = 1e-6; on_panel = (abs(yl) < tol) & (xl >= 0) & (xl <= S);
            yl(abs(yl) < tol) = sign(yl(abs(yl) < tol)) .* tol; yl(yl == 0) = tol; 
            
            r1 = max(xl.^2 + yl.^2, 1e-12); r2 = max((xl - S).^2 + yl.^2, 1e-12);
            dtheta = atan2(yl, xl - S) - atan2(yl, xl);
            
            ul = (q/(4*pi))*log(r1./r2) - (gamma/(2*pi))*dtheta;
            vl = (q/(2*pi))*dtheta + (gamma/(4*pi))*log(r2./r1);
            
            ul(on_panel) = -gamma/2; vl(on_panel) = q/2;
            u = ul .* cos(theta) - vl .* sin(theta); v = ul .* sin(theta) + vl .* cos(theta);
        end
        
        %% Boundary Layer Solver (Stratford turbulent separation & Displacement Thickness)
        function [tau_w, delta, dpdx, separation_x, dstar] = solveBoundaryLayer(app, x, V, rho, mu, P, re_crit, trip_x)
            N = length(x);
            delta = zeros(N, 1); tau_w = zeros(N, 1); dstar = zeros(N,1);
            
            V_smooth = smoothdata(V, 'sgolay', 10);
            dpdx_raw = gradient(P, x); dpdx = smoothdata(dpdx_raw, 'sgolay', 10);
            
            sep_idx = -1; theta_sq = 0; nu = mu ./ rho;
            V_max = max(V_smooth); idx_vmax = find(V_smooth == V_max, 1);
            
            for i = 1:N
                if i == 1
                    dUdx = (V_smooth(2) - V_smooth(1)) / (x(2) - x(1));
                    theta_sq = max(0, 0.075 * nu(1) / dUdx);
                    continue;
                end
                
                dx = x(i) - x(i-1); U = max(V_smooth(i), 0.05 * app.FlowData.V_inf);
                theta_sq = theta_sq + (0.45 * nu(i) / U^6) * (U^5 * dx);
                theta = sqrt(abs(theta_sq));
                
                lambda = (theta^2 / nu(i)) * ((V_smooth(i) - V_smooth(i-1))/dx);
                lambda = max(-0.09, min(0.1, lambda)); 
                Re_x = rho(i) * U * x(i) / mu;
                
                if Re_x < re_crit && x(i) < trip_x && sep_idx == -1
                    % Laminar (Thwaites)
                    l_lambda = 0.22 + 1.57*lambda - 1.8*lambda^2;
                    tau_w(i) = mu * U / max(theta, 1e-6) * l_lambda;
                    delta(i) = theta * 7.5; 
                    H = 2.61 - 3.75*lambda + 5.24*lambda^2;
                    dstar(i) = theta * H;
                    if lambda <= -0.09, sep_idx = i; end
                else
                    % Turbulent (Stratford Separation Criteria)
                    Cf = 0.0592 / max(Re_x^0.2, 1);
                    tau_w(i) = 0.5 * rho(i) * U^2 * Cf;
                    delta(i) = 0.37 * x(i) / max(Re_x^0.2, 1);
                    dstar(i) = theta * 1.4; % Flat plate turbulent H approx 1.4
                    
                    if i > idx_vmax && sep_idx == -1
                        Cp_prime = 1 - (U/V_max)^2;
                        x_prime = x(i) - x(idx_vmax);
                        if x_prime > 0
                            dCp_dx = (Cp_prime - (1 - (V_smooth(i-1)/V_max)^2)) / dx;
                            Stratford = Cp_prime * sqrt(x_prime * max(0, dCp_dx)) * (1e-6 * Re_x)^-0.1;
                            if Stratford >= 0.39, sep_idx = i; end
                        end
                    end
                end
                
                if sep_idx ~= -1 && i > sep_idx
                    tau_w(i) = 0;
                    % Physically scaled post-separation gap
                    delta(i) = delta(i-1) + dx * 0.05 * (delta(i-1)/x(i)); 
                    dstar(i) = delta(i) / 3;
                end
            end
            if sep_idx > 0, separation_x = x(sep_idx); else, separation_x = -1; end
        end
        
        %% Supersonic Solver
        function [P, M, T, V] = solveSupersonic(app, theta, M_inf, P_inf, T_inf, gamma, R)
            N = length(theta);
            M = zeros(N, 1); P = zeros(N, 1); T = zeros(N, 1); V = zeros(N, 1);
            
            theta_le = theta(1);
            theta_max = 0.8 * app.getThetaMax(M_inf, gamma); 
            
            if theta_le > theta_max || M_inf < 1.05
                Cp_max = (2/(gamma*M_inf^2)) * ( (((gamma+1)^2 * M_inf^2)/(4*gamma*M_inf^2 - 2*(gamma-1)))^(gamma/(gamma-1)) * ((1-gamma+2*gamma*M_inf^2)/(gamma+1)) - 1 );
                Cp = Cp_max * sin(max(0, theta_le))^2;
                P(1) = P_inf * (1 + (gamma/2)*M_inf^2 * Cp);
                T(1) = T_inf * (P(1)/P_inf)^((gamma-1)/gamma);
                M(1) = max(1.01, sqrt(max(0, (2/(gamma-1)) * ((1 + (gamma-1)/2 * M_inf^2) * (P_inf/P(1))^((gamma-1)/gamma) - 1))));
            elseif theta_le > 0
                beta = app.solveObliqueShock(M_inf, theta_le, gamma);
                Mn1 = M_inf * sin(beta);
                Mn2 = sqrt((1 + 0.5*(gamma-1)*Mn1^2) / (gamma*Mn1^2 - 0.5*(gamma-1)));
                M(1) = max(1.01, Mn2 / sin(beta - theta_le));
                P(1) = P_inf * (1 + (2*gamma/(gamma+1))*(Mn1^2 - 1));
                T(1) = T_inf * (P(1)/P_inf) * ((2 + (gamma-1)*Mn1^2)/((gamma+1)*Mn1^2));
            else
                nu1 = app.prandtlMeyer(M_inf, gamma);
                nu2 = nu1 - theta_le;
                M(1) = app.invPrandtlMeyer(nu2, M_inf, gamma);
                P(1) = P_inf * ((1 + 0.5*(gamma-1)*M_inf^2) / (1 + 0.5*(gamma-1)*M(1)^2))^(gamma/(gamma-1));
                T(1) = T_inf * (1 + 0.5*(gamma-1)*M_inf^2) / (1 + 0.5*(gamma-1)*M(1)^2);
            end
            
            V(1) = M(1) * sqrt(max(0, gamma * R * T(1)));
            
            for i = 2:N
                dTheta = theta(i) - theta(i-1);
                if abs(dTheta) < 1e-4
                    M(i) = M(i-1); P(i) = P(i-1); T(i) = T(i-1);
                elseif dTheta > 0
                    beta = app.solveObliqueShock(M(i-1), dTheta, gamma);
                    Mn1 = M(i-1) * sin(beta);
                    Mn2 = sqrt((1 + 0.5*(gamma-1)*Mn1^2) / (gamma*Mn1^2 - 0.5*(gamma-1)));
                    M(i) = max(1.01, Mn2 / sin(beta - dTheta));
                    pr = (1 + (2*gamma/(gamma+1))*(Mn1^2 - 1));
                    P(i) = P(i-1) * pr;
                    T(i) = T(i-1) * pr * ((2 + (gamma-1)*Mn1^2)/((gamma+1)*Mn1^2));
                else 
                    nu1 = app.prandtlMeyer(M(i-1), gamma);
                    nu2 = nu1 - dTheta; 
                    M(i) = app.invPrandtlMeyer(nu2, M(i-1), gamma);
                    tr = (1 + 0.5*(gamma-1)*M(i-1)^2) / (1 + 0.5*(gamma-1)*M(i)^2);
                    T(i) = T(i-1) * tr;
                    P(i) = P(i-1) * tr^(gamma/(gamma-1));
                end
                
                if ~isreal(M(i)) || M(i) < 0.1, M(i) = 0.1; end
                V(i) = M(i) * sqrt(max(0, gamma * R * T(i)));
            end
        end
        
        %% Unit Matrix Influence Generation
        function precomputeBaseField(app, M_inf, aoa, x_c, V_u, P_u, V_l, P_l, x_node, y_node)
            [X, Y] = meshgrid(linspace(-0.5, 1.5, 150), linspace(-0.6, 0.6, 150));
            V_inf = app.FlowData.V_inf;
            
            if M_inf < 1 
                N = length(x_node) - 1;
                num_pts = numel(X);
                
                % Vectorized Unit Influence Matrices (54MB memory footprint)
                U_inf_mat = zeros(num_pts, N+1); V_inf_mat = zeros(num_pts, N+1);
                uv_tot = zeros(num_pts,1); vv_tot = zeros(num_pts,1);
                
                for j = 1:N
                    [us, vs] = app.panelVelocity(X(:), Y(:), x_node(j), y_node(j), x_node(j+1), y_node(j+1), 1, 0);
                    U_inf_mat(:,j) = us; V_inf_mat(:,j) = vs;
                    
                    [uv, vv] = app.panelVelocity(X(:), Y(:), x_node(j), y_node(j), x_node(j+1), y_node(j+1), 0, 1);
                    uv_tot = uv_tot + uv; vv_tot = vv_tot + vv;
                end
                U_inf_mat(:, N+1) = uv_tot; V_inf_mat(:, N+1) = vv_tot;
                
                app.FlowData.CFD_PanelData.U_inf_mat = U_inf_mat;
                app.FlowData.CFD_PanelData.V_inf_mat = V_inf_mat;
                
                sol_base = [app.FlowData.CFD_PanelData.q; app.FlowData.CFD_PanelData.gamma];
                app.FlowData.U_base_field = reshape(V_inf + U_inf_mat * sol_base, 150, 150);
                app.FlowData.V_base_field = reshape(V_inf_mat * sol_base, 150, 150);
            else 
                dx = X - 0.5; dy = Y;
                X_rot = dx*cos(aoa) - dy*sin(aoa) + 0.5; Y_rot = dx*sin(aoa) + dy*cos(aoa);
                idx_over = Y_rot >= 0; idx_under = Y_rot < 0;
                
                V_field = V_inf * ones(size(X)); P_field = app.FlowData.P_inf * ones(size(X));
                mu_ang = asin(1/max(1.05, M_inf)); 
                
                shift_x_u = Y_rot(idx_over) / tan(mu_ang); src_x_u = X_rot(idx_over) - shift_x_u;
                valid_u = src_x_u >= 0 & src_x_u <= 1; active_idx_u = find(idx_over);
                V_surf_u = interp1(x_c, V_u, src_x_u(valid_u), 'nearest', V_inf);
                P_surf_u = interp1(x_c, P_u, src_x_u(valid_u), 'nearest', app.FlowData.P_inf);
                decay_u = exp(-4 * Y_rot(active_idx_u(valid_u))); 
                V_field(active_idx_u(valid_u)) = V_inf + (V_surf_u - V_inf) .* decay_u;
                P_field(active_idx_u(valid_u)) = app.FlowData.P_inf + (P_surf_u - app.FlowData.P_inf) .* decay_u;
                
                shift_x_l = abs(Y_rot(idx_under)) / tan(mu_ang); src_x_l = X_rot(idx_under) - shift_x_l;
                valid_l = src_x_l >= 0 & src_x_l <= 1; active_idx_l = find(idx_under);
                V_surf_l = interp1(x_c, V_l, src_x_l(valid_l), 'nearest', V_inf);
                P_surf_l = interp1(x_c, P_l, src_x_l(valid_l), 'nearest', app.FlowData.P_inf);
                decay_l = exp(-4 * abs(Y_rot(active_idx_l(valid_l))));
                V_field(active_idx_l(valid_l)) = V_inf + (V_surf_l - V_inf) .* decay_l;
                P_field(active_idx_l(valid_l)) = app.FlowData.P_inf + (P_surf_l - app.FlowData.P_inf) .* decay_l;
                
                app.FlowData.U_base_field = V_field; 
                app.FlowData.V_base_field = zeros(size(X));
                app.FlowData.P_static_supersonic = P_field;
            end
            
            app.FlowData.GridX = X; app.FlowData.GridY = Y;
        end
        
        function animateFrame(app)
            app.AnimationState.Frame = app.AnimationState.Frame + 1;
            if app.AnimationState.Frame > app.AnimationState.MaxFrames
                stop(app.Timer); app.lblStatus.Text = 'Status: Simulation Complete.';
                app.btnToggleAnim.Text = 'Replay Animation'; return;
            end
            f = app.AnimationState.Frame;
            app.lblStatus.Text = sprintf('Status: Animating Frame %d/%d (15s Simulation)', f, app.AnimationState.MaxFrames);
            
            if app.cbVelocity.Value
                app.surfHeatmap.CData = app.FlowData.Frames.V_field(:,:,f);
            elseif app.cbPressure.Value
                app.surfHeatmap.CData = app.FlowData.Frames.P_field(:,:,f);
            end
            
            if app.cbPitching.Value && app.FlowData.M_inf < 1.0
                aoa_t = app.FlowData.aoa + 5 * (pi/180) * sin(2*pi * f / app.AnimationState.MaxFrames);
                R_mat = [cos(-aoa_t), -sin(-aoa_t); sin(-aoa_t), cos(-aoa_t)];
                coords_rot = (R_mat * app.AirfoilData.coords_shifted')' + [0.5, 0];
                app.patchAirfoil.XData = coords_rot(:,1); app.patchAirfoil.YData = coords_rot(:,2);
            end
            
            if app.cbParticles.Value
                app.scatterParticles.XData = app.FlowData.Frames.ParticlesX(:,f);
                app.scatterParticles.YData = app.FlowData.Frames.ParticlesY(:,f);
            end
        end
        
        function updateVisuals(app)
            if ~app.AnimationState.IsSolved || isempty(app.AirfoilData) || isempty(app.FlowData) || ~isfield(app.FlowData, 'Frames')
                return;
            end
            f = max(1, min(app.AnimationState.MaxFrames, app.AnimationState.Frame));
            
            if app.cbVelocity.Value
                app.surfHeatmap.CData = app.FlowData.Frames.V_field(:,:,f);
                title(app.axMain, sprintf('Velocity Heatmap (M_{inf} = %.2f, V_{inf} = %.0f m/s)', app.FlowData.M_inf, real(app.FlowData.V_inf)), 'Color', 'w');
                app.surfHeatmap.Visible = 'on'; app.cbMain.Visible = 'on';
                app.cbMain.Label.String = 'Velocity (m/s) [Blue=Low, Red=High]';
            elseif app.cbPressure.Value
                app.surfHeatmap.CData = app.FlowData.Frames.P_field(:,:,f);
                title(app.axMain, sprintf('Pressure Heatmap (Pa) (Freestream = %.0f Pa)', real(app.FlowData.P_inf)), 'Color', 'w');
                app.surfHeatmap.Visible = 'on'; app.cbMain.Visible = 'on';
                app.cbMain.Label.String = 'Absolute Pressure (Pa) [Blue=Low, Red=High]';
            else
                app.surfHeatmap.Visible = 'off'; app.cbMain.Visible = 'off';
                title(app.axMain, 'Flow Field', 'Color', 'w');
            end
            
            if app.cbParticles.Value
                app.scatterParticles.XData = app.FlowData.Frames.ParticlesX(:,f);
                app.scatterParticles.YData = app.FlowData.Frames.ParticlesY(:,f);
                app.scatterParticles.Visible = 'on';
            else
                app.scatterParticles.Visible = 'off';
            end
            
            if app.FlowData.separation_x > 0
                idx = find(app.AirfoilData.x_c >= app.FlowData.separation_x, 1);
                if ~isempty(idx)
                    app.plotSep.XData = app.AirfoilData.x_rot(idx);
                    app.plotSep.YData = app.AirfoilData.y_rot(idx);
                    app.plotSep.ZData = 0.2;
                end
            else
                app.plotSep.XData = NaN; app.plotSep.YData = NaN; app.plotSep.ZData = NaN;
            end
            
            app.plotDPDX.XData = app.AirfoilData.x_c;
            app.plotDPDX.YData = app.FlowData.dpdx;
            axis(app.axPressureGrad, 'tight'); xlim(app.axPressureGrad, [0 1]);
            
            app.plotShear.XData = app.AirfoilData.x_c;
            app.plotShear.YData = app.FlowData.tau_w;
            axis(app.axShearStress, 'tight'); xlim(app.axShearStress, [0 1]);
            if min(app.FlowData.tau_w) <= 0
                ylim(app.axShearStress, [min(app.FlowData.tau_w) - 1, max(app.FlowData.tau_w) + 1]);
            end
            
            aoa = app.FlowData.aoa;
            theta_surface = atan(app.AirfoilData.dyu);
            if app.FlowData.separation_x > 0
                idx_eval = find(app.AirfoilData.x_c >= app.FlowData.separation_x, 1) - 1;
            else
                idx_eval = length(app.AirfoilData.x_c) - 10;
            end
            idx_eval = max(2, min(length(app.AirfoilData.x_c), idx_eval));
            del = app.FlowData.delta(idx_eval);
            
            if del > 0
                x_eval = app.AirfoilData.x_rot(idx_eval); y_eval = app.AirfoilData.y_rot(idx_eval);
                nx = -sin(theta_surface(idx_eval) + aoa); ny = cos(theta_surface(idx_eval) + aoa);
                tx = cos(theta_surface(idx_eval) + aoa);  ty = sin(theta_surface(idx_eval) + aoa);
                
                h_arr = linspace(0, del, 50);
                if app.FlowData.tau_w(idx_eval) == 0 
                    u_bl_zoom = zeros(size(h_arr));
                elseif max(app.FlowData.tau_w) > 0 && app.FlowData.tau_w(idx_eval) < 0.2*max(app.FlowData.tau_w) 
                    u_bl_zoom = (h_arr / del).^(1/7);
                else
                    u_bl_zoom = sin(h_arr / del * pi/2);
                end
                
                scale_u = max(0.015, del * 0.75); 
                prof_x = x_eval + h_arr .* nx - (u_bl_zoom * scale_u) .* tx;
                prof_y = y_eval + h_arr .* ny - (u_bl_zoom * scale_u) .* ty;
                norm_x = [x_eval, x_eval + del * nx]; norm_y = [y_eval, y_eval + del * ny];
                
                app.patchAirfoilZoom.XData = app.AirfoilData.x_rot; app.patchAirfoilZoom.YData = app.AirfoilData.y_rot;
                
                coords_bl = [app.AirfoilData.x_c + app.FlowData.delta .* -sin(theta_surface), ...
                             app.AirfoilData.yu + app.FlowData.delta .* cos(theta_surface)];
                coords_bl_shifted = [coords_bl(:,1) - 0.5, coords_bl(:,2)];
                R_mat = [cos(-aoa), -sin(-aoa); sin(-aoa), cos(-aoa)];
                coords_bl_rot = (R_mat * coords_bl_shifted')' + [0.5, 0];
                app.plotBLEdgeZoom.XData = coords_bl_rot(:,1); app.plotBLEdgeZoom.YData = coords_bl_rot(:,2);
                app.plotBLProfile.XData = prof_x; app.plotBLProfile.YData = prof_y;
                app.plotBLNormal.XData = norm_x; app.plotBLNormal.YData = norm_y;
                
                pad = max(0.04, del * 2); xlim(app.axBL, [x_eval - pad, x_eval + pad]); ylim(app.axBL, [y_eval - pad/2, y_eval + pad*1.2]);
                title(app.axBL, sprintf('Physical Boundary Layer (x/c = %.2f)', app.AirfoilData.x_c(idx_eval)));
            end
        end
        
        function theta_max = getThetaMax(~, M, gamma)
            f = @(beta) tan(beta) - 2*cot(beta) * (M^2*sin(beta)^2 - 1)/(M^2*(gamma+cos(2*beta))+2);
            opts = optimset('Display','none');
            [beta_max, ~] = fminbnd(@(b) -f(b), asin(1/M), pi/2, opts);
            theta_max = f(beta_max);
        end
        
        function beta = solveObliqueShock(app, M_guess, theta, gamma)
            M = max(1.01, M_guess); theta_max = app.getThetaMax(M, gamma);
            if theta > theta_max * 0.95, theta = theta_max * 0.95; end
            beta = asin(1/M) + theta + 0.05; 
            for k = 1:10
                cB = cot(beta); s2B = sin(beta)^2; c2B = cos(2*beta);
                num = M^2 * s2B - 1; den = M^2 * (gamma + c2B) + 2;
                f = tan(theta) - 2 * cB * (num / den); db = 1e-5;
                f_plus = tan(theta) - 2*cot(beta+db)*((M^2*sin(beta+db)^2 - 1)/(M^2*(gamma+cos(2*(beta+db)))+2));
                df = (f_plus - f) / db;
                if abs(df) < 1e-6, break; end; beta = beta - f/df;
            end
            beta = max(asin(1/M), min(beta, pi/2));
        end
        
        function nu = prandtlMeyer(~, M, gamma)
            v1 = sqrt((gamma+1)/(gamma-1)); v2 = sqrt((gamma-1)/(gamma+1) * (M^2 - 1));
            nu = v1 * atan(v2) - atan(sqrt(max(0, M^2 - 1)));
        end
        
        function M = invPrandtlMeyer(app, nu_target, M_guess, gamma)
            M = max(1.001, M_guess); nu_max = (sqrt((gamma+1)/(gamma-1)) - 1) * pi/2;
            nu_target = min(nu_target, nu_max * 0.98); 
            for k = 1:20
                nu_current = app.prandtlMeyer(M, gamma); err = nu_current - nu_target;
                if abs(err) < 1e-5, break; end
                dnu_dM = sqrt(max(0, M^2 - 1)) / (M * (1 + 0.5*(gamma-1)*M^2));
                M = M - err / max(1e-4, dnu_dM); M = max(1.001, M);
            end
        end
        
        function [T, P, rho, a, mu] = standardAtmosphere(~, h)
            if h < 11000
                T = 288.15 - 0.0065 * h; P = 101325 * (T / 288.15)^5.25588;
            else
                T = 216.65; P = 22632 * exp(-9.80665 * (h - 11000) / (287.05 * 216.65));
            end
            rho = P / (287.05 * T); a = sqrt(1.4 * 287.05 * T);
            mu = 1.716e-5 * (T/273.15)^1.5 * (273.15 + 111)/(T + 111);
        end
        
        function [x, yu, yl, dyu, dyl, x_node, y_node] = generateGeometry(app)
            type = app.ddShapeType.Value; x = linspace(0, 1, 300)'; 
            if strcmp(type, 'Standard (NACA)')
                id = app.ddStandardID.Value;
                m = str2double(id(1))/100; p = str2double(id(2))/10; t = str2double(id(3:4))/100;
                yt = 5*t*(0.2969*sqrt(x) - 0.1260*x - 0.3516*x.^2 + 0.2843*x.^3 - 0.1036*x.^4);
                dyt = 5*t*(0.5*0.2969./max(sqrt(x),1e-4) - 0.1260 - 2*0.3516*x + 3*0.2843*x.^2 - 4*0.1036*x.^3);
                yc = zeros(size(x)); dyc = zeros(size(x));
                for i = 1:length(x)
                    if x(i) <= p && p > 0
                        yc(i) = m/p^2 * (2*p*x(i) - x(i)^2); dyc(i) = 2*m/p^2 * (p - x(i));
                    elseif p > 0
                        yc(i) = m/(1-p)^2 * ((1-2*p) + 2*p*x(i) - x(i)^2); dyc(i) = 2*m/(1-p)^2 * (p - x(i));
                    end
                end
                theta = atan(dyc);
                yu = yc + yt.*cos(theta); yl = yc - yt.*cos(theta);
                dyu = dyc + dyt; dyl = dyc - dyt;
            elseif strcmp(type, 'Diamond')
                yu = interp1([0 0.5 1], [0 0.08 0], x); yl = interp1([0 0.5 1], [0 -0.08 0], x);
                dyu = interp1([0 0.499 0.501 1], [0.16 0.16 -0.16 -0.16], x, 'nearest'); dyl = interp1([0 0.499 0.501 1], [-0.16 -0.16 0.16 0.16], x, 'nearest');
            elseif strcmp(type, 'Wedge')
                yu = interp1([0 1], [0 0.1], x); yl = interp1([0 1], [0 -0.1], x);
                dyu = 0.1 * ones(size(x)); dyl = -0.1 * ones(size(x));
            else 
                yu = 1e-4 * ones(size(x)); yl = -1e-4 * ones(size(x));
                yu(1) = 0; yu(end) = 0; yl(1) = 0; yl(end) = 0;
                dyu = zeros(size(x)); dyl = zeros(size(x));
            end
            x_node = [flipud(x(2:end)); x]; y_node = [flipud(yl(2:end)); yu];
        end
    end
end