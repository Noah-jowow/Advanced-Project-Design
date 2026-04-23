classdef AzElDirector < handle
    % =====================================================================
    % EXACT AZIMUTH/ELEVATION (AZ/EL) GIMBAL TRACKING SIMULATION
    % =====================================================================
    % This is a standalone modular script designed for future integration 
    % into the "System of Systems" Master Simulator.
    %
    % MATHEMATICAL FRAMEWORK: Exact Euler-Lagrange Dynamics
    % - Non-linear coupled moments of inertia.
    % - Exact Centrifugal and Coriolis torque compensation.
    % - Actuator force governors and continuous minimum-roll wrap-around.
    %
    % The core effector logic is isolated in stepAzElEffector() to ensure
    % rapid hot-swapping with other modules (like the Hexapod).
    % =====================================================================

    properties (Access = private)
        % UI Components
        UIFigure
        BtnRun
        BtnStop
        CheckAutoOrbit
        SliderX, SliderY, SliderZ
        TxtX, TxtY, TxtZ
        
        % Axes & Visuals
        Ax3D
        GimbalBase
        GimbalAzFork
        GimbalElPayload
        LaserBeam
        TargetMarker
        
        % Transforms for Hierarchy
        TAz % Azimuth Transform
        TEl % Elevation Transform
        
        % Telemetry UI
        TelemetryText
        WarningText
        
        % Simulation State
        simRunning = false;
        
        % Physics Parameters
        azelParams
        azelState
        renderCounter = 0;
    end
    
    methods
        function obj = AzElDirector()
            obj.setupParams();
            obj.buildUI();
            obj.drawGimbal();
        end
    end
    
    methods (Access = private)
        
        function setupParams(obj)
            % 1. Mass and Inertia Properties
            % Assumes a balanced payload (Center of Mass at intersection of axes)
            obj.azelParams.m_az = 50.0; % kg (Fork mass)
            obj.azelParams.J_z  = 4.0;  % kg*m^2 (Inertia of fork about Z axis)
            
            % Payload (Telescope / Radar housing) modeled as cylinder
            obj.azelParams.m_p = 30.0;  % kg
            r_p = 0.2; L_p = 0.8;
            % Inertial tensor for payload (X = forward pointing, Y = elevation axis)
            obj.azelParams.I_x = 0.5 * obj.azelParams.m_p * r_p^2;
            obj.azelParams.I_y = (1/12) * obj.azelParams.m_p * (3*r_p^2 + L_p^2);
            obj.azelParams.I_z = obj.azelParams.I_y; 
            
            % 2. Hardware Limitations (Governors)
            obj.azelParams.Tau_max_az = 500;  % Nm (Max Azimuth Motor Torque)
            obj.azelParams.Tau_max_el = 300;  % Nm (Max Elevation Motor Torque)
            obj.azelParams.Max_Vel = 3.0;     % rad/s (Slip-ring/bearing limit)
            
            % 3. Initial State [Azimuth; Elevation]
            obj.azelState.q = [0; 0];         % Current angles (rad)
            obj.azelState.q_dot = [0; 0];     % Current velocities (rad/s)
        end
        
        function buildUI(obj)
            % Main Figure
            obj.UIFigure = figure('Name', 'Az/El Exact Dynamic Director Module', ...
                                  'Position', [100, 100, 1200, 800], ...
                                  'MenuBar', 'none', 'NumberTitle', 'off', ...
                                  'Color', 'w', 'CloseRequestFcn', @obj.onClose);
                              
            % Controls Panel
            pnlControls = uipanel(obj.UIFigure, 'Title', 'Target Generation & C2', ...
                                  'Position', [0.02, 0.02, 0.25, 0.96], 'FontWeight', 'bold');
                              
            obj.BtnRun = uicontrol('Parent', pnlControls, 'Style', 'pushbutton', 'String', '▶ Run Simulator', ...
                                   'Units', 'normalized', 'Position', [0.05, 0.90, 0.90, 0.05], ...
                                   'BackgroundColor', [0 0.5 0], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
                                   'Callback', @(src,event) obj.startSimulation());
                               
            obj.BtnStop = uicontrol('Parent', pnlControls, 'Style', 'pushbutton', 'String', '■ Stop', ...
                                   'Units', 'normalized', 'Position', [0.05, 0.84, 0.90, 0.05], ...
                                   'BackgroundColor', [0.8 0 0], 'ForegroundColor', 'w', 'FontWeight', 'bold', ...
                                   'Callback', @(src,event) obj.stopSimulation());
                               
            obj.CheckAutoOrbit = uicontrol('Parent', pnlControls, 'Style', 'checkbox', 'String', 'Auto-Orbit Target Trajectory', ...
                                      'Units', 'normalized', 'Position', [0.05, 0.75, 0.90, 0.05], 'Value', 1, 'FontWeight', 'bold');
                                  
            % Manual Target Sliders
            uicontrol('Parent', pnlControls, 'Style', 'text', 'String', 'Manual Target X (m):', 'Units', 'normalized', 'Position', [0.05, 0.65, 0.90, 0.03], 'HorizontalAlignment', 'left');
            obj.SliderX = uicontrol('Parent', pnlControls, 'Style', 'slider', 'Min', -100, 'Max', 100, 'Value', 50, 'Units', 'normalized', 'Position', [0.05, 0.61, 0.90, 0.04]);
            
            uicontrol('Parent', pnlControls, 'Style', 'text', 'String', 'Manual Target Y (m):', 'Units', 'normalized', 'Position', [0.05, 0.55, 0.90, 0.03], 'HorizontalAlignment', 'left');
            obj.SliderY = uicontrol('Parent', pnlControls, 'Style', 'slider', 'Min', -100, 'Max', 100, 'Value', 50, 'Units', 'normalized', 'Position', [0.05, 0.51, 0.90, 0.04]);
            
            uicontrol('Parent', pnlControls, 'Style', 'text', 'String', 'Manual Target Z (m):', 'Units', 'normalized', 'Position', [0.05, 0.45, 0.90, 0.03], 'HorizontalAlignment', 'left');
            obj.SliderZ = uicontrol('Parent', pnlControls, 'Style', 'slider', 'Min', 0, 'Max', 100, 'Value', 20, 'Units', 'normalized', 'Position', [0.05, 0.41, 0.90, 0.04]);
            
            % Telemetry output
            obj.WarningText = uicontrol('Parent', obj.UIFigure, 'Style', 'text', 'Position', [850, 720, 300, 30], ...
                                           'String', 'SYSTEM NOMINAL', ...
                                           'BackgroundColor', 'g', 'ForegroundColor', 'k', ...
                                           'FontWeight', 'bold', 'FontSize', 14);
                                       
            obj.TelemetryText = uicontrol('Parent', obj.UIFigure, 'Style', 'text', 'Position', [850, 450, 320, 250], ...
                                             'String', 'Waiting for telemetry...', ...
                                             'BackgroundColor', 'w', 'HorizontalAlignment', 'left', ...
                                             'FontName', 'Courier New', 'FontSize', 11);
                                         
            % 3D Axes
            obj.Ax3D = axes('Parent', obj.UIFigure, 'Position', [0.30, 0.05, 0.55, 0.90]);
            hold(obj.Ax3D, 'on'); grid(obj.Ax3D, 'on'); view(obj.Ax3D, 3);
            xlabel(obj.Ax3D, 'East X (m)'); ylabel(obj.Ax3D, 'North Y (m)'); zlabel(obj.Ax3D, 'Alt Z (m)');
            title(obj.Ax3D, 'Exact Az/El Kinematics & Euler-Lagrange Dynamics', 'FontSize', 14);
            axis(obj.Ax3D, 'equal');
            xlim(obj.Ax3D, [-100 100]); ylim(obj.Ax3D, [-100 100]); zlim(obj.Ax3D, [0 100]); 
        end
        
        function drawGimbal(obj)
            % Hierarchical setup using hgtransform for extremely fast and exact rendering
            obj.TAz = hgtransform('Parent', obj.Ax3D);
            obj.TEl = hgtransform('Parent', obj.TAz);
            
            % 1. Static Base (Gray block)
            [Xb, Yb, Zb] = cylinder([1.5, 1.5], 20);
            Zb = Zb * 1.5; % Height 1.5
            surface('Parent', obj.Ax3D, 'XData', Xb, 'YData', Yb, 'ZData', Zb, 'FaceColor', [0.4 0.4 0.4], 'EdgeColor', 'none');
            
            % 2. Azimuth Fork (Blue) attached to TAz
            % Center column
            [Xc, Yc, Zc] = cylinder([0.8, 0.8], 20);
            Zc = Zc * 1.0 + 1.5; 
            surface('Parent', obj.TAz, 'XData', Xc, 'YData', Yc, 'ZData', Zc, 'FaceColor', [0.2 0.5 0.8], 'EdgeColor', 'none');
            % Left Arm
            patch('Parent', obj.TAz, 'XData', [-0.5 -0.5 0.5 0.5], 'YData', [-1.2 -0.8 -0.8 -1.2], 'ZData', [2.5 2.5 2.5 2.5], 'FaceColor', [0.2 0.5 0.8]);
            patch('Parent', obj.TAz, 'XData', [-0.5 -0.5 0.5 0.5], 'YData', [-1.2 -0.8 -0.8 -1.2], 'ZData', [4.0 4.0 4.0 4.0], 'FaceColor', [0.2 0.5 0.8]);
            fill3(obj.TAz, [-0.5 -0.5 0.5 0.5], [-1.2 -0.8 -0.8 -1.2], [2.5 4.0 4.0 2.5], [0.2 0.5 0.8]);
            % Right Arm
            fill3(obj.TAz, [-0.5 -0.5 0.5 0.5], [0.8 1.2 1.2 0.8], [2.5 4.0 4.0 2.5], [0.2 0.5 0.8]);
            
            % Shift the Elevation transform to the pivot point between the arms
            obj.TEl.Matrix = makehgtform('translate', [0, 0, 3.25]);
            
            % 3. Elevation Payload (Gold Cylinder pointing along X-axis initially)
            [Xp, Yp, Zp] = cylinder([0.6, 0.6], 20);
            % Rotate cylinder to lay flat along X axis
            p_mat = makehgtform('yrotate', pi/2);
            pts = [Xp(:)'; Yp(:)'; Zp(:)'; ones(1, numel(Xp))];
            pts = p_mat * pts;
            Xp(:) = pts(1,:); Yp(:) = pts(2,:); Zp(:) = pts(3,:);
            
            % Shift it to center on origin
            Xp = Xp - 0.5;
            Xp = Xp * 3.0; % Make it 3m long
            surface('Parent', obj.TEl, 'XData', Xp, 'YData', Yp, 'ZData', Zp, 'FaceColor', [0.8 0.6 0.1], 'EdgeColor', 'k');
            
            % 4. Laser / Beam (Red line emerging from payload)
            obj.LaserBeam = plot3(obj.TEl, [0, 200], [0, 0], [0, 0], 'r-', 'LineWidth', 3);
            
            % 5. Target Marker
            obj.TargetMarker = scatter3(obj.Ax3D, 0, 0, 0, 100, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');
        end
        
        function onClose(obj, ~, ~)
            obj.simRunning = false;
            delete(obj.UIFigure);
        end
        
        function stopSimulation(obj)
            obj.simRunning = false;
        end
        
        function startSimulation(obj)
            if obj.simRunning, return; end
            
            obj.simRunning = true;
            obj.BtnRun.Enable = 'off';
            
            dt = 0.01; % 100 Hz servo loop integration
            t = 0;
            
            while obj.simRunning && isvalid(obj.UIFigure)
                loop_start = tic;
                t = t + dt;
                
                % --- 1. TARGET GENERATION ---
                if obj.CheckAutoOrbit.Value
                    % Create a rapid maneuvering Lissajous target pattern
                    target_pos = [60 * cos(0.5 * t); 
                                  60 * sin(0.7 * t); 
                                  40 + 20 * sin(0.3 * t)];
                              
                    % Sync sliders for visual feedback
                    obj.SliderX.Value = target_pos(1);
                    obj.SliderY.Value = target_pos(2);
                    obj.SliderZ.Value = target_pos(3);
                else
                    target_pos = [obj.SliderX.Value; 
                                  obj.SliderY.Value; 
                                  obj.SliderZ.Value];
                end
                
                % Update target visual
                set(obj.TargetMarker, 'XData', target_pos(1), 'YData', target_pos(2), 'ZData', target_pos(3));
                
                % --- 2. MODULAR EFFECTOR CALL ---
                % This exact function block is designed to be lifted directly into the Master File.
                obj.stepAzElEffector(target_pos, dt);
                
                % --- 3. TIMING & RENDER ---
                drawnow limitrate;
                loop_time = toc(loop_start);
                pause_time = dt - loop_time;
                if pause_time > 0, pause(pause_time); end
            end
            
            if isvalid(obj.UIFigure), obj.BtnRun.Enable = 'on'; end
        end
        
        % =========================================================
        % --- EFFECTOR MODULE: AZ/EL STEP EXECUTION ---
        % =========================================================
        function stepAzElEffector(obj, target_pos, dt)
            % This function computes the required tracking angles, applies a pursuit 
            % filter, and calculates the EXACT Euler-Lagrange kinetic torques required.
            
            % 1. Kinematic Target Translation (World to Spherical)
            % The gimbal pivot is at [0, 0, 3.25] based on the geometric drawing.
            gimbal_origin = [0; 0; 3.25];
            v_target = target_pos - gimbal_origin;
            
            % Calculate commanded angles
            % Azimuth maps exactly to atan2.
            q_az_cmd = atan2(v_target(2), v_target(1));
            
            % Elevation is angle from horizontal XY plane
            xy_dist = sqrt(v_target(1)^2 + v_target(2)^2);
            q_el_cmd = atan2(v_target(3), xy_dist);
            
            % Extract current state
            q_az = obj.azelState.q(1);
            q_el = obj.azelState.q(2);
            q_dot_az = obj.azelState.q_dot(1);
            q_dot_el = obj.azelState.q_dot(2);
            
            % 2. State-Space Pursuit Filter (Critically Damped)
            % Calculate exact angular error using modular continuous math (wrap-around safe)
            err_az = mod(q_az_cmd - q_az + pi, 2*pi) - pi;
            err_el = q_el_cmd - q_el; % El rarely exceeds -pi/2 to pi/2, no wrap needed
            
            omega_n = 5.0; % Rad/s response speed
            zeta = 1.0;    % Critically damped
            
            q_ddot_az_req = omega_n^2 * err_az - 2 * zeta * omega_n * q_dot_az;
            q_ddot_el_req = omega_n^2 * err_el - 2 * zeta * omega_n * q_dot_el;
            
            % 3. EXACT Euler-Lagrange Dynamics Matrix
            p = obj.azelParams;
            
            % Effective Azimuth Inertia dynamically changes based on Elevation angle!
            % As the payload pitches up, its mass moves closer to the Z-axis.
            I_eff_az = p.J_z + p.I_x * sin(q_el)^2 + p.I_z * cos(q_el)^2;
            
            % Centrifugal/Coriolis Coupling terms (Non-linear effects of rapid motion)
            % Rapid panning forces the elevation axis to try and pitch down.
            C_az = (p.I_x - p.I_z) * sin(2 * q_el) * q_dot_az * q_dot_el;
            C_el = 0.5 * (p.I_x - p.I_z) * sin(2 * q_el) * q_dot_az^2;
            
            % Calculate required Torques to achieve the commanded acceleration
            tau_req_az = I_eff_az * q_ddot_az_req + C_az;
            tau_req_el = p.I_y * q_ddot_el_req - C_el;
            
            % 4. HARDWARE GOVERNORS (Physical Limits)
            warn_str = 'SYSTEM NOMINAL'; warn_col = 'g';
            
            tau_act_az = max(-p.Tau_max_az, min(p.Tau_max_az, tau_req_az));
            tau_act_el = max(-p.Tau_max_el, min(p.Tau_max_el, tau_req_el));
            
            if abs(tau_req_az) > p.Tau_max_az || abs(tau_req_el) > p.Tau_max_el
                warn_str = 'ERR: TORQUE SATURATION'; warn_col = 'y';
            end
            
            % Recalculate achievable acceleration based on limited torque
            q_ddot_az_act = (tau_act_az - C_az) / I_eff_az;
            q_ddot_el_act = (tau_act_el + C_el) / p.I_y;
            
            % Velocity limits
            if abs(q_dot_az) >= p.Max_Vel && sign(q_ddot_az_act) == sign(q_dot_az)
                q_ddot_az_act = 0; % Stop accelerating
                warn_str = 'ERR: MAX PAN VELOCITY'; warn_col = 'r';
            end
            if abs(q_dot_el) >= p.Max_Vel && sign(q_ddot_el_act) == sign(q_dot_el)
                q_ddot_el_act = 0;
                warn_str = 'ERR: MAX TILT VELOCITY'; warn_col = 'r';
            end
            
            % 5. INTEGRATION (Semi-Implicit Euler)
            obj.azelState.q_dot(1) = obj.azelState.q_dot(1) + q_ddot_az_act * dt;
            obj.azelState.q_dot(2) = obj.azelState.q_dot(2) + q_ddot_el_act * dt;
            
            obj.azelState.q(1) = obj.azelState.q(1) + obj.azelState.q_dot(1) * dt;
            obj.azelState.q(2) = obj.azelState.q(2) + obj.azelState.q_dot(2) * dt;
            
            % Keep Azimuth bounded [-pi, pi] internally
            obj.azelState.q(1) = mod(obj.azelState.q(1) + pi, 2*pi) - pi;
            
            % 6. UI RENDER (Throttled for performance)
            obj.renderCounter = obj.renderCounter + 1;
            if mod(obj.renderCounter, 3) == 0 && isvalid(obj.UIFigure)
                % Update Transforms using exact hierarchical matrix rotation
                obj.TAz.Matrix = makehgtform('zrotate', obj.azelState.q(1));
                
                % TEl is a child of TAz. We apply translation to pivot point, then rotate.
                % Negative El rotation because typical aviation standard pitches *up* for positive.
                obj.TEl.Matrix = makehgtform('translate', [0, 0, 3.25], 'yrotate', -obj.azelState.q(2));
                
                % Update Telemetry
                set(obj.WarningText, 'String', warn_str, 'BackgroundColor', warn_col);
                
                data_str = sprintf(['--- KINEMATIC STATE ---\n' ...
                                    'Cmd Azimuth:  %+6.2f deg\n' ...
                                    'Act Azimuth:  %+6.2f deg\n' ...
                                    'Cmd Elevatn:  %+6.2f deg\n' ...
                                    'Act Elevatn:  %+6.2f deg\n\n' ...
                                    '--- DYNAMIC COUPLING ---\n' ...
                                    'Eff Inertia:  %6.2f kgm^2\n' ...
                                    'Coriolis C_az: %+6.1f Nm\n' ...
                                    'Centrif  C_el: %+6.1f Nm\n\n' ...
                                    '--- MOTOR OUTPUT ---\n' ...
                                    'Tau Azimuth:   %+6.1f Nm\n' ...
                                    'Tau Elevation: %+6.1f Nm\n\n'], ...
                                    rad2deg(q_az_cmd), rad2deg(obj.azelState.q(1)), ...
                                    rad2deg(q_el_cmd), rad2deg(obj.azelState.q(2)), ...
                                    I_eff_az, C_az, C_el, tau_act_az, tau_act_el);
                set(obj.TelemetryText, 'String', data_str);
            end
        end
    end
end