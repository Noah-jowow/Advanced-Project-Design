% =========================================================================
% Interactive Stewart Platform Exact Newton-Euler Tracking App
% =========================================================================
% This script implements a mathematically exact rigid-body model for the 
% top plate of a hexapod. 
%
% COMMERCIAL OPTIMIZATIONS IN THIS VERSION:
% 1. Predictive Dynamic Trajectory Scaling (Force Governors).
% 2. Kinematic Stroke Limits: Actuator min/max extension boundaries.
% 3. Singularity Avoidance: Real-time Jacobian condition number monitoring.
% 4. Memory Efficiency: Invariant physical properties packed into a single 
%    memory-contiguous struct.
% 5. Mathematical Safeguards: Divide-by-zero prevention on target intersection.
% =========================================================================

clear; clc;

%% 1. Hexapod Non-Linear Geometry & Mass Properties
% Pack invariants into a struct for memory-efficient function passing
params.R_base = 0.5;   
params.R_top = 0.3;    
params.Z_nom = 0.6;    

% Exact Mass and Inertia Properties
params.m_top = 25.0; % kg
params.I_local = diag([ (1/4)*params.m_top*params.R_top^2, ...
                        (1/4)*params.m_top*params.R_top^2, ...
                        (1/2)*params.m_top*params.R_top^2 ]);
params.g_vec = [0; 0; -9.81]; 

% Anchor point angles
theta_b = deg2rad([15, 105, 135, 225, 255, 345]);
theta_t = deg2rad([45, 75, 165, 195, 285, 315]);

% Pre-allocate Base and Top anchor points
params.B = zeros(3, 6);
params.P = zeros(3, 6);
for i = 1:6
    params.B(:, i) = [params.R_base * cos(theta_b(i)); params.R_base * sin(theta_b(i)); 0];
    params.P(:, i) = [params.R_top * cos(theta_t(i)); params.R_top * sin(theta_t(i)); 0];
end

% HARDWARE LIMITS (The safety bounds)
F_limit = 1500;         % Maximum Actuator Force (Newtons)
L_min = 0.40;           % Minimum Actuator Stroke (Meters)
L_max = 0.95;           % Maximum Actuator Stroke (Meters)
Cond_Limit = 500;       % Maximum allowable Jacobian Condition Number

%% 2. Setup Interactive UI Figure
fig = figure('Name', 'Interactive Exact Hexapod Tracking', ...
             'NumberTitle', 'off', 'Color', 'w', ...
             'Position', [100, 100, 1200, 700]);

% Create Sliders for Target Control
uicontrol('Style', 'text', 'Position', [50, 580, 200, 20], 'String', 'Target X Position (m)', 'BackgroundColor', 'w', 'FontWeight', 'bold');
slider_X = uicontrol('Style', 'slider', 'Min', -15, 'Max', 15, 'Value', 5, 'Position', [50, 560, 200, 20]);

uicontrol('Style', 'text', 'Position', [50, 500, 200, 20], 'String', 'Target Y Position (m)', 'BackgroundColor', 'w', 'FontWeight', 'bold');
slider_Y = uicontrol('Style', 'slider', 'Min', -15, 'Max', 15, 'Value', 5, 'Position', [50, 480, 200, 20]);

uicontrol('Style', 'text', 'Position', [50, 420, 200, 20], 'String', 'Target Z Position (m)', 'BackgroundColor', 'w', 'FontWeight', 'bold');
slider_Z = uicontrol('Style', 'slider', 'Min', 1.0, 'Max', 20, 'Value', 10, 'Position', [50, 400, 200, 20]);

% Data Output Text
info_text = uicontrol('Style', 'text', 'Position', [50, 100, 250, 250], ...
                      'String', 'Initializing Dynamics...', ...
                      'BackgroundColor', 'w', 'HorizontalAlignment', 'left', 'FontSize', 10);

% Motion Inhibitor Warning Tag
warning_text = uicontrol('Style', 'text', 'Position', [50, 360, 250, 30], ...
                         'String', 'SYSTEM NOMINAL', ...
                         'BackgroundColor', 'g', 'ForegroundColor', 'k', ...
                         'FontWeight', 'bold', 'FontSize', 12);

%% 3. Setup Close-up 3D Visualization
ax3d = axes('Parent', fig, 'Position', [0.35, 0.1, 0.6, 0.8]);
hold(ax3d, 'on'); grid(ax3d, 'on'); view(ax3d, 3);
xlabel(ax3d, 'X (m)'); ylabel(ax3d, 'Y (m)'); zlabel(ax3d, 'Z (m)');
title(ax3d, 'Close-Up: Hexapod Joint & Actuator Dynamics', 'FontSize', 14);

% Draw Base Plate
patch('Parent', ax3d, 'XData', params.B(1,:), 'YData', params.B(2,:), 'ZData', params.B(3,:), ...
      'FaceColor', [0.3 0.3 0.3], 'FaceAlpha', 0.5, 'EdgeColor', 'k', 'LineWidth', 2);

% Initialize Top Plate graphics
top_plate_patch = patch('Parent', ax3d, 'XData', params.P(1,:), 'YData', params.P(2,:), 'ZData', params.P(3,:)+params.Z_nom, ...
                        'FaceColor', [0.2 0.6 0.8], 'FaceAlpha', 0.8, 'EdgeColor', 'k', 'LineWidth', 1.5);

% Initialize 6 Legs
leg_lines = gobjects(1,6);
for i = 1:6
    leg_lines(i) = plot3(ax3d, [params.B(1,i), params.P(1,i)], [params.B(2,i), params.P(2,i)], [params.B(3,i), params.P(3,i)+params.Z_nom], ...
                         'Color', [0.8 0.4 0.1], 'LineWidth', 4);
end

% Initialize Laser Beam
laser_line = plot3(ax3d, [0, 0], [0, 0], [params.Z_nom, 10], 'r-', 'LineWidth', 2);

% LOCK CAMERA TO CLOSE UP VIEW
axis(ax3d, 'equal');
xlim(ax3d, [-0.8 0.8]); 
ylim(ax3d, [-0.8 0.8]); 
zlim(ax3d, [0 1.2]); 

%% 4. State Tracking Initialization
T = [0; 0; params.Z_nom]; 
T_ddot = zeros(3,1); 

% Trajectory Generator States
Pt = [slider_X.Value; slider_Y.Value; slider_Z.Value]; 
V_t = zeros(3,1);

t_last = tic;
render_counter = 0; 

%% 5. Real-Time Interactive Loop
while isvalid(fig)
    if ~isvalid(slider_X), break; end
    
    dt = toc(t_last);
    if dt < 0.01
        pause(0.01 - dt); 
        dt = toc(t_last);
    end
    t_last = tic;
    % Hardware safety bound on numerical derivatives
    dt = max(dt, 1e-4); 
    
    % Read Commanded Target from UI
    Pt_cmd = [slider_X.Value; slider_Y.Value; slider_Z.Value];
    
    % --- A. IDEAL COMMANDED TRAJECTORY ---
    % Critically damped 2nd-order state-space filter
    omega_n = 4.0; % Rad/s response speed
    zeta = 1.0;    % Critically damped
    A_t_cmd = omega_n^2 * (Pt_cmd - Pt) - 2 * zeta * omega_n * V_t;
    
    % --- B. PREDICTIVE CALCULATION ---
    [F_cmd, Top_Points, l_cmd, u, cond_J] = compute_hexapod_state(Pt, V_t, A_t_cmd, T, T_ddot, params);
    
    % --- C. MULTI-VARIABLE SAFETY GOVERNOR ---
    % 1. Check Jacobian Conditioning (Singularity Avoidance)
    if cond_J > Cond_Limit
        A_t_actual = -10.0 * V_t; % Emergency Brake
        [F_act, Top_Points, l_cmd, u, ~] = compute_hexapod_state(Pt, V_t, A_t_actual, T, T_ddot, params);
        warning_str = 'ERR: KINEMATIC SINGULARITY';
        warning_color = 'r';
        
    % 2. Check Kinematic Stroke Limits
    elseif any(l_cmd < L_min) || any(l_cmd > L_max)
        A_t_actual = -10.0 * V_t; % Emergency Brake
        [F_act, Top_Points, l_cmd, u, ~] = compute_hexapod_state(Pt, V_t, A_t_actual, T, T_ddot, params);
        warning_str = 'ERR: STROKE LIMIT EXCEEDED';
        warning_color = 'r';
        
    % 3. Check Actuator Force Limits (Predictive Scaling)
    elseif max(abs(F_cmd)) > F_limit
        % Predict baseline forces for zero acceleration
        [F_base, ~, ~, ~, ~] = compute_hexapod_state(Pt, V_t, zeros(3,1), T, T_ddot, params);
        
        if max(abs(F_base)) > F_limit
            A_t_actual = -5.0 * V_t; % Emergency Braking (Gravity alone exceeds limits)
            [F_act, Top_Points, l_cmd, u, ~] = compute_hexapod_state(Pt, V_t, A_t_actual, T, T_ddot, params);
            warning_str = 'ERR: OVERLOAD BRAKING';
            warning_color = 'r';
        else
            % Exact Affine Force Interpolation
            alpha = 1.0;
            dF = F_cmd - F_base;
            for i = 1:6
                if abs(dF(i)) > 1e-6
                    if dF(i) > 0
                        a_max_leg = (F_limit - F_base(i)) / dF(i);
                    else
                        a_max_leg = (-F_limit - F_base(i)) / dF(i);
                    end
                    alpha = min(alpha, a_max_leg);
                end
            end
            
            alpha = max(0, alpha); 
            A_t_actual = alpha * A_t_cmd;
            [F_act, Top_Points, l_cmd, u, ~] = compute_hexapod_state(Pt, V_t, A_t_actual, T, T_ddot, params);
            warning_str = sprintf('GOVERNOR: FORCE (Scale: %.2f)', alpha);
            warning_color = 'y';
        end
        
    % 4. System Nominal
    else
        A_t_actual = A_t_cmd;
        F_act = F_cmd;
        warning_str = 'SYSTEM NOMINAL';
        warning_color = 'g';
    end
    
    % --- D. STATE INTEGRATION (Semi-Implicit Euler) ---
    V_t = V_t + A_t_actual * dt;
    Pt = Pt + V_t * dt;
    
    % --- E. VISUALIZATION & UI UPDATE (THROTTLED) ---
    render_counter = render_counter + 1;
    if mod(render_counter, 5) == 0 && isvalid(fig)
        
        set(top_plate_patch, 'XData', Top_Points(1,:), 'YData', Top_Points(2,:), 'ZData', Top_Points(3,:));
        for i = 1:6
            set(leg_lines(i), 'XData', [params.B(1,i), Top_Points(1,i)], ...
                              'YData', [params.B(2,i), Top_Points(2,i)], ...
                              'ZData', [params.B(3,i), Top_Points(3,i)]);
        end
        
        laser_end = T + u * 5; 
        set(laser_line, 'XData', [T(1), laser_end(1)], ...
                        'YData', [T(2), laser_end(2)], ...
                        'ZData', [T(3), laser_end(3)]);
        
        if isvalid(warning_text)
            set(warning_text, 'String', warning_str, 'BackgroundColor', warning_color);
        end
        
        if isvalid(info_text)
            data_str = sprintf(['Target Pos: [%.1f, %.1f, %.1f] m\n' ...
                                'Jacobian Cond #: %.1f\n\n' ...
                                'Actuator Forces (N) [Lim: %d]:\n' ...
                                'L1: %8.1f   L4: %8.1f\n' ...
                                'L2: %8.1f   L5: %8.1f\n' ...
                                'L3: %8.1f   L6: %8.1f\n\n' ...
                                'Leg Lengths (m) [%.2f - %.2f]:\n' ...
                                'L1: %.3f   L4: %.3f\n' ...
                                'L2: %.3f   L5: %.3f\n' ...
                                'L3: %.3f   L6: %.3f'], ...
                                Pt(1), Pt(2), Pt(3), cond_J, F_limit, ...
                                F_act(1), F_act(4), F_act(2), F_act(5), F_act(3), F_act(6), ...
                                L_min, L_max, ...
                                l_cmd(1), l_cmd(4), l_cmd(2), l_cmd(5), l_cmd(3), l_cmd(6));
            set(info_text, 'String', data_str);
        end
        
        drawnow limitrate;
    end
end

%% Helper Function: Exact Analytical Hexapod State Engine
function [F_act, Top_Points, l_cmd, u, cond_J] = compute_hexapod_state(Pt, V_t, A_t, T, T_ddot, p)
    % 1. Target Vector & Exact Derivatives
    v = Pt - T;
    v_mag = norm(v);
    
    % MATHEMATICAL SAFEGUARD: Prevent singularity if target intersects origin
    if v_mag < 0.1
        v_mag = 0.1;
        v = v / norm(v) * v_mag;
    end
    
    u = v / v_mag; % Boresight unit vector

    % Quotient rule for exact first & second derivatives
    v_mag_dot = dot(u, V_t);
    num = V_t - u * v_mag_dot;
    u_dot = num / v_mag;

    v_mag_ddot = dot(u_dot, V_t) + dot(u, A_t);
    num_dot = A_t - (u_dot * v_mag_dot + u * v_mag_ddot);
    u_ddot = (num_dot * v_mag - num * v_mag_dot) / (v_mag^2);

    % Exact Analytical Angular Velocity and Acceleration
    omega = cross(u, u_dot);
    omega_dot = cross(u, u_ddot); 

    % 2. Rotation Matrix Formulation (Continuous Roll Avoidance)
    y_up = [0; 1; 0];
    x_alt = [1; 0; 0];
    x_cmd = cross(y_up, u); 
    
    % If pointing directly up/down the Y axis, switch reference to prevent NaN
    if norm(x_cmd) < 1e-6
        x_cmd = cross(x_alt, u); 
    end
    
    x_cmd = x_cmd / norm(x_cmd);
    y_cmd = cross(u, x_cmd);
    R = [x_cmd, y_cmd, u];

    % 3. Newton-Euler Dynamics
    I_inertial = R * p.I_local * R';
    F_req = p.m_top * (T_ddot - p.g_vec);
    tau_req = I_inertial * omega_dot + cross(omega, I_inertial * omega);
    Wrench_req = [F_req; tau_req];

    % 4. Geometric Jacobian & Inverse Kinematics
    J = zeros(6, 6);
    Top_Points = zeros(3, 6);
    l_cmd = zeros(6, 1);
    
    for i = 1:6
        P_inertial = R * p.P(:, i);
        Top_Points(:, i) = T + P_inertial;
        L_i = Top_Points(:, i) - p.B(:, i);
        l_cmd(i) = norm(L_i);
        s_i = L_i / l_cmd(i); 
        J(i, 1:3) = s_i';                 
        J(i, 4:6) = cross(P_inertial, s_i)'; 
    end

    % 5. Jacobian Conditioning Check
    cond_J = cond(J);

    % 6. Wrench to Actuator Force Mapping (Safe Inversion)
    if cond_J > 1e4
        % Hard singularity fallback to prevent math crash
        F_act = zeros(6,1); 
    else
        % Use LAPACK optimized solver
        F_act = J' \ Wrench_req; 
    end
end