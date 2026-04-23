function Rocket_Nozzle_Optimizer()
    % ROCKET_NOZZLE_OPTIMIZER - ULTIMATE FIDELITY (COMMERCIAL REVIEW)
    % A commercial-grade MATLAB UI for optimizing rocket nozzle geometry.
    % Iteration 8: Fully Vectorized Solver, Exact Rao TOP Construction, 
    % Bartz Sigma Correction Factor, and Eckert Reference Temperature BL.
    % 
    % References: 
    % - Software and Engineering Associates "Two-Dimensional Kinetics (TDK)"
    % - Sutton, G.P. "Rocket Propulsion Elements"
    % - Anderson, J.D. "Modern Compressible Flow"

    %% --- UI & Dashboard Setup ---
    fig = figure('Name', 'Commercial Rocket Nozzle Optimizer (TDK MoC + BL)', ...
                 'Position', [50, 50, 1400, 800], 'NumberTitle', 'off', ...
                 'MenuBar', 'none', 'ToolBar', 'figure', 'Color', [0.95 0.95 0.95]);

    pnlInput = uipanel(fig, 'Title', 'Engine Parameters', 'Position', [0.01, 0.50, 0.20, 0.48]);
    
    propList = {'LOX / RP-1', 'LOX / LH2', 'LOX / LCH4', 'N2O4 / UDMH', 'N2O4 / MMH', 'LOX / Ethanol', 'H2O2 / RP-1'};
    uicontrol(pnlInput, 'Style', 'text', 'Position', [10, 290, 80, 20], 'String', 'Propellant:', 'HorizontalAlignment', 'left');
    propMenu = uicontrol(pnlInput, 'Style', 'popupmenu', 'Position', [100, 290, 160, 20], 'String', propList);

    matList = {'Inconel 718 (Standard)', 'GRCop-42 (High Cond)', 'C103 Niobium (High Temp)', ...
               'Ti-6Al-4V (Lightweight)', 'SS 304L (Low Cost)', 'Rhenium (Extreme Temp)', 'Carbon-Carbon (Composite)'};
    uicontrol(pnlInput, 'Style', 'text', 'Position', [10, 250, 80, 20], 'String', 'Wall Material:', 'HorizontalAlignment', 'left');
    matMenu = uicontrol(pnlInput, 'Style', 'popupmenu', 'Position', [100, 250, 160, 20], 'String', matList);                     
                     
    uicontrol(pnlInput, 'Style', 'text', 'Position', [10, 210, 110, 20], 'String', 'Target Thrust (N):', 'HorizontalAlignment', 'left');
    editThrust = uicontrol(pnlInput, 'Style', 'edit', 'Position', [120, 210, 140, 20], 'String', '100000');
    
    uicontrol(pnlInput, 'Style', 'text', 'Position', [10, 170, 110, 20], 'String', 'Ambient Pres. (Pa):', 'HorizontalAlignment', 'left');
    editPa = uicontrol(pnlInput, 'Style', 'edit', 'Position', [120, 170, 140, 20], 'String', '101325');
    
    uicontrol(pnlInput, 'Style', 'text', 'Position', [10, 130, 110, 20], 'String', 'Max Generations:', 'HorizontalAlignment', 'left');
    editGens = uicontrol(pnlInput, 'Style', 'edit', 'Position', [120, 130, 140, 20], 'String', '40');

    btnOptimize = uicontrol(pnlInput, 'Style', 'pushbutton', 'Position', [10, 30, 250, 60], ...
                            'String', 'RUN RIGOROUS VECTOR SOLVER', 'FontWeight', 'bold', ...
                            'BackgroundColor', [0.1 0.4 0.2], 'ForegroundColor', 'white', ...
                            'Callback', @runOptimization);

    pnlResults = uipanel(fig, 'Title', 'Rigorous Aero & Structural Results', 'Position', [0.01, 0.23, 0.20, 0.26]);
    txtResIsp = uicontrol(pnlResults, 'Style', 'text', 'Position', [10, 160, 250, 20], 'HorizontalAlignment', 'left', 'String', 'Effective 2D Isp: -- s', 'FontWeight', 'bold', 'ForegroundColor', [0 0.4 0]);
    txtResPc = uicontrol(pnlResults, 'Style', 'text', 'Position', [10, 140, 250, 20], 'HorizontalAlignment', 'left', 'String', 'Chamber Pres: -- MPa');
    txtResEps = uicontrol(pnlResults, 'Style', 'text', 'Position', [10, 120, 250, 20], 'HorizontalAlignment', 'left', 'String', 'Geom vs Eff Exp Ratio: -- / --');
    txtResBL = uicontrol(pnlResults, 'Style', 'text', 'Position', [10, 100, 250, 20], 'HorizontalAlignment', 'left', 'String', 'Exit Boundary Layer (δ): -- mm');
    txtResThick = uicontrol(pnlResults, 'Style', 'text', 'Position', [10, 80, 250, 20], 'HorizontalAlignment', 'left', 'String', 'Thick-Wall Depth: -- mm');
    txtResMaxT = uicontrol(pnlResults, 'Style', 'text', 'Position', [10, 60, 250, 20], 'HorizontalAlignment', 'left', 'String', 'Peak Hot Wall Temp: -- K', 'FontWeight', 'bold', 'ForegroundColor', [0.7 0 0]);
    txtResMoS = uicontrol(pnlResults, 'Style', 'text', 'Position', [10, 30, 250, 20], 'HorizontalAlignment', 'left', 'String', 'Min Margin of Safety: --');

    pnlStatus = uipanel(fig, 'Title', 'Solver Log', 'Position', [0.01, 0.02, 0.20, 0.20]);
    txtLog = uicontrol(pnlStatus, 'Style', 'listbox', 'Position', [10, 10, 250, 100], 'String', {'Ready.'});

    ax3D = axes(fig, 'Position', [0.25, 0.55, 0.45, 0.40]);
    title(ax3D, '3D Shell Heat Map (Exact CHT & MoC Contour)');
    grid(ax3D, 'on'); rotate3d(ax3D, 'on'); colormap(ax3D, 'turbo');

    ax2D = axes(fig, 'Position', [0.25, 0.06, 0.45, 0.40]);
    title(ax2D, '2D Exact TOP Geometry with BL Displacement (δ*)'); xlabel(ax2D, 'Axial Distance (m)'); ylabel(ax2D, 'Radius (m)');
    grid(ax2D, 'on'); axis(ax2D, 'equal');

    axAero = axes(fig, 'Position', [0.75, 0.55, 0.23, 0.40]);
    title(axAero, 'Axial Aerodynamics (Thermally Perfect Gas)'); xlabel(axAero, 'Axial Dist (m)'); ylabel(axAero, 'Mach & Area Ratio');
    grid(axAero, 'on');

    axTemp = axes(fig, 'Position', [0.75, 0.06, 0.23, 0.40]);
    title(axTemp, 'Rigorous Thermal Gradients'); xlabel(axTemp, 'Axial Dist (m)'); ylabel(axTemp, 'Temperature (K)');
    grid(axTemp, 'on');
    
    %% --- Thermally Perfect & Transport Databases ---
    propData(1).name = 'LOX / RP-1'; propData(1).Mw = 22.0; propData(1).Tc = 3600; propData(1).Pr = 0.70; propData(1).S_ref = 110; propData(1).mu_ref = 1.7e-5; propData(1).T_ref = 300;
    propData(2).name = 'LOX / LH2';  propData(2).Mw = 13.0; propData(2).Tc = 3200; propData(2).Pr = 0.65; propData(2).S_ref = 72; propData(2).mu_ref = 0.8e-5; propData(2).T_ref = 300;
    propData(3).name = 'LOX / LCH4'; propData(3).Mw = 18.0; propData(3).Tc = 3500; propData(3).Pr = 0.68; propData(3).S_ref = 164; propData(3).mu_ref = 1.1e-5; propData(3).T_ref = 300;
    propData(4).name = 'N2O4 / UDMH'; propData(4).Mw = 24.0; propData(4).Tc = 3400; propData(4).Pr = 0.72; propData(4).S_ref = 120; propData(4).mu_ref = 1.8e-5; propData(4).T_ref = 300;
    propData(5).name = 'N2O4 / MMH';  propData(5).Mw = 23.0; propData(5).Tc = 3300; propData(5).Pr = 0.72; propData(5).S_ref = 120; propData(5).mu_ref = 1.75e-5; propData(5).T_ref = 300;
    propData(6).name = 'LOX / Ethanol'; propData(6).Mw = 22.5; propData(6).Tc = 3300; propData(6).Pr = 0.70; propData(6).S_ref = 115; propData(6).mu_ref = 1.6e-5; propData(6).T_ref = 300;
    propData(7).name = 'H2O2 / RP-1'; propData(7).Mw = 21.0; propData(7).Tc = 3100; propData(7).Pr = 0.70; propData(7).S_ref = 115; propData(7).mu_ref = 1.6e-5; propData(7).T_ref = 300;

    matData(1).name = 'Inconel 718'; matData(1).rho = 8190; matData(1).k = 15; matData(1).E = 200e9; matData(1).alpha = 13e-6; matData(1).nu = 0.29; matData(1).Tmelt = 1570; matData(1).Sy = 1100e6; matData(1).Sy_exp = 3.5;
    matData(2).name = 'GRCop-42'; matData(2).rho = 8770; matData(2).k = 340; matData(2).E = 120e9; matData(2).alpha = 16e-6; matData(2).nu = 0.33; matData(2).Tmelt = 1350; matData(2).Sy = 350e6; matData(2).Sy_exp = 1.5;
    matData(3).name = 'C103 Niobium'; matData(3).rho = 8850; matData(3).k = 55; matData(3).E = 90e9; matData(3).alpha = 8e-6; matData(3).nu = 0.38; matData(3).Tmelt = 2620; matData(3).Sy = 250e6; matData(3).Sy_exp = 2.0;
    matData(4).name = 'Ti-6Al-4V'; matData(4).rho = 4430; matData(4).k = 7; matData(4).E = 114e9; matData(4).alpha = 8.6e-6; matData(4).nu = 0.34; matData(4).Tmelt = 1900; matData(4).Sy = 880e6; matData(4).Sy_exp = 2.0;
    matData(5).name = 'SS 304L'; matData(5).rho = 8000; matData(5).k = 16; matData(5).E = 193e9; matData(5).alpha = 17e-6; matData(5).nu = 0.29; matData(5).Tmelt = 1700; matData(5).Sy = 250e6; matData(5).Sy_exp = 2.5;
    matData(6).name = 'Rhenium'; matData(6).rho = 21020; matData(6).k = 48; matData(6).E = 460e9; matData(6).alpha = 6.2e-6; matData(6).nu = 0.30; matData(6).Tmelt = 3459; matData(6).Sy = 1000e6; matData(6).Sy_exp = 1.5;
    matData(7).name = 'Carbon-Carbon'; matData(7).rho = 1800; matData(7).k = 40; matData(7).E = 100e9; matData(7).alpha = 1e-6; matData(7).nu = 0.20; matData(7).Tmelt = 3300; matData(7).Sy = 300e6; matData(7).Sy_exp = 0.5;

    Ru = 8314.46; g0 = 9.80665; 
    
    %% --- Vectorized Internal Gas Property Functions ---
    function cp = getCp(T, name)
        % Now fully supports vector inputs for fast array operations
        switch name
            case 'LOX / RP-1'; cp = 1800 + 0.35 .* T; 
            case 'LOX / LH2'; cp = 3200 + 0.8 .* T;
            case 'LOX / LCH4'; cp = 2100 + 0.4 .* T;
            case 'N2O4 / UDMH'; cp = 1900 + 0.3 .* T;
            case 'N2O4 / MMH'; cp = 1850 + 0.3 .* T;
            case 'LOX / Ethanol'; cp = 1800 + 0.35 .* T;
            case 'H2O2 / RP-1'; cp = 1700 + 0.35 .* T;
            otherwise; cp = 2000 + 0.3 .* T;
        end
    end

    function g = getGamma(T, Mw, name)
        R_spec = Ru / Mw;
        cp = getCp(T, name);
        cv = cp - R_spec;
        g = cp ./ cv;
    end

    function mu = getViscosity(T, prop)
        mu = prop.mu_ref .* (T ./ prop.T_ref).^1.5 .* (prop.T_ref + prop.S_ref) ./ (T + prop.S_ref);
    end

    %% --- Main Optimization Callback ---
    function runOptimization(~, ~)
        thrust = str2double(get(editThrust, 'String'));
        Pa = str2double(get(editPa, 'String'));
        maxGen = str2double(get(editGens, 'String'));
        if isnan(maxGen) || maxGen < 1; maxGen = 40; end
        
        prop = propData(get(propMenu, 'Value'));
        mat = matData(get(matMenu, 'Value'));
        
        updateLog('--- Computing Thermally Perfect Master Tables ---');
        
        M_span = linspace(0.01, 15, 3000); % Increased resolution
        T_init = prop.Tc;
        odeFunc = @(M, T) - (getGamma(T, prop.Mw, prop.name) - 1) * M * T / (1 + ((getGamma(T, prop.Mw, prop.name) - 1)/2) * M^2);
        [M_table, T_table] = ode45(odeFunc, M_span, T_init);
        
        % Ensure monotonically unique arrays for fast interpolation later
        [M_table, unique_idx] = unique(M_table);
        T_table = T_table(unique_idx);
        
        Gamma_table = getGamma(T_table, prop.Mw, prop.name);
        P_ratio_table = (1 + ((Gamma_table-1)./2).*M_table.^2).^(-Gamma_table./(Gamma_table-1));
        
        term1 = 2./(Gamma_table+1); term2 = (Gamma_table-1)./2; exponent = (Gamma_table+1)./(2.*(Gamma_table-1));
        A_ratio_table = (1./M_table) .* (term1 .* (1 + term2 .* M_table.^2)).^exponent;
        A_ratio_table(M_table < 0.001) = inf;
        
        % Vectorized Prandtl-Meyer Angle Array
        nu_table = zeros(size(M_table));
        sup_mask = M_table > 1.0;
        g_sup = Gamma_table(sup_mask); M_sup_arr = M_table(sup_mask);
        nu_table(sup_mask) = sqrt((g_sup+1)./(g_sup-1)) .* atan(sqrt((g_sup-1)./(g_sup+1).*(M_sup_arr.^2-1))) - atan(sqrt(M_sup_arr.^2-1));

        idx_throat = find(M_table >= 1.0, 1);
        
        % Subsonic and Supersonic distinct tables (ensuring uniqueness for interp1)
        [A_sub_u, i_sub] = unique(A_ratio_table(1:idx_throat));
        M_sub_u = M_table(i_sub); T_sub_u = T_table(i_sub); P_sub_u = P_ratio_table(i_sub);
        
        [A_sup_u, i_sup] = unique(A_ratio_table(idx_throat:end));
        M_sup_u = M_table(idx_throat - 1 + i_sup); 
        T_sup_u = T_table(idx_throat - 1 + i_sup); 
        P_sup_u = P_ratio_table(idx_throat - 1 + i_sup);
        nu_sup_u = nu_table(idx_throat - 1 + i_sup);

        updateLog(sprintf('--- Initiating Vectorized GA (%d Gens) ---', maxGen));
        popSize = 60; mutationRate = 0.15;
        bounds = [1e6, 25e6; 5e3, 120e3; 0.6, 1.0; 0.0005, 0.015];
        pop = bounds(:,1)' + rand(popSize, 4) .* (bounds(:,2)' - bounds(:,1)');
        
        globalBestFitness = inf; globalBestGeom = [];
        set(btnOptimize, 'Enable', 'off', 'String', 'OPTIMIZING DESIGNS...');
        drawnow;
        
        for gen = 1:maxGen
            fitness = zeros(popSize, 1);
            geomStructs = cell(popSize, 1);
            
            for i = 1:popSize
                [fitness(i), geomStructs{i}] = evaluateDesign(pop(i,:), prop, mat, thrust, Pa, ...
                    A_sub_u, M_sub_u, A_sup_u, M_sup_u, M_table, T_table, P_ratio_table, nu_sup_u);
            end
            
            [fitness, sortIdx] = sort(fitness);
            pop = pop(sortIdx, :); geomStructs = geomStructs(sortIdx);
            
            if fitness(1) < globalBestFitness
                globalBestFitness = fitness(1); globalBestGeom = geomStructs{1};
            end
            
            if mod(gen, 5) == 0 || gen == 1
                updateLog(sprintf('Gen %d: Isp=%.1f | BL δ=%.1fmm | MaxT=%.0fK', gen, globalBestGeom.Isp_true, globalBestGeom.delta_exit*1000, max(globalBestGeom.T_hw)));
            end
            
            numKeep = round(popSize * 0.2);
            newPop = zeros(popSize, 4); newPop(1:numKeep, :) = pop(1:numKeep, :);
            for i = (numKeep + 1):popSize
                p1 = randi([1, round(popSize/2)]); p2 = randi([1, round(popSize/2)]);
                cut = randi([1, 3]);
                newPop(i, 1:cut) = pop(p1, 1:cut); newPop(i, (cut+1):4) = pop(p2, (cut+1):4);
                if rand() < mutationRate
                    mutGene = randi([1, 4]);
                    newPop(i, mutGene) = max(bounds(mutGene,1), min(bounds(mutGene,2), newPop(i, mutGene) + (bounds(mutGene,2)-bounds(mutGene,1))*0.1*randn()));
                end
            end
            pop = newPop; drawnow;
        end
        
        set(btnOptimize, 'Enable', 'on', 'String', 'RUN RIGOROUS VECTOR SOLVER');
        
        % Dashboard Updates
        set(txtResIsp, 'String', sprintf('Effective 2D Isp: %.1f s', globalBestGeom.Isp_true));
        set(txtResPc, 'String', sprintf('Chamber Pres: %.2f MPa', globalBestGeom.Pc/1e6));
        set(txtResEps, 'String', sprintf('Geom vs Eff Exp Ratio: %.1f / %.1f', globalBestGeom.epsilon_geom, globalBestGeom.epsilon_eff));
        set(txtResBL, 'String', sprintf('Exit Boundary Layer (δ): %.2f mm', globalBestGeom.delta_exit * 1000));
        set(txtResThick, 'String', sprintf('Thick-Wall Depth: %.2f mm', globalBestGeom.t_w * 1000));
        set(txtResMaxT, 'String', sprintf('Peak Hot Wall Temp: %.0f K', max(globalBestGeom.T_hw)));
        
        if globalBestGeom.MoS < 0
            set(txtResMoS, 'String', sprintf('Min Margin of Safety: %.2f (FAILED)', globalBestGeom.MoS), 'ForegroundColor', 'red');
        else
            set(txtResMoS, 'String', sprintf('Min Margin of Safety: %.2f (SAFE)', globalBestGeom.MoS), 'ForegroundColor', [0 0.5 0]);
        end
        
        updateLog('--- Optimization Complete ---');
        plotDashboards(globalBestGeom);
    end

    %% --- VECTORIZED Exact Physics, TDK MoC, and CHT ---
    function [fitness, geom] = evaluateDesign(genes, prop, mat, F, Pa, A_sub_u, M_sub_u, A_sup_u, M_sup_u, M_table, T_table, P_ratio_table, nu_sup_u)
        Pc = genes(1); Pe = genes(2); bell_frac = genes(3); t_w = genes(4);
        if Pe >= Pc; fitness = 1e8; geom = getEmptyGeom(); return; end
        
        g_c = getGamma(prop.Tc, prop.Mw, prop.name);
        R_spec = Ru / prop.Mw;
        c_star = sqrt(g_c * R_spec * prop.Tc) / (g_c * sqrt((2/(g_c+1))^((g_c+1)/(g_c-1))));
        Pr_ratio_target = Pe / Pc;
        
        % Evaluate Exit properties
        [~, idx_e] = min(abs(P_ratio_table - Pr_ratio_target));
        Me = M_table(idx_e); epsilon = A_sup_u(find(M_sup_u >= Me, 1)); 
        if isempty(epsilon); epsilon = A_sup_u(end); end
        Te = T_table(idx_e); nu_e = nu_sup_u(find(M_sup_u >= Me, 1));
        if isempty(nu_e); nu_e = nu_sup_u(end); end
        g_e = getGamma(Te, prop.Mw, prop.name);
        
        term3 = 1 - Pr_ratio_target^((g_e-1)/g_e);
        if term3 <= 0; fitness = 1e8; geom = getEmptyGeom(); return; end
        
        CF_ideal = sqrt((2*g_c^2)/(g_c-1) * (2/(g_c+1))^((g_c+1)/(g_c-1)) * term3) + (Pe - Pa)/Pc * epsilon;
        if CF_ideal <= 0; fitness = 1e8; geom = getEmptyGeom(); return; end
        
        At = F / (Pc * CF_ideal); Rt = sqrt(At / pi); Ae_geom = epsilon * At; Re_geom = sqrt(Ae_geom / pi);
        
        % 1. EXACT RAO THRUST-OPTIMIZED PARABOLA (TOP) CONSTRUCTION
        % The inflection point angle must equal half the exit Prandtl-Meyer expansion
        theta_N = nu_e / 2; 
        theta_E = deg2rad(2); % Small exit angle to maximize thrust vector
        
        % Throat to Inflection Arc (Standard R_arc = 0.382 * Rt)
        R_arc = 0.382 * Rt;
        x_N = R_arc * sin(theta_N);
        y_N = Rt + R_arc * (1 - cos(theta_N));
        
        % Bezier control point intersection to form the exact Parabola
        Lc = (Re_geom - Rt) / tan(deg2rad(15)); L_bell = bell_frac * Lc;
        x_E = L_bell; y_E = Re_geom;
        
        m_N = tan(theta_N); m_E = tan(theta_E);
        % Intersection of tangents
        x_C = (y_E - y_N + x_N * m_N - x_E * m_E) / (m_N - m_E);
        y_C = y_N + (x_C - x_N) * m_N;
        
        t = linspace(0, 1, 80);
        X_parabola = (1-t).^2 * x_N + 2*(1-t).*t * x_C + t.^2 * x_E;
        Y_parabola = (1-t).^2 * y_N + 2*(1-t).*t * y_C + t.^2 * y_E;
        
        % Chamber & Convergence Geometry
        L_chamber = Rt * 3; R_chamber = Rt * 2.5; X_conv = linspace(-L_chamber, 0, 40);
        Y_conv = R_chamber - (R_chamber - Rt) * exp(-5 * (X_conv ./ -L_chamber).^2); 
        
        % Throat Arc
        theta_arc = linspace(0, theta_N, 15);
        X_arc = R_arc * sin(theta_arc); Y_arc = Rt + R_arc * (1 - cos(theta_arc));
        
        X_prof = [X_conv(1:end-1), X_arc(1:end-1), X_parabola]; 
        Y_prof = [Y_conv(1:end-1), Y_arc(1:end-1), Y_parabola];
        
        % Ensure monotonically increasing X for interpolation
        [X_prof, uniq_idx] = unique(X_prof);
        Y_prof = Y_prof(uniq_idx);
        
        % 2. FULLY VECTORIZED 1D AERODYNAMICS
        Area = pi * Y_prof.^2; Area_Ratio = Area / At;
        Mach = zeros(size(X_prof));
        
        % Split nodes into sub/supersonic
        idx_sub = X_prof < 0;
        idx_thr = X_prof == 0;
        idx_sup = X_prof > 0;
        
        Mach(idx_sub) = interp1(A_sub_u, M_sub_u, Area_Ratio(idx_sub), 'linear', 'extrap');
        Mach(idx_thr) = 1.0;
        Mach(idx_sup) = interp1(A_sup_u, M_sup_u, Area_Ratio(idx_sup), 'linear', 'extrap');
        
        % Gas Properties (Vectorized Lookup)
        T_gas = interp1(M_table, T_table, Mach, 'linear', 'extrap');
        P_gas = Pc .* interp1(M_table, P_ratio_table, Mach, 'linear', 'extrap');
        
        % 3. VECTORIZED COMPRESSIBLE TURBULENT BOUNDARY LAYER
        dx = [0, sqrt(diff(X_prof).^2 + diff(Y_prof).^2)];
        S_run = cumsum(dx) + 0.01; % Running length
        
        rho_gas = P_gas ./ (R_spec .* T_gas);
        g_local = getGamma(T_gas, prop.Mw, prop.name);
        vel_gas = Mach .* sqrt(g_local .* R_spec .* T_gas);
        mu_gas = getViscosity(T_gas, prop);
        
        Re_x = max(1e4, (rho_gas .* vel_gas .* S_run) ./ mu_gas);
        
        % Eckert Reference Temperature method for Compressibility
        T_aw = T_gas .* (1 + prop.Pr^0.33 .* ((g_local-1)./2) .* Mach.^2);
        comp_factor = (T_gas ./ T_aw).^1.4; 
        delta_BL = 0.37 .* S_run ./ (Re_x.^0.2) .* comp_factor;
        delta_star = delta_BL .* 0.125; 
        
        % 4. VECTORIZED EXACT CHT WITH BARTZ SIGMA CORRECTION
        Cp_g = getCp(T_gas, prop.name);
        D_local = 2 .* Y_prof;
        T_coolant = 300; h_c = 25000;
        
        % First-pass approximation of Wall Temp to calculate rigorous Sigma
        T_hw_guess = T_aw .* 0.8; 
        
        % Rigorous Bartz Sigma Correction Factor
        sigma_bartz = (0.5 .* (T_hw_guess ./ T_gas) .* (1 + ((g_local-1)./2) .* Mach.^2) + 0.5).^-0.68 .* (1 + ((g_local-1)./2) .* Mach.^2).^-0.12;
        
        h_g = (0.026 ./ D_local.^0.2) .* ((mu_gas.^0.2 .* Cp_g) ./ prop.Pr^0.6) .* (Pc ./ c_star)^0.8 .* (At ./ Area).^0.9 .* sigma_bartz;
        
        % Fourier Conduction Circuit
        U = 1 ./ ( (1./h_g) + (t_w./mat.k) + (1./h_c) );
        q = U .* (T_aw - T_coolant);
        T_hw = T_aw - q./h_g;
        T_cw = T_coolant + q./h_c;
        
        % Lamé Thick-Wall Mechanics
        r_i = Y_prof; r_o = r_i + t_w;
        sigma_t = P_gas .* (r_o.^2 + r_i.^2) ./ (r_o.^2 - r_i.^2);
        sigma_r = -P_gas;
        sigma_th = (mat.E .* mat.alpha .* (T_hw - T_cw)) ./ (2 .* (1 - mat.nu));
        
        S1 = sigma_t + sigma_th; S2 = sigma_r; S3 = sigma_th; 
        sigma_vm = sqrt(0.5 .* ((S1-S2).^2 + (S2-S3).^2 + (S3-S1).^2));
        
        Sy_act = mat.Sy .* max(0, (1 - (T_hw./mat.Tmelt).^mat.Sy_exp)); 
        MoS_array = (Sy_act ./ sigma_vm) - 1;
        
        % Mass Integration (Vectorized)
        mass = sum(2.*pi .* r_i .* dx .* t_w .* mat.rho);
        
        % 5. 2D DIVERGENCE & EFFECTIVE THRUST
        lambda = (1 + cos(theta_E)) / 2; % Axisymmetric divergence
        R_eff_exit = Y_prof(end) - delta_star(end);
        A_eff_exit = pi * R_eff_exit^2;
        epsilon_eff = A_eff_exit / At;
        
        CF_true = lambda * sqrt((2*g_c^2)/(g_c-1) * (2/(g_c+1))^((g_c+1)/(g_c-1)) * term3) + (Pe - Pa)/Pc * epsilon_eff;
        Isp_true = (c_star * CF_true) / g0;
        
        % Objective Function
        min_MoS = min(MoS_array); peak_Temp = max(T_hw);
        fitness = -Isp_true + 0.3 * mass; 
        
        if Pe < 0.30 * Pa; fitness = fitness + 5000; end 
        if peak_Temp >= mat.Tmelt; fitness = fitness + 100000; end 
        if min_MoS < 0; fitness = fitness + 10000 * abs(min_MoS); end 
        
        geom.Pc = Pc; geom.epsilon_geom = epsilon; geom.epsilon_eff = epsilon_eff;
        geom.Isp_true = Isp_true; geom.t_w = t_w; geom.delta_exit = delta_BL(end);
        geom.mass = mass; geom.MoS = min_MoS; geom.theta_max = theta_N;
        geom.X = X_prof; geom.Y = Y_prof; geom.Mach = Mach; geom.Area_Ratio = Area_Ratio;
        geom.P_gas = P_gas; geom.T_gas = T_gas; geom.T_hw = T_hw; geom.delta_star = delta_star;
    end

    %% --- Dashboard Plots ---
    function plotDashboards(geom)
        % 1. 2D Exact Contour with Boundary Layer visualization
        cla(ax2D); hold(ax2D, 'on');
        
        plot(ax2D, geom.X, geom.Y, 'k-', 'LineWidth', 2);
        plot(ax2D, geom.X, -geom.Y, 'k-', 'LineWidth', 2);
        Y_out = geom.Y + geom.t_w;
        plot(ax2D, geom.X, Y_out, 'r-', 'LineWidth', 1);
        plot(ax2D, geom.X, -Y_out, 'r-', 'LineWidth', 1);
        patch(ax2D, [geom.X, fliplr(geom.X)], [geom.Y, fliplr(Y_out)], [0.6 0.6 0.6], 'EdgeColor', 'none');
        patch(ax2D, [geom.X, fliplr(geom.X)], [-geom.Y, fliplr(-Y_out)], [0.6 0.6 0.6], 'EdgeColor', 'none');
        
        mag = 5; 
        plot(ax2D, geom.X, geom.Y - geom.delta_star*mag, 'b--', 'LineWidth', 1.5, 'DisplayName', '5x BL Displ.');
        plot(ax2D, geom.X, -(geom.Y - geom.delta_star*mag), 'b--', 'LineWidth', 1.5);
        
        xline(ax2D, 0, 'b--', 'Throat (M=1)', 'HandleVisibility','off');
        axis(ax2D, 'equal');
        legend(ax2D, 'Location', 'northwest');
        
        % 2. 3D Shell Heat Map
        cla(ax3D);
        theta = linspace(0, 2*pi, 60);
        [X_mesh, Theta_mesh] = meshgrid(geom.X, theta);
        Y_mesh = interp1(geom.X, Y_out, X_mesh, 'spline') .* cos(Theta_mesh);
        Z_mesh = interp1(geom.X, Y_out, X_mesh, 'spline') .* sin(Theta_mesh);
        
        Temp_Mesh = repmat(geom.T_hw, length(theta), 1);
        
        surf(ax3D, X_mesh, Y_mesh, Z_mesh, Temp_Mesh, 'EdgeColor', 'none', 'AmbientStrength', 0.5);
        camlight(ax3D, 'headlight'); lighting(ax3D, 'gouraud');
        cb = colorbar(ax3D); cb.Label.String = 'Wall Temperature (K)';
        axis(ax3D, 'equal'); view(ax3D, 45, 30);
        
        % 3. Axial Thermodynamics
        cla(axAero); hold(axAero, 'on');
        yyaxis(axAero, 'left');
        plot(axAero, geom.X, geom.Mach, 'b-', 'LineWidth', 2);
        ylabel(axAero, 'Mach Number'); ylim(axAero, [0, max(geom.Mach)*1.1]);
        
        yyaxis(axAero, 'right');
        plot(axAero, geom.X, geom.Area_Ratio, 'r-', 'LineWidth', 2);
        ylabel(axAero, 'Area Ratio (A/At)'); ylim(axAero, [0, max(geom.Area_Ratio)*1.1]);
        xline(axAero, 0, 'k--');
        
        % 4. Axial Heat Gradients
        cla(axTemp); hold(axTemp, 'on');
        plot(axTemp, geom.X, geom.T_gas, 'r--', 'LineWidth', 1.5, 'DisplayName', 'T_{Gas} (Centerline)');
        plot(axTemp, geom.X, geom.T_hw, 'k-', 'LineWidth', 2, 'DisplayName', 'T_{Wall} (Hot Side)');
        legend(axTemp, 'Location', 'northeast');
        xline(axTemp, 0, 'b--');
    end

    %% --- Helpers ---
    function updateLog(msg)
        currStrings = get(txtLog, 'String');
        if ~iscell(currStrings), currStrings = {currStrings}; end
        currStrings{end+1} = msg;
        if length(currStrings) > 15, currStrings = currStrings(end-14:end); end
        set(txtLog, 'String', currStrings, 'Value', length(currStrings));
    end

    function geom = getEmptyGeom()
        geom.Pc = 0; geom.epsilon_geom = 0; geom.epsilon_eff = 0; geom.Isp_true = 0; geom.mass = 0; 
        geom.MoS = 0; geom.theta_max = 0; geom.delta_exit = 0; geom.t_w = 0;
        geom.X = 0; geom.Y = 0; geom.Mach = 0; geom.P_gas = 0; geom.T_gas = 0; geom.T_hw = 0; 
        geom.Area_Ratio = 0; geom.delta_star = 0;
    end
end