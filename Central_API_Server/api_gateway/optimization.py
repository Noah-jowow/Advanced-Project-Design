import os
import json
import numpy as np
from scipy.optimize import minimize
import prop_core

PRECOMPUTE_DB_PATH = os.path.join(os.path.dirname(__file__), "precomputed_guesses.json")

def load_precomputed_guess(thrust, pa, prop_idx, mat_idx, num_angles=20):
    if os.path.exists(PRECOMPUTE_DB_PATH):
        try:
            with open(PRECOMPUTE_DB_PATH, "r") as f:
                db = json.load(f)
            
            best_dist = float('inf')
            best_guess = None
            
            for entry in db:
                if entry['prop_idx'] == prop_idx and entry['mat_idx'] == mat_idx:
                    dt = (entry['thrust'] - thrust) / max(thrust, 1)
                    dp = (entry['pa'] - pa) / max(pa, 1)
                    dist = dt*dt + dp*dp
                    if dist < best_dist:
                        best_dist = dist
                        best_guess = entry['x_opt']
            
            if best_guess is not None:
                if len(best_guess) == 4 + num_angles:
                    return best_guess
        except Exception as e:
            print(f"Warning: Failed to load precomputed guess: {e}")
            
    x0 = [5e6, max(pa * 1.5, 10000), 0.8, 0.005]
    theta_max = np.radians(30.0)
    for i in range(num_angles):
        t = i / (num_angles - 1)
        theta = theta_max * (1.0 - t)**1.5
        x0.append(theta)
        
    return x0

def optimize_nozzle(prop_idx, mat_idx, thrust, pa, num_angles=20, log_callback=None, telemetry_callback=None, config=None):
    optimizer = prop_core.NozzleOptimizer()
    prop = prop_core.NozzleOptimizer.get_propellant_data(prop_idx)
    mat = prop_core.NozzleOptimizer.get_material_data(mat_idx)
    
    if config is None:
        config = {}
    min_mos = float(config.get("minMoS", 0.10))
    max_temp_ratio = float(config.get("maxTempRatio", 0.90))
    max_length = float(config.get("maxLength", 2.5))
    max_exit_radius = float(config.get("maxExitRadius", 1.0))
    max_pump_pressure = float(config.get("maxPumpPressure", 25e6))
    sep_margin = float(config.get("sepMargin", 0.35))
    pc_min = float(config.get("pcMin", 1e6))
    pc_max = float(config.get("pcMax", 20e6))
    tw_min = float(config.get("twMin", 0.001))
    tw_max = float(config.get("twMax", 0.020))
    max_iter = max(5, min(int(config.get("maxIter", 50)), 200))
    
    def log(msg: str):
        if log_callback:
            try:
                log_callback(msg)
            except Exception:
                pass
        else:
            print(msg)

    log(f"[PROP_PARAMS] Configuration loaded: Thrust={thrust:,.0f} N, Pa={pa:,.0f} Pa | Propellant={prop.name}, Material={mat.name}")
    log(f"[PROP_PARAMS] User Active Constraints: Min MoS >= {min_mos:.2f}, Max Wall Temp <= {mat.Tmelt*max_temp_ratio:.1f} K ({max_temp_ratio*100:.0f}% Tmelt), Max L <= {max_length:.2f} m, Max R <= {max_exit_radius:.2f} m, Pump Head <= {max_pump_pressure/1e5:.0f} bar")
    log(f"[PROP_CORE] Initializing SLSQP Inverse Nozzle Optimization with {num_angles} characteristic rays...")

    SCALE_PC = 1e6   # Chamber pressure in MPa
    SCALE_PE = 1e4   # Exit pressure in 10 kPa
    SCALE_TW = 1e-3  # Wall thickness in mm

    def decode_vector(u):
        genes = np.array([u[0] * SCALE_PC, u[1] * SCALE_PE, u[2], u[3] * SCALE_TW], dtype=np.float64)
        thetas = np.array(u[4:], dtype=np.float64)
        return genes, thetas

    x0 = load_precomputed_guess(thrust, pa, prop_idx, mat_idx, num_angles)
    x0[0] = np.clip(x0[0], pc_min, pc_max)
    x0[3] = np.clip(x0[3], tw_min, tw_max)
    
    # Normalized initial vector
    u0 = [x0[0] / SCALE_PC, x0[1] / SCALE_PE, x0[2], x0[3] / SCALE_TW] + list(x0[4:])
    log(f"[PROP_CORE] Normalized seed vector: Pc={u0[0]:.2f} MPa, Pe={u0[1]*10:.1f} kPa, L_f={u0[2]:.2f}, t_w={u0[3]:.1f} mm")
    
    def objective(u):
        genes, thetas = decode_vector(u)
        geom = optimizer.evaluate_design_angles(genes, thetas, prop, mat, thrust, pa)
        if geom.Isp_true <= 0:
            return 1e6
        # Multidisciplinary objective: Maximize delivered Isp with structural mass penalty
        return -(geom.Isp_true - 20.0 * geom.mass)

    def constraint_structural(u):
        genes, thetas = decode_vector(u)
        geom = optimizer.evaluate_design_angles(genes, thetas, prop, mat, thrust, pa)
        return geom.MoS - min_mos

    def constraint_flow_separation(u):
        Pe = u[1] * SCALE_PE
        return (Pe - (sep_margin * pa)) / max(sep_margin * pa, 1.0)
        
    def constraint_heat_flux(u):
        genes, thetas = decode_vector(u)
        geom = optimizer.evaluate_design_angles(genes, thetas, prop, mat, thrust, pa)
        return ((mat.Tmelt * max_temp_ratio) - geom.T_hw_max) / 500.0
        
    def constraint_exit_radius(u):
        genes, thetas = decode_vector(u)
        geom = optimizer.evaluate_design_angles(genes, thetas, prop, mat, thrust, pa)
        r_exit = geom.y[-1] if len(geom.y) > 0 else 100.0
        return (max_exit_radius - r_exit) / max(max_exit_radius, 0.01)

    def constraint_max_length(u):
        genes, thetas = decode_vector(u)
        geom = optimizer.evaluate_design_angles(genes, thetas, prop, mat, thrust, pa)
        x_exit = geom.x[-1] if len(geom.x) > 0 else 100.0
        return (max_length - x_exit) / max(max_length, 0.01)

    def constraint_pump_pressure(u):
        genes, thetas = decode_vector(u)
        geom = optimizer.evaluate_design_angles(genes, thetas, prop, mat, thrust, pa)
        return (max_pump_pressure - (1.25 * geom.Pc + geom.delta_p_cool)) / max(max_pump_pressure, 1.0)

    bounds = [
        (pc_min / SCALE_PC, pc_max / SCALE_PC),      
        (max(5000.0, sep_margin * pa * 0.9) / SCALE_PE, 300000.0 / SCALE_PE),   
        (0.3, 1.4),       
        (tw_min / SCALE_TW, tw_max / SCALE_TW)     
    ]
    for _ in range(num_angles):
        bounds.append((0.0, 0.785))

    constraints = [
        {'type': 'ineq', 'fun': constraint_structural},
        {'type': 'ineq', 'fun': constraint_flow_separation},
        {'type': 'ineq', 'fun': constraint_heat_flux},
        {'type': 'ineq', 'fun': constraint_exit_radius},
        {'type': 'ineq', 'fun': constraint_max_length},
        {'type': 'ineq', 'fun': constraint_pump_pressure}
    ]
    
    for i in range(num_angles - 1):
        def monotonic_constraint(u, idx=i):
            return u[4 + idx] - u[4 + idx + 1]
        constraints.append({'type': 'ineq', 'fun': monotonic_constraint})

    iter_counter = {"count": 0}

    def iteration_callback(uk):
        iter_counter["count"] += 1
        k = iter_counter["count"]
        try:
            genes, thetas = decode_vector(uk)
            g = optimizer.evaluate_design_angles(genes, thetas, prop, mat, thrust, pa)
            log(f"[PROP_CORE] Iteration {k:02d} | Delivered Isp: {g.Isp_true:.2f} s | Mass: {g.mass:.2f} kg | MoS: {g.MoS:.3f} | T_hw: {g.T_hw_max:.1f} K | Pc: {g.Pc/1e5:.2f} bar | dP_cool: {g.delta_p_cool/1e5:.1f} bar")
            if telemetry_callback:
                telemetry_callback({
                    "status": "optimizing",
                    "iter": k,
                    "isp": float(g.Isp_true),
                    "mass": float(g.mass),
                    "mos": float(g.MoS),
                    "pc": float(g.Pc),
                    "pe": float(genes[1]),
                    "t_hw_max": float(g.T_hw_max),
                    "delta_p_cool": float(g.delta_p_cool),
                    "epsilon_eff": float(g.epsilon_eff),
                    "delta_exit": float(g.delta_exit),
                    "q_max": float(g.q_max)
                })
        except Exception as e:
            log(f"[PROP_CORE] Iteration {k:02d} callback warning: {e}")

    log(f"[PROP_CORE] Executing SciPy SLSQP Solver (MaxIter={max_iter}, Tolerance=1e-4)...")
    res = minimize(
        objective,
        x0=u0,
        method='SLSQP',
        bounds=bounds,
        constraints=constraints,
        callback=iteration_callback,
        options={'maxiter': max_iter, 'disp': False}
    )
    
    log(f"[PROP_CORE] SLSQP Complete: {res.message} | Total Iterations: {res.nit} | Function Evals: {res.nfev}")
    
    genes, thetas = decode_vector(res.x)
    geom = optimizer.evaluate_design_angles(genes, thetas, prop, mat, thrust, pa)
    log(f"[PROP_CORE] Converged Solution: Delivered Isp={geom.Isp_true:.2f} s, Mass={geom.mass:.2f} kg, MoS={geom.MoS:.3f}, Epsilon_eff={geom.epsilon_eff:.2f}, Peak Q={geom.q_max/1e6:.2f} MW/m^2")
    
    return {
        "x": list(geom.x),
        "y": list(geom.y),
        "mach": list(geom.mach),
        "pressure": list(geom.pressure),
        "temperature": list(geom.temperature),
        "T_hw": list(geom.T_hw),
        "margin_of_safety": list(geom.margin_of_safety),
        "q_flux": list(geom.q_flux),
        "delta_p_cool_dist": list(geom.delta_p_cool_dist),
        "delta_star": list(geom.delta_star),
        "Isp_true": geom.Isp_true,
        "thrust_delivered": geom.thrust_delivered,
        "mass": geom.mass,
        "MoS": geom.MoS,
        "epsilon_geom": geom.epsilon_geom,
        "epsilon_eff": geom.epsilon_eff,
        "delta_exit": geom.delta_exit,
        "lambda_div": geom.lambda_div,
        "delta_p_cool": geom.delta_p_cool,
        "q_max": geom.q_max,
        "T_hw_max": geom.T_hw_max,
        "Pc": geom.Pc,
        "t_w": geom.t_w,
        "max_length": max_length,
        "max_exit_radius": max_exit_radius,
        "min_mos": min_mos,
        "max_temp_ratio": max_temp_ratio,
        "max_iter": max_iter,
        "nit": int(res.nit)
    }

