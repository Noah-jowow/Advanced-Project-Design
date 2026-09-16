import numpy as np

# Ensure prop_core is built and installed, or accessible
from api_gateway.optimization import optimize_nozzle

def precompute_grid():
    print("Starting offline grid pre-computation for initial guesses...")
    
    # Simple coarse grid to build our dictionary
    thrusts = [100e3, 500e3, 2000e3]
    pas = [1000, 50000, 101325]
    prop_indices = [0, 1, 2]
    mat_indices = [0, 1]
    
    total = len(thrusts) * len(pas) * len(prop_indices) * len(mat_indices)
    count = 0
    
    for prop_idx in prop_indices:
        for mat_idx in mat_indices:
            for thrust in thrusts:
                for pa in pas:
                    count += 1
                    print(f"[{count}/{total}] Computing Prop={prop_idx}, Mat={mat_idx}, Thrust={thrust/1000}kN, Pa={pa}Pa")
                    
                    try:
                        res = optimize_nozzle(prop_idx, mat_idx, thrust, pa, num_angles=10)
                        # We would append the successful runs to the JSON database
                        _ = {
                            "prop_idx": prop_idx,
                            "mat_idx": mat_idx,
                            "thrust": thrust,
                            "pa": pa,
                            "x_opt": [res["Pc"], res.get("Pe", 50000), 1.0, res["t_w"]] + [np.radians(15.0)]*10
                        }
                    except Exception as e:
                        print(f"Failed: {e}")
                        
if __name__ == "__main__":
    print("Run this script manually to build the JSON database.")
