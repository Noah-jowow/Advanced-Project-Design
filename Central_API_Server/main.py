from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import os
import asyncio
import json
import random
import sys
import numpy as np
import concurrent.futures
import multiprocessing
from api_gateway import mesher

app = FastAPI(title="Advanced Projects System of Systems")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- WebSocket Infrastructure ---
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def send_json(self, data: dict, websocket: WebSocket):
        await websocket.send_json(data)

    async def log(self, message: str, websocket: WebSocket):
        print(message)
        try:
            await self.send_json({"type": "log", "message": message}, websocket)
        except Exception:
            pass

manager = ConnectionManager()

# --- Core Logic Path Setup ---
def setup_core_paths():
    root = os.path.dirname(os.path.dirname(__file__))
    paths = [
        os.path.join(root, "Radar_Systems", "build"),
        os.path.join(root, "Aerodynamics_CFD", "build"),
        os.path.join(root, "Propulsion", "build")
    ]
    for p in paths:
        if os.path.exists(p) and p not in sys.path:
            sys.path.append(p)
    
    if os.name == 'nt':
        # ==============================================================================
        # DEVOPS NOTICE: PRE-DOCKERIZATION DLL PATH RESOLUTION
        # ==============================================================================
        # We are explicitly hardcoding the WinLibs MinGW64 bin directory.
        # WHY: The C++ Pybind11 extensions (aero_core, prop_core, radar_core) rely on
        # libstdc++-6.dll and libgcc_s_seh-1.dll. If these are not explicitly loaded 
        # via os.add_dll_directory() on Python 3.8+ Windows environments, the imports 
        # will silently fail, causing WebSocket disconnects and broken UI run buttons.
        #
        # ACTION REQUIRED FOR DEVOPS: 
        # When transitioning to the final Dockerized Alpine/Ubuntu containers, this 
        # entire Windows-specific block must be removed as Linux handles shared object
        # (.so) linking intrinsically via LD_LIBRARY_PATH.
        # ==============================================================================
        mingw_bin = r"C:\Users\noahj\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin"
        
        if os.path.exists(mingw_bin):
            try:
                os.add_dll_directory(mingw_bin)
                if multiprocessing.current_process().name == 'MainProcess':
                    print(f"[SYSTEM] Hardware Redundancy: Verified MinGW DLL path loaded successfully: {mingw_bin}")
            except Exception as e:
                print(f"[WARNING] Could not add DLL directory. C++ cores may fail to load: {e}")
        else:
            print("[CRITICAL WARNING] Hardcoded MinGW path not found! C++ Extensions (.pyd) will fail to import.")


setup_core_paths()

try:
    import radar_core
    RADAR_AVAILABLE = True
except ImportError:
    RADAR_AVAILABLE = False

try:
    import aero_core
    AERO_AVAILABLE = True
except ImportError:
    AERO_AVAILABLE = False

try:
    import prop_core
    PROP_AVAILABLE = True
except ImportError:
    PROP_AVAILABLE = False

# Future microservice routers will be imported here
from api_gateway.radar_router import router as radar_router, radar_process_command
from api_gateway.prop_router import router as prop_router
app.include_router(radar_router, prefix="/api/radar")
app.include_router(prop_router, prefix="/api/prop")

@app.get("/api/health")
def health_check():
    return {
        "status": "online", 
        "message": "Advanced Projects System of Systems is running.",
        "cores": {
            "radar": RADAR_AVAILABLE,
            "aero": AERO_AVAILABLE,
            "prop": PROP_AVAILABLE
        }
    }

@app.websocket("/ws/stream/{domain}")
async def websocket_endpoint(websocket: WebSocket, domain: str):
    await manager.connect(websocket)
    try:
        # State Initialization
        if domain == "radar" and RADAR_AVAILABLE:
            dsp = radar_core.RadarDSP(16, 512)
            dsp.generateMatchedFilter(1e6, 1e-5, 10e6)
            tracker = radar_core.TrackerIMM()
            # Initialize a track at (1000, 100, 5000)
            initial_state = np.array([1000.0, 100.0, 5000.0, 300.0, 0.1, 0.0, 0.01])
            initial_cov = np.eye(7) * 10.0
            tracker.add_track(1, initial_state, initial_cov)
            t_last = asyncio.get_event_loop().time()

        
        if domain == "aero" and AERO_AVAILABLE:
            cfg = aero_core.SolverConfig()
            cfg.mach = 0.8
            cfg.use_ddes = True
            solver = aero_core.AeroSolver3D(cfg)


            
        if domain == "prop" and PROP_AVAILABLE:
            opt = prop_core.NozzleOptimizer()
            altitude_idx = 0
            
        while True:
            try:
                # Wait for interaction command from the frontend
                data = await websocket.receive_json()
                command = data.get("command")
                
                payload = {
                    "domain": domain,
                    "type": "result",
                    "timestamp": asyncio.get_event_loop().time(),
                    "data": {}
                }
                
                if domain == "radar" and command == "scan" and RADAR_AVAILABLE:
                    # Extract from nested 'data' object
                    sim_data = data.get("data", {})
                    
                    # Use the delegated radar processor from radar_router.py
                    radar_results = radar_process_command(dsp, tracker, command, sim_data)
                    payload["data"].update(radar_results)
                    
                    # Update Tracker
                    t_now = asyncio.get_event_loop().time()
                    dt = t_now - t_last
                    t_last = t_now
                    # Simulated measurement with noise
                    meas = np.array([1000.0 + random.uniform(-5, 5), 100.0 + random.uniform(-5, 5), 5000.0 + random.uniform(-2, 2)])
                    tracker.update(1, meas, np.eye(3) * 5.0, dt)
                    
                    estimates = tracker.get_multi_track_estimates()
                    if 1 in estimates:
                        est = estimates[1]
                        payload["data"]["track_history_x"] = [est[0]]
                        payload["data"]["track_history_y"] = [est[1]]
                        payload["data"]["track_history_z"] = [est[2]]
                        
                        # Generate Covariance Ellipsoid Point Cloud
                        try:
                            cov = tracker.getCovariance(1)
                            # Extract the 3x3 spatial covariance
                            P_xyz = cov[:3, :3]
                            # Decompose to generate points
                            U, s, _ = np.linalg.svd(P_xyz)
                            L = U @ np.diag(np.sqrt(s))
                            
                            # Standard normal sphere points
                            u_theta = np.linspace(0, 2 * np.pi, 20)
                            v_phi = np.linspace(0, np.pi, 20)
                            x_sphere = np.outer(np.cos(u_theta), np.sin(v_phi)).flatten()
                            y_sphere = np.outer(np.sin(u_theta), np.sin(v_phi)).flatten()
                            z_sphere = np.outer(np.ones_like(u_theta), np.cos(v_phi)).flatten()
                            sphere_pts = np.vstack((x_sphere, y_sphere, z_sphere))
                            
                            # Scale and translate
                            # Plotting the 3-sigma ellipsoid (scale by 3)
                            ellipsoid = (L @ (sphere_pts * 3.0)).T
                            ex = ellipsoid[:, 0] + est[0]
                            ey = ellipsoid[:, 1] + est[1]
                            ez = ellipsoid[:, 2] + est[2]
                            
                            payload["data"]["ellipsoid_x"] = ex.tolist()
                            payload["data"]["ellipsoid_y"] = ey.tolist()
                            payload["data"]["ellipsoid_z"] = ez.tolist()
                        except Exception as e:
                            print(f"Ellipsoid Generation Error: {e}")


                    
                elif domain == "aero" and command == "preview_geometry":
                    sim_data = data.get("data", {})
                    
                    # TIER 1: Fast NumPy preview (no Gmsh, runs in-thread)
                    loop = asyncio.get_event_loop()
                    c, t = await loop.run_in_executor(None, mesher.generate_preview, sim_data)
                    
                    payload["data"] = {
                        "mode": "geometry",
                        "x": c[:, 0].tolist(),
                        "y": c[:, 1].tolist(),
                        "z": c[:, 2].tolist(),
                        "i": t[:, 0].tolist(),
                        "j": t[:, 1].tolist(),
                        "k": t[:, 2].tolist()
                    }

                elif domain == "aero" and command == "generate_final_geometry" and AERO_AVAILABLE:
                    sim_data = data.get("data", {})
                    sim_data["mode"] = "geometry"
                    
                    # TIER 2: Full Gmsh OCC solid (for domain subtraction)
                    loop = asyncio.get_event_loop()
                    with concurrent.futures.ProcessPoolExecutor() as pool:
                        c, t = await loop.run_in_executor(pool, mesher.generate_mesh, sim_data)
                    
                    payload["data"] = {
                        "mode": "geometry",
                        "x": c[:, 0].tolist(),
                        "y": c[:, 1].tolist(),
                        "z": c[:, 2].tolist(),
                        "i": t[:, 0].tolist(),
                        "j": t[:, 1].tolist(),
                        "k": t[:, 2].tolist()
                    }

                elif domain == "aero" and command == "preview_domain" and AERO_AVAILABLE:
                    sim_data = data.get("data", {})
                    sim_data["mode"] = "domain"
                    
                    try:
                        await manager.log("[SYSTEM] Spooling up Mesh process for Bounding Domain Preview...", websocket)
                        loop = asyncio.get_event_loop()
                        m = multiprocessing.Manager()
                        log_queue = m.Queue()

                        async def poll_logs():
                            while True:
                                try:
                                    while not log_queue.empty():
                                        msg = log_queue.get_nowait()
                                        if msg == "DONE": return
                                        await manager.log(msg, websocket)
                                except Exception:
                                    pass
                                await asyncio.sleep(0.1)

                        poll_task = asyncio.create_task(poll_logs())
                        
                        with concurrent.futures.ProcessPoolExecutor() as pool:
                            c, t, colors = await loop.run_in_executor(pool, mesher.generate_mesh, sim_data, log_queue)
                        
                        log_queue.put("DONE")
                        await poll_task
                        
                        await manager.log("[SYSTEM] Generated Bounding Domain. Processing geometry slice...", websocket)
                        # Generate the solid geometry separately to overlay in the UI
                        aero_c, aero_t = await loop.run_in_executor(None, mesher.generate_preview, sim_data)
                        await manager.log("[SYSTEM] Domain Preview Ready. Sending to client.", websocket)
                        
                        payload["data"] = {
                            "mode": "domain",
                            "x": c[:, 0].tolist(),
                            "y": c[:, 1].tolist(),
                            "z": c[:, 2].tolist(),
                            "i": t[:, 0].tolist(),
                            "j": t[:, 1].tolist(),
                            "k": t[:, 2].tolist(),
                            "c": colors.tolist(),
                            "aero_x": aero_c[:, 0].tolist(),
                            "aero_y": aero_c[:, 1].tolist(),
                            "aero_z": aero_c[:, 2].tolist(),
                            "aero_i": aero_t[:, 0].tolist(),
                            "aero_j": aero_t[:, 1].tolist(),
                            "aero_k": aero_t[:, 2].tolist()
                        }
                    except Exception as e:
                        print(f"[SYSTEM] Domain Generation Error: {e}")
                        payload["data"] = {"error": f"Domain Meshing Failed: {str(e)}"}

                elif domain == "aero" and command == "calculate" and AERO_AVAILABLE:
                    sim_data = data.get("data", {})
                    sim_data["mode"] = "full"
                    
                    cfg = aero_core.SolverConfig()
                    cfg.alpha = float(sim_data.get("aoa", 2.0))  # Degrees — solver converts internally
                    cfg.mach = float(sim_data.get("mach", 0.8))
                    cfg.altitude = float(sim_data.get("alt", 10000.0))
                    cfg.cfl = float(sim_data.get("cfl", 1.5))
                    cfg.is_implicit = True
                    cfg.use_ddes = (sim_data.get("turb", "DDES") == "DDES")
                    
                    # New gas properties from UI
                    cfg.gamma = float(sim_data.get("gamma", 1.4))
                    cfg.gas_constant = float(sim_data.get("gas_constant", 287.05))
                    cfg.P_sl = float(sim_data.get("P_sl", 101325.0))
                    cfg.T_sl = float(sim_data.get("T_sl", 288.15))
                    cfg.prandtl_number = float(sim_data.get("prandtl", 0.72))
                    cfg.cp = cfg.gamma * cfg.gas_constant / (cfg.gamma - 1.0)
                    
                    # Standard Constants (Not exposed to UI yet, but dynamic)
                    cfg.lapse_rate = float(sim_data.get("lapseRate", 0.0065))
                    cfg.suth_mu0 = float(sim_data.get("suth_mu0", 1.716e-5))
                    cfg.suth_T0 = float(sim_data.get("suth_T0", 273.15))
                    cfg.suth_S = float(sim_data.get("suth_S", 110.4))
                    
                    solver = aero_core.AeroSolver3D(cfg)
                    loop = asyncio.get_event_loop()
                    
                    await manager.log("[SYSTEM] Generating Unstructured AFM Mesh via Gmsh OpenMP and HXT 3D...", websocket)
                    m = multiprocessing.Manager()
                    log_queue = m.Queue()
                    
                    async def poll_logs_full():
                        while True:
                            try:
                                while not log_queue.empty():
                                    msg = log_queue.get_nowait()
                                    if msg == "DONE": return
                                    await manager.log(msg, websocket)
                            except Exception:
                                pass
                            await asyncio.sleep(0.1)
                            
                    poll_task = asyncio.create_task(poll_logs_full())

                    with concurrent.futures.ProcessPoolExecutor() as pool:
                        c, t, bt = await loop.run_in_executor(pool, mesher.generate_mesh, sim_data, log_queue)
                        
                    log_queue.put("DONE")
                    await poll_task
                    
                    await manager.log("[SYSTEM] Mesh generation complete. Loading memory into AeroSolver3D...", websocket)
                    solver.set_mesh(c, t, bt)
                    await manager.log("[SYSTEM] Solver Initialized. Starting iterative execution...", websocket)
                    
                    max_iters = int(sim_data.get("maxIters", 500))
                    epsilon = float(sim_data.get("epsilon", 1e-5))
                    chunk_size = 10
                    
                    for step in range(0, max_iters, chunk_size):
                        def run_chunk():
                            for _ in range(chunk_size):
                                solver.step()
                        await loop.run_in_executor(None, run_chunk)
                        
                        if (step + chunk_size) % 10 == 0:
                            await loop.run_in_executor(None, solver.export_surface_vtk, "live_surface.vtk")
                        
                        res_hist = solver.get_residual_history()
                        res_rho = solver.get_res_rho_history()
                        res_rhoU = solver.get_res_rhoU_history()
                        res_rhoE = solver.get_res_rhoE_history()
                        
                        cl_hist = solver.get_cl_history()
                        cd_hist = solver.get_cd_history()
                        
                        live_payload = {
                            "type": "result",
                            "data": {
                                "status": "solving",
                                "iter": step + chunk_size,
                                "cl": cl_hist,
                                "cd": cd_hist,
                                "residual": res_hist,
                                "res_rho": res_rho,
                                "res_rhoU": res_rhoU,
                                "res_rhoE": res_rhoE,
                                "max_mach": solver.get_max_mach_history(),
                                "max_vel": solver.get_max_vel_history(),
                                "max_enthalpy": solver.get_max_enthalpy_history(),
                                "min_pressure": solver.get_min_pressure_history(),
                                "min_density": solver.get_min_density_history(),
                                "max_temp": solver.get_max_temp_history()
                            }
                        }
                        
                        if (step + chunk_size) % 10 == 0:
                            try:
                                mach = np.array(solver.get_mach_field())
                                vel = np.array(solver.get_velocity_field()).reshape(-1, 3)
                                centroids = np.array(solver.get_cell_centroids()).reshape(-1, 3)
                                
                                z_vals = np.unique(np.round(centroids[:, 2], 4))
                                if len(z_vals) > 0:
                                    mid_z = z_vals[len(z_vals) // 2]
                                    slice_idx = np.where(np.abs(centroids[:, 2] - mid_z) < 1e-3)[0]
                                    
                                    live_payload["data"]["cfd_x"] = centroids[slice_idx, 0].tolist()
                                    live_payload["data"]["cfd_y"] = centroids[slice_idx, 1].tolist()
                                    live_payload["data"]["cfd_z"] = centroids[slice_idx, 2].tolist()
                                    live_payload["data"]["cfd_mach"] = mach[slice_idx].tolist()
                                    live_payload["data"]["cfd_u"] = vel[slice_idx, 0].tolist()
                                    live_payload["data"]["cfd_v"] = vel[slice_idx, 1].tolist()
                                    live_payload["data"]["cfd_w"] = vel[slice_idx, 2].tolist()
                                    
                                    live_payload["data"]["cfd_i"] = solver.get_delaunay_i()
                                    live_payload["data"]["cfd_j"] = solver.get_delaunay_j()
                                    live_payload["data"]["cfd_k"] = solver.get_delaunay_k()
                                    
                                live_payload["data"]["mach_current"] = float(mach.max()) if isinstance(mach, np.ndarray) else mach
                            except Exception as e:
                                print(f"Error computing live CFD slice: {e}")

                        await websocket.send_json(live_payload)
                        log_msg = f"[AERO_CORE] Step {step + chunk_size}/{max_iters} | CL: {cl_hist[-1]:.4f} | Res: {res_hist[-1]:.2e} [rho:{res_rho[-1]:.2e}, rhoU:{res_rhoU[-1]:.2e}, rhoE:{res_rhoE[-1]:.2e}] | Extrema: [Mach:{live_payload['data']['max_mach'][-1]:.2f}, Vel:{live_payload['data']['max_vel'][-1]:.1f}, Temp:{live_payload['data']['max_temp'][-1]:.1f}, P_min:{live_payload['data']['min_pressure'][-1]:.1f}]"
                        await manager.log(log_msg, websocket)
                        
                        if len(res_hist) > 0 and res_hist[-1] < epsilon:
                            await manager.log(f"[SYSTEM] Solver Converged at iteration {step + chunk_size}", websocket)
                            break
                    
                    await manager.log("[SYSTEM] Simulation Complete. Constructing 3D result slice...", websocket)
                    try:
                        mach = np.array(solver.get_mach_field())
                        vel = np.array(solver.get_velocity_field()).reshape(-1, 3)
                        centroids = np.array(solver.get_cell_centroids()).reshape(-1, 3)
                        
                        z_vals = np.unique(np.round(centroids[:, 2], 4))
                        if len(z_vals) > 0:
                            mid_z = z_vals[len(z_vals) // 2]
                            slice_idx = np.where(np.abs(centroids[:, 2] - mid_z) < 1e-3)[0]
                            
                            payload["data"]["cfd_x"] = centroids[slice_idx, 0].tolist()
                            payload["data"]["cfd_y"] = centroids[slice_idx, 1].tolist()
                            payload["data"]["cfd_z"] = centroids[slice_idx, 2].tolist()
                            payload["data"]["cfd_mach"] = mach[slice_idx].tolist()
                            payload["data"]["cfd_u"] = vel[slice_idx, 0].tolist()
                            payload["data"]["cfd_v"] = vel[slice_idx, 1].tolist()
                            payload["data"]["cfd_w"] = vel[slice_idx, 2].tolist()
                            
                            payload["data"]["cfd_i"] = solver.get_delaunay_i()
                            payload["data"]["cfd_j"] = solver.get_delaunay_j()
                            payload["data"]["cfd_k"] = solver.get_delaunay_k()
                            
                        payload["data"]["mach_current"] = float(mach.max()) if isinstance(mach, np.ndarray) else mach
                        payload["data"]["cl"] = solver.get_cl_history()
                        payload["data"]["cd"] = solver.get_cd_history()
                        
                        # Fetch full surface Post-Processing Data
                        await manager.log("[SYSTEM] Extracting full 3D Surface Boundary Data...", websocket)
                        surf_data = await loop.run_in_executor(None, solver.get_surface_data)
                        
                        post_payload = {
                            "type": "result",
                            "data": {
                                "mode": "post_process",
                                "post_x": list(surf_data.x),
                                "post_y": list(surf_data.y),
                                "post_z": list(surf_data.z),
                                "post_i": list(surf_data.i),
                                "post_j": list(surf_data.j),
                                "post_k": list(surf_data.k),
                                "post_mach": list(surf_data.mach),
                                "post_pressure": list(surf_data.pressure),
                                "post_velocity": list(surf_data.velocity)
                            }
                        }
                        await websocket.send_json(post_payload)
                        await manager.log("[SYSTEM] Post-Processing Data Sent. UI Switch allowed.", websocket)
                    except Exception as e:
                        print(f"CFD Telemetry Error: {e}")


                    
                elif domain == "prop" and command == "optimize" and PROP_AVAILABLE:
                    sim_data = data.get("data", {})
                    thrust = sim_data.get("thrust", 50000.0)
                    pa = sim_data.get("pa", 101325.0)
                    prop_idx = sim_data.get("prop", 0)
                    mat_idx = sim_data.get("mat", 0)
                    gens = sim_data.get("gens", 10)
                    pc_min = sim_data.get("pcMin", 1e6)
                    pc_max = sim_data.get("pcMax", 21e6)
                    tw_min = sim_data.get("twMin", 0.001)
                    tw_max = sim_data.get("twMax", 0.011)
                    
                    # Execute C++ GA Optimizer in a separate thread
                    loop = asyncio.get_event_loop()
                    res = await loop.run_in_executor(None, opt.optimize, prop_idx, mat_idx, thrust, pa, gens, pc_min, pc_max, tw_min, tw_max)
                    
                    payload["data"]["geometry_x"] = res.x.tolist()
                    payload["data"]["geometry_y"] = res.y.tolist()
                    payload["data"]["mach_dist"] = res.mach.tolist()
                    payload["data"]["temp"] = res.temperature.tolist()
                    payload["data"]["t_hw"] = res.T_hw.tolist()
                    payload["data"]["margin_of_safety"] = res.margin_of_safety.tolist()
                    payload["data"]["isp"] = res.Isp_true
                    payload["data"]["mos"] = res.MoS




                await manager.send_json(payload, websocket)
                
            except WebSocketDisconnect:
                break
            except RuntimeError as e:
                # Handle cases where websocket is already closed/disconnected
                if "accept" in str(e).lower() or "close" in str(e).lower():
                    break
                print(f"WebSocket command error: {e}")
            except Exception as e:
                # Handle disconnects or bad messages gracefully
                if "disconnect" in str(e).lower() or "close" in str(e).lower():
                    break
                print(f"WebSocket command error: {e}")
                
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        print(f"WebSocket Error: {e}")
        manager.disconnect(websocket)



if __name__ == "__main__":
    uvicorn.run("main:app", host="127.0.0.1", port=8000, reload=True)
