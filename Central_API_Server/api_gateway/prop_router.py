from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from api_gateway.optimization import optimize_nozzle
import sys
import os

# Path to the compiled C++ module
PROP_CORE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../Propulsion/build"))
if PROP_CORE_DIR not in sys.path:
    sys.path.append(PROP_CORE_DIR)

# On Windows, we may need to add the MinGW bin directory to the DLL search path
if os.name == 'nt':
    MINGW_BIN = r"C:\Users\noahj\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin"
    if os.path.exists(MINGW_BIN):
        os.add_dll_directory(MINGW_BIN)

try:
    import prop_core
    CORE_LOADED = True
except ImportError:
    CORE_LOADED = False

router = APIRouter()

class ThrustRequest(BaseModel):
    pressure_ratio: float
    gamma: float

class OptimizationRequest(BaseModel):
    target_altitude: float
    ambient_pressure: float

@router.get("/health")
async def health():
    return {
        "service": "Propulsion",
        "status": "online" if CORE_LOADED else "degraded",
        "core_module_loaded": CORE_LOADED
    }

@router.post("/calculate_thrust")
async def calculate_thrust(req: ThrustRequest):
    if not CORE_LOADED:
        raise HTTPException(status_code=503, detail="Propulsion C++ core module not available")
    
    try:
        optimizer = prop_core.NozzleOptimizer()
        result = optimizer.calculate_thrust_coefficient(req.pressure_ratio, req.gamma)
        return {"thrust_coefficient": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/optimize_expansion")
async def optimize_expansion(req: OptimizationRequest):
    if not CORE_LOADED:
        raise HTTPException(status_code=503, detail="Propulsion C++ core module not available")
    
    try:
        optimizer = prop_core.NozzleOptimizer()
        result = optimizer.optimize_expansion_ratio(req.target_altitude, req.ambient_pressure)
        return {"optimized_expansion_ratio": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

async def prop_process_command(data: dict, opt=None, manager=None, websocket=None) -> dict:
    import asyncio
    import queue
    
    command = data.get("command")
    if command == "optimize":
        sim_data = data.get("data", {})
        thrust = float(sim_data.get("thrust", 50000.0))
        pa = float(sim_data.get("pa", 101325.0))
        prop_idx = int(sim_data.get("prop", 0))
        mat_idx = int(sim_data.get("mat", 0))
        num_angles = int(sim_data.get("angles", 20))
        
        loop = asyncio.get_event_loop()
        msg_queue = queue.Queue()
        
        def thread_log(msg: str):
            msg_queue.put({"type": "log", "msg": msg})
            
        def thread_telemetry(t_data: dict):
            msg_queue.put({"type": "telemetry", "data": t_data})
            
        async def poll_stream():
            while True:
                try:
                    while not msg_queue.empty():
                        item = msg_queue.get_nowait()
                        if item == "DONE":
                            return
                        if item["type"] == "log" and manager and websocket:
                            await manager.log(item["msg"], websocket)
                        elif item["type"] == "telemetry" and manager and websocket:
                            await manager.send_json({
                                "domain": "prop",
                                "type": "result",
                                "data": item["data"]
                            }, websocket)
                except Exception:
                    pass
                await asyncio.sleep(0.05)
                
        poll_task = asyncio.create_task(poll_stream())
        
        try:
            res = await loop.run_in_executor(
                None,
                optimize_nozzle,
                prop_idx,
                mat_idx,
                thrust,
                pa,
                num_angles,
                thread_log,
                thread_telemetry,
                sim_data
            )
        finally:
            msg_queue.put("DONE")
            await poll_task
        
        return {
            "status": "completed",
            "geometry_x": res["x"],
            "geometry_y": res["y"],
            "mach_dist": res["mach"],
            "temp": res["temperature"],
            "t_hw": res["T_hw"],
            "margin_of_safety": res["margin_of_safety"],
            "q_flux": res.get("q_flux", []),
            "delta_p_cool_dist": res.get("delta_p_cool_dist", []),
            "delta_star": res.get("delta_star", []),
            "isp": res["Isp_true"],
            "thrust_delivered": res.get("thrust_delivered", thrust),
            "mos": res["MoS"],
            "mass": res["mass"],
            "epsilon": res["epsilon_geom"],
            "epsilon_eff": res.get("epsilon_eff", res["epsilon_geom"]),
            "delta_exit": res.get("delta_exit", 0.0),
            "lambda_div": res.get("lambda_div", 1.0),
            "delta_p_cool": res.get("delta_p_cool", 0.0),
            "q_max": res.get("q_max", 0.0),
            "t_hw_max": res.get("T_hw_max", 0.0),
            "pc": res["Pc"],
            "t_w": res["t_w"],
            "max_length": res.get("max_length", 2.5),
            "max_exit_radius": res.get("max_exit_radius", 1.0),
            "min_mos": res.get("min_mos", 0.10),
            "max_temp_ratio": res.get("max_temp_ratio", 0.90),
            "max_iter": res.get("max_iter", 50),
            "nit": res.get("nit", 0)
        }
    return {}

