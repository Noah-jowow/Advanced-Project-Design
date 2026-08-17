from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
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
