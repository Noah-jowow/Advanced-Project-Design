from fastapi import APIRouter
from pydantic import BaseModel
import sys
import os
import numpy as np

# Adjust python path to find the radar_core C++ extension
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../Radar_Systems/build")))

# For Python 3.8+ on Windows, we need to add the DLL directory for MinGW libraries
mingw_bin = r"C:\Users\noahj\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin"
if os.path.exists(mingw_bin):
    try:
        os.add_dll_directory(mingw_bin)
    except Exception:
        pass

try:
    import radar_core
except ImportError:
    pass

router = APIRouter()

class RadarRequest(BaseModel):
    num_pulses: int = 16
    num_samples: int = 1000
    bw: float = 50e6
    tau: float = 10e6
    fs: float = 125e6
    prf: float = 5000.0

def radar_process_command(dsp, tracker, env_sim, command, sim_data):
    """
    Process radar commands from WebSocket, mapping frontend parameters 
    to C++ RadarDSP and TrackerIMM functions.
    """
    payload = {}
    
    if command == "scan":
        freq = sim_data.get("freq", 10e9)
        tau = sim_data.get("tau", 10e-6)
        prf = sim_data.get("prf", 5000.0)
        bw = sim_data.get("bw", 50e6)
        Ny = sim_data.get("Ny", 16)
        Nz = sim_data.get("Nz", 16)
        taper = sim_data.get("taper", 1)
        steerAz = sim_data.get("steerAz", 45.0)
        steerEl = sim_data.get("steerEl", 15.0)
        steerType = sim_data.get("steerType", 1)
        
        # 1. 3D Spatial Pattern (Wideband Beamforming)
        af_db = dsp.calculate3DSpatialPattern(Ny, Nz, 0.05, 0.05, freq, steerAz, steerEl, steerType, taper, bw, tau)
        payload["spatial_pattern"] = af_db.tolist()
        
        # Calculate Azimuth and Elevation cuts
        mid_row = af_db.shape[0] // 2
        mid_col = af_db.shape[1] // 2
        payload["azimuth_cut"] = af_db[mid_row, :].tolist()
        payload["elevation_cut"] = af_db[:, mid_col].tolist()
        
        # 2. Process CPI & CFAR
        num_pulses = 16
        num_samples = 512
        rx_mat = (np.random.randn(num_pulses, num_samples) + 
                  1j * np.random.randn(num_pulses, num_samples)).astype(np.complex128)
        
        # Inject true target signatures based on env_sim
        targets = env_sim.getTargets()
        c = 299792458.0
        fs = 125e6
        for tgt_id, tgt in targets.items():
            r = np.linalg.norm(tgt.pos)
            v_r = -np.dot(tgt.vel, tgt.pos) / max(1e-6, r)
            f_d = 2 * v_r * freq / c
            # Convert range to bin
            bin_idx = int((2 * r / c) * fs)
            if 0 <= bin_idx < num_samples:
                # Basic injection (ignoring realistic SNR for now)
                # We'll inject it with amplitude ~ 50.0
                for p in range(num_pulses):
                    rx_mat[p, bin_idx] += 50.0 * np.exp(1j * 2 * np.pi * f_d * p / prf)

        rd_mat = dsp.processCPI(rx_mat)
        rd_mag_sq = np.abs(rd_mat)**2
        
        # Send rd_matrix for heatmap
        payload["rd_matrix"] = (10 * np.log10(rd_mag_sq + 1e-10)).tolist()
        
        # CFAR Detections
        detections = dsp.runCACFAR2D(rd_mag_sq, 2, 2, 4, 4, 15.0) 
        payload["cfar_detections"] = detections.tolist()

        # Matched Filter Profile (single pulse)
        pulse = rx_mat[0, :]
        mf_out = dsp.applyLFMMatchedFilter(pulse, bw, tau, 125e6)
        payload["mf_profile"] = (10 * np.log10(np.abs(mf_out)**2 + 1e-10)).tolist()

        # Fundamentals: SNR vs Range (simulated curve)
        ranges = np.linspace(1000, 20000, 100)
        pt = sim_data.get("pwr", 50.0) * 1e3
        snr_curve = 10 * np.log10((pt * 1e12) / (ranges**4 + 1e-1))
        payload["snr_range"] = snr_curve.tolist()
        
        # 3. STAP Clutter Cancellation
        N = Ny 
        M = 16 
        v = 50.0 
        d = 0.05 
        jammerAz = sim_data.get("jammerAz", -45.0)
        jnr_db = sim_data.get("jnr_db", -100.0)
        targetRange = sim_data.get("targetRange", 5000.0)
        rainRate = sim_data.get("rainRate", 10.0)

        weights = dsp.calculateSTAPCancellation(
            N, M, d, v, prf, freq, steerAz, 500.0, 20.0, 40.0,
            jammerAz, jnr_db, targetRange, rainRate
        )
        payload["stap_weights"] = [str(w) for w in weights.tolist()]
        
        # 4. STAP Covariance Matrix
        R_x = dsp.calculateSTAPCovariance(N, M, d, v, prf, freq, 40.0, jammerAz, jnr_db)
        payload["stap_covariance_norm"] = float(np.linalg.norm(R_x))
        payload["stap_covariance"] = np.abs(R_x).tolist()

    elif command == "add_target":
        # Extract target parameters
        id = sim_data.get("id", 1)
        x = sim_data.get("x", 1000.0)
        y = sim_data.get("y", 100.0)
        z = sim_data.get("z", 5000.0)
        vx = sim_data.get("vx", 300.0)
        vy = sim_data.get("vy", 0.0)
        vz = sim_data.get("vz", 0.0)
        
        # Add to EnvironmentSimulator instead of direct TrackerIMM
        # Default to Airliner (10.0 rcs)
        env_sim.addTarget(id, "Airliner", np.array([x, y, z]), np.array([vx, vy, vz]), 10.0, False, False)
        payload["message"] = f"Track {id} added successfully to EnvironmentSimulator."

    return payload

@router.post("/process_cpi")
def process_cpi(req: RadarRequest):
    try:
        import radar_core
    except ImportError:
        return {"error": "radar_core C++ extension not found in build directory"}
        
    try:
        # Instantiate the DSP core
        dsp = radar_core.RadarDSP(req.num_pulses, req.num_samples)
        
        # 1. Generate Matched Filter
        dsp.generateMatchedFilter(req.bw, req.tau, req.fs)
        
        # 2. Generate dummy RX matrix (Complex Double)
        # Using complex128 to match MatrixXcd
        rx_mat = (np.random.randn(req.num_pulses, req.num_samples) + 
                  1j * np.random.randn(req.num_pulses, req.num_samples)).astype(np.complex128)
        
        # 3. Process CPI (Pulse Compression + Doppler FFT)
        rd_mat = dsp.processCPI(rx_mat)
        
        # 4. Magnitude Squared for Detection
        rd_mag_sq = np.abs(rd_mat)**2
        
        # 5. Run 2D CA-CFAR
        # Parameters: guard_r, guard_c, ref_r, ref_c, threshold_factor
        detections = dsp.runCACFAR2D(rd_mag_sq, 2, 2, 4, 4, 10.0) 
        
        # 6. Test TrackerIMM briefly
        tracker = radar_core.TrackerIMM()
        init_state = np.zeros(7)
        init_cov = np.eye(7) * 100
        tracker.addTrack(1, init_state, init_cov)
        
        # Update with a dummy measurement [r, az, el]
        tracker.update(1, np.array([1000.0, 0.1, 0.05]), np.eye(3), 1.0)
        est = tracker.getEstimate(1)
        
        return {
            "status": "success",
            "rd_matrix_shape": rd_mat.shape,
            "num_detections": int(np.sum(detections)),
            "tracker_estimate": est.tolist()
        }
    except Exception as e:
        return {"status": "error", "message": str(e)}
