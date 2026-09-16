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

_prev_radar_params = {}

def radar_process_command(dsp, tracker, env_sim, command, sim_data, log_callback=None):
    """
    Process radar commands from WebSocket, mapping frontend parameters 
    to C++ RadarDSP and TrackerIMM functions, with rich developer telemetry.
    """
    global _prev_radar_params
    payload = {}
    
    def log(msg: str):
        if log_callback:
            try:
                log_callback(msg)
            except Exception:
                pass
        else:
            print(msg)
    
    if command == "scan":
        freq = float(sim_data.get("freq", 10e9))
        tau = float(sim_data.get("tau", 10e-6))
        prf = float(sim_data.get("prf", 5000.0))
        bw = float(sim_data.get("bw", 50e6))
        Ny = int(sim_data.get("Ny", 16))
        Nz = int(sim_data.get("Nz", 16))
        taper = int(sim_data.get("taper", 1))
        steerAz = float(sim_data.get("steerAz", 45.0))
        steerEl = float(sim_data.get("steerEl", 15.0))
        steerType = int(sim_data.get("steerType", 1))
        pwr_kw = float(sim_data.get("pwr", 50.0))
        jammerAz = float(sim_data.get("jammerAz", -45.0))
        jnr_db = float(sim_data.get("jnr_db", -100.0))
        targetRange = float(sim_data.get("targetRange", 5000.0))
        rainRate = float(sim_data.get("rainRate", 10.0))

        # Check for parameter changes from previous run
        curr_params = {
            "freq": freq, "pwr": pwr_kw, "tau": tau, "prf": prf, "bw": bw,
            "Ny": Ny, "Nz": Nz, "taper": taper, "az": steerAz, "el": steerEl,
            "jammerAz": jammerAz, "jnr": jnr_db
        }
        param_diff = {k: v for k, v in curr_params.items() if _prev_radar_params.get(k) != v}
        if param_diff or not _prev_radar_params:
            _prev_radar_params = curr_params
            log(f"[RADAR_PARAMS] Input change received & applied: Freq={freq/1e9:.2f} GHz, Pwr={pwr_kw:.1f} kW, PRF={prf:.0f} Hz, Tau={tau*1e6:.1f} µs, BW={bw/1e6:.1f} MHz, Beam=[Az:{steerAz:.1f}°, El:{steerEl:.1f}°], Aperture=[{Ny}x{Nz}]. Solution updated.")
        
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
            bin_idx = int((2 * r / c) * fs)
            if 0 <= bin_idx < num_samples:
                for p in range(num_pulses):
                    rx_mat[p, bin_idx] += 50.0 * np.exp(1j * 2 * np.pi * f_d * p / prf)

        rd_mat = dsp.processCPI(rx_mat)
        rd_mag_sq = np.abs(rd_mat)**2
        
        # Send rd_matrix for heatmap
        payload["rd_matrix"] = (10 * np.log10(rd_mag_sq + 1e-10)).tolist()
        
        # CFAR Detections
        detections = dsp.runCACFAR2D(rd_mag_sq, 2, 2, 4, 4, 15.0) 
        n_detections = int(np.sum(detections))
        payload["cfar_detections"] = detections.tolist()

        # Matched Filter Profile (single pulse)
        pulse = rx_mat[0, :]
        mf_out = dsp.applyLFMMatchedFilter(pulse, bw, tau, 125e6)
        payload["mf_profile"] = (10 * np.log10(np.abs(mf_out)**2 + 1e-10)).tolist()

        # Fundamentals: SNR vs Range (simulated curve)
        ranges = np.linspace(1000, 20000, 100)
        pt = pwr_kw * 1e3
        snr_curve = 10 * np.log10((pt * 1e12) / (ranges**4 + 1e-1))
        payload["snr_range"] = snr_curve.tolist()
        
        # 3. STAP Clutter Cancellation
        N = Ny 
        M = 16 
        v = 50.0 
        d = 0.05 

        weights = dsp.calculateSTAPCancellation(
            N, M, d, v, prf, freq, steerAz, 500.0, 20.0, 40.0,
            jammerAz, jnr_db, targetRange, rainRate
        )
        payload["stap_weights"] = [str(w) for w in weights.tolist()]
        
        # 4. STAP Covariance Matrix
        R_x = dsp.calculateSTAPCovariance(N, M, d, v, prf, freq, 40.0, jammerAz, jnr_db)
        stap_norm = float(np.linalg.norm(R_x))
        payload["stap_covariance_norm"] = stap_norm
        payload["stap_covariance"] = np.abs(R_x).tolist()

        if param_diff:
            log(f"[RADAR_DSP] CPI processed: 16 pulses x 512 samples | CFAR Detections: {n_detections} | STAP Covariance Norm: {stap_norm:.2e}")

    elif command == "add_target":
        id = sim_data.get("id", 1)
        x = float(sim_data.get("x", 1000.0))
        y = float(sim_data.get("y", 100.0))
        z = float(sim_data.get("z", 5000.0))
        vx = float(sim_data.get("vx", 300.0))
        vy = float(sim_data.get("vy", 0.0))
        vz = float(sim_data.get("vz", 0.0))
        
        env_sim.addTarget(id, "Airliner", np.array([x, y, z]), np.array([vx, vy, vz]), 10.0, False, False)
        payload["message"] = f"Track {id} added successfully to EnvironmentSimulator."
        log(f"[RADAR_PARAMS] Input change received: Injected Target ID={id} at Pos=[{x:.0f}, {y:.0f}, {z:.0f}] m, Vel=[{vx:.0f}, {vy:.0f}, {vz:.0f}] m/s. Committed to active environment.")

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
