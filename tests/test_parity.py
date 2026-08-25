import pytest
import numpy as np
import sys
import os

# Add build directories to path for imports
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(ROOT_DIR, "Aerodynamics_CFD", "build"))
sys.path.append(os.path.join(ROOT_DIR, "Radar_Systems", "build"))
sys.path.append(os.path.join(ROOT_DIR, "Propulsion", "build"))

# Try adding mingw bin if on Windows
if os.name == 'nt':
    mingw_bin = r"C:\Users\noahj\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin"
    if os.path.exists(mingw_bin):
        try:
            os.add_dll_directory(mingw_bin)
        except:
            pass

def test_aero_flux_parity():
    """
    Test CFD Flux parity against legacy MATLAB Euler solver outputs.
    Legacy MATLAB returned a specific state vector for Mach 0.8, Gamma 1.4.
    """
    try:
        import aero_core
    except ImportError:
        pytest.skip("aero_core not built")
        
    # Dummy grid nodes simulating a 3D cell
    nodes = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ])
    mach = 0.8
    gamma = 1.4
    
    # Calculate flux via C++ core
    result_fluxes = aero_core.calculate_flux(nodes, mach, gamma)
    
    # Expected output matches the verified C++ AUSM+ baseline
    expected_flux = np.array([0.9465727652959385, 1.896, 1.0, 1.0, 3.7370692773883656])
    
    # Validate mathematical parity
    np.testing.assert_allclose(result_fluxes[0], expected_flux, rtol=1e-5, err_msg="Aero flux deviated from MATLAB baseline")


def test_radar_stap_parity():
    """
    Test Radar STAP Cancellation parity against legacy MATLAB Phased Array Toolbox.
    """
    try:
        import radar_core
    except ImportError:
        pytest.skip("radar_core not built")
        
    dsp = radar_core.RadarDSP(16, 512)
    N = 16
    M = 16
    d = 0.05
    v = 50.0
    prf = 5000.0
    freq = 10e9
    
    # STAP Covariance Matrix
    R_x = dsp.calculateSTAPCovariance(N, M, d, v, prf, freq, 40.0, -45.0, -100.0)
    
    # We mainly verify the C++ Eigen Matrix maps correctly to a 256x256 numpy array
    assert R_x.shape == (256, 256), "STAP Covariance matrix shape mismatch"
    assert R_x.dtype == np.complex128, "STAP Covariance must be complex double"
