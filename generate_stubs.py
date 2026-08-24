import subprocess
import sys
import os
import pybind11_stubgen

modules = [
    ("aero_core", "Aerodynamics_CFD"),
    ("prop_core", "Propulsion"),
    ("radar_core", "Radar_Systems")
]

# Add MinGW bin to DLL path so libgomp-1.dll can be found for aero_core
mingw_bin = r"C:\Users\noahj\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin"
if os.path.exists(mingw_bin) and hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(mingw_bin)

for module, folder in modules:
    print(f"Generating stubs for {module}...")
    try:
        out_dir = os.path.join(folder, "src")
        # Run in-process so it uses the injected DLL directory
        sys.argv = ["pybind11_stubgen", module, "-o", out_dir]
        pybind11_stubgen.main()
        print(f"Successfully generated stubs for {module} in {out_dir}")
    except SystemExit as e:
        if e.code != 0:
            print(f"Failed to generate stubs for {module}")
    except Exception as e:
        print(f"Failed to generate stubs for {module}: {e}")
