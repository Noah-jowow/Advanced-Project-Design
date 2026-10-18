# Advanced Project Design Architecture: System of Systems

Welcome to the Advanced Project Design workspace. This environment enforces strict architectural isolation between mathematical domains while unifying them under a single, highly optimized API gateway.

## 🚀 The Architecture (Single Computation Server)

This workspace has been structurally modernized from monolithic MATLAB scripts into a distributed **System of Systems**. The core design philosophy is **Centralized Gateway, Decentralized Mathematics**.

### Why This Architecture?
Rather than spinning up dozens of independent servers for dozens of projects, we utilize a **Monolithic API Gateway** (`Central_API_Server/main.py`) running FastAPI. 
1. **Decentralized Math:** The heavy mathematical processing for each domain (Radar, CFD, Propulsion, Spacecraft SADC) is completely isolated in raw C++ (`Eigen`) microservices.
2. **Pybind11 Middleware:** These C++ cores are compiled into native Python extensions (`.pyd`).
3. **Centralized Routing:** The single FastAPI server dynamically imports these C++ extensions (`radar_core`, `aero_core`, `prop_core`, `sadc_core`). It holds the mathematical state in shared memory and handles high-frequency WebSocket streaming to the frontend.
4. **React SPA Frontend:** The user interface is a unified React/Vite single-page application (`Central_API_Server/frontend`). It reads URL parameters (e.g. `?project=RADAR`) to dynamically render the exact engineering workbench without reloading the DOM.

This ensures massive scalability. You only ever need to run *one* backend server and *one* frontend server, no matter how many projects you add to the workspace.

---

## ⚡ How to Launch

We have abstracted all compile logic and server initialization into a single click:

1. Double-click the **`AeroSys_Launcher.bat`** file in the root directory.
2. Select your desired project from the menu:
   - `[1] Boot Radar C2 Workspace`
   - `[2] Boot Aerodynamics CFD Workspace`
   - `[3] Boot Rocket Propulsion Workspace`
   - `[4] Boot Spacecraft Attitude & Orbit (SADC) Workspace`

The launcher will auto-compile any C++ changes using Ninja, boot the FastAPI backend securely on `127.0.0.1:8000`, boot the Vite frontend on `localhost:5173`, and automatically launch your default browser directly into the selected workbench.

---

## 🏛️ Engineering Domains

The repository is partitioned into strict, functional domain repositories:
- **`Radar_Systems/`**: High-fidelity phased array beamforming, pulsed Doppler DSP, CA-CFAR detection, 7-state IMM-EKF kinematic tracking, true STAP clutter cancellation, and Newton-Euler Hexapod / Euler-Lagrange Az/El Pointer Dynamics (`radar_core`).
- **`Aerodynamics_CFD/`**: 3D finite volume Navier-Stokes and Euler compressible flow solvers utilizing AUSM+ flux splitting, LSQ gradient reconstruction with limiters, DDES turbulence, and unstructured meshing (`aero_core`).
- **`Propulsion/`**: Chemical rocket thermodynamic analysis, Method of Characteristics (MoC) Rao optimum bell nozzle contouring, Bartz convective heat transfer, Fourier wall conduction, and Lamé stress margins (`prop_core`).
- **`Spacecraft_SADC/`**: Two-body continuous orbital mechanics, Earth sidereal rotation, 6-DOF quaternion kinematics, Euler's rotational equations with inertia cross-coupling, gravity gradient disturbance, and LQR closed-loop line-of-sight tracking for ground stations (`sadc_core`).
- **`Tracker/`**: Control system laws (PID, LQR) and SolidWorks CAD hardware models for physical target tracking systems (Az/El rotors, slip rings, bevel gear reduction).
- **`Central_API_Server/`**: High-throughput FastAPI gateway, asynchronous WebSocket streaming (`/ws/stream/{domain}`), and commercial-grade React/Vite/Plotly single-page application.

**Golden Rule:** Do not mix functional domains. Code generated for one domain must remain modular and isolated from the others. Dependencies between domains are strictly managed via path configuration.

---

## Environment Initialization (`startup.m`)

We utilize a **decentralized environment initialization**. Instead of a master path script, each domain folder contains its own `startup.m`. 

### How to Start Work in MATLAB
1. Launch MATLAB.
2. Navigate your Current Folder directly into the domain you want to work on (e.g., `cd Aerodynamics_CFD`).
3. The `startup.m` script will automatically execute.
4. The script will securely load the domain's subfolders and dynamically link shared libraries, ignoring backups and Git directories.

---

## C++ MEX Development

This workspace also supports legacy MEX acceleration when working directly in MATLAB. All shared C++ algorithms reside in `System_Shared_Libraries/`. Always utilize the `BoilerplateMEXWrapper.m` pattern to ensure strict input validation and automatic compilation fallbacks.
