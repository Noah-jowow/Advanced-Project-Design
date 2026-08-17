# Advanced Project Design: System Architecture

## Overview
This workspace is designed using a **System of Systems** architecture. It integrates multiple complex engineering disciplines (Aerodynamics, Propulsion, Radar, Orbital Mechanics, and Control Systems) into a unified, high-performance ecosystem. 

The original codebase consisted of massive, monolithic MATLAB scripts. The current architectural goal is to decouple these domains into independent, highly optimized C++ and Python microservices that communicate via a central FastAPI Gateway.

---

## 1. Directory & Repository Structure
The workspace is split into six independent Git repositories, each representing a distinct engineering domain:

- **`Aerodynamics_CFD/`**: 2D/3D Euler and Navier-Stokes flow solvers.
- **`Propulsion/`**: Chemical rocket thermodynamic analysis and converging-diverging nozzle optimization.
- **`Radar_Systems/`**: Phased array synthesis, pulsed Doppler Digital Signal Processing (DSP), and kinematic tracking.
- **`Tracker/`**: Control system laws (PID, LQR) and CAD hardware models for physical target tracking systems.
- **`ToolDev/`**: Orbital mechanics (Lambert's Problem solvers), astrodynamics engines, and the future central API Gateway.
- **`Career_Development/`**: An NPM Monorepo housing local React/Vite frontends for job tracking, resumes, and career analytics.

*Note: The root `Advanced-Project-Design` folder acts as an umbrella directory. It contains a global `.gitignore` to prevent nested git conflicts and this `ARCHITECTURE.md` file. Each domain folder above is its own independent git repository.*

---

## 2. Core Architectural Principles

### A. The "C++ as the Source of Truth" Rule
To achieve real-time simulation performance, the mathematical bottlenecks of the legacy MATLAB scripts are being ported into C++ utilizing the `Eigen` library for linear algebra. The C++ source code is considered the ultimate ground truth for the mathematical and physical behavior of the systems.

### B. Python API Bindings (Pybind11)
The C++ core engines are not compiled into standalone executables. Instead, they are compiled into Python extension modules (`.pyd` / `.so`) using `pybind11`. This allows the central Python API (housed in `ToolDev`) to rapidly call the C++ physics engines as if they were native Python functions.

### C. Rigorous Mathematical Documentation
All code strictly adheres to Professional Engineering standards:
- Derivations for complex equations are included in the source code comments.
- Approximations (e.g., Patched Conic approximation, linearized Jacobians, constant-gamma expansions) must be explicitly stated.
- Published literature references (e.g., *Bartz 1957*, *Wang 2000*) are mandated for empirical formulas.

---

## 3. Data Flow & The API Gateway
*(Currently in Transition)*

1. **Front-End**: A user interacts with a React frontend (or a legacy MATLAB UI).
2. **Gateway**: The request hits the central FastAPI Gateway (Python).
3. **Dispatch**: The Gateway identifies the domain (e.g., a Radar DSP request) and calls the pre-compiled C++ extension for that domain.
4. **Execution**: The C++ engine executes the rigorous math (e.g., a Matched Filter or CFAR detection) natively using CPU-optimized `Eigen` routines.
5. **Response**: The results are serialized back to Python and returned to the UI.

---

## 4. Legacy Archive Strategy
Legacy MATLAB monoliths (such as `OmniRadarDirector.m` and `Rocket_Nozzle_Optimizer.m`) are retained for historical reference and algorithm parity checks. They are kept alongside the modern C++ implementations but are considered deprecated for active production development.

---
*Maintained under strict adherence to the agent-principles and professional engineering guidelines.*
