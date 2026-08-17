# Advanced Engineering Project Descriptions for LinkedIn

This document contains professionally curated descriptions of the projects found in the `Advanced-Project-Design` workspace. These descriptions highlight technical depth, mathematical rigor, and architectural sophistication.

---

## 1. Omni Radar Director: Unified System of Systems Simulation
**Role:** Lead Developer / Systems Architect
**Technologies:** MATLAB, Handle Graphics, Eigen (C++), IMM-EKF, Digital Signal Processing (DSP)

### Summary
A comprehensive System-of-Systems simulation pipeline for advanced radar sensor modeling and multi-target tracking, leveraging an IMM-EKF backend and high-performance C++ MEX signal processing.

Generates complex target signatures from system inputs including active jammers, multi-polarization radar cross-sections (RCS), and maneuvering 7-state trajectories.

Allows the parameterization of AESA/MPAR array synthesis, digital beamforming tapers (Hamming/Hann), and robotic effector dynamics including 6-DOF Stewart Platforms and Az/El Gimbals.

Performance metrics visualized through real-time PPI/RHI plots, CA-CFAR detection heatmaps, and STAP cancellation profiles. Track robustness and state estimation errors are plotted over time alongside a 3D tactical view of the engagement envelope.

### Key Technical Contributions
*   **Multi-Model Tracking:** Implemented an **Interacting Multiple Model Extended Kalman Filter (IMM-EKF)** with a 7-state vector to maintain robust tracks on maneuvering targets.
*   **Advanced DSP Pipeline:** Engineered a comprehensive signal processing chain including **Space-Time Adaptive Processing (STAP)**, **Matched Filtering (MF)**, **Doppler MTI**, and **CA-CFAR** detection.
*   **Phased Array Synthesis:** Designed a vectorized synthesis engine for AESA and MPAR systems, supporting **True Time Delay (TTD)**, phase squint modeling, and custom element weighting (Hamming/Hann tapers).
*   **Robotic Effector Dynamics:** Integrated Newton-Euler rigid body dynamics for a **6-DOF Stewart Platform (Hexapod)** and Euler-Lagrange gimbals for high-precision beam pointing.
*   **EW Environment:** Simulated spectral environments with active jammers and dual-polarization signature classification for target identification.

### Skills Used
*   **Engineering Domains:** Radar Systems, Digital Signal Processing (DSP), Control Systems, Robotics & Kinematics.
*   **Methodologies:** Kalman Filtering (IMM-EKF), Adaptive Filtering (STAP), Numerical Integration (ODE45/Euler).
*   **Software & Tools:** MATLAB, Handle Graphics, Phased Array System Toolbox, Signal Processing Toolbox.

---

## 2. Enterprise 3D Unstructured FVM CFD Solver
**Role:** Computational Fluid Dynamics Researcher
**Technologies:** C++ (OpenMP), MATLAB MEX, Eigen, GPU/VRAM Streaming, Algebraic Multigrid (AMG)

### Summary
This project involved the architecture and development of a professional-grade 3D Unstructured Finite Volume Method (FVM) CFD solver, specifically optimized for high-performance aerodynamic research. Designed to bridge the gap between commercial black-box tools and custom research code, the solver emphasizes mathematical transparency and extreme computational efficiency.

The core of the solver is a high-performance back-end composed of lock-free, OpenMP-threaded C++ MEX extensions. These extensions handle computationally intensive tasks such as Jacobian matrix construction and spatial flux evaluations, delivering speedups of several orders of magnitude over native interpreted implementations. To further accelerate convergence, the solver utilizes a deep-hierarchy Smoothed Aggregation Algebraic Multigrid (SA-AMG) algorithm.

To ensure industrial relevance, the solver integrates advanced physics models such as Delayed Detached Eddy Simulation (DDES) and the SST k-omega turbulence model. The results are visualized through a custom GPU-accelerated architecture that streams volumetric heatmaps and Q-criterion vortex data directly to the user interface, enabling rapid iteration on complex 3D aerodynamic geometries.

### Key Technical Contributions
*   **High-Performance Back-end:** Developed lock-free, **OpenMP-threaded C++ MEX extensions** for Jacobian computations and spatial flux evaluations, achieving significant speedups over native MATLAB implementations.
*   **Algebraic Multigrid (AMG):** Implemented a deep hierarchy **Smoothed Aggregation AMG (SA-AMG)** solver with recursive Galerkin projections to accelerate convergence on complex 3D meshes.
*   **Advanced Physics Modeling:** Integrated **Delayed Detached Eddy Simulation (DDES)** and the SST k-omega turbulence model to capture complex separation and wake dynamics.
*   **Vectorized Mesh Processing:** Engineered an unstructured mesh generation and acceleration suite, utilizing **C++ graph extraction** for strength-of-connection coarsening.
*   **GPU Visualization:** Implemented a persistent VBO (Vertex Buffer Object) streaming architecture for real-time 3D volumetric heatmaps of Mach, Pressure, and Q-criterion.

### Skills Used
*   **High-Performance Computing (HPC):** C++, OpenMP Multi-threading, MATLAB MEX API, Eigen Library.
*   **Numerical Methods:** Finite Volume Method (FVM), Algebraic Multigrid (AMG), Sparse Matrix Algebra.
*   **CFD Physics:** Turbulence Modeling (DDES/SST), Compressible Flow (AUSM+), Numerical Flux Schemes.
*   **Visualization:** OpenGL/VBO Concepts, Scientific Visualization, GPU Data Streaming.

---

## 3. Commercial-Grade Rocket Nozzle Optimizer
**Role:** Propulsion Systems Engineer
**Technologies:** MATLAB, Genetic Algorithms, Method of Characteristics (MoC), Conjugate Heat Transfer (CHT)

### Summary
A rigorous propulsion design pipeline for Rao Thrust-Optimized Parabola (TOP) synthesis, integrating Method of Characteristics (MoC) aerodynamics with high-fidelity thermal and structural validation.

Generates optimal nozzle contours from propellant input parameters (LOX/LH2, LOX/LCH4) and thermally perfect gas properties, including boundary layer displacement corrections.

Allows the parameterization of structural constraints (Lamé thick-wall stress) and thermal gradients (Bartz/Eckert models) before executing a multi-objective Genetic Algorithm (GA) for Isp optimization.

Design validity visualized through temperature-dependent Von Mises stress heatmaps and Mach number contour plots. Specific Impulse, thrust efficiency, and Margin of Safety are plotted across the optimization generations alongside a 2D slice of the internal flow expansion.

### Key Technical Contributions
*   **Exact Aero Synthesis:** Developed a **Method of Characteristics (MoC)** solver for exact Rao TOP contour construction, accounting for 2D divergence losses and boundary layer displacement effects.
*   **Rigorous Thermal Analysis:** Implemented a **Conjugate Heat Transfer (CHT)** model using the **Bartz Sigma correction factor** and Eckert Reference Temperature method for compressible turbulent boundary layers.
*   **Structural Verification:** Integrated **Lamé thick-wall mechanics** and Von Mises stress analysis with temperature-dependent material properties to calculate real-time Margins of Safety (MoS).
*   **Thermally Perfect Chemistry:** Built a vectorized database for thermally perfect gas properties (variable gamma and Cp) supporting advanced propellants like LOX/LH2 and LOX/Methane.
*   **Evolutionary Optimization:** Leveraged a custom **Genetic Algorithm (GA)** to optimize nozzle geometry for maximum Specific Impulse (Isp) within mass and thermal constraints.

### Skills Used
*   **Propulsion Engineering:** Rocket Propulsion Elements, Gas Dynamics, Thermochemistry.
*   **Thermodynamics & Fluids:** Conjugate Heat Transfer (CHT), Boundary Layer Theory, Method of Characteristics (MoC).
*   **Optimization & AI:** Genetic Algorithms (GA), Constrained Optimization, Parameter Tuning.
*   **Mechanical Analysis:** Structural Mechanics (Von Mises), Material Selection (High-Temp Alloys/Composites).

---

## 4. Deep Space Mission Architect & MGA Router
**Role:** Mission Designer / Full-Stack Engineer
**Technologies:** Python (Poliastro), React Three Fiber (R3F), FastAPI, Astrodynamics, N-Body Physics

### Summary
An end-to-end interplanetary mission design pipeline for Multi-Gravity Assist (MGA) trajectory optimization, utilizing a high-fidelity N-body physics engine and React Three Fiber 3D visualization.

Generates optimal launch windows and flyby sequences from destination inputs including all major planets, 20+ moons, and L1 Lagrange points via JPL SPICE ephemeris.

Allows the parameterization of spacecraft ballistic coefficients (SRP modeling), mass budgets, and multi-leg transfer constraints before executing a Lambert-solver-based global search.

Mission manifest visualized through 3D interactive tactical maps, Delta-V budget summaries, and Keplerian element profiles. Trajectory drift and Mid-Course Correction (MCC) requirements are plotted over the mission timeline alongside a real-time kinematic interpolation of the interplanetary coast.

### Key Technical Contributions
*   **MGA Trajectory Optimization:** Built a robust search engine using **Lambert’s problem** solvers to identify optimal gravity-assist sequences (e.g., Earth-Venus-Mars) to minimize propellant requirements.
*   **High-Fidelity Perturbation Modeling:** Implemented an N-body physics engine that accounts for **Third-Body Gravitational Perturbations** from all major planets and **Solar Radiation Pressure (SRP)**.
*   **Mid-Course Correction (MCC) Budgeting:** Quantified trajectory drift through numerical integration, allowing for precise Delta-V allocations for non-Keplerian effects.
*   **Lagrange Point & Moon Targeting:** Engineered support for non-planetary destinations, including **L1 Lagrange points** and a comprehensive database of 20+ moons with fallback Keplerian elements.
*   **Interactive 3D Visualizer:** Designed a professional mission control dashboard using **React Three Fiber (R3F)** and **Three.js**, providing real-time kinematic interpolation of planetary and spacecraft states.

### Skills Used
*   **Astrodynamics:** Lambert’s Problem, Orbital Mechanics, Ephemeris Querying (JPL SPICE), Perturbation Theory.
*   **Software Development:** Python (FastAPI/Poliastro), JavaScript (React/Vite/Three.js), CSS (Modern Dark UI).
*   **Mathematics:** Numerical Integration, Coordinate Transformations (ICRF to Ecliptic), Ballistic Coefficient Modeling.
*   **Mission Design:** Launch Window Analysis, Delta-V Budgeting, Gravity Assist Phasing.

---

## 5. Architectural Modernization & Microservice Integration
**Role:** DevOps & Software Architect
**Technologies:** Python (FastAPI), C++, Docker, CI/CD, pybind11, Eigen

### Summary
A strategic DevOps modernization pipeline for transitioning monolithic engineering tools into containerized microservice architectures, leveraging FastAPI and pybind11 for cross-language parity.

Generates scalable backend services from legacy MATLAB/C++ logic, utilizing Eigen-accelerated microservices and multi-stage Docker builds.

Allows the parameterization of API endpoints, middleware wrappers, and automated CI/CD security analysis before deploying a unified System-of-Systems network.

Architectural integrity visualized through mathematical parity test reports (1e-8 tolerance), system health dashboards, and performance benchmarks. Resource utilization and request latency are plotted across the containerized network alongside a dependency map of the decoupled microservices.

### Key Technical Contributions
*   **Microservice Scaffolding:** Decoupled monolithic MATLAB scripts into focused services using **FastAPI** gateways and optimized C++ back-ends.
*   **Bidirectional Middleware:** Developed high-performance wrappers using **pybind11** (Python) and **MEX/clib** (MATLAB) to expose core C++/Eigen DSP logic to multiple execution environments.
*   **Mathematical Parity Testing:** Built a rigorous TDD harness using **Pytest** and binary serialization to guarantee algorithmic perfection (within 1e-8 tolerance) during code migration.
*   **Containerized Toolchain:** Engineered multi-stage **Dockerfiles** and orchestrated local networks with **Docker Compose**, significantly reducing the environment setup overhead for complex simulations.
*   **CI/CD Pipeline:** Implemented automated pipelines for compilation, security analysis (`cppcheck`, `bandit`), and automated mathematical verification.

### Skills Used
*   **Software Engineering:** Python (FastAPI), C++, Microservice Architecture, API Design.
*   **DevOps & Infrastructure:** Docker, Docker Compose, CI/CD Pipelines, Linux Toolchains.
*   **Testing & QA:** Test-Driven Development (TDD), Pytest, Mathematical Parity Verification.
*   **Middleware:** pybind11, MATLAB C++ API (MEX), Binary Serialization.

---
 
 ## 6. SOLMAR: Solar Weather Detection & Martian Asset Protection
 **Role:** Lead Design Engineer (Capstone)
 **Technologies:** Systems Engineering, Mars-Sun L1 Astrodynamics, Sensor Integration, PDR/CDR Methodology
 
 ### Summary
 A full systems engineering design and verification pipeline for a Mars-Sun L1 hosted payload (SOLMAR) dedicated to autonomous solar weather early-detection and planetary asset protection.
 
 Synthesizes a modular payload architecture from mission requirements including multi-spectral solar monitoring, real-time alert low-latency telemetry, and autonomous threshold-based decision making.
 
 Allows the parameterization of sensor sensitivity (Magnetometer/Faraday Cup), orbital positioning (L1 stability), and structural interface constraints before executing a system-level trade study for mass and power optimization.
 
 System readiness visualized through mission-consequence matrices, sensor coverage heatmaps, and autonomous alert-level profiles. Design integrity is verified through PDR/CDR-level trade studies and functional verification of the sensor-support electronics interface.
 
 ### Key Technical Contributions
 *   **Lagrange Point Mission Design:** Engineered a mission architecture for a hosted payload at the **Mars-Sun Lagrange Point 1 (L1)** to provide early warning of inbound Coronal Mass Ejections (CMEs) and solar flares.
 *   **Multi-Spectral Sensor Suite:** Integrated a diverse sensor package including a **Magnetometer**, **Faraday Cup**, **Optical Camera**, and **Radio Antenna** to detect high-energy solar particles and electromagnetic pulses.
 *   **Autonomous Alert Logic:** Developed encoded safety thresholds for real-time Martian asset protection, enabling automated safe-mode transitions and power-system management via low-latency alerts.
 *   **Hosted Payload Integration:** Designed a structural and electrical interface compatible with host spacecraft standards, ensuring robust power delivery and data relay for long-duration deep space operations.
 *   **Verification & Validation:** Led the development of the **Preliminary Design Report (PDR)** and **System Specification**, managing trade studies for sensor selection and structural mass budgets.
 
 ### Skills Used
 *   **Space Systems Engineering:** Requirements Decomposition, Trade Studies, PDR/CDR Processes, Interface Control.
 *   **Astrodynamics:** Interplanetary Mission Design, Lagrange Point Stability, Orbital Maneuvering.
 *   **Hardware Integration:** Multi-Sensor Fusion, Space-Grade Electronics, Structural Mounting Design.
 *   **Project Management:** Team Leadership (AE 427 Capstone), Deliverable Tracking, Technical Documentation.
