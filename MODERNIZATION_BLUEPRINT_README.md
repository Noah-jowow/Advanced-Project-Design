# Architectural Blueprint & Developer Guide
**System of Systems Development Methodology**

This document serves as the standard operating procedure (SOP) and exact structural blueprint for how this workspace is designed. 

**ATTENTION LLM AGENTS:** If you are an AI assistant reading this file, you must strictly adhere to this architectural pattern when requested to add a new project, mathematically upgrade an existing project, or modify the interface.

---

## 1. The Architecture: "Centralized Gateway, Decentralized Mathematics"
This environment operates as a highly scalable **System of Systems**. We do not run independent servers for each project.
1. **Mathematical Isolation (C++ & Eigen):** All heavy mathematics run natively in C++ microservices using the `Eigen` library for maximum speed.
2. **Middleware (Pybind11):** The C++ code is compiled into native Python extensions (`.pyd`).
3. **Monolithic API Gateway (FastAPI):** A single Python server (`Central_API_Server/main.py`) dynamically imports the `.pyd` extensions, stores the active system state in memory, and pipes the data through WebSockets.
4. **Single Page Application (React/Vite):** A unified frontend (`Central_API_Server/frontend`) reads the URL query parameters (e.g., `?project=RADAR`) to dynamically render the exact UI component requested without reloading the DOM.

---

## 2. How to Add a New Engineering Project
When adding a new mathematical simulation to this workspace, you must execute the following 5 phases in exact order:

### Phase 1: Construct the C++ Core
1. Create a new directory at the root (e.g., `Orbital_Mechanics/src/`).
2. Write pure, zero-dependency C++ code using `<Eigen/Dense>`. Do not include any Python or UI logic here. 
3. Implement `CMakeLists.txt` using `pybind11_add_module()` to compile the core into a Python extension.

### Phase 2: Expose via Pybind11
1. Inside your C++ directory, write an extension wrapper (e.g., `orbit_python_ext.cpp`).
2. Use `#include <pybind11/pybind11.h>` and `#include <pybind11/eigen.h>` to bind your structs and classes so Python can ingest them natively.

### Phase 3: Integrate into the FastAPI Gateway
1. Open `Central_API_Server/main.py`.
2. Add the path to your new `build` directory in the `setup_core_paths()` function so Python can find the `.pyd` file.
3. Add a `try/except ImportError` block to gracefully handle loading your module (e.g., `import orbit_core`).
4. Inside the `@app.websocket("/ws/stream/{domain}")` endpoint, add an `if domain == "orbit":` block to handle instantiation and command routing to your C++ core.

### Phase 4: Create the React Component
1. Open `Central_API_Server/frontend/src/components/`.
2. Create a new file (e.g., `OrbitPanel.tsx`). Use the custom `useWebSocket.ts` hook to connect to your new `domain`.
3. Use the `PlotCard.tsx` wrapper and `react-plotly.js` for data visualization. Use the workspace's Tailwind CSS industrial tokens (`bg-panel`, `text-accent-blue`, etc.).
4. In `App.tsx`, import your component. Add it to the `activeDomain` state hook fallback logic:
   `if (proj === 'ORBIT') return 'ORBIT';`

### Phase 5: Update the Launcher
1. Open `AeroSys_Launcher.bat` in the root directory.
2. Add your project to the CMake build sequence.
3. Add a new menu selection (e.g., `[4] Boot Orbital Mechanics`).
4. Map that selection to set `URL_TARGET=http://localhost:5173/?project=ORBIT` and `goto ULTRA_BOOT`.

---

## 3. Mathematical Parity Testing
Before finalizing any project, you must prove mathematical parity using Test-Driven Development.
1. Build a Pytest harness (e.g., `tests/test_parity.py`).
2. Use `scipy.io` or CSVs to import baseline data (usually from legacy MATLAB).
3. Execute the C++ Pybind11 module and assert exact numerical alignment using `numpy.testing.assert_allclose` with strict tolerances (`rtol=1e-5`).
