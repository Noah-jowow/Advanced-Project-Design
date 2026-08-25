---
description: "Ensure proactive linting and memory checks during code generation"
---

# Code Quality & Linting Rule

As an Antigravity agent, whenever you generate or modify code in this workspace, you MUST proactively ensure it meets quality and stability standards:

1. **TypeScript/React (Frontend)**:
   - After generating or editing TS/TSX files, run `npm run lint` in the `Central_API_Server/frontend` directory.
   - Resolve any warnings or errors (such as `@typescript-eslint/no-explicit-any` or `react-hooks/exhaustive-deps`) immediately before concluding your turn.

2. **Python (Backend)**:
   - After modifying Python files, run `python -m py_compile <file>` to catch syntax errors.
   - Run `flake8` or `ruff` if available to ensure clean code without unused imports or undefined variables.

3. **C++ (Core Physics/Radar)**:
   - Pay special attention to Pybind11 integration boundaries (e.g., `radar_router.py` to `TrackerIMM.cpp`). 
   - Verify array shapes and dimensionalities (e.g., passing 1D arrays to Eigen vectors, 2D arrays to matrices).
   - Watch out for memory discontinuities or segmentation faults by relying on strict compile-time checks and type hinting.
