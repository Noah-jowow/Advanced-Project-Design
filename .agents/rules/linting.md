---
name: Strict Post-Generation Review
description: Enforce linting, static analysis, and interface validation after any code generation or modification.
---

# Strict Post-Generation Review

Whenever you generate or modify code, you MUST follow these validation steps before proceeding:

1. **Frontend (TypeScript/React):**
   - Run `npm run lint` in the relevant frontend directory (e.g., `Central_API_Server/frontend`).
   - Resolve ALL warnings and errors immediately. Do not ignore them.

2. **Backend (Python):**
   - Run `python -m py_compile <file>` to check for syntax errors.
   - If available, use `flake8` or `ruff` to validate the code.

3. **C++ / Pybind11 Integrations:**
   - Validate C++ integrations for memory discontinuities.
   - Verify array shape matches between Python (NumPy) and C++ (Eigen).
   - Either run parity tests or statically analyze Pybind11 bindings before finalizing changes.
