@echo off
setlocal EnableDelayedExpansion
title AeroSys v3.0 - Engineering Workbench

:MENU
cls
echo =======================================================
echo          AEROSYS v3.0 - SYSTEM OF SYSTEMS
echo =======================================================
echo.
echo Please select a project to boot:
echo.
echo   [1] Boot Radar C2 Workspace
echo   [2] Boot Aerodynamics CFD Workspace
echo   [3] Boot Rocket Propulsion Workspace
echo   [Q] Quit
echo.
set /p "choice=Enter your choice [1-3, Q]: "

if /I "%choice%"=="1" (
    set URL_TARGET=http://localhost:5173/?project=RADAR
    goto ULTRA_BOOT
)
if /I "%choice%"=="2" (
    set URL_TARGET=http://localhost:5173/?project=CFD
    goto ULTRA_BOOT
)
if /I "%choice%"=="3" (
    set URL_TARGET=http://localhost:5173/?project=PROPULSION
    goto ULTRA_BOOT
)
if /I "%choice%"=="Q" exit
goto MENU


:ULTRA_BOOT
echo.
echo Executing One-Click Ultra-Boot Sequence...
echo.

echo [1/3] Compiling and verifying C++ Cores (CMake/Ninja)...
cd Radar_Systems
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release >nul 2>&1
cmake --build build >nul 2>&1
cd ..\Aerodynamics_CFD
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release >nul 2>&1
cmake --build build >nul 2>&1
cd ..\Propulsion
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release >nul 2>&1
cmake --build build >nul 2>&1
cd ..

echo [2/3] Booting Secure Local Backend (FastAPI on 127.0.0.1)...
start "AeroSys Backend (Python)" cmd /k "cd Central_API_Server && py main.py"

echo [3/3] Booting Commercial React Frontend (Vite)...
start "AeroSys Frontend (React)" cmd /k "cd Central_API_Server\frontend && npm run dev"

echo Waiting for servers to initialize...
timeout /t 5 >nul
start "" "%URL_TARGET%"

echo.
echo =======================================================
echo Launch sequence complete!
echo The Engineering Workbench is now open in your browser.
echo You may close this launcher window.
echo =======================================================
timeout /t 3 >nul
exit
