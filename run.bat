@echo off
setlocal EnableDelayedExpansion
rem ===========================================================================
rem  Urban Traffic Simulator - one-shot build & run (Windows).
rem
rem  Usage:
rem    run.bat                     build (if needed), serve small_map on :8080
rem    run.bat rebuild             wipe build\ and reconfigure first
rem    run.bat <map.geojson>       serve a specific map
rem    run.bat <map.geojson> <port>
rem
rem  Needs CMake + a compiler (Ninja + g++ from MinGW-w64). If they aren't on
rem  PATH this script also looks in C:\mingw64\bin.
rem ===========================================================================

rem Always run from the repo root (this script's own folder).
cd /d "%~dp0"

rem Make the MinGW toolchain reachable even if it isn't on the system PATH.
where cmake >nul 2>&1 || set "PATH=C:\mingw64\bin;%PATH%"
where cmake >nul 2>&1
if errorlevel 1 (
    echo [error] cmake was not found on PATH or in C:\mingw64\bin.
    echo         Install CMake + MinGW-w64, or edit the PATH line in this script.
    exit /b 1
)

rem ---- parse args ----
set "MAP=data\maps\small_map.geojson"
set "PORT=8080"
if /i "%~1"=="rebuild" (
    echo Removing build\ for a clean rebuild...
    if exist build rmdir /s /q build
    shift
)
if not "%~1"=="" set "MAP=%~1"
if not "%~2"=="" set "PORT=%~2"

rem ---- configure (first time only) ----
if not exist build\CMakeCache.txt (
    echo Configuring CMake ^(Ninja^)...
    cmake -S . -B build -G Ninja
    if errorlevel 1 (
        echo Ninja generator failed; retrying with the default generator...
        cmake -S . -B build
        if errorlevel 1 (
            echo [error] CMake configuration failed. See the messages above.
            exit /b 1
        )
    )
)

rem ---- build ----
echo Building...
cmake --build build
if errorlevel 1 (
    echo [error] Build failed. See the compiler messages above.
    exit /b 1
)

if not exist "build\traffic_simulator.exe" (
    echo [error] build\traffic_simulator.exe was not produced.
    exit /b 1
)

rem ---- run ----
echo.
echo ============================================================
echo   Serving %MAP%
echo   Open http://localhost:%PORT%   ^(Ctrl+C here to stop^)
echo ============================================================
echo.
rem Open the browser a moment later, once the server has bound the port.
start "" cmd /c "timeout /t 2 /nobreak >nul & start http://localhost:%PORT%"

build\traffic_simulator.exe "%MAP%" %PORT%
