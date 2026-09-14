@echo off
setlocal

cd /d "%~dp0\.."

echo ============================================================
echo  KINGPHOENIX GCS - PORTABLE WINDOWS BUILD
echo ============================================================

REM Prefer the currently active Python interpreter. This allows the script to
REM run directly from an already-active project .venv instead of requiring the
REM Windows py launcher to have a specific 3.11/3.12 installation registered.
where python >nul 2>nul
if errorlevel 1 (
  echo [ERROR] "python" not found in PATH.
  echo Activate your project virtual environment first, or install Python x64.
  pause
  exit /b 1
)

for /f "delims=" %%P in ('python -c "import sys; print(sys.executable)"') do set "SOURCE_PYTHON=%%P"
for /f "delims=" %%V in ('python -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"') do set "SOURCE_VERSION=%%V"

echo [BUILD] Source Python: %SOURCE_PYTHON%
echo [BUILD] Python version: %SOURCE_VERSION%

REM Build into a dedicated environment so packaging dependencies do not pollute
REM the normal GCS runtime environment. It is created using the Python that is
REM already active in this terminal.
if not exist ".venv-gcs-build\Scripts\python.exe" (
  echo [BUILD] Creating isolated build environment from active Python...
  "%SOURCE_PYTHON%" -m venv .venv-gcs-build
  if errorlevel 1 (
    echo [ERROR] Could not create .venv-gcs-build from the active Python runtime.
    echo Try: python -m venv .venv-gcs-build
    pause
    exit /b 1
  )
)

call .venv-gcs-build\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install -r gcs\requirements-gcs-build.txt
if errorlevel 1 goto :fail

echo [BUILD] Cleaning old artifacts...
if exist build\Kingphoenix-GCS rmdir /s /q build\Kingphoenix-GCS
if exist dist\Kingphoenix-GCS rmdir /s /q dist\Kingphoenix-GCS

echo [BUILD] Building portable onedir application...
python -m PyInstaller ^
  --noconfirm ^
  --clean ^
  --onedir ^
  --windowed ^
  --name Kingphoenix-GCS ^
  --paths gcs ^
  --add-data "gcs\background.png;." ^
  --collect-all dearpygui ^
  --collect-all av ^
  --collect-all cv2 ^
  --collect-all numpy ^
  gcs\kingphoenix_gcs_app.py
if errorlevel 1 goto :fail

echo.
echo ============================================================
echo  BUILD SUCCESS
echo  Portable folder: dist\Kingphoenix-GCS\
echo  Run: dist\Kingphoenix-GCS\Kingphoenix-GCS.exe
echo ============================================================
echo Copy the ENTIRE Kingphoenix-GCS folder to another Windows x64 laptop.
pause
exit /b 0

:fail
echo.
echo [ERROR] Kingphoenix GCS build failed.
pause
exit /b 1
