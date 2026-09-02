@echo off
setlocal

cd /d "%~dp0\.."

echo ============================================================
echo  KINGPHOENIX GCS - PORTABLE WINDOWS BUILD
echo ============================================================

where py >nul 2>nul
if errorlevel 1 (
  echo [ERROR] Python launcher "py" not found.
  echo Install Python 3.11 or 3.12 x64 first.
  pause
  exit /b 1
)

if not exist ".venv-gcs-build\Scripts\python.exe" (
  echo [BUILD] Creating isolated build environment...
  py -3.12 -m venv .venv-gcs-build 2>nul
  if errorlevel 1 py -3.11 -m venv .venv-gcs-build
  if errorlevel 1 (
    echo [ERROR] Could not create Python 3.11/3.12 environment.
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
