@echo off
setlocal

set SCRIPT_DIR=%~dp0

powershell -ExecutionPolicy Bypass -File "%SCRIPT_DIR%publish-win.ps1" -Runtime win-x64 -Configuration Release
if errorlevel 1 (
  echo.
  echo Error durante la publicacion.
  pause
  exit /b 1
)

echo.
echo Publicacion finalizada correctamente.
pause
