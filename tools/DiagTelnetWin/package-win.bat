@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT_DIR%package-win.ps1" -Runtime win-x64 -Configuration Release -Version v1.0.0

if errorlevel 1 (
  echo.
  echo Error al empaquetar. No se ha generado un paquete nuevo.
  pause
  exit /b 1
)

echo.
echo Paquete generado en dist\DiagTelnetWin-win-x64 y dist\DiagTelnetWin-win-x64.zip
pause
exit /b 0
