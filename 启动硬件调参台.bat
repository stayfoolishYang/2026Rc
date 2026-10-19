@echo off
setlocal
cd /d "%~dp0"
if defined TUNING_PYTHON goto run_configured
where py >nul 2>&1
if not errorlevel 1 goto run_py_launcher
where python >nul 2>&1
if not errorlevel 1 goto run_python
echo 未找到 Python。请安装 Python 3.10 或更高版本，并勾选“Add Python to PATH”。
pause
exit /b 1

:run_configured
"%TUNING_PYTHON%" "%~dp0stm32_hardware_tuning_gui.py" %*
exit /b %errorlevel%

:run_py_launcher
py -3 "%~dp0stm32_hardware_tuning_gui.py" %*
exit /b %errorlevel%

:run_python
exit /b %errorlevel%
endlocal
