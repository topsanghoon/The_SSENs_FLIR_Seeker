@echo off
echo Simple Click Sender for Net_RxThread Test
echo ==========================================
echo.

REM Check if Python is installed
C:/Users/DSO19/AppData/Local/Programs/Python/Python313/python.exe --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.x from https://python.org
    pause
    exit /b 1
)

echo Installing required packages...
C:/Users/DSO19/AppData/Local/Programs/Python/Python313/python.exe -m pip install opencv-python numpy

if %errorlevel% neq 0 (
    echo ERROR: Failed to install required packages
    echo Please install manually: pip install opencv-python numpy
    pause
    exit /b 1
)

echo.
echo Starting Simple Click Sender...
echo Edit simple_click_sender.py to set your Linux PC IP address
echo.

C:/Users/DSO19/AppData/Local/Programs/Python/Python313/python.exe simple_click_sender.py
pause