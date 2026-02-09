@echo off
echo Building workspace...
call colcon build --symlink-install

if %ERRORLEVEL% EQU 0 (
    echo Build successful. Sourcing setup file...
    call install\setup.bat

    echo Launching lane detection...
    ros2 launch lane_detection lane_detection.launch.py
) else (
    echo Build failed!
    exit /b 1
)