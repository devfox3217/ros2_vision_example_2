#!/bin/bash

echo "Building workspace..."
colcon build --symlink-install

if [ $? -eq 0 ]; then
    echo "Build successful. Sourcing setup file..."
    source install/setup.bash

    echo "Launching lane detection..."
    ros2 launch lane_detection lane_detection.launch.py
else
    echo "Build failed!"
    exit 1
fi