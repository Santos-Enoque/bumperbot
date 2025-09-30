#!/bin/bash
# Manual workspace setup script to bypass the corrupted setup.bash

# Set up ROS 2 Jazzy environment manually
export ROS_DISTRO=jazzy
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Set up paths
export AMENT_PREFIX_PATH="/opt/ros/jazzy"
export CMAKE_PREFIX_PATH="/opt/ros/jazzy/opt/gz_math_vendor:/opt/ros/jazzy/opt/gz_utils_vendor:/opt/ros/jazzy/opt/gz_cmake_vendor"
export LD_LIBRARY_PATH="/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib"
export PATH="/opt/ros/jazzy/bin:$PATH"
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages"

# Add workspace paths
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -d "$WORKSPACE_ROOT/install" ]; then
    export AMENT_PREFIX_PATH="$WORKSPACE_ROOT/install:$AMENT_PREFIX_PATH"
    export PATH="$WORKSPACE_ROOT/install/bin:$PATH"
    
    # Add workspace libraries to path if they exist
    if [ -d "$WORKSPACE_ROOT/install/lib" ]; then
        export LD_LIBRARY_PATH="$WORKSPACE_ROOT/install/lib:$LD_LIBRARY_PATH"
    fi
    
    # Add workspace Python packages if they exist
    if [ -d "$WORKSPACE_ROOT/install/lib/python3.12/site-packages" ]; then
        export PYTHONPATH="$WORKSPACE_ROOT/install/lib/python3.12/site-packages:$PYTHONPATH"
    fi
    
    # Source package-specific setup scripts if they exist
    if ls "$WORKSPACE_ROOT/install"/*/share/*/local_setup.bash 1> /dev/null 2>&1; then
        for setup_script in "$WORKSPACE_ROOT/install"/*/share/*/local_setup.bash; do
            if [ -f "$setup_script" ]; then
                source "$setup_script"
            fi
        done
    fi
fi

# Force RMW implementation after all setup scripts
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "ROS 2 Jazzy workspace environment set up manually (RMW: $RMW_IMPLEMENTATION)"