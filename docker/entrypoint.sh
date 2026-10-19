#!/bin/bash
set -e

# Source ROS and any workspace that has been built.
# A missing install/ folder no longer kills the container.
source /opt/ros/humble/setup.bash

for ws in /dxl_ws /vins_ws; do
    if [ -f "${ws}/install/setup.bash" ]; then
        source "${ws}/install/setup.bash"
    fi
done

# Run whatever was passed (default: bash)
exec "$@"
