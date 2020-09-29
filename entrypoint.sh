#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/ws/install/setup.bash"

if [[ -n ${PACKAGE_NAME} ]]; then
  if [[ -n ${LAUNCHFILE_NAME} ]]; then
    echo "Running Launchfile: ${LAUNCHFILE_NAME} of package ${PACKAGE_NAME}"
    stdbuf -o L ros2 launch "${PACKAGE_NAME}" "${LAUNCHFILE_NAME}" "$@"
  else
    echo "Please set LAUNCHFILE_NAME:"
    echo "  ENV LAUNCHFILE_NAME=my_launchfile.launch.py"
    echo "  ENV NODE_NAME=my_node"
    echo "in your Dockerfile."
  fi
else
  echo "Please set PACKAGE_NAME and LAUNCHFILE_NAME:"
  echo "  ENV PACKAGE_NAME=my_package"
  echo "  ENV LAUNCHFILE_NAME=my_launchfile.launch.py"
  echo "in your Dockerfile."
fi
