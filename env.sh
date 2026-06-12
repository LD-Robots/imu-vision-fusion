#!/usr/bin/env bash
# Source before launching: sets the domain so the pelvis BNO055 (/pelvis/imu) is visible,
# pins Cyclone DDS to match the Jetson, and overlays this workspace if built.
#
#   source env.sh
#
export ROS_DOMAIN_ID=62
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/jazzy/setup.bash

_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Unicast discovery peers: the AP drops wireless-to-wireless multicast,
# so laptop <-> NUC discovery needs this (see cyclonedds.xml).
export CYCLONEDDS_URI="file://${_WS_DIR}/cyclonedds.xml"
if [ -f "${_WS_DIR}/install/setup.bash" ]; then
  source "${_WS_DIR}/install/setup.bash"
fi
unset _WS_DIR
