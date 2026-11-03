#!/bin/bash
# Bangun paket dari sumber yang di-mount (/ws/src/ksti_mav) lalu jalankan perintah.
set -e
source /opt/ros/humble/setup.bash
cd /ws
colcon build --symlink-install --packages-select ksti_mav > /tmp/build.log 2>&1 || { cat /tmp/build.log; exit 1; }
source /ws/install/setup.bash
exec "$@"
