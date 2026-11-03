#!/bin/bash
# Pasang stack mobil di Jetson Orin Nano (JetPack 6.2.x = Ubuntu 22.04). Jalankan sekali:
#   bash ~/ws/src/ksti_mav/jetson/setup.sh
# Repo diharapkan sudah di ~/ws/src/ksti_mav (git clone / salin via USB).
set -euo pipefail

. /etc/os-release
[ "$VERSION_ID" = "22.04" ] || { echo "Butuh Ubuntu 22.04 (JetPack 6.x). Terdeteksi $VERSION_ID"; exit 1; }

# 1. ROS 2 Humble (ros-base cukup; tanpa Gazebo — Gazebo hanya di laptop)
if [ ! -f /opt/ros/humble/setup.bash ]; then
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y ros-humble-ros-base ros-humble-camera-calibration python3-colcon-common-extensions
fi

# 2. Dependensi Python. OpenCV bawaan JetPack (dengan GStreamer) dipakai; jangan pip install opencv-python.
sudo apt-get install -y python3-serial python3-numpy
python3 -c "import cv2; assert 'GStreamer:                   YES' in cv2.getBuildInformation(), 'OpenCV tanpa GStreamer'" \
  || echo "PERINGATAN: OpenCV tanpa GStreamer -> kamera CSI tak bisa dibuka (pakai OpenCV bawaan JetPack)."

# 3. Akses serial tanpa sudo
sudo usermod -aG dialout "$USER"

# 4. Build
source /opt/ros/humble/setup.bash
cd ~/ws
colcon build --symlink-install --packages-select ksti_mav

# 5. Mode daya maksimum + jam maksimum saat lomba
sudo nvpmodel -m 0 || true
sudo jetson_clocks || true

# 6. Autostart saat mobil dinyalakan (lomba: tak boleh ada komunikasi -> semua jalan sendiri)
sudo cp ~/ws/src/ksti_mav/jetson/ksti-mav.service /etc/systemd/system/
sudo sed -i "s/__USER__/$USER/g" /etc/systemd/system/ksti-mav.service
sudo systemctl daemon-reload
echo
echo "Selesai. Aktifkan autostart:  sudo systemctl enable --now ksti-mav"
echo "Log langsung:                  journalctl -u ksti-mav -f"
echo "Logout/login sekali agar grup dialout aktif."
