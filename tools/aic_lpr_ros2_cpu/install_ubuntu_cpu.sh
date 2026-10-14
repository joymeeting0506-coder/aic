#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo 'First source your installed ROS 2: source /opt/ros/<distribution>/setup.bash'
  echo 'Installed distributions:'
  ls /opt/ros 2>/dev/null || true
  exit 1
fi
/usr/bin/python3 -c 'import sys; assert (3,10) <= sys.version_info[:2] <= (3,12), "This installer targets Ubuntu 22.04/24.04 system Python 3.10-3.12; report your Ubuntu/ROS version for other versions."'
sudo apt-get update
sudo apt-get install -y python3-venv
# Reuse ROS's system Python ABI, while keeping ML dependencies in this folder.
/usr/bin/python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
.venv/bin/python -m pip install 'ultralytics==8.4.177' 'opencv-python==4.11.0.86' 'numpy==1.26.4' Pillow
.venv/bin/python -c 'import rclpy, torch; from sensor_msgs.msg import Image; from std_msgs.msg import String; from predict import PlateReader; PlateReader(device="cpu"); print("ROS imports and both CPU models loaded successfully")'
echo 'Ready. Run: .venv/bin/python ros2_plate_node.py --ros-args -p image_topic:=/YOUR/CAMERA/TOPIC'
