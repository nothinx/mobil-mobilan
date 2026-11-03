"""Mobil asli (Jetson Orin Nano): kamera + autopilot + jembatan serial ke MCU.

  ros2 launch ksti_mav robot.launch.py                    # misi Autonomous Race
  ros2 launch ksti_mav robot.launch.py challenge:=true    # misi Challenge (hindari obstacle)
Parameter mobil di config/robot.yaml (WAJIB diisi hasil ukur & kalibrasi).
Mobil menunggu tombol START fisik; e-stop fisik memutus daya motor + memberi sinyal ke MCU.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("ksti_mav"), "config", "robot.yaml")
    challenge = LaunchConfiguration("challenge")
    return LaunchDescription([
        DeclareLaunchArgument("challenge", default_value="false"),
        Node(package="ksti_mav", executable="camera", parameters=[cfg], output="screen", respawn=True),
        Node(package="ksti_mav", executable="autopilot", output="screen", respawn=True,
             parameters=[cfg, {"pilot.avoid": PythonExpression(["'", challenge, "' == 'true'"])}]),
        Node(package="ksti_mav", executable="serial_bridge", parameters=[cfg], output="screen", respawn=True),
    ])
