"""Simulasi Gazebo Fortress: lintasan contoh KSTI + mobil + autopilot + wasit.

  ros2 launch ksti_mav sim.launch.py                         # marka putih, lantai gelap
  ros2 launch ksti_mav sim.launch.py world:=ksti_light       # marka hitam, lantai terang
  ros2 launch ksti_mav sim.launch.py world:=ksti_dark_challenge   # + obstacle
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    from ksti_mav.core.track import Track

    share = get_package_share_directory("ksti_mav")
    world = LaunchConfiguration("world").perform(context)
    auto = float(LaunchConfiguration("auto_start").perform(context))
    challenge = "challenge" in world
    x, y, yaw = Track.load(os.path.join(share, "config", "track.json")).start_pose(back=0.05)
    models = os.path.join(share, "models")
    res = os.pathsep.join(p for p in (models, os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")) if p)
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {os.path.join(share, 'worlds', world + '.sdf')}"}.items())
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-name", "mav_car", "-file", os.path.join(models, "mav_car", "model.sdf"),
                            "-x", f"{x:.3f}", "-y", f"{y:.3f}", "-z", "0.01", "-Y", f"{yaw:.3f}"])
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen", arguments=[
        "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
        "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
    ])
    sim = {"use_sim_time": True}
    autopilot = Node(package="ksti_mav", executable="autopilot", output="screen",
                     parameters=[sim, {"auto_start_s": auto, "pilot.avoid": challenge}])
    referee = Node(package="ksti_mav", executable="referee", output="screen",
                   parameters=[sim, {"challenge": challenge}])
    return [SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", res), gz, spawn, bridge, autopilot, referee]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="ksti_dark",
                              description="ksti_dark | ksti_light | ksti_dark_challenge | ksti_light_challenge"),
        DeclareLaunchArgument("auto_start", default_value="4.0", description="detik sebelum start otomatis"),
        OpaqueFunction(function=setup),
    ])
