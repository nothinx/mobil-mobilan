# Simulasi di laptop: ROS 2 Humble + Gazebo Fortress (pasangan resmi). Bukan untuk Jetson.
FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz python3-opencv python3-serial python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ksti_mav", "sim.launch.py"]
