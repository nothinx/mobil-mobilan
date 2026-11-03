"""Node autopilot: /camera/image_raw -> AutoPilot -> /cmd_vel (Twist: v, yaw rate).

Sama persis di Gazebo dan mobil asli. Topik:
  sub  /camera/image_raw (sensor_msgs/Image)   /race/start (std_msgs/Empty)   /race/estop (std_msgs/Bool)
  pub  /cmd_vel (geometry_msgs/Twist)   /autopilot/state (std_msgs/String, JSON)   /autopilot/debug (Image BEV)
Parameter: camera.*, lane.*, marks.*, pilot.* (lihat dataclass di ksti_mav/core) + auto_start_s, debug_every.
"""
import json
import math

import cv2
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, String

from ksti_mav.core.geometry import CameraConfig
from ksti_mav.core.lane import LaneConfig
from ksti_mav.core.marks import MarkConfig
from ksti_mav.core.pilot import AutoPilot, PilotConfig
from ksti_mav.nodes.common import bgr_to_image, declare_dataclass, image_to_bgr


class AutopilotNode(Node):
    def __init__(self):
        super().__init__("autopilot")
        self.cam = declare_dataclass(self, "camera", CameraConfig())
        lane = declare_dataclass(self, "lane", LaneConfig())
        marks = declare_dataclass(self, "marks", MarkConfig())
        pc = declare_dataclass(self, "pilot", PilotConfig())
        self.auto_start = self.declare_parameter("auto_start_s", -1.0).value  # sim: start otomatis
        self.debug_every = int(self.declare_parameter("debug_every", 3).value)
        self.pilot = AutoPilot(self.cam.make(), lane, marks, pc)
        self.wb = pc.wheelbase
        self.t0 = None
        self.n = 0
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(String, "/autopilot/state", 10)
        self.pub_dbg = self.create_publisher(Image, "/autopilot/debug", qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_subscription(Empty, "/race/start", lambda _: self.pilot.start(self.now()), 10)
        self.create_subscription(Bool, "/race/estop", lambda m: self.pilot.estop(m.data), 10)
        self.get_logger().info(f"autopilot siap: kamera {self.cam}, v_max {pc.v_max}, avoid {pc.avoid}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_image(self, msg):
        img = image_to_bgr(msg)
        if img.shape[:2] != (self.cam.height, self.cam.width):
            img = cv2.resize(img, (self.cam.width, self.cam.height))
        t = self.now()
        if self.t0 is None:
            self.t0 = t
        if self.auto_start >= 0 and self.pilot.state == "WAIT" and t - self.t0 >= self.auto_start:
            self.pilot.start(t)
            self.get_logger().info("start otomatis")
        prev = self.pilot.state
        cmd = self.pilot.step(img, t)
        tw = Twist()
        tw.linear.x = cmd.v
        tw.angular.z = cmd.v * math.tan(cmd.steer) / self.wb
        self.pub_cmd.publish(tw)
        if cmd.state != prev:
            self.get_logger().info(f"state {prev} -> {cmd.state}")
        self.pub_state.publish(String(data=json.dumps({"state": cmd.state, "steer": round(cmd.steer, 3), **cmd.info})))
        self.n += 1
        if self.debug_every > 0 and self.n % self.debug_every == 0 and self.pilot.last is not None:
            self.pub_dbg.publish(bgr_to_image(self.pilot.lane.draw(self.pilot.last), msg.header.stamp, "bev"))


def main():
    rclpy.init()
    node = AutopilotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_cmd.publish(Twist())  # berhenti saat node dimatikan
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
