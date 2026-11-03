"""Wasit Gazebo: pose ground-truth (/odom dari OdometryPublisher, frame dunia) -> checkpoint, keluar jalur,
waktu, skor S_AR sesuai guidebook. Hasil di /referee/status (JSON) dan log."""
import json
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from ksti_mav.core.score import Referee, total
from ksti_mav.core.track import Track


class RefereeNode(Node):
    def __init__(self):
        super().__init__("referee")
        share = get_package_share_directory("ksti_mav")
        self.tr = Track.load(os.path.join(share, "config", "track.json"))
        challenge = self.declare_parameter("challenge", False).value
        self.wb = self.declare_parameter("wheelbase", 0.26).value
        self.ref = Referee(self.tr, self.declare_parameter("car_width", 0.19).value, 0.5 if challenge else 0.3)
        self.started = False
        self.pub = self.create_publisher(String, "/referee/status", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_timer(0.5, self.report)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = p.x + self.wb / 2 * math.cos(yaw), p.y + self.wb / 2 * math.sin(yaw)
        if not self.started:
            if math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) < 0.02:
                return  # waktu mulai saat mobil mulai bergerak
            self.started = True
            self.ref.reset(self.now())
            self.ref.s_prev = self.tr.project(x, y)[0]
            self.get_logger().info("wasit: mobil bergerak, waktu berjalan")
        for e in self.ref.update(x, y, self.now()):
            self.get_logger().info(f"wasit: {e} (cp {self.ref.cp}, pelanggaran {len(self.ref.violations)})")

    def report(self):
        if self.started:
            r = self.ref.result(self.now())
            r["total_if_ackermann"] = round(total(r["S_AR"]), 3)
            self.pub.publish(String(data=json.dumps(r)))


def main():
    rclpy.init()
    node = RefereeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
