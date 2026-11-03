"""Hanya simulasi: /cmd_vel -> sudut setir kiri/kanan (geometri Ackermann) + kecepatan roda belakang (diferensial).

Pengganti plugin AckermannSteering Fortress 6.x yang setirnya lambat (~2 s ke 90 %, tanpa parameter gain).
Setir dibatasi laju `servo_rate` agar mirip servo asli; sendi digerakkan JointPositionController.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class SimDrive(Node):
    def __init__(self):
        super().__init__("sim_drive")
        p = self.declare_parameter
        self.L = p("wheelbase", 0.26).value
        self.k = p("kingpin_width", 0.14).value
        self.s = p("wheel_separation", 0.16).value
        self.r = p("wheel_radius", 0.033).value
        self.max_steer = p("max_steer", 0.60).value
        self.rate = p("servo_rate", 5.0).value  # rad/s, servo digital ~0.1 s/60 deg tanpa beban
        self.v = self.target = self.delta = 0.0
        self.pub = {n: self.create_publisher(Float64, f"/sim/{n}", 10)
                    for n in ("steer_left", "steer_right", "wheel_left", "wheel_right")}
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.dt = 0.01
        self.create_timer(self.dt, self.tick)

    def on_cmd(self, m):
        self.v = m.linear.x
        if abs(self.v) > 0.02:
            self.target = max(-self.max_steer, min(self.max_steer, math.atan(self.L * m.angular.z / self.v)))

    def tick(self):
        step = self.rate * self.dt
        self.delta += max(-step, min(step, self.target - self.delta))
        d = self.delta
        if abs(d) < 1e-4:
            dl = dr = d
            fl = fr = 1.0
        else:
            R = self.L / math.tan(d)  # radius poros belakang, + = belok kiri
            dl = math.atan(self.L / (R - self.k / 2))
            dr = math.atan(self.L / (R + self.k / 2))
            fl, fr = (R - self.s / 2) / R, (R + self.s / 2) / R
        w = self.v / self.r
        for n, val in (("steer_left", dl), ("steer_right", dr), ("wheel_left", w * fl), ("wheel_right", w * fr)):
            self.pub[n].publish(Float64(data=val))


def main():
    rclpy.init()
    node = SimDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
