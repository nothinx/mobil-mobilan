"""Jembatan Jetson <-> MCU (firmware/mcu_bridge) lewat USB serial.

/cmd_vel (v, yaw rate) -> sudut roda (model sepeda) -> pulsa servo & ESC [mikrodetik] -> "C <servo_us> <esc_us>\\n"
MCU -> "B START" (tombol start) -> /race/start ; "E 1"/"E 0" (e-stop ditekan/dilepas) -> /race/estop
Bila /cmd_vel berhenti datang > cmd_timeout, kirim netral. MCU sendiri punya watchdog 300 ms.

Semua angka *_us = KALIBRASI per mobil (docs/MEKANIKAL-ELEKTRIKAL.md bagian kalibrasi).
"""
import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Empty


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")
        p = self.declare_parameter
        self.port = p("port", "/dev/ttyUSB0").value
        self.baud = p("baud", 115200).value
        self.wb = p("wheelbase", 0.26).value
        self.max_steer = p("max_steer", 0.60).value
        self.servo_center = p("servo_center_us", 1500).value
        self.servo_per_rad = p("servo_us_per_rad", 800.0).value   # tanda negatif bila setir terbalik
        self.servo_min = p("servo_min_us", 1000).value
        self.servo_max = p("servo_max_us", 2000).value
        self.esc_neutral = p("esc_neutral_us", 1500).value
        self.esc_per_mps = p("esc_us_per_mps", 120.0).value       # tanda negatif bila motor terbalik
        self.esc_start = p("esc_deadband_us", 50).value           # pulsa minimum agar roda mulai berputar
        self.esc_max = p("esc_max_us", 1750).value                # batas atas keamanan
        self.timeout = p("cmd_timeout", 0.25).value
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        self.last = (self.servo_center, self.esc_neutral)
        self.t_cmd = 0.0
        self.pub_start = self.create_publisher(Empty, "/race/start", 10)
        self.pub_estop = self.create_publisher(Bool, "/race/estop", 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(0.05, self.tick)
        threading.Thread(target=self.reader, daemon=True).start()
        self.get_logger().info(f"serial {self.port} @ {self.baud}")

    def on_cmd(self, m):
        v, w = m.linear.x, m.angular.z
        steer = math.atan(self.wb * w / v) if abs(v) > 0.05 else 0.0
        steer = max(-self.max_steer, min(self.max_steer, steer))
        servo = int(round(self.servo_center + self.servo_per_rad * steer))
        servo = max(self.servo_min, min(self.servo_max, servo))
        if v > 0.02:
            esc = self.esc_neutral + math.copysign(self.esc_start + abs(self.esc_per_mps) * v, self.esc_per_mps)
        else:
            esc = self.esc_neutral  # ponytail: tanpa rem aktif/mundur; tambah bila ESC mendukung & perlu
        esc = int(round(max(2 * self.esc_neutral - self.esc_max, min(self.esc_max, esc))))
        self.last = (servo, esc)
        self.t_cmd = time.monotonic()
        self.send(*self.last)

    def tick(self):
        if time.monotonic() - self.t_cmd > self.timeout:
            self.send(self.servo_center, self.esc_neutral)

    def send(self, servo, esc):
        try:
            self.ser.write(f"C {servo} {esc}\n".encode())
        except serial.SerialException as e:
            self.get_logger().error(f"serial: {e}", throttle_duration_sec=2.0)

    def reader(self):
        while rclpy.ok():
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except serial.SerialException:
                time.sleep(0.2)
                continue
            if line == "B START":
                self.pub_start.publish(Empty())
                self.get_logger().info("tombol START")
            elif line.startswith("E "):
                self.pub_estop.publish(Bool(data=line.endswith("1")))
                self.get_logger().warn(f"E-STOP {'DITEKAN' if line.endswith('1') else 'dilepas'}")


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(node.servo_center, node.esc_neutral)
        node.destroy_node()
        rclpy.try_shutdown()
