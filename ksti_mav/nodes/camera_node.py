"""Kamera mobil asli -> /camera/image_raw (bgr8). CSI (GStreamer nvarguscamerasrc) atau USB (indeks V4L2).

Parameter:
  source   string pipeline GStreamer, atau "0"/"1" untuk kamera USB
  width, height, fps
  K (9), D (5)  hasil `ros2 run camera_calibration cameracalibrator` -> gambar di-undistort sebelum dikirim
                (isi juga camera.fx/fy/cx/cy autopilot dengan nilai K yang sama)
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ksti_mav.nodes.common import bgr_to_image

CSI = ("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1640,height=1232,framerate=30/1 ! "
       "nvvidconv ! video/x-raw,width={w},height={h},format=BGRx ! videoconvert ! video/x-raw,format=BGR ! "
       "appsink drop=1 max-buffers=1")


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera")
        self.w = self.declare_parameter("width", 640).value
        self.h = self.declare_parameter("height", 480).value
        fps = self.declare_parameter("fps", 30).value
        src = self.declare_parameter("source", "csi").value
        K = list(self.declare_parameter("K", [0.0] * 9).value)
        D = list(self.declare_parameter("D", [0.0] * 5).value)
        if src == "csi":
            self.cap = cv2.VideoCapture(CSI.format(w=self.w, h=self.h), cv2.CAP_GSTREAMER)
        elif src.isdigit():
            self.cap = cv2.VideoCapture(int(src), cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            self.cap = cv2.VideoCapture(src, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(f"kamera tidak terbuka: {src} (cek kabel CSI / `ls /dev/video*`)")
        self.maps = None
        if any(D) and any(K):
            Km = np.array(K, float).reshape(3, 3)
            self.maps = cv2.initUndistortRectifyMap(Km, np.array(D, float), None, Km, (self.w, self.h), cv2.CV_16SC2)
        self.pub = self.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
        self.create_timer(1.0 / (fps * 2), self.grab)  # baca lebih cepat dari fps agar buffer tak menumpuk

    def grab(self):
        ok, img = self.cap.read()
        if not ok:
            self.get_logger().warn("frame gagal dibaca", throttle_duration_sec=2.0)
            return
        if img.shape[1] != self.w or img.shape[0] != self.h:
            img = cv2.resize(img, (self.w, self.h))
        if self.maps is not None:
            img = cv2.remap(img, *self.maps, cv2.INTER_LINEAR)
        self.pub.publish(bgr_to_image(img, self.get_clock().now().to_msg()))


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()
