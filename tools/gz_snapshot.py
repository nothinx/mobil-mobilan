"""Rekam kamera + BEV debug + state autopilot dari simulasi atau mobil asli yang sedang jalan.

  python3 tools/gz_snapshot.py <folder>              # satu snapshot
  python3 tools/gz_snapshot.py <folder> --record 30  # rekam 30 s: frame tiap 0.25 s + state.jsonl
"""
import argparse
import json
import pathlib
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ksti_mav.nodes.common import image_to_bgr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", type=pathlib.Path)
    ap.add_argument("--record", type=float, default=0.0, help="detik merekam (0 = satu snapshot)")
    ap.add_argument("--every", type=float, default=0.25)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("snapshot")
    last = {}
    node.create_subscription(Image, "/camera/image_raw", lambda m: last.__setitem__("cam", m), qos_profile_sensor_data)
    node.create_subscription(Image, "/autopilot/debug", lambda m: last.__setitem__("bev", m), qos_profile_sensor_data)
    node.create_subscription(String, "/autopilot/state", lambda m: last.__setitem__("state", m.data), 10)
    node.create_subscription(Odometry, "/odom", lambda m: last.__setitem__("odom", m), 10)

    def save(tag):
        imgs = [image_to_bgr(last[k]) for k in ("cam", "bev") if k in last]
        if imgs:
            h = imgs[0].shape[0]
            imgs = [cv2.resize(i, (int(i.shape[1] * h / i.shape[0]), h)) for i in imgs]
            cv2.imwrite(str(a.out / f"{tag}.jpg"), np.hstack(imgs))
        rec = {"tag": tag, "state": json.loads(last["state"]) if "state" in last else None}
        if "odom" in last:
            p = last["odom"].pose.pose.position
            rec["odom"] = [round(p.x, 3), round(p.y, 3)]
        return rec

    t0 = time.time()
    while len(last) < 3 and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.05)
    if a.record <= 0:
        print(json.dumps(save("snap")))
    else:
        with open(a.out / "state.jsonl", "w") as f:
            t_next, t0 = 0.0, time.time()
            while time.time() - t0 < a.record:
                rclpy.spin_once(node, timeout_sec=0.02)
                el = time.time() - t0
                if el >= t_next:
                    f.write(json.dumps(save(f"{el:06.2f}")) + "\n")
                    t_next += a.every
    rclpy.shutdown()


if __name__ == "__main__":
    main()
