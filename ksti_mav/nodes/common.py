"""Utilitas bersama node ROS: dataclass <-> parameter, konversi Image tanpa cv_bridge."""
from dataclasses import fields, replace

import numpy as np
from sensor_msgs.msg import Image


def declare_dataclass(node, prefix, dc):
    """Deklarasikan field skalar dataclass sebagai parameter `prefix.field`; kembalikan salinan berisi nilai ROS."""
    vals = {}
    for f in fields(dc):
        v = getattr(dc, f.name)
        if isinstance(v, (bool, int, float, str)):
            vals[f.name] = type(v)(node.declare_parameter(f"{prefix}.{f.name}", v).value)
    return replace(dc, **vals)


def image_to_bgr(msg: Image):
    ch = 4 if msg.encoding in ("bgra8", "rgba8") else 3
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    a = a.reshape(msg.height, msg.width, ch)[:, :, :3]
    if msg.encoding.startswith("rgb"):
        a = a[:, :, ::-1]
    return np.ascontiguousarray(a)


def bgr_to_image(img, stamp, frame_id="camera"):
    m = Image()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.height, m.width = img.shape[:2]
    m.encoding = "bgr8"
    m.step = m.width * 3
    m.data = np.ascontiguousarray(img).tobytes()
    return m
