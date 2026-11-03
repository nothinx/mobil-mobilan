"""Tes inti tanpa ROS (jalan di Windows/Linux/Jetson):  python tests/test_core.py   (atau pytest)

Yang dijaga: geometri kamera, rumus skor guidebook, kecocokan model Gazebo <-> config, dan satu lap
penuh di sim 2D (lintasan gelap + terang + challenge) tanpa pelanggaran.
"""
import math
import pathlib
import re
import sys

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from ksti_mav.core.geometry import CameraConfig  # noqa: E402
from ksti_mav.core.pilot import PilotConfig  # noqa: E402
from ksti_mav.core.score import penalty, s_ar, s_arc, total  # noqa: E402


def test_ground_roundtrip():
    cam = CameraConfig().make()
    g = np.array([1.0, 0.2, 1.0])
    p = cam.H_g2i @ g
    p /= p[2]
    assert 0 < p[0] < cam.w and 0 < p[1] < cam.h
    assert p[0] < cam.w / 2  # titik di kiri mobil tampak di kiri gambar
    back = np.linalg.inv(cam.H_g2i) @ p
    assert np.allclose(back / back[2], g, atol=1e-9)


def test_score_formulas():
    assert penalty("keluar", 8) == 30.0
    assert math.isclose(s_ar(60, 8), 14.4)
    assert s_ar(400, 8) == 8  # lewat batas waktu: tanpa bonus waktu
    assert math.isclose(s_arc(0, 8, 8), 1.0)  # challenge sempurna = +1.0 pengali
    assert math.isclose(total(10, True, 0.5), 17.5)


def test_sdf_matches_config():
    sdf = (ROOT / "models/mav_car/model.sdf").read_text(encoding="utf-8")
    cam, pc = CameraConfig(), PilotConfig()
    pose = re.search(r'<sensor name="camera".*?<pose>([^<]+)</pose>', sdf, re.S).group(1).split()
    assert [float(v) for v in pose] == [cam.cam_x, 0, cam.cam_h, 0, cam.pitch, 0]
    assert float(re.search(r"<horizontal_fov>([^<]+)<", sdf).group(1)) == cam.hfov
    assert f"<width>{cam.width}</width><height>{cam.height}</height>" in sdf
    assert float(re.search(r"<wheel_base>([^<]+)<", sdf).group(1)) == pc.wheelbase
    assert float(re.search(r"<steering_limit>([^<]+)<", sdf).group(1)) == pc.max_steer


def _lap(**kw):
    from sim2d import Sim
    r = Sim(**kw).run()
    assert r["finished"] and r["cp"] == r["tcp"] and not r["violations"], r
    return r


def test_lap_dark():
    _lap()


def test_lap_light():
    _lap(light=True)


def test_lap_challenge():
    _lap(challenge=True)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
