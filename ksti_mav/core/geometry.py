"""Kamera pinhole di atas bidang tanah datar: homografi piksel <-> tanah, dan citra bird's-eye (BEV).

Frame mobil (konvensi ROS): x maju, y kiri, z atas; asal = titik tanah di bawah poros roda belakang
(titik acuan pure pursuit). Frame optik kamera: z depan, x kanan, y bawah.
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class CameraConfig:
    """Default = kamera di models/mav_car/model.sdf (tes test_sdf_sync menjaga keduanya sama).
    Mobil asli: isi dari hasil ukur + kalibrasi (lihat docs/MEKANIKAL-ELEKTRIKAL.md)."""
    width: int = 640
    height: int = 480
    hfov: float = 1.60     # rad (~92 deg; lensa IMX219 160 deg setelah undistort ~ 100 deg)
    cam_x: float = 0.22    # m di depan poros belakang
    cam_h: float = 0.24    # m tinggi lensa dari lantai
    pitch: float = 0.55    # rad menunduk (~31 deg)
    fx: float = 0.0        # 0 = hitung dari hfov; isi dari kalibrasi kamera asli
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0

    def make(self):
        return GroundCamera(self.width, self.height, self.hfov, self.cam_x, self.cam_h, self.pitch,
                            fx=self.fx or None, fy=self.fy or None, cx=self.cx or None, cy=self.cy or None)


class GroundCamera:
    def __init__(self, width, height, hfov, cam_x, cam_h, pitch, cam_y=0.0, fx=None, fy=None, cx=None, cy=None):
        """hfov [rad]; cam_x/cam_y/cam_h [m] posisi lensa di frame mobil; pitch [rad] positif = menunduk.
        fx..cy dari kalibrasi kamera asli (opsional; default dari hfov, piksel persegi)."""
        self.w, self.h = int(width), int(height)
        f = (self.w / 2) / math.tan(hfov / 2)
        self.K = np.array([[fx or f, 0, self.w / 2 if cx is None else cx],
                           [0, fy or f, self.h / 2 if cy is None else cy],
                           [0, 0, 1.0]])
        c, s = math.cos(pitch), math.sin(pitch)
        zc = np.array([c, 0, -s])          # sumbu optik
        xc = np.array([0, -1.0, 0])        # kanan gambar = -y mobil
        yc = np.cross(zc, xc)              # bawah gambar
        self.R = np.stack([xc, yc, zc])    # mobil -> kamera
        self.C = np.array([cam_x, cam_y, cam_h])
        Rt = np.column_stack([self.R[:, 0], self.R[:, 1], -self.R @ self.C])
        self.H_g2i = self.K @ Rt            # [X, Y, 1] tanah -> piksel (homogen)

    def bev_homography(self, x_min, x_max, y_half, res):
        """Homografi piksel BEV -> piksel kamera. Baris 0 BEV = x_max (jauh), kolom 0 = y kiri."""
        B = np.array([[0, -res, x_max], [-res, 0, y_half], [0, 0, 1.0]])
        size = (int(round(2 * y_half / res)), int(round((x_max - x_min) / res)))  # (lebar, tinggi)
        return self.H_g2i @ B, B, size

    def pixel_rays_hit_ground(self):
        """Mask baris/kolom citra kamera yang sinarnya mengenai tanah (di bawah horizon)."""
        u, v = np.meshgrid(np.arange(self.w) + 0.5, np.arange(self.h) + 0.5)
        d = np.linalg.inv(self.K) @ np.stack([u.ravel(), v.ravel(), np.ones(u.size)])
        dz = (self.R.T @ d)[2]
        return (dz < -1e-3).reshape(self.h, self.w)
