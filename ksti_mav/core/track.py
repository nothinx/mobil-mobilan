"""Lintasan: dibangun dari gambar lintasan panitia, dipakai wasit simulasi & penempatan mobil.

Koordinat dunia: asal di tengah gambar, x ke kanan, y ke atas (kolom/baris gambar: v ke bawah).
Garis tengah tersimpan sebagai poligon tertutup berjarak rata, searah arah balap (CCW).
"""
import json
import math

import numpy as np


class Track:
    def __init__(self, data):
        self.d = data
        self.mpp = data["m_per_px"]
        self.w_px, self.h_px = data["size_px"]
        self.center = np.asarray(data["centerline"], float)  # (N,2) meter
        self.width = np.asarray(data["width"], float)        # (N,) meter
        seg = np.diff(np.vstack([self.center, self.center[:1]]), axis=0)
        self.seglen = np.hypot(seg[:, 0], seg[:, 1])
        self.s = np.concatenate([[0], np.cumsum(self.seglen)[:-1]])
        self.length = float(self.seglen.sum())
        self.start_idx = data["start_idx"]

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            return cls(json.load(f))

    def world_to_px(self, x, y):
        return x / self.mpp + self.w_px / 2, self.h_px / 2 - y / self.mpp

    def heading(self, i):
        a, b = self.center[i - 1], self.center[(i + 1) % len(self.center)]
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def start_pose(self, back=0.0):
        """Pose (x, y, yaw) poros belakang; `back` = mundur sekian meter dari garis start."""
        i = self.start_idx
        yaw = self.heading(i)
        x, y = self.center[i]
        return x - back * math.cos(yaw), y - back * math.sin(yaw), yaw

    def project(self, x, y):
        """-> (s dari start [m], offset lateral [m, + = kiri], lebar lintasan [m])."""
        d = np.hypot(self.center[:, 0] - x, self.center[:, 1] - y)
        i = int(np.argmin(d))
        h = self.heading(i)
        dx, dy = x - self.center[i, 0], y - self.center[i, 1]
        lat = -math.sin(h) * dx + math.cos(h) * dy
        s = (self.s[i] - self.s[self.start_idx]) % self.length
        return s, lat, float(self.width[i])


def build_from_raster(gray, track_width=0.50, start_px=None, n_checkpoints=8):
    """Gambar abu-abu lintasan (marka terang di lantai gelap) -> dict data Track.

    Garis tepi = dua komponen putih dengan kotak pembatas terbesar; garis tengah = potongan
    garis putus-putus. Skala dinormalkan agar median lebar lintasan = `track_width`.
    """
    import cv2

    white = (gray > 128).astype(np.uint8)
    n, lab, st, cen = cv2.connectedComponentsWithStats(white, connectivity=8)
    bbox = st[1:, 2] * st[1:, 3]
    outer, inner = (np.argsort(-bbox)[:2] + 1).tolist()

    def filled(comp):
        cs, _ = cv2.findContours((lab == comp).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        m = np.zeros_like(white)
        cv2.drawContours(m, cs, -1, 1, cv2.FILLED)
        return m

    road = filled(outer) & (1 - filled(inner))
    dash_area = np.median(st[1:, 4])  # komponen terbanyak = potongan garis putus-putus
    pts = np.array([cen[i] for i in range(1, n)
                    if i not in (outer, inner) and st[i, 4] < 4 * dash_area
                    and road[int(cen[i][1]), int(cen[i][0])]])

    # rantai tetangga terdekat -> poligon tertutup
    order, left = [0], set(range(1, len(pts)))
    while left:
        last = pts[order[-1]]
        j = min(left, key=lambda k: (pts[k][0] - last[0]) ** 2 + (pts[k][1] - last[1]) ** 2)
        order.append(j)
        left.remove(j)
    poly = pts[order]

    # resample rata + haluskan melingkar
    closed = np.vstack([poly, poly[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    step = 8.0  # px
    ss = np.arange(0, s[-1], step)
    c = np.stack([np.interp(ss, s, closed[:, 0]), np.interp(ss, s, closed[:, 1])], 1)
    k = np.exp(-0.5 * (np.arange(-12, 13) / 4.0) ** 2)
    k /= k.sum()
    c = np.stack([np.convolve(np.concatenate([c[-12:, i], c[:, i], c[:12, i]]), k, "valid") for i in (0, 1)], 1)

    # lebar = jarak ke garis tepi luar + dalam (+ setengah lebar marka masing-masing)
    cs, _ = cv2.findContours((lab == outer).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    line_w = st[outer, 4] / cv2.arcLength(max(cs, key=len), True)  # cincin tipis: luas / keliling
    d_out = cv2.distanceTransform((lab != outer).astype(np.uint8), cv2.DIST_L2, 5)
    d_in = cv2.distanceTransform((lab != inner).astype(np.uint8), cv2.DIST_L2, 5)
    ci = np.clip(c.round().astype(int), 0, [gray.shape[1] - 1, gray.shape[0] - 1])
    width_px = d_out[ci[:, 1], ci[:, 0]] + d_in[ci[:, 1], ci[:, 0]] + line_w
    mpp = track_width / float(np.median(width_px))

    h, w = gray.shape
    world = np.stack([(c[:, 0] - w / 2) * mpp, (h / 2 - c[:, 1]) * mpp], 1)
    x, y = world[:, 0], world[:, 1]
    if 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) < 0:  # paksa CCW
        world, c, width_px = world[::-1], c[::-1], width_px[::-1]

    start_idx = 0
    if start_px is not None:
        start_idx = int(np.argmin(np.hypot(c[:, 0] - start_px[0], c[:, 1] - start_px[1])))

    # radius belok (lingkaran lewat 3 titik berjarak ~20 cm)
    m = max(1, int(round(0.2 / (step * mpp))))
    a, b, d = np.roll(world, m, 0), world, np.roll(world, -m, 0)
    ab, bd, ad = np.hypot(*(a - b).T), np.hypot(*(b - d).T), np.hypot(*(a - d).T)
    cross = np.abs((b - a)[:, 0] * (d - a)[:, 1] - (b - a)[:, 1] * (d - a)[:, 0])
    radius = ab * bd * ad / np.maximum(2 * cross, 1e-9)

    n_pts = len(world)
    return {
        "m_per_px": mpp,
        "size_px": [w, h],
        "size_m": [w * mpp, h * mpp],
        "line_width_m": line_w * mpp,
        "centerline": world.round(4).tolist(),
        "width": (width_px * mpp).round(4).tolist(),
        "start_idx": start_idx,
        "checkpoints_idx": [(start_idx + n_pts * (i + 1) // (n_checkpoints + 1)) % n_pts for i in range(n_checkpoints)],
        "stats": {
            "length_m": float(np.sum(np.hypot(*np.diff(np.vstack([world, world[:1]]), axis=0).T))),
            "width_min_m": float(width_px.min() * mpp), "width_max_m": float(width_px.max() * mpp),
            "min_radius_m": float(radius.min()),
        },
    }
