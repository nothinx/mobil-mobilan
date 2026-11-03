"""Marka berwarna (start/finish, checkpoint) dan obstacle, dari citra BEV yang sama dengan deteksi lajur."""
from dataclasses import dataclass

import cv2
import numpy as np

from .lane import signed_offset


@dataclass
class MarkConfig:
    # HSV OpenCV (H 0..180). Start/finish "belum tentu merah" kata panitia -> ubah saat technical meeting.
    finish_hsv: tuple = ((0, 110, 70, 10, 255, 255), (170, 110, 70, 180, 255, 255))
    checkpoint_hsv: tuple = ((40, 80, 50, 85, 255, 255),)
    band_x: tuple = (0.45, 0.75)   # pita dekat tempat warna diukur [m]
    corridor: float = 0.22         # |y - garis tengah| yang dianggap di dalam lintasan [m]; di luar garis tepi diabaikan
    frac_on: float = 0.05          # fraksi piksel berwarna di pita agar dianggap terlihat
    obstacle_min: float = 0.06     # obstacle lebih lebar dari marka mana pun [m]
    obstacle_contrast: int = 45
    obstacle_x_min: float = 0.50   # moncong mobil sendiri jangan dianggap obstacle
    obstacle_x_max: float = 1.30


def hsv_mask(hsv, ranges):
    m = np.zeros(hsv.shape[:2], bool)
    for r in ranges:
        m |= cv2.inRange(hsv, np.array(r[:3]), np.array(r[3:])) > 0
    return m


class Debounce:
    """Rising edge setelah `on` frame berturut-turut; lepas setelah `off` frame kosong."""

    def __init__(self, on=2, off=6):
        self.on, self.off, self.n_on, self.n_off, self.state = on, off, 0, 0, False

    def update(self, seen):
        if seen:
            self.n_on, self.n_off = self.n_on + 1, 0
        else:
            self.n_on, self.n_off = 0, self.n_off + 1
        if not self.state and self.n_on >= self.on:
            self.state = True
            return True
        if self.state and self.n_off >= self.off:
            self.state = False
        return False


@dataclass
class Obstacle:
    x: float        # jarak depan dari poros belakang [m]
    y: float
    lateral: float  # relatif garis tengah lintasan (+ = kiri)


class MarkDetector:
    def __init__(self, lane_detector, cfg: MarkConfig | None = None):
        self.cfg = c = cfg or MarkConfig()
        self.ld = lane_detector
        lc = lane_detector.cfg
        self.row_x, self.col_y = lane_detector.row_x, lane_detector.col_y
        self.band = (self.row_x >= c.band_x[0]) & (self.row_x <= c.band_x[1])
        k = int(round(c.obstacle_min / lc.res)) | 1
        self.k_obs = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        self.min_area = (c.obstacle_min / lc.res) ** 2
        self.k_mark = np.ones((15, 15), np.uint8)  # ~7 cm: tepi buram marka jauh
        self.finish = Debounce()
        self.checkpoint = Debounce()

    def _corridor(self, path):
        return self.ld.corridor(path, self.cfg.corridor)

    def update(self, lane):
        """-> (finish_event, checkpoint_event, Obstacle|None) untuk satu frame."""
        c = self.cfg
        hsv = cv2.cvtColor(lane.bev, cv2.COLOR_BGR2HSV)
        corr = self._corridor(lane.path) & lane.valid
        band = corr & self.band[:, None]
        n = max(1, int(band.sum()))
        red = hsv_mask(hsv, c.finish_hsv)
        green = hsv_mask(hsv, c.checkpoint_hsv)
        fin = self.finish.update((red & band).sum() / n > c.frac_on)
        cp = self.checkpoint.update((green & band).sum() / n > c.frac_on)

        gray = cv2.cvtColor(lane.bev, cv2.COLOR_BGR2GRAY).astype(np.int16)
        floor = float(np.median(gray[corr])) if corr.sum() > 200 else 0.0  # lantai lintasan, bukan sekelilingnya
        odd = (np.abs(gray - floor) > c.obstacle_contrast) | (hsv[:, :, 1] > 90)
        # marka berwarna TIPIS (start/finish, checkpoint) + tepi buramnya bukan obstacle. Blob berwarna yang
        # besar tetap obstacle (obstacle merah/hijau jangan sampai hilang).
        col = (red | green).astype(np.uint8)
        thin = col & (1 - cv2.morphologyEx(col, cv2.MORPH_OPEN, self.k_obs))
        excl = cv2.dilate(thin, self.k_mark) > 0
        odd &= corr & ~excl & ((self.row_x >= c.obstacle_x_min) & (self.row_x <= c.obstacle_x_max))[:, None]
        odd = cv2.morphologyEx(odd.astype(np.uint8), cv2.MORPH_OPEN, self.k_obs)
        n_cc, lab, st, _ = cv2.connectedComponentsWithStats(odd, connectivity=8)
        best = None
        for i in range(1, n_cc):
            if st[i, 4] < self.min_area:
                continue
            bottom = st[i, 1] + st[i, 3] - 1  # baris terdekat ke mobil
            if best is None or bottom > best[1]:
                best = (i, bottom)
        obs = None
        if best:
            i, bottom = best
            rows = slice(max(0, bottom - int(0.03 / self.ld.cfg.res)), bottom + 1)
            cols = np.nonzero(lab[rows] == i)[1]
            x = float(self.row_x[bottom])
            y = float(self.col_y[cols].mean())
            lat = signed_offset(np.array([[x, y]]), lane.path, max_along=0.5)[0]
            obs = Obstacle(x, y, 0.0 if np.isnan(lat) else float(lat))
        return fin, cp, obs
