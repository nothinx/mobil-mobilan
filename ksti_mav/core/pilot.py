"""Autopilot: persepsi -> keputusan balap -> pure pursuit. Satu kelas dipakai ROS node, sim 2D, dan tes."""
import math
from dataclasses import dataclass, field

import numpy as np

from .lane import LaneConfig, LaneDetector, curvature, offset_polyline
from .marks import MarkConfig, MarkDetector


@dataclass
class PilotConfig:
    wheelbase: float = 0.26        # [m] ukur di sasis asli
    max_steer: float = 0.60        # [rad] sudut roda maksimum; contoh lintasan butuh >= 0.6 (34 deg)
    v_max: float = 1.2             # [m/s]
    v_min: float = 0.35
    a_lat: float = 1.8             # percepatan lateral maks untuk batas kecepatan tikungan [m/s^2]
    ld_min: float = 0.45           # lookahead pure pursuit [m]
    ld_max: float = 0.95
    ld_gain: float = 0.35          # ld = ld_min + ld_gain * v
    lost_stop_s: float = 0.6       # lajur hilang lebih lama dari ini -> berhenti
    race_offset: float = 0.0       # garis balap relatif garis tengah lintasan (+ kiri)
    avoid: bool = True             # aktifkan penghindaran obstacle (misi challenge)
    avoid_offset: float = 0.14
    avoid_trigger_x: float = 1.30  # mulai menghindar bila obstacle lebih dekat dari ini [m]
    v_avoid: float = 0.7           # kecepatan saat menghindar; lookahead dipendekkan ke ld_min
    avoid_hold_m: float = 0.55     # tetap di lajur hindar sejauh ini setelah obstacle hilang
    offset_rate: float = 2.0       # laju perubahan offset lateral [m/s]
    min_lap_s: float = 8.0         # garis finish diabaikan sebelum ini (mobil start di atasnya)
    laps: int = 1
    finish_run_m: float = 0.9      # jarak tetap jalan setelah garis finish terdeteksi di depan


@dataclass
class Command:
    v: float
    steer: float       # sudut roda depan [rad], + = kiri
    state: str
    info: dict = field(default_factory=dict)


class AutoPilot:
    def __init__(self, cam, lane_cfg=None, mark_cfg=None, cfg=None):
        self.cfg = cfg or PilotConfig()
        self.lane = LaneDetector(cam, lane_cfg or LaneConfig())
        self.marks = MarkDetector(self.lane, mark_cfg or MarkConfig())
        self.state = "WAIT"
        self.t_start = None
        self.t_last = None
        self.t_seen = None
        self.steer = 0.0
        self.v = 0.0
        self.offset = self.cfg.race_offset
        self.avoid_target = None
        self.hold_left = 0.0
        self.laps_done = 0
        self.checkpoints = 0
        self.finish_left = None
        self.last = None

    def start(self, t):
        if self.state == "WAIT":
            self.state, self.t_start, self.t_seen = "RUN", t, t

    def estop(self, on=True):
        if on:
            self.state = "ESTOP"
        elif self.state == "ESTOP":
            self.state = "WAIT"  # setelah e-stop dilepas tunggu tombol start lagi (aturan: taruh di checkpoint lalu start)

    def step(self, bgr, t) -> Command:
        c = self.cfg
        dt = 0.0 if self.t_last is None else max(0.0, t - self.t_last)
        self.t_last = t
        self.lane.predict(self.v * dt, self.v * math.tan(self.steer) / c.wheelbase * dt)
        lane = self.lane.detect(bgr)
        fin, cp, obs = self.marks.update(lane)
        self.last = lane
        info = {"conf": round(lane.conf, 2), "pol": lane.polarity, "obs": None}

        if self.state != "RUN":
            self.v = 0.0
            return Command(0.0, 0.0, self.state, info)

        # --- kejadian balap ---
        if cp:
            self.checkpoints += 1
        if fin and t - self.t_start > c.min_lap_s:
            self.laps_done += 1
            if self.laps_done >= c.laps:
                self.finish_left = c.finish_run_m  # garis terlihat di depan: lewati dulu baru berhenti
        if self.finish_left is not None:
            self.finish_left -= self.v * dt
            if self.finish_left <= 0:
                self.state = "FINISH"
                return Command(0.0, 0.0, self.state, info)

        # --- penghindaran obstacle: pindah ke lajur yang kosong ---
        target = c.race_offset
        if c.avoid and obs is not None and obs.x < c.avoid_trigger_x:
            info["obs"] = (round(obs.x, 2), round(obs.lateral, 2))
            if self.avoid_target is None:
                self.avoid_target = c.avoid_offset if obs.lateral < 0 else -c.avoid_offset
            self.hold_left = c.avoid_hold_m
        elif self.avoid_target is not None:
            self.hold_left -= self.v * dt
            if self.hold_left <= 0:
                self.avoid_target = None
        if self.avoid_target is not None:
            target = self.avoid_target
        step = c.offset_rate * dt
        self.offset += float(np.clip(target - self.offset, -step, step))

        # --- lajur hilang ---
        if lane.conf > 0.15:
            self.t_seen = t
        elif t - self.t_seen > c.lost_stop_s:
            self.v = 0.0
            return Command(0.0, self.steer, "RUN", {**info, "lost": True})

        # --- kecepatan: batas percepatan lateral dari kelengkungan lintasan di depan ---
        ahead = lane.path[np.hypot(lane.path[:, 0], lane.path[:, 1]) < 1.3]
        kappa = curvature(ahead)
        v = min(c.v_max, math.sqrt(c.a_lat / max(float(kappa.max()) if len(kappa) else 0.0, 1e-3)))
        v = max(c.v_min, v * (0.5 + 0.5 * lane.conf))

        # --- pure pursuit pada garis tengah + offset ---
        ld = min(c.ld_max, c.ld_min + c.ld_gain * self.v)
        if self.avoid_target is not None:  # manuver pindah lajur butuh respons cepat
            ld, v = c.ld_min, min(v, c.v_avoid)
        path = offset_polyline(lane.path, self.offset) if self.offset else lane.path
        d = np.hypot(path[:, 0], path[:, 1])
        k = int(np.argmax(d >= ld)) if (d >= ld).any() else len(path) - 1
        px, py = path[k]
        curv = 2 * py / max(px * px + py * py, 1e-6)
        self.steer = float(np.clip(math.atan(c.wheelbase * curv), -c.max_steer, c.max_steer))
        # perlambat bila setir hampir mentok (tikungan lebih tajam dari kemampuan sasis)
        if abs(self.steer) > 0.8 * c.max_steer:
            v = c.v_min
        self.v = v
        info.update(v=round(v, 2), steer=round(self.steer, 3), off=round(self.offset, 3), ld=round(ld, 2))
        return Command(v, self.steer, "RUN", info)
