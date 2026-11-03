"""Simulator 2D tertutup tanpa ROS/Gazebo: kamera dirender dari tekstur lintasan resmi,
autopilot ASLI (ksti_mav.core.pilot) mengemudi, wasit menghitung skor sesuai guidebook.

  python tools/sim2d.py                          # lintasan gelap, marka putih
  python tools/sim2d.py --light --challenge      # marka hitam + obstacle
  python tools/sim2d.py --video out.mp4          # rekam kamera | BEV | peta
  python tools/sim2d.py --show                   # jendela langsung (q = keluar)
"""
import argparse
import math
import pathlib
import sys

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ksti_mav.core.geometry import CameraConfig  # noqa: E402
from ksti_mav.core.pilot import AutoPilot, PilotConfig  # noqa: E402
from ksti_mav.core.score import Referee, total  # noqa: E402
from ksti_mav.core.track import Track  # noqa: E402

CAR_LEN, CAR_W = 0.40, 0.19


class Sim:
    def __init__(self, light=False, challenge=False, pilot_cfg=None, cam_cfg=None, seed=0):
        self.tr = Track.load(ROOT / "config/track.json")
        name = "ksti_track_light" if light else "ksti_track_dark"
        tex = cv2.imread(str(ROOT / "models" / name / "materials/textures/track.png"))
        self.challenge = challenge
        self.obstacles = self.tr.d["obstacles"] if challenge else []
        if challenge:  # obstacle digambar datar dari atas (cukup untuk uji logika)
            half = self.tr.d["obstacle_size_m"] / 2 / self.tr.mpp
            for x, y, yaw in self.obstacles:
                u, v = self.tr.world_to_px(x, y)
                box = cv2.boxPoints(((u, v), (2 * half, 2 * half), -math.degrees(yaw)))
                cv2.fillPoly(tex, [box.astype(np.int32)], (10, 110, 240))
        self.tex = cv2.GaussianBlur(tex, (3, 3), 0)
        self.floor = tuple(int(v) for v in np.median(tex.reshape(-1, 3), axis=0))
        self.cc = cam_cfg or CameraConfig()
        self.cam = self.cc.make()
        self.sky = ~self.cam.pixel_rays_hit_ground()
        self.pc = pilot_cfg or PilotConfig()
        self.pilot = AutoPilot(self.cam, cfg=self.pc)
        self.ref = Referee(self.tr, CAR_W, out_frac=0.5 if challenge else 0.3)
        self.rng = np.random.default_rng(seed)
        self.x, self.y, self.yaw = self.tr.start_pose(back=0.05)
        self.v = self.delta = 0.0
        self.t = 0.0
        mpp = self.tr.mpp
        self.A = np.array([[1 / mpp, 0, self.tr.w_px / 2], [0, -1 / mpp, self.tr.h_px / 2], [0, 0, 1]])
        self.Hi = np.linalg.inv(self.cam.H_g2i)
        self.log = []

    def render(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        T = np.array([[c, -s, self.x], [s, c, self.y], [0, 0, 1]])
        M = self.A @ T @ self.Hi
        img = cv2.warpPerspective(self.tex, M, (self.cam.w, self.cam.h),
                                  flags=cv2.WARP_INVERSE_MAP | cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=self.floor)
        img[self.sky] = (150, 150, 150)
        noise = self.rng.normal(0, 4, img.shape)
        gain = 1 + 0.1 * math.sin(self.t * 0.7)  # pencahayaan berubah pelan
        return np.clip(img * gain + noise, 0, 255).astype(np.uint8)

    def collide(self):
        cx = self.x + self.pc.wheelbase / 2 * math.cos(self.yaw)
        cy = self.y + self.pc.wheelbase / 2 * math.sin(self.yaw)
        h = self.tr.d.get("obstacle_size_m", 0.15) / 2
        for ox, oy, _ in self.obstacles:
            dx, dy = ox - cx, oy - cy
            lx = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
            ly = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
            if abs(lx) < CAR_LEN / 2 + h and abs(ly) < CAR_W / 2 + h:
                return True
        return False

    def place_at(self, s):
        n = len(self.tr.center)
        i = int(self.tr.start_idx + s / self.tr.length * n) % n
        self.x, self.y = self.tr.center[i]
        self.yaw = self.tr.heading(i)
        self.v = self.delta = 0.0
        self.pilot.lane.reset()
        self.pilot.state = "WAIT"
        self.pilot.start(self.t)

    def run(self, t_max=300.0, dt=1 / 30, latency=1, video=None, show=False):
        self.pilot.start(0.0)
        self.ref.reset(0.0)
        queue = [(0.0, 0.0)] * latency
        bumped = False
        while self.t < t_max:
            frame = self.render()
            cmd = self.pilot.step(frame, self.t)
            queue.append((cmd.v, cmd.steer))
            v_cmd, d_cmd = queue.pop(0)
            # aktuator: ESC orde-1 (tau 0.25 s), servo dibatasi laju 4 rad/s
            self.v += (v_cmd - self.v) * min(1.0, dt / 0.25)
            self.delta += float(np.clip(d_cmd - self.delta, -4 * dt, 4 * dt))
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            self.yaw += self.v / self.pc.wheelbase * math.tan(self.delta) * dt
            self.t += dt

            cx = self.x + self.pc.wheelbase / 2 * math.cos(self.yaw)
            cy = self.y + self.pc.wheelbase / 2 * math.sin(self.yaw)
            ev = self.ref.update(cx, cy, self.t)
            hit = self.collide()
            if hit and not bumped:
                self.ref.violations.append("obstacle")
                ev.append("obstacle")
            bumped = hit
            for e in ev:
                self.log.append((round(self.t, 2), e))
            if ("keluar" in ev or "obstacle" in ev) and not self.ref.finished:
                self.place_at(self.ref.last_checkpoint_s())  # aturan: pindah ke checkpoint terakhir
                self.ref.s_prev = self.tr.project(self.x, self.y)[0]
                self.ref.out = False
            if len(self.ref.violations) > 3:
                break
            if video is not None or show:
                vis = self.visual(frame, cmd)
                if video is not None:
                    video.write(vis)
                if show:
                    cv2.imshow("sim2d", vis)
                    if cv2.waitKey(1) == ord("q"):
                        break
            if self.ref.finished or cmd.state == "FINISH":
                break
        r = self.ref.result(self.t)
        r["pilot_state"] = self.pilot.state
        r["total_if_ackermann"] = round(total(r["S_AR"]), 3)
        return r

    def visual(self, frame, cmd):
        lane = self.pilot.last
        bev = self.pilot.lane.draw(lane) if lane is not None else np.zeros((480, 300, 3), np.uint8)
        bev = cv2.resize(bev, (int(bev.shape[1] * 480 / bev.shape[0]), 480))
        m = cv2.resize(self.tex, (384, 480), interpolation=cv2.INTER_AREA)
        sx = 384 / self.tr.w_px
        u, v = self.tr.world_to_px(self.x, self.y)
        cv2.circle(m, (int(u * sx), int(v * sx)), 6, (0, 255, 255), -1)
        f = frame.copy()
        cv2.putText(f, f"t={self.t:5.1f} {cmd.state} v={cmd.v:.2f} d={math.degrees(cmd.steer):+.0f} cp={self.ref.cp}",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return np.hstack([f, bev, m])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--light", action="store_true", help="lantai terang, marka hitam")
    ap.add_argument("--challenge", action="store_true", help="tambah obstacle (Autonomous Race Challenge)")
    ap.add_argument("--vmax", type=float, default=None)
    ap.add_argument("--video", type=pathlib.Path)
    ap.add_argument("--show", action="store_true")
    a = ap.parse_args()
    pc = PilotConfig()
    if a.vmax:
        pc.v_max = a.vmax
    sim = Sim(a.light, a.challenge, pc)
    vw = None
    if a.video:
        probe = sim.visual(sim.render(), sim.pilot.step(sim.render(), 0.0))
        vw = cv2.VideoWriter(str(a.video), cv2.VideoWriter_fourcc(*"mp4v"), 30, probe.shape[1::-1])
        sim = Sim(a.light, a.challenge, pc)
    r = sim.run(video=vw, show=a.show)
    if vw:
        vw.release()
    print(r)
    print("kejadian:", sim.log)
    sys.exit(0 if r["finished"] and not r["violations"] else 1)


if __name__ == "__main__":
    main()
