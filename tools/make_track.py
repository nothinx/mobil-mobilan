"""Bangun semua aset lintasan dari gambar contoh panitia (PDF/PNG).

Hasil (ditulis ke repo, ikut di-commit):
  config/track.json                     garis tengah, lebar, start, checkpoint, obstacle
  models/ksti_track_{dark,light}/       tekstur lantai Gazebo (marka putih / marka hitam)
  worlds/ksti_{dark,light}[_challenge].sdf

Pakai:  python tools/make_track.py docs/lintasan-contoh-ksti2026.pdf
Butuh pdftoppm (poppler) bila input PDF.
"""
import argparse
import json
import math
import pathlib
import subprocess
import sys
import tempfile

import cv2
import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ksti_mav.core.track import Track, build_from_raster  # noqa: E402

PX = (2400, 3000)  # resolusi tekstur; halaman contoh 60x75 cm


def load_gray(path):
    if path.suffix.lower() == ".pdf":
        with tempfile.TemporaryDirectory() as td:
            out = pathlib.Path(td) / "t"
            subprocess.run(["pdftoppm", "-png", "-singlefile", "-scale-to-x", str(PX[0]),
                            "-scale-to-y", str(PX[1]), str(path), str(out)], check=True)
            return cv2.imread(str(out) + ".png", cv2.IMREAD_GRAYSCALE)
    return cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)


def perp_bar(img, tr, i, half_len, thick, color):
    """Gambar palang melintang lintasan di titik garis tengah i (meter -> piksel)."""
    x, y = tr.center[i]
    h = tr.heading(i)
    nx, ny = -math.sin(h), math.cos(h)
    p1 = tr.world_to_px(x + nx * half_len, y + ny * half_len)
    p2 = tr.world_to_px(x - nx * half_len, y - ny * half_len)
    cv2.line(img, tuple(int(round(v)) for v in p1), tuple(int(round(v)) for v in p2), color,
             max(1, int(round(thick / tr.mpp))), cv2.LINE_AA)


def texture(gray, tr, light):
    g = 255 - gray if light else gray
    if light:  # lantai terang abu-abu muda, bukan putih murni
        g = np.clip(g.astype(np.int16) - 30, 0, 255).astype(np.uint8)
    img = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    perp_bar(img, tr, tr.start_idx, 0.24, 0.05, (40, 40, 220))          # start/finish merah
    for i in tr.d["checkpoints_idx"]:
        perp_bar(img, tr, i, 0.24, 0.03, (60, 170, 60))                  # checkpoint hijau
    return img


TRACK_SDF = """<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>{w:.4f} {h:.4f}</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><plane><normal>0 0 1</normal><size>{w:.4f} {h:.4f}</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal>
            <albedo_map>materials/textures/track.png</albedo_map>
            <roughness>1.0</roughness><metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model><name>{name}</name><version>1.0</version><sdf version="1.8">model.sdf</sdf>
<description>{desc}</description></model>
"""

WORLD_SDF = """<?xml version="1.0"?>
<!-- Dibangkitkan oleh tools/make_track.py; jangan edit tangan. -->
<sdf version="1.8">
  <world name="ksti">
    <physics name="2ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene>
      <ambient>0.9 0.9 0.9 1</ambient>
      <background>0.75 0.75 0.75 1</background>
      <grid>false</grid>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <direction>-0.2 0.1 -1</direction>
    </light>
    <model name="hall_floor">
      <static>true</static>
      <pose>0 0 -0.002 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <material><ambient>{floor} 1</ambient><diffuse>{floor} 1</diffuse></material>
        </visual>
      </link>
    </model>
    <include><uri>model://{track}</uri></include>
{obstacles}  </world>
</sdf>
"""

OBSTACLE = """    <model name="obstacle_{i}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0.075 0 0 {yaw:.3f}</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>0.15 0.15 0.15</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>0.15 0.15 0.15</size></box></geometry>
          <material><ambient>0.95 0.45 0.05 1</ambient><diffuse>0.95 0.45 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=pathlib.Path)
    ap.add_argument("--width", type=float, default=0.50, help="median lebar lintasan [m]")
    ap.add_argument("--start-px", type=float, nargs=2, default=(360, 2410),
                    help="lokasi garis start pada gambar 2400x3000 (default: tanda di kiri bawah contoh)")
    ap.add_argument("--checkpoints", type=int, default=8)
    a = ap.parse_args()

    gray = load_gray(a.src)
    if gray.shape[::-1] != PX:
        gray = cv2.resize(gray, PX, interpolation=cv2.INTER_AREA)
    data = build_from_raster(gray, a.width, a.start_px, a.checkpoints)
    tr = Track(data)

    # obstacle challenge: kotak di tiap checkpoint, bergantian di lajur kiri / kanan
    obs = []
    for k, i in enumerate(data["checkpoints_idx"]):
        h = tr.heading(i)
        side = 0.125 if k % 2 else -0.125
        x, y = tr.center[i]
        obs.append([round(x - math.sin(h) * side, 3), round(y + math.cos(h) * side, 3), round(h, 3)])
    data["obstacles"] = obs
    data["obstacle_size_m"] = 0.15

    (ROOT / "config").mkdir(exist_ok=True)
    (ROOT / "config/track.json").write_bytes(json.dumps(data, indent=1).encode())

    w, h = data["size_m"]
    for light in (False, True):
        name = "ksti_track_light" if light else "ksti_track_dark"
        mdir = ROOT / "models" / name
        (mdir / "materials/textures").mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(mdir / "materials/textures/track.png"), texture(gray, tr, light))
        (mdir / "model.sdf").write_bytes(TRACK_SDF.format(name=name, w=w, h=h).encode())
        desc = "Lintasan contoh KSTI 2026, " + ("marka hitam lantai terang" if light else "marka putih lantai gelap")
        (mdir / "model.config").write_bytes(MODEL_CONFIG.format(name=name, desc=desc).encode())
        for challenge in (False, True):
            ob = "".join(OBSTACLE.format(i=i, x=o[0], y=o[1], yaw=o[2]) for i, o in enumerate(obs)) if challenge else ""
            world = WORLD_SDF.format(track=name, obstacles=ob, floor="0.8 0.8 0.8" if light else "0.12 0.12 0.13")
            fn = f"ksti_{'light' if light else 'dark'}{'_challenge' if challenge else ''}.sdf"
            (ROOT / "worlds" / fn).write_bytes(world.encode())

    st = data["stats"]
    print(f"skala {data['m_per_px'] * 1000:.2f} mm/px, arena {w:.2f} x {h:.2f} m, panjang {st['length_m']:.1f} m")
    print(f"lebar {st['width_min_m']:.3f}..{st['width_max_m']:.3f} m, marka {data['line_width_m'] * 100:.1f} cm, "
          f"radius min garis tengah {st['min_radius_m']:.2f} m, {len(data['centerline'])} titik")


if __name__ == "__main__":
    main()
