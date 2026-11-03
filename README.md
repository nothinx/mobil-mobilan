# ksti-mav — Mobil Otonom 1:10 untuk KSTI 2026 Mini Autonomous Vehicle

Perangkat lunak lengkap untuk lomba **Mini Autonomous Vehicle — Robotics Competition at KSTI 2026**
(ITS Surabaya, 12 Nov 2026). Mobil RC 1:10 dengan kemudi Ackermann + **Jetson Orin Nano** + kamera:
menyusuri lintasan bermarka (putih atau hitam), melewati checkpoint, berhenti di garis finish, dan
menghindari obstacle (misi Challenge).

| Bagian | Isi |
|---|---|
| `ksti_mav/core/` | Algoritma tanpa ROS: kamera→BEV, deteksi lajur, marka warna & obstacle, autopilot, skor guidebook |
| `ksti_mav/nodes/` | Node ROS 2 Humble: `autopilot`, `camera`, `serial_bridge`, `referee` (wasit sim) |
| `launch/` | `sim.launch.py` (Gazebo Fortress), `robot.launch.py` (Jetson) |
| `models/`, `worlds/` | Mobil Ackermann + kamera; lintasan **contoh resmi panitia** (gelap & terang, ± obstacle) |
| `tools/` | `make_track.py` (aset lintasan dari PDF panitia), `sim2d.py` (simulator cepat tanpa ROS) |
| `firmware/mcu_bridge/` | Arduino Nano: servo + ESC + tombol START + e-stop + watchdog |
| `jetson/` | Skrip pemasangan & service autostart |
| `docs/RISET.md` | Analisis aturan & skor, ukur lintasan, lomba sejenis, keputusan teknis, hasil uji |
| `docs/MEKANIKAL-ELEKTRIKAL.md` | Sasis, tata letak, kelistrikan, e-stop, belanja, kalibrasi, checklist |

## Mulai cepat

### 1. Simulasi 2D (Windows/Linux, tanpa ROS) — paling cepat untuk menyetel algoritma

```bash
pip install numpy opencv-python
python tools/sim2d.py                         # lintasan gelap
python tools/sim2d.py --light --challenge     # marka hitam + obstacle
python tools/sim2d.py --show                  # tampilkan kamera | BEV | peta
python tools/sim2d.py --video lap.mp4
python tests/test_core.py                     # 6 tes, ~1.5 menit
```

### 2. Gazebo Fortress (Docker Desktop + WSLg, atau Linux)

```bash
docker compose build
docker compose run --rm sim                                                     # gelap
docker compose run --rm sim ros2 launch ksti_mav sim.launch.py world:=ksti_light_challenge
```

Mobil start otomatis 4 s setelah dunia siap; wasit melapor di `/referee/status`. Pantau BEV:
`ros2 run rqt_image_view rqt_image_view /autopilot/debug`.

### 3. Mobil asli (Jetson Orin Nano, JetPack 6.2 / Ubuntu 22.04)

```bash
mkdir -p ~/ws/src && git clone <repo> ~/ws/src/ksti_mav
bash ~/ws/src/ksti_mav/jetson/setup.sh
# isi config/robot.yaml (docs/MEKANIKAL-ELEKTRIKAL.md §6), lalu:
sudo systemctl enable --now ksti-mav
```

Mobil menunggu tombol **START** fisik. Challenge: `ros2 launch ksti_mav robot.launch.py challenge:=true`.

## Topik ROS

| Topik | Tipe | Arah |
|---|---|---|
| `/camera/image_raw` | sensor_msgs/Image | kamera / Gazebo → autopilot |
| `/cmd_vel` | geometry_msgs/Twist (v, yaw rate) | autopilot → serial_bridge / Gazebo |
| `/race/start`, `/race/estop` | std_msgs/Empty, Bool | MCU → autopilot |
| `/autopilot/state`, `/autopilot/debug` | String (JSON), Image (BEV) | diagnosa |
| `/odom`, `/referee/status` | Odometry, String (JSON) | hanya simulasi |

## Status

- Sim 2D: 6 skenario (gelap/terang × race/challenge, v_max 1.2–1.6) **lulus tanpa pelanggaran**.
- Firmware: kompilasi untuk Arduino Nano OK (4.6 KB).
- Gazebo & node ROS: ditulis untuk Humble + Fortress; jalankan `docker compose run --rm sim` untuk verifikasi pertama.
- Mobil asli: menunggu perakitan & kalibrasi.
