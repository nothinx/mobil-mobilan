# KONTEKS — ksti-mav (untuk agent)

- **Apa:** mobil otonom 1:10 untuk lomba KSTI 2026 Mini Autonomous Vehicle (ITS, 12 Nov 2026, TM 8 Nov).
  Guidebook asli: `C:\Users\HP\Downloads\GUIDEBOOK PESERTA MINI AUTONOMOUS VEHICLE KSTI 2026.pdf`.
- **Hardware user:** Jetson Orin Nano. Sasis/servo/ESC belum dibeli per 3 Nov 2026.
- **Sumber kebenaran algoritma:** `ksti_mav/core/` (tanpa ROS). Node ROS hanya pembungkus tipis.
- **Setiap ubah algoritma:** jalankan `python tests/test_core.py` (6 tes, ~1.5 menit). Lap gagal =
  regresi. Debug pakai `tools/sim2d.py --show` atau rekam frame lewat `Sim.visual()`.
- Aset lintasan (`config/track.json`, `models/ksti_track_*`, `worlds/*.sdf`) dibangkitkan
  `tools/make_track.py` — jangan edit tangan.
- Kamera di `models/mav_car/model.sdf` harus sama dengan `CameraConfig` (dijaga tes).
- Keputusan & pelajaran (polyline > polinom, konsensus peran garis, dll.): `docs/RISET.md` §5.
- Upload firmware / flash Jetson = konfirmasi user dulu.
