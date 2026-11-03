# Riset — KSTI 2026 Mini Autonomous Vehicle

Dokumen ini merangkum analisis aturan, hitungan skor, pengukuran lintasan contoh, riset lomba sejenis,
dan alasan setiap keputusan teknis di repo ini. Disusun 3 Nov 2026; lomba 12 Nov 2026 (TM 8 Nov) di
Hall Robotika ITS. Klaim dari web disertai sumber; yang belum terverifikasi ditandai.

---

## 1. Aturan → implikasi teknis

| Aturan (guidebook) | Implikasi |
|---|---|
| Mobil listrik skala 1:10, minimal 1 gandar kemudi, penggerak diferensial, tanpa omni/mecanum | Sasis RC 1:10 biasa (TT-02, 104001, dll.) langsung memenuhi. |
| Ackermann = **+0.25** pada pengali skor | Wajib. Sasis RC sudah Ackermann; jangan pakai robot diferensial. |
| Dimensi: track 130–350 mm, wheelbase 160–550 mm, tinggi ≤ 450, lebar ≤ 400, panjang ≤ 600 mm | Tiang kamera ~24 cm aman. Ukur ulang setelah semua terpasang. |
| **Sensor IR / line follower dilarang**, sensor lain boleh | Persepsi berbasis kamera (dipilih). ToF/ultrasonik boleh untuk obstacle. |
| **Semua komunikasi ke robot dilarang** selama lomba | Semua jalan di Jetson; autostart via systemd; mulai dari **tombol fisik**. |
| Tombol fisik e-stop mudah dijangkau + tombol start | `firmware/mcu_bridge`: e-stop memutus daya ESC, start memicu `/race/start`. |
| Marka putih **atau** hitam tergantung lantai arena | Deteksi polaritas otomatis (top-hat / black-hat) — diuji di dua varian. |
| Lebar lintasan 50 cm ±10 %, tidak selalu sama | Lebar 0.50 m di config; toleransi peran garis 9 cm. |
| Dua garis tepi utuh + garis tengah putus-putus | Pola utuh vs putus-putus = satu-satunya pembeda tepi vs tengah (jarak keduanya sama 25 cm). |
| Radius putar min 1.5 m ±10 % (radius luar) | **Lintasan contoh jauh lebih tajam (~0.5 m)** — lihat §3. |
| Checkpoint hijau (boleh tanpa marka), start/finish merah (**belum tentu merah**) | Rentang HSV bisa diubah (`MarkConfig`). Jangan bergantung pada checkpoint bermarka. |
| Keluar jalur = >30 % badan di luar (Race), >50 % (Challenge) | Mobil 19 cm boleh menyimpang ±21 cm dari garis tengah sebelum dihitung keluar. |
| Mobil boleh di lajur kiri atau kanan | Garis balap default = **garis tengah lintasan** (margin terbesar, ±15 cm). Konfirmasi saat TM. |
| 3 percobaan × 5 menit, skor tertinggi diambil; >3 pelanggaran = percobaan selesai | Percobaan 1 dibuat aman (v_max rendah), percobaan 2–3 menaikkan kecepatan. |

## 2. Hitungan skor — apa yang benar-benar penting

Rumus guidebook (diimplementasi di `ksti_mav/core/score.py`, `python -m ksti_mav.core.score`):

```
S_AR  = (1 + max((T_stage − T_total)/T_stage, 0)) · cp           T_stage = 300 s
S_ARC = S_AR_challenge / (2 · Tcp)                                (maks 1.0)
Total = S_AR · (1 + STP + S_ARC)                                  STP = 0.25 bila Ackermann
Penalti: keluar jalur +0.8·T/Tcp, tidak lewat checkpoint/tabrak obstacle +0.5·T/Tcp, bagian lepas +0.2·T/Tcp
```

Dengan Tcp = 8 (asumsi; jumlah asli diumumkan panitia):

| Waktu lap | S_AR (8/8 cp) | Total + Ackermann | Total + Ackermann + Challenge setara |
|---|---|---|---|
| 20 s | 15.47 | 19.33 | 34.28 |
| 30 s | 15.20 | 19.00 | 33.44 |
| 60 s | 14.40 | 18.00 | 30.96 |
| 120 s | 12.80 | 16.00 | 26.24 |

**Kesimpulan strategi (urut prioritas):**
1. **Selesaikan lap dengan semua checkpoint.** Checkpoint adalah pengali linear; waktu hanya
   memberi bonus 0–100 %, dan lap 20 s vs 60 s hanya beda ~7 %.
2. **Ikut Challenge.** S_ARC bisa menambah pengali hingga **+1.0** (skor hampir dua kali lipat) —
   jauh lebih besar dari bonus kecepatan apa pun.
3. **Ackermann** (+0.25) — gratis bila memakai sasis RC.
4. **Nol pelanggaran.** Satu kali keluar jalur = +30 s (Tcp 8) ≈ sama dengan lap 30 s lebih lambat,
   *dan* mobil dipindah ke checkpoint terakhir. Lebih baik pelan dan bersih.
5. Baru setelah itu: kecepatan.

## 3. Lintasan contoh panitia — hasil ukur

Sumber: tautan "Lintasan" di guidebook (PDF 1 halaman, disalin ke `docs/lintasan-contoh-ksti2026.pdf`).
Diolah otomatis oleh `tools/make_track.py` (komponen garis tepi, potongan garis putus-putus, jarak transform).

| Besaran | Nilai |
|---|---|
| Ukuran halaman | 60 × 75 cm → bila 1:10, arena 6.0 × 7.5 m |
| Skala dipakai | dinormalkan agar median lebar = 0.50 m → **2.19 mm/px**, arena **5.25 × 6.56 m** |
| Panjang satu lap (garis tengah) | **18.5 m** |
| Lebar lintasan | 0.490–0.507 m (konsisten, sesuai aturan) |
| Lebar marka | ~1.7 cm |
| Garis putus-putus | ±8.8 cm garis, jarak antar-pusat 21.8 cm (celah ±13 cm) |
| Radius belok garis tengah tersempit | **~0.48 m** (diukur sepanjang 30 cm; persentil-5 = 0.72 m) |

**Temuan penting:** belokan S di sudut kanan-bawah dan kiri-atas contoh **jauh lebih tajam** dari
"radius min 1.5 m" di aturan, pada skala mana pun yang masuk akal (bila 1:10 apa adanya, radius
~0.55 m). Kemungkinan gambar contoh tidak berskala, tetapi kita **merancang untuk yang terburuk**:

- Sudut roda yang dibutuhkan di garis tengah: atan(0.26 / 0.48) ≈ **28°**.
- Menghindari obstacle di lajur luar tikungan tertajam memaksa jalur dalam radius ~0.36 m → **≈ 36°**.
  Di simulasi: sudut maks 0.50 rad (29°) → 2 tabrakan; 0.60 rad (34°) → bersih.
- Maka syarat sasis: **sudut roda ≥ 34°** (atau radius putar poros belakang ≤ 0.40 m). Lihat
  `docs/MEKANIKAL-ELEKTRIKAL.md` §1.

Tanyakan saat Technical Meeting (8 Nov): apakah lintasan asli memakai bentuk contoh ini dan berapa skala aslinya.

## 4. Lomba sejenis & repo acuan

Format KSTI hampir identik dengan **Carolo-Cup** (TU Braunschweig): dua lajur, marka tepi utuh, garis
tengah putus-putus 200/200 mm, marka ±20 mm, radius dalam tikungan tertajam 1000 mm
([Regelwerk 2017 §5.3](https://www.tu-braunschweig.de/fileadmin/Redaktionsgruppen/Institute_Fakultaet_5/Carolo-Cup/Regelwerk_2017_20161016_en_0.pdf)).
Lomba 1:10 lain yang relevan: **Bosch Future Mobility Challenge (BFMC)**.

| Repo | Rekam jejak (sumber) | Yang diambil untuk repo ini |
|---|---|---|
| [KITcar-Team/kitcar-gazebo-simulation](https://github.com/KITcar-Team/kitcar-gazebo-simulation) | KITcar juara Carolo-Cup 2018, 2020–2022 ([wiki](https://de.wikipedia.org/wiki/Carolo-Cup)) | Ide: lintasan simulasi dibangkitkan dari spesifikasi → di sini `make_track.py` dari gambar panitia. |
| [teamspatzenhirn/teamspatzenhirn_simulator_interface](https://github.com/teamspatzenhirn/teamspatzenhirn_simulator_interface) | Spatzenhirn juara 2015/2017/2019 ([wiki](https://de.wikipedia.org/wiki/Carolo-Cup)) | Pola antarmuka ROS 2 ↔ simulator (`/clock`, sim time). |
| [ECC-BFMC](https://github.com/ECC-BFMC) (Brain, Embedded_Platform, Simulator) | Kit resmi penyelenggara BFMC | **Pola Jetson/RPi → MCU lewat UART + watchdog** → `serial_bridge` + `mcu_bridge`. |
| [matgrand/bfmc_2024](https://github.com/matgrand/bfmc_2024) | DEI UNIPD juara 2 BFMC 2024 ([hasil resmi](https://boschfuturemobility.com/?p=22762)); repo dibekukan sebelum final | Referensi kontrol (tidak disalin). |
| [David1234500/CaroloCup2022](https://github.com/David1234500/CaroloCup2022) | Peringkat belum terverifikasi | Struktur ROS 2 di Jetson — paling mirip rencana ini. |
| [simonli357/AD](https://github.com/simonli357/AD), [Renbago/autonomus_vehicle](https://github.com/Renbago/autonomus_vehicle) | Finalis BFMC (README sendiri) | Pola state machine obstacle (ikuti lajur → pindah → lewati → kembali). MPC dinilai berlebihan untuk 9 hari. |
| [NVIDIA-AI-IOT/jetracer](https://github.com/NVIDIA-AI-IOT/jetracer) | Platform resmi NVIDIA, 1.2k★ | Alternatif PCA9685 (I2C) bila MCU bermasalah. |

Tidak ditemukan repo juara yang sudah ROS 2 Humble + Gazebo Fortress, dan tidak ada repo publik
lomba mobil otonom kamera Indonesia yang terverifikasi menang. Repo ini karena itu **ditulis dari nol**
dengan pola yang terbukti di lomba-lomba di atas, lalu divalidasi di simulasi.

## 5. Arsitektur & alasan

```
Kamera CSI ─► camera_node ─► /camera/image_raw ─► autopilot_node ─► /cmd_vel ─► serial_bridge ─► MCU ─► servo + ESC
                                                     │  (sama persis di Gazebo: ros_gz_bridge ↔ plugin Ackermann)
                         tombol START / E-STOP ◄──────┴──── MCU
```

**Persepsi klasik, bukan jaringan saraf.** Waktu 9 hari, data latih belum ada, dan lantai bisa
berganti polaritas. Pipeline geometris bisa dijelaskan, disetel di lapangan, dan diuji di simulasi.
End-to-end (JetRacer/Donkeycar) cepat jalan tetapi rapuh saat warna lantai berubah.

Pelajaran dari simulasi (iterasi nyata, dicatat agar tidak diulang):

| Masalah | Gejala | Perbaikan |
|---|---|---|
| Sliding window vertikal | Garis yang mendatar di BEV (tikungan 90°) hilang | Penelusur arah: melangkah 4 cm mengikuti arah garis, orientasi apa pun, lompati celah putus-putus. |
| Polinom y(x) derajat 2 | Tak mampu mewakili belokan 90° → hipotesis garis tengah melenceng | **Polyline** + offset sepanjang normal. Ini perbaikan yang menuntaskan semua kegagalan tikungan. |
| Peran garis dari frame lalu saja | Satu frame salah → terkunci salah (umpan balik positif) | **Konsensus**: tiap garis dicoba sebagai tepi kiri/tengah/kanan; hipotesis yang disetujui terbanyak menang, frame lalu hanya penalti lunak berbobot kepercayaan. |
| Ambiguitas 25 cm | Pasangan (tepi kanan, tengah) tertukar dengan (tengah, tepi kiri) | Pola **putus-putus wajib** untuk peran tengah bila garis cukup panjang dinilai. |
| Finish terdeteksi di depan | Mobil berhenti sebelum garis | Tetap jalan 0.9 m setelah finish terlihat. |
| Menghindar terlambat | Lookahead panjang = geser lateral lambat | Saat menghindar: lookahead minimum + kecepatan 0.7 m/s. |

**Kontrol:** pure pursuit pada garis tengah (+ offset lajur saat menghindar), lookahead
0.45 + 0.35·v (maks 0.95 m), kecepatan dibatasi percepatan lateral 1.8 m/s² dari kelengkungan
yang terlihat. Pilihan ini sesuai studi pembanding ([Snider 2009, CMU-RI-TR-09-08](https://www.ri.cmu.edu/publications/automatic-steering-methods-for-autonomous-automobile-path-tracking)):
pure pursuit paling stabil di kecepatan rendah.

## 6. Hasil simulasi (sim 2D, autopilot asli, wasit sesuai guidebook)

`tools/sim2d.py` merender kamera dari tekstur lintasan resmi, memberi derau, perubahan cahaya,
latensi 1 frame, ESC orde-1 (τ 0.25 s), dan servo dibatasi 4 rad/s.

| Skenario | Waktu | Checkpoint | Pelanggaran |
|---|---|---|---|
| Lantai gelap, marka putih | 17.0 s | 8/8 | 0 |
| Lantai terang, marka hitam | 17.2 s | 8/8 | 0 |
| Challenge (8 obstacle, lajur bergantian), gelap | 23.1 s | 8/8 | 0 |
| Challenge, terang | 23.3 s | 8/8 | 0 |
| v_max 1.6 m/s, gelap / terang | 15.5 / 15.8 s | 8/8 | 0 |

Waktu proses autopilot: ~15 ms/frame di laptop (x86). Perkiraan Orin Nano (CPU ARM A78AE)
2–3× lebih lambat → 30–45 ms → 22–30 fps (belum terverifikasi; ukur dengan `ros2 topic hz /cmd_vel`).

**Batas simulasi 2D:** obstacle digambar datar, tanpa perspektif 3D, tanpa slip ban, tanpa blur gerak.
Gazebo (`sim.launch.py`) menutup sebagian (obstacle kotak 3D, fisika roda). Mobil asli tetap penentu.

## 7. Jetson Orin Nano

- **JetPack 6.2.x (Ubuntu 22.04) + ROS 2 Humble native.** Jangan naik ke JetPack 7.2 (Ubuntu 24.04) selama
  stack Humble ([NVIDIA JetPack 7.2.1](https://developer.nvidia.com/embedded/jetpack/downloads/archive-7.2.1.md),
  [panduan Clearpath JP 6.2.1](https://docs.clearpathrobotics.com/docs/ros2humble/ros/installation/nvidia)).
- Gazebo **tidak** tersedia di apt Jetson ([forum NVIDIA](https://forums.developer.nvidia.com/t/unable-to-install-gazebo-packages-on-jetson-orin-nx-jetpack-6-2-ubuntu-22-04-ros-2-humble/324764)) → simulasi di laptop, Jetson hanya runtime.
- Kamera CSI IMX219: aktifkan lewat `sudo /opt/nvidia/jetson-io/jetson-io.py`. OpenCV harus versi bawaan
  JetPack (ada GStreamer); laporan forum: OpenCV pip gagal membuka kamera padahal GStreamer jalan
  ([forum](https://forums.developer.nvidia.com/t/subject-issues-with-imx477-and-imx219-cameras-on-jetson-orin-nano-super-developer-kit-gstreamer-works-but-opencv-fails-hot-plugging-causes-crashes/350262)).
- Mode daya: `sudo nvpmodel -m 0 && sudo jetson_clocks` ([NVIDIA how-to](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/howto.html)).

## 8. Risiko & mitigasi

| Risiko | Mitigasi |
|---|---|
| Pencahayaan aula (pantulan lampu di lantai mengkilap) | Top-hat adaptif; uji saat test day; turunkan `lane.min_contrast` / ubah `pitch` agar pantulan di luar BEV. |
| Warna start/finish bukan merah | Ubah `MarkConfig.finish_hsv`; atau andalkan `pilot.min_lap_s` + jarak tempuh. |
| Obstacle berwarna sama dengan lantai | Tambah sensor ToF (VL53L1X) ke MCU — sensor non-garis diizinkan. |
| Kalibrasi pitch kamera salah | Garis lurus tidak sejajar di `/autopilot/debug` → setel `camera.pitch` (§6 panduan ME). |
| Jetson hang / USB lepas | Watchdog MCU 300 ms → ESC netral. |
| ESC tersentak di kecepatan rendah | Motor brushed dengan reduksi tinggi atau brushless sensored; `esc_deadband_us`. |
| Mobil keluar jalur saat percobaan | Turunkan `pilot.v_max`; percobaan 1 selalu dengan setelan aman. |

## 9. Rencana 9 hari

| Tanggal | Target |
|---|---|
| 3–4 Nov | Beli/rakit sasis, pasang servo/ESC, MCU, e-stop. Flash JetPack 6.2, jalankan `jetson/setup.sh`. |
| 5 Nov | Kalibrasi servo & ESC (§6 panduan ME), kalibrasi kamera, ukur cam_x/cam_h/pitch. |
| 6 Nov | Cetak/lakban lintasan uji (min. lurus + 1 tikungan R 0.5–1.5 m, marka 2 cm). Lap pertama v_max 0.6. |
| 7 Nov | Uji marka hitam di lantai terang, obstacle (kotak), start/finish merah. Naikkan v_max bertahap. |
| 8 Nov | **Technical Meeting**: tanyakan daftar di §10. Sesuaikan config. |
| 9–11 Nov | Uji ketahanan (10 lap berturut-turut), baterai cadangan, checklist keselamatan. |
| 12 Nov | Lomba: test day → setel `lane.polarity`, warna finish, `pitch`; percobaan 1 aman, 2–3 lebih cepat. |

## 10. Pertanyaan untuk Technical Meeting

1. Bentuk & skala lintasan asli — sama dengan contoh? Radius tersempit?
2. Warna lantai & marka (putih/hitam)? Warna garis start/finish?
3. Apakah checkpoint bermarka? Berapa jumlahnya (Tcp)?
4. Obstacle: bentuk, ukuran, warna, posisi (lajur mana)? Boleh pindah lajur melintasi garis putus-putus?
5. Boleh berjalan di atas garis tengah (mengangkangi lajur)?
6. Satu lap atau lebih? Arah putaran?
7. Posisi start: di belakang garis atau tepat di atas?
8. Batas tinggi/berat? Pemeriksaan teknis apa saja (e-stop, dimensi)?
