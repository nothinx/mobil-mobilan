# Panduan Mekanikal & Elektrikal

Panduan membangun mobil fisik yang cocok dengan perangkat lunak di repo ini. Angka bertanda
**(cek)** adalah perkiraan atau dari marketplace dan harus diverifikasi di barang aslinya.
Setelah mobil jadi, **ukur** dan isi `config/robot.yaml`.

---

## 1. Sasis

### Syarat (dari aturan + hasil simulasi)

| Syarat | Nilai | Alasan |
|---|---|---|
| Kemudi | **Ackermann**, servo 3 kabel standar (PWM) | +25 % skor; servo dikendalikan MCU |
| Sudut roda maksimum | **≥ 34° (0.60 rad)** | Tikungan tersempit lintasan contoh + obstacle (RISET §3). Bila lintasan asli R ≥ 1.5 m, 20° sudah cukup. |
| Wheelbase | 160–550 mm (aturan); ideal **250–280 mm** | Pendek = lincah di tikungan 0.5 m |
| Track width | 130–350 mm (aturan); ideal 160–190 mm | Badan ≤ ~20 cm → margin ±15 cm di lintasan 50 cm |
| Penggerak | 2WD/4WD **dengan diferensial**; tanpa omni/mecanum | Aturan |
| Motor | Brushed 540 dengan reduksi tinggi, atau brushless **sensored** | Kecepatan target hanya 0.5–1.6 m/s; ESC brushless sensorless tersendat di kecepatan rendah |
| Suspensi | Rendah (on-road) | Kamera stabil; buggy/short-course bergoyang |

### Pilihan yang mudah didapat

| Sasis | Kisaran harga | Catatan |
|---|---|---|
| **Tamiya TT-02** (kit on-road 1:10) | Rp2,1–4,3 jt ([Tokopedia](https://www.tokopedia.com/malangkotadingin/tamiya-rc-1-10-tt-02-type-s-chassis-kit-no-58600)) | **Rekomendasi.** Wheelbase ±257 mm (cek), rendah, suku cadang banyak. Servo & ESC dibeli terpisah = bebas pilih. Sudut setir standar mungkin < 34° (cek) → lihat "menambah sudut setir". |
| WLtoys 104001 (1:10 4WD) | ~Rp1,5 jt ([Tokopedia](https://www.tokopedia.com/rcmuliahobby/wltoys-wl-toys-104001-buggy-1-10-rc-car-4wd)) | Murah, tapi buggy (suspensi tinggi) dan brushless sensorless. Cek apakah servo 3 kabel. |
| ~~WLtoys 144001~~ | — | Skala 1:14 dan servo 5 kabel menyatu dengan receiver ([rc10talk](https://rc10talk.com/viewtopic.php?p=518323)). Hindari. |
| Traxxas Slash 2WD | ~Rp7,2 jt | Kokoh tapi mahal dan tinggi. |

**Menambah sudut setir** bila kurang dari 34°: pakai knuckle/steering block dengan sudut lebih besar
(part opsional TT-02 "high steering angle"), perpanjang lengan servo saver, atau pasang stopper lebih
lebar. Ukur ulang lalu isi `pilot.max_steer` dan `serial_bridge.max_steer`. Bila tetap < 30°, kurangi
`pilot.v_min` dan andalkan kecepatan rendah di tikungan.

## 2. Tata letak

```
            tampak samping (ukuran perkiraan, ukur ulang)
                 kamera (lensa) ── menunduk ~31°
                     ▼
                    [■]─┐            ← tinggi lensa cam_h ≈ 24 cm dari lantai
                     │  │ tiang kaku (alu/3D print), JANGAN bergoyang
   [E-STOP]          │
   ┌──────────[Jetson]┴──────┐       ← total tinggi < 45 cm (aturan)
   │ [baterai motor] [MCU]   │
   └─(O)──────────────────(O)┘
     ^ poros belakang       ^ poros depan
     |◄──── cam_x ≈ 22 cm ──►|  (lensa diukur dari poros BELAKANG)
     |◄── wheelbase ≈ 26 cm ►|
```

- **Kamera**: di atas/dekat poros depan, tinggi ~24 cm, menunduk ~31°. Posisi ini membuat jangkauan
  pandang 0.3–1.5 m di depan poros belakang, cukup untuk tikungan 0.5 m. Tiang harus kaku: getaran =
  BEV bergetar = setir bergetar. Pasang lensa wide 120–160° (IMX219-160) lalu undistort (§6).
- **Jetson**: di tengah, di atas standoff karet. Kipas Jetson jangan tertutup.
- **Baterai**: serendah mungkin, di tengah. Strap + velcro, jangan sampai copot (bagian lepas = penalti).
- **E-stop**: tombol jamur merah di bagian atas, mudah dipukul dari samping (aturan).
- **Tombol START**: terpisah, tidak bersebelahan dengan e-stop.
- Semua kabel diikat (cable tie). Bagian yang lepas saat lomba = pelanggaran + percobaan selesai.

## 3. Elektrikal

### Blok daya

```
 LiPo 2S 7.4V ──XT60──[E-STOP pole A (NC) via sakelar ESC]──► ESC ──► motor
 (motor)                                                       └─ BEC 6V ──► servo (merah/hitam)

 Sumber Jetson (pilih satu):
   a) Power bank USB-PD ≥ 65 W + kabel PD-trigger 19/20 V → jack DC 5.5×2.5 Jetson   (paling sederhana)
   b) LiPo 4S 14.8V ─► sekring 5 A ─► buck-boost 19 V ≥ 4 A ─► jack DC Jetson
 Jetson ──USB──► MCU (Arduino Nano) ── sinyal ──► servo (D9), ESC (D10)
```

Rentang tegangan input jack DC devkit Orin Nano: adaptor bawaan 19 V; rentang pastinya **cek di
datasheet carrier** sebelum memakai sumber lain (forum menyebut 9–20 V, belum terverifikasi).
Pisahkan baterai motor dan baterai Jetson: lonjakan arus motor bisa me-reset Jetson.

### E-stop (aturan: tombol fisik yang mudah dijangkau)

Tombol jamur **2 kutub**:
- **Kutub A (NC)** memutus jalur sakelar ESC (kabel on/off ESC). Kebanyakan ESC RC punya sakelar
  arus kecil; memutusnya mematikan ESC → motor berhenti **tanpa bergantung pada perangkat lunak**.
  Pastikan dengan uji: tekan e-stop saat roda berputar → roda harus berhenti. Bila ESC tanpa sakelar,
  pakai relay otomotif 30 A (koil diputus oleh kutub A) di jalur baterai motor.
- **Kutub B (NO)** ke pin **D3** MCU dan GND → MCU melapor `E 1` → autopilot masuk ESTOP.

Setelah e-stop dilepas, autopilot kembali ke WAIT dan menunggu tombol START (sesuai alur lomba:
mobil diletakkan di checkpoint terakhir lalu tombol start ditekan lagi).

### Pinout MCU (Arduino Nano, `firmware/mcu_bridge`)

| Pin | Ke | Catatan |
|---|---|---|
| D9 | Sinyal servo (kuning/putih) | Daya servo dari BEC ESC, **bukan** dari 5V Nano |
| D10 | Sinyal ESC | Kabel merah BEC ESC: pilih satu sumber 5–6 V servo, jangan dobel BEC |
| D2 | Tombol START (NO) → GND | Pull-up internal |
| D3 | E-stop kutub B (NO) → GND | Pull-up internal |
| GND | GND ESC/BEC | **Ground bersama wajib** agar sinyal PWM terbaca |
| USB | Jetson | Daya + data; Nano clone CH340 = `/dev/ttyUSB0` |

Flash: `arduino-cli compile -b arduino:avr:nano firmware/mcu_bridge` lalu
`arduino-cli upload -b arduino:avr:nano -p COMx firmware/mcu_bridge` (Nano clone lama: board
`arduino:avr:nano:cpu=atmega328old`).

### Kabel & proteksi

- Baterai motor → ESC: 14 AWG + XT60. Jetson: 18 AWG + sekring 5 A.
- Alarm tegangan LiPo (buzzer) di kedua baterai; jangan turun di bawah 3.5 V/sel saat lomba.
- Simpan & isi LiPo di LiPo bag.

## 4. Daftar belanja (perkiraan)

| Barang | Jumlah | Perkiraan (cek) |
|---|---|---|
| Sasis TT-02 kit + ban on-road | 1 | Rp2,1–4,3 jt |
| Motor brushed 540 + pinion kecil, atau brushless sensored + ESC sensored | 1 set | Rp300 rb–1,2 jt |
| ESC (dengan BEC, ada sakelar on/off) | 1 | Rp200–600 rb |
| Servo digital metal gear ≥ 10 kg·cm | 1 | Rp150–400 rb |
| Kamera IMX219-160 (CSI, lensa 160°) + kabel 22-pin untuk Orin Nano | 1 | Rp250–450 rb |
| Arduino Nano (atau clone CH340) | 1 (+1 cadangan) | Rp50–120 rb |
| Tombol jamur e-stop 2 kutub (NC+NO) | 1 | Rp50–150 rb |
| Tombol tekan start | 1 | Rp10 rb |
| LiPo 2S 3000–5000 mAh (motor) | 2 | Rp250–500 rb/bh |
| Power bank PD 65 W + kabel PD-trigger 19/20 V, atau LiPo 4S + buck-boost 19 V | 1 set | Rp400 rb–1 jt |
| Bahan tiang kamera, standoff, velcro, cable tie, XT60, kabel | — | Rp150 rb |
| Lakban/vinyl putih & hitam 2 cm + karton/MMT gelap untuk lintasan uji | — | Rp150–300 rb |

## 5. Lintasan uji di rumah/kampus

Minimal: satu lurus 3 m + satu tikungan 90° radius 1.5 m **dan** satu radius 0.5 m (yang terburuk
dari contoh). Lebar 50 cm: dua garis tepi utuh 2 cm + garis tengah putus-putus (±9 cm garis / ±13 cm celah,
seperti contoh). Buat juga versi lantai terang + lakban hitam. Tambahkan pita merah 5 cm melintang
sebagai start/finish dan kotak ±15 cm sebagai obstacle.

## 6. Kalibrasi (urut)

Jalankan di Jetson. Isi hasilnya ke `config/robot.yaml`, lalu `colcon build` (tidak perlu bila
`--symlink-install`) dan restart service.

1. **Servo tengah**: angkat mobil (roda tidak menyentuh lantai). Kirim pulsa manual lewat serial:
   `python3 -m serial.tools.miniterm /dev/ttyUSB0 115200`, ketik `C 1500 1500` berulang (watchdog 300 ms).
   Ubah angka pertama sampai roda lurus → `servo_center_us` (juga konstanta `SERVO_CENTER_US` di firmware).
2. **Batas setir**: cari pulsa saat roda mentok kiri dan kanan tanpa servo mengerang →
   `servo_min_us`/`servo_max_us`. Ukur sudut roda di dua ujung (busur derajat atau foto dari atas) →
   `servo_us_per_rad` = (selisih us dari tengah) / (sudut rad). Negatif bila arahnya terbalik
   (perintah kiri malah belok kanan).
3. **ESC**: roda terangkat. Naikkan angka kedua dari 1500 sedikit demi sedikit sampai roda mulai
   berputar → selisihnya = `esc_deadband_us`. Bila roda mundur, `esc_us_per_mps` negatif.
4. **Kecepatan**: di lantai, kirim pulsa tetap selama 3 detik, ukur jarak → m/s. Ulangi di 2–3 pulsa →
   `esc_us_per_mps` = Δus / Δ(m/s). Catatan: laju berubah sesuai tegangan baterai. (Upgrade nanti:
   encoder roda + loop kecepatan di MCU.)
5. **Kamera (intrinsik)**: `ros2 run ksti_mav camera` lalu
   `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=/camera/image_raw`
   (papan catur cetak A4, kotak 25 mm). Salin `camera_matrix` → `camera.K` + `autopilot camera.fx/fy/cx/cy`,
   `distortion_coefficients` → `camera.D`.
6. **Moncong di kamera**: buka gambar kamera (`python3 tools/gz_snapshot.py snap`), cari baris piksel
   teratas tempat bumper/moncong/kabel mobil terlihat, kurangi ~15 → `lane.hood_row`. Tanpa ini tepi
   moncong terbaca sebagai garis dan mobil bisa keluar jalur (terbukti di Gazebo).
7. **Pose kamera**: ukur `cam_x` (lensa ke poros belakang), `cam_h` (lensa ke lantai), perkiraan `pitch`.
   Lalu letakkan mobil lurus di bagian lurus lintasan, buka `/autopilot/debug`
   (`ros2 run rqt_image_view rqt_image_view`, dari laptop via jaringan sebelum lomba). **Garis harus
   sejajar dan berjarak 25 cm.** Melebar ke atas = pitch terlalu kecil; menyempit = terlalu besar.
   Setel per 0.01 rad.
8. **Uji jalan**: `pilot.v_max 0.6` → 3 lap bersih → naikkan 0.1 → ulangi. Catat nilai aman tertinggi
   untuk lantai gelap dan terang.

## 7. Checklist sebelum tiap percobaan

- [ ] Baterai motor & Jetson penuh (alarm terpasang)
- [ ] `systemctl status ksti-mav` aktif; `journalctl -u ksti-mav -n 20` tanpa error
- [ ] Wi-Fi Jetson dimatikan saat lomba (`nmcli radio wifi off`) — aturan melarang komunikasi
- [ ] E-stop diuji: tekan → roda berhenti; lepas → menunggu START
- [ ] Mobil di garis start, lurus, tidak disentuh setelah START ditekan
- [ ] Config sesuai lantai hari itu (`lane.polarity`, warna finish, `v_max` sesuai percobaan ke-n)
- [ ] Semua baut, ban, tiang kamera kencang; kabel terikat
