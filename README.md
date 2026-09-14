# Kingphoenix 2026

Sistem drone otonom dengan Raspberry Pi, ArduPilot, dual-camera streaming,
YOLO26 pada Hailo, dan Ground Control Station (GCS).

**Status diperbarui: 14 September 2026.**
Dokumen ini membedakan fitur yang ada di `dev`, perubahan pada branch pengembangan,
laporan pengujian lapangan, dan pekerjaan yang belum diimplementasikan.

## Uji waypoint terbaru

Branch `feat/auto-waypoint-test` menambahkan `runtestwp.py`: **GUIDED takeoff →
BRAKE terverifikasi → AUTO waypoint terpilih → LAND di titik terakhir**. Contoh preview:
`python runtestwp.py --strat r --goto 2-2`. Rentang `1-2` berarti WP1 lalu WP2;
`2-2` berarti hanya WP2, dimulai dari posisi takeoff saat ini.

Waypoint disimpan melalui GCS dengan koordinat global; titik lokal lama perlu
capture ulang. Misi di-backup, di-upload dan dibaca balik sebelum ARM.
DONE memerlukan waypoint reached, landed, dan disarmed. **46 unit test lulus;
belum validasi SITL atau penerbangan fisik.** Takeoff sekarang memakai LiDAR untuk memicu BRAKE dan memverifikasi tinggi stabil;
cruise AUTO tetap memakai relative altitude FC. Lihat [panduan lengkap](WAYPOINT_TEST.md)
untuk perintah execute, capture waypoint, streaming dan abort.

## Status branch

| Branch | Status |
|---|---|
| `feat/auto-waypoint-test` | Runner AUTO waypoint dan dokumentasi; eksperimen berbasis branch PR #1, belum merge dev |
| `main` | Checkpoint lama V2.1; bukan acuan fitur terbaru |
| `dev` | Stack V2.3: runtime Pi, GCS V2.3.5, guarded bench control dan Mission Planner bridge |
| `fix/verified-immediate-takeoff` | Perbaikan ARM/takeoff; 20 uji simulasi lulus, belum validasi SITL atau drone fisik |

Perubahan ARM/takeoff ada di [PR #1](https://github.com/jrcoreid-repo/kingphoenix-2026/pull/1).
Pada checkpoint dokumentasi ini PR masih **draft dan belum merge ke dev**.
Nomor versi runtime Pi dan GCS tidak selalu sama: `main.py` menunjuk stack
`main_v2_3_2`, sementara launcher GCS memakai V2.3.5.

## Capaian terbaru

| Area | Capaian dan batasannya |
|---|---|
| Tuning PID | Operator melaporkan hasil sudah lumayan, tetapi belum sepenuhnya memuaskan. Langkah lapangan berikutnya adalah uji arming dan takeoff; belum ada hasil uji takeoff baru dalam checkpoint ini. |
| MAVLink | UART langsung ke FC, dedicated RX, state kendaraan, pemeriksaan umur telemetry, ACK dan STATUSTEXT. |
| Startup | Startup self-test/readiness dan diagnostik Hailo tersedia. |
| Streaming | Kamera depan dan bawah, video serta metadata/frame ID, dengan pengiriman frame setelah GCS terhubung. |
| Profil stream | 640×400 pada 20 fps; bitrate depan 2000 kbps, bawah 1700 kbps; keyframe interval 20. Ini konfigurasi, bukan jaminan FPS lapangan. |
| AI | YOLO26 pada Hailo dengan perpindahan rute kamera; estimasi bbox dan intent gerakan tersedia. Pengiriman gerakan vision masih dinonaktifkan. |
| GCS | ARM/DISARM dan perubahan mode saat disarmed dengan pengaman bench. Takeoff melalui tombol GCS belum diaktifkan. |
| Mission Planner bridge | Pengaturan IP dari GCS, filter sumber paket masuk, dan perbaikan kontrol panel yang terpotong melalui modal ringkas. |
| ARM/takeoff baru | Implementasi dan 20 uji simulasi tersedia di branch PR #1; belum digabung ke dev. |
| Kontrol takeoff LiDAR | Branch ini: LiDAR memicu BRAKE, lalu verifikasi tinggi ±10 cm stabil ≥0.5 s; digunakan kedua tool. Belum diuji SITL/fisik. |

## Arsitektur vision

| Tahap | Konfigurasi |
|---|---|
| Sensor | AR0234 full-wide 1920×1200; mode sensor mendukung 60 fps |
| Capture runtime | Target 30 fps untuk mengurangi beban/temperatur |
| Downlink GCS | Native 640×400 pada 20 fps |
| Input AI | Resize 640×400, padding 120 piksel atas/bawah menjadi 640×640 |
| Inference | YOLO26 pada Hailo, kamera aktif mengikuti AI router |
| Output | Bbox dipetakan kembali ke koordinat 1920×1200, lalu estimator menulis ke SharedState |

Runtime normal tidak memuat pose HEF. solvePnP/kalibrasi tersedia untuk riset/debug,
bukan dependensi penerbangan normal. Estimator menghasilkan error posisi bbox,
rasio luas, fase dan intent kecepatan terbatas; **intent belum dikirim menjadi gerakan FC**.

## Menjalankan runtime dan GCS

Jalankan dari root repository memakai environment proyek yang sudah disiapkan.

RPi:
```bash
python3 main.py
```

Windows GCS:
```bash
python gcs/kingphoenix_gcs_app.py
```

| Entrypoint/tool | Perilaku stream |
|---|---|
| `main.py` | Runtime dan stream tetap aktif sampai dihentikan |
| `tools.mavlink_arm_disarm_test` | Memulai runtime/stream, kemudian menghentikannya ketika tes selesai |
| `tools.flight_command_test` | Memulai runtime/stream, kemudian menghentikannya ketika tes selesai |
| `tools.vision_stream_test` | Tes vision/stream tanpa membutuhkan FC; tidak mengirim movement MAVLink |

Frame dikirim setelah handshake/koneksi GCS dan tujuan stream tersedia.
Tool flight yang singkat dapat selesai sebelum video sempat tampil.
Jangan menjalankan beberapa runtime/tool flight yang membuka serial FC yang sama.
Gunakan satu sumber perintah selama pengujian; hindari input MP/GCS bersamaan.

Tes vision:
```bash
python3 -m tools.vision_stream_test
```

## Takeoff dengan pengereman berdasarkan LiDAR

Semua pemanggil `CommandManager.takeoff()` memakai jalur yang sama: tool
`tools.flight_command_test`, `runtestwp.py`, dan TAKEOFF pada step executor.
Riwayat PR #1 memakai relative altitude untuk verifikasi; **branch ini menggantinya
dengan LiDAR**. ARM tetap diverifikasi dan TAKEOFF dikirim tanpa delay motor tambahan.

`takeoff(1)` berarti target sekitar 1 m jarak vertikal sensor ke tanah.
Target awal FC diterjemahkan dari relative altitude saat ini + sisa kenaikan LiDAR ×1.05.
LiDAR mencapai target → minta BRAKE → verifikasi heartbeat BRAKE dan tinggi stabil.
DONE memerlukan ACK TAKEOFF diterima dan minimal lima sampel LiDAR baru selama
≥0.5 s dalam ±10 cm dengan perubahan tinggi ≤0.15 m/s.

- Takeoff-only selesai dengan pesawat tetap armed dan menahan posisi di BRAKE.
- Runner waypoint: GUIDED takeoff → BRAKE terverifikasi → AUTO → NAV_LAND.
- Hilang LiDAR sebelum AUTO: misi tidak lanjut. Data basi/tidak valid tidak diganti barometer.
- Ambil alih dengan **pindah mode melalui RC**, karena BRAKE mengabaikan input stick.
- Cruise waypoint tetap home-relative (default 1.2 m), bukan terrain following LiDAR.

```bash
python3 -m tools.flight_command_test takeoff --altitude 1 --confirm-flight-area-clear
```

BRAKE memiliki jarak pengereman; ini bukan jaminan berhenti tepat 1.00 m atau
pengendali LiDAR kontinu. 46 unit test lulus, termasuk alur takeoff→BRAKE→AUTO→landing
menggunakan FC simulasi. **Belum diuji SITL atau fisik**.
[Lihat prosedur, batas sensor, timeout dan recovery](VERIFIED_TAKEOFF.md).

## Riwayat checkpoint

| Versi/tanggal | Ringkasan |
|---|---|
| V2.0 | Arsitektur YOLO26 ringan dengan bbox estimator; movement TX belum aktif |
| V2.0.1 | Perbaikan aktivasi network group Hailo dan serialisasi inference saat perpindahan kamera |
| V2.1 | Pi health checkpoint |
| V2.2 | Startup supervisor/self-test dan perluasan telemetry/readiness |
| V2.3 / 3 September 2026 | Guarded GCS bench control, stream ringan, GCS V2.3.5 dan Mission Planner bridge |
| 10 September 2026 | Laporan tuning PID, draft PR #1 untuk verified immediate takeoff, dan kebutuhan kontrol LiDAR dicatat |

## Dokumentasi lain

- [Pi health](PI_HEALTH_INTEGRATION.md)
- [Debug streaming](STREAM_DEBUG.md)
- [Status validasi](VALIDATION_STATUS.md)
- [Status validasi vision](VISION_VALIDATION_STATUS.md)
- [Versioning](VERSIONING.md)
- [Kalibrasi kamera](calibration/CALIBRATION_GUIDE.md)

Dokumen checkpoint lama dapat mencerminkan versi sebelumnya; gunakan tabel status
branch dan tanggal pembaruan di atas untuk memahami cakupan capaian terbaru.


| 14 September 2026 | Runner AUTO waypoint dan takeoff dengan pemicu BRAKE berbasis LiDAR; 46 unit test, belum SITL/fisik |
