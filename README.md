# Kingphoenix 2026

Sistem drone otonom dengan Raspberry Pi, ArduPilot, dual-camera streaming,
YOLO26 pada Hailo, dan Ground Control Station (GCS).

**Status diperbarui: 14 September 2026.**
Dokumen ini membedakan fitur yang ada di `dev`, perubahan pada branch pengembangan,
laporan pengujian lapangan, dan pekerjaan yang belum diimplementasikan.

## Uji waypoint terbaru

Branch `feat/auto-waypoint-test` menambahkan `runtestwp.py`: **GUIDED takeoff →
AUTO waypoint terpilih → LAND di titik terakhir**. Contoh preview:
`python runtestwp.py --strat r --goto 2-2`. Rentang `1-2` berarti WP1 lalu WP2;
`2-2` berarti hanya WP2, dimulai dari posisi takeoff saat ini.

Waypoint disimpan melalui GCS dengan koordinat global; titik lokal lama perlu
capture ulang. Misi di-backup, di-upload dan dibaca balik sebelum ARM.
DONE memerlukan waypoint reached, landed, dan disarmed. **33 unit test lulus;
belum validasi SITL atau penerbangan fisik.** Tinggi takeoff masih relative altitude
FC, belum menggunakan LiDAR sebagai pengendali. Lihat [panduan lengkap](WAYPOINT_TEST.md)
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
| Kontrol takeoff LiDAR | **Belum diimplementasikan.** Verifikasi takeoff pada PR #1 masih memakai relative altitude FC. |

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

## ARM dan immediate takeoff — branch PR #1

Urutan pada `fix/verified-immediate-takeoff`:

1. Pastikan heartbeat dan relative altitude FC masih baru.
2. Pindah ke GUIDED dan verifikasi mode.
3. Kirim ARM; tunggu ACK diterima dan heartbeat setelah pengiriman menunjukkan armed.
4. Kirim TAKEOFF segera setelah ARM terverifikasi, tanpa tambahan delay tetap.
5. Verifikasi ACK takeoff dan tiga sampel relative altitude baru di sekitar target.

FC tetap mengatur spool motor dan respons naik. Immediate berarti tidak ada
jeda tambahan di RPi, bukan throttle penuh atau jaminan langsung lepas tanah.

- ARM ditolak: TAKEOFF tidak dikirim.
- ACK `IN_PROGRESS`: tunggu hasil akhir dengan timeout terbatas.
- Setelah pengiriman berhasil, perintah yang memerlukan ACK tidak diulang otomatis.
- ACK/state timeout: `UNCONFIRMED`; perintah di FC mungkin masih aktif.
- Link hilang/stale, disarm atau perubahan mode: verifikasi dihentikan; ini bukan pembatalan perintah di FC.
- Tidak ada DISARM otomatis saat hasil takeoff meragukan.
- Sukses mencapai target belum berarti hover stabil.

**Verifikasi ketinggian di PR ini masih GLOBAL_POSITION_INT.relative_alt,
bukan LiDAR.** Data LiDAR tidak dipakai untuk menggantikan acuan home secara diam-diam.

20 uji memakai jam deterministik dan respons FC simulasi, mencakup ARM lambat,
ACK hilang/sumber salah, penolakan, IN_PROGRESS, telemetry stale/beku, tidak naik,
overshoot dan interupsi status terbang. Belum ada pengujian SITL/FC fisik untuk patch ini.

[Lihat prosedur dan batasan lengkap](https://github.com/jrcoreid-repo/kingphoenix-2026/blob/fix/verified-immediate-takeoff/VERIFIED_TAKEOFF.md).

## Tahap berikutnya: takeoff dan hold berbasis LiDAR

Kebutuhan operator: `takeoff(1)` ditujukan untuk mencapai sekitar 1 m dari tanah,
dengan LiDAR sebagai umpan balik pencapaian dan penahanan ketinggian.

**Status: rancangan kebutuhan, belum ada implementasi atau hasil uji.**

Rencana alur:
1. ARM terverifikasi dan mulai lepas landas.
2. Gunakan LiDAR valid untuk mengatur kenaikan, dengan perlambatan mendekati target.
3. Kirim kontrol aktif untuk menahan ketinggian.
4. Tandai DONE setelah beberapa sampel valid berada dalam toleransi dan kenaikan mereda.

Menandai DONE di RPi tidak menghentikan target naik yang masih aktif di FC.
Menambah target relative altitude sebesar 5% saja tidak menyelesaikan masalah
perbedaan acuan sensor maupun memastikan drone berhenti pada jarak LiDAR yang diinginkan.

Pekerjaan yang masih diperlukan:
- Pilih dan validasi jalur kontrol ketinggian sesuai firmware ArduPilot dan mode.
- Tetapkan acuan jarak sensor/airframe, pengaruh kemiringan, dan permukaan di bawah drone.
- Validasi umur sampel, rentang, orientasi dan kualitas pembacaan LiDAR.
- Ukur laju telemetry aktual: konfigurasi meminta `DISTANCE_SENSOR` **20 Hz**;
  ini bukan bukti laju aktual 10 atau 20 Hz.
- Definisikan perlambatan, toleransi DONE, hold, batas waktu dan respons kehilangan LiDAR.
- Uji simulasi/SITL, lalu pengujian fisik bertahap sebelum digunakan untuk misi.

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

