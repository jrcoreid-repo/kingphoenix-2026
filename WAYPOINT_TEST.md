# Uji takeoff → AUTO waypoint → LAND

Status 14 September 2026: implementasi eksperimen pada `feat/auto-waypoint-test`,
berbasis perbaikan ARM/takeoff PR #1. 46 unit test dengan state/protokol tiruan
lulus; **belum diuji SITL maupun penerbangan fisik**.

## Siapkan waypoint

Gunakan runtime Pi pada branch ini dan GCS V2.3.5 yang sudah ada.
Jalankan `python main.py`, hubungkan GCS, pilih LEFT/RIGHT, unlock edit,
kemudian SAVE WAYPOINT pada lokasi drone masing-masing. Tombol ini merekam
posisi drone saat itu; bukan memasukkan koordinat dengan klik peta.
Lakukan capture dengan drone disarmed dan GPS 3D fix stabil.

Simpan ulang titik lama: runner memerlukan `lat_deg`, `lon_deg`,
`frame: GLOBAL_RELATIVE_ALT` dalam `config/waypoints.json`.
Koordinat x/y lokal lama tidak cukup karena bergantung pada origin EKF.
Ketinggian nominal mengikuti `nominal_waypoint_alt_m` dalam
`config/flight_defaults.json` (default 1.2 m); validasi runner membatasi 0.5–3 m.

Matikan `main.py` sebelum menjalankan runner. Satu proses saja boleh memiliki
UART FC. Runner memulai runtime sendiri, termasuk streaming/AI sesuai konfigurasi,
dan GCS dapat tetap terhubung. Jangan memakai tombol ARM bench untuk memulai tes.

## Perintah

Dari root repo, dengan environment Python Pi yang biasa digunakan:

```bash
# Preview: membaca telemetry dan menampilkan rute, tanpa arm/upload misi
python runtestwp.py --strat r --goto 2-2

# Penerbangan nyata: lakukan setelah validasi SITL dan kesiapan lapangan
python runtestwp.py --strat r --goto 2-2 --execute --confirm-flight-area-clear

# Takeoff dari posisi saat ini → WP1 → WP2 → landing di WP2
python runtestwp.py --strat r --goto 1-2 --execute --confirm-flight-area-clear
```

`-goto` juga diterima. `r/l` berarti RIGHT/LEFT. `1-1` hanya WP1,
`1-2` berarti rentang WP1 hingga WP2 (inklusif), `2-2` hanya WP2.
Drone tidak perlu pergi ke WP1 atau home sebelum takeoff menuju WP2.
Posisi horizontal memakai koordinat global yang tersimpan; tinggi **waypoint AUTO** tetap relatif
terhadap home FC. Takeoff memakai angka tinggi waypoint pertama sebagai target
LiDAR sensor-ke-tanah, lalu BRAKE terverifikasi sebelum AUTO. Kedua acuan ini
berbeda; perubahan ini tidak menjadikan seluruh cruise mengikuti LiDAR.
Runner tidak mengirim DO_SET_HOME; perilaku penetapan home saat arm tetap milik FC.

Opsi: `--speed 1.0` m/s (0.2–3), `--max-leg 100` m (1–200),
`--ready-timeout 30`, `--goto-timeout 120`, `--land-timeout 60` detik.
Batas leg juga berlaku dari posisi takeoff ke waypoint pertama. Kecepatan AUTO
mengikuti DO_CHANGE_SPEED dan bisa tetap memengaruhi misi FC berikutnya.

## Urutan dan verifikasi

1. Memerlukan heartbeat, posisi global dan GPS 3D fix yang fresh, serta status
   ON_GROUND dan disarmed. Pemeriksaan arming FC tetap aktif.
2. Backup misi FC ke `logs/fc-mission-before-wptest-*.json`, lalu mengganti misi FC
   dengan DO_CHANGE_SPEED, waypoint terpilih, dan NAV_LAND di titik terakhir.
   Upload harus diterima dan semua item dibaca balik sebelum ARM. Backup adalah
   rekaman MAVLink JSON untuk pemulihan manual, bukan file impor Mission Planner.
3. GUIDED → ARM terverifikasi → TAKEOFF tanpa sleep motor tambahan. Takeoff
   memakai target awal FC dengan margin kenaikan 5%. LiDAR mencapai target memicu
   BRAKE; ACK TAKEOFF, heartbeat BRAKE dan tinggi stabil ±10 cm ≥0.5 s wajib terverifikasi.
   Lihat [kontrak takeoff LiDAR](VERIFIED_TAKEOFF.md).
4. Dalam BRAKE, periksa LiDAR tetap valid/dekat target. Memilih indeks misi pertama dan memverifikasi MISSION_CURRENT, lalu meminta AUTO
   dan menunggu heartbeat AUTO. LiDAR hilang sebelum AUTO membatalkan kelanjutan.
   Cruise selanjutnya dijalankan autopilot.
5. DONE hanya setelah seluruh waypoint menerima MISSION_ITEM_REACHED, FC melaporkan
   ON_GROUND yang fresh, dan heartbeat menunjukkan disarmed. NAV_LAND dapat berjalan
   dengan mode tetap AUTO; tampilan tidak harus berubah menjadi mode LAND.

Selama tes, tombol flight GCS lain dikunci, perubahan waypoint dikunci setelah
execute dimulai, dan gateway Mission Planner dipaksa memakai kebijakan RACE.
ABORT_MISSION GCS atau Ctrl+C menghentikan tes; jika masih armed di GUIDED/AUTO
serta link fresh, runner mencoba BRAKE dan memverifikasinya. Tidak ada disarm udara.
Mode pilihan pilot/FC lain (termasuk LAND failsafe) dipertahankan. Kehilangan link,
GPS, kegagalan verifikasi atau timeout menghasilkan NOT DONE. Jika BRAKE tidak
terkonfirmasi, misi FC bisa tetap berjalan: ambil alih melalui RC. Hilangnya paket
REACHED juga tidak dianggap sukses. Failsafe autopilot tetap perlu dikonfigurasi.

## Validasi

```bash
python -m unittest discover -s tests -v
python runtestwp.py --help
```

46 unit test mencakup kontrak takeoff, rentang WP, penolakan koordinat lama/invalid,
backup/upload/readback misi, serta penolakan DONE pada early LAND, abort,
mode override, belum landed, dan waypoint yang belum terkonfirmasi.
Pengujian ini tidak membuktikan kompatibilitas firmware FC atau perilaku terbang.
Validasi SITL urutan GUIDED→BRAKE→AUTO→NAV_LAND dan abort diperlukan sebelum uji fisik.

Dalam BRAKE, ambil alih dengan pindah mode RC terlebih dahulu; input stick diabaikan.
