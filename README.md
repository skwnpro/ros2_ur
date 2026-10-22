# ros2_ur — Brush Kart Underwater Robot

Program **underwater robot** Brush Kart: Raspberry Pi 4B dengan Ubuntu
Server 24.04 + ROS 2 Jazzy. Pasangannya: repo `ros2_gs` (ground
station, mini PC). Dokumentasi project lengkap ada di vault Obsidian
`ocid_second_brain` (folder "ROS 2").

## Package di workspace ini

| Package | Isi |
|---|---|
| `brushkart_sensor` | `sensor_node` + driver BNO055 & MS5803 |
| `brushkart_operator` | `relay_operator_node` — satu-satunya node yang menyentuh GPIO |
| `brushkart_camera` | `camera_node` — IMX290 (CSI) & kamera USB, multi-instance |
| `brushkart_network` | `network_monitor_node` — kesehatan tether |
| `brushkart_msgs` | Definisi message bersama (**harus identik** dengan salinan di `ros2_gs`) |
| `brushkart_ur_bringup` | Launch file seluruh stack robot |

## Instalasi di Raspberry Pi

```bash
sudo apt install ros-jazzy-ros-base python3-psutil
pip install smbus2          # driver sensor I2C
# RPi.GPIO: gunakan paket bawaan Ubuntu (python3-rpi.gpio) bila tersedia
```

### Kamera IMX290 — WAJIB dibaca

Kamera utama (Arducam IMX290, CSI) hanya bekerja lewat **libcamerify**
(emulasi V4L2 di atas libcamera) di Ubuntu Server — lihat catatan
troubleshooting 26–28 September 2026 di vault. Launch file sudah
membungkus instance kamera utama dengan `prefix='libcamerify'`;
kamera USB **tidak** dibungkus.

Overlay IMX290 tidak ter-load otomatis. Jadikan permanen agar kamera
langsung siap setiap boot — tambahkan baris berikut ke
`/boot/firmware/config.txt` lalu reboot:

```
dtoverlay=imx290
```

## Menjalankan

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch brushkart_ur_bringup ur.launch.py
```

Dengan dua kamera USB tambahan:

```bash
ros2 launch brushkart_ur_bringup ur.launch.py \
  enable_usb_cameras:=true \
  usb1_path:=/dev/v4l/by-id/<id-kamera-1> \
  usb2_path:=/dev/v4l/by-id/<id-kamera-2>
```

## Peta relay → GPIO (BCM)

Sumber kebenaran ada di `relay_operator_node.py` (parameter `pin_*`):

| Relay | Fungsi | GPIO |
|---|---|---|
| 1 | Brush | 5 |
| 2 | Drive forward | 6 |
| 3 | Drive retreat | 13 |
| 4 | Steering left | 19 |
| 5 | Steering right | 26 |
| 6 | Light (belum dipakai) | 16 |
| 7–10 | Thruster DC 1–4 | 12, 23, 24, 25 |

Pengaman di sisi robot: **watchdog** di `relay_operator_node`
mematikan semua relay bila `/relay_operator` berhenti datang
> 1 detik (tether putus / ground station mati), plus interlock
forward+retreat dan left+right.

## Catatan sensor tekanan

Sensor terpasang saat ini **MS5803-01BA (0–1 bar absolut)** — jenuh
di sekitar permukaan air, sehingga `depth_m` **tidak valid** di bawah
air sampai sensor diganti varian yang sesuai (mis. MS5837-30BA).
Tekanan permukaan dikalibrasi otomatis oleh `sensor_node` dari
20 sampel pertama setelah start (robot harus sedang di permukaan),
atau diisi lewat parameter `surface_pressure_mbar`.
