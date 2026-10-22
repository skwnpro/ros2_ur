"""Launch seluruh stack UNDERWATER ROBOT Brush Kart dalam satu perintah.

Jalankan dari workspace ros2_ur di Raspberry Pi:
    ros2 launch brushkart_ur_bringup ur.launch.py

Dengan dua kamera USB tambahan:
    ros2 launch brushkart_ur_bringup ur.launch.py \
        enable_usb_cameras:=true \
        usb1_path:=/dev/v4l/by-id/<id-kamera-1> \
        usb2_path:=/dev/v4l/by-id/<id-kamera-2>

Node yang dinyalakan:
  1. sensor_node           — BNO055 + MS5803 -> /sensor_data
  2. network_monitor_node  — kesehatan tether  -> /network_status
  3. relay_operator_node   — GPIO relay (hidrolik + thruster)
  4. camera_node (utama)   — IMX290 CSI di namespace /camera,
                             dibungkus prefix libcamerify
  5. camera_node x2 (USB)  — namespace /camera_usb1 & /camera_usb2,
                             V4L2 native TANPA libcamerify (opsional)

PRASYARAT kamera utama: overlay imx290 harus sudah ter-load
(lihat README repo — jadikan permanen lewat /boot/firmware/config.txt
agar tidak perlu perintah manual setiap boot).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_usb = LaunchConfiguration('enable_usb_cameras')
    usb1_path = LaunchConfiguration('usb1_path')
    usb2_path = LaunchConfiguration('usb2_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_usb_cameras', default_value='false',
            description='Aktifkan dua kamera USB tambahan'),
        DeclareLaunchArgument(
            'usb1_path', default_value='',
            description='Path /dev/v4l/by-id kamera USB 1 '
                        '(kosong = pakai device_id 1)'),
        DeclareLaunchArgument(
            'usb2_path', default_value='',
            description='Path /dev/v4l/by-id kamera USB 2 '
                        '(kosong = pakai device_id 2)'),

        Node(
            package='brushkart_sensor',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
        ),

        Node(
            package='brushkart_network',
            executable='network_monitor_node',
            name='network_monitor_node',
            output='screen',
        ),

        Node(
            package='brushkart_operator',
            executable='relay_operator_node',
            name='relay_operator_node',
            output='screen',
        ),

        # Kamera utama IMX290 (CSI) — satu-satunya proses yang
        # dibungkus libcamerify.
        Node(
            package='brushkart_camera',
            executable='camera_node',
            name='camera_node',
            namespace='camera',
            prefix='libcamerify',
            output='screen',
            parameters=[{'device_id': 0}],
        ),

        # Kamera USB tambahan — V4L2 native, tanpa libcamerify.
        Node(
            package='brushkart_camera',
            executable='camera_node',
            name='camera_node',
            namespace='camera_usb1',
            output='screen',
            condition=IfCondition(enable_usb),
            parameters=[{
                'device_path': usb1_path,
                'device_id': 1,
            }],
        ),

        Node(
            package='brushkart_camera',
            executable='camera_node',
            name='camera_node',
            namespace='camera_usb2',
            output='screen',
            condition=IfCondition(enable_usb),
            parameters=[{
                'device_path': usb2_path,
                'device_id': 2,
            }],
        ),
    ])
