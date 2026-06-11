"""Bring up the Intel RealSense D435i with depth aligned to color.

Thin wrapper around realsense2_camera's rs_launch.py. We force align_depth.enable
(required so rgbd_odometry receives color-registered depth) and disable the camera's
own IMU + pointcloud (the pelvis BNO055 owns orientation; pointcloud is unused).

Default topic namespace yields:
    /camera/camera/color/image_raw
    /camera/camera/aligned_depth_to_color/image_raw
    /camera/camera/color/camera_info
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rs_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py"
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            "align_depth.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",      # cam IMU unused
            "enable_accel": "false",     # cam IMU unused
            "pointcloud.enable": "false",
            # Modest profile that runs comfortably over USB3; lower if on USB2.
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
        }.items(),
    )

    return LaunchDescription([realsense])
