"""Bring up the Orbbec Gemini 336L (Gemini 330 series) with depth aligned to color.

Thin wrapper around orbbec_camera's gemini_330_series launch. We enable
depth_registration (D2C alignment, required so rgbd_odometry receives color-registered
depth) and disable the camera's own IMU + pointcloud (the pelvis BNO055 owns orientation).

Default topic namespace (camera_name:=camera) yields:
    /camera/color/image_raw
    /camera/depth/image_raw          (aligned to color when depth_registration=true)
    /camera/color/camera_info

NOTE: the included launch file name can vary by orbbec_camera version. If it errors, list
the available ones:  ls $(ros2 pkg prefix orbbec_camera)/share/orbbec_camera/launch/
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    orbbec_launch = os.path.join(
        get_package_share_directory("orbbec_camera"),
        "launch",
        "gemini_330_series.launch.py",
    )

    orbbec = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch),
        launch_arguments={
            "depth_registration": "true",        # align depth to color (rgbd_odometry needs this)
            "enable_color": "true",
            "enable_depth": "true",
            # 15 fps: rgbd_odometry needs ~40 ms/frame, so 30 fps input builds a multi-
            # second backlog (seen as delay=2.2s in the odom log). 15 fps leaves headroom.
            "color_fps": "15",
            "depth_fps": "15",
            # Capture depth+color at the same instant so their stamps match (kills the
            # "time difference between rgb and depth frames is high" warning).
            "enable_frame_sync": "true",
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",
            "enable_accel": "false",             # cam IMU unused
            "enable_gyro": "false",              # cam IMU unused
        }.items(),
    )

    return LaunchDescription([orbbec])
