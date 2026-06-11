"""Top-level IMU-vision fusion bringup using the Orbbec Gemini 336L.

Same pipeline as bringup.launch.py but with the Orbbec camera instead of the RealSense:
    orbbec_camera (Gemini 336L, depth aligned to color)
        -> rgbd_odometry  -> /vo/odom  (publish_tf=false)
        -> ekf_filter_node (robot_localization)
             odom0 = /vo/odom    (position x,y,z)
             imu0  = /pelvis/imu (orientation + angular velocity, BNO055 on domain 52)
             => /odometry/filtered  +  tf odom -> base_link

Differences from the RealSense bringup:
  - includes camera_orbbec.launch.py
  - rgbd_odometry remaps to the Orbbec topic names (/camera/... single namespace)
  - base_link->camera_link is IDENTITY (camera mounted correctly; no 180 deg flip)

The pelvis IMU transform is identical to the RealSense bringup.
Run on ROS_DOMAIN_ID=52 so /pelvis/imu is visible (see env.sh).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("imu_vision_fusion_bringup")
    ekf_params = os.path.join(pkg_share, "config", "ekf.yaml")
    vo_params = os.path.join(pkg_share, "config", "rgbd_odometry.yaml")

    # --- Orbbec Gemini 336L ---
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "camera_orbbec.launch.py")
        )
    )

    # --- RGB-D visual odometry (Orbbec topic names) ---
    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[vo_params],
        remappings=[
            ("rgb/image", "/camera/color/image_raw"),
            ("depth/image", "/camera/depth/image_raw"),
            ("rgb/camera_info", "/camera/color/camera_info"),
            ("odom", "/vo/odom"),
        ],
    )

    # --- EKF fusion ---
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params],
    )

    # --- Static transforms ---
    # BNO055 pelvis IMU (same as RealSense bringup): IMU X->left, Y->up, Z->front.
    tf_base_to_pelvis = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_pelvis",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "1.5708", "--pitch", "0", "--yaw", "1.5708",
            "--frame-id", "base_link", "--child-frame-id", "pelvis_link",
        ],
    )

    # Orbbec mounted CORRECTLY -> identity rotation (no flip).
    tf_base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_camera",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link", "--child-frame-id", "camera_link",
        ],
    )

    return LaunchDescription([
        camera, rgbd_odometry, ekf, tf_base_to_pelvis, tf_base_to_camera,
    ])
