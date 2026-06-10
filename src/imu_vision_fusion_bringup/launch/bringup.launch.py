"""Top-level IMU-vision fusion bringup.

Pipeline:
    realsense2_camera (D435i, depth aligned to color)
        -> rgbd_odometry  -> /vo/odom  (nav_msgs/Odometry, publish_tf=false)
        -> ekf_filter_node (robot_localization)
             odom0 = /vo/odom   (position x,y,z)
             imu0  = /pelvis/imu (orientation + angular velocity, BNO055 on domain 52)
             => /odometry/filtered  +  tf odom -> base_link

Static transforms (MEASURE THESE on the real rig and replace the zeros):
    base_link -> pelvis_link   (where the BNO055 is mounted)
    base_link -> camera_link   (where the RealSense is mounted)

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

    # --- RealSense D435i ---
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "camera.launch.py")
        )
    )

    # --- RGB-D visual odometry ---
    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[vo_params],
        remappings=[
            ("rgb/image", "/camera/camera/color/image_raw"),
            ("depth/image", "/camera/camera/aligned_depth_to_color/image_raw"),
            ("rgb/camera_info", "/camera/camera/color/camera_info"),
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
        # /odometry/filtered is the default output topic.
    )

    # --- Static transforms ---
    # args: --x --y --z --roll --pitch --yaw --frame-id <parent> --child-frame-id <child>
    #
    # ROTATIONS matter (they must match the real sensor mounting, or the IMU and VO
    # headings disagree and the EKF fights itself). TRANSLATIONS are not critical for this
    # loosely-coupled EKF (no rigorous lever-arm compensation) -- rough/zero is fine; the
    # constant offset only shifts where base_link sits.

    # BNO055 pelvis IMU. If the chip is mounted aligned with base_link axes, leave the
    # rotation at zero; otherwise set roll/pitch/yaw to its actual mounting orientation.
    tf_base_to_pelvis = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_pelvis",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link", "--child-frame-id", "pelvis_link",
        ],
    )

    # RealSense D435i mounted UPSIDE DOWN -> 180 deg roll about the forward axis.
    # (If the camera also faces backward/sideways, add yaw accordingly.)
    tf_base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_camera",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "3.14159", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link", "--child-frame-id", "camera_link",
        ],
    )

    return LaunchDescription([
        camera,
        rgbd_odometry,
        ekf,
        tf_base_to_pelvis,
        tf_base_to_camera,
    ])
