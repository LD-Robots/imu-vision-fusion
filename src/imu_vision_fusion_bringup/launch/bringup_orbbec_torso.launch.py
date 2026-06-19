"""Top-level IMU-vision fusion bringup using the Orbbec Gemini 336L + the TORSO BNO055.

Same pipeline as bringup_orbbec.launch.py, but the orientation source is the torso BNO055
(/torso/imu, frame torso_link) instead of the pelvis BNO055 (/pelvis/imu, frame pelvis_link):
    orbbec_camera (Gemini 336L, depth aligned to color)
        -> rgbd_odometry  -> /vo/odom  (publish_tf=false)
        -> ekf_filter_node (robot_localization)
             odom0 = /vo/odom   (body-frame linear velocity vx,vy,vz)
             imu0  = /torso/imu (orientation + angular velocity, BNO055 on domain 62)
             => /odometry/filtered  +  tf odom -> base_link

Both /torso/imu and /pelvis/imu are BNO055-class sensors that publish a fused orientation
quaternion, so this is a drop-in swap of the orientation source -- no madgwick filter needed.
Use this variant when the torso IMU is the body segment you want base_link to track.

Differences from bringup_orbbec.launch.py:
  - ekf uses ekf_torso.yaml (imu0 = /torso/imu)
  - publishes base_link->torso_link instead of base_link->pelvis_link
  - records /torso/imu instead of /pelvis/imu

The IMU header.frame_id must match the static transform's child frame (torso_link); if the
torso BNO055 driver stamps a different frame_id, rename the child frame below to match it.
Run on ROS_DOMAIN_ID=62 so /torso/imu is visible (see env.sh).
"""
import os
import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rosbag_recorder(topics):
    """OpaqueFunction that starts `ros2 bag record` for `topics` when record:=true.

    Writes to a timestamped dir under bag_dir. Start is delayed a few seconds so the
    camera's image_transport publishers are advertised before the recorder subscribes
    (the /compressed topics are lazy / subscriber-driven)."""
    def _start(context, *args, **kwargs):
        if LaunchConfiguration("record").perform(context).lower() not in ("true", "1", "yes"):
            return []
        bag_dir = os.path.expanduser(LaunchConfiguration("bag_dir").perform(context))
        stamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        bag_path = os.path.join(bag_dir, f"imu_vision_{stamp}")
        return [TimerAction(period=3.0, actions=[
            ExecuteProcess(
                cmd=["ros2", "bag", "record", "-o", bag_path, *topics],
                output="screen",
            ),
        ])]
    return OpaqueFunction(function=_start)


def generate_launch_description():
    pkg_share = get_package_share_directory("imu_vision_fusion_bringup")
    ekf_params = os.path.join(pkg_share, "config", "ekf_torso.yaml")
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
    # TORSO BNO055 IMU mounting. Measured axes vs base_link (X-fwd, Y-left, Z-up):
    #   IMU X -> up (base +Z),  IMU Y -> left (base +Y),  IMU Z -> back (base -X).
    # That mapping is a pure -90 deg pitch, i.e. Ry(-pi/2) -> roll=0, pitch=-pi/2, yaw=0.
    # (This differs from the pelvis mount, whose Z faces FRONT rather than BACK.)
    # Translation is non-critical (only orientation + gyro are fused).
    tf_base_to_torso = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_torso",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "-1.5708", "--yaw", "0",
            "--frame-id", "base_link", "--child-frame-id", "torso_link",
        ],
    )

    # Orbbec mounted CORRECTLY -> identity rotation (no flip).
    # TRANSLATION matters: rgbd_odometry runs with frame_id=base_link and uses this
    # lever arm to separate camera arc motion (from body pitch/yaw) from real base
    # translation. Camera is 0.88 m above the pelvis; add --x if the lens sits
    # noticeably forward of the pelvis center.
    tf_base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_camera",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.88",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "base_link", "--child-frame-id", "camera_link",
        ],
    )

    # --- rosbag recording (record:=true by default; disable with record:=false) ---
    # Compressed transports need ros-jazzy-image-transport-plugins installed.
    record = DeclareLaunchArgument(
        "record", default_value="true",
        description="Record compressed video + fusion debug topics to a rosbag.",
    )
    bag_dir = DeclareLaunchArgument(
        "bag_dir", default_value="~/imu_vision_bags",
        description="Directory under which a timestamped bag (imu_vision_<ts>) is written.",
    )
    record_topics = [
        # --- compressed video feed (Orbbec single /camera namespace) ---
        "/camera/color/image_raw/compressed",        # sensor_msgs/CompressedImage (JPEG)
        "/camera/depth/image_raw/compressedDepth",   # PNG-packed depth; lets you replay rgbd_odometry
        "/camera/color/camera_info",
        "/camera/depth/camera_info",
        # --- fusion debug data ---
        "/vo/odom",              # rgbd_odometry output
        "/odometry/filtered",    # EKF output
        "/torso/imu",            # BNO055 input
        "/diagnostics",          # robot_localization diagnostics (print_diagnostics:=true)
        "/tf", "/tf_static",
    ]

    return LaunchDescription([
        record, bag_dir,
        camera, rgbd_odometry, ekf, tf_base_to_torso, tf_base_to_camera,
        _rosbag_recorder(record_topics),
    ])
