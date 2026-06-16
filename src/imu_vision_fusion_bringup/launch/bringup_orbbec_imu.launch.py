"""Top-level IMU-vision fusion bringup using the Orbbec Gemini 336L's BUILT-IN IMU.

Same pipeline as bringup_orbbec.launch.py but the orientation source is the camera's own
IMU instead of the pelvis BNO055 -- so there is no dependency on the Jetson / ROS_DOMAIN_ID
62 / cross-domain /pelvis/imu.

    orbbec_camera (Gemini 336L, depth aligned to color, enable_imu:=true)
        -> /camera/imu                      raw accel+gyro (sensor_msgs/Imu, NO orientation)
        -> imu_filter_madgwick (use_mag=false)
        -> /camera/imu/filtered             orientation + angular velocity
        -> rgbd_odometry  -> /vo/odom       (body-frame linear velocity vx,vy,vz)
        -> ekf_filter_node (robot_localization)
             odom0 = /vo/odom               (body-frame linear velocity)
             imu0  = /camera/imu/filtered   (orientation + angular velocity)
             => /odometry/filtered  +  tf odom -> base_link

WHY THE MADGWICK FILTER: the Orbbec IMU is a RAW 6-axis accel+gyro -- it has no onboard
sensor fusion and publishes no orientation (unlike the BNO055 in IMUPLUS mode, which the
EKF expected). imu_filter_madgwick turns raw accel+gyro into a gravity-referenced roll/pitch
+ gyro-integrated (boot-relative) yaw quaternion -- the same character the BNO055 provided --
so the EKF orientation fusion is unchanged. Needs:  ros-jazzy-imu-filter-madgwick

Differences from bringup_orbbec.launch.py:
  - camera launched with enable_imu:=true
  - adds imu_filter_madgwick (/camera/imu -> /camera/imu/filtered)
  - ekf uses ekf_orbbec_imu.yaml (imu0 = /camera/imu/filtered)
  - NO base_link->pelvis_link transform (the BNO055 is not used)
  - the Orbbec driver publishes camera_link->camera_imu_optical_frame, so the EKF gets the
    IMU->base_link rotation through that TF + base_link->camera_link below.
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
    ekf_params = os.path.join(pkg_share, "config", "ekf_orbbec_imu.yaml")
    vo_params = os.path.join(pkg_share, "config", "rgbd_odometry.yaml")

    # --- Orbbec Gemini 336L with built-in IMU enabled ---
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "camera_orbbec.launch.py")
        ),
        launch_arguments={"enable_imu": "true"}.items(),
    )

    # --- Attitude filter: raw /camera/imu (accel+gyro) -> /camera/imu/filtered (orientation) ---
    # use_mag=false: the Orbbec IMU has no magnetometer, so yaw is gyro-integrated (relative to
    # boot) -- which is exactly what the EKF's imu0_relative=true expects. world_frame=enu to
    # match robot_localization. publish_tf=false so it does not fight the EKF's odom->base_link.
    imu_madgwick = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        output="screen",
        parameters=[{
            "use_mag": False,
            "world_frame": "enu",
            "publish_tf": False,
        }],
        remappings=[
            ("imu/data_raw", "/camera/imu"),
            ("imu/data", "/camera/imu/filtered"),
        ],
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

    # --- Static transform ---
    # Orbbec mounted CORRECTLY -> identity rotation (no flip). TRANSLATION matters:
    # rgbd_odometry runs with frame_id=base_link and uses this lever arm to separate camera
    # arc motion (from body pitch/yaw) from real base translation. Camera is 0.88 m above the
    # pelvis; add --x if the lens sits noticeably forward of the pelvis center.
    # (No base_link->pelvis_link here: the BNO055 is not used in this bringup.)
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
        "/camera/imu",           # raw Orbbec accel+gyro (no orientation)
        "/camera/imu/filtered",  # madgwick orientation + angular velocity (EKF input)
        "/vo/odom",              # rgbd_odometry output
        "/odometry/filtered",    # EKF output
        "/diagnostics",          # robot_localization diagnostics (print_diagnostics:=true)
        "/tf", "/tf_static",
    ]

    return LaunchDescription([
        record, bag_dir,
        camera, imu_madgwick, rgbd_odometry, ekf, tf_base_to_camera,
        _rosbag_recorder(record_topics),
    ])
