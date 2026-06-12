# imu-vision-fusion

Fuse the **pelvis BNO055 IMU** (`/pelvis/imu`, ROS_DOMAIN_ID 62, frame `pelvis_link`) with
**Intel RealSense D435i** RGB-D visual odometry into a single smoothed pose using a
`robot_localization` EKF.

```
realsense2_camera (D435i, depth aligned to color)
   └─► rgbd_odometry ──► /vo/odom ──┐
                                    ├─► ekf_filter_node ──► /odometry/filtered + tf odom→base_link
              /pelvis/imu (BNO055) ─┘
```

The EKF takes **position** from visual odometry and **orientation** from the BNO055. The
D435i's own IMU is intentionally not fused (clean future enhancement: add as `imu1`).

## 1. Dependencies (one-time)

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-realsense2-camera ros-jazzy-realsense2-camera-msgs \
  ros-jazzy-rtabmap-odom ros-jazzy-rtabmap-ros
# robot_localization ships with the desktop install.
rs-enumerate-devices -s        # confirm the D435i is seen by librealsense
```

## 2. Build

```bash
cd <this-workspace>
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 3. Configure mounting (REQUIRED before trusting the output)

Edit `src/imu_vision_fusion_bringup/launch/bringup.launch.py` and replace the **zeros** in
the two `static_transform_publisher` nodes with the **measured** offsets of:
- `base_link → pelvis_link` (BNO055 mounting pose)
- `base_link → camera_link` (RealSense mounting pose)

Translation in metres, rotation in radians (`--roll/--pitch/--yaw`). Until these are
correct, the fused pose will be geometrically wrong even if topics flow.

## 4. Run

The Jetson USB-net link (`192.168.55.1`) must be up so `/pelvis/imu` is reachable.

There are two camera variants — pick the one matching the attached camera:

```bash
source env.sh                  # ROS_DOMAIN_ID=62 + Cyclone DDS + overlay

# Intel RealSense D435i (mounted upside down -> camera transform has 180 deg roll):
ros2 launch imu_vision_fusion_bringup bringup.launch.py

# Orbbec Gemini 336L (mounted correctly -> identity camera transform):
ros2 launch imu_vision_fusion_bringup bringup_orbbec.launch.py
```

The Orbbec variant needs its driver installed:
```bash
sudo apt install -y ros-jazzy-orbbec-camera ros-jazzy-orbbec-camera-msgs
# install udev rules shipped with the package, then replug the camera.
```

Camera-only smoke tests: `camera.launch.py` (RealSense) or `camera_orbbec.launch.py` (Orbbec).
Both bringups share the same `ekf.yaml`, `rgbd_odometry.yaml`, and the pelvis IMU transform;
they differ only in the camera driver, the VO topic remaps, and the camera mounting rotation.

## 5. Verify

```bash
source env.sh
ros2 topic hz /camera/camera/color/image_raw                      # ~30 Hz
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw     # ~30 Hz
ros2 topic hz /pelvis/imu                                         # ~100 Hz
ros2 topic echo /vo/odom --once                                   # finite pose; changes when you move the camera
ros2 topic echo /odometry/filtered --once                        # finite pose + twist
ros2 run tf2_ros tf2_echo odom base_link                         # transform present
ros2 run tf2_tools view_frames                                   # one connected tree, no duplicate odom→base_link
```

Motion sanity: rotate the rig → filtered yaw tracks the BNO055; translate the rig →
filtered X/Y tracks the visual odometry. Visualise in `rviz2` (Fixed Frame = `odom`).

## Tuning notes

- **Depth↔color alignment is mandatory** — `camera.launch.py` forces `align_depth.enable:=true`.
  If `rgbd_odometry` logs "Did not receive data", check the remapped topic names and QoS.
- **QoS mismatch:** if image topics are best-effort, set `qos: 2` in `config/rgbd_odometry.yaml`.
- **VO drift / loss** in low-texture scenes is expected; the EKF coasts on the IMU
  (`publish_null_when_lost` keeps the chain clean).
- **IMU vs VO heading fight:** tune `imu0_pose_rejection_threshold` / `odom0_pose_rejection_threshold`
  in `config/ekf.yaml`.
- **RealSense profile args** (`rgb_camera.color_profile`, `depth_module.depth_profile`) assume a
  recent realsense2_camera; if the launch rejects them, drop them to use driver defaults
  (lower the resolution/FPS on USB2).
- Do **not** auto-edit `~/.bashrc` for the domain — `env.sh` keeps it scoped to this project.
