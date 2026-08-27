# aruco_localisation

ROS2 package for ArUco marker detection, pose estimation, TF broadcasting,
and OptiTrack-based extrinsic calibration for localisation.

## Package purpose

Detects ArUco fiducial markers from a camera stream, estimates each
marker's 6-DoF pose with `solvePnP`, publishes and broadcasts that pose,
and (optionally) calibrates the camera's pose relative to an OptiTrack
motion-capture world frame so ArUco-based and OptiTrack-based marker
poses can be directly compared — useful for validating fiducial-marker
accuracy against sub-millimetre ground truth.

## Node responsibilities

| Node | Responsibility |
|---|---|
| `detector` | Subscribes to the camera image topic, detects ArUco markers, runs `solvePnP`, publishes marker pose(s) and an annotated debug image. |
| `tf_publisher` | Subscribes to `/aruco/pose` and broadcasts it as a TF transform from the camera frame to the marker frame. |
| `optitrack_transform` | Subscribes to `/aruco/pose` and the OptiTrack rigid-body pose topic. Collects synchronized sample pairs to solve the camera-to-OptiTrack calibration transform (`C_T_O`), then publishes the OptiTrack-derived rigid-body and marker poses in the camera frame for comparison against ArUco's own estimate. |

## Topics

**Published by `detector`:**
- `/aruco/pose` (`geometry_msgs/PoseStamped`) — marker pose in camera frame
- `/aruco/id` (`std_msgs/Int32`) — ID of the most recently published marker
- `/aruco/total_frames` (`std_msgs/Int32`) — running count of frames received
- `/aruco/detected_frames` (`std_msgs/Int32`) — running count of frames with at least one detection
- `/aruco/annotated_image` (`sensor_msgs/Image`) — debug image with detected markers drawn

**Subscribed by `detector`:**
- Camera image topic (default `/image_raw`, `sensor_msgs/Image`)

**Subscribed by `tf_publisher`:**
- `/aruco/pose`

**Subscribed by `optitrack_transform`:**
- `/aruco/pose`
- OptiTrack rigid-body pose (default `/vrpn_mocap/rigidbody1/pose`, `geometry_msgs/PoseStamped`)

**Published by `optitrack_transform`:**
- `/optitrack/rigid_body_in_camera` (`geometry_msgs/PoseStamped`)
- `/optitrack/marker_in_camera` (`geometry_msgs/PoseStamped`) — OptiTrack-derived marker pose, for comparison with `/aruco/pose`

## Frame definitions

- `C` — camera frame
- `O` — OptiTrack world frame
- `B` — OptiTrack rigid-body frame (attached to the marker mount)
- `M` — marker frame

`X_T_Y` denotes the transform of frame `Y` expressed in frame `X`
(i.e. it maps a point in `Y` into `C`'s parent frame `X`). The core
relationship:

```
C_T_M = C_T_O * O_T_B * B_T_M
```

Rearranged to solve for the unknown camera-to-OptiTrack calibration:

```
C_T_O = C_T_M * inv(B_T_M) * inv(O_T_B)
```

## Calibration procedure

1. Physically measure (or otherwise determine) `B_T_M`, the fixed offset
   from the OptiTrack rigid body to the marker's own frame, and set it
   in `config/params.yaml` under `optitrack_transform.rigid_body_to_marker`.
2. Launch the full pipeline with both the ArUco camera view and the
   OptiTrack rigid body visible/tracked simultaneously.
3. `optitrack_transform` accumulates `calibration_samples` synchronized
   `(C_T_M, O_T_B)` pairs and solves for `C_T_O` once enough samples are
   collected (default 30, configurable).
4. The solved `C_T_O` is frozen for the remainder of the run and saved
   to `calibration_output_path` (default `config/camera_to_optitrack.yaml`)
   for reuse in later sessions.
5. After calibration, the node continuously publishes the OptiTrack-
   derived marker pose in the camera frame for comparison against the
   ArUco-estimated pose on `/aruco/pose`.

## Required equipment

- A calibrated camera (see below) with a ROS2 image driver publishing
  on the configured image topic.
- Printed ArUco marker(s) from the configured dictionary
  (`aruco_dictionary` parameter) at the configured `marker_length_m`.
- An OptiTrack (or other VRPN-compatible) motion-capture system
  publishing rigid-body poses via `vrpn_mocap` or equivalent, with a
  rigid body rigidly mounted at a known, fixed offset from the marker.

## Camera calibration

`detector` expects `camera_matrix` and `distortion_coefficients` as
parameters (see `config/params.yaml`) rather than computing them live.
Generate these once with a standard OpenCV checkerboard calibration
(`cv2.calibrateCamera`) and paste the results into the config file —
pose accuracy depends heavily on having an accurate camera model.

## Running

```bash
colcon build --packages-select aruco_localisation
source install/setup.bash
ros2 launch aruco_localisation aruco_localisation.launch.py
```

To run a minimal camera-only version first (recommended — see
"Known limitations" and the plan's staged rollout), disable OptiTrack:

```bash
ros2 launch aruco_localisation aruco_localisation.launch.py enable_optitrack:=false
```

You'll still need a camera driver node (e.g. `usb_cam` or `v4l2_camera`)
publishing on the configured `image_topic`; this package does not include
one. Add it to the launch file or run it alongside.

## Known limitations

- No live detection-confidence filtering or outlier rejection on pose
  estimates yet (see plan step 14 — robustness improvements).
- Calibration rotation averaging uses a simple sign-aligned quaternion
  mean, which is adequate for small sample counts but not as robust as
  an SVD-based rotation-averaging method for larger, noisier datasets.
- `optitrack_transform` assumes `/aruco/pose` and the OptiTrack topic
  arrive at comparable rates; it pairs whatever the *latest* message on
  each topic is rather than doing timestamp-based synchronization.
- Only a single marker's pose is republished on `/aruco/pose` per
  callback per detected marker (multiple markers in view each get a
  publish, but downstream nodes only track "the latest" unless you key
  off `/aruco/id`).
