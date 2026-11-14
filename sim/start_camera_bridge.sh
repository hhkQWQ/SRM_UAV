#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
pnp_camera_base=/world/pnp_pillar/model/x500_mono_cam_down_0/link/camera_link/sensor/imager

children=()
cleanup() {
  trap - EXIT INT TERM
  for child in "${children[@]}"; do kill -TERM "$child" 2>/dev/null || true; done
  for child in "${children[@]}"; do wait "$child" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

ros2 run ros_gz_bridge parameter_bridge \
  "${pnp_camera_base}/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  "${pnp_camera_base}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
  --ros-args \
  -r "${pnp_camera_base}/image:=/pnp/gazebo_image_raw" \
  -r "${pnp_camera_base}/camera_info:=/pnp/gazebo_camera_info" &
children+=("$!")

bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
/usr/bin/python3 -B "$bundle_dir/pnp_integration/camera_bridge.py" "$@" &
children+=("$!")

# If either half exits, stop its sibling so a broken camera cannot look healthy.
wait -n "${children[@]}"
exit 1
