#!/usr/bin/env bash
set -eo pipefail

bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
px4_dir=/home/hhkqwq/PX4-Autopilot
run_tag="$(date +%Y%m%d_%H%M%S)"
mkdir -p "$bundle_dir/logs"
bridge_log="$bundle_dir/logs/bridge_${run_tag}.log"
algorithm_log="$bundle_dir/logs/algorithm_${run_tag}.log"
mission_option=0
for option in "$@"; do
  if [[ "$option" == --mission ]]; then mission_option=1; fi
done
child_groups=()

cleanup() {
  trap - EXIT INT TERM
  for child in "${child_groups[@]}"; do
    kill -TERM -- "-$child" 2>/dev/null || true
  done
  # Bound cleanup and only signal the new process groups created below.
  for attempt in {1..30}; do
    alive=0
    for child in "${child_groups[@]}"; do
      if kill -0 -- "-$child" 2>/dev/null; then alive=1; fi
    done
    if [ "$alive" -eq 0 ]; then break; fi
    sleep 0.1
  done
  for child in "${child_groups[@]}"; do
    kill -TERM -- "-$child" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

install -m 644 "$bundle_dir/pnp_pillar.sdf" \
  "$px4_dir/Tools/simulation/gz/worlds/pnp_pillar.sdf"

# Keep the physical camera pose, algorithm extrinsics and truth conversion in
# agreement with the original OpenMV configuration on every fresh launch.
/usr/bin/python3 -B "$bundle_dir/pnp_integration/camera_mount.py" \
  --px4-dir "$px4_dir" "$@"

# Gazebo Harmonic lists subscribed topics too. Starting GroundTruth before
# PX4 makes its clock subscription look like an already running world to
# px4-rc.gzsim. Wait for the real SceneBroadcaster service before subscribing.
setsid bash -c '
bundle_dir=$1
bridge_log=$2
algorithm_log=$3
shift 3
world_ready=0
for attempt in {1..40}; do
  service_info=$(timeout 3s gz service -i --service /world/pnp_pillar/scene/info 2>&1) || true
  if [[ "$service_info" == *"Service providers"* ]]; then
    world_ready=1
    break
  fi
  sleep 0.5
done
if [ "$world_ready" -ne 1 ]; then
  printf "Gazebo 世界服务未就绪，未启动相机桥接和算法。请检查 PX4 启动输出。\n" >&2
  exit 1
fi
printf "Gazebo 世界服务已就绪，正在启动相机桥接和算法。\n"
bash "$bundle_dir/start_camera_bridge.sh" "$@" >"$bridge_log" 2>&1 &
source /opt/ros/humble/setup.bash
/usr/bin/python3 -B "$bundle_dir/pnp_integration/pnp_sim_node.py" "$@" >"$algorithm_log" 2>&1 &
wait
' pnp_sim "$bundle_dir" "$bridge_log" "$algorithm_log" "$@" &
child_groups+=("$!")

printf '相机桥接日志：%s\n算法日志：%s\n' "$bridge_log" "$algorithm_log"
printf 'PX4、Gazebo、桥接和算法由本终端一起启动；相机窗口选择 /pnp/debug_image。\n'
if [ "$mission_option" -eq 1 ]; then
  printf 'Mission视觉模式：等待手动起飞并粗导航，出现 READY_FOR_MISSION 后手动切 Mission。\n'
fi

cd "$px4_dir"
source /home/hhkqwq/px4-venv/bin/activate

export PX4_GZ_WORLD=pnp_pillar
export PX4_GZ_MODEL_POSE=0,0,0,0,0,0
make px4_sitl gz_x500_mono_cam_down
