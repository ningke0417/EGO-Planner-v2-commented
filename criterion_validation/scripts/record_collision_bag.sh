#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: record_collision_bag.sh <4|6> <tag> [core|full] [output_dir]

Examples:
  record_collision_bag.sh 4 crossing_01 core
  record_collision_bag.sh 4 crossing_sensor_01 full
  record_collision_bag.sh 6 stuck_01 core

Stop recording with Ctrl-C. The full profile also records each UAV's local
point cloud and can grow much faster than the core profile.
EOF
}

if [[ $# -lt 2 || $# -gt 4 ]]; then
  usage >&2
  exit 2
fi

uav_count="$1"
tag="$2"
profile="${3:-core}"

if [[ "$uav_count" != "4" && "$uav_count" != "6" ]]; then
  echo "UAV count must be 4 or 6." >&2
  exit 2
fi
if [[ "$profile" != "core" && "$profile" != "full" ]]; then
  echo "Profile must be core or full." >&2
  exit 2
fi
if [[ ! "$tag" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "Tag may contain only letters, digits, dot, underscore and hyphen." >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
workspace="$repo_root/swarm-playground/formation_ws"
output_dir="${4:-$repo_root/bags}"

source /opt/ros/noetic/setup.bash
if [[ -f "$workspace/devel/setup.bash" ]]; then
  source "$workspace/devel/setup.bash"
fi

if ! rostopic list >/dev/null 2>&1; then
  echo "Cannot reach ROS master. Start the experiment launch first." >&2
  exit 1
fi

mkdir -p "$output_dir"
timestamp="$(date +%Y%m%d_%H%M%S)"
bag_path="$output_dir/${timestamp}_${uav_count}uav_${tag}.bag"
params_path="${bag_path%.bag}.params.yaml"

topics=(
  /clock
  /rosout
  /rosout_agg
  /traj_start_trigger
  /goal
  /map_generator/global_cloud
  /broadcast_traj_from_planner
  /broadcast_traj_to_planner
  /formation_backend/manual_split
  /formation_backend/topology_components
  /formation_backend/topology_command
  /formation_backend/topology_ack
  /formation_backend/split_formation_scale
  /formation_backend/split_progress_sync_scale
  /formation_backend/auto_split_status
  /formation_backend/subgroup_summary
  /starconvex_shadow_4uav/state
  /starconvex_shadow_4uav/scores
  /starconvex_shadow_4uav/merge_scores
  /starconvex_shadow_4uav/summary
  /subgroup_route_intent_4uav/intents
  /subgroup_route_intent_4uav/summary
)

if [[ "$uav_count" == "6" ]]; then
  topics+=(
    /starconvex_shadow_6uav/state
    /starconvex_shadow_6uav/scores
    /starconvex_shadow_6uav/merge_scores
    /starconvex_shadow_6uav/summary
    /starconvex_shadow_6uav/markers
  )
fi

for ((uav = 0; uav < uav_count; ++uav)); do
  topics+=(
    "/drone_${uav}_visual_slam/odom"
    "/drone_${uav}_planning/pos_cmd"
    "/drone_${uav}_planning/trajectory"
    "/drone_${uav}_planning/data_display"
    "/formation_backend/drone_${uav}_group"
    "/formation_backend/drone_${uav}_route_intent"
    "/formation_safety/drone_${uav}/status"
    "/formation_safety/drone_${uav}/first_wall_violation"
  )
  if [[ "$profile" == "full" ]]; then
    topics+=("/drone_${uav}_pcl_render_node/cloud")
  fi
done

echo "Recording $profile profile to: $bag_path"
rosparam dump "$params_path"
echo "ROS parameter snapshot: $params_path"
echo "UAV count: $uav_count. Stop with Ctrl-C after the vehicles get stuck."
exec rosbag record --lz4 --buffsize=1024 -O "$bag_path" "${topics[@]}"
