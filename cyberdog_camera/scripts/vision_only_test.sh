#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/home/cyberdog_sim"

source /opt/ros/galactic/setup.bash
source "${ROOT_DIR}/install/setup.bash"

wait_briefly() {
  sleep 1
}

set_pose_center() {
  gz model -w earth -m robot -x 3.10 -y 14.9636 -z 0.25 -R 0 -P 0 -Y -1.57
}

set_pose_right() {
  gz model -w earth -m robot -x 2.95 -y 14.9636 -z 0.25 -R 0 -P 0 -Y -1.57
}

set_pose_left() {
  gz model -w earth -m robot -x 3.25 -y 14.9636 -z 0.25 -R 0 -P 0 -Y -1.57
}

set_pose_end() {
  gz model -w earth -m robot -x 3.10 -y 13.5 -z 0.25 -R 0 -P 0 -Y -1.57
}

set_mode() {
  local mode_name="$1"
  timeout 6s ros2 service call /vision/set_mode xmdog_msgs/srv/SetVisionMode "{mode: '${mode_name}'}"
}

show_state_once() {
  timeout 6s ros2 topic echo /vision/perception_state --once
}

show_state_fields() {
  timeout 6s ros2 topic echo /vision/perception_state --once | \
    rg "line_found|line_center_offset|line_is_horizontal|ball_found|ball_color|ball_x_offset|ball_size_ratio|height_limit_danger|obstacle_danger|obstacle_distance|target_found|target_type|target_distance" || true
}

call_soccer_service() {
  timeout 6s ros2 service call /get_soccer_pixel_pos xmdog_msgs/srv/GetDistance "{}"
}

call_height_service() {
  timeout 6s ros2 service call /get_height_limit_distance xmdog_msgs/srv/GetDistance "{}"
}

call_path_service() {
  timeout 6s ros2 service call /get_path_choice xmdog_msgs/srv/GetPath "{}"
}

show_topics() {
  ros2 topic list | rg "^/vision/perception_state|^/rgb_sensor/image_raw|^/camera/image_raw|^/camera/depth/image_raw|^/camera/points" || true
}

show_services() {
  ros2 service list | rg "^/vision/set_mode|^/get_soccer_pixel_pos|^/get_height_limit_distance|^/get_path_choice" || true
}

usage() {
  cat <<'EOF'
用法:
  bash src/cyberdog_camera/scripts/vision_only_test.sh <command>

常用命令:
  check              查看视觉测试相关 topic / service
  pose_center        把机器人放到直道中心
  pose_left          把机器人放到直道左侧
  pose_right         把机器人放到直道右侧
  pose_end           把机器人放到直道末端
  default_center     直道中心 + 切 default + 看一次 VisionState
  default_left       直道左侧 + 切 default + 看一次 VisionState
  default_right      直道右侧 + 切 default + 看一次 VisionState
  stage6             直道中心 + 切 stage_6 + 看一次 VisionState + 调足球兼容服务
  stage3             保持当前位置 + 切 stage_3 + 看一次 VisionState + 调限高兼容服务
  path               保持当前位置 + 调路径兼容服务
  idle               切 idle
  smoke              连续执行 default_center / default_left / default_right / stage6
EOF
}

command="${1:-}"

case "${command}" in
  check)
    show_topics
    echo "-----"
    show_services
    ;;
  pose_center)
    set_pose_center
    ;;
  pose_left)
    set_pose_left
    ;;
  pose_right)
    set_pose_right
    ;;
  pose_end)
    set_pose_end
    ;;
  default_center)
    set_pose_center
    wait_briefly
    set_mode "default"
    wait_briefly
    show_state_fields
    ;;
  default_left)
    set_pose_left
    wait_briefly
    set_mode "default"
    wait_briefly
    show_state_fields
    ;;
  default_right)
    set_pose_right
    wait_briefly
    set_mode "default"
    wait_briefly
    show_state_fields
    ;;
  stage6)
    set_pose_center
    wait_briefly
    set_mode "stage_6"
    wait_briefly
    show_state_fields
    echo "-----"
    call_soccer_service
    ;;
  stage3)
    set_mode "stage_3"
    wait_briefly
    show_state_fields
    echo "-----"
    call_height_service
    ;;
  path)
    call_path_service
    ;;
  idle)
    set_mode "idle"
    ;;
  smoke)
    echo "[1/4] default_center"
    set_pose_center
    wait_briefly
    set_mode "default"
    wait_briefly
    show_state_fields
    echo "-----"
    echo "[2/4] default_left"
    set_pose_left
    wait_briefly
    show_state_fields
    echo "-----"
    echo "[3/4] default_right"
    set_pose_right
    wait_briefly
    show_state_fields
    echo "-----"
    echo "[4/4] stage6"
    set_pose_center
    wait_briefly
    set_mode "stage_6"
    wait_briefly
    show_state_fields
    echo "-----"
    call_soccer_service
    ;;
  *)
    usage
    exit 1
    ;;
esac
