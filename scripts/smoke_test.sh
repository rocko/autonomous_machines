#!/usr/bin/env bash
set -euo pipefail

# Resolve the repository root regardless of the current working directory.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE=(docker compose -f "$ROOT/infra/compose.yaml")

# Start a standard ROS 2 talker and listen for one message from a separate process.
# The timeout prevents a broken discovery configuration from hanging forever.
"${COMPOSE[@]}" exec -T ros bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 run demo_nodes_cpp talker >/tmp/uav_ros_talker.log 2>&1 &'
if "${COMPOSE[@]}" exec -T ros bash -lc \
  'source /opt/ros/jazzy/setup.bash && timeout 20 ros2 topic echo /chatter std_msgs/msg/String --once' ; then
  echo "PASS: ROS 2 pub/sub"
else
  echo "FAIL: ROS 2 pub/sub" >&2
  exit 1
fi
