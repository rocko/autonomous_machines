
#!/usr/bin/env bash
set -euo pipefail

# Resolve the repository root regardless of the current directory.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

COMPOSE_FILE="infra/compose.yaml"

# Start the headless simulation. The GUI is not required.
docker compose -f "$COMPOSE_FILE" up -d ros

# Run the orchestrator inside the ROS container.
# -T disables pseudo-terminal allocation, making it suitable
# for scripts, automation, and future CI jobs.
docker compose -f "$COMPOSE_FILE" exec -T ros bash -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /workspace/ros_ws/install/setup.bash
    exec bash /workspace/scripts/execute_mission.sh
'
