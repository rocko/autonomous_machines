
#!/usr/bin/env bash
set -euo pipefail

# Resolve the repository root regardless of the current directory.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

COMPOSE_FILE="infra/compose.yaml"

# Start the headless simulation. The GUI is not required.
docker compose -f "$COMPOSE_FILE" up -d ros


# Accept a configuration filename located in scripts/experiments/.
# Only filenames are accepted, not arbitrary paths.
CONFIG_NAME="${1:-waypoint_baseline.yaml}"

if [[ "$CONFIG_NAME" == */* || "$CONFIG_NAME" != *.yaml ]]; then
    echo "ERROR: Supply a YAML filename from scripts/experiments/." >&2
    exit 1
fi

if [[ ! -f "scripts/experiments/$CONFIG_NAME" ]]; then
    echo "ERROR: Experiment configuration not found." >&2
    exit 1
fi

# Run the existing orchestrator with the selected configuration.
docker compose -f "$COMPOSE_FILE" exec -T ros \
    bash -c '
        set -e
        source /opt/ros/jazzy/setup.bash
        source /workspace/ros_ws/install/setup.bash

        exec bash /workspace/scripts/execute_mission.sh "$1"
    ' _ "/workspace/scripts/experiments/$CONFIG_NAME"
