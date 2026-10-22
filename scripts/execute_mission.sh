
#!/usr/bin/env bash
set -euo pipefail

# EXP-0007: Automated autonomous waypoint navigation.
#
# Runs entirely inside the existing ROS 2 container.
# Requires:
#   - Gazebo diff_drive world
#   - waypoint_navigation ROS package
#   - ros_gz_bridge
#   - rosbag2
#   - scripts/analyze_mission.py

WORLD="diff_drive"
ROBOT="vehicle_blue"

BASE="/workspace/ros_ws"
MISSION="$BASE/src/waypoint_navigation/config/mission.yaml"
ANALYZER="/workspace/scripts/analyze_mission.py"

# Every experiment receives its own timestamped directory.
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT="$BASE/experiments/EXP-0007/$RUN_ID"
BAG="$OUTPUT/mission"

MAX_DURATION=180

BRIDGE_PID=""
BAG_PID=""
CONTROLLER_PID=""
BAG_STOPPED=false

mkdir -p "$OUTPUT"

log() {
    printf '[experiment] %s\n' "$*"
}

# Stop a subprocess gracefully, then force termination only
# if it fails to exit within the specified timeout.
stop_process() {
    local pid="$1"
    local max_wait="${2:-10}"

    if [[ -z "$pid" ]]; then
        return
    fi

    if ! kill -0 "$pid" 2>/dev/null; then
        wait "$pid" 2>/dev/null || true
        return
    fi

    kill -INT "$pid" 2>/dev/null || true

    for ((i = 0; i < max_wait * 10; i++)); do
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null || true
            return
        fi
        sleep 0.1
    done

    # SIGTERM is a fallback if graceful shutdown hangs.
    kill -TERM "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
}

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM

    # Stop the controller before terminating the recorder.
    # The controller's shutdown handler publishes zero velocity.
    stop_process "$CONTROLLER_PID" 5

    # The recorder needs SIGINT to finalize its bag metadata.
    if [[ "$BAG_STOPPED" == false ]]; then
        stop_process "$BAG_PID" 15
    fi

    stop_process "$BRIDGE_PID" 5

    log "Run directory: $OUTPUT"
    exit "$exit_code"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

log "Run ID: $RUN_ID"

# ------------------------------------------------------------------
# 1. Wait for the Gazebo world.
# ------------------------------------------------------------------

log "Checking Gazebo server..."

READY=false

for ((i = 0; i < 30; i++)); do
    if gz service -l | grep -qx "/world/$WORLD/control"; then
        READY=true
        break
    fi
    sleep 1
done

if [[ "$READY" != true ]]; then
    log "ERROR: Gazebo world is unavailable."
    exit 1
fi

# ------------------------------------------------------------------
# 2. Reset and unpause simulation.
# ------------------------------------------------------------------

log "Resetting simulation..."

CONTROL_RESPONSE="$(
    gz service \
        -s "/world/$WORLD/control" \
        --reqtype gz.msgs.WorldControl \
        --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req 'reset: {all: true} pause: false'
)"

if ! grep -Eq 'data:[[:space:]]*true' <<< "$CONTROL_RESPONSE"; then
    log "ERROR: World reset was not acknowledged."
    printf '%s\n' "$CONTROL_RESPONSE"
    exit 1
fi

# ------------------------------------------------------------------
# 3. Start the ROS/Gazebo bridge.
# ------------------------------------------------------------------

log "Starting ROS/Gazebo bridge..."

ros2 run ros_gz_bridge parameter_bridge \
    "/model/$ROBOT/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" \
    "/model/$ROBOT/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
    > "$OUTPUT/bridge.log" 2>&1 &

BRIDGE_PID=$!

# A received odometry message proves that the bridge is usable.
log "Waiting for odometry..."

if ! timeout 20 ros2 topic echo \
    "/model/$ROBOT/odometry" \
    nav_msgs/msg/Odometry \
    --once > "$OUTPUT/odometry_check.log" 2>&1; then
    log "ERROR: No ROS 2 odometry received."
    exit 1
fi

# ------------------------------------------------------------------
# 4. Start rosbag recording.
# ------------------------------------------------------------------

log "Starting rosbag recording..."

ros2 bag record \
    -o "$BAG" \
    "/model/$ROBOT/odometry" \
    "/model/$ROBOT/cmd_vel" \
    > "$OUTPUT/rosbag.log" 2>&1 &

BAG_PID=$!

# Allow the recorder to discover and subscribe to its topics.
sleep 3

if ! kill -0 "$BAG_PID" 2>/dev/null; then
    log "ERROR: rosbag recorder exited unexpectedly."
    exit 1
fi

# ------------------------------------------------------------------
# 5. Execute the autonomous mission.
# ------------------------------------------------------------------

log "Starting autonomous waypoint controller..."

ros2 run waypoint_navigation waypoint_controller \
    --ros-args \
    --params-file "$MISSION" \
    > "$OUTPUT/controller.log" 2>&1 &

CONTROLLER_PID=$!

COMPLETE=false

# Check completion and timeout without blocking indefinitely.
for ((elapsed = 0; elapsed < MAX_DURATION; elapsed++)); do

    if grep -q 'Mission complete\.' "$OUTPUT/controller.log"; then
        COMPLETE=true
        break
    fi

    if ! kill -0 "$CONTROLLER_PID" 2>/dev/null; then
        log "ERROR: Controller exited before mission completion."
        break
    fi

    sleep 1
done

if [[ "$COMPLETE" != true ]]; then
    log "ERROR: Mission failed or exceeded timeout."
    exit 1
fi

log "Mission complete."

# ------------------------------------------------------------------
# 6. Stop mission processes and finalize recording.
# ------------------------------------------------------------------

stop_process "$CONTROLLER_PID" 5
CONTROLLER_PID=""

stop_process "$BAG_PID" 15
BAG_PID=""
BAG_STOPPED=true

# ------------------------------------------------------------------
# 7. Analyze the recording.
# ------------------------------------------------------------------

log "Analyzing recorded telemetry..."

python3 "$ANALYZER" \
    --bag "$BAG" \
    --mission "$MISSION" \
    --output "$OUTPUT/results"

log "Experiment completed successfully."
log "Results: $OUTPUT/results"
