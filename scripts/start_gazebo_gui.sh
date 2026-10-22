
#!/usr/bin/env bash
set -euo pipefail

export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1

PIDS=()

cleanup() {
    # Stop every background process started by this launcher.
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done

    # Reap child processes before exiting.
    wait 2>/dev/null || true
}

trap cleanup EXIT
trap 'exit 0' INT TERM

# Start a virtual screen for the GUI.
Xvfb :99 -screen 0 1600x900x24 -nolisten tcp &
PIDS+=("$!")

# Wait for the virtual display to become available.
for attempt in $(seq 1 30); do
    if DISPLAY=:99 xdpyinfo >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

if ! DISPLAY=:99 xdpyinfo >/dev/null 2>&1; then
    echo "ERROR: Xvfb display did not become ready."
    exit 1
fi

# Publish the virtual display over VNC.
#
# MIT-SHM is disabled because framebuffer attachment produced
# X_ShmAttach / BadAccess errors in the Docker environment.
x11vnc \
    -display :99 \
    -rfbport 5900 \
    -localhost \
    -nopw \
    -forever \
    -shared \
    -noxdamage \
    -noshm &
PIDS+=("$!")

# Expose the VNC viewer through HTTP/WebSocket.
websockify \
    --web=/usr/share/novnc/ \
    0.0.0.0:6080 \
    localhost:5900 &
PIDS+=("$!")

# Wait for Gazebo's world-discovery service.
# Unlike /world/<name>/control, this service is independent
# of the particular simulation world being loaded.
READY=false

for attempt in $(seq 1 30); do
    if gz service -l | grep -qx '/gazebo/worlds'; then
        READY=true
        break
    fi

    sleep 1
done

if [[ "$READY" != "true" ]]; then
    echo "ERROR: Gazebo server not discoverable."
    exit 1
fi

echo "Gazebo server detected. Starting graphical client."

# Ogre is used because this renderer worked with Xvfb.
#gz sim -g -v 3 --render-engine ogre &
#PIDS+=("$!")

# Keep the container running while monitoring its children.
# Terminate the service if any essential process exits.
#wait -n "${PIDS[@]}"

# Start Gazebo as the primary application.
# Its exit status determines whether the GUI service succeeds.
#
# Supporting processes are stopped by the existing cleanup trap
# when Gazebo terminates.
gz sim -g -v 4 --render-engine ogre
