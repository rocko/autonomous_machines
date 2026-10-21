
#!/usr/bin/env bash
set -euo pipefail

# Start the physics server in headless mode.
# -s: server only
# -r: immediately start simulation
# -v 3: informational logging
# exec: forward container signals directly to Gazebo
exec gz sim -s -r -v 3 shapes.sdf
