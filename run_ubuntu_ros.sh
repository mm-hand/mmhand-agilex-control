#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
READ_PICO_ROS=1 exec python3 "$root/pc/run.py" "$@"
