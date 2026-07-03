#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u
READ_PICO_ROS=1 exec /usr/bin/python3 "$root/pc/run.py" "$@"
