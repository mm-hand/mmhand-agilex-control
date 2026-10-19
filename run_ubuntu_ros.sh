#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
env_name="${MMHAND_ROS_ENV:-mmhand-ros}"

if [[ -z "${CONDA_PREFIX:-}" || "$(basename "$CONDA_PREFIX")" != "$env_name" ]]; then
  conda_sh="${CONDA_SH:-}"
  if [[ -z "$conda_sh" ]]; then
    for candidate in \
      "$HOME/miniconda3/etc/profile.d/conda.sh" \
      "$HOME/anaconda3/etc/profile.d/conda.sh" \
      "/opt/conda/etc/profile.d/conda.sh"; do
      if [[ -f "$candidate" ]]; then
        conda_sh="$candidate"
        break
      fi
    done
  fi
  if [[ -z "$conda_sh" || ! -f "$conda_sh" ]]; then
    echo "error: conda.sh not found; set CONDA_SH=/path/to/conda.sh" >&2
    exit 1
  fi
  set +u
  # shellcheck source=/dev/null
  source "$conda_sh"
  conda activate "$env_name"
  set -u
fi

export PYTHONNOUSERSITE="${PYTHONNOUSERSITE:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
READ_POSE_ROS=1 exec python "$root/pc/run.py" "$@"
