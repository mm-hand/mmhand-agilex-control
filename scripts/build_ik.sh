#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

python_bin="${CONDA_PREFIX:-}/bin/python"
if [[ ! -x "$python_bin" ]]; then
    python_bin="$(command -v python3 || command -v python)"
fi
if ! "$python_bin" -c "import Cython" >/dev/null 2>&1; then
    echo "Cython is not installed in $python_bin" >&2
    echo "Activate the existing conda environment and run: pip install -r requirements.txt" >&2
    exit 1
fi

echo "Building IK with $python_bin"
"$python_bin" setup.py build_ext --inplace
