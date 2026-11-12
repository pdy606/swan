#!/usr/bin/env bash
# Keep ROS on the system interpreter; MBRA gets its own environment.
set -euo pipefail
AI_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 -B "$AI_ROOT/tools/setup_environment.py" "$@"
