#!/usr/bin/env bash
set -euo pipefail
AI_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 "$AI_ROOT/tools/experiment_cli.py" "$@"
