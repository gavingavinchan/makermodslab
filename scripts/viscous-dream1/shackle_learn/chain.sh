#!/bin/bash
set -euo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export PYTHONDONTWRITEBYTECODE=1
exec /home/gavin/dreamtrial/.venv/bin/python "$HERE/chain.py" "$@"
