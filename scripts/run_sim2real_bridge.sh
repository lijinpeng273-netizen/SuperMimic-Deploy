#!/usr/bin/env bash
set -euo pipefail
DEPLOY_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$DEPLOY_ROOT"
export PYTHONPATH="$DEPLOY_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
exec "${H1_PYTHON:-python}" scripts/deploy/bridge/h1_runner_bridge.py "$@"
