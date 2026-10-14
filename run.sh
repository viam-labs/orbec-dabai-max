#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

VENV="${VIAM_MODULE_DATA:?VIAM_MODULE_DATA is not set}/venv"
REDIST_FILE="${VIAM_MODULE_DATA}/openni/redist.path"
if [[ -z "${OPENNI2_REDIST:-}" && -f "${REDIST_FILE}" ]]; then
  OPENNI2_REDIST="$(<"${REDIST_FILE}")"
  export OPENNI2_REDIST
fi
if [[ -n "${OPENNI2_REDIST:-}" ]]; then
  export LD_LIBRARY_PATH="${OPENNI2_REDIST}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi

exec "$VENV/bin/python" src/main.py "$@"
