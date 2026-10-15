#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
# The cloud builder skips the build when the entrypoint is already executable.
# Keep run.sh non-executable in the source tree, and mark it executable in the package.
chmod +x run.sh setup.sh scripts/install_openni.sh
mkdir -p dist
tar -czf dist/archive.tar.gz \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  meta.json \
  requirements.txt \
  README.md \
  run.sh \
  setup.sh \
  src \
  scripts \
  build-setup.sh
