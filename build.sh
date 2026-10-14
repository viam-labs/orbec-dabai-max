#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
mkdir -p dist
tar -czf dist/archive.tar.gz \
  meta.json \
  requirements.txt \
  README.md \
  run.sh \
  setup.sh \
  src \
  scripts \
  build-setup.sh
