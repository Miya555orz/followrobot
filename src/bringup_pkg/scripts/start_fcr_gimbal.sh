#!/usr/bin/env bash
set -Eeuo pipefail

# Lightweight real-hardware profile:
# Sony -> TensorRT YOLO -> tracker/face aim -> independent RS2 2D fast loop.
#
# All CAN discovery, validation and recovery remains centralized in
# start_fcr.sh. Keeping this file as a thin profile wrapper prevents the two
# entrypoints from drifting apart after future hardware fixes.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

exec "$SCRIPT_DIR/start_fcr.sh" --gimbal-only "$@"
