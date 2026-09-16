#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Compatibility launcher: lanes + standalone Foxglove."
echo "Starting both local tmux sessions without attaching..."

ATTACH=0 "$SCRIPT_DIR/05_lanes.sh"
ATTACH=0 "$SCRIPT_DIR/07_foxglove.sh"

echo
echo "Started:"
echo "  carla_lanes"
echo "  carla_foxglove"
echo
echo "Attach with:"
echo "  tmux attach -t carla_lanes"
echo "  tmux attach -t carla_foxglove"
