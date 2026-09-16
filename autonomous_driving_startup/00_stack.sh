#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib/common.sh"

SESSIONS=(
  carla_sim
  carla_perception
  carla_depth_fusion
  carla_spatial
  carla_lanes
  carla_planning
  carla_foxglove
)

usage() {
  cat <<'EOF'
Usage:
  ./00_stack.sh core
  ./00_stack.sh full
  ./00_stack.sh status
  ./00_stack.sh attach SESSION
  ./00_stack.sh stop
  ./00_stack.sh close-ssh

core:
  01_sim + 05_lanes + 06_planning + 07_foxglove

full:
  all stages, including legacy/object perception, depth/fusion, and spatial
EOF
}

start_script() {
  local name="$1"
  echo
  echo ">>> $name"
  ATTACH=0 "$SCRIPT_DIR/$name"
}

case "${1:-}" in
  core)
    start_script 01_sim.sh
    start_script 05_lanes.sh
    start_script 06_planning.sh
    start_script 07_foxglove.sh
    echo
    echo "Core stack launched. Use ./00_stack.sh status."
    ;;
  full)
    start_script 01_sim.sh
    start_script 02_perception.sh
    start_script 03_depth_fusion.sh
    start_script 04_spatial.sh
    start_script 05_lanes.sh
    start_script 06_planning.sh
    start_script 07_foxglove.sh
    echo
    echo "Full stack launched. Use ./00_stack.sh status."
    ;;
  status)
    printf "%-24s %s\n" "SESSION" "STATE"
    printf "%-24s %s\n" "-------" "-----"
    for session in "${SESSIONS[@]}"; do
      if tmux has-session -t "$session" 2>/dev/null; then
        printf "%-24s %s\n" "$session" "RUNNING"
      else
        printf "%-24s %s\n" "$session" "-"
      fi
    done
    ;;
  attach)
    session="${2:-}"
    [ -n "$session" ] || { echo "ERROR: specify a tmux session."; exit 1; }
    tmux attach -t "$session"
    ;;
  stop)
    for session in "${SESSIONS[@]}"; do
      tmux kill-session -t "$session" 2>/dev/null || true
    done
    echo "Stopped local launcher tmux sessions."
    echo "The shared SSH master is kept alive for reuse."
    ;;
  close-ssh)
    close_ssh_master
    echo "Closed shared SSH master."
    ;;
  *)
    usage
    exit 1
    ;;
esac
