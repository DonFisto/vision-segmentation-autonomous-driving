#!/usr/bin/env bash
# Shared local launcher helpers.
#
# These scripts run on the LOCAL workstation. tmux is local; every pane opens
# an SSH session to limoneros and starts the corresponding remote process.
# No sudo or remote tmux is required.

SERVER_USER="${SERVER_USER:-danielmartinez}"
SERVER_HOST="${SERVER_HOST:-limoneros.inf.um.es}"
SERVER_PORT="${SERVER_PORT:-32122}"

SSH_CONNECT_DELAY="${SSH_CONNECT_DELAY:-1.5}"
ATTACH="${ATTACH:-1}"

# One shared SSH master for all local launcher sessions.
CONTROL_SOCKET="${CONTROL_SOCKET:-/tmp/carla_ros_${SERVER_USER}_${SERVER_PORT}.sock}"

ROS_WS='${HOME}/vision-segmentation-autonomous-driving/ros/ros2_ws'
REPO='${HOME}/vision-segmentation-autonomous-driving'
CARLA_DIR='${HOME}/CARLA_0.9.16'
FOX_WS='${HOME}/fox_ws'

ACTIVATE_SEG='source ${HOME}/miniconda3/etc/profile.d/conda.sh && mamba activate ros2seg'
ACTIVATE_DEPTH='source ${HOME}/miniconda3/etc/profile.d/conda.sh && mamba activate ros2depth'

SSH_MASTER_CMD=(
  ssh
  -MNf
  -S "$CONTROL_SOCKET"
  -o ControlMaster=yes
  -o ControlPersist=4h
  -o ServerAliveInterval=30
  -o ServerAliveCountMax=6
  -p "$SERVER_PORT"
  "${SERVER_USER}@${SERVER_HOST}"
)

SSH_CMD="ssh -tt -S ${CONTROL_SOCKET} -p ${SERVER_PORT} ${SERVER_USER}@${SERVER_HOST}"

require_local_tools() {
  local missing=0
  for tool in ssh tmux; do
    if ! command -v "$tool" >/dev/null 2>&1; then
      echo "ERROR: local dependency '$tool' was not found."
      missing=1
    fi
  done
  [ "$missing" -eq 0 ] || exit 1
}

ensure_ssh_master() {
  require_local_tools

  if ssh -S "$CONTROL_SOCKET" -O check \
      -p "$SERVER_PORT" \
      "${SERVER_USER}@${SERVER_HOST}" \
      >/dev/null 2>&1; then
    echo "Reusing SSH master: ${SERVER_USER}@${SERVER_HOST}:${SERVER_PORT}"
    return
  fi

  rm -f "$CONTROL_SOCKET"
  echo "Starting SSH master: ${SERVER_USER}@${SERVER_HOST}:${SERVER_PORT}"
  "${SSH_MASTER_CMD[@]}"
}

close_ssh_master() {
  ssh -S "$CONTROL_SOCKET" -O exit \
    -p "$SERVER_PORT" \
    "${SERVER_USER}@${SERVER_HOST}" \
    >/dev/null 2>&1 || true
  rm -f "$CONTROL_SOCKET"
}

recreate_session() {
  local session="$1"
  local window="$2"
  local width="${3:-220}"
  local height="${4:-65}"

  tmux kill-session -t "$session" 2>/dev/null || true
  tmux new-session -d -s "$session" -n "$window" -x "$width" -y "$height"
}

connect_pane() {
  local target="$1"
  tmux send-keys -t "$target" "$SSH_CMD" C-m
  sleep "$SSH_CONNECT_DELAY"
}

remote_depth_shell() {
  local target="$1"
  tmux send-keys -t "$target" "$ACTIVATE_DEPTH" C-m
  tmux send-keys -t "$target" "cd $ROS_WS" C-m
  tmux send-keys -t "$target" "source install/setup.bash" C-m
}

remote_seg_shell() {
  local target="$1"
  tmux send-keys -t "$target" "$ACTIVATE_SEG" C-m
  tmux send-keys -t "$target" "cd $ROS_WS" C-m
  tmux send-keys -t "$target" "source install/setup.bash" C-m
}

wait_topic() {
  local target="$1"
  local topic="$2"
  local label="${3:-$topic}"

  tmux send-keys -t "$target" \
    "echo 'Waiting for ${label}...'; until ros2 topic list --no-daemon --spin-time 2 2>/dev/null | grep -Fx '${topic}' >/dev/null; do sleep 2; done; echo '${label}: ready'" \
    C-m
}

wait_carla_rpc() {
  local target="$1"
  tmux send-keys -t "$target" \
    "echo 'Waiting for CARLA RPC :2000...'; until timeout 1 bash -c 'echo >/dev/tcp/127.0.0.1/2000' >/dev/null 2>&1; do sleep 2; done; echo 'CARLA RPC: ready'" \
    C-m
}

finish_session() {
  local session="$1"
  local window="${2:-}"

  if [ -n "$window" ]; then
    tmux select-window -t "${session}:${window}" 2>/dev/null || true
  fi

  echo "Started local tmux session: $session"

  if [ "$ATTACH" = "1" ]; then
    tmux attach -t "$session"
  else
    echo "Attach with: tmux attach -t $session"
  fi
}
