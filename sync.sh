#!/usr/bin/env bash
set -e

# ==========================
# ROS2 Workspace Sync Script
# Auto-detect workspace root
# ==========================

CACHE_FILE="/tmp/last_lance_sync.conf"

# ---- AUTO-DETECT WORKSPACE ROOT ----
detect_workspace() {
  local script_path
  script_path="$(realpath "${BASH_SOURCE[0]}")"
  local dir
  dir="$(dirname "$script_path")"

  while [[ "$dir" != "/" ]]; do
    if [[ -d "$dir/src" ]]; then
      echo "$dir"
      return 0
    fi
    dir="$(dirname "$dir")"
  done

  return 1
}

AUTO_LOCAL_WS="$(detect_workspace || true)"

# ---- DEFAULTS ----
LOCAL_WS="$AUTO_LOCAL_WS"
REMOTE_USER=""
REMOTE_HOST=""
REMOTE_WS=""

SSH_PORT=22
SSH_OPTS="-p $SSH_PORT"

DELETE_ENABLED=true
EXTRA_EXCLUDES=()

RSYNC_BASE_OPTS=(-avz --progress)
DRY_RUN=""

# ---- ROS EXCLUDES (keep .git!) ----
BASE_EXCLUDES=(
  build/
  install/
  log/
  __pycache__/
)

# ---- LOAD CACHE ----
if [[ -f "$CACHE_FILE" ]]; then
  source "$CACHE_FILE"
fi

# ---- USAGE ----
usage() {
  echo "Usage:"
  echo "  $0 <push|pull|twoway> [options]"
  echo ""
  echo "Options:"
  echo "  -u USER        Remote SSH user"
  echo "  -h HOST        Remote IP or hostname"
  echo "  -r PATH        Remote workspace path"
  echo "  -l PATH        Local workspace path (override auto-detect)"
  echo "  -n             Dry run"
  echo "  --no-delete    Disable rsync --delete"
  echo "  -x DIRS        Extra excludes (comma-separated, relative paths)"
  echo ""
  echo "Workspace auto-detected as:"
  echo "  ${AUTO_LOCAL_WS:-<not found>}"
  exit 1
}

# ---- PARSE ARGS ----
MODE="$1"
shift || true

while [[ $# -gt 0 ]]; do
  case "$1" in
    -u) REMOTE_USER="$2"; shift 2 ;;
    -h) REMOTE_HOST="$2"; shift 2 ;;
    -r) REMOTE_WS="$2"; shift 2 ;;
    -l) LOCAL_WS="$2"; shift 2 ;;
    -n) DRY_RUN="--dry-run"; shift ;;
    --no-delete) DELETE_ENABLED=false; shift ;;
    -x)
      IFS=',' read -ra EXTRA_EXCLUDES <<< "$2"
      shift 2
      ;;
    *) usage ;;
  esac
done

if [[ -z "$MODE" ]]; then
  usage
fi

# ---- VALIDATION ----
if [[ -z "$LOCAL_WS" ]]; then
  echo "Could not determine local workspace root."
  echo "Run the script from inside a ROS 2 workspace src/ tree,"
  echo "or specify -l <path>."
  exit 1
fi

if [[ -z "$REMOTE_USER" || -z "$REMOTE_HOST" || -z "$REMOTE_WS" ]]; then
  echo "Missing remote configuration."
  echo "Provide -u, -h, and -r at least once."
  exit 1
fi

# ---- BUILD RSYNC OPTIONS ----
RSYNC_OPTS=("${RSYNC_BASE_OPTS[@]}")
[[ "$DELETE_ENABLED" == true ]] && RSYNC_OPTS+=(--delete)
[[ -n "$DRY_RUN" ]] && RSYNC_OPTS+=("$DRY_RUN")

# ---- BUILD EXCLUDES ----
RSYNC_EXCLUDES=()
for ex in "${BASE_EXCLUDES[@]}"; do
  RSYNC_EXCLUDES+=(--exclude "$ex")
done

for ex in "${EXTRA_EXCLUDES[@]}"; do
  RSYNC_EXCLUDES+=(--exclude "$ex")
done

# ---- SAVE CACHE ----
cat > "$CACHE_FILE" <<EOF
REMOTE_USER="$REMOTE_USER"
REMOTE_HOST="$REMOTE_HOST"
REMOTE_WS="$REMOTE_WS"
LOCAL_WS="$LOCAL_WS"
DELETE_ENABLED=$DELETE_ENABLED
EXTRA_EXCLUDES="${EXTRA_EXCLUDES[*]}"
EOF

REMOTE="$REMOTE_USER@$REMOTE_HOST"

# ---- SYNC ----
case "$MODE" in
  push)
    echo "Syncing LOCAL → REMOTE"
    ;;

  pull)
    echo "Syncing REMOTE → LOCAL"
    ;;

  twoway)
    echo "Two-way sync"
    echo "WARNING: This may delete files on either side!"
    read -p "Continue? (y/N): " confirm
    [[ "$confirm" != "y" ]] && exit 0
    ;;
  *)
    usage
    ;;
esac

if [[ "$MODE" == "push" || "$MODE" == "twoway" ]]; then
  rsync "${RSYNC_OPTS[@]}" -e "ssh $SSH_OPTS" \
    "${RSYNC_EXCLUDES[@]}" \
    "$LOCAL_WS/" \
    "$REMOTE:$REMOTE_WS/"
fi

if [[ "$MODE" == "pull" || "$MODE" == "twoway" ]]; then
  rsync "${RSYNC_OPTS[@]}" -e "ssh $SSH_OPTS" \
    "${RSYNC_EXCLUDES[@]}" \
    "$REMOTE:$REMOTE_WS/" \
    "$LOCAL_WS/"
fi

echo "Sync complete."
echo "Cached config: $CACHE_FILE"
