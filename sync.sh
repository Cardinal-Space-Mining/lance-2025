#!/usr/bin/env bash
set -e

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

# ---- DEFAULTS ----
LOCAL_WS="$(detect_workspace || true)"
REMOTE_USER=""
REMOTE_HOST=""
REMOTE_WS=""
DELETE_ENABLED=false

SSH_PORT=22
SSH_OPTS="-p $SSH_PORT"

RSYNC_BASE_OPTS=(-avz --progress)

DRY_RUN=""

WS_EXCLUDES=("*csm-sim*")
WS_INCLUDES=("src/***" ".git/***")

BAG_EXCLUDES=()
BAG_INCLUDES=("bag_recordings/***")

EXCLUDES=()
INCLUDES=()

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
  echo "  -l PATH        Local workspace path"
  echo "  -n             Dry run"
  echo "  -d             Apply rsync --delete"
  echo "  -x PATTERNS    Exclude patterns (comma-separated, precedence over includes)"
  echo "  -i PATTERNS    Include patterns (comma-separated)"
  echo "  --force        Don't validate before running"
  echo "  --ws           Sync workspace selection (preconfigured inc/exc, delete enabled)"
  echo "  --bags         Sync bags (preconfigured inc/exc, delete disabled)"
  echo ""
  exit 1
}

# ---- PARSE ARGS ----
MODE="$1"
FORCE=false
shift || true

while [[ $# -gt 0 ]]; do
  case "$1" in
    -u) REMOTE_USER="$2"; shift 2 ;;
    -h) REMOTE_HOST="$2"; shift 2 ;;
    -r) REMOTE_WS="$2"; shift 2 ;;
    -l) LOCAL_WS="$2"; shift 2 ;;
    -n) DRY_RUN="--dry-run"; shift ;;
    -d) DELETE_ENABLED=true; shift ;;
    -x)
      EXC=()
      IFS=',' read -ra EXC <<< "$2"
      EXCLUDES+=("${EXC[@]}")
      shift 2
      ;;
    -i)
      INC=()
      IFS=',' read -ra INC <<< "$2"
      INCLUDES+=("${INC[@]}")
      shift 2
      ;;
    --force) FORCE=true; shift ;;
    --ws)
        DELETE_ENABLED=true
        EXCLUDES+=("${WS_EXCLUDES[@]}")
        INCLUDES+=("${WS_INCLUDES[@]}")
        shift
        ;;
    --bags)
        DELETE_ENABLED=false
        EXCLUDES+=("${BAG_EXCLUDES[@]}")
        INCLUDES+=("${BAG_INCLUDES[@]}")
        shift
        ;;
    *) usage ;;
  esac
done

# ---- VALIDATION ----
if [[ -z "$MODE" ]]; then
  usage
fi

if [[ -z "$LOCAL_WS" ]]; then
  echo "Could not determine local workspace root."
  exit 1
fi

if [[ -z "$REMOTE_USER" || -z "$REMOTE_HOST" || -z "$REMOTE_WS" ]]; then
  echo "Missing remote configuration."
  echo "Provide -u, -h, and -r at least once."
  exit 1
fi

# ---- SAVE CACHE (NO include patterns!) ----
cat > "$CACHE_FILE" <<EOF
REMOTE_USER="$REMOTE_USER"
REMOTE_HOST="$REMOTE_HOST"
REMOTE_WS="$REMOTE_WS"
LOCAL_WS="$LOCAL_WS"
EOF

# ---- BUILD RSYNC OPTIONS ----
RSYNC_OPTS=("${RSYNC_BASE_OPTS[@]}")
[[ "$DELETE_ENABLED" == true ]] && RSYNC_OPTS+=(--delete)
[[ -n "$DRY_RUN" ]] && RSYNC_OPTS+=("$DRY_RUN")

# ---- BUILD FILTER RULES ----
RSYNC_FILTERS=()
for exc in "${EXCLUDES[@]}"; do
    RSYNC_FILTERS+=(--exclude "$exc")
done
for inc in "${INCLUDES[@]}"; do
    RSYNC_FILTERS+=(--include "$inc")
done
RSYNC_FILTERS+=(--exclude "*")

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
    ;;
  *)
    usage ;;
esac

if [[ "$MODE" == "push" || "$MODE" == "twoway" ]]; then
    PUSH_CMD=(
        rsync
        "${RSYNC_OPTS[@]}"
        -e "ssh $SSH_OPTS"
        "${RSYNC_FILTERS[@]}"
        "$LOCAL_WS/"
        "$REMOTE:$REMOTE_WS/"
    )
    if [[ "${FORCE}" == false ]]; then
        echo "${PUSH_CMD[@]}"
    fi
fi
if [[ "$MODE" == "pull" || "$MODE" == "twoway" ]]; then
    PULL_CMD=(
        rsync
        "${RSYNC_OPTS[@]}"
        -e "ssh $SSH_OPTS"
        "${RSYNC_FILTERS[@]}"
        "$REMOTE:$REMOTE_WS/"
        "$LOCAL_WS/"
    )
    if [[ "${FORCE}" == false ]]; then
        echo "${PULL_CMD[@]}"
    fi
fi
if [[ "${FORCE}" == false ]]; then
    read -p "Do you want to continue? (Y/n): " confirm
    [[ "$confirm" != "Y" ]] && exit 0
fi

if [[ -n "${PUSH_CMD}" ]]; then
    "${PUSH_CMD[@]}"
fi
if [[ -n "${PULL_CMD}" ]]; then
    "${PULL_CMD[@]}"
fi

echo "Sync complete."
echo "Cached config: $CACHE_FILE"
