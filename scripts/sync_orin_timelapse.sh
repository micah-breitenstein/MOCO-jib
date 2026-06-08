#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

REMOTE_USER="micah"
REMOTE_HOST="10.42.0.1"
REMOTE_DIR="/home/micah/projects/holy-grail/holy-grail-timelapse/timelapse"
LOCAL_ROOT="$SCRIPT_DIR/timelapse_orin"
FOLDER_NAME=""
WATCH_MODE=0
INTERVAL_SECONDS=60
RUN_ANALYZER=1
SSH_TIMEOUT_SECONDS=12
RSYNC_TIMEOUT_SECONDS=30
ANALYZER_PATH="$SCRIPT_DIR/camera_ai_analyzer.py"
ANALYZER_ARGS=""
SSH_KEY_FILE="${ORIN_SYNC_SSH_KEY:-$HOME/.ssh/id_ed25519_orin_nopass}"
LIST_VIEW="optimal"
STABLE_CHECK_DELAY_SECONDS=2
STABLE_MIN_AGE_SECONDS=3
ANALYZER_PYTHON=""

if [[ -x "$SCRIPT_DIR/../.venv/bin/python" ]]; then
  ANALYZER_PYTHON="$SCRIPT_DIR/../.venv/bin/python"
else
  ANALYZER_PYTHON="python3"
fi

usage() {
  cat <<'EOF'
Sync timelapse images from an Orin over SSH, then process only new frames.

Usage:
  ./sync_orin_timelapse.sh [--folder <timelapse_timestamp_folder>] [options]

Selection:
  --folder <name>              Sync only one remote timelapse folder (timelapse_YYYYMMDD_HHMMSS)
                               If omitted, auto-discovers all remote timelapse_* folders.

Connection options:
  --remote-user <user>         SSH user (default: micah)
  --remote-host <host>         SSH host/IP (default: 192.168.87.35)
  --remote-dir <path>          Remote timelapse directory
  --ssh-key <path>             SSH private key path (default: ORIN_SYNC_SSH_KEY or ~/.ssh/id_ed25519_orin_nopass)

Behavior options:
  --watch                      Run forever: sync/process every interval
  --interval <seconds>         Loop interval in watch mode (default: 60)
  --list-view <all|optimal>    Show remote directory listing using all/ or optimal/ paths (default: optimal)
  --local-root <path>          Local root that contains named folders
  --no-analyzer                Skip analyzer processing step
  --analyzer-path <path>       Analyzer script path (default: camera_ai_analyzer.py)
  --analyzer-args <args>       Extra args appended to analyzer command
  --help                       Show this help text

What it does:
  1) Finds remote timelapse_[timestamp] folders
  2) Incrementally syncs each remote folder's optimal/ files (resume-safe)
  3) Mirrors into local: timelapse_orin/timelapse_[timestamp]/optimal/
  4) Tracks already processed files in .sync_state/processed_files.txt (per timelapse folder)
  5) Processes only newly-arrived frames via camera_ai_analyzer.py --json
  6) Writes one JSON file per frame in .sync_state/analysis/

Examples:
  ./sync_orin_timelapse.sh

  ./sync_orin_timelapse.sh \
    --watch --interval 30

  ./sync_orin_timelapse.sh \
    --folder timelapse_20260602_204016 \
    --watch --interval 30

  ./sync_orin_timelapse.sh \
    --folder timelapse_20260602_204016 \
    --analyzer-args '--target-luma 118 --max-step-ev 0.67'
EOF
}

log() {
  printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*"
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"
}

rsync_supports_option() {
  local option="$1"
  rsync --help 2>&1 | grep -Fq -- "$option"
}

validate_folder_name() {
  local name="$1"
  [[ -n "$name" ]] || return 0
  [[ "$name" != *"/"* ]] || die "--folder must be a simple name, not a path"
  [[ "$name" != "." && "$name" != ".." ]] || die "invalid --folder value"
  [[ "$name" =~ ^timelapse_[0-9]{8}_[0-9]{6}$ ]] || die "--folder must match timelapse_YYYYMMDD_HHMMSS"
}

validate_list_view() {
  local mode="$1"
  [[ "$mode" == "all" || "$mode" == "optimal" ]] || die "--list-view must be all or optimal"
}

count_local_images() {
  local dir="$1"
  find "$dir" -maxdepth 1 -type f \( -name 'timelapse_*' -o -name 'capt_*' \) | wc -l | tr -d ' '
}

list_stable_remote_frames() {
  local remote_optimal_dir="$1"
  local ssh_cmd=(
    ssh
    -o BatchMode=yes
    -o NumberOfPasswordPrompts=0
    -o ConnectTimeout="${SSH_TIMEOUT_SECONDS}"
    -o ServerAliveInterval=15
  )
  if [[ -f "$SSH_KEY_FILE" ]]; then
    ssh_cmd+=( -o IdentitiesOnly=yes -i "$SSH_KEY_FILE" )
  fi

  local remote_q
  remote_q="$(printf '%q' "$remote_optimal_dir")"
  local delay_q
  delay_q="$(printf '%q' "$STABLE_CHECK_DELAY_SECONDS")"

  local remote_cmd
  remote_cmd="
if [ ! -d ${remote_q} ]; then
  exit 0
fi
tmp1=\"\$(mktemp)\"
tmp2=\"\$(mktemp)\"
cleanup() {
  rm -f \"\$tmp1\" \"\$tmp2\"
}
trap cleanup EXIT
{
  find ${remote_q} -maxdepth 1 -type f -name 'timelapse_*' -printf '%f\\t%s\\t%T@\\n'
  find ${remote_q} -maxdepth 1 -type f -name 'capt_*' -printf '%f\\t%s\\t%T@\\n'
} | LC_ALL=C sort -u >\"\$tmp1\"
sleep ${delay_q}
{
  find ${remote_q} -maxdepth 1 -type f -name 'timelapse_*' -printf '%f\\t%s\\t%T@\\n'
  find ${remote_q} -maxdepth 1 -type f -name 'capt_*' -printf '%f\\t%s\\t%T@\\n'
} | LC_ALL=C sort -u >\"\$tmp2\"
comm -12 \"\$tmp1\" \"\$tmp2\" | cut -f1 | LC_ALL=C sort -u
"

  "${ssh_cmd[@]}" "${REMOTE_USER}@${REMOTE_HOST}" "$remote_cmd"
}

sync_once() {
  local remote_optimal_dir="$1"
  local target_optimal_dir="$2"
  local before_count after_count
  local ssh_cmd
  local -a progress_args
  local stable_files
  before_count="$(count_local_images "$target_optimal_dir")"

  ssh_cmd="ssh -o BatchMode=yes -o NumberOfPasswordPrompts=0 -o ConnectTimeout=${SSH_TIMEOUT_SECONDS} -o ServerAliveInterval=15"
  if [[ -f "$SSH_KEY_FILE" ]]; then
    ssh_cmd+=" -o IdentitiesOnly=yes -i ${SSH_KEY_FILE}"
  fi

  if rsync_supports_option '--info=progress2'; then
    progress_args=(--info=progress2 --human-readable)
  else
    progress_args=(--progress)
  fi

  stable_files="$(list_stable_remote_frames "$remote_optimal_dir")" || return 1
  if [[ -z "$stable_files" ]]; then
    after_count="$(count_local_images "$target_optimal_dir")"
    echo $((after_count - before_count))
    return 0
  fi

  printf '%s\n' "$stable_files" | rsync \
    -av \
    "${progress_args[@]}" \
    --ignore-existing \
    --partial \
    --append \
    --timeout="$RSYNC_TIMEOUT_SECONDS" \
    --files-from=- \
    -e "$ssh_cmd" \
    "${REMOTE_USER}@${REMOTE_HOST}:${remote_optimal_dir}/" \
    "$target_optimal_dir/" || return 1

  after_count="$(count_local_images "$target_optimal_dir")"
  echo $((after_count - before_count))
}

list_remote_timelapse_dirs() {
  local ssh_cmd=(
    ssh
    -o BatchMode=yes
    -o NumberOfPasswordPrompts=0
    -o ConnectTimeout="${SSH_TIMEOUT_SECONDS}"
    -o ServerAliveInterval=15
  )
  if [[ -f "$SSH_KEY_FILE" ]]; then
    ssh_cmd+=( -o IdentitiesOnly=yes -i "$SSH_KEY_FILE" )
  fi

  local remote_dir_escaped
  remote_dir_escaped="$(printf '%q' "$REMOTE_DIR")"
  local remote_cmd="find ${remote_dir_escaped} -mindepth 1 -maxdepth 1 -type d -name 'timelapse_[0-9]*_[0-9]*' -printf '%f\\n' | LC_ALL=C sort"
  "${ssh_cmd[@]}" "${REMOTE_USER}@${REMOTE_HOST}" "$remote_cmd"
}

build_to_process_list() {
  local target_dir="$1"
  local state_dir="$2"
  local all_list="$state_dir/all_frames.txt"
  local processed_list="$state_dir/processed_files.txt"
  local processed_sorted="$state_dir/processed_files.sorted.txt"
  local to_process="$state_dir/to_process.txt"

  find "$target_dir" -maxdepth 1 -type f \( -name 'timelapse_*' -o -name 'capt_*' \) -print \
    | sed 's#^.*/##' \
    | LC_ALL=C sort -u >"$all_list"

  LC_ALL=C sort -u "$processed_list" >"$processed_sorted"
  comm -23 "$all_list" "$processed_sorted" >"$to_process"
  echo "$to_process"
}

process_new_files() {
  local target_dir="$1"
  local state_dir="$2"
  local analysis_dir="$state_dir/analysis"
  local processed_list="$state_dir/processed_files.txt"
  local failed_log="$state_dir/failed_processing.log"
  local to_process
  local processed_now=0
  local failed_now=0

  mkdir -p "$analysis_dir"
  to_process="$(build_to_process_list "$target_dir" "$state_dir")"

  if [[ ! -s "$to_process" ]]; then
    echo "0 0"
    return 0
  fi

  while IFS= read -r base_name; do
    [[ -n "$base_name" ]] || continue
    local image_path="$target_dir/$base_name"
    [[ -f "$image_path" ]] || continue

    local stem="${base_name%.*}"
    local out_json="$analysis_dir/${stem}.json"

    if [[ "$RUN_ANALYZER" -eq 1 ]]; then
      local cmd=("$ANALYZER_PYTHON" "$ANALYZER_PATH" --image "$image_path" --json)
      if [[ -n "$ANALYZER_ARGS" ]]; then
        # Intentional split for advanced analyzer flag passthrough.
        # shellcheck disable=SC2206
        local extra=( $ANALYZER_ARGS )
        cmd+=("${extra[@]}")
      fi

      if "${cmd[@]}" >"$out_json" 2>/dev/null; then
        printf '%s\n' "$base_name" >>"$processed_list"
        processed_now=$((processed_now + 1))
      else
        printf '%s\t%s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$base_name" >>"$failed_log"
        failed_now=$((failed_now + 1))
      fi
    else
      printf '%s\n' "$base_name" >>"$processed_list"
      processed_now=$((processed_now + 1))
    fi
  done <"$to_process"

  echo "$processed_now $failed_now"
}

run_cycle() {
  local timelapse_name="$1"
  local target_dir="$LOCAL_ROOT/$timelapse_name"
  local selected_subdir="$LIST_VIEW"
  local target_selected_dir="$target_dir/$selected_subdir"
  local state_dir="$target_dir/.sync_state"
  local remote_source_dir="$REMOTE_DIR/$timelapse_name/$selected_subdir"

  mkdir -p "$target_selected_dir" "$state_dir"
  touch "$state_dir/processed_files.txt"

  log "sync start: ${REMOTE_USER}@${REMOTE_HOST}:${remote_source_dir} -> ${target_selected_dir}"
  local synced_count
  synced_count="$(sync_once "$remote_source_dir" "$target_selected_dir")" || {
    log "sync failed"
    return 1
  }
  log "sync complete: new_files=${synced_count}"

  local process_counts processed_now failed_now
  process_counts="$(process_new_files "$target_selected_dir" "$state_dir")"
  processed_now="${process_counts%% *}"
  failed_now="${process_counts##* }"
  log "process complete: processed_new=${processed_now} failed=${failed_now}"
}

run_discovery_cycle() {
  local names
  names="$(list_remote_timelapse_dirs)" || {
    log "failed to list remote timelapse folders"
    return 1
  }

  if [[ -z "$names" ]]; then
    log "no remote timelapse_* folders found in ${REMOTE_DIR}"
    return 0
  fi

  echo "## Timelapse Directories"
  local listed=0
  while IFS= read -r name; do
    [[ -n "$name" ]] || continue
    if [[ -n "$FOLDER_NAME" && "$name" != "$FOLDER_NAME" ]]; then
      continue
    fi
    echo "- ${name}/${LIST_VIEW}"
    listed=1
  done <<<"$names"
  if [[ "$listed" -eq 0 ]]; then
    echo "- (none matched current filter)"
  fi

  while IFS= read -r name; do
    [[ -n "$name" ]] || continue
    if [[ -n "$FOLDER_NAME" && "$name" != "$FOLDER_NAME" ]]; then
      continue
    fi
    run_cycle "$name" || return 1
  done <<<"$names"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --folder)
      FOLDER_NAME="${2:-}"
      shift 2
      ;;
    --remote-user)
      REMOTE_USER="${2:-}"
      shift 2
      ;;
    --remote-host)
      REMOTE_HOST="${2:-}"
      shift 2
      ;;
    --remote-dir)
      REMOTE_DIR="${2:-}"
      shift 2
      ;;
    --ssh-key)
      SSH_KEY_FILE="${2:-}"
      shift 2
      ;;
    --watch)
      WATCH_MODE=1
      shift
      ;;
    --interval)
      INTERVAL_SECONDS="${2:-}"
      shift 2
      ;;
    --list-view)
      LIST_VIEW="${2:-}"
      shift 2
      ;;
    --local-root)
      LOCAL_ROOT="${2:-}"
      shift 2
      ;;
    --no-analyzer)
      RUN_ANALYZER=0
      shift
      ;;
    --analyzer-path)
      ANALYZER_PATH="${2:-}"
      shift 2
      ;;
    --analyzer-args)
      ANALYZER_ARGS="${2:-}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

validate_folder_name "$FOLDER_NAME"
validate_list_view "$LIST_VIEW"

if ! [[ "$INTERVAL_SECONDS" =~ ^[0-9]+$ ]] || [[ "$INTERVAL_SECONDS" -lt 5 ]]; then
  die "--interval must be an integer >= 5"
fi

require_cmd ssh
require_cmd rsync
if [[ "$RUN_ANALYZER" -eq 1 ]]; then
  require_cmd python3
  [[ -f "$ANALYZER_PATH" ]] || die "analyzer not found: $ANALYZER_PATH"
fi

mkdir -p "$LOCAL_ROOT"
if [[ -n "$FOLDER_NAME" ]]; then
  log "single-folder mode: ${FOLDER_NAME}"
else
  log "auto-discovery mode: all remote timelapse_* folders"
fi
log "local root: $LOCAL_ROOT"

if [[ "$WATCH_MODE" -eq 1 ]]; then
  log "watch mode enabled: interval=${INTERVAL_SECONDS}s"
  while true; do
    if ! run_discovery_cycle; then
      log "cycle failed; retrying after delay"
    fi
    log "sleeping ${INTERVAL_SECONDS}s"
    sleep "$INTERVAL_SECONDS"
  done
else
  run_discovery_cycle || exit 1
fi
