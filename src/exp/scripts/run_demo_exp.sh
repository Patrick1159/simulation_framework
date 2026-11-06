#!/usr/bin/env bash
# run_demo_exp.sh
# Run demo occlusion experiment (per-method) with rosbag recording.
# Reads runner defaults from demo_occlusion.yaml; supports --dry-run.

set -euo pipefail

# ----- Constants ---------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
CONFIG_DIR="$WS_ROOT/src/exp/configs"
STATS="$SCRIPT_DIR/stats_viz"
PROCESS_BAG="$SCRIPT_DIR/process_bag.sh"

ALLOWED_METHODS=(ours_full ours_no_clip ours_no_sam stage1_only dodt fapp oracle_gt)

# ----- Helpers -----------------------------------------------------------
log() { printf '[exp] %s\n' "$*" >&2; }

usage() {
  cat <<'EOF'
Usage: run_demo_exp.sh --method M [options]

Required:
  --method {ours_full|ours_no_clip|ours_no_sam|stage1_only|dodt|fapp|oracle_gt}

Options:
  --apply-cmd-vel {true|false}  (default: true)
  --occlusion-start SEC         (default: 10.0)
  --occlusion-duration SEC      (default: 2.0)
  --duration SEC                (default: from yaml)
  --repeats N                   (default: 1)
  --config PATH                 (default: src/exp/configs/demo_occlusion.yaml)
  --out-root PATH               (default: src/exp/data/demo_occlusion)
  --use-rviz {true|false}       (default: true)
  --gui {true|false}            (default: true)
  --no-process
  --dry-run
  -h|--help
EOF
}

yaml_get() {
  python3 -c "import yaml,sys; d=yaml.safe_load(open('$1')); print(eval(\"d$2\"))"
}

in_list() {
  local needle="$1"; shift
  local x
  for x in "$@"; do [[ "$x" == "$needle" ]] && return 0; done
  return 1
}

# ----- Defaults ----------------------------------------------------------
METHOD=""
APPLY="true"
OS="10.0"
OD="2.0"
DURATION=""
REPEATS=1
CONFIG=""
OUT_ROOT=""
USE_RVIZ="true"
GUI="true"
NO_PROCESS=0
DRY_RUN=0

# ----- Argparse ----------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --method)             METHOD="$2"; shift 2 ;;
    --apply-cmd-vel)      APPLY="$2"; shift 2 ;;
    --occlusion-start)    OS="$2"; shift 2 ;;
    --occlusion-duration) OD="$2"; shift 2 ;;
    --duration)           DURATION="$2"; shift 2 ;;
    --repeats)            REPEATS="$2"; shift 2 ;;
    --config)             CONFIG="$2"; shift 2 ;;
    --out-root)           OUT_ROOT="$2"; shift 2 ;;
    --use-rviz)           USE_RVIZ="$2"; shift 2 ;;
    --gui)                GUI="$2"; shift 2 ;;
    --no-process)         NO_PROCESS=1; shift ;;
    --dry-run)            DRY_RUN=1; shift ;;
    -h|--help)            usage; exit 0 ;;
    *) log "unknown arg: $1"; usage; exit 2 ;;
  esac
done

# ----- Validation --------------------------------------------------------
if [[ -z "$METHOD" ]]; then log "ERROR: --method required"; usage; exit 2; fi
if ! in_list "$METHOD" "${ALLOWED_METHODS[@]}"; then
  log "ERROR: invalid --method '$METHOD' (allowed: ${ALLOWED_METHODS[*]})"
  exit 2
fi

# ----- Config resolution -------------------------------------------------
if [[ -z "$CONFIG" ]]; then
  CONFIG="$CONFIG_DIR/demo_occlusion.yaml"
fi
if [[ ! -f "$CONFIG" ]]; then
  log "ERROR: config not found: $CONFIG"
  exit 2
fi
ABS_CONFIG="$(cd "$(dirname "$CONFIG")" && pwd)/$(basename "$CONFIG")"

if [[ -z "$OUT_ROOT" ]]; then
  OUT_ROOT="$WS_ROOT/src/exp/data/demo_occlusion"
fi

# ----- Read yaml values --------------------------------------------------
DEFAULT_DURATION="$(yaml_get "$CONFIG" "['runner']['default_duration']")"
WARMUP_SEC="$(yaml_get "$CONFIG" "['runner']['warmup_sec']")"
FREE_DISK_MIN_GB="$(yaml_get "$CONFIG" "['runner']['free_disk_min_gb']")"
TOPICS="$(python3 -c "import yaml; print(' '.join(yaml.safe_load(open('$CONFIG'))['runner']['rosbag_topics']))")"

if [[ -z "$DURATION" ]]; then DURATION="$DEFAULT_DURATION"; fi

DSCENE="$(yaml_get "$CONFIG" "['scene']['dynamic_scene_id']")"

log "method=$METHOD apply_cmd_vel=$APPLY occlusion=[${OS}s, +${OD}s] duration=$DURATION repeats=$REPEATS"
log "config=$ABS_CONFIG"
log "out_root=$OUT_ROOT"

# ----- Trap --------------------------------------------------------------
LAUNCH_PID=""
BAG_PID=""
cleanup() {
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

dry_echo() { printf '[DRY] %s\n' "$*"; }

# ----- Loop --------------------------------------------------------------
LOOP_END=$REPEATS
if [[ "$DRY_RUN" -eq 1 ]]; then LOOP_END=1; fi

for ((i=1; i<=LOOP_END; i++)); do
  RUN_DIR="$OUT_ROOT/$METHOD/run_$i"
  log "----- run $i/$REPEATS  ->  $RUN_DIR -----"

  if [[ "$DRY_RUN" -eq 1 ]]; then
    dry_echo "mkdir -p $RUN_DIR"
    dry_echo "df-check >= ${FREE_DISK_MIN_GB} GB on $RUN_DIR"
    if [[ "$METHOD" == "fapp" ]]; then
      dry_echo "python3 -c 'import obj_state_msgs'  (skip run if missing)"
    fi
    dry_echo "roslaunch sup_exp demo_exp.launch method:=$METHOD dynamic_scene:=$DSCENE config_file:=$ABS_CONFIG use_rviz:=$USE_RVIZ gui:=$GUI apply_cmd_vel:=$APPLY &"
    dry_echo "sleep $WARMUP_SEC  # warmup"
    dry_echo "rosbag record -O $RUN_DIR/raw.bag $TOPICS &"
    dry_echo "sleep $DURATION"
    dry_echo "kill -INT \$BAG_PID; wait; kill -INT \$LAUNCH_PID; wait"
    if [[ "$NO_PROCESS" -eq 0 ]]; then
      dry_echo "bash $PROCESS_BAG $RUN_DIR --demo --occlusion-start $OS --occlusion-duration $OD"
    fi
    continue
  fi

  mkdir -p "$RUN_DIR"

  AVAIL_GB="$(df --output=avail -B1G "$RUN_DIR" | tail -1 | tr -d ' ')"
  if (( AVAIL_GB < FREE_DISK_MIN_GB )); then
    log "ERROR: insufficient disk: ${AVAIL_GB}G available, need ${FREE_DISK_MIN_GB}G at $RUN_DIR"
    exit 3
  fi

  if [[ "$METHOD" == "fapp" ]]; then
    if ! python3 -c "import obj_state_msgs" 2>/dev/null; then
      log "WARNING: obj_state_msgs not importable; FAPP run skipped."
      continue
    fi
  fi

  log "launching demo_exp.launch"
  roslaunch sup_exp demo_exp.launch \
    method:="$METHOD" \
    dynamic_scene:="$DSCENE" \
    config_file:="$ABS_CONFIG" \
    use_rviz:="$USE_RVIZ" \
    gui:="$GUI" \
    apply_cmd_vel:="$APPLY" &
  LAUNCH_PID=$!

  log "warmup ${WARMUP_SEC}s"
  sleep "$WARMUP_SEC"

  # shellcheck disable=SC2086
  rosbag record -O "$RUN_DIR/raw.bag" $TOPICS &
  BAG_PID=$!

  log "recording for ${DURATION}s"
  sleep "$DURATION"

  log "stopping rosbag (pid=$BAG_PID)"
  kill -INT "$BAG_PID" 2>/dev/null || true
  wait "$BAG_PID" 2>/dev/null || true
  BAG_PID=""

  log "stopping launch (pid=$LAUNCH_PID)"
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  LAUNCH_PID=""

  if [[ "$NO_PROCESS" -eq 0 ]]; then
    log "processing bag"
    if ! bash "$PROCESS_BAG" "$RUN_DIR" --demo --occlusion-start "$OS" --occlusion-duration "$OD"; then
      log "WARNING: process_bag.sh failed for $RUN_DIR; continuing"
    fi
  fi
done

# ----- Post-run aggregation ---------------------------------------------
log "all runs complete; data under $OUT_ROOT"

if [[ "$DRY_RUN" -eq 1 ]]; then
  dry_echo "python3 $STATS/summarize_sup_results.py --root $OUT_ROOT"
  dry_echo "python3 $STATS/sup_exp_figures.py --root $OUT_ROOT"
  exit 0
fi

if (( REPEATS > 0 )); then
  python3 "$STATS/summarize_sup_results.py" --root "$OUT_ROOT" || true
  python3 "$STATS/sup_exp_figures.py" --root "$OUT_ROOT" || true
fi

log "done."
