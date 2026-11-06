#!/usr/bin/env bash
# run_avoidance_exp.sh
# Run avoidance experiments (per-subscene, per-method) with rosbag recording.
# Reads runner defaults from the chosen yaml config; supports --dry-run.

set -euo pipefail

# ----- Constants ---------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"  # .../simulation_framework
CONFIG_DIR="$WS_ROOT/src/exp/configs"
STATS="$SCRIPT_DIR/stats_viz"
PROCESS_BAG="$SCRIPT_DIR/process_bag.sh"

# Source workspace devel so roslaunch can find packages and Python can
# import built ROS messages (obj_state_msgs, mpc_nav, etc.).
# Use setup.bash (not .zsh) since this script is bash.
_SETUP_SCRIPT="$WS_ROOT/devel/setup.bash"
if [[ -f "$_SETUP_SCRIPT" ]]; then
  source "$_SETUP_SCRIPT"
else
  log "ERROR: workspace not built — $WS_ROOT/devel/setup.bash not found"
  log "Run: cd $WS_ROOT && catkin build"
  exit 3
fi

# Gazebo's GPU depth camera (D435 sim) needs an X rendering context. Headless
# (DISPLAY unset, no xvfb) silently disables the depth sensor: /d435/depth/* has
# no publisher and every depth-consuming method (dodt/fapp/ours_*) gets zero
# detections while physics still runs (robot drives blind). Point at the real X
# server so the offscreen depth sensor can render (still gui:=false, no window).
if [[ -z "${DISPLAY:-}" ]]; then
  export DISPLAY=:1
  : "${XAUTHORITY:=/run/user/$(id -u)/gdm/Xauthority}"
  export XAUTHORITY
  printf '[exp] %s\n' "DISPLAY was unset; defaulting to DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY (needed for Gazebo depth camera)" >&2
fi

# T-MPC's jackalsimulator_planner links acados (libhpipm.so). These exports live
# in .zshrc; provide a fallback here so the bash harness works non-interactively.
if [[ -z "${ACADOS_SOURCE_DIR:-}" && -d /home/patrick/third_party/acados ]]; then
  export ACADOS_SOURCE_DIR=/home/patrick/third_party/acados
fi
if [[ -n "${ACADOS_SOURCE_DIR:-}" ]] && [[ ":${LD_LIBRARY_PATH:-}:" != *":${ACADOS_SOURCE_DIR}/lib:"* ]]; then
  export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH:-}"
fi

# Planner config the solver was built with (written by the solver generator).
PLANNER="lmpcc_goal"
if [[ -f "$WS_ROOT/.planner_config" ]]; then
  PLANNER="$(tr -d '[:space:]' < "$WS_ROOT/.planner_config")"
  [[ -z "$PLANNER" ]] && PLANNER="lmpcc_goal"
fi

ALLOWED_SUBSCENES=(pair_crossing front_crossing random_multi corridor occlusion)
ALLOWED_METHODS=(ours_full ours_no_clip ours_no_sam stage1_only dodt fapp oracle_gt)

# ----- Helpers -----------------------------------------------------------
log() { printf '[exp] %s\n' "$*" >&2; }

usage() {
  cat <<'EOF'
Usage: run_avoidance_exp.sh --subscene SUB --method M [options]

Required:
  --subscene {pair_crossing|front_crossing|random_multi|corridor}
  --method   {ours_full|ours_no_clip|ours_no_sam|stage1_only|dodt|fapp|oracle_gt}

Options:
  --duration SEC          (default: from yaml runner.default_duration)
  --repeats N             (default: 1)
  --base-seed N           (default: -1 = deterministic scene). When >=0, run i uses
                          scene_seed = N + i, perturbing per-obstacle speed/timing in
                          the crossing scene (scene 2) so each trial differs.
  --config PATH           (default: auto: avoidance_corridor.yaml if subscene=corridor
                                       else avoidance_crossing.yaml)
  --out-root PATH         (default: src/exp/data/avoidance, relative to workspace root)
  --use-rviz {true|false} (default: false)
  --gui {true|false}      (default: true)
  --enable-navigation {true|false} (default: true)
  --no-process            (skip process_bag.sh after each run)
  --dry-run               (print commands, do not execute)
  -h|--help               (print this usage and exit 0)

Note: cross-method comparison figures (gen_compare.py / compare_tracking_keyframes.py)
must be invoked manually after enough method directories exist.
EOF
}

yaml_get() {
  # yaml_get <path.yaml> <python-index-expression e.g. "['runner']['default_duration']">
  python3 -c "import yaml,sys; d=yaml.safe_load(open('$1')); print(eval(\"d$2\"))"
}

in_list() {
  local needle="$1"; shift
  local x
  for x in "$@"; do [[ "$x" == "$needle" ]] && return 0; done
  return 1
}

dyn_scene_for_subscene() {
  case "$1" in
    pair_crossing)  echo 2 ;;
    front_crossing) echo 3 ;;
    random_multi)   echo 0 ;;
    corridor)       echo 1 ;;
    occlusion)      echo 5 ;;
    *) return 1 ;;
  esac
}

# ----- Defaults ----------------------------------------------------------
SUBSCENE=""
METHOD=""
DURATION=""
REPEATS=1
BASE_SEED=-1
CONFIG=""
OUT_ROOT=""
USE_RVIZ="true"
GUI="true"
ENABLE_NAV="true"
NO_PROCESS=0
DRY_RUN=0

# ----- Argparse ----------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --subscene)      SUBSCENE="$2"; shift 2 ;;
    --method)        METHOD="$2"; shift 2 ;;
    --duration)      DURATION="$2"; shift 2 ;;
    --repeats)       REPEATS="$2"; shift 2 ;;
    --base-seed)     BASE_SEED="$2"; shift 2 ;;
    --config)        CONFIG="$2"; shift 2 ;;
    --out-root)      OUT_ROOT="$2"; shift 2 ;;
    --use-rviz)      USE_RVIZ="$2"; shift 2 ;;
    --gui)                GUI="$2"; shift 2 ;;
    --enable-navigation)  ENABLE_NAV="$2"; shift 2 ;;
    --no-process)    NO_PROCESS=1; shift ;;
    --dry-run)       DRY_RUN=1; shift ;;
    -h|--help)       usage; exit 0 ;;
    *) log "unknown arg: $1"; usage; exit 2 ;;
  esac
done

# ----- Validation --------------------------------------------------------
if [[ -z "$SUBSCENE" ]]; then log "ERROR: --subscene required"; usage; exit 2; fi
if [[ -z "$METHOD"   ]]; then log "ERROR: --method required";   usage; exit 2; fi

if ! in_list "$SUBSCENE" "${ALLOWED_SUBSCENES[@]}"; then
  log "ERROR: invalid --subscene '$SUBSCENE' (allowed: ${ALLOWED_SUBSCENES[*]})"
  exit 2
fi
if ! in_list "$METHOD" "${ALLOWED_METHODS[@]}"; then
  log "ERROR: invalid --method '$METHOD' (allowed: ${ALLOWED_METHODS[*]})"
  exit 2
fi

# ----- Config resolution -------------------------------------------------
if [[ -z "$CONFIG" ]]; then
  if [[ "$SUBSCENE" == "corridor" ]]; then
    CONFIG="$CONFIG_DIR/avoidance_corridor.yaml"
  else
    CONFIG="$CONFIG_DIR/avoidance_crossing.yaml"
  fi
fi
if [[ ! -f "$CONFIG" ]]; then
  log "ERROR: config not found: $CONFIG"
  exit 2
fi
ABS_CONFIG="$(cd "$(dirname "$CONFIG")" && pwd)/$(basename "$CONFIG")"

# Out root default
if [[ -z "$OUT_ROOT" ]]; then
  OUT_ROOT="/data/rosbags/avoidance"
fi

# ----- Read yaml values --------------------------------------------------
DEFAULT_DURATION="$(yaml_get "$CONFIG" "['runner']['default_duration']")"
WARMUP_SEC="$(yaml_get "$CONFIG" "['runner']['warmup_sec']")"
FREE_DISK_MIN_GB="$(yaml_get "$CONFIG" "['runner']['free_disk_min_gb']")"
TOPICS="$(python3 -c "import yaml; print(' '.join(yaml.safe_load(open('$CONFIG'))['runner']['rosbag_topics']))")"

if [[ -z "$DURATION" ]]; then DURATION="$DEFAULT_DURATION"; fi

DSCENE="$(dyn_scene_for_subscene "$SUBSCENE")"

log "subscene=$SUBSCENE method=$METHOD dyn_scene=$DSCENE duration=$DURATION repeats=$REPEATS"
log "config=$ABS_CONFIG"
log "planner=$PLANNER"
log "out_root=$OUT_ROOT"

# ----- Trap to clean up background processes -----------------------------
LAUNCH_PID=""
BAG_PID=""
GOAL_PID=""
cleanup() {
  if [[ -n "${GOAL_PID:-}" ]] && kill -0 "$GOAL_PID" 2>/dev/null; then
    kill -INT "$GOAL_PID" 2>/dev/null || true
    wait "$GOAL_PID" 2>/dev/null || true
  fi
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

# ----- dry-run helper ----------------------------------------------------
dry_echo() { printf '[DRY] %s\n' "$*"; }

# ----- Loop --------------------------------------------------------------
LOOP_END=$REPEATS
if [[ "$DRY_RUN" -eq 1 ]]; then LOOP_END=1; fi  # show one iteration

for ((i=1; i<=LOOP_END; i++)); do
  RUN_DIR="$OUT_ROOT/$SUBSCENE/$METHOD/run_$i"
  # Per-run scene seed: -1 keeps the scene deterministic; otherwise each run
  # gets a distinct seed so per-trial speed/timing differ (needed for N>=20).
  if (( BASE_SEED >= 0 )); then
    SEED=$((BASE_SEED + i))
  else
    SEED=-1
  fi
  log "----- run $i/$REPEATS  (scene_seed=$SEED)  ->  $RUN_DIR -----"

  if [[ "$DRY_RUN" -eq 1 ]]; then
    dry_echo "mkdir -p $RUN_DIR"
    dry_echo "echo $PLANNER > $RUN_DIR/planner.txt"
    dry_echo "df-check >= ${FREE_DISK_MIN_GB} GB on $RUN_DIR"
    if [[ "$METHOD" == "fapp" ]]; then
      dry_echo "python3 -c 'import obj_state_msgs'  (skip run if missing)"
    fi
    dry_echo "roslaunch sup_exp avoidance_exp.launch method:=$METHOD dynamic_scene:=$DSCENE config_file:=$ABS_CONFIG use_rviz:=$USE_RVIZ gui:=$GUI enable_navigation:=$ENABLE_NAV scene_seed:=$SEED planner:=$PLANNER auto_goal:=false &"
    dry_echo "sleep $WARMUP_SEC  # warmup"
    if [[ "$METHOD" == "fapp" ]]; then
      dry_echo "sleep 3  # fapp extra delay (perception init)"
    fi
    case "$METHOD" in
      ours_full)
        dry_echo "wait for >=2 msgs on /rematch_statistics_sam and /rematch_statistics_clip (perception warm-up)"
        ;;
      ours_no_clip)
        dry_echo "wait for >=2 msgs on /rematch_statistics_sam (perception warm-up; CLIP runs with dummy features)"
        ;;
      ours_no_sam)
        dry_echo "wait for >=2 msgs on /rematch_statistics_clip (perception warm-up; SAM node not launched)"
        ;;
    esac
    dry_echo "rosbag record -O $RUN_DIR/raw.bag $TOPICS &"
    dry_echo "wait (max 10s) for $RUN_DIR/raw.bag.active (or raw.bag) to appear, then sleep 1  # recording active"
    if [[ "$ENABLE_NAV" == "true" ]]; then
      dry_echo "rosrun sup_exp goal_publisher.py _delay:=0.5 &  # goal published only after recording started"
      dry_echo "wait (max 15s) for first msg on /move_base_simple/goal  # goal published"
    fi
    dry_echo "sleep $DURATION  # counted from goal publication"
    if [[ "$ENABLE_NAV" == "true" ]]; then
      dry_echo "kill -INT \$GOAL_PID; wait"
    fi
    dry_echo "kill -INT \$BAG_PID; wait; kill -INT \$LAUNCH_PID; wait"
    if [[ "$NO_PROCESS" -eq 0 ]]; then
      dry_echo "bash $PROCESS_BAG $RUN_DIR"
    fi
    continue
  fi

  mkdir -p "$RUN_DIR"
  echo "$PLANNER" > "$RUN_DIR/planner.txt"

  # Disk check
  AVAIL_GB="$(df --output=avail -B1G "$RUN_DIR" | tail -1 | tr -d ' ')"
  if (( AVAIL_GB < FREE_DISK_MIN_GB )); then
    log "ERROR: insufficient disk: ${AVAIL_GB}G available, need ${FREE_DISK_MIN_GB}G at $RUN_DIR"
    exit 3
  fi

  # FAPP pre-flight
  if [[ "$METHOD" == "fapp" ]]; then
    if ! python3 -c "import obj_state_msgs" 2>/dev/null; then
      log "WARNING: obj_state_msgs not importable; FAPP run skipped. (catkin build obj_state_msgs first)"
      continue
    fi
  fi

  # Launch
  log "launching avoidance_exp.launch"
  roslaunch sup_exp avoidance_exp.launch \
    method:="$METHOD" \
    dynamic_scene:="$DSCENE" \
    config_file:="$ABS_CONFIG" \
    use_rviz:="$USE_RVIZ" \
    gui:="$GUI" \
    enable_navigation:="$ENABLE_NAV" \
    scene_seed:="$SEED" \
    planner:="$PLANNER" \
    auto_goal:=false &
  LAUNCH_PID=$!

  log "warmup ${WARMUP_SEC}s"
  sleep "$WARMUP_SEC"

  # FAPP extra delay: pointcloud_generator + mot_mapping need extra time to
  # initialize before the goal is published, otherwise the robot moves before
  # the perception pipeline is producing /states output.
  if [[ "$METHOD" == "fapp" ]]; then
    log "fapp extra delay 3s (perception init)"
    sleep 3
  fi

  # Perception warm-up: the first real SAM/CLIP inference call is a cold
  # start (~0.3-0.9s vs ~0.01-0.03s steady-state). If that cold-start frame
  # lands right at track initialization it pollutes the KF velocity seed.
  # Block here (before rosbag starts) until each model that does real
  # inference for this method has produced >=2 samples, i.e. is warm.
  wait_for_warm_msgs() {
    local topic="$1" count="$2" timeout_s="$3"
    log "waiting for ${count} msg(s) on ${topic} (perception warm-up, timeout ${timeout_s}s)"
    if ! timeout "$timeout_s" rostopic echo -n "$count" "$topic" >/dev/null 2>&1; then
      log "WARNING: timed out waiting for ${count} msgs on ${topic}; proceeding without full warm-up"
    fi
  }
  WARMUP_TOPIC_TIMEOUT_SEC=60
  case "$METHOD" in
    ours_full)
      wait_for_warm_msgs /rematch_statistics_sam 2 "$WARMUP_TOPIC_TIMEOUT_SEC"
      wait_for_warm_msgs /rematch_statistics_clip 2 "$WARMUP_TOPIC_TIMEOUT_SEC"
      ;;
    ours_no_clip)
      # CLIP node runs with run_clip:=false (dummy features, no real
      # inference), so only SAM needs warm-up here.
      wait_for_warm_msgs /rematch_statistics_sam 2 "$WARMUP_TOPIC_TIMEOUT_SEC"
      ;;
    ours_no_sam)
      # SAM node is not launched at all for this method.
      wait_for_warm_msgs /rematch_statistics_clip 2 "$WARMUP_TOPIC_TIMEOUT_SEC"
      ;;
  esac

  # Rosbag
  # shellcheck disable=SC2086
  rosbag record -O "$RUN_DIR/raw.bag" $TOPICS &
  BAG_PID=$!

  # Wait until the bag file exists (rosbag writes raw.bag.active while
  # recording) so the goal is published only once recording is live.
  BAG_WAIT=0
  until [[ -e "$RUN_DIR/raw.bag.active" || -e "$RUN_DIR/raw.bag" ]]; do
    if ! kill -0 "$BAG_PID" 2>/dev/null; then
      log "WARNING: rosbag record exited before creating the bag"
      break
    fi
    if (( BAG_WAIT >= 20 )); then
      log "WARNING: bag file did not appear within 10s; proceeding"
      break
    fi
    sleep 0.5
    BAG_WAIT=$((BAG_WAIT + 1))
  done
  sleep 1  # let rosbag finish subscribing to its topics

  # Goal publisher: started by the harness (launch has auto_goal:=false) so
  # that the robot starts moving only after recording has begun. The yaml
  # (goal_pose) was loaded globally by the launch; delay is a private param.
  if [[ "$ENABLE_NAV" == "true" ]]; then
    log "starting goal_publisher (bag is recording)"
    rosrun sup_exp goal_publisher.py _delay:=0.5 &
    GOAL_PID=$!
    # The goal is latched, so this returns as soon as it has been published.
    if ! timeout 15 rostopic echo -n 1 /move_base_simple/goal >/dev/null 2>&1; then
      log "WARNING: no goal observed on /move_base_simple/goal within 15s"
    fi
  fi

  log "recording for ${DURATION}s (counted from goal publication)"
  sleep "$DURATION"

  if [[ -n "$GOAL_PID" ]]; then
    log "stopping goal_publisher (pid=$GOAL_PID)"
    kill -INT "$GOAL_PID" 2>/dev/null || true
    wait "$GOAL_PID" 2>/dev/null || true
    GOAL_PID=""
  fi

  log "stopping rosbag (pid=$BAG_PID)"
  kill -INT "$BAG_PID" 2>/dev/null || true
  wait "$BAG_PID" 2>/dev/null || true
  BAG_PID=""

  log "stopping launch (pid=$LAUNCH_PID)"
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  LAUNCH_PID=""

  # Process
  if [[ "$NO_PROCESS" -eq 0 ]]; then
    log "processing bag"
    if ! bash "$PROCESS_BAG" "$RUN_DIR"; then
      log "WARNING: process_bag.sh failed for $RUN_DIR; continuing"
    fi
  fi
done

# ----- Post-run aggregation ---------------------------------------------
log "all runs complete; data under $OUT_ROOT"

if [[ "$DRY_RUN" -eq 1 ]]; then
  dry_echo "python3 $STATS/summarize_sup_results.py --root $OUT_ROOT"
  if [[ "$SUBSCENE" == "corridor" ]]; then
    dry_echo "python3 $STATS/make_corridor_tables.py --root $OUT_ROOT/$SUBSCENE"
  fi
  dry_echo "python3 $STATS/sup_exp_figures.py --root $OUT_ROOT"
  exit 0
fi

if (( REPEATS > 0 )); then
  python3 "$STATS/summarize_sup_results.py" --root "$OUT_ROOT" || true
  if [[ "$SUBSCENE" == "corridor" ]]; then
    python3 "$STATS/make_corridor_tables.py" --root "$OUT_ROOT/$SUBSCENE" || true
  fi
  python3 "$STATS/sup_exp_figures.py" --root "$OUT_ROOT" || true
fi

log "done."
