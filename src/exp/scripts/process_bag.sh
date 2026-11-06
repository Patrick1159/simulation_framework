#!/usr/bin/env bash
# process_bag.sh — Offline reprocessing helper for a recorded experiment run.
#
# Reruns the stats_viz/ analysis pipeline against an existing raw.bag without
# re-running the experiment itself. Does not modify the bag.
#
# Usage:
#   process_bag.sh <run_dir> [--topic TRACKER_TOPIC] \
#                            [--occlusion-start SEC] \
#                            [--occlusion-duration SEC] \
#                            [--demo]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
STATS="${SCRIPT_DIR}/stats_viz"

# ---------- arg parsing ----------
RUN_DIR=""
TRACKER_TOPIC=""
OS=""
OD=""
DEMO=0

usage() {
    cat <<EOF
Usage: $0 <run_dir> [--topic TRACKER_TOPIC] [--occlusion-start SEC] [--occlusion-duration SEC] [--demo]

  <run_dir>             Directory containing raw.bag.
  --topic TOPIC         Tracker topic for extract_tracker.py (auto-detect if omitted).
  --demo                Enable occlusion Re-ID metric (step 9).
  --occlusion-start S   (required with --demo) occlusion start time, seconds.
  --occlusion-duration S (required with --demo) occlusion duration, seconds.
EOF
}

if [[ $# -lt 1 ]]; then
    usage >&2
    exit 1
fi

RUN_DIR="$1"
shift

while [[ $# -gt 0 ]]; do
    case "$1" in
        --topic)
            TRACKER_TOPIC="$2"
            shift 2
            ;;
        --occlusion-start)
            OS="$2"
            shift 2
            ;;
        --occlusion-duration)
            OD="$2"
            shift 2
            ;;
        --demo)
            DEMO=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

# ---------- validation ----------
if [[ ! -d "$RUN_DIR" ]]; then
    echo "ERROR: run_dir does not exist: $RUN_DIR" >&2
    exit 1
fi

if [[ ! -f "${RUN_DIR}/raw.bag" ]]; then
    echo "ERROR: raw.bag not found in: $RUN_DIR" >&2
    exit 1
fi

if [[ "$DEMO" -eq 1 ]]; then
    if [[ -z "$OS" || -z "$OD" ]]; then
        echo "ERROR: --demo requires both --occlusion-start and --occlusion-duration" >&2
        exit 1
    fi
fi

cd "$RUN_DIR"

# ---------- auto-detect tracker topic ----------
if [[ -z "$TRACKER_TOPIC" ]]; then
    echo "==> detecting tracker topic from raw.bag"
    CANDIDATES=(
        "/rematch/tracker_details"
        "/doa_obstacles"
        "/tracker/dynamic_obstacles"
        "/onboard_detector/dynamic_bboxes"
        "/states"
        "/obstacles"
    )
    BAG_INFO="$(rosbag info raw.bag 2>/dev/null || true)"
    for topic in "${CANDIDATES[@]}"; do
        # rosbag info prints lines like:  "topics:    /foo   123 msgs ..."
        # Match the topic followed by whitespace and a positive msg count.
        line="$(echo "$BAG_INFO" | grep -E "[[:space:]]${topic}[[:space:]]+[0-9]+[[:space:]]+msgs?" || true)"
        if [[ -n "$line" ]]; then
            count="$(echo "$line" | awk -v topic="$topic" '{for(i=1;i<=NF;i++) if($i==topic){print $(i+1); exit}}')"
            if [[ -n "$count" && "$count" -gt 0 ]]; then
                TRACKER_TOPIC="$topic"
                echo "    selected: $TRACKER_TOPIC ($count msgs)"
                break
            fi
        fi
    done
    if [[ -z "$TRACKER_TOPIC" ]]; then
        echo "WARN: no tracker topic auto-detected; defaulting to /rematch/tracker_details"
        TRACKER_TOPIC="/rematch/tracker_details"
    fi
fi

# ---------- step runner ----------
STEP12_OK=1
STEP3_OK=1

run_step() {
    local n="$1"
    local name="$2"
    shift 2
    echo "==> step ${n}: ${name}"
    if "$@"; then
        return 0
    else
        local rc=$?
        echo "WARN: step ${n} failed (exit=${rc}), continuing"
        return $rc
    fi
}

# ---------- pipeline ----------
set +e

run_step 1 "extract_gt" \
    python3 "${STATS}/extract_gt.py" --bag raw.bag --output gt.csv
[[ $? -ne 0 ]] && STEP12_OK=0

EXTRACT_TRACKER_EXTRA=""
if [[ "$TRACKER_TOPIC" == /onboard_detector/* ]]; then
    EXTRACT_TRACKER_EXTRA="--no-cluster"
fi

run_step 2 "extract_tracker (topic=${TRACKER_TOPIC})" \
    python3 "${STATS}/extract_tracker.py" --bag raw.bag --topic "$TRACKER_TOPIC" --output tracker.csv \
        ${EXTRACT_TRACKER_EXTRA}
[[ $? -ne 0 ]] && STEP12_OK=0

run_step 3 "calc_metrics" \
    python3 "${STATS}/calc_metrics.py" --gt gt.csv --tracker tracker.csv --bag raw.bag --output metrics.csv
[[ $? -ne 0 ]] && STEP3_OK=0

run_step 4 "calc_prediction_error" \
    python3 "${STATS}/calc_prediction_error.py" --gt gt.csv --tracker tracker.csv --output prediction_error.csv

PLANNER_TOPIC="/doa_obstacles_tmpc"
if [[ "$TRACKER_TOPIC" == "/doa_obstacles" || "$TRACKER_TOPIC" == "/rematch/tracker_details" || "$TRACKER_TOPIC" == "/tracker/dynamic_obstacles" ]]; then
    PLANNER_TOPIC="/doa_obstacles"
fi

run_step 5 "calc_planning_metrics" \
    python3 "${STATS}/calc_planning_metrics.py" --bag raw.bag --output-dir . \
        --planner-topic "$PLANNER_TOPIC"

run_step 5b "extract_cmd_vel" \
    python3 "${STATS}/extract_cmd_vel.py" raw.bag cmd_vel.csv

if [[ -f cmd_vel.csv ]]; then
    run_step 5c "calc_control_smoothness" \
        python3 "${STATS}/calc_control_smoothness.py" --csv cmd_vel.csv \
            --json control_smoothness.json --out-csv control_smoothness.csv
else
    echo "==> step 5c: skipped control_smoothness (cmd_vel.csv not present)"
fi

run_step 6 "calc_runtime_table" \
    python3 "${STATS}/calc_runtime_table.py" --bag raw.bag --output runtime.csv

echo "==> step 7: visualize_keyframes"
if [[ -f clearance_timeseries.csv ]]; then
    if ! python3 "${STATS}/visualize_keyframes.py" .; then
        echo "WARN: step 7 failed (exit=$?), continuing"
    fi
else
    echo "    skipped: clearance_timeseries.csv not present (GT or odom missing)"
fi

run_step 8 "plot_results" \
    python3 "${STATS}/plot_results.py" --data-dir .

if [[ "$DEMO" -eq 1 ]]; then
    run_step 9 "calc_occlusion_reid" \
        python3 "${STATS}/calc_occlusion_reid.py" \
            --gt gt.csv --tracker tracker.csv --output-dir . \
            --occlusion-start "$OS" --occlusion-duration "$OD"
fi

set -e

# ---------- summary ----------
echo "==> outputs in $(pwd):"
{
    ls -1 *.csv *.json 2>/dev/null
    [[ -d keyframes ]] && echo "keyframes/"
} | sort -u | tr '\n' ' '
echo ""

if [[ "$STEP12_OK" -eq 1 && "$STEP3_OK" -eq 1 ]]; then
    exit 0
else
    echo "ERROR: core steps 1-3 did not all succeed" >&2
    exit 1
fi
