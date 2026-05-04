#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Scan manipulation sweep — minimal version.
#
# Assumes the simulation is ALREADY running (Gazebo + SITL + MAVROS + LIO-SAM +
# drone in the air). This script only orchestrates the attacker + logger for
# one run at a time, or a small batch if SLAM survives between trials.
#
# Single-run mode (recommended for the fast/destructive rates):
#   ./sweep_runner.sh 2.0 1
#   ./sweep_runner.sh 5.0 1
#   # restart sim between destructive trials — SLAM does not recover.
#
# Batch mode (useful for 0.5°/s where SLAM usually survives):
#   ./sweep_runner.sh batch 0.5 3
#   # runs trial 1, 2, 3 at 0.5°/s with 30s cooldown between.
#
# All runs append to the same summary.csv. Trial numbers are not auto-
# incremented across separate invocations — you pass the trial number in.
# -----------------------------------------------------------------------------
set -u

RESULTS_DIR="${RESULTS_DIR:-$HOME/results/scan_sweep}"
ATTACKER_PKG="${ATTACKER_PKG:-swarm_mission}"
LOGGER_NODE="${LOGGER_NODE:-slam_failure_logger}"   # ros2 run swarm_mission slam_failure_logger
BASELINE_S="${BASELINE_S:-10}"
ATTACK_S="${ATTACK_S:-60}"
RECOVERY_S="${RECOVERY_S:-30}"
TEST_WINDOW_S="${TEST_WINDOW_S:-180}"   # logger exits after this many seconds post-attack-start
COOLDOWN_S="${COOLDOWN_S:-30}"

SUMMARY_CSV="${RESULTS_DIR}/summary.csv"
mkdir -p "${RESULTS_DIR}"

# ROS2 --ros-args parses "10" as INTEGER and "10.0" as DOUBLE. Our params
# are declared as DOUBLE in Python, so we must emit a decimal point.
to_f() {
  case "$1" in
    *.*) echo "$1" ;;
    *)   echo "$1.0" ;;
  esac
}

run_single() {
  local rate="$1"
  local trial="$2"

  local zombies
  zombies=$(ros2 node list 2>/dev/null | grep -cE "lidar_manipulator|slam_failure_logger" || true)
  if [[ "${zombies}" -gt 0 ]]; then
    echo "[ERROR] ${zombies} zombie attacker/logger node(s) in graph — clean before running" >&2
    return 2
  fi

  local run_id="rate_${rate}_run${trial}"
  local run_dir="${RESULTS_DIR}/rate_${rate}/run${trial}"
  mkdir -p "${run_dir}"

  echo
  echo "============================================================"
  echo "[RUN] ${run_id}  →  ${run_dir}"
  echo "============================================================"

  # Logger in background — self-terminates on failure detection or timeout.
  ros2 run "${ATTACKER_PKG}" "${LOGGER_NODE}" --ros-args \
      -p rotation_rate_dps:="$(to_f "${rate}")" \
      -p run_id:="${run_id}" \
      -p output_csv:="${SUMMARY_CSV}" \
      -p test_window_s:="$(to_f "${TEST_WINDOW_S}")" \
      >"${run_dir}/logger.log" 2>&1 &
  local logger_pid=$!
  echo "[logger] pid=${logger_pid}"
  sleep 1

  # Attacker in background.
  ros2 run "${ATTACKER_PKG}" lidar_manipulator --ros-args \
      -p target_drone:=drone1 \
      -p mode:=drift \
      -p drift_type:=rotate \
      -p drift_rate_deg:="$(to_f "${rate}")" \
      -p baseline_duration:="$(to_f "${BASELINE_S}")" \
      -p attack_duration:="$(to_f "${ATTACK_S}")" \
      -p recovery_duration:="$(to_f "${RECOVERY_S}")" \
      -p output_dir:="${run_dir}" \
      >"${run_dir}/manipulator.log" 2>&1 &
  local atk_pid=$!
  echo "[attacker] pid=${atk_pid}"

  # Wait for logger to self-exit, or hard-cap at baseline+attack+recovery+slack.
  local hard_cap=$((3*( BASELINE_S + ATTACK_S + RECOVERY_S)))
  local waited=0
  while kill -0 "${logger_pid}" 2>/dev/null; do
    sleep 2
    waited=$(( waited + 2 ))
    if (( waited >= hard_cap )); then
      echo "[runner] hard cap ${hard_cap}s reached"
      kill -TERM "${logger_pid}" 2>/dev/null || true
      break
    fi
  done

  # Kill the attacker. ros2 run's SIGTERM often only hits the wrapper,
  # so pkill the node executable too.
  sleep 2
  kill -TERM "${atk_pid}" 2>/dev/null || true
  wait "${atk_pid}" 2>/dev/null || true
  pkill -9 -f "${ATTACKER_PKG}.lidar_manipulator" 2>/dev/null || true
  pkill -9 -f "${ATTACKER_PKG}.${LOGGER_NODE}" 2>/dev/null || true
  sleep 1

  # Stash the attacker's own metrics CSV with a predictable name.
  local mcsv
  mcsv=$(ls -t "${run_dir}"/lidar_manip_metrics_*.csv 2>/dev/null | head -n1 || true)
  [[ -n "${mcsv}" ]] && cp "${mcsv}" "${run_dir}/manipulator_metrics.csv"

  echo "[done] ${run_id}"
  tail -n1 "${SUMMARY_CSV}" 2>/dev/null || true
}

usage() {
  cat <<EOF
Usage:
  $0 <rate_dps> <trial_number>          # one run at a specific rate + trial
  $0 batch <rate_dps> <num_trials>      # N trials at one rate, with cooldown

Examples:
  $0 1.0 1
  $0 5.0 1
  $0 batch 0.5 3

Env overrides:
  RESULTS_DIR=${RESULTS_DIR}
  ATTACKER_PKG=${ATTACKER_PKG}
  LOGGER_NODE=${LOGGER_NODE}
  BASELINE_S=${BASELINE_S} ATTACK_S=${ATTACK_S} RECOVERY_S=${RECOVERY_S}
  TEST_WINDOW_S=${TEST_WINDOW_S} COOLDOWN_S=${COOLDOWN_S}
EOF
}

if [[ $# -lt 2 ]]; then
  usage
  exit 1
fi

if [[ "$1" == "batch" ]]; then
  rate="$2"
  n="$3"
  for t in $(seq 1 "${n}"); do
    run_single "${rate}" "${t}"
    if (( t < n )); then
      echo "[cooldown] ${COOLDOWN_S}s before next trial"
      sleep "${COOLDOWN_S}"
    fi
  done
else
  run_single "$1" "$2"
fi

echo
echo "summary: ${SUMMARY_CSV}"
