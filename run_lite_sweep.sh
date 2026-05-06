#!/usr/bin/env bash
# run_lite_sweep.sh
# =================
# Driver for the Layer 1 scaling experiments using swarm_lite_sim.py.
#
# Sweep grid:
#   N (drones)        : 2, 5, 10
#   attack            : none, coverage_spoof, phantom_drone,
#                       selective_denial, interleaved
#   spoof_delay       : 0.5
#   trials per cell   : 5  (override with TRIALS=...)
#
# Plus one Sybil / BFT experiment at N=10, f+1=4 phantoms.
#
# Output: $OUT/N${N}_${ATK}.csv and $OUT/sybil_N10.csv
#
# Usage:
#   chmod +x run_lite_sweep.sh
#   ./run_lite_sweep.sh                # full sweep
#   TRIALS=3 ./run_lite_sweep.sh       # quick smoke test
#   N_LIST="2" ./run_lite_sweep.sh     # validate against full-stack at N=2

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

OUT="${OUT:-$HOME/lite_sim_results}"
mkdir -p "$OUT"

N_LIST="${N_LIST:-2 5 10}"
ATTACKS="${ATTACKS:-none coverage_spoof phantom_drone selective_denial interleaved}"
TRIALS="${TRIALS:-5}"
SPOOF_DELAY="${SPOOF_DELAY:-0.5}"
DISCOVERY="${DISCOVERY:-3.0}"
WP_PER_DRONE="${WP_PER_DRONE:-9}"
MEAN_FLIGHT="${MEAN_FLIGHT:-3.5}"
STD_FLIGHT="${STD_FLIGHT:-0.5}"
TIMEOUT="${TIMEOUT:-90}"

ATTACKER_PY="${ATTACKER_PY:-$HOME/ros2_ws/src/swarm_mission/swarm_mission/network_attacker.py}"

set +u  # ROS setup scripts reference unset vars; relax strict mode for sourcing
source /opt/ros/humble/setup.bash || true
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
set -u

cleanup() {
    pkill -9 -f swarm_lite_sim.py        2>/dev/null || true
    pkill -9 -f ground_truth_logger_lite 2>/dev/null || true
    pkill -9 -f network_attacker.py      2>/dev/null || true
    pkill -9 -f interleaved_attacker.py  2>/dev/null || true
    pkill -9 -f sybil_attacker_lite.py   2>/dev/null || true
}
trap cleanup EXIT

run_trial () {
    local N=$1 ATK=$2 RUN=$3
    local WP=$((WP_PER_DRONE * N))
    local CSV="$OUT/N${N}_${ATK}.csv"

    echo "=== N=$N attack=$ATK run=$RUN/$TRIALS ==="

    # 1. Ground-truth logger (background)
    python3 "$HERE/ground_truth_logger_lite.py" \
        --num-drones "$N" --total-wp "$WP" \
        --scenario "$ATK" --output "$CSV" \
        --mission-timeout "$TIMEOUT" \
        > "$OUT/last_logger.log" 2>&1 &
    local LOGGER_PID=$!
    sleep 1

    # 2. Attacker (background, optional)
    local ATK_PID=""
    case "$ATK" in
        none)
            ;;
        coverage_spoof|phantom_drone|selective_denial)
            python3 "$ATTACKER_PY" --ros-args \
                -p attack:="$ATK" -p target_drone:=drone2 \
                -p discovery_time:="$DISCOVERY" \
                -p spoof_delay:="$SPOOF_DELAY" \
                > "$OUT/last_attacker.log" 2>&1 &
            ATK_PID=$!
            ;;
        interleaved)
            python3 "$HERE/interleaved_attacker.py" \
                --num-drones "$N" \
                --waypoints-per-drone "$WP_PER_DRONE" \
                --discovery-time "$DISCOVERY" \
                --spoof-delay "$SPOOF_DELAY" \
                > "$OUT/last_attacker.log" 2>&1 &
            ATK_PID=$!
            ;;
        *)
            echo "Unknown attack: $ATK"; return 1 ;;
    esac
    sleep 0.5

    # 3. Virtual drones (foreground)
    python3 "$HERE/swarm_lite_sim.py" \
        --num-drones "$N" \
        --waypoints-per-drone "$WP_PER_DRONE" \
        --mean-flight "$MEAN_FLIGHT" \
        --std-flight "$STD_FLIGHT" \
        --mission-timeout "$TIMEOUT" \
        --seed "$RUN" \
        > "$OUT/last_sim.log" 2>&1 || true

    # 4. Cleanup background processes
    [ -n "$ATK_PID" ] && kill -9 "$ATK_PID"   2>/dev/null || true
    sleep 1
    kill -9 "$LOGGER_PID" 2>/dev/null || true
    wait 2>/dev/null || true
    sleep 1
}

echo "OUT=$OUT  N_LIST=$N_LIST  ATTACKS=$ATTACKS  TRIALS=$TRIALS"

for N in $N_LIST; do
    for ATK in $ATTACKS; do
        for RUN in $(seq 1 "$TRIALS"); do
            run_trial "$N" "$ATK" "$RUN"
        done
    done
done

# ---- Sybil headline experiment at N=10, f+1=4 phantoms ----
if [[ " $N_LIST " == *" 10 "* ]]; then
    echo "=== Sybil headline: N=10 + 4 phantoms, n=$TRIALS ==="
    SYBIL_CSV="$OUT/sybil_N10.csv"
    for RUN in $(seq 1 "$TRIALS"); do
        WP=$((WP_PER_DRONE * 10))
        python3 "$HERE/ground_truth_logger_lite.py" \
            --num-drones 10 --total-wp "$WP" \
            --scenario sybil_4phantoms \
            --output "$SYBIL_CSV" \
            --mission-timeout "$TIMEOUT" \
            > "$OUT/last_logger.log" 2>&1 &
        LOGGER_PID=$!
        sleep 1
        python3 "$HERE/sybil_attacker_lite.py" \
            --num-real 10 --num-phantoms 4 --total-wp "$WP" \
            --discovery-time "$DISCOVERY" \
            --spoof-delay "$SPOOF_DELAY" \
            > "$OUT/last_attacker.log" 2>&1 &
        ATK_PID=$!
        sleep 0.5
        python3 "$HERE/swarm_lite_sim.py" \
            --num-drones 10 \
            --waypoints-per-drone "$WP_PER_DRONE" \
            --mean-flight "$MEAN_FLIGHT" \
            --std-flight "$STD_FLIGHT" \
            --mission-timeout "$TIMEOUT" \
            --seed "$RUN" \
            > "$OUT/last_sim.log" 2>&1 || true
        kill -9 "$ATK_PID" 2>/dev/null || true
        sleep 1
        kill -9 "$LOGGER_PID" 2>/dev/null || true
        wait 2>/dev/null || true
        sleep 1
    done
fi

echo
echo "Done. Results in $OUT/"
ls -la "$OUT/"
