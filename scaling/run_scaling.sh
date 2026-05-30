#!/usr/bin/env bash
#
# run_scaling.sh — reproduce Table VII of the IEEE paper.
#
# Sweeps the lite discrete-event simulator across:
#   N ∈ {2, 5, 10}                                    (swarm size)
#   scenario ∈ {none, coverage_spoof, phantom_drone,
#               selective_denial, interleaved, sybil_4phantoms}
#
# Each (N, scenario) pair is run REPS times. Results are appended to
# lite_sim_results/N{N}_{scenario}.csv. After the sweep, the per-config
# CSVs are concatenated into data/scaling_results.csv at the repo root.
#
# Usage:
#   ./run_scaling.sh                # full sweep (default REPS=10)
#   REPS=3 ./run_scaling.sh         # quick sanity sweep
#   N_VALUES="5 10" ./run_scaling.sh # subset
#
# Prerequisites: ROS2 Humble sourced, swarm_mission package built and
# sourced (so `ros2 run swarm_mission` works), Python deps from
# requirements.txt installed.
#
set -u

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="${OUT_DIR:-$HOME/lite_sim_results}"
mkdir -p "$OUT_DIR"

REPS="${REPS:-10}"
N_VALUES="${N_VALUES:-2 5 10}"
WP_PER_DRONE="${WP_PER_DRONE:-9}"
DISCOVERY_S="${DISCOVERY_S:-3.0}"
SPOOF_DELAY_S="${SPOOF_DELAY_S:-0.5}"
SCENARIOS="${SCENARIOS:-none coverage_spoof phantom_drone selective_denial interleaved sybil_4phantoms}"

LITE_NODE="python3 $(dirname "$0")/virtual_drone_lite.py"
LITE_LOGGER="python3 $(dirname "$0")/gt_logger_lite.py"
SYBIL_NODE="python3 $(dirname "$0")/sybil_attacker_lite.py"
INTERLEAVED_NODE="python3 $(dirname "$0")/interleaved_attacker_lite.py"
NETWORK_NODE="ros2 run swarm_mission network_attacker"

pids=()
cleanup() {
    for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done
    pids=()
}
trap cleanup EXIT

launch_swarm() {
    local n=$1
    local total_wp=$2
    pids=()
    for i in $(seq 1 "$n"); do
        local zone_start=$(( (i - 1) * WP_PER_DRONE ))
        local zone_end=$(( i * WP_PER_DRONE ))
        $LITE_NODE --ros-args \
            -r __node:=virtual_drone_$i \
            -p drone_id:=drone$i \
            -p zone_start:=$zone_start \
            -p zone_end:=$zone_end \
            -p total_wp:=$total_wp \
            >>"$OUT_DIR/last_sim.log" 2>&1 &
        pids+=($!)
    done
}

launch_attacker() {
    local scenario=$1
    local n=$2
    local total_wp=$3
    case "$scenario" in
        none) return 0 ;;
        coverage_spoof|phantom_drone|selective_denial)
            $NETWORK_NODE --ros-args \
                -p attack:="$scenario" \
                -p target_drone:=drone2 \
                -p num_waypoints:=$total_wp \
                -p discovery_time:=$DISCOVERY_S \
                -p spoof_delay:=$SPOOF_DELAY_S \
                >>"$OUT_DIR/last_attacker.log" 2>&1 &
            pids+=($!)
            ;;
        interleaved)
            $INTERLEAVED_NODE --ros-args \
                -p n_real_drones:=$n \
                -p total_wp:=$total_wp \
                -p discovery_time_s:=$DISCOVERY_S \
                >>"$OUT_DIR/last_attacker.log" 2>&1 &
            pids+=($!)
            ;;
        sybil_4phantoms)
            $SYBIL_NODE --ros-args \
                -p n_real_drones:=$n \
                -p n_phantoms:=4 \
                -p total_wp:=$total_wp \
                -p discovery_time_s:=$DISCOVERY_S \
                -p spoof_delay_s:=$SPOOF_DELAY_S \
                >>"$OUT_DIR/last_attacker.log" 2>&1 &
            pids+=($!)
            ;;
    esac
}

one_run() {
    local n=$1
    local scenario=$2
    local total_wp=$(( n * WP_PER_DRONE ))
    local csv
    if [ "$scenario" = "sybil_4phantoms" ]; then
        csv="$OUT_DIR/sybil_N${n}.csv"
    else
        csv="$OUT_DIR/N${n}_${scenario}.csv"
    fi

    : >"$OUT_DIR/last_sim.log"
    : >"$OUT_DIR/last_attacker.log"
    : >"$OUT_DIR/last_logger.log"

    # Logger first, so it never misses an early publish.
    $LITE_LOGGER --ros-args \
        -p n_drones:=$n \
        -p total_wp:=$total_wp \
        -p scenario:="$scenario" \
        -p output_csv:="$csv" \
        >>"$OUT_DIR/last_logger.log" 2>&1 &
    pids+=($!)
    sleep 0.5

    launch_swarm "$n" "$total_wp"
    launch_attacker "$scenario" "$n" "$total_wp"

    # Bound: realistic mission < 60s under all scenarios. Watch the log
    # for the "All drones complete" sentinel from gt_logger_lite.
    local deadline=$((SECONDS + 60))
    while [ $SECONDS -lt $deadline ]; do
        if grep -q "Summary written" "$OUT_DIR/last_logger.log" 2>/dev/null; then
            break
        fi
        sleep 0.5
    done

    cleanup
    sleep 0.5
}

echo "Sweep: N=$N_VALUES, scenarios=$SCENARIOS, reps=$REPS, out=$OUT_DIR"
for n in $N_VALUES; do
    for scenario in $SCENARIOS; do
        for rep in $(seq 1 "$REPS"); do
            echo "  [N=$n] [$scenario] rep $rep/$REPS"
            one_run "$n" "$scenario"
        done
    done
done

# Aggregate into the canonical scaling_results.csv
AGG="$REPO_ROOT/data/scaling_results.csv"
mkdir -p "$(dirname "$AGG")"
echo "scenario,n_drones,total_wp,gt_visited,reported_visited,coverage_gap,gap_pct,false_claims,mission_time_s" >"$AGG"
for n in $N_VALUES; do
    for scenario in $SCENARIOS; do
        if [ "$scenario" = "sybil_4phantoms" ]; then
            f="$OUT_DIR/sybil_N${n}.csv"
        else
            f="$OUT_DIR/N${n}_${scenario}.csv"
        fi
        [ -f "$f" ] && tail -n +2 "$f" >>"$AGG"
    done
done
echo "Aggregated -> $AGG"
