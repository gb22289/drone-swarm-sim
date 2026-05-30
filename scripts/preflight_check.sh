#!/usr/bin/env bash
# preflight_check.sh
#
# Run before any experimental trial. Verifies that the live stack is
# healthy enough to produce meaningful measurements:
#   - IMU and LiDAR topics are publishing at expected rates
#   - vision_pose flow is reaching MAVROS
#   - No zombie processes from earlier trials are lingering
#
# This is the "is the stack hot and clean?" check. For a deeper
# catalogue of lingering state (DDS shm, SITL temp dirs, etc.) use
# state_audit.sh instead.
#
# Exit code:
#   0  all checks passed
#   1  one or more checks failed (details printed)
#
# Usage:
#   ./preflight_check.sh [--drone N]   # default: drone1

set -u

DRONE="drone1"
if [ "${1:-}" = "--drone" ] && [ -n "${2:-}" ]; then
    DRONE="drone${2}"
fi

FAIL=0
pass()  { echo "  [ OK ]  $1"; }
fail()  { echo "  [FAIL]  $1"; FAIL=1; }
warn()  { echo "  [WARN]  $1"; }
header(){ echo ""; echo "── $1 ──"; }

echo "============================================================"
echo "  PRE-FLIGHT CHECK — $DRONE"
echo "  $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

# ----------------------------------------------------------------
# 1. ROS2 environment sourced?
# ----------------------------------------------------------------
header "ROS2 environment"
if ! command -v ros2 >/dev/null 2>&1; then
    fail "ros2 CLI not on PATH — source install/setup.bash first"
    echo "============================================================"
    exit 1
fi
pass "ros2 CLI available ($(ros2 --version 2>/dev/null || echo present))"

# ----------------------------------------------------------------
# 2. Sensor topic rates
# ----------------------------------------------------------------
# Read 4s of hz output, take the median "average rate" line.
sample_hz() {
    local topic="$1"
    timeout 4 ros2 topic hz "$topic" 2>/dev/null \
        | grep -oE "average rate: [0-9.]+" \
        | tail -1 \
        | awk '{print $3}'
}

header "Sensor topic rates"

IMU_TOPIC="/${DRONE}/imu"
IMU_HZ=$(sample_hz "$IMU_TOPIC")
if [ -z "$IMU_HZ" ]; then
    fail "$IMU_TOPIC — no messages observed in 4s"
else
    awk -v r="$IMU_HZ" 'BEGIN { exit (r < 50 ? 1 : 0) }' \
        && pass "$IMU_TOPIC at ${IMU_HZ} Hz" \
        || fail "$IMU_TOPIC at ${IMU_HZ} Hz (expected >= 50 Hz)"
fi

LIDAR_TOPIC="/${DRONE}/lidar/points"
LIDAR_HZ=$(sample_hz "$LIDAR_TOPIC")
if [ -z "$LIDAR_HZ" ]; then
    fail "$LIDAR_TOPIC — no messages observed in 4s"
else
    # 0.94 Hz typical on dev hardware (47% RTF, 2 Hz sim-time). Accept >= 0.5.
    awk -v r="$LIDAR_HZ" 'BEGIN { exit (r < 0.5 ? 1 : 0) }' \
        && pass "$LIDAR_TOPIC at ${LIDAR_HZ} Hz" \
        || fail "$LIDAR_TOPIC at ${LIDAR_HZ} Hz (expected >= 0.5 Hz)"
fi

# ----------------------------------------------------------------
# 3. vision_pose flow into MAVROS
# ----------------------------------------------------------------
header "vision_pose flow"
VP_TOPIC="/${DRONE}/mavros/vision_pose/pose"
VP_HZ=$(sample_hz "$VP_TOPIC")
if [ -z "$VP_HZ" ]; then
    fail "$VP_TOPIC — no messages observed in 4s (LIO-SAM bridge not flowing)"
else
    awk -v r="$VP_HZ" 'BEGIN { exit (r < 5 ? 1 : 0) }' \
        && pass "$VP_TOPIC at ${VP_HZ} Hz" \
        || fail "$VP_TOPIC at ${VP_HZ} Hz (expected ~20 Hz)"
fi

# ----------------------------------------------------------------
# 4. Zombie processes from earlier trials
# ----------------------------------------------------------------
header "Zombie / leftover processes"
ZOMBIES=$(ps -eo pid,state,cmd | awk '$2 ~ /Z/' || true)
if [ -n "$ZOMBIES" ]; then
    fail "Found zombie processes:"
    echo "$ZOMBIES" | sed 's/^/        /'
else
    pass "No zombie processes"
fi

LEFTOVER_PATTERNS="lio_mavros_bridge|lidar_deskew_shim|lidar_manipulator|slam_failure_logger|imu_injector|pointcloud_injector|qos_poisoner|network_attacker"
# Count instances; we expect exactly one of each live node, but
# multiples of the same node indicate a botched cleanup.
DUPS=$(ps -eo cmd | grep -E "$LEFTOVER_PATTERNS" | grep -v grep \
       | sort | uniq -c | awk '$1 > 1 {print}' || true)
if [ -n "$DUPS" ]; then
    fail "Duplicate stack-node processes detected:"
    echo "$DUPS" | sed 's/^/        /'
else
    pass "No duplicate stack-node processes"
fi

echo ""
echo "============================================================"
if [ $FAIL -eq 0 ]; then
    echo "  PRE-FLIGHT: PASS"
    echo "============================================================"
    exit 0
else
    echo "  PRE-FLIGHT: FAIL — fix the issues above before running a trial"
    echo "============================================================"
    exit 1
fi
