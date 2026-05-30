#!/usr/bin/env bash
# state_audit.sh
#
# Catalogue every place that state could be lingering from a previous
# run of the simulation stack. Run this:
#   (a) IMMEDIATELY after a fresh PC reboot, before launching anything
#       — capture the "clean baseline".
#   (b) After your usual Ctrl+C / pkill shutdown sequence, before the
#       next launch — compare against the clean baseline.
#
# Differences between (a) and (b) tell you what survives your cleanup.
#
# Usage:  ./state_audit.sh [label]
#   label is optional, gets stamped into the output (e.g., "post-reboot")

LABEL="${1:-unlabeled}"
echo "============================================================"
echo "  STACK STATE AUDIT — $LABEL"
echo "  $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

section() { echo ""; echo "── $1 ──"; }

# ----------------------------------------------------------------
# 1. Stack-related processes
# ----------------------------------------------------------------
section "Process table — anything matching the stack"
PATTERNS="gz sim|gzserver|gazebo|ign gazebo|arducopter|sim_vehicle|mavros|lio_sam|lio_mavros_bridge|lidar_deskew_shim|lidar_manipulator|slam_failure_logger|imu_injector|pointcloud_injector|qos_poisoner|ros_gz_bridge|robot_state_publisher|static_transform_publisher|rviz2|_ros2_daemon"
ps_out=$(ps -ef | grep -E "$PATTERNS" | grep -v grep || true)
if [ -z "$ps_out" ]; then
    echo "(none — process table is clean)"
else
    echo "$ps_out"
    echo ""
    echo "Count: $(echo "$ps_out" | wc -l) processes"
fi

# ----------------------------------------------------------------
# 2. Defunct (zombie) processes
# ----------------------------------------------------------------
section "Zombie processes (status Z)"
zombies=$(ps -eo pid,ppid,state,cmd | awk '$3=="Z"' || true)
if [ -z "$zombies" ]; then
    echo "(none)"
else
    echo "$zombies"
fi

# ----------------------------------------------------------------
# 3. DDS shared memory
# ----------------------------------------------------------------
section "DDS shared memory in /dev/shm"
shm_count=$(ls /dev/shm/ 2>/dev/null | grep -cE "fastrtps|fastdds|sem\.fastrtps|sem\.fastdds" || true)
echo "fastrtps/fastdds entries: $shm_count"
if [ "$shm_count" -gt 0 ]; then
    echo "Sample (first 10):"
    ls -la /dev/shm/ | grep -E "fastrtps|fastdds" | head -10
fi

other_shm=$(ls /dev/shm/ 2>/dev/null | grep -vE "fastrtps|fastdds" || true)
if [ -n "$other_shm" ]; then
    echo ""
    echo "Other /dev/shm entries that may belong to the stack:"
    ls -la /dev/shm/ | grep -vE "^d|^total|fastrtps|fastdds" | head -10
fi

# ----------------------------------------------------------------
# 4. ros2 daemon state
# ----------------------------------------------------------------
section "ros2 daemon"
daemon_pid=$(pgrep -f _ros2_daemon || true)
if [ -z "$daemon_pid" ]; then
    echo "Daemon: NOT running"
else
    echo "Daemon: running (pid $daemon_pid)"
    if command -v ros2 > /dev/null; then
        node_count=$(ros2 node list 2>/dev/null | wc -l)
        echo "Nodes registered with daemon: $node_count"
        if [ "$node_count" -gt 0 ]; then
            echo "Sample (first 15):"
            ros2 node list 2>/dev/null | head -15
        fi
    fi
fi

# ----------------------------------------------------------------
# 5. Network sockets — ROS2 + MAVLink
# ----------------------------------------------------------------
section "Network sockets — relevant ports"
ports="14550|14551|14552|14555|14560|14561|11311|7400|7401|7410|7411|7420|7421"
ss_out=$(ss -tulpn 2>/dev/null | grep -E ":($ports) " || true)
if [ -z "$ss_out" ]; then
    echo "(no listeners on common ROS/MAVLink ports)"
else
    echo "$ss_out"
fi

# ----------------------------------------------------------------
# 6. Ardupilot / SITL temp state
# ----------------------------------------------------------------
section "ArduPilot / SITL temp files"
ap_dirs=("/tmp/ardupilot" "/tmp/sitl" "/tmp/ArduCopter" "$HOME/.ardupilot" "$HOME/sim/ardupilot/ArduCopter/logs")
for d in "${ap_dirs[@]}"; do
    if [ -e "$d" ]; then
        size=$(du -sh "$d" 2>/dev/null | awk '{print $1}')
        echo "  $d  ($size)"
    fi
done
sitl_files=$(ls /tmp/ 2>/dev/null | grep -iE "ardupilot|sitl|mavlink" || true)
if [ -n "$sitl_files" ]; then
    echo "Other /tmp files:"
    echo "$sitl_files"
fi

# ----------------------------------------------------------------
# 7. Gazebo temp state
# ----------------------------------------------------------------
section "Gazebo temp state"
gz_dirs=("$HOME/.gz" "$HOME/.ignition" "/tmp/gz" "/tmp/.gazebo")
for d in "${gz_dirs[@]}"; do
    if [ -e "$d" ]; then
        size=$(du -sh "$d" 2>/dev/null | awk '{print $1}')
        echo "  $d  ($size)"
    fi
done

# ----------------------------------------------------------------
# 8. ROS2 log directory
# ----------------------------------------------------------------
section "ROS2 log directory"
ros_log_dir="$HOME/.ros/log"
if [ -d "$ros_log_dir" ]; then
    log_count=$(ls "$ros_log_dir" 2>/dev/null | wc -l)
    echo "$ros_log_dir contains $log_count entries"
    latest=$(ls -t "$ros_log_dir" 2>/dev/null | head -1)
    if [ -n "$latest" ]; then
        echo "Most recent: $latest"
    fi
fi

# ----------------------------------------------------------------
# 9. /tmp file descriptors held open by lingering processes
# ----------------------------------------------------------------
section "Open file descriptors in /tmp from stack processes"
if [ -n "$ps_out" ]; then
    # extract PIDs from the earlier process table
    pids=$(echo "$ps_out" | awk '{print $2}')
    for pid in $pids; do
        files=$(ls -l /proc/$pid/fd 2>/dev/null | grep -E "/tmp|/dev/shm" | head -5 || true)
        if [ -n "$files" ]; then
            cmd=$(ps -p $pid -o comm= 2>/dev/null)
            echo "PID $pid ($cmd):"
            echo "$files"
        fi
    done
else
    echo "(no relevant processes to check)"
fi

# ----------------------------------------------------------------
# 10. Free vs used memory
# ----------------------------------------------------------------
section "Memory usage"
free -h | head -2

echo ""
echo "============================================================"
echo "  AUDIT COMPLETE — $LABEL"
echo "============================================================"
