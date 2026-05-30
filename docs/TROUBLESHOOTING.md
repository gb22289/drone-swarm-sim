# Troubleshooting

Comprehensive failure-mode-to-fix table covering issues encountered during development. Cross-referenced with the relevant section in [SETUP.md](SETUP.md), [LAUNCH.md](LAUNCH.md), or [ATTACKS.md](ATTACKS.md) where applicable.

| Issue | Fix |
|---|---|
| `/clock` has 0 publishers | `bridge.yaml` is publishing to `/clock_raw` or similar. Fix `ros_topic_name` to `/clock` (SETUP §7), restart bridge |
| `Point cloud timestamp not available, deskew function disabled` | Deskew shim not running or started after LIO-SAM. Start shim first, then restart LIO-SAM (SETUP §8) |
| LIO-SAM odom diverges to 100+ metres during hover | Deskew disabled. Ensure shim is running and LIO-SAM's `pointCloudTopic` is `/droneN/lidar/points_timed` |
| `Large velocity, reset IMU-preintegration!` warnings during baseline | `/clock` not flowing correctly — check SETUP §7 bridge config |
| `gtsam::IndeterminantLinearSystemException` crash | Apply clock jump patches from `lio_sam_patches/imu_preintegration_dt_guards.patch` and rebuild |
| `pcl::KdTreeFLANN::setInputCloud` empty cloud + mapOptimization segfault | Apply `lio_sam_patches/mapoptmization_kdtree_check.patch`. Note: closes ~89% of crashes; one ICP-internal site remains uncovered (see dissertation §3.5.3) |
| `imu_injector_v2` shows zero LIO drift during attack | Verify `use_sim_time` is propagating to the injector — `self.get_clock().now()` must return sim-time, not wall-time. Also raise `accel_bias_x` from 5.0 to 10.0 |
| Multiple `imu_injector` instances zombie after Ctrl+C | `pkill -9 -f "swarm_mission.imu_injector"` then `ros2 daemon stop; sleep 1; ros2 daemon start` |
| `imu_injector_v2` doesn't see LIO odometry | Topic name should be `/drone1/lio_sam/mapping/odometry` (no `_incremental` suffix) |
| `PreArm: VisOdom: not healthy` | LIO-SAM not publishing — use GPS bootstrap procedure (LAUNCH §GPS-Denied Bootstrap) |
| `AHRS: waiting for home` | GPS not locked — restart SITL after setting `GPS1_TYPE 1` |
| `EKF3 IMU stopped aiding` | Re-enable compass: `COMPASS_ENABLE 1`, `EK3_SRC1_YAW 1` |
| `param set` Unknown setting | Run `param fetch` first to refresh cache |
| LiDAR not in ROS2 | Check bridge is running after Gazebo loads; verify gz topic is `/lidar/points/points` |
| LIO-SAM no odometry on flat ground | Switch to warehouse world — runway is too featureless for SLAM |
| `ros-humble-ros-gzharmonic` not found | Build ros_gz from source with `GZ_VERSION=harmonic` (SETUP §4) |
| LiDAR sensor registered but zero messages | Ensure `type="gpu_lidar"` in model SDF — `type="lidar"` is not supported in Gazebo Harmonic |
| LiDAR link renamed to `lidar_link(1)` | LiDAR block is in wrong model file — must be in `iris_with_standoffs`, not `iris_with_gimbal` (SETUP §6) |
| Drone ignores setpoint commands | Must be in GUIDED mode (`mode guided` in MAVProxy) and continuously publishing setpoints at ≥10 Hz |
| `/mavros/local_position/pose` not publishing | MAVROS not receiving vision pose — check `lio_mavros_bridge.py` is running AND rewriting stamp to wall-clock (SETUP §9) |
| LIO-SAM odometry z plummets to -300+ | Wrong extrinsics — `extrinsicRot` must flip NED→ENU, `extrinsicRPY` must be identity (SETUP §5) |
| LIO-SAM nodes collide / crash on drone2 launch | Use `namespace:=droneN` in launch command, not `PushRosNamespace` wrapper |
| Vision pose `Subscription count: 0` | MAVROS namespace mismatch — add remap to `lio_mavros_bridge.py` (LAUNCH Terminal 14) |
| `use_sim_time` not taking effect | Check for duplicate `/**:` blocks or duplicate `ros__parameters:` keys in YAML |
| "Not enough features" → drift | Lower `edgeFeatureMinValidNum` to 2, `edgeThreshold` to 0.5 |
| MAVROS `connected: false` persists | Run `output add 127.0.0.1:14551` (drone 1) or `14561` (drone 2) in the correct MAVProxy console |
| MAVROS drone 2 `detected remote address 1.1` | Both SITL instances default to MAV_SYSID=1. Run `param set MAV_SYSID 2` and `param save` in drone 2's MAVProxy |
| Sweep runner reports zombie nodes | `pkill -9 -f "swarm_mission.lidar_manipulator"` and same for logger; then `ros2 daemon stop; sleep 1; ros2 daemon start` |
| MAVROS `Time jump detected` / EKF3 lane switches | Cosmetic under WSL2 clock irregularities; not blocking unless `/clock` rate drops below ~100 Hz |
| MAVROS IMU rate 1.9 Hz vs Gazebo IMU 470 Hz | Expected — LIO-SAM uses `/droneN/imu/data` (Gazebo sensor direct), not `/mavros/imu/data` |
| Gazebo viewport empty / RTF stuck near zero | WSL2 graphics passthrough degraded. From Windows PowerShell: `wsl --shutdown`, then reopen WSL and relaunch the stack |
| `git push` rejected (non-fast-forward) | Repo's auto-init commit conflicts with local. Force push (`git push -f`) if local is authoritative, or `git pull --allow-unrelated-histories` to merge |
| Empty folders after `package_submission.sh` run | Don't run with `sudo` — `$HOME` resolves to `/root` and source paths fail. Run as the regular user with destination in your home directory |
