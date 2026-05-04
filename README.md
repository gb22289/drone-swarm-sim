# \# GPS-Denied Drone Swarm — Security Analysis

# 

# Code accompanying the MEng dissertation \*\*"Breaking Trust in the Swarm: A Security Analysis of ROS2-Based Cooperative Drone Systems"\*\* (University of Bristol, COMSM0052).

# 

# This repository contains the attack and defence nodes, configuration, experiment scaffolding, and raw data used to characterise vulnerabilities and unintended defences in a ROS2-based GPS-denied drone swarm. The simulation stack comprises Gazebo Harmonic, ArduCopter SITL, MAVROS, and LIO-SAM.

# 

# \---

# 

# \## Repository Layout

# 

# | Path | Purpose |

# |------|---------|

# | `swarm\_mission/` | ROS2 package — all attack and defence nodes, plus mission/coordination logic |

# | `lio\_mavros\_bridge.py` | 20 Hz republisher bridging LIO-SAM odometry to MAVROS `vision\_pose` |

# | `lio\_sam\_patches/` | Diffs to apply over upstream LIO-SAM (clock-jump dt-guards + KDTree bounds checks) |

# | `lio\_sam\_config/` | LIO-SAM YAML configurations for drone1 and drone2 |

# | `configs/` | ros\_gz\_bridge, MAVROS, and Gazebo SDF configurations |

# | `scripts/` | Experiment automation, pre-flight validation, plotting, audit |

# | `data/` | Raw experimental data (CSVs) |

# 

# \---

# 

# \## Attack and Defence Nodes

# 

# All nodes live in `swarm\_mission/swarm\_mission/` and follow a consistent CLI:

# 

# ```bash

# ros2 run swarm\_mission <node\_name> --ros-args -p <param>:=<value>

# ```

# 

# | Node | Layer | Purpose |

# |------|-------|---------|

# | `network\_attacker` | 1 | Coordination-layer attacker (modes: `coverage\_spoof`, `phantom\_drone`, `selective\_denial`) |

# | `lidar\_manipulator` | 2 | Gradual scan-rotation attack on the LiDAR topic |

# | `pointcloud\_injector` | 2 | Phantom-wall injection — negative-result attack |

# | `imu\_injector` | 2 | Reused-timestamp IMU injection — blocked by dt-guard |

# | `imu\_injector\_v2` | 2 | Advancing-timestamp IMU injection — bypasses dt-guard, blocked by upstream LIO-SAM mechanisms |

# | `qos\_poisoner` | 2 | DDS QoS-incompatibility attack — blocked by DDS protocol-level isolation |

# | `slam\_failure\_logger` | — | Detector — sustained-span pattern matching on LIO-SAM rosout for TTF measurement |

# | `lidar\_deskew\_shim` | — | Per-point timestamp synthesis to enable LIO-SAM deskew on Gazebo's `gpu\_lidar` |

# | `waypoint\_navigator` | — | Mission node (honest or Byzantine); each drone runs an instance |

# | `ground\_truth\_logger` | — | Independent measurement of mission outcomes for Layer 1 evaluation |

# 

# \---

# 

# \## Building

# 

# Assumes Ubuntu 22.04 LTS with ROS2 Humble, Gazebo Harmonic, and ArduCopter SITL already installed. Full installation procedure is in dissertation Chapter 3.2.

# 

# ```bash

# \# 1. Set up ROS2 workspace

# mkdir -p \~/ros2\_ws/src

# cd \~/ros2\_ws/src

# 

# \# 2. Clone upstream dependencies

# git clone https://github.com/TixiaoShan/LIO-SAM.git -b ros2

# git clone https://github.com/gazebosim/ros\_gz.git -b humble

# \# (ArduPilot + ardupilot\_gazebo into \~/sim/ separately)

# 

# \# 3. Apply LIO-SAM patches

# cd \~/ros2\_ws/src/LIO-SAM

# patch -p1 < /path/to/this/repo/lio\_sam\_patches/imu\_preintegration\_dt\_guards.patch

# patch -p1 < /path/to/this/repo/lio\_sam\_patches/mapoptmization\_kdtree\_check.patch

# 

# \# 4. Copy submission contents into the workspace

# cp -r /path/to/this/repo/swarm\_mission \~/ros2\_ws/src/

# cp /path/to/this/repo/lio\_sam\_config/\*.yaml \~/ros2\_ws/src/LIO-SAM/config/

# cp /path/to/this/repo/lio\_mavros\_bridge.py \~/ros2\_ws/src/

# 

# \# 5. Build

# cd \~/ros2\_ws

# export GZ\_VERSION=harmonic

# colcon build

# source install/setup.bash

# ```

# 

# \---

# 

# \## Running

# 

# Launch order matters. `/clock` from Gazebo must be flowing before any node with `use\_sim\_time:=true` starts, and the `lidar\_deskew\_shim` must be running before LIO-SAM. The dissertation Section 3.2 documents the full sequence; the abridged single-drone version:

# 

# ```bash

# \# Terminal 1 — Gazebo

# gz sim worlds/iris\_warehouse.sdf -r

# 

# \# Terminal 2 — ros\_gz bridge (publishes /clock)

# ros2 run ros\_gz\_bridge parameter\_bridge --ros-args \\

# &#x20; -p config\_file:=/path/to/configs/bridge.yaml

# 

# \# Terminal 3 — ArduCopter SITL

# sim\_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console -I0

# 

# \# Terminal 4 — MAVROS

# 

# \# Terminal 5 — LiDAR deskew shim (must precede LIO-SAM)

# ros2 run swarm\_mission lidar\_deskew\_shim --ros-args \\

# &#x20; -p input\_topic:=/drone1/lidar/points \\

# &#x20; -p output\_topic:=/drone1/lidar/points\_timed \\

# &#x20; -p scan\_period:=0.1

# 

# \# Terminal 6 — LIO-SAM

# 

# \# Terminal 7 — LIO-SAM → MAVROS bridge

# python3 \~/ros2\_ws/src/lio\_mavros\_bridge.py --ros-args \\

# &#x20; -p drone\_ns:=drone1 \\

# &#x20; -r /drone1/mavros/vision\_pose/pose:=/mavros/vision\_pose/pose

# ```

# 

# \### Pre-flight check

# 

# Before any trial, verify stack health:

# 

# ```bash

# ./scripts/preflight\_check.sh

# ```

# 

# Validates IMU and LiDAR rates, vision\_pose flow, drone hover position, LIO-SAM odometry magnitude, and the absence of zombie processes from earlier trials.

# 

# \### Running an attack — examples

# 

# ```bash

# \# Layer 1 — coordination spoof (other modes: phantom\_drone, selective\_denial)

# python3 \~/ros2\_ws/src/swarm\_mission/swarm\_mission/network\_attacker.py --ros-args \\

# &#x20; -p attack:=coverage\_spoof \\

# &#x20; -p target\_drone:=drone2 \\

# &#x20; -p discovery\_time:=5.0 \\

# &#x20; -p spoof\_delay:=0.5

# 

# \# Layer 2 — gradual scan rotation, with TTF logged via the sweep runner

# ./scripts/sweep\_runner.sh 1.0 1   # 1.0 °/s, run number 1

# 

# \# Layer 2 — IMU injection with advancing timestamps (defeats dt-guard)

# ros2 run swarm\_mission imu\_injector\_v2 --ros-args \\

# &#x20; -p target\_drone:=drone1 \\

# &#x20; -p mode:=bias \\

# &#x20; -p accel\_bias\_x:=5.0 \\

# &#x20; -p baseline\_duration:=15.0 \\

# &#x20; -p attack\_duration:=45.0 \\

# &#x20; -p recovery\_duration:=15.0

# ```

# 

# \---

# 

# \## Data Files

# 

# | File | Contents |

# |------|----------|

# | `data/scan\_sweep\_summary.csv` | TTF measurements for the rotation-rate sweep (Section 3.5.3), pre and post KDTree-patch |

# | `data/layer1\_mission\_results.csv` | Per-waypoint ground-truth measurements for Layer 1 attacks (Section 3.4) |

# | `data/imu\_injection\_v2\_metrics\_\*.csv` | LIO-SAM and drone drift trajectories during advancing-stamp IMU injection (Section 3.5.5) |

# 

# \---

# 

# \## Reproducing the Figures

# 

# The dissertation's primary figure (rotation-rate vs. time-to-failure) is regenerated from `data/scan\_sweep\_summary.csv`:

# 

# ```bash

# python3 scripts/plot\_rotation\_sweep.py \\

# &#x20; --csv data/scan\_sweep\_summary.csv \\

# &#x20; --out fig\_rotation\_sweep \\

# &#x20; --test-window-s 180

# ```

# 

# Output: `fig\_rotation\_sweep.pdf` and `fig\_rotation\_sweep.png`.

# 

# \---

# 

# \## LIO-SAM Patches

# 

# Two patches in `lio\_sam\_patches/` modify upstream LIO-SAM:

# 

# 1\. \*\*`imu\_preintegration\_dt\_guards.patch`\*\* — Adds `dt > 0` and `dt < 0.02` guards to the three IMU-integration paths in `imuPreintegration.cpp`. Required for stable operation under the multi-SITL / single-Gazebo clock-synchronisation regime documented in dissertation Section 3.2.4. Without these guards, `gtsam::IndeterminantLinearSystemException` crashes occur on clock jumps.

# 

# 2\. \*\*`mapoptmization\_kdtree\_check.patch`\*\* — Adds bounds checks at four `setInputCloud` call sites in `mapOptmization.cpp` (scan-to-map hot path, global-map visualisation, loop-closure detection, surrounding-keyframe extraction). Eliminates the dominant denial-of-process crashes observed under the scan-rotation attack at low rotation rates. Discussed as a partial defence in Section 3.5.3 — closes \~89% of crashes; one ICP-internal site remains uncovered.

# 

# Both patches are also documented inline in their respective `.patch` files.

# 

# \---

# 

# \## Methodology Notes

# 

# \- \*\*Trials use GPS-hold position\*\* (`VISO\_TYPE=0`, `GPS1\_TYPE=1`) to isolate the attack's effect on the SLAM pipeline from feedback dynamics in ArduCopter's EKF3. The attacker still publishes to `/droneN/lidar/points` and LIO-SAM still processes those scans; only the EKF3 fusion of vision\_pose is bypassed.

# \- \*\*Per-trial cold restarts\*\* are recommended — LIO-SAM's factor graph and accumulated map state can carry corruption from prior trials. The `state\_audit.sh` script catalogues lingering state if cleanups appear incomplete.

# \- \*\*Real-time factor (RTF)\*\* of approximately 47% on the development hardware (WSL2 / consumer laptop) yields a wall-clock LiDAR rate of \~0.94 Hz against a 2 Hz sim-time configuration. Higher-RTF deployments should reproduce qualitatively but with shorter time-to-failure.

# 

# \---

# 

# \## Citation

# 

# If you use this code, please cite:

# 

# > Mohai, P. \*Breaking Trust in the Swarm: A Security Analysis of ROS2-Based Cooperative Drone Systems.\* MEng dissertation, University of Bristol, 2026.

# 

# \---

# 

# \## License

# 

# Original code is released under the MIT License (see `LICENSE`). Patches to LIO-SAM are derivative works of LIO-SAM and inherit its BSD-3-Clause license.

