"""고규범 신규 로직 (2026-10-21): vehicle-local threshold and remote-input watchdog.

Only this process publishes its command topic. stdin JSON lines come from the
operator-side SSH client; no /dev/input device or ROS network on the operator PC.
"""
import json
import math
import queue
import signal
import sys
import threading
import time
import uuid

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray, String
from vehicle_recorder.msg import CommandEvent, StopStatus

from manual_control.brake_test_logic import BrakeTrial


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


class BrakeTestNode(Node):
    def __init__(self, read_stdin=True):
        super().__init__('brake_test')
        defaults = dict(target_kph=10., remote_timeout_s=.5, gnss_timeout_s=.3,
                        throttle_step=.005, max_throttle=1.0, cmd_topic='/brake_test/preview_cmd',
                        gnss_topic='/gnss/odom', stop_status_topic='/vehicle/stop_status',
                        map_frame='map', base_frame='base_link', max_speed_variance=1.,
                        max_position_std_m=3.)
        self.settings = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.trial = BrakeTrial(*(self.settings[k] for k in (
            'target_kph', 'remote_timeout_s', 'gnss_timeout_s', 'throttle_step', 'max_throttle')))
        for key in ('max_speed_variance', 'max_position_std_m'):
            if not math.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError(key + ' must be finite and positive')
        self.commands = self.create_publisher(Float32MultiArray, self.settings['cmd_topic'], 1)
        self.events = self.create_publisher(CommandEvent, '/vehicle/command_events', 100)
        self.status = self.create_publisher(String, '/vehicle/brake_test',
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, self.settings['gnss_topic'], self.odometry, qos_profile_sensor_data)
        self.create_subscription(StopStatus, self.settings['stop_status_topic'], self.stop_status, 10)
        self.stop_message, self.stop_received = None, 0.
        self.stop_command_ns = 0
        self.session = str(uuid.uuid4())
        self.inbox = queue.Queue(maxsize=64)
        self.terminal_output = queue.Queue(maxsize=64)
        self.input_failed = threading.Event()
        self.external_stop = threading.Event()
        self.shutdown_requested = False
        self.last_status_at = 0.
        self.completion_emitted = False
        self.create_timer(.01, self.tick)  # 100 Hz command heartbeat; bounded work per tick.
        threading.Thread(target=self.write_output, daemon=True).start()
        if read_stdin:
            threading.Thread(target=self.read_input, daemon=True).start()

    def write_output(self):
        # SSH/stdout may stall. Never put terminal writes on the ROS control path.
        while True:
            encoded = self.terminal_output.get()
            try:
                print(encoded, flush=True)
            except (BrokenPipeError, OSError):
                self.input_failed.set()
                return

    def read_input(self):
        try:
            while True:
                line = sys.stdin.readline(4097)
                if not line or len(line) > 4096 or not line.endswith('\n'):
                    break
                payload = json.loads(line)
                if (not isinstance(payload, dict) or payload.get('action') not in
                    ('heartbeat', 'arm', 'up', 'down', 'left', 'right', 'center', 'stop', 'quit')):
                    break
                self.inbox.put_nowait((payload, time.monotonic()))
        except (ValueError, OSError, queue.Full):
            pass
        self.input_failed.set()

    def stopped(self, now):
        m = self.stop_message
        age = (self.get_clock().now().nanoseconds - ns(m.header.stamp))/1e9 if m else math.inf
        return bool(m and 0 <= now-self.stop_received <= .3 and -.05 <= age <= .3
                    and m.command_topic == self.commands.topic_name and m.command_active
                    and m.command_fresh and m.speed_valid and m.stopped)

    def stop_status(self, message):
        if message.command_topic == self.commands.topic_name:
            self.stop_message, self.stop_received = message, time.monotonic()

    def odometry(self, message):
        speed = message.twist.twist.linear.x
        variance = message.twist.covariance[0]
        valid = (message.header.frame_id == self.settings['map_frame'] and
                 message.child_frame_id == self.settings['base_frame'] and
                 math.isfinite(variance) and 0 <= variance <= self.settings['max_speed_variance'])
        x, y = message.pose.pose.position.x, message.pose.pose.position.y
        vx, vy = message.pose.covariance[0], message.pose.covariance[7]
        xy = [x, y] if (all(math.isfinite(v) for v in (x, y, vx, vy)) and min(vx, vy) >= 0
                       and vx+vy <= self.settings['max_position_std_m']**2) else None
        self.trial.sample(speed, ns(message.header.stamp), self.get_clock().now().nanoseconds,
                          time.monotonic(), valid, xy)
        # Threshold handling is local and publishes immediately on this GNSS callback.
        if self.trial.events:
            self.publish_command()

    def emit(self, event, **extra):
        row = dict(schema=1, session=self.session, ros_ns=self.get_clock().now().nanoseconds,
                   command_topic=self.commands.topic_name, **event, **extra)
        encoded = json.dumps(row, allow_nan=False, separators=(',', ':'))
        self.status.publish(String(data=encoded))
        try:
            self.terminal_output.put_nowait(encoded)
        except queue.Full:
            # ROS recording remains independent; stalled terminal triggers a stop.
            self.trial.stop('terminal_output_stalled')

    def publish_command(self):
        steering, speed, brake = self.trial.output()
        event = CommandEvent(command_topic=self.commands.topic_name, stage=CommandEvent.COMMAND,
                             steering_raw=steering, speed_raw=speed, brake=bool(brake))
        event.header.frame_id = 'base_link'
        event.header.stamp = self.get_clock().now().to_msg()
        self.commands.publish(Float32MultiArray(data=[steering, speed, brake]))
        self.events.publish(event)
        pending, self.trial.events = self.trial.events, []
        for item in pending:
            if item['event'] == 'brake':
                self.stop_command_ns = ns(event.header.stamp)
            if item['event'] == 'armed':
                self.stop_command_ns, self.completion_emitted = 0, False
            self.emit(item, command_ns=ns(event.header.stamp), steering_raw=steering)

    def tick(self):
        now = time.monotonic()
        if self.input_failed.is_set() or self.external_stop.is_set():
            self.trial.stop('remote_disconnected' if self.input_failed.is_set() else 'process_signal')
            self.trial.quit_requested = True
        for _ in range(64):
            try:
                payload, received = self.inbox.get_nowait()
            except queue.Empty:
                break
            # EOF/signal/overload wins over queued throttle/arm commands.
            if not self.trial.quit_requested and 0 <= now-received <= self.trial.timeout_s:
                self.trial.check(now)
                can_arm = self.stopped(now) and self.count_publishers(self.commands.topic_name) == 1
                self.trial.command(payload['action'], payload.get('seq'), now, can_arm, received_at=received)
                if self.trial.events:
                    self.publish_command()
        self.trial.check(now)
        if (self.trial.state == 'BRAKING' and self.stopped(now)
                and ns(self.stop_message.command_stamp) == self.stop_command_ns):
            m = self.stop_message
            self.trial.confirmed()
            distance = (math.hypot(m.stop_x_m-m.command_x_m, m.stop_y_m-m.command_y_m)
                        if m.command_position_valid and m.stop_position_valid else None)
            self.emit(dict(event='result', trial=self.trial.trial, target_kph=self.trial.target_kph,
                reason=self.trial.reason, command_ns=ns(m.command_stamp), transmit_ns=ns(m.transmit_stamp),
                stopped_since_ns=ns(m.stopped_since), confirmed_at_ns=ns(m.confirmed_at),
                command_xy=[m.command_x_m, m.command_y_m] if m.command_position_valid else None,
                stop_xy=[m.stop_x_m, m.stop_y_m] if m.stop_position_valid else None,
                command_distance_m=distance, transmit_distance_m=m.distance_m if m.distance_valid else None,
                stop_sample_ns=ns(m.sample_stamp)))
            self.completion_emitted = True
        self.publish_command()
        if now-self.last_status_at >= .2:
            if self.count_publishers(self.commands.topic_name) > 1:
                self.trial.stop('conflicting_command_publisher')
                self.publish_command()
            self.emit(dict(event='status', trial=self.trial.trial, state=self.trial.state, reason=self.trial.reason,
                target_kph=self.trial.target_kph, speed_kph=abs(self.trial.speed_mps)*3.6 if self.trial.speed_mps is not None else None,
                gnss_fresh=self.trial.gnss_fresh(now), remote_fresh=self.trial.remote_fresh(now),
                can_arm=(self.trial.state in ('WAITING', 'COMPLETE') and self.stopped(now)
                         and self.trial.gnss_fresh(now) and self.count_publishers(self.commands.topic_name) == 1),
                steering_raw=self.trial.steering, throttle_raw=self.trial.throttle,
                settings=self.settings))
            self.last_status_at = now
        # Never abandon an unconfirmed stop on SSH EOF, Ctrl+C or GNSS failure.
        if (self.trial.quit_requested and self.stopped(now) and
                self.trial.state in ('WAITING', 'COMPLETE')):
            self.shutdown_requested = True


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BrakeTestNode()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: node.external_stop.set())
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
