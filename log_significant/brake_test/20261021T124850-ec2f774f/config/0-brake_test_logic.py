"""고규범 신규 로직 (2026-10-21): remote brake trial; no ROS or file I/O."""
import math


class BrakeTrial:
    def __init__(self, target_kph, timeout_s=.5, gnss_timeout_s=.3,
                 throttle_step=.005, max_throttle=1.0):
        values = (target_kph, timeout_s, gnss_timeout_s, throttle_step, max_throttle)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('parameters must be finite and positive')
        if target_kph > 50 or max_throttle > 1 or throttle_step > max_throttle:
            raise ValueError('target <= 50 km/h, throttle_step <= max_throttle <= 1 required')
        self.target_kph, self.timeout_s, self.gnss_timeout_s = target_kph, timeout_s, gnss_timeout_s
        self.throttle_step, self.max_throttle = throttle_step, max_throttle
        self.state, self.reason = 'WAITING', 'not_armed'
        self.steering = self.throttle = 0.
        self.last_remote = self.last_gnss = None
        self.sample_ns, self.speed_mps, self.xy = 0, None, None
        self.gnss_valid = False
        self.trial = 0
        self.events = []
        self.sequence = -1
        self.quit_requested = False

    def remote_fresh(self, now):
        return self.last_remote is not None and 0 <= now - self.last_remote <= self.timeout_s

    def gnss_fresh(self, now):
        return (self.gnss_valid and self.last_gnss is not None and
                0 <= now - self.last_gnss <= self.gnss_timeout_s)

    def sample(self, speed, source_ns, ros_ns, now, valid=True, xy=None):
        if not valid or not math.isfinite(speed) or source_ns <= 0 or (source_ns-ros_ns)/1e9 > .05:
            self.gnss_valid = False
            self.stop('gnss_invalid')
            return
        # Replayed/reordered samples never extend the freshness deadline.
        if source_ns <= self.sample_ns:
            return
        if (ros_ns-source_ns)/1e9 > self.gnss_timeout_s:
            self.gnss_valid = False
            self.stop('gnss_invalid')
            return
        self.speed_mps, self.sample_ns, self.last_gnss = speed, source_ns, now
        self.xy, self.gnss_valid = xy, True
        if self.state == 'ACCELERATING' and abs(speed) * 3.6 >= self.target_kph:
            self.stop('target_reached')

    def check(self, now):
        if self.state == 'ACCELERATING':
            if not self.remote_fresh(now):
                self.stop('remote_timeout')
            elif not self.gnss_fresh(now):
                self.stop('gnss_timeout')

    def stop(self, reason):
        if self.state == 'ACCELERATING':
            self.state, self.reason, self.throttle = 'BRAKING', reason, 0.
            self.events.append(dict(event='brake', trial=self.trial, reason=reason,
                                    target_kph=self.target_kph, observed_speed_kph=abs(self.speed_mps)*3.6,
                                    trigger_sample_ns=self.sample_ns, trigger_xy=self.xy))

    def command(self, action, sequence, now, stopped=False, received_at=None):
        if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence <= self.sequence:
            return
        self.sequence = sequence
        self.last_remote = now if received_at is None else received_at
        if action in ('stop', 'quit'):
            self.stop('operator_stop' if action == 'stop' else 'operator_quit')
            self.quit_requested |= action == 'quit'
        elif action == 'arm':
            if (self.state in ('WAITING', 'COMPLETE') and stopped and self.gnss_fresh(now)
                    and abs(self.speed_mps) <= .1 and not self.quit_requested):
                self.trial += 1
                self.state, self.reason = 'ACCELERATING', 'manual_acceleration'
                self.steering = self.throttle = 0.
                self.events.append(dict(event='armed', trial=self.trial, target_kph=self.target_kph))
        elif self.state == 'ACCELERATING':
            if action == 'up':
                self.throttle = min(self.max_throttle, self.throttle+self.throttle_step)
            elif action == 'down':
                self.throttle = max(0., self.throttle-self.throttle_step)
            elif action in ('left', 'right'):
                self.steering = max(-1., min(1., self.steering + (-.01 if action == 'left' else .01)))
            elif action == 'center':
                self.steering = 0.

    def confirmed(self):
        if self.state == 'BRAKING':
            self.state = 'COMPLETE'

    def output(self):
        braking = self.state != 'ACCELERATING'
        return self.steering, 0. if braking else self.throttle, float(braking)
