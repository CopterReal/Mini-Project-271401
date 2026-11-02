"""Smoothed steering decisions for a robot controlled using W/A/S/D/X.

This filters visual jitter and adds hysteresis. It does not vary motor speed:
proportional motor steering requires corresponding ESP32 firmware support.
"""
from dataclasses import dataclass
import time


@dataclass
class Decision:
    cmd: str
    label: str
    error: float = 0.0


class PIDNode:
    def __init__(self, deadband=0.30, stop_coverage=0.12,
                 turn_enter=None, turn_exit=None, error_alpha=0.25,
                 switch_frames=3):
        self.deadband = deadband
        self.stop_coverage = stop_coverage
        self.turn_enter = deadband if turn_enter is None else turn_enter
        self.turn_exit = max(0.0, self.turn_enter - 0.10) if turn_exit is None else turn_exit
        if not 0 <= self.turn_exit < self.turn_enter <= 1:
            raise ValueError('Require 0 <= turn_exit < turn_enter <= 1')
        if not 0 < error_alpha <= 1:
            raise ValueError('Require 0 < error_alpha <= 1')
        self.error_alpha = error_alpha
        self.switch_frames = max(1, int(switch_frames))
        self.filtered_error = None
        self.turn = 'X'
        self.pending = None
        self.pending_count = 0
        self.last_valid = None

    def reset(self):
        self.filtered_error = None
        self.turn = 'X'
        self.pending = None
        self.pending_count = 0
        self.last_valid = None

    def decide(self, data, forward_allowed=False):
        center = data.get('center')
        w = data.get('width', 0)
        coverage = data.get('coverage', 0.0)

        if center is None or w <= 0:
            self.reset()
            return Decision('X', 'NO TARGET — STOP')

        raw_error = max(-1.0, min(1.0, (center[0] - w / 2) / (w / 2)))
        if coverage >= self.stop_coverage:
            self.reset()
            return Decision('X', 'TARGET LARGE — STOP', raw_error)

        now = time.monotonic()
        if self.last_valid is None or now - self.last_valid > 0.5:
            self.filtered_error = raw_error
            self.turn = 'X'
            self.pending = None
            self.pending_count = 0
        else:
            self.filtered_error += self.error_alpha * (raw_error - self.filtered_error)
        self.last_valid = now
        error = self.filtered_error

        # Hysteresis: once turning, continue until nearly centered.
        if self.turn == 'A':
            desired = 'A' if error < -self.turn_exit else ('D' if error > self.turn_enter else 'X')
        elif self.turn == 'D':
            desired = 'D' if error > self.turn_exit else ('A' if error < -self.turn_enter else 'X')
        else:
            desired = 'A' if error < -self.turn_enter else ('D' if error > self.turn_enter else 'X')

        # Require multiple detections before starting/changing a turn.
        # Stopping a turn is immediate, to reduce overshoot.
        if desired == 'X':
            self.turn = 'X'
            self.pending = None
            self.pending_count = 0
        elif desired == self.turn:
            self.pending = None
            self.pending_count = 0
        else:
            if desired == self.pending:
                self.pending_count += 1
            else:
                self.pending = desired
                self.pending_count = 1
            if self.pending_count >= self.switch_frames:
                self.turn = desired
                self.pending = None
                self.pending_count = 0
            elif self.turn != 'X' and desired != self.turn:
                self.turn = 'X'  # stop rather than rapidly reverse direction

        if self.turn == 'A':
            return Decision('A', 'TURN LEFT (SMOOTH)', error)
        if self.turn == 'D':
            return Decision('D', 'TURN RIGHT (SMOOTH)', error)
        if self.pending is not None:
            return Decision('X', 'ALIGNING — STABILIZING', error)
        if forward_allowed:
            return Decision('W', 'FORWARD', error)
        return Decision('X', 'CENTERED — FORWARD LOCKED', error)
