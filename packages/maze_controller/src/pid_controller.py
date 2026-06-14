#!/usr/bin/env python3
"""
pid_controller.py – Generic discrete PID controller.

Usage
-----
pid = PIDController(kp=1.0, ki=0.01, kd=0.1, output_limits=(-1.0, 1.0))
output = pid.compute(error, dt)
pid.reset()
"""

import math


class PIDController:
    """
    Discrete-time PID controller with anti-windup clamping.

    Parameters
    ----------
    kp : float  Proportional gain
    ki : float  Integral gain
    kd : float  Derivative gain
    output_limits : (float, float)
        (min, max) clamp for the output signal.
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_limits: tuple = (-1.0, 1.0),
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._out_min, self._out_max = output_limits

        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._first_call: bool = True

    # ------------------------------------------------------------------
    def compute(self, error: float, dt: float) -> float:
        """
        Compute the PID output for the given *error* over time step *dt*.

        Parameters
        ----------
        error : float
            Signed error signal (setpoint − measured).
        dt : float
            Time since last call in seconds. Must be > 0.

        Returns
        -------
        float
            Controller output, clamped to *output_limits*.
        """
        if dt <= 0.0:
            return 0.0

        # Proportional term
        p = self.kp * error

        # Integral term (with anti-windup via output clamping)
        self._integral += error * dt
        i = self.ki * self._integral

        # Derivative term (backward difference; skip on first call)
        if self._first_call:
            d = 0.0
            self._first_call = False
        else:
            d = self.kd * (error - self._prev_error) / dt

        self._prev_error = error

        output = p + i + d

        # Anti-windup: stop integrating when saturated
        if output > self._out_max:
            output = self._out_max
            self._integral -= error * dt   # undo last step
        elif output < self._out_min:
            output = self._out_min
            self._integral -= error * dt

        return output

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Reset integral accumulator and derivative memory."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True
