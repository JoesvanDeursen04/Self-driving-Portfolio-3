#!/usr/bin/env python3
"""
odometry_localizer.py – Wheel-encoder based distance tracking.

Subscribes to wheel encoder ticks and integrates them to estimate
how far the robot has travelled since the last known node.

Subscribed topics
-----------------
/<veh>/left_wheel_encoder_node/tick   (duckietown_msgs/WheelEncoderStamped)
/<veh>/right_wheel_encoder_node/tick  (duckietown_msgs/WheelEncoderStamped)
"""

import math
import rospy

try:
    from duckietown_msgs.msg import WheelEncoderStamped
    _HAS_DT_MSGS = True
except ImportError:
    _HAS_DT_MSGS = False


# Duckiebot hardware constants (DB21 / DB18)
WHEEL_RADIUS_M = 0.0318      # metres
WHEEL_BASE_M = 0.102         # metres (distance between wheels)
TICKS_PER_REVOLUTION = 135   # encoder ticks per full wheel rotation


class OdometryLocalizer:
    """
    Tracks accumulated distance and heading change using wheel encoders.

    Call :meth:`distance_since_reset` to query the distance travelled
    since the last :meth:`reset` call.
    """

    def __init__(self, vehicle_name: str = 'duckiebot'):
        self._vehicle = vehicle_name
        self._left_ticks: int = 0
        self._right_ticks: int = 0
        self._prev_left: int = 0
        self._prev_right: int = 0
        self._distance: float = 0.0   # metres since last reset
        self._heading: float = 0.0    # radians (cumulative)

        if _HAS_DT_MSGS:
            self._sub_left = rospy.Subscriber(
                f'/{vehicle_name}/left_wheel_encoder_node/tick',
                WheelEncoderStamped,
                self._cb_left,
                queue_size=10,
            )
            self._sub_right = rospy.Subscriber(
                f'/{vehicle_name}/right_wheel_encoder_node/tick',
                WheelEncoderStamped,
                self._cb_right,
                queue_size=10,
            )
        else:
            rospy.logwarn('[OdometryLocalizer] duckietown_msgs not available – '
                          'odometry disabled.')

    # ------------------------------------------------------------------
    # Encoder callbacks
    # ------------------------------------------------------------------
    def _cb_left(self, msg) -> None:
        delta = msg.data - self._prev_left
        self._prev_left = msg.data
        self._left_ticks += delta
        self._integrate(delta, 0)

    def _cb_right(self, msg) -> None:
        delta = msg.data - self._prev_right
        self._prev_right = msg.data
        self._right_ticks += delta
        self._integrate(0, delta)

    def _integrate(self, delta_left: int, delta_right: int) -> None:
        """Convert tick deltas to distance / heading change."""
        d_left = (delta_left / TICKS_PER_REVOLUTION) * (2 * math.pi * WHEEL_RADIUS_M)
        d_right = (delta_right / TICKS_PER_REVOLUTION) * (2 * math.pi * WHEEL_RADIUS_M)
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / WHEEL_BASE_M
        self._distance += abs(d_center)
        self._heading += d_theta

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------
    def distance_since_reset(self) -> float:
        """Return total distance (m) travelled since last reset."""
        return self._distance

    def heading_change(self) -> float:
        """Return accumulated heading change (radians) since last reset."""
        return self._heading

    def reset(self) -> None:
        """Reset distance and heading counters (call when a new node is reached)."""
        self._distance = 0.0
        self._heading = 0.0
