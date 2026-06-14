#!/usr/bin/env python3
"""
maze_localizer_node.py – ROS node for Duckietown maze localisation.

Fuses three sources of information to maintain an estimate of the
robot's current position within the maze graph:

  1. AprilTag detections  – give absolute node identification (highest priority)
  2. Wheel odometry       – track distance between nodes (used as fallback)
  3. Path knowledge       – knowing the planned path constrains which node
                            can come next

Subscribed topics
-----------------
/maze/path              (std_msgs/String)    Planned path (JSON list of nodes).

Published topics
----------------
/maze/current_node      (std_msgs/String)    Name of the current node.
/maze/next_node         (std_msgs/String)    Name of the next node on the path.
/maze/remaining_path    (std_msgs/String)    JSON list of remaining nodes.

Parameters
----------
~vehicle_name       (str,   default 'duckiebot')
~tile_length_m      (float, default 0.585)    Physical tile size in metres.
~node_arrival_dist  (float, default 0.4)      Distance (m) threshold to
                                               declare a node reached via odometry.
"""

import json
import sys
import os
import rospy
from std_msgs.msg import String

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)

from odometry_localizer import OdometryLocalizer      # noqa: E402
from apriltag_localizer import AprilTagLocalizer       # noqa: E402

try:
    from duckietown.dtros import DTROS, NodeType
    _USE_DTROS = True
except ImportError:
    _USE_DTROS = False


class MazeLocalizerNode(DTROS if _USE_DTROS else object):
    """Fused localisation node for the Duckietown maze."""

    def __init__(self, node_name: str = 'maze_localizer_node'):
        if _USE_DTROS:
            super().__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        else:
            rospy.init_node(node_name)

        # Parameters
        self._vehicle = rospy.get_param('~vehicle_name', 'duckiebot')
        self._tile_length = rospy.get_param('~tile_length_m', 0.585)
        self._arrival_dist = rospy.get_param('~node_arrival_dist', 0.4)

        # State
        self._path: list = []
        self._path_index: int = 0   # index of current node in _path

        # Sub-systems
        self._odometry = OdometryLocalizer(self._vehicle)
        self._apriltag = AprilTagLocalizer(self._vehicle)
        self._apriltag.register_callback(self._on_apriltag_detected)

        # Publishers
        self._pub_current = rospy.Publisher(
            '/maze/current_node', String, queue_size=1, latch=True)
        self._pub_next = rospy.Publisher(
            '/maze/next_node', String, queue_size=1, latch=True)
        self._pub_remaining = rospy.Publisher(
            '/maze/remaining_path', String, queue_size=1, latch=True)

        # Subscriber
        rospy.Subscriber('/maze/path', String, self._cb_path, queue_size=1)

        # Timer: poll odometry at 10 Hz as a backup when no AprilTag is visible
        self._timer = rospy.Timer(rospy.Duration(0.1), self._odometry_tick)

        rospy.loginfo('[Localizer] Node started.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _cb_path(self, msg: String) -> None:
        self._path = json.loads(msg.data)
        self._path_index = 0
        rospy.loginfo(f'[Localizer] Received path: {self._path}')
        self._publish_state()

    def _on_apriltag_detected(self, node_name: str) -> None:
        """High-priority update: robot is at *node_name* (AprilTag detected)."""
        if not self._path:
            return
        if node_name in self._path:
            new_index = self._path.index(node_name)
            if new_index != self._path_index:
                rospy.loginfo(
                    f'[Localizer] AprilTag: moved from '
                    f'{self._path[self._path_index]} to {node_name}')
                self._path_index = new_index
                self._odometry.reset()
                self._publish_state()

    def _odometry_tick(self, _event) -> None:
        """
        Fallback localisation: advance to the next node when the odometry
        reports that the robot has travelled one tile length.
        """
        if not self._path:
            return
        if self._path_index >= len(self._path) - 1:
            return  # already at goal

        dist = self._odometry.distance_since_reset()
        if dist >= self._arrival_dist:
            self._path_index = min(self._path_index + 1, len(self._path) - 1)
            self._odometry.reset()
            rospy.loginfo(
                f'[Localizer] Odometry: advanced to {self._path[self._path_index]}')
            self._publish_state()

    # ------------------------------------------------------------------
    def _publish_state(self) -> None:
        if not self._path:
            return
        current = self._path[self._path_index]
        remaining = self._path[self._path_index:]
        next_node = self._path[self._path_index + 1] \
            if self._path_index + 1 < len(self._path) else current

        self._pub_current.publish(String(data=current))
        self._pub_next.publish(String(data=next_node))
        self._pub_remaining.publish(String(data=json.dumps(remaining)))


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    node = MazeLocalizerNode()
    rospy.spin()
