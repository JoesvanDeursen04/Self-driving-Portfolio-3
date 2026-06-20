#!/usr/bin/env python3
"""
apriltag_localizer.py – AprilTag-based node identification.

Each intersection in the maze has a unique AprilTag.
When the camera detects a tag, the robot knows exactly which node it is at.

Subscribed topics
-----------------
/<veh>/apriltag_detector_node/detections  (duckietown_msgs/AprilTagDetectionArray)

AprilTag → node mapping
-----------------------
Defined via the ~apriltag_map ROS parameter (YAML dict) or the
DEFAULT_TAG_MAP constant below.
"""

import rospy
from typing import Optional

try:
    from duckietown_msgs.msg import AprilTagDetectionArray
    _HAS_DT_MSGS = True
except ImportError:
    _HAS_DT_MSGS = False

# Default mapping: AprilTag ID -> node name in the maze graph
# Update these IDs to match the physical tags placed at each intersection.
DEFAULT_TAG_MAP = {
    1:  'A',
    2:  'B',
    3:  'C',
    4:  'D',
    5:  'E',
    6:  'F',
    7:  'G',
    8:  'H',
    9:  'S',   # Start
    10: 'I',
    11: 'J',
    12: 'T',   # Goal
}

class AprilTagLocalizer:
    """
    Listens for AprilTag detections and maps them to maze node names.

    The most recently detected node is available via :attr:`last_known_node`.
    """

    def __init__(self, vehicle_name: str = 'duckiebot', tag_map: dict = None):
        self._tag_map: dict = tag_map or DEFAULT_TAG_MAP
        self.last_known_node: Optional[str] = None
        self._detection_callback = None   # optional external callback

        if _HAS_DT_MSGS:
            self._sub = rospy.Subscriber(
                f'/{vehicle_name}/apriltag_detector_node/detections',
                AprilTagDetectionArray,
                self._cb_detections,
                queue_size=5,
            )
        else:
            rospy.logwarn('[AprilTagLocalizer] duckietown_msgs not available – '
                          'AprilTag localisation disabled.')

    # ------------------------------------------------------------------
    def _cb_detections(self, msg) -> None:
        best_node = None
        best_z = float('inf')

        for detection in msg.detections:
            tag_id = detection.tag_id
            z_dist = self._extract_z_distance(detection)
            if z_dist is None:
                continue
            if tag_id in self._tag_map and z_dist < best_z:
                best_z = z_dist
                best_node = self._tag_map[tag_id]

        if best_node is not None:
            if best_node != self.last_known_node:
                rospy.loginfo(f'[AprilTagLocalizer] Node detected: {best_node}')
            self.last_known_node = best_node
            if callable(self._detection_callback):
                self._detection_callback(best_node)

    def _extract_z_distance(self, detection) -> Optional[float]:
        """
        Return camera-to-tag Z distance in metres, or None when unavailable.

        Different AprilTag message definitions expose pose fields slightly
        differently, so we probe common layouts defensively.
        """
        try:
            z_val = float(detection.transform.translation.z)
            return z_val if z_val > 0.0 else None
        except (AttributeError, TypeError, ValueError):
            pass

        try:
            z_val = float(detection.pose.pose.position.z)
            return z_val if z_val > 0.0 else None
        except (AttributeError, TypeError, ValueError):
            pass

        try:
            z_val = float(detection.pose.position.z)
            return z_val if z_val > 0.0 else None
        except (AttributeError, TypeError, ValueError):
            return None

    # ------------------------------------------------------------------
    def register_callback(self, fn) -> None:
        """Register a callable *fn(node_name)* called on every new detection."""
        self._detection_callback = fn
