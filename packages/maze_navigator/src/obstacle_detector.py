#!/usr/bin/env python3
"""
obstacle_detector.py – Duckie obstacle detection from camera image.

Uses a YOLOv5/v8 ONNX model (best.onnx) to detect duckies in front of
the Duckiebot.  Falls back to HSV colour segmentation when the model file
is not found or OpenCV DNN is unavailable.

Subscribed topics
-----------------
/<veh>/camera_node/image/compressed  (sensor_msgs/CompressedImage)

The :attr:`obstacle_detected` flag is *True* when a duckie is detected
in the lower half of the image (i.e. close enough to warrant stopping).
"""

import os
import rospy
import numpy as np

try:
    import cv2
    from cv_bridge import CvBridge
    _HAS_CV = True
except ImportError:
    _HAS_CV = False

try:
    from sensor_msgs.msg import CompressedImage
    _HAS_SENSOR = True
except ImportError:
    _HAS_SENSOR = False

# ---------------------------------------------------------------------------
# ONNX model settings
# ---------------------------------------------------------------------------
# Path inside the Docker container – assets/ is mounted at /data/assets/
MODEL_PATH = os.environ.get(
    'DUCKIE_MODEL_PATH',
    '/data/assets/best.onnx',
)

# Input resolution expected by the model (must match training config)
MODEL_INPUT_SIZE = 640       # pixels (square)

# Detection thresholds
CONF_THRESHOLD = 0.40        # minimum objectness × class confidence
NMS_THRESHOLD = 0.45         # IoU threshold for Non-Maximum Suppression

# Only flag obstacles whose bounding-box bottom edge is in the lower fraction
# of the image (i.e. the duckie is physically close to the robot).
CLOSE_REGION_FRACTION = 0.50

# ---------------------------------------------------------------------------
# HSV fallback settings (used when ONNX model is unavailable)
# ---------------------------------------------------------------------------
DUCKIE_HSV_LOWER = np.array([20, 100, 100])
DUCKIE_HSV_UPPER = np.array([35, 255, 255])
MIN_BLOB_AREA = 1500


class ObstacleDetector:
    """
    Detects duckie obstacles using a YOLO ONNX model.

    Falls back to HSV colour segmentation if the model cannot be loaded.
    """

    def __init__(self, vehicle_name: str = 'duckiebot'):
        self.obstacle_detected: bool = False
        self._bridge = CvBridge() if _HAS_CV else None
        self._net = None

        # Try to load the ONNX model
        if _HAS_CV:
            self._net = self._load_model(MODEL_PATH)

        if _HAS_CV and _HAS_SENSOR:
            self._sub = rospy.Subscriber(
                f'/{vehicle_name}/camera_node/image/compressed',
                CompressedImage,
                self._cb_image,
                queue_size=1,
                buff_size=2**24,
            )
        else:
            rospy.logwarn('[ObstacleDetector] OpenCV or sensor_msgs not available.')

    # ------------------------------------------------------------------
    @staticmethod
    def _load_model(path: str):
        """Load the ONNX model via OpenCV DNN. Returns None on failure."""
        if not os.path.isfile(path):
            rospy.logwarn(f'[ObstacleDetector] Model not found at {path} – '
                          'falling back to HSV detection.')
            return None
        try:
            net = cv2.dnn.readNetFromONNX(path)
            rospy.loginfo(f'[ObstacleDetector] Loaded ONNX model: {path}')
            return net
        except Exception as exc:
            rospy.logwarn(f'[ObstacleDetector] Failed to load model: {exc} – '
                          'falling back to HSV detection.')
            return None

    # ------------------------------------------------------------------
    def _cb_image(self, msg) -> None:
        try:
            img = self._bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:
            rospy.logwarn_throttle(5.0, f'[ObstacleDetector] Image decode: {exc}')
            return

        if self._net is not None:
            self.obstacle_detected = self._detect_onnx(img)
        else:
            self.obstacle_detected = self._detect_hsv(img)

        if self.obstacle_detected:
            rospy.logwarn_throttle(1.0, '[ObstacleDetector] Duckie detected – stopping!')

    # ------------------------------------------------------------------
    # ONNX inference (YOLOv5 / YOLOv8 compatible)
    # ------------------------------------------------------------------
    def _detect_onnx(self, img: np.ndarray) -> bool:
        """Run YOLO ONNX inference and return True if a duckie is close."""
        h, w = img.shape[:2]

        # Pre-process: resize + normalise to [0, 1] float32 NCHW blob
        blob = cv2.dnn.blobFromImage(
            img,
            scalefactor=1.0 / 255.0,
            size=(MODEL_INPUT_SIZE, MODEL_INPUT_SIZE),
            swapRB=True,         # BGR → RGB
            crop=False,
        )
        self._net.setInput(blob)

        # Forward pass
        outputs = self._net.forward()

        # outputs shape: (1, num_predictions, 5 + num_classes)
        # Each row: [cx, cy, w, h, obj_conf, cls_conf0, cls_conf1, ...]
        predictions = outputs[0]  # shape (num_pred, 5+C)

        close_threshold_y = h * (1.0 - CLOSE_REGION_FRACTION)

        # Scale factors from model input back to original image
        sx = w / MODEL_INPUT_SIZE
        sy = h / MODEL_INPUT_SIZE

        for pred in predictions:
            obj_conf = float(pred[4])
            if obj_conf < CONF_THRESHOLD:
                continue

            cls_scores = pred[5:]
            class_id = int(np.argmax(cls_scores))
            confidence = obj_conf * float(cls_scores[class_id])

            if confidence < CONF_THRESHOLD:
                continue

            # Convert centre-format to pixel coordinates
            cx = float(pred[0]) * sx
            cy = float(pred[1]) * sy
            bw = float(pred[2]) * sx
            bh = float(pred[3]) * sy
            y_bottom = cy + bh / 2.0

            # Only flag as obstacle if the bottom of the box is in the close region
            if y_bottom >= close_threshold_y:
                return True

        return False

    # ------------------------------------------------------------------
    # HSV fallback
    # ------------------------------------------------------------------
    def _detect_hsv(self, img: np.ndarray) -> bool:
        """Colour-based fallback detector for yellow duckies."""
        h, w = img.shape[:2]
        roi = img[int(h * (1 - CLOSE_REGION_FRACTION)):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, DUCKIE_HSV_LOWER, DUCKIE_HSV_UPPER)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(cv2.contourArea(c) >= MIN_BLOB_AREA for c in contours)
