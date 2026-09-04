#!/usr/bin/env python3
"""Associate overview and wrist RGB-D estimates without averaging poses.

The two cameras have different extrinsic-error and depth-noise profiles.  A
numeric average therefore makes a good close wrist estimate worse.  This node
uses the base-mounted/overview camera only as a coarse world-frame hint and
publishes the wrist estimate unchanged once the two observations agree on
label and world position.
"""
import json
import math
from collections import defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _stamp(payload):
    try:
        return float(payload.get("timestamp"))
    except (TypeError, ValueError):
        return None


def _items(payload):
    values = payload.get("detections", []) if isinstance(payload, dict) else []
    result = []
    for item in values:
        if not isinstance(item, dict):
            continue
        label = str(item.get("label", "")).strip()
        position = item.get("position", [])
        if (not label or len(position) != 3):
            continue
        try:
            point = tuple(float(value) for value in position)
        except (TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in point):
            result.append({
                "label": label,
                "position": point,
                "confidence": float(item.get("confidence", 0.0)),
            })
    return result


class DualCameraFusionNode(Node):
    """Publish coarse hints and wrist-confirmed poses with audit metadata."""

    def __init__(self):
        super().__init__("dual_camera_fusion")
        defaults = {
            "overview_pose_topic": "/yolo/bench/poses",
            "wrist_pose_topic": "/yolo/poses",
            "hint_topic": "/perception/overview/hints",
            "confirmed_pose_topic": "/perception/dual_camera/confirmed_poses",
            "metrics_topic": "/perception/dual_camera/metrics",
            # A station-aiming hint must describe the current stopped view;
            # retaining it for 30 s could steer the wrist using a prior aisle.
            "hint_max_age_sec": 2.0,
            "max_world_association_distance_m": 0.35,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._hints = defaultdict(list)
        self._hint_pub = self.create_publisher(
            String, self.get_parameter("hint_topic").value, 20)
        self._confirmed_pub = self.create_publisher(
            String, self.get_parameter("confirmed_pose_topic").value, 20)
        self._metrics_pub = self.create_publisher(
            String, self.get_parameter("metrics_topic").value, 20)
        self.create_subscription(
            String, self.get_parameter("overview_pose_topic").value,
            self._overview_cb, 20)
        self.create_subscription(
            String, self.get_parameter("wrist_pose_topic").value,
            self._wrist_cb, 20)

    def _overview_cb(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        stamp = _stamp(payload)
        if stamp is None:
            return
        frame_id = str(payload.get("frame_id", ""))
        for item in _items(payload):
            hint = {**item, "timestamp": stamp, "frame_id": frame_id}
            history = self._hints[item["label"]]
            history.append(hint)
            # Preserve only a compact history: the nearest matching position
            # is needed for repeated instances such as beakers/test tubes.
            del history[:-20]
            self._hint_pub.publish(_json({
                "event": "overview_hint", "camera_source": "overview",
                **hint,
            }))
            self._metrics_pub.publish(_json({
                "event": "overview_world_hint", "label": item["label"],
                "timestamp": stamp,
            }))

    def _wrist_cb(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        stamp = _stamp(payload)
        if stamp is None:
            return
        frame_id = str(payload.get("frame_id", ""))
        age_limit = float(self.get_parameter("hint_max_age_sec").value)
        distance_limit = float(
            self.get_parameter("max_world_association_distance_m").value)
        for item in _items(payload):
            candidates = [
                hint for hint in self._hints.get(item["label"], [])
                if 0.0 <= stamp - hint["timestamp"] <= age_limit
            ]
            nearest = min(
                candidates,
                key=lambda hint: _distance(item["position"], hint["position"]),
                default=None,
            )
            separation = (None if nearest is None else
                          _distance(item["position"], nearest["position"]))
            confirmed = separation is not None and separation <= distance_limit
            event = {
                "event": "wrist_refined_pose",
                "camera_source": "wrist",
                "label": item["label"], "position": list(item["position"]),
                "confidence": item["confidence"], "timestamp": stamp,
                "frame_id": frame_id, "overview_confirmed": confirmed,
                "overview_distance_m": separation,
            }
            if nearest is not None:
                event["overview_position"] = list(nearest["position"])
                event["overview_timestamp"] = nearest["timestamp"]
            # The wrist point is deliberately forwarded unchanged: it is the
            # close, centred measurement used by error evaluation/grasping.
            self._confirmed_pub.publish(_json(event))
            self._metrics_pub.publish(_json({
                "event": "wrist_world_transform",
                "label": item["label"], "timestamp": stamp,
                "overview_confirmed": confirmed,
                "overview_distance_m": separation,
            }))


def _distance(first, second):
    return math.sqrt(sum((first[index] - second[index]) ** 2 for index in range(3)))


def _json(value):
    message = String()
    message.data = json.dumps(value)
    return message


def main(args=None):
    rclpy.init(args=args)
    node = DualCameraFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
