#!/usr/bin/env python3
"""Voice command entry node."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from lab_cobot_bringup.voice_transcriber import (
    FasterWhisperTranscriber,
    Transcriber,
)


class VoiceNode(Node):
    def __init__(self, transcriber: Transcriber | None = None):
        super().__init__("voice_node")
        self.declare_parameter("audio_file", "")
        self.declare_parameter("model_size", "small")
        self.declare_parameter("language", "zh")
        self.declare_parameter("offline_only", True)
        self.declare_parameter("auto_publish_on_start", True)
        self._audio_file = str(self.get_parameter("audio_file").value)
        self._model_size = str(self.get_parameter("model_size").value)
        self._language = str(self.get_parameter("language").value)
        self._offline_only = bool(self.get_parameter("offline_only").value)
        self._transcriber = transcriber
        self._publisher = self.create_publisher(String, "/task/instruction", 10)
        self.create_subscription(Empty, "/voice/trigger", self._on_trigger, 10)
        if bool(self.get_parameter("auto_publish_on_start").value):
            self._startup_timer = self.create_timer(0.5, self._publish_on_start)
        else:
            self._startup_timer = None

    def _get_transcriber(self) -> Transcriber:
        if self._transcriber is None:
            self._transcriber = FasterWhisperTranscriber(
                model_size=self._model_size,
                offline_only=self._offline_only,
            )
        return self._transcriber

    def _publish_on_start(self) -> None:
        if self._startup_timer is not None:
            self._startup_timer.cancel()
        self._publish_transcription()

    def _publish_transcription(self) -> bool:
        if not self._audio_file:
            self.get_logger().warn("voice audio_file is empty")
            return False
        text = self._get_transcriber().transcribe(self._audio_file, self._language)
        if not text:
            self.get_logger().warn("voice transcription is empty")
            return False
        msg = String()
        msg.data = text
        self._publisher.publish(msg)
        self.get_logger().info(f"voice instruction published: {text}")
        return True

    def _on_trigger(self, _msg: Empty) -> None:
        self._publish_transcription()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
