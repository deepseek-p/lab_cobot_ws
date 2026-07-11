"""Voice node unit tests with fake transcriber injection."""
from std_msgs.msg import Empty

from lab_cobot_bringup.voice_node import VoiceNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(str(msg.data))


class FakeLogger:
    def __init__(self):
        self.logs = []

    def info(self, message):
        self.logs.append(("info", message))

    def warn(self, message):
        self.logs.append(("warn", message))

    def error(self, message):
        self.logs.append(("error", message))


class FakeTranscriber:
    def __init__(self, text):
        self.text = text
        self.calls = []

    def transcribe(self, path, language):
        self.calls.append((path, language))
        return self.text


def _node(audio_file="", text="把样件从A送到B"):
    node = VoiceNode.__new__(VoiceNode)
    node._audio_file = audio_file
    node._language = "zh"
    node._publisher = FakePublisher()
    node._transcriber = FakeTranscriber(text)
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    return node


def test_voice_node_publishes_transcribed_audio_file():
    node = _node(audio_file="/tmp/command.wav")

    assert VoiceNode._publish_transcription(node)
    assert node._publisher.messages == ["把样件从A送到B"]
    assert node._transcriber.calls == [("/tmp/command.wav", "zh")]


def test_voice_node_skips_empty_audio_file():
    node = _node(audio_file="")

    assert not VoiceNode._publish_transcription(node)
    assert node._publisher.messages == []
    assert node._transcriber.calls == []


def test_voice_node_trigger_republishes_current_audio_file():
    node = _node(audio_file="/tmp/command.wav", text="去A工位检查一下样件")

    VoiceNode._on_trigger(node, Empty())

    assert node._publisher.messages == ["去A工位检查一下样件"]
    assert node._transcriber.calls == [("/tmp/command.wav", "zh")]
