"""Voice transcriber contracts with fake faster_whisper injection."""
import ast
import sys
import types
from pathlib import Path

from lab_cobot_bringup.voice_transcriber import FasterWhisperTranscriber


def test_voice_transcriber_has_no_top_level_faster_whisper_import():
    module = (
        Path(__file__).resolve().parents[1]
        / "lab_cobot_bringup"
        / "voice_transcriber.py"
    )
    tree = ast.parse(module.read_text(encoding="utf-8"))
    imports = []
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.extend(alias.name for alias in node.names)
        if isinstance(node, ast.ImportFrom):
            imports.append(node.module)

    assert "faster_whisper" not in imports


def test_faster_whisper_transcriber_forces_cpu_int8_and_offline(monkeypatch):
    captured = {}

    class FakeSegment:
        def __init__(self, text):
            self.text = text

    class FakeWhisperModel:
        def __init__(self, model_size, **kwargs):
            captured["model_size"] = model_size
            captured.update(kwargs)

        def transcribe(self, path, language=None):
            captured["path"] = path
            captured["language"] = language
            return [FakeSegment("把样件"), FakeSegment("送到B")], object()

    fake_module = types.SimpleNamespace(WhisperModel=FakeWhisperModel)
    monkeypatch.setitem(sys.modules, "faster_whisper", fake_module)

    transcriber = FasterWhisperTranscriber(
        model_size="small",
        offline_only=True,
        cpu_threads=4,
    )

    assert transcriber.transcribe("/tmp/voice.wav", language="zh") == "把样件送到B"
    assert captured["model_size"] == "small"
    assert captured["device"] == "cpu"
    assert captured["compute_type"] == "int8"
    assert captured["cpu_threads"] == 4
    assert captured["local_files_only"] is True
    assert captured["path"] == "/tmp/voice.wav"
    assert captured["language"] == "zh"
