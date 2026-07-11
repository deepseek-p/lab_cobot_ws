"""Offline voice transcription boundary."""
from __future__ import annotations

from typing import Protocol


class Transcriber(Protocol):
    def transcribe(self, path: str, language: str) -> str:
        """Transcribe an audio file."""


class FasterWhisperTranscriber:
    """CPU-only faster-whisper transcriber."""

    def __init__(
        self,
        model_size: str = "small",
        offline_only: bool = True,
        cpu_threads: int = 4,
    ) -> None:
        from faster_whisper import WhisperModel

        self._model = WhisperModel(
            model_size,
            device="cpu",
            compute_type="int8",
            cpu_threads=int(cpu_threads),
            local_files_only=bool(offline_only),
        )

    def transcribe(self, path: str, language: str) -> str:
        """Transcribe an audio file."""
        segments, _info = self._model.transcribe(path, language=language)
        return "".join(segment.text for segment in segments).strip()
