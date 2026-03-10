# wakeword_detector.py — Whisper 기반 웨이크워드 감지기
# 7장: 음성 제어 로봇 (STT/TTS)

import sounddevice as sd
import numpy as np
from faster_whisper import WhisperModel


SAMPLE_RATE = 16000
WAKE_WORDS  = ["고투야", "고투", "고투 야"]  # 인식될 웨이크워드 목록


class WakeWordDetector:
    """Whisper 기반 웨이크워드 감지기"""

    def __init__(self, model: WhisperModel,
                 check_duration: float = 1.5):
        """
        Args:
            model         : 공유할 WhisperModel 인스턴스 (STT와 동일 모델)
            check_duration: 한 번에 감지할 오디오 길이 (초)
        """
        self.model          = model
        self.check_duration = check_duration

    def _record_chunk(self) -> np.ndarray:
        """짧은 오디오 클립 캡처"""
        audio = sd.rec(
            int(self.check_duration * SAMPLE_RATE),
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype='float32'
        )
        sd.wait()
        return audio.flatten()

    def _is_wake_word(self, audio: np.ndarray) -> bool:
        """오디오 클립에 웨이크워드가 포함되어 있는지 확인"""
        segments, _ = self.model.transcribe(
            audio,
            language="ko",
            beam_size=1,      # 빠른 감지를 위해 beam_size 최소화
            vad_filter=True,
        )
        text = " ".join(s.text.strip() for s in segments).strip()

        for wake_word in WAKE_WORDS:
            if wake_word in text:
                print(f"웨이크워드 감지: '{text}'")
                return True
        return False

    def wait_for_wake_word(self):
        """웨이크워드가 감지될 때까지 반복 청취"""
        print("대기 중... ('고투야'라고 말해주세요)")
        while True:
            audio = self._record_chunk()
            if self._is_wake_word(audio):
                return  # 웨이크워드 감지 → 메인 루프로 반환
