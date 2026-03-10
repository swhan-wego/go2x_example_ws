# stt_module.py — Whisper 기반 STT(음성 인식) 엔진
# 7장: 음성 제어 로봇 (STT/TTS)

import sounddevice as sd
import numpy as np
from faster_whisper import WhisperModel


SAMPLE_RATE    = 16000
ROBOT_COMMANDS = "앞으로, 뒤로, 왼쪽, 오른쪽, 멈춰, 앉아, 일어서, 엎드려, 달려"


class STTEngine:
    """Whisper 기반 음성 인식 엔진"""

    def __init__(self, model_size: str = "small"):
        print(f"STT 엔진 초기화 중 (모델: {model_size})...")
        self.model = WhisperModel(model_size, device="cpu",
                                  compute_type="int8")
        print("STT 엔진 준비 완료!")

    def record(self, duration: float = 3.0) -> np.ndarray:
        """고정 시간 동안 오디오 캡처"""
        audio = sd.rec(
            int(duration * SAMPLE_RATE),
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype='float32'
        )
        sd.wait()
        return audio.flatten()

    def transcribe(self, audio: np.ndarray) -> str:
        """오디오 → 텍스트 변환"""
        # 오디오 정규화
        max_val = np.max(np.abs(audio))
        if max_val > 0:
            audio = audio / max_val

        segments, _ = self.model.transcribe(
            audio,
            language="ko",
            beam_size=5,
            vad_filter=True,
            initial_prompt=ROBOT_COMMANDS,
        )
        return " ".join(s.text.strip() for s in segments).strip()

    def listen(self, duration: float = 3.0) -> str:
        """녹음 + 변환을 한 번에 수행"""
        audio = self.record(duration)
        return self.transcribe(audio)


# 단독 실행 시 테스트
if __name__ == "__main__":
    stt = STTEngine("small")
    print("3초 동안 말씀해 주세요...")
    text = stt.listen(duration=3.0)
    print(f"인식 결과: {text}")
