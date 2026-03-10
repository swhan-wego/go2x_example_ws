# audio_test.py — 마이크 + Whisper STT + TTS 통합 테스트
# 7장: 음성 제어 로봇 (STT/TTS)
# 실습 전 환경 동작 확인용 스크립트

import asyncio
import os

import edge_tts
import numpy as np
import pygame
import sounddevice as sd
from faster_whisper import WhisperModel

# ─── 설정값 ───────────────────────────────────────────────
SAMPLE_RATE = 16000
DURATION    = 3
MODEL_SIZE  = "small"


def check_audio_devices():
    """연결된 오디오 장치 목록 확인"""
    print("=== 오디오 장치 목록 ===")
    print(sd.query_devices())
    print()

    default_input = sd.query_devices(kind='input')
    print(f"기본 마이크: {default_input['name']}")
    print(f"샘플링 레이트: {default_input['default_samplerate']} Hz")
    print(f"채널 수: {default_input['max_input_channels']}")
    print()

    default_output = sd.query_devices(kind='output')
    print(f"기본 스피커: {default_output['name']}")
    print()


def record_audio(duration: int) -> np.ndarray:
    """마이크에서 오디오를 캡처하여 numpy 배열로 반환"""
    print(f"🎤 {duration}초 동안 말씀해 주세요...")
    audio = sd.rec(
        int(duration * SAMPLE_RATE),
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype='float32'
    )
    sd.wait()
    return audio.flatten()


async def speak(text: str):
    """텍스트를 TTS로 변환하여 재생"""
    output_file = "/tmp/tts_output.mp3"
    communicate = edge_tts.Communicate(text, voice="ko-KR-SunHiNeural")
    await communicate.save(output_file)

    pygame.mixer.init()
    pygame.mixer.music.load(output_file)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)
    os.remove(output_file)


def main():
    # 오디오 장치 확인
    check_audio_devices()

    # Whisper 모델 로드
    print(f"Whisper {MODEL_SIZE} 모델 로딩 중...")
    model = WhisperModel(MODEL_SIZE, device="cpu", compute_type="int8")
    print("모델 로딩 완료!")
    print()

    # 음성 캡처
    audio = record_audio(DURATION)

    # STT 변환
    print("음성 인식 중...")
    segments, info = model.transcribe(audio, language="ko")
    recognized_text = " ".join([seg.text for seg in segments]).strip()
    print(f"인식 결과: {recognized_text}")

    # TTS 피드백
    if recognized_text:
        feedback = f"인식된 내용: {recognized_text}"
        asyncio.run(speak(feedback))
    else:
        print("음성을 인식하지 못했습니다.")


if __name__ == "__main__":
    main()
