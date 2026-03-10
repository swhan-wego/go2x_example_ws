# tts_module.py — edge-tts + pygame 기반 TTS(음성 합성) 엔진
# 7장: 음성 제어 로봇 (STT/TTS)

import asyncio
import os
import tempfile
import threading

import edge_tts
import pygame


# edge-tts 한국어 음성 목록
VOICE_FEMALE = "ko-KR-SunHiNeural"   # 여성 음성 (밝고 명확)
VOICE_MALE   = "ko-KR-InJoonNeural"  # 남성 음성 (차분하고 안정적)


class TTSEngine:
    """edge-tts + pygame 기반 TTS 엔진 (스레드 비동기 재생 지원)"""

    def __init__(self, voice: str = VOICE_FEMALE):
        self.voice = voice
        pygame.mixer.init()
        self._lock = threading.Lock()  # 동시 재생 방지
        print(f"TTS 엔진 준비 완료 (음성: {voice})")

    async def _generate(self, text: str) -> str:
        """텍스트 → MP3 파일 생성 (임시 파일 경로 반환)"""
        tmp = tempfile.NamedTemporaryFile(suffix=".mp3", delete=False)
        tmp.close()
        communicate = edge_tts.Communicate(text, voice=self.voice)
        await communicate.save(tmp.name)
        return tmp.name

    def _play(self, text: str):
        """스레드 내부에서 실행되는 재생 함수"""
        with self._lock:  # 중복 재생 방지
            audio_path = asyncio.run(self._generate(text))
            pygame.mixer.music.load(audio_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
            os.remove(audio_path)

    def speak(self, text: str, blocking: bool = False):
        """
        텍스트를 음성으로 재생합니다.

        Args:
            text    : 읽을 텍스트
            blocking: True이면 재생 완료까지 대기,
                      False이면 백그라운드에서 재생 (권장)
        """
        print(f"[TTS] {text}")
        if blocking:
            self._play(text)
        else:
            thread = threading.Thread(target=self._play, args=(text,),
                                      daemon=True)
            thread.start()

    def __del__(self):
        pygame.mixer.quit()


# 단독 실행 시 테스트
if __name__ == "__main__":
    tts = TTSEngine(voice=VOICE_FEMALE)
    tts.speak("안녕하세요. TTS 테스트입니다.", blocking=True)
    tts.speak("앞으로 이동합니다.", blocking=True)
