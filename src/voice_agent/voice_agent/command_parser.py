# command_parser.py — 음성 명령어 파싱 모듈
# 7장: 음성 제어 로봇 (STT/TTS)

# 명령어별 키워드 매핑 딕셔너리
COMMAND_MAP = {
    "forward": {
        "keywords": ["앞으로", "전진", "앞으로 가", "앞으로 이동", "앞"],
        "feedback": "앞으로 이동합니다.",
    },
    "backward": {
        "keywords": ["뒤로", "후진", "뒤로 가", "뒤로 이동", "뒤"],
        "feedback": "뒤로 이동합니다.",
    },
    "turn_left": {
        "keywords": ["왼쪽", "좌회전", "왼쪽으로", "좌"],
        "feedback": "왼쪽으로 회전합니다.",
    },
    "turn_right": {
        "keywords": ["오른쪽", "우회전", "오른쪽으로", "우"],
        "feedback": "오른쪽으로 회전합니다.",
    },
    "stop": {
        "keywords": ["멈춰", "스톱", "그만", "정지", "stop"],
        "feedback": "정지합니다.",
    },
    "sit": {
        "keywords": ["앉아", "앉아라", "앉기"],
        "feedback": "앉겠습니다.",
    },
    "stand": {
        "keywords": ["일어서", "일어나", "스탠드업"],
        "feedback": "일어서겠습니다.",
    },
    "down": {
        "keywords": ["엎드려", "다운", "엎드려라"],
        "feedback": "엎드리겠습니다.",
    },
}


def parse_command(text: str) -> tuple[str | None, str | None]:
    """
    인식된 텍스트에서 명령을 추출합니다.

    Args:
        text: STT로 변환된 텍스트

    Returns:
        (command_key, feedback_text) 튜플
        인식 실패 시 (None, None) 반환
    """
    if not text:
        return None, None

    # 정지 명령을 최우선으로 확인 (안전)
    stop_entry = COMMAND_MAP["stop"]
    for kw in stop_entry["keywords"]:
        if kw in text:
            return "stop", stop_entry["feedback"]

    # 나머지 명령 순차 확인
    for cmd, entry in COMMAND_MAP.items():
        if cmd == "stop":
            continue
        for kw in entry["keywords"]:
            if kw in text:
                return cmd, entry["feedback"]

    return None, None


# 단독 실행 시 테스트
if __name__ == "__main__":
    test_cases = [
        "앞으로 가줘",
        "왼쪽으로 회전해",
        "멈춰",
        "앉아라",
        "알 수 없는 명령",
    ]
    for t in test_cases:
        cmd, fb = parse_command(t)
        print(f"입력: '{t}' → 명령: {cmd}, 피드백: {fb}")
