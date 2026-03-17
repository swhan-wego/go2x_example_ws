# COCO 클래스 ID 목록

`object_yolo_node`의 `target_class` 파라미터에 사용할 COCO 클래스 ID 목록입니다.

## 사람

| ID | 클래스 |
|----|--------|
| 0  | person |

## 탈것

| ID | 클래스 |
|----|--------|
| 1  | bicycle |
| 2  | car |
| 3  | motorcycle |
| 4  | airplane |
| 5  | bus |
| 6  | train |
| 7  | truck |
| 8  | boat |

## 도로/교통

| ID | 클래스 |
|----|--------|
| 9  | traffic light |
| 10 | fire hydrant |
| 11 | stop sign |
| 12 | parking meter |
| 13 | bench |

## 동물

| ID | 클래스 |
|----|--------|
| 14 | bird |
| 15 | cat |
| 16 | dog |
| 17 | horse |
| 18 | sheep |
| 19 | cow |
| 20 | elephant |
| 21 | bear |
| 22 | zebra |
| 23 | giraffe |

## 액세서리

| ID | 클래스 |
|----|--------|
| 24 | backpack |
| 25 | umbrella |
| 26 | handbag |
| 27 | tie |
| 28 | suitcase |

## 스포츠

| ID | 클래스 |
|----|--------|
| 29 | frisbee |
| 30 | skis |
| 31 | snowboard |
| 32 | sports ball |
| 33 | kite |
| 34 | baseball bat |
| 35 | baseball glove |
| 36 | skateboard |
| 37 | surfboard |
| 38 | tennis racket |

## 주방/음식

| ID | 클래스 |
|----|--------|
| 39 | bottle |
| 40 | wine glass |
| 41 | cup |
| 42 | fork |
| 43 | knife |
| 44 | spoon |
| 45 | bowl |
| 46 | banana |
| 47 | apple |
| 48 | sandwich |
| 49 | orange |
| 50 | broccoli |
| 51 | carrot |
| 52 | hot dog |
| 53 | pizza |
| 54 | donut |
| 55 | cake |

## 가구/실내

| ID | 클래스 |
|----|--------|
| 56 | chair |
| 57 | couch |
| 58 | potted plant |
| 59 | bed |
| 60 | dining table |
| 61 | toilet |

## 전자기기

| ID | 클래스 |
|----|--------|
| 62 | tv |
| 63 | laptop |
| 64 | mouse |
| 65 | remote |
| 66 | keyboard |
| 67 | cell phone |

## 가전/기타 실내용품

| ID | 클래스 |
|----|--------|
| 68 | microwave |
| 69 | oven |
| 70 | toaster |
| 71 | sink |
| 72 | refrigerator |
| 73 | book |
| 74 | clock |
| 75 | vase |
| 76 | scissors |
| 77 | teddy bear |
| 78 | hair drier |
| 79 | toothbrush |

## 사용 예

```bash
# 사람 추적 (기본값)
ros2 launch image_agent object_yolo_node.launch.py

# 고양이 추적
ros2 launch image_agent object_yolo_node.launch.py target_class:=15

# 자동차 추적
ros2 launch image_agent object_yolo_node.launch.py target_class:=2

# 강아지 추적, 모든 검출 결과 발행
ros2 launch image_agent object_yolo_node.launch.py target_class:=16 publish_all:=true
```
