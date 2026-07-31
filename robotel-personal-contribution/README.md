# ROBOTEL — 개인 구현 정리

호텔 리셉션 자동화 프로젝트에서 제가 직접 구현·실험·검증한 내용을 정리한 저장소입니다.

> 이 저장소는 전체 팀 프로젝트가 아니라 **개인 기여 코드와 경험만 추린 포트폴리오용 저장소**입니다.

## 한눈에 보기

| 영역 | 내가 한 내용 | 핵심 기술 |
|---|---|---|
| 로봇 제어 | 카드키와 3등 상품 키링의 좌표 기반 Pick & Place | Doosan M0609, OnRobot RG2, `posx`, `posj` |
| 객체 검출 | 캔 4종 데이터셋 구성 및 YOLO 모델 비교 | YOLOv8/YOLO11, mAP, Precision, Recall, F1 |
| 얼굴·문자 인식 | 여권 사진과 실물 얼굴 비교, 경품 등수 OCR | YuNet, SFace, PaddleOCR, OpenCV |

## 폴더 구조

```text
.
├── README.md
├── robot_control/
│   ├── card_move_node.py       # 카드키 좌표 이동
│   ├── keyring_move_node.py    # 3등 상품 키링 이동
│   ├── onrobot_card.py         # 카드 이동용 RG2 제어
│   └── onrobot_rg2.py          # 키링 이동용 RG2 제어
├── vision/
│   ├── face_verification_node.py  # YuNet + SFace + Depth 얼굴 확인
│   └── rank_ocr_node.py           # PaddleOCR 경품 등수 인식
└── yolo/
    └── README.md               # 캔 검출 학습·평가 경험
```

## 1. 좌표 기반 로봇 제어

카드키와 3등 상품 키링을 이동시키기 위해 작업별 TCP 좌표와 경유점을 구성했습니다.

```text
초기 위치 → 상단 접근 → Pick → Lift → Place 상단 → Place 하단 → 초기 위치
```

- `posx`: 실제 TCP 좌표 기반 선형 이동
- `posj`: 초기 자세와 티칭 기준 자세 기록
- `width / force`: 물체 두께와 접촉면에 맞게 그리퍼 값 조정
- `pick_above`, `pick_lift`, `place_above`를 분리해 충돌 가능성 감소

**배운 점:** 정확한 목표 좌표만 찾는 것보다 환경을 고정하고 안전한 이동 순서를 설계하는 것이 중요했습니다.

## 2. YOLO 캔 검출

`coffee`, `cola`, `tea`, `sikhye` 4개 클래스를 대상으로 데이터셋을 구성하고 모델별 결과를 비교했습니다.

- 위치·각도·순서·거리·배경 변화를 데이터에 포함
- YOLOv8/YOLO11 계열 결과 비교
- mAP, Precision, Recall, F1 확인
- 지표뿐 아니라 실제 예측 이미지의 오검출·누락도 함께 확인

자세한 내용은 [`yolo/README.md`](yolo/README.md)에 정리했습니다.

## 3. 얼굴 확인과 경품 등수 OCR

### 얼굴 확인

```text
여권 얼굴 검출 → 특징 벡터 추출 → 실시간 얼굴 비교 → 여러 프레임 점수 안정화 → Depth 확인
```

- YuNet으로 얼굴 검출
- SFace로 얼굴 특징 벡터 추출 및 비교
- 여러 프레임의 점수를 사용해 판정 흔들림 완화
- RealSense Depth 조건으로 평면 사진과 실제 얼굴 구분 보조

### 경품 등수 OCR

```text
좁은 ROI → 전처리 변형 → PaddleOCR → 선명한 프레임 선택 → 신뢰도 가중 투표
```

- 한국어 PaddleOCR 모델 사용
- 문자 영역을 좁게 제한해 불필요한 배경 제거
- 전처리 결과를 비교하고 여러 프레임의 결과를 투표해 안정화

## 문제 해결 경험

| 문제 | 해결 |
|---|---|
| 좌표는 맞지만 주변 구조물과 충돌 | 상단 접근점과 들어올림 지점을 분리 |
| 카드키·키링이 미끄러지거나 걸림 | 물체별 좌표·높이·그리퍼 값을 한 세트로 조정 |
| YOLO 지표와 실제 화면 성능 차이 | 실제 배치와 비슷한 위치·각도·배경 데이터를 추가 |
| 얼굴·문자 문제에 학습 비용 증가 | YuNet·SFace·PaddleOCR 등 검증된 전용 모델을 우선 적용 |

## 실행 전 주의

- 좌표와 그리퍼 값은 프로젝트 작업대와 물체 배치를 기준으로 티칭한 값입니다.
- 실제 로봇에서는 **저속·Dry Run·충돌 영역 확인 후** 실행해야 합니다.
- 비전 노드는 ROS 2 인터페이스와 카메라 토픽, 얼굴 모델 파일이 추가로 필요합니다.
- 전체 실행 환경은 Ubuntu 22.04, ROS 2 Humble, Doosan M0609, OnRobot RG2를 기준으로 했습니다.
