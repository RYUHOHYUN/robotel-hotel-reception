# My Contribution

## 역할

ROBOTEL은 5명이 기능을 병렬로 개발한 팀 프로젝트입니다. 저는 다음 영역을 담당했습니다.

- 전체 시스템 설계도와 비전·로봇 연결 구조 정리
- 종이 뽑기판의 `1등`~`4등` 인식용 PaddleOCR 노드 개발
- YuNet, SFace, RealSense Depth를 이용한 얼굴 본인 확인 프로그램 개발
- 고정 좌표와 경유점을 이용한 협동로봇 제어 코드 작성
- 비전 노드와 로봇 Service의 통합 테스트 및 오류 수정

## 제가 직접 구현한 부분

### 뽑기 등수 OCR

단순히 PaddleOCR를 호출하는 것에 그치지 않고, 실제 카메라 환경에서 오인식을 줄이기 위한 흐름을 구성했습니다.

- ROI를 비율 좌표로 관리해 해상도 변화에 대응
- CLAHE, sharpening, Otsu threshold 전처리 비교
- OCR 결과의 특수문자와 유사 문자를 정규화
- `1`, `2`, `3`, `4` 숫자만 나온 결과는 거부
- 숫자와 `등` 계열 문자가 함께 존재해야 등수 후보로 인정
- 여러 프레임에서 반복된 결과를 가중 투표해 최종 확정
- OCR worker thread와 queue를 사용해 ROS callback 지연 완화

### 얼굴 본인 확인

여권 사진과 실시간 얼굴을 2D 유사도만으로 비교하면 평면 사진 공격을 막기 어렵다고 판단했습니다.

- YuNet으로 얼굴과 5개 landmark 검출
- SFace로 정렬된 얼굴의 특징 벡터 추출
- cosine similarity를 여러 프레임에서 수집하고 median으로 안정화
- RGB와 aligned Depth의 최신 프레임을 짝지어 사용
- 얼굴 영역의 Depth 유효 비율, 깊이 span, 코-볼 깊이 차이를 검사
- 유사도와 Depth liveness를 동시에 통과해야 인증 성공

### 좌표 기반 로봇 제어

시연 환경에서는 물체와 트레이 위치가 고정되어 있었기 때문에, 비전 기반 동적 경로 계획보다 검증된 고정 좌표와 경유점을 사용해 반복성을 우선했습니다.

- 안전 자세는 `movej`, 물체 접근·하강은 `movel` 사용
- 상단 접근 → 수직 하강 → 파지 → 수직 상승 → 수평 이동 → 수직 하강 순서 유지
- RG2 그리퍼 폭과 force 설정
- 모션 중복 요청 방지를 위한 lock
- 좌표 유효성 및 API 반환값 확인
- 실제 로봇 없이 Service 흐름을 확인할 수 있는 dry-run 제공

## 팀 시스템 맥락

다음 기능은 ROBOTEL 팀 전체 시스템의 구성 요소이지만, 이 저장소에서는 제 구현을 이해하기 위한 맥락으로만 다룹니다.

- Flask 키오스크와 관리자 페이지
- 예약·객실·로그 SQLite DB Manager
- STT, LLM, TTS 기반 다국어 안내
- QR 예약 조회
- 여권 MRZ OCR
- 체크아웃 카드번호 OCR과 카드 반환
- 웰컴 드링크·어메니티 서비스
- 전체 Launch와 시나리오 통합

## 사용한 기술과 구현한 기술의 구분

### 기존 기술을 사용한 부분

- PaddleOCR의 사전학습 한국어 text recognition model
- OpenCV YuNet face detector
- OpenCV SFace face recognizer
- Intel RealSense RGB-D camera
- Doosan Robotics `movej`, `movel` API
- OnRobot RG2 control interface

### 직접 설계·구현한 부분

- 모델 입력을 위한 전처리와 ROI 구성
- OCR 결과 정규화와 오인식 거부 규칙
- 다중 프레임 투표 기반 안정 판정
- 얼굴 유사도와 Depth liveness 결합 로직
- ROS 2 Service/Topic 인터페이스와 상태 관리
- 고정 좌표 기반 pick/place 시퀀스와 예외 처리
- 팀 시스템에서 각 노드가 연결되는 구조 정리

## 구현 근거 파일

| 파일 | 내용 |
|---|---|
| `src/realtime_rank_ocr_paddle.py` | 실시간 등수 OCR, 전처리, 투표, ROS Service/Topic |
| `src/verification_node.py` | YuNet/SFace 얼굴 비교와 RGB-D liveness |
| `src/event_prize_delivery_node.py` | 3등 경품 고정 좌표 pick/place |
| `src/passport_robot_motion_node.py` | 좌표 입력 기반 여권 파지와 트레이 이동 |
| `config/fixed_robot_poses.yaml` | 코드의 주요 관절·BASE 좌표 참고 정리 |
