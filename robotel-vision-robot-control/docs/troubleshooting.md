# Troubleshooting and Lessons Learned

## 1. 숫자 단독 OCR 오인식

### Issue

종이가 일부만 보이거나 ROI 경계와 배경 무늬가 포함되면 `1`~`4` 숫자만 검출되어 잘못된 등수로 확정될 수 있었습니다.

### Solution

- 숫자 단독 결과는 거부
- 숫자와 `등` 계열 문자가 함께 검출된 경우에만 후보 인정
- 여러 전처리 결과 중 score가 높은 후보 선택
- 단일 프레임이 아닌 최근 프레임의 가중 투표 적용
- 최소 노출 시간 조건 추가

### Lesson

OCR model score만 신뢰하기보다 서비스 도메인의 문법을 후처리 규칙에 반영하는 것이 중요했습니다.

## 2. 얼굴 외모 변화와 조명

### Issue

수염, 헤어스타일, 조명, 얼굴 각도 차이로 여권 사진과 실시간 얼굴의 similarity가 변했습니다.

### Solution

- YuNet으로 얼굴 landmark 검출
- SFace로 정렬된 얼굴 특징 비교
- 여러 프레임 score의 median 사용
- RealSense Depth liveness를 별도 조건으로 적용
- 동일인·타인 threshold 사이를 즉시 판정하지 않고 uncertain 상태로 처리

### Result

결과보고서 기준 동일인 20/20 통과, 타인과 여권 평면 사진은 각각 통과 0/20으로 기록되었습니다.

## 3. RGB와 Depth 불일치

### Issue

새 RGB 프레임과 오래된 Depth 프레임을 함께 사용하면 얼굴 위치와 Depth ROI가 어긋날 수 있습니다.

### Solution

- RGB와 Depth sequence가 모두 갱신된 경우에만 사용
- 두 frame의 수신 시각 차이 확인
- image shape이 다르면 판정 중단
- `aligned_depth_to_color` topic 사용

## 4. 로봇 좌표 이동의 반복성

### Issue

낮은 높이에서 대각선으로 이동하거나 목적지에 바로 접근하면 fixture와 충돌할 위험이 있고, 파지 반복성이 떨어질 수 있습니다.

### Solution

- 상단 경유점 추가
- 수직 하강과 수직 상승 분리
- 관절 이동과 직선 이동의 목적 구분
- 고정 환경에서 검증한 좌표 사용
- 동작 단계마다 API 반환값과 완료 상태 확인

## 5. OpenCV, NumPy, MediaPipe, OCR 의존성 충돌

### Issue

MediaPipe, ROS `cv_bridge`, OCR package가 요구하는 NumPy/OpenCV 조합이 달라 통합 실행 시 import 오류가 발생했습니다.

### Applied Environment

- Python 3.10
- NumPy 1.24.4
- OpenCV Contrib 4.10.0.84
- MediaPipe 0.10.35

`opencv-python`, `opencv-python-headless`, `opencv-contrib-python`을 동시에 설치하지 않고 contrib package 하나만 사용하는 방식으로 정리했습니다.

## 6. 통합 시연 중 CPU 과부하

### Issue

여러 카메라, OCR, MediaPipe, 얼굴 인증, 키오스크 렌더링, 화면 녹화를 동시에 실행하면서 CPU 병목이 발생했습니다.

### Solution

- OCR 호출 주기 제한
- 얼굴 노드의 OpenCV thread 수 제한
- 필요 시점에만 카메라와 OCR 활성화
- 화면 출력·녹화 부하를 다른 장치로 분산

### Lesson

개별 기능이 정상 동작해도 통합 시에는 CPU, 카메라 점유, memory copy, GUI rendering 비용을 함께 측정해야 합니다.

## 7. 공개 저장소 정리

원본 workspace에는 `.env`, DB, 여권 이미지, 얼굴 이미지가 포함되어 있었습니다. 구현 경험을 공개하더라도 개인정보와 secret은 코드 근거와 분리해야 합니다.
