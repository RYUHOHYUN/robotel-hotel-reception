# Test Results and Evidence Boundary

## 결과보고서에 기록된 수치

| Test item | Recorded result | Scope |
|---|---:|---|
| 동일인 얼굴 인증 | 20/20 통과 | 얼굴 검증 기능 |
| 타인 얼굴 인증 | 통과 0/20 | 얼굴 검증 기능 |
| 여권 평면 사진 | 통과 0/20 | RGB-D liveness |
| 카드키 OCR | 20/20 | 팀 통합 기능 |
| 로봇 카드키 반환·물품 전달 | 20/20 | 팀 통합 기능 |
| 여권 OCR 완전 일치 | 60/150 | 다른 팀 기능 포함 |
| 여권 OCR N/M 제외 | 142/150 | 다른 팀 기능 포함 |

## 얼굴 인증 해석

얼굴 인증은 SFace similarity만으로 성공 처리하지 않고 RealSense Depth liveness를 함께 만족해야 성공하도록 구성했습니다.

- 동일인: similarity와 liveness 조건을 함께 통과
- 타인: similarity가 낮아 동일인으로 인정하지 않음
- 여권 사진: 평면 Depth 조건으로 통과하지 않음

## Threshold 기록 차이

결과보고서에는 테스트 당시 얼굴 인식 최종 임계값으로 `0.554`가 적혀 있습니다.

제공된 source snapshot은 환경변수 기본값으로 다음 값을 사용합니다.

```text
FACE_SAME_THRESHOLD=0.40
FACE_DIFFERENT_THRESHOLD=0.32
```

두 자료의 생성 시점이나 tuning 상태가 동일하다는 근거가 없으므로, 이 저장소에서는 다음과 같이 구분합니다.

- `0.554`: 결과보고서에 기록된 테스트 당시 값
- `0.40 / 0.32`: 제공된 source snapshot의 기본값

실제 재현 시에는 동일한 카메라, 거리, 조명, 여권 사진 crop 조건에서 validation set을 다시 구성해 threshold를 결정해야 합니다.

## Prize Rank OCR Limitation

결과보고서에는 PaddleOCR 기반 등수 판독을 단위 기능으로 검증했다고 기록되어 있으나 별도의 정확도, confusion matrix, 반복 횟수는 제공되지 않았습니다.

따라서 다음 표현은 사용하지 않습니다.

- “등수 OCR 정확도 100%”
- “모든 환경에서 인식 성공”
- 근거 없는 latency 또는 FPS 수치

대신 코드에서 확인되는 안정화 방식과 실제 통합 경험을 설명합니다.

## Team-level Result Boundary

카드키 OCR 20/20과 로봇 동작 20/20은 전체 팀 시나리오에서 기록된 결과입니다. 이 수치를 개인 단독 구현의 독립 벤치마크로 표현하지 않습니다.
