# Media Placeholder

다음과 같은 비식별 자료를 추가하면 README의 전달력이 좋아집니다.

- `demo_preview.gif`: 등수 OCR → 로봇 경품 전달 짧은 시연
- `rank_ocr_overlay.png`: ROI와 안정 판정이 보이는 화면
- `face_verification_overlay.png`: 얼굴 box, similarity, liveness 상태 화면
- `robot_waypoint_sequence.png`: 고정 좌표 이동 경로 도식
- `system_architecture.png`: 전체 시스템 구성도

## Privacy Rules

- 실제 여권, 이름, 여권번호, 생년월일은 모자이크 처리
- 얼굴 공개 동의가 없는 사람은 식별 불가능하게 처리
- 예약번호, 객실 배정, DB 화면은 샘플 데이터로 교체
- 원본 프로젝트의 `src/output/` 이미지를 그대로 업로드하지 않기
