# Source Evidence

이 폴더의 코드는 사용자가 제공한 ROBOTEL ROS 2 workspace에서 개인 기여와 직접 관련된 구현 파일을 발췌한 것입니다.

| File | Purpose |
|---|---|
| `realtime_rank_ocr_paddle.py` | PaddleOCR 기반 뽑기 등수 판독 |
| `verification_node.py` | YuNet + SFace + RealSense Depth 얼굴 본인 확인 |
| `event_prize_delivery_node.py` | 고정 좌표 기반 3등 경품 전달 |
| `passport_robot_motion_node.py` | 비전 좌표 입력과 고정 경유점을 이용한 여권 pick/place |

## Execution Limitation

이 파일만 복사해서는 전체 시스템이 실행되지 않습니다. 실제 실행에는 다음 항목이 필요합니다.

- ROS 2 Humble
- `checkin_interfaces` package
- OpenCV YuNet/SFace model files
- PaddleOCR와 장비 환경에 맞는 PaddlePaddle
- Intel RealSense ROS 2 topics
- Doosan Robotics ROS 2 package와 bringup
- OnRobot RG2 wrapper
- 원본 package의 `setup.py`, `package.xml`, launch configuration

공개 저장소에는 개인정보, DB, `.env`, 촬영 결과 이미지, 외부 model binary를 포함하지 않았습니다.
