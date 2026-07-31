#!/usr/bin/env python3
"""Doosan passport pick, safe release, and face-scan pose services."""

from __future__ import annotations

import math

import rclpy
from checkin_interfaces.srv import PickPassport
from rclpy.node import Node
from std_srvs.srv import Trigger

import DR_init


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

GRIP_OUTPUT = 1
UNGRIP_OUTPUT = 2
FIXED_RX = 0.0
FIXED_RY = 155.0
FIXED_RZ = 0.0

# 이 노드에서 실행하는 모든 movej/movel의 속도와 가속도를 50으로 통일합니다.
MOVE_VELOCITY = 50
MOVE_ACCELERATION = 50
RELEASE_DESCENT_VELOCITY = 50
RELEASE_DESCENT_ACCELERATION = 50

GRIP_WAIT_SEC = 1.0
PASSPORT_WAIT_SEC = 3.0
FACE_CAMERA_SETTLE_SEC = 0.3

PICK_SERVICE = "/checkin/pick_passport"
MOVE_TO_GRASP_DETECTION_SERVICE = "/checkin/move_to_passport_detection_pose"
# 기존 서비스 이름은 다른 패키지와의 호환성을 위해 유지합니다.
RELEASE_AND_FACE_SERVICE = "/checkin/release_passport_and_move_to_face"

# 체크인 대기 중 사람을 처음 감지할 때 사용하는 자세입니다.
# 여권을 내려놓은 뒤 얼굴 스캔도 이 자세에서 수행합니다.
INITIAL_DETECTION_JOINT_POS = [0.0, 10.0, 90.0, 0.0, 0.0, 90.0]
PASSPORT_DETECTION_JOINT_POS = [0.0, 0.0, 90.0, 0.0, 65.0, 90.0]
AFTER_GRIP_JOINT_POS = [-17.42 ,17.69 , 56.29 , 1.61 , 99.80, 18.19 ] 

# 여권 사진 저장 및 사람 매칭 비교가 끝난 뒤 사용하는 트레이 좌표입니다.
# 사진 촬영 자세에서 별도의 중간 movej 없이, BASE 기준 안전 높이로
# 바로 이동한 다음 트레이 놓기 좌표까지 직선으로 하강합니다.
TRAY_ABOVE_LINEAR_POSE = [514.660, -4.010, 250.390, 19.57, -171.91, -159.3]
TRAY_INSERT_LINEAR_POSE = [514.660, -4.010, 150.390, 19.57, -171.91, -159.3]


def grip() -> None:
    set_digital_output(GRIP_OUTPUT, OFF)
    set_digital_output(UNGRIP_OUTPUT, OFF)
    set_digital_output(GRIP_OUTPUT, ON)


def gripper_open() -> None:
    set_digital_output(GRIP_OUTPUT, OFF)
    set_digital_output(UNGRIP_OUTPUT, OFF)
    set_digital_output(UNGRIP_OUTPUT, ON)


class PassportRobotMotionNode(Node):
    def __init__(self) -> None:
        super().__init__("passport_robot_motion_node")
        self.pick_service = self.create_service(
            PickPassport,
            PICK_SERVICE,
            self.pick_callback,
        )
        self.move_to_detection_service = self.create_service(
            Trigger,
            MOVE_TO_GRASP_DETECTION_SERVICE,
            self.move_to_detection_callback,
        )
        self.release_and_face_service = self.create_service(
            Trigger,
            RELEASE_AND_FACE_SERVICE,
            self.release_and_move_to_face_callback,
        )
        self.passport_in_gripper = False
        self.get_logger().info(f"여권 파지 서비스: {PICK_SERVICE}")
        self.get_logger().info(
            "여권 안전 내려놓기/사람 감지 자세 복귀 서비스: "
            f"{RELEASE_AND_FACE_SERVICE}"
        )

    def move_to_detection_callback(self, request, response):
        del request
        try:
            result = movej(
                posj(PASSPORT_DETECTION_JOINT_POS),
                vel=MOVE_VELOCITY,
                acc=MOVE_ACCELERATION,
            )
            response.success = result == 0
            response.message = (
                "여권 위치 인식 자세 이동 완료"
                if response.success
                else f"여권 위치 인식 자세 movej 실패(return={result})"
            )
        except Exception as exc:
            response.success = False
            response.message = f"여권 위치 인식 자세 이동 오류: {exc}"
        self._log_response(response)
        return response

    def pick_callback(self, request, response):
        x, y, z = float(request.x), float(request.y), float(request.z)
        try:
            if not all(math.isfinite(value) for value in (x, y, z)):
                raise ValueError("유효하지 않은 여권 파지 좌표입니다.")

            self.get_logger().info(
                f"파지 요청: x={x:.2f}, y={y:.2f}, z={z:.2f} mm"
            )
            gripper_open()
            wait(PASSPORT_WAIT_SEC)

            result = movel(
                posx([x, y, z, FIXED_RX, FIXED_RY, FIXED_RZ]),
                vel=MOVE_VELOCITY,
                acc=MOVE_ACCELERATION,
            )
            if result != 0:
                raise RuntimeError(f"여권 접근 movel 실패(return={result})")

            grip()
            self.passport_in_gripper = True
            wait(GRIP_WAIT_SEC)

            result = movej(
                posj(AFTER_GRIP_JOINT_POS),
                vel=MOVE_VELOCITY,
                acc=MOVE_ACCELERATION,
            )
            if result != 0:
                raise RuntimeError(f"웹캠 앞 이동 실패(return={result})")

            response.success = True
            response.message = "여권 파지 및 웹캠 앞 이동 완료"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._log_response(response)
        return response

    def release_and_move_to_face_callback(self, request, response):
        """여권을 안전하게 놓은 뒤 최초 사람 감지 자세로 복귀합니다."""
        del request
        try:
            if not self.passport_in_gripper:
                self.get_logger().warning(
                    "현재 파지 중인 여권이 없습니다. 얼굴 스캔 자세만 확인합니다."
                )
                result = movej(
                    posj(INITIAL_DETECTION_JOINT_POS),
                    vel=MOVE_VELOCITY,
                    acc=MOVE_ACCELERATION,
                )
                if result != 0:
                    raise RuntimeError(
                        f"사람 감지 자세 복귀 실패(return={result})"
                    )
                response.success = True
                response.message = "이미 내려놓기 완료, 얼굴 스캔 자세 준비 완료"
                self._log_response(response)
                return response

            self.get_logger().info(
                "사진 촬영 자세에서 트레이 안전 높이로 바로 이동합니다: "
                f"{TRAY_ABOVE_LINEAR_POSE}"
            )
            result = movel(
                posx(TRAY_ABOVE_LINEAR_POSE),
                vel=MOVE_VELOCITY,
                acc=MOVE_ACCELERATION,
                ref=DR_BASE,
            )
            if result != 0:
                raise RuntimeError(f"트레이 안전 높이 movel 실패(return={result})")

            self.get_logger().info(
                "트레이 꽂기 좌표까지 직선 이동합니다: "
                f"{TRAY_INSERT_LINEAR_POSE}"
            )
            result = movel(
                posx(TRAY_INSERT_LINEAR_POSE),
                vel=RELEASE_DESCENT_VELOCITY,
                acc=RELEASE_DESCENT_ACCELERATION,
                ref=DR_BASE,
            )
            if result != 0:
                raise RuntimeError(f"트레이 꽂기 movel 실패(return={result})")

            gripper_open()
            self.passport_in_gripper = False
            wait(GRIP_WAIT_SEC)
            self.get_logger().info("그리퍼를 열어 여권을 트레이에 놓았습니다.")

            self.get_logger().info(
                "트레이 안전 높이까지 직선으로 빠져나옵니다: "
                f"{TRAY_ABOVE_LINEAR_POSE}"
            )
            result = movel(
                posx(TRAY_ABOVE_LINEAR_POSE),
                vel=MOVE_VELOCITY,
                acc=MOVE_ACCELERATION,
                ref=DR_BASE,
            )
            if result != 0:
                raise RuntimeError(f"트레이 이탈 movel 실패(return={result})")

            self.get_logger().info(
                "최초 사람 감지 자세로 복귀하여 얼굴 스캔을 준비합니다: "
                f"{INITIAL_DETECTION_JOINT_POS}"
            )
            result = movej(
                posj(INITIAL_DETECTION_JOINT_POS),
                vel=MOVE_VELOCITY,
                acc=MOVE_ACCELERATION,
            )
            if result != 0:
                raise RuntimeError(f"사람 감지 자세 복귀 실패(return={result})")

            if FACE_CAMERA_SETTLE_SEC > 0:
                wait(FACE_CAMERA_SETTLE_SEC)

            response.success = True
            response.message = (
                "여권 내려놓기 완료 및 최초 사람 감지 자세 얼굴 스캔 준비 완료"
            )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self._log_response(response)
        return response

    def _log_response(self, response) -> None:
        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    dsr_node = rclpy.create_node(
        "passport_robot_motion_dsr_node",
        namespace=ROBOT_ID,
    )
    DR_init.__dsr__node = dsr_node
    motion_node = None
    try:
        global movel, movej, posx, posj, wait
        global set_digital_output, ON, OFF, DR_BASE
        from DSR_ROBOT2 import (
            DR_BASE,
            OFF,
            ON,
            movej,
            movel,
            posj,
            posx,
            set_digital_output,
            wait,
        )

        dsr_node.get_logger().info("사람 감지 초기 자세로 이동합니다.")
        result = movej(
            posj(INITIAL_DETECTION_JOINT_POS),
            vel=MOVE_VELOCITY,
            acc=MOVE_ACCELERATION,
        )
        if result != 0:
            raise RuntimeError(f"초기 자세 movej 실패(return={result})")

        motion_node = PassportRobotMotionNode()
        rclpy.spin(motion_node)
    except KeyboardInterrupt:
        pass
    finally:
        if motion_node is not None:
            motion_node.destroy_node()
        dsr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
