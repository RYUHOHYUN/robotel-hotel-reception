#!/usr/bin/env python3
# 개인 기여 코드: 3등 상품 키링 좌표 기반 이동
"""Deliver the 3rd-prize keyring using the verified figures_move.py motion."""

from __future__ import annotations

import threading
import time

import DR_init
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from onrobot_rg2 import RG2


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_VELOCITY = 60
ROBOT_ACCELERATION = 60

GRIPPER_MODEL = "rg2"
GRIPPER_IP = "192.168.1.1"
GRIPPER_PORT = 502
GRIPPER_WAIT_SEC = 2.0

# Verified coordinates copied without modification from figures_move.py.
INITIAL = {
    "x": [367.449, 7.678, 192.104, 60.196, 179.958, 60.032],
    "j": [0.022, -0.044, 90.028, 0.003, 89.995, 0.011],
}
PRODUCT_FRONT = {
    "x": [-96.437, -364.13, 212.352, 178.354, 90.036, 86.129],
    "j": [-56.474, 30.784, 107.594, -62.144, 112.635, -129.788],
}
PRODUCT = {
    "x": [-161.352, -360.457, 209.142, 177.558, 90.112, 85.687],
    "j": [-63.947, 27.988, 113.419, -66.882, 107.393, -129.293],
}
PULL_OUT = {
    "x": [-96.437, -364.13, 212.352, 178.354, 90.036, 86.129],
    "j": [-56.474, 30.784, 107.594, -62.144, 112.635, -129.788],
}
BASKET_BOTTOM = {
    "x": [585.44, 200.53, 188.79, 164.39, -176.62, -106.18],
    "j": [-52.79, 45.11, 30.27, 1.47, 101.42, 124.38],
}
BASKET_TOP = {
    "x": [585.44, 200.53, 338.79, 164.39, -176.62, -106.18],
    "j": [-52.79, 45.11, 30.27, 1.47, 101.42, 124.38],
}

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class EventPrizeDeliveryNode(Node):
    def __init__(self) -> None:
        super().__init__("event_prize_delivery_node")
        self.declare_parameter("service_name", "/event/deliver_keyring")
        self.declare_parameter("dry_run", False)
        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._motion_lock = threading.Lock()
        # DSR_ROBOT2는 import 순간 DR_init.__dsr__node를 사용합니다.
        # 서비스 콜백을 처리하는 메인 노드와 DSR 동기 API 노드를 분리해
        # 중첩 spin을 피하고, import 전에 반드시 전용 노드를 등록합니다.
        self._dsr_node = Node(
            "event_prize_dsr_api",
            namespace=ROBOT_ID,
            context=self.context,
        )
        setattr(DR_init, "__dsr__id", ROBOT_ID)
        setattr(DR_init, "__dsr__model", ROBOT_MODEL)
        setattr(DR_init, "__dsr__node", self._dsr_node)
        if getattr(DR_init, "__dsr__node", None) is not self._dsr_node:
            raise RuntimeError("DR_init.__dsr__node 설정에 실패했습니다.")

        from DSR_ROBOT2 import DR_BASE, movej, movel, mwait
        from DR_common2 import posj, posx

        self._dr_base = DR_BASE
        self._movej = movej
        self._movel = movel
        self._mwait = mwait
        self._posj = posj
        self._posx = posx

        self.create_service(
            Trigger,
            str(self.get_parameter("service_name").value),
            self._delivery_callback,
        )
        self.get_logger().info(
            f"3등 상품 전달 서비스 준비 완료: dry_run={self._dry_run}, "
            f"service={self.get_parameter('service_name').value}"
        )

    @staticmethod
    def _command_gripper(gripper: RG2, width_mm: float, force_n: float) -> None:
        gripper.move_gripper_mm_n(width_mm=width_mm, force_n=force_n)
        time.sleep(GRIPPER_WAIT_SEC)

    def _run_delivery(self) -> None:
        if self._dry_run:
            self.get_logger().warning("dry_run=true: 3등 상품 전달 로봇 명령을 생략합니다.")
            time.sleep(1.0)
            return

        gripper = RG2(GRIPPER_IP, GRIPPER_PORT)
        try:
            def move_joint(name: str, joint: list[float]) -> None:
                self.get_logger().info(f"3등 상품 이동: {name}")
                self._movej(
                    self._posj(joint),
                    vel=ROBOT_VELOCITY,
                    acc=ROBOT_ACCELERATION,
                )
                self._mwait()

            def move_linear(name: str, position: list[float]) -> None:
                self.get_logger().info(f"3등 상품 이동: {name}")
                self._movel(
                    self._posx(position),
                    vel=ROBOT_VELOCITY,
                    acc=ROBOT_ACCELERATION,
                    ref=self._dr_base,
                )
                self._mwait()

            # The sequence below is copied from the tested figures_move.py.
            move_joint("initial", INITIAL["j"])
            self._command_gripper(gripper, width_mm=16.0, force_n=40.0)

            move_linear("product front", PRODUCT_FRONT["x"])
            self._command_gripper(gripper, width_mm=16.0, force_n=40.0)

            move_linear("product", PRODUCT["x"])
            self._command_gripper(gripper, width_mm=4.0, force_n=40.0)

            move_linear("pull out", PULL_OUT["x"])
            self._command_gripper(gripper, width_mm=4.0, force_n=40.0)

            move_joint("initial", INITIAL["j"])
            self._command_gripper(gripper, width_mm=4.0, force_n=40.0)

            move_linear("basket top", BASKET_TOP["x"])
            self._command_gripper(gripper, width_mm=4.0, force_n=40.0)

            move_linear("basket bottom", BASKET_BOTTOM["x"])
            self._command_gripper(gripper, width_mm=70.0, force_n=40.0)
        finally:
            gripper.close_connection()

    def _delivery_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if not self._motion_lock.acquire(blocking=False):
            response.success = False
            response.message = "3등 상품 전달 작업이 이미 진행 중입니다."
            return response
        try:
            self.get_logger().info("3등 상품 전달을 시작합니다.")
            self._run_delivery()
            response.success = True
            response.message = "3등 상품 전달을 완료했습니다."
            self.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = f"3등 상품 전달 실패: {type(exc).__name__}: {exc}"
            self.get_logger().error(response.message)
        finally:
            self._motion_lock.release()
        return response

    def destroy_node(self) -> bool:
        try:
            self._dsr_node.destroy_node()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EventPrizeDeliveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
