#!/usr/bin/env python3
# 개인 기여 코드: 카드키 좌표 기반 이동

"""Doosan M0609과 OnRobot RG2를 이용한 번호별 pick-and-place 작업."""

import argparse
import time

import rclpy
import DR_init

from onrobot_card import RG


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_VELOCITY = 60
ROBOT_ACCELERATION = 60

GRIPPER_MODEL = "rg2"
GRIPPER_IP = "192.168.1.1"
GRIPPER_PORT = 502
GRIPPER_WAIT_SEC = 2.0

INITIAL_X = [367.64, 7.52, 192.00, 90.45, 179.97, 90.30]
INITIAL_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# 모든 객실 카드가 놓이는 공통 트레이 위치입니다.
# 먼저 트레이 위쪽으로 이동한 뒤 Z축 방향으로만 내려갑니다.
TRAY_ABOVE_X = [387.67, -52.58, 180.0, 13.64, -165.08, 12.37]
TRAY_BELOW_X = [387.67, -52.58, 80.31, 13.64, -165.08, 12.37]

# 각 작업의 x는 posx, j는 티칭 시 함께 기록한 posj 좌표입니다.
# 현재 시퀀스는 초기 위치에만 posj를 사용하고 작업 위치에는 posx를 사용합니다.
TASKS = {
    "601": {
        "pick_above": {"x": [17.503, -378.3, 212.467, 87.866, -170.683, 176.65], "j": [-87.802, -6.381, 93.527, -0.745, 83.566, 1.157]},
        "pick": {"x": [15.953, -406.483, 47.278, 87.241, -170.808, 175.95], "j": [-88.141, 1.958, 111.924, -0.945, 56.962, 1.172]},
        "pick_lift": {"x": [17.503, -378.3, 212.467, 87.866, -170.683, 176.65], "j": [-87.802, -6.381, 93.527, -0.745, 83.566, 1.157]},
        "place_above": {"x": TRAY_ABOVE_X},
        "place_below": {"x": TRAY_BELOW_X},
        "release_force": 40.0,
    },
    "602": {
        "pick_above": {"x": [86.82, -379.692, 219.872, 87.222, -171.77, 176.292], "j": [-76.052, -3.612, 89.711, -2.412, 86.025, 13.17]},
        "pick": {"x": [85.331, -406.9, 47.881, 86.666, -171.813, 175.68], "j": [-77.243, 4.168, 109.308, -2.716, 58.691, 13.175]},
        "pick_lift": {"x": [86.82, -379.692, 219.872, 87.222, -171.77, 176.292], "j": [-76.052, -3.612, 89.711, -2.412, 86.025, 13.17]},
        "place_above": {"x": TRAY_ABOVE_X},
        "place_below": {"x": TRAY_BELOW_X},
        "release_force": 20.0,
    },
    "603": {
        "pick_above": {"x": [18.55, -416.954, 212.71, 86.828, -170.637, 175.529], "j": [-87.703, -1.01, 88.511, -0.951, 83.181, 1.185]},
        "pick": {"x": [16.863, -445.457, 42.446, 86.942, -170.465, 175.589], "j": [-88.129, 7.264, 106.979, -1.067, 56.264, 1.187]},
        "pick_lift": {"x": [18.55, -416.954, 212.71, 86.828, -170.637, 175.529], "j": [-87.703, -1.01, 88.511, -0.951, 83.181, 1.185]},
        "place_above": {"x": TRAY_ABOVE_X},
        "place_below": {"x": TRAY_BELOW_X},
        "release_force": 20.0,
    },
    "604": {
        "pick_above": {"x": [90.51, -423.972, 212.658, 82.881, -172.504, 171.56], "j": [-76.797, 3.208, 83.946, -2.667, 85.824, 12.063]},
        "pick": {"x": [87.234, -445.682, 47.257, 83.564, -172.306, 172.201], "j": [-77.963, 9.778, 102.906, -2.891, 60.053, 12.108]},
        "pick_lift": {"x": [90.51, -423.972, 212.658, 82.881, -172.504, 171.56], "j": [-76.797, 3.208, 83.946, -2.667, 85.824, 12.063]},
        "place_above": {"x": TRAY_ABOVE_X},
        "place_below": {"x": TRAY_BELOW_X},
        "release_force": 20.0,
    },
    "701": {
        "pick_above": {"x": [22.047, -462.474, 199.067, 82.678, -172.324, 171.792], "j": [-87.141, 6.841, 82.099, -1.437, 83.509, 2.19]},
        "pick": {"x": [18.726, -483.446, 46.319, 82.397, -172.242, 171.427], "j": [-87.682, 13.462, 98.495, -1.631, 60.413, 2.205]},
        "pick_lift": {"x": [22.047, -462.474, 199.067, 82.678, -172.324, 171.792], "j": [-87.141, 6.841, 82.099, -1.437, 83.509, 2.19]},
        "place_above": {"x": TRAY_ABOVE_X},
        "place_below": {"x": TRAY_BELOW_X},
        "release_force": 20.0,
    },
    "702": {
        "pick_above": {"x": [90.075, -459.126, 202.725, 82.541, -172.738, 171.812], "j": [-77.967, 8.154, 79.974, -2.504, 85.035, 11.52]},
        "pick": {"x": [87.693, -479.455, 47.639, 80.773, -173.374, 169.896], "j": [-78.832, 14.965, 96.182, -2.692, 62.671, 11.542]},
        "pick_lift": {"x": [90.075, -459.126, 202.725, 82.541, -172.738, 171.812], "j": [-77.967, 8.154, 79.974, -2.504, 85.035, 11.52]},
        "place_above": {"x": TRAY_ABOVE_X},
        "place_below": {"x": TRAY_BELOW_X},
        "release_force": 20.0,
    },
}

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def parse_args(args=None):
    parser = argparse.ArgumentParser(description="번호별 pick-and-place 작업")
    parser.add_argument(
        "task_id",
        nargs="?",
        choices=sorted(TASKS),
        help="실행할 작업 번호 (미입력 시 터미널에서 선택)",
    )
    return parser.parse_known_args(args)


def select_task(task_id):
    if task_id is None:
        available = ", ".join(sorted(TASKS))
        task_id = input(f"작업 번호를 입력하세요 ({available}): ").strip()
    if task_id not in TASKS:
        raise ValueError(
            f"지원하지 않는 작업 번호입니다: {task_id!r}. "
            f"가능한 번호: {', '.join(sorted(TASKS))}"
        )
    return task_id, TASKS[task_id]


def command_gripper(gripper, width_mm, force_n):
    """mm와 N 단위의 값을 RG2 레지스터 단위(0.1 mm, 0.1 N)로 변환합니다."""
    width_value = round(width_mm * 10)
    force_value = round(force_n * 10)

    if not 0 <= width_value <= gripper.max_width:
        raise ValueError(f"Gripper width out of range: {width_mm} mm")
    if not 0 <= force_value <= gripper.max_force:
        raise ValueError(f"Gripper force out of range: {force_n} N")

    print(f"Gripper: width={width_mm} mm, force={force_n} N")
    gripper.move_gripper(width_val=width_value, force_val=force_value)
    time.sleep(GRIPPER_WAIT_SEC)


def main(args=None):
    parsed_args, ros_args = parse_args(args)
    task_id, task = select_task(parsed_args.task_id)

    rclpy.init(args=ros_args)
    node = rclpy.create_node("card_move_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    gripper = None

    try:
        from DSR_ROBOT2 import DR_BASE, movej, movel, mwait
        from DR_common2 import posj, posx

        gripper = RG(GRIPPER_MODEL, GRIPPER_IP, GRIPPER_PORT)

        def move_joint(name, joint):
            print(f"Move: {name}")
            movej(posj(joint), vel=ROBOT_VELOCITY, acc=ROBOT_ACCELERATION)
            mwait()

        def move_linear(name, position):
            print(f"Move: {name}")
            movel(
                posx(position),
                vel=ROBOT_VELOCITY,
                acc=ROBOT_ACCELERATION,
                ref=DR_BASE,
            )
            mwait()

        print(f"Start task: {task_id}")

        # 첨부된 티칭 순서와 그리퍼 설정을 그대로 실행합니다.
        move_joint("initial", INITIAL_J)
        command_gripper(gripper, width_mm=30.0, force_n=40.0)

        move_linear("pick above", task["pick_above"]["x"])
        command_gripper(gripper, width_mm=30.0, force_n=40.0)

        move_linear("pick", task["pick"]["x"])
        command_gripper(gripper, width_mm=30.0, force_n=40.0)

        command_gripper(gripper, width_mm=4.0, force_n=40.0)
        move_linear("pick lift", task["pick_lift"]["x"])

        # 카드 파지 후 먼저 기준 관절 자세로 복귀합니다.
        # 트레이 근처에서 회전하며 내려가지 않도록 위쪽 접근과 수직 하강을 분리합니다.
        move_joint("tray approach initial", INITIAL_J)
        move_linear("place above", task["place_above"]["x"])
        command_gripper(gripper, width_mm=4.0, force_n=40.0)
        move_linear("place below", task["place_below"]["x"])
        command_gripper(
            gripper, width_mm=70.0, force_n=task["release_force"]
        )

        move_joint("initial", INITIAL_J)
        command_gripper(
            gripper, width_mm=70.0, force_n=task["release_force"]
        )

        print(f"Task {task_id} completed.")
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        if gripper is not None:
            gripper.close_connection()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
