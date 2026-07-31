import math
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from ultralytics import YOLO

import DR_init
from checkin_interfaces.srv import GetPassportGraspPoint


# ============================================================
# 로봇/카메라 기본 설정
# ============================================================

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


class PassportGraspPoseNode(Node):
    """
    여권과 여권 내부 사진을 YOLO Segmentation으로 검출하고,
    로봇이 잡을 위치 및 자세를 반복 계산하고 최신 Base XYZ를
    서비스 응답으로 제공하는 노드.

    이 노드는 로봇을 움직이지 않는다.

    출력:
        x, y, z   : 로봇 Base 기준 파지 위치(mm)
        rx, ry    : 현재 Tool Flange의 rx, ry를 그대로 사용
        rz        : 현재 Tool Flange rz + 사진 마스크의 평면 회전 보정값
    """

    # ========================================================
    # 모델/클래스 설정
    # ========================================================

    MODEL_FILENAME = "passport_best.pt"

    PASSPORT_CLASS_NAME = "passport"
    PHOTO_CLASS_NAME = "photo"

    CONFIDENCE_THRESHOLD = 0.50
    YOLO_IMAGE_SIZE = 640

    # 카메라가 거꾸로 장착된 경우 RGB와 Depth를 함께 회전한다.
    ROTATE_CAMERA_180 = False

    # ========================================================
    # 반복 출력 설정
    # ========================================================

    # 유효 결과를 이 시간 간격으로 반복 출력한다.
    OUTPUT_INTERVAL_SEC = 0.5

    # 0.1초는 "총 인식시간"이 아니라 카메라를 확인하는 주기입니다.
    # 서비스 요청 후 아래 안정화 시간 동안 여러 좌표를 모아 최종 위치를 결정합니다.
    DETECTION_TIMER_SEC = 0.1
    PASSPORT_APPEAR_TIMEOUT_SEC = 30.0
    STABILITY_TIMEOUT_SEC = 5.0
    STABILITY_HISTORY_SEC = 1.8
    STABILITY_MIN_DURATION_SEC = 1.0
    STABILITY_MIN_SAMPLE_COUNT = 8
    STABILITY_MAX_XY_DEVIATION_MM = 8.0
    STABILITY_MAX_Z_DEVIATION_MM = 12.0

    # ========================================================
    # 파지점 설정
    # ========================================================

    # 선택한 여권 끝변 중앙에서 여권 중심 방향으로 들어가는 비율
    # 0.00: 정확히 끝선
    # 0.05: 끝선에서 중심 방향으로 5% 이동
    GRASP_INWARD_RATIO = 0.10

    # Depth를 읽을 때 대표 픽셀 주변 탐색 반경
    DEPTH_SAMPLE_RADIUS = 4

    MIN_DEPTH_MM = 100.0
    MAX_DEPTH_MM = 2000.0

    # ========================================================
    # 자세 보정 설정
    # ========================================================

    # 사진 마스크에서 측정한 각도에 곱할 부호
    # 실제 로봇에서 방향이 반대로 움직이면 -1.0으로 변경한다.
    RZ_SIGN = 1.0

    # 사진 각도 외에 추가할 고정 보정값(deg)
    RZ_OFFSET_DEG = 0.0

    # 한 번의 측정에서 허용할 최대 Rz 보정량
    MAX_ABS_RZ_CORRECTION_DEG = 45.0

    # ========================================================
    # 표시 설정
    # ========================================================

    WINDOW_NAME = "Passport Grasp Pose"
    SHOW_DEBUG_WINDOW = False

    GRASP_POINT_SERVICE = "/checkin/get_passport_grasp_point"

    def __init__(self):
        super().__init__("robotel_passport_grasp_pose_node")

        package_share = Path(get_package_share_directory("hotel_vision"))

        self.model_path = (
            package_share
            / "models"
            / self.MODEL_FILENAME
        )

        self.transform_path = (
            package_share
            / "calibration"
            / "T_flange_camera.npy"
        )

        self.model = None
        self.flange_camera = None

        self.bridge = CvBridge()

        self.color_frame: Optional[np.ndarray] = None
        self.depth_frame: Optional[np.ndarray] = None
        self.intrinsics = None

        self.last_output_time = 0.0
        self.latest_base_position: Optional[np.ndarray] = None
        self.stable_base_position: Optional[np.ndarray] = None
        self.first_valid_sample_time: Optional[float] = None
        self.detection_requested = False
        self.detection_in_progress = False

        self.frame_lock = threading.Lock()
        self.sample_condition = threading.Condition()
        self.position_samples: List[Tuple[float, np.ndarray]] = []
        self.service_callback_group = ReentrantCallbackGroup()
        self.detection_callback_group = MutuallyExclusiveCallbackGroup()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.grasp_point_service = self.create_service(
            GetPassportGraspPoint,
            self.GRASP_POINT_SERVICE,
            self.get_grasp_point_callback,
            callback_group=self.service_callback_group,
        )

        self.color_subscription = self.create_subscription(
            msg_type=Image,
            topic="/camera/camera/color/image_raw",
            callback=self.color_callback,
            qos_profile=10,
        )

        self.depth_subscription = self.create_subscription(
            msg_type=Image,
            topic="/camera/camera/aligned_depth_to_color/image_raw",
            callback=self.depth_callback,
            qos_profile=10,
        )

        self.camera_info_subscription = self.create_subscription(
            msg_type=CameraInfo,
            topic="/camera/camera/color/camera_info",
            callback=self.camera_info_callback,
            qos_profile=10,
        )

        self.timer = self.create_timer(
            self.DETECTION_TIMER_SEC,
            self.detect_and_print,
            callback_group=self.detection_callback_group,
        )

        self.get_logger().info(
            "여권 파지 좌표 서비스 요청을 기다립니다."
        )
        self.get_logger().info(
            "이 노드는 로봇을 움직이지 않고 "
            "x, y, z, rx, ry, rz만 반복 출력합니다."
        )
        self.get_logger().info(
            f"출력 간격: {self.OUTPUT_INTERVAL_SEC:.2f}초"
        )
        self.get_logger().info(
            "파지 좌표 결정 방식: "
            f"여권 대기 최대 {self.PASSPORT_APPEAR_TIMEOUT_SEC:.0f}초, "
            f"감지 후 안정화 최대 {self.STABILITY_TIMEOUT_SEC:.1f}초"
        )
        self.get_logger().info(
            f"여권 파지 좌표 서비스: {self.GRASP_POINT_SERVICE}"
        )
        self.get_logger().info(
            "ESC: 노드 종료"
        )

    def _validate_resource_files(self) -> None:
        if not self.model_path.exists():
            raise FileNotFoundError(
                "여권 세그멘테이션 모델이 없습니다.\n"
                f"다음 경로에 모델을 넣으세요:\n{self.model_path}"
            )

        if not self.transform_path.exists():
            raise FileNotFoundError(
                "손목-카메라 변환행렬이 없습니다.\n"
                f"다음 경로에 파일을 넣으세요:\n{self.transform_path}"
            )

    def _initialize_resources(self) -> None:
        if self.model is not None:
            return
        self._validate_resource_files()
        self.get_logger().info(
            f"여권 세그멘테이션 모델 로딩: {self.model_path}"
        )
        self.model = YOLO(str(self.model_path))
        self.flange_camera = np.load(str(self.transform_path))
        if self.flange_camera.shape != (4, 4):
            raise ValueError(
                "T_flange_camera.npy는 4x4 행렬이어야 합니다. "
                f"현재 shape={self.flange_camera.shape}"
            )

    def get_grasp_point_callback(self, request, response):
        del request

        self._initialize_resources()

        with self.sample_condition:
            if self.detection_in_progress:
                response.x = float("nan")
                response.y = float("nan")
                response.z = float("nan")
                self.get_logger().warning("여권 파지 위치 안정화가 이미 진행 중입니다.")
                return response

            self.detection_in_progress = True
            self.detection_requested = True
            self.latest_base_position = None
            self.stable_base_position = None
            self.first_valid_sample_time = None
            self.position_samples.clear()

        self.get_logger().info(
            "여권 위치 인식을 시작합니다. "
            f"여권이 화면에 들어올 때까지 최대 {self.PASSPORT_APPEAR_TIMEOUT_SEC:.0f}초 기다리고, "
            f"감지 후 최대 {self.STABILITY_TIMEOUT_SEC:.1f}초 동안 좌표를 안정화합니다."
        )

        started_at = time.monotonic()
        appearance_deadline = started_at + self.PASSPORT_APPEAR_TIMEOUT_SEC
        stable_position = None

        try:
            with self.sample_condition:
                while rclpy.ok():
                    if self.stable_base_position is not None:
                        stable_position = self.stable_base_position.copy()
                        break

                    now = time.monotonic()
                    if self.first_valid_sample_time is None:
                        deadline = appearance_deadline
                    else:
                        deadline = (
                            self.first_valid_sample_time
                            + self.STABILITY_TIMEOUT_SEC
                        )

                    remaining = deadline - now
                    if remaining <= 0:
                        break

                    self.sample_condition.wait(timeout=min(0.2, remaining))
        finally:
            with self.sample_condition:
                self.detection_requested = False
                self.detection_in_progress = False
                self.sample_condition.notify_all()

        if stable_position is None:
            response.x = float("nan")
            response.y = float("nan")
            response.z = float("nan")
            self.get_logger().warning(
                "안정된 여권 파지 좌표를 확정하지 못했습니다. "
                "여권 전체와 사진 면이 RealSense 화면에 보이도록 다시 놓아주세요."
            )
            return response

        response.x = float(stable_position[0])
        response.y = float(stable_position[1])
        response.z = float(stable_position[2])

        self.get_logger().info(
            "안정화된 여권 파지 좌표 서비스 응답: "
            f"x={response.x:.2f}, y={response.y:.2f}, "
            f"z={response.z:.2f} mm"
        )
        try:
            cv2.destroyWindow(self.WINDOW_NAME)
        except cv2.error:
            pass
        return response

    def color_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
            with self.frame_lock:
                self.color_frame = frame
        except Exception as error:
            self.get_logger().error(
                f"RGB 영상 변환 실패: {error}"
            )

    def depth_callback(self, msg: Image) -> None:
        try:
            depth_frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )

            if depth_frame.dtype == np.float32:
                depth_frame = depth_frame * 1000.0

            with self.frame_lock:
                self.depth_frame = np.asarray(depth_frame)

        except Exception as error:
            self.get_logger().error(
                f"Depth 영상 변환 실패: {error}"
            )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        ppx = float(msg.k[2])
        ppy = float(msg.k[5])

        if self.ROTATE_CAMERA_180:
            ppx = float(msg.width - 1) - ppx
            ppy = float(msg.height - 1) - ppy

        with self.frame_lock:
            self.intrinsics = {
                "fx": float(msg.k[0]),
                "fy": float(msg.k[4]),
                "ppx": ppx,
                "ppy": ppy,
            }

    def detect_and_print(self) -> None:
        if not self.detection_requested:
            return

        with self.sample_condition:
            if not self.detection_requested:
                return
            self._prune_position_samples(time.monotonic())

        with self.frame_lock:
            if self.intrinsics is None:
                return
            if self.color_frame is None:
                return
            if self.depth_frame is None:
                return
            color_frame = self.color_frame.copy()
            depth_frame = self.depth_frame.copy()

        if color_frame.shape[:2] != depth_frame.shape[:2]:
            self.get_logger().warning(
                "RGB와 Depth 해상도가 다릅니다: "
                f"RGB={color_frame.shape[:2]}, "
                f"Depth={depth_frame.shape[:2]}"
            )
            return

        if self.ROTATE_CAMERA_180:
            color_frame = cv2.rotate(
                color_frame,
                cv2.ROTATE_180,
            )
            depth_frame = cv2.rotate(
                depth_frame,
                cv2.ROTATE_180,
            )

        results = self.model.predict(
            source=color_frame,
            conf=self.CONFIDENCE_THRESHOLD,
            imgsz=self.YOLO_IMAGE_SIZE,
            verbose=False,
        )

        passport_detection, photo_detection = (
            self.select_passport_and_photo(results)
        )

        annotated = color_frame.copy()

        if passport_detection is None:
            self.draw_status(
                annotated,
                "passport not detected",
                (0, 0, 255),
            )
            self.show_frame(annotated)
            return

        if photo_detection is None:
            self.draw_status(
                annotated,
                "photo not detected",
                (0, 0, 255),
            )
            self.draw_polygon(
                annotated,
                passport_detection[1],
                (255, 0, 255),
            )
            self.show_frame(annotated)
            return

        passport_confidence, passport_polygon = passport_detection
        photo_confidence, photo_polygon = photo_detection

        passport_center = np.mean(passport_polygon, axis=0)
        photo_center = np.mean(photo_polygon, axis=0)

        # 카메라 영상 기준으로 여권사진이 왼쪽에 있으면 계산하지 않는다.
        if photo_center[0] <= passport_center[0]:
            self.draw_polygon(
                annotated,
                passport_polygon,
                (255, 0, 255),
            )
            self.draw_polygon(
                annotated,
                photo_polygon,
                (0, 255, 0),
            )
            self.draw_status(
                annotated,
                "PHOTO IS ON THE LEFT - ROTATE PASSPORT RIGHT",
                (0, 0, 255),
            )
            self.get_logger().error(
                "여권사진이 왼쪽에 있습니다. "
                "여권을 오른쪽으로 돌려주세요."
            )
            self.show_frame(annotated)
            return

        current_pose = self.get_current_flange_pose()
        base_to_flange = self.get_robot_pose_matrix(*current_pose)
        base_to_camera = base_to_flange @ self.flange_camera

        grasp_result = self.calculate_grasp_point(
            passport_polygon=passport_polygon,
            photo_polygon=photo_polygon,
            depth_frame=depth_frame,
            base_to_camera=base_to_camera,
        )

        if grasp_result is None:
            self.draw_status(
                annotated,
                "valid grasp depth not found",
                (0, 0, 255),
            )
            self.draw_polygon(
                annotated,
                passport_polygon,
                (255, 0, 255),
            )
            self.draw_polygon(
                annotated,
                photo_polygon,
                (0, 255, 0),
            )
            self.show_frame(annotated)
            return

        grasp_pixel, base_position, selected_edge = grasp_result
        position = np.asarray(base_position, dtype=float).copy()
        self.latest_base_position = position
        self._record_position_sample(position)

        photo_angle_deg = self.calculate_photo_angle(
            photo_polygon
        )

        rz_correction = (
            self.RZ_SIGN * photo_angle_deg
            + self.RZ_OFFSET_DEG
        )

        rz_correction = float(np.clip(
            rz_correction,
            -self.MAX_ABS_RZ_CORRECTION_DEG,
            self.MAX_ABS_RZ_CORRECTION_DEG,
        ))

        target_rx = float(current_pose[3])
        target_ry = float(current_pose[4])
        target_rz = self.normalize_angle_deg(
            float(current_pose[5]) + rz_correction
        )

        self.draw_polygon(
            annotated,
            passport_polygon,
            (255, 0, 255),
        )
        self.draw_polygon(
            annotated,
            photo_polygon,
            (0, 255, 0),
        )

        cv2.line(
            annotated,
            tuple(np.round(selected_edge[0]).astype(int)),
            tuple(np.round(selected_edge[1]).astype(int)),
            (255, 255, 0),
            3,
        )

        cv2.circle(
            annotated,
            grasp_pixel,
            7,
            (0, 0, 255),
            -1,
        )

        cv2.putText(
            annotated,
            f"photo angle={photo_angle_deg:+.2f} deg",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        cv2.putText(
            annotated,
            (
                f"XYZ=({base_position[0]:.1f}, "
                f"{base_position[1]:.1f}, "
                f"{base_position[2]:.1f})"
            ),
            (20, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )

        cv2.putText(
            annotated,
            (
                f"R=({target_rx:.1f}, "
                f"{target_ry:.1f}, "
                f"{target_rz:.1f})"
            ),
            (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )

        current_time = time.monotonic()

        if (
            current_time - self.last_output_time
            >= self.OUTPUT_INTERVAL_SEC
        ):
            self.last_output_time = current_time

            self.get_logger().info(
                "여권 파지 자세 계산 결과: "
                f"x={base_position[0]:.2f}, "
                f"y={base_position[1]:.2f}, "
                f"z={base_position[2]:.2f} mm, "
                f"rx={target_rx:.2f}, "
                f"ry={target_ry:.2f}, "
                f"rz={target_rz:.2f} deg, "
                f"photo_angle={photo_angle_deg:+.2f} deg, "
                f"passport_conf={passport_confidence:.3f}, "
                f"photo_conf={photo_confidence:.3f}"
            )

        self.show_frame(annotated)

    def _prune_position_samples(self, now: float) -> None:
        cutoff = now - self.STABILITY_HISTORY_SEC
        self.position_samples = [
            (timestamp, position)
            for timestamp, position in self.position_samples
            if timestamp >= cutoff
        ]

    def _record_position_sample(self, position: np.ndarray) -> None:
        now = time.monotonic()
        with self.sample_condition:
            if not self.detection_requested:
                return

            if self.first_valid_sample_time is None:
                self.first_valid_sample_time = now
                self.get_logger().info(
                    "여권이 화면에 들어왔습니다. 안정된 파지 위치를 계산합니다."
                )
                self.sample_condition.notify_all()

            self.position_samples.append((now, position.copy()))
            self._prune_position_samples(now)
            stable = self._calculate_stable_position()

            if stable is not None and self.stable_base_position is None:
                self.stable_base_position = stable
                self.get_logger().info(
                    "여권 파지 위치 안정화 완료: "
                    f"samples={len(self.position_samples)}, "
                    f"xyz=({stable[0]:.2f}, {stable[1]:.2f}, {stable[2]:.2f}) mm"
                )
                self.sample_condition.notify_all()

    def _calculate_stable_position(self) -> Optional[np.ndarray]:
        if len(self.position_samples) < self.STABILITY_MIN_SAMPLE_COUNT:
            return None

        timestamps = np.asarray(
            [timestamp for timestamp, _ in self.position_samples],
            dtype=float,
        )
        if timestamps[-1] - timestamps[0] < self.STABILITY_MIN_DURATION_SEC:
            return None

        points = np.asarray(
            [position for _, position in self.position_samples],
            dtype=float,
        )
        median = np.median(points, axis=0)
        xy_deviation = np.linalg.norm(points[:, :2] - median[:2], axis=1)
        z_deviation = np.abs(points[:, 2] - median[2])
        inliers = (
            (xy_deviation <= self.STABILITY_MAX_XY_DEVIATION_MM)
            & (z_deviation <= self.STABILITY_MAX_Z_DEVIATION_MM)
        )

        if int(np.count_nonzero(inliers)) < self.STABILITY_MIN_SAMPLE_COUNT:
            return None

        return np.median(points[inliers], axis=0)

    def select_passport_and_photo(
        self,
        results,
    ) -> Tuple[
        Optional[Tuple[float, np.ndarray]],
        Optional[Tuple[float, np.ndarray]],
    ]:
        if not results:
            return None, None

        result = results[0]

        if result.boxes is None:
            return None, None

        if result.masks is None:
            return None, None

        if result.masks.xy is None:
            return None, None

        passport_best = None
        photo_best = None

        polygon_list = result.masks.xy

        count = min(
            len(result.boxes),
            len(polygon_list),
        )

        for index in range(count):
            box = result.boxes[index]

            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            class_name = str(self.model.names[class_id])

            polygon = np.asarray(
                polygon_list[index],
                dtype=np.float32,
            )

            if polygon.shape[0] < 3:
                continue

            if class_name == self.PASSPORT_CLASS_NAME:
                if (
                    passport_best is None
                    or confidence > passport_best[0]
                ):
                    passport_best = (
                        confidence,
                        polygon,
                    )

            elif class_name == self.PHOTO_CLASS_NAME:
                if (
                    photo_best is None
                    or confidence > photo_best[0]
                ):
                    photo_best = (
                        confidence,
                        polygon,
                    )

        return passport_best, photo_best

    def calculate_grasp_point(
        self,
        passport_polygon: np.ndarray,
        photo_polygon: np.ndarray,
        depth_frame: np.ndarray,
        base_to_camera: np.ndarray,
    ) -> Optional[
        Tuple[
            Tuple[int, int],
            np.ndarray,
            Tuple[np.ndarray, np.ndarray],
        ]
    ]:
        """
        선택 기준:
        1. 여권의 최소 회전 사각형에서 4개 변을 만든다.
        2. 변의 양 끝점이 모두 여권사진 영역의 최하단보다
           카메라 영상 기준 아래쪽에 있는 변만 후보로 사용한다.
        3. 후보 중 길이가 가장 긴 변을 선택한다.
        4. 선택한 긴 변의 중앙에서 여권 중심 방향으로
           GRASP_INWARD_RATIO만큼 들어간 점을 파지점으로 사용한다.
        """

        rect = cv2.minAreaRect(
            passport_polygon.astype(np.float32)
        )

        box = cv2.boxPoints(rect).astype(np.float32)

        passport_center = np.mean(
            passport_polygon,
            axis=0,
        )

        # 초록색 여권사진 세그멘테이션 영역의 최하단 y좌표
        photo_bottom_y = float(
            np.max(photo_polygon[:, 1])
        )

        candidates: List[
            Tuple[
                float,
                np.ndarray,
                np.ndarray,
            ]
        ] = []

        for index in range(4):
            point_a = box[index]
            point_b = box[(index + 1) % 4]

            # 영상 좌표에서는 y가 클수록 아래쪽이다.
            # 변의 양 끝점이 모두 사진 영역보다 아래에 있어야 한다.
            if (
                float(point_a[1]) <= photo_bottom_y
                or float(point_b[1]) <= photo_bottom_y
            ):
                continue

            edge_length = float(
                np.linalg.norm(point_b - point_a)
            )

            candidates.append(
                (
                    edge_length,
                    point_a,
                    point_b,
                )
            )

        if not candidates:
            return None

        # 사진 영역보다 아래쪽에 완전히 위치한 변 중 가장 긴 변
        _, point_a, point_b = max(
            candidates,
            key=lambda item: item[0],
        )

        # 선택한 긴 변의 정확한 중앙점
        edge_center = (
            point_a + point_b
        ) / 2.0

        inward_point = (
            edge_center
            + self.GRASP_INWARD_RATIO
            * (passport_center - edge_center)
        )

        pixel_u = int(round(float(inward_point[0])))
        pixel_v = int(round(float(inward_point[1])))

        depth_mm = self.get_stable_depth(
            pixel_u,
            pixel_v,
            depth_frame,
        )

        if depth_mm is None:
            return None

        camera_position = self.pixel_to_camera_position(
            pixel_u,
            pixel_v,
            depth_mm,
        )

        base_position = self.transform_camera_point_to_base(
            camera_position,
            base_to_camera,
        )

        return (
            (pixel_u, pixel_v),
            base_position,
            (point_a, point_b),
        )

    def calculate_photo_angle(
        self,
        photo_polygon: np.ndarray,
    ) -> float:
        """
        사진 마스크의 세로축 방향을 이용해
        영상 평면 회전각을 계산한다.

        반환 범위:
            -90도 이상 90도 미만
        """

        rect = cv2.minAreaRect(
            photo_polygon.astype(np.float32)
        )

        (_, _), (width, height), angle = rect

        if width <= 0.0 or height <= 0.0:
            return 0.0

        # 세로축 방향을 기준으로 통일한다.
        if width < height:
            photo_angle_deg = float(angle)
        else:
            photo_angle_deg = float(angle + 90.0)

        while photo_angle_deg >= 90.0:
            photo_angle_deg -= 180.0

        while photo_angle_deg < -90.0:
            photo_angle_deg += 180.0

        return photo_angle_deg

    def get_stable_depth(
        self,
        center_x: int,
        center_y: int,
        depth_frame: np.ndarray,
    ) -> Optional[float]:
        height, width = depth_frame.shape[:2]

        if (
            center_x < 0
            or center_x >= width
            or center_y < 0
            or center_y >= height
        ):
            return None

        radius = self.DEPTH_SAMPLE_RADIUS

        x_start = max(0, center_x - radius)
        x_end = min(width, center_x + radius + 1)

        y_start = max(0, center_y - radius)
        y_end = min(height, center_y + radius + 1)

        roi = depth_frame[
            y_start:y_end,
            x_start:x_end,
        ].astype(np.float64)

        valid_depths = roi[
            (roi >= self.MIN_DEPTH_MM)
            & (roi <= self.MAX_DEPTH_MM)
        ]

        if valid_depths.size == 0:
            return None

        return float(np.median(valid_depths))

    def pixel_to_camera_position(
        self,
        pixel_x: int,
        pixel_y: int,
        depth_mm: float,
    ) -> np.ndarray:
        camera_x = (
            pixel_x - self.intrinsics["ppx"]
        ) * depth_mm / self.intrinsics["fx"]

        camera_y = (
            pixel_y - self.intrinsics["ppy"]
        ) * depth_mm / self.intrinsics["fy"]

        camera_z = depth_mm

        return np.array(
            [camera_x, camera_y, camera_z],
            dtype=np.float64,
        )

    def get_current_flange_pose(self) -> np.ndarray:
        current_pose = get_current_tool_flange_posx()

        if (
            isinstance(current_pose, (tuple, list))
            and len(current_pose) > 0
            and isinstance(
                current_pose[0],
                (tuple, list, np.ndarray),
            )
        ):
            current_pose = current_pose[0]

        current_pose = np.asarray(
            current_pose,
            dtype=np.float64,
        ).reshape(-1)

        if current_pose.size != 6:
            raise RuntimeError(
                "Tool Flange 자세가 6개 값이 아닙니다: "
                f"{current_pose}"
            )

        return current_pose

    def get_robot_pose_matrix(
        self,
        x: float,
        y: float,
        z: float,
        rx: float,
        ry: float,
        rz: float,
    ) -> np.ndarray:
        rotation = Rotation.from_euler(
            "ZYZ",
            [rx, ry, rz],
            degrees=True,
        ).as_matrix()

        transform = np.eye(
            4,
            dtype=np.float64,
        )

        transform[:3, :3] = rotation
        transform[:3, 3] = [x, y, z]

        return transform

    def transform_camera_point_to_base(
        self,
        camera_position: np.ndarray,
        base_to_camera: np.ndarray,
    ) -> np.ndarray:
        if self.ROTATE_CAMERA_180:
            camera_position_for_calibration = np.array(
                [
                    -camera_position[0],
                    -camera_position[1],
                    camera_position[2],
                ],
                dtype=np.float64,
            )
        else:
            camera_position_for_calibration = camera_position

        camera_homogeneous = np.append(
            camera_position_for_calibration,
            1.0,
        )

        base_homogeneous = (
            base_to_camera
            @ camera_homogeneous
        )

        return base_homogeneous[:3]

    def normalize_angle_deg(
        self,
        angle_deg: float,
    ) -> float:
        while angle_deg >= 180.0:
            angle_deg -= 360.0

        while angle_deg < -180.0:
            angle_deg += 360.0

        return angle_deg

    def draw_polygon(
        self,
        image: np.ndarray,
        polygon: np.ndarray,
        color: Tuple[int, int, int],
    ) -> None:
        points = np.round(
            polygon
        ).astype(np.int32)

        cv2.polylines(
            image,
            [points],
            True,
            color,
            2,
        )

    def draw_status(
        self,
        image: np.ndarray,
        text: str,
        color: Tuple[int, int, int],
    ) -> None:
        cv2.putText(
            image,
            text,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
        )

    def show_frame(
        self,
        image: np.ndarray,
    ) -> None:
        if not self.SHOW_DEBUG_WINDOW:
            return

        cv2.imshow(
            self.WINDOW_NAME,
            image,
        )

        key = cv2.waitKey(1) & 0xFF

        if key == 27:
            self.get_logger().info(
                "ESC키로 종료합니다."
            )

            if rclpy.ok():
                rclpy.shutdown()

    def destroy_node(self):
        if self.SHOW_DEBUG_WINDOW:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    dsr_node = rclpy.create_node(
        "passport_grasp_dsr_node",
        namespace=ROBOT_ID,
    )

    DR_init.__dsr__node = dsr_node

    detector_node = None

    try:
        global get_current_tool_flange_posx

        from DSR_ROBOT2 import get_current_tool_flange_posx

        detector_node = PassportGraspPoseNode()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(detector_node)
        executor.spin()

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if detector_node is not None:
            detector_node.get_logger().error(
                f"여권 파지 자세 계산 노드 오류: {error}"
            )
        else:
            print(
                f"여권 파지 자세 계산 노드 생성 실패: {error}"
            )

    finally:
        if detector_node is not None:
            detector_node.destroy_node()

        dsr_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
