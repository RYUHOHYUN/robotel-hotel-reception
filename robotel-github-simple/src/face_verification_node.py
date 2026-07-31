#!/usr/bin/env python3
"""Passport/live-face comparison with RealSense depth liveness validation."""

from __future__ import annotations

import os
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from checkin_interfaces.srv import VerifyPassportFace
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class PassportFaceVerificationNode(Node):
    SERVICE_NAME = "/passport/verify_face"
    RESULT_TOPIC = "/passport/face_verified"
    STREAM_TOPIC = "/robotel/vision/face_verification/image"
    COLOR_TOPIC = "/camera/camera/color/image_raw"
    DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"

    SAME_THRESHOLD = float(os.getenv("FACE_SAME_THRESHOLD", "0.40"))
    DIFFERENT_THRESHOLD = float(os.getenv("FACE_DIFFERENT_THRESHOLD", "0.32"))
    SCORE_WINDOW_SIZE = int(os.getenv("FACE_SCORE_WINDOW_SIZE", "15"))
    MIN_SCORE_COUNT = int(os.getenv("FACE_MIN_SCORE_COUNT", "8"))
    VERIFY_TIMEOUT_SEC = float(os.getenv("FACE_VERIFY_TIMEOUT_SEC", "15.0"))

    MIN_FACE_DISTANCE_MM = float(os.getenv("FACE_MIN_DISTANCE_MM", "300"))
    MAX_FACE_DISTANCE_MM = float(os.getenv("FACE_MAX_DISTANCE_MM", "1500"))
    MIN_DEPTH_VALID_RATIO = float(os.getenv("FACE_MIN_DEPTH_VALID_RATIO", "0.45"))
    MIN_DEPTH_SPAN_MM = float(os.getenv("FACE_MIN_DEPTH_SPAN_MM", "18"))
    MAX_DEPTH_SPAN_MM = float(os.getenv("FACE_MAX_DEPTH_SPAN_MM", "220"))
    MIN_NOSE_PROTRUSION_MM = float(os.getenv("FACE_MIN_NOSE_PROTRUSION_MM", "6"))

    OUTPUT_DIRECTORY = Path(
        os.getenv(
            "ROBOTEL_OUTPUT_DIRECTORY",
            str(Path.cwd() / "output"),
        )
    )

    def __init__(self) -> None:
        super().__init__("passport_face_verification_node")
        cv2.setNumThreads(1)

        package_share = Path(get_package_share_directory("hotel_vision"))
        yunet_path = package_share / "models" / "face_detection_yunet_2023mar.onnx"
        sface_path = package_share / "models" / "face_recognition_sface_2021dec.onnx"
        if not yunet_path.is_file() or not sface_path.is_file():
            raise FileNotFoundError(
                f"얼굴 모델 파일이 없습니다: yunet={yunet_path}, sface={sface_path}"
            )

        self.detector = cv2.FaceDetectorYN.create(
            str(yunet_path),
            "",
            (320, 320),
            score_threshold=0.9,
            nms_threshold=0.3,
            top_k=5000,
        )
        self.recognizer = cv2.FaceRecognizerSF.create(str(sface_path), "")

        self.output_directory = self.OUTPUT_DIRECTORY.expanduser()
        self.output_directory.mkdir(parents=True, exist_ok=True)

        self.frame_condition = threading.Condition()
        self.color_frame: Optional[np.ndarray] = None
        self.depth_frame_mm: Optional[np.ndarray] = None
        self.color_sequence = 0
        self.depth_sequence = 0
        self.color_received_at = 0.0
        self.depth_received_at = 0.0
        self.verification_lock = threading.Lock()

        group = ReentrantCallbackGroup()
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.color_subscription = self.create_subscription(
            Image,
            self.COLOR_TOPIC,
            self.color_callback,
            image_qos,
            callback_group=group,
        )
        self.depth_subscription = self.create_subscription(
            Image,
            self.DEPTH_TOPIC,
            self.depth_callback,
            image_qos,
            callback_group=group,
        )
        self.stream_publisher = self.create_publisher(Image, self.STREAM_TOPIC, image_qos)
        self.result_publisher = self.create_publisher(Bool, self.RESULT_TOPIC, 10)
        self.service = self.create_service(
            VerifyPassportFace,
            self.SERVICE_NAME,
            self.verify_callback,
            callback_group=group,
        )

        self.get_logger().info(f"Depth 얼굴 검증 서비스: {self.SERVICE_NAME}")
        self.get_logger().info(f"RGB 토픽: {self.COLOR_TOPIC}")
        self.get_logger().info(f"정렬 Depth 토픽: {self.DEPTH_TOPIC}")
        self.get_logger().info(f"공용 저장 폴더: {self.output_directory}")

    def color_callback(self, msg: Image) -> None:
        try:
            frame = self._color_message_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warning(f"RGB 변환 실패: {exc}")
            return

        with self.frame_condition:
            self.color_frame = frame
            self.color_sequence += 1
            self.color_received_at = time.monotonic()
            self.frame_condition.notify_all()

        # 검증 전에도 GUI에서 카메라가 계속 보이도록 원본 RGB를 발행합니다.
        self.stream_publisher.publish(self._bgr_to_message(frame, msg.header))

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self._depth_message_to_mm(msg)
        except Exception as exc:
            self.get_logger().warning(f"Depth 변환 실패: {exc}")
            return
        with self.frame_condition:
            self.depth_frame_mm = depth
            self.depth_sequence += 1
            self.depth_received_at = time.monotonic()
            self.frame_condition.notify_all()

    def verify_callback(self, request, response):
        if not self.verification_lock.acquire(blocking=False):
            return self._set_response(
                response,
                success=False,
                liveness=False,
                verified=False,
                similarity=0.0,
                message="face_verification_busy",
            )

        try:
            photo_path = Path(str(request.passport_photo_path).strip()).expanduser()
            if not photo_path.is_file():
                return self._set_response(
                    response,
                    success=False,
                    liveness=False,
                    verified=False,
                    similarity=0.0,
                    message=f"passport_photo_not_found:{photo_path}",
                )

            passport_image = cv2.imread(str(photo_path), cv2.IMREAD_COLOR)
            if passport_image is None:
                return self._set_response(
                    response,
                    success=False,
                    liveness=False,
                    verified=False,
                    similarity=0.0,
                    message="passport_photo_read_failed",
                )

            _, passport_feature, passport_message = self._extract_feature(
                passport_image,
                require_single_face=False,
            )
            if passport_feature is None:
                return self._set_response(
                    response,
                    success=False,
                    liveness=False,
                    verified=False,
                    similarity=0.0,
                    message=f"passport_face_feature_failed:{passport_message}",
                )

            result = self._verify_live_face(passport_feature)
            verified_msg = Bool()
            verified_msg.data = bool(result["verified"])
            self.result_publisher.publish(verified_msg)

            return self._set_response(
                response,
                success=bool(result["success"]),
                liveness=bool(result["liveness_passed"]),
                verified=bool(result["verified"]),
                similarity=float(result["similarity"]),
                message=str(result["message"]),
                live_photo_path=str(result.get("live_photo_path", "")),
            )
        except Exception as exc:
            self.get_logger().error(f"얼굴 검증 오류: {type(exc).__name__}: {exc}")
            return self._set_response(
                response,
                success=False,
                liveness=False,
                verified=False,
                similarity=0.0,
                message=f"face_verification_error:{type(exc).__name__}",
            )
        finally:
            self.verification_lock.release()

    def _verify_live_face(self, passport_feature: np.ndarray) -> dict:
        deadline = time.monotonic() + self.VERIFY_TIMEOUT_SEC
        scores: deque[float] = deque(maxlen=self.SCORE_WINDOW_SIZE)
        last_color_sequence = -1
        last_depth_sequence = -1
        liveness_seen = False
        last_message = "face_not_found"
        best_frame = None
        best_face = None

        while rclpy.ok() and time.monotonic() < deadline:
            pair = self._wait_for_frame_pair(
                last_color_sequence,
                last_depth_sequence,
                timeout=0.5,
            )
            if pair is None:
                last_message = "camera_frame_timeout"
                continue
            frame, depth_mm, last_color_sequence, last_depth_sequence = pair

            face, feature, message = self._extract_feature(frame, require_single_face=True)
            if feature is None or face is None:
                scores.clear()
                last_message = message.lower().replace(" ", "_")
                self._publish_overlay(frame, None, last_message, 0.0, False)
                continue

            liveness_passed, liveness_message = self._depth_liveness(depth_mm, face)
            if not liveness_passed:
                scores.clear()
                last_message = liveness_message
                self._publish_overlay(frame, face, liveness_message, 0.0, False)
                continue

            liveness_seen = True
            current_score = self._compare_features(passport_feature, feature)
            scores.append(current_score)
            best_frame = frame.copy()
            best_face = face.copy()
            stable_score = float(np.median(scores))
            self._publish_overlay(
                frame,
                face,
                f"DEPTH LIVE {len(scores)}/{self.MIN_SCORE_COUNT}",
                stable_score,
                True,
            )

            if len(scores) < self.MIN_SCORE_COUNT:
                continue

            live_photo_path = self._save_live_face(best_frame, best_face)
            if stable_score >= self.SAME_THRESHOLD:
                return {
                    "success": True,
                    "liveness_passed": True,
                    "verified": True,
                    "similarity": stable_score,
                    "message": "verified",
                    "live_photo_path": live_photo_path,
                }
            if stable_score <= self.DIFFERENT_THRESHOLD:
                return {
                    "success": True,
                    "liveness_passed": True,
                    "verified": False,
                    "similarity": stable_score,
                    "message": "different_person",
                    "live_photo_path": live_photo_path,
                }
            last_message = "face_match_uncertain"

        return {
            "success": False,
            "liveness_passed": liveness_seen,
            "verified": False,
            "similarity": float(np.median(scores)) if scores else 0.0,
            "message": last_message,
            "live_photo_path": "",
        }

    def _wait_for_frame_pair(
        self,
        previous_color_sequence: int,
        previous_depth_sequence: int,
        timeout: float,
    ) -> Optional[Tuple[np.ndarray, np.ndarray, int, int]]:
        deadline = time.monotonic() + timeout
        with self.frame_condition:
            while rclpy.ok():
                if (
                    self.color_frame is not None
                    and self.depth_frame_mm is not None
                    and self.color_sequence != previous_color_sequence
                    and self.depth_sequence != previous_depth_sequence
                ):
                    # 멈춘 Depth 프레임을 새 RGB에 재사용하지 않습니다.
                    if abs(self.color_received_at - self.depth_received_at) > 0.35:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0:
                            return None
                        self.frame_condition.wait(timeout=min(remaining, 0.05))
                        continue

                    color = self.color_frame.copy()
                    depth = self.depth_frame_mm.copy()
                    if color.shape[:2] != depth.shape[:2]:
                        self.get_logger().warning(
                            "RGB/Depth 크기가 다릅니다. aligned_depth_to_color 토픽을 확인하세요: "
                            f"color={color.shape[:2]}, depth={depth.shape[:2]}"
                        )
                        return None
                    return (
                        color,
                        depth,
                        self.color_sequence,
                        self.depth_sequence,
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.frame_condition.wait(timeout=remaining)
        return None

    def _depth_liveness(self, depth_mm: np.ndarray, face: np.ndarray) -> Tuple[bool, str]:
        x, y, width, height = face[:4].astype(int)
        image_h, image_w = depth_mm.shape[:2]
        x1 = max(0, x)
        y1 = max(0, y)
        x2 = min(image_w, x + width)
        y2 = min(image_h, y + height)
        if x2 - x1 < 20 or y2 - y1 < 20:
            return False, "face_depth_roi_invalid"

        roi = depth_mm[y1:y2, x1:x2]
        valid = roi[(roi >= self.MIN_FACE_DISTANCE_MM) & (roi <= self.MAX_FACE_DISTANCE_MM)]
        valid_ratio = float(valid.size) / float(max(roi.size, 1))
        if valid.size < 100 or valid_ratio < self.MIN_DEPTH_VALID_RATIO:
            return False, "depth_data_insufficient"

        near = float(np.percentile(valid, 10))
        far = float(np.percentile(valid, 90))
        span = far - near
        if span < self.MIN_DEPTH_SPAN_MM:
            return False, "flat_surface_detected"
        if span > self.MAX_DEPTH_SPAN_MM:
            return False, "depth_background_contamination"

        landmarks = face[4:14].reshape(5, 2)
        nose_x, nose_y = landmarks[2]
        left_eye, right_eye = landmarks[0], landmarks[1]
        left_mouth, right_mouth = landmarks[3], landmarks[4]

        nose_depth = self._patch_depth(depth_mm, nose_x, nose_y, width, height)
        cheek_a = (left_eye + left_mouth) / 2.0
        cheek_b = (right_eye + right_mouth) / 2.0
        cheek_depth_a = self._patch_depth(depth_mm, cheek_a[0], cheek_a[1], width, height)
        cheek_depth_b = self._patch_depth(depth_mm, cheek_b[0], cheek_b[1], width, height)

        if nose_depth is None or cheek_depth_a is None or cheek_depth_b is None:
            return False, "face_depth_landmark_missing"

        cheek_depth = float(np.median([cheek_depth_a, cheek_depth_b]))
        nose_protrusion = cheek_depth - nose_depth
        if nose_protrusion < self.MIN_NOSE_PROTRUSION_MM:
            return False, "flat_face_profile_detected"

        return True, "depth_liveness_passed"

    def _patch_depth(
        self,
        depth_mm: np.ndarray,
        center_x: float,
        center_y: float,
        face_width: int,
        face_height: int,
    ) -> Optional[float]:
        radius = max(2, int(round(min(face_width, face_height) * 0.025)))
        x = int(round(center_x))
        y = int(round(center_y))
        x1, x2 = max(0, x - radius), min(depth_mm.shape[1], x + radius + 1)
        y1, y2 = max(0, y - radius), min(depth_mm.shape[0], y + radius + 1)
        patch = depth_mm[y1:y2, x1:x2]
        valid = patch[(patch >= self.MIN_FACE_DISTANCE_MM) & (patch <= self.MAX_FACE_DISTANCE_MM)]
        if valid.size < 3:
            return None
        return float(np.median(valid))

    def _extract_feature(
        self,
        image: np.ndarray,
        require_single_face: bool,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
        height, width = image.shape[:2]
        self.detector.setInputSize((width, height))
        _, faces = self.detector.detect(image)
        if faces is None or len(faces) == 0:
            return None, None, "FACE NOT FOUND"
        if require_single_face and len(faces) > 1:
            return None, None, "ONE PERSON ONLY"
        face = max(faces, key=lambda item: float(item[2] * item[3]))
        aligned = self.recognizer.alignCrop(image, face)
        feature = self.recognizer.feature(aligned).copy()
        return face, feature, "OK"

    def _compare_features(self, passport_feature: np.ndarray, live_feature: np.ndarray) -> float:
        return float(
            self.recognizer.match(
                passport_feature,
                live_feature,
                cv2.FaceRecognizerSF_FR_COSINE,
            )
        )

    def _publish_overlay(
        self,
        frame: np.ndarray,
        face: Optional[np.ndarray],
        status: str,
        score: float,
        live: bool,
    ) -> None:
        display = frame.copy()
        color = (0, 255, 0) if live else (0, 0, 255)
        if face is not None:
            x, y, width, height = face[:4].astype(int)
            cv2.rectangle(display, (x, y), (x + width, y + height), color, 2)
        cv2.putText(
            display,
            status[:55],
            (30, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            color,
            2,
            cv2.LINE_AA,
        )
        if score:
            cv2.putText(
                display,
                f"similarity: {score:.3f}",
                (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )
        self.stream_publisher.publish(self._bgr_to_message(display))

    def _save_live_face(
        self,
        frame: Optional[np.ndarray],
        face: Optional[np.ndarray],
    ) -> str:
        if frame is None or face is None:
            return ""
        x, y, width, height = face[:4].astype(int)
        margin_x = int(width * 0.15)
        margin_y = int(height * 0.15)
        x1 = max(0, x - margin_x)
        y1 = max(0, y - margin_y)
        x2 = min(frame.shape[1], x + width + margin_x)
        y2 = min(frame.shape[0], y + height + margin_y)
        crop = frame[y1:y2, x1:x2]
        if crop.size == 0:
            return ""
        path = self.output_directory / f"live_face_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        if cv2.imwrite(str(path), crop):
            self.get_logger().info(f"실제 얼굴 사진 저장 완료: {path}")
            return str(path)
        return ""

    @staticmethod
    def _color_message_to_bgr(msg: Image) -> np.ndarray:
        height, width, step = int(msg.height), int(msg.width), int(msg.step)
        encoding = str(msg.encoding).lower().strip()
        channels = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1}.get(encoding)
        if channels is None:
            raise ValueError(f"unsupported color encoding: {msg.encoding}")
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        required = height * step
        if raw.size < required:
            raise ValueError("color image data too short")
        rows = raw[:required].reshape(height, step)
        pixels = rows[:, : width * channels]
        frame = pixels.reshape(height, width) if channels == 1 else pixels.reshape(height, width, channels)
        if encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif encoding == "bgra8":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        elif encoding == "rgba8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        elif encoding == "mono8":
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return np.ascontiguousarray(frame)

    @staticmethod
    def _depth_message_to_mm(msg: Image) -> np.ndarray:
        height, width, step = int(msg.height), int(msg.width), int(msg.step)
        encoding = str(msg.encoding).lower().strip()
        if encoding in {"16uc1", "mono16"}:
            dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
            item_size = 2
            scale = 1.0
        elif encoding == "32fc1":
            dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
            item_size = 4
            scale = 1000.0
        else:
            raise ValueError(f"unsupported depth encoding: {msg.encoding}")
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        required = height * step
        if raw.size < required:
            raise ValueError("depth image data too short")
        rows = raw[:required].reshape(height, step)
        pixels = rows[:, : width * item_size].copy()
        depth = pixels.view(dtype).reshape(height, width).astype(np.float32)
        return depth * scale

    def _bgr_to_message(self, frame: np.ndarray, source_header=None) -> Image:
        contiguous = np.ascontiguousarray(frame, dtype=np.uint8)
        height, width = contiguous.shape[:2]
        msg = Image()
        if source_header is not None:
            msg.header = source_header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "face_verification_color_frame"
        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = contiguous.tobytes()
        return msg

    @staticmethod
    def _set_response(
        response,
        *,
        success: bool,
        liveness: bool,
        verified: bool,
        similarity: float,
        message: str,
        live_photo_path: str = "",
    ):
        response.success = success
        response.liveness_passed = liveness
        response.verified = verified
        response.similarity = float(similarity)
        response.message = message
        response.live_photo_path = live_photo_path
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PassportFaceVerificationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
