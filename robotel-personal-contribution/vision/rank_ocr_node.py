#!/usr/bin/env python3
# 개인 기여 코드: PaddleOCR 기반 경품 등수 인식
"""ROS2 event rank OCR node using PaddleOCR PP-OCRv5.

The node keeps the configured ABKO camera closed until ``/event/rank_ocr/start`` is
called. While active it publishes the same camera frame used for OCR to
``/hand_teleop/event_ocr_camera/image_raw`` so the kiosk prize page and OCR
always see the identical image.
"""

from __future__ import annotations

import json
import os
import queue
import re
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

try:
    import paddle
    from paddleocr import TextRecognition
except Exception as exc:  # keep services alive and expose the import failure
    paddle = None
    TextRecognition = None
    PADDLE_IMPORT_ERROR: Optional[str] = f"{type(exc).__name__}: {exc}"
else:
    PADDLE_IMPORT_ERROR = None


MODEL_NAME = "korean_PP-OCRv5_mobile_rec"
ENGINE = "paddle_static"
CPU_THREADS = max(2, min(8, (os.cpu_count() or 4) - 1))
TARGET_RANKS = ("1등", "2등", "3등", "4등")

OCR_EVERY_N_FRAMES = 4
SHARPEST_FRAME_POOL = 4
RESULT_TIMEOUT_SECONDS = 1.5
VOTE_WINDOW = 12
VOTE_MIN_COUNT = 5
VOTE_MIN_WEIGHT = 2.75
VOTE_WINNER_MARGIN = 0.45
MIN_ACCEPT_SCORE = 0.55
MIN_OCR_VISIBLE_SECONDS = 2.0
TARGET_TEXT_HEIGHT = 96
MAX_TEXT_WIDTH = 768
ADD_THRESHOLD_VARIANT = True
DEFAULT_ROI = (0.33, 0.40, 0.34, 0.20)


@dataclass
class RecognitionCandidate:
    rank: Optional[str]
    raw_text: str
    model_score: float
    adjusted_score: float
    variant_name: str


@dataclass
class OCRResult:
    candidates: list[RecognitionCandidate]
    best: Optional[RecognitionCandidate]
    elapsed_ms: float
    timestamp: float
    sharpness: float
    error: Optional[str] = None


def normalize_ocr_text(text: str) -> str:
    text = str(text).strip().replace(" ", "")
    text = text.translate(
        str.maketrans(
            {
                "I": "1",
                "l": "1",
                "|": "1",
                "①": "1",
                "②": "2",
                "③": "3",
                "④": "4",
            }
        )
    )
    return re.sub(r"[^0-9A-Za-z가-힣]", "", text)


def classify_text(text: str, score: float, variant_name: str) -> RecognitionCandidate:
    """Accept only a rank expression that includes both the digit and 등-like text.

    A lone 1-4 digit is deliberately rejected. This prevents background shapes, ROI
    borders, or a partially visible coupon from being finalized as a prize rank.
    """
    normalized = normalize_ocr_text(text)
    for target in TARGET_RANKS:
        if normalized == target and score >= MIN_ACCEPT_SCORE:
            return RecognitionCandidate(
                target, text, score, min(1.0, score + 0.08), variant_name
            )
        if target in normalized and score >= MIN_ACCEPT_SCORE + 0.05:
            return RecognitionCandidate(
                target, text, score, min(1.0, score + 0.03), variant_name
            )

    digits = re.findall(r"[1-4]", normalized)
    rank_like_syllable = any(
        ch in normalized for ch in ("등", "둥", "동", "듬", "듕", "응")
    )
    if len(set(digits)) == 1 and rank_like_syllable:
        adjusted = score * 0.88
        rank = f"{digits[0]}등" if adjusted >= MIN_ACCEPT_SCORE else None
        return RecognitionCandidate(rank, text, score, adjusted, variant_name)

    return RecognitionCandidate(None, text, score, score * 0.20, variant_name)


def fit_text_line(image: np.ndarray) -> np.ndarray:
    if image.size == 0:
        raise ValueError("ROI가 비어 있습니다.")
    height, width = image.shape[:2]
    pad_y = max(5, int(round(height * 0.10)))
    pad_x = max(8, int(round(width * 0.06)))
    bordered = cv2.copyMakeBorder(
        image, pad_y, pad_y, pad_x, pad_x, cv2.BORDER_REPLICATE
    )
    height, width = bordered.shape[:2]
    scale = TARGET_TEXT_HEIGHT / max(1, height)
    new_width = max(32, int(round(width * scale)))
    new_height = TARGET_TEXT_HEIGHT
    if new_width > MAX_TEXT_WIDTH:
        scale = MAX_TEXT_WIDTH / max(1, width)
        new_width = MAX_TEXT_WIDTH
        new_height = max(32, int(round(height * scale)))
    interpolation = cv2.INTER_CUBIC if scale >= 1.0 else cv2.INTER_AREA
    return cv2.resize(bordered, (new_width, new_height), interpolation=interpolation)


def build_preprocess_variants(roi: np.ndarray) -> list[tuple[str, np.ndarray]]:
    base = fit_text_line(roi)
    gray = cv2.cvtColor(base, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
    blur = cv2.GaussianBlur(base, (0, 0), 1.2)
    sharpened = cv2.addWeighted(base, 1.8, blur, -0.8, 0)
    variants: list[tuple[str, np.ndarray]] = [
        ("original", base),
        ("clahe", cv2.cvtColor(clahe, cv2.COLOR_GRAY2BGR)),
        ("sharpen", sharpened),
    ]
    if ADD_THRESHOLD_VARIANT:
        binary = cv2.threshold(clahe, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        if float(np.mean(binary)) < 127.0:
            binary = cv2.bitwise_not(binary)
        binary = cv2.morphologyEx(
            binary,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)),
        )
        variants.append(("otsu", cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)))
    return variants



def parse_paddle_result(result: Any) -> tuple[str, float]:
    data = getattr(result, "json", result)
    if callable(data):
        data = data()
    if isinstance(data, str):
        data = json.loads(data)
    if not isinstance(data, dict):
        return "", 0.0
    payload = data.get("res", data)
    if not isinstance(payload, dict):
        return "", 0.0
    text = str(payload.get("rec_text", ""))
    try:
        score = float(payload.get("rec_score", 0.0))
    except (TypeError, ValueError):
        score = 0.0
    return text, score


def roi_sharpness(roi: np.ndarray) -> float:
    if roi.size == 0:
        return 0.0
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def get_stable_rank(history: deque[tuple[Optional[str], float]]) -> tuple[Optional[str], int, float]:
    valid = [(rank, weight) for rank, weight in history if rank is not None]
    if not valid:
        return None, 0, 0.0
    counts = Counter(rank for rank, _ in valid)
    weights: dict[str, float] = {rank: 0.0 for rank in TARGET_RANKS}
    for rank, weight in valid:
        assert rank is not None
        weights[rank] += weight
    ordered = sorted(weights.items(), key=lambda item: item[1], reverse=True)
    winner, winner_weight = ordered[0]
    second_weight = ordered[1][1] if len(ordered) > 1 else 0.0
    count = counts[winner]
    stable = (
        count >= VOTE_MIN_COUNT
        and winner_weight >= VOTE_MIN_WEIGHT
        and winner_weight - second_weight >= VOTE_WINNER_MARGIN
    )
    return (winner if stable else None), count, winner_weight


class OCRWorker(threading.Thread):
    def __init__(self, model: Any, input_queue: queue.Queue, output_queue: queue.Queue, stop_event: threading.Event) -> None:
        super().__init__(daemon=True)
        self.model = model
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.stop_event = stop_event

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                payload = self.input_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if payload is None:
                break
            roi, sharpness = payload
            started = time.perf_counter()
            candidates: list[RecognitionCandidate] = []
            error: Optional[str] = None
            try:
                variants = build_preprocess_variants(roi)
                images = [image for _, image in variants]
                outputs = self.model.predict(input=images, batch_size=len(images))
                for (variant_name, _), result in zip(variants, outputs):
                    raw_text, model_score = parse_paddle_result(result)
                    candidates.append(classify_text(raw_text, model_score, variant_name))
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
            valid = [
                candidate
                for candidate in candidates
                if candidate.rank is not None and candidate.adjusted_score >= MIN_ACCEPT_SCORE
            ]
            best = max(valid, key=lambda candidate: candidate.adjusted_score, default=None)
            result = OCRResult(
                candidates=candidates,
                best=best,
                elapsed_ms=(time.perf_counter() - started) * 1000.0,
                timestamp=time.monotonic(),
                sharpness=sharpness,
                error=error,
            )
            try:
                while True:
                    self.output_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.output_queue.put_nowait(result)
            except queue.Full:
                pass


class RealtimeRankOCRNode(Node):
    def __init__(self) -> None:
        super().__init__("realtime_rank_ocr_paddle")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter(
            "camera_device",
            "/dev/v4l/by-id/usb-VNV_ABKO_APC930_QHD_WEBCAM-video-index0",
        )
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        self.declare_parameter("camera_fps", 30.0)
        self.declare_parameter("camera_open_timeout_sec", 4.0)
        self.declare_parameter("image_topic", "/hand_teleop/event_ocr_camera/image_raw")
        self.declare_parameter("rank_topic", "/event/rank_ocr/detected")
        self.declare_parameter("start_service", "/event/rank_ocr/start")
        self.declare_parameter("stop_service", "/event/rank_ocr/stop")
        self.declare_parameter("result_service", "/event/rank_ocr/result")
        self.declare_parameter("roi_config_file", "rank_roi.json")
        self.declare_parameter("try_autofocus", True)

        self._camera_index = int(self.get_parameter("camera_index").value)
        configured_camera_device = str(
            self.get_parameter("camera_device").value
        ).strip()
        self._camera_source = (
            configured_camera_device
            if configured_camera_device
            else self._camera_index
        )
        self._camera_label = (
            configured_camera_device
            if configured_camera_device
            else f"index={self._camera_index}"
        )
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)
        self._camera_fps = float(self.get_parameter("camera_fps").value)
        self._camera_open_timeout_sec = max(
            0.5,
            float(self.get_parameter("camera_open_timeout_sec").value),
        )
        self._roi_config_file = Path(str(self.get_parameter("roi_config_file").value)).expanduser()
        self._try_autofocus = bool(self.get_parameter("try_autofocus").value)
        self._roi = self._load_roi()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._image_publisher = self.create_publisher(
            Image, str(self.get_parameter("image_topic").value), image_qos
        )
        rank_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._rank_publisher = self.create_publisher(
            Int32,
            str(self.get_parameter("rank_topic").value),
            rank_qos,
        )
        self.create_service(
            Trigger, str(self.get_parameter("start_service").value), self._start_callback
        )
        self.create_service(
            Trigger, str(self.get_parameter("stop_service").value), self._stop_callback
        )
        self.create_service(
            Trigger, str(self.get_parameter("result_service").value), self._result_callback
        )

        self._lock = threading.RLock()
        self._capture: Optional[cv2.VideoCapture] = None
        self._active = False
        self._frame_number = 0
        self._sharp_frame_pool: deque[tuple[float, np.ndarray]] = deque(maxlen=SHARPEST_FRAME_POOL)
        self._vote_history: deque[tuple[Optional[str], float]] = deque(maxlen=VOTE_WINDOW)
        self._last_result: Optional[OCRResult] = None
        self._last_result_received_at = 0.0
        self._stable_rank: Optional[str] = None
        self._last_published_rank = 0
        self._stable_count = 0
        self._stable_weight = 0.0
        self._recognition_started_at = 0.0
        self._model = None
        self._engine_name = ""
        self._model_ready = False
        self._model_error = PADDLE_IMPORT_ERROR
        self._input_queue: queue.Queue = queue.Queue(maxsize=1)
        self._output_queue: queue.Queue = queue.Queue(maxsize=1)
        self._worker_stop = threading.Event()
        self._worker: Optional[OCRWorker] = None

        self._timer = self.create_timer(1.0 / max(1.0, self._camera_fps), self._timer_callback)
        threading.Thread(target=self._initialize_model, daemon=True).start()
        self.get_logger().info(
            f"이벤트 등수 OCR 노드 대기 중: camera={self._camera_label}, "
            f"topic={self.get_parameter('image_topic').value}"
        )

    def _load_roi(self) -> tuple[float, float, float, float]:
        try:
            data = json.loads(self._roi_config_file.read_text(encoding="utf-8"))
            values = (float(data["x"]), float(data["y"]), float(data["w"]), float(data["h"]))
            x, y, width, height = values
            width = float(np.clip(width, 0.08, 1.0))
            height = float(np.clip(height, 0.06, 1.0))
            x = float(np.clip(x, 0.0, 1.0 - width))
            y = float(np.clip(y, 0.0, 1.0 - height))

            # 구버전의 지나치게 넓은 기본 ROI가 rank_roi.json에 저장되어 있으면
            # 인식 전용 모델에 맞는 새 좁은 ROI로 자동 마이그레이션합니다.
            legacy_roi = (0.15, 0.20, 0.70, 0.60)
            loaded_roi = (x, y, width, height)
            if all(abs(current - legacy) < 1e-6 for current, legacy in zip(loaded_roi, legacy_roi)):
                return DEFAULT_ROI

            return loaded_roi
        except Exception:
            return DEFAULT_ROI

    def _initialize_model(self) -> None:
        if TextRecognition is None or paddle is None:
            error = PADDLE_IMPORT_ERROR or "PaddleOCR TextRecognition을 불러오지 못했습니다."
            with self._lock:
                self._model_error = error
                self._engine_name = "PaddleOCR unavailable"
            self.get_logger().error(
                f"이벤트 등수 OCR은 PaddleOCR 전용입니다. 초기화 실패: {error}"
            )
            return

        try:
            # This small event model is intentionally kept on CPU. It avoids CUDA /
            # Paddle binary mismatches and GPU-memory contention with the kiosk's
            # Whisper, vision, and TTS nodes while remaining fast enough for one line.
            device = "cpu"

            self.get_logger().info(
                f"PaddleOCR 모델 로딩 중: {MODEL_NAME}, device={device}"
            )
            model = TextRecognition(
                model_name=MODEL_NAME,
                device=device,
                engine=ENGINE,
                enable_mkldnn=(device == "cpu"),
                cpu_threads=CPU_THREADS,
            )
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            with self._lock:
                self._model_error = error
                self._engine_name = "PaddleOCR initialization failed"
            self.get_logger().error(
                f"PaddleOCR 초기화 실패. Tesseract로 전환하지 않습니다: {error}"
            )
            return

        worker = OCRWorker(model, self._input_queue, self._output_queue, self._worker_stop)
        worker.start()
        with self._lock:
            self._model = model
            self._worker = worker
            self._model_ready = True
            self._model_error = None
            self._engine_name = "PaddleOCR PP-OCRv5 Korean"
        self.get_logger().info(
            "이벤트 등수 OCR 준비 완료: engine=PaddleOCR PP-OCRv5 Korean"
        )

    def _open_camera(self) -> None:
        deadline = time.monotonic() + self._camera_open_timeout_sec
        last_error = "카메라를 열지 못했습니다."
        while time.monotonic() < deadline:
            capture = cv2.VideoCapture(self._camera_source, cv2.CAP_V4L2)
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._camera_width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._camera_height)
            capture.set(cv2.CAP_PROP_FPS, self._camera_fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if self._try_autofocus:
                capture.set(cv2.CAP_PROP_AUTOFOCUS, 1)
            if not capture.isOpened():
                last_error = "장치가 아직 열리지 않았습니다."
                capture.release()
                time.sleep(0.20)
                continue
            ok, frame = capture.read()
            if not ok or frame is None:
                last_error = "첫 프레임을 읽지 못했습니다."
                capture.release()
                time.sleep(0.20)
                continue
            self._capture = capture
            return
        raise RuntimeError(f"{self._camera_label}: {last_error}")

    def _release_camera(self) -> None:
        capture = self._capture
        self._capture = None
        if capture is not None:
            capture.release()

    def _reset_recognition(self) -> None:
        self._frame_number = 0
        self._sharp_frame_pool.clear()
        self._vote_history.clear()
        self._last_result = None
        self._last_result_received_at = 0.0
        self._stable_rank = None
        self._publish_rank(0)
        self._stable_count = 0
        self._stable_weight = 0.0
        self._recognition_started_at = 0.0
        try:
            while True:
                self._input_queue.get_nowait()
        except queue.Empty:
            pass
        try:
            while True:
                self._output_queue.get_nowait()
        except queue.Empty:
            pass

    def _start_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            if self._active:
                response.success = True
                response.message = "이벤트 등수 OCR 카메라가 이미 실행 중입니다."
                return response
            try:
                self._reset_recognition()
                self._open_camera()
                self._recognition_started_at = time.monotonic()
                self._active = True
                response.success = True
                response.message = (
                    f"이벤트 등수 OCR을 시작했습니다. camera={self._camera_label}"
                )
                self.get_logger().info(response.message)
            except Exception as exc:
                self._release_camera()
                response.success = False
                response.message = f"이벤트 등수 OCR 시작 실패: {exc}"
                self.get_logger().error(response.message)
        return response

    def _stop_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            self._active = False
            self._release_camera()
            self._publish_rank(0)
            response.success = True
            response.message = (
                f"이벤트 등수 OCR을 중지하고 카메라를 해제했습니다: "
                f"{self._camera_label}"
            )
            self.get_logger().info(response.message)
        return response

    def _result_payload(self) -> dict[str, Any]:
        fresh = (
            self._last_result is not None
            and time.monotonic() - self._last_result_received_at <= RESULT_TIMEOUT_SECONDS
        )
        recognized = bool(fresh and self._stable_rank)
        rank = int(self._stable_rank[0]) if recognized and self._stable_rank else None
        raw_text = ""
        score = 0.0
        elapsed_ms = 0.0
        error = self._model_error
        if self._last_result is not None:
            elapsed_ms = self._last_result.elapsed_ms
            error = self._last_result.error or error
            candidate = self._last_result.best
            if candidate is None and self._last_result.candidates:
                candidate = max(self._last_result.candidates, key=lambda item: item.model_score)
            if candidate is not None:
                raw_text = candidate.raw_text
                score = candidate.model_score
        if recognized:
            status = "recognized"
        elif not self._active:
            status = "camera_inactive"
        elif not self._model_ready:
            status = "model_error" if self._model_error else "model_loading"
        elif error:
            status = "ocr_error"
        else:
            status = "prize_not_recognized"
        return {
            "recognized": recognized,
            "rank": rank,
            "status": status,
            "raw_text": raw_text,
            "score": score,
            "vote_count": self._stable_count,
            "vote_weight": self._stable_weight,
            "ocr_ms": elapsed_ms,
            "camera_active": self._active,
            "model_ready": self._model_ready,
            "error": error or "",
            "engine": self._engine_name,
        }

    def _result_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._lock:
            payload = self._result_payload()
        response.success = not bool(payload.get("error"))
        response.message = json.dumps(payload, ensure_ascii=False)
        return response

    def _roi_rect(self, frame: np.ndarray) -> tuple[int, int, int, int]:
        height, width = frame.shape[:2]
        x, y, roi_width, roi_height = self._roi
        x1 = int(round(x * width))
        y1 = int(round(y * height))
        x2 = int(round((x + roi_width) * width))
        y2 = int(round((y + roi_height) * height))
        x1 = int(np.clip(x1, 0, max(0, width - 2)))
        y1 = int(np.clip(y1, 0, max(0, height - 2)))
        x2 = int(np.clip(x2, x1 + 2, width))
        y2 = int(np.clip(y2, y1 + 2, height))
        return x1, y1, x2, y2

    def _drain_results(self) -> None:
        try:
            while True:
                result = self._output_queue.get_nowait()
                self._last_result = result
                self._last_result_received_at = time.monotonic()
                if result.best is None:
                    self._vote_history.append((None, 0.0))
                else:
                    self._vote_history.append((result.best.rank, result.best.adjusted_score))
        except queue.Empty:
            pass
        candidate_rank, self._stable_count, self._stable_weight = get_stable_rank(
            self._vote_history
        )
        visible_long_enough = (
            self._recognition_started_at > 0.0
            and time.monotonic() - self._recognition_started_at
            >= MIN_OCR_VISIBLE_SECONDS
        )
        self._stable_rank = candidate_rank if visible_long_enough else None
        if self._stable_rank:
            self._publish_rank(int(self._stable_rank[0]))

    def _publish_rank(self, rank: int) -> None:
        normalized_rank = int(rank) if int(rank) in (1, 2, 3, 4) else 0
        if normalized_rank == self._last_published_rank:
            return
        message = Int32()
        message.data = normalized_rank
        self._rank_publisher.publish(message)
        self._last_published_rank = normalized_rank
        if normalized_rank:
            self.get_logger().info(
                f"GUI 전달용 당첨 등수 발행: {normalized_rank}등"
            )

    def _publish_frame(self, frame: np.ndarray) -> None:
        height, width = frame.shape[:2]
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "event_rank_ocr_camera"
        message.height = height
        message.width = width
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = width * 3
        message.data = np.ascontiguousarray(frame).tobytes()
        self._image_publisher.publish(message)

    def _timer_callback(self) -> None:
        with self._lock:
            if not self._active or self._capture is None:
                return
            ok, frame = self._capture.read()
            if not ok or frame is None:
                self.get_logger().warning("이벤트 OCR 카메라 프레임을 읽지 못했습니다.")
                return
            self._frame_number += 1
            x1, y1, x2, y2 = self._roi_rect(frame)
            roi = frame[y1:y2, x1:x2]
            sharpness = roi_sharpness(roi)
            self._sharp_frame_pool.append((sharpness, roi.copy()))

            if (
                self._model_ready
                and self._frame_number % OCR_EVERY_N_FRAMES == 0
                and self._input_queue.empty()
                and self._sharp_frame_pool
            ):
                best_sharpness, best_roi = max(
                    self._sharp_frame_pool, key=lambda item: item[0]
                )
                self._sharp_frame_pool.clear()
                try:
                    self._input_queue.put_nowait((best_roi, best_sharpness))
                except queue.Full:
                    pass

            self._drain_results()
            annotated = frame.copy()
            color = (0, 255, 0) if self._stable_rank else (0, 230, 255)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 3)
            label = (
                f"RANK {self._stable_rank[0]}"
                if self._stable_rank
                else ("MODEL LOADING" if not self._model_ready else "SHOW 1-4 RANK")
            )
            cv2.putText(
                annotated,
                label,
                (25, 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                color,
                3,
                cv2.LINE_AA,
            )
            self._publish_frame(annotated)

    def destroy_node(self) -> bool:
        with self._lock:
            self._active = False
            self._release_camera()
        self._worker_stop.set()
        try:
            self._input_queue.put_nowait(None)
        except queue.Full:
            pass
        worker = self._worker
        if worker is not None and worker.is_alive():
            worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealtimeRankOCRNode()
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
