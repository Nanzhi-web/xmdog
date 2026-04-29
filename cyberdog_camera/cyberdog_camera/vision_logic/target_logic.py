from pathlib import Path

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory

_original_copyMakeBorder = cv2.copyMakeBorder


def _safe_copyMakeBorder(src, top, bottom, left, right, borderType, *args, **kwargs):
    return _original_copyMakeBorder(
        src,
        int(top),
        int(bottom),
        int(left),
        int(right),
        borderType,
        *args,
        **kwargs,
    )


cv2.copyMakeBorder = _safe_copyMakeBorder

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


_DEFAULT_RESULT = {
    "found": False,
    "type": "none",
    "distance": 99.9,
}
_ALLOWED_CLASSES = {"coke", "orange_ball", "soccer"}
_MODEL = None
_MODEL_LOAD_FAILED = False
_MODEL_PATH = Path(get_package_share_directory("cyberdog_camera")) / "model" / "best.pt"
_CENTER_TIE_THRESHOLD = 0.05

_DISTANCE_RULES = {
    "soccer": {
        "near_area_ratio_threshold": 0.038594,
        "mid_area_ratio_threshold": 0.007337,
        "near_distance": 0.5,
        "mid_distance": 1.0,
        "far_distance": 2.0,
    },
    "orange_ball": {
        "near_area_ratio_threshold": 0.138135,
        "mid_area_ratio_threshold": 0.012109,
        "near_distance": 0.5,
        "mid_distance": 1.0,
        "far_distance": 2.0,
    },
    "coke": {
        "near_area_ratio_threshold": 0.056318,
        "mid_area_ratio_threshold": 0.018418,
        "near_distance": 0.5,
        "mid_distance": 1.0,
        "far_distance": 2.0,
    },
}
_DEFAULT_DISTANCE_RULE = {
    "near_area_ratio_threshold": 0.023519,
    "mid_area_ratio_threshold": 0.02,
    "near_distance": 0.5,
    "mid_distance": 1.0,
    "far_distance": 2.0,
}


def _load_model():
    global _MODEL, _MODEL_LOAD_FAILED

    if _MODEL is not None:
        return _MODEL
    if _MODEL_LOAD_FAILED:
        return None
    if YOLO is None or not _MODEL_PATH.exists():
        _MODEL_LOAD_FAILED = True
        return None

    try:
        _MODEL = YOLO(str(_MODEL_PATH))
    except Exception:
        _MODEL_LOAD_FAILED = True
        return None

    return _MODEL


def _estimate_distance_monocular(target_type, box_xyxy, frame_shape):
    x1, y1, x2, y2 = box_xyxy
    height, width = frame_shape[:2]
    frame_area = float(max(height * width, 1))
    box_area = float(max((x2 - x1) * (y2 - y1), 1.0))
    area_ratio = box_area / frame_area
    distance_rule = _DISTANCE_RULES.get(target_type, _DEFAULT_DISTANCE_RULE)

    if area_ratio > distance_rule["near_area_ratio_threshold"]:
        return distance_rule["near_distance"]
    if area_ratio > distance_rule["mid_area_ratio_threshold"]:
        return distance_rule["mid_distance"]
    return distance_rule["far_distance"]


def _select_best_match(candidates, frame_width):
    if not candidates:
        return None

    frame_center_x = frame_width / 2.0
    for candidate in candidates:
        x1, _, x2, _ = candidate["xyxy"]
        candidate_center_x = (x1 + x2) / 2.0
        candidate["center_offset_abs"] = abs(candidate_center_x - frame_center_x) / max(frame_center_x, 1.0)

    candidates.sort(key=lambda item: item["center_offset_abs"])
    best = candidates[0]

    close_to_center = [
        candidate
        for candidate in candidates
        if abs(candidate["center_offset_abs"] - best["center_offset_abs"]) <= _CENTER_TIE_THRESHOLD
    ]

    if len(close_to_center) > 1:
        close_to_center.sort(key=lambda item: item["conf"], reverse=True)
        return close_to_center[0]

    return best


def detect_specific_targets(frame, depth_frame=None):
    del depth_frame

    if frame is None:
        return dict(_DEFAULT_RESULT)

    model = _load_model()
    if model is None:
        return dict(_DEFAULT_RESULT)

    try:
        frame = frame.astype("uint8")
        frame = np.ascontiguousarray(frame)
    except Exception:
        return dict(_DEFAULT_RESULT)

    try:
        results = model.predict(frame, imgsz=640, conf=0.5, verbose=False)
    except Exception:
        return dict(_DEFAULT_RESULT)

    if not results:
        return dict(_DEFAULT_RESULT)

    result = results[0]
    boxes = getattr(result, "boxes", None)
    names = getattr(result, "names", {})
    if boxes is None or len(boxes) == 0:
        return dict(_DEFAULT_RESULT)

    candidates = []

    for box in boxes:
        try:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            target_type = names.get(cls_id, "none")
            if target_type not in _ALLOWED_CLASSES:
                continue

            x1, y1, x2, y2 = [int(val) for val in box.xyxy[0].tolist()]
            candidates.append(
                {
                    "type": target_type,
                    "conf": conf,
                    "xyxy": [x1, y1, x2, y2],
                }
            )
        except Exception:
            continue

    best_match = _select_best_match(candidates, frame.shape[1])
    if best_match is None:
        return dict(_DEFAULT_RESULT)

    distance = _estimate_distance_monocular(best_match["type"], best_match["xyxy"], frame.shape)
    return {
        "found": True,
        "type": best_match["type"],
        "distance": distance,
    }
