import cv2
import numpy as np


def _empty_ball_result():
    return {
        "found": False,
        "color": "",
        "x_offset": 0.0,
        "ratio": 0.0,
    }


def _empty_soccer_result():
    return {
        "found": False,
        "x_offset": 0.0,
    }


def _empty_depth_soccer_details():
    return {
        "found": False,
        "center_x": 0.0,
        "center_y": 0.0,
        "radius": 0.0,
        "x_offset": 0.0,
    }


def _normalize_offset(center_x, image_width):
    if image_width <= 0:
        return 0.0
    return float(np.clip((center_x - (image_width / 2.0)) / (image_width / 2.0), -1.0, 1.0))


def _prepare_mask(frame, lower, upper):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def _collect_round_candidates(mask, image_shape, color_name):
    height, width = image_shape[:2]
    image_area = float(max(height * width, 1))
    candidates = []

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 80.0:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0.0:
            continue

        circularity = (4.0 * np.pi * area) / (perimeter * perimeter)
        if circularity < 0.45:
            continue

        (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
        if radius < 6.0:
            continue

        bbox_x, bbox_y, bbox_w, bbox_h = cv2.boundingRect(contour)
        ratio = float((bbox_w * bbox_h) / image_area)

        candidates.append(
            {
                "color": color_name,
                "center_x": float(center_x),
                "center_y": float(center_y),
                "radius": float(radius),
                "ratio": ratio,
                "score": area,
            }
        )

    return candidates


def _soccer_candidates(frame):
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    white_mask = _prepare_mask(
        blurred,
        np.array([0, 0, 170], dtype=np.uint8),
        np.array([180, 70, 255], dtype=np.uint8),
    )
    return _collect_round_candidates(white_mask, frame.shape, "soccer")


def _depth_to_meters(depth_frame):
    if depth_frame is None or depth_frame.size == 0:
        return None

    original_dtype = depth_frame.dtype
    depth = depth_frame.astype(np.float32)
    if original_dtype.kind in ("u", "i"):
        depth = depth / 1000.0
    elif np.nanmax(depth) > 20.0:
        depth = depth / 1000.0

    depth[~np.isfinite(depth)] = 0.0
    return depth


def _depth_soccer_candidates(depth_frame):
    depth_m = _depth_to_meters(depth_frame)
    if depth_m is None:
        return []

    height, width = depth_m.shape[:2]
    if height == 0 or width == 0:
        return []

    valid_mask = (depth_m > 0.08) & (depth_m < 3.5)
    if np.count_nonzero(valid_mask) < 100:
        return []

    roi = np.zeros_like(valid_mask, dtype=np.uint8)
    roi[int(height * 0.15):, :] = 1
    valid_roi = valid_mask & roi.astype(bool)
    valid_depths = depth_m[valid_roi]
    if valid_depths.size < 100:
        return []

    near_threshold = float(np.percentile(valid_depths, 8))
    candidate_mask = (depth_m > 0.08) & (depth_m <= near_threshold + 0.10) & valid_roi
    candidate_mask = candidate_mask.astype(np.uint8) * 255

    kernel = np.ones((5, 5), np.uint8)
    candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_OPEN, kernel)
    candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(candidate_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    image_area = float(max(height * width, 1))

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 80.0:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0.0:
            continue

        circularity = (4.0 * np.pi * area) / (perimeter * perimeter)
        if circularity < 0.25:
            continue

        center_x, center_y, bbox_w, bbox_h = cv2.boundingRect(contour)
        if bbox_w <= 0 or bbox_h <= 0:
            continue

        aspect_ratio = bbox_w / float(bbox_h)
        if not 0.5 <= aspect_ratio <= 1.8:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] <= 0.0:
            continue

        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]
        radius = 0.25 * (bbox_w + bbox_h)

        candidates.append(
            {
                "center_x": float(cx),
                "center_y": float(cy),
                "radius": float(radius),
                "score": float(area / image_area),
            }
        )

    return candidates


def detect_largest_ball(frame):
    result = _empty_ball_result()
    if frame is None or frame.size == 0:
        return result

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    candidates = []

    color_ranges = (
        ("orange", np.array([5, 100, 80], dtype=np.uint8), np.array([25, 255, 255], dtype=np.uint8)),
        ("blue", np.array([85, 70, 60], dtype=np.uint8), np.array([130, 255, 255], dtype=np.uint8)),
        ("red", np.array([0, 100, 70], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)),
        ("red", np.array([160, 100, 70], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8)),
    )

    for color_name, lower, upper in color_ranges:
        mask = _prepare_mask(blurred, lower, upper)
        candidates.extend(_collect_round_candidates(mask, frame.shape, color_name))

    if not candidates:
        return result

    best = max(candidates, key=lambda candidate: candidate["score"])
    result["found"] = True
    result["color"] = best["color"]
    result["x_offset"] = _normalize_offset(best["center_x"], frame.shape[1])
    result["ratio"] = float(np.clip(best["ratio"], 0.0, 1.0))
    return result


def detect_soccer_details(frame):
    if frame is None or frame.size == 0:
        return {
            "found": False,
            "center_x": 0.0,
            "center_y": 0.0,
            "radius": 0.0,
            "x_offset": 0.0,
        }

    candidates = _soccer_candidates(frame)
    if not candidates:
        return {
            "found": False,
            "center_x": 0.0,
            "center_y": 0.0,
            "radius": 0.0,
            "x_offset": 0.0,
        }

    best = max(candidates, key=lambda candidate: candidate["score"])
    return {
        "found": True,
        "center_x": best["center_x"],
        "center_y": best["center_y"],
        "radius": best["radius"],
        "x_offset": _normalize_offset(best["center_x"], frame.shape[1]),
    }


def detect_soccer(frame):
    details = detect_soccer_details(frame)
    result = _empty_soccer_result()
    result["found"] = details["found"]
    result["x_offset"] = details["x_offset"]
    return result


def detect_soccer_depth_details(depth_frame):
    if depth_frame is None or depth_frame.size == 0:
        return _empty_depth_soccer_details()

    candidates = _depth_soccer_candidates(depth_frame)
    if not candidates:
        return _empty_depth_soccer_details()

    best = max(candidates, key=lambda candidate: candidate["score"])
    return {
        "found": True,
        "center_x": best["center_x"],
        "center_y": best["center_y"],
        "radius": best["radius"],
        "x_offset": _normalize_offset(best["center_x"], depth_frame.shape[1]),
    }


def detect_soccer_depth(depth_frame):
    details = detect_soccer_depth_details(depth_frame)
    result = _empty_soccer_result()
    result["found"] = details["found"]
    result["x_offset"] = details["x_offset"]
    return result
