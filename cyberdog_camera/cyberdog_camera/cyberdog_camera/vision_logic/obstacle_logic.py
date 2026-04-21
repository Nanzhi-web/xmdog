import cv2
import numpy as np


# ==========================
# 1. 灰色横杆检测（限高杆）
# ==========================

def detect_gray_bar(cv_image):
    try:
        if cv_image is None:
            return {
                'is_horizontal_bar': -1,
                'angle': None,
                'points': None,
                'error': 'Image is None'
            }

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_gray = np.array([0, 0, 40], dtype=np.uint8)
        upper_gray = np.array([180, 40, 200], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_gray, upper_gray)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        hough_params = {
            'rho': 1,
            'theta': np.pi / 180,
            'threshold': 40,
            'minLineLength': int(mask.shape[1] * 0.3),
            'maxLineGap': 20
        }
        lines = cv2.HoughLinesP(mask, **hough_params)

        if lines is None:
            return {
                'is_horizontal_bar': -1,
                'angle': None,
                'points': None,
                'reason': 'No lines detected'
            }

        candidates = []
        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x1 > x2:
                x1, x2, y1, y2 = x2, x1, y2, y1

            dx = x2 - x1
            dy = y2 - y1
            if dx == 0:
                continue

            angle = np.degrees(np.arctan2(dy, dx))

            if not (-15.0 <= angle <= 15.0):
                continue

            y_avg = (y1 + y2) / 2.0
            length = dx

            if length < 0.4 * cv_image.shape[1]:
                continue

            candidates.append({
                'angle': angle,
                'points': (x1, y1, x2, y2),
                'y_avg': y_avg,
                'length': length
            })

        if not candidates:
            return {
                'is_horizontal_bar': -1,
                'angle': None,
                'points': None,
                'reason': 'No horizontal lines found'
            }

        best = max(candidates, key=lambda x: x['y_avg'])
        angle = best['angle']
        points = best['points']

        if abs(angle) < 1.1:
            is_horizontal = 3
        elif angle >= 1.1:
            is_horizontal = 2
        else:
            is_horizontal = 1

        return {
            'is_horizontal_bar': is_horizontal,
            'angle': angle,
            'points': points,
            'y_position': best['y_avg'],
            'success': True
        }
    except Exception as e:
        return {
            'is_horizontal_bar': -1,
            'angle': None,
            'points': None,
            'error': str(e)
        }


def detect_height_distance(cv_image):
    try:
        if cv_image is None:
            return 'far', {'error': 'Image is None'}

        bar_result = detect_gray_bar(cv_image)
        if not bar_result.get('success', False):
            return 'far', {
                'reason': 'No bar detected',
                'default': True,
                'details': bar_result
            }

        img_height = cv_image.shape[0]
        y_position = bar_result.get('y_position')
        if y_position is None:
            return 'far', {'reason': 'Could not determine bar position'}

        distance_ratio = y_position / float(img_height)

        DISTANCE_THRESHOLD = 0.5

        if distance_ratio > DISTANCE_THRESHOLD:
            distance = 'near'
        else:
            distance = 'far'

        return distance, {
            'success': True,
            'y_position': y_position,
            'img_height': img_height,
            'distance_ratio': distance_ratio,
            'threshold': DISTANCE_THRESHOLD,
            'bar_details': bar_result,
            'distance': distance
        }
    except Exception as e:
        return 'far', {'error': str(e)}


# ==========================
# 2. 蓝色障碍物检测（左右下角填满）
# ==========================

def detect_blue_obstacle(frame):
    """
    规则：
    - 只看图像下半部分
    - 分成左下角、右下角两个区域
    - 如果左右两边都被蓝色“填得比较多”，则认为前方有障碍
    """
    try:
        if frame is None:
            return {
                "success": False,
                "left_fill": 0.0,
                "right_fill": 0.0,
                "obstacle_near": False
            }

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 蓝色范围，先给一个常用范围
        lower_blue = np.array([90, 80, 40], dtype=np.uint8)
        upper_blue = np.array([135, 255, 255], dtype=np.uint8)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        kernel = np.ones((5, 5), np.uint8)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

        # 只看下半部分
        y0 = int(h * 0.55)
        y1 = h

        # 左下角 / 右下角
        left_x0 = 0
        left_x1 = int(w * 0.35)

        right_x0 = int(w * 0.65)
        right_x1 = w

        left_roi = blue_mask[y0:y1, left_x0:left_x1]
        right_roi = blue_mask[y0:y1, right_x0:right_x1]

        if left_roi.size == 0 or right_roi.size == 0:
            return {
                "success": False,
                "left_fill": 0.0,
                "right_fill": 0.0,
                "obstacle_near": False
            }

        left_fill = np.count_nonzero(left_roi) / float(left_roi.size)
        right_fill = np.count_nonzero(right_roi) / float(right_roi.size)

        # 关键逻辑：
        # 左右下角都被蓝色填到一定程度才算 True
        LEFT_THRESHOLD = 0.18
        RIGHT_THRESHOLD = 0.18

        obstacle_near = (left_fill > LEFT_THRESHOLD) and (right_fill > RIGHT_THRESHOLD)

        # distance 这里给一个大概值：取左右填充较小的一边作为保守估计
        near_score = min(left_fill, right_fill)

        return {
            "success": True,
            "left_fill": left_fill,
            "right_fill": right_fill,
            "near_score": near_score,
            "obstacle_near": obstacle_near
        }

    except Exception as e:
        return {
            "success": False,
            "left_fill": 0.0,
            "right_fill": 0.0,
            "obstacle_near": False,
            "error": str(e)
        }


# ==========================
# 3. 统一接口
# ==========================

def detect_all(frame):
    """
    返回:
    {
        "height_limit_near": bool,
        "obstacle_near": bool,
        "distance": float
    }
    """
    result = {
        "height_limit_near": False,
        "obstacle_near": False,
        "distance": 99.9,
    }

    # 限高杆
    distance_flag, _ = detect_height_distance(frame)
    if distance_flag == 'near':
        result["height_limit_near"] = True

    # 蓝色障碍
    obs = detect_blue_obstacle(frame)
    if obs.get("success", False) and obs.get("obstacle_near", False):
        result["obstacle_near"] = True
        result["distance"] = float(obs.get("near_score", 0.0))
    else:
        result["obstacle_near"] = False
        result["distance"] = 99.9

    return result