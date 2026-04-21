import cv2
import numpy as np

def detect_black_bar(cv_image):
    try:
        if cv_image is None:
            return {'is_horizontal_bar': -1, 'angle': None, 'points': None, 'error': 'Image is None'}
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 60])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        hough_params = {
            'rho': 1,
            'theta': np.pi / 180,
            'threshold': 40,
            'minLineLength': int(mask.shape[1] * 0.2),
            'maxLineGap': 15
        }
        lines = cv2.HoughLinesP(mask, **hough_params)
        if lines is None:
            return {'is_horizontal_bar': -1, 'angle': None, 'points': None, 'reason': 'No lines detected'}
        candidates = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 > x2: x1, x2, y1, y2 = x2, x1, y2, y1
            dx = x2 - x1
            dy = y2 - y1
            angle = np.degrees(np.arctan2(dy, dx))
            if not (-10 <= angle <= 10): continue
            y_avg = (y1 + y2) / 2
            candidates.append({'angle': angle, 'points': (x1, y1, x2, y2), 'y_avg': y_avg, 'length': dx})
        if not candidates:
            return {'is_horizontal_bar': -1, 'angle': None, 'points': None, 'reason': 'No horizontal lines found'}
        best = max(candidates, key=lambda x: x['y_avg'])
        angle = best['angle']
        points = best['points']
        if abs(angle) < 1.1: is_horizontal = 3
        elif angle >= 1.1: is_horizontal = 2
        else: is_horizontal = 1
        return {'is_horizontal_bar': is_horizontal, 'angle': angle, 'points': points, 'y_position': best['y_avg'], 'success': True}
    except Exception as e:
        return {'is_horizontal_bar': -1, 'angle': None, 'points': None, 'error': str(e)}

def detect_height_distance(cv_image):
    try:
        if cv_image is None:
            return 'far', {'error': 'Image is None'}
        bar_result = detect_black_bar(cv_image)
        if not bar_result.get('success', False):
            return 'far', {'reason': 'No bar detected', 'default': True, 'details': bar_result}
        img_height = cv_image.shape[0]
        y_position = bar_result.get('y_position')
        if y_position is None:
            return 'far', {'reason': 'Could not determine bar position'}
        distance_ratio = y_position / img_height
        DISTANCE_THRESHOLD = 0.5
        if distance_ratio > DISTANCE_THRESHOLD:
            distance = 'near'
        else:
            distance = 'far'
        return distance, {'success': True, 'y_position': y_position, 'img_height': img_height, 'distance_ratio': distance_ratio, 'threshold': DISTANCE_THRESHOLD, 'bar_details': bar_result, 'distance': distance}
    except Exception as e:
        return 'far', {'error': str(e)}
