import cv2
import numpy as np

def detect_arrow_direction(cv_image):
    try:
        if cv_image is None:
            return 'stone', {'error': 'Image is None'}
        height, width = cv_image.shape[:2]
        whole_image = cv_image[:, :width]
        hsv = cv2.cvtColor(whole_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        scan_results = []
        for col in range(mask.shape[1]):
            column = mask[:, col]
            in_segment = False
            current_length = 0
            max_length = 0
            for pixel in column:
                if pixel == 255:
                    current_length += 1
                    if not in_segment:
                        in_segment = True
                else:
                    if in_segment:
                        if current_length > max_length:
                            max_length = current_length
                        current_length = 0
                        in_segment = False
            if in_segment and current_length > max_length:
                max_length = current_length
            if max_length > 0:
                scan_results.append(max_length)
        if len(scan_results) == 0:
            return 'stone', {
                'reason': 'No green arrow detected',
                'scan_results_count': 0
            }
        significant_changes_index = 0
        change_threshold = 1
        for i in range(1, len(scan_results)):
            if abs(scan_results[i] - scan_results[i-1]) > change_threshold:
                change_threshold = abs(scan_results[i] - scan_results[i-1])
                significant_changes_index = i
        arrow_direction = 'slope' if significant_changes_index > len(scan_results)//2 else 'stone'
        return arrow_direction, {
            'scan_results_count': len(scan_results),
            'significant_change_index': significant_changes_index,
            'scan_results_max': max(scan_results) if scan_results else 0,
            'arrow_direction': arrow_direction
        }
    except Exception as e:
        return 'stone', {'error': str(e)}
