from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import os
import easyocr
from pyzbar.pyzbar import decode

def detect_qr_code(cv_image):
    print("[detect_qr_code] enter")
    try:
        if cv_image is None:
            print("[detect_qr_code] image is None")
            return False, None, {'error': 'Image is None'}
        print("[detect_qr_code] image shape:", None if cv_image is None else cv_image.shape)
        img = cv2.resize(cv_image[0:100, 0:300], None, fx=2.5, fy=2.5, interpolation=cv2.INTER_CUBIC)
        kernel_sharpening = np.array([[-1, -1, -1],[-1, 9, -1],[-1, -1, -1]])
        img = cv2.filter2D(img, -1, kernel_sharpening)
        img = cv2.convertScaleAbs(img, alpha=3, beta=50)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape)==3 else img
        decoded_objects = decode(gray)
        print("[detect_qr_code] decoded count:", len(decoded_objects) if decoded_objects is not None else 0)
        if not decoded_objects:
            print("[detect_qr_code] no qr detected by pyzbar")
            return False, None, {'reason': 'No QR code detected'}
        qr_data = decoded_objects[0].data.decode('utf-8')
        print("[detect_qr_code] qr_data:", qr_data)
        return True, qr_data, {
            'method': 'pyzbar',
            'detected_count': len(decoded_objects),
            'data': qr_data
        }
    except Exception as e:
        print("[detect_qr_code] exception:", e)
        return False, None, {'error': str(e)}

def detect_wechat_qr(cv_image):
    print("[detect_wechat_qr] enter")
    try:
        if cv_image is None:
            print("[detect_wechat_qr] image is None")
            return False, None, {'error': 'Image is None'}
        print("[detect_wechat_qr] image shape:", cv_image.shape)

        package_share_dir = get_package_share_directory('cyberdog_camera')
        
        # 2. 模型文件夹现在位于 'share/cyberdog_camera/qr_detector'
        model_dir = os.path.join(package_share_dir, 'qr_detector')
        # current_dir = os.path.dirname(os.path.abspath(__file__))
        # model_dir = os.path.join(current_dir, '..', 'qr_detector')
        model_files = [
            os.path.join(model_dir, 'detect.prototxt'),
            os.path.join(model_dir, 'detect.caffemodel'),
            os.path.join(model_dir, 'sr.prototxt'),
            os.path.join(model_dir, 'sr.caffemodel')
        ]
        for model_file in model_files:
            if not os.path.exists(model_file):
                print("[detect_wechat_qr] model file not found:", model_file)
                return False, None, {'error': f'Model file not found: {model_file}', 'model_dir': model_dir}
        print("[detect_wechat_qr] model files found, creating detector")
        detector = cv2.wechat_qrcode_WeChatQRCode(model_files[0], model_files[1], model_files[2], model_files[3])
        img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        res, _ = detector.detectAndDecode(img_rgb)
        print("[detect_wechat_qr] wechat res count:", len(res) if res is not None else 0)
        if not res or len(res) == 0:
            print("[detect_wechat_qr] no wechat qr detected")
            return False, None, {'reason': 'No WeChat QR code detected'}
        qr_data = res[0]
        print("[detect_wechat_qr] qr_data:", qr_data)
        return True, qr_data, {
            'method': 'wechat_detector',
            'detected_count': len(res),
            'data': qr_data
        }
    except Exception as e:
        print("[detect_wechat_qr] exception:", e)
        return False, None, {'error': str(e)}

def detect_ocr_text(cv_image, languages=['en']):
    print("[detect_ocr_text] enter")
    try:
        if cv_image is None:
            print("[detect_ocr_text] image is None")
            return False, None, {'error': 'Image is None'}
        print("[detect_ocr_text] image shape:", cv_image.shape)
        reader = easyocr.Reader(languages, gpu=False)
        results = reader.readtext(cv_image)
        print("[detect_ocr_text] raw results count:", len(results) if results is not None else 0)
        if not results:
            print("[detect_ocr_text] no text detected by easyocr")
            return False, None, {'reason': 'No text detected'}
        text_list = []
        details = {'detections': []}
        for (bbox, text, prob) in results:
            print("[detect_ocr_text] detection:", text, "prob:", prob)
            if prob > 0.3:
                text_list.append(text)
                details['detections'].append({'text': text,'confidence': prob,'bbox': bbox})
        print("[detect_ocr_text] filtered text count:", len(text_list))
        if not text_list:
            print("[detect_ocr_text] no high-confidence text")
            return False, None, {'reason': 'No high-confidence text'}
        combined_text = ''.join(text_list)
        print("[detect_ocr_text] combined_text:", combined_text)
        return True, combined_text, {
            'method': 'easyocr',
            'text_count': len(text_list),
            'text_list': text_list,
            'details': details['detections']
        }
    except Exception as e:
        print("[detect_ocr_text] exception:", e)
        return False, None, {'error': str(e)}

def detect_qr_with_fallback(cv_image):
    print("[detect_qr_with_fallback] enter")
    try:
        if cv_image is None:
            print("[detect_qr_with_fallback] image is None")
            return False, None, 'none', {'error': 'Image is None'}
        # 先尝试 wechat detector
        success, data, details = detect_wechat_qr(cv_image)
        print("[detect_qr_with_fallback] wechat attempt success:", success, "data:", data, "details:", details)
        if success:
            return success, data, 'wechat', details
        # 再尝试 pyzbar
        success, data, details = detect_qr_code(cv_image)
        print("[detect_qr_with_fallback] pyzbar attempt success:", success, "data:", data, "details:", details)
        if success:
            return success, data, 'pyzbar', details
        print("[detect_qr_with_fallback] all methods failed")
        return False, None, 'failed', {
            'wechat_result': details,
            'pyzbar_result': details,
            'reason': 'All QR code detection methods failed'
        }
    except Exception as e:
        print("[detect_qr_with_fallback] exception:", e)
        return False, None, 'error', {'error': str(e)}

def detect_target_from_qr(cv_image):
    print("[detect_target_from_qr] enter")
    try:
        success, qr_data, method, details = detect_qr_with_fallback(cv_image)
        print("[detect_target_from_qr] fallback returned:", success, qr_data, method, details)
        if success and qr_data:
            qr_str = str(qr_data).upper().strip()
            print("[detect_target_from_qr] qr_str:", qr_str)
            if 'A1' in qr_str or 'A-1' in qr_str: 
                print("[detect_target_from_qr] mapped to A-1")
                return 'A-1'
            elif 'A2' in qr_str or 'A-2' in qr_str:
                print("[detect_target_from_qr] mapped to A-2")
                return 'A-2'
            elif 'B1' in qr_str or 'B-1' in qr_str:
                print("[detect_target_from_qr] mapped to B-1")
                return 'B-1'
            elif 'B2' in qr_str or 'B-2' in qr_str:
                print("[detect_target_from_qr] mapped to B-2")
                return 'B-2'
            else:
                print("[detect_target_from_qr] returning raw qr_data")
                return qr_data
        success_ocr, text, details_ocr = detect_ocr_text(cv_image, languages=['en'])
        print("[detect_target_from_qr] ocr attempt returned:", success_ocr, text, details_ocr)
        if success_ocr and text:
            text_upper = str(text).upper()
            print("[detect_target_from_qr] ocr text_upper:", text_upper)
            if 'A1' in text_upper or 'A-1' in text_upper:
                print("[detect_target_from_qr] ocr mapped to A-1")
                return 'A-1'
            elif 'A2' in text_upper or 'A-2' in text_upper:
                print("[detect_target_from_qr] ocr mapped to A-2")
                return 'A-2'
            elif 'B1' in text_upper or 'B-1' in text_upper:
                print("[detect_target_from_qr] ocr mapped to B-1")
                return 'B-1'
            elif 'B2' in text_upper or 'B-2' in text_upper:
                print("[detect_target_from_qr] ocr mapped to B-2")
                return 'B-2'
        print("[detect_target_from_qr] no qr or ocr result -> return None")
        return None
    except Exception as e:
        print("[detect_target_from_qr] exception:", e)
        return None