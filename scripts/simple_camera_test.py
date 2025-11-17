#!/usr/bin/env python3
"""
簡單的相機測試工具 - 直接測試相機是否可用
"""
import cv2
import os
import sys

def test_device_exists():
    """檢查設備是否存在"""
    print("="*60)
    print("檢查視訊設備")
    print("="*60)

    for i in range(4):
        device = f'/dev/video{i}'
        if os.path.exists(device):
            print(f"✓ {device} 存在")
        else:
            print(f"✗ {device} 不存在")
    print()

def test_opencv_simple(device_id):
    """最簡單的 OpenCV 測試"""
    print(f"測試 OpenCV VideoCapture (device {device_id})...")

    try:
        cap = cv2.VideoCapture(device_id)

        if not cap.isOpened():
            print(f"✗ 無法開啟設備 {device_id}")
            return False

        print(f"✓ 設備 {device_id} 已開啟")

        # 嘗試讀取
        ret, frame = cap.read()

        if not ret or frame is None:
            print(f"✗ 無法讀取畫面")
            cap.release()
            return False

        print(f"✓ 成功讀取畫面: {frame.shape}")
        cap.release()
        return True

    except Exception as e:
        print(f"✗ 錯誤: {e}")
        return False

def test_gstreamer_argus(sensor_id):
    """測試 NVIDIA Argus 管道"""
    print(f"\n測試 NVIDIA Argus (sensor_id={sensor_id})...")

    # 使用與之前工作版本相同的 Argus 管道配置
    # 注意：使用 sensor_id (下劃線) 而非 sensor-id (連字符)
    pipeline = (
        f'nvarguscamerasrc sensor_id={sensor_id} ! '
        f'video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! '
        f'nvvidconv ! '
        f'video/x-raw, format=BGRx ! '
        f'videoconvert ! '
        f'video/x-raw, format=BGR ! '
        f'appsink drop=1 max-buffers=2'
    )

    print(f"管道: {pipeline}")

    try:
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            print(f"✗ 無法開啟 Argus 管道")
            return False

        print(f"✓ Argus 管道已開啟")

        ret, frame = cap.read()

        if not ret or frame is None:
            print(f"✗ 無法讀取畫面")
            cap.release()
            return False

        print(f"✓ 成功讀取畫面: {frame.shape}")
        cap.release()
        return True

    except Exception as e:
        print(f"✗ 錯誤: {e}")
        return False

def test_gstreamer_v4l2_simple(device_id):
    """測試最簡單的 V4L2 管道"""
    print(f"\n測試 V4L2 簡單管道 (/dev/video{device_id})...")

    # 最簡單的 V4L2 管道 - 使用自動格式
    pipeline = (
        f'v4l2src device=/dev/video{device_id} ! '
        f'videoconvert ! '
        f'video/x-raw, format=BGR ! '
        f'appsink'
    )

    print(f"管道: {pipeline}")

    try:
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            print(f"✗ 無法開啟 V4L2 管道")
            return False

        print(f"✓ V4L2 管道已開啟")

        ret, frame = cap.read()

        if not ret or frame is None:
            print(f"✗ 無法讀取畫面")
            cap.release()
            return False

        print(f"✓ 成功讀取畫面: {frame.shape}")
        cap.release()
        return True

    except Exception as e:
        print(f"✗ 錯誤: {e}")
        return False

def main():
    print("\n" + "="*60)
    print("     簡單相機測試工具")
    print("="*60 + "\n")

    # 1. 檢查設備
    test_device_exists()

    # 2. 測試每個可能的相機
    for device_id in [0, 1]:
        if not os.path.exists(f'/dev/video{device_id}'):
            continue

        print("\n" + "="*60)
        print(f"  測試相機 {device_id}")
        print("="*60)

        # 測試 OpenCV 簡單模式
        opencv_ok = test_opencv_simple(device_id)

        # 測試 Argus
        argus_ok = test_gstreamer_argus(device_id)

        # 測試 V4L2
        v4l2_ok = test_gstreamer_v4l2_simple(device_id)

        print(f"\n總結 (device {device_id}):")
        print(f"  OpenCV: {'✓' if opencv_ok else '✗'}")
        print(f"  Argus:  {'✓' if argus_ok else '✗'}")
        print(f"  V4L2:   {'✓' if v4l2_ok else '✗'}")

    print("\n" + "="*60)
    print("測試完成")
    print("="*60)

if __name__ == '__main__':
    main()
