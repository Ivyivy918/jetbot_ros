#!/usr/bin/env python3
"""
相機診斷工具 - 用於檢測 IMX219 相機問題
"""
import cv2
import os
import subprocess
import sys

def print_section(title):
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60)

def check_video_devices():
    """檢查 /dev/video 設備"""
    print_section("檢查視訊設備")

    devices = []
    for i in range(10):
        device = f'/dev/video{i}'
        if os.path.exists(device):
            devices.append(device)
            print(f"✓ 找到設備: {device}")

            # 嘗試獲取設備資訊
            try:
                result = subprocess.run(
                    ['v4l2-ctl', '--device', device, '--all'],
                    capture_output=True, text=True, timeout=2
                )
                if result.returncode == 0:
                    # 提取關鍵資訊
                    for line in result.stdout.split('\n'):
                        if 'Driver' in line or 'Card' in line or 'Bus' in line:
                            print(f"  {line.strip()}")
                        if 'Format' in line or 'Width' in line or 'Height' in line:
                            print(f"  {line.strip()}")
            except:
                print(f"  無法獲取設備詳細資訊")

    if not devices:
        print("✗ 未找到任何視訊設備！")
        print("  請檢查：")
        print("  1. 相機是否正確連接")
        print("  2. CSI 排線是否插好")
        print("  3. 是否需要重新啟動系統")
        return False

    return True

def check_gstreamer():
    """檢查 GStreamer 安裝"""
    print_section("檢查 GStreamer")

    # 檢查 gst-launch
    try:
        result = subprocess.run(['which', 'gst-launch-1.0'],
                              capture_output=True, timeout=2)
        if result.returncode == 0:
            print("✓ GStreamer 已安裝")
        else:
            print("✗ GStreamer 未安裝")
            return False
    except:
        print("✗ 無法檢查 GStreamer")
        return False

    # 檢查 nvarguscamerasrc (Jetson 專用)
    try:
        result = subprocess.run(
            ['gst-inspect-1.0', 'nvarguscamerasrc'],
            capture_output=True, timeout=2
        )
        if result.returncode == 0:
            print("✓ NVIDIA Argus 插件可用 (推薦)")
        else:
            print("ℹ NVIDIA Argus 插件不可用 (將使用 V4L2)")
    except:
        pass

    # 檢查 v4l2src
    try:
        result = subprocess.run(
            ['gst-inspect-1.0', 'v4l2src'],
            capture_output=True, timeout=2
        )
        if result.returncode == 0:
            print("✓ V4L2 插件可用")
        else:
            print("✗ V4L2 插件不可用")
            return False
    except:
        pass

    return True

def test_camera_with_opencv(sensor_id):
    """測試 OpenCV 相機讀取"""
    print_section(f"測試 OpenCV 讀取 (sensor_id={sensor_id})")

    try:
        cap = cv2.VideoCapture(sensor_id)
        if not cap.isOpened():
            print(f"✗ 無法開啟相機 {sensor_id}")
            return False

        ret, frame = cap.read()
        if not ret or frame is None:
            print(f"✗ 無法讀取畫面")
            cap.release()
            return False

        print(f"✓ 成功讀取畫面: {frame.shape}")
        print(f"  尺寸: {frame.shape[1]}x{frame.shape[0]}")
        print(f"  通道: {frame.shape[2] if len(frame.shape) > 2 else 1}")

        cap.release()
        return True

    except Exception as e:
        print(f"✗ 發生錯誤: {e}")
        return False

def test_camera_with_gstreamer(sensor_id, pipeline_type):
    """測試 GStreamer 管道"""
    print_section(f"測試 GStreamer {pipeline_type} (sensor_id={sensor_id})")

    pipelines = {
        'argus': (
            f'nvarguscamerasrc sensor-id={sensor_id} ! '
            f'video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! '
            f'nvvidconv ! '
            f'video/x-raw, width=640, height=480, format=BGRx ! '
            f'videoconvert ! '
            f'video/x-raw, format=BGR ! '
            f'appsink max-buffers=1 drop=true'
        ),
        'v4l2_bayer_rggb': (
            f'v4l2src device=/dev/video{sensor_id} ! '
            f'video/x-bayer, width=1280, height=720, framerate=30/1, format=rggb ! '
            f'bayer2rgb ! '
            f'videoscale ! '
            f'video/x-raw, width=640, height=480 ! '
            f'videoconvert ! '
            f'video/x-raw, format=BGR ! '
            f'appsink max-buffers=1 drop=true'
        ),
        'v4l2_bayer_grbg': (
            f'v4l2src device=/dev/video{sensor_id} ! '
            f'video/x-bayer, width=1280, height=720, framerate=30/1, format=grbg ! '
            f'bayer2rgb ! '
            f'videoscale ! '
            f'video/x-raw, width=640, height=480 ! '
            f'videoconvert ! '
            f'video/x-raw, format=BGR ! '
            f'appsink max-buffers=1 drop=true'
        ),
        'v4l2_simple': (
            f'v4l2src device=/dev/video{sensor_id} ! '
            f'video/x-raw, width=1280, height=720, framerate=30/1 ! '
            f'videoscale ! '
            f'video/x-raw, width=640, height=480 ! '
            f'videoconvert ! '
            f'video/x-raw, format=BGR ! '
            f'appsink max-buffers=1 drop=true'
        ),
    }

    if pipeline_type not in pipelines:
        print(f"✗ 未知的管道類型: {pipeline_type}")
        return False

    pipeline = pipelines[pipeline_type]
    print(f"管道: {pipeline}")

    try:
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print(f"✗ 無法開啟 GStreamer 管道")
            return False

        # 嘗試讀取幾幀
        success_count = 0
        for i in range(5):
            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                success_count += 1

        if success_count == 0:
            print(f"✗ 無法讀取畫面")
            cap.release()
            return False

        print(f"✓ 成功讀取 {success_count}/5 幀")
        print(f"  尺寸: {frame.shape[1]}x{frame.shape[0]}")
        print(f"  通道: {frame.shape[2] if len(frame.shape) > 2 else 1}")

        # 檢查是否為綠屏 (簡單啟發式)
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            mean_color = frame.mean(axis=(0, 1))
            print(f"  平均顏色 (BGR): {mean_color}")

            # 如果綠色通道明顯高於其他通道，可能是綠屏
            if mean_color[1] > mean_color[0] * 1.5 and mean_color[1] > mean_color[2] * 1.5:
                print(f"  ⚠ 警告: 可能存在綠屏問題 (綠色通道異常高)")

        cap.release()
        return True

    except Exception as e:
        print(f"✗ 發生錯誤: {e}")
        return False

def main():
    print("\n")
    print("╔" + "═"*58 + "╗")
    print("║" + " "*15 + "IMX219 相機診斷工具" + " "*22 + "║")
    print("╚" + "═"*58 + "╝")

    # 1. 檢查視訊設備
    if not check_video_devices():
        print("\n請先確認相機硬體連接後再執行此診斷工具")
        sys.exit(1)

    # 2. 檢查 GStreamer
    has_gstreamer = check_gstreamer()

    # 3. 測試每個相機
    for sensor_id in [0, 1]:
        device = f'/dev/video{sensor_id}'
        if not os.path.exists(device):
            continue

        print(f"\n{'='*60}")
        print(f"  測試相機 {sensor_id} ({device})")
        print(f"{'='*60}")

        # 測試 OpenCV
        opencv_ok = test_camera_with_opencv(sensor_id)

        if has_gstreamer:
            # 測試 Argus
            argus_ok = test_camera_with_gstreamer(sensor_id, 'argus')

            # 測試 V4L2 Bayer (RGGB)
            v4l2_rggb_ok = test_camera_with_gstreamer(sensor_id, 'v4l2_bayer_rggb')

            # 測試 V4L2 Bayer (GRBG)
            v4l2_grbg_ok = test_camera_with_gstreamer(sensor_id, 'v4l2_bayer_grbg')

            # 測試 V4L2 Simple
            v4l2_simple_ok = test_camera_with_gstreamer(sensor_id, 'v4l2_simple')

    # 總結
    print_section("診斷總結")
    print("如果所有測試都失敗，請檢查：")
    print("  1. 相機是否正確連接到 CSI 接口")
    print("  2. CSI 排線方向是否正確")
    print("  3. 是否有硬體故障")
    print("  4. Jetson 是否已正確安裝驅動")
    print("\n如果出現綠屏，建議：")
    print("  1. 嘗試不同的 Bayer 格式 (rggb, grbg, bggr, gbrg)")
    print("  2. 使用 NVIDIA Argus 管道 (如果可用)")
    print("  3. 檢查相機是否需要重新校正")
    print("  4. 確認 JetPack 版本與驅動兼容")
    print("")

if __name__ == '__main__':
    main()
