#!/usr/bin/env python3
"""
簡單的相機測試程式 - 針對JetBot
"""
import cv2
import sys
import time

def test_camera_methods():
    """測試不同的相機初始化方法"""
    
    print("=== 相機連接測試 ===\n")
    
    # 測試方法列表
    test_methods = [
        {
            'name': 'USB相機 - /dev/video0',
            'source': '/dev/video0',
            'backend': cv2.CAP_V4L2
        },
        {
            'name': 'USB相機 - /dev/video1', 
            'source': '/dev/video1',
            'backend': cv2.CAP_V4L2
        },
        {
            'name': 'CSI相機 - sensor_id=0 (完整管道)',
            'source': 'nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1 max-buffers=2',
            'backend': cv2.CAP_GSTREAMER
        },
        {
            'name': 'CSI相機 - sensor_id=1 (完整管道)',
            'source': 'nvarguscamerasrc sensor_id=1 ! video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1 max-buffers=2',
            'backend': cv2.CAP_GSTREAMER
        },
        {
            'name': 'CSI相機 - sensor_id=0 (簡化管道)',
            'source': 'nvarguscamerasrc sensor_id=0 ! video/x-raw(memory:NVMM), width=640, height=480 ! nvvidconv ! video/x-raw, format=BGR ! videoconvert ! appsink',
            'backend': cv2.CAP_GSTREAMER
        },
        {
            'name': 'CSI相機 - sensor_id=1 (簡化管道)',
            'source': 'nvarguscamerasrc sensor_id=1 ! video/x-raw(memory:NVMM), width=640, height=480 ! nvvidconv ! video/x-raw, format=BGR ! videoconvert ! appsink',
            'backend': cv2.CAP_GSTREAMER
        },
        {
            'name': '預設相機 - index 0',
            'source': 0,
            'backend': cv2.CAP_ANY
        },
        {
            'name': '預設相機 - index 1',
            'source': 1,
            'backend': cv2.CAP_ANY
        }
    ]
    
    successful_methods = []
    
    for i, method in enumerate(test_methods):
        print(f"測試 {i+1}/8: {method['name']}")
        print(f"來源: {method['source']}")
        
        try:
            # 嘗試開啟相機
            if method['backend'] == cv2.CAP_ANY:
                cap = cv2.VideoCapture(method['source'])
            else:
                cap = cv2.VideoCapture(method['source'], method['backend'])
            
            if cap.isOpened():
                print("  ✓ 相機開啟成功")
                
                # 嘗試設定解析度
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                
                # 獲取實際解析度
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = cap.get(cv2.CAP_PROP_FPS)
                
                print(f"  解析度: {width}x{height}, FPS: {fps}")
                
                # 嘗試讀取幾幀影像
                success_count = 0
                for attempt in range(5):
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        success_count += 1
                        if attempt == 0:
                            print(f"  影像尺寸: {frame.shape}")
                    time.sleep(0.1)
                
                if success_count > 0:
                    print(f"  ✓ 成功讀取 {success_count}/5 幀影像")
                    successful_methods.append({
                        'method': method,
                        'success_rate': success_count / 5,
                        'resolution': (width, height),
                        'fps': fps
                    })
                else:
                    print("  ✗ 無法讀取影像")
                
            else:
                print("  ✗ 無法開啟相機")
            
            cap.release()
            
        except Exception as e:
            print(f"  ✗ 錯誤: {e}")
        
        print("-" * 50)
        time.sleep(0.5)  # 短暫延遲避免衝突
    
    # 總結結果
    print("\n=== 測試結果總結 ===")
    if successful_methods:
        print(f"找到 {len(successful_methods)} 種可用的相機連接方法：\n")
        
        for i, result in enumerate(successful_methods):
            method = result['method']
            print(f"{i+1}. {method['name']}")
            print(f"   來源: {method['source']}")
            print(f"   成功率: {result['success_rate']*100:.0f}%")
            print(f"   解析度: {result['resolution']}")
            print(f"   FPS: {result['fps']:.1f}")
            print()
        
        # 推薦最佳方法
        best_method = max(successful_methods, key=lambda x: x['success_rate'])
        print(f"🎯 推薦使用: {best_method['method']['name']}")
        print(f"   來源: {best_method['method']['source']}")
        
    else:
        print("❌ 沒有找到可用的相機連接方法")
        print("\n可能的問題：")
        print("1. 相機硬體未正確連接")
        print("2. 驅動程式問題")
        print("3. 權限問題")
        print("4. 其他程式佔用相機")
        print("\n建議檢查：")
        print("- ls -la /dev/video*")
        print("- dmesg | grep -i camera")
        print("- sudo fuser /dev/video*")

def main():
    try:
        test_camera_methods()
        
        print("\n按 Enter 繼續，或 Ctrl+C 退出...")
        input()
        
    except KeyboardInterrupt:
        print("\n測試中止")
    except Exception as e:
        print(f"程式錯誤: {e}")

if __name__ == '__main__':
    main()