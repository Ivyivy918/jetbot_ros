#!/usr/bin/env python3
"""
相機擷取測試程式
測試不同方式開啟相機並顯示畫面
"""
import cv2
import sys
import time

class CameraTest:
    def __init__(self):
        self.test_results = []
    
    def print_header(self, title):
        """印出測試標題"""
        print("\n" + "="*60)
        print(f"  {title}")
        print("="*60)
    
    def test_v4l2_direct(self, device_id):
        """測試方法 1: 直接使用 V4L2 裝置 ID"""
        self.print_header(f"測試 1: V4L2 直接開啟 (device_id={device_id})")
        
        try:
            cap = cv2.VideoCapture(device_id)
            
            if not cap.isOpened():
                print(f"❌ 無法開啟 /dev/video{device_id}")
                self.test_results.append(f"V4L2-ID-{device_id}: 失敗")
                return False
            
            # 設定解析度
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            
            # 取得實際設定值
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = int(cap.get(cv2.CAP_PROP_FPS))
            
            print(f"✓ 成功開啟相機")
            print(f"  解析度: {width}x{height}")
            print(f"  FPS: {fps}")
            
            # 嘗試讀取畫面
            print("  正在讀取畫面...")
            ret, frame = cap.read()
            
            if ret and frame is not None:
                print(f"✓ 成功讀取畫面 (shape: {frame.shape})")
                
                # 顯示畫面
                cv2.imshow(f'Camera {device_id} - V4L2 Direct', frame)
                print("  按任意鍵繼續...")
                cv2.waitKey(2000)  # 顯示 2 秒
                cv2.destroyAllWindows()
                
                cap.release()
                self.test_results.append(f"V4L2-ID-{device_id}: 成功")
                return True
            else:
                print(f"❌ 無法讀取畫面")
                cap.release()
                self.test_results.append(f"V4L2-ID-{device_id}: 開啟成功但無法讀取")
                return False
                
        except Exception as e:
            print(f"❌ 錯誤: {e}")
            self.test_results.append(f"V4L2-ID-{device_id}: 例外 - {e}")
            return False
    
    def test_v4l2_path(self, device_path):
        """測試方法 2: 使用裝置路徑"""
        self.print_header(f"測試 2: V4L2 裝置路徑 ({device_path})")
        
        try:
            cap = cv2.VideoCapture(device_path)
            
            if not cap.isOpened():
                print(f"❌ 無法開啟 {device_path}")
                self.test_results.append(f"V4L2-PATH-{device_path}: 失敗")
                return False
            
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = int(cap.get(cv2.CAP_PROP_FPS))
            
            print(f"✓ 成功開啟相機")
            print(f"  解析度: {width}x{height}")
            print(f"  FPS: {fps}")
            
            ret, frame = cap.read()
            
            if ret and frame is not None:
                print(f"✓ 成功讀取畫面 (shape: {frame.shape})")
                
                cv2.imshow(f'{device_path} - V4L2 Path', frame)
                print("  按任意鍵繼續...")
                cv2.waitKey(2000)
                cv2.destroyAllWindows()
                
                cap.release()
                self.test_results.append(f"V4L2-PATH-{device_path}: 成功")
                return True
            else:
                print(f"❌ 無法讀取畫面")
                cap.release()
                self.test_results.append(f"V4L2-PATH-{device_path}: 開啟成功但無法讀取")
                return False
                
        except Exception as e:
            print(f"❌ 錯誤: {e}")
            self.test_results.append(f"V4L2-PATH-{device_path}: 例外 - {e}")
            return False
    
    def test_gstreamer_v4l2(self, device_path, label):
        """測試方法 3: GStreamer + V4L2"""
        self.print_header(f"測試 3: GStreamer V4L2 ({device_path})")
        
        try:
            gst_pipeline = (
                f"v4l2src device={device_path} ! "
                "video/x-raw, width=640, height=480, framerate=30/1 ! "
                "videoconvert ! "
                "appsink drop=1 max-buffers=2"
            )
            
            print(f"Pipeline: {gst_pipeline}")
            
            cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            
            if not cap.isOpened():
                print(f"❌ 無法開啟 GStreamer pipeline")
                self.test_results.append(f"GST-V4L2-{label}: 失敗")
                return False
            
            print(f"✓ 成功開啟 GStreamer pipeline")
            
            ret, frame = cap.read()
            
            if ret and frame is not None:
                print(f"✓ 成功讀取畫面 (shape: {frame.shape})")
                
                cv2.imshow(f'{label} - GStreamer V4L2', frame)
                print("  按任意鍵繼續...")
                cv2.waitKey(2000)
                cv2.destroyAllWindows()
                
                cap.release()
                self.test_results.append(f"GST-V4L2-{label}: 成功")
                return True
            else:
                print(f"❌ 無法讀取畫面")
                cap.release()
                self.test_results.append(f"GST-V4L2-{label}: 開啟成功但無法讀取")
                return False
                
        except Exception as e:
            print(f"❌ 錯誤: {e}")
            self.test_results.append(f"GST-V4L2-{label}: 例外 - {e}")
            return False
    
    def test_gstreamer_nvargus(self, sensor_id, label):
        """測試方法 4: GStreamer + NVARGUS (Jetson 專用)"""
        self.print_header(f"測試 4: GStreamer NVARGUS (sensor_id={sensor_id})")
        
        try:
            gst_pipeline = (
                f"nvarguscamerasrc sensor_id={sensor_id} ! "
                "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
                "nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! "
                "appsink drop=1 max-buffers=2"
            )
            
            print(f"Pipeline: {gst_pipeline}")
            
            cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
            
            if not cap.isOpened():
                print(f"❌ 無法開啟 NVARGUS pipeline")
                self.test_results.append(f"GST-NVARGUS-{label}: 失敗")
                return False
            
            print(f"✓ 成功開啟 NVARGUS pipeline")
            
            ret, frame = cap.read()
            
            if ret and frame is not None:
                print(f"✓ 成功讀取畫面 (shape: {frame.shape})")
                
                cv2.imshow(f'{label} - GStreamer NVARGUS', frame)
                print("  按任意鍵繼續...")
                cv2.waitKey(2000)
                cv2.destroyAllWindows()
                
                cap.release()
                self.test_results.append(f"GST-NVARGUS-{label}: 成功")
                return True
            else:
                print(f"❌ 無法讀取畫面")
                cap.release()
                self.test_results.append(f"GST-NVARGUS-{label}: 開啟成功但無法讀取")
                return False
                
        except Exception as e:
            print(f"❌ 錯誤: {e}")
            self.test_results.append(f"GST-NVARGUS-{label}: 例外 - {e}")
            return False
    
    def test_continuous_capture(self, device_id, duration=5):
        """測試連續擷取畫面"""
        self.print_header(f"測試 5: 連續擷取 {duration} 秒 (device_id={device_id})")
        
        try:
            cap = cv2.VideoCapture(device_id)
            
            if not cap.isOpened():
                print(f"❌ 無法開啟相機")
                return False
            
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            print(f"✓ 開始連續擷取 {duration} 秒...")
            print("  按 'q' 提前結束")
            
            start_time = time.time()
            frame_count = 0
            
            while time.time() - start_time < duration:
                ret, frame = cap.read()
                
                if not ret:
                    print(f"❌ 第 {frame_count} 幀讀取失敗")
                    break
                
                frame_count += 1
                
                # 在畫面上顯示幀數和 FPS
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                
                cv2.putText(frame, f"Frame: {frame_count}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow(f'Continuous Capture - Device {device_id}', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("  使用者中斷")
                    break
            
            elapsed = time.time() - start_time
            avg_fps = frame_count / elapsed if elapsed > 0 else 0
            
            print(f"✓ 擷取完成")
            print(f"  總幀數: {frame_count}")
            print(f"  總時間: {elapsed:.2f} 秒")
            print(f"  平均 FPS: {avg_fps:.2f}")
            
            cv2.destroyAllWindows()
            cap.release()
            
            self.test_results.append(f"連續擷取-{device_id}: 成功 ({frame_count} 幀)")
            return True
            
        except Exception as e:
            print(f"❌ 錯誤: {e}")
            self.test_results.append(f"連續擷取-{device_id}: 例外 - {e}")
            return False
    
    def print_summary(self):
        """印出測試總結"""
        self.print_header("測試總結")
        
        for i, result in enumerate(self.test_results, 1):
            print(f"{i}. {result}")
        
        success_count = sum(1 for r in self.test_results if "成功" in r)
        total_count = len(self.test_results)
        
        print(f"\n成功: {success_count}/{total_count}")
        print("="*60 + "\n")


def main():
    print("""
╔════════════════════════════════════════════════════════════╗
║          相機擷取測試程式                                    ║
║          測試不同方式開啟相機並顯示畫面                      ║
╚════════════════════════════════════════════════════════════╝
    """)
    
    tester = CameraTest()
    
    # 測試 /dev/video0
    print("\n【測試左相機 - /dev/video0】")
    tester.test_v4l2_direct(0)
    time.sleep(0.5)
    
    tester.test_v4l2_path('/dev/video0')
    time.sleep(0.5)
    
    tester.test_gstreamer_v4l2('/dev/video0', 'Left')
    time.sleep(0.5)
    
    # 測試 /dev/video1
    print("\n【測試右相機 - /dev/video1】")
    tester.test_v4l2_direct(1)
    time.sleep(0.5)
    
    tester.test_v4l2_path('/dev/video1')
    time.sleep(0.5)
    
    tester.test_gstreamer_v4l2('/dev/video1', 'Right')
    time.sleep(0.5)
    
    # 測試 NVARGUS (如果是 Jetson)
    print("\n【測試 NVARGUS (Jetson 專用)】")
    tester.test_gstreamer_nvargus(0, 'Right-CSI0')
    time.sleep(0.5)
    
    tester.test_gstreamer_nvargus(1, 'Left-CSI1')
    time.sleep(0.5)
    
    # 連續擷取測試
    print("\n【測試連續擷取】")
    response = input("是否進行連續擷取測試？(y/n): ")
    if response.lower() == 'y':
        device = input("請輸入裝置 ID (0 或 1): ")
        try:
            device_id = int(device)
            tester.test_continuous_capture(device_id, duration=5)
        except ValueError:
            print("無效的裝置 ID")
    
    # 印出總結
    tester.print_summary()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程式被使用者中斷")
        cv2.destroyAllWindows()
        sys.exit(0)