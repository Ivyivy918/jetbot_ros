#!/usr/bin/env python3
import cv2

# GStreamer pipeline 打開 CSI 相機
gst_pipeline = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
    "nvvidconv ! videoconvert ! appsink"
)

# 打開相機
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Cannot open camera. Make sure you have HDMI/display connected.")
    exit(1)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Cannot read frame")
        break

    # 顯示畫面
    cv2.imshow("Jetson Camera", frame)

    # 按 q 離開
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
