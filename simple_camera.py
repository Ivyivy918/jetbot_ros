#!/usr/bin/env python3
import cv2

# 開啟 /dev/video0
cap = cv2.VideoCapture(1, cv2.CAP_V4L2)

if not cap.isOpened():
    print("❌ Cannot open /dev/video0")
    exit()

# 設定解析度
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)

print("✅ Camera /dev/video0 opened!")
print(f"Resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
print("\nControls:")
print("  'q' - Quit")
print("  's' - Save image")

while True:
    ret, frame = cap.read()
    
    if ret:
        # 顯示畫面
        cv2.imshow('Camera 0 (/dev/video0)', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('captured_image.jpg', frame)
            print("✅ Image saved!")
    else:
        print("❌ Failed to read frame")
        break

cap.release()
cv2.destroyAllWindows()



