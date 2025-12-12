#!/usr/bin/env python3
"""診斷 OpenCV 版本衝突"""
import sys
import os

print("="*70)
print("OpenCV 版本衝突診斷")
print("="*70)

# 1. 檢查 Python 搜尋路徑
print("\n【1. Python 搜尋路徑】")
for i, path in enumerate(sys.path, 1):
    print(f"{i}. {path}")

# 2. 嘗試 import OpenCV
print("\n【2. OpenCV Import 測試】")
try:
    import cv2
    print(f"✓ OpenCV 版本: {cv2.__version__}")
    print(f"✓ OpenCV 路徑: {cv2.__file__}")
    
    # 檢查 GStreamer
    build_info = cv2.getBuildInformation()
    has_gstreamer = False
    for line in build_info.split('\n'):
        if 'GStreamer:' in line:
            print(f"✓ {line.strip()}")
            if 'YES' in line:
                has_gstreamer = True
    
    if not has_gstreamer:
        print("❌ 當前 OpenCV 不支援 GStreamer")
    else:
        print("✓ 當前 OpenCV 支援 GStreamer")
        
except ImportError as e:
    print(f"❌ 無法 import OpenCV: {e}")

# 3. 尋找所有 cv2 模組
print("\n【3. 系統中所有 cv2 模組】")
import subprocess

# 使用 find 指令
result = subprocess.run(
    ['find', '/usr', '-name', 'cv2*.so', '-o', '-name', 'cv2.*.so'],
    capture_output=True,
    text=True,
    stderr=subprocess.DEVNULL
)

opencv_files = result.stdout.strip().split('\n')
opencv_files = [f for f in opencv_files if f]

if opencv_files:
    for i, path in enumerate(opencv_files, 1):
        print(f"{i}. {path}")
else:
    print("未找到系統 cv2 模組")

# 4. 檢查 pip 安裝的 OpenCV
print("\n【4. pip 安裝的 OpenCV】")
result = subprocess.run(
    ['pip3', 'list', '--format=freeze'],
    capture_output=True,
    text=True
)

opencv_packages = [line for line in result.stdout.split('\n') if 'opencv' in line.lower()]
if opencv_packages:
    for pkg in opencv_packages:
        print(f"  - {pkg}")
else:
    print("  無 pip 安裝的 OpenCV")

# 5. 檢查 apt 安裝的 OpenCV
print("\n【5. apt 安裝的 OpenCV】")
result = subprocess.run(
    ['dpkg', '-l'],
    capture_output=True,
    text=True
)

opencv_packages = [line for line in result.stdout.split('\n') if 'opencv' in line.lower() and line.startswith('ii')]
if opencv_packages:
    for line in opencv_packages[:5]:  # 只顯示前 5 個
        parts = line.split()
        if len(parts) >= 3:
            print(f"  - {parts[1]}: {parts[2]}")
    if len(opencv_packages) > 5:
        print(f"  ... 還有 {len(opencv_packages)-5} 個套件")
else:
    print("  無 apt 安裝的 OpenCV")

print("\n" + "="*70)