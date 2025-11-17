#!/bin/bash
# 一鍵修復當前的相機權限問題

echo "========================================"
echo "  IMX219 相機權限一鍵修復"
echo "========================================"
echo ""

# 1. 檢查並加入 video 組
echo "1. 檢查用戶組..."
if groups | grep -q video; then
    echo "✓ 用戶已在 video 組"
else
    echo "⚠ 正在將用戶加入 video 組..."
    sudo usermod -a -G video $USER
    echo "✓ 已加入 video 組"
    echo "  請執行: newgrp video"
    echo "  或重新登錄以使更改生效"
fi

echo ""

# 2. 修復設備權限
echo "2. 修復設備權限..."
for device in /dev/video0 /dev/video1; do
    if [ -e "$device" ]; then
        sudo chmod 666 "$device"
        echo "✓ 已修復 $device (666)"
    fi
done

echo ""

# 3. 驗證權限
echo "3. 驗證權限..."
ls -l /dev/video*

echo ""

# 4. 創建永久 udev 規則
echo "4. 創建永久 udev 規則..."
if [ ! -f /etc/udev/rules.d/99-camera.rules ]; then
    sudo tee /etc/udev/rules.d/99-camera.rules > /dev/null << 'EOF'
# IMX219 相機權限規則
KERNEL=="video[0-9]*", SUBSYSTEM=="video4linux", MODE="0666", GROUP="video"
EOF
    echo "✓ 已創建 udev 規則"

    # 重新載入
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "✓ 已重新載入 udev 規則"
else
    echo "ℹ udev 規則已存在"
fi

echo ""

# 5. 驗證驅動
echo "5. 驗證驅動載入..."
if lsmod | grep -q "nv_imx219"; then
    echo "✓ nv_imx219 驅動已載入"
    lsmod | grep nv_imx219
else
    echo "✗ nv_imx219 驅動未載入"
fi

echo ""
echo "========================================"
echo "修復完成！"
echo "========================================"
echo ""
echo "下一步："
echo "  1. 如果剛加入 video 組，執行: newgrp video"
echo "  2. 運行測試: python3 scripts/simple_camera_test.py"
echo "  3. 如果測試通過，啟動: ros2 launch jetbot_ros jetbot.launch.py"
echo ""
