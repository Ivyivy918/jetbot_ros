#!/bin/bash
# 相機權限修復腳本

echo "========================================="
echo "  IMX219 相機權限修復工具"
echo "========================================="
echo ""

# 1. 檢查設備
echo "1. 檢查視訊設備..."
if ls /dev/video* 1> /dev/null 2>&1; then
    ls -l /dev/video*
else
    echo "✗ 未找到 /dev/video 設備"
    echo ""
    echo "請檢查："
    echo "  - 相機是否正確連接到 CSI 接口"
    echo "  - CSI 排線方向是否正確"
    echo "  - 是否需要重新啟動系統"
    exit 1
fi

echo ""

# 2. 修復權限
echo "2. 修復設備權限..."
for device in /dev/video*; do
    if [ -e "$device" ]; then
        sudo chmod 666 "$device"
        echo "✓ 已修復 $device 權限"
    fi
done

echo ""

# 3. 檢查用戶組
echo "3. 檢查用戶組..."
if groups | grep -q video; then
    echo "✓ 用戶已在 video 組"
else
    echo "⚠ 用戶不在 video 組，正在添加..."
    sudo usermod -a -G video $USER
    echo "✓ 已添加到 video 組（需要重新登錄生效）"
fi

echo ""

# 4. 檢查相機模組
echo "4. 檢查相機驅動模組..."
if lsmod | grep -q "imx219\|nv_imx219"; then
    echo "✓ IMX219 驅動模組已載入"
    lsmod | grep -E "imx219|nv_imx219"
else
    echo "⚠ IMX219 驅動模組未載入"
    echo "  嘗試載入 nv_imx219..."
    sudo modprobe nv_imx219 2>/dev/null
    if [ $? -eq 0 ]; then
        echo "✓ 已載入 nv_imx219 模組"
    else
        echo "✗ 無法載入驅動模組"
        echo "  可能需要重新啟動系統"
    fi
fi

echo ""

# 5. 檢查設備資訊
echo "5. 設備詳細資訊..."
if command -v v4l2-ctl &> /dev/null; then
    v4l2-ctl --list-devices
else
    echo "⚠ v4l2-ctl 未安裝"
    echo "安裝方式: sudo apt install v4l-utils"
fi

echo ""
echo "========================================="
echo "修復完成！"
echo "========================================="
echo ""
echo "下一步："
echo "  1. 運行測試: python3 scripts/simple_camera_test.py"
echo "  2. 如果仍有問題，請重新啟動系統"
echo ""
