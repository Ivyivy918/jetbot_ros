# IMX219 相機綠屏問題修復說明

## 修復內容摘要

本次修復解決了 JetBot ROS2 系統中 IMX219 相機在 RViz 中顯示綠屏的問題，並完善了雙眼立體相機的支持。

### 主要修改

1. **修復 csi_camera_node.py 中的程式錯誤**
   - 修正第 171 行的縮進錯誤
   - 改進錯誤處理和日誌記錄

2. **增強 GStreamer 管道支持**
   - 添加 NVIDIA Argus 管道支持（推薦用於 Jetson）
   - 支持多種 Bayer 格式自動偵測（rggb, grbg, bggr, gbrg）
   - 添加 V4L2 簡單模式作為備用
   - 保留 OpenCV 回退選項

3. **新增相機診斷工具**
   - `scripts/diagnose_camera.py` - 完整的相機診斷工具
   - 自動測試所有可用的 GStreamer 管道
   - 檢測綠屏問題並提供建議

---

## 修復的問題

### 問題 1: RViz 中相機畫面顯示綠屏

**原因：**
- GStreamer 管道配置不正確
- Bayer 格式選擇錯誤
- 缺少對 NVIDIA Argus 的支持

**解決方案：**
相機節點現在會自動嘗試以下管道（按優先順序）：

1. **NVIDIA Argus** (最佳) - 使用 nvarguscamerasrc
   ```
   nvarguscamerasrc sensor-id=X !
   video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 !
   nvvidconv ! video/x-raw, width=640, height=480, format=BGRx !
   videoconvert ! video/x-raw, format=BGR ! appsink
   ```

2. **V4L2 with Bayer** - 自動測試 4 種 Bayer 格式
   ```
   v4l2src device=/dev/videoX !
   video/x-bayer, width=1280, height=720, format=rggb !
   bayer2rgb ! videoscale ! video/x-raw, width=640, height=480 !
   videoconvert ! video/x-raw, format=BGR ! appsink
   ```

3. **V4L2 Simple** - 不使用 Bayer 轉換
4. **OpenCV** - 最基本的回退選項

### 問題 2: 雙眼立體相機和點雲生成

**現狀：**
系統已經配置完整的立體視覺管道：

```
左相機 (sensor_id=1) → /stereo/left/image_raw
右相機 (sensor_id=0) → /stereo/right/image_raw
    ↓
圖像矯正 (image_proc/rectify_node)
    ↓
視差計算 (stereo_image_proc/disparity_node)
    ↓
點雲生成 (stereo_image_proc/point_cloud_node) → /stereo/points2
    ↓
RTAB-Map SLAM → /rtabmap/cloud_map
```

**立體視覺配置：**
- 基線距離: 0.16 m (相機間距)
- 輸出解析度: 640×480
- 幀率: 30 fps
- 已配置立體校正參數: `config/camera_calibration/stereo/`

---

## 使用說明

### 1. 診斷相機問題

在運行主程式之前，建議先使用診斷工具檢查相機：

```bash
cd /home/user/jetbot_ros
python3 scripts/diagnose_camera.py
```

診斷工具會：
- 檢查 `/dev/video` 設備
- 測試 GStreamer 是否安裝
- 測試所有可用的相機管道
- 檢測是否存在綠屏問題
- 提供詳細的錯誤信息和建議

### 2. 重新構建 ROS 套件

在 Jetson 設備上，進入 Docker 容器並重新構建：

```bash
# 進入 Docker 容器
cd /path/to/jetbot_ros
docker/build.sh  # 或使用您的容器啟動腳本

# 在容器內重新構建
source /opt/ros/${ROS_DISTRO}/install/setup.bash
cd /workspace
colcon build --symlink-install --packages-select jetbot_ros
source install/local_setup.bash
```

### 3. 啟動完整系統

使用主 launch 文件啟動所有節點（相機、立體視覺、點雲、SLAM、RViz）：

```bash
ros2 launch jetbot_ros jetbot.launch.py
```

### 4. 僅測試立體相機和點雲

如果只想測試相機和點雲功能（不含 SLAM 和其他功能）：

```bash
ros2 launch jetbot_ros stereo_camera.launch.py
```

### 5. 在 RViz 中查看

RViz 會自動啟動，您應該能看到：

1. **Left Camera** - 左相機畫面 (`/camera_left/image_raw`)
2. **Right Camera** - 右相機畫面 (`/camera_right/image_raw`)
3. **PointCloud Map** - 點雲地圖 (`/rtabmap/cloud_map`)
4. **Map** - 柵格地圖 (`/rtabmap/grid_map`)
5. **TF** - 坐標變換
6. **Robot Model** - 機器人模型

---

## 常見問題排除

### Q1: 仍然看到綠屏怎麼辦？

**步驟 1：** 運行診斷工具
```bash
python3 scripts/diagnose_camera.py
```

**步驟 2：** 檢查診斷結果
- 如果 "NVIDIA Argus" 測試成功 → 應該沒有綠屏
- 如果只有某個 Bayer 格式成功 → 相機節點會自動選擇
- 如果所有測試都失敗 → 檢查硬體連接

**步驟 3：** 手動指定 Bayer 格式（如果需要）

如果診斷工具發現特定的 Bayer 格式有效，您可以修改 `jetbot_ros/csi_camera_node.py` 中的 `try_v4l2_bayer_camera()` 函數，只測試該格式：

```python
# 將這行：
bayer_formats = ['rggb', 'grbg', 'bggr', 'gbrg']

# 改為（例如只使用 grbg）：
bayer_formats = ['grbg']
```

### Q2: 相機無法開啟

**檢查清單：**

1. 確認相機已連接到正確的 CSI 接口
   ```bash
   ls -l /dev/video*
   ```
   應該看到 `/dev/video0` 和 `/dev/video1`

2. 檢查相機權限
   ```bash
   sudo chmod 666 /dev/video0
   sudo chmod 666 /dev/video1
   ```

3. 檢查相機是否被其他程序佔用
   ```bash
   sudo fuser /dev/video0
   sudo fuser /dev/video1
   ```

4. 重新啟動 Jetson 設備

### Q3: 點雲沒有顯示

**可能原因：**

1. **相機校正檔案缺失或不正確**
   - 檢查 `config/camera_calibration/stereo/` 目錄
   - 確保 `left.yaml` 和 `right.yaml` 存在

2. **相機未同步**
   - 檢查時間戳是否一致
   - 確認 `approximate_sync: True` 已設置

3. **視差計算失敗**
   - 檢查左右相機畫面是否正常
   - 確認場景有足夠的紋理特徵

4. **TF 變換問題**
   - 檢查 `robot_state_publisher` 是否運行
   - 驗證 URDF 模型正確載入

### Q4: RViz 中看到的點雲很稀疏

這是正常的，可以通過以下方式改善：

1. **增加場景紋理**
   - 立體視覺需要豐富的紋理特徵
   - 避免純白牆面或光滑表面

2. **調整視差參數**
   - 在 `launch/jetbot.launch.py` 中調整 `disparity_node` 參數

3. **改善光照**
   - 確保場景有良好的照明
   - 避免過曝或過暗

### Q5: RTAB-Map SLAM 無法運行

**檢查：**

1. 確認點雲數據正在發佈
   ```bash
   ros2 topic echo /stereo/points2 --once
   ```

2. 檢查 RTAB-Map 日誌
   ```bash
   ros2 node info /rtabmap
   ```

3. 重置 RTAB-Map 數據庫
   - 啟動時會自動使用 `--delete_db_on_start`

---

## 技術細節

### 相機節點改進

**新增功能：**

1. **自動管道選擇**
   - 智能嘗試多種管道配置
   - 每個管道都進行實際測試讀取
   - 自動選擇第一個成功的管道

2. **詳細日誌**
   - 顯示嘗試的每個管道
   - 報告成功/失敗狀態
   - 幀計數和統計信息

3. **錯誤恢復**
   - 如果一個管道失敗，自動嘗試下一個
   - 提供清晰的錯誤信息
   - 建議用戶檢查項目

### GStreamer 管道優化

**NVIDIA Argus 優勢：**
- 使用 ISP (Image Signal Processor)
- 硬體加速處理
- 更好的色彩還原
- 更低的 CPU 使用率

**V4L2 Bayer 靈活性：**
- 支持標準 V4L2 驅動
- 自動檢測正確的 Bayer 格式
- 適用於不同的相機模組

---

## 文件結構

```
jetbot_ros/
├── jetbot_ros/
│   └── csi_camera_node.py          # ✨ 已修復和增強
├── launch/
│   ├── jetbot.launch.py            # 完整系統啟動
│   └── stereo_camera.launch.py    # 僅相機和點雲
├── scripts/
│   └── diagnose_camera.py          # ✨ 新增診斷工具
├── config/
│   └── camera_calibration/
│       ├── stereo/                 # 立體校正參數
│       │   ├── left.yaml
│       │   └── right.yaml
│       ├── left/                   # 左相機單獨校正
│       └── right/                  # 右相機單獨校正
├── rviz/
│   └── jetbot_mapping.rviz         # RViz 配置
└── urdf/
    └── jetbot.urdf.xacro           # 機器人模型

```

---

## 相關資源

### 官方文檔

- [ROS 2 Foxy 文檔](https://docs.ros.org/en/foxy/)
- [stereo_image_proc](http://wiki.ros.org/stereo_image_proc)
- [RTAB-Map](http://wiki.ros.org/rtabmap_ros)
- [image_proc](http://wiki.ros.org/image_proc)

### GStreamer 參考

- [GStreamer 文檔](https://gstreamer.freedesktop.org/documentation/)
- [NVIDIA Jetson GStreamer 指南](https://developer.nvidia.com/embedded/learn/tutorials/first-picture-csi-usb-camera)

### 相機校正

如果需要重新校正相機：

```bash
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.024 \
    --no-service-check \
    image:=/stereo/left/image_raw \
    camera:=/stereo/left
```

---

## 變更歷史

### 2024-11-16 - 綠屏問題修復

- ✅ 修復 csi_camera_node.py 縮進錯誤
- ✅ 添加 NVIDIA Argus 管道支持
- ✅ 添加多 Bayer 格式自動檢測
- ✅ 改進錯誤處理和日誌
- ✅ 創建相機診斷工具
- ✅ 驗證立體視覺和點雲管道

---

## 支持

如有問題，請檢查：

1. **日誌輸出** - 相機節點會提供詳細的診斷信息
2. **診斷工具** - 使用 `scripts/diagnose_camera.py` 全面檢查
3. **硬體連接** - 確認 CSI 排線正確連接
4. **權限設置** - 確保有訪問 `/dev/video` 的權限

祝您使用順利！🚀
