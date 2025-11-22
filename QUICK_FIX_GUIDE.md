# IMX219 相機快速修復指南

## ⚠️ 您遇到的錯誤

```
GStreamer warning: Internal data stream error
Failed to open camera with sensor_id=1
```

這個錯誤表示相機硬體無法正確訪問。讓我們逐步解決。

---

## 🔧 立即執行（按順序）

### 步驟 1: 修復權限並檢查硬體

```bash
cd /home/user/jetbot_ros

# 運行修復腳本
bash scripts/fix_camera_permissions.sh
```

這會自動：
- ✅ 檢查 /dev/video 設備是否存在
- ✅ 修復設備權限
- ✅ 檢查並載入相機驅動模組
- ✅ 顯示設備詳細資訊

### 步驟 2: 運行簡單測試

```bash
# 運行簡化的相機測試
python3 scripts/simple_camera_test.py
```

這會測試所有可能的相機打開方式，告訴您哪個能用。

### 步驟 3: 手動檢查硬體

```bash
# 1. 檢查設備是否存在
ls -l /dev/video*

# 應該看到：
# /dev/video0
# /dev/video1

# 2. 查看設備資訊（如果有 v4l2-ctl）
v4l2-ctl --list-devices

# 3. 檢查系統日誌
dmesg | grep -i imx219
dmesg | grep -i csi

# 4. 查看載入的模組
lsmod | grep imx219
```

---

## 🎯 根據測試結果修復

### 情況 A: /dev/video 設備不存在

**問題：** 相機硬體未被識別

**解決方案：**

1. **檢查物理連接**
   - 關閉 Jetson 電源
   - 檢查 CSI 排線是否插緊
   - 確認排線方向正確（藍色朝向電路板）
   - 重新連接後開機

2. **載入驅動模組**
   ```bash
   # 載入 IMX219 驅動
   sudo modprobe imx219

   # 確認是否載入
   lsmod | grep imx219

   # 檢查設備
   ls -l /dev/video*
   ```

3. **檢查設備樹配置** (Jetson Orin Nano)
   ```bash
   # 確認 CSI 接口已啟用
   cat /proc/device-tree/model

   # 查看啟動日誌
   dmesg | grep -i camera
   ```

4. **如果仍然不行，重新啟動**
   ```bash
   sudo reboot
   ```

### 情況 B: 設備存在但無法打開

**問題：** 權限或驅動問題

**解決方案：**

1. **確認權限**
   ```bash
   # 查看權限
   ls -l /dev/video*

   # 應該是：
   # crw-rw-rw- 1 root video ... /dev/video0

   # 如果不是，修復權限
   sudo chmod 666 /dev/video0
   sudo chmod 666 /dev/video1
   ```

2. **檢查是否被佔用**
   ```bash
   # 查看哪個程序在使用
   sudo fuser /dev/video0
   sudo fuser /dev/video1

   # 如果有輸出，停止該程序
   # 例如：kill -9 <PID>
   ```

3. **安裝必要工具**
   ```bash
   # 安裝 v4l-utils
   sudo apt update
   sudo apt install v4l-utils

   # 查看支援的格式
   v4l2-ctl --device=/dev/video0 --list-formats-ext
   v4l2-ctl --device=/dev/video1 --list-formats-ext
   ```

### 情況 C: 測試工具顯示特定管道可用

**如果簡單測試顯示某個管道可用，更新 launch 文件：**

#### C1. 如果 NVIDIA Argus 可用（最佳）

編輯 `launch/jetbot.launch.py`，不需要修改，這已經是首選方案。

#### C2. 如果只有 V4L2 簡單模式可用

修改 `jetbot_ros/csi_camera_node.py`，在 `try_argus_camera` 後面添加優先級：

```python
# 在第 67 行後調整順序
# 方法1: 嘗試 V4L2 簡單模式 (提高優先級)
if self.try_v4l2_simple_camera(sensor_id, width, height, fps):
    self.pipeline_type = "v4l2_simple"
    self.get_logger().info("✓ Successfully using V4L2 simple pipeline")

# 方法2: 嘗試 NVIDIA Argus
elif self.try_argus_camera(sensor_id, width, height, fps, flip_method):
    self.pipeline_type = "argus"
    self.get_logger().info("✓ Successfully using NVIDIA Argus camera pipeline")
```

#### C3. 如果只有 OpenCV 可用

在 `launch/jetbot.launch.py` 中添加參數：

```python
parameters=[{
    'sensor_id': 1,
    'width': 640,      # 降低解析度
    'height': 480,     # 降低解析度
    'fps': 30,
    # ... 其他參數
}]
```

---

## 🚀 特定場景解決方案

### 場景 1: JetPack 6.x 上的 IMX219

```bash
# 1. 確認 JetPack 版本
dpkg -l | grep nvidia-jetpack

# 2. 檢查 nvarguscamerasrc 是否可用
gst-inspect-1.0 nvarguscamerasrc

# 3. 如果不可用，安裝
sudo apt update
sudo apt install nvidia-l4t-gstreamer
```

### 場景 2: 雙相機配置

```bash
# 確認兩個相機都被識別
ls -l /dev/video0 /dev/video1

# 測試每個相機
# 左相機 (sensor_id=1)
gst-launch-1.0 nvarguscamerasrc sensor-id=1 ! nvoverlaysink

# 右相機 (sensor_id=0)
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvoverlaysink

# 按 Ctrl+C 停止
```

### 場景 3: 在 Docker 中運行

```bash
# 確保 Docker 有設備訪問權限
docker run --runtime nvidia --device /dev/video0:/dev/video0 \
           --device /dev/video1:/dev/video1 \
           -it <your-image>

# 在 Dockerfile 中添加權限
RUN usermod -a -G video root
```

---

## 📋 完整診斷檢查清單

在聯繫支援之前，請完成以下檢查：

- [ ] `/dev/video0` 和 `/dev/video1` 存在
- [ ] 設備權限是 `crw-rw-rw-`
- [ ] `lsmod | grep imx219` 顯示驅動已載入
- [ ] `v4l2-ctl --list-devices` 顯示相機設備
- [ ] 運行了 `scripts/simple_camera_test.py`
- [ ] 運行了 `scripts/fix_camera_permissions.sh`
- [ ] CSI 排線已重新插拔並確認方向
- [ ] 系統已重新啟動

---

## 🔍 查看具體錯誤日誌

如果上述方法都不行，收集以下資訊：

```bash
# 1. 系統資訊
cat /proc/device-tree/model
uname -a

# 2. JetPack 版本
dpkg -l | grep nvidia-jetpack

# 3. GStreamer 版本和插件
gst-inspect-1.0 --version
gst-inspect-1.0 nvarguscamerasrc

# 4. 設備詳細資訊
v4l2-ctl --device=/dev/video0 --all > /tmp/video0_info.txt
v4l2-ctl --device=/dev/video1 --all > /tmp/video1_info.txt

# 5. 核心日誌
dmesg | grep -i camera > /tmp/camera_dmesg.log
dmesg | grep -i csi >> /tmp/camera_dmesg.log
dmesg | grep -i imx219 >> /tmp/camera_dmesg.log

# 6. 測試結果
python3 scripts/simple_camera_test.py > /tmp/camera_test.log 2>&1
```

將這些檔案的內容提供給支援團隊。

---

## ✅ 成功標準

修復成功後，您應該能看到：

```bash
# 運行測試
python3 scripts/simple_camera_test.py

# 輸出應該包含：
✓ /dev/video0 存在
✓ /dev/video1 存在
✓ 設備 0 已開啟
✓ 成功讀取畫面: (480, 640, 3)
✓ 設備 1 已開啟
✓ 成功讀取畫面: (480, 640, 3)
```

然後可以啟動 ROS 節點：

```bash
ros2 launch jetbot_ros jetbot.launch.py
```

RViz 中應該能看到正常的相機畫面（不是綠屏）。

---

## 🆘 仍然無法解決？

請提供以下資訊：

1. **硬體配置**
   - Jetson 型號
   - 相機型號（IMX219）
   - CSI 接口位置（CAM0/CAM1）

2. **軟體版本**
   - JetPack 版本
   - ROS 2 版本
   - OpenCV 版本

3. **測試結果**
   - `simple_camera_test.py` 的完整輸出
   - `fix_camera_permissions.sh` 的完整輸出
   - 任何錯誤訊息的截圖

4. **已嘗試的步驟**
   - 列出您已經嘗試的所有解決方案
   - 每個步驟的結果

祝您順利解決問題！🚀
