# YKY Project (LLM + YOLO + Techman Robot)

這個專案結合了大型語言模型 (LLM)、YOLOv8 視覺辨識以及 Intel RealSense，來控制 Techman Robot (達明機器人) 與周邊硬體 (Arduino 夾爪、Wacoh Dynpick 力矩感測器)。

🔗 **GitHub 專案位址：** [https://github.com/hsiehsiu/yky_project](https://github.com/hsiehsiu/yky_project)

---

## 💻 專案下載與環境建置 (使用 Docker)

要在**全新的一台電腦**上完整複製與執行此專案，請按照以下步驟進行。

### 1. 下載專案與達明機器人驅動程式 (tmdriver)

由於專案底下的達明機器人驅動程式 (`tmr_ros2` / `tmdriver`) 檔案較龐大且官方有提供標準版本，因此未全部放入此 GitHub 儲存庫中。在新電腦上，您需要先 Clone 本專案，再補上官方的 TM Driver：

```bash
# 1. 下載本專案
git clone https://github.com/hsiehsiu/yky_project.git
cd yky_project

# 2. 在 src 目錄下，額外下載官方的 tmr_ros2 驅動程式 (這裡以 humble 分支為例)
cd src
git clone https://github.com/TechmanRobotInc/tmr_ros2.git -b humble
cd ..
```
*(備註：若您原本手邊有特定修改過的 `tmdriver` 版本，請直接將該資料夾複製進 `src/` 目錄下)*

### 2. 打包 Docker Image

回到專案根目錄 (`yky_project`)，執行以下指令將環境和程式碼打包成 Docker Image：
```bash
docker build -t tmr_ros2_env .
```
*(這一步會自動安裝 Ubuntu 22.04、ROS 2 Humble，並透過 `rosdep install` 與 `colcon build` 自動將 `realsense`、`tmdriver` 以及您的所有套件編譯安裝完畢。)*

### 3. 執行 Docker 容器

在別台電腦執行時，因為程式碼已經包在容器內的 `/ros2_ws`，您可以直接啟動：

**A. 單純執行模式 (Production Mode)**
```bash
docker run -it --rm \
    --net host \
    --privileged \
    -v /dev:/dev \
    tmr_ros2_env
```

**B. 開發除錯模式 (Development Mode)**
如果您需要修改程式碼並即時測試，可以把剛下載的 `src` 目錄掛載進去取代容器內的程式碼：
```bash
docker run -it --rm \
    --net host \
    --privileged \
    -v /dev:/dev \
    -v $(pwd)/src:/ros2_ws/src \
    tmr_ros2_env
```
*(進入容器後若有修改程式碼，請手動執行 `colcon build --symlink-install`)*

---

## 🔌 硬體設備連線與設定教學

為確保程式能正常連線硬體，啟動前請先確認以下設備設定：

### 1. 達明機器人 (Techman Robot)
- **網路連線**：將電腦的網路線接到機器人的 LAN 埠，並將本機 IP 與機器人設為同網段 (例如機器人是 `192.168.10.2`，您的電腦可設為 `192.168.10.10`)。
- **開啟 Listen Node**：務必在 TM Flow 的專案中放入並執行「**Listen Node**」，讓手臂進入等待外部連線接收指令的狀態。
- **啟動參數**：若您的機器人 IP 不是預設的 `192.168.10.2`，請在 Launch 啟動時加入參數：`robot_ip:=您的IP`。

### 2. Intel RealSense 攝影機
- **USB 頻寬**：必須使用 **USB 3.0** (藍色孔) 以上的連接埠與線材。若插在 USB 2.0 上，可能無法傳送 Depth (深度) 影像資料導致 YOLO 3D 節點報錯。
- **權限設定**：Docker 啟動時已經加上 `--privileged` 與 `-v /dev:/dev`，只要主機端有正確抓到設備，容器內就能直接存取。

### 3. Arduino (夾爪/末端工具)
- **硬體連接**：直接透過 USB 連接到電腦，一般會在 `/dev/ttyACM0` 或 `/dev/ttyUSB0`。
- **鮑率(Baudrate)**：請確保 Arduino 燒錄的程式鮑率與 ROS 2 Node 內的設定一致。

### 4. Wacoh Dynpick 力矩感測器
- **硬體連接**：透過專用的 USB/RS422 轉換線連接到電腦，裝置名稱預設通常為 `/dev/ttyUSB*`。
- 若啟動時發生「Permission Denied」或讀不到裝置，通常是因為 Linux 權限問題，此問題已透過 Docker `--privileged` 解決。

---

## 🚀 系統啟動指令 (How to Run)

當您進入 Docker 容器並確認硬體都接妥後，執行以下指令即可一次啟動所有節點：

```bash
ros2 launch bringup main.launch.py robot_ip:=192.168.10.2
```

預設的 `main.launch.py` 會啟動以下節點：
- Intel RealSense 攝影機
- YOLOv8 視覺辨識與 3D 座標換算
- Arduino 夾爪控制

*(若需要啟動達明機器人控制或力矩感測器，請進入 `src/bringup/launch/main.launch.py` 將被註解起來的 `tm_driver` 與 `transfer` 區塊取消註解！)*
