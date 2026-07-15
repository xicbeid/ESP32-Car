# ESP32-P4 搜救小车 — 异构双芯边缘AI视觉智控平台

基于 ESP32-P4 Function EV Board v1.6 与 ESP32-C6 协处理器的搜救移动平台，具备 UART 编码器闭环运动控制、CSI 摄像头灰度 MJPEG 实时推流、10轴 IMU 惯导姿态感知、片上 ESP-DL 人脸检测，以及 Web + Android APK 双终端遥控。

```
ESP32-P4 WiFi AP "ESP32-Car" (192.168.4.1)
  ├── UART0 GPIO20/21 → 电机驱动板 ($spd:... 文本协议, 115200-8N1)
  ├── SDIO GPIO14-19, 54 → C6 (ESP-Hosted-NG WiFi)
  ├── SC2336 MIPI-CSI 2-Lane RAW10 (SCCB I2C0 GPIO7/8)
  │     └── CSI Ctrl → ISP → RGB565 → HW JPEG → 灰度 MJPEG
  │     └── 软件 AE: 绿通道采样 → EMA(α=0.85) → V4L2 CID 曝光/增益 + 50Hz 抗频闪
  ├── USB 2.0 Type-A → CP2102 → Wit-Motion 10轴IMU
  │     └── USB CDC Host → Wit-Motion 协议解析 (0x55 帧头) → EMA 滤波(α=0.15) → Web 倾角面板
  ├── ESP-DL MSRMNP → 人脸检测 + 5 关键点 (异步 3Hz, pre-JPEG 画框)
  ├── HTTP :80 → 网页遥控 (D-Pad 速度 + GO 位置 + MJPEG + IMU 面板 + 人脸检测开关)
  │     HTTP :81 → MJPEG 推流 (独立 TCP task, 不阻塞 httpd)
  └── uni-app (Vue 3) Android APK → 5 Tab 专业遥控终端
```

## 硬件接线

| P4 引脚 | 方向 | 连接目标 | 接口协议 | 功能说明 |
|---------|:----:|----------|----------|----------|
| GPIO20 (U0TXD) | Out | 电机驱动板 RX | UART 115200-8N1 | 发送运动控制指令 ($spd/$mtype 文本帧) |
| GPIO21 (U0RXD) | In | 电机驱动板 TX | UART 115200-8N1 | 接收四路编码器实时脉冲数据 |
| GPIO7 (SDA) | Bi-dir | SC2336 摄像头 | I2C (SCCB) | 摄像头传感器寄存器配置 |
| GPIO8 (SCL) | Out | SC2336 摄像头 | I2C (SCCB) | SCCB 串行同步时钟 |
| GPIO14-19, 54 | Bi-dir | ESP32-C6 | SDIO 4-bit | 核心高带宽网络卸载数据总线 (40MHz) |
| MIPI D0±/D1±/CLK± | In | SC2336 传感器 | MIPI-CSI 2-Lane | RAW10 差分信号接收 |
| USB_OTG_DP,DM | Bi-dir | Wit-Motion IMU (CP2102) | USB 2.0 FS CDC | 虚拟串口 CDC 驱动 (0x10C4:0xEA60) |
| GND | — | 电机驱动板 GND | — | 共地 |

> **注意**: 电机驱动板独立 5V 供电，不从 P4 取电。IMU 通过 USB 口供电。上电顺序：先驱动板 → 再 P4。

## 双模式操控

| 模式 | 触发方式 | 行为 | 控制路径 |
|------|----------|------|----------|
| **速度模式** (D-Pad) | 按下/松手方向键 | 直接 `Contrl_Speed(M1..M4)`，松手即时 STOP | 速度模式优先 → 清空位置队列 → 延时 20ms 同步 → UART 指令 |
| **位置模式** (GO) | 点击 GO 按钮 | 编码器闭环走目标距离，10ms 周期比对脉冲增量 | `xQueueSend` → Motor_Task 独占消费 → 编码器实时反馈 |

- STOP 指令具有最高优先级：立即清空位置模式队列 + 速度模式标志位归零 + 发送零速指令
- `g_velocity_active` / `g_position_active` 标志位实现状态机互斥仲裁

## FreeRTOS 任务架构

| 任务名 | 栈 | 优先级 | 核心职责 |
|--------|:--:|:------:|----------|
| Motor_Task | 4096 | 5 | 100Hz 编码器闭环位置控制 (4 通道同步) |
| CamV4L2 | 8192 | 5 | MIPI-CSI 采集 + ISP + HW JPEG + 软件 AE |
| mjpeg_srv | 4096 | 5 | Port 81 MJPEG 推流 (独立 TCP, 不阻塞 httpd) |
| httpd (:80) | — | — | 网页 + 控制 API + 快照 + 状态 JSON |
| HuDetTask | 12288 | 3 | ESP-DL MSRMNP 人脸检测 + 5 关键点 (异步 3Hz) |
| usb_lib | 4096 | 20 | USB Host 库事件处理 |
| imu_conn | 4096 | 10 | IMU CDC 设备连接管理 + 数据接收 |
| WiFi_Scan | — | — | WiFi 扫描 (临时任务) |
| WiFi_Connect | — | — | WiFi 连接 (临时任务) |

> **关键设计**: Port 80 (httpd 控制) 与 Port 81 (MJPEG 推流) 物理隔离，消除大帧传输 (单帧 ~26KB) 对实时控制指令的排队阻塞。

## 组件说明

| 组件 | 路径 | 说明 |
|------|------|------|
| `uart_module` | `components/uart_module/` | UART0 驱动，GPIO20/21 115200 |
| `motor_module` | `components/motor_module/` | 电机文本协议解析 ($spd/$mtype)，编码器数据接收 ($MAll/$MTEP/$MSPD) |
| `wifi_module` | `components/wifi_module/` | ESP-Hosted-NG SDIO WiFi AP (WPA2-PSK) |
| `camera_module` | `components/camera_module/` | V4L2 + ISP + HW JPEG + 软件 AE + 50Hz 抗频闪 |
| `imu_usb` | `components/imu_usb/` | USB CDC Host + Wit-Motion 协议解析 + EMA 滤波 |
| `human_detect` | `components/human_detect/` | ESP-DL MSRMNP 人脸检测 + 5 关键点 (替代帧差法) |
| `web_control` | `main/web_control.c` | HTTP :80 (httpd) + MJPEG :81 (raw TCP) + 操控 HTML + IMU 面板 + 人脸检测开关 |

## 运动控制

### 电机配置

| 参数 | 值 | 说明 |
|------|-----|------|
| 电机类型 | 310 (MOTOR_TYPE=2) | 编码器减速电机 |
| 编码器相位 | 20 (PULSE_PHASE) | — |
| 编码器线数 | 13 (PULSE_LINE) | — |
| 轮径 | 48.00 mm | 轮周长 ~150.8 mm |
| 减速比 | 3.5 : 1 | 电机轴→车轮，实测校准值 |
| 电机轴脉冲 | 260 脉冲/转 | 20 × 13 |
| 车轮脉冲 | 910 脉冲/转 | 260 × 3.5 |
| 理论位移分辨力 | 0.17 mm/脉冲 | 150.8 ÷ 910 |
| 死区阈值 | 1300 | 电机驱动板最小 PWM 启动值 |
| 控制协议 | UART 115200-8N1 | `$spd:M1,M2,M3,M4#` |

### 位置模式精度

实测条件：白色网格条纹尼龙布地面，温度 25.6℃，风速可忽略。

| 距离/速度 | 次数 | 均值 | 标准差 | 相对误差 | 极差 |
|-----------|:----:|:-----:|:-----:|:--------:|:----:|
| 50cm / speed=50 | 10 | 48.87 cm | 0.06 cm | 2.3% | 0.16 cm |
| 80cm / speed=60 | 10 | 77.55 cm | 0.06 cm | 3.1% | 0.20 cm |

- 重复性：标准差 ≤ 0.06 cm，极差 ≤ 0.20 cm
- 系统偏差表现为稳定比例误差 (2.3%~3.1%)，可通过微调减速比校准系数消除

### 指令响应延迟

实测条件：250cm 距离，两级速度各 5 次，测量 HTTP 指令发出至电机启动的端到端延迟。

| 速度 | 次数 | 均值 | 标准差 | 极差 |
|:----:|:----:|:----:|:-----:|:----:|
| 80 | 5 | 71.2 ms | 3.6 ms | 8 ms |
| 40 | 5 | 74.8 ms | 1.9 ms | 5 ms |
| **综合** | **10** | **73.0 ms** | **3.3 ms** | **9 ms** |

- 延迟主要由 FreeRTOS 任务调度抖动 (tick 10ms) 及 UART 串行传输 (< 2ms @ 115200bps) 构成
- 两组速度间均值差异 < 4ms，验证控制路径速率无关性

## 10轴 IMU 惯导模块

| 项目 | 说明 |
|------|------|
| 型号 | Wit-Motion 10轴 IMU (亚博智能) |
| 接口 | USB CDC → CP2102 (0x10C4:0xEA60, bInterfaceClass=0xff) |
| 协议 | Wit-Motion Normal Protocol (0x55 帧头, 11 字节帧, 校验和) |
| 波特率 | 9600 8N1 |
| 数据帧 | 加速度(0x51) / 角速度(0x52) / 欧拉角(0x53) / 磁场(0x54) / 气压(0x56) / 四元数(0x59) |
| EMA 滤波 | α = 0.15, 对 Accel X/Y、Gyro X/Y/Z、Roll、Pitch 低通平滑 |
| 保持原始值 | Accel Z (重力轴)、Yaw (航向角) — 不滤波，避免相位失真 |

### 网页 IMU 面板

- **Roll / Pitch**: ±45° 条形图 + 数值，超 ±30° 变红警告
- **Yaw**: 地磁航向角数值
- **加速度/角速度/磁场**: 三轴数值显示
- **气压/高度/四元数/温度**: 完整传感器数据
- 离线时面板半透明不可用

## 人脸检测

| 项目 | 说明 |
|------|------|
| 推理引擎 | ESP-DL v3.1.0 |
| 模型 | MSRMNP (MSR 候选扫描 + MNP 关键点回归), int8 量化 |
| 模型大小 | MSR ~61KB + MNP ~130KB ≈ 191KB (嵌入 flash RODATA) |
| 检测频率 | 3 Hz (异步 FreeRTOS task, 不阻塞摄像头管线) |
| 输入快照 | 640×360 RGB565 (从 640×480 主帧下采样) |
| 输出 | 人脸位置框 + 5 关键点 (左右眼/鼻子/左右嘴角) |
| 画框 | 红框 + 彩色关键点 (眼青/鼻黄/嘴粉), pre-JPEG 原地修改 |
| 控制 | `/detect` 端点, Web UI 开关按钮, `/status` 含 detect 字段 |

## WiFi AP

| 项目 | 值 |
|------|-----|
| SSID | `ESP32-Car` |
| 安全 | WPA2-PSK |
| 密码 | `12345678` |
| 信道 | Channel 1 (2412 MHz) |
| 最大客户端 | 4 |
| IP | `192.168.4.1` |
| 控制页 | http://192.168.4.1/ |
| 摄像头流 | http://192.168.4.1:81/ |
| 传输框架 | ESP-Hosted-NG (esp_wifi_remote) |
| SDIO 总线 | 4-bit @ 40MHz |

## HTTP 端点

| 路径 | 端口 | 说明 |
|------|:----:|------|
| `/` | 80 | 网页遥控界面 (D-Pad + GO/STOP + MJPEG + IMU 倾角 + 人脸检测开关) |
| `/ctrl?cmd=...&dist=...&speed=...` | 80 | 电机控制 API |
| `/snapshot` | 80 | 单帧 JPEG 快照 |
| `/status` | 80 | 实时遥测 JSON (编码器/速度/FPS/AE/IMU/人脸检测) |
| `/detect?en=0\|1` | 80 | 人脸检测开关 / 状态查询 |
| `/favicon.ico` | 80 | 204 No Content |
| `/` | 81 | MJPEG 流 (独立 FreeRTOS task) |

### 控制命令

| cmd | 含义 | 模式 |
|-----|------|------|
| `vel_fwd` | 前进 | 速度模式 (D-Pad, 限速 600) |
| `vel_back` | 后退 | 速度模式 (D-Pad, 限速 600) |
| `vel_left` | 左转 (原地差速) | 速度模式 (D-Pad) |
| `vel_right` | 右转 (原地差速) | 速度模式 (D-Pad) |
| `stop` | 立即停止 + 清空队列 | 最高优先级 |
| `go` | 前进指定距离 | 位置模式 (编码器闭环, speed 100–1000) |

示例: `http://192.168.4.1/ctrl?cmd=go&dist=50&speed=50`

### /status JSON API

```json
{"t":123456,"fps":25,"ae":{"b":128.5,"e":100,"g":4},
 "m":[{"e":1234,"s":150.5},...],"mode":"velocity","recv":1,
 "imu":{"r":-0.34,"p":1.16,"y":-67.3,
        "ax":-0.021,"ay":-0.006,"az":1.002,
        "gx":0.00,"gy":0.00,"gz":0.00,
        "mx":123,"my":456,"mz":789,
        "pr":101325,"alt":12.3,
        "q0":0.9999,"q1":0.0001,"q2":0.0002,"q3":0.0003,
        "tmp":25.3,
        "ok":1},
 "detect":{"en":1,"n":2}}
```

| 字段 | 说明 |
|------|------|
| `t` | 运行时长 (ms) |
| `fps` | 摄像头帧率 (~1s 滑动窗口) |
| `ae.b` | AE 平滑亮度 (0-255, EMA α=0.85) |
| `ae.e` | 当前曝光 (100μs 单位) |
| `ae.g` | 传感器增益索引 |
| `m[N].e` | 电机 N 编码器累计脉冲 |
| `m[N].s` | 电机 N 实际速度 (驱动板反馈, 驱动板内部单位) |
| `mode` | velocity / position / idle |
| `recv` | UART 数据接收标志 |
| `imu.r/p/y` | Roll/Pitch/Yaw 欧拉角 (°) |
| `imu.ax/ay/az` | 三轴加速度 (g) |
| `imu.gx/gy/gz` | 三轴角速度 (°/s) |
| `imu.mx/my/mz` | 三轴磁场 (原始 ADC) |
| `imu.pr` | 气压 (Pa) |
| `imu.alt` | 高度 (cm) |
| `imu.q0-q3` | 四元数 |
| `imu.tmp` | IMU 温度 (°C) |
| `imu.ok` | IMU 连接状态 (1=在线, 0=离线) |
| `detect.en` | 人脸检测开关状态 |
| `detect.n` | 当前帧检测到的人脸数 |

## 摄像头

| 参数 | 值 | 说明 |
|------|-----|------|
| 传感器 | SC2336 | 自动检测, 也支持 OV5647 |
| 接口 | MIPI-CSI 2-Lane | RAW10 像素格式 |
| 分辨率 | 640×480 | VGA @ 50fps (传感器) |
| MJPEG 输出 | ~20 fps | 灰度 (RAW10→8bit, 无 ISP 彩色管线) |
| 单帧大小 | ~26 KB (典型值) | quality=55 |
| 视频带宽 | ~520 KB/s | 仅占 SDIO 吞吐 ~3% |
| 处理管线 | CSI Ctrl → ISP (RAW10→RGB565) → HW JPEG | V4L2 + esp_video |
| SCCB I2C | I2C0 GPIO7/8 @ 100kHz | 传感器寄存器配置 |

### 软件自动曝光 (AE)

| 参数 | 值 | 说明 |
|------|-----|------|
| 采样方式 | RGB565 绿通道 (G6→G8 转换) | 每 6 行/列间隔采样 |
| AE 频率 | 2.5 Hz | 每 20 帧触发一次 (50fps ÷ 20) |
| 平滑滤波 | EMA α = 0.85 | 新数据权重 85%，快速响应亮度变化 |
| 死区 | ±15 (128 阶亮度) | 防止微小波动震荡 |
| 目标亮度 | 65 (~25%) | 为暗光人脸检测保留对比度 |
| 方向确认 | 连续 2 次同方向 | 单次波动不触发调整 |
| 静默期 | 42 帧 (~840ms) | 寄存器写入后传感器充分稳定 |
| 方向翻转惩罚 | 额外 50 帧 (~1s) | 防止反复震荡 |
| 曝光步进 | ±10% (正常) / -12% (过曝) | 增益步进 5 档 |
| **抗频闪** | **50Hz — 曝光强制取整至 10ms 整数倍** | 消除室内日光灯画面滚动条纹 |

### 数据流

```
SC2336 → MIPI-CSI 2-Lane RAW10 → CSI Controller
  → ISP (拜耳→RGB565, 640×480) → /dev/video0
  → 软件 AE (绿通道采样 + EMA + V4L2 CID)
  → 人脸检测快照 (640×360 下采样) → ESP-DL (异步 3Hz)
  → HW JPEG Encoder (pre-JPEG 画框) → 双缓冲 + 信号量 → HTTP MJPEG :81

IMU → CP2102 → USB 2.0 Type-A → DWC OTG PHY
  → CDC ACM Host → Wit-Motion 协议解析 → EMA 滤波 → /status JSON → Web 面板
```

## 控制摘要

| 控制指标 | 数值 | 取证方式 |
|----------|:----:|:--------:|
| 闭环控制频率 | 100 Hz (10ms) | 代码 `delay_ms(10)` |
| 指令响应延迟 | 73.0 ± 3.3 ms | 10 次实测 |
| UART 传输延时 | < 2 ms | 计算 (20B ÷ 115200bps) |
| 指令帧长 | < 20 字节 | `$spd:±xxx,...,±xxx#` |
| 位置精度 (50cm) | ≤ 2.3% | 10 次实测 |
| 位置精度 (80cm) | ≤ 3.1% | 10 次实测 |
| 位移重复性 | 标准差 ≤ 0.06 cm | 10 次实测 |
| 操作模式 | 2 种互斥 | 代码 |
| 传输隔离 | 双端口 (80/81) | 代码 |
| FreeRTOS 任务数 | 9 | 代码 `xTaskCreate` |
| Web 控件 | ≥ 5 项 | 代码 |
| APK 页面 | 5 Tab | uni-app |
| 人脸检测 | 3 Hz 异步 | 代码 |

## Android APK 遥控终端

基于 uni-app (Vue 3) 跨平台框架开发的配套 Android 原生 APK，与 Web 控制面板构成互补双终端操控体系。

| Tab | 功能 |
|-----|------|
| 控制中心 | WiFi 连接管理 + 状态总览 |
| 搜救小车 | HTTP RESTful API 双向数据交互 |
| 图传 | MJPEG 视频流实时显示 |
| IMU 姿态监视 | 实时倾角数据可视化 |
| 摇杆 | 虚拟摇杆全向操控 |

## 构建与烧录

### 前提条件

- ESP-IDF **v5.5.4** (P4 Function Board v1.6)
- ESP32-C6 从机已烧录 ESP-Hosted slave 固件
- Wit-Motion 10轴 IMU 模块 (USB 连接)
- MSYS2 / MINGW 环境 (Windows)

### 构建

```bash
cd F:/project_ESP32_p4/UART
python build.py set-target esp32p4
python build.py build
```

`build.py` 会自动处理 MSYS2/IDF 环境差异 (清除 `MSYSTEM`)，无需手动 `export.bat`。

### 烧录与监控

```bash
python build.py flash        # 烧录
python build.py monitor      # 串口监视器
```

或在 **VS Code ESP-IDF 插件**中点击 ⚡ Flash 图标 (推荐)。

### 一键构建

```bash
build_esp.bat    # Windows CMD
```

## IDF v5.4 → v5.5.4 升级注意事项

1. **Managed components Kconfig 兼容**: `esp_hosted` 和 `esp_wifi_remote` 的 Kconfig 中 `$(ESP_IDF_VERSION)` 在 v5.5.4 不可用，需 patch 为硬编码 `idf_v5.5/`
2. **分区表扩容**: 加 IMU + USB + ESP-DL 模型后固件 > 1MB，自定义 `partitions_16MiB.csv` (factory=2.25MB)
3. **MSYS2 构建**: `build.py` 自动清除 `MSYSTEM` 环境变量，绕过 `idf.py` 的 MSYS 检查
4. 关键 patch 文件:
   - `managed_components/espressif__esp_hosted/Kconfig:1191`
   - `managed_components/espressif__esp_hosted/slave/main/Kconfig.projbuild:539`
   - `managed_components/espressif__esp_wifi_remote/Kconfig:8-10,21`
5. 删除 `dependencies.lock` 和 `managed_components/` 后重建会重新拉取，patch 会丢失

## 关键 sdkconfig 配置

| 配置 | 值 | 说明 |
|------|-----|------|
| `IDF_TARGET` | esp32p4 | 目标芯片 |
| `ESP_DEFAULT_CPU_FREQ_MHZ` | 400 | CPU 主频 |
| `FREERTOS_NUMBER_OF_CORES` | 2 | RISC-V 双核 |
| `SPIRAM_MODE` | HEX | PSRAM Hex SPI |
| `SPIRAM_SPEED` | 200MHz | PSRAM 频率 |
| `ESPTOOLPY_FLASHSIZE` | 16MB | Flash 容量 |
| `PARTITION_TABLE_CUSTOM` | partitions_16MiB.csv | factory 2.25MB |
| `ESP_HOSTED_SDIO_HOST_INTERFACE` | y | SDIO 主机 |
| `ESP_HOSTED_CP_TARGET` | ESP32C6 | C6 从机 |
| `ESP_HOSTED_SDIO_4_BIT_BUS` | y | 4-bit SDIO |
| `ESP_HOSTED_SDIO_CLOCK_FREQ_KHZ` | 40000 | 40MHz 时钟 |
| `ESP_HOSTED_USE_MEMPOOL` | y | mempool 启用 |
| `ESP_HOSTED_MEMPOOL_PREFER_SPIRAM` | y | mempool 优先 PSRAM |
| `CAMERA_SC2336_MIPI_RAW10_640X480_50FPS` | y | 640×480 RAW10 50fps |
| `ESP_VIDEO_ENABLE_ISP` | y | ISP 管线 |

## SDIO 引脚 (P4 ↔ C6)

| 信号 | P4 GPIO |
|------|:------:|
| CMD | 19 |
| CLK | 18 |
| D0 | 14 |
| D1 | 15 |
| D2 | 16 |
| D3 | 17 |
| RESET | 54 |

## 日志静默

启动时自动静默以下日志噪音:

- `httpd_txrx` → ERROR (抑制 ECONNRESET=104)
- `ov5647` → NONE (抑制传感器自动检测失败)
- `sdmmc_req` → NONE (抑制 SDIO 空闲事件)
- `sccb_i2c` → ERROR
- `i2c.master` → ERROR

## 已知问题 / 注意事项

1. **GPIO10 Blink LED 未实现**: 引脚已从 GPIO8 (与摄像头 SCL 冲突) 移至 GPIO10，Kconfig 已配置但代码未实现 LED 控制逻辑
2. **UART0 控制台冲突**: `motor_init()` 重新初始化 UART0 (安装 RX 队列)，覆盖 ESP 默认控制台输出。初始化后 `printf` 通过 UART0 发送到电机驱动板
3. **电机驱动板独立供电**: 必须先给驱动板上电，再上电 P4，否则 UART 通信无响应
4. **CP2102 枚举**: IMU 模块的 CP2102 为 vendor-specific (0xff)，不能启用 `CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK`
5. **IMU 开机校准**: Wit-Motion 模块上电后 2-3 秒自动校准陀螺仪零偏，期间保持小车静止
6. **IMG 灰度输出**: 当前 sdkconfig 为 RAW10 模式，无 ISP 彩色管线，输出为灰度 MJPEG。如需彩色，需切换至 RAW8 + ISP RGB565 路径
7. **位置模式仅支持前进**: GO 按钮只实现前进方向编码器闭环，后退需通过 D-Pad 手动操控

## 目录结构

```
UART/
├── main/
│   ├── uart.c                # 主程序入口 + 电机控制逻辑 + 编码器闭环
│   ├── web_control.c         # HTTP :80 + MJPEG :81 + 操控 HTML + IMU JSON
│   ├── web_control.h         # 电机命令枚举 + 回调接口
│   ├── Kconfig.projbuild     # Blink LED 等配置项
│   └── idf_component.yml     # 组件依赖声明
├── components/
│   ├── uart_module/          # UART0 驱动 (GPIO20/21, 115200)
│   ├── motor_module/         # 电机驱动板文本协议 ($spd/$mtype) + 编码器数据解析
│   ├── wifi_module/          # ESP-Hosted-NG WiFi AP (WPA2-PSK)
│   ├── camera_module/        # V4L2 + CSI + ISP + HW JPEG + 软件 AE + 50Hz 抗频闪
│   ├── imu_usb/              # USB CDC Host + Wit-Motion 协议解析 + EMA 滤波
│   │   ├── imu_usb.c         # USB Host 初始化 + 连接任务
│   │   ├── imu_usb.h         # 公共 API
│   │   ├── imu_parser.c      # Wit-Motion Normal Protocol 解析器
│   │   └── imu_parser.h      # 数据结构 (imu_data_t) + 回调接口
│   └── human_detect/         # ESP-DL MSRMNP 人脸检测 + 5 关键点
│       ├── human_detect.cpp  # C 包装层 + 检测管线
│       ├── human_detect.h    # 公共 API
│       └── human_face_detect.cpp # ESP-DL 推理实现
├── partitions_16MiB.csv      # 自定义分区表
├── sdkconfig                 # 当前 Kconfig 配置
├── CMakeLists.txt            # 顶层 CMake
├── build.py                  # MSYS2/IDF 兼容构建封装
├── build_esp.bat             # Windows CMD 构建脚本
└── README.md                 # 本文件
```

## 版本历史

| 日期 | 变更 |
|------|------|
| 2026-07-08 | ✅ README 完整重写：性能实测数据、ESP-DL 人脸检测、APK、FreeRTOS 架构、AE 参数 |
| 2026-07-05 | ✅ IMU 气压/四元数修复 (发送配置命令启用输出) + 编码器减速比校准 3.5 |
| 2026-07-03 | ✅ ESP-DL MSRMNP 人脸检测 (替代帧差法) + 5 关键点 + 异步 3Hz + 分区表扩至 2.25MB |
| 2026-06-27 | ✅ Web 全中文化 + mode 映射修复 + IMU 数据补全 (加速度/角速度/磁场/气压/四元数/温度) |
| 2026-06-25 | ✅ Wit-Motion 10轴 IMU (USB CDC Host + EMA 滤波 + Web 倾角面板) + IDF v5.4→v5.5.4 |
| 2026-06-21 | ✅ /status JSON API + Web 编码器显示 + FPS + 模式徽章 |
| 2026-06-16 | ✅ 双模式操控 (D-Pad 速度 + GO 位置) + MJPEG Port 81 独立 task |
| 2026-06-14 | ✅ ISP RGB565 管线 + 软件 AE |
| 2026-06-09 | ✅ 项目创建，基础 UART + WiFi + 摄像头架构 |

## 2026-07-15: 人体检测 + 人脸识别升级

### ✅ PicoDet 人体检测 (~14FPS)
- **组件**: `components/body_detect/`
- **模型**: `espressif/pedestrian_detect` v0.2.0 (224x224, ~550KB PSRAM, ~71ms 推理)
- **特点**: 正面/侧面/背面都能检出，橙色框标记
- **API**: `/ctrl?cmd=pedestrian&dist=1` 开启, `dist=0` 关闭
- **状态**: `/status` JSON 新增 `pedestrian: {en, n}` 字段

### ✅ 人脸识别 (NCC 模板匹配)
- **组件**: `components/face_recog/`
- **算法**: MSRMNP 检测 → 关键点对齐 → 112x112 灰度裁剪 → NCC 模板匹配
- **数据库**: SPIFFS 存储, 每张人脸 12578 字节 (100人 < 1.2MB)
- **优势**: 零额外模型, 零 Flash 开销, ~5ms/匹配
- **API**:
  - `/enroll?name=xxx` — 录入当前帧人脸
  - `/faces` — 列出已注册人脸 `{faces: [{id, name}], count}`
  - `/faces?del=N` — 删除
  - `/recognize?en=1|0` — 识别开关
- **状态**: `/status` JSON 新增 `recognize: {en, db, n}` 字段

### 构建
```bash
cd F:/project_ESP32_p4/UART
python build.py build    # 编译 (固件 ~2.6MB, 4MB factory分区)
python build.py flash    # 烧录
```
