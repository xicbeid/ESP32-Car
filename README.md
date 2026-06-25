# ESP32-P4 UART WiFi 遥控小车 (带摄像头 + 10轴IMU)

基于 ESP32-P4 Function EV Board v1.6 的 WiFi AP 网页遥控小车，具备 UART 电机控制、CSI 摄像头 MJPEG 实时推流、10轴 IMU 惯导姿态显示、编码器闭环位置控制与速度模式。
[README.md](https://github.com/user-attachments/files/29142725/README.md)
# ESP32-P4 UART WiFi 遥控小车 (带摄像头)

基于 ESP32-P4 Function EV Board 的 WiFi AP 网页遥控小车，具备 UART 电机控制、CSI 摄像头 MJPEG 实时推流、编码器闭环位置控制与速度模式。

```
ESP32-P4 WiFi AP "ESP32-Car" (192.168.4.1)
  ├── UART0 GPIO20/21 → 电机驱动板 ($spd:... 文本协议)
  ├── SDIO GPIO14-19, 54 → C6 (ESP-Hosted WiFi)
  ├── SC2336 MIPI-CSI (SCCB I2C0 GPIO7/8)
  │     └── ISP → /dev/video0 (RGB565) → HW JPEG → 彩色 JPEG
  │     └── 软件 AE: RGB565 绿通道采样 → V4L2 CID 调曝光/增益
  ├── USB 2.0 Type-A → CP2102 → Wit-Motion 10轴IMU
  │     └── USB CDC Host → 协议解析 → EMA 滤波 → 网页倾角面板
  └── HTTP :80 → 网页遥控 (D-Pad 速度模式 + GO 位置模式 + MJPEG 实时画面 + IMU 姿态)
  └── HTTP :80 → 网页遥控 (D-Pad 速度模式 + GO 位置模式 + MJPEG 实时画面)
        HTTP :81 → MJPEG stream (独立 TCP task, 不阻塞 httpd)
```

## 硬件接线

| P4 引脚 | 连接目标 | 说明 |
|---------|----------|------|
| GPIO20 (TX) | 电机驱动板 RX | UART0 发送 |
| GPIO21 (RX) | 电机驱动板 TX | UART0 接收 |
| GPIO7 (SDA) | 摄像头 SCCB SDA | I2C0 传感器配置 |
| GPIO8 (SCL) | 摄像头 SCCB SCL | I2C0 传感器配置 |
| GPIO10 | Blink LED | 运行指示灯 (原 GPIO8 冲突) |
| GPIO14-19, 54 | ESP32-C6 SDIO | ESP-Hosted WiFi 协处理 |
| USB 2.0 Type-A | IMU 惯导模块 (CP2102) | USB CDC Host, 独立 DWC OTG PHY |
| GND | 电机驱动板 GND | 共地 |

> **注意**: 电机驱动板独立 5V 供电，不从 P4 取电。IMU 通过 USB 口供电。
| GND | 电机驱动板 GND | 共地 |

> **注意**: 电机驱动板独立 5V 供电，不从 P4 取电。

## 双模式操控

| 模式 | 触发方式 | 行为 |
|------|----------|------|
| **速度模式** (D-Pad) | 按下方向键 | 直接 `Contrl_Speed()` 低速运行，松手即停 |
| **位置模式** (GO) | 点击 GO 按钮 | 编码器闭环走目标距离 |

- D-Pad 固定低速 (speed=25/100)，绕过编码器队列，即时响应
- GO 按钮使用滑块设定的距离+速度，走 encoder closed-loop
- STOP 按钮 / D-Pad 松手 → 即时停止 + 清空队列
- `g_velocity_active` 标志位防止两种模式互相干扰

## 架构

```
┌─────────────────────────────────────────────────────────────┐
│                       app_main()                            │
│  uart.c: NVS → Camera → Motor → WiFi → IMU USB → Web       │
├──────────┬──────────┬──────────┬──────────┬────────────────┤
│ uart_    │ motor_   │ wifi_    │ imu_usb  │  web_control   │
│ module   │ module   │ module   │          │                │
│ UART0    │ 电机协议 │ ESP-     │ CDC Host │ Port 80: httpd  │
│ GPIO20/21│ $spd/... │ Hosted   │ CP2102   │  /ctrl/snapshot │
│ 115200   │          │ AP       │ 10轴IMU  │ Port 81: MJPEG  │
│          │          │          │          │  raw TCP task   │
├──────────┴──────────┴──────────┴──────────┴────────────────┤
│ camera_module: CSI → ISP → HW JPEG + 软件 AE               │
└─────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────┐
│                    app_main()                        │
│  uart.c: NVS → Camera → Motor → WiFi → Web          │
├─────────────┬──────────┬──────────┬─────────────────┤
│ uart_module │  motor   │  wifi    │   web_control   │
│ UART0 驱动  │ _module  │ _module  │                 │
│ GPIO20/21   │ 电机协议 │ ESP-     │ Port 80: httpd   │
│ 115200      │ $spd/... │ Hosted   │   /ctrl/snapshot │
│             │          │ AP       │ Port 81: MJPEG   │
│             │          │          │   raw TCP task   │
├─────────────┴──────────┴──────────┴─────────────────┤
│ camera_module: CSI → ISP → HW JPEG + 软件 AE        │
└─────────────────────────────────────────────────────┘
```

### 组件说明

| 组件 | 文件 | 说明 |
|------|------|------|
| `uart_module` | `components/uart_module/` | UART0 驱动，GPIO20/21 115200 |
| `motor_module` | `components/motor_module/` | 电机文本协议解析，编码器数据 |
| `wifi_module` | `components/wifi_module/` | ESP-Hosted SDIO WiFi AP |
| `camera_module` | `components/camera_module/` | esp_video (V4L2) + ISP + HW JPEG + 软件 AE |
| `imu_usb` | `components/imu_usb/` | USB CDC Host + Wit-Motion 协议解析 + EMA 滤波 |
| `web_control` | `main/web_control.c` | HTTP :80 (httpd) + MJPEG :81 (raw TCP) + 操控 HTML + IMU 倾角面板 |

## 10轴 IMU 惯导模块

- **型号**: Wit-Motion 10轴 IMU (亚博智能)
- **接口**: USB 转串口 (CP2102, bInterfaceClass=0xff)
- **协议**: Wit-Motion Normal Protocol (0x55 帧头, 11 字节帧, 校验和)
- **数据帧**: 加速度 (0x51) / 角速度 (0x52) / 欧拉角 (0x53) / 磁场 (0x54) / 气压 (0x56) / 四元数 (0x59)
- **波特率**: 9600 8N1 (模块默认)
- **滤波**: 对 Accel/Gyro/Roll/Pitch 使用 EMA 低通滤波 (α=0.15)，抑制电机振动

### 网页显示

网页控制面板增加 IMU 实时倾角显示：
- **Roll / Pitch**: ±45° 条形图 + 数值，超 ±30° 变红警告
- **Yaw**: 地磁航向角数值
- 断开时面板半透明不可用
| `web_control` | `main/web_control.c` | HTTP :80 (httpd) + MJPEG :81 (raw TCP) + 双模式操控 HTML |

## WiFi AP

| 项目 | 值 |
|------|-----|
| SSID | `ESP32-Car` |
| 密码 | `12345678` |
| 信道 | 1 |
| IP | `192.168.4.1` |
| 控制页 | http://192.168.4.1/ |
| 摄像头流 | http://192.168.4.1:81/ |

## HTTP 端点

| 路径 | 端口 | 说明 |
|------|------|------|
| `/` | 80 | 网页遥控界面 (D-Pad + GO/STOP + 实时画面 + IMU 倾角) |
| `/ctrl?cmd=...&dist=...&speed=...` | 80 | 电机控制 API |
| `/snapshot` | 80 | 单帧 JPEG 快照 |
| `/status` | 80 | 实时遥测 JSON (编码器/速度/FPS/AE/模式/IMU) |
| `/` | 80 | 网页遥控界面 (D-Pad + GO/STOP + 实时画面) |
| `/ctrl?cmd=...&dist=...&speed=...` | 80 | 电机控制 API |
| `/snapshot` | 80 | 单帧 JPEG 快照 |
| `/favicon.ico` | 80 | 204 No Content (静默) |
| `/` | 81 | MJPEG 流 (独立 FreeRTOS task) |

### 控制命令 (cmd)

| cmd | 含义 | 模式 |
|-----|------|------|
| `vel_fwd` | 前进 | 速度模式 (D-Pad) |
| `vel_back` | 后退 | 速度模式 (D-Pad) |
| `vel_left` | 左转 | 速度模式 (D-Pad) |
| `vel_right` | 右转 | 速度模式 (D-Pad) |
| `stop` | 立即停止 | - |
| `go` | 前进指定距离 | 位置模式 (GO) |

示例: `http://192.168.4.1/ctrl?cmd=go&dist=30&speed=50`

## 摄像头

- **传感器**: SC2336 (自动检测，也支持 OV5647)
- **接口**: MIPI-CSI 2-lane RAW8
- **分辨率**: 1280×720 @ 50fps (RAW10) → JPEG 约 26KB/帧
- **处理管线**: CSI Ctrl → ISP (RAW8→RGB565) → HW JPEG Encoder
- **自动曝光**: RGB565 绿通道每 10 帧采样 → V4L2 CID 调节曝光/增益
- **MJPEG 推流**: ~20fps，独立 TCP task (port 81)，不阻塞 httpd
- **实时遥测仪表盘**: Web 页面内嵌 IMU 倾角面板 + 编码器柱状图 + 速度显示 + FPS + 模式徽章，200ms JSON 轮询

### /status JSON API

```json
{"t":123456,"fps":25,"ae":{"b":128.5,"e":100,"g":4},
 "m":[{"e":1234,"s":150.5},...],"mode":"velocity","recv":1,
 "imu":{"r":-0.34,"p":1.16,"y":-67.3,
        "ax":-0.021,"ay":-0.006,"az":1.002,
        "gx":0.00,"gy":0.00,"gz":0.00,
        "ok":1}}
```

| 字段 | 说明 |
|------|------|
| `t` | 运行时长 (ms) |
| `fps` | 摄像头帧率 (~1s 滑动窗口) |
| `ae.b` | 软件 AE 平滑亮度 (0-255) |
| `ae.e` | 当前曝光 (100μs 单位) |
| `ae.g` | 传感器增益索引 |
| `m[N].e` | 电机 N 编码器累计脉冲 |
| `m[N].s` | 电机 N 实际速度 (mm/s) |
| `mode` | velocity / position / idle |
| `recv` | UART 数据接收标志 |
| `imu.r` | Roll 倾角 (°) |
| `imu.p` | Pitch 倾角 (°) |
| `imu.y` | Yaw 航向 (°) |
| `imu.ax/ay/az` | 加速度 (g) |
| `imu.gx/gy/gz` | 角速度 (°/s) |
| `imu.ok` | IMU 连接状态 (1=在线, 0=离线) |
- **分辨率**: 640×480 @ 50fps (RAW10)
- **处理管线**: CSI Ctrl → ISP (RAW8→RGB565) → HW JPEG Encoder
- **自动曝光**: RGB565 绿通道每 10 帧采样 → V4L2 CID 调节曝光/增益
- **MJPEG 推流**: ~20fps，独立 TCP task (port 81)，不阻塞 httpd

### 数据流

```
SC2336 → MIPI-CSI 2-lane RAW8 → CSI Controller
  → ISP (拜耳→RGB565) → /dev/video0
  → HW JPEG Encoder → 双缓冲 + 信号量 → HTTP MJPEG

IMU → CP2102 → USB 2.0 Type-A → DWC OTG PHY
  → CDC ACM Host → 协议解析 → EMA 滤波 → /status JSON → Web 面板
```

## 电机配置

| 参数 | 值 |
|------|-----|
| 电机类型 | 310 电机 (`MOTOR_TYPE=2`) |
| 编码器相位 | 20 (`PULSE_PHASE`) |
| 编码器线数 | 13 (`PULSE_LINE`) |
| 轮径 | 48.00 mm |
| 死区 | 1300 |
| 上传数据 | 总量编码器 (`UPLOAD_DATA=1`) |
| 每圈脉冲 | 260 (20×13) |
| 每 cm 脉冲 | ~17.24 |

> 支持 5 种电机类型切换，修改 `uart.c` 中 `MOTOR_TYPE` 宏即可。

## 构建与烧录

### 前提条件

- ESP-IDF **v5.5.4** (P4 Function Board v1.6 要求 >5.4)
- ESP32-C6 从机已烧录 ESP-Hosted slave 固件
- Wit-Motion 10轴 IMU 模块 (通过 USB 连接)
- ESP-IDF v5.3.1+ (带 PSRAM + esp_video 支持)
- ESP32-C6 从机已烧录 ESP-Hosted slave 固件

### 构建

```bash
cd F:/project_ESP32_p4/UART
python build.py set-target esp32p4
python build.py build
```

`build.py` 会自动处理 MSYS2/IDF 环境差异，无需手动 `export.bat`。

### 烧录与监控

```bash
python build.py flash        # 烧录 (需先设置 ESPPORT 或指定 -p COMx)
python build.py monitor      # 串口监视器
```

或在 **VS Code ESP-IDF 插件**中点击 ⚡ Flash 图标 (推荐)。

### 一键构建脚本

```bash
build_esp.bat    # Windows CMD (MSYSTEM 自动清除)
```

## 构建注意事项

项目从 IDF v5.4 升级到 v5.5.4 时进行了以下适配 (patch 记录):

1. **Managed components Kconfig 兼容**: `esp_hosted` (v2.12.9) 和 `esp_wifi_remote` (v0.14.5) 的 Kconfig 中 `$(ESP_IDF_VERSION)` 宏替换为硬编码 `idf_v5.5/` 路径
2. **分区表扩容**: 增加 IMU 和 USB 组件后固件 >1MB，使用自定义 `partitions_16MiB.csv` (factory 分区 ~2MB)
3. **MSYS2 构建**: `build.py` 自动清除 `MSYSTEM` 环境变量规避 IDF 中 MSYS 检查拦截

idf.py set-target esp32p4
idf.py build
```

### 烧录与监控

```bash
idf.py -p COMx flash monitor
```

### 一键构建脚本

```bash
./build_esp.sh   # Linux/Mac
build_esp.bat    # Windows
```

## 关键配置项 (sdkconfig)

| 配置 | 值 | 说明 |
|------|-----|------|
| `IDF_TARGET` | esp32p4 | 目标芯片 |
| `SPIRAM_MODE` | HEX | PSRAM Quad HEX |
| `SPIRAM_SPEED` | 200MHz | PSRAM 频率 |
| `SPIRAM_USE_CAPS_ALLOC` | y | 使用 caps 分配器 |
| `SPIRAM_USE_CAPS_ALLOC` | y | 使用 caps 分配器 (与 brookesia 一致) |
| `ESP_HOSTED_SDIO_HOST_INTERFACE` | y | SDIO 主机模式 |
| `ESP_HOSTED_CP_TARGET` | ESP32C6 | 从机为 C6 |
| `ESP_HOSTED_SDIO_4_BIT_BUS` | y | 4-bit SDIO |
| `ESP_HOSTED_SDIO_CLOCK_FREQ_KHZ` | 40000 | SDIO 时钟 40MHz |
| `ESP_HOSTED_USE_MEMPOOL` | y | 使用 mempool |
| `ESP_HOSTED_MEMPOOL_PREFER_SPIRAM` | y | mempool 优先 PSRAM |
| `CAMERA_SC2336` | y | 启用 SC2336 |
| `ESP_VIDEO_ENABLE_ISP` | y | 启用 ISP |
| `BLINK_GPIO` | 10 | LED 引脚 |
| `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE` | 512 | USB 控制传输 |
| `CONFIG_PARTITION_TABLE_CUSTOM` | y | 自定义分区表 |
| `CONFIG_ESPTOOLPY_FLASHSIZE` | 16MB | 16MB Flash |

## SDIO 引脚 (P4 ↔ C6)

| 信号 | P4 GPIO |
|------|---------|
| CMD | 19 |
| CLK | 18 |
| D0 | 14 |
| D1 | 15 |
| D2 | 16 |
| D3 | 17 |
| RESET | 54 |

## 日志级别

启动时自动静默以下日志噪音:
- `httpd_txrx` → ERROR (抑制 ECONNRESET=104)
- `ov5647` → NONE (抑制传感器自动检测失败)
- `sdmmc_req` → NONE (抑制 SDIO 空闲事件)
- `sccb_i2c` → ERROR
- `i2c.master` → ERROR

## 已知问题 / 注意事项

1. **GPIO8 冲突**: 原 Blink LED 使用 GPIO8，但 GPIO8 也是摄像头 SCL (SCCB I2C)。已将 Blink LED 移至 GPIO10。
2. **UART0 控制台冲突**: `motor_init()` 会重新初始化 UART0 (安装 RX 队列)，覆盖 ESP 默认控制台输出。初始化后 `printf` 通过 UART0 发送到电机驱动板而非串口终端。
3. **ESP-Hosted mempool**: 使用 PSRAM mempool，与 brookesia phone 项目一致 (`USE_CAPS_ALLOC` + `mempool_prefer_spiram`)，不使用 OCT/MEMMAP 模式。
4. **电机驱动板独立供电**: 必须先给驱动板上电，再上电 P4 启动程序，否则 UART 通信无响应。
5. **CP2102 枚举**: IMU 模块的 CP2102 芯片 `bInterfaceClass=0xff` (vendor-specific)，不是标准 CDC ACM。不能启用 `CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK`，否则枚举被过滤。
6. **IMU 开机校准**: Wit-Motion 模块上电后 2-3 秒自动校准陀螺仪零偏，期间保持小车静止。

## 目录结构

```
UART/
├── main/
│   ├── uart.c           # 主程序入口 + 电机控制逻辑
│   ├── web_control.c    # HTTP 服务器 + MJPEG 流 + 网页 + IMU JSON
│   ├── web_control.c    # HTTP 服务器 + MJPEG 流 + 网页
│   └── web_control.h    # 电机命令枚举 + 回调接口
├── components/
│   ├── uart_module/     # UART0 驱动 (GPIO20/21)
│   ├── motor_module/    # 电机驱动板文本协议
│   ├── wifi_module/     # ESP-Hosted WiFi AP 封装
│   ├── camera_module/   # CSI 摄像头 + ISP + HW JPEG
│   └── imu_usb/         # USB CDC Host + Wit-Motion 协议解析 + EMA 滤波
│       ├── imu_usb.c    # USB Host 初始化 + 连接任务
│       ├── imu_usb.h    # 公共 API
│       ├── imu_parser.c # Wit-Motion Normal Protocol 解析器
│       └── imu_parser.h # 数据结构 + 回调接口
├── partitions_16MiB.csv # 自定义分区表 (16MB Flash)
├── CMakeLists.txt       # 顶层 CMake (引入 components/)
├── sdkconfig.defaults   # 默认 Kconfig 配置
├── build.py             # MSYS2/IDF 兼容构建封装
├── build_esp.bat        # Windows CMD 构建脚本
└── README.md            # 本文件
```

## 版本历史

| 日期 | 变更 |
|------|------|
| 2026-06-25 | ✅ 集成 Wit-Motion 10轴 IMU (USB CDC Host + 协议解析 + EMA 滤波 + Web 面板) + IDF v5.4→v5.5.4 升级 + 自定义分区表 |
| 2026-06-21 | ✅ 实时遥测仪表盘：/status JSON API + Web 编码器柱状图 + 速度 + FPS + 模式徽章 |
│   └── camera_module/   # CSI 摄像头 + ISP + HW JPEG
├── CMakeLists.txt       # 顶层 CMake (引入 components/)
├── sdkconfig.defaults   # 默认 Kconfig 配置
├── build_esp.bat        # Windows 构建脚本
└── build_esp.cmd        # 备选构建脚本
```

## 版本历史

| 日期 | 变更 |
|------|------|
| 2026-06-16 | ✅ 双模式操控 (D-Pad 速度 + GO 位置) + MJPEG 分离到 port 81 独立 task |
| 2026-06-14 | ✅ ISP 管线修复，RGB565 彩色画面，软件 AE |
| 2026-06-09 | ✅ 项目创建，基础 UART + WiFi + 摄像头架构 |
