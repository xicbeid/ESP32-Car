# ESP32-P4 搜救小车 — 异构双芯边缘AI视觉智控平台

基于 ESP32-P4/C6 的搜救移动平台，具备 UART 编码器闭环运动控制、CSI 彩色 MJPEG 实时推流、10轴 IMU 惯导姿态感知、片上 ESP-DL 人脸检测+识别，以及 Web 遥控终端。

```
ESP32-P4 WiFi AP "ESP32-Car" (192.168.4.1)
  ├── UART0 GPIO20/21 → 电机驱动板 ($spd:... 文本协议, 115200-8N1)
  ├── SDIO GPIO14-19,54 → C6 (ESP-Hosted-NG WiFi)
  ├── SC2336 MIPI-CSI 2-Lane RAW8 800×800@30fps (SCCB I2C0 GPIO7/8)
  │     └── CSI Ctrl → ISP(RGB565, AE/AWB/CCM/Gamma/BLC/LSC 全自动)
  │         → DMA buf(memcpy) → draw overlay → HW JPEG(Q=55) → MJPEG
  ├── USB 2.0 Type-A → CP2102 → Wit-Motion 10轴IMU
  │     └── USB CDC Host → Wit-Motion 协议解析 → EMA 滤波(α=0.15) → Web 面板
  ├── ESP-DL MSRMNP → 人脸检测 + 5关键点 (异步 ~7Hz, pre-JPEG 红框+绿点)
  ├── NCC 模板匹配 → 人脸识别 (异步 5Hz, SPIFFS 数据库, 零模型依赖)
  ├── HTTP :80 → 网页遥控 (D-Pad + GO + MJPEG + IMU + 10 端点)
  │     HTTP :81 → MJPEG 推流 (独立 TCP task)
  └── 10 FreeRTOS 任务, 双核 RISC-V @400MHz, 32MB PSRAM, 固件 2.7MB
```

## 硬件接线

| P4 引脚 | 方向 | 连接目标 | 接口 | 说明 |
|---------|:----:|----------|------|------|
| GPIO20 (U0TXD) | Out | 电机驱动板 RX | UART 115200-8N1 | 运动控制指令 ($spd 文本帧) |
| GPIO21 (U0RXD) | In | 电机驱动板 TX | UART 115200-8N1 | 四路编码器脉冲数据 |
| GPIO7 (SDA) | Bi-dir | SC2336 SCCB | I2C @100kHz | 传感器寄存器配置 |
| GPIO8 (SCL) | Out | SC2336 SCCB | I2C @100kHz | SCCB 时钟 |
| GPIO14-19,54 | Bi-dir | ESP32-C6 | SDIO 4-bit @40MHz | WiFi 数据总线 |
| MIPI D0±/D1±/CLK± | In | SC2336 | MIPI-CSI 2-Lane | 差分视频信号 |
| USB_OTG_DP,DM | Bi-dir | CP2102 IMU | USB 2.0 FS CDC | IMU 虚拟串口 |
| GND | — | 驱动板 GND | — | 共地 |

> 驱动板独立 5V 供电。上电顺序：先驱动板 → 再 P4。IMU USB 供电。

## 摄像头

| 参数 | 值 | 说明 |
|------|-----|------|
| 传感器 | SC2336, 自动检测 | 也支持 OV5647 |
| 接口 | MIPI-CSI 2-Lane | RAW8 800×800 @30fps |
| ISP 输出 | RGB565 800×800 | Bayer→RGB + 全自动 AE/AWB/CCM/Gamma/BLC/LSC |
| ISP 控制器 | `CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER=y` | 后台 IPA 算法引擎 |
| 缓冲模式 | V4L2 MMAP (3 buffers) → memcpy → DMA buf (非缓存 PSRAM) | 零帧缓冲 msync |
| JPEG | HW 编码, RGB565→JPEG, Q=55 | ~15-26KB/帧 |
| 帧率 | ~18-20fps | MJPEG 带宽 ~400-500 KB/s |
| 缓存策略 | DMA 缓冲区 (MALLOC_CAP_DMA) 不经 CPU 缓存 | 画框写入 PSRAM 直接硬件可见 |

### 数据流
```
SC2336 RAW8 800×800@30fps → MIPI-CSI → ISP(RGB565, AE/AWB/Gamma auto)
  → V4L2 MMAP DQBUF → memcpy → DMA buf(非缓存)
  → 人脸检测画框(pre-JPEG) → HW JPEG → 双缓冲 → MJPEG :81
```

## FreeRTOS 任务架构

| 任务 | 优先级 | 栈 | 职责 |
|------|:-----:|:--:|------|
| CamV4L2 | 5 | 8KB | V4L2 DQBUF → DMA buf → draw → JPEG → 双缓冲 |
| Motor_Task | 5 | 4KB | 100Hz 编码器闭环位置控制 |
| mjpeg_srv | 5 | 4KB | TCP :81 MJPEG 推流 |
| HuDetTask | 3 | 12KB | ESP-DL MSRMNP 人脸检测 ~7Hz |
| RecognTask | 2 | 16KB | NCC 人脸识别 5Hz |
| UART_RX | 2 | 4KB | 电机板 UART 数据接收 |
| Enc_Mon | 1 | 2KB | 编码器监控 |
| imu_conn | 10 | 4KB | IMU CDC 连接管理 |
| usb_lib | 20 | 4KB | USB Host 库事件 |

## 组件说明

| 组件 | 路径 | 说明 |
|------|------|------|
| `uart_module` | `components/uart_module/` | UART0 GPIO20/21 115200 |
| `motor_module` | `components/motor_module/` | $spd/$mtype 文本协议, 编码器解析 |
| `wifi_module` | `components/wifi_module/` | ESP-Hosted-NG SDIO WiFi AP |
| `camera_module` | `components/camera_module/` | V4L2 MMAP + DMA buf + HW JPEG + ISP 管线控制器 |
| `imu_usb` | `components/imu_usb/` | USB CDC Host + Wit-Motion 协议 + EMA 滤波 |
| `human_detect` | `components/human_detect/` | ESP-DL MSRMNP 人脸检测 + 5关键点 |
| `face_recog` | `components/face_recog/` | NCC 模板匹配人脸识别 (SPIFFS DB) |
| `body_detect` | `components/body_detect/` | PicoDet 人体检测 (**暂禁用**) |

## 人脸检测

| 项目 | 值 |
|------|-----|
| 模型 | ESP-DL **MSRMNP** (MSR 61KB + MNP 130KB, int8, flash RODATA) |
| 检测频率 | **~7Hz** (每3帧一次, DETECT_SKIP_FRAMES=2) |
| 快照 | 640×360 RGB565 (从 800×800 下采样, ~460KB PSRAM) |
| 置信度阈值 | 0.30 (暗光宽松) |
| 画框 | 红色 0xF800 线宽2 + 5关键点 (眼青/鼻黄/嘴粉) |
| 暗光增强 | 自动 sqrt 伽马 LUT (max_g<100 触发, 零亮光开销) |
| 缓存 | snapshot_enhance 后 C2M → detection task 跨核可见 |
| 框更新 | 人脸出现/消失后 ~0.9s 内刷新 |

## 人脸识别

| 项目 | 值 |
|------|-----|
| 算法 | **NCC 归一化互相关** 模板匹配 (零模型依赖) |
| 流程 | 检测人脸 → 双线性插值 112×112 灰度 → 逐 DB 匹配 |
| 识别频率 | **5Hz** (200ms 间隔, 匹配检测 7Hz 节奏) |
| 匹配阈值 | **0.42** (暗光/微角度容忍) |
| 数据库 | SPIFFS binary, 每张脸 12.6KB, 100人 <1.2MB |
| 最小人脸 | <50px 跳过 (插值噪声) |

## 双模式操控

| 模式 | 触发 | 行为 |
|------|------|------|
| **速度模式** (D-Pad) | 按住走/松手停 | Contrl_Speed() 直控, STOP 最高优先级 |
| **位置模式** (GO) | 点击 GO | 编码器闭环 100Hz, 超时保护 |

| 性能指标 | 值 |
|----------|:--:|
| 响应延迟 | **73.0±3.3 ms** |
| 位置精度 | 50cm: 2.3%, 80cm: 3.1% |
| 重复性 | 标准差 ≤0.06 cm |

## WiFi AP

| 项目 | 值 |
|------|-----|
| SSID | `ESP32-Car` |
| 密码 | `12345678` |
| IP | `192.168.4.1` |
| 信道 | 1, WPA2-PSK, 最大4客户端 |
| 框架 | ESP-Hosted-NG SDIO 4-bit @40MHz |

## HTTP 端点 (:80)

| 端点 | 说明 |
|------|------|
| `/` | 嵌入式 HTML 遥控界面 |
| `/ctrl?cmd=vel_fwd\|stop\|go&dist=50&speed=50` | 电机控制 |
| `/status` | 实时 JSON 遥测 |
| `/snapshot` | 单帧 JPEG |
| `/detect?en=0\|1` | 人脸检测开关 |
| `/recognize?en=0\|1` | 人脸识别开关 |
| `/enroll?name=xxx` | 录入人脸 |
| `/faces` | 列出/删除已注册人脸 (?del=N) |
| `/favicon.ico` | 204 |

## /status JSON

```json
{
  "t": 123456, "fps": 20,
  "ae": {"b":128.5, "e":100, "g":4},
  "m": [{"e":1234,"s":150.5}, ...],
  "mode": "velocity", "recv": 1,
  "imu": {
    "r":-0.3,"p":1.2,"y":-67,
    "ax":0,"ay":0,"az":1, "gx":0,"gy":0,"gz":0,
    "mx":123,"my":456,"mz":789,
    "pr":101325,"alt":12.3,
    "q0":1,"q1":0,"q2":0,"q3":0,
    "tmp":25.3,"ok":1
  },
  "detect": {"en":1,"n":2},
  "recognize": {"en":1,"db":3,"n":1}
}
```

## 构建

**前提**: ESP-IDF v5.5.4, ESP32-C6 slave 已烧录, MSYS2/MINGW

```bash
cd F:/project_ESP32_p4/UART
python build.py build      # 编译
python build.py flash      # 烧录
python build.py monitor    # 监视器
```

**Kconfig 补丁** (每次删 managed_components 后需要):

```bash
sed -i '1191s/.*/\t\t\t\t# patched/' managed_components/espressif__esp_hosted/Kconfig
sed -i 's/\$ESP_IDF_VERSION/v5.5/g' managed_components/espressif__esp_wifi_remote/Kconfig
sed -i 's/\$ESP_IDF_VERSION/v5.5/g' managed_components/espressif__esp_wifi_remote/Kconfig.rpc.in
```

## 目录结构

```
UART/
├── main/
│   ├── uart.c / web_control.c / web_control.h
│   ├── CMakeLists.txt / idf_component.yml / Kconfig.projbuild
├── components/
│   ├── camera_module/  # V4L2 + ISP + JPEG (DMA buf, 零帧缓存 msync)
│   ├── human_detect/   # ESP-DL MSRMNP 人脸检测
│   ├── face_recog/      # NCC 模板匹配人脸识别
│   ├── body_detect/     # PicoDet 人体检测 (暂禁用)
│   ├── imu_usb/         # Wit-Motion 10轴IMU
│   ├── motor_module/    # 电机驱动 + 编码器
│   ├── uart_module/     # UART0 驱动
│   └── wifi_module/     # ESP-Hosted WiFi AP
├── partitions_16MiB.csv # factory 4MB + vfs 8MB + storage 4MB
├── sdkconfig / sdkconfig.defaults
└── build.py
```

## 已知问题

1. **人脸识别侧脸无效**: NCC 模板匹配需 ±30° 正脸, 侧脸/背对不可识别
2. **人体检测已禁用**: 代码保留在 body_detect/, 恢复只需取消注释
3. **UART0 控制台冲突**: motor_init() 重建 UART0 驱动, printf 输出到电机板
4. **CP2102 枚举**: 不可启用 `CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK`
5. **IMU 开机 2-3 秒校准**: 保持静止

## 版本历史

| 日期 | 变更 |
|------|------|
| 2026-07-16 | 🔧 摄像头重构: RAW8 800×800 @30fps + ISP 管线控制器全自动 AE/AWB/Gamma |
| 2026-07-16 | 🔧 DMA 缓冲区方案彻底解决 JPEG 缓存一致性闪烁 |
| 2026-07-16 | ✅ 人脸识别: NCC 模板匹配 + 阈值 0.42 + 5Hz 异步 + 快照缓存对齐 |
| 2026-07-16 | 📝 README 完整重写为当前状态 |
| 2026-07-15 | ✅ 人体检测 PicoDet (~14FPS) + 人脸识别 NCC (已注释人体检测) |
| 2026-07-15 | 🔧 检测帧率 3Hz→7Hz + 暗光增强 sqrt 伽马 + 框过期 30→6帧 |
| 2026-07-03 | ✅ ESP-DL MSRMNP 人脸检测 + 5关键点 |
| 2026-06-27 | ✅ 网页全中文化 + IMU 数据补全 |
| 2026-06-25 | ✅ WiFi AP + 10轴IMU + IDF v5.4→v5.5.4 |
| 2026-06-16 | ✅ 双模式操控 + MJPEG :81 |
| 2026-06-09 | ✅ 项目创建 |
