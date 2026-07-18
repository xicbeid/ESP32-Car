# ESP32-P4 搜救小车 — 异构双芯边缘AI视觉智控平台

基于 ESP32-P4/C6 的搜救移动平台，具备 UART 编码器闭环运动控制、CSI 彩色 MJPEG 实时推流、10轴 IMU 惯导姿态感知、片上 ESP-DL 人脸检测+识别、LED/蜂鸣器/照明外设控制，以及 Web 遥控终端。

```
ESP32-P4 WiFi AP "ESP32-Car" (192.168.4.1)
  ├── UART0 GPIO20/21 → 电机驱动板 ($spd:... 文本协议, 115200-8N1)
  ├── SDIO GPIO14-19,54 → C6 (ESP-Hosted-NG WiFi)
  ├── SC2336 MIPI-CSI 2-Lane RAW8 800×800@30fps (SCCB I2C0 GPIO7/8)
  │     └── CSI Ctrl → ISP(RGB565, AE/AWB/CCM/Gamma 全自动)
  │         → DMA buf(memcpy) → draw overlay → HW JPEG(Q=55) → MJPEG
  ├── USB 2.0 Type-A → CP2102 → Wit-Motion 10轴IMU
  │     └── USB CDC Host → Wit-Motion 协议解析 → EMA 滤波(α=0.15) → Web 面板
  ├── ESP-DL MSRMNP → 人脸检测 + 5关键点 (异步 ~7Hz, pre-JPEG 红框+绿点)
  ├── NCC 模板匹配 → 人脸识别 (异步 5Hz, SPIFFS 数据库, 零模型依赖)
  ├── LED(GPIO23) → 状态指示 (AP就绪亮/客户端连入灭)
  ├── 蜂鸣器(GPIO22) → 人脸检测提示 ("滴滴-滴滴" 模式)
  ├── 照明(GPIO26) → 继电器控制探照灯 (网页开关)
  ├── HTTP :80 → 网页遥控 (11 端点)
  │     HTTP :81 → MJPEG 推流 (httpd chunked)
  └── 12 FreeRTOS 任务, 双核 RISC-V @400MHz, 32MB PSRAM, 固件 ~2.2MB
```

## 硬件接线

| P4 排针 | GPIO | 连接目标 | 说明 |
|:------:|:----:|----------|------|
| - | GPIO20 (U0TXD) | 电机驱动板 RX | UART 115200-8N1 |
| - | GPIO21 (U0RXD) | 电机驱动板 TX | 编码器脉冲数据 |
| - | GPIO7 (SDA) | SC2336 SCCB | I2C @100kHz |
| - | GPIO8 (SCL) | SC2336 SCCB | 时钟 |
| - | GPIO14-19,54 | ESP32-C6 | SDIO 4-bit @40MHz |
| - | MIPI D0±/D1±/CLK± | SC2336 | 差分视频 |
| - | USB_OTG | CP2102 IMU | USB CDC |
| **Pin 7** | **GPIO23** | **LED (有源蜂鸣器模块)** | 低电平亮, 外部供电 |
| **Pin 12** | **GPIO22** | **蜂鸣器 (有源模块)** | 低电平响, "滴滴-滴滴" |
| **Pin 31** | **GPIO26** | **照明继电器** | 低电平=灯泡亮, 外部充电宝供电 |
| Pin 3,6,9,14,20,25,30,34,39 | GND | 各模块共地 | — |

> 驱动板独立 5V 供电。上电顺序：先驱动板 → 再 P4。摄像头金属后盖需接地（MIPI 信号完整性）。

## 外设模块

### LED 状态灯 (GPIO23, 低电平有效)

| 事件 | LED |
|------|:--:|
| 板子上电, WiFi AP 就绪 | 💡 亮 |
| 手机/电脑连接 WiFi | 🔲 灭 |
| 所有客户端断开 | 💡 亮 |

### 蜂鸣器 (GPIO22, 低电平有效)

```
检测到人脸 → "滴滴-滴滴" 循环
  ├─ 滴 100ms ─┤├─ 停 100ms ─┤├─ 滴 100ms ─┤├─ 停 700ms ─┤

无人脸 / 关闭检测 → 立即静音
```

独立 FreeRTOS 任务 `BuzzerTask` (prio 1, 2KB 栈)。

### 照明继电器 (GPIO26, 低电平有效)

```
控制端: P4 3V3 → 继电器 VCC, GND → 继电器 GND, GPIO26 → 信号
被控端: 充电宝 → 继电器 COM+NO → 3W 灯泡
```

网页 "💡 照明" 按钮, API: `/ctrl?cmd=light&dist=1` (开) / `dist=0` (关)

## 摄像头

| 参数 | 值 |
|------|-----|
| 传感器 | SC2336, 自动检测 |
| 接口 | MIPI-CSI 2-Lane, RAW8 800×800 @30fps |
| ISP 输出 | RGB565, AE/AWB/CCM/Gamma 全自动 |
| 缓冲 | V4L2 MMAP (3) → memcpy → DMA buf (MALLOC_CAP_DMA, 非缓存) |
| JPEG | HW 编码, Q=55, ~15-30KB/帧 |
| 帧率 | ~18-20fps |
| **缓存策略** | DMA 缓冲区不经 CPU 缓存, 零帧缓冲 msync |

### 数据流
```
SC2336 RAW8 800×800@30fps → MIPI-CSI → ISP(RGB565, AE/AWB auto)
  → V4L2 MMAP DQBUF → memcpy → DMA buf(非缓存)
  → 人脸检测画框(pre-JPEG) → HW JPEG → 双缓冲 → MJPEG :81
```

## FreeRTOS 任务架构

| 任务 | 优先级 | 栈 | 职责 |
|------|:-----:|:--:|------|
| CamV4L2 | 5 | 8KB | V4L2 DQBUF → DMA buf → draw → JPEG |
| Motor_Task | 5 | 4KB | 100Hz 编码器闭环 |
| mjpeg_srv | 5 | 4KB | TCP :81 MJPEG 推流 |
| HuDetTask | 3 | 12KB | MSRMNP 人脸检测 ~7Hz |
| RecognTask | 2 | 16KB | NCC 人脸识别 5Hz |
| BuzzerTask | 1 | 2KB | 蜂鸣器 "滴滴-滴滴" 时序 |
| UART_RX | 2 | 4KB | 电机板 UART 接收 |
| Enc_Mon | 1 | 2KB | 编码器监控 |
| imu_conn | 10 | 4KB | IMU CDC |
| usb_lib | 20 | 4KB | USB Host |

## 组件

| 组件 | 路径 | 说明 |
|------|------|------|
| `camera_module` | `components/camera_module/` | V4L2 MMAP + DMA buf + HW JPEG + ISP |
| `human_detect` | `components/human_detect/` | ESP-DL MSRMNP 人脸检测+关键点 |
| `face_recog` | `components/face_recog/` | NCC 模板匹配, SPIFFS |
| `body_detect` | `components/body_detect/` | PicoDet 人体检测 (**暂禁用**) |
| `led_buzzer` | `components/led_buzzer/` | LED + 蜂鸣器 + 照明 GPIO |
| `imu_usb` | `components/imu_usb/` | Wit-Motion 10轴IMU |
| `motor_module` | `components/motor_module/` | $spd/$mtype 协议 + 编码器 |
| `uart_module` | `components/uart_module/` | UART0 GPIO20/21 |
| `wifi_module` | `components/wifi_module/` | ESP-Hosted WiFi AP |

## 人脸检测

| 项目 | 值 |
|------|-----|
| 模型 | ESP-DL MSRMNP (MSR 61KB + MNP 130KB, int8) |
| 频率 | ~7Hz (DETECT_SKIP_FRAMES=2) |
| 快照 | 640×360 RGB565 |
| 画框 | 红框 0xF800 线宽2 + 5关键点(眼青/鼻黄/嘴粉) |
| 暗光增强 | sqrt 伽马 LUT (max_g<100 触发) |
| 框过期 | ~0.9s |

## 人脸识别

| 项目 | 值 |
|------|-----|
| 算法 | NCC 归一化互相关, 零模型依赖 |
| 频率 | 5Hz (200ms) |
| 阈值 | 0.42 |
| 模板 | 112×112 灰度, 12.6KB/人 |
| 录入 | `/enroll?name=xxx` (需正脸) |

## 双模式操控

| 模式 | 触发 | 行为 |
|------|------|------|
| 速度 (D-Pad) | 按住/松手 | Contrl_Speed(), STOP 最高优先级 |
| 位置 (GO) | 点击 GO | 编码器闭环 100Hz |

| 指标 | 值 |
|------|:--:|
| 响应延迟 | 73.0±3.3 ms |
| 位置精度 | 2.3-3.1% |
| 重复性 | ≤0.06 cm |

## WiFi

| 项目 | 值 |
|------|-----|
| SSID | `ESP32-Car`, 密码 `12345678` |
| IP | `192.168.4.1`, 信道 1, WPA2-PSK |
| 框架 | ESP-Hosted-NG SDIO 4-bit @40MHz |

## HTTP 端点 (:80)

| 端点 | 说明 |
|------|------|
| `/` | HTML 遥控界面 |
| `/ctrl?cmd=vel_fwd\|stop\|go\|light&dist=…` | 电机/照明控制 |
| `/status` | JSON 遥测 |
| `/snapshot` | 单帧 JPEG |
| `/detect?en=0\|1` | 人脸检测开关 |
| `/recognize?en=0\|1` | 人脸识别开关 |
| `/enroll?name=xxx` | 录入人脸 |
| `/faces` | 列出/删除注册人脸 |
| `/favicon.ico` | 204 |

## 构建

**前提**: ESP-IDF v5.5.4, ESP32-C6 slave 烧录, MSYS2

```bash
cd F:/project_ESP32_p4/UART
python build.py build      # 编译 (~2.2MB, 4MB factory)
python build.py flash      # 烧录
python build.py monitor    # 监视器
```

**Kconfig 补丁** (删 managed_components 后):

```bash
sed -i '1191s/.*/\t\t\t\t# patched/' managed_components/espressif__esp_hosted/Kconfig
sed -i 's/\$ESP_IDF_VERSION/v5.5/g' managed_components/espressif__esp_wifi_remote/Kconfig
sed -i 's/\$ESP_IDF_VERSION/v5.5/g' managed_components/espressif__esp_wifi_remote/Kconfig.rpc.in
```

## 已知问题

1. **人脸识别侧脸无效**: NCC 需 ±30° 正脸
2. **人体检测已禁用**: 代码保留在 body_detect/
3. **摄像头金属壳需接地**: MIPI 差分信号完整性要求
4. **UART0 控制台冲突**: motor_init() 重建驱动
5. **IMU 开机 2-3s 校准**: 保持静止

## 版本历史

| 日期 | 变更 |
|------|------|
| 2026-07-17 | ✅ LED(GPIO23) + 蜂鸣器(GPIO22) + 照明(GPIO26) 外设模块 |
| 2026-07-17 | 📝 README 更新至当前最新状态 |
| 2026-07-16 | 🔧 DMA buf 方案彻底解决 JPEG 缓存一致性问题 |
| 2026-07-16 | ✅ 人脸识别 NCC 5Hz + 阈值 0.42 + 检测帧率 3→7Hz |
| 2026-07-03 | ✅ ESP-DL MSRMNP 人脸检测 + 5关键点 |
| 2026-06-25 | ✅ WiFi AP + 10轴IMU + IDF v5.4→v5.5.4 |
| 2026-06-16 | ✅ 双模式操控 + MJPEG :81 |
| 2026-06-09 | ✅ 项目创建 |
