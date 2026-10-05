# 引脚、总线与协议接口

`bike.ioc` 来自较早的 CubeMX 工程，UART7、I2C2、TIM2 等后来由源码补充。因此发生冲突时，以当前 `Src/*.c`、`Inc/*.h` 和实车接线为准，不要只看 `.ioc`。

## 1. STM32 接口总表

| 功能 | 外设 | STM32 引脚 | 参数 | 对端 |
| --- | --- | --- | --- | --- |
| SWD 数据 | SYS_JTMS-SWDIO | PA13 | 3.3 V SWD | ST-Link SWDIO |
| SWD 时钟 | SYS_JTCK-SWCLK | PA14 | 建议从 1 MHz 调试 | ST-Link SWCLK |
| ODrive CAN 接收 | CAN2_RX | PB12 | 250 kbit/s | CAN 收发器 RX |
| ODrive CAN 发送 | CAN2_TX | PB13 | 250 kbit/s | CAN 收发器 TX |
| CH100 接收 | UART8_RX | PE0 | 460800 8N1 | CH100 TX |
| CH100 发送 | UART8_TX | PE1 | 460800 8N1 | CH100 RX |
| Mac 遥测发送 | UART7_TX | PE8 | 115200 8N1，20 Hz CSV | USB-UART RX |
| Mac/上位机接收 | UART7_RX | PE7 | 115200 8N1 | USB-UART TX |
| 历史上位机串口 | USART6_TX | PG14 | 默认 115200 8N1 | 旧上位机接口 |
| 历史上位机串口 | USART6_RX | PG9 | 默认 115200 8N1 | 旧上位机接口 |
| 舵机 PWM | TIM2_CH3 | PA2 | 50 Hz，当前 580–980 µs | DS5180 信号线 |
| OLED 数据 | I2C2_SDA | PF0 | 100 kHz，7 位地址 0x3C | SSD1306 |
| OLED 时钟 | I2C2_SCL | PF1 | 100 kHz | SSD1306 |
| 红色 LED | GPIO output | PE11 | 推挽输出 | 板载 LED |
| 绿色 LED | GPIO output | PF14 | 推挽输出 | 板载 LED |
| 其他输出 | GPIO output | PF6 | 当前用途待确认 | 板载/扩展 |
| 按键 | GPIO input | PE4 | 源码中读取 | KEY0 |

## 2. CAN 与 ODriveSimple

### 物理与位时序

- 外设：CAN2。
- APB1 时钟：42 MHz。
- Prescaler：12。
- BS1：12 TQ。
- BS2：1 TQ。
- SJW：1 TQ。
- 位率：`42 MHz / 12 / (1 + 12 + 1) = 250 kbit/s`。
- 标准 11 位 ID。
- 过滤器 Bank 14，当前使用全通掩码，由软件检查节点和命令。

CAN2 在 STM32F4 上与 CAN1 共享过滤器资源，所以初始化中同时打开 CAN1 时钟，并将 `SlaveStartFilterBank` 设为 14。

### 节点映射

| 逻辑轴 | ODrive 轴 | CAN node ID | 固件宏 | 功能 |
| --- | --- | --- | --- | --- |
| 0 | Axis 0 | 24 / `0x18` | `AXIS0_CAN_NODE_ID` | 动量轮 |
| 1 | Axis 1 | 16 / `0x10` | `AXIS1_CAN_NODE_ID` | 后轮 |

ODriveSimple 标准帧 ID：

```text
can_id = (node_id << 5) | command_id
```

当前使用：

- `Set_Input_Vel`：发送 8 字节数据，前 4 字节为 little-endian `float` 速度，单位 turns/s；后 4 字节为零。
- `Get_Encoder_Estimates`：远程帧请求，收到的后 4 字节为 little-endian `float` 速度估计。
- Axis 0 每 2 ms 发送速度命令并请求速度反馈。
- Axis 1 每 40 ms 发送一次速度命令；当前后轮默认指令为 0。
- 速度反馈使用最近 3 个样本的移动平均。

### 方向约定

方向与实车安装有关。当前源码包含以下符号约定：

- 动量轮速度命令：`odrive.set_speed0` 直接发送到 Axis 0。
- 后轮速度命令：发送时使用 `-odrive.set_speed1`。
- 运行状态下当前后轮目标为 `-0.7`，再经过发送处取反；上电 `run_flag = 0`，不允许自行匀速转动。

更换相线、编码器方向、IMU 安装方向或电机安装方向后，必须重新进行悬空、固定车架的方向验证。

## 3. UART8：CH100

- PE0：RX。
- PE1：TX。
- 460800 baud，8N1，无硬件流控，16 倍过采样。
- RXNE 中断逐字节送入 `packet_decode()`。
- 解码结果来自 `id0x91`：欧拉角 `eul[]` 和角速度 `gyr[]`。
- 当前横滚映射：`roll = eul[1]`，`roll_rate = gyr[0]`。

注意：接口只要持续保留旧值，看起来仍“有数据”；因此安全状态机必须按最后有效帧时间判断新鲜度，而不能只检查数值是否有限。当前自动启动做了 100 ms 新鲜度检查，但运行中超时停机尚待完成。

## 4. UART7：Mac 与遥测

- PE8：TX。
- PE7：RX。
- 115200 baud，8N1，无硬件流控。
- 正式固件每 50 ms 输出一条 ASCII CSV，即 20 Hz。
- UART7 不是 STM32F427 ROM bootloader 支持的下载口，只能用于业务通信/遥测。
- 使用 ROM 串口下载时应改接 USART1 或 USART3，并正确控制 BOOT0；当前实车更推荐 ST-Link SWD。

只看遥测时可只接 `PE8 -> 转接器 RX` 和 GND。若进行回环诊断，再交叉连接 TX/RX 并刷入诊断构建。

## 5. 舵机 PWM

TIM2 输入时钟 84 MHz：

```text
Prescaler = 84 - 1  -> 1 MHz 计数，即 1 µs/计数
Period    = 20000-1 -> 20 ms，即 50 Hz
Channel   = 3       -> PA2
```

`servo_set_duty(duty)` 的参数是相对中心的整数偏移：

```text
pulse_us = clamp(780 + duty, 580, 980)
```

源码 `servo_init()` 的旧注释写着 PA0，但初始化和 PWM 输出实际均为 TIM2_CH3 / PA2。

## 6. I2C OLED

- I2C2：PF0 SDA、PF1 SCL。
- 100 kHz，开漏，上拉。
- SSD1306 7 位地址：`0x3C`。
- OLED 初始化目前在 `main.c` 中被注释，默认不参与运行。

## 7. 控制定时器与中断优先级

TIM3：

- 84 MHz 定时器时钟。
- Prescaler：`840 - 1`，得到 100 kHz。
- Period：`200 - 1`，得到 500 Hz / 2 ms 中断。
- 2 ms 中断中读取 IMU 状态、自动启动判断、平衡计算、ODrive CAN 命令与遥测快照。

主要中断优先级：

| 中断 | 抢占/子优先级 |
| --- | --- |
| CAN2 RX0 | 1 / 1 |
| UART8 | 1 / 1 |
| TIM3 | 1 / 3 |
| UART7 | 2 / 2 |

## 8. Flash 接口约束

固件映像最大区域为 `0x08000000–0x081BFFFF`。零点持久化占用 `0x081C0000–0x081FFFFF`。如果重新生成链接脚本、改变 MCU 型号、使用 CubeMX/Keil 默认散装文件或执行全片擦除，必须重新核对这两个区域，防止代码与数据重叠。
