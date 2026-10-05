# macOS + VS Code 开发与烧录

## 当前硬件识别结果

- 本次检查开始时 Mac 已识别 WCH USB 转串口：`/dev/cu.usbserial-1140`；最终复查时该设备已断开，重插后再确认实际设备名。
- 现有固件的 UART7 为 `PE8(TX) / PE7(RX)`，参数为 `115200 8N1`。
- UART8 为 CH100 IMU 接口，参数为 `460800 8N1`，不要用作上位机串口。
- 原固件没有周期日志，且 UART7 接收中断函数为空，因此静默监听不到数据不等于接线失败。
- STM32F427 的 ROM 串口下载只支持 USART1/USART3，不支持 UART7。

## 编译

终端执行：

```sh
cmake --preset mac-debug
cmake --build --preset mac-debug
```

输出文件：

- `build/debug/bike`：带调试信息的 ELF
- `build/debug/bike.hex`
- `build/debug/bike.bin`

VS Code 中可运行 `Terminal -> Run Task -> STM32: Build`。Arm GNU Toolchain 位于
`~/Library/ArmGNUToolchain/15.3.rel1`，CMake 也会在 `PATH` 和 Arm 官方默认目录中查找。

## 推荐烧录方式：ST-Link（SWD）

烧录前先让车轮离地，并断开 ODrive/电机动力电源，仅保留主控所需供电。

ST-LINK/V2 外壳上标注红/蓝/黑/白的四线接口是 SWIM，不能用于 STM32F427。应从
20 针 JTAG/SWD 接口连接目标板：`1 VAPP -> 3.3V`、`7 -> SWDIO`、
`9 -> SWCLK`、任一 GND 引脚（例如 `4 -> GND`），建议再接 `15 -> NRST`。
VAPP 是目标电压参考，目标主控板必须自行供电并接近 3.3 V。

1. 在 VS Code 运行 `STM32: Probe ST-Link`，确认输出中能识别到 STM32F42x/F43x，
   且 `chipid` 不是 `0x000`。注意 `st-info --probe` 可能执行软件复位；探测前也必须
   让车架固定、车轮悬空。
2. 运行 `STM32: Flash ST-Link`；任务会先重新编译，再写入并复位。

等价命令：

```sh
~/.local/bin/st-info --probe --freq=1000
~/.local/bin/st-flash --freq 1000 --reset write build/debug/bike.bin 0x08000000
```

## 备选烧录方式：板载 USB DFU

使用支持数据的 Micro-USB 线直接连接开发板与 Mac，按硬件手册要求进入系统
Bootloader 后，运行 `STM32: List DFU Devices`，再运行 `STM32: Flash DFU`。
USB-UART 线不能代替 USB 数据线。

## 验证现有 UART7 接线

工程提供了一个诊断构建。它会正确清除 UART7 接收中断，并把收到的字节原样回显；正式构建只接收并计数，不回显。

1. 进入 DFU，运行 `STM32: Flash DFU UART7 Diagnostic`。
2. 恢复 `BOOT0=0` 并复位。
3. 确认 USB-UART 使用 3.3 V TTL，接线为 `转接器 TX -> PE7/RX`、`转接器 RX -> PE8/TX`、`GND -> GND`。
4. 运行 `STM32: Test UART7 Echo`。成功时会输出 `PASS`。

不要同时连接 USB-UART 的 5 V 与已经供电的开发板；业务串口只需 TX、RX、GND。

## 在线平衡遥测

正式固件通过 UART7 的 `PE8/TX` 以 `115200 8N1` 输出 20 Hz CSV。数据包括横滚角、
横滚角速度、动量轮速度、速度指令、由指令差分得到的加速度指令、实际控制零点、
动态零点候选值、已持久化零点和序号、轮速回正偏置及状态标志。

接线只需要 `PE8/TX -> USB-UART RX` 和共地，不要连接转接器的 5 V。运行 VS Code
任务 `STM32: Monitor Balance Telemetry`，输入当前 `/dev/cu.usbserial-*` 设备名；监视器
会实时显示数据，并在 `logs/` 下保存带时间戳的 CSV。等价命令：

```sh
python3 tools/balance_telemetry.py /dev/cu.usbserial-1140
```

动态零点候选值会以不超过 `0.003°/s` 的速度参与控制，相对工厂基础零点最多偏移
`±1.50°`。只有横滚误差不超过 2°、角速度不超过
2°/s、动量轮速度不超过 15 tps 且回正偏置不超过 0.5° 时才允许学习。`ZERO_GATE` 表示
这些条件满足，连续满足 5 秒后出现 `ZERO_LEARNING`；0.5 tps 以内为零速死区。
`ZERO_APPLIED` 表示候选值正在用于控制，
`ZERO_LIMIT` 表示已经到达验证范围边界。
原始速度指令的瞬时尖峰仅用于 `VELOCITY_SAT` 遥测告警，不参与动态零点学习门控；实际
轮速、车身姿态和角速度仍参与门控。

动态零点使用 STM32F427 最后的两个 128 KiB Flash 扇区（22、23）持久化，固件链接区域
已缩小到 1792 KiB，避免程序覆盖参数区。每条追加记录包含版本、CRC、反码和最后提交
标记；扇区写满后才切换并擦除另一个扇区，断电时仍保留上一条有效记录。进入
`ZERO_LEARNING`（此前已经连续满足稳定条件 5 秒）且零点变化至少 `0.002°` 时允许首次
保存；之后距上次尝试至少 15 秒且仍满足变化门限时才追加。`ZERO_PERSIST_VALID` 表示存在可启动加载的记录，
`ZERO_PERSIST_SAVED` 表示本次上电已成功保存，`ZERO_PERSIST_ERROR` 表示最近一次保存失败。
突然断电最多可能丢失最近约 15 秒的微小修正；无有效记录或记录越界/校验失败时会安全
回退到工厂零点 `-2.18°`。

固件上电后不会立即启动动量轮平衡，而是保持 Axis0 速度指令为零。若 Flash 中存在有效
零点则先加载该值，否则采用 `-2.18°`。手工把车扶到启动零点
零点 `±3°` 内，并让横滚角速度保持在 `±1°/s` 内连续 2 秒后，控制器才自动启动；
离开窗口会重新计时。遥测中的 `ARM_WINDOW` 表示正在进行稳定计时，`CONTROL_ARMED`
表示平衡控制已经启动。还必须至少收到 5 个有效 IMU 数据帧，且最新帧不超过 100 ms；
`IMU_READY` 表示该启动条件满足。启动时会清空 PID 历史量，并抑制角速度环第一次计算的
微分冲击。IMU 新鲜度目前只阻止上电解锁，不会在已启动后自动停机。

## 备选：ROM 串口烧录

如果要用现在的 USB-UART 烧录，必须把信号线从 UART7 改接到板子的 USART3 口
（STM32 引脚 `PD8/TX`、`PD9/RX`），并以 `BOOT0=1`、`BOOT1=0` 复位进入 System Memory。
然后运行 `STM32: Flash ROM over USART3 (not UART7)`。

安装好的烧录器位于 `~/.local/bin/stm32flash`。UART7、UART8、USART6 均不能用于 STM32F427 ROM 串口烧录。

## SWD 调试

需要断点、单步和变量监视时，使用主控板的 SWD 口连接 ST-Link 或 J-Link。
不要把普通 USB-UART 或 ST-LINK/V2 的 SWIM 彩色四线接到 SWD 口。
