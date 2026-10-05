# Driverless Bicycle / 无人自行车

基于 STM32F427、CH100 IMU、ODrive v3.6、反作用动量轮和转向舵机的无人自行车底层控制项目。

当前仓库以本地实车代码为准，重点是先把静止自平衡、安全启动、动态零点和在线遥测做可靠，再逐步扩展动态骑行、视觉、激光雷达、建图和路径规划。

> **高能量系统安全警告**
>
> 本项目包含 48 V 级电源、双无刷电机和高速动量轮。烧录、复位、上电或修改控制参数都可能让动量轮瞬间高速转动并使车体倾倒。任何测试都应先架空车轮、固定车架、清空旋转平面、准备实体断电开关，并由人员扶车。不要把本文中的厂家极限参数当作安全工作参数。

## 当前能力

- STM32F427IIH6，168 MHz，TIM3 以 2 ms 周期运行底层平衡控制。
- CH100 IMU 通过 UART8（460800 8N1）提供姿态角与角速度。
- ODrive 56 V v3.6 通过 CAN2（250 kbit/s）驱动动量轮和后轮。
- 动量轮采用串级控制：角度环、角速度环、轮速回正环。
- 上电后动量轮保持零指令；车辆扶正并稳定 2 秒后自动使能平衡。
- 后轮上电默认不转，只有明确运行指令才允许驱动。
- 动态零点在线估计，范围为厂家零点 `-2.18° ± 1.5°`。
- 动态零点以双 Flash 扇区追加日志方式持久化，带 CRC、反码和提交标记。
- UART7 以 20 Hz 输出横滚角、角速度、轮速、速度/加速度指令、零点和饱和状态。
- macOS + VS Code + CMake + Arm GNU Toolchain 已打通编译；ST-Link SWD 已用于实车烧录。

## 系统结构

```text
                 CH100 IMU
              UART8 / 460800
                    │
                    ▼
              STM32F427IIH6
          ┌─────────┼──────────┐
          │         │          │
   CAN2 / 250k   TIM2 PWM    UART7 / 115200
          │         │          │
          ▼         ▼          ▼
    ODrive v3.6   DS5180     Mac / 上位机
      ┌───┴───┐   转向舵机   在线遥测与记录
      │       │
  Axis 0   Axis 1
  动量轮    后轮电机
  node 24   node 16
```

自行车的静止平衡依靠动量轮的**角加速度/反作用力矩**，不是依靠持续保持某个轮速。轮速回正环的作用是让动量轮平均速度回到零附近，避免长期单向加速后触及速度饱和。

## 仓库导航

| 路径 | 内容 |
| --- | --- |
| `Src/`、`Inc/` | STM32 应用、驱动、控制、遥测和零点持久化源码 |
| `Drivers/` | STM32F4 HAL/LL 与 CMSIS |
| `bike.ioc` | 历史 CubeMX 工程；部分后加外设以源码为准 |
| `CMakeLists.txt`、`CMakePresets.json` | macOS/命令行构建入口 |
| `cmake/` | Arm GCC 工具链与链接脚本 |
| `.vscode/` | VS Code 推荐扩展和任务 |
| `MDK-ARM/` | Keil 工程入口；编译产物不再纳入版本控制 |
| `tools/` | UART7 回环测试和在线遥测记录脚本 |
| `hardware/` | 厂家配置参考、实拍和注意事项 |
| `docs/` | 硬件、接口、控制、开发环境与项目状态说明 |

详细文档：

- [硬件与电气说明](docs/HARDWARE.md)
- [引脚、总线与协议接口](docs/INTERFACES.md)
- [平衡控制、动态零点与遥测](docs/CONTROL_AND_TELEMETRY.md)
- [开发、编译、烧录与调试环境](docs/DEVELOPMENT.md)
- [项目状态、已知风险与路线图](docs/STATUS.md)
- [macOS 与 VS Code 实操记录](docs/MAC_VSCODE.md)
- [ODrive 厂家资料与实车差异](hardware/vendor/README.md)

## 快速开始：只编译，不连接车辆

### 1. 安装依赖

macOS 推荐安装：

```bash
brew install cmake ninja stlink dfu-util stm32flash
```

另行安装 Arm GNU Toolchain。当前已验证版本为 `15.3.Rel1`，默认搜索路径：

```text
~/Library/ArmGNUToolchain/15.3.rel1/bin
```

### 2. 配置与编译

```bash
cmake --preset mac-debug
cmake --build --preset mac-debug
```

输出文件：

```text
build/debug/bike.elf
build/debug/bike.hex
build/debug/bike.bin
build/debug/bike.map
```

VS Code 中也可运行默认构建任务 `STM32: Build`。

## 烧录前检查

烧录会复位 MCU，实车可能立即进入自动扶正等待，随后启动动量轮。仓库中的任务不会替你完成机械安全确认。

1. 后轮与动量轮均离地，车架固定，旋转区域无人和杂物。
2. 操作者能够立即切断 ODrive/电机总电源。
3. ST-Link 连接 `SWDIO`、`SWCLK`、`3V3`、`GND`；目标板没有引出的 NRST 也可正常 SWD 烧录。
4. 确认当前编译产物和目标板型号为 STM32F427IIHx。
5. 明确同意本次复位/烧录后再执行命令。

```bash
~/.local/bin/st-info --probe --freq=1000
~/.local/bin/st-flash --freq 1000 --reset write build/debug/bike.bin 0x08000000
```

更多方式和故障排查见 [开发环境文档](docs/DEVELOPMENT.md)。

## 在线遥测

UART7 接线：

```text
STM32 PE8 / UART7_TX  -> USB-UART RX
STM32 PE7 / UART7_RX  <- USB-UART TX（只看遥测时可不接）
STM32 GND             --- USB-UART GND
```

参数为 115200、8 数据位、无校验、1 停止位。不要把 USB-UART 的 5 V 接到已供电主控。

```bash
ls /dev/cu.usbserial*
python3 tools/balance_telemetry.py /dev/cu.usbserial-120
```

日志写入 `logs/balance_YYYYMMDD_HHMMSS.csv`，该目录不会提交到 Git。

## 自动启动与零点

上电时：

1. 从 Flash 读取最新合法零点；没有记录或记录损坏时使用 `-2.18°`。
2. 动量轮和后轮命令保持为零。
3. IMU 至少收到 5 帧且数据新鲜。
4. 横滚角位于启动零点 ±3°、横滚角速度不超过 1°/s，并连续保持 2 秒。
5. 自动进入平衡控制。

动态零点只在控制已启动、后轮停止、姿态稳定且动量轮速度低于 15 tps 等条件同时满足 5 秒后学习。变化达到 0.002° 时保存，保存间隔不少于 15 秒。意外瞬时断电可能丢失最近一个保存周期内的变化；要实现真正的“掉电瞬间保存”还需要掉电检测与保持能量硬件。

## 当前限制

- 运行中的 IMU 超时和 CAN 心跳超时尚未形成完整的失效停机状态机。
- 动量轮的独立速度、加速度、加加速度和电流保护仍需结合机械强度实测后落地；现有遥测饱和标记不是完整保护。
- 3D 打印支架承载能力未知，不应按电机或 ODrive 厂家极限测试。
- Flash 写入已经在运行中验证；重新上电后读取最新记录仍需一次受控复位验证。
- PID 仍是实车调参阶段，应严格按“角速度环 → 角度环 → 轮速回正环”逐级调整。
- 视觉、激光雷达、建图和路径规划属于后续上位机阶段，尚未在本仓库实现。

## 许可证

仓库目前未声明开源许可证。在仓库所有者明确选择许可证前，默认保留全部权利；第三方 STM32 HAL/CMSIS 文件仍遵循各自文件中的许可声明。
