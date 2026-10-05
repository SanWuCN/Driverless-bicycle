# 开发、编译、烧录与调试环境

## 1. 支持的工程入口

| 环境 | 入口 | 状态 |
| --- | --- | --- |
| macOS + VS Code | CMake Presets、Ninja、Arm GNU Toolchain | 当前主要开发环境，已验证编译和 ST-Link 烧录 |
| 命令行 | `cmake --preset mac-debug` | 已验证 |
| Windows + Keil MDK | `MDK-ARM/bike.uvprojx` | 保留历史工程；新增文件后需确认工程文件同步 |
| STM32CubeMX | `bike.ioc` | 历史配置，仅作参考，不能无审查覆盖源码 |

## 2. 已验证版本

本机验证环境：

- macOS，Apple Silicon。
- VS Code。
- CMake 4.2.1。
- Ninja 1.13.2。
- Arm GNU Toolchain 15.3.Rel1，GCC 15.3.1。
- Python 3.14.6（遥测脚本只用标准库）。
- stlink 1.9.0。
- ODrive Python 虚拟环境：`~/.local/share/odrive-venv`。
- 实车 ODrive 固件：0.5.6。

CMake 最低要求是 3.22；其他较新版本通常也可用，但应以 CI/本机实际构建为准。

## 3. macOS 安装

### Homebrew 工具

```bash
brew install cmake ninja stlink dfu-util stm32flash
```

### Arm GNU Toolchain

从 Arm 官方安装 `arm-none-eabi` 工具链。项目工具链文件会依次搜索 PATH 和以下路径：

```text
~/Library/ArmGNUToolchain/15.3.rel1/bin
/Applications/ArmGNUToolchain/15.3.rel1/arm-none-eabi/bin
```

检查：

```bash
~/Library/ArmGNUToolchain/15.3.rel1/bin/arm-none-eabi-gcc --version
```

### VS Code 推荐扩展

打开仓库时 `.vscode/extensions.json` 会推荐：

- CMake Tools
- C/C++
- Serial Monitor
- Cortex-Debug

## 4. 编译

### Debug 固件

```bash
cmake --preset mac-debug
cmake --build --preset mac-debug
```

完全重新配置时可删除 `build/debug` 后再配置，但不要删除仓库或其他工作目录。构建输出不会被 Git 跟踪。

### UART7 回环诊断固件

```bash
cmake --preset mac-uart-diagnostic
cmake --build --preset mac-uart-diagnostic
```

该构建定义 `BIKE_UART7_ECHO=1`，会把 UART7 收到的字节原样回显。它只用于确认物理连线，不是正式平衡固件。

### 产物与内存布局

| 文件 | 用途 |
| --- | --- |
| `bike.elf` | 符号、调试和反汇编 |
| `bike.hex` | Intel HEX 烧录映像 |
| `bike.bin` | 从 `0x08000000` 开始的裸二进制 |
| `bike.map` | 链接映射与尺寸分析 |

固件可用 Flash 只有 1792 KiB，因为最后 256 KiB 用于动态零点。修改链接脚本或 Keil scatter 文件时必须保持一致。

## 5. VS Code 任务

命令面板运行 `Tasks: Run Task`：

| 任务 | 说明 |
| --- | --- |
| `STM32: Configure` | 配置 Debug 构建 |
| `STM32: Build` | 默认编译任务 |
| `STM32: Probe ST-Link` | 只读探测，不复位 |
| `STM32: Flash ST-Link` | 编译并经 SWD 写入，会复位 |
| `STM32: Configure/Build UART7 Diagnostic` | 构建回环诊断固件 |
| `STM32: Test UART7 Echo` | 运行回环测试脚本 |
| `STM32: Monitor Balance Telemetry` | 监控并记录遥测 |
| `STM32: Flash ROM over USART3` | 仅在接到 ROM 支持的 USART3 且 BOOT0 正确时使用 |

任何包含 `Flash` 的任务都可能复位主控并让动量轮启动。每一次执行都必须重新进行机械安全确认。

## 6. ST-Link SWD

### 接线

```text
ST-Link SWDIO -> 主控 SWDIO / PA13
ST-Link SWCLK -> 主控 SWCLK / PA14
ST-Link 3V3   -> 主控 3.3 V 参考
ST-Link GND   -> 主控 GND
```

没有 NRST 时仍可正常访问 SWD。先做只读探测：

```bash
~/.local/bin/st-info --probe --freq=1000
```

确认安全且明确允许烧录后：

```bash
~/.local/bin/st-flash --freq 1000 --reset write build/debug/bike.bin 0x08000000
```

如果连接不稳定：

- 确认主控已供电且 ST-Link 读到目标电压。
- 必须共地，SWDIO/SWCLK 不要接反。
- 缩短杜邦线，并将 SWD 频率降到 100–400 kHz。
- 断开高噪声电机动力后再探测，但保持主控供电。
- 检查 PA13/PA14 没有被外设或外部电路占用。

不要在未获同意时用“反复尝试”代替故障诊断，因为每次 reset 都可能改变机械状态。

## 7. UART 与 ROM 下载限制

UART7 用于遥测，不是 STM32F427 的系统 Bootloader 下载接口。USB-UART 若要 ROM 烧录，需改接 USART1 或 USART3，并设置 BOOT0 进入 System Memory；当前 VS Code 任务预留 USART3 的 `stm32flash` 命令。

正式开发推荐：

- ST-Link：编译、烧录、断点调试。
- UART7：在线遥测、参数观察、后续命令接口。
- ODrive USB：独立查看和备份 ODrive 配置。

## 8. 在线遥测

查找串口：

```bash
ls /dev/cu.usbserial* /dev/cu.usbmodem* 2>/dev/null
```

运行：

```bash
python3 tools/balance_telemetry.py /dev/cu.usbserial-120
```

若只监控不保存：

```bash
python3 tools/balance_telemetry.py /dev/cu.usbserial-120 --no-log
```

若 3 秒没有数据：

- 检查 PE8/TX 是否接转接器 RX。
- 检查 GND 共地。
- 检查端口名是否变化或被 VS Code Serial Monitor 占用。
- 确认正式固件已经运行，且 UART7 为 115200 8N1。

## 9. ODrive 工具环境

当前本机使用独立虚拟环境，避免不同 ODrive API 版本冲突：

```bash
source ~/.local/share/odrive-venv/bin/activate
odrivetool
```

连接后应先备份，不要直接恢复厂家 JSON：

```bash
odrivetool backup-config odrive_before_change.json
```

ODrive 配置、固件版本和 Python 工具版本高度相关。每次修改前记录 `odrv0.fw_version_*`、硬件版本、轴错误、编码器错误和当前配置。

## 10. Keil 与源码编码

旧工程中的 `Src/task.c`、`Src/main.c`、`Src/packet.c` 含 GB18030/GBK 中文注释和混合换行；VS Code 已启用自动猜测编码。若编辑器误用 UTF-8 保存，中文注释可能乱码。

建议逐文件转换并单独验证，而不是一次性改全仓库：

```bash
iconv -f GB18030 -t UTF-8 Src/task.c > /tmp/task.c.utf8
```

新增文件统一使用 UTF-8。CMake/GCC 构建不依赖中文注释编码，但代码评审和 Keil 显示会受影响。

## 11. 调试边界

没有 NRST 不影响普通 SWD 烧录，但在线断点调试仍受机械系统限制：暂停 MCU 时 ODrive 可能保持最后命令。真实车辆上不应在闭环平衡运行中随意下断点。

更安全的方式：

- UART7 连续遥测。
- GPIO/LED 状态标记。
- 车架固定、轮子悬空时做短时断点。
- 在 ODrive 侧启用 watchdog，并让 MCU 周期喂狗。
- 调试器暂停前由软件发送零指令并使 ODrive 进入 Idle。

## 12. 提交前检查

```bash
cmake --preset mac-debug
cmake --build --preset mac-debug
git status --short
git diff --check
```

不要提交 `build/`、`logs/`、Keil 编译目录、串口日志、Python 缓存或本机调试器配置。
