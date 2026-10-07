# UART7 无线串口配置（ATK-MW579D / ATK-BLE02）

## 当前两只模块

| 位置 | 名称 | 模式 | MAC | UART | 自动连接 |
| --- | --- | --- | --- | --- | --- |
| 自行车端 | `BIKE-UART7` | 从机 `S` | `383B2636265B` | 115200 8N1 | ON |
| Mac 端 | `MAC-BIKE-LINK` | 主机 `M` | `383B263621D1` | 115200 8N1 | ON |

Mac 端已保存默认连接地址 `383B2636265B`。模块重新上电后会自动搜索并连接自行车端；
LINK 指示灯常亮且 `AT+LINK?` 返回 `PeerAddr:383B2636265B`、`+LINK:OnLine`
表示连接正确。

## 自行车端接线

| 无线模块 | STM32 | 说明 |
| --- | --- | --- |
| VCC | 稳定 3.3 V | 不得接 12 V/24 V/48 V；确认载板版本后才能考虑 5 V |
| GND | GND | 必须共地 |
| RXD | PE8 / UART7_TX | 交叉连接 |
| TXD | PE7 / UART7_RX | 交叉连接 |
| STA | 不接 | 可选连接状态输出 |
| WKUP | 不接 | 当前不使用低功耗唤醒 |

无线模块接入时，应断开原 USB-UART 在 PE7/PE8 上的信号线；两个 TX 同时驱动 PE7 会发生
电气冲突。ST-Link 的 SWDIO/SWCLK 与 UART7 无冲突，可以继续连接用于烧录。

## Mac 端使用

当前 Mac 端设备名为：

```text
/dev/cu.usbserial-140
```

设备名在重插或换 USB 口后可能改变：

```bash
ls /dev/cu.usbserial-*
```

启动 Web 上位机：

```bash
~/.local/share/odrive-venv/bin/python tools/control_station.py \
  --port /dev/cu.usbserial-140
```

只看遥测：

```bash
python3 tools/balance_telemetry.py /dev/cu.usbserial-140 --no-log
```

## 模块 AT 模式

UART 发送 `+++`，收到 `a` 后再发送 `a`，模块回复 `+ok` 即进入 AT 模式。AT 命令使用
CRLF 结尾；`AT+ENTM` 返回透传模式。常用只读检查：

```text
AT+NAME?
AT+MODE?
AT+MAC?
AT+UART?
AT+CONNADD?
AT+LINK?
AT+MAXPUT?
```

不要在自行车正在平衡或转向时进入模块 AT 模式；进入 AT 模式会中断上位机命令与遥测。

## 为什么固件必须使用按需遥测

BLE 模块 UART 侧虽然设置为 115200 baud，但无线侧理论吞吐不超过约 4 kB/s；资料建议将
持续转发控制在约 1.5 kB/s 以下并在上层做校验。旧固件以 20 Hz 同时发送 `B` 和 `S`
长文本行，实测出现截断、拼行且 8 秒内没有一条完整平衡数据。

新固件默认只发2 Hz基础状态，约92 byte/s；只有网页明确开启详细遥测或开始记录时，
才提升到约1.5 kB/s。每帧都有CRC16。命令保活不要求回执，避免周期确认包与详细遥测
争用无线带宽。两端保持标准`MAXPUT=OFF`即可；若以后启用`MAXPUT=ON`，两端必须同时
配置，不能只改一端。

## 故障判断

1. LINK 灯未常亮：检查两端供电、主从模式和默认 MAC。
2. 有连接但无数据：检查 PE8→RXD、PE7←TXD 和共地。
3. 能看到乱码/残缺文本：STM32 仍是旧的完整文本固件，需在明确许可后烧录紧凑遥测固件。
4. 上位机能显示数据但不能调参：检查 TXD→PE7 回传方向，并确认没有第二个串口程序占用设备。
5. 运行中偶发断链：记录 RSSI、CRC 错误和序号跳变；不要在未定位链路问题前进行无人运动测试。
