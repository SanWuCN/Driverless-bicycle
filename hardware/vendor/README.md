# ODrive 厂家资料说明

本目录保存技术公司提供的原始参考文件，目的是保留可追溯资料，不表示这些参数可以直接写入当前自行车。

## 文件

- `odrive_company_notes.txt`：厂家给出的命令记录，描述 14 极对、8192 CPR、node ID 24/16、250 kbit/s 和 120 A 等参数。
- `config_8192cpr_20PP_reference.json`：另一份 ODrive 配置备份，文件名和内容均指向 20 pole-pairs 方案。

## 当前实车采用的关键信息

| 参数 | Axis 0 动量轮 | Axis 1 后轮 |
| --- | ---: | ---: |
| 电机 | X8015 KV125 | X8015 KV125 |
| 极数 / 极对 | 28 极 / 14 极对 | 28 极 / 14 极对 |
| 编码器 CPR | 8192 | 8192 |
| CAN node ID | 24 | 16 |
| CAN 波特率 | 250 kbit/s | 250 kbit/s |
| 控制模式 | Velocity | Velocity |
| 输入模式 | Vel Ramp | Vel Ramp |
| 配置速度上限 | 50 turns/s | 50 turns/s |
| 配置斜坡率 | 50 turns/s² | 50 turns/s² |

## JSON 与实车的关键差异

参考 JSON 中至少包含：

- 电机极对：20，而实车电机 24N28P，应为 14 极对。
- CAN node ID：0/1，而 STM32 固件使用 24/16。
- Axis 0/1 速度上限：30/2 turns/s，而厂家文本和当前实车配置为 50/50。
- 电流上限：20 A，而厂家文本中的另一方案为 120 A。
- 速度斜坡率：1 turns/s²，而厂家文本中的另一方案为 50 turns/s²。

因此严禁执行：

```text
odrivetool restore-config hardware/vendor/config_8192cpr_20PP_reference.json
```

正确流程是：连接当前 ODrive，先 `backup-config`，逐项读取并比较，再以单项命令修改。涉及极对数、编码器、方向、电流和启动校准的修改必须架空车轮并准备断电。
