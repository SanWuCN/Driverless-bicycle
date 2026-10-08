# 平衡控制、动态零点与在线遥测

## 1. 控制目标

静止时，自行车横滚方向没有车轮转向产生的动态稳定作用，主要依靠动量轮加速产生反作用力矩。控制系统需要同时满足：

1. 横滚角快速回到平衡零点。
2. 横滚角速度得到足够阻尼，避免振荡。
3. 动量轮平均速度回到零附近，保留双向控制余量。
4. 零点能够缓慢适应 IMU 安装、轮胎压力和地面倾斜变化。
5. 任何学习动作都不能追随瞬时扰动或掩盖传感器/机械故障。

## 2. 当前控制周期

- 主控制中断：500 Hz（2 ms）。
- 角度环：500 Hz。
- 角速度环：500 Hz。
- 轮速回正环：当前代码每 20 次中断更新一次，即 25 Hz。
- UART7 遥测：20 Hz。
- Axis 0 CAN 命令/查询：500 Hz。
- Axis 1 CAN 命令：25 Hz。

## 3. 串级控制结构

当前代码流程：

```text
动量轮速度反馈 ── 轮速回正环 ──> 零点偏置 PWM_accel（±1.5°）
                                      │
动态零点 ─────────────────────────────┼──> 角度目标
                                      ▼
横滚角 ──────────────── 角度环 ──> 目标横滚角速度
                                      ▼
横滚角速度 ────────── 角速度环 ──> 动量轮加速度命令
                                      ▼ 限幅、速度包络并积分
                                  ODrive Axis 0 速度命令
```

当前参数位于 `Src/task.c::param_init()`：

| 环路 | Kp | Ki | Kd | 备注 |
| --- | ---: | ---: | ---: | --- |
| 角度环 | -5.0 | 0 | -1.5 | D 项直接使用陀螺仪角速度 |
| 角速度环 | -20.0 | 0 | -0.004 | 输出解释为 turns/s²；D 项为标准时间微分 |
| 动量轮速度回正环 | 0.06 | 0 | 0 | 输出取反，0.9/0.1 低通 |
| 航向转向 | 1.5 | 0 | 0.2 | 第一阶段默认关闭，完成舵机校准后使用 |

这些是编译默认值，不代表“最优参数”。旧速度目标结构下的第一轮实车比较得到 `RATE_KP=-16、RATE_KI=0、RATE_KD=-2`，但改为直接加速度控制后增益物理含义已经变化，必须重新从固定悬空工况验证，不能直接把旧结论视为最终值。调参应保留每次试验日志、测试姿态、限幅和版本号。

## 4. 自动启动

上电初始化：

- `scope_flag = 0`，平衡控制未使能。
- Axis 0 命令为 0。
- `run_flag = 0`，Axis 1 后轮命令为 0。
- 所有 PID 积分和历史状态清零。

自动启动条件必须连续满足 2 秒：

- 至少收到 5 帧 IMU 数据。
- 最后一帧 IMU 不超过 100 ms。
- 横滚角、横滚角速度是有限值。
- ODrive Axis 0 心跳和编码器反馈不超过 300 ms、无轴错误且处于闭环状态。
- 横滚角相对启动零点不超过 ±3°。
- 横滚角速度绝对值不超过 1°/s。

条件不满足时持续清零控制器状态；满足后自动设置 `scope_flag = 1`。

运行中若单纯出现 ODrive 心跳或编码器反馈超时，STM32 进入降级状态：冻结动态零点与轮速回正慢环，但继续依靠 IMU 运行角度和角速度快环，避免普通 CAN 接收抖动直接导致摔车。只有 IMU 硬超时/数据非法，或新鲜心跳明确报告轴错误、退出闭环时，才撤销平衡控制。若 Axis 0 无错误且处于 IDLE，只有在扶正窗口满足时才请求重新进入闭环，不会打断 ODrive 的校准状态。该逻辑已经编译，仍需受控实车故障注入验证。

角速度环直接生成动量轮加速度需求。STM32 正常将其限制在 ±80 tps²；当横滚误差绝对值至少为 0.60°、横滚角速度绝对值至少为 0.50°/s、车身仍在向误差增大的方向倾倒、控制输出方向正确且原始需求已经超过 80 tps² 时，临时开放至 ±120 tps²。一旦开始回正或任一门槛不满足，立即恢复 ±80 tps²。

限幅后的加速度按实际控制周期积分成 Axis 0 速度目标，并限制在 ±35 tps。实际轮速绝对值从 25 tps 增长到 35 tps 时，只对继续增大轮速的加速度做线性衰减；制动和反转方向保留完整控制权。ODrive 使用 Velocity Control + Vel Ramp，当前安全回退配置为 30 A电流上限、8 A裕量、50 tps速度上限和50 tps²斜坡。板载HSE实际为12 MHz，工程已在HAL配置、CMSIS系统文件、CMake和Keil中统一该值。

## 5. 动态零点估计

### 学习门控

以下条件同时满足并连续保持 5 秒才进入 `ZERO_LEARNING`：

- 输入均为有限值。
- 平衡控制已经启动。
- 后轮处于停止状态。
- 横滚角相对当前控制零点不超过 ±2°。
- 横滚角速度绝对值不超过 2°/s。
- 动量轮实际速度绝对值不超过 15 turns/s。
- 轮速回正环给出的零点偏置绝对值不超过 0.5°。

门控断开时 5 秒计时重新开始。速度命令的瞬时尖峰不参与门控，避免导数/离散指令尖峰让学习永远无法进入；真正的安全限幅仍应在执行器命令生成处实现。

### 学习律

1. 对动量轮实际速度做时间常数 3 s 的一阶低通。
2. 对 ±0.5 tps 内速度设置死区。
3. 零点变化率为 `-0.001 × 死区外轮速`。
4. 零点最大变化率为 ±0.003°/s。
5. 候选零点限制在厂家零点 `-2.18° ± 1.5°`。
6. 候选零点当前直接用于控制（`DYNAMIC_ZERO_PILOT_APPLY = 1`）。

理想现象：动量轮平均速度接近 0；轮速在零附近正负交替而不长期偏向一侧；零点不持续单向爬升；正常工作不触及 ±1.5° 边界。

## 6. Flash 持久化

### 存储结构

- 扇区 22 和 23 交替作为追加日志。
- 每条记录 32 字节。
- 字段包括 magic、版本、序号、float 位表示、CRC32、序号反码、零点反码和最后写入的提交标记。
- 启动扫描两个扇区，只接受所有校验通过且处于允许范围内的记录。
- 序号按无符号回绕规则比较，选择最新记录。
- 当前扇区满后先擦除另一个扇区，再写入新记录；旧扇区在新记录提交前仍保留有效数据。

### 保存条件

- 动态零点正在学习。
- 候选零点处于允许范围。
- 与最后保存值相差至少 0.002°。
- 两次保存尝试至少间隔 15 秒。

启动时若没有合法记录，使用厂家零点 `-2.18°`。Flash 写入错误会置位遥测错误标志但不会使用非法记录。

### 掉电语义

该方案是周期性检查点，不是掉电瞬间捕获。突然断电时，最近不足 15 秒的变化可能尚未保存。若必须保存“断电那一刻”的值，需要加入电源掉电检测、足够的保持电容/备用电源以及专门的紧急写入流程。

## 7. UART7 遥测协议

### 默认基础状态模式

正式固件复位后进入`BASIC`，每500 ms发送一条46字节基础状态帧（约92 byte/s）。它只含
平衡状态标志、舵机模式/目标/输出/PWM以及后轮模式/速度/编码器里程/演示阶段，不含横滚、
角速度、航向等IMU数据。正常运动时保持此模式，给控制命令和回执留出充足无线带宽。

### 按需详细模式

收到`TELEM,LIVE`或兼容命令`TELEM,COMPACT`后启用详细紧凑协议：

- 平衡帧：65 字节，20 Hz，包含横滚角/角速度、动量轮速度、速度与加速度命令、动态零点、PID 分项、电流命令和全部状态标志。
- 转向帧：34字节，约3.3 Hz，包含航向、航向角速度、目标航向、舵机目标/平滑输出、PWM和转向状态标志。
- 每帧以 `A5 5A` 开头，包含协议版本、帧类型、长度、序号/时间戳，并以 CRC16-CCITT 结束。
- 加上2 Hz基础摘要后总持续负载约`65×20 + 34×3.3 + 46×2 ≈ 1505 byte/s`，
  不计偶发命令回执；因此不观察曲线时应切回BASIC。
- 上位机同时支持紧凑二进制帧和旧文本帧；PID/转向命令仍使用原有带序号和 CRC16 的 ASCII 命令，因此在线调参接口不变。

采用默认BASIC、按需LIVE是因为BLE低功耗串口模块并不等同于115200 baud的有线UART：UART
一侧可以接收 115200 baud，但无线侧持续吞吐远低于此速率。原来的两行长文本遥测约 8–10
kB/s，会出现截断和连包，不能用于安全在线调参。

### 有线完整文本模式

上位机可按需切换：

```text
@P,<seq>,TELEM,BASIC,<crc16>
@P,<seq>,TELEM,LIVE,<crc16>
@P,<seq>,TELEM,FULL,<crc16>
@P,<seq>,TELEM,COMPACT,<crc16>
```

模式只存于RAM，复位后回到BASIC。`FULL`只适合直连USB-UART，不应通过BLE使用。

完整文本模式先输出表头，随后每 50 ms 输出一行：

```text
#B,time_ms,seq,roll_deg,roll_rate_dps,wheel_tps,vel_cmd_tps,accel_cmd_tps2,zero_base_deg,zero_control_deg,zero_candidate_deg,zero_persisted_deg,zero_persist_seq,zero_rate_dps,wheel_bias_deg,rate_target_dps,rate_error_dps,rate_p_tps,rate_i_tps,rate_d_tps,vel_raw_tps,vel_slew_error_tps,accel_raw_tps2,accel_limit_error_tps2,current_cmd_a,torque_cmd_nm,rate_kp,rate_ki,rate_kd,angle_kp,angle_ki,angle_kd,wheel_kp,wheel_ki,flags
```

字段：

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `time_ms` | ms | MCU 上电时间 |
| `seq` | — | 500 Hz 快照序号，不是 20 Hz 输出序号 |
| `roll_deg` | ° | CH100 横滚角 |
| `roll_rate_dps` | °/s | 滤波后的横滚角速度 |
| `wheel_tps` | turns/s | 动量轮实际速度 |
| `vel_cmd_tps` | turns/s | 发给 Axis 0 的速度命令 |
| `accel_cmd_tps2` | turns/s² | 速度包络和限幅后的加速度需求 |
| `zero_base_deg` | ° | 本次启动载入的基础零点 |
| `zero_control_deg` | ° | 控制当前实际使用的零点 |
| `zero_candidate_deg` | ° | 在线估计候选零点 |
| `zero_persisted_deg` | ° | 最近成功写入 Flash 的零点 |
| `zero_persist_seq` | — | 持久化记录序号 |
| `zero_rate_dps` | °/s | 当前零点自适应速率 |
| `wheel_bias_deg` | ° | 轮速回正环给角度目标的偏置 |
| `rate_target_dps` | °/s | 角度环给角速度内环的目标 |
| `rate_error_dps` | °/s | 目标角速度减实际角速度 |
| `rate_p_tps` | turns/s² | 角速度环 P 项加速度贡献；字段名为兼容旧日志保留 |
| `rate_i_tps` | turns/s² | 角速度环 I 项加速度贡献；字段名为兼容旧日志保留 |
| `rate_d_tps` | turns/s² | 角速度环 D 项加速度贡献；字段名为兼容旧日志保留 |
| `vel_raw_tps` | turns/s | 本周期积分后、速度限幅前的目标 |
| `vel_slew_error_tps` | turns/s | 原始速度目标与限幅后速度目标的差 |
| `accel_raw_tps2` | turns/s² | 角速度环给出的原始加速度需求 |
| `accel_limit_error_tps2` | turns/s² | 原始加速度减限幅后加速度 |
| `current_cmd_a` | A | 为兼容直接力矩实验日志保留；速度版本恒为 0 |
| `torque_cmd_nm` | N·m | 为兼容直接力矩实验日志保留；速度版本恒为 0 |
| `rate_kp/ki/kd` | — | 当前 RAM 中的角速度环参数 |
| `angle_kp/ki/kd` | — | 当前 RAM 中的角度环参数 |
| `wheel_kp/ki` | — | 当前 RAM 中的轮速回正环参数 |
| `flags` | bitmask | 状态与饱和标志 |

标志位：

| 位 | 十六进制 | 名称 | 含义 |
| ---: | ---: | --- | --- |
| 0 | `0x0001` | `ZERO_GATE` | 动态零点门控条件当前满足 |
| 1 | `0x0002` | `ZERO_LEARNING` | 已保持 5 秒，正在学习 |
| 2 | `0x0004` | `BIAS_SAT` | 轮速回正偏置达到 ±1.5° |
| 3 | `0x0008` | `VELOCITY_SAT` | Axis 0速度命令达到STM32的35 tps软边界 |
| 4 | `0x0010` | `ACCEL_SAT` | 原始加速度需求超过当前生效的 80 或 120 tps² 限制 |
| 5 | `0x0020` | `ZERO_APPLIED` | 动态零点已应用到控制 |
| 6 | `0x0040` | `ZERO_LIMIT` | 候选零点达到 ±1.5° 边界 |
| 7 | `0x0080` | `CONTROL_ARMED` | 平衡控制已使能 |
| 8 | `0x0100` | `ARM_WINDOW` | 自动启动稳定计时中 |
| 9 | `0x0200` | `IMU_READY` | 启动判定中的 IMU 数据新鲜 |
| 10 | `0x0400` | `ZERO_PERSIST_VALID` | 启动后存在合法持久化记录 |
| 11 | `0x0800` | `ZERO_PERSIST_SAVED` | 本次上电已经成功保存过 |
| 12 | `0x1000` | `ZERO_PERSIST_ERROR` | Flash 保存发生错误 |
| 13 | `0x2000` | `RATE_SLEW` | 旧速度目标控制固件的斜率限制标志；直接加速度版本不再置位 |
| 14 | `0x4000` | `ODRIVE_READY` | Axis 0 反馈新鲜、无错误且处于闭环 |
| 15 | `0x8000` | `ODRIVE_TIMEOUT` | Axis 0 心跳或编码器反馈超时 |
| 16 | `0x10000` | `ODRIVE_FAULT` | Axis 0 心跳报告非零轴错误 |
| 17 | `0x20000` | `UART7_RX_SEEN` | UART7 已收到过合法或待解析数据 |
| 18 | `0x40000` | `UART7_RX_OVERFLOW` | UART7 接收环形队列发生过溢出 |
| 19 | `0x80000` | `RATE_TARGET_SAT` | 角度环目标横滚角速度达到 ±5°/s |
| 20 | `0x100000` | `FALL_DISARM` | 横滚超过 ±8° 后已撤销平衡并将 Axis 0 置为 IDLE，等待重新扶正 |
| 21 | `0x200000` | `ZERO_STEER_BLOCK` | 转向不在中位稳定窗口，动态零点学习已暂停 |
| 22 | `0x400000` | `ACCEL_BOOST_120` | 当前满足向外倾倒救车条件，加速度上限临时提升至 ±120 tps² |
| 23 | `0x800000` | `SPEED_ENVELOPE` | 实际轮速进入25–35 tps包络，继续向外加速被渐进削弱 |
| 24 | `0x1000000` | `TORQUE_CONTROL` | 仅用于识别直接力矩实验固件；当前速度版本不置位 |

正常闭环阶段不应持续出现 `ACCEL_SAT` 或 `VELOCITY_SAT`。调参时应结合 P/I/D 加速度分项、`accel_raw_tps2` 与 `accel_limit_error_tps2` 判断控制器是否长期要求超出机构能力。

## 8. 监控工具

```bash
python3 tools/balance_telemetry.py /dev/cu.usbserial-120
python3 tools/balance_telemetry.py /dev/cu.usbserial-120 --no-log
python3 tools/balance_telemetry.py /dev/cu.usbserial-120 --duration 60
python3 tools/balance_telemetry.py /dev/cu.usbserial-120 --output logs/test.csv
```

工具只使用 Python 标准库和 macOS `termios`，不依赖 pyserial。它会自动识别无线紧凑帧和
有线文本帧，验证 CRC、显示实时值、估算丢帧并保存 CSV。

同步采集 STM32 控制遥测与 ODrive USB 电流/状态：

```bash
~/.local/share/odrive-venv/bin/python \
  tools/synchronized_balance_odrive_telemetry.py \
  /dev/cu.usbserial-120 --duration 90 --odrive-rate 20 \
  --output logs/synchronized-test.csv
```

记录器用 Mac 单调时钟对齐 UART7 与 ODrive USB，在同一 CSV 中保存 `Iq_setpoint`、`Iq_measured`、母线电压/电流、ODrive 轮速、轴状态及各类错误。`odrive_age_ms` 是每个 STM32 样本与最近 ODrive 样本的时差；对齐质量应与试验结果一起记录。

### 在线调参

新版固件允许通过 UART7 在 RAM 中读取、修改和回退 PID 参数：

```bash
python3 tools/balance_telemetry.py /dev/cu.usbserial-120 --get-params --duration 5 --no-log
python3 tools/balance_telemetry.py /dev/cu.usbserial-120 --set-param RATE_KP -8 --duration 30 --output logs/rate-kp-minus8.csv
python3 tools/balance_telemetry.py /dev/cu.usbserial-120 --revert-params --duration 5 --no-log
```

命令使用 CRC16、序号、参数白名单和固件侧范围检查；后轮运行时拒绝修改。每次修改只重置对应控制环的积分/历史状态，力矩命令仍经过加速度、电流和轮速包络限制。在线值不写 Flash，STM32 复位后恢复编译默认值；只有同一组参数通过固定车架、扰动、长时间和故障注入验证后，才允许另行固化。

允许参数：`RATE_KP/KI/KD`、`ANGLE_KP/KI/KD`、`WHEEL_KP/KI`。遥测每一行都携带当前参数，使 CSV 能独立追溯试验配置。

### 转向校准与航向 PID（第一阶段）

转向舵机内部已经有位置闭环；STM32 当前没有独立舵角传感器，因此外部闭环控制量是
CH100 的航向角，而不是舵机轴角：

```text
heading_error = wrap(target_heading - yaw)
servo_offset  = Kp * heading_error + Ki * integral - Kd * yaw_rate
```

默认参数为 `STEER_KP=1.5`、`STEER_KI=0`、`STEER_KD=0.2`，只存于 RAM。
控制分成三个用途明确的模式：`CAL` 仅用于停驶标定，`ANGLE` 是静止和行驶均可使用的
正式舵角/PWM 偏移模式，`HOLD` 是车辆已经运动后的航向闭环。`ANGLE` 限制为
`±150 µs`，非零命令必须位于 `±50…±150 µs`，并按 `5 µs` 步进；`-50…+50 µs`
之间视为机械死区，只允许精确的 `0` 回中。其中负偏移/较小脉宽为实车右转，正偏移/较大脉宽为实车左转。
网页不直接显示脉宽，而按当前标定 `1° = 10 µs` 显示为 `±15°` 指令角、`0.5°`
步进和 `±5°` 非零死区；这不是未经测量就宣称的前轮真实机械角，后续可用实测值更新
上位机中的单一换算系数，不改变固件协议。
该模式不依赖后轮速度，但要求平衡已启动、IMU 有效且未触发倒车保护；
`HOLD` 仍要求后轮反馈有效且速度不低于 `0.20 tps`，防止静止时航向不变化造成 PID
持续顶住限幅。

所有正常转向使用 `20 µs/s` 最大速度、`40 µs/s²` 最大加速度的梯形轨迹，
中位到 `±150 µs` 约需 8 秒。舵角模式
从一侧回中时先反向越过中位 `40 µs`、停留 `300 ms` 再回到中位，以消除连杆回差：
`1620 → 1480 → 1520 µs`，另一方向对称为 `1420 → 1560 → 1520 µs`。失衡、
IMU 无效等安全事件会取消回差补偿并直接慢速回中。

转向与平衡不是两个互不相关的控制器。舵角轨迹同步向平衡角度目标加入实测辨识的
不对称前馈：实车右转（负偏移）使用 `-0.0039°/µs`，左转（正偏移）使用
`-0.0034°/µs`。动量轮速度回正环在此基础上使用 `Kp=0.06`、`Ki=0.003` 的 PI
修正残余误差；积分使用 `tps·s` 单位并带 `±1.5°` 输出抗饱和。这样前馈负责转向
开始时的即时载荷补偿，PI 负责轮胎气压、地面和结构回差变化，目标是在保持舵角时
仍让动量轮速度收敛回 0。积分仅在转向及回中后的稳定门控期间累积；动态零点学习
恢复后以约 4 秒时间常数平滑衰减，把长期 IMU/轮胎漂移重新交给可持久化的动态零点。
该临时平衡偏置不进入动态零点学习，也不写入 Flash。
当前补偿属于静态舵角前馈加轮速 PI 自校正，静止和行驶都生效；尚未加入依据车速、
前进/后退方向和偏航角速度计算的动态倾角前馈。由于当前最高仅 `0.10 m/s`，理论转弯
向心倾角很小，应先用实车转向遥测确认符号和机械载荷，再逐步加入限幅的
`atan(v·yaw_rate/g)` 前馈，不能在方向未验证时直接启用。
横滚误差从 5° 到 8° 时转向量连续衰减到零，8°失衡保护、IMU失效、
后轮反馈失效、后轮停止或上位机命令超时都会让舵机限速回中。
任何转向模式、舵机尚未完全回中或回中后 5 秒稳定等待期间，动态零点学习与
Flash 保存都会暂停；随后还需重新通过原有 5 秒静稳门控才恢复学习，避免把转向
侧向载荷和连杆回差误学成 IMU 零点漂移。`ZERO_STEER_BLOCK` 表示该门控正在生效。

详细模式另外以约3.3 Hz输出紧凑转向帧；`FULL`有线诊断模式输出`S`文本行，包括航向、
偏航角速度、后轮速度、目标/误差、PID参数、原始/实际舵机偏移、PWM脉宽和安全门控标志。
Mac端使用：

```bash
# 只观察
python3 tools/steering_console.py /dev/cu.usbserial-120 --no-log

# 前轮悬空后，小步验证中位与左右方向；程序结束时自动发送 DISABLE
python3 tools/steering_console.py /dev/cu.usbserial-120 \
  --cal-offset 10 --duration 10

# 正式舵角模式：静止或行驶均可，范围 -150...+150 us；负值右转
python3 tools/steering_console.py /dev/cu.usbserial-120 \
  --angle-offset -150 --duration 15

# 后续低速行驶阶段捕获并保持启动时的航向
python3 tools/steering_console.py /dev/cu.usbserial-120 \
  --capture --duration 30
```

现有 `balance_telemetry.py` 会忽略 `S` 行，因此不会破坏原平衡日志格式；转向日志单独
写入 `logs/steering_*.csv`。

UART7 接收同时保留中断路径和主循环轮询兜底。遥测标志 `UART7_RX_SEEN` 表示本次上电至少收到过一个 PE7 字节，`UART7_RX_OVERFLOW` 表示 256 字节接收环形缓冲曾溢出。若 Mac 能持续收到 PE8 遥测、主机写串口成功，但反复发送 `GET` 后仍无 `UART7_RX_SEEN`，应优先检查 `USB-UART TXD -> PE7/UART7_RX`、共地、3.3 V TTL 电平和转接器本地回环，而不是继续修改协议或 PID。

## 9. 偶发停止加固

针对运行数分钟后控制、CAN 和遥测同时停止的现象，已修补 IMU 数据解析中的越界风险：验证总包长度、每个数据项长度和网关设备数量，修正数组索引，并只让同时含有效角速度和欧拉角的数据帧刷新 IMU 新鲜状态。UART8 溢出会重置解析器，UART7 发送也改为有限等待，避免外设异常把主循环永久锁死。修正版需要烧录后进行至少 30 分钟悬空耐久验证。

## 10. 正确调参顺序

不要同时调整三个平衡环。

### 第一步：方向与保护

- 固定车架、车轮悬空。
- 验证 IMU 横滚角和角速度正负方向。
- 验证动量轮施力能把车体推回零点，而不是加剧倾倒。
- 设置保守的轮速、加速度、加加速度和电流上限。
- 完成 IMU/CAN 超时停机和急停。

### 第二步：角速度环

- 暂时让角度环只给小幅目标，关闭/弱化轮速回正。
- 先调 Kp 得到快速、不过度振荡的角速度响应。
- D 使用标准时间尺度 `Kd * d(error)/dt`；500 Hz 下旧参数 `-2` 的等效标准参数为 `-0.004`。
- I 使用 `Ki * integral(error * dt)`，有积分限幅和输出饱和条件抗饱和；Ki 默认保持 0，只有角速度误差均值在置信区间外持续偏离 0 时才调。
- Kp 决定对角速度误差的即时加速度响应；Kd 主要改变快速变化时的阻尼和噪声放大；Ki 只用于消除持续偏置，不能用来掩盖零点误差或轮速回正环问题。
- 不按单组结果即时判断。先完成预先定义的细网格批量采样，再一次性比较横滚、角速度、轮速、加速度和饱和指标。

细网格批量采集示例（候选按固定随机种子打乱，每 5 组插入一次基准组以识别温漂/零点漂移）：

```bash
python3 tools/pid_batch_sweep.py /dev/cu.usbserial-120 \
  --parameter RATE_KP --start -18 --stop -22 --step 0.1 \
  --duration 30 --settle 5 --repeat 2 \
  --anchor -20 --anchor-every 5 --restore -20 \
  --output logs/rate-kp-fine.csv

python3 tools/analyze_pid_sweep.py logs/rate-kp-fine.csv
```

批处理会在横滚角、角速度、轮速、ODrive 超时/故障或遥测中断越界时终止，并在正常结束或异常退出时尝试恢复基准参数。分析器只接受状态为 `COMPLETE` 的整批试验，并给出无故障、无饱和候选的 Pareto 前沿，不用人为权重把四个目标强行混成一个分数。

### 第三步：角度环

- 在角速度内环可靠后调角度 Kp。
- 逐步增加阻尼，观察横滚超调、稳定时间和动量轮加速度峰值。
- 每次只改一个参数，保留同一测试动作和遥测日志。

### 第四步：轮速回正环

- 确认车体能稳定后再让轮速慢慢回零。
- 该环应明显慢于姿态环，不能为了快速回零而把车体拖倒。
- 观察长期平均轮速、零点学习方向、轮速正负交替和饱和次数。

### 第五步：动态零点

- 先锁定 PID，再评估零点学习；不要让自适应掩盖不稳定 PID。
- 对比 `zero_candidate_deg`、`zero_persisted_deg` 和平均轮速。
- 在不同胎压、地面和 IMU 安装状态下验证，但不要用动态零点容忍松动的 IMU。
