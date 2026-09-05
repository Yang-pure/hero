# Hero · RoboMaster 英雄机器人电控

基于 **STM32F405RG、STM32 HAL、FreeRTOS 和 C++** 的整车控制工程，包含麦克纳姆轮底盘、Yaw/Pitch 云台、三摩擦轮发射机构、拨弹与电推杆控制，以及 IMU、遥控器和上位机通信。

工程采用 Visual Studio / VisualGDB 开发，围绕“输入解码 → 目标生成 → 电机闭环 → CAN 输出”组织控制链路，用于英雄机器人整车集成、控制调试与功能扩展。

## 功能概览

- **底盘运动**：四轮麦克纳姆轮解算、轮速比例限幅、平移与旋转控制。
- **双轴云台**：Yaw 单圈位置控制、Pitch 连续多圈位置控制，采用位置—速度串级 PID。
- **Yaw 稳定控制**：IMU 世界角保持、底盘旋转指令前馈、角速度补偿与正反向分区摩擦补偿。
- **遥控交互**：RESET、FOLLOW、ROTATION、MANUAL_YAW、SHOOT 五种模式，切换时更新云台保持目标。
- **发射机构**：三摩擦轮、拨弹轮、弹丸检测与电推杆协同，支持单发锁存和连发状态机。
- **设备通信**：CAN 电机通信、UART/DMA 接收、IMU 解码，以及带 CRC16 的上位机收发协议。
- **扩展模块**：包含 DM 电机驱动及裁判系统、功率、超级电容相关代码。

> 参数对应当前机械结构与接线配置。移植时需重新确认电机方向、传动比和限位。上位机通信接口已实现，完整视觉自瞄与自动发射联动仍需进一步接入。

## 快速开始

### 开发环境

| 组件 | 工程配置 |
|---|---|
| IDE | Visual Studio + VisualGDB |
| 编译器 | ARM GCC 10.3.1 |
| 调试工具链 | GDB 10.2.90 |
| BSP | Sysprogs STM32 BSP 3.6 |
| 目标芯片 | STM32F405RG |
| 软件框架 | STM32 HAL、CMSIS、FreeRTOS |

配置文件见 [`stm32.xml`](STM32F405/stm32.xml)。HAL、CMSIS 和 FreeRTOS 通过外部 BSP 引用，构建前需要安装对应依赖。

### 获取与构建

```bash
git clone https://github.com/Yang-pure/hero.git
cd hero
```

1. 使用 Visual Studio 打开 `FreeRTOS.sln`。
2. 在 VisualGDB 项目属性中配置 ARM 工具链、STM32 BSP 和调试器。
3. 核对目标芯片、板卡接线、电机 ID 与供电条件。
4. 选择 Debug 配置并重新生成工程。
5. 确认构建成功后，通过已配置的调试器下载固件。

### 首次运行

1. 在无弹、机构运动区域隔离且可立即断电的条件下上电。
2. 使用 Live Watch 检查遥控通道、IMU 数据及各电机反馈。
3. 逐个核对电机方向，再进行低速、小角度操作。
4. 验证模式切换、摇杆归中保持、编码器回绕和 Pitch 限位。
5. 完成失联、故障停机及机械安全验证后，再进行带载整车测试。

**RESET 是软件控制模式，不是断电急停；进入 SHOOT 会立即启动摩擦轮。**

## 工程结构与控制链路

```text
FreeRTOS.sln                 Visual Studio / VisualGDB 解决方案
STM32F405/
  STM32F405.cpp              外设初始化、电机清单、入口
  taskslist.cpp             任务创建、周期调度、DM 初始化状态机
  RC.cpp/h                  遥控解析与模式映射
  control.cpp/h             底盘、云台、发射机构控制
  motor.cpp/h, pid.h        DJI 电机反馈、串级 PID、输出补偿
  can.cpp/h, usart.cpp/h    CAN、UART/DMA 与中断
  imu.cpp/h                IMU 解码
  xuc.cpp/h                上位机协议
  HTmotor.cpp/h            DM 电机协议
  label.cpp/h              公共参数、任务优先级
  judgement.*, Power_read.*, supercap.*  裁判/功率相关模块
tests/pid_reset_test.cpp    PID Reset 主机测试源码
```

主要数据流：遥控/IMU 接收 → `DecodeTask` → `ControlTask` 生成目标 → `MotorUpdateTask` 计算电机输出 → `CanTransimtTask` 发送。

| 任务 | 代码中的周期/方式 | 职责 |
|---|---|---|
| `DecodeTask` | 延时 5 tick；当前 tick 为 1 ms | 遥控、IMU、上位机解码 |
| `ControlTask` | 5 ms | 模式、目标、云台前馈、底盘/发射机构、上位机发送 |
| `MotorUpdateTask` | 名义 2 ms | 电机反馈与 PID 更新 |
| `CanTransimtTask` | 名义 1 ms，三相轮询 | DM、`0x1FF`、`0x200` 分组发送 |

周期是代码设定值，不是实测时序保证。CAN/UART 阻塞发送及任务实现会影响实际周期。

## 硬件映射

以下为软件配置，接线与机械方向需现场核对；ID 沿用工程中的标识，不将 GM6020 的工程 ID 等同于其硬件拨码编号。

| 功能 | 总线/接口 | 配置 |
|---|---|---|
| 底盘四轮 M3508 | CAN1 | 工程 ID5/6/7/8：左前、右前、右后、左后 |
| 三摩擦轮 M3508 | CAN2 | ID1/2/3 |
| Yaw GM6020 | CAN2 | ID6，`can2_motor[3]` |
| 拨弹 M3508 | CAN2 | ID5，`can2_motor[4]` |
| Pitch M3508 | CAN2 | ID4，`can2_motor[5]`，连续多圈位置 |
| 四路 DM 电机 | CAN1 | ID1～4，独立初始化流程；不是本工程的 Pitch 电机 |
| CH010 IMU | USART1 | 115200 |
| 遥控器 | UART4 | 100000 |
| 上位机 | USART3 | 460800 |
| 弹丸检测 | PA8 | 下拉输入，高电平有效 |
| 电推杆 | PC9 | 高电平推出、低电平收回 |

裁判、功率和超级电容模块的运行接入状态，以主程序初始化及任务调用为准。

## 遥控模式

表格按解码变量 `rc.s[0] / rc.s[1]` 描述，物理左右拨杆应通过 Live Watch 确认。

| s[0] | s[1] | 模式 | 行为 |
|---|---|---|---|
| MID | MID | RESET | 底盘目标归零、退出发射；云台保持目标，并非断电急停 |
| UP | MID | FOLLOW | 底盘平移，`-ch[2]` 控制底盘旋转；Yaw 世界角保持 |
| UP | UP | ROTATION | 底盘旋转目标 `speedz=1000`，Yaw 保持与补偿 |
| MID | UP | MANUAL_YAW | `ch[2]` 更新 Yaw 机械目标，`ch[3]` 控制 Pitch；归中后保持 |
| DOWN | DOWN | SHOOT | 底盘停止，三摩擦轮启动，`ch[2]` 单发、`ch[3]` 连发 |

其余组合未定义，除退出 SHOOT 的特殊处理外可能保留上一模式，不能当作安全挡位。

SHOOT 中摩擦轮目标为 `-4000/+4000/-4000`，单发使用上升沿锁存；触发区间为通道绝对值 `500～660`。推杆状态为 `WAIT → PUSHING → BACK`，推出保持 50 ms、回位等待 30 ms。**进入 SHOOT 即启动摩擦轮，首次测试必须无弹并隔离运动机构。**

## 参数配置与调试

| 配置内容 | 入口文件 |
|---|---|
| 电机型号、ID 与 PID | [`STM32F405.cpp`](STM32F405/STM32F405.cpp) |
| 底盘速度、Pitch 限位与遥控增益 | [`label.cpp`](STM32F405/label.cpp) |
| Yaw 前馈、角速度补偿与发射时序 | [`control.h`](STM32F405/control.h) |
| 输出限幅与分区补偿 | [`motor.cpp`](STM32F405/motor.cpp) |
| 任务优先级与周期 | [`label.h`](STM32F405/label.h)、[`taskslist.cpp`](STM32F405/taskslist.cpp) |

### 云台默认参数

| 项目 | 默认配置 |
|---|---|
| Yaw 速度环 | `Kp=99.66, Ti=0.47, Td=0` |
| Yaw 位置环 | `Kp=0.05, Ti=0, Td=0` |
| Pitch 速度环 | `Kp=20, Ti=0, Td=1.5` |
| Pitch 位置环 | `Kp=0.25, Ti=0, Td=0, alpha=0.1` |
| Yaw 指令前馈 / 角速度补偿 | `0.0075 / 0.4` |
| 前馈限幅 / 滤波系数 | `±300 / 0.25` |
| POS 最终目标速度限幅 | Yaw `±50`；Pitch `±500` |

`PID::Position(error, max_limit)` 的第二参数限制的是**累积误差**，不是最终输出；`Ti` 在该实现中是每次调用的积分增益，并非积分时间常数。改变任务周期会改变积分作用。

Yaw 单圈编码周期为 8192；Pitch 使用 `sum_angle`。`yaw_angle_plot = angle[now]/400` 仅供同图显示，**不是角度值**。电机转速与机构角速度的转换还需要传动比。

Pitch 的 `pitch_min=-90000`、`pitch_max=805000` 是相对上电 home 的编码偏移，不是角度。上电姿态、有效反馈与限位需要验证后才能带负载运行。

## 上位机协议

USART3 接收采用帧头 `0xA5`，当前 V2 长度 31 字节：

| 偏移 | 内容 |
|---|---|
| 0 | 帧头 |
| 1～4 / 5～8 | pitch / yaw，float |
| 9～12 / 13～16 | yaw_diff / pitch_diff，float |
| 17～20 | distance，float |
| 21 | fireadvice，取 bit0 |
| 22～24 | 保留 |
| 25～28 | v_y，float |
| 29～30 | CRC16，低字节在前 |

float 按 MCU 内存布局读取；pitch 和 pitch_diff 解码后由度转换为弧度。CRC 覆盖前 29 字节。旧 29 字节帧通过校验时仍接收，但强制 `v_y=0`。

连续 50 个有效且 `distance>0` 的新帧使 `track_flag` 置位；超过 100 ms 无有效通信后撤销锁定。发送帧头为 `0x5A`，包含颜色位、姿态、瞄准位置和 CRC。

视觉目标到云台控制及自动发射的完整联动属于后续扩展内容。

## 开发说明

- 调试时建议同时观察 `setangle`、`angle[now]`/`sum_angle`、`setspeed`、`curspeed` 与 `setcurrent`，分别判断目标生成、位置跟随与速度跟随。
- 改变控制周期、机械传动或电机型号后，应重新验证参数、方向与限幅。
- Live Watch 中临时修改的参数需要写回源码并重新构建，才能在后续上电时保持一致。
- `tests/pid_reset_test.cpp` 提供 PID Reset 测试源码，检查误差、滤波、微分历史复位与参数保留。主机运行需配置 HAL 头依赖；大小写敏感系统还需统一 `PID.h` 与 `pid.h` 的引用。
