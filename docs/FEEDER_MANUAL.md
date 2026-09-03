# 手动拨弹轮 / FIRE 模式

## 本次范围

双拨杆 DOWN（`rc.rc.s[0] == 2 && rc.rc.s[1] == 2`）进入 `CONTROL::FIRE`（数值 7），底盘三轴目标速度强制归零。右摇杆上下 `rc.rc.ch[1]` 比例控制拨弹电机正反转。

拨弹轮为 `can2_motor[4]`，CAN2 / ID5（反馈 `0x205`），M3508，保留用户设置的 SPD 模式。退出 FIRE 时拨弹目标和电流命令归零。其它模式、Yaw/Pitch 算法、串口初始化不在本次修改范围。双 DOWN 原来的右摇杆横向摩擦轮开关保留。

这是手动速度模式调试功能；应先完成空载验证再进行受控带载测试，不代表当前参数已解决带载退弹卡滞。没有加入定量拨弹、自动射击、视觉联动、堵转重试、电推杆或微动开关逻辑。手动拨弹不依赖 `openRub`，所以可以关闭摩擦轮调试。

## 独立函数与合并接入点

| 文件 | 作用 / 合并方式 |
| --- | --- |
| `STM32F405/feeder_command.h` | 独立输入策略：比例映射、回中解锁、超时；无 HAL 依赖 |
| `STM32F405/feeder.h/.cpp` | 独立电机适配：`Init`、`UpdateCommand`、`OnFeedback`、`Update`、`Stop` |
| `RC.h/.cpp` | 增加 `UpdateFireMode()`；`OnRC()` 开头调用；真正收到新 RC 帧后调用 `feeder.UpdateCommand(...)` |
| `control.h/.cpp` | 枚举末尾追加 FIRE；底盘 FIRE 目标归零；删除旧 SHOOTER 内固定 2160 rpm 拨弹写入 |
| `motor.cpp` | SPD 分支仅 `function == supply` 转交 `feeder.Update(*this)`，其余电机保持原速度环 |
| `can.cpp` | CAN2 ID5 标准 8 字节数据反馈后调用 `feeder.OnFeedback(HAL_GetTick())` |
| `pid.h` | 追加 `Reset()`，清历史误差和滤波状态，不修改 PID 参数 |
| `STM32F405.cpp` | 供弹轮独立速度PID为20/0.1/1.5，微分滤波系数0；保留SPD和用户串口修改 |
| `.vcxproj` / `.filters` | 注册三个 feeder 源/头文件 |

合并时优先复制独立模块，再逐个合入小接入点，不要整份覆盖队友的 `motor.cpp`、`RC.cpp`、`control.cpp`。以后自动拨弹也应通过统一模块给目标，不要重新增加第二处 `supply_motor[0]->setspeed` 写入。

## 调参入口与当前参数

本次按用户要求固化 `P=20、I=0.1`，D保留 `1.5`；没有修改控制流程。

- **PID**：`STM32F405/STM32F405.cpp` 的 `Motor(M3508,SPD,supply, ID5, PID(20.f, 0.1f, 1.5f,0.f))`。参数顺序为P、I、D、微分滤波系数，对应成员 `m_Kp/m_Ti/m_Td/m_alpha`。
- **速度、方向、死区、超时**：`STM32F405/feeder_command.h` 的 `FeederCommand` 初始字段。
- **供弹电流上限**：`STM32F405/feeder.h` 的 `max_current` 和 `CurrentLimit()`；不要为供弹调参改动所有M3508共用的 `Motor::getmax()`。
- **积分累计误差上限**：`STM32F405/feeder.cpp` 中 `Position(error, 30000.0f)` 的第二参数，和电流上限不是同一概念。
- **过温阈值**：`STM32F405/feeder.cpp` 的 `motor.temperature > 70`；这是保护条件，不是常规调参项。

| 变量 | 初值 | 含义 |
| --- | --- | --- |
| `feeder.command.max_speed` | 300 | 电机反馈 RPM 上限，不是减速后拨弹盘 RPM |
| `feeder.command.deadband` | 20 | 摇杆回中死区 |
| `feeder.command.direction` | -1 | 已按实体机构确认：上推负转速送料，下推正转速退弹 |
| `feeder.max_current` | 30000 | 请求限幅；实际发送不得超过 C620 的 ±16384，不是毫安值 |
| `feeder.effective_current_limit` | 运行时计算 | min(请求限幅，电机maxcurrent，16384)，正常为16384 |
| `feeder.command.rc_timeout_ms` | 100 | RC 新帧最大允许间隔 |
| `feeder.command.feedback_timeout_ms` | 100 | 拨弹电机 CAN 反馈最大允许间隔 |
| `can2_motor[4].pid[0]` | Kp=20，Ki=0.1，Kd=1.5，alpha=0 | 供弹轮独立实例，不与底盘共享PID历史或调参 |

最大上推 +660 对应 -300 RPM（送料），最大下推 -660 对应 +300 RPM（退弹）；半推 ±330 对应约 ∓145 RPM。按用户观察视角，逆时针为上弹、顺时针为退弹；电机命令的正负号不等同于操作意义上的正反转。模块速度硬上限3800 RPM；当前参数不代表已经适合所有供弹负载。

设置30000不是强制输出30000。输出由PID决定，最后限制在有效上限内。C620协议允许命令范围为±16384：[官方手册，CAN通信协议](https://cdn-hz.robomaster.com/robomasters/public/document/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf)。不能把GM6020的30000命令范围套给C620。

## 使能和停止行为

### 完整执行流程

1. **接收遥控输入**：`RC::Decode()` 仅在队列实际取到新帧并通过已有帧检查后，解析双拨杆和右摇杆竖直轴，将 `rc.s[0]/rc.s[1]/rc.ch[1]` 交给 `FEEDER::UpdateCommand()`。没有新帧就不刷新RC时间戳。
2. **选择模式**：`RC::OnRC()` 先执行 `UpdateFireMode()`；双DOWN设置FIRE并把底盘三轴目标清零，随后返回，避免继续把右摇杆当作底盘指令。`CHASSIS::Update()` 在FIRE内也保持三轴目标为0。右摇杆横向的 `abs(ch[0]) > 330` 摩擦轮开关保留，不影响竖直轴供弹使能。退出双DOWN后，若还处于FIRE则先回RESET，再由原模式逻辑处理。
3. **记录电机反馈**：CAN2的ID5标准8字节数据反馈更新 `last_feedback_ms`；`Motor::Ontimer()` 从对应反馈槽解析转速、电流和温度。正反转使用同一反馈解析，不翻转反馈符号。
4. **检查运行条件**：电机更新任务中，`Motor::Ontimer()` 的 `SPD && function==supply` 路径调用 `FEEDER::Update()`。检查FIRE、双DOWN、合法输入、新RC/新CAN反馈和回中解锁，再检查温度、模式与有效电流上限；任一条件不满足就清目标、电流及PID历史。
5. **生成目标转速**：死区内为0；死区外 `rpm = (abs(axis)-deadband) * min(max_speed,3800) / (660-deadband)`，整数除法截断，然后乘轴符号及 `direction`。这只是速度目标，没有自动退堵动作、速度斜坡或正反转独立PID。
6. **计算速度PID**：误差 `e = setspeed - curspeed`；使用供弹电机自身 `pid[0]` 计算限幅前输出，保存到 `feeder.pid_output`。FIRE内摇杆回中仍计算零速闭环，直接换向也保留PID历史。
7. **限幅并发送**：`min(feeder.max_current, motor.maxcurrent, 16384)` 为有效电流指令上限；对PID输出做正负对称限幅后写 `setcurrent`。`Motor::Ontimer()` 再按电机上限生成 `current` 并写CAN发送缓冲，ID5在 `0x1FF` 帧第一组两字节。`torque_current` 则是电机反馈值，不是本机输出指令。

当前 `MotorUpdateTask` 配置的更新间隔为2ms；实际调度可能存在抖动。PID内部没有显式乘除采样周期，修改任务周期会改变I/D的实际效果。

### PID与两种限幅的区别

对应 `pid.h::Position()`，记累计误差为S、前次误差为e_prev、上次微分项为D_prev：

```text
e = 目标转速 - 反馈转速
S = clamp(S + e, -30000, +30000)
D = Kd × (1-alpha) × (e-e_prev) + alpha × D_prev
pid_output = Kp × e + Ki × S + D
setcurrent = clamp(pid_output, -effective_current_limit, +effective_current_limit)
```

这里的 `m_Ti` 实际直接充当Ki系数；`m_error[0]` 保存S。界面可见的PID成员 `max_limit=1000` 没有被这个函数调用使用：函数同名参数接收的是供弹模块传入的30000。当前 `Ki=0.1` 因而只允许积分项最多贡献±3000，并不意味着PID总输出也限到3000或9000。

例如目标300、反馈持续0且微分项消退后，P20贡献6000，积分同方向积满时再贡献3000，合计9000；最终发送上限虽然为16384，也不会自动把9000放大到16384。这个计算是给定条件下的示例，不是对所有实时输出的预测。

### 回中、换向和保护

1. 进入 FIRE、RC 失联恢复、或 CAN 反馈超时恢复后，须在双 DOWN 下先把右摇杆回中，才允许再次驱动；一直推住摇杆不会自动重启。
2. 没有拨弹电机反馈、RC/反馈超时、输入非法、温度大于 70 摄氏度、配置无效或 PID 输出非有限数时，拨弹输出归零。
3. FIRE 内回中：目标速度为 0，保留速度环制动，`setcurrent` 不一定立即为 0。
4. 非 FIRE 或失联：目标速度、电流命令和 PID 历史清零。这是撤销驱动力，不保证机构惯性立即消失。
5. `Stop()` 是电机更新任务内部的输出清零助手；若未来新增外部停机入口，应同时解除 command 使能，不能只调用一次 `Stop()` 当作锁存停机。

当前唯一拨弹电机固定为 CAN2 ID5。此模块不是整车急停系统，RC/CAN 超时保护只作用于本次拨弹功能。不得依靠 Live Watch 暂停图形实现急停，也不要在电机有输出时用断点停住控制任务。

## 空载验证步骤

测试前不装弹、断开摩擦轮动力、固定底盘并确认拨弹机构活动范围无人接触，准备能直接切断动力的停止方式。

1. 双 MID 启动：拨弹 `setspeed/setcurrent/current` 应为 0。
2. 双 DOWN、右摇杆回中：`ctrl.mode == 7`；收到 RC 和 CAN 反馈后 `feeder.command.armed/enabled == true`；底盘四个电机目标速度应均为 0。
3. 轻推右摇杆向上再回中、向下再回中：目标速度分别为负（送料）、零、正（退弹）、零；反馈 `curspeed` 应跟随。只交换目标映射，不翻转电机反馈速度符号。该方向修改不代表满仓退弹卡滞已解决。
4. 保持少量行程，切回双 MID：拨弹输出归零；再次进入双 DOWN、摇杆未回中时不得自行恢复。
5. 保持空载、低速，关闭遥控器验证 RC 超时；恢复连接但保持摇杆偏转，应仍不使能，回中后才能继续。
6. CAN 超时可先停动力后断开拨弹反馈验证，再用安全台架验证运行中失联；保护触发后 `enabled == false`、`setcurrent/current == 0`，反馈恢复后也需要回中。

首次先验证闭环方向和停止行为，不用摩擦轮发射来验证拨弹代码。如目标非零而反馈长时间为零，停止测试检查记录，不要直接大幅提高 PID 或电流限幅。

### Live Watch 建议

- 零电流优先看：`feeder.stop_reason`、`feeder.command.last_fault`、`feeder.pid_output`、`feeder.effective_current_limit`。
- 时效直接看：`feeder.rc_age_ms`、`feeder.feedback_age_ms`；从未收到时显示UINT32_MAX。
- 模式/输入：`ctrl.mode`、`rc.rc.s[0]`、`rc.rc.s[1]`、`rc.rc.ch[1]`。
- 使能：`feeder.command.armed`、`feeder.command.enabled`、`feeder.command.rc_seen`、`feeder.command.feedback_seen`。
- 新鲜度：`feeder.command.last_rc_ms`、`feeder.command.last_feedback_ms`（应随新帧递增）。
- 电机：`can2_motor[4].setspeed`、`.curspeed`、`.setcurrent`、`.current`、`.torque_current`、`.temperature`。
- 画图先只选 `setspeed` / `curspeed`，判断目标与反馈是否同号、回中是否减速、是否振荡。

### stop_reason 对照

| 值 | 名称 | 含义 |
| --- | --- | --- |
| 0 | RUNNING | 允许运行；此时电流为零要查目标、误差与pid_output，不是被互锁禁用 |
| 1 | NOT_FIRE | 不在FIRE或不为双DOWN |
| 2 | WAIT_RC | 还未收到有效RC帧 |
| 3 | RC_TIMEOUT | RC新帧间隔达到100ms |
| 4 | WAIT_FEEDBACK | 还未收到拨弹CAN2 ID5反馈 |
| 5 | FEEDBACK_TIMEOUT | 拨弹反馈间隔达到100ms |
| 6 | WAIT_NEUTRAL | 等待双DOWN下回中；不能一直推住等待自行恢复 |
| 7 | INVALID_RC | 摇杆越过±660或拨杆编码非法 |
| 8 | INVALID_CONFIG | 限速、死区、方向或电流限幅配置无效 |
| 9 | OVER_TEMPERATURE | 电机反馈温度大于70摄氏度 |
| 10 | INVALID_PID | PID输出NaN/Inf等非有限数 |
| 11 | WRONG_MOTOR_MODE | 模块收到非SPD电机（正常SPD调用路径不会出现） |

`last_fault`保留最近一次异常，使超时恢复后即使stop_reason已经变为6，也能看到是RC还是反馈超时导致的。它是历史原因，不表示该故障当前仍存在。`pid_output`为本周期限幅前输出；被禁止运行时清零。`speed_error`为本周期目标减反馈。

### 已有实测与当前边界（用户反馈）

- 空载P20/I0.1可平滑正反转；带弹上弹正常，退回弹舱交接位置时可能卡住，用户观察到 `current` 约9000且不转。
- 用户已验证P30/I0能完成退弹全过程，并报告反转期间 `current` 保持16384；该持续值仍需结合同一时段的目标转速、反馈转速和PID原始输出来解释，不能据此断言积分是唯一原因。
- 当前源码按用户明确要求选择P20/I0.1/D1.5，不是把P30/I0自动固化，也不宣称P20/I0.1已经解决带载退弹卡滞。
- 没有加入堵转计时停机、自动反打或正反转独立补偿。反馈有效且不过温时，机械卡住不会自动被识别为失联；遇到持续零速高电流应及时退出FIRE或切断动力，不可长期顶住。

## 软件验证

仓库根目录 PowerShell：

```powershell
& .\tests\run_feeder_tests.ps1
& .\tests\check_feeder_integration.ps1
```

第一项运行真实纯输入策略、PID Reset、已确认机构方向、PID正负对称性以及零输出原因/有效限幅测试；第二项检查工程接入约束，是静态检查，不代替硬件集成测试。默认主机编译器 `C:\mingw64\bin\g++.exe` 可通过 `-Compiler` 指定；输出写入临时目录。

固件仍通过原 VisualGDB / STM32F405 Debug 工程编译。本次只做软件测试和编译，没有烧录、实车驱动或实测 PID。
