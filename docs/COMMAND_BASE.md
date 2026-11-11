# Killerqueen：jrc CommandBase 重构与逐链路对照

当前工作分支：`jrc`，跟踪 `origin/jrc`。
行为基线：`96796900f20abc62661e565ae0a93e830493b372`（`11.11-1`）。
先前在 main 上完成的 CommandBase 结构已迁入，并逐项整合 jrc 领先 main 的 5 次提交。
FTCLib 保持 `2.1.1`，Pedro 保持 `3.0.2`；只改本仓库，没有混入 Overtune/Spinup 配置。

## 车体尺寸（2026-11-11 用户提供）

Killerqueen：长 **311 mm**，宽 **344 mm**（约 **12.2441 × 13.5433 in**）。
仅记录车体长宽；不是轮距或轴距，不代入底盘标定参数。单位按当前对话沿用毫米。

## 从哪里开始读

```text
Teleop                         初始化、INIT 定位恢复、START/STOP、遥测
└─ TeleopControlCommand        周期顺序、按键边沿、B 发射与左摇杆复位优先级
   ├─ DriveCommand            场地坐标驾驶（B 发射期间禁止驾驶）
   ├─ TurretCommand           云台模式和舵机更新
   ├─ ShooterCommand          A 开关和飞轮 PD+F 更新
   ├─ BlockerCommand          LB 开关、上下微调
   ├─ IntakeCommand           RB / RT / LT 送料
   ├─ ResetLocalizationCommand  左摇杆按下一次性复位
   └─ AimAndShootCommand      B 联合发射，按 jrc 等 0.1 秒送料

AutoPath_full
└─ SequentialCommandGroup
   └─ FollowPathCommand × 3 → PathFollowerSubsystem → Pedro Follower
                                └─ 更新定位后保存 AutoEndPose

AutoPath_demo
└─ TimedDriveCommand → DriveSubsystem
   定位循环保存 AutoEndPose，STOP 时保存最后有效位置

DashTuner                      保留原 Dashboard 配置字段
└─ DashboardTuningCommand → DashboardTuningSubsystem
```

所有 `command/` 类都继承 FTCLib `CommandBase`，声明所写子系统的 requirements。
入口统一使用 FTCLib `CommandScheduler`，移除 Ivy 依赖。
`OpMode`/`LinearOpMode` 是 FTC 入口类型，保留它们不妨碍使用 CommandBase。

## jrc 的新增行为全部保留

| 相比 main 的改变 | 合入后的行为 |
|---|---|
| 底盘电机方向 | 手动、demo、Pedro 都使用左侧 `REVERSE`、右侧 `FORWARD` |
| B 发射时底盘停止 | B 联合发射处于活动状态时，无论摇杆输入如何都调用 `drive.stop()` |
| 发射延时 | `AimAndShootCommand` 沿用 jrc 原文件，等待 **0.1 秒**而非 main 的 1 秒 |
| 复位键变化 | **左摇杆按下**触发；Start 不再执行复位 |
| 复位取消发射 | 左摇杆复位先结束联合发射，再复位定位/云台；继续按住 B 不会重新开始，须释放后再按 |
| 自动末位置继承 | 两个自动清除旧记录，运行及退出时保存位置；手动 INIT 等待 Pinpoint READY 后恢复并重新读取 |
| 手动航向来源 | 已完成位置初始化时使用 Pinpoint 场地航向；尚未初始化时用 Hub IMU；单独保留 IMU 遥测 |
| 手动 Pinpoint 偏移 | 沿用 jrc `(0.787402, 5.323543) in`，不替换成 main 或 Pedro 的数值 |
| 新增图解/调参资料 | jrc 的图解、页面文件和截图保留；说明文档同步当前控制方式 |

当前 Pedro 配置直接引用 `LocalizationSubsystem.PINPOINT_X_OFFSET_IN` / `PINPOINT_Y_OFFSET_IN`，自动与手动共用 `(0.787402, 5.323543) in`。下方 jrc 行为回放验证手动/命令生命周期，不代表这次自动偏移变更已通过实车定位标定。

## 为什么手动使用一个有序组合命令

`TeleopControlCommand` 持有所有私有子命令 requirements 的并集，由外层调度单元占有六个子系统。
子命令不另行提交全局调度器，组合负责其 `initialize/execute/end`。
这样能明确保证 jrc 的同周期执行顺序，而不会因为分别调度发生意外抢占。

左摇杆复位只结束组合内部的 B 发射，再复位定位和云台，不会取消整个手动控制循环。
外部命令抢占任一资源则取消整个组合并停机，私有发射命令也会被清理。

B 期间，云台和飞轮控制器仍每周期运行，只屏蔽手动目标变更。
`AimAndShootCommand` 管目标和送料时序，**本身不是完整的飞轮 PD+F/云台更新循环**；
复用到别的入口时仍需提供这些周期更新。

## 一条控制链一条控制链核对

| 控制链 | 保留的 jrc 行为 | 负责位置 |
|---|---|---|
| INIT 定位 | 自动记录优先，否则 `(8,111,0°)`；READY 后写入并重新读取；没有 READY 不冒充有效坐标 | `Teleop.init_loop()` → `LocalizationSubsystem` → `AutoEndPose` |
| 每周期定位 | 在按键处理和控制前更新；保留 jrc 的初始化、暂时失去 READY 时的状态处理 | `TeleopControlCommand` → 原 `LocalizationSubsystem.update()` |
| 底盘 | `-左Y/左X/右X`，死区 `<0.05`；场地坐标混控、归一化不变；B 活动时停止 | `DriveCommand` / `drive.stop()` → 原 `DriveSubsystem` |
| 云台 | X、Y、左、右顺序处理；模式互斥；步长 2°；回中 0.5；0.2～0.8 限位；瞄点滞回、机械比例和震动不变 | `TurretCommand` → `TurretSubsystem`；组合处理震动 |
| 飞轮 | A 切换 3300 RPM；B 时禁用 A；每周期原滤波和 PD+F；28 ticks/rev、符号及全部系数保留 | `ShooterCommand` → 原 `ShooterSubsystem` |
| 挡板 | LB 边沿切换 0.38/0.55；上下同按不微调，单按 ±0.01；松 B 当周期禁止微调，并同步 LB 边沿 | `BlockerCommand` → `flushOutput()` |
| 手动送料 | RB > RT > LT；RB `(1,0)`；RT 同向、LT 反向；阈值 `>0.05`；松 B 同周期恢复手动输入 | `IntakeCommand` → 原 `IntakeSubsystem` |
| B 按下 | 关挡板、停送料、启动飞轮、记时；START 前就按住 B 不触发 | 原 `AimAndShootCommand.initialize()` |
| B 持续 | 满 0.1 秒开挡板、两送料电机全功率；Pinpoint READY 决定是否开自瞄与状态文字 | 原 `AimAndShootCommand.execute()` |
| B 松开 | 停送料、关挡板、停飞轮并写输出、关自瞄，然后恢复手动处理；长按 LB 不算新按键 | 原 `AimAndShootCommand.end()` + 组合边沿 |
| 左摇杆复位 | 在本周期手动更新之后取消发射并复位；本周期若先前在发射，底盘仍停，下周期恢复驾驶 | `TeleopControlCommand` + `ResetLocalizationCommand` |
| 自动完整路径 | 3 段坐标不变，第三段起始朝向仍 180°；参数终点切段；正常完成保留 Pedro 终点保持；更新后保存位置 | `FollowPathCommand` + `PathFollowerSubsystem` |
| 自动 demo | 从自己的默认起点开始；0.7 功率、0.2 秒；保留 INIT/运行定位循环和退出保存；取消归零 | `TimedDriveCommand` + `AutoPath_demo` |
| DashTuner | 原配置字段、加载条件、从电机负速度、位置 PID、功率/舵机/传感器遥测保留；STOP 显式归零 | `DashboardTuningCommand` / `DashboardTuningSubsystem` |

手动周期：定位 → B 边沿 → 底盘 → 云台/震动 → 飞轮 → 手动挡板/送料 → 左摇杆复位 → 记住按键 → 联合送料。
挡板在命令内显式写出，避免 FTCLib 前置执行 `periodic()` 让微调晚一周期。

### 本次保留的限制

- jrc 联合发射只等 **0.1 秒**，不等待实际达速、瞄准到位或定位 READY。没有擅自增加放行条件。
- 飞轮误差超过 200 的提示仍只是遥测提示。
- 手动航向选择保留原 `positionInitialized` 判定；初始化后短暂非 READY 时会沿用已保存场地航向。
- 完整自动仍只有三段行驶，没有新增吸球或射击。
- 第 2 段末 90° 到第 3 段起始 180° 的变化保留。
- DashTuner 仍在 INIT 加载设备；运行中改设备名可能引用未加载设备，不在本次重构里改变。
- 挡板微调不额外夹紧变量，仍由 Servo API 处理最终限幅。

路径完成条件依据 [Ivy v1.1.1 PedroCommands.follow 源码](https://github.com/Pedro-Pathing/Ivy/blob/v1.1.1/pedro/src/main/java/com/pedropathing/ivy/pedro/PedroCommands.java)，
保留 `atParametricEnd()`，没有改成额外等待静止或误差收敛。

## 云台固定模式：已预留，当前关闭

`TurretSubsystem.FIXED_POSITION_ENABLED = false`。
用户确认将来固定到初始化位置 **0.5**，随后要求先禁用，等待确认。
当前默认构造、自瞄、方向锁定、左右微调与 B 云台行为均维持 jrc 逻辑。

启用后的逻辑已经写好：双舵机固定 0.5，X/Y/左右/复位/B 均不改变该位置，车身转动不补偿。
送料和飞轮时序不变。模式仅在构造时确定，测试使用隔离的 `true` 对象，不改变生产默认开关。
这是云台子系统的模式，不接管纯底盘自动或直接操作舵机的 DashTuner。

## 验证与边界

- 本仓库 `:TeamCode:assembleDebug` 编译成功。
- **6,083 条快照与原版 jrc 完全一致**，使用真实 FTCLib 2.1.1、实际机构控制代码、同一组硬件替身和时钟。
  覆盖 6,000 个随机组合周期、B/复位/同时按键、0.1 秒边界、INIT 坐标恢复/READY 等待、DashTuner 输出。
- 独立断言验证资源抢占清理、B 锁底盘、左摇杆取消发射、保存航向/坐标单位、无效位置拒绝、显式自动起点、
  路径更新后保存、顺序交接、正常终点保持、200ms demo、挡板单独调度及 DashTuner STOP。
- 使用实际 `AutoPath_full` / `AutoPath_demo` 入口配合硬件/生命周期替身，验证正常结束、提前 STOP、
  INIT 中取消、缺少 Pinpoint 时的位置保存条件和电机清理。
- 云台隔离锁定对象经过 100 轮切换/复位/发射始终输出 0.5，同时断言默认开关为 false。

运行方法见 [local-tests/README.md](../local-tests/README.md)。
这不是形式化等价证明，也不是实车测试；不模拟电机惯性、真实定位噪声或 Android 调度抖动。
比较的是每周期完成后的状态/输出，不保证硬件写入的微秒级先后相同。
新增生命周期改进为取消清理、demo 单调运行时计时与 DashTuner 显式停机。
原 main 上的 6,073 条结果只是前一轮记录，不能代替本轮 jrc 基线的验证。

当前版本在 jrc 分支维护，尚未装车验证。实车需验证方向、B 停底盘/松键恢复、
左摇杆取消、自动→手动位置继承、三段路径衔接和每个入口 STOP 归零。

## 新增 AutoFormal（2026-11-11）

按用户要求复制当前 AutoPath_full，保留原 AutoPath。驾驶站名称与类名均为 `AutoFormal`。
顺序为 `path101path1 → path2 → path3 → path4 → path5 → path6 → path7 → path8`。
起点为 (54.2726, 7.9672, 90°)，终点为 (8.6715, 107.8308, 0°)。
第 4 段有三个控制点，第 5 段有两个控制点，其余六段为直线。
第 4 段以 270° 结束，第 5 段在同一坐标以 90° 起始。
沿用原命令调度、自动末位置保存与停止清理，仅换路径，不新增机构动作。
Java编译及自动入口正常结束/提前STOP/INIT取消检查通过，尚未实车运行。
