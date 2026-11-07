# AI 接手说明

## 长期协作规则

- 2026-11-07 用户要求：以后每完成一次任务，都要把本次所做的修改、验证结果和需要注意事项更新到这个 Markdown 文件，方便其他 AI 接手时了解程序情况。
- 后续记录按日期追加，保留重要历史状态；如果后续修改推翻了旧记录，应明确注明，不要让旧说明误导接手者。

## 当前程序概况

- 主遥控程序：`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotTeleOp.java`。
- 当前主要包名为 `subsystem`（小写、单数），不要恢复成旧的 `subSystems`。
- `subsystem/TurretSubsystem.java`：云台三态主控制，基于 Pinpoint od 坐标和朝向，不使用独立 IMU 作为云台反馈。
- `subsystem/TurretAim.java`：双 Hive 自瞄解算，瞄点为 `(60, 60)` 和 `(60, 84)`，在 `y=72` 附近带 1 英寸滞回。
- 云台按键：常规状态固定 0.5；X 切换自瞄，Y 切换场地方向锁定，两者互斥；dpad 左右微调 2°；START 重置定位并恢复常规状态；仅自瞄超出行程时手柄震动。
- 云台参数保留：300° 舵机行程、0.98 传动比、180° 机械安装偏移。
- `subsystem/DriveSubsystem.java`：麦轮底盘及 Pinpoint；当前初始位置为 `(72, 72, 0°)`，偏移为 `(-20, -135.2) mm`。
- `AutoPath.java` 当前已启用四点自动路线；`subsystem/Shooter.java` 为双飞轮 PD 控制，同时供遥控和自动程序使用。
- Shooter 电机名为 `shooterMotor1`、`shooterMotor2`；A 切换 3500 RPM。RB 按住时仅 intake 正转，RT/LT 按住时 intake/transit 联动，所有进料均为松开即停。
- `subsystem/Blocker.java` 已启用并接入主程序；设备名 `blockerServo`，关闭位置 0.11，打开测试位置 0.50，左 Bumper 上升沿切换状态，每帧刷新舵机命令。

## 2026-11-07：修复进料定时、Shooter 响应与 Blocker 动作

- 删除 Shooter 中 RB 触发固定 1 秒发射的 `fireActive/fireEndTime` 逻辑，不再自动开启 Shooter。
- RB 改为按住时仅 intake 正转；RT/LT 保持按住联动、松开停止；`IntakeSubsystem` 新增 `setIntakeOnly()`。
- A 键处理移到 Shooter PD 计算之前，目标转速和电机输出当帧生效；增加 A 输入、目标/实际 RPM、功率和两个电机输出遥测。
- Shooter 软件逻辑在 A 开启且实际 RPM 为 0 时应输出较大正功率；若遥测显示目标 3500、输出非零而电机不转，应检查电机供电、端口、配置名称和机械卡滞，而不是继续调按键代码。
- Blocker 打开位置由 0.10 改为 0.50 做动作测试，关闭仍为 0.11；每帧调用 `setPosition()` 并增加 LB 输入遥测。实车确认后应重新测定安全开合位置。
- 验证：`TeamCode:assembleRelease` 编译成功；三项功能仍需实车确认。

## 2026-11-07：新增 Blocker 子系统

- 根据新版测试程序提取 Blocker 逻辑，取消原空类注释并完成实现。
- 在 `NameConstants` 注册 `BLOCKER_SERVO = "blockerServo"`。
- `Blocker` 构造时关闭舵机，提供 `open()`、`close()`、`isOpen()` 和手柄更新接口。
- `RobotTeleOp` 已创建、启动并每圈更新 Blocker；左 Bumper 每按一次切换开关，遥测显示状态和位置。
- 本次只提取 Blocker，没有同步附件中右 Bumper 改为单独 intake 的其他控制变化。
- 验证：`TeamCode:assembleRelease` 编译成功；0.10/0.11 的实际开合方向与幅度需实车确认。

## 2026-11-07：同步新版云台与 Shooter

- 根据新版 637 行测试程序提取云台和 Shooter 逻辑，未搬入底盘和 IMU 相关代码。
- `TurretSubsystem` 改为常规／自瞄／锁定三态；Y 进入锁定时记录当前云台绝对朝向，避免突然甩动。
- `TurretAim` 改为独立双 Hive 解算类，负责瞄点滞回、目标方位角、微调偏移和近距离最后方向保持。
- `Shooter` 改为两个电机，PD+F 参数同步为 `kP=0.0060`、`kD=0.00001`、`kF=0.00020`；功率范围为 `-0.1~1.0`。
- `RobotTeleOp` 已接入 Shooter 和进料控制；`IntakeSubsystem.setPower()` 保留 intake/transit 反向功率关系。
- 保留 AutoPath 使用的旧 `update(...)` 转速接口，但旧发射电机 2/3 不再由 Shooter 控制，自动供弹由 IntakeSubsystem 完成。
- 验证：`TeamCode:assembleRelease` 编译成功；电机名称、方向和 PD 参数仍需实车确认。

## 2026-11-07：四点自动路线与 Hive 射击

- `AutoPath.java` 路线改为：`(58, 8, 90°)` 起点 → `(58, 23, 90°)` 射击点 → 射击 `(58, 52)` 的 Hive → `(58, 108, 90°)` → `(14, 108, 90°)` 终点。
- 初始前进方向为 `+Y`，按项目坐标约定确定初始朝向为 90°，不是 270°。
- 最后一段按“向左平移”实现，车头保持 90°，不是在终点转向 180°。
- 自动程序使用 Pinpoint 闭环走点和朝向比例控制；位置到位后只修正朝向，每段有 8 秒超时保护。
- 恢复 `subsystem/Shooter.java`，自动射击目标为 3500 RPM，达到 95% 后供弹 3 秒；这些参数仍需实车调参。
- 为 `Shooter` 和 `IntakeSubsystem` 增加 `stop()`，射击结束或 OpMode 停止时将相关电机功率清零。
- AutoPath 的 Pinpoint 偏移已同步为 `(-20, -135.2) mm`。
- 验证：`TeamCode:assembleRelease` 编译成功；尚未进行实车路线和射击验证。

## 2026-11-07：GitHub 合并后的检查与修复

- 检查了合并改动、Git 冲突标记及 TeamCode 编译结果。
- 修复 `TurretSubsystem.java` 中两代代码混杂的问题：删除重复的旧 `servo` 构造器和旧舵机字段，只保留 `TurretSubsystem(HardwareMap)`。
- 统一云台接口为 `update(Gamepad, Telemetry)`，与 `RobotTeleOp` 的调用一致。
- 删除误合并的旧版 Y/B/reset 手动控制块，并修复 `setB othPosition` 拼写损坏。
- 将 `AutoPath.java` 恢复为整体注释状态，保留源代码作为备份；其旧代码依赖已改名的包和目前禁用的 Shooter，不能直接启用。
- 验证：`TeamCode:assembleRelease` 编译成功，未发现残留 Git 冲突标记。

## 构建命令

在项目根目录执行：

```bash
export JAVA_HOME="D:\\Android studio\\jbr"; ./gradlew.bat :TeamCode:assembleRelease
```
