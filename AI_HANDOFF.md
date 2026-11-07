# AI 接手说明

## 长期协作规则

- 2026-11-07 用户要求：以后每完成一次任务，都要把本次所做的修改、验证结果和需要注意事项更新到这个 Markdown 文件，方便其他 AI 接手时了解程序情况。
- 后续记录按日期追加，保留重要历史状态；如果后续修改推翻了旧记录，应明确注明，不要让旧说明误导接手者。

## 当前程序概况

- 主遥控程序：`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotTeleOp.java`。
- 当前主要包名为 `subsystem`（小写、单数），不要恢复成旧的 `subSystems`。
- `subsystem/TurretSubsystem.java`：基于 Pinpoint od 坐标和朝向的双 Hive 云台自瞄；不使用独立 IMU 作为云台反馈。
- 云台瞄点：`(60, 60)` 和 `(60, 84)`，在 `y=72` 附近带 1 英寸滞回。
- 云台按键：X 切换自瞄，dpad 左右微调 2°，START 重置定位和云台；目标超出舵机行程时手柄震动。
- 云台参数保留：300° 舵机行程、0.98 传动比、180° 机械安装偏移。
- `subsystem/DriveSubsystem.java`：麦轮底盘及 Pinpoint；当前初始位置为 `(72, 72, 0°)`，偏移为 `(-20, -135.2) mm`。
- `AutoPath.java` 和 `subsystem/Shooter.java` 当前均为整体注释的备份代码，不参与编译和运行。

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
