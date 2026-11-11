# 本地行为对照测试

依赖：Python 3、JDK、工程构建时使用的 FTCLib 2.1.1 `classes.jar`。
不联网、不连接机器人、不改变当前 Git 工作区，不保存另一份完整工程。
临时目录只提取测试所需 Java 源码，测试结束自动清除。

```sh
python3 local-tests/run_replay.py --ftclib /path/to/core-2.1.1/jars/classes.jar
```

用 `JAVA_HOME` 选择 JDK。FTCLib jar 通常在 Gradle 缓存 `transforms/.../core-2.1.1/jars/classes.jar`。

- `run_replay.py`：从 Git 基线 `96796900f20abc62661e565ae0a93e830493b372` 读取旧代码，
  分别编译旧版与当前代码；只替换 FTC/Android/Pedro 硬件边界，FTCLib 使用真实库。
- `BehaviorReplay.java`：两边执行同一份用例，比较完整的逐周期输出/状态快照。
  固定事件覆盖长按、B 起停、左摇杆复位与 B 同按、相反 D-pad、0.1 秒边界和没有 Pinpoint；
  固定随机种子追加 6,000 个按键、姿态和编码器输入组合；新增 INIT 自动末位置恢复，合计 6,083 条快照。
- `LifecycleCheck.java`：对现版单独断言资源占用、命令中断、参数终点换段、正常完成不停止保持、
  200ms demo 边界、B 锁底盘、左摇杆取消、自动末位置继承、独立挡板命令及 DashTuner STOP；
  云台固定模式使用隔离对象测试，并断言生产默认开关保持关闭。
  新增的 `isFixedPositionEnabled()` 不属于原版 getter 集合，故不计入旧版逐项比较。

- `AutoLifecycleCheck.java`：运行实际自动入口，验证正常/提前结束、INIT 取消、缺少 Pinpoint、
  默认自动起点、最后位置保存和停止清理。Pedro 位姿与 FTC 运行状态使用边界替身。

硬件替身只用于软件等价与生命周期测试，不能据此宣布路径或机构已通过实车测试。
DashTuner 的位置闭环回放将 I/D 设为零，避免真实库的墙钟采样使比较不可重复；
其余控制流程与生产实现一致。非零 I/D 的实际采样行为仍需实机检查。

2026-11-11：DashTuner 换成 Spinup 的 `DashboardTuner`，故从旧版逐项对照中移除，改由 `LifecycleCheck.tunerStop` 单独断言方向、主从功率、shooterMotor2 闭环禁用、运行中换绑和 STOP。

2026-11-11：按住 B 改为移植自 Overtune 的 `ShootAssistCommand`（达速 + 底盘锁定后开挡板、短反转、持续送料），并新增未被调用的 `AutoShootCommand`。
B 行为与旧基线有意不同，请使用 `--current-only`。`LifecycleCheck.shootingAndOwnership` 断言：达速前不开门不送料、锁定后反转→送料、送料提交后持续、定位失效/LT 排堵取消、松 B 全停；`autoShoot` 断言送料 500ms 完成与 3000ms 超时。
