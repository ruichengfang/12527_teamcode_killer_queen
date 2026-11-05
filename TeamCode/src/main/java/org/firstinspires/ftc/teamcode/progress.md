# 开发进度

## 文件地址

```
D:\study\FRC\FtcRobotController\TeamCode\src\main\java\org\firstinspires\ftc\teamcode
```

## 2026-11-04 当前进度

1. 底盘：Mecanum 场地方向驱动 + Pinpoint(od) 定位；右 Bumper 自瞄回正 0°。
2. 发射：shooter 飞轮 0/1 PID 控速，电机 2/3 供弹，已接入主程序。
3. 入料：intake 与 transit 电机合并到 inTake 子系统，同步正反转。
4. 云台：servo IMU 反向补偿；AutoAim 自瞄时新增云台回中（0.5）。
5. 自动：AutoPath 已整体注释，暂停使用。
6. 所有设备名统一在 NameConstants 管理。
7. 编译通过。

### 已知待修问题

1. ~~MecanumDrive intake 初始化被注释，运行空指针闪退~~（2026-11-05 已修复）
2. 右 Bumper 同时触发自瞄和 shooter 供弹，按键冲突。
3. 主循环的 servo.update 会覆盖 AutoAim 的云台回中。
4. inTake.constant() 只降速 intake 电机，transit 电机会保持上次功率不停转。
