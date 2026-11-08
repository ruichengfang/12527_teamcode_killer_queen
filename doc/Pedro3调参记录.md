# Pedro Pathing 3 调参记录

记录日期：2026-11-08。适用于本机器人的麦克纳姆底盘和 goBILDA Pinpoint。

独立 Pedro 项目：`C:\Users\ruich\Desktop\coding\FRC\5449\PedroPathing3\PedroPathing3-Quickstart\Quickstart-master`。完整可执行配置保存在该项目的 `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`。

## 底盘

用户已确认电机名称和位置正确；方向来自原程序和用户提供的 Mecanum 调参结果。

| 位置 | HardwareMap 名称 | 方向 |
| --- | --- | --- |
| 左前 | `leftFrontMotor` | `REVERSE` |
| 右前 | `rightFrontMotor` | `FORWARD` |
| 左后 | `leftBackMotor` | `REVERSE` |
| 右后 | `rightBackMotor` | `FORWARD` |

## Pinpoint

设备名 `od`；里程计轮类型 `goBILDA_4_BAR_POD`。位置单位与偏移单位均为英寸。

- X 偏移：`0.35498074659212364`
- Y 偏移：`-5.000615457850178`
- X、Y 编码器方向：均为 `FORWARD`

## Foresight

- 前向主 / 次比例系数：`0.38393330018585853` / `0.14185309553195313`
- 横向主 / 次比例系数：`0.6152812620499513` / `0.22732998570934385`
- 前向、横向分段切换阈值：`2.5`
- coast / brake 前馈系数：`0.013665014127100502` / `0.011615262008035426`
- heading 比例系数：`5.18755016785564`
- heading 刹车系数：`(0.06338642014730769, 0.005807550581152556)`
- 线性刹车矩阵对角线：`(0.14263630430518828, 0.09510941432054501)`
- 二次刹车矩阵对角线：`(9.681199759397902E-4, 0.001749740870689422)`
- 最大前向 / 横向速度：`73.71144044955173` / `58.59378224521996`
- 自然前向 / 横向减速度：`31.365307658162813` / `62.98938045238759`

## 验证状态

项目已通过 Gradle 编译。对话中尚未确认包含这些最终参数的新版程序已上传到机器人，也未确认 Pose Test 和 Line Test 的结果；需要实机验证。
