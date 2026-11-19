# OpenMV / PX4 1.17 室内 Offboard 视觉导航

运行逻辑：人工启动程序，在 Position 中起飞、靠近装甲板并减速 → 零速度参考预热 → 遥控器切入 Offboard → 开始识别、PnP 和跟踪 → 跟踪状态为有效的 `MEASURED` 或 `PREDICTED` 时执行视觉导航 → 对准并保持机体参考点距靶面 1 米。

运行只使用现有的 `main.py → TargetTracker → VisualServo → FlightLink`。仿真也直接调用main.py中的 `vision_enabled/process_image/update_control/pose_log_row`，不再复制另一套处理链。目标确认由跟踪器完成，控制器直接使用其结果；没有独立接管文件，也没有额外的 `ACTIVE` 阶段、稳定1秒或等待5秒门槛。控制仍检查解锁、实际Offboard、飞控遥测、标定配置、三维位置和观测年龄。

## 部署与配置

将以下文件放在 OpenMV 同一目录：

- `main.py`、`config.py`、`armor_detect.py`
- `pose_solver.py`、`target_tracker.py`、`visual_servo.py`、`flight_link.py`

当前 `NAV_ENABLE=True`，启用人工 Offboard 导航；`NAV_ENABLE=False` 时持续识别和记录，只观察不发送控制参考。板端和仿真均不使用Mission，不上传任务或GPS保持点。

程序不自动解锁、起飞或请求进入 Offboard。Offboard 通道在 QGC 中自行映射，OpenMV 读取 PX4 心跳中的实际模式，不需要配置原始 RC 通道号。没有增加自动启动脚本；板卡原有 `main.py` 启动机制未修改。

当前串口为 UART3 / 115200 / 8N1。H7 Plus 的 P4(TX) 接飞控 RX，P5(RX) 接飞控 TX，并共地。以 TELEM2 配给 MAVLink 实例1为例：`MAV_1_CONFIG=TELEM 2`、`MAV_1_MODE=Onboard`、`SER_TEL2_BAUD=115200`；仅接 TX/RX/GND 时设置 `MAV_1_FLOW_CTRL=0`。根据实际端口、实例和设备手册调整，需要重启的参数在修改后重启飞控生效。

核实相机、焦点、标定及实物尺寸。当前输入320×240，两灯条中心线间距135mm、端点间距55mm；镜头朝下、机头在画面下方，光心位于机体参考点前方14cm、下方4cm。正确到位时，目标相机位置约为 `(0,-0.14,0.96)m`，不必位于光轴中心。`TARGET_HEIGHT_M=1.0` 是机体参考点到靶面的重力方向间距，不是相机深度或距地高度。

## 统一状态与导航条件

`TargetTracker` 输出以下目标状态：

| `track` | 含义 | 是否允许目标导航 |
| --- | --- | --- |
| `ACQUIRING` | 中心尚在确认，或已有二维中心但可信三维数据不足；原 `CENTER_ONLY` 已并入此状态 | 否 |
| `MEASURED` | 新的真实三维观测已经过跟踪器确认和一致性检查 | 满足Offboard及健康条件时允许 |
| `PREDICTED` | 利用已建立的三维观测和飞控速度进行有限时间预测 | 满足Offboard及健康条件时允许 |
| `LOST` | 目标不可用，或预测超过帧数/时间边界 | 否 |

板端未启动识别时，日志中 `track=IDLE`，表示待命，而非进行识别后丢失目标。

三维状态初建或失效后，仍需跟踪器内原有的3次一致观测。预测最多保留4帧，并限制在250ms内，需要有效飞控速度；不会把预测时间当作新的真实观测。控制器接收 `MEASURED/PREDICTED` 后直接计算参考，不再重复统计确认帧或等待额外稳定窗口。

`PRIMING` 是原VisualServo已有的“Position中发送零速度预热”状态，不是独立状态机；不参与目标状态分类，也没有额外目标确认计时器。

`VisualServo.update()` 在一次更新中执行：
`配置/解锁/遥测检查 → 实际模式检查 → 目标状态与位置时效检查 → 计算有界速度或制动`。

`nav` 直接记录该控制器状态：`PRIMING`、`WAIT_OFFBOARD`、`ACQUIRING`、`CENTERING`、`ALIGN`、`PREDICT`、`HOLD`、`BRAKING`、`LOST_RELEASE` 等，不再记录单独接管阶段，也不重复保存同义的 `servo` 列。

## 执行与退出

1. 人工启动程序并在 Position 中起飞，靠近装甲板、减速。程序在已解锁、配置就绪、遥测健康的 Position 中输出零速度参考，`nav=PRIMING`。相机可采图预览，但不运行检测或PnP。
2. `PRIMING` 后等待至少约1.2秒，再由遥控器切Offboard。PX4 1.17要求切换前持续接收设定值至少约1秒，运行中高于2Hz；该等待是操作要求，没有另设程序接管计时器。程序以心跳中的实际Offboard为准。[PX4 Offboard](https://docs.px4.io/v1.17/en/flight_modes/offboard)
3. 进入Offboard后开始识别和跟踪；未确认的中心或二维中心统一为 `ACQUIRING`，不据此朝目标移动。一旦跟踪器给出有效 `MEASURED/PREDICTED`，即按原控制器导航，仍保留加速度约束和目标切换时的过渡。
4. 先水平对准，进入10cm水平走廊才允许下降；超出16cm或目标接近图像边缘时停止下降。预测允许水平导航，竖直参考向零制动，不继续盲降。最大水平/垂直速度为0.30/0.15m/s，并限制加速度。
5. 数据不足或丢靶时，只执行原有零目标速度的有界制动，不再产生目标引导；持续超过 `LOST_BRAKE_MS=800` ms 后，`nav=LOST_RELEASE`，停止参考。本地位置和遥测健康时，板端在进入该释放状态时请求一次Position；其他情况依靠PX4已配置的失效处置。
6. 人工切走Offboard后，板端停止识别并清除上一轮图像跟踪历史；控制器清除运动参考、下降许可和到位确认。回到Position后只预热零速度，再次切Offboard按同一套条件工作。

短时缺测恢复到有效 `MEASURED/PREDICTED` 且飞控仍在Offboard时，可以继续导航；没有额外接管层的锁止或重复确认。飞控若已经退出Offboard，则必须由操作者重新切入。遥测不健康或控制循环间隔超过250ms时停止参考；进入 `LOOP_STALE` 且本地位置健康时同样有界请求一次Position。

## 纯室内定位与飞控配置

不需要GPS或标靶全球坐标，但PX4仍需可用的姿态、速度和本地定位。用户提供的来源可能是MTF-02光流，接线、测距和EKF融合尚未实测；应先确认无GPS环境下Position可正常工作。PnP相对观测目前没有作为 `ODOMETRY` 等测量送入EKF，光流测距也不替代到靶板的PnP距离。[PX4光流](https://docs.px4.io/v1.17/en/sensor/optical_flow)

在QGC的Flight Modes中自行选择模式通道与Offboard档位，保留可切回Position的操作。配置并验证 `COM_OF_LOSS_T`、`COM_OBL_RC_ACT`；Position请求能否成功取决于当时定位，不能承诺状态估计失效后仍能定点保持。程序不写飞控参数。

## 日志

运行时在OpenMV当前根文件系统创建 `/pnp_pose_0001.csv` 等递增编号文件，启动打印 `LOG=实际路径`。默认约每200ms记录一次，每秒刷新缓冲，正常退出时关闭文件。停桨不关闭日志；断电或强制重启不能保证最后未刷盘数据完整。`LOG_ERROR=...; recording stopped` 表示记录已停止。只保存数值，不保存图像或视频。

| 字段 | 含义 |
| --- | --- |
| `capture_time_us` / `log_time_us` / `control_time_us` | 图像采集近似时刻、记录时刻、最近控制更新时刻；均为本次程序的本地单调时钟 |
| `vision_enabled` | 该帧是否运行识别与PnP |
| `track` | 图像跟踪状态，如 `MEASURED/PREDICTED/ACQUIRING/LOST`，待命为 `IDLE` |
| `control_track` | 最近控制更新经过延迟/超时检查后实际使用的跟踪状态 |
| `nav` | 现有 `VisualServo` 状态，如 `ALIGN/HOLD/ACQUIRING/BRAKING` |
| `flight` | 飞控遥测就绪情况或拒绝原因 |
| `armed` / `offboard` / `main_mode` / `sub_mode` | 心跳中的实际模式与解锁状态 |
| `setpoint_submitted` | 最近参考是否进入发送缓冲，不等于飞控执行确认 |
| `command_vn_mps` / `command_ve_mps` / `command_vd_mps` | 最近控制更新产生的北/东/下速度参考 |
| `p_local_x_m` / `p_local_y_m` / `p_local_z_m` | 图像跟踪器估计的目标相对位置，NED，单位m |
| `pose_valid` / `relative_valid` / `observation_age_ms` | PnP、坐标变换有效性及真实观测年龄 |

例如 `track=MEASURED, control_track=MEASURED, nav=HOLD, offboard=1` 表示控制器使用真实观测并判断已经到位；`MEASURED`本身不等于到位，需要结合相对位置判断。5Hz采样可能遗漏短暂状态变化，时间戳不直接等同北京时间或PX4日志时间。

## 仿真与验证范围

原手动Offboard仿真命令：

```bash
bash /home/hhkqwq/SRM_UAV/sim/start_simulation.sh
```

仿真直接调用本目录的模式门控、识别/PnP、跟踪、延迟补偿、控制发送和日志函数；CSV及状态话题使用PnP字段，只附加8个真值字段，不写旧别名。分析工具按同一字段计算误差和到位，参数取当次PnP配置。仅观察使用 `--observe`；原Mission、独立接管模块和仿真独立的姿态/帧率入口已移除。原有同步诊断和历史日志保留。详见 [sim/README.md](/home/hhkqwq/SRM_UAV/sim/README.md)。

2026-11-19：当前字段和共享处理链通过24项离线验证及语法检查，覆盖板端主循环、模式门控、实测/预测导航、制动与释放、两端字段一致性及分析边界。旧日志只在分析读取时转换，未修改控制参数或新增接管状态机。临时测试文件在完成后清理，不进入项目。未连接真机、未运行Gazebo完整闭环；MTF-02融合、实际频率、串口负载和悬停效果仍需实测。
