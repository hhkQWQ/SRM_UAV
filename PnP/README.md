# OpenMV / PX4 1.17 室内 Offboard 视觉导航

运行逻辑：人工启动程序，在 Position 中起飞、靠近装甲板并减速 → 零速度参考预热 → 遥控器切入 Offboard → 开始识别、PnP 和跟踪 → 跟踪状态为有效的 `MEASURED` 或 `PREDICTED` 时执行视觉导航 → 对准并保持机体参考点距靶面 1 米。

运行只使用现有的 `main.py → TargetTracker → VisualServo → FlightLink`。仿真也直接调用main.py中的 `vision_enabled/process_image/update_control/pose_log_row`，不再复制另一套处理链。目标确认由跟踪器完成，控制器直接使用其结果；没有独立接管文件，也没有额外的 `ACTIVE` 阶段、稳定1秒或等待5秒门槛。控制仍检查解锁、实际Offboard、飞控遥测、标定配置、三维位置和观测年龄。

## 部署与配置

将以下文件放在 OpenMV 同一目录：

- `main.py`、`config.py`、`armor_detect.py`
- `pose_solver.py`、`target_tracker.py`、`visual_servo.py`、`flight_link.py`、`comm_log.py`

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

首次建轨还有二维中心3帧确认。理想连续有效情况下，第3帧确认中心并计入第一次三维确认，第5帧才首次输出MEASURED。漏检、姿态匹配失败或三维不一致会延长这一过程；切入Offboard后的800ms数据不足窗口仍沿用原规则。

`PRIMING` 是原VisualServo已有的“Position中发送零速度预热”状态，不是独立状态机；不参与目标状态分类，也没有额外目标确认计时器。

`VisualServo.update()` 在一次更新中执行：
`配置/解锁/遥测检查 → 实际模式检查 → 目标状态与位置时效检查 → 计算有界速度或制动`。

`nav` 直接记录该控制器状态：`PRIMING`、`WAIT_OFFBOARD`、`ACQUIRING`、`CENTERING`、`ALIGN`、`PREDICT`、`HOLD`、`BRAKING`、`LOST_RELEASE` 等，不再记录单独接管阶段，也不重复保存同义的 `servo` 列。

### 控制状态 nav

这些状态由现有VisualServo根据当前数据选择，不要求按表从上到下逐级通过。表中的“速度参考”只描述本机输出，是否执行以PX4实际模式为准。

| 状态 | 触发情况 | 本机行为及恢复条件 |
| --- | --- | --- |
| `DISABLED` | NAV_ENABLE=False | 不发送导航参考；保持观察 |
| `CONFIG_UNCONFIRMED` | 标定、模型、外参或控制配置检查不通过 | 停止参考；核对配置JSON中的configured/config_errors及相机参数 |
| `DISARMED` | 链路健康但未解锁 | 停止参考；健康检查在解锁检查之前，故可能先显示flight失败原因 |
| `PRIMING` | 健康、已解锁、实际主模式为3（Position） | 零速度并跟随当前航向预热；不运行目标检测；不是“PX4已允许切换”的确认 |
| `WAIT_OFFBOARD` | 健康、已解锁，处于其他非Offboard模式 | 不发送参考；当前程序只在Position中预热 |
| `ACQUIRING` | 实际Offboard，目标或三维位置尚在确认 | 期望速度为零，按加速度限制制动；有效数据恢复后可导航 |
| `BRAKING` | 实际Offboard，目标位置缺失、过期或不可补偿 | 同上，缺测持续超过800ms则释放 |
| `TARGET_CHANGED` | 已有控制目标后，跟踪目标代数发生变化 | 本次期望速度置零、清除到位与下降许可，随后使用新目标；不是额外稳定等待阶段 |
| `CENTERING` | 仍高于目标高度且没有下降许可 | 水平纠偏，竖直参考向零制动 |
| `ALIGN` | 有效实测目标，尚未满足持续到位条件 | 计算有界三轴速度；下降仍检查水平走廊和图像边缘 |
| `PREDICT` | 跟踪器输出有效PREDICTED | 有限水平导航，竖直参考向零制动，不建立到位确认 |
| `HOLD` | 实测输入的水平/高度误差均≤5cm，持续≥700ms | 仍运行比例外环；超出容差或转为预测后取消到位确认 |
| `LOST_RELEASE` | 数据不足持续>800ms | 停止参考；满足条件时请求一次Position；有效数据恢复且实际仍Offboard时可按原规则继续 |
| `LOOP_STALE` | 控制更新间隔<0或>250ms | 本次停止参考、清除运动参考；满足条件时请求一次Position |
| 与flight相同的失败原因 | 遥测或同步不健康 | 停止参考，健康恢复后重新按实际模式处理 |

### 飞控健康原因 flight

`flight=OK`只表示本程序的通信/估计门控通过，不代表PX4已经允许Offboard，也不包含解锁、模式、配置或目标确认条件。若多个条件失败，reason按下表顺序显示首项；HEALTH事件和周期摘要保留原始判据供电脑端核对。

| 原因 | 当前判据 | 查看哪些数据 |
| --- | --- | --- |
| `HEARTBEAT_STALE` | 未识别目标飞控或有效心跳年龄不在0～1500ms | heartbeat_age_us、rx_bytes、rx_heartbeats、rx_last_hb_*、签名/CRC计数 |
| `TIME_UNSYNCED` | 未获得至少3个一致样本，或同步映射超过5秒未更新 | offset_us、sync_age_us、rtt_us及同步异常事件 |
| `ATTITUDE_STALE` | 姿态接收年龄不在0～200ms | attitude_age_us、SERVICE_DELAY及流请求事件 |
| `VELOCITY_STALE` | 速度接收年龄不在0～200ms | velocity_age_us、SERVICE_DELAY及流请求事件 |
| `ESTIMATOR_UNREADY` | 无估计器消息、消息年龄不在0～1000ms、flags不合格或vel_ratio不合格 | estimator_age_us、estimator_flags、vel_ratio |
| `ATTITUDE_SAMPLE_STALE` | 已通过以上检查，但姿态源时间相对当前时间不在−30～200ms | attitude_age_us与同步状态 |
| `VELOCITY_SAMPLE_STALE` | 同上，速度源时间不在−30～200ms | velocity_age_us与同步状态 |
| `LINK_DISABLED` | LINK_ENABLE=False | 配置；没有飞控链路可用 |

estimator_flags按MAVLink位值解释：1=姿态、2=水平速度、4=垂直速度、8=相对水平位置、16=绝对水平位置、32=垂直位置、128=恒定位置模式、2048=加速度异常。当前健康门控要求1/2/4均置位、128/2048均清零，并要求有限的`0≤vel_ratio<1`。Position退出请求还要求8和32置位。原始flags不是PX4的vehicle_local_position.xy_valid；当前发送的是速度参考，复盘还需结合PX4的速度有效性与模式拒绝记录。[MAVLink估计器标志](https://mavlink.io/en/messages/common.html#ESTIMATOR_STATUS_FLAGS)

控制CSV的attitude_status是“图像采集时姿态匹配结果”，与flight当前健康原因不同：EXACT/INTERPOLATED/EXTRAPOLATED表示成功；UNSYNCED、NO_HISTORY、BEFORE_HISTORY、GAP_TOO_WIDE、EXTRAPOLATION_LIMIT表示该帧缺少可用姿态。

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

## 时间同步与延迟补偿

板端与仿真使用同一份 `FlightLink`：TIMESYNC在首字节即将write时填写时间戳，零字节发送失败后重试会重新取时，部分写入开始后保持原报文不变。只有实际开始发送才建立待回复事务；接收每份消息时重新取时，发送队列等待不再进入时钟偏移公式。同步报文也有排队期限，过期后丢弃并重试。

启动和恢复阶段每200ms采样，稳定后每1秒采样；至少3个近期一致样本后建立同步。低RTT样本与中位数抑制偶发延迟，连续3个一致变化允许重新收敛；单个大离群点不清空历史。确认的大幅时钟变化、PX4重启或本地时钟回退会使旧观测失效。保留30ms RTT上限、5秒同步有效期、200ms遥测时效与30ms姿态外推上限；具体数值统一由 `config.py` 管理。

图像按采集时间查询姿态插值/受限外推和速度插值。控制前重新取时，以PX4速度历史的梯形积分补偿采集至控制的位移；无法覆盖该区间时不继续使用旧目标位置。补偿只作用于控制副本，不写回采集时刻的跟踪状态。短时姿态缺失使用原跟踪器的有限预测，不因单帧缺失额外清空轨迹；同步或整体遥测失效仍执行原健康门控。

板端使用小步非阻塞轮询，不再在采图前整段等待同步回复。检测、候选配对/去重和PnP迭代边界穿插同一个服务函数；等待新图像时也继续收发和控制。仿真保留其ROS/UDP调度及独立通信线程，仅复用PnP算法和链路接口，不模拟OpenMV的硬件执行耗时。服务间隔由日志中的 `poll_gap_us/poll_wall_gap_us` 观察。

这些时间是软件write和消息处理时刻，不是硬件线路时间戳。真实曝光时刻仍由snapshot前后中点近似，`CAMERA_CAPTURE_DELAY_US`需要实机测量；同步估计不能消除全部单向传输延迟。

## 板端自适应调度

`service_runtime()`只负责何时执行已有逻辑，没有新增接管阶段。常规服务调用以COMM_SERVICE_INTERVAL_MS=2ms为目标，阶段边界强制服务；控制以CONTROL_INTERVAL_MS=50ms为目标。健康或模式变化时立即重评控制门控，不必等待下个周期。

例如上次控制在0ms，本应50ms更新，但实际80ms才获得执行机会，则本次dt=80ms，下次从80ms计时，目标130ms；没有补发错过周期，也没有在处理完一帧后额外sleep50ms。等待相机时通过get_frame_available查询，并短暂让出约1ms；缺少该接口的固件退回snapshot前后服务，阻塞影响仍需实测。[OpenMV相机API](https://docs.openmv.io/v4.5.9/library/omv.sensor.html)

控制读取最近完整图像结果，每次用当前遥测重新做位移补偿；重复使用不刷新capture_time_us/measurement_ms，也不重复调用跟踪确认。识别启停或链路代数变化会使缓存及正在处理的旧帧失效；内部revision仅为缓存失效编号，不是控制状态。超过原观测有效期仍制动/释放，不能靠重复发送延长旧目标寿命。

控制CSV只在完整图像记录处采样；无新图时仍维护通信摘要、退出事件和文件刷新，不伪造新frame_id。日志前先服务到期控制，日志后立即恢复通信。Python服务点不能抢占原生find_blobs、snapshot或存储驱动内部的阻塞；50ms/2ms是调度目标，不是硬实时保证。终端solve_ms包含迭代中穿插服务的时间。

## 日志

运行时自动编号创建 `/pnp_pose_NNNN.csv`、同名 `.json` 配置和 `/pnp_pose_NNNN_sync.csv` 通信记录，启动会打印实际路径。控制日志默认约每200ms记录一次，并补记状态变化；通信日志正常每1秒摘要，只对异常和状态转换单独记事件。每秒刷新缓冲，正常退出关闭文件；写入错误只停止相应日志并提示。历史文件不覆盖，不保存图像或视频。

控制CSV由原70列精简为34列，删除完整四元数、重复欧拉角、旋转矩阵、像素角点、滤波位置副本、帧率和求解耗时等常驻列。保留定位有效性、原始相对位置、实际控制输入和必要时序；精确列名以 `main.POSE_LOG_FIELDS` 为准。

| 字段 | 含义 |
| --- | --- |
| `frame_id/capture_time_us` | 当前图像编号与采集时间 |
| `control_time_us/control_capture_time_us` | 最近控制更新的时间及其实际使用的源图像时间；不可默认与当前图像相同 |
| `link_generation` | 本次观测所属链路代数，重置前后的观测不能混用 |
| `track/control_track/nav/flight` | 图像跟踪状态、控制实际采用状态、VisualServo行为和飞控健康原因 |
| `armed/offboard/vision_enabled` | 解锁、实际Offboard和识别启用情况 |
| `pose_valid/pose_reason/relative_valid/relative_reason/reprojection_px` | 区分视觉求解失败、姿态不可用和坐标变换拒绝 |
| `attitude_status/attitude_before_dt_ms/attitude_after_dt_ms` | 姿态匹配结果及前后样本相对采集时刻的时间差 |
| `velocity_age_ms/observation_age_ms` | 采集时速度匹配年龄与最近真实目标观测的年龄 |
| `p_local_raw_x_m/y_m/z_m` | 当前图像采集时的原始NED相对测量 |
| `control_p_local_x_m/y_m/z_m` | 经预测、延迟与有效性检查后实际用于控制的NED位置 |
| `command_vn_mps/ve_mps/vd_mps/setpoint_submitted` | 速度参考和本机缓冲提交结果 |
| `tx_status/tx_image_age_ms` | 发送状态及已write参考的源图像年龄；write不等于PX4采用或执行ACK |

tx_status描述本机最近的发送处理，可被后续提交覆盖，不是每个包的执行确认：

| tx_status | 含义 |
| --- | --- |
| NONE | 尚无速度参考提交 |
| QUEUED | 最新参考已提交，尚未完成完整write；也可能正在部分写入 |
| WRITTEN | 最近一份参考已完整write；单看此值不能证明参考流仍连续 |
| CANCELLED | 待发参考被取消，或链路重置清理；部分已写入的数据帧允许写完，避免破坏串口帧 |
| EXPIRED | 尚未开始写入时超过100ms排队期限 |
| REJECTED | 尚未开始写入时，健康、解锁或真实观测年龄复查失败 |

通信CSV字段定义在 `comm_log.COMM_LOG_FIELDS`。正常摘要保留偏移、最近RTT/创新量、同步年龄、排队等待、心跳和遥测年龄、累计最大服务间隔、发送时刻与源图像年龄，以及CRC、倒序、回复不匹配、同步失效、发送丢弃和诊断丢事件计数。异常行只填写该事件相关字段。

### 基础预热诊断与200ms异常

通信CSV在原字段后追加9列，原34列控制CSV不变；配置JSON以schema_version=3记录调度参数和configured/config_errors。

| 新字段 | 含义 |
| --- | --- |
| armed/main_mode/sub_mode | 心跳确认的解锁及PX4实际模式；主模式3=Position、6=Offboard |
| estimator_flags/vel_ratio/estimator_age_us | 原始估计状态、速度创新量和接收年龄；空值表示不可用/非有限，不能当作0 |
| setpoint_tx_count | 从本次程序启动累计完整写入的速度参考数；零写入和部分写入尚未完成时不计 |
| setpoint_max_gap_us | 本次启动以来、应连续发送期间的最大等待/写入间隔，含首份参考等待；主动停发或链路重置分段，不跨段统计 |
| delay_us | 延迟事件的实测间隔；正常摘要留空 |

正常仍约1秒摘要，健康原因变化时补记HEALTH原始快照；不在线解释每个flags或评定“预热合格”。相邻摘要的setpoint_tx_count差除以时间差可估算区间平均频率，不能证明区间内没有短断流；结合最大间隔、事件和PX4接收状态核对。PRIMING是正在提供零参考，setpoint_submitted是进入本机缓冲，WRITTEN是本地完整write，均不等于PX4已采用。

新增性能事件仅在DIAGNOSTIC_DELAY_MS=200ms及以上记录：

| 事件 | 对应情况 |
| --- | --- |
| SERVICE_DELAY | 相邻通信poll间隔≥200ms；reason是服务所在阶段（capture/detect/pair/dedup/pnp/log等），只能提示停顿附近位置 |
| CONTROL_DELAY | 相邻控制更新间隔≥200ms |
| SETPOINT_DELAY | 应连续发送期间≥200ms未完整写入速度参考；每个停顿只报一次，完成写入后重新计时 |
| LOG_DELAY | 控制日志写入或维护刷新耗时≥200ms；reason为POSE或MAINTENANCE |

该门限只决定新增性能事件，不改变30ms同步RTT、200ms遥测、250ms预测/控制或800ms丢靶规则。现有同步/发送错误事件与CRC等计数继续保留；正常几十毫秒波动不新增延迟事件。同一停顿可能同时造成服务、控制和发送异常，应按时间一起阅读。

### 退出控制的证据

| 事件 | 含义 |
| --- | --- |
| CONTROL_RELEASE | 本机停止一个已经开始提供的参考流；reason为控制状态、链路原因或PROGRAM_EXIT，仅停止时记录一次 |
| MODE_REQUEST | 一次模式请求及入队结果；reason含main/sub、queued、发送seq和触发原因 |
| MODE_TX | 该模式命令完整写入本地串口；main/sub及seq用于与入队事件核对 |
| MODE_ACK | 收到目标飞控的模式命令ACK；记录原始result/progress，不在线匹配为某次请求成功 |
| FLIGHT_MODE | 首份有效心跳或实际模式/解锁变化；新值在armed/main_mode/sub_mode，reason保存旧值 |

ACK的result常见值：0=接受、1=暂拒、2=拒绝、3=不支持、4=失败、5=进行中、6=取消。ACK不回显请求seq，接受也不等于已经切换；实际模式以FLIGHT_MODE及后续摘要为准。queued=0表示未入队；入队后也可能TX_EXPIRED。缺少ACK可能是没有回复或日志丢失，不能自动认定退出成功。

原有释放条件只请求一次Position，没有新增重试或锁定。人工Offboard→Position会停止目标导航并继续零速度预热，因此不一定出现CONTROL_RELEASE，但应有FLIGHT_MODE。日志中有HEARTBEAT_STALE时，不能用旧模式值证明飞机此刻仍处于该模式。

预热排障顺序：先核对configured/config_errors，再看flight/HEALTH的原始判据、实际main_mode和armed，最后看参考写入计数/间隔。PX4端可查看offboard_control_mode是否持续更新；1.17在实际Offboard后才发布对应trajectory_setpoint，预热时后者不更新不能单独证明未收到参考。[PX4参考接收源码](https://github.com/PX4/PX4-Autopilot/blob/v1.17.0/src/modules/mavlink/mavlink_receiver.cpp)

每秒摘要也记录累计串口读取字节 `rx_bytes`、最近读取年龄 `rx_age_us`、串口写入返回的累计字节 `tx_bytes`，以及通过解析器校验的受支持帧数 `rx_frames`。`rx_heartbeats` 和 `rx_last_hb_*` 统计源过滤前的有效格式心跳及其最近来源，`heartbeat_age_us` 只计算已接受的飞控心跳。`rx_signed_rejected`、`rx_unknown_frames` 记录解析器跳过的签名帧和未知消息候选。台架上对照这些计数和 PX4 `mavlink status`，可以判断收到的原始字节和飞控心跳是否仍在更新。

`SYNC_REPLY_TIMEOUT`表示一笔请求未及时得到回复，`SYNC_TIMEOUT`表示超过5秒未接受有效偏移；两者分别统计。`poll_gap_us/poll_wall_gap_us`是启动以来仿真/本地时间与电脑单调时间下的最大服务间隔。暂停仿真时，电脑时间只用于通信调度和日志刷新，不推进物理预测。

将板端通信CSV复制到电脑后，可交给仿真目录中的 `analyze_sync.py` 分析。新通信CSV和历史JSONL均可读取。摘要或异常采样的延迟分布不是全包延迟统计；缺少回复不能单独证明唯一故障原因。

## 仿真

原手动Offboard仿真命令：

```bash
bash /home/hhkqwq/SRM_UAV/sim/start_simulation.sh
```

仿真直接调用本目录的模式门控、识别/PnP、跟踪、延迟补偿、控制发送和日志函数；CSV及状态话题使用PnP字段，只附加5个采集时对齐真值字段，不写旧别名。分析工具按同一字段计算误差和到位，参数取当次PnP配置。仅观察使用 `--observe`；原Mission、独立接管模块和仿真独立的姿态/帧率入口已移除。新通信日志使用精简CSV，历史CSV/JSONL仍可分析。详见 [sim/README.md](/home/hhkqwq/SRM_UAV/sim/README.md)。

## 已完成的修改

- 将通信与控制改为按实际执行时间调度，在等待图像和处理过程中穿插服务。
- 控制使用最近完整图像结果，保留原始观测时间并执行延迟补偿。
- 补齐预热原始诊断、参考写入统计及200ms以上延迟事件。
- 分别记录停止参考、模式请求、命令写入、ACK和实际模式变化。
- 补全目标、控制、健康和发送状态说明，统一PnP与仿真的算法及日志接口。
