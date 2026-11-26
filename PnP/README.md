# OpenMV / PX4 1.17 室内 Offboard 视觉导航

运行逻辑：人工启动程序，在 Position 中起飞、靠近装甲板并减速 → 零速度参考预热 → 遥控器切入 Offboard → 开始识别、PnP 和跟踪 → 跟踪状态为有效的 `MEASURED` 或 `PREDICTED` 时执行视觉导航 → 对准并保持机体参考点距靶面 1 米。

运行只使用现有的 `main.py → TargetTracker → VisualServo → FlightLink`。仿真也直接调用main.py中的 `vision_enabled/process_image/update_control/pose_log_row`，不再复制另一套处理链。目标确认由跟踪器完成，控制器直接使用其结果；没有独立接管文件，也没有额外的 `ACTIVE` 阶段、稳定1秒或等待5秒门槛。控制仍检查解锁、实际Offboard、飞控遥测、标定配置、三维位置和观测年龄。

## 部署与配置

将以下文件放在 OpenMV 同一目录：

- `main.py`、`config.py`、`armor_detect.py`
- `pose_solver.py`、`target_tracker.py`、`visual_servo.py`、`flight_link.py`、`comm_log.py`

当前 `NAV_ENABLE=True`，启用人工 Offboard 导航；`NAV_ENABLE=False` 时持续识别和记录，只观察不发送控制参考。板端和仿真均不使用Mission，不上传任务或GPS保持点。

程序不自动解锁、起飞或请求进入 Offboard。Offboard 通道在 QGC 中自行映射，OpenMV 读取 PX4 心跳中的实际模式，不需要配置原始 RC 通道号。没有增加自动启动脚本；板卡原有 `main.py` 启动机制未修改。

核实相机、焦点、标定及实物尺寸。当前输入320×240，两灯条中心线间距135mm、端点间距55mm；镜头朝下、机头在画面下方，光心位于机体参考点前方0cm、下方2cm。正确到位时，目标相机位置约为 `(0,0,0.98)m`，不必位于光轴中心。`TARGET_HEIGHT_M=1.0` 是机体参考点到靶面的重力方向间距，不是相机深度或距地高度。

## 统一状态与导航条件

`TargetTracker` 输出以下目标状态：
`IDLE`：待机状态，视觉处理未启动；不允许导航
`ACQUIRING`:中心尚在确认，或已有二维中心但可信三维数据不足；原 `CENTER_ONLY` 已并入此状态;不允许导航
`MEASURED`：新的真实三维观测已经过跟踪器确认和一致性检查，Offboard+条件健康时允许导航
`PREDICTED`：利用已建立的三维观测和飞控速度进行有限时间预测，Offboard+条件健康时允许
`LOST`：目标不可用，或预测超过帧数/时间边界；不允许导航

三维状态初建或失效后，仍需跟踪器内原有的3次一致观测。预测最多保留4帧，并限制在250ms内，需要有效飞控速度；不会把预测时间当作新的真实观测。控制器接收 `MEASURED/PREDICTED` 后直接计算参考，不再重复统计确认帧或等待额外稳定窗口。

首次建轨还有二维中心3帧确认。理想连续有效情况下，第3帧确认中心并计入第一次三维确认，第5帧才首次输出MEASURED。漏检、姿态匹配失败或三维不一致会延长这一过程；切入Offboard后的800ms数据不足窗口仍沿用原规则。

`PRIMING` 是原VisualServo已有的“Position中发送零速度预热”状态，不是独立状态机；不参与目标状态分类，也没有额外目标确认计时器。

`VisualServo.update()` 在一次更新中执行：
`配置/解锁/遥测检查 → 实际模式检查 → 目标状态与位置时效检查 → 计算有界速度或制动`。

`nav` 直接记录该控制器状态：`PRIMING`、`WAIT_OFFBOARD`、`ACQUIRING`、`CENTERING`、`ALIGN`、`PREDICT`、`HOLD`、`BRAKING`、`LOST_RELEASE` ，具体定义如下：

### 控制状态 nav

这些状态由现有VisualServo根据当前数据选择，不要求按表从上到下逐级通过。表中的“速度参考”只描述本机输出，是否执行以PX4实际模式为准。
`DISABLED`：NAV_ENABLE=False；相机继续观察和识别，控制器停止提供导航参考。
`CONFIG_UNCONFIRMED`：模型、标定、外参或控制配置未通过检查。控制器停止提供参考，具体原因可结合日志配套 JSON 的 configured 和 config_errors 查看。
`DISARMED`：遥测检查通过，但飞控未解锁。健康检查优先，所以未解锁且遥测也异常时，会先显示遥测失败原因。
`PRIMING`：飞控健康、已解锁且实际处于 Position。程序用零速度和当前航向提供预热参考，视觉跟踪处于 IDLE。
`WAIT_OFFBOARD`：飞控健康、已解锁，但当前既不在 Position，也不在 Offboard。程序等待模式变化，暂不提供速度参考。
`ACQUIRING`：处于Offboard模式，有候选，正在确认。可能还在确认二维中心，也可能已经跟住中心、正在累计三维观测。期望速度设为零，按加速度限制收速||期望速度为零，按加速度限制制动；有效数据恢复后可导航||
`BRAKING`：处于Offboard模式，当前拿不到可用目标位置。可能是还没看到目标、位置已过期，或缺少速度数据而无法做位移补偿。期望速度同样设为零||同上，缺测持续超过800ms则释放||
`TARGET_CHANGED`：已有控制目标后，跟踪目标代数发生变化；本次期望速度置零、清除到位与下降许可，随后使用新目标
`CENTERING`：当前比目标高度高出5cm以上，且下降条件还未满足。先做水平纠偏，竖直速度参考向零收敛。
`ALIGN`：使用有效实测位置计算限幅后的三轴速度。向下运动仍需满足水平对准和图像边缘条件。刚进入到位范围、尚未持续满 700 ms 时，也可以是这个状态。
`PREDICT`：暂时缺少新测量，短时预测位置仍可用。控制器继续有限的水平纠偏，竖直速度参考向零收敛，并清除到位计时。
`HOLD`：水平距离误差和高度误差分别在 5 cm 内，且实测输入持续满足条件至少 700 ms。比例外环继续纠偏，超出容差或转入预测就退出 HOLD。
`LOST_RELEASE`：在 Offboard 中连续 超过 800 ms 没有可用目标位置，停止提供速度参考。ACQUIRING和BRAKING共用这段计时。如果有效数据恢复且飞控仍在 Offboard，控制器可以重新计算参考。
`LOOP_STALE`：两次控制更新时间差为负或超过 250 ms，本次停止提供参考，并清除内部速度指令和相关计时。
与flight相同的失败原因：遥测健康检查失败，`nav` 会直接显示原因，例如 TIME_UNSYNCED 或 VELOCITY_STALE。此时停止提供参考，恢复后按实际飞控模式重新判断。


### 飞控健康原因 flight
`flight=OK`只表示本程序的通信/估计门控通过，不代表PX4已经允许Offboard，也不包含解锁、模式、配置或目标确认条件。若多个条件失败，reason按下表顺序显示首项；HEALTH事件和周期摘要保留原始判据供电脑端核对。

`HEARTBEAT_STALE` | 目标 PX4 的心跳没能刷新。 可能还没识别到目标飞控，也可能最近一次有效心跳的接收年龄超出 0～1500 ms。| 需要关注heartbeat_age_us、rx_bytes、rx_heartbeats、rx_last_hb_*、签名/CRC计数 |
`TIME_UNSYNCED` | 两端时钟还没对齐，或已有的同步已失效。 初次建立时钟映射需要至少 3 个一致样本。距上次接受的同步更新超过 5 秒，映射失效 | 需要关注offset_us、sync_age_us、rtt_us及同步异常事件 |
`ATTITUDE_STALE` | OpenMV 没有及时接纳有效姿态。没有有效四元数历史，或最近一条的接收年龄超出 0～200 ms。 | 需要关注attitude_age_us、SERVICE_DELAY及流请求事件 |
`VELOCITY_STALE` | OpenMV 没有及时接纳有效速度。 没有有效速度历史，或最近一条的接收年龄超出 0～200 ms。 | 需要关注velocity_age_us、SERVICE_DELAY及流请求事件 |
`ESTIMATOR_UNREADY` | 飞控估计器没通过本程序的检查。 可能缺少 ESTIMATOR_STATUS、接收年龄超出 0～1000 ms，也可能是 estimator_flags 或 vel_ratio 不合格。 | 需要关注estimator_age_us、estimator_flags、vel_ratio |
`ATTITUDE_SAMPLE_STALE` | 姿态刚收到也可能报这个。按 PX4 消息自带的时间戳计算，最新姿态离当前时刻太远，超出 −30～200 ms。| 需要关注attitude_age_us与同步状态 |
`VELOCITY_SAMPLE_STALE` | 同样检查 PX4 消息自带的时间戳，对象是最新速度，允许范围也是 −30～200 ms。 | 需要关注velocity_age_us与同步状态 |
`LINK_DISABLED` | LINK_ENABLE=False | 需要关注LINK_ENABLE数值 |

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

光流和测距帮 PX4 判断飞机自己怎么动，PnP 帮 OpenMV 判断靶板相对飞机在哪。 室内可以不用 GPS，也不用给靶板设经纬度，但 PX4 仍需要知道飞机的姿态、速度和本地位置。我们使用MTF-02光流一体化模块对无人机进行定位；应先确认无GPS环境下Position可正常工作。[PX4光流](https://docs.px4.io/v1.17/en/sensor/optical_flow)

当前代码先用相机外参把 PnP 结果从相机坐标系转换到机体系，再用拍照时刻的 PX4 姿态把向量转到本地 NED 轴向。这个向量仍然表示从飞机指向靶板的位置，由 VisualServo 用来计算速度和航向设定值。PnP相对观测目前没有作为 `ODOMETRY` 等测量送入EKF，光流测距也不替代到靶板的PnP距离。

在 QGC 里配好能手动切换 Position 和 Offboard 的档位。`COM_OF_LOSS_T` 决定 PX4 收不到外部设定值多久后触发处置，`COM_OBL_RC_ACT` 决定触发后怎么处理。这些参数由你在 QGC 里设置。飞机能不能切回 Position 并定点保持，取决于当时的定位还是否有效。

## 时间同步与延迟补偿

时间同步解决的是「这张图对应哪一刻的飞控数据」。延迟补偿解决的是「从拍照到控制更新，飞机又移动了多少」。
1. 用 FlightLink 对齐 OpenMV 和 PX4 的时钟。 TIMESYNC 在串口首次写入前填入发送时间，接收端按每条报文的处理时刻取时间，因此软件发送队列里的等待不会算进往返时间。一次写入若返回 0 字节，下次会重新取时。若已经写出部分报文，剩余字节沿用原报文继续发送。
2. 用多次测量估计时钟偏移。 启动和恢复时约每 200 ms 尝试一次，稳定后约每 1 秒一次。程序优先使用往返时间较短的样本，至少有 3 个一致样本才建立同步。往返时间上限是 30 ms，已建立的同步超过 5 秒未更新就会失效。单次偏差过大时先拒绝该次更新，确认时钟大幅变化、PX4 重启或本地时钟回退后，旧观测会失效。
3. 按图像采集时间匹配遥测，再补偿到控制时刻。 process_image() 用图像时间查询姿态和速度，有前后样本时做插值。姿态在最新样本之后最多外推 30 ms，遥测时效门限是 200 ms。到了 update_control()，程序对采集到控制之间的速度做梯形积分，用图像时刻的目标相对位置减去这段时间内飞机的位移。`track` 对应采集时刻，`control_track` 是控制器实际采用的副本。速度历史无法覆盖补偿区间时，该副本按 LOST 处理。偶尔一帧缺姿态时，跟踪器仍可在原有条件内做短时预测。
4. 在图像处理间隙继续处理通信。 OpenMV 等待新帧、做检测和执行 PnP 时，会在可让出的计算边界调用 service_runtime()。仿真复用同一套 PnP 和 FlightLink，收发由独立线程调度。OpenMV 实际最长停顿多久，看板端日志的 poll_gap_us 和 poll_wall_gap_us。
图像时间目前取 snapshot() 起止时刻的中点减去 CAMERA_CAPTURE_DELAY_US。这个中点是曝光时刻的近似，修正值需实测。串口对时同样使用软件时间戳，仍可能受到收发两侧延迟不对称的影响。

## 板端自适应调度

`service_runtime()`只负责何时执行已有逻辑，没有新增接管阶段。常规服务调用以COMM_SERVICE_INTERVAL_MS=2ms为目标，阶段边界强制服务；控制以CONTROL_INTERVAL_MS=50ms为目标。健康或模式变化时立即重评控制门控，不必等待下个周期。

例如上次控制在0ms，本应50ms更新，但实际80ms才获得执行机会，则本次dt=80ms，下次从80ms计时，目标130ms；没有补发错过周期，也没有在处理完一帧后额外sleep50ms。等待相机时通过get_frame_available查询，并短暂让出约1ms；缺少该接口的固件退回snapshot前后服务，阻塞影响仍需实测。[OpenMV相机API](https://docs.openmv.io/v4.5.9/library/omv.sensor.html)

控制读取最近完整图像结果，每次用当前遥测重新做位移补偿；重复使用不刷新capture_time_us/measurement_ms，也不重复调用跟踪确认。识别启停或链路代数变化会使缓存及正在处理的旧帧失效；内部revision仅为缓存失效编号，不是控制状态。超过原观测有效期仍制动/释放，不能靠重复发送延长旧目标寿命。

控制CSV只在完整图像记录处采样；无新图时仍维护通信摘要、退出事件和文件刷新，不伪造新frame_id。日志前先服务到期控制，日志后立即恢复通信。Python服务点不能抢占原生find_blobs、snapshot或存储驱动内部的阻塞；50ms/2ms是调度目标，不是硬实时保证。终端solve_ms包含迭代中穿插服务的时间。

## 日志

运行时自动编号创建 `/pnp_pose_NNNN.csv`、同名 `.json` 配置和 `/pnp_pose_NNNN_sync.csv` 通信记录，启动会打印实际路径。控制日志默认约每200ms记录一次，并补记状态变化；通信日志正常每1秒摘要，只对异常和状态转换单独记事件。每秒刷新缓冲，正常退出关闭文件；写入错误只停止相应日志并提示。历史文件不覆盖，不保存图像或视频。

控制CSV为34列，保留定位有效性、原始相对位置、实际控制输入和必要时序；精确列名以 `main.POSE_LOG_FIELDS` 为准。

`frame_id/capture_time_us` ：当前图像编号与采集时间
`control_time_us/control_capture_time_us`：最近控制更新的时间及其实际使用的源图像时间；不可默认与当前图像相同
`link_generation`：本次观测所属链路代数，重置前后的观测不能混用
`track/control_track/nav/flight`：图像跟踪状态、控制实际采用状态、VisualServo行为和飞控健康原因
`armed/offboard/vision_enabled`：解锁、实际Offboard和识别启用情况
`pose_valid/pose_reason/relative_valid/relative_reason/reprojection_px`：区分视觉求解失败、姿态不可用和坐标变换拒绝
`attitude_status/attitude_before_dt_ms/attitude_after_dt_ms`：姿态匹配结果及前后样本相对采集时刻的时间差
`velocity_age_ms/observation_age_ms`：采集时速度匹配年龄与最近真实目标观测的年龄
`p_local_raw_x_m/y_m/z_m`：当前图像采集时的原始NED相对测量
`control_p_local_x_m/y_m/z_m`：经预测、延迟与有效性检查后实际用于控制的NED位置
`command_vn_mps/ve_mps/vd_mps/setpoint_submitted`：速度参考和本机缓冲提交结果
`tx_status/tx_image_age_ms`：发送状态及已write参考的源图像年龄；write不等于PX4采用或执行ACK

tx_status描述本机最近的发送处理，可被后续提交覆盖，不是每个包的执行确认：

NONE 尚无速度参考提交
QUEUED 最新参考已提交，尚未完成完整write；也可能正在部分写入
WRITTEN 最近一份参考已完整write；单看此值不能证明参考流仍连续
CANCELLED 待发参考被取消，或链路重置清理；部分已写入的数据帧允许写完，避免破坏串口帧
EXPIRED 尚未开始写入时超过100ms排队期限
REJECTED 尚未开始写入时，健康、解锁或真实观测年龄复查失败

通信CSV字段定义在 `comm_log.COMM_LOG_FIELDS`。正常摘要保留偏移、最近RTT/创新量、同步年龄、排队等待、心跳和遥测年龄、累计最大服务间隔、发送时刻与源图像年龄，以及CRC、倒序、回复不匹配、同步失效、发送丢弃和诊断丢事件计数。异常行只填写该事件相关字段。

### 基础预热诊断与200ms异常

通信CSV在原字段后追加9列；配置JSON以schema_version=3记录调度参数和configured/config_errors。

armed/main_mode/sub_mode：心跳确认的解锁及PX4实际模式；主模式3=Position、6=Offboard
estimator_flags/vel_ratio/estimator_age_us：原始估计状态、速度创新量和接收年龄；空值表示不可用/非有限，不能当作0
setpoint_tx_count：从本次程序启动累计完整写入的速度参考数；零写入和部分写入尚未完成时不计
setpoint_max_gap_us：本次启动以来、应连续发送期间的最大等待/写入间隔，含首份参考等待；主动停发或链路重置分段，不跨段统计
delay_us：延迟事件的实测间隔；正常摘要留空

正常仍约1秒摘要，健康原因变化时补记HEALTH原始快照；不在线解释每个flags或评定“预热合格”。相邻摘要的setpoint_tx_count差除以时间差可估算区间平均频率，不能证明区间内没有短断流；结合最大间隔、事件和PX4接收状态核对。PRIMING是正在提供零参考，setpoint_submitted是进入本机缓冲，WRITTEN是本地完整write，均不等于PX4已采用。

新增性能事件仅在DIAGNOSTIC_DELAY_MS=200ms及以上记录：

SERVICE_DELAY：相邻通信poll间隔≥200ms；reason是服务所在阶段（capture/detect/pair/dedup/pnp/log等），只能提示停顿附近位置
CONTROL_DELAY：相邻控制更新间隔≥200ms
SETPOINT_DELAY：应连续发送期间≥200ms未完整写入速度参考；每个停顿只报一次，完成写入后重新计时
LOG_DELAY：控制日志写入或维护刷新耗时≥200ms；reason为POSE或MAINTENANCE

该门限只决定新增性能事件，不改变30ms同步RTT、200ms遥测、250ms预测/控制或800ms丢靶规则。现有同步/发送错误事件与CRC等计数继续保留；正常几十毫秒波动不新增延迟事件。同一停顿可能同时造成服务、控制和发送异常，应按时间一起阅读。

### 退出控制的证据

CONTROL_RELEASE：本机停止一个已经开始提供的参考流；reason为控制状态、链路原因或PROGRAM_EXIT，仅停止时记录一次
MODE_REQUEST：一次模式请求及入队结果；reason含main/sub、queued、发送seq和触发原因
MODE_TX：该模式命令完整写入本地串口；main/sub及seq用于与入队事件核对
MODE_ACK：收到目标飞控的模式命令ACK；记录原始result/progress，不在线匹配为某次请求成功
FLIGHT_MODE：首份有效心跳或实际模式/解锁变化；新值在armed/main_mode/sub_mode，reason保存旧值

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

## Openmv对接飞控串口设置
飞控上为GPS1口 飞控QGC设置为MAV_2 
GPS_1_CONFIG ：disabled
MAV_2_CONFIG ：GPS1(201)
MAV_2_MODE : Normal(0)或者Onboard(2)
MAV_2_FLOW_CTRL : Force off(0)
SER_GPS1_BAUD : 115200