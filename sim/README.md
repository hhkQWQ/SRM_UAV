# PX4 / Gazebo PnP Offboard 仿真

仿真只适配ROS图像、UDP、仿真时钟和显示。模式门控、识别/PnP、跟踪、延迟补偿、控制发送与日志生成直接调用PnP现有函数，不再维护一套平行导航逻辑或控制参数。

仿真用于验证PnP算法逻辑，沿用ROS定时控制与独立通信线程，直接调用PnP的检测、求解、跟踪和控制算法。共用通信CSV包含预热原始判据、写入计数和模式事件。完整状态与事件定义见 [PnP说明](../PnP/README.md)。

## 启动

```bash
bash /home/hhkqwq/SRM_UAV/sim/start_simulation.sh
```

导航开关默认读取 `PnP/config.py` 的 `NAV_ENABLE`，当前为True；仅观察使用 `--observe`。程序不上传任务，不自动解锁、起飞或切Offboard。原 `--mission` 已移除。

```bash
bash /home/hhkqwq/SRM_UAV/sim/start_simulation.sh --observe
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view /pnp/debug_image
```

相机保持320×240输入、PnP内参/畸变、外参和检测配置。OpenCV只适配OpenMV图像API，硬件像素行为与耗时仍有差异。仿真不再提供独立姿态真值辅助或帧率上限入口，飞控姿态与速度只来自PX4；导航参数统一在PnP配置中设置。

## 一套状态与控制流程

`PRIMING` 是原 `VisualServo` 已有的状态值，表示健康、已解锁的飞机仍在Position，程序只发送零速度参考等待人工切Offboard。它不是新的状态机，也不增加目标确认计时器。PX4切换前需要持续接收参考至少约1秒，操作上在PRIMING后保持约1.2秒再切换。[PX4 1.17 Offboard](https://docs.px4.io/v1.17/en/flight_modes/offboard)

1. 人工在Position中起飞、靠近装甲板并减速。此时相机保留预览，不识别，`track=IDLE`、`nav=PRIMING`。
2. 人工切Offboard，实际心跳确认后开始识别和PnP；目标先经过原跟踪器确认。未确认中心或只有二维中心统一为 `ACQUIRING`，不能产生目标引导。
3. 目标为有效 `MEASURED` 或 `PREDICTED`，且配置、解锁、遥测、实际Offboard、位置与观测年龄满足条件，直接使用原控制器。没有额外ACTIVE阶段、稳定1秒或等待5秒。
4. 实测允许水平/竖直修正；预测允许有限水平导航，竖直向零制动。保留原先居中后下降、速度与加速度限制，以及到位保持规则。
5. 数据不足时执行原有制动；持续超过800ms进入 `LOST_RELEASE`，停止参考。本地定位与遥测仍健康时请求一次Position。循环停顿进入 `LOOP_STALE` 时按同一释放逻辑处理。
6. 切走Offboard后停止识别并清空旧跟踪历史。健康Position只预热，下次人工切入时仍按上述同一条件工作。

目标状态是 `track`：IDLE、ACQUIRING、MEASURED、PREDICTED、LOST。控制行为是 `nav`：PRIMING、WAIT_OFFBOARD、CENTERING、ALIGN、PREDICT、HOLD、ACQUIRING、BRAKING、LOST_RELEASE等；两者是原有跟踪器与控制器各自的输出，不是两套接管流程。

仿真直接调用 `PnP/main.py` 中的：

| 函数 | 板端与仿真共用职责 |
| --- | --- |
| `vision_enabled()` | 模式与健康门控 |
| `process_image()` | 检测、取图时刻姿态/速度、PnP、坐标变换和跟踪 |
| `update_control()` | 延迟补偿、原VisualServo更新、发送与释放 |
| `pose_log_row()` | 同名、同义的运行字段 |

真值缺失不等待、不阻止识别或导航。场景靶面世界高度1.5米，建议先在靶面上方2～3米粗导航，最终机体距靶面1米；目标位置只由视觉反馈取得。现有SITL传感器配置不变，不代表已验证MTF-02的无GPS融合。

## 通信与时间对齐

板端和仿真共用 `PnP/flight_link.py` 的收发、同步、历史查询和诊断决策。TIMESYNC在即将开始write时取发送时间，零字节重试重新记时，部分发送后续发原报文；每份接收消息重新取时。发送排队单独记录，不再作为时钟偏移的一部分。

恢复阶段200ms采样、稳定后1秒采样，至少3个近期一致样本后同步。低RTT样本和中位数抑制离群点，连续一致的偏移变化允许重新收敛；确认的大幅变化或时钟回退使旧观测失效。30ms RTT上限、5秒同步有效期以及姿态和速度时效保护仍由共用配置管理。

仿真保留独立通信线程，调度目标约2ms。短锁只保护链路状态操作，图像计算和日志写盘在锁外执行；实际服务间隔以日志为准。线程异常使健康原因变成 `LINK_WORKER_FAILED`，不继续正常目标引导。板端使用采图等待期间及处理阶段的协作式服务，不再整段等待同步回复；两端共享算法和链路判据，不要求硬件调度相同。

姿态按采集时刻插值或受限外推，速度按历史插值。控制位置通过采集到控制之间的速度积分补偿，无法覆盖的时间段不使用旧目标位置。真值缺失不等待，单帧姿态缺失不额外清空跟踪器；两者都使用共用PnP规则。

Gazebo时钟负责物理时间、图像和控制时刻；电脑单调时钟只用于通信线程调度和日志刷新。暂停不会用电脑时间推进目标预测，回退会清理旧时间映射。软件write时刻不是硬件线路到达时间，实际PX4执行也不能由write直接确认。

## 字段与画面

CSV、`/pnp/status`和画面使用同一套PnP字段。控制CSV默认5Hz，并补记状态变化。共用控制字段由原70列精简为34列，删除完整四元数/RPY、旋转矩阵、像素角点、相机位置、重复滤波位置、帧率/求解耗时等常驻列；字段全集由 `PnP/main.py` 的 `POSE_LOG_FIELDS` 定义。

| 字段 | 含义 |
| --- | --- |
| `frame_id/capture_time_us` | 当前图像编号与采集时间 |
| `control_time_us/control_capture_time_us/link_generation` | 控制时间、实际使用的源图像时间和观测链路代数 |
| `vision_enabled/track/control_track/nav/flight` | 识别门控、图像与控制端目标状态、控制行为和健康原因 |
| `armed/offboard` | PX4实际解锁及Offboard状态 |
| `pose_valid/pose_reason/relative_valid/relative_reason/reprojection_px` | 区分视觉求解、姿态匹配和坐标变换问题 |
| `attitude_status/attitude_before_dt_ms/attitude_after_dt_ms/velocity_age_ms` | 采集时刻的遥测匹配情况 |
| `observation_age_ms` | 最近真实三维观测年龄 |
| `p_local_raw_x_m/y_m/z_m` | 采集时的原始NED相对位置 |
| `control_p_local_x_m/y_m/z_m` | 实际用于控制的延迟补偿后位置，不能直接与采集时真值相减 |
| `command_vn_mps/ve_mps/vd_mps/setpoint_submitted` | 速度参考和发送缓冲提交结果 |
| `tx_status/tx_image_age_ms` | 发送状态和已write参考的源图像年龄，不等于PX4执行ACK |

仿真只附加5个采集时真值字段：`true_p_local_x_m/y_m/z_m`、`truth_dt_us`、`truth_source`。有可靠对齐数据才写真值，缺失留空；这些字段不进入控制。原始相对位置与真值在同一采集时间比较，控制时位置不与采集时真值比较。

日志保存在 `sim/logs`。`pnp_sim_时间.csv/json`记录控制数据及当次配置。`pnp_sim_时间_sync.csv`记录通信摘要和异常事件，字段与板端共用。修改源码后重启算法才能生效，UDP14540 由一个算法进程使用。

## 分析工具

```bash
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_log.py --latest --last-seconds 20
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_navigation.py --latest
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_sync.py --latest
```

- `analyze_log.py`读取精简格式和历史格式。新格式只计算采集时原始相对位置与同刻真值误差，不补造已经移除的相机/滤波位置误差。待命IDLE不计为识别失败，缺少真值不当作零误差；飞行日志选择会排除通信CSV。
- `analyze_navigation.py`选择最后一段实际已解锁Offboard记录，按当次PnP的 `ARRIVAL_XY_M/ARRIVAL_Z_M/ARRIVAL_DWELL_MS` 检查采样HOLD。新格式要求控制源图像时间等于对应真值图像时间；PREDICTED计入有效导航但不建立到位确认。真值检查位于源图像采集时刻，不代表稍后的发送或执行时刻。
- 到位参数与日志间隔优先取同名JSON中的运行配置；历史缺项会标注当前配置回退。采样间隔超过两倍记录周期会中断持续到位证据，未记录的控制输入或源图像关联不补造。
- `analyze_sync.py`同时支持新的 `*_sync.csv` 和旧的 `*_sync.jsonl`。新格式只周期性摘要及稀疏异常事件，报告的是采样延迟分布，不宣称全包P95。`request_timeouts`是请求等待超时，`timeouts`是5秒有效映射失效；等待时长不混入实测RTT分布。

通信CSV直接记录共用FlightLink实际决策：偏移、RTT/创新量、同步年龄、排队等待、心跳/姿态/速度采样年龄、累计最大服务间隔、发送年龄和必要异常计数。删除每条事件重复的完整快照、Mission上下文和每次正常交换记录。`poll_gap_us/poll_wall_gap_us`分别是物理时间与电脑单调时间下启动以来的最大服务间隔；异常行只填相关字段，空值不当作零。

通信日志按电脑时间约每1秒刷新摘要，即使仿真暂停也可记录状态；物理时效仍用Gazebo时间。诊断缓冲有界，溢出计入 `events_dropped`，写盘失败只停止相应日志并提示。正常退出刷新，强制终止可能留下残缺末行；分析器会报告无效行。缺少回复本身不能区分对端未回、丢包或处理延迟。

## 已完成的修改

- 统一调用PnP的识别、定位、跟踪、延迟补偿和控制逻辑。
- 统一控制与通信日志，保留采集时刻对齐的仿真真值。
- 保留历史CSV和JSONL日志的读取兼容。
