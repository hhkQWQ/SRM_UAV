# PX4 / Gazebo PnP Offboard 仿真

仿真只适配ROS图像、UDP、仿真时钟和显示。模式门控、识别/PnP、跟踪、延迟补偿、控制发送与日志生成直接调用PnP现有函数，不再维护一套平行导航逻辑或控制参数。

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

## 字段与画面

CSV、`/pnp/status`和画面使用同一套PnP字段。新日志不再写 `track_state/control_state/control_submitted`、`sim_time_s` 等旧别名，也不重复保存可从向量计算的误差列。

| 字段 | 含义 |
| --- | --- |
| `capture_time_us/log_time_us/control_time_us` | 图像、记录、控制时刻，单位微秒，使用同一Gazebo时间域 |
| `vision_enabled/track/control_track/nav` | 是否识别、图像跟踪状态、控制端实际使用状态、控制器行为 |
| `armed/offboard/main_mode/sub_mode/flight` | 实际飞控模式与遥测健康原因 |
| `pose_valid/relative_valid/observation_age_ms` | PnP、坐标变换有效性和真实观测年龄 |
| `p_C_x_m/y_m/z_m` | 相机系目标位置 |
| `p_local_raw_x_m/y_m/z_m` | 当前真实三维观测的NED相对位置 |
| `p_local_x_m/y_m/z_m` | 跟踪器滤波或预测后的NED相对位置 |
| `command_vn_mps/ve_mps/vd_mps/setpoint_submitted` | 速度参考和发送缓冲提交结果，不等于飞控执行ACK |
| `fps/solve_ms/reprojection_px` | 帧率、求解耗时与重投影误差 |

仿真只附加8个对齐真值字段：`true_p_C_x_m/y_m/z_m`、`true_p_local_x_m/y_m/z_m`、`truth_dt_us`、`truth_source`。这些字段不进入控制。ROS图像不提供采集耗时，所以 `capture_ms` 留空。

日志保存在 `sim/logs`，历史CSV/JSON/同步诊断不改写。修改源码后重启算法才能生效，不要同时运行两个占用UDP14540的算法进程。

## 分析工具

```bash
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_log.py --latest --last-seconds 20
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_navigation.py --latest
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_sync.py --latest
```

- `analyze_log.py`：按新向量字段计算相机、原始相对位置、滤波/预测位置误差；报告状态计数、耗时和观测年龄。识别有效率以实际识别样本为分母，不把IDLE算作识别失败。缺少真值不当作零误差。
- `analyze_navigation.py`：选择最后一段实际已解锁Offboard记录，分析末尾窗口；停机后的IDLE不覆盖上一段结果。`passed`仅表示末段连续采样满足PnP自身 `ARRIVAL_XY_M/ARRIVAL_Z_M/ARRIVAL_DWELL_MS`，并且控制器为HOLD、采用MEASURED、参考已提交。PREDICTED正常计入导航，但和原控制器一样不建立到位确认。不再使用另设的10cm、95%导航或90%实测通过门槛。
- 到位参数与日志间隔优先取同名JSON中的当次PnP配置；历史记录不完整时明确标注当前配置回退。采样间隔超过两倍记录周期会中断到位证据，防止仅两个稀疏样本被当作持续悬停。
- 旧字段只在分析读取时转换，不写回历史日志；未记录的控制端输入不补造。工具报告残缺行、时钟回退和重复时间戳，输出统一为JSON。
- `analyze_sync.py`保留原同步原因分析，状态上下文改用同一 `nav/flight/control_track`，不改变同步判定或飞控通信。

2026-11-19：24项离线验证覆盖共享调用、板端主循环、导航条件、字段一致性及分析边界；Python/Shell语法检查通过。未启动Gazebo完整闭环或进行实机飞行，项目不保留临时测试文件。板端说明见 [PnP/README.md](/home/hhkqwq/SRM_UAV/PnP/README.md)。

## 时钟同步诊断（仅sim内修改）

sim在原始 `PnP/flight_link.py` 的类外附加诊断包装：收包、发送、轮询和同步判断仍执行原方法，包装只观察调用前后状态。PnP源码、阈值和导航逻辑保持原样；无需修改启动命令。

算法启动日志显示 `TIMESYNC diagnostics=完整路径`，独立日志为 `sim/logs/pnp_sim_时间_sync.jsonl`。停止旧仿真、重新启动后自行复现 `TIME_UNSYNCED`，运行：

```bash
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_sync.py --latest
```

`timeouts` 列出每次超时前的最近回复，以及自上次接受样本以来的拒收次数；`event_counts` 汇总实际观察到的分支。文件中记录原始模块文件SHA-256，便于确认本轮运行的源码版本。

| 事件/字段 | 能确认的现象 |
| --- | --- |
| `REQUEST_QUEUED` / `REQUEST_QUEUE_FULL` | 同步请求入队或入队失败 |
| `REQUEST_TX.queue_wait_us` | 请求排队至UART/socket write的耗时 |
| `REJECT_RTT.rtt_us` | 原程序使用的请求至回复处理间隔被判超限 |
| `REJECT_JITTER.innovation_us` | 偏移样本超过滤波阈值，原程序未接受该样本 |
| `ACCEPT_INITIAL` / `ACCEPT_FILTER` / `ACCEPT_RESET` | 观察到原程序建立偏移、正常更新或重置 |
| `REPLY_TOKEN_MISMATCH` / `REPLY_NO_PENDING` / `REQUEST_REPLACED` | 无法匹配当前请求的回复或旧请求被替换 |
| `BOOT_BACKWARD` / `LOCAL_CLOCK_BACKWARD` | 飞控启动时间或本地观察时间回退 |
| `SYNC_STATUS.reason` | 无偏移、样本不足、接受超时等具体同步状态 |
| `poll_gap_us` / `poll_wall_gap_us` | 仿真时钟和电脑单调时钟下的轮询间隔 |
| `prior_export_wall_us` / `max_export_wall_us` | 上次和最长日志导出耗时，检查记录本身的影响 |

记录独立于相机帧，只要sim计时器继续运行，仿真时钟暂停时也可按电脑时间写快照。`local_us`沿用原模块时钟，`wall_us`为电脑单调时钟，`remote_us`来自PX4回复。回复的 `local_us` 是原poll使用的时间戳，`observed_local_us` 为包装处理回复后的观察时刻。

事件缓冲128项，溢出计入 `events_dropped`；文件写入失败只停止诊断导出并提示。正常退出刷新，强制终止可能留下不完整末行或丢失末尾缓冲，分析工具会提示可观察到的缺失。

发送记录是write API观察时刻，接收记录是poll/解析观察时刻，均非线路或硬件到达时间戳。`drain_context`是导出时状态，不是每个事件发生时的精确状态。缺少回复不能单独区分飞控未回复、链路丢包和接收处理延迟；必要时仍需飞控端日志或抓包。报告只列证据，不宣称唯一根因。
