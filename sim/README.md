# PX4 / Gazebo PnP 视觉导航仿真

仿真沿用原 `PnP` 检测、求解、跟踪、控制程序，ROS 图像与 UDP 只负责替换 OpenMV 的取图和串口接口。输入保持 320×240、原相机内参与畸变、前移 14 cm / 下移 4 cm 的相机安装参数，以及 GRAY / ANY 灯条检测配置。Gazebo 原始相机通过桥接重映射为该标定图像。

## 你需要的操作方式

```bash
bash /home/hhkqwq/SRM_UAV/sim/start_simulation.sh --mission
```

此终端启动 PX4、Gazebo、图像桥接与算法。打开 QGC，并在相机窗口选择 `/pnp/debug_image`：

```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view /pnp/debug_image
```

1. 自行解锁、起飞，手动粗导航到能看见柱顶双灯条的位置。靶面世界高度为 1.5 米；建议机体先在靶面上方 2～3 米，保留正常灯条方向。
2. 减速并保持装甲板完整可见。程序持续获得三维位置且速度小于 0.2 m/s 后，准备 Mission 保持项。
3. 等待画面或算法日志显示 `READY_FOR_MISSION`，再由你在 QGC / 遥控器切到 Mission。
4. 程序确认稳定观测，预热零速度参考，自动进入 Offboard；随后先水平对准，再下降并悬停在机体距靶面 1 米的位置。该地图最终世界高度约为 2.5 米。

不需要在 QGC 标目标点。程序自动上传的任务仅包含飞机当前 GPS 位置和高度上的单项保持点，用于让 PX4 接受 Mission 切换。手动移动超过 25 cm、改变高度超过 15 cm 或准备点超过 10 秒时会重新准备，因此 `--mission` 会替换现有飞控任务。标靶的位置与最终悬停位置由 PnP 相对反馈计算，不使用 Gazebo 标靶坐标。

Mission 是接管触发；PX4 在 Offboard 中执行程序发来的视觉速度参考，因此视觉阶段 QGC 显示 Offboard。[PX4 Mission](https://docs.px4.io/main/en/flight_modes_mc/mission)、[PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard)。

`--mission` 不自动解锁、起飞或切入 Mission。到位后保持 Offboard，程序会持续修正悬停，无需再切 Mission。人工切走模式、时钟异常或丢靶会锁止视觉接管并显示 `BLOCKED_原因`；此时切 Mission 将被退回 Hold。重新接管时，先由操作者切 Position，等待新鲜观测稳定、当前位置保持点重新准备并出现新的 `READY_FOR_MISSION`，再切 Mission。短时丢靶先制动，持续丢靶后请求 Hold 并停止视觉参考。保持点准备需要 PX4 的全球位置；无 GPS 的纯室内方案需要其他任务触发接口。

## 观察和手动 Offboard

仅观察，不发送导航参考：

```bash
bash /home/hhkqwq/SRM_UAV/sim/start_simulation.sh
```

保留原手动 Offboard 方式：

```bash
bash /home/hhkqwq/SRM_UAV/sim/start_simulation.sh --enable-control
```

自行起飞到目标可见位置，程序预热后手动切 Offboard。此方式不准备 Mission 保持项。所有模式都保持相同检测配置；不要同时启动两个占用 UDP 14540 的算法进程。停止旧仿真后再启动新一轮。

## 检查精度与收敛

```bash
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_log.py --latest --last-seconds 20
python3 /home/hhkqwq/SRM_UAV/sim/pnp_integration/analyze_mission.py --latest
```

第一个命令比较定位输出与同一时刻真值。第二个检查最后 10 秒的真实水平误差、距靶高度误差、视觉接管与真实观测连续性；只有真值完整、持续处于视觉 Offboard 且误差达标才报告 `passed=true`。刚起飞、没有收敛或缺少真值时不会报告成功。

`HORIZONTAL_TARGET=True` 使用靶面水平先验与 PX4 曝光时刻姿态约束法向，降低少像素端点造成的高度波动。输出的该法向并非独立测得的靶板姿态；验证自由旋转精度时需关闭该选项。仿真真值仅用于统计；`--truth-attitude` 仅允许显式观察诊断，与控制 / Mission 不得并用。

日志位于 `sim/logs`：

- `algorithm_*.log`：识别、遥测、保持项准备与接管状态。
- `bridge_*.log`：相机桥接状态。
- `pnp_sim_*.csv/json`：图像时间、四点、姿态、PnP 模式、控制输出、任务状态、真值误差与运行配置。

运行模块和板端配置说明见 [PnP/README.md](/home/hhkqwq/SRM_UAV/PnP/README.md)。本次 70 项离线检查通过；实际悬停验证由用户自行进行。本次临时测试脚本和重复副本已清理，原有 CSV、JSON、飞行日志及文档均保留。
