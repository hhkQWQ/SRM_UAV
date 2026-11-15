# PnP 视觉接管与靶面上方 1 米悬停

操作流程：手动粗导航到装甲板可见的位置并减速 → 等待 `READY_FOR_MISSION` → 操作者切到 Mission → 程序预热并自动切入 Offboard → 先水平对准，再下降到机体参考点距靶面 1 米，持续修正悬停。

目标位置来自图像，不需要在 QGC 标目标点。PX4 的 Mission 执行已存储任务，视觉速度参考由 Offboard 接收，所以切 Mission 是本任务的触发操作，实际视觉控制期间 QGC 显示 Offboard。[PX4 Mission](https://docs.px4.io/main/en/flight_modes_mc/mission)、[PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard)。

## 启动仿真

```bash
bash /home/hhkqwq/SRM_UAV/sim/start_simulation.sh --mission
```

保持 QGC 和相机窗口运行，相机选择 `/pnp/debug_image`。先自行起飞、靠近柱顶标靶并停止移动；算法终端/日志显示 `READY_FOR_MISSION` 后再切 Mission。程序用 PX4 当前经纬度和高度自动上传一个保持任务项，供 Mission 切换过渡；因此该选项会替换当前飞控任务。该保持点不代表标靶位置，最终导航只使用 PnP 相对位置。手动水平移动超过 25 cm、改变高度超过 15 cm 或准备点超过 10 秒时重新准备，避免回到过时点位。

柱顶靶面世界高度为 1.5 米；最终机体世界高度约 2.5 米。粗导航建议机体距靶面 2～3 米，并保持灯条方向在原检测器允许的范围内。无需知道靶面高度也能完成视觉阶段。

仅观察：

```bash
bash /home/hhkqwq/SRM_UAV/sim/start_simulation.sh
```

已有的 `--enable-control` 保留为手动进入 Offboard 的方式。完整仿真说明见 [sim/README.md](/home/hhkqwq/SRM_UAV/sim/README.md)。

## OpenMV 部署

将以下运行文件放在 OpenMV 同一目录：

- `main.py`、`config.py`、`armor_detect.py`
- `pose_solver.py`、`target_tracker.py`、`visual_servo.py`
- `flight_link.py`、`mission_handover.py`、`mission_standby.py`

电脑侧默认 `NAV_ENABLE=False`、`MISSION_ENABLE=False`，不会自动接管。启用上述 Mission 操作需两者同时为 True，且保留已确认的模型、内参、安装外参和新鲜 PX4 遥测。程序不自动解锁、起飞或切入 Mission。板端尚未验证实际耗时与串口负载，仿真通过不代表已完成实机整定。

当前相机配置为 320×240；光心位于机体前方 14 cm、下方 4 cm，镜头朝下且机头在画面下方。正确到位时，目标相机位置约为 `(0,-0.14,0.96)m`，因此目标无需处在图像光轴中心。`TARGET_HEIGHT_M=1.0` 是机体参考点至靶面的重力方向间距，不是相机深度或距地高度。

## 求解、跟踪与控制

`HORIZONTAL_TARGET=True` 表示靶面已知水平朝上。利用曝光时刻 PX4 姿态把 Down 转为相机法向，固定该法向后求解平面内角度与三维平移，降低四个低像素端点造成的距离/法向耦合。该法向是先验，不是独立测得的板姿态。关闭该选项或没有姿态时，保留自由平面 PnP；单应与历史位姿分别优化后再选择重投影更好的候选。

检测器阈值、颜色模式、实测模型及 320×240 输入保持原设置。仍检查正深度、几何、重投影、采样年龄与坐标变换；小重投影误差不能单独证明模型、标定或板姿态正确。

三维位置初次建立或失效后，需要 3 次一致观测。预测只在有限帧数和 250 ms 范围内有效，且需要飞控速度；重复时间戳不会累计确认。导航先减小水平误差，进入 10 cm 走廊才允许下降；超出 16 cm 或目标接近图像边缘则停止下降。预测期间竖直速度向零减小。最大水平/垂直速度为 0.30/0.15 m/s，仍限制加速度。

准备保持点和接管都要求新鲜真实三维观测与低速状态；总观测年龄不超过 250 ms 的短预测只保留已建立的稳定窗口，不能开始任务上传或触发模式切换。接管稳定 1 秒后预热至少 1.2 秒零速度参考，最多请求 Offboard 3 次。收到实际 Offboard 心跳后才执行视觉运动。人工切走模式、控制循环停顿、飞控时钟/世代异常或长时丢靶后锁止，不自动抢回；丢靶先制动，之后请求 PX4 Hold 并停止视觉参考。锁止时显示 `BLOCKED_原因`，再次切 Mission 会请求退回 Hold。重新接管需由操作者切 Position，等待健康恢复和新鲜观测稳定，再等待新的 `READY_FOR_MISSION`；程序会重新准备当前位置保持点。到位后保持 Offboard 即可持续悬停，无需再切 Mission。

姿态、速度、估计器健康、心跳与 TIMESYNC 都参与门控。MAVLink 只接受选定 PX4 的遥测，任务请求/ACK还检查接收方。默认不写飞控参数。需要 global position 才能准备 Mission 保持项；没有全球位置时，Mission 操作流程不成立。

## 验证状态

2026-11-15：70 项离线检查通过，覆盖求解、跟踪、控制、任务协议、短时缺帧和模式退出。本次临时测试脚本与重复副本已清理，原有测试数据保留；实际悬停效果由用户自行仿真验证，本次不声明 PX4 完整闭环通过。

运行日志包含姿态、PnP 模式、测量年龄、接管/控制状态与真值误差。Gazebo 真值仅用于统计，不进入视觉位置控制；统计命令见 sim 使用说明。
