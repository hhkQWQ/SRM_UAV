# OpenMV 单核下视水平视觉里程计

固定使用 **320×240 灰度图**，不设目标 FPS。MicroPython 负责采图、飞控通信和状态管理，原生 `rm_vo` 模块负责主要视觉计算。N/E 由图像解算，姿态来自飞控，高度来自 MTF-02P ToF；ToF 模块的光流不参与视觉匹配。

**本版需要包含 rm_vo 的自编译固件。仅复制 Python 文件到原来的 4.5.9 固件不能运行。**缺模块时会直接提示编译安装方法，不会自动切回旧算法。

## 文件与入口

| 文件 | 职责 |
|---|---|
| `config.py` | 已标定内参、安装外参、曝光、串口、跟踪与输出参数 |
| `algorithm.py` | 姿态运算、紧凑原生接口、按需 OpenMV ORB 恢复 |
| `control.py` | 输入时间对齐、ToF 高度、状态恢复、质量与方差、显式重置 |
| `comm.py` | MAVLink、飞控发现、同步、固定容量历史环与最新观测发送 |
| `main.py` | QVGA 采图、曝光控制、单核主循环和实际 FPS |
| `output.py` | 一行摘要与独立 ODOMETRY 发布准入 |
| `test.py` | 同一主循环的详细诊断、实际新图像 Hz 和有效定位 Hz |
| `native/vo_core.c/.h` | 可移植 C99 核心；无相机、UART、MicroPython 或堆分配依赖 |
| `native/modvo.c` | MicroPython 缓冲协议绑定，模块名 `rm_vo` |
| `native/prepare_openmv.py` | OpenMV 4.5.9 STM32 固件接入 |
| `native/verify.py` | 使用真实 C 核心的简要 PC 合成检查 |

板上同时替换七个 Python 文件。正常入口为 `main.py`；测试入口为 `test.py`。不要混用旧接口。

## 算法与运行行为

1. 将图像时刻对应到飞控时钟，缓存中插值姿态；仅在有限时段使用角速度外推。ToF 在其测量时刻做倾角与安装偏移补偿，再对齐图像。
2. 初始化以原分辨率 FAST-9、6×4 网格建立约 48 个分散特征，保存独立参考图像。生成 ORB 描述子供以后恢复。
3. 用飞控相对姿态、每帧高度、相机安装位置以及最近有效视觉速度预测特征位置。在特征周围按局部仿射模型重采样原始模板。
4. 先做 8×8 稀疏 KLT；失败时做 SAD 候选搜索、KLT 亚像素精化和反向检查。小范围整体拟合失败后扩大到搜索上限，再尝试 ORB 恢复。所有图像坐标始终是 QVGA 坐标。
5. 对候选点去畸变、投影到统一地面平面，拟合水平平移与受限残余偏航。位移粗分组后拟合，再按像素重投影残差筛选并精化；必要时使用有限次数的两点 RANSAC。
6. 小运动优先匹配固定关键帧；大运动优先近邻，并每三帧优先尝试固定关键帧。跟踪点减少时仅在缺点网格补充特征。关键帧依据位移、高度、旋转与匹配比例续接，原点保持连续。
7. 短暂失跟保留参考并尝试恢复；连续视觉失败达到 `LOST_RESET_US` 后明确丢弃旧轨迹，下帧建立新零点，并增加 `reset_counter`。输入缺失时位置无效，不能把预测或冻结位置作为有效观测。
8. 有效结果使用其原始图像时间戳，最多发送一次；发送队列保留最新的尚未开始观测。显示限频独立于视觉处理与发布检查。

残余偏航只吸收小幅旋转误差，不替换飞控显示姿态。相机安装臂长参与地面投影，减少原地旋转造成的假平移。固定关键帧抑制同一视野内的累积误差，持续换帧后仍有里程计漂移。

四个项目的具体参考入口与实现边界见 [native/SOURCES.md](native/SOURCES.md)。

## 参数变化

| 参数 | 默认与含义 |
|---|---|
| 图像 | 320×240 GRAYSCALE，已删除缩图与目标 FPS 配置及其依赖 |
| `MAX_KEYPOINTS` | 48；原生固定容量上限 64 |
| `FAST_THRESHOLD` | 15，原生 FAST-9 的灰度阈值 |
| `KLT_MAX_ITERATIONS` / `KLT_MAX_ERROR` | 12 / 15；迭代上限与去亮度偏移后窗口 RMS 灰度误差 |
| `KLT_FB_ERROR_PX` | 1.5 px，正反向跟踪误差 |
| `SEARCH_MIN_PX` / `SEARCH_MAX_PX` | 6 / 40 px，围绕预测位置的搜索半径；不是总位移上限 |
| `PREDICTION_ACCEL_M_S2` | 8，只用于扩大搜索不确定性，不作为速度或加速度否决门槛 |
| `MAX_RESIDUAL_YAW_RAD` | 0.12 rad，几何模型允许的剩余偏航 |
| `NEAR_REFERENCE_FLOW_PX` | 8 px，大运动优先近邻参考 |
| `FEATURE_TOPUP_RATIO` | 0.75，点数低于目标比例时补充缺点网格 |
| `LOST_RESET_US` | 2,000,000 us，持续失跟后显式重置轨迹 |
| `ATTITUDE_HZ` / `RANGE_HZ` | 50 / 30 Hz，独立 MAVLink 请求目标，实际以接收情况为准 |
| `ATTITUDE_WAIT_US` / `SYNC_SERVICE_US` | 2,000 / 6,000 us，姿态与正常同步服务的短等待上限；初次或失效后同步可等待完整的 30,000 us RTT 窗口 |
| `EXPOSURE_MIN_US` / `EXPOSURE_MAX_US` | 300 / 12,000 us；依据亮度、视觉平移速度与角速度调整 |
| `MAX_BLUR_PX` | 2 px，预测曝光运动模糊预算 |
| `MAX_TILT_DEG` / `MAX_GYRO_RAD_S` / `MIN_IMAGE_STDEV` | 默认 None；依据有效地面交点、分布和实际匹配判定，可手动加额外限制 |

`KEYPOINT_THRESHOLD`、`FEATURE_DETECTOR`、`KEYPOINT_NORMALIZED` 只控制按需 ORB；正常 KLT 不逐帧生成描述子。`KEYPOINT_NORMALIZED=True` 保持 OpenMV 的单尺度模式。运动预测过期时停止速度外推、扩大搜索。

## 图像所有权与内存

| 区域 | 大小与生命周期 |
|---|---|
| 相机当前灰度图 | 76,800 字节 = 75 KiB；只在同步原生调用期间借用 |
| 原生关键帧图像 | 75 KiB，换关键帧时复制，期间保持原模板 |
| 原生前一有效帧图像 | 75 KiB，每次接受视觉解后复制当前帧 |
| 特征、元数据、匹配与窗口工作区 | PC 64 位构建为 4,872 字节，板上以 `rm_vo.CONTEXT_BYTES` 为准 |
| 原生 context 合计 | PC 构建 158,472 字节；无逐帧 C 分配 |
| 姿态、ToF、高度缓存 | 每类固定容量 100 的历史环；Python 元组和 ORB 描述子另计 |

总的三幅图像为 225 KiB，不含相机驱动、IDE 预览、Python 堆与 ORB 内部临时空间。没有完整梯度图。Python 使用复用的 float32 输入/输出数组，64 位时间戳通过两项 uint32 传入，不经浮点转换。

**4.5.9 默认保留单个采集缓冲。**其公开接口没有逐缓冲曝光时间标识，不能用“最新回调时间”去标记可能更旧的排队图像，因此本版没有启用多缓冲采集/计算重叠。`snapshot_return` 与可选的单缓冲 `frame_callback` 记录时刻仍需通过 `CAMERA_TIME_OFFSET_US` 标定到曝光中心；它们不是硬件曝光时间戳。原生参考有独立所有权，相机复用不会覆盖参考。

常规跟踪中不再每秒固定调用 GC；空闲堆低于 `GC_LOW_WATER_BYTES` 时主动回收，MicroPython 自身仍可能在分配时回收。

## 固件构建与部署

本机已通过严格 C 编译、合成图像链路检查，并将模块实际编译进 MicroPython 1.24.0 Windows 运行时，验证原生调用、64 位时间戳、Python 前端和 ORB 候选传递。没有连接 OpenMV，也没有 ARM 固件工具链，**未生成或刷入板上 firmware.bin**。

请在 Linux/WSL 的 ARM 构建环境中，将 OpenMV **v4.5.9** 源码及其对应子模块放在本项目内部；Make 不善于处理带空格的绝对路径，构建时使用无空格的相对路径。下面从本项目目录开始：

```sh
git clone --branch v4.5.9 https://github.com/openmv/openmv.git native/_build/openmv
git -C native/_build/openmv submodule update --init --recursive
python3 -B native/prepare_openmv.py native/_build/openmv
cd native/_build/openmv/src
make -j4 TARGET=OPENMV4P
```

`OPENMV4P` 对应官方 H7 Plus；实际板型需与硬件匹配。准备脚本只允许修改本项目内部的源码树：复制模块到 `src/omv/rm_vo`，利用原构建的模块发现机制，同时将两个目标文件加入 STM32 最终链接。脚本可重复运行。

完成后用 OpenMV IDE 加载所构建的固件。先在 REPL 检查：

```python
import rm_vo
print(rm_vo.ABI_VERSION, rm_vo.CONTEXT_BYTES)
```

应能导入且 ABI 为 1，再将七个 Python 文件复制到板上运行。通用 MicroPython Make / CMake 接入文件也位于 `native/`，供后续移植使用。自研 H7 只需要 `vo_core.c/.h` 与自己的图像、传感器、存储调度；DMA 缓冲需由板级工程正确处理存储区域和缓存一致性。

## 默认输出与飞控融合

```text
状态=跟踪正常（近似） | N=+0.123 E=-0.045 m | 高度=0.803 m | 横滚/俯仰/航向=+1.2/-0.8/+32.5 度 | FPS=19.0 | 质量=35/100
```

N/E 的零点由初始化建立，单位米；质量综合内点数量、比例、空间分布、残差及高度新鲜度。IDE 显示不依赖 `SEND_ODOMETRY`。失跟时显示“位置=无效”。

相机系 C 为图像右/下/光轴前，机体系 B 为前/右/下，位置在 NED。保留标定内参和五项畸变系数，默认安装画面上方对应机头。以下项目仍需实测填写：

```python
CAMERA_POS_B_M = None
TOF_POS_B_M = None
CAMERA_ORIENTATION_CONFIRMED = False
SENSOR_TIMING_CONFIRMED = False
```

允许共点近似时可显示位置，标“近似”、质量上限 35；未确认的安装和时序不发布有效 ODOMETRY。默认 `SEND_ODOMETRY=False`。回传仅融合水平位置，保留时间戳、方差和重置计数；飞控姿态和 ToF 高度不重复包装成独立视觉观测。ODOMETRY 仍为 LOCAL_NED，z 为大方差的有限零占位，姿态及速度为 NaN；飞控融合配置须另行核对。

串口继续默认 UART3 / 115200，使用原有 GPS2 链路配置，不改飞控模式、EKF 参数或解锁状态。输入必须来自近似水平且静止的地面，单点 ToF 不能描述台阶或坡面。

## 必要验证

PC 运行 `python -B native/verify.py`，只依赖标准库和已有 GCC。它编译真实 C 核心，检查亚像素位移及方向/尺度、参考缓冲所有权、关键帧连续性、旋转/高度/臂长补偿、残余偏航与异常匹配、遮挡恢复和 Python 状态重置。构建产物留在 `native/_build`，不访问相机和串口。

板上检查静止、已知 N/E 平移、原地旋转和遮挡恢复；查看实际处理 FPS 与有效定位 Hz。运行 `test.py`，`MODE=1` 输出每秒详细诊断；`MODE=2` 依次采图基线 3 秒、预热 5 秒、采样 15 秒后退出。测试入口始终禁止回传 ODOMETRY，不设目标帧率，也不报告“超预算帧”。原生跟踪、投影与拟合合并计时，避免虚构无法独立测得的子阶段。
