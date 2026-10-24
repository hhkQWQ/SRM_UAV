# 参考与实现边界

本目录是为当前 QVGA、姿态和 ToF 辅助条件新写的单核实现，除下述OpenMV FAST判定树外，没有复制以下项目的源文件，也没有引入它们的 GAP9、多核、完整 VIO 或位姿图后端。工程现有许可不在此次修改中变更。

| 来源 | 参考入口 | 此实现采用的内容 |
|---|---|---|
| [Downfacing-VIO](https://github.com/ETH-PBL/Downfacing-VIO) | `utility/rigid_body_motion/`；[论文 III](https://arxiv.org/html/2509.10021v1#S3) | 平移与偏航分解、先剔除再拟合再筛选。此处由飞控姿态提供旋转，按高度投影到地面后做确定性二维平移一致性筛选，不引入视觉偏航拟合或RANSAC |
| [PX4FLOW](https://github.com/PX4/PX4-Flow/blob/master/src/modules/flow/flow.c) | `compute_sad_8x8()`、`compute_subpixel()`、DSP SAD 指令 | 自行实现 8×8 SAD 与 M7 `USADA8` 分支，用 KLT 完成亚像素精化；行宽显式传递、地址采用 `size_t`，适配 76,800 字节图像 |
| [EdgeFlow](https://github.com/paparazzi/paparazzi/blob/master/sw/airborne/modules/computer_vision/lib/vision/edge_flow.c) | `calc_previous_frame_nr()` | 借鉴自适应时间间隔思想：小运动优先固定关键帧，相邻有效帧连续跟踪、每五帧选择固定关键帧；每帧最多一轮完整跟踪；没有复制 GPL 边缘直方图前端代码 |
| [LEVIO vo.c](https://github.com/ETH-PBL/levio/blob/main/levio_gap9_project/visual_odometry/vo.c)、[pose_graph.c](https://github.com/ETH-PBL/levio/blob/main/levio_gap9_project/definitions/pose_graph.c) | `allocate_work_memory()`、`initialize_data_storage()`、`storage_move_in/out()` | 持久图像/特征区与阶段工作区分开；固定容量、按需复用，无逐帧 C 堆分配。没有复制 GAP9 DMA 或存储调度实现 |

`vo_fast.c`引入OpenMV **v4.5.9** [`src/omv/imlib/fast.c`](https://github.com/openmv/openmv/blob/v4.5.9/src/omv/imlib/fast.c) 中Edward Rosten生成的FAST-9判定树，保留源文件完整BSD许可。文件头记录原文件SHA256。适配只提取逐像素判定；删除图像对象、帧缓冲分配、关键点对象及描述子包装，使用调用者提供的偏移和固定容量候选。打分与间距筛选属于本项目。PC和板端编译同一份判定实现，不依赖Python的find_keypoints或ORB，便于数值回归。

逆组合KLT的梯度/矩阵复用参考[Baker与Matthews的统一框架](https://www.ri.cmu.edu/pub_files/pub3/baker_simon_2004_1/baker_simon_2004_1.pdf)。局部仿射模板、固定预算网格补点、受限搜索、地面平移模型及其少点质量/方差策略是本项目实现；没有复制上述论文的源码。

固件接入依据：[MicroPython 外部 C 模块](https://docs.openmv.io/dev/develop/cmodules.html)、[OpenMV v4.5.9 图像缓冲协议](https://github.com/openmv/openmv/blob/v4.5.9/src/omv/modules/py_image.c)、[4.5.9 STM32 构建与链接配置](https://github.com/openmv/openmv/blob/v4.5.9/src/omv/ports/stm32/omv_portconfig.mk)。`prepare_openmv.py` 同时接入编译和最终链接，避免只有模块注册但缺少目标文件。
