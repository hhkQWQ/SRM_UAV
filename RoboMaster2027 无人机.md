------------------------------------------------------------------------

## 1. 无人机 / 空中机器人定位变化

定位变化：“大尺寸空中火力平台”转变为“小型蜂群无人机”
------------------------------------------------------------------------

## 2. 部署方式

### 2.1 机库
自制机库，每局内最多部署4架小型无人机
最大收纳尺寸限制为300mm*300mm*200mm

**需要重点关注：**

------------------------------------------------------------------------

### 2.2 使用方式

1.无人机局内自动起飞，执行任务，执行后可以落入场地内，也可以飞回机库充电
2.单次起飞续航至多30s（通过规范限制单机最高储能），但可反复降落充电起飞
3.无人机可以在场内任意飞行（不可以飞裁判席）

------------------------------------------------------------------------

## 3. 关键参数
1.重量基准值上限249g，下限150g
2.尺寸基准值直径下限60mm，上限140mm
3.尺寸重量呈现线性限制关系（xy坐标系上两端点分别为60，249和140，151）
4.需要有全包围桨保

------------------------------------------------------------------------

## 4. 思考点
#充电方案
#飞在空中是可以收集战场信息进行回传到裁判系统

------------------------------------------------------------------------

##5. AI推荐项目参考（项目部分）
*1.Crazyswarm — 微型蜂群无人机标杆框架 [github.com/USC-ACTLab/crazyswarm]——最新版(https://imrclab.github.io/crazyswarm2/)
2.EGO-Planner — 浙大FAST Lab开源自主飞行规划器 github.com/ZJU-FAST-Lab/EGO-Planner
3.PX4-Autopilot + MAVROS — 开源飞控与ROS通信桥梁 github.com/PX4/PX4-Autopilot ；github.com/mavlink/mavros
4.VINS-Mono / VINS-Fusion — 港科大视觉惯性SLAM github.com/HKUST-Aerial-Robotics/VINS-Mono ；VINS-Fusion
5.AprilTag 3 — 视觉基准标记系统 github.com/AprilRobotics/apriltag
6.Crazyflie LPS（Loco Positioning System）— UWB室内定位 github.com/bitcraze/lps-node-firmware

##一些其他的开源项目参考
1.https://github.com/rpng/open_vins 对应第四点

------------------------------------------------------------------------

##6. AI推荐项目参考（论文部分）
1.Preiss et al., "Crazyswarm: A Large Nano-Quadcopter Swarm" — ICRA 2017
arXiv（免费获取）：https://arxiv.org/abs/1704.01655
IEEE Xplore：https://ieeexplore.ieee.org/document/7989376
项目主页：https://crazyswarm.readthedocs.io/
2.Zhou et al., "EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors" — RAL 2021
arXiv（免费获取）：https://arxiv.org/abs/2008.08831
IEEE Xplore：https://ieeexplore.ieee.org/document/9346363
开源代码：https://github.com/ZJU-FAST-Lab/EGO-Planner
3.How et al., "Real-Time Indoor Autonomous Vehicle Test Environment" — IEEE Control Systems Magazine 2008（MIT RAVEN项目）
IEEE Xplore：https://ieeexplore.ieee.org/document/4475483
4.基于视觉的精确降落综述：Al-Kaff et al., "Vision-Based Autonomous Landing for Unmanned Aerial Vehicles: A Survey" — 2020
IEEE Xplore（开放获取）：https://ieeexplore.ieee.org/document/9154180
DOI 直链：https://doi.org/10.1109/ACCESS.2020.3014585

------------------------------------------------------------------------

7.AI建议
结合以上项目，针对你提到的 充电方案 和 战场信息回传 两点，补充几个关键思考方向：

1.充电方案
30秒续航 + 反复降落充电，这对充电速率要求极高。以150-249g级无人机，悬停功耗约50-80W，30秒耗能约0.7Wh。若充电时间目标为3-5分钟，充电功率需>15W，触点式充电（如底部金属触点对接到机库充电板）是首选。
可参考 DJI RoboMaster EP 或 RoboMaster机甲大师地面机器人 的接触式充电设计，以及Crazyflie的充电扩展板（Qi无线充电，但功率较低，不适合快充）。
建议自行设计触点式充电：无人机底部装两个弹性触点（正负极），机库上装对应铜片，靠重力/磁吸对准，降落时自动接通。
2.战场信息回传
规则要求飞机在空中收集战场信息回传。结合机载电脑为树莓派，建议：
下视/前视摄像头（如Raspberry Pi Camera Module 3或全局快门相机）采集战场图像；
通过 5.8GHz模拟图传（轻量、低延迟）或 WiFi/4G 回传至上位机；
结合AprilTag识别敌方装甲板/机器人，将目标坐标回传至己方机器人进行协同打击；
具体通信协议可参考 RoboMaster 官方裁判系统的串口协议，将无人机信息接入裁判系统。
3.整体架构建议
飞控层：STM32飞控（PX4/iNav/自研）→ 姿态控制、电机驱动
机载电脑层：树莓派 → VINS?/EGO-Planner?/AprilTag?检测
通信层：MAVLink（树莓派↔飞控）+ 5.8G图传（视频回传）+ UWB（定位）
充电层：底部触点式快速充电（配合机库弹簧铜片）

------------------------------------------------------------------------

##初步思路
1.px4飞控+树莓派电脑
#为后续算法铺路的思考#
2.定位系统
  2.1.考虑UWB室内定位 能否将UWB地面基站和机库进行融合 单位飞机搭载UWB标签 
     锚点固件参考开源项目：https://github.com/bitcraze/lps-node-firmware  上位机工具：https://github.com/bitcraze/crazyflie-lib-python
     无人机端支持：https://github.com/bitcraze/crazyflie-firmware  ROS集成：https://github.com/whoenig/crazyflie_ros
     基于ros的uwb定位节点：https://github.com/mavlink/uwb_localization
     问题在于是使用crazyfile作为飞控 如果要使用px4和ros的结合需要将数据处理放在树莓派上再发给px4
     并且开源工程项目可参考的较少
  2.2.OpenVINS 一个开源的视觉惯性里程计（VIO）框架 https://github.com/rpng/open_vins
     尝试和UWB融合或单独使用？ 能够支持ros2和树莓派
#
3.核心部件
| 模块 | 推荐型号/规格 | 重量（g） | 说明 |
|---|---|---:|---|
| 飞控 | Matek F405-miniTE | 7 | PX4 固件 |
| 四合一电调 | 20A，支持 DShot600，刷 Bluejay/AM32 | 9 | 与飞控叠装 |
| 电机 ×4 | 1104 或 1202.5，2S，8500~9500KV | 24 | 单颗约 6g |
| 螺旋桨 ×4 | 1.5 英寸两叶 PC 桨 | 4 | 每对约 1g |
| 电池 | 2S 550~650mAh 80C LiHV | 38~42 | 可选 550 减重，650 增加续航 |
| 接收机 | ELRS 2.4GHz（CRSF） | 2~3 | 焊接或插头 |
| 机架+桨保 | 碳板中心板 + PA12 分体桨保 | 30~35 | 根据设计可调 |   *可能需要设计落地架 锂聚合物不适合直接着地
| 线材/螺丝/焊接 | AWG20、XT30、M2 等 | 10 | 包含电池扎带等 |
| **合计** |  | **约 150~160g** | 下限 150g，上限 187.75g |


