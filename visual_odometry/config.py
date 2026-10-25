"""集中参数。OpenMV 4.5.9 / OV5640 / Pixhawk 6C Mini / PX4 1.17。

机体B: 前、右、下；相机C: 图像右、图像下、光轴前；位置: 固定NED。
先调试再融合。None表示未知，绝不表示已经标定的零。
"""

# ---------- 标定：camera calibration/calibration_output/camera_calibration.json ----------
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
FX = 302.11830698360654
FY = 302.03599395976175
CX = 163.3557244238438
CY = 117.66487385855488
DIST_COEFFS = (0.09984656079919686, -0.23986524630434,
               -0.0007887596884331264, 0.00012132382106465136,
               0.16915439320944928)

# 下视，画面上方=机头、画面右方=机体右。R_BC将光学向量变换到机体系。
R_BC = (0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
CAMERA_ORIENTATION_CONFIRMED = False  # 实际方向确认后改True
CAMERA_POS_B_M = None                # 待填 (前, 右, 下)，单位m
TOF_POS_B_M = None                   # 待填 (前, 右, 下)，单位m
TOF_AXIS_B = (0.0, 0.0, 1.0)
ALLOW_APPROXIMATE_EXTRINSICS = True   # 共点近似：输出标“近似”，不回传有效ODOMETRY

# ---------- 采图与时间 ----------
EXPOSURE_US = 6000                   # 启动曝光；随后按亮度和预测像素速度调节
FRAME_BUFFERS = 1                    # 4.5.9无逐缓冲时间戳接口，默认单缓冲可靠对应图像
GAIN_DB = 18.0                       # 启动增益；软件调节关闭时保持此值，None使用相机自动增益
GAIN_CEILING_DB = 36.0               # 自动增益上限；暗处会增加噪声，按实测降低
CAMERA_AUTO_TUNE = True              # 每0.5秒按亮度/角速度调节；开启后接管相机自动曝光/增益
EXPOSURE_MIN_US = 300
EXPOSURE_MAX_US = 12000
MAX_BLUR_PX = 2.0                    # 预测平移+旋转运动允许的曝光模糊
GC_LOW_WATER_BYTES = 65536           # 空闲堆不足时回收；不再每秒固定GC
CAMERA_SETTLE_MS = 2000              # 自动曝光/增益收敛，保持相机面对测试地面
AUTOFOCUS_ON_START = True            # 与标定采集流程一致；运行中不连续自动对焦
HMIRROR = False
VFLIP = False
CAMERA_TIMESTAMP_MODE = "snapshot_return"  # 可选frame_callback；均需实测曝光偏差
CAMERA_TIME_OFFSET_US = 0            # 曝光时刻 = 记录时刻 + 此偏移
ATTITUDE_TIME_OFFSET_US = 0          # 实际时刻 = 消息时间 + 此偏移
RANGE_TIME_OFFSET_US = 0
SENSOR_TIMING_CONFIRMED = False      # 曝光/传感器延迟标定后改True

# ---------- 通信：GPS2端口需已配置为MAVLink，两个设备波特率一致 ----------
UART_ID = 3                          # 常见STM32 OpenMV: P4 TX / P5 RX，按实际板核对
BAUDRATE = 115200                    # 性能不足可两端一致提高至460800
SOURCE_COMPONENT = 191
TARGET_SYSTEM = None                # None: 从PX4 HEARTBEAT发现
TARGET_COMPONENT = 1
STREAM_POLICY = "auto"             # auto先复用现有流，缺失/不足才请求；listen只接收；request主动请求
ATTITUDE_HZ = 50                    # 独立请求频率，和视觉处理频率无关
RANGE_HZ = 30
REQUEST_VIBRATION = True
VIBRATION_HZ = 5
HEARTBEAT_TIMEOUT_US = 3000000
COMMAND_TIMEOUT_US = 1500000
SYNC_INTERVAL_US = 200000
SYNC_MIN_SAMPLES = 8
SYNC_MAX_RTT_US = 30000
SYNC_MAX_JITTER_US = 5000
SYNC_TIMEOUT_US = 3000000
SENSOR_HISTORY_SIZE = 100
RX_BYTES_PER_POLL = 2048
MAX_TX_AGE_US = 150000
TOF_SENSOR_ID = None                 # 多个下视测距器时必须手动指定，不能只凭朝向区分
TOF_ORIENTATION = 25                 # MAV_SENSOR_ROTATION_PITCH_270: 向下

# ---------- 数据健康 ----------
MAX_ATTITUDE_EXTRAPOLATION_US = 40000
ATTITUDE_WAIT_US = 2000              # 先使用缓存/有限外推，缺数据时最多短暂等待
SYNC_SERVICE_US = 6000               # 正常同步短等待；初次同步使用完整RTT窗口
MAX_ATTITUDE_GAP_US = 100000
MAX_RANGE_AGE_US = 100000
MIN_RANGE_M = None                  # None采用DISTANCE_SENSOR报告的量程；可填软件额外限制
MAX_RANGE_M = None
MAX_TILT_DEG = None                  # 可选额外限制；默认依据有效地面交点与匹配判断
MAX_GYRO_RAD_S = None                # 可选额外限制；不因固定转速门槛跳过视觉
MIN_IMAGE_STDEV = None               # 可选纹理预筛；默认依据二维特征与几何判断
RANGE_NOISE_M = 0.03
RANGE_RELATIVE_NOISE = 0.015
RANGE_JUMP_BASE_M = 0.12
MAX_VERTICAL_SPEED_M_S = 3.0
HEIGHT_RECOVERY_SAMPLES = 3
POSE_RECOVERY_FRAMES = 3
VIBRATION_LIMITS = None              # 可填PX4三项指标阈值；需由日志定，不能套用其它固件单位
CLIPPING_HOLD_US = 500000
ATTITUDE_RESET_MARGIN_RAD = 0.20

# ---------- 原生FAST / 稀疏KLT / 姿态高度辅助二维平移 ----------
MAX_KEYPOINTS = 24                 # 正常目标24；原生容量64，最低2个可靠点可继续跟踪
FAST_THRESHOLD = 15               # OpenMV v4.5.9 FAST-9灰度差阈值，无ORB描述子
FEATURE_TOPUP_RATIO = 0.75         # 少于目标的75%才补点；24点目标对应低于18点
FEATURE_TOPUP_CELLS = 6            # 每帧最多扫描缺点网格数，轮转覆盖6x4网格
FEATURE_TOPUP_POINTS = 4           # 每帧最多新增4点；首张初始化图可全网格检测
KLT_MAX_ITERATIONS = 12            # 模板梯度/矩阵预计算一次，保留收敛预算
KLT_MAX_ERROR = 15.0               # 去亮度偏移后的8x8 RMS灰度误差
KLT_MIN_EIGENVALUE = 4.0           # 两个方向均需具备可跟踪纹理
KLT_FB_ERROR_PX = 1.5
SEARCH_MIN_PX = 6
SEARCH_MAX_PX = 8                  # 不再扩大到40px或反复换参考
SAD_UNIQUENESS_RATIO = 0.90        # 最佳代价必须低于另一独立峰的90%
KEYFRAME_CHECK_INTERVAL = 5        # 每5帧选择关键帧；每帧仅一轮完整跟踪
TEMPLATE_WARP_ANGLE_RAD = 0.0174533 # 相对旋转超过约1度时重采样仿射模板
TEMPLATE_WARP_HEIGHT_RATIO = 0.02  # 相对高度变化超过2%时重采样
TRACK_HOLD_US = 1000000            # us；距最后真实匹配<=1s时预测位置，调大可容忍更长遮挡但漂移增大
LOST_RESET_US = 3000000            # us；距最后真实匹配>=3s时重建原点，必须大于TRACK_HOLD_US
PREDICTION_HISTORY_SIZE = 5        # 最近真实成功结果数量，用实际时间拟合匀速；预测结果不入历史
PREDICTION_VELOCITY_STD_M_S = 0.5  # m/s；速度不确定度下限，调大使预测方差增长更快、置信度更低
MIN_MATCHES = 2
MIN_INLIERS = 2
MIN_INLIER_RATIO = 0.55            # 同时要求严格多数；双点必须全部一致
SPARSE_MAX_INLIERS = 3             # 2/3内点标记少点状态，使用统一置信度公式
TWO_POINT_VARIANCE_FACTOR = 4.0    # 放大随机测量项，公共误差保持独立下限
THREE_POINT_VARIANCE_FACTOR = 2.0
MIN_RAY_DOWN = 0.25
TRANSLATION_GATE_PX = 2.5          # px；平移一致性重投影门槛，调大更宽松，不执行RANSAC
KEYFRAME_DISTANCE_HEIGHT_RATIO = 0.25
KEYFRAME_HEIGHT_RATIO = 1.25
KEYFRAME_ROTATION_RAD = 0.25
KEYFRAME_REFRESH_INLIER_RATIO = 0.65
POSITION_NOISE_FLOOR_M = 0.02
ATTITUDE_STD_RAD = 0.00873
PIXEL_STD = 0.8

# ---------- 唯一置信度：70%方差 + 10%点数 + 10%覆盖 + 10%匹配质量 ----------
CONFIDENCE_SIGMA0_M = 0.05         # m；位置标准差达到此值时方差项得半分，调大使同等误差评分更高
CONFIDENCE_FULL_POINTS = 16        # 可靠内点达到16即点数项满分；超过16不继续加分
CONFIDENCE_FULL_CELLS = 6          # 3x3网格覆盖6格即覆盖项满分，不再叠乘分散程度惩罚

# ---------- 输出：只有水平位置融合 ----------
SEND_ODOMETRY = False               # 旁路检查后显式开启；不会修改飞控任何EKF参数
ODOMETRY_HZ = 30
MIN_PUBLISH_QUALITY = 40           # 0~100唯一置信度的发布门槛；预测位置不作为新视觉测量回传
Z_PLACEHOLDER_VARIANCE = 1000000.0
DISPLAY_HZ = 2                      # 中文状态摘要频率，不影响运算/回传频率

