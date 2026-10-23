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
UNDISTORT_ITERATIONS = 6

# 下视，画面上方=机头、画面右方=机体右。R_BC将光学向量变换到机体系。
R_BC = (0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
CAMERA_ORIENTATION_CONFIRMED = False  # 实际方向确认后改True
CAMERA_POS_B_M = None                # 待填 (前, 右, 下)，单位m
TOF_POS_B_M = None                   # 待填 (前, 右, 下)，单位m
TOF_AXIS_B = (0.0, 0.0, 1.0)
ALLOW_APPROXIMATE_EXTRINSICS = True   # 共点近似：输出标“近似”，不回传有效ODOMETRY
APPROXIMATE_QUALITY_CAP = 35

# ---------- 采图与时间 ----------
EXPOSURE_US = 20000                  # 启动曝光；软件调节关闭时保持此值，None使用相机自动曝光
FRAME_BUFFERS = 1                    # 单缓冲便于将时间戳与snapshot图像对应
GAIN_DB = 18.0                       # 启动增益；软件调节关闭时保持此值，None使用相机自动增益
GAIN_CEILING_DB = 36.0               # 自动增益上限；暗处会增加噪声，按实测降低
CAMERA_AUTO_TUNE = True              # 每0.5秒按亮度/角速度调节；开启后接管相机自动曝光/增益
TARGET_FPS = 30                     # 计算预算目标，实际FPS由实机统计
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
ATTITUDE_HZ = None                  # None按TARGET_FPS产生请求目标；实际时间戳/频率始终取接收数据
RANGE_HZ = None
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
ATTITUDE_WAIT_US = 40000             # 采图后等待匹配姿态，仍使用原插值/外推有效性门槛
MAX_ATTITUDE_GAP_US = 100000
MAX_RANGE_AGE_US = 100000
MIN_RANGE_M = None                  # None采用DISTANCE_SENSOR报告的量程；可填软件额外限制
MAX_RANGE_M = None
MAX_TILT_DEG = 35.0
MAX_GYRO_RAD_S = 3.0
MIN_IMAGE_STDEV = 7.0
RANGE_NOISE_M = 0.03
RANGE_RELATIVE_NOISE = 0.015
RANGE_JUMP_BASE_M = 0.12
MAX_VERTICAL_SPEED_M_S = 3.0
HEIGHT_RECOVERY_SAMPLES = 3
POSE_RECOVERY_FRAMES = 3
VIBRATION_LIMITS = None              # 可填PX4三项指标阈值；需由日志定，不能套用其它固件单位
CLIPPING_HOLD_US = 500000
ATTITUDE_RESET_MARGIN_RAD = 0.20

# ---------- AGAST / ORB / 几何 / RANSAC ----------
FEATURE_SCALE = 0.5                 # QVGA采图，原生AREA缩至160x120进行特征提取
FEATURE_ADAPTIVE = True
FEATURE_DETECTOR = "auto"           # 先FAST，连续缺点时尝试AGAST；可固定FAST或AGAST
MAX_KEYPOINTS = 48                  # 自动计算预算的上限，最少24；不反复重跑同一帧
KEYPOINT_THRESHOLD = 20
KEYPOINT_NORMALIZED = True          # True=优先单尺度；相对关键帧高度变化大时自动使用多尺度
KEYPOINT_SCALE_FACTOR = 1.5
MATCH_THRESHOLD = 75
MIN_MATCHES = 8
MIN_INLIERS = 6
MIN_INLIER_RATIO = 0.55
MIN_OCCUPIED_CELLS = 3              # 3x3网格中的覆盖数量
MIN_RAY_DOWN = 0.25
RANSAC_THRESHOLD_PX = 2.5
RANSAC_MAX_ITERATIONS = 16
RANSAC_MIN_ITERATIONS = 6
RANSAC_CONFIDENCE = 0.995
KEYFRAME_DISTANCE_HEIGHT_RATIO = 0.25
KEYFRAME_HEIGHT_RATIO = 1.25
KEYFRAME_ROTATION_RAD = 0.25
KEYFRAME_REFRESH_INLIER_RATIO = 0.65
POSITION_NOISE_FLOOR_M = 0.02
ATTITUDE_STD_RAD = 0.00873           # 初始保守估计约0.5度，按实测调整
PIXEL_STD = 0.8

# ---------- 输出：只有水平位置融合 ----------
SEND_ODOMETRY = False               # 旁路检查后显式开启；不会修改飞控任何EKF参数
ODOMETRY_HZ = 30
MIN_PUBLISH_QUALITY = 40
Z_PLACEHOLDER_VARIANCE = 1000000.0
DISPLAY_HZ = 2                      # 中文状态摘要频率，不影响运算/回传频率

