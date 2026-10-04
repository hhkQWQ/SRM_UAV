# ============================================================
# config.py
# 全项目可调参数集中管理
#
# 现场标定时只需要改这个文件；
# 相机内参请改 camera_params.py（由标定结果生成）。
# ============================================================


# ============================================================
# 1. Debug 开关
# ============================================================

# True：显示原始 blob 黄色 / 青色框
DEBUG_DRAW_RAW_BLOBS = False

# True：显示通过灯条粗筛后的红 / 蓝框
DEBUG_DRAW_LIGHTS = False

# True：显示跟踪器输出的最终目标中心
DEBUG_DRAW_PAIR = True

# True：周期性打印检测 / 跟踪 / 解算调试信息
DEBUG_PRINT = False

# True：把即将发送的 MAVLink 帧以十六进制打印到 IDE 串口
DEBUG_PRINT_HEX_FRAME = False

# 调试打印间隔（ms）
DEBUG_PRINT_INTERVAL_MS = 5000


# ============================================================
# 2. 颜色阈值
# ============================================================

# OpenMV IDE 阈值编辑器格式：(L_min, L_max, A_min, A_max, B_min, B_max)
# L：亮度   A：负值偏绿，正值偏红   B：负值偏蓝，正值偏黄
RED_THRESHOLD = (10, 100, 20, 127, -30, 127)
BLUE_THRESHOLD = (36, 100, -60, 80, -128, 0)


# ============================================================
# 3. 灯条粗筛参数
# ============================================================

# 远距离灯条像素很少，所以这里必须放宽
MIN_PIXELS = 3

# 灯条长边至少几个像素
MIN_LIGHT_LEN = 3

# 只用于排除特别明显的块状区域（圆形、方形光斑）
MIN_ELONGATION = 0.30


# ============================================================
# 4. 灯条配对硬约束
# ============================================================

# 两灯条最大允许角度差
MAX_ANGLE_DIFF_DEG = 35.0

# 两灯条最大长度差比例
MAX_LENGTH_DIFF_RATIO = 0.50

# 两灯条沿灯条方向最大错位
MAX_PARALLEL_RATIO = 1.50

# 两灯条法线方向最小 / 最大间距
MIN_NORMAL_RATIO = 1.0
MAX_NORMAL_RATIO = 3.0


# ============================================================
# 5. 几何评分参数
# ============================================================

# 理想情况下：两灯条间距 / 平均灯条长度
IDEAL_NORMAL_RATIO = 2.4

# 最低装甲板几何得分（低于它即使是最高分也不输出）
MIN_PAIR_SCORE = 0.55


# ============================================================
# 6. 跟踪器参数
#
# 用法：MotionTracker(**config.TRACKER_PARAMS)
# 各参数含义见 armor_tracker.py 与
# "OpenMV装甲板独立跟踪与运动数据说明.md"
# ============================================================

TRACKER_PARAMS = {
    # 新目标连续 3 帧运动一致才正式锁定
    "confirm_frames": 3,

    # 已锁定目标最多允许连续 4 帧没有正常匹配
    "max_lost_frames": 4,

    # 初次捕获阶段（还没有可靠速度，门限放宽）
    "acquire_base_gate": 35.0,
    "acquire_size_factor": 1.20,
    "acquire_speed_factor": 0.80,
    "acquire_max_gate": 110.0,

    # 正式跟踪阶段
    "track_base_gate": 10.0,
    "track_size_factor": 0.60,
    "track_speed_factor": 0.90,
    "track_max_gate": 100.0,

    # Alpha-Beta 滤波器
    "track_alpha": 0.70,
    "track_beta": 0.20,

    # 候选速度平滑
    "pending_velocity_alpha": 0.60,

    # 丢失时速度衰减
    "lost_velocity_decay": 0.92,

    # 单次预测最大 dt
    "track_max_dt_ms": 150,
}


# ============================================================
# 7. 装甲板真实尺寸（距离解算用）
#
# !! 当前是估计值，必须用已知距离实测标定后修正 !!
#
# 标定方法：
#   把装甲板放在已知距离 D（米），读出画面中的 spacing_px，
#   则 REAL_LIGHT_SPACING_MM = spacing_px * D * 1000 / FX
# ============================================================

# 两根灯条中心之间的真实距离（mm）
REAL_LIGHT_SPACING_MM = 135.0

# 单根灯条的真实长度（mm）
REAL_LIGHT_LEN_MM = 55.0

# 距离解算方法：
#   "SPACING"：用两灯条中心距（受目标偏航影响，默认）
#   "LENGTH" ：用灯条长度（受目标俯仰影响）
DISTANCE_METHOD = "SPACING"

# 距离低通系数（0~1，越大越相信新测量）
DISTANCE_EMA_ALPHA = 0.30


# ============================================================
# 8. 镜头畸变
# ============================================================

# True：对目标中心点做单点去畸变（开销可忽略）
#       同时用中心处的径向尺度一阶修正灯条像素尺寸
UNDISTORT = True

# 迭代去畸变的迭代次数
UNDISTORT_ITERATIONS = 10


# ============================================================
# 9. 通信（MAVLink v2 over UART）
# ============================================================

# !! 需按 OpenMV Pro Plus 实际引脚排布核对 !!
UART_PORT = 3
UART_BAUD = 115200

# MAVLink 身份
MAV_SYSTEM_ID = 1
MAV_COMPONENT_ID = 100        # MAV_COMP_ID_CAMERA

# LANDING_TARGET 发送间隔下限（ms）。0 表示每帧都发
SEND_INTERVAL_MS = 0

# True：目标丢失时也发送 LANDING_TARGET（position_valid=0，
#       target_num 携带 LOST 状态），让下位机知道目标消失。
# 注意：标准 PX4 精准降落会把任何 LANDING_TARGET 当作有效检测，
#       接 PX4 原生功能时应改为 False。
SEND_WHEN_LOST = True

# True：周期性发送 HEARTBEAT，方便 QGC / Mission Planner 自动识别
SEND_HEARTBEAT = True
HEARTBEAT_INTERVAL_MS = 1000

# True：裁掉载荷末尾的 0 字节（MAVLink v2 标准截断，省带宽）
TRIM_PAYLOAD = False

# LANDING_TARGET.frame：MAV_FRAME_BODY_FRD = 12
# 与坐标解算输出的 x 前 / y 右 / z 下 一致
LANDING_TARGET_FRAME = 12

# LANDING_TARGET.type：LANDING_TARGET_TYPE_VISION_OTHER = 3
LANDING_TARGET_TYPE = 3


# ============================================================
# 10. 摄像头
# ============================================================

# 关闭自动曝光后的固定曝光时间（us）
EXPOSURE_US = 2500
