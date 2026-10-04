# ============================================================
# coordinate_solver.py
# 像素坐标 -> 物理量 解算（纯函数，无任何硬件 / IO 依赖）
#
# 输入：跟踪器输出的稳定中心 (cx, cy)、像平面速度、灯条像素尺寸
# 输出：视线角、距离、角尺寸、机体坐标、状态机编码
#
# 坐标 / 符号约定（联调时必须与飞控核对，否则会反向打舵）：
#   yaw_error   > 0 ：目标在光轴右侧
#   pitch_error > 0 ：目标在光轴下方（图像 y 轴向下）
#   机体坐标 FRD    ：x 前（沿光轴） y 右  z 下
#
# 本模块可以直接在 PC 的 CPython 下 import 做单元测试。
# ============================================================

import math

import config
import camera_params as cam


# ============================================================
# 1. 状态机 / 颜色编码常量
# ============================================================

STATE_LOST = 0        # 完全丢失，无候选
STATE_TRACKING = 1    # 正在稳定跟踪
STATE_TEMP_LOST = 2   # 短暂丢帧，靠历史速度预测
STATE_SUSPECT = 3     # 有可疑新候选，尚未通过连续帧确认

STATE_NAMES = ("LOST", "TRACKING", "TEMP_LOST", "SUSPECT")

COLOR_NONE = 0
COLOR_RED = 1
COLOR_BLUE = 2

COLOR_NAMES = ("NONE", "RED", "BLUE")


# ============================================================
# 2. 像素 -> 归一化坐标（含单点去畸变）
# ============================================================

def undistort_point(u, v, iterations=None):
    """
    对单个像素点做去畸变，返回归一化相机坐标 (x, y)。

    采用不动点迭代反解 OpenCV 畸变模型：
        x_d = x (1 + k1 r^2 + k2 r^4 + k3 r^6) + 切向项
    每次迭代用当前估计的 (x, y) 计算畸变量，再从观测值中扣除。
    10 次迭代即可收敛到远小于 1e-6 的精度。
    """
    if iterations is None:
        iterations = config.UNDISTORT_ITERATIONS

    k1, k2, p1, p2, k3 = cam.DIST_COEFFS

    xd = (u - cam.CX) / cam.FX
    yd = (v - cam.CY) / cam.FY

    x = xd
    y = yd

    for _ in range(iterations):
        r2 = x * x + y * y
        r4 = r2 * r2

        radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r4 * r2

        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y

        x = (xd - dx) / radial
        y = (yd - dy) / radial

    return x, y


def pixel_to_normalized(u, v, undistort=None):
    """
    像素坐标 -> 归一化相机坐标 (x, y)。
        x = (u - cx) / fx
        y = (v - cy) / fy
    undistort=True 时先去畸变。
    """
    if undistort is None:
        undistort = config.UNDISTORT

    if undistort:
        return undistort_point(u, v)

    return (
        (u - cam.CX) / cam.FX,
        (v - cam.CY) / cam.FY,
    )


def pixel_to_angles(u, v, undistort=None):
    """
    像素坐标 -> 视线角 (yaw_error, pitch_error)，单位 rad。
        yaw_error   = atan(x)
        pitch_error = atan(y)
    两个轴独立取反正切，与 PX4 对 LANDING_TARGET.angle_x/angle_y 的解释一致。
    """
    x, y = pixel_to_normalized(u, v, undistort)
    return math.atan(x), math.atan(y)


def radial_scale(u, v, x_n, y_n):
    """
    目标中心处的畸变径向尺度：|畸变坐标| / |无畸变坐标|。
    画面中心为 1.0，边缘约 1.013。
    用于对灯条像素尺寸做一阶修正（尺寸 / 尺度）。
    """
    xd = (u - cam.CX) / cam.FX
    yd = (v - cam.CY) / cam.FY

    ru = math.sqrt(x_n * x_n + y_n * y_n)

    if ru < 1e-6:
        return 1.0

    return math.sqrt(xd * xd + yd * yd) / ru


# ============================================================
# 3. 距离 / 尺寸 / 速度 / 坐标
# ============================================================

def estimate_distance(spacing_px, light_len_px, method=None):
    """
    相似三角形估计深度 Z（沿光轴，单位 m）：
        l = f * L / Z   =>   Z = f * L / l
    method:
        "SPACING"：L = 灯条中心距，l = spacing_px（默认）
        "LENGTH" ：L = 灯条长度，  l = light_len_px
    输入不合法时返回 0.0。
    """
    if method is None:
        method = config.DISTANCE_METHOD

    if method == "LENGTH":
        if light_len_px <= 0:
            return 0.0
        return cam.FY * (config.REAL_LIGHT_LEN_MM * 0.001) / light_len_px

    if spacing_px <= 0:
        return 0.0
    return cam.FX * (config.REAL_LIGHT_SPACING_MM * 0.001) / spacing_px


def estimate_angular_size(spacing_px, light_len_px):
    """
    目标在画面中的角尺寸 (size_x, size_y)，单位 rad。
        size = 2 * atan(0.5 * l / f)
    """
    return (
        2.0 * math.atan(0.5 * spacing_px / cam.FX),
        2.0 * math.atan(0.5 * light_len_px / cam.FY),
    )


def los_rates(vx_px_ms, vy_px_ms, x_n=0.0, y_n=0.0):
    """
    像平面速度 (pixel/ms) -> 视线角速度 (rad/s)。
        theta = atan(x)  =>  d(theta)/dt = (dx/dt) / (1 + x^2)
    注意：这是目标方向相对相机的变化率，
          不是相机自身角速度，也不是无人机线速度。
    """
    yaw_rate = (vx_px_ms * 1000.0 / cam.FX) / (1.0 + x_n * x_n)
    pitch_rate = (vy_px_ms * 1000.0 / cam.FY) / (1.0 + y_n * y_n)
    return yaw_rate, pitch_rate


def body_coords(depth, yaw, pitch):
    """
    深度 + 视线角 -> 机体 FRD 坐标 (x, y, z)，单位 m。
        x = Z（前）  y = Z * tan(yaw)（右）  z = Z * tan(pitch)（下）
    """
    return (
        depth,
        depth * math.tan(yaw),
        depth * math.tan(pitch),
    )


# ============================================================
# 4. 状态机 / 颜色编码
# ============================================================

def map_state(stable_valid, lost_count, pending_count):
    """
    由 MotionTracker 的三个量映射到状态机：
        已锁定 且 未丢帧        -> TRACKING
        已锁定 但 正在丢帧预测  -> TEMP_LOST
        未锁定 但 有候选在确认  -> SUSPECT
        未锁定 且 无候选        -> LOST
    """
    if stable_valid:
        if lost_count == 0:
            return STATE_TRACKING
        return STATE_TEMP_LOST

    if pending_count > 0:
        return STATE_SUSPECT

    return STATE_LOST


def color_code(color_name):
    """ "RED" -> 1, "BLUE" -> 2, 其他 -> 0 """
    if color_name == "RED":
        return COLOR_RED
    if color_name == "BLUE":
        return COLOR_BLUE
    return COLOR_NONE


def encode_target_num(state, color):
    """
    把状态机与颜色打包进 LANDING_TARGET.target_num（uint8）：
        bit[3:2] = state    bit[1:0] = color
    """
    return ((state & 0x03) << 2) | (color & 0x03)


def decode_target_num(target_num):
    """ 返回 (state, color) """
    return (target_num >> 2) & 0x03, target_num & 0x03


# ============================================================
# 5. 带状态的解算器
# ============================================================

class TargetSolver:
    """
    持有距离低通与"最近一次可信的灯条像素尺寸"。

    每帧调用 solve()，返回统一 dict，供 comm 打包发送。
    灯条尺寸只在 TRACKING 状态下采信（此时检测与稳定目标匹配），
    TEMP_LOST 时沿用上一次，LOST 时清零。
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.distance = 0.0
        self.has_distance = False

        self.last_spacing_px = 0.0
        self.last_light_len_px = 0.0

    def _empty_output(self, state, color_name):
        color = color_code(color_name)
        return {
            "valid": False,
            "state": state,
            "color": color,
            "target_num": encode_target_num(state, color),

            "cx": 0,
            "cy": 0,

            "yaw_error": 0.0,
            "pitch_error": 0.0,

            "distance": 0.0,     # 深度 Z（沿光轴）
            "range": 0.0,        # 欧氏距离

            "size_x": 0.0,
            "size_y": 0.0,

            "x": 0.0,
            "y": 0.0,
            "z": 0.0,

            "yaw_rate": 0.0,
            "pitch_rate": 0.0,
        }

    def solve(
        self,
        valid,
        state,
        color_name,
        cx,
        cy,
        vx_px_ms=0.0,
        vy_px_ms=0.0,
        spacing_px=None,
        light_len_px=None,
    ):
        """
        参数：
            valid / state / color_name：跟踪器结果与 map_state() 输出
            cx, cy                    ：跟踪器输出的稳定中心（pixel）
            vx_px_ms, vy_px_ms        ：跟踪器像平面速度（pixel/ms）
            spacing_px, light_len_px  ：当前帧原始检测的灯条尺寸，可为 None
        返回：
            统一输出 dict（字段见 _empty_output）
        """
        out = self._empty_output(state, color_name)

        if not valid:
            if state == STATE_LOST:
                self.reset()
            return out

        out["valid"] = True
        out["cx"] = cx
        out["cy"] = cy

        # ---------- 角度 ----------
        x_n, y_n = pixel_to_normalized(cx, cy)

        yaw = math.atan(x_n)
        pitch = math.atan(y_n)

        out["yaw_error"] = yaw
        out["pitch_error"] = pitch

        # ---------- 尺寸采信 + 距离低通 ----------
        if (
            state == STATE_TRACKING and
            spacing_px is not None and
            light_len_px is not None and
            spacing_px > 0
        ):
            if config.UNDISTORT:
                scale = radial_scale(cx, cy, x_n, y_n)
                spacing_px = spacing_px / scale
                light_len_px = light_len_px / scale

            self.last_spacing_px = spacing_px
            self.last_light_len_px = light_len_px

            d_new = estimate_distance(spacing_px, light_len_px)

            if d_new > 0:
                if self.has_distance:
                    a = config.DISTANCE_EMA_ALPHA
                    self.distance = (1.0 - a) * self.distance + a * d_new
                else:
                    self.distance = d_new
                    self.has_distance = True

        depth = self.distance if self.has_distance else 0.0

        out["distance"] = depth
        out["range"] = depth * math.sqrt(1.0 + x_n * x_n + y_n * y_n)

        # ---------- 角尺寸 ----------
        size_x, size_y = estimate_angular_size(
            self.last_spacing_px,
            self.last_light_len_px,
        )
        out["size_x"] = size_x
        out["size_y"] = size_y

        # ---------- 机体坐标 ----------
        bx, by, bz = body_coords(depth, yaw, pitch)
        out["x"] = bx
        out["y"] = by
        out["z"] = bz

        # ---------- 视线角速度 ----------
        yaw_rate, pitch_rate = los_rates(vx_px_ms, vy_px_ms, x_n, y_n)
        out["yaw_rate"] = yaw_rate
        out["pitch_rate"] = pitch_rate

        return out
