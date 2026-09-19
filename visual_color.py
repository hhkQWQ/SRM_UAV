# ============================================================
# 1. 导入库
# ============================================================
import sensor
import time
import math
# ============================================================
# 2. 参数配置区
#    所有需要调节的参数
# ============================================================
# ---------- Debug 开关 ----------
# True：显示原始 blob 黄色框
DEBUG_DRAW_RAW_BLOBS = False
# True：显示通过灯条筛选后的红框
DEBUG_DRAW_LIGHTS = False
# True：显示最终装甲板中心位置
DEBUG_DRAW_PAIR = True
# True：串口打印调试信息
DEBUG_PRINT = False

# ---------- 红色 LAB 阈值 ----------
# 使用 OpenMV IDE 的阈值编辑器调整
# OpenMV IDE阈值编辑器格式：(L_min, L_max, A_min, A_max, B_min, B_max)
# 大致理解为：L：亮度  A：负值偏绿，正值偏红  B：负值偏蓝，正值偏黄
RED_THRESHOLD = (40, 100, 20, 127, 20, 127)
BLUE_THRESHOLD = (36, 100, -60, 80, -128, 15)

# ---------- 灯条粗筛参数 ----------
# 远距离灯条像素很少，所以这里必须放宽
MIN_PIXELS = 5
# 灯条长边至少几个像素
MIN_LIGHT_LEN = 3
# 只用于排除特别明显的块状区域
# 不再要求灯条必须非常细长
MIN_ELONGATION = 0.30

# ---------- 灯条配对硬约束 ----------
# 两灯条最大允许角度差
MAX_ANGLE_DIFF_DEG = 35.0
# 两灯条最大长度差比例
MAX_LENGTH_DIFF_RATIO = 0.75
# 两灯条沿灯条方向最大错位
MAX_PARALLEL_RATIO = 1.50
# 两灯条法线方向最小间距
MIN_NORMAL_RATIO = 0.40
# 两灯条法线方向最大间距
MAX_NORMAL_RATIO = 5.50

# ---------- 几何评分参数 ----------
# 理想情况下：
# 两灯条间距 / 平均灯条长度
# 后续建议根据实际装甲板进行标定
IDEAL_NORMAL_RATIO = 2.4
# 最低装甲板几何得分
# 低于这个值即使是所有组合中的最高分也不输出
MIN_PAIR_SCORE = 0.45

# ============================================================
# 3. 摄像头初始化
# ============================================================

sensor.reset()
# 使用 RGB565，因为后续需要进行颜色识别
sensor.set_pixformat(sensor.RGB565)
# QVGA = 320 x 240
sensor.set_framesize(sensor.QVGA)
# 等待摄像头自动调整稳定
sensor.skip_frames(time=2000)
# 关闭自动增益，避免画面亮度不断变化
sensor.set_auto_gain(False)
# 关闭自动白平衡，避免红色阈值漂移
sensor.set_auto_whitebal(False)
# 创建 FPS 计时器
clock = time.clock()
# 关闭自动曝光并设置曝光时间
sensor.set_auto_exposure(False, exposure_us=2000)


# ============================================================
# 4. 工具函数
# ============================================================

def rad_to_deg(rad):
    """
    OpenMV blob.rotation() 返回弧度。
    将弧度转换为角度。
    """
    return rad * 180.0 / math.pi


def angle_diff_deg(a, b):
    """
    计算两个灯条方向的角度差。
    灯条方向具有 180° 对称性：
    例如 10° 与 170° 实际相差的是20°
    因此不能直接 abs(a-b)。
    """
    # 将差值限制到 0~180
    diff = abs(a - b) % 180.0
    # 方向具有 180° 对称性
    if diff > 90:
        diff = 180 - diff
    return diff

# ============================================================
# 5. 灯条粗筛选
# ============================================================


def is_light_strip(blob):

    # 对灯条识别目标进行初筛选，真正的装甲板判断交给后面的几何评分。

    # --------------------------------------------------------
    # 条件 1：像素数量
    # --------------------------------------------------------
    # 只过滤特别小的噪声，排除单个像素等等被误识别为灯条
    # 远距离灯条可能只有很少像素，因此不能设置太高
    if blob.pixels() < MIN_PIXELS:
        return False
    # --------------------------------------------------------
    # 条件 2：长边尺寸
    # --------------------------------------------------------
    # 判断灯条尺寸需要具有一定像素大小
    # 因为灯条发生旋转以后长短边会发生改变
    # 因此使用长边可以提高旋转情况下的稳定性
    long_side = max(
        blob.w(),
        blob.h()
    )
    if long_side < MIN_LIGHT_LEN:
        return False
    # --------------------------------------------------------
    # 条件 3：极宽松 elongation
    # --------------------------------------------------------
    # 排除不是明显长条形的识别结果（圆形、方形块状光斑）
    # elongation=1-min(w,h)/max(w,h)
    if blob.elongation() < MIN_ELONGATION:
        return False
    return True


def detect_lights(img):
    """
    通过一次 find_blobs 同时检测红色和蓝色灯条。
    返回：red_lights/blue_lights
    同时预计算灯条长度 length，供 calc_pair_score() 重复使用。
    """
    blobs = img.find_blobs(
        [RED_THRESHOLD, BLUE_THRESHOLD],     # 颜色阈值列表
        x_stride=1,                          # 全分辨率扫描x轴步长
        y_stride=1,                          # 全分辨率扫描y轴步长
        area_threshold=MIN_PIXELS,           # 面积阈值，只保留外接矩形面积大于等于该值的色块
        pixels_threshold=MIN_PIXELS,         # 像素数阈值，只保留实际包含像素数量大于等于该值的色块
        merge=False                          # 表示合并相邻色块，false表示否
    )
    red_lights = []
    blue_lights = []
    for blob in blobs:
        # ====================================================
        # Debug：原始 blob
        # ====================================================
        code = blob.code()
        if DEBUG_DRAW_RAW_BLOBS:
            if code & 0x01:
                img.draw_rectangle(
                    blob.rect(),
                    color=(255, 255, 0)
                )

            if code & 0x02:
                img.draw_rectangle(
                    blob.rect(),
                    color=(0, 255, 255)
                )

        # ====================================================
        # 灯条粗筛
        # ====================================================
        if not is_light_strip(blob):
            continue

        x = blob.x()
        y = blob.y()
        w = blob.w()
        h = blob.h()
        cx = blob.cx()
        cy = blob.cy()

        # ====================================================
        # 每根灯条只计算一次长度
        # 一个 blob 一帧只 sqrt 一次
        # ====================================================
        length = math.sqrt(
            w * w +
            h * h
        )

        angle = rad_to_deg(
            blob.rotation()
        )

        # ====================================================
        # 灯条公共属性只构造一次
        # color 字段包含红蓝分支
        # ====================================================
        light = {
            "blob": blob,
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "cx": cx,
            "cy": cy,
            "pixels": blob.pixels(),
            "elongation": blob.elongation(),
            "angle": angle,
            "length": length
        }

        # ====================================================
        # 红色 threshold
        # 第 0 个 threshold -> bit 0 -> 0x01
        # ====================================================
        if code & 0x01:
            red_lights.append(
                dict(light, color="RED")
            )

            if DEBUG_DRAW_LIGHTS:
                img.draw_rectangle(
                    blob.rect(),
                    color=(255, 0, 0),
                    thickness=2
                )

                img.draw_cross(
                    cx,
                    cy,
                    color=(255, 0, 0)
                )

        # ====================================================
        # 蓝色 threshold
        # 第 1 个 threshold -> bit 1 -> 0x02
        # ====================================================
        if code & 0x02:
            blue_lights.append(
                dict(light, color="BLUE")
            )

            if DEBUG_DRAW_LIGHTS:
                img.draw_rectangle(
                    blob.rect(),
                    color=(0, 0, 255),
                    thickness=2
                )

                img.draw_cross(
                    cx,
                    cy,
                    color=(0, 0, 255)
                )

    return red_lights, blue_lights

# ============================================================
# 6. 计算两根灯条的几何评分
# ============================================================


def calc_pair_score(light1, light2, detail=False):
    """
    计算两个灯条组成装甲板的可能性。
    detail=False:
        只返回 score(float)
        用于大量 pair 的快速比较，
        避免每个 pair 创建 dict。
    detail=True:
        返回完整评分信息 dict。
        只用于最终最佳 pair。
    """
    # ========================================================
    # 1. 直接读取预计算灯条长度
    # ========================================================

    len1 = light1["length"]
    len2 = light2["length"]

    avg_len = (len1 + len2) * 0.5

    if avg_len <= 0:
        return None

    inv_avg_len = 1.0 / avg_len

    # ========================================================
    # 2. 两灯条角度差
    # ========================================================
    angle_diff = angle_diff_deg(
        light1["angle"],
        light2["angle"]
    )
    if angle_diff > MAX_ANGLE_DIFF_DEG:
        return None

    # ========================================================
    # 3. 两灯条长度一致性
    # ========================================================
    max_len = max(len1, len2)
    length_diff_ratio = (
        abs(len1 - len2) / max_len
    )
    if length_diff_ratio > MAX_LENGTH_DIFF_RATIO:
        return None

    # ========================================================
    # 4. 两灯条中心向量
    # ========================================================

    dx = (light2["cx"] - light1["cx"])
    dy = (light2["cy"] - light1["cy"])
    # ========================================================
    # 5. 计算两灯条平均方向
    # ========================================================

    angle1 = light1["angle"]
    angle2 = light2["angle"]

    angle_delta = (angle2 - angle1)

    if angle_delta > 90:
        angle2 -= 180

    elif angle_delta < -90:
        angle2 += 180

    avg_angle = (angle1 + angle2) * 0.5

    # ========================================================
    # 6. 转换为弧度
    # ========================================================

    theta = (avg_angle * math.pi / 180.0)

    # ========================================================
    # 7. 灯条方向单位向量
    # ========================================================

    ux = math.cos(theta)
    uy = math.sin(theta)

    # ========================================================
    # 8. 沿灯条方向错位
    # ========================================================

    parallel_offset = abs(dx * ux + dy * uy)

    # ========================================================
    # 9. 法线方向距离
    # ========================================================

    normal_distance = abs(
        -dx * uy +
        dy * ux
    )

    # ========================================================
    # 10. 归一化
    # ========================================================

    parallel_ratio = (parallel_offset * inv_avg_len)
    normal_ratio = (normal_distance * inv_avg_len)

    # ========================================================
    # 11. 硬约束
    # ========================================================

    if parallel_ratio > MAX_PARALLEL_RATIO:
        return None

    if normal_ratio < MIN_NORMAL_RATIO:
        return None

    if normal_ratio > MAX_NORMAL_RATIO:
        return None

    # ========================================================
    # 12. 角度评分
    # ========================================================

    angle_score = max(0.0, 1.0 - angle_diff / 30.0)

    # ========================================================
    # 13. 平行错位评分
    # ========================================================

    offset_score = max(0.0, 1.0 - parallel_ratio)

    # ========================================================
    # 14. 长度一致性评分
    # ========================================================

    length_score = max(0.0, 1.0 - length_diff_ratio / 0.70)

    # ========================================================
    # 15. 灯条间距评分
    # ========================================================

    distance_error = abs(normal_ratio - IDEAL_NORMAL_RATIO)
    distance_score = max(0.0, 1.0 - distance_error / IDEAL_NORMAL_RATIO)

    # ========================================================
    # 16. 根据距离动态调整评分权重
    # ========================================================

    if avg_len < 8:

        score = (
            0.55 * angle_score +
            0.25 * offset_score +
            0.05 * length_score +
            0.15 * distance_score
        )

    elif avg_len < 15:

        score = (
            0.50 * angle_score +
            0.20 * offset_score +
            0.10 * length_score +
            0.20 * distance_score
        )

    else:

        score = (
            0.40 * angle_score +
            0.25 * offset_score +
            0.20 * length_score +
            0.15 * distance_score
        )

    # ========================================================
    # 17. 快速模式
    # 大部分 pair 到这里直接返回一个 float。
    # 不创建 dict。
    # ========================================================
    if not detail:
        return score

    # ========================================================
    # 18. 详细模式
    # 只有最终最佳 pair 才会执行这里。
    # ========================================================

    return {
        "score":
            score,

        "angle_diff":
            angle_diff,

        "length_diff_ratio":
            length_diff_ratio,

        "parallel_ratio":
            parallel_ratio,

        "normal_ratio":
            normal_ratio,

        "len1":
            len1,

        "len2":
            len2,

        "angle_score":
            angle_score,

        "offset_score":
            offset_score,

        "length_score":
            length_score,

        "distance_score":
            distance_score
    }
# ============================================================
# 7. M2：寻找最佳灯条配对
# ============================================================


def find_best_pair(lights):
    """
    对所有候选灯条进行两两组合。
    第一阶段：
        calc_pair_score() 只返回 float score，
        不创建详细 dict。
    第二阶段：
        找出最佳 pair 后，
        只对最佳 pair 再计算一次详细数据。
    评分规则保持不变。
    """

    # ========================================================
    # 少于两根灯条无法组成装甲板
    # ========================================================

    count = len(lights)

    if count < 2:
        return None

    # ========================================================
    # 保存当前最佳组合
    # ========================================================

    best_score = -1.0

    best_a = None
    best_b = None

    # ========================================================
    # 遍历所有两两组合
    # ========================================================

    for i in range(count):

        a = lights[i]

        for j in range(
            i + 1,
            count
        ):

            b = lights[j]

            # =================================================
            # 快速模式
            # 这里只返回：float score
            # 或： None
            # 不再为每个 pair 创建 dict。
            # =================================================

            score = calc_pair_score(
                a,
                b
            )

            # 明显不合理的组合
            if score is None:
                continue

            # =================================================
            # 只保存当前最高分对应的两根灯条
            # =================================================

            if score > best_score:

                best_score = score

                best_a = a
                best_b = b

    # ========================================================
    # 没有合法组合，或低于最低评分限制
    # ========================================================

    if best_a is None or best_score < MIN_PAIR_SCORE:
        return None

    # ========================================================
    # 根据 x 坐标确定左右灯条
    # ========================================================

    if best_a["cx"] <= best_b["cx"]:

        left = best_a
        right = best_b

    else:

        left = best_b
        right = best_a

    # ========================================================
    # 只对最终胜出的这一组，
    # 再计算一次完整详细信息。
    # 一帧每种颜色最多执行一次。
    # ========================================================

    detail = calc_pair_score(
        best_a,
        best_b,
        detail=True
    )

    # 理论上不会发生。
    # 因为刚才这个 pair 已经通过快速模式。
    if detail is None:
        return None

    # ========================================================
    # 返回结构保持与原程序一致
    # ========================================================

    return {
        "left":
            left,

        "right":
            right,

        "geometry_score":
            detail["score"],

        "angle_diff":
            detail["angle_diff"],

        "length_diff_ratio":
            detail["length_diff_ratio"],

        "parallel_ratio":
            detail["parallel_ratio"],

        "normal_ratio":
            detail["normal_ratio"],

        "angle_score":
            detail["angle_score"],

        "offset_score":
            detail["offset_score"],

        "length_score":
            detail["length_score"],

        "distance_score":
            detail["distance_score"]
    }
# ============================================================
# 8. Debug：绘制最终装甲候选
# ============================================================


def get_pair_geometry(pair):
    left = pair["left"]
    right = pair["right"]

    x1 = min(left["x"], right["x"])
    y1 = min(left["y"], right["y"])

    x2 = max(
        left["x"] + left["w"],
        right["x"] + right["w"]
    )

    y2 = max(
        left["y"] + left["h"],
        right["y"] + right["h"]
    )

    w = x2 - x1
    h = y2 - y1

    cx = (left["cx"] + right["cx"]) // 2
    cy = (left["cy"] + right["cy"]) // 2

    return cx, cy, x1, y1, w, h

# ============================================================
# 9. 主循环
# ============================================================


# ------------------------------------------------------------
# 串口 Debug 输出计时
# DEBUG_PRINT=True 时，每 5 秒输出一次 Debug + FPS
# ------------------------------------------------------------
last_serial_print = time.ticks_ms()
SERIAL_PRINT_INTERVAL = 5000  # 5000ms = 5秒

# ------------------------------------------------------------
# 保存上一次识别到的颜色，用于防止 COLOR 每一帧重复刷屏
# ------------------------------------------------------------
last_armor_color = None


while True:

    # ========================================================
    # 1. 开始统计当前帧
    # ========================================================
    clock.tick()
    # ========================================================
    # 2. 获取一帧图像
    # ========================================================
    img = sensor.snapshot()
    # ========================================================
    # 3. 寻找红色和蓝色灯条
    # ========================================================
    red_lights, blue_lights = detect_lights(img)
    # ========================================================
    # 4. 红蓝分别进行装甲板配对
    # ========================================================
    red_pair = find_best_pair(
        red_lights
    )

    blue_pair = find_best_pair(
        blue_lights
    )
    # ========================================================
    # 5. 选择最终装甲板
    # ========================================================
    best_pair = None
    armor_color = None
    # --------------------------------------------------------
    # 红色和蓝色都识别到了
    # 选择几何评分更高的一个
    # --------------------------------------------------------
    if red_pair is not None and blue_pair is not None:

        if red_pair["geometry_score"] >= blue_pair["geometry_score"]:

            best_pair = red_pair
            armor_color = "RED"

        else:

            best_pair = blue_pair
            armor_color = "BLUE"
    # --------------------------------------------------------
    # 只识别到红色
    # --------------------------------------------------------
    elif red_pair is not None:

        best_pair = red_pair
        armor_color = "RED"
    # --------------------------------------------------------
    # 只识别到蓝色
    # --------------------------------------------------------
    elif blue_pair is not None:

        best_pair = blue_pair
        armor_color = "BLUE"
    # ========================================================
    # 6. 找到合法装甲板
    # ========================================================
    if best_pair is not None:

        cx, cy, x, y, w, h = get_pair_geometry(
            best_pair
        )
        if armor_color != last_armor_color:
            print(
                "COLOR=%s"
                % armor_color
            )
            last_armor_color = armor_color
        if DEBUG_DRAW_PAIR:
            img.draw_cross(
                cx,
                cy,
                size=8,
                color=(0, 255, 0),
                thickness=2
            )
    # ========================================================
    # 7. 当前没有找到合法装甲板
    # ========================================================
    else:
        if last_armor_color is not None:
            print(
                "COLOR=NONE"
            )
            last_armor_color = None
    # ========================================================
    # 8. 串口周期输出
    # ========================================================
    now = time.ticks_ms()
    if time.ticks_diff(
        now,
        last_serial_print
    ) >= SERIAL_PRINT_INTERVAL:
        # ====================================================
        # FPS 始终输出
        # ====================================================
        print(
            "FPS=%.2f"
            % clock.fps()
        )
        # ====================================================
        # 详细 Debug 信息由 DEBUG_PRINT 控制
        # ====================================================
        if DEBUG_PRINT:
            # ------------------------------------------------
            # 当前识别到了装甲板
            # ------------------------------------------------
            if best_pair is not None:
                print(
                    "PAIR "
                    "color=%s "
                    "red_lights=%d "
                    "blue_lights=%d "
                    "score=%.2f "
                    "center=(%d,%d) "
                    "angle=%.1f "
                    "parallel=%.2f "
                    "normal=%.2f "
                    "len_diff=%.2f"
                    % (
                        armor_color,
                        len(red_lights),
                        len(blue_lights),
                        best_pair["geometry_score"],
                        cx,
                        cy,
                        best_pair["angle_diff"],
                        best_pair["parallel_ratio"],
                        best_pair["normal_ratio"],
                        best_pair["length_diff_ratio"]
                    )
                )
                print(
                    "SCORE "
                    "angle=%.2f "
                    "offset=%.2f "
                    "length=%.2f "
                    "distance=%.2f"
                    % (
                        best_pair["angle_score"],
                        best_pair["offset_score"],
                        best_pair["length_score"],
                        best_pair["distance_score"]
                    )
                )
                print(
                    "BOX "
                    "x=%d "
                    "y=%d "
                    "w=%d "
                    "h=%d"
                    % (
                        x,
                        y,
                        w,
                        h
                    )
                )
            # ------------------------------------------------
            # 当前没有识别到装甲板
            # ------------------------------------------------
            else:
                print(
                    "NO_PAIR "
                    "red_lights=%d "
                    "blue_lights=%d"
                    % (
                        len(red_lights),
                        len(blue_lights)
                    )
                )
            print("")
        # ====================================================
        # 无论 DEBUG_PRINT 是否开启，都必须更新时间
        # ====================================================
        last_serial_print = now
