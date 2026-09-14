# ============================================================
# 1. 导入库
# ============================================================
import sensor
import time
import math
# ============================================================
# 2. 参数配置区
#    所有需要现场调节的参数集中放这里
# ============================================================
# ---------- Debug 开关 ----------
# True：显示原始 blob 黄色框
DEBUG_DRAW_RAW_BLOBS = False
# True：显示通过灯条筛选后的红框
DEBUG_DRAW_LIGHTS = False
# True：显示最终装甲板白框
DEBUG_DRAW_PAIR = True
# True：串口打印调试信息
DEBUG_PRINT = False

# ---------- 红色 LAB 阈值 ----------
# 使用 OpenMV IDE 的阈值编辑器调整
# OpenMV IDE阈值编辑器格式：(L_min, L_max, A_min, A_max, B_min, B_max)
# 大致理解为：L：亮度  A：负值偏绿，正值偏红  B：负值偏蓝，正值偏黄
RED_THRESHOLD = (40, 100, 20, 127, 30, 127)
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
sensor.set_auto_exposure(False, exposure_us=1500)


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
    例如 10° 与 170° 实际方向非常接近，
    因此不能直接 abs(a-b)。
    """
    diff = abs(a - b)

    # 将差值限制到 0~180
    while diff > 180:
        diff -= 180

    # 方向具有 180° 对称性
    if diff > 90:
        diff = 180 - diff
    return diff

# ============================================================
# 5. M1：灯条粗筛选
# ============================================================


def is_light_strip(blob):
    """
    对红色 blob 做宽松灯条筛选。
    M1 的目的不是确认它一定是灯条，
    而只是排除明显噪声。
    真正的装甲板判断交给后面的 M2 几何评分。
    """
    # --------------------------------------------------------
    # 条件 1：像素数量
    # --------------------------------------------------------
    # 只过滤特别小的噪声。
    # 远距离灯条可能只有很少像素，因此不能设置太高。
    if blob.pixels() < MIN_PIXELS:
        return False
    # --------------------------------------------------------
    # 条件 2：长边尺寸
    # --------------------------------------------------------
    # 不再使用单独的 h() 判断。
    # 因为灯条发生旋转以后：
    # h 可能变小，
    # w 可能变大。
    # 使用长边可以提高旋转情况下的稳定性。
    long_side = max(
        blob.w(),
        blob.h()
    )
    if long_side < MIN_LIGHT_LEN:
        return False

    # --------------------------------------------------------
    # 条件 3：极宽松 elongation
    # --------------------------------------------------------
    # 只排除明显的块状物体。
    # 因为远距离、过曝、边缘断裂时 density 波动比较明显。
    if blob.elongation() < MIN_ELONGATION:
        return False
    return True


def detect_lights(img):
    """
    一次 find_blobs 同时检测红色和蓝色灯条。
    返回：
        red_lights
        blue_lights
    同时预计算灯条长度 length，
    供 calc_pair_score() 重复使用。
    """
    blobs = img.find_blobs(
        [RED_THRESHOLD, BLUE_THRESHOLD],
        x_stride=1,
        y_stride=1,
        area_threshold=5,
        pixels_threshold=5,
        merge=False
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
        # M1：灯条粗筛
        # ====================================================
        if not is_light_strip(blob):
            continue

        # ====================================================
        # blob 属性只读取一次
        # ====================================================
        x = blob.x()
        y = blob.y()
        w = blob.w()
        h = blob.h()
        cx = blob.cx()
        cy = blob.cy()

        # ====================================================
        # 每根灯条只计算一次长度
        #
        # 原来：
        # calc_pair_score() 每次配对重新 sqrt
        #
        # 现在：
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
        # 红色 threshold
        # 第 0 个 threshold -> bit 0 -> 0x01
        # ====================================================
        if code & 0x01:
            red_light = {
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
                "length": length,
                "color": "RED"
            }
            red_lights.append(
                red_light
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
            blue_light = {
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
                "length": length,
                "color": "BLUE"
            }

            blue_lights.append(
                blue_light
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
# 6. M2：计算两根灯条的几何评分
# ============================================================


def calc_pair_score(light1, light2):

    # 计算两个灯条组成装甲板的可能性。
    # ========================================================
    # 1. 直接读取预计算灯条长度
    # ========================================================

    len1 = light1["length"]
    len2 = light2["length"]

    avg_len = (
        len1 +
        len2
    ) * 0.5

    if avg_len <= 0:
        return None

    # 后面有多个值都需要 / avg_len
    # 所以只做一次除法
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

    if len1 > len2:
        max_len = len1
    else:
        max_len = len2

    if max_len <= 0:
        return None

    length_diff_ratio = (
        abs(len1 - len2) /
        max_len
    )

    if length_diff_ratio > MAX_LENGTH_DIFF_RATIO:
        return None

    # ========================================================
    # 4. 两灯条中心向量
    # ========================================================

    dx = (
        light2["cx"] -
        light1["cx"]
    )

    dy = (
        light2["cy"] -
        light1["cy"]
    )

    # ========================================================
    # 5. 计算两灯条平均方向
    # ========================================================

    angle1 = light1["angle"]
    angle2 = light2["angle"]

    angle_delta = (
        angle2 -
        angle1
    )

    if angle_delta > 90:
        angle2 -= 180

    elif angle_delta < -90:
        angle2 += 180

    avg_angle = (
        angle1 +
        angle2
    ) * 0.5

    # ========================================================
    # 6. 平均方向转弧度
    # ========================================================

    theta = (
        avg_angle *
        math.pi /
        180.0
    )

    # ========================================================
    # 7. 灯条方向单位向量
    # ========================================================

    ux = math.cos(theta)
    uy = math.sin(theta)

    # ========================================================
    # 8. 沿灯条方向投影
    # ========================================================

    parallel_offset = abs(
        dx * ux +
        dy * uy
    )

    # ========================================================
    # 9. 垂直灯条方向投影
    # ========================================================

    normal_distance = abs(
        -dx * uy +
        dy * ux
    )

    # ========================================================
    # 10. 归一化
    #
    # 原来是两个除法：
    #
    # parallel_offset / avg_len
    # normal_distance / avg_len
    #
    # 现在只有：
    #
    # inv_avg_len = 1 / avg_len
    #
    # 后面使用乘法
    # ========================================================

    parallel_ratio = (
        parallel_offset *
        inv_avg_len
    )

    normal_ratio = (
        normal_distance *
        inv_avg_len
    )

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

    angle_score = (
        1.0 -
        angle_diff / 30.0
    )

    if angle_score < 0.0:
        angle_score = 0.0

    # ========================================================
    # 13. 平行方向错位评分
    # ========================================================

    offset_score = (
        1.0 -
        parallel_ratio
    )

    if offset_score < 0.0:
        offset_score = 0.0

    # ========================================================
    # 14. 长度一致性评分
    # ========================================================

    length_score = (
        1.0 -
        length_diff_ratio / 0.70
    )

    if length_score < 0.0:
        length_score = 0.0

    # ========================================================
    # 15. 灯条间距评分
    # ========================================================

    distance_error = abs(
        normal_ratio -
        IDEAL_NORMAL_RATIO
    )

    distance_score = (
        1.0 -
        distance_error /
        IDEAL_NORMAL_RATIO
    )

    if distance_score < 0.0:
        distance_score = 0.0

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
    # 17. 返回
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
    每一组调用 calc_pair_score()。
    最终选择：geometry_score 最高的一组。
    同时加入 MIN_PAIR_SCORE，
    防止所有组合都很差时仍然强行输出装甲板。
    """
    # 少于两根灯条不可能组成装甲板
    if len(lights) < 2:
        return None
    best_pair = None
    best_score = -1.0

    # ========================================================
    # 遍历所有两两组合
    # ========================================================
    for i in range(len(lights)):
        for j in range(
            i + 1,
            len(lights)
        ):
            a = lights[i]
            b = lights[j]
            # =================================================
            # 计算当前组合几何评分
            # =================================================
            result = calc_pair_score(
                a,
                b
            )
            # 明显不合理的组合
            if result is None:
                continue
            # =================================================
            # 根据 x 坐标确定左右灯条
            # =================================================
            # 注意：
            # 这里只用于：
            # 绘图、中心计算以及后续数据结构。
            # 不参与几何评分。
            # 所以装甲板旋转不会受到影响。
            # =================================================
            if a["cx"] <= b["cx"]:

                left = a
                right = b
            else:
                left = b
                right = a
            # =================================================
            # 保存目前最高分组合
            # =================================================
            if result["score"] > best_score:
                best_score = result["score"]
                best_pair = {
                    "left": left,
                    "right": right,
                    # 总评分
                    "geometry_score":
                        result["score"],
                    # -----------------------------------------
                    # 几何数据
                    # -----------------------------------------
                    "angle_diff":
                        result["angle_diff"],
                    "length_diff_ratio":
                        result[
                            "length_diff_ratio"
                        ],
                    "parallel_ratio":
                        result["parallel_ratio"],
                    "normal_ratio":
                        result["normal_ratio"],
                    # -----------------------------------------
                    # 独立评分
                    # 后期调参时可以直接观察
                    # 到底是哪一个指标导致分数下降
                    # -----------------------------------------
                    "angle_score":
                        result["angle_score"],
                    "offset_score":
                        result["offset_score"],
                    "length_score":
                        result["length_score"],
                    "distance_score":
                        result["distance_score"]
                }

    # ========================================================
    # 没有任何组合通过硬条件
    # ========================================================
    if best_pair is None:
        return None
    # ========================================================
    # 最低评分限制
    # 即使它是所有组合里的最高分，
    # 如果最高分本身都非常低，
    # 也不能强制认为它是装甲板。
    # ========================================================
    if best_pair["geometry_score"] < MIN_PAIR_SCORE:
        return None

    return best_pair

# ============================================================
# 8. Debug：绘制最终装甲候选
# ============================================================


def draw_pair(img, pair):
    """
    计算装甲板中心及外接框。
    图像上只绘制装甲板中心十字。
    """
    left = pair["left"]
    right = pair["right"]
    # 外接框
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
    # 装甲板中心
    cx = (left["cx"] + right["cx"]) // 2
    cy = (left["cy"] + right["cy"]) // 2
    # 只画中心十字
    img.draw_cross(
        cx,
        cy,
        size=8,
        color=(0, 255, 0),
        thickness=2
    )
    return (
        cx,
        cy,
        x1,
        y1,
        w,
        h
    )

# ============================================================
# 9. 主循环
# ============================================================


last_fps_print = time.ticks_ms()
FPS_PRINT_INTERVAL = 30000  # 30秒

while True:
    # 开始统计当前帧耗时
    clock.tick()
    # 获取一帧图像
    img = sensor.snapshot()
    # ========================================================
    # M1：分别寻找红色和蓝色灯条
    # ========================================================
    red_lights, blue_lights = detect_lights(img)
    # ========================================================
    # M2：红蓝分别进行装甲板配对
    # ========================================================
    red_pair = find_best_pair(
        red_lights
    )
    blue_pair = find_best_pair(
        blue_lights
    )
    # ========================================================
    # 选择最终装甲板
    # ========================================================
    best_pair = None
    armor_color = None
    # 红蓝都识别到了
    if red_pair is not None and blue_pair is not None:
        if red_pair["geometry_score"] >= blue_pair["geometry_score"]:
            best_pair = red_pair
            armor_color = "RED"
        else:
            best_pair = blue_pair
            armor_color = "BLUE"
    # 只识别到红色
    elif red_pair is not None:
        best_pair = red_pair
        armor_color = "RED"
    # 只识别到蓝色
    elif blue_pair is not None:
        best_pair = blue_pair
        armor_color = "BLUE"
    # ========================================================
    # 如果找到装甲板候选
    # ========================================================
    if best_pair is not None:
        # ----------------------------------------------------
        # 绘制最终装甲板候选
        # draw_pair() 返回：
        # cx, cy
        #     装甲板几何中心
        # x, y, w, h
        #     装甲板外接矩形
        # ----------------------------------------------------
        cx, cy, x, y, w, h = draw_pair(
            img,
            best_pair
        )
        # ========================================================
        # 显示当前装甲板颜色
        # ========================================================
        if armor_color == "RED":
            img.draw_string(
                cx + 10,
                cy,
                "RED",
                color=(255, 0, 0),
                scale=2
            )
        elif armor_color == "BLUE":
            img.draw_string(
                cx + 10,
                cy,
                "BLUE",
                color=(0, 0, 255),
                scale=2
            )

        # ====================================================
        # Debug 输出
        # ====================================================
        if DEBUG_PRINT:

            # ------------------------------------------------
            # 第一行：
            # 最终识别结果以及主要几何参数
            # ------------------------------------------------

            print(
                "PAIR "
                "lights=%d "
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
                    best_pair[
                        "geometry_score"
                    ],
                    cx,
                    cy,
                    best_pair[
                        "angle_diff"
                    ],
                    best_pair[
                        "parallel_ratio"
                    ],
                    best_pair[
                        "normal_ratio"
                    ],
                    best_pair[
                        "length_diff_ratio"
                    ]
                )
            )

            # ------------------------------------------------
            # 第二行：
            # 四项独立评分
            # 用于现场调参
            # ------------------------------------------------

            print(
                "SCORE "
                "angle=%.2f "
                "offset=%.2f "
                "length=%.2f "
                "distance=%.2f"
                % (
                    best_pair[
                        "angle_score"
                    ],

                    best_pair[
                        "offset_score"
                    ],

                    best_pair[
                        "length_score"
                    ],

                    best_pair[
                        "distance_score"
                    ]
                )
            )

            # ------------------------------------------------
            # 第三行：
            # 当前装甲板外接框信息
            # 后续如果需要串口发送目标大小、
            # 估算距离，可以直接使用这些值
            # ------------------------------------------------

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

    # ========================================================
    # 当前帧没有找到合法装甲板
    # ========================================================
    else:
        if DEBUG_PRINT:

            print(
                "NO_PAIR"
                "red_lights=%d "
                "blue_lights=%d"
                % (
                    len(red_lights),
                    len(blue_lights)
                )
            )
    # ========================================================
    # FPS 输出
    # ========================================================
    if DEBUG_PRINT:
        now = time.ticks_ms()
        if time.ticks_diff(now, last_fps_print) >= FPS_PRINT_INTERVAL:
            print(
                "FPS=%.2f"
                % clock.fps()
            )
            print("")
            last_fps_print = now
