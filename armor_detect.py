# ============================================================
# armor_detect.py
# 单帧装甲板检测：灯条提取 -> 粗筛 -> 两两配对 -> 几何评分
#
# 由原 visual_color.py 拆分而来，逻辑与行为保持一致，
# 所有阈值改为引用 config.py。
#
# 对外接口：
#   detect_lights(img)              -> red_lights, blue_lights
#   find_best_pair(lights)          -> pair dict 或 None
#   select_best_pair(red, blue)     -> (pair, "RED"/"BLUE") 或 (None, None)
#   get_pair_geometry(pair)         -> cx, cy, x, y, w, h
#   get_pair_measurements(pair)     -> spacing_px, light_len_px
# ============================================================

import math

import config


# ============================================================
# 1. 工具函数
# ============================================================

def rad_to_deg(rad):
    """ OpenMV blob.rotation() 返回弧度，转为角度。 """
    return rad * 180.0 / math.pi


def angle_diff_deg(a, b):
    """
    两个灯条方向的角度差。
    灯条方向具有 180° 对称性（10° 与 170° 实际只差 20°），
    因此不能直接 abs(a - b)。
    """
    diff = abs(a - b) % 180.0
    if diff > 90:
        diff = 180 - diff
    return diff


# ============================================================
# 2. 灯条粗筛
# ============================================================

def is_light_strip(blob):
    """
    对灯条候选做初筛，真正的装甲板判断交给后面的几何评分。
    """
    # 条件 1：像素数量。远距离灯条像素很少，不能设太高
    if blob.pixels() < config.MIN_PIXELS:
        return False

    # 条件 2：长边尺寸。旋转后长短边会变，用长边更稳定
    long_side = max(blob.w(), blob.h())
    if long_side < config.MIN_LIGHT_LEN:
        return False

    # 条件 3：极宽松 elongation，只排除圆形 / 方形块状光斑
    if blob.elongation() < config.MIN_ELONGATION:
        return False

    return True


def detect_lights(img):
    """
    一次 find_blobs 同时检测红色和蓝色灯条。
    返回：red_lights, blue_lights
    每根灯条预计算 length，供 calc_pair_score() 重复使用。
    """
    blobs = img.find_blobs(
        [config.RED_THRESHOLD, config.BLUE_THRESHOLD],
        x_stride=1,
        y_stride=1,
        area_threshold=config.MIN_PIXELS,
        pixels_threshold=config.MIN_PIXELS,
        merge=True,
        margin=2,
    )

    red_lights = []
    blue_lights = []

    for blob in blobs:
        code = blob.code()

        # Debug：原始 blob
        if config.DEBUG_DRAW_RAW_BLOBS:
            if code & 0x01:
                img.draw_rectangle(blob.rect(), color=(255, 255, 0))
            if code & 0x02:
                img.draw_rectangle(blob.rect(), color=(0, 255, 255))

        if not is_light_strip(blob):
            continue

        x = blob.x()
        y = blob.y()
        w = blob.w()
        h = blob.h()
        cx = blob.cx()
        cy = blob.cy()

        # 每根灯条只算一次 sqrt
        length = math.sqrt(w * w + h * h)
        angle = rad_to_deg(blob.rotation())

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
            "length": length,
        }

        # 第 0 个 threshold -> bit 0 -> 红色
        if code & 0x01:
            red_lights.append(dict(light, color="RED"))

            if config.DEBUG_DRAW_LIGHTS:
                img.draw_rectangle(blob.rect(), color=(255, 0, 0), thickness=2)
                img.draw_cross(cx, cy, color=(255, 0, 0))

        # 第 1 个 threshold -> bit 1 -> 蓝色
        if code & 0x02:
            blue_lights.append(dict(light, color="BLUE"))

            if config.DEBUG_DRAW_LIGHTS:
                img.draw_rectangle(blob.rect(), color=(0, 0, 255), thickness=2)
                img.draw_cross(cx, cy, color=(0, 0, 255))

    return red_lights, blue_lights


# ============================================================
# 3. 两根灯条的几何评分
# ============================================================

def calc_pair_score(light1, light2, detail=False):
    """
    计算两个灯条组成装甲板的可能性。
    detail=False：只返回 score(float)，用于大量 pair 的快速比较。
    detail=True ：返回完整评分 dict，只用于最终最佳 pair。
    不合法组合返回 None。
    """
    # 1. 预计算长度
    len1 = light1["length"]
    len2 = light2["length"]

    avg_len = (len1 + len2) * 0.5
    if avg_len <= 0:
        return None

    inv_avg_len = 1.0 / avg_len

    # 2. 角度差
    angle_diff = angle_diff_deg(light1["angle"], light2["angle"])
    if angle_diff > config.MAX_ANGLE_DIFF_DEG:
        return None

    # 3. 长度一致性
    max_len = max(len1, len2)
    length_diff_ratio = abs(len1 - len2) / max_len
    if length_diff_ratio > config.MAX_LENGTH_DIFF_RATIO:
        return None

    # 4. 中心向量
    dx = light2["cx"] - light1["cx"]
    dy = light2["cy"] - light1["cy"]

    # 5. 平均方向（处理 180° 跨越）
    angle1 = light1["angle"]
    angle2 = light2["angle"]

    angle_delta = angle2 - angle1
    if angle_delta > 90:
        angle2 -= 180
    elif angle_delta < -90:
        angle2 += 180

    avg_angle = (angle1 + angle2) * 0.5

    # 6~7. 灯条方向单位向量
    theta = avg_angle * math.pi / 180.0
    ux = math.cos(theta)
    uy = math.sin(theta)

    # 8. 沿灯条方向错位
    parallel_offset = abs(dx * ux + dy * uy)

    # 9. 法线方向距离，再减去两边各半个灯条厚度得到真正空隙
    normal_distance = abs(-dx * uy + dy * ux)

    thickness1 = min(light1["w"], light1["h"])
    thickness2 = min(light2["w"], light2["h"])

    normal_gap = normal_distance - 0.5 * thickness1 - 0.5 * thickness2
    if normal_gap < 0:
        normal_gap = 0

    # 10. 归一化
    parallel_ratio = parallel_offset * inv_avg_len
    normal_ratio = normal_gap * inv_avg_len

    # 11. 硬约束
    if parallel_ratio > config.MAX_PARALLEL_RATIO:
        return None
    if normal_ratio < config.MIN_NORMAL_RATIO:
        return None
    if normal_ratio > config.MAX_NORMAL_RATIO:
        return None

    # 12~15. 各项评分
    angle_score = max(0.0, 1.0 - angle_diff / 30.0)
    offset_score = max(0.0, 1.0 - parallel_ratio)
    length_score = max(0.0, 1.0 - length_diff_ratio / 0.70)

    distance_error = abs(normal_ratio - config.IDEAL_NORMAL_RATIO)
    distance_score = max(0.0, 1.0 - distance_error / config.IDEAL_NORMAL_RATIO)

    # 16. 根据距离（灯条像素长度）动态调整权重
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

    # 17. 快速模式
    if not detail:
        return score

    # 18. 详细模式
    return {
        "score": score,
        "angle_diff": angle_diff,
        "length_diff_ratio": length_diff_ratio,
        "parallel_ratio": parallel_ratio,
        "normal_ratio": normal_ratio,
        "len1": len1,
        "len2": len2,
        "angle_score": angle_score,
        "offset_score": offset_score,
        "length_score": length_score,
        "distance_score": distance_score,
    }


# ============================================================
# 4. 寻找最佳灯条配对
# ============================================================

def find_best_pair(lights):
    """
    对所有候选灯条两两组合。
    第一阶段只比较 float score；第二阶段只对最佳 pair 算一次详细数据。
    """
    count = len(lights)
    if count < 2:
        return None

    best_score = -1.0
    best_a = None
    best_b = None

    for i in range(count):
        a = lights[i]
        for j in range(i + 1, count):
            b = lights[j]

            score = calc_pair_score(a, b)
            if score is None:
                continue

            if score > best_score:
                best_score = score
                best_a = a
                best_b = b

    if best_a is None or best_score < config.MIN_PAIR_SCORE:
        return None

    # 按 x 坐标确定左右
    if best_a["cx"] <= best_b["cx"]:
        left = best_a
        right = best_b
    else:
        left = best_b
        right = best_a

    detail = calc_pair_score(best_a, best_b, detail=True)
    if detail is None:
        return None

    return {
        "left": left,
        "right": right,
        "geometry_score": detail["score"],
        "angle_diff": detail["angle_diff"],
        "length_diff_ratio": detail["length_diff_ratio"],
        "parallel_ratio": detail["parallel_ratio"],
        "normal_ratio": detail["normal_ratio"],
        "angle_score": detail["angle_score"],
        "offset_score": detail["offset_score"],
        "length_score": detail["length_score"],
        "distance_score": detail["distance_score"],
    }


def select_best_pair(red_pair, blue_pair):
    """
    红蓝都识别到时取几何评分更高的一个。
    返回 (pair, color)；没有目标时返回 (None, None)。
    """
    if red_pair is not None and blue_pair is not None:
        if red_pair["geometry_score"] >= blue_pair["geometry_score"]:
            return red_pair, "RED"
        return blue_pair, "BLUE"

    if red_pair is not None:
        return red_pair, "RED"

    if blue_pair is not None:
        return blue_pair, "BLUE"

    return None, None


# ============================================================
# 5. 装甲板几何量
# ============================================================

def get_pair_geometry(pair):
    """
    返回装甲板中心与整体包围框：cx, cy, x, y, w, h
    """
    left = pair["left"]
    right = pair["right"]

    x1 = min(left["x"], right["x"])
    y1 = min(left["y"], right["y"])

    x2 = max(left["x"] + left["w"], right["x"] + right["w"])
    y2 = max(left["y"] + left["h"], right["y"] + right["h"])

    w = x2 - x1
    h = y2 - y1

    cx = (left["cx"] + right["cx"]) // 2
    cy = (left["cy"] + right["cy"]) // 2

    return cx, cy, x1, y1, w, h


def get_pair_measurements(pair):
    """
    距离解算用的像素尺寸：
        spacing_px   ：两灯条中心距
        light_len_px ：两灯条长度均值（复用 detect_lights 预计算的 length）
    """
    left = pair["left"]
    right = pair["right"]

    dx = right["cx"] - left["cx"]
    dy = right["cy"] - left["cy"]

    spacing_px = math.sqrt(dx * dx + dy * dy)
    light_len_px = 0.5 * (left["length"] + right["length"])

    return spacing_px, light_len_px
