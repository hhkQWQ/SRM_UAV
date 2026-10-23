# ============================================================
# armor_detect.py
# 单帧装甲板检测：灯条提取（灰度 / LAB 双模式）-> 粗筛 -> 判色
#              -> 两两配对 -> 几何评分 -> 去重
#
# 由原 visual_color.py 拆分而来，所有阈值改为引用 config.py。
#
# 对外接口：
#   detect_lights(img)              -> red_lights, blue_lights
#   find_best_pair(lights, debug_img=None) -> pair dict 或 None
#   select_best_pair(red, blue)     -> (pair, "RED"/"BLUE") 或 (None, None)
#   get_pair_geometry(pair)         -> cx, cy, x, y, w, h
#   get_pair_measurements(pair)     -> spacing_px, light_len_px
#
# 调试数据（main 周期打印用）：
#   stats       ：检测漏斗计数，只累加，由 reset_stats() 清零
#   blob_log    ：最后一帧每个亮斑的数值（仅 DEBUG_MODE >= 2 采集）
# ============================================================

import math

import image

import config


# ============================================================
# 0. 调试统计
# ============================================================

STAT_KEYS = (
    "blobs",                                # find_blobs 返回的亮斑
    "size", "ratio", "fill", "color",       # 灯条筛选各步剔除数
    "RED", "BLUE",                          # 通过筛选的灯条
    "angle", "length", "near", "far", "tilt", "score",     # 配对各约束剔除数
    "cand",                                 # 达标候选对
    "dup",                                  # 被去重淘汰的候选对
)

stats = {}

# 元素：(cx, cy, pixels, ratio, fill, color_diff, 结果)
#   结果 = "RED" / "BLUE" 或剔除原因；ratio / color_diff 未计算时为 None
blob_log = []


def reset_stats():
    for k in STAT_KEYS:
        stats[k] = 0


reset_stats()


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
# 2. 灯条粗筛 + 判色
# ============================================================

def _line_len(line):
    """ (x1, y1, x2, y2) 线段 -> 长度。 """
    dx = line[2] - line[0]
    dy = line[3] - line[1]
    return math.sqrt(dx * dx + dy * dy)


def _axis_ratio(blob):
    """ 宽长比 = 旋转矩形短轴 / 长轴（旋转无关）。长轴为 0 返回 None。 """
    major = _line_len(blob.major_axis_line())
    if major <= 0:
        return None
    return _line_len(blob.minor_axis_line()) / major


def is_light_strip(blob):
    """
    对灯条候选做初筛，真正的装甲板判断交给后面的几何评分。
    通过返回 None，否则返回剔除原因 "size" / "ratio" / "fill"。
    """
    # 条件 1：像素数量。远距离灯条像素很少，不能设太高
    if blob.pixels() < config.MIN_PIXELS:
        return "size"

    # 条件 2：长边尺寸。旋转后长短边会变，用长边更稳定
    long_side = max(blob.w(), blob.h())
    if long_side < config.MIN_LIGHT_LEN:
        return "size"

    # 条件 3：极宽松 elongation，只排除圆形 / 方形块状光斑
    if blob.elongation() < config.MIN_ELONGATION:
        return "size"

    # 条件 4 / 5：宽长比 + 填充率。远距离像素太少时量化误差大，跳过
    if blob.pixels() >= config.SHAPE_CHECK_MIN_PIXELS:
        ratio = _axis_ratio(blob)
        if ratio is not None and (
            ratio < config.MIN_LIGHT_RATIO or ratio > config.MAX_LIGHT_RATIO
        ):
            return "ratio"

        # 填充率 = 像素数 / 最小旋转矩形面积
        if blob.solidity() < config.MIN_FILL_RATIO:
            return "fill"

    return None


def classify_light_color(color_diff):
    """
    按红蓝通道均值差判色（参考武科：差异不够大视为白光 / 杂色）。
    返回 "RED" / "BLUE" / None。
    """
    if color_diff > config.COLOR_DIFF_MIN:
        return "RED"
    if color_diff < -config.COLOR_DIFF_MIN:
        return "BLUE"
    return None


def _sample_color_diff(img, blob):
    """
    在 blob 外框外扩的 ROI 内，统计 L >= COLOR_SAMPLE_L_MIN 的像素（即色晕），
    取 LAB 均值转回 RGB，返回红蓝通道均值差 r - b。
    过曝灯条中心发白，颜色信息在外圈色晕，因此采样区要外扩、阈值要比提取阈值低。
    """
    x = blob.x()
    y = blob.y()

    x0 = max(0, x - config.COLOR_ROI_PAD)
    y0 = max(0, y - config.COLOR_ROI_PAD)
    x1 = min(img.width(), x + blob.w() + config.COLOR_ROI_PAD)
    y1 = min(img.height(), y + blob.h() + config.COLOR_ROI_PAD)

    stats_roi = img.get_statistics(
        thresholds=[(config.COLOR_SAMPLE_L_MIN, 100, -128, 127, -128, 127)],
        roi=(x0, y0, x1 - x0, y1 - y0),
    )
    r, g, b = image.lab_to_rgb(
        (stats_roi.l_mean(), stats_roi.a_mean(), stats_roi.b_mean())
    )
    return r - b


def detect_lights(img):
    """
    提取红 / 蓝灯条。
    GRAY 模式：亮度阈值提取所有亮斑，再逐根独立判色；
    LAB  模式：红 / 蓝 LAB 阈值直接提取（旧方式）。
    返回：red_lights, blue_lights
    每根灯条预计算 length，供 calc_pair_score() 重复使用。
    """
    gray_mode = config.LIGHT_EXTRACT_MODE == "GRAY"
    debug = config.DEBUG_MODE >= 2

    if debug:
        blob_log.clear()

    if gray_mode:
        thresholds = [(config.LIGHT_L_MIN, 100, -128, 127, -128, 127)]
    else:
        thresholds = [config.RED_THRESHOLD, config.BLUE_THRESHOLD]

    blobs = img.find_blobs(
        thresholds,
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
        stats["blobs"] += 1

        reason = is_light_strip(blob)
        color_diff = None
        colors = ()

        if reason is None:
            if gray_mode:
                # 独立判色：色晕内红蓝均值差决定颜色
                color_diff = _sample_color_diff(img, blob)
                color = classify_light_color(color_diff)
                if color is None:
                    reason = "color"
                else:
                    colors = (color,)
            else:
                # LAB 模式：第 0 个 threshold -> bit 0 -> 红，第 1 个 -> bit 1 -> 蓝
                code = blob.code()
                if code & 0x01 and code & 0x02:
                    colors = ("RED", "BLUE")
                elif code & 0x01:
                    colors = ("RED",)
                else:
                    colors = ("BLUE",)

        if debug:
            blob_log.append((
                blob.cx(),
                blob.cy(),
                blob.pixels(),
                _axis_ratio(blob),
                blob.solidity(),
                color_diff,
                reason or "+".join(colors),
            ))

        if reason is not None:
            stats[reason] += 1
            continue

        w = blob.w()
        h = blob.h()

        light = {
            "x": blob.x(),
            "y": blob.y(),
            "w": w,
            "h": h,
            "cx": blob.cx(),
            "cy": blob.cy(),
            "angle": rad_to_deg(blob.rotation()),
            # 每根灯条只算一次 sqrt
            "length": math.sqrt(w * w + h * h),
        }

        for color in colors:
            stats[color] += 1
            if color == "RED":
                red_lights.append(dict(light, color=color))
            else:
                blue_lights.append(dict(light, color=color))

            if debug:
                draw_color = (255, 0, 0) if color == "RED" else (0, 0, 255)
                img.draw_rectangle(blob.rect(), color=draw_color, thickness=2)

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
        stats["angle"] += 1
        return None

    # 3. 长度一致性
    max_len = max(len1, len2)
    length_diff_ratio = abs(len1 - len2) / max_len
    if length_diff_ratio > config.MAX_LENGTH_DIFF_RATIO:
        stats["length"] += 1
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

    # 10. 间距硬约束
    normal_ratio = normal_gap * inv_avg_len
    if normal_ratio < config.MIN_NORMAL_RATIO:
        stats["near"] += 1
        return None
    if normal_ratio > config.MAX_NORMAL_RATIO:
        stats["far"] += 1
        return None

    # 11. 倾斜角硬约束（参考同济 rectangular_error）
    # 用法向距离而不是灯长归一化：远距离时灯长只有几像素且易被阈值切短，
    # 间距才是最可靠的量。上面 normal_ratio >= MIN_NORMAL_RATIO 保证分母 > 0
    tilt = math.atan(parallel_offset / normal_distance) * 180.0 / math.pi
    if tilt > config.MAX_TILT_DEG:
        stats["tilt"] += 1
        return None

    # 12. 各项评分：统一为"离硬约束上限多远"，上限处为 0
    tilt_score = 1.0 - tilt / config.MAX_TILT_DEG
    length_score = 1.0 - length_diff_ratio / config.MAX_LENGTH_DIFF_RATIO
    angle_score = 1.0 - angle_diff / config.MAX_ANGLE_DIFF_DEG

    distance_error = abs(normal_ratio - config.IDEAL_NORMAL_RATIO)
    distance_score = max(0.0, 1.0 - distance_error / config.IDEAL_NORMAL_RATIO)

    # 13. 加权。远距离灯条像素太少，rotation() 基本是噪声，去掉角度项按比例补足
    w_tilt, w_dist, w_len, w_angle = config.PAIR_WEIGHTS
    use_angle = avg_len >= config.ANGLE_SCORE_MIN_LEN
    if not use_angle:
        w_angle = 0.0

    score = (
        w_tilt * tilt_score +
        w_dist * distance_score +
        w_len * length_score +
        w_angle * angle_score
    ) / (w_tilt + w_dist + w_len + w_angle)

    # 14. 快速模式
    if not detail:
        return score

    # 15. 详细模式
    return {
        "score": score,
        "tilt": tilt,
        "normal_ratio": normal_ratio,
        "length_diff_ratio": length_diff_ratio,
        "angle_diff": angle_diff,
        "tilt_score": tilt_score,
        "distance_score": distance_score,
        "length_score": length_score,
        "angle_score": angle_score if use_angle else None,
    }


# ============================================================
# 4. 候选生成与去重（参考同济 sp_vision_25 detector.cpp）
# ============================================================

def find_candidate_pairs(lights):
    """
    两两枚举所有灯条组合，几何合法且 score >= config.MIN_PAIR_SCORE 的进入候选。
    返回 list[candidate dict]，字段：
      left_id / right_id : lights 列表下标，已按 cx 排序（left.cx <= right.cx），
                           仅在本帧、本颜色列表内有效
      left_cx / right_cx : 去重前确定性排序用
      score              : calc_pair_score 快速模式分数
      area               : 两灯条包围框面积 w*h（同济 pattern ROI 面积的替代）
      alive              : 去重存活标记（此处恒为 True）
    """
    candidates = []
    count = len(lights)
    if count < 2:
        return candidates

    for i in range(count):
        a = lights[i]
        for j in range(i + 1, count):
            b = lights[j]

            score = calc_pair_score(a, b)
            if score is None:
                continue
            if score < config.MIN_PAIR_SCORE:
                stats["score"] += 1
                continue

            # 左右按 cx 排
            if a["cx"] <= b["cx"]:
                lid, rid = i, j
            else:
                lid, rid = j, i

            # 包围框面积：与 get_pair_geometry 的 w*h 完全一致
            x1 = min(a["x"], b["x"])
            y1 = min(a["y"], b["y"])
            x2 = max(a["x"] + a["w"], b["x"] + b["w"])
            y2 = max(a["y"] + a["h"], b["y"] + b["h"])

            candidates.append({
                "left_id": lid,
                "right_id": rid,
                "left_cx": lights[lid]["cx"],
                "right_cx": lights[rid]["cx"],
                "score": score,
                "area": (x2 - x1) * (y2 - y1),
                "alive": True,
            })

    # 确定性排序：去重是贪心算法，结果依赖比较顺序；
    # find_blobs 返回顺序在灯条面积相近时可能逐帧翻转，
    # 排序保证同输入同输出，避免输出逐帧抖动。
    candidates.sort(key=lambda c: (c["left_cx"], c["right_cx"], c["score"]))

    return candidates


def dedup_pairs(candidates):
    """
    按同济 sp_vision_25 detector.cpp 规则去重：
      同侧共用灯条（left==left 或 right==right）-> 保 area 小
      交叉共用灯条（left==right 或 right==left）-> 保 score 大
    修复同济链式标记缺陷：已淘汰候选不再参与后续比较（外层 continue、
    内层 continue、a 被淘汰即 break），保证不会全灭。
    返回存活列表（新列表）。
    """
    n = len(candidates)
    for i in range(n):
        a = candidates[i]
        if not a["alive"]:
            continue
        for j in range(i + 1, n):
            b = candidates[j]
            if not b["alive"]:
                continue

            same_left = a["left_id"] == b["left_id"]
            same_right = a["right_id"] == b["right_id"]
            cross_lr = a["left_id"] == b["right_id"]
            cross_rl = a["right_id"] == b["left_id"]

            if not (same_left or same_right or cross_lr or cross_rl):
                continue

            if same_left or same_right:
                # 同侧共用：保面积小（相等时淘汰 a，复刻同济 else 分支语义）
                if a["area"] < b["area"]:
                    b["alive"] = False
                else:
                    a["alive"] = False
            else:
                # 交叉共用：保分数大（相等时淘汰 b，复刻同济 else 分支语义）
                if a["score"] < b["score"]:
                    a["alive"] = False
                else:
                    b["alive"] = False

            # 链式缺陷修复关键：a 一旦被淘汰，立即停止用它去淘汰别人
            if not a["alive"]:
                break

    return [c for c in candidates if c["alive"]]


def _draw_candidates(img, lights, survivors):
    """ 调参绘制：黄线 = 去重后保留的候选对。 """
    for c in survivors:
        la = lights[c["left_id"]]
        lb = lights[c["right_id"]]
        img.draw_line(la["cx"], la["cy"], lb["cx"], lb["cy"],
                      color=(255, 255, 0), thickness=2)


# ============================================================
# 5. 寻找最佳灯条配对
# ============================================================

def find_best_pair(lights, debug_img=None):
    """
    候选生成 -> 去重 -> 最高分胜出。
    多装甲板场景下去重可压制与真对共用灯条的高分伪对。
    debug_img 不为 None 时在其上画出保留的候选对。
    """
    candidates = find_candidate_pairs(lights)
    if not candidates:
        return None

    survivors = dedup_pairs(candidates)

    stats["cand"] += len(candidates)
    stats["dup"] += len(candidates) - len(survivors)

    if debug_img is not None:
        _draw_candidates(debug_img, lights, survivors)

    best = None
    for c in survivors:
        if best is None or c["score"] > best["score"]:
            best = c
    if best is None:
        return None

    left = lights[best["left_id"]]
    right = lights[best["right_id"]]

    detail = calc_pair_score(left, right, detail=True)
    if detail is None:
        # 防御：快速模式已通过，理论上不会发生
        return None

    # 评分明细（tilt / normal_ratio / *_score 等）供 DEBUG_MODE=2 的 SCORE 行打印
    return dict(
        detail,
        left=left,
        right=right,
        geometry_score=detail["score"],
    )


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
# 6. 装甲板几何量
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
