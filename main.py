# ============================================================
# main.py
# OpenMV 装甲板识别 -> 跟踪 -> 坐标解算 -> MAVLink 发送
#
# 本文件只做编排，是项目中唯一接触 sensor 的文件。
#
# 数据流：
#   snapshot
#     -> armor_detect   ：灯条检测 / 配对        （单帧）
#     -> armor_tracker  ：连续帧确认 / 运动预测  （时间维度）
#     -> coordinate_solver：像素 -> 角度 / 距离 / 状态
#     -> comm           ：MAVLink v2 LANDING_TARGET over UART
# ============================================================

import sensor
import time

import config

from armor_detect import (
    detect_lights,
    find_best_pair,
    select_best_pair,
    get_pair_geometry,
    get_pair_measurements,
    stats as detect_stats,
    blob_log,
    reset_stats,
)
from armor_tracker import MotionTracker
from coordinate_solver import (
    TargetSolver,
    map_state,
    STATE_NAMES,
    STATE_TRACKING,
    STATE_SUSPECT,
)
from comm import ArmorComm


RAD2DEG = 57.2957795


# ============================================================
# 1. 初始化
# ============================================================

def setup_sensor():
    sensor.reset()
    # RGB565：后续需要颜色识别
    sensor.set_pixformat(sensor.RGB565)
    # QVGA = 320 x 240，与 camera_params.py 的标定分辨率一致
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    # 关闭自动增益 / 白平衡 / 曝光，避免颜色阈值漂移
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False, exposure_us=config.EXPOSURE_US)


def setup_comm():
    """
    串口初始化失败（端口配置错误等）时不中断视觉流程，
    只打印错误并以无通信模式继续运行，便于在 IDE 中单独调试识别。
    """
    try:
        comm = ArmorComm()
        print("COMM: UART%d @ %d, MAVLink sysid=%d compid=%d" % (
            config.UART_PORT,
            config.UART_BAUD,
            config.MAV_SYSTEM_ID,
            config.MAV_COMPONENT_ID,
        ))
        return comm
    except Exception as e:
        print("COMM: UART init failed, running without comm:", e)
        return None


# ============================================================
# 2. 调试输出
# ============================================================

# 灯条筛选阶段剔除原因 -> 提示
LIGHT_HINTS = {
    "size": "亮斑多被尺寸筛掉：目标太远 / 太暗，或 MIN_PIXELS / MIN_LIGHT_LEN / MIN_ELONGATION 太严",
    "ratio": "灯条宽长比不合格：DEBUG_MODE=2 看 BLOB 行 ratio，调 MIN_LIGHT_RATIO / MAX_LIGHT_RATIO",
    "fill": "灯条填充率不合格：DEBUG_MODE=2 看 BLOB 行 fill，调 MIN_FILL_RATIO（LAB 模式约 0.6）",
    "color": "判色失败（白光 / 杂色）：DEBUG_MODE=2 看 BLOB 行 diff，调 COLOR_DIFF_MIN / COLOR_SAMPLE_L_MIN，或曝光过高灯条发白",
}

# 配对阶段剔除原因 -> 提示
PAIR_HINTS = {
    "angle": "两灯条角度差超限：调 MAX_ANGLE_DIFF_DEG",
    "length": "两灯条长度差超限（一根被截断 / 过曝粘连？）：调 MAX_LENGTH_DIFF_RATIO",
    "near": "两灯条间距过小（斜视 / 小装甲板）：调 MIN_NORMAL_RATIO",
    "far": "两灯条间距过大（大装甲板）：调 MAX_NORMAL_RATIO / IDEAL_NORMAL_RATIO",
    "tilt": "两灯条连线倾斜过大：调 MAX_TILT_DEG；若画面上灯条明显被切短，先降 LIGHT_L_MIN",
    "score": "几何得分低于 MIN_PAIR_SCORE：DEBUG_MODE=2 看 SCORE 行哪一项低",
}

LIGHT_KEYS = ("size", "ratio", "fill", "color")
PAIR_KEYS = ("angle", "length", "near", "far", "tilt", "score")


def _max_key(avg, keys):
    best = keys[0]
    for k in keys:
        if avg[k] > avg[best]:
            best = k
    return best


def _bottleneck(avg):
    """
    最可能的瓶颈：同色灯条不足 2 根时看筛选阶段，否则看配对阶段，
    取该阶段剔除数最多的一项。配对阶段没有剔除记录时返回 None。
    """
    if max(avg["RED"], avg["BLUE"]) < 2:
        key = _max_key(avg, LIGHT_KEYS)
        if avg[key] > 0:
            return LIGHT_HINTS[key]
        # 没有任何剔除记录：灯条根本没被 find_blobs 提取出来
        return "灯条没被提取到（目标不在画面内或太暗）：降 LIGHT_L_MIN 或升 EXPOSURE_US"

    key = _max_key(avg, PAIR_KEYS)
    if avg[key] > 0:
        return "配对失败，" + PAIR_HINTS[key]
    return None


def diagnose(avg, frames, pair_frames, state_frames):
    """
    沿检测漏斗从前往后找第一个明显瓶颈，返回一句提示。
    avg 为本周期每帧平均计数。
    """
    if state_frames[STATE_TRACKING] >= 0.9 * frames:
        if avg["blobs"] > 30:
            return "跟踪稳定，但亮斑过多拖慢帧率：降 EXPOSURE_US 或升 LIGHT_L_MIN"
        return "OK"

    # 一半以上的帧没有装甲板：问题在检测
    if pair_frames < 0.5 * frames:
        if avg["blobs"] < 1:
            return "几乎没有亮斑：EXPOSURE_US 太低或 LIGHT_L_MIN 太高"

        hint = _bottleneck(avg)
        if hint is not None:
            return hint
        return "同色灯条数够但配不成对：检查红蓝是否判反"

    # 检测基本正常：问题在跟踪
    if state_frames[STATE_SUSPECT] > state_frames[STATE_TRACKING]:
        return "检测到装甲板但锁不住：位置 / 颜色逐帧跳变，调 acquire_* 门限或 confirm_frames"

    hint = _bottleneck(avg)
    return "检测时有时无（有对帧 %d%%）：逐帧闪断，%s" % (
        pair_frames * 100 // frames,
        "看哪一步剔除数偏高" if hint is None else "最可能：" + hint,
    )


def print_report(fps, frames, pair_frames, state_frames, out, best_pair,
                 armor_color, raw_cx, raw_cy, spacing_px, light_len_px, comm):
    """ 周期报告（DEBUG_MODE >= 1）。 """
    avg = {}
    for k in detect_stats:
        avg[k] = detect_stats[k] / frames

    print("FPS=%.2f frames=%d pair=%d | %s" % (
        fps,
        frames,
        pair_frames,
        " ".join(["%s=%d" % (STATE_NAMES[i], state_frames[i]) for i in range(4)]),
    ))

    print(
        "DETECT/帧 blobs=%.1f -> size=%.1f ratio=%.1f fill=%.1f color=%.1f "
        "-> R=%.1f B=%.1f" % (
            avg["blobs"], avg["size"], avg["ratio"], avg["fill"], avg["color"],
            avg["RED"], avg["BLUE"],
        )
    )
    print(
        "PAIR/帧  angle=%.1f length=%.1f near=%.1f far=%.1f tilt=%.1f "
        "score=%.1f -> cand=%.1f dup=%.1f" % (
            avg["angle"], avg["length"], avg["near"], avg["far"], avg["tilt"],
            avg["score"], avg["cand"], avg["dup"],
        )
    )

    if out["valid"]:
        print(
            "TARGET state=%s color=%s px=(%d,%d) "
            "yaw=%.2fdeg pitch=%.2fdeg dist=%.2fm "
            "size=(%.2f,%.2f)deg rate=(%.2f,%.2f)deg/s" % (
                STATE_NAMES[out["state"]],
                out["color"],
                out["cx"],
                out["cy"],
                out["yaw_error"] * RAD2DEG,
                out["pitch_error"] * RAD2DEG,
                out["distance"],
                out["size_x"] * RAD2DEG,
                out["size_y"] * RAD2DEG,
                out["yaw_rate"] * RAD2DEG,
                out["pitch_rate"] * RAD2DEG,
            )
        )

    if best_pair is not None:
        print("PAIR color=%s score=%.2f raw=(%d,%d) spacing=%.1fpx len=%.1fpx" % (
            armor_color,
            best_pair["geometry_score"],
            raw_cx,
            raw_cy,
            spacing_px,
            light_len_px,
        ))

    if comm is not None:
        print("COMM frames_sent=%d" % comm.frames_sent)

    print("HINT " + diagnose(avg, frames, pair_frames, state_frames))

    if config.DEBUG_MODE >= 2:
        if best_pair is not None:
            # 括号内为实测值；angle=off 表示灯条过短，角度项未参与评分
            angle_score = best_pair["angle_score"]
            print(
                "SCORE tilt=%.2f(%.1fdeg) dist=%.2f(%.2f) "
                "length=%.2f(%.2f) angle=%s(%.1fdeg)" % (
                    best_pair["tilt_score"], best_pair["tilt"],
                    best_pair["distance_score"], best_pair["normal_ratio"],
                    best_pair["length_score"], best_pair["length_diff_ratio"],
                    "off" if angle_score is None else "%.2f" % angle_score,
                    best_pair["angle_diff"],
                )
            )

        # 最后一帧的亮斑，按像素数从大到小，最多 10 条
        for cx, cy, pixels, ratio, fill, diff, result in sorted(
            blob_log, key=lambda b: b[2], reverse=True
        )[:10]:
            print("BLOB (%d,%d) pix=%d ratio=%s fill=%.2f diff=%s -> %s" % (
                cx,
                cy,
                pixels,
                "-" if ratio is None else "%.2f" % ratio,
                fill,
                "-" if diff is None else "%+d" % diff,
                result,
            ))

    print("")


# ============================================================
# 3. 主循环
# ============================================================

def main():
    setup_sensor()

    clock = time.clock()

    tracker = MotionTracker(**config.TRACKER_PARAMS)
    solver = TargetSolver()
    comm = setup_comm()

    last_debug_print = time.ticks_ms()
    last_state = None
    last_color = None

    # 周期统计（每次周期打印后清零）
    frames = 0
    pair_frames = 0
    state_frames = [0, 0, 0, 0]

    while True:
        clock.tick()

        # ---------- 1. 采图 ----------
        img = sensor.snapshot()

        # ---------- 2. 单帧检测 ----------
        red_lights, blue_lights = detect_lights(img)

        debug_img = img if config.DEBUG_MODE >= 2 else None
        best_pair, armor_color = select_best_pair(
            find_best_pair(red_lights, debug_img),
            find_best_pair(blue_lights, debug_img),
        )

        # ---------- 3. 连续帧跟踪 ----------
        now_ms = time.ticks_ms()

        spacing_px = None
        light_len_px = None
        raw_cx = raw_cy = 0

        if best_pair is not None:
            raw_cx, raw_cy, _, _, bw, bh = get_pair_geometry(best_pair)
            spacing_px, light_len_px = get_pair_measurements(best_pair)

            valid, color, cx, cy = tracker.update(
                True,
                now_ms,
                armor_color,
                raw_cx,
                raw_cy,
                max(bw, bh),
            )
        else:
            valid, color, cx, cy = tracker.update(False, now_ms)

        state = map_state(
            valid,
            tracker.get_lost_count(),
            tracker.get_pending_count(),
        )
        vx, vy = tracker.get_velocity()

        # 防止短暂预测把坐标带出图像范围
        if valid:
            cx = max(0, min(img.width() - 1, cx))
            cy = max(0, min(img.height() - 1, cy))

        # ---------- 4. 坐标解算 ----------
        out = solver.solve(
            valid,
            state,
            color,
            cx,
            cy,
            vx,
            vy,
            spacing_px,
            light_len_px,
        )

        # ---------- 5. 发送 ----------
        if comm is not None:
            comm.update(out, now_ms)

        # ---------- 6. Debug ----------
        frames += 1
        if best_pair is not None:
            pair_frames += 1
        state_frames[state] += 1

        if config.DEBUG_MODE >= 1:
            if valid:
                img.draw_cross(cx, cy, size=8, color=(0, 255, 0), thickness=2)

            # 状态 / 颜色变化时实时打印
            if state != last_state or color != last_color:
                print("STATE=%s COLOR=%s" % (STATE_NAMES[state], color))
                last_state = state
                last_color = color

        # ---------- 7. 周期打印 ----------
        if time.ticks_diff(now_ms, last_debug_print) >= config.DEBUG_PRINT_INTERVAL_MS:
            if config.DEBUG_MODE >= 1:
                print_report(
                    clock.fps(), frames, pair_frames, state_frames, out,
                    best_pair, armor_color, raw_cx, raw_cy,
                    spacing_px, light_len_px, comm,
                )
            else:
                print("FPS=%.2f" % clock.fps())

            frames = 0
            pair_frames = 0
            state_frames = [0, 0, 0, 0]
            reset_stats()
            last_debug_print = now_ms


main()
