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
)
from armor_tracker import MotionTracker
from coordinate_solver import TargetSolver, map_state, STATE_NAMES
from comm import ArmorComm


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
# 2. 主循环
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

    while True:
        clock.tick()

        # ---------- 1. 采图 ----------
        img = sensor.snapshot()

        # ---------- 2. 单帧检测 ----------
        red_lights, blue_lights = detect_lights(img)

        best_pair, armor_color = select_best_pair(
            find_best_pair(red_lights),
            find_best_pair(blue_lights),
        )

        # ---------- 3. 连续帧跟踪 ----------
        now_ms = time.ticks_ms()

        spacing_px = None
        light_len_px = None
        raw_cx = raw_cy = 0

        if best_pair is not None:
            raw_cx, raw_cy, bx, by, bw, bh = get_pair_geometry(best_pair)
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

        # ---------- 6. Debug 绘制 ----------
        if valid and config.DEBUG_DRAW_PAIR:
            img.draw_cross(cx, cy, size=8, color=(0, 255, 0), thickness=2)

        # 状态 / 颜色变化时打印一次
        if state != last_state or color != last_color:
            print("STATE=%s COLOR=%s" % (STATE_NAMES[state], color))
            last_state = state
            last_color = color

        # ---------- 7. 周期性打印 ----------
        if time.ticks_diff(now_ms, last_debug_print) >= config.DEBUG_PRINT_INTERVAL_MS:
            print("FPS=%.2f" % clock.fps())

            if config.DEBUG_PRINT:
                print(
                    "TARGET state=%s color=%s px=(%d,%d) "
                    "yaw=%.2fdeg pitch=%.2fdeg dist=%.2fm "
                    "size=(%.2f,%.2f)deg rate=(%.2f,%.2f)deg/s" % (
                        STATE_NAMES[out["state"]],
                        out["color"],
                        out["cx"],
                        out["cy"],
                        out["yaw_error"] * 57.2957795,
                        out["pitch_error"] * 57.2957795,
                        out["distance"],
                        out["size_x"] * 57.2957795,
                        out["size_y"] * 57.2957795,
                        out["yaw_rate"] * 57.2957795,
                        out["pitch_rate"] * 57.2957795,
                    )
                )

                if best_pair is not None:
                    print(
                        "PAIR color=%s red=%d blue=%d score=%.2f raw=(%d,%d) "
                        "spacing=%.1fpx len=%.1fpx" % (
                            armor_color,
                            len(red_lights),
                            len(blue_lights),
                            best_pair["geometry_score"],
                            raw_cx,
                            raw_cy,
                            spacing_px,
                            light_len_px,
                        )
                    )
                else:
                    print(
                        "NO_PAIR red=%d blue=%d" % (
                            len(red_lights),
                            len(blue_lights),
                        )
                    )

                if comm is not None:
                    print("COMM frames_sent=%d" % comm.frames_sent)

                print("")

            last_debug_print = now_ms


main()
