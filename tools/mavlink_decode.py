# ============================================================
# tools/mavlink_decode.py
# PC 侧 MAVLink 解码工具（CPython + pymavlink）
#
# 用途：无飞控阶段的端到端验证。
#   OpenMV UART -> USB-TTL -> PC 串口 -> 本脚本解码并打印。
#
# 用法：
#   python tools/mavlink_decode.py --port COM5
#   python tools/mavlink_decode.py --port /dev/ttyUSB0 --baud 115200
#   python tools/mavlink_decode.py --port COM5 --raw   # 同时打印原始字节
#
# 依赖：pip install pymavlink pyserial
# ============================================================

import argparse
import math
import sys
import time

from pymavlink import mavutil


STATE_NAMES = ("LOST", "TRACKING", "TEMP_LOST", "SUSPECT")
COLOR_NAMES = ("NONE", "RED", "BLUE")


def decode_target_num(target_num):
    """ 与 coordinate_solver.encode_target_num 对应 """
    return (target_num >> 2) & 0x03, target_num & 0x03


def fmt_landing_target(msg):
    state, color = decode_target_num(msg.target_num)
    deg = 180.0 / math.pi

    return (
        "LANDING_TARGET  t=%10.3fs  state=%-9s color=%-4s valid=%d  "
        "yaw=%+7.2fdeg pitch=%+7.2fdeg dist=%6.2fm  "
        "size=(%.2f,%.2f)deg  xyz=(%.2f,%.2f,%.2f)m  frame=%d type=%d"
        % (
            msg.time_usec / 1e6,
            STATE_NAMES[state],
            COLOR_NAMES[color],
            msg.position_valid,
            msg.angle_x * deg,
            msg.angle_y * deg,
            msg.distance,
            msg.size_x * deg,
            msg.size_y * deg,
            msg.x,
            msg.y,
            msg.z,
            msg.frame,
            msg.type,
        )
    )


def main():
    parser = argparse.ArgumentParser(description="Decode OpenMV MAVLink output")
    parser.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--raw", action="store_true", help="also print raw frame bytes")
    parser.add_argument("--all", action="store_true", help="print every message type, not only LANDING_TARGET/HEARTBEAT")
    args = parser.parse_args()

    print("Opening %s @ %d ..." % (args.port, args.baud))
    conn = mavutil.mavlink_connection(args.port, baud=args.baud, dialect="common")

    count = 0
    bad_crc_last = 0
    t_last_stat = time.time()

    try:
        while True:
            msg = conn.recv_match(blocking=True, timeout=1.0)

            if msg is None:
                # 1 秒没数据，报告一下链路状态
                print("... no data (bad_crc=%d, total=%d)" % (
                    conn.mav.total_receive_errors, count))
                continue

            mtype = msg.get_type()

            if mtype == "BAD_DATA":
                continue

            count += 1

            if mtype == "LANDING_TARGET":
                print(fmt_landing_target(msg))
            elif mtype == "HEARTBEAT":
                print("HEARTBEAT       sysid=%d compid=%d type=%d autopilot=%d status=%d" % (
                    msg.get_srcSystem(), msg.get_srcComponent(),
                    msg.type, msg.autopilot, msg.system_status))
            elif args.all:
                print(msg)

            if args.raw:
                print("   raw: " + msg.get_msgbuf().hex())

            now = time.time()
            if now - t_last_stat >= 5.0:
                errs = conn.mav.total_receive_errors
                print("--- stats: msgs=%d  crc_errors=%d (+%d) ---" % (
                    count, errs, errs - bad_crc_last))
                bad_crc_last = errs
                t_last_stat = now

    except KeyboardInterrupt:
        print("\nstopped. total msgs=%d crc_errors=%d" % (
            count, conn.mav.total_receive_errors))
        return 0


if __name__ == "__main__":
    sys.exit(main())
