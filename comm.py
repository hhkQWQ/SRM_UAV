# ============================================================
# comm.py
# UART 通信层：把解算结果打包成 MAVLink 帧并发送
#
# 这是项目中唯一接触串口硬件的文件。
# ============================================================

import time

import config
from mavlink import MAVLinkV2


def _open_uart():
    """
    打开 OpenMV 的 UART。
    固件 4.5.x 同时提供 machine.UART 与 pyb.UART，优先用 machine。
    """
    try:
        from machine import UART
    except ImportError:
        from pyb import UART

    return UART(config.UART_PORT, config.UART_BAUD)


class ArmorComm:

    def __init__(self):
        self.uart = _open_uart()

        self.mav = MAVLinkV2(
            config.MAV_SYSTEM_ID,
            config.MAV_COMPONENT_ID,
        )

        self.last_send_ms = None
        self.last_heartbeat_ms = None

        self.frames_sent = 0

    # --------------------------------------------------------
    # 内部
    # --------------------------------------------------------

    def _write(self, frame):
        self.uart.write(frame)
        self.frames_sent += 1

        if config.DEBUG_PRINT_HEX_FRAME:
            print("TX " + "".join("%02X" % b for b in frame))

    # --------------------------------------------------------
    # 对外接口
    # --------------------------------------------------------

    def send_heartbeat(self, now_ms):
        self._write(self.mav.heartbeat(trim=config.TRIM_PAYLOAD))
        self.last_heartbeat_ms = now_ms

    def send_target(self, out, now_ms):
        """
        把 TargetSolver.solve() 的输出打包为 LANDING_TARGET 发送。
        返回是否真正发出。
        """
        if not out["valid"] and not config.SEND_WHEN_LOST:
            return False

        if (
            config.SEND_INTERVAL_MS > 0 and
            self.last_send_ms is not None and
            time.ticks_diff(now_ms, self.last_send_ms) < config.SEND_INTERVAL_MS
        ):
            return False

        frame = self.mav.landing_target(
            time_usec=now_ms * 1000,
            target_num=out["target_num"],
            frame=config.LANDING_TARGET_FRAME,
            angle_x=out["yaw_error"],
            angle_y=out["pitch_error"],
            distance=out["distance"],
            size_x=out["size_x"],
            size_y=out["size_y"],
            x=out["x"],
            y=out["y"],
            z=out["z"],
            msg_type=config.LANDING_TARGET_TYPE,
            position_valid=1 if out["valid"] else 0,
            trim=config.TRIM_PAYLOAD,
        )

        self._write(frame)
        self.last_send_ms = now_ms
        return True

    def update(self, out, now_ms):
        """
        每帧调用一次：按需发心跳，再发目标。
        返回本帧是否发出了 LANDING_TARGET。
        """
        if config.SEND_HEARTBEAT and (
            self.last_heartbeat_ms is None or
            time.ticks_diff(now_ms, self.last_heartbeat_ms) >= config.HEARTBEAT_INTERVAL_MS
        ):
            self.send_heartbeat(now_ms)

        return self.send_target(out, now_ms)
