# ============================================================
# mavlink.py
# 最小 MAVLink v2 组帧器（MicroPython，无第三方依赖）
#
# 只实现本项目需要的两条消息：
#   HEARTBEAT      (#0,   crc_extra = 50)
#   LANDING_TARGET (#149, crc_extra = 200)
#
# 帧格式（v2）：
#   STX(0xFD) | len | incompat | compat | seq | sysid | compid |
#   msgid(3 字节小端) | payload | crc16(2 字节小端)
#
# CRC：CRC-16/MCRF4XX（即 MAVLink X.25），覆盖 len ~ payload 末尾，
#      再追加 1 字节 crc_extra。
#
# LANDING_TARGET 载荷线序（60 字节，无对齐填充，由 pymavlink 实测）：
#   off  size  field
#     0     8  time_usec      uint64
#     8     4  angle_x        float
#    12     4  angle_y        float
#    16     4  distance       float
#    20     4  size_x         float
#    24     4  size_y         float
#    28     1  target_num     uint8
#    29     1  frame          uint8
#    30     4  x              float
#    34     4  y              float
#    38     4  z              float
#    42    16  q[4]           float[4]
#    58     1  type           uint8
#    59     1  position_valid uint8
# ============================================================

import struct


MAVLINK_STX_V2 = 0xFD

MAVLINK_MSG_ID_HEARTBEAT = 0
HEARTBEAT_CRC_EXTRA = 50
HEARTBEAT_FMT = "<IBBBBB"

MAVLINK_MSG_ID_LANDING_TARGET = 149
LANDING_TARGET_CRC_EXTRA = 200
# time_usec 单独按两个 uint32 打包（见 _pack_u64），这里是其后的 52 字节
LANDING_TARGET_FMT = "<fffffBBfffffffBB"

# HEARTBEAT 常用枚举
MAV_TYPE_CAMERA = 30
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4
MAVLINK_VERSION = 3


# ============================================================
# 1. CRC-16/MCRF4XX（MAVLink 官方 crc_accumulate 的逐字节实现）
# ============================================================

def crc16_mcrf4xx(data, crc=0xFFFF):
    for b in data:
        tmp = (b ^ crc) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def _pack_u64(value):
    """
    小端 uint64。拆成两个 uint32 打包，
    避免依赖 MicroPython 对 '<Q' 的支持，字节与 '<Q' 完全一致。
    """
    return struct.pack(
        "<II",
        value & 0xFFFFFFFF,
        (value >> 32) & 0xFFFFFFFF,
    )


# ============================================================
# 2. 组帧器
# ============================================================

class MAVLinkV2:

    def __init__(self, system_id=1, component_id=100):
        self.system_id = system_id & 0xFF
        self.component_id = component_id & 0xFF
        self.seq = 0

    # --------------------------------------------------------
    # 通用组帧
    # --------------------------------------------------------

    def pack(self, msg_id, payload, crc_extra, trim=False):
        """
        把 payload 封装为一帧 MAVLink v2，返回 bytes。
        trim=True 时按 v2 规范裁掉载荷末尾的 0 字节（至少保留 1 字节）。
        """
        if trim:
            end = len(payload)
            while end > 1 and payload[end - 1] == 0:
                end -= 1
            payload = payload[:end]

        header = bytes([
            MAVLINK_STX_V2,
            len(payload),
            0,                          # incompat_flags
            0,                          # compat_flags
            self.seq,
            self.system_id,
            self.component_id,
            msg_id & 0xFF,
            (msg_id >> 8) & 0xFF,
            (msg_id >> 16) & 0xFF,
        ])

        # CRC 不含 STX，覆盖 len ~ payload，最后混入 crc_extra
        crc = crc16_mcrf4xx(header[1:])
        crc = crc16_mcrf4xx(payload, crc)
        crc = crc16_mcrf4xx(bytes([crc_extra]), crc)

        self.seq = (self.seq + 1) & 0xFF

        return header + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    # --------------------------------------------------------
    # HEARTBEAT (#0)
    # --------------------------------------------------------

    def heartbeat(
        self,
        mav_type=MAV_TYPE_CAMERA,
        autopilot=MAV_AUTOPILOT_INVALID,
        base_mode=0,
        custom_mode=0,
        system_status=MAV_STATE_ACTIVE,
        trim=False,
    ):
        payload = struct.pack(
            HEARTBEAT_FMT,
            custom_mode & 0xFFFFFFFF,
            mav_type & 0xFF,
            autopilot & 0xFF,
            base_mode & 0xFF,
            system_status & 0xFF,
            MAVLINK_VERSION,
        )
        return self.pack(
            MAVLINK_MSG_ID_HEARTBEAT,
            payload,
            HEARTBEAT_CRC_EXTRA,
            trim,
        )

    # --------------------------------------------------------
    # LANDING_TARGET (#149)
    # --------------------------------------------------------

    def landing_target(
        self,
        time_usec,
        target_num,
        frame,
        angle_x,
        angle_y,
        distance,
        size_x,
        size_y,
        x=0.0,
        y=0.0,
        z=0.0,
        q=(1.0, 0.0, 0.0, 0.0),
        msg_type=0,
        position_valid=0,
        trim=False,
    ):
        payload = _pack_u64(int(time_usec)) + struct.pack(
            LANDING_TARGET_FMT,
            float(angle_x),
            float(angle_y),
            float(distance),
            float(size_x),
            float(size_y),
            target_num & 0xFF,
            frame & 0xFF,
            float(x),
            float(y),
            float(z),
            float(q[0]),
            float(q[1]),
            float(q[2]),
            float(q[3]),
            msg_type & 0xFF,
            position_valid & 0xFF,
        )
        return self.pack(
            MAVLINK_MSG_ID_LANDING_TARGET,
            payload,
            LANDING_TARGET_CRC_EXTRA,
            trim,
        )
