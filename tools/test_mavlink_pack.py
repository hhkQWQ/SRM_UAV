# ============================================================
# tools/test_mavlink_pack.py
# 用本机 pymavlink 作为参考实现，对自研 mavlink.py 做逐字节对拍。
#
# 这是协议正确性的决定性证据：
#   同一组输入 -> 两个实现分别打包 -> 整帧（含 CRC）必须完全相同。
#
# 用法：python tools/test_mavlink_pack.py
# ============================================================

import os
import random
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink.dialects.v20 import common as ref

import mavlink


SYSID = 1
COMPID = 100


def ref_mav(seq):
    """ 构造一个与我们 seq 对齐的 pymavlink 发送器 """
    m = ref.MAVLink(None, srcSystem=SYSID, srcComponent=COMPID)
    m.seq = seq
    return m


def check(name, ours, theirs):
    ok = ours == theirs
    print("%-50s %s" % (name, "PASS" if ok else "FAIL"))
    if not ok:
        print("   ours  :", ours.hex())
        print("   theirs:", theirs.hex())
        n = min(len(ours), len(theirs))
        for i in range(n):
            if ours[i] != theirs[i]:
                print("   first diff at byte %d: %02x vs %02x" % (i, ours[i], theirs[i]))
                break
        if len(ours) != len(theirs):
            print("   length %d vs %d" % (len(ours), len(theirs)))
    return ok


def main():
    random.seed(20261004)
    all_ok = True

    # ---------- 1. CRC 已知向量 ----------
    # MAVLink 官方 crc_calculate 对空串应为 0xFFFF，对 "123456789" 为 0x6F91
    all_ok &= check(
        "crc16: empty -> 0xFFFF",
        struct.pack("<H", mavlink.crc16_mcrf4xx(b"")),
        struct.pack("<H", 0xFFFF),
    )
    all_ok &= check(
        "crc16: '123456789' -> 0x6F91",
        struct.pack("<H", mavlink.crc16_mcrf4xx(b"123456789")),
        struct.pack("<H", 0x6F91),
    )

    # ---------- 2. HEARTBEAT ----------
    for seq in (0, 1, 127, 255):
        ours_mav = mavlink.MAVLinkV2(SYSID, COMPID)
        ours_mav.seq = seq
        ours = ours_mav.heartbeat()

        theirs = ref_mav(seq).heartbeat_encode(
            type=mavlink.MAV_TYPE_CAMERA,
            autopilot=mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=mavlink.MAV_STATE_ACTIVE,
        ).pack(ref_mav(seq))

        all_ok &= check("HEARTBEAT seq=%d" % seq, ours, theirs)

    # ---------- 3. LANDING_TARGET：固定可读向量 ----------
    fixed = dict(
        time_usec=0x0102030405060708,
        target_num=0xAA,
        frame=12,
        angle_x=0.1234,
        angle_y=-0.5678,
        distance=3.5,
        size_x=0.02,
        size_y=0.01,
        x=3.5,
        y=0.43,
        z=-2.2,
        q=(1.0, 0.0, 0.0, 0.0),
        type=3,
        position_valid=1,
    )
    ours_mav = mavlink.MAVLinkV2(SYSID, COMPID)
    ours = ours_mav.landing_target(
        time_usec=fixed["time_usec"], target_num=fixed["target_num"], frame=fixed["frame"],
        angle_x=fixed["angle_x"], angle_y=fixed["angle_y"], distance=fixed["distance"],
        size_x=fixed["size_x"], size_y=fixed["size_y"], x=fixed["x"], y=fixed["y"], z=fixed["z"],
        q=fixed["q"], msg_type=fixed["type"], position_valid=fixed["position_valid"],
    )
    theirs = ref_mav(0).landing_target_encode(**fixed).pack(ref_mav(0))
    all_ok &= check("LANDING_TARGET fixed vector (no trim)", ours, theirs)

    # ---------- 4. LANDING_TARGET：随机向量 ----------
    # pymavlink 的 v2 打包总是裁掉载荷尾部 0 字节，
    # 随机向量可能出现 type=0 且 position_valid=0，
    # 因此这里统一用 trim=True 才是同语义比较。
    for i in range(200):
        seq = random.randrange(256)
        kw = dict(
            time_usec=random.getrandbits(64),
            target_num=random.randrange(256),
            frame=random.randrange(256),
            angle_x=random.uniform(-1.6, 1.6),
            angle_y=random.uniform(-1.6, 1.6),
            distance=random.uniform(0, 50),
            size_x=random.uniform(0, 1),
            size_y=random.uniform(0, 1),
            x=random.uniform(-50, 50),
            y=random.uniform(-50, 50),
            z=random.uniform(-50, 50),
            q=tuple(random.uniform(-1, 1) for _ in range(4)),
            type=random.randrange(256),
            position_valid=random.randrange(2),
        )
        ours_mav = mavlink.MAVLinkV2(SYSID, COMPID)
        ours_mav.seq = seq
        ours = ours_mav.landing_target(
            time_usec=kw["time_usec"], target_num=kw["target_num"], frame=kw["frame"],
            angle_x=kw["angle_x"], angle_y=kw["angle_y"], distance=kw["distance"],
            size_x=kw["size_x"], size_y=kw["size_y"], x=kw["x"], y=kw["y"], z=kw["z"],
            q=kw["q"], msg_type=kw["type"], position_valid=kw["position_valid"],
            trim=True,
        )
        theirs = ref_mav(seq).landing_target_encode(**kw).pack(ref_mav(seq))
        if ours != theirs:
            all_ok &= check("LANDING_TARGET random #%d" % i, ours, theirs)
            break
    else:
        print("%-50s PASS" % "LANDING_TARGET 200 random vectors")

    # ---------- 5. LANDING_TARGET：零载荷尾部 + trim ----------
    # pymavlink 默认按 v2 规范截断尾部 0；我们用 trim=True 应与其一致
    zero_tail = dict(
        time_usec=123456789, target_num=5, frame=12,
        angle_x=0.1, angle_y=0.2, distance=1.0, size_x=0.0, size_y=0.0,
        x=0.0, y=0.0, z=0.0, q=(0.0, 0.0, 0.0, 0.0), type=0, position_valid=0,
    )
    ours_mav = mavlink.MAVLinkV2(SYSID, COMPID)
    ours = ours_mav.landing_target(
        time_usec=zero_tail["time_usec"], target_num=zero_tail["target_num"], frame=zero_tail["frame"],
        angle_x=zero_tail["angle_x"], angle_y=zero_tail["angle_y"], distance=zero_tail["distance"],
        size_x=zero_tail["size_x"], size_y=zero_tail["size_y"], x=0.0, y=0.0, z=0.0,
        q=zero_tail["q"], msg_type=0, position_valid=0, trim=True,
    )
    theirs = ref_mav(0).landing_target_encode(**zero_tail).pack(ref_mav(0))
    all_ok &= check("LANDING_TARGET trailing zeros, trim=True", ours, theirs)
    print("   trimmed frame length: ours=%d theirs=%d (full would be 72)" % (len(ours), len(theirs)))

    # ---------- 6. 我们发的帧能被 pymavlink 解析回原值 ----------
    parser = ref.MAVLink(None)
    parser.robust_parsing = True
    ours_mav = mavlink.MAVLinkV2(SYSID, COMPID)
    frame = ours_mav.landing_target(
        time_usec=fixed["time_usec"], target_num=fixed["target_num"], frame=fixed["frame"],
        angle_x=fixed["angle_x"], angle_y=fixed["angle_y"], distance=fixed["distance"],
        size_x=fixed["size_x"], size_y=fixed["size_y"], x=fixed["x"], y=fixed["y"], z=fixed["z"],
        q=fixed["q"], msg_type=fixed["type"], position_valid=fixed["position_valid"],
    )
    decoded = parser.parse_buffer(frame)
    ok = (
        decoded is not None and len(decoded) == 1 and
        decoded[0].get_type() == "LANDING_TARGET" and
        decoded[0].time_usec == fixed["time_usec"] and
        decoded[0].target_num == fixed["target_num"] and
        abs(decoded[0].angle_x - fixed["angle_x"]) < 1e-6 and
        abs(decoded[0].distance - fixed["distance"]) < 1e-6 and
        decoded[0].position_valid == 1 and
        decoded[0].get_srcSystem() == SYSID and
        decoded[0].get_srcComponent() == COMPID
    )
    print("%-50s %s" % ("pymavlink parses our frame back to same values", "PASS" if ok else "FAIL"))
    all_ok &= ok

    print()
    print("ALL PASS" if all_ok else "SOME TESTS FAILED")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
