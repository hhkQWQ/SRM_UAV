# ============================================================
# tools/test_coordinate_solver.py
# coordinate_solver.py 的 PC 侧单元验证（CPython，无需硬件）
#
# 用法：python tools/test_coordinate_solver.py
# ============================================================

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
import camera_params as cam
import coordinate_solver as cs


DEG = 180.0 / math.pi
results = []


def check(name, cond, detail=""):
    results.append(cond)
    print("%-60s %s %s" % (name, "PASS" if cond else "FAIL", detail))


def distort_forward(x, y):
    """ OpenCV 正向畸变模型：无畸变归一化坐标 -> 畸变后像素坐标 """
    k1, k2, p1, p2, k3 = cam.DIST_COEFFS
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return cam.FX * xd + cam.CX, cam.FY * yd + cam.CY


def main():
    # ---------- 1. 主点 -> 零角度 ----------
    yaw, pitch = cs.pixel_to_angles(cam.CX, cam.CY, undistort=False)
    check("principal point -> yaw=pitch=0 (no undistort)",
          abs(yaw) < 1e-9 and abs(pitch) < 1e-9)

    yaw, pitch = cs.pixel_to_angles(cam.CX, cam.CY, undistort=True)
    check("principal point -> yaw=pitch=0 (undistort)",
          abs(yaw) < 1e-9 and abs(pitch) < 1e-9)

    # ---------- 2. 已知角度反解（无畸变路径精确） ----------
    for a in (5.0, 10.0, 20.0, -15.0):
        u = cam.CX + cam.FX * math.tan(a / DEG)
        yaw, _ = cs.pixel_to_angles(u, cam.CY, undistort=False)
        check("known yaw %+5.1f deg reconstructs" % a,
              abs(yaw * DEG - a) < 1e-9, "got %.6f" % (yaw * DEG))

    # ---------- 3. 符号约定 ----------
    yaw_r, _ = cs.pixel_to_angles(cam.CX + 50, cam.CY)
    _, pitch_d = cs.pixel_to_angles(cam.CX, cam.CY + 50)
    check("right of center -> yaw > 0", yaw_r > 0)
    check("below center (image y down) -> pitch > 0", pitch_d > 0)

    # ---------- 4. 去畸变往返：正向畸变 -> 反解 应恢复原值 ----------
    worst = 0.0
    for x in (-0.5, -0.3, 0.0, 0.2, 0.45):
        for y in (-0.38, -0.1, 0.0, 0.25, 0.4):
            u, v = distort_forward(x, y)
            xr, yr = cs.undistort_point(u, v)
            worst = max(worst, abs(xr - x), abs(yr - y))
    check("undistort round-trip on 25-point grid (err < 1e-7)",
          worst < 1e-7, "worst=%.2e" % worst)

    # ---------- 5. 开 / 关去畸变的差异量级（应与分析一致，<= 0.35 deg） ----------
    max_diff = 0.0
    for u in (0, 80, 160, 240, 319):
        for v in (0, 60, 120, 180, 239):
            y1, p1 = cs.pixel_to_angles(u, v, undistort=True)
            y0, p0 = cs.pixel_to_angles(u, v, undistort=False)
            max_diff = max(max_diff, abs(y1 - y0) * DEG, abs(p1 - p0) * DEG)
    check("undistort vs raw max angle diff <= 0.35 deg", max_diff <= 0.35,
          "max=%.3f deg" % max_diff)
    check("undistort actually changes edge angles (> 0.2 deg)", max_diff > 0.2)

    # ---------- 6. 径向尺度：中心 1.0，边缘 ~1.013 ----------
    xn, yn = cs.pixel_to_normalized(cam.CX, cam.CY)
    check("radial_scale at center == 1.0",
          abs(cs.radial_scale(cam.CX, cam.CY, xn, yn) - 1.0) < 1e-9)
    xn, yn = cs.pixel_to_normalized(319, 120)
    s = cs.radial_scale(319, 120, xn, yn)
    check("radial_scale at right edge in [1.005, 1.02]", 1.005 < s < 1.02, "s=%.4f" % s)

    # ---------- 7. 距离公式 ----------
    config.DISTANCE_METHOD = "SPACING"
    L = config.REAL_LIGHT_SPACING_MM * 0.001
    for D in (1.0, 2.0, 5.0):
        spacing_px = cam.FX * L / D
        d = cs.estimate_distance(spacing_px, 1.0)
        check("distance SPACING: %.0fm armor reconstructs" % D, abs(d - D) < 1e-9)

    config.DISTANCE_METHOD = "LENGTH"
    Ll = config.REAL_LIGHT_LEN_MM * 0.001
    d = cs.estimate_distance(1.0, cam.FY * Ll / 3.0)
    check("distance LENGTH: 3m armor reconstructs", abs(d - 3.0) < 1e-9)
    config.DISTANCE_METHOD = "SPACING"

    check("distance with zero pixels -> 0.0", cs.estimate_distance(0, 0) == 0.0)

    # ---------- 8. 角尺寸 ----------
    sx, sy = cs.estimate_angular_size(cam.FX, cam.FY)   # l = f -> 2*atan(0.5) = 53.13 deg
    check("angular size l=f -> 53.13 deg", abs(sx * DEG - 53.1301) < 1e-3 and abs(sy * DEG - 53.1301) < 1e-3)

    # ---------- 9. 视线角速度 ----------
    yr, pr = cs.los_rates(0.302118306983606, 0.0)      # ~1 pixel/ms*(FX/1000) -> 1 rad/s at center
    check("los_rates: FX/1000 px/ms at center -> 1 rad/s", abs(yr - 1.0) < 1e-9 and pr == 0.0)
    yr_off, _ = cs.los_rates(0.302118306983606, 0.0, x_n=1.0)
    check("los_rates: (1+x^2) factor halves rate at x_n=1", abs(yr_off - 0.5) < 1e-9)

    # ---------- 10. 机体坐标 ----------
    bx, by, bz = cs.body_coords(2.0, math.atan(0.5), math.atan(-0.25))
    check("body_coords: (2, 2*0.5, 2*-0.25)", abs(bx - 2) < 1e-9 and abs(by - 1.0) < 1e-9 and abs(bz + 0.5) < 1e-9)

    # ---------- 11. 状态机映射 ----------
    check("map_state: valid, lost=0 -> TRACKING", cs.map_state(True, 0, 0) == cs.STATE_TRACKING)
    check("map_state: valid, lost=2 -> TEMP_LOST", cs.map_state(True, 2, 0) == cs.STATE_TEMP_LOST)
    check("map_state: invalid, pending=2 -> SUSPECT", cs.map_state(False, 0, 2) == cs.STATE_SUSPECT)
    check("map_state: invalid, pending=0 -> LOST", cs.map_state(False, 5, 0) == cs.STATE_LOST)

    # ---------- 12. target_num 编解码 ----------
    ok = True
    for st in range(4):
        for col in range(3):
            tn = cs.encode_target_num(st, col)
            ok &= (0 <= tn <= 255) and cs.decode_target_num(tn) == (st, col)
    check("target_num encode/decode round-trip (12 combos)", ok)
    check("target_num TRACKING+BLUE == 0b0110 == 6",
          cs.encode_target_num(cs.STATE_TRACKING, cs.COLOR_BLUE) == 6)

    # ---------- 13. TargetSolver 全流程 ----------
    config.UNDISTORT = True
    config.DISTANCE_EMA_ALPHA = 0.30
    solver = cs.TargetSolver()

    # 13a. LOST：全零
    out = solver.solve(False, cs.STATE_LOST, None, 0, 0)
    check("solver LOST -> valid False, all zeros",
          not out["valid"] and out["distance"] == 0.0 and out["yaw_error"] == 0.0
          and out["target_num"] == cs.encode_target_num(cs.STATE_LOST, cs.COLOR_NONE))

    # 13b. TRACKING at 2 m, slightly right of center
    sp2 = cam.FX * L / 2.0
    u = cam.CX + 30
    out1 = solver.solve(True, cs.STATE_TRACKING, "RED", u, cam.CY, 0.1, 0.0, sp2, 10.0)
    check("solver TRACKING -> valid, RED, yaw>0",
          out1["valid"] and out1["color"] == cs.COLOR_RED and out1["yaw_error"] > 0)
    # 一阶尺度修正会让距离略偏离 2.0（在 u=cx+30 处尺度≈1.001），允许 1%
    check("solver TRACKING first frame distance ~ 2.0 m (within 1%)",
          abs(out1["distance"] - 2.0) < 0.02, "d=%.4f" % out1["distance"])
    check("solver range >= depth", out1["range"] >= out1["distance"])
    check("solver body y = depth*tan(yaw)",
          abs(out1["y"] - out1["distance"] * math.tan(out1["yaw_error"])) < 1e-9)
    check("solver yaw_rate > 0 for vx > 0", out1["yaw_rate"] > 0)

    # 13c. 第二帧测量跳到 1 m -> EMA 只走 30%
    sp1 = cam.FX * L / 1.0
    out2 = solver.solve(True, cs.STATE_TRACKING, "RED", cam.CX, cam.CY, 0, 0, sp1, 10.0)
    expect = 0.7 * out1["distance"] + 0.3 * 1.0
    check("solver EMA: 0.7*old + 0.3*new", abs(out2["distance"] - expect) < 1e-9,
          "d=%.4f expect=%.4f" % (out2["distance"], expect))

    # 13d. TEMP_LOST：忽略传入尺寸，距离保持
    out3 = solver.solve(True, cs.STATE_TEMP_LOST, "RED", cam.CX, cam.CY, 0, 0, 1.0, 1.0)
    check("solver TEMP_LOST keeps last distance, ignores bogus spacing",
          out3["valid"] and abs(out3["distance"] - out2["distance"]) < 1e-12)

    # 13e. SUSPECT：无效输出但不清距离记忆；随后 LOST 清零
    out4 = solver.solve(False, cs.STATE_SUSPECT, "BLUE", 0, 0)
    check("solver SUSPECT -> invalid, target_num carries SUSPECT+BLUE",
          not out4["valid"] and cs.decode_target_num(out4["target_num"]) == (cs.STATE_SUSPECT, cs.COLOR_BLUE))
    check("solver SUSPECT does not wipe distance memory", solver.has_distance)
    solver.solve(False, cs.STATE_LOST, None, 0, 0)
    check("solver LOST wipes distance memory", not solver.has_distance and solver.distance == 0.0)

    # 13f. 去畸变关闭时，TRACKING 在中心处距离精确
    config.UNDISTORT = False
    solver2 = cs.TargetSolver()
    out5 = solver2.solve(True, cs.STATE_TRACKING, "RED", cam.CX, cam.CY, 0, 0, sp2, 10.0)
    check("solver UNDISTORT=False at center: distance exactly 2.0",
          abs(out5["distance"] - 2.0) < 1e-9)
    config.UNDISTORT = True

    print()
    n_fail = results.count(False)
    print("%d checks, %d failed" % (len(results), n_fail))
    print("ALL PASS" if n_fail == 0 else "SOME TESTS FAILED")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
