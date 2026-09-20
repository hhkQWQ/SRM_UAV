# armor_tracker.py
# ============================================================
# OpenMV 装甲板连续帧确认 / 运动预测跟踪器
#
# 使用方式：
#     from armor_tracker import MotionTracker
#
#     tracker = MotionTracker(...)
#
#     valid, color, cx, cy = tracker.update(
#         found=True,
#         now_ms=time.ticks_ms(),
#         color="RED",
#         cx=160,
#         cy=120,
#         target_size=40
#     )
#
# 说明：
# - vx / vy 是“目标在图像中的像素速度”，单位 pixel/ms。
# - 它不是无人机物理速度，也不是 GPS 速度。
# - 当前帧只有在通过历史运动模型验证后，才会修正稳定速度。
# ============================================================

import math
import time


class MotionTracker:

    def __init__(
        self,

        confirm_frames=3,
        max_lost_frames=4,

        acquire_base_gate=35.0,
        acquire_size_factor=1.20,
        acquire_speed_factor=0.80,
        acquire_max_gate=110.0,

        track_base_gate=10.0,
        track_size_factor=0.60,
        track_speed_factor=0.90,
        track_max_gate=100.0,

        track_alpha=0.70,
        track_beta=0.20,

        pending_velocity_alpha=0.60,
        lost_velocity_decay=0.92,

        track_max_dt_ms=150
    ):

        # --------------------------------------------------------
        # 参数
        # --------------------------------------------------------

        self.confirm_frames = confirm_frames
        self.max_lost_frames = max_lost_frames

        self.acquire_base_gate = acquire_base_gate
        self.acquire_size_factor = acquire_size_factor
        self.acquire_speed_factor = acquire_speed_factor
        self.acquire_max_gate = acquire_max_gate

        self.track_base_gate = track_base_gate
        self.track_size_factor = track_size_factor
        self.track_speed_factor = track_speed_factor
        self.track_max_gate = track_max_gate

        self.track_alpha = track_alpha
        self.track_beta = track_beta

        self.pending_velocity_alpha = pending_velocity_alpha
        self.lost_velocity_decay = lost_velocity_decay

        self.track_max_dt_ms = track_max_dt_ms

        # --------------------------------------------------------
        # 稳定目标状态
        # --------------------------------------------------------

        self.stable_valid = False
        self.stable_color = None

        self.stable_cx = 0.0
        self.stable_cy = 0.0

        # 单位：pixel / ms
        self.stable_vx = 0.0
        self.stable_vy = 0.0

        self.stable_size = 0.0
        self.stable_time = time.ticks_ms()

        self.lost_count = 0

        # --------------------------------------------------------
        # 等待连续确认的新候选
        # --------------------------------------------------------

        self._reset_pending()


    # ============================================================
    # 对外接口
    # ============================================================

    def reset(self):
        """
        完全重置跟踪器。
        """
        self.stable_valid = False
        self.stable_color = None

        self.stable_cx = 0.0
        self.stable_cy = 0.0

        self.stable_vx = 0.0
        self.stable_vy = 0.0

        self.stable_size = 0.0
        self.stable_time = time.ticks_ms()

        self.lost_count = 0

        self._reset_pending()


    def get_velocity(self):
        """
        返回当前稳定目标的图像速度：
            vx, vy
        单位：
            pixel / ms
        """
        return (
            self.stable_vx,
            self.stable_vy
        )


    def get_lost_count(self):
        return self.lost_count


    def get_pending_count(self):
        return self.pending_count


    def update(
        self,
        found,
        now_ms,
        color=None,
        cx=0,
        cy=0,
        target_size=0
    ):
        """
        每帧调用一次。

        参数：
            found:
                当前帧是否存在 best_pair

            now_ms:
                当前 time.ticks_ms()

            color:
                "RED" / "BLUE"

            cx, cy:
                当前帧原始装甲中心

            target_size:
                当前装甲候选在画面中的代表尺寸
                推荐使用 max(w, h)

        返回：
            valid, color, stable_cx, stable_cy

        规则：
        1. 新目标必须连续 confirm_frames 帧才正式锁定；
        2. 连续帧允许出现明显像素位移；
        3. 已锁定后使用历史速度预测下一帧位置；
        4. 当前检测先与预测位置比较；
        5. 只有通过验证的检测才会修改 stable_vx / stable_vy；
        6. 突然跳到远处的检测进入 pending，不立即接管；
        7. 短暂丢失时继续使用历史速度预测。
        """

        # ========================================================
        # A. 当前帧没有检测到任何装甲候选
        # ========================================================

        if not found:

            # 尚未正式锁定时，
            # 连续确认过程被中断，重新开始。
            if not self.stable_valid:

                self._reset_pending()

                return (
                    False,
                    None,
                    0,
                    0
                )

            self.lost_count += 1

            # 丢失过久：正式清除目标
            if self.lost_count > self.max_lost_frames:

                self.stable_valid = False
                self.stable_color = None

                self.stable_vx = 0.0
                self.stable_vy = 0.0

                self._reset_pending()

                return (
                    False,
                    None,
                    0,
                    0
                )

            # 短暂丢失：
            # 使用已经确认的历史速度继续预测。
            dt = self._calc_dt(
                now_ms,
                self.stable_time
            )

            self.stable_cx += (
                self.stable_vx * dt
            )

            self.stable_cy += (
                self.stable_vy * dt
            )

            # 丢失时逐渐衰减速度，
            # 避免预测点长期向画面外飞。
            self.stable_vx *= (
                self.lost_velocity_decay
            )

            self.stable_vy *= (
                self.lost_velocity_decay
            )

            self.stable_time = now_ms

            self._reset_pending()

            return self._stable_result()


        # ========================================================
        # B. 尚未正式锁定目标
        # ========================================================

        if not self.stable_valid:

            confirmed = self._update_pending(
                color,
                cx,
                cy,
                target_size,
                now_ms
            )

            if confirmed:

                self._accept_pending(
                    now_ms
                )

            if not self.stable_valid:

                return (
                    False,
                    None,
                    0,
                    0
                )

            return self._stable_result()


        # ========================================================
        # C. 已经锁定：
        #    先根据“上一时刻已经确认的状态”预测当前帧。
        # ========================================================

        dt = self._calc_dt(
            now_ms,
            self.stable_time
        )

        predicted_cx = (
            self.stable_cx +
            self.stable_vx * dt
        )

        predicted_cy = (
            self.stable_cy +
            self.stable_vy * dt
        )


        # ========================================================
        # D. 用当前测量与预测位置做比较
        # ========================================================

        error_x = (
            cx -
            predicted_cx
        )

        error_y = (
            cy -
            predicted_cy
        )

        error_sq = (
            error_x * error_x +
            error_y * error_y
        )

        speed = math.sqrt(
            self.stable_vx * self.stable_vx +
            self.stable_vy * self.stable_vy
        )

        expected_move = (
            speed * dt
        )

        size_ref = max(
            float(target_size),
            self.stable_size
        )

        gate = (
            self.track_base_gate +

            self.track_size_factor *
            size_ref +

            self.track_speed_factor *
            expected_move
        )

        if gate > self.track_max_gate:
            gate = self.track_max_gate

        same_target = (
            color == self.stable_color and
            error_sq <= gate * gate
        )


        # ========================================================
        # E. 当前检测符合历史运动模型
        #
        # 注意：
        # 到这里以后，当前测量才有资格修改 stable_vx/vy。
        # ========================================================

        if same_target:

            self.lost_count = 0

            # Alpha-Beta Filter
            self.stable_cx = (
                predicted_cx +
                self.track_alpha * error_x
            )

            self.stable_cy = (
                predicted_cy +
                self.track_alpha * error_y
            )

            self.stable_vx = (
                self.stable_vx +
                self.track_beta *
                error_x / dt
            )

            self.stable_vy = (
                self.stable_vy +
                self.track_beta *
                error_y / dt
            )

            # 尺寸轻微平滑，
            # 防止动态门限被一帧 blob 尺寸波动明显改变。
            self.stable_size = (
                0.70 * self.stable_size +
                0.30 * float(target_size)
            )

            self.stable_time = now_ms

            # 原目标正常出现，
            # 之前的异常跳点候选全部作废。
            self._reset_pending()

            return self._stable_result()


        # ========================================================
        # F. 当前检测与预测位置严重不符
        #
        # 不能用这个点直接修改 stable_vx / stable_vy。
        # 它只会先进入 pending。
        # ========================================================

        self.lost_count += 1

        confirmed = self._update_pending(
            color,
            cx,
            cy,
            target_size,
            now_ms
        )

        # 新位置连续多帧成立：
        # 允许它正式接管。
        if confirmed:

            self._accept_pending(
                now_ms
            )

            return self._stable_result()


        # 新候选尚未确认：
        # 原目标继续沿历史运动趋势预测。
        self.stable_cx = predicted_cx
        self.stable_cy = predicted_cy

        self.stable_vx *= (
            self.lost_velocity_decay
        )

        self.stable_vy *= (
            self.lost_velocity_decay
        )

        self.stable_time = now_ms


        # 原目标长期没有回来，
        # 新候选又始终不能连续确认。
        if self.lost_count > self.max_lost_frames:

            self.stable_valid = False
            self.stable_color = None

            self.stable_vx = 0.0
            self.stable_vy = 0.0

            self._reset_pending()

            return (
                False,
                None,
                0,
                0
            )


        return self._stable_result()


    # ============================================================
    # 内部函数
    # ============================================================

    def _calc_dt(
        self,
        now_ms,
        old_ms
    ):

        dt = time.ticks_diff(
            now_ms,
            old_ms
        )

        if dt < 1:
            dt = 1

        if dt > self.track_max_dt_ms:
            dt = self.track_max_dt_ms

        return dt


    def _stable_result(self):

        return (
            self.stable_valid,
            self.stable_color,
            int(self.stable_cx),
            int(self.stable_cy)
        )


    def _reset_pending(self):

        self.pending_color = None

        self.pending_cx = 0.0
        self.pending_cy = 0.0

        self.pending_vx = 0.0
        self.pending_vy = 0.0

        self.pending_size = 0.0

        self.pending_count = 0

        self.pending_time = 0


    def _update_pending(
        self,
        color,
        cx,
        cy,
        target_size,
        now_ms
    ):
        """
        新目标 / 大跳变目标的连续帧确认。

        第 1 帧：
            建立候选。

        第 2 帧：
            根据动态门限判断是否连续，
            并开始估算像素速度。

        第 3 帧及以后：
            使用候选历史速度预测位置。
        """

        # --------------------------------------------------------
        # 第一个候选点，或者颜色变化
        # --------------------------------------------------------

        if (
            self.pending_count <= 0 or
            color != self.pending_color
        ):

            self.pending_color = color

            self.pending_cx = float(cx)
            self.pending_cy = float(cy)

            self.pending_vx = 0.0
            self.pending_vy = 0.0

            self.pending_size = float(
                target_size
            )

            self.pending_count = 1

            self.pending_time = now_ms

            return (
                self.pending_count >=
                self.confirm_frames
            )


        dt = self._calc_dt(
            now_ms,
            self.pending_time
        )


        # --------------------------------------------------------
        # 用“候选之前已经估计的速度”预测当前位置
        # --------------------------------------------------------

        predicted_cx = (
            self.pending_cx +
            self.pending_vx * dt
        )

        predicted_cy = (
            self.pending_cy +
            self.pending_vy * dt
        )

        error_x = (
            cx -
            predicted_cx
        )

        error_y = (
            cy -
            predicted_cy
        )

        error_sq = (
            error_x * error_x +
            error_y * error_y
        )

        speed = math.sqrt(
            self.pending_vx * self.pending_vx +
            self.pending_vy * self.pending_vy
        )

        expected_move = (
            speed * dt
        )

        size_ref = max(
            float(target_size),
            self.pending_size
        )

        gate = (
            self.acquire_base_gate +

            self.acquire_size_factor *
            size_ref +

            self.acquire_speed_factor *
            expected_move
        )

        if gate > self.acquire_max_gate:
            gate = self.acquire_max_gate


        # --------------------------------------------------------
        # 符合候选运动趋势
        # --------------------------------------------------------

        if error_sq <= gate * gate:

            measured_vx = (
                cx -
                self.pending_cx
            ) / dt

            measured_vy = (
                cy -
                self.pending_cy
            ) / dt

            a = self.pending_velocity_alpha

            self.pending_vx = (
                (1.0 - a) *
                self.pending_vx +
                a *
                measured_vx
            )

            self.pending_vy = (
                (1.0 - a) *
                self.pending_vy +
                a *
                measured_vy
            )

            self.pending_cx = float(cx)
            self.pending_cy = float(cy)

            self.pending_size = float(
                target_size
            )

            self.pending_time = now_ms

            self.pending_count += 1


        # --------------------------------------------------------
        # 与旧候选差异太大：
        # 从当前检测重新开始计数
        # --------------------------------------------------------

        else:

            self.pending_color = color

            self.pending_cx = float(cx)
            self.pending_cy = float(cy)

            self.pending_vx = 0.0
            self.pending_vy = 0.0

            self.pending_size = float(
                target_size
            )

            self.pending_count = 1

            self.pending_time = now_ms


        return (
            self.pending_count >=
            self.confirm_frames
        )


    def _accept_pending(
        self,
        now_ms
    ):
        """
        将已经连续确认的 pending 候选升级为正式稳定目标。
        """

        self.stable_valid = True

        self.stable_color = (
            self.pending_color
        )

        self.stable_cx = (
            self.pending_cx
        )

        self.stable_cy = (
            self.pending_cy
        )

        # 初次正式锁定时继承候选阶段估计出的图像速度。
        self.stable_vx = (
            self.pending_vx
        )

        self.stable_vy = (
            self.pending_vy
        )

        self.stable_size = (
            self.pending_size
        )

        self.stable_time = now_ms

        self.lost_count = 0

        self._reset_pending()
