"""OpenMV测试入口：只改下面的MODE。与main.py复用同一VO循环。"""

MODE = 1                  # 1=每秒详细诊断；2=采图基线+完整流程FPS测量，完成后退出
DETAIL_HZ = 1
BASELINE_SECONDS = 3       # 模式2：采图基线，包含同一亮度调节
WARMUP_SECONDS = 5         # 模式2：完整链路预热，等待连接、同步和自适应
MEASURE_SECONDS = 15       # 模式2：采样时长；结束后集中输出耗时表
SAMPLE_CAPACITY = 256      # 每阶段保留最近256次耗时，限制内存；平均/最大值覆盖全部样本

import math
from array import array
import config as C
from algorithm import _us, _elapsed
from control import range_invalid_detail
from output import _state, Output
from main import run


# 上层阶段互不重叠；VO子项包含在vo_total中，GC包含在housekeeping中。
PHASES = (
    ("poll_before", "采图前通信/同步"), ("capture", "采图等待"),
    ("poll_after", "采图后通信/同步"), ("wait_attitude", "等待姿态"),
    ("camera", "曝光调节/统计"), ("vo_total", "VO总处理（含以下子项）"),
    ("resize", "  缩图"), ("features", "  检测+ORB描述子"),
    ("match", "  描述子匹配"), ("project", "  去畸变/投影"),
    ("fit", "  RANSAC/精化"), ("vision_poll", "  视觉内通信"),
    ("image_stats", "  VO内纹理统计"), ("vo_other", "  VO其余逻辑"),
    ("housekeeping", "循环维护（含GC）"), ("gc", "  垃圾回收"),
    ("output", "发布检查/位姿摘要"), ("test_output", "测试诊断输出"),
    ("cycle", "完整循环"),
)
INDEX = {name: i for i, (name, _) in enumerate(PHASES)}
VO_PARTS = ("resize", "features", "match", "project", "fit", "vision_poll", "image_stats")


def number(value, digits=1):
    return "--" if value is None else ("%+."+str(digits)+"f") % value


def history_hz(history):
    if len(history) < 2:
        return None
    latest, first, count = history[-1][0], history[-1][0], 0
    for item in history:
        if item[0] >= latest-2000000:
            first, count = min(first, item[0]), count+1
    return None if count < 2 or first >= latest else (count-1)*1000000.0/(latest-first)


class Samples:
    """固定容量的原生整数数组；不保存图像，不逐帧打印。"""
    def __init__(self, capacity=SAMPLE_CAPACITY):
        self.values = array("I", [0])*capacity
        self.count = self.total = self.maximum = 0

    def add(self, value):
        value = max(0, int(value))
        self.values[self.count % len(self.values)] = value
        self.count += 1
        self.total += value
        self.maximum = max(self.maximum, value)

    def mean_ms(self):
        return 0.0 if not self.count else self.total/self.count*.001

    def percentile_ms(self, fraction):
        values = sorted(self.values[:min(self.count, len(self.values))])
        return 0.0 if not values else values[max(0, int(math.ceil(len(values)*fraction))-1)]*.001


class TestSession:
    def __init__(self, mode=MODE):
        if mode not in (1, 2):
            raise ValueError("MODE must be 1 (details) or 2 (FPS measurement)")
        if (DETAIL_HZ <= 0 or BASELINE_SECONDS <= 0 or WARMUP_SECONDS < 0
                or MEASURE_SECONDS <= 0 or SAMPLE_CAPACITY < 1):
            raise ValueError("invalid test duration, frequency or sample capacity")
        self.mode = mode
        self.values = [0]*len(PHASES)
        self.seen = [False]*len(PHASES)
        self.stats = [Samples() for _ in PHASES] if mode == 2 else None
        self.active = False
        self.last_detail = None
        self.measure_start = None
        self.frames = self.valid_frames = self.loops = self.over_budget = 0
        self.reasons = {}

    def prepare(self, sensor, link, controller, capture, adapter, output):
        self.sensor, self.link, self.controller = sensor, link, controller
        self.capture, self.adapter, self.output = capture, adapter, output
        # 测试入口只观察VO；即使正式配置开启发布，也不回传测试中的ODOMETRY。
        output.publish = False
        output.display = self.mode == 2  # 模式2保留正式摘要，计入其实际开销
        print("测试模式=%d | 采图=%dx%d 特征缩放=%.2f 缓冲=%d | 测试期间不发布ODOMETRY" % (
            self.mode, C.IMAGE_WIDTH, C.IMAGE_HEIGHT, C.FEATURE_SCALE, C.FRAME_BUFFERS))
        if self.mode == 2:
            print("流程：采图基线%.1fs -> 链路预热%.1fs -> 完整流程采样%.1fs；保留正式位姿摘要。" % (
                BASELINE_SECONDS, WARMUP_SECONDS, MEASURE_SECONDS))
            count, capture_samples = 0, Samples()
            start = _us()
            while _elapsed(start) < BASELINE_SECONDS*1000000:
                before = _us()
                image, _ = capture.get()
                capture_samples.add(_elapsed(before))
                if image is not None:
                    count += 1
                    adapter.update(image, link.clock.now_us())
            elapsed = _elapsed(start)
            self.baseline_hz = count*1000000.0/max(1, elapsed)
            self.baseline_exposure = sensor.get_exposure_us()
            print("采图基线 | 新图像=%.1f Hz | snapshot均值/P95=%.2f/%.2f ms | 曝光=%dus" % (
                self.baseline_hz, capture_samples.mean_ms(), capture_samples.percentile_ms(.95),
                self.baseline_exposure))
        self.full_start = _us()

    def begin_frame(self):
        for i in range(len(PHASES)):
            self.values[i], self.seen[i] = 0, False
        self.frame_start = self.cursor = _us()
        self.active = True

    def record(self, name, duration):
        if self.active:
            index = INDEX[name]
            self.values[index] += max(0, int(duration))
            self.seen[index] = True

    def mark(self, name):
        self.record(name, _elapsed(self.cursor))
        self.cursor = _us()

    def finish_frame(self, result, fps):
        if self.mode == 1 and result is not None and (
                self.last_detail is None or _elapsed(self.last_detail) >= 1000000/DETAIL_HZ):
            self.last_detail = _us()
            self.details(result, fps)
            self.record("test_output", _elapsed(self.last_detail))
        if self.seen[INDEX["vo_total"]]:
            children = sum(self.values[INDEX[name]] for name in VO_PARTS)
            self.record("vo_other", max(0, self.values[INDEX["vo_total"]]-children))
        self.record("cycle", _elapsed(self.frame_start))
        self.active = False
        if self.mode == 1 or _elapsed(self.full_start) < WARMUP_SECONDS*1000000:
            return False
        if self.measure_start is None:
            self.measure_start = self.frame_start
        for i, samples in enumerate(self.stats):
            if self.seen[i]:
                samples.add(self.values[i])
        self.loops += 1
        if result is not None:
            self.frames += 1
            self.valid_frames += int(result["valid"])
            reason = result["reason"]
        else:
            reason = "NO_NEW_FRAME"
        self.reasons[reason] = self.reasons.get(reason, 0)+1
        self.over_budget += int(self.values[INDEX["cycle"]] > 1000000/C.TARGET_FPS)
        elapsed = _elapsed(self.measure_start)
        if elapsed < MEASURE_SECONDS*1000000:
            return False
        self.report(elapsed)
        return True

    def details(self, result, fps):
        link, now = self.link, self.link.clock.now_us()
        self.output._summary(result, fps)
        stamp = link.fc_time(result["timestamp_us"])
        delta = None if stamp is None or not link.attitudes else (stamp-link.attitudes[-1][0])*.001
        att_age = None if not link.attitudes else (now-link.attitudes[-1][-1])*.001
        range_age = None if not link.ranges else (now-link.ranges[-1][-1])*.001
        print("通信 | 连接=%s 同步=%s | 姿态/测距=%s/%s Hz | 接收年龄=%s/%s ms | 帧姿态差=%s ms" % (
            link.connected(now), link.synced(now), number(history_hz(link.attitudes)),
            number(history_hz(link.ranges)), number(att_age), number(range_age), number(delta)))
        print("链路 | RTT=%s ms 拒绝=%d 同步样本=%d CRC错误=%d 发送丢弃=%d 请求结果=%s" % (
            number(None if link.last_sync_rtt_us is None else link.last_sync_rtt_us*.001),
            link.sync_rtt_rejected, link.sync_count, link.parser.bad_crc, link.tx_dropped, link.last_ack))
        if link.ranges:
            sample = link.ranges[-1]
            detail = range_invalid_detail(sample[1], sample[2], sample[3], sample[4])
            print("测距 | 原始=%.3fm 量程=%.3f~%.3fm ID=%s 信号质量=%d 检查=%s | 高度处理=%s 跳过旧测距=%d" % (
                sample[1], sample[2], sample[3], link.range_id, sample[4], _state(detail or "OK"),
                _state(result.get("range_state", "WAIT_RANGE")), result.get("range_skipped_attitude", 0)))
        vision = result.get("vision")
        if vision:
            print("视觉 | 工作图=%s 检测器=%s 阈值=%d 上限=%d 单尺度=%s | 特征=%d 匹配/内点=%d/%d 网格=%s 分散度=%s | %s" % (
                vision.get("work_size"), vision.get("detector_name"), vision.get("detector", 0),
                vision.get("limit", 0), vision.get("normalized"), vision.get("keypoints", 0),
                result["matches"], result["inliers"], vision.get("cells", "--"),
                number(vision.get("spread"), 3), _state(vision.get("vision_reason", "READY"))))
        else:
            print("视觉 | 未执行：%s" % _state(result["reason"]))
        parts = []
        for name in ("capture", "wait_attitude", "resize", "features", "match", "project", "fit", "vo_total"):
            i = INDEX[name]
            parts.append("%s=%s" % (PHASES[i][1].strip(),
                "%.2fms" % (self.values[i]*.001) if self.seen[i] else "未执行"))
        print("本帧耗时 | "+" | ".join(parts))
        print("相机 | 曝光=%dus 增益=%.1fdB | 残差=%s px 重置=%d 关键帧=%d | 详细打印耗时计入本模式FPS" % (
            self.sensor.get_exposure_us(), self.sensor.get_gain_db(), number(result["residual_px"], 2),
            result["reset_counter"], result["keyframe"]))

    def report(self, elapsed):
        cycle_total = self.stats[INDEX["cycle"]].total
        hz = self.frames*1000000.0/max(1, elapsed)
        print("\nFPS测量完成 | %.2fs 新图像=%d 实际=%.2fHz 有效定位=%.2fHz 有效率=%.1f%%" % (
            elapsed*1e-6, self.frames, hz, self.valid_frames*1000000.0/max(1, elapsed),
            self.valid_frames*100.0/max(1, self.frames)))
        print("目标=%.1fHz 周期预算=%.2fms | 超预算=%d/%d | P50/P95使用各阶段最近最多%d次，平均/最大值覆盖全部。" % (
            C.TARGET_FPS, 1000/C.TARGET_FPS, self.over_budget, self.loops, SAMPLE_CAPACITY))
        print("阶段 | 执行次数 | 平均/P50/P95/最大 ms | 占循环时间")
        for i, (_, label) in enumerate(PHASES):
            samples = self.stats[i]
            if not samples.count:
                print("%s | 未执行" % label)
            else:
                print("%s | %d | %.2f/%.2f/%.2f/%.2f | %.1f%%" % (
                    label, samples.count, samples.mean_ms(), samples.percentile_ms(.5),
                    samples.percentile_ms(.95), samples.maximum*.001,
                    samples.total*100.0/max(1, cycle_total)))
        overhead = max(0, elapsed-cycle_total)
        print("统计管理/未分项间隙 | %.2fms/循环 | %.1f%%测量墙钟时间" % (
            overhead*.001/max(1, self.loops), overhead*100.0/max(1, elapsed)))
        print("说明：VO总处理包含缩图至其余逻辑；循环维护包含GC，父子项不能相加。采图耗时含等待，不等于CPU占用。")
        print("状态分布 | "+" | ".join("%s=%d" % (_state(name), count) for name, count in self.reasons.items()))
        now = self.link.clock.now_us()
        print("输入核对 | 连接=%s 同步=%s | 姿态/测距=%s/%sHz | RTT=%sms CRC错误=%d" % (
            self.link.connected(now), self.link.synced(now), number(history_hz(self.link.attitudes)),
            number(history_hz(self.link.ranges)),
            number(None if self.link.last_sync_rtt_us is None else self.link.last_sync_rtt_us*.001),
            self.link.parser.bad_crc))
        # 排序使用互不重叠的阶段，避免将VO父项与其子项重复计入。
        names = ("poll_before", "capture", "poll_after", "wait_attitude", "camera",
                 "resize", "features", "match", "project", "fit", "vision_poll", "image_stats",
                 "vo_other", "housekeeping", "output")
        ranked = sorted(((self.stats[INDEX[name]].total, name) for name in names), reverse=True)
        print("耗时前三项 | "+" | ".join("%s=%.1f%%" % (
            PHASES[INDEX[name]][1].strip(), total*100.0/max(1, cycle_total)) for total, name in ranked[:3]))
        top = ranked[0][1]
        if top == "capture":
            print("判断线索：采图等待最多。对照采图基线，检查读出、缓冲和错过帧；尚不能仅凭此认定传感器硬限速。")
        elif top in ("features", "resize", "match", "project", "fit", "vo_other"):
            print("判断线索：视觉阶段耗时最多，优先针对上表中的具体阶段优化。")
        else:
            print("判断线索：通信、等待或维护开销较高，先检查对应阶段，不宜先减少视觉信息。")
        exposure = self.sensor.get_exposure_us()
        print("采图对照 | 基线=%.2fHz 完整=%.2fHz | 基线/结束曝光=%d/%dus | 缓冲=%d" % (
            self.baseline_hz, hz, self.baseline_exposure, exposure, C.FRAME_BUFFERS))
        if abs(exposure-self.baseline_exposure) > max(200, self.baseline_exposure*.05):
            print("对照条件：曝光发生明显变化，基线与完整流程的差值还包含相机工作条件变化。")
        if not self.valid_frames:
            print("当前无有效位置：本次结果不能代表正常跟踪时的完整负载；未执行阶段需在可跟踪画面下复测。")
        print("定位算法和分辨率未由测试入口修改；测试包含计时开销，IDE图像传输影响需另做开/关对照。")


def main():
    run(TestSession(MODE))


if __name__ == "__main__":
    main()
