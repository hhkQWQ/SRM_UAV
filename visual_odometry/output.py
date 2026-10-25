"""正式运行的位姿摘要和ODOMETRY输出准入。详细诊断由test.py负责。"""
import config as C
from algorithm import euler_degrees


_STATES = {
    "TRACKING": "跟踪正常", "SPARSE_TRACKING": "少点跟踪", "PREDICTING": "短时预测", "INITIALIZING": "建立参考图",
    "REINITIALIZING": "轨迹重置，下帧重建原点", "POSE_RECOVERING": "确认跟踪稳定",
    "WAIT_HEARTBEAT": "等待飞控连接", "TIME_UNSYNCED": "等待时钟同步",
    "WAIT_ATTITUDE": "等待图像时刻的姿态", "EXTRINSICS_PENDING": "等待安装参数",
    "WAIT_RANGE": "等待测距", "WAIT_RANGE_ATTITUDE": "等待测距时刻的姿态",
    "RANGE_ATTITUDE_GAP": "测距缺少对应姿态", "SELECT_TOF_ID": "请指定测距器ID",
    "RANGE_INVALID": "测距无效", "RANGE_TILT": "测距倾角过大",
    "RANGE_JUMP": "测距突变", "RANGE_RECOVERING": "确认测距稳定",
    "RANGE_STALE": "高度过期或无效", "ATTITUDE_RESET": "姿态跳变，重新初始化",
    "DYNAMIC_LIMIT": "倾角或转速过大", "VIBRATION": "振动超限",
    "LOW_TEXTURE": "画面纹理不足", "NO_FEATURES": "角点不足",
    "LOST": "跟踪丢失", "MATCH_COUNT": "匹配点不足",
    "INLIER_RATIO": "可靠匹配比例低",
    "NOT_FINITE": "距离非数值", "LIMITS_INVALID": "量程无效",
    "SIGNAL_INVALID": "传感器报告无效", "TOO_NEAR": "距离过近", "TOO_FAR": "距离过远",
    "AMBIGUOUS_MATCH": "匹配有歧义",
    "INPUT_INVALID": "视觉输入无效", "OK": "可用", "TRACK_FAILED": "视觉跟踪失败，原因未细分",
}


def _state(code):
    return _STATES.get(code, code)


BLOCK_CODES = ("TIMING_UNCONFIRMED", "POSITION_INVALID", "EXTRINSICS_UNCONFIRMED",
               "LOW_QUALITY", "DISCONNECTED", "UNSYNCED", "OBSERVATION_STALE", "NOT_MEASURED")


def publication_block_mask(result, link, now):
    """共用准入位掩码；逐帧不构造检查元组、生成器和原因字符串。"""
    mask = 0
    if not C.SENSOR_TIMING_CONFIRMED:
        mask |= 1
    if not result["valid"]:
        mask |= 2
    if not result["body_valid"]:
        mask |= 4
    if not result["quality"] >= C.MIN_PUBLISH_QUALITY:
        mask |= 8
    if not link.connected(now):
        mask |= 16
    if not link.synced(now):
        mask |= 32
    if not 0 <= now-result["timestamp_us"] <= C.MAX_TX_AGE_US:
        mask |= 64
    if result.get('predicted', False) or not result.get('measured', False):
        mask |= 128
    return mask


def publication_blockers(result, link, now):
    """保留名称接口供按需诊断使用；正式输出和测试热路径使用位掩码。"""
    mask = publication_block_mask(result, link, now)
    return tuple(name for i, name in enumerate(BLOCK_CODES) if mask & (1 << i))


class PublicationGate:
    """按图像时间戳去重及限频；测试只运行此准入，不进入发送队列。"""
    def __init__(self):
        self.last_publish = -10000000
        self.last_stamp = -1

    def accept(self, result, now, blockers):
        if (blockers or result.get('predicted', False) or not result.get('measured', False)
                or result["timestamp_us"] <= self.last_stamp
                or now-self.last_publish < int(1000000/C.ODOMETRY_HZ)):
            return False
        self.last_publish, self.last_stamp = now, result["timestamp_us"]
        return True


class Output:
    def __init__(self, link, display=True, publish=True):
        self.link = link
        self.display = display
        self.publish = publish
        self.last_display = -10000000
        self.gate = PublicationGate()

    def update(self, result, fps):
        now = self.link.clock.now_us()
        # 发布与失效取消逐帧检查，不能被显示/诊断限频挡住。
        blockers = 0
        allowed = self.publish and C.SEND_ODOMETRY
        if allowed:
            blockers = publication_block_mask(result, self.link, now)
            allowed = not blockers
        if not allowed:
            self.link.cancel_odometry()
        elif self.gate.accept(result, now, blockers):
            self.link.send_odometry(result)
        if self.display and now-self.last_display >= int(1000000/C.DISPLAY_HZ):
            self.last_display = now
            self._summary(result, fps)

    def _summary(self, result, fps):
        status = _state(result["reason"])
        detail = None
        if result["reason"] == "RANGE_INVALID":
            detail = result.get("range_detail")
        elif result["reason"] == "LOST":
            detail = (result.get("vision") or {}).get("vision_reason")
        if detail:
            status += "："+_state(detail)
        if result["approximate"]:
            status += "（近似）"
        if result["valid"]:
            position = "N=%+.3f E=%+.3f m" % result["position"]
        else:
            position = "位置=无效"
        height = "--" if result["height"] is None else "%.3f m" % result["height"]
        attitude = ("--" if result["q"] is None else
                    "%+.1f/%+.1f/%+.1f 度" % euler_degrees(result["q"]))
        print("状态=%s | %s | 高度=%s | 横滚/俯仰/航向=%s | FPS=%.1f | 置信度=%d/100" % (
            status, position, height, attitude, fps, result["quality"] if result["valid"] else 0))

