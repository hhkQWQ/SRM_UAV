"""Small communication CSV shared by MicroPython and simulation.

The link supplies actual decisions. Link calls finish before CSV formatting or
storage, so disk operations never hold the simulation communication lock.
"""
import math
import os
import time


COMM_LOG_FIELDS = (
    "local_us", "event", "reason", "generation", "offset_us", "rtt_us",
    "innovation_us", "sync_age_us", "queue_wait_us", "poll_gap_us",
    "poll_wall_gap_us", "heartbeat_age_us", "attitude_age_us", "velocity_age_us",
    "bad_crc", "rx_out_of_order", "reply_mismatch", "sync_timeouts", "tx_dropped",
    "events_dropped", "tx_capture_us", "tx_us", "tx_age_us",
)

_ROUTINE_EVENTS = (
    "ACCEPT_FILTER", "REQUEST_QUEUED", "REQUEST_TX", "REMOTE_REQUEST", "SYNC_WARMUP",
)


def _cell(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        value = "%.7g" % value
    else:
        value = str(value)
    if any(char in value for char in (",", '"', "\n", "\r")):
        return '"' + value.replace('"', '""') + '"'
    return value


class CommCSVLogger:
    def __init__(self, link, path, clock=None, interval_ms=1000):
        self.link = link
        self.clock = clock or link.clock
        self.interval_us = max(1, int(interval_ms)) * 1000
        self.path = path
        self.file = None
        self.error = None
        self.written_records = 0
        self._last_snapshot_tick = self._last_flush_tick = None
        try:
            try:
                os.stat(path)
            except OSError as error:
                if not error.args or error.args[0] != 2:
                    raise
            else:
                raise OSError("COMM_LOG_ALREADY_EXISTS")
            self.file = open(path, "w")
            self._write_line(",".join(COMM_LOG_FIELDS) + "\n")
            self.file.flush()
        except Exception as error:
            self._fail(error)

    def _write_line(self, line):
        written = self.file.write(line)
        if written is not None and written != len(line):
            raise OSError("COMM_LOG_SHORT_WRITE")

    def _write(self, item):
        self._write_line(",".join(_cell(item.get(key)) for key in COMM_LOG_FIELDS) + "\n")
        self.written_records += 1

    def _fail(self, error):
        self.error = str(error) or repr(error)
        if self.file is not None:
            try:
                self.file.close()
            except Exception:
                pass
        self.file = None

    @staticmethod
    def _due(now, previous, interval):
        return previous is None or not 0 <= now-previous < interval

    def record(self, now_us=None, force=False):
        if self.file is None:
            return
        try:
            now = self.clock.now_us() if now_us is None else now_us
            # Host time schedules exports during a paused simulation, never
            # physical prediction or timestamp matching.
            tick = time.monotonic_ns()//1000 if hasattr(time, "monotonic_ns") else now
            snapshot_due = force or self._due(tick, self._last_snapshot_tick, self.interval_us)
            events = self.link.drain_comm_events()
            snapshot = dict(self.link.comm_snapshot(now)) if snapshot_due else None
            for event in events:
                if event.get("event") not in _ROUTINE_EVENTS:
                    self._write(event)
            if snapshot is not None:
                snapshot["event"] = "SNAPSHOT"
                snapshot.setdefault("local_us", now)
                self._write(snapshot)
                self._last_snapshot_tick = tick
            if force or self._due(tick, self._last_flush_tick, 1000000):
                self.file.flush()
                self._last_flush_tick = tick
        except Exception as error:
            self._fail(error)

    def close(self):
        self.record(force=True)
        if self.file is not None:
            try:
                self.file.flush()
                self.file.close()
            except Exception as error:
                self._fail(error)
            self.file = None
