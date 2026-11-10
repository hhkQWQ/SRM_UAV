
import os
import time

import config as C
from main import CameraAdapter, setup_camera


# Set to zero to preview without saving. A new directory is made for each run.
MAX_SAVED_IMAGES = 8
SAVE_INTERVAL_MS = 2000
REPORT_INTERVAL_MS = 1000
SAVE_ROOT = "/sd/vo_preview"


def _is_directory(path):
    try:
        return bool(os.stat(path)[0] & 0x4000)
    except OSError:
        return False


def _session_directory():
    if MAX_SAVED_IMAGES <= 0 or not _is_directory("/sd"):
        return None
    if not _is_directory(SAVE_ROOT):
        os.mkdir(SAVE_ROOT)
    for number in range(1, 10000):
        path = "%s/session_%04d" % (SAVE_ROOT, number)
        try:
            os.stat(path)
        except OSError:
            os.mkdir(path)
            metadata = open(path + "/metadata.csv", "wb")
            try:
                metadata.write(
                    b"frame,elapsed_ms,exposure_us,gain_db,mean_gray,stdev_gray,file\n"
                )
            finally:
                metadata.close()
            return path
    raise OSError("too many camera preview sessions")


def _save_frame(image, directory, number, elapsed_ms, exposure_us, gain_db, statistics):
    filename = "frame_%03d.pgm" % number
    frame_file = open(directory + "/" + filename, "wb")
    try:
        frame_file.write(b"P5\n320 240\n255\n")
        frame_file.write(image)
    finally:
        frame_file.close()
    metadata = open(directory + "/metadata.csv", "ab")
    try:
        metadata.write(
            ("%d,%d,%d,%.3f,%.2f,%.2f,%s\n" % (
                number, elapsed_ms, exposure_us, gain_db,
                statistics.mean(), statistics.stdev(), filename
            )).encode("ascii")
        )
    finally:
        metadata.close()
    print("保存原始灰度图：%s/%s" % (directory, filename))


def main():
    sensor = setup_camera()
    adapter = CameraAdapter(sensor)
    try:
        directory = _session_directory()
    except OSError as error:
        print("SD保存不可用，继续实时预览：%s" % error)
        directory = None
    if MAX_SAVED_IMAGES and directory is None:
        print("未检测到可用SD卡；仅在IDE中实时预览。")
    print("OpenMV IDE Frame Buffer：320x240灰度原图，不叠加标记。")
    print("启动配置：曝光=%s us 增益=%s dB 自动调节=%s" % (
        C.EXPOSURE_US, C.GAIN_DB, C.CAMERA_AUTO_TUNE
    ))
    print("独立预览没有飞控角速度和VO平移速度；运动时请以MODE2实际曝光记录为准。")

    start_ms = time.ticks_ms()
    report_ms = start_ms
    frames = saved = 0
    next_save_ms = SAVE_INTERVAL_MS
    try:
        while True:
            image = sensor.snapshot()
            now_ms = time.ticks_ms()
            elapsed_ms = time.ticks_diff(now_ms, start_ms)
            frames += 1
            report_due = time.ticks_diff(now_ms, report_ms) >= REPORT_INTERVAL_MS
            save_due = (
                directory is not None and saved < MAX_SAVED_IMAGES
                and elapsed_ms >= next_save_ms
            )
            # Read the sensor before CameraAdapter writes the next setting.
            if report_due or save_due:
                exposure_us = sensor.get_exposure_us()
                gain_db = sensor.get_gain_db()
            statistics = adapter.update(image, now_ms * 1000)

            if save_due:
                if statistics is None:
                    statistics = image.get_statistics()
                try:
                    _save_frame(
                        image, directory, saved + 1, elapsed_ms,
                        exposure_us, gain_db, statistics
                    )
                    saved += 1
                except OSError as error:
                    print("保存失败，继续预览：%s" % error)
                    directory = None
                next_save_ms += SAVE_INTERVAL_MS

            if report_due:
                window_ms = time.ticks_diff(now_ms, report_ms)
                print("实际曝光=%d us | 实际增益=%.1f dB | 预览=%.1f FPS" % (
                    exposure_us, gain_db, frames * 1000.0 / window_ms
                ))
                report_ms = now_ms
                frames = 0
    except KeyboardInterrupt:
        print("预览结束；已保存%d张原始灰度图。" % saved)


if __name__ == "__main__":
    main()
