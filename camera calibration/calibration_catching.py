# ============================================================
# OpenMV H7 Plus + OV5640
# Camera Calibration Image Capture
#
# Firmware:
# OpenMV v4.5.9
#
# Automatic capture every 5 seconds
#
# Camera:
#   OV5640
#   GRAYSCALE
#   QVGA = 320 x 240
#
# Calibration board:
#   Inner corners = 11 x 8
#   Square size   = 30 mm
#
# Images saved directly in OpenMV main directory:
#   calib_001.pgm
#   calib_002.pgm
#   ...
# ============================================================

import sensor
import time
import os


# ============================================================
# Settings
# ============================================================

FRAME_SIZE = sensor.QVGA

# 每隔 5 秒拍一张
CAPTURE_INTERVAL_MS = 5000

# 总共拍摄 30 张
MAX_IMAGES = 30


# ============================================================
# Camera initialization
# ============================================================

print("")
print("====================================")
print("OV5640 Calibration Camera")
print("====================================")
print("")

sensor.reset()

# 灰度图
sensor.set_pixformat(sensor.GRAYSCALE)

# QVGA = 320 x 240
sensor.set_framesize(FRAME_SIZE)


# ============================================================
# Check camera
# ============================================================

camera_id = sensor.get_id()

print("Camera ID =", camera_id)
print("OV5640 ID =", sensor.OV5640)

if camera_id == sensor.OV5640:
    print("OV5640 detected successfully.")
else:
    print("WARNING: Camera is not OV5640!")


# ============================================================
# Let AE / AG stabilize
# ============================================================

print("")
print("Waiting for exposure/gain to stabilize...")

sensor.skip_frames(time=3000)


# ============================================================
# Autofocus ONCE
# ============================================================

print("")
print("====================================")
print("Starting autofocus...")
print("Keep checkerboard still.")
print("====================================")

try:

    sensor.ioctl(
        sensor.IOCTL_TRIGGER_AUTO_FOCUS
    )

    sensor.ioctl(
        sensor.IOCTL_WAIT_ON_AUTO_FOCUS,
        5000
    )

    print("")
    print("Autofocus completed.")
    print("Focus will remain fixed.")

except Exception as e:

    print("")
    print("Autofocus failed:")
    print(e)


# 等待对焦后图像稳定
sensor.skip_frames(time=1000)


# ============================================================
# Lock exposure and gain
# ============================================================

print("")
print("Locking exposure and gain...")

try:

    exposure = sensor.get_exposure_us()
    gain = sensor.get_gain_db()

    print("")
    print("Exposure =", exposure, "us")
    print("Gain     =", gain, "dB")

    sensor.set_auto_exposure(
        False,
        exposure_us=exposure
    )

    sensor.set_auto_gain(
        False,
        gain_db=gain
    )

    print("")
    print("Exposure locked.")
    print("Gain locked.")

except Exception as e:

    print("")
    print("Could not lock exposure/gain:")
    print(e)


# ============================================================
# Show files in current directory
# ============================================================

print("")
print("Current directory:")
print(os.getcwd())

print("")
print("Existing files:")
print(os.listdir())


# ============================================================
# Find next image number
#
# 避免覆盖以前的照片
# ============================================================

photo_index = 1

try:

    existing_files = os.listdir()

    while True:

        filename = "calib_%03d.pgm" % photo_index

        if filename not in existing_files:
            break

        photo_index += 1

except Exception as e:

    print("")
    print("Directory scan error:")
    print(e)

    raise


# ============================================================
# Ready
# ============================================================

print("")
print("====================================")
print("Calibration Capture Ready")
print("====================================")
print("")
print("Resolution : 320 x 240")
print("Format     : GRAYSCALE / PGM")
print("")
print("Board:")
print("Corners    : 11 x 8")
print("Square     : 30 mm")
print("")
print("Interval   : 5 seconds")
print("Images     :", MAX_IMAGES)
print("")
print(
    "First file : calib_%03d.pgm"
    % photo_index
)
print("")
print("Move checkerboard between captures.")
print("Stop moving before countdown reaches 1.")
print("====================================")
print("")


# ============================================================
# Automatic capture loop
# ============================================================

captured_count = 0


while captured_count < MAX_IMAGES:

    print("")
    print("------------------------------------")
    print(
        "Next image:",
        captured_count + 1,
        "/",
        MAX_IMAGES
    )
    print("------------------------------------")


    # ========================================================
    # 5-second countdown
    # ========================================================

    start_time = time.ticks_ms()

    last_second = -1


    while (
        time.ticks_diff(
            time.ticks_ms(),
            start_time
        )
        < CAPTURE_INTERVAL_MS
    ):

        # 实时更新 OpenMV IDE Frame Buffer
        img = sensor.snapshot()

        elapsed = time.ticks_diff(
            time.ticks_ms(),
            start_time
        )

        remaining_ms = (
            CAPTURE_INTERVAL_MS - elapsed
        )

        remaining_sec = (
            remaining_ms + 999
        ) // 1000


        if remaining_sec != last_second:

            print(
                "Capture in",
                remaining_sec,
                "seconds..."
            )

            last_second = remaining_sec


    # ========================================================
    # Capture fresh image
    # ========================================================

    img = sensor.snapshot()


    # ========================================================
    # Filename
    #
    # 保存到当前 OpenMV 主目录
    # ========================================================

    filename = (
        "calib_%03d.pgm"
        % photo_index
    )


    # ========================================================
    # Save
    # ========================================================

    try:

        img.save(filename)

    except Exception as e:

        print("")
        print("====================================")
        print("IMAGE SAVE ERROR")
        print("====================================")
        print("File:")
        print(filename)
        print("")
        print("Error:")
        print(e)

        raise


    # ========================================================
    # Success
    # ========================================================

    captured_count += 1

    print("")
    print(
        "Captured:",
        captured_count,
        "/",
        MAX_IMAGES
    )

    print("Saved:")
    print(filename)

    photo_index += 1


# ============================================================
# Finished
# ============================================================

print("")
print("====================================")
print("Calibration capture finished.")
print("====================================")
print("")
print("Captured:", captured_count)
print("")
print("Files are in OpenMV main directory.")
print("")
print(os.listdir())
print("")
print("====================================")


# Keep final framebuffer visible
while True:
    time.sleep_ms(1000)
