# -*- coding: utf-8 -*-
"""
OpenMV OV5640 Camera Calibration with OpenCV

Calibration board:
- Inner corners: 11 x 8
- Square size: 30 mm

Input images:
- calib_001.pgm
- calib_002.pgm
- ...

Recommended folder structure:
camera_calibration/
├── calibrate_camera.py
└── images/
    ├── calib_001.pgm
    ├── calib_002.pgm
    └── ...

Run:
    python calibrate_camera.py

Dependencies:
    pip install opencv-python numpy
"""

import cv2
import numpy as np
from pathlib import Path
import json

# ============================================================
# Configuration
# ============================================================

# 棋盘格内角点数量
BOARD_COLS = 11
BOARD_ROWS = 8
BOARD_SIZE = (BOARD_COLS, BOARD_ROWS)

# 单位：mm
SQUARE_SIZE_MM = 30.0

# 标定图片目录
IMAGE_DIR = Path("./images")

# 匹配 OpenMV 拍摄的灰度 PGM 图片
IMAGE_PATTERN = "calib_*.pgm"

# 输出目录
OUTPUT_DIR = Path("./calibration_output")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# 是否保存角点检测结果图
SAVE_CORNER_IMAGES = True


def main():
    print("=" * 70)
    print("OpenMV OV5640 Camera Calibration")
    print("=" * 70)
    print(f"Board inner corners : {BOARD_COLS} x {BOARD_ROWS}")
    print(f"Square size         : {SQUARE_SIZE_MM:.2f} mm")
    print(f"Image directory     : {IMAGE_DIR.resolve()}")
    print(f"Image pattern       : {IMAGE_PATTERN}")
    print("=" * 70)

    image_paths = sorted(IMAGE_DIR.glob(IMAGE_PATTERN))

    if not image_paths:
        raise FileNotFoundError(
            f"\n没有找到标定图片。\n"
            f"请把 calib_*.pgm 放到目录：\n{IMAGE_DIR.resolve()}\n"
        )

    print(f"\nFound {len(image_paths)} images.\n")

    # ========================================================
    # 世界坐标系中的棋盘格角点
    #
    # (0,0,0), (30,0,0), (60,0,0), ...
    # ========================================================

    objp = np.zeros((BOARD_ROWS * BOARD_COLS, 3), np.float32)

    grid = np.mgrid[0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2)
    objp[:, :2] = grid * SQUARE_SIZE_MM

    # 每一张有效图片的 3D 世界点
    objpoints = []

    # 每一张有效图片的 2D 图像点
    imgpoints = []

    valid_images = []
    failed_images = []

    image_size = None

    # 亚像素优化终止条件
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        50,
        1e-4
    )

    # ========================================================
    # 棋盘格检测
    # ========================================================

    for idx, image_path in enumerate(image_paths, start=1):
        gray = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)

        if gray is None:
            print(f"[{idx:02d}/{len(image_paths):02d}] READ FAILED : {image_path.name}")
            failed_images.append(image_path.name)
            continue

        h, w = gray.shape[:2]

        if image_size is None:
            image_size = (w, h)
            print(f"Image size: {w} x {h}\n")
        else:
            if image_size != (w, h):
                print(
                    f"[{idx:02d}/{len(image_paths):02d}] SIZE MISMATCH: "
                    f"{image_path.name} = {w}x{h}, expected {image_size[0]}x{image_size[1]}"
                )
                failed_images.append(image_path.name)
                continue

        # 首选 findChessboardCornersSB，鲁棒性通常更好
        found = False
        corners = None

        try:
            found, corners = cv2.findChessboardCornersSB(
                gray,
                BOARD_SIZE,
                flags=(
                    cv2.CALIB_CB_NORMALIZE_IMAGE
                    | cv2.CALIB_CB_EXHAUSTIVE
                    | cv2.CALIB_CB_ACCURACY
                )
            )
        except Exception:
            found = False

        # 如果 SB 失败，则回退到传统算法
        if not found:
            found, corners = cv2.findChessboardCorners(
                gray,
                BOARD_SIZE,
                flags=(
                    cv2.CALIB_CB_ADAPTIVE_THRESH
                    | cv2.CALIB_CB_NORMALIZE_IMAGE
                    | cv2.CALIB_CB_FAST_CHECK
                )
            )

            if found:
                corners = cv2.cornerSubPix(
                    gray,
                    corners,
                    winSize=(11, 11),
                    zeroZone=(-1, -1),
                    criteria=criteria
                )

        if found:
            objpoints.append(objp.copy())
            imgpoints.append(corners.astype(np.float32))
            valid_images.append(image_path.name)

            print(
                f"[{idx:02d}/{len(image_paths):02d}] OK          : "
                f"{image_path.name}"
            )

            if SAVE_CORNER_IMAGES:
                preview = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(
                    preview,
                    BOARD_SIZE,
                    corners,
                    found
                )

                out_path = OUTPUT_DIR / f"corners_{image_path.stem}.png"
                cv2.imwrite(str(out_path), preview)

        else:
            failed_images.append(image_path.name)

            print(
                f"[{idx:02d}/{len(image_paths):02d}] NOT FOUND   : "
                f"{image_path.name}"
            )

    # ========================================================
    # 检查有效图片数量
    # ========================================================

    print("\n" + "=" * 70)
    print("Corner detection summary")
    print("=" * 70)
    print(f"Total images  : {len(image_paths)}")
    print(f"Valid images  : {len(valid_images)}")
    print(f"Failed images : {len(failed_images)}")

    if len(valid_images) < 8:
        raise RuntimeError(
            "\n有效标定图片少于 8 张，建议重新拍摄更多不同角度、不同位置的棋盘格图片。"
        )

    # ========================================================
    # 相机标定
    # ========================================================

    print("\nRunning cv2.calibrateCamera() ...")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None
    )

    # ========================================================
    # 重投影误差
    # ========================================================

    per_image_errors = []
    total_squared_error = 0.0
    total_points = 0

    for i in range(len(objpoints)):
        projected_points, _ = cv2.projectPoints(
            objpoints[i],
            rvecs[i],
            tvecs[i],
            camera_matrix,
            dist_coeffs
        )

        # OpenCV 5.x 下 findChessboardCornersSB() 返回的角点形状
        # 可能是 (N, 2)，而 projectPoints() 返回 (N, 1, 2)。
        # cv2.norm() 要求二者 shape/channel 完全一致，因此统一 reshape 为 (N, 2)。
        observed = np.asarray(
            imgpoints[i],
            dtype=np.float64
        ).reshape(-1, 2)

        projected = np.asarray(
            projected_points,
            dtype=np.float64
        ).reshape(-1, 2)

        residuals = observed - projected

        # 每个角点包含 x/y 两个方向误差。
        # 这里计算“每个角点的二维欧氏重投影 RMSE”，单位为像素。
        squared_error = np.sum(residuals ** 2)

        num_points = observed.shape[0]

        image_rmse = np.sqrt(
            squared_error / num_points
        )

        per_image_errors.append(
            float(image_rmse)
        )

        total_squared_error += squared_error
        total_points += num_points

    global_reprojection_rmse = np.sqrt(
        total_squared_error / total_points
    )

    # ========================================================
    # 输出结果
    # ========================================================

    print("\n" + "=" * 70)
    print("Calibration Result")
    print("=" * 70)

    print(f"\nRMS returned by OpenCV:")
    print(f"{rms:.8f} pixels")

    print("\nCamera Matrix K:")
    print(camera_matrix)

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    print("\nIntrinsic parameters:")
    print(f"fx = {fx:.8f} pixels")
    print(f"fy = {fy:.8f} pixels")
    print(f"cx = {cx:.8f} pixels")
    print(f"cy = {cy:.8f} pixels")

    print("\nDistortion coefficients:")
    print(dist_coeffs)

    flat_dist = dist_coeffs.ravel()

    names = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"]

    print("\nDistortion parameters:")
    for i, value in enumerate(flat_dist):
        if i < len(names):
            print(f"{names[i]} = {value:.12f}")
        else:
            print(f"d{i} = {value:.12f}")

    print("\nGlobal reprojection RMSE:")
    print(f"{global_reprojection_rmse:.8f} pixels")

    print("\nPer-image reprojection RMSE:")
    for name, error in zip(valid_images, per_image_errors):
        print(f"{name:20s} : {error:.8f} px")

    # ========================================================
    # 计算一个新的相机矩阵，用于去畸变
    # ========================================================

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix,
        dist_coeffs,
        image_size,
        alpha=0.0,
        newImgSize=image_size
    )

    print("\nOptimal new camera matrix:")
    print(new_camera_matrix)

    print("\nROI:")
    print(roi)

    # ========================================================
    # 保存 NPZ
    # ========================================================

    np.savez(
        OUTPUT_DIR / "camera_calibration.npz",
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        new_camera_matrix=new_camera_matrix,
        image_size=np.array(image_size),
        board_size=np.array(BOARD_SIZE),
        square_size_mm=SQUARE_SIZE_MM,
        rms=rms,
        reprojection_rmse=global_reprojection_rmse
    )

    # ========================================================
    # 保存 YAML
    # ========================================================

    fs = cv2.FileStorage(
        str(OUTPUT_DIR / "camera_calibration.yaml"),
        cv2.FILE_STORAGE_WRITE
    )

    fs.write("image_width", image_size[0])
    fs.write("image_height", image_size[1])

    fs.write("board_cols", BOARD_COLS)
    fs.write("board_rows", BOARD_ROWS)
    fs.write("square_size_mm", SQUARE_SIZE_MM)

    fs.write("camera_matrix", camera_matrix)
    fs.write("distortion_coefficients", dist_coeffs)
    fs.write("new_camera_matrix", new_camera_matrix)

    fs.write("opencv_rms", float(rms))
    fs.write(
        "reprojection_rmse",
        float(global_reprojection_rmse)
    )

    fs.release()

    # ========================================================
    # 保存 JSON
    # ========================================================

    json_result = {
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "board_inner_corners": [
            BOARD_COLS,
            BOARD_ROWS
        ],
        "square_size_mm": float(SQUARE_SIZE_MM),

        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": dist_coeffs.ravel().tolist(),
        "new_camera_matrix": new_camera_matrix.tolist(),

        "fx": float(fx),
        "fy": float(fy),
        "cx": float(cx),
        "cy": float(cy),

        "opencv_rms_pixels": float(rms),
        "global_reprojection_rmse_pixels": float(
            global_reprojection_rmse
        ),

        "valid_images": valid_images,
        "failed_images": failed_images,

        "per_image_reprojection_rmse_pixels": {
            name: err
            for name, err in zip(
                valid_images,
                per_image_errors
            )
        }
    }

    with open(
        OUTPUT_DIR / "camera_calibration.json",
        "w",
        encoding="utf-8"
    ) as f:
        json.dump(
            json_result,
            f,
            ensure_ascii=False,
            indent=4
        )

    # ========================================================
    # 用第一张有效图做去畸变示例
    # ========================================================

    if valid_images:
        test_image_path = IMAGE_DIR / valid_images[0]

        test_img = cv2.imread(
            str(test_image_path),
            cv2.IMREAD_GRAYSCALE
        )

        undistorted = cv2.undistort(
            test_img,
            camera_matrix,
            dist_coeffs,
            None,
            new_camera_matrix
        )

        cv2.imwrite(
            str(OUTPUT_DIR / "undistorted_example.png"),
            undistorted
        )

    print("\n" + "=" * 70)
    print("Calibration finished successfully.")
    print("=" * 70)

    print(f"\nResults saved to:")
    print(OUTPUT_DIR.resolve())

    print("\nFiles:")
    print("  camera_calibration.npz")
    print("  camera_calibration.yaml")
    print("  camera_calibration.json")
    print("  undistorted_example.png")

    if SAVE_CORNER_IMAGES:
        print("  corners_calib_xxx.png")

    if failed_images:
        print("\nImages where corners were NOT detected:")
        for name in failed_images:
            print(" ", name)

    print("\nDone.")


if __name__ == "__main__":
    main()
