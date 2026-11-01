"""Calibrates one webcam (e.g. an Arducam on the robot) with a ChArUco board, for the FTC
AprilTag processor's setLensIntrinsics(fx, fy, cx, cy).

Plug the camera into the laptop over USB, then:

    pip install opencv-python numpy
    python tools/camera-calibration/calibrate.py --name CameraL --square-mm 25

The defaults match our board: A4, 25 mm squares, markers 73.3% of the square, DICT_6X6_250,
legacy ChArUco layout, and 9 columns by 6 rows as OpenCV counts them (our generator called it
9 rows by 6 columns; with the wrong way round, or without the legacy layout, no corners are
found). Measure the printed squares and pass the real size
with --square-mm: printers scale by a percent or two, and that size sets the scale.

Live window: SPACE saves the current frame (only when enough corners are seen), A toggles
auto-capture, C calibrates, Q quits. Aim for 20 to 30 frames covering the whole picture,
corners and edges included, with the board tilted up to about 45 degrees in different
directions. Saved frames go to tools/camera-calibration/captures/<name>/ so you can recalibrate
without the camera:

    python tools/camera-calibration/calibrate.py --name CameraL --square-mm 25 --images tools/camera-calibration/captures/CameraL

The calibration is only valid at the resolution it was taken at, so use the same resolution
as the robot code (VisionPortal.Builder.setCameraResolution).
"""

import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

MIN_CORNERS = 12  # A frame needs at least this many ChArUco corners to be used
MIN_FRAMES = 15  # Calibrating needs at least this many frames
AUTO_CAPTURE_PERIOD_S = 1.0  # Auto-capture saves at most one frame this often
AUTO_CAPTURE_MIN_MOVE_PX = 40  # ...and only once the board has moved this far, so frames differ


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", default="camera", help="Label for the output files, e.g. CameraL")
    p.add_argument("--camera", type=int, default=0, help="Laptop camera index (0, 1, ...)")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--columns", type=int, default=9, help="Board squares along OpenCV's x")
    p.add_argument("--rows", type=int, default=6, help="Board squares along OpenCV's y")
    p.add_argument("--square-mm", type=float, default=25.0, help="Measured square width, after printing")
    p.add_argument("--marker-ratio", type=float, default=0.733,
                   help="ArUco marker width as a share of the square width. Only used to build the board; "
                        "it does not change the result")
    p.add_argument("--dictionary", default="DICT_6X6_250")
    p.add_argument("--no-legacy", action="store_true",
                   help="Use OpenCV's new ChArUco layout (only matters with an even number of rows)")
    p.add_argument("--images", help="Calibrate from saved images in this folder instead of a live camera")
    return p.parse_args()


def make_board(args):
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "CharucoDetector"):
        sys.exit("Needs OpenCV 4.7 or newer: pip install --upgrade opencv-python")
    if not 0 < args.marker_ratio < 1:
        sys.exit("--marker-ratio must be between 0 and 1")
    args.marker_mm = args.square_mm * args.marker_ratio
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
    board = cv2.aruco.CharucoBoard((args.columns, args.rows), args.square_mm / 1000.0,
                                   args.marker_mm / 1000.0, dictionary)
    board.setLegacyPattern(not args.no_legacy)
    return board, cv2.aruco.CharucoDetector(board)


def detect(detector, gray):
    """Returns (charuco corners, ids), or (None, None) when too few corners are seen."""
    corners, ids, _, _ = detector.detectBoard(gray)
    if ids is None or len(ids) < MIN_CORNERS:
        return None, None
    # OpenCV 5 returns (N, 2) corners, but drawing and matching want (N, 1, 2) like OpenCV 4
    return corners.reshape(-1, 1, 2).astype(np.float32), ids.reshape(-1, 1)


def calibrate(board, views, image_size):
    """views: list of (charuco corners, ids). Returns (rms, camera matrix, distortion, per-view rms)."""
    object_points, image_points = [], []
    for corners, ids in views:
        obj, img = board.matchImagePoints(corners, ids)
        object_points.append(obj)
        image_points.append(img)
    rms, matrix, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, image_size, None, None)
    per_view = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, dist)
        # Flattened, since OpenCV 5 gives image points as (N, 2) but projectPoints as (N, 1, 2)
        error = projected.reshape(-1, 2) - img.reshape(-1, 2)
        per_view.append(float(np.sqrt(np.mean(np.sum(error ** 2, axis=1)))))
    return rms, matrix, dist, per_view


def report(args, rms, matrix, dist, per_view, image_size, names):
    fx, fy, cx, cy = matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2]
    width, height = image_size
    hfov = np.degrees(2 * np.arctan(width / (2 * fx)))
    print()
    print(f"{args.name}  {width}x{height}  {len(per_view)} frames")
    print(f"  RMS reprojection error: {rms:.3f} px  (under 0.5 is good, over 1 means retake frames)")
    print(f"  fx {fx:.3f}  fy {fy:.3f}  cx {cx:.3f}  cy {cy:.3f}  (horizontal field of view {hfov:.1f} deg)")
    print(f"  distortion k1 k2 p1 p2 k3: {np.array2string(dist.ravel()[:5], precision=5)}")
    worst = sorted(zip(per_view, names), reverse=True)[:3]
    print("  worst frames: " + ", ".join(f"{name} {err:.2f}px" for err, name in worst))
    print()
    print("  Java (AprilTagProcessor.Builder):")
    print(f"    .setLensIntrinsics({fx:.3f}, {fy:.3f}, {cx:.3f}, {cy:.3f})")

    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"{args.name}_{width}x{height}.json")
    with open(out_path, "w") as f:
        json.dump({
            "name": args.name, "width": width, "height": height, "frames": len(per_view),
            "rms_px": rms, "fx": fx, "fy": fy, "cx": cx, "cy": cy,
            "distortion": dist.ravel().tolist(),
            "board": {"columns": args.columns, "rows": args.rows, "square_mm": args.square_mm,
                      "marker_mm": args.marker_mm, "dictionary": args.dictionary,
                      "legacy": not args.no_legacy},
            "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        }, f, indent=2)
    print(f"  saved {out_path}")


def from_images(args, board, detector):
    paths = sorted(glob.glob(os.path.join(args.images, "*.png")) + glob.glob(os.path.join(args.images, "*.jpg")))
    views, names, image_size = [], [], None
    for path in paths:
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print(f"  skipped {os.path.basename(path)}: unreadable")
            continue
        size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = size
        elif size != image_size:
            sys.exit(f"{path} is {size[0]}x{size[1]}, but earlier images are {image_size[0]}x{image_size[1]}")
        corners, ids = detect(detector, gray)
        if corners is None:
            print(f"  skipped {os.path.basename(path)}: too few corners")
            continue
        views.append((corners, ids))
        names.append(os.path.basename(path))
    if len(views) < MIN_FRAMES:
        sys.exit(f"Only {len(views)} usable images; need at least {MIN_FRAMES}")
    report(args, *calibrate(board, views, image_size), image_size, names)


def open_camera(args):
    # DirectShow on Windows; MJPG so 1280x720 isn't limited by raw USB bandwidth
    api = cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY
    cap = cv2.VideoCapture(args.camera, api)
    if not cap.isOpened():
        sys.exit(f"Could not open camera {args.camera}. Try --camera 1 or 2.")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    ok, frame = cap.read()
    if not ok:
        sys.exit("Camera opened but gave no picture")
    actual = (frame.shape[1], frame.shape[0])
    if actual != (args.width, args.height):
        sys.exit(f"Asked for {args.width}x{args.height} but the camera gives {actual[0]}x{actual[1]}. "
                 "Pick a resolution it supports (and use the same one on the robot).")
    return cap, actual


def live(args, board, detector):
    cap, image_size = open_camera(args)
    save_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures", args.name)
    os.makedirs(save_dir, exist_ok=True)
    views, names = [], []
    auto = False
    last_auto_s, last_centre = 0.0, None
    window = f"{args.name} calibration"
    print("SPACE save frame | A auto-capture | C calibrate | Q quit")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Lost the camera")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = detect(detector, gray)
        shown = frame.copy()
        for saved_corners, _ in views:  # Orange dots show where saved frames already cover
            for c in saved_corners.reshape(-1, 2):
                cv2.circle(shown, (int(c[0]), int(c[1])), 2, (0, 140, 255), -1)
        if corners is not None:
            cv2.aruco.drawDetectedCornersCharuco(shown, corners, ids, (0, 255, 0))
        sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()  # Higher is sharper: turn the lens for the peak
        status = (f"frames {len(views)}  corners {0 if ids is None else len(ids)}  "
                  f"auto {'ON' if auto else 'off'}  sharpness {sharpness:.0f}")
        cv2.putText(shown, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow(window, shown)

        key = cv2.waitKey(1) & 0xFF
        save = False
        if key == ord(" "):
            save = corners is not None
            if not save:
                print("Not enough corners in view")
        elif key in (ord("a"), ord("A")):
            auto = not auto
        elif key in (ord("c"), ord("C")):
            if len(views) < MIN_FRAMES:
                print(f"Need at least {MIN_FRAMES} frames, have {len(views)}")
            else:
                report(args, *calibrate(board, views, image_size), image_size, names)
        elif key in (ord("q"), ord("Q"), 27):
            break

        if auto and corners is not None and time.time() - last_auto_s >= AUTO_CAPTURE_PERIOD_S:
            centre = corners.reshape(-1, 2).mean(axis=0)
            if last_centre is None or np.linalg.norm(centre - last_centre) >= AUTO_CAPTURE_MIN_MOVE_PX:
                save, last_auto_s, last_centre = True, time.time(), centre

        if save:
            name = f"{args.name}_{len(views):03d}.png"
            cv2.imwrite(os.path.join(save_dir, name), frame)  # The raw frame, not the drawn one
            views.append((corners, ids))
            names.append(name)
            print(f"saved {name} ({len(ids)} corners)")
    cap.release()
    cv2.destroyAllWindows()


def main():
    args = parse_args()
    board, detector = make_board(args)
    if args.images:
        from_images(args, board, detector)
    else:
        live(args, board, detector)


if __name__ == "__main__":
    main()
