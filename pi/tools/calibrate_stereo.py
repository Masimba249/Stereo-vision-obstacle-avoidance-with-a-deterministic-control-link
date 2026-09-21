#!/usr/bin/env python3
"""Stereo calibration for the Waveshare binocular module.

Without this, every distance the pipeline reports is a guess.  The pipeline
refuses to start on real hardware without a calibration file for exactly that
reason: a plausible-looking wrong distance is more dangerous than no distance.

Usage:
    # 1. capture pairs (press SPACE to keep a view, Q when done)
    python3 pi/tools/calibrate_stereo.py capture --out captures/

    # 2. solve
    python3 pi/tools/calibrate_stereo.py solve --in captures/ \
        --rows 6 --cols 9 --square-mm 25 --out config/stereo_calibration.yml

Aim for 20+ pairs with the board at varied distances, angles and image
positions - especially near the frame edges, where distortion actually lives.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from stereolink.calibration import StereoCalibration  # noqa: E402

TERM = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-6)


def split_eyes(frame: np.ndarray, swap: bool) -> tuple[np.ndarray, np.ndarray]:
    half = frame.shape[1] // 2
    a, b = frame[:, :half], frame[:, half:]
    return (b, a) if swap else (a, b)


def cmd_capture(args: argparse.Namespace) -> int:
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"cannot open device {args.device}")
        return 2

    pattern = (args.cols, args.rows)
    saved = 0
    print("SPACE = keep this pair, Q = finish")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            left, right = split_eyes(frame, args.swap_eyes)
            gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

            fl, cl = cv2.findChessboardCorners(gl, pattern, None)
            fr, cr = cv2.findChessboardCorners(gr, pattern, None)

            preview = np.hstack([left.copy(), right.copy()])
            if fl and fr:
                cv2.drawChessboardCorners(preview[:, :left.shape[1]], pattern, cl, fl)
                cv2.drawChessboardCorners(preview[:, left.shape[1]:], pattern, cr, fr)
            status = "BOTH EYES OK" if (fl and fr) else "board not seen by both eyes"
            cv2.putText(preview, f"{status} | saved={saved}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if (fl and fr) else (0, 0, 255), 2)
            cv2.imshow("calibration capture", cv2.resize(preview, None, fx=0.5, fy=0.5))

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" ") and fl and fr:
                cv2.imwrite(str(out_dir / f"left_{saved:03d}.png"), left)
                cv2.imwrite(str(out_dir / f"right_{saved:03d}.png"), right)
                saved += 1
                print(f"saved pair {saved}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
    print(f"{saved} pairs written to {out_dir}")
    return 0


def cmd_solve(args: argparse.Namespace) -> int:
    in_dir = Path(args.inp)
    lefts = sorted(in_dir.glob("left_*.png"))
    if not lefts:
        print(f"no left_*.png in {in_dir}")
        return 2

    pattern = (args.cols, args.rows)
    # Object points in metres, so T comes out in metres and depth is metric.
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square_mm / 1000.0

    obj_points, img_left, img_right = [], [], []
    size = None
    for lp in lefts:
        rp = lp.with_name(lp.name.replace("left_", "right_"))
        if not rp.exists():
            continue
        gl = cv2.imread(str(lp), cv2.IMREAD_GRAYSCALE)
        gr = cv2.imread(str(rp), cv2.IMREAD_GRAYSCALE)
        if gl is None or gr is None:
            continue
        size = (gl.shape[1], gl.shape[0])
        fl, cl = cv2.findChessboardCorners(gl, pattern, None)
        fr, cr = cv2.findChessboardCorners(gr, pattern, None)
        if not (fl and fr):
            print(f"  skipped {lp.name}: board not found in both eyes")
            continue
        # Sub-pixel refinement matters: whole-pixel corners put roughly a
        # percent of error straight into the baseline.
        cv2.cornerSubPix(gl, cl, (11, 11), (-1, -1), TERM)
        cv2.cornerSubPix(gr, cr, (11, 11), (-1, -1), TERM)
        obj_points.append(objp)
        img_left.append(cl)
        img_right.append(cr)

    n = len(obj_points)
    print(f"{n} usable pairs")
    if n < 8:
        print("need at least 8 good pairs for a trustworthy solve")
        return 2

    flags_mono = cv2.CALIB_RATIONAL_MODEL
    rms1, K1, D1, *_ = cv2.calibrateCamera(obj_points, img_left, size, None, None,
                                           flags=flags_mono)
    rms2, K2, D2, *_ = cv2.calibrateCamera(obj_points, img_right, size, None, None,
                                           flags=flags_mono)
    print(f"  mono RMS: left={rms1:.4f} px  right={rms2:.4f} px")

    # Fix the intrinsics and solve only the extrinsics: with a rigid module the
    # per-eye models are already well determined, and letting them float again
    # trades a better residual for a worse baseline.
    rms, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        obj_points, img_left, img_right, K1, D1, K2, D2, size,
        criteria=TERM, flags=cv2.CALIB_FIX_INTRINSIC)

    calib = StereoCalibration(size, K1, D1, K2, D2, R, T, float(rms))
    calib.save(args.out)

    baseline_mm = calib.baseline_m * 1000.0
    print(f"  stereo RMS: {rms:.4f} px")
    print(f"  baseline:   {baseline_mm:.2f} mm")
    print(f"  focal:      {K1[0, 0]:.1f} px")
    print(f"  written to  {args.out}")

    if rms > 1.0:
        print("\n  WARNING: RMS above 1 px. Recapture with more varied poses; "
              "depth from this calibration will be unreliable.")
    if not 20.0 < baseline_mm < 200.0:
        print(f"\n  WARNING: baseline of {baseline_mm:.0f} mm looks wrong for "
              "this module. Check --square-mm and that the eyes are not swapped.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    cap = sub.add_parser("capture", help="grab chessboard pairs from the camera")
    cap.add_argument("--device", type=int, default=0)
    cap.add_argument("--width", type=int, default=2560)
    cap.add_argument("--height", type=int, default=720)
    cap.add_argument("--rows", type=int, default=6, help="inner corners per column")
    cap.add_argument("--cols", type=int, default=9, help="inner corners per row")
    cap.add_argument("--swap-eyes", action="store_true")
    cap.add_argument("--out", default="captures")
    cap.set_defaults(func=cmd_capture)

    sol = sub.add_parser("solve", help="solve calibration from captured pairs")
    sol.add_argument("--in", dest="inp", default="captures")
    sol.add_argument("--rows", type=int, default=6)
    sol.add_argument("--cols", type=int, default=9)
    sol.add_argument("--square-mm", type=float, default=25.0)
    sol.add_argument("--out", default="config/stereo_calibration.yml")
    sol.set_defaults(func=cmd_solve)

    args = ap.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
