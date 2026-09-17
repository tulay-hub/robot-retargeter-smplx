#!/usr/bin/env python3
"""Resample a retargeted robot-motion qpos CSV to a different frame rate.

Input CSV rows are MuJoCo qpos-like rows:
    [tx, ty, tz, qx, qy, qz, qw, joint_0, joint_1, ...]
with the base quaternion stored as XYZW.

Interpolation:
- translation (cols 0-2) and joint angles (cols 7+): linear interpolation
- quaternion (cols 3-6): spherical linear interpolation (SLERP) in XYZW order

Usage:
    python scripts/resample_motion_csv.py \
        --input output_data/robot_motion/side_roll_R_002__A415_M_from_g1_lens110.csv \
        --output output_data/robot_motion/side_roll_R_002__A415_M_from_g1_100hz_lens110.csv \
        --input-fps 120 --output-fps 100
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def slerp_xyzw(q0: np.ndarray, q1: np.ndarray, t: np.ndarray) -> np.ndarray:
    """SLERP between two XYZW quaternions at fractions t (shape [N])."""
    q0w = np.concatenate([q0[3:4], q0[0:3]])
    q1w = np.concatenate([q1[3:4], q1[0:3]])
    dot = float(np.dot(q0w, q1w))
    if dot < 0.0:
        q1w = -q1w
        dot = -dot
    dot = min(dot, 1.0)
    if dot > 0.9995:
        result = q0w + t[:, None] * (q1w - q0w)
    else:
        theta0 = np.arccos(dot)
        sin_theta0 = np.sin(theta0)
        s0 = np.sin((1.0 - t) * theta0) / sin_theta0
        s1 = np.sin(t * theta0) / sin_theta0
        result = s0[:, None] * q0w + s1[:, None] * q1w
    result /= np.linalg.norm(result, axis=1, keepdims=True)
    return np.concatenate([result[:, 1:4], result[:, 0:1]], axis=1)


def resample(data: np.ndarray, src_fps: float, dst_fps: float) -> np.ndarray:
    n_src = data.shape[0]
    duration = n_src / src_fps
    n_dst = int(round(duration * dst_fps))
    t_src = np.arange(n_src, dtype=np.float64) / src_fps
    t_dst = np.arange(n_dst, dtype=np.float64) / dst_fps

    out = np.zeros((n_dst, data.shape[1]), dtype=np.float64)
    for col in range(data.shape[1]):
        if 3 <= col <= 6:
            continue
        out[:, col] = np.interp(t_dst, t_src, data[:, col])

    q_src = data[:, 3:7]
    for i, t in enumerate(t_dst):
        idx = np.searchsorted(t_src, t, side="right") - 1
        idx = min(max(idx, 0), n_src - 2)
        frac = (t - t_src[idx]) / (t_src[idx + 1] - t_src[idx])
        out[i, 3:7] = slerp_xyzw(q_src[idx], q_src[idx + 1], np.array([frac]))[0]
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Resample robot-motion qpos CSV frame rate")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--input-fps", type=float, default=120.0)
    parser.add_argument("--output-fps", type=float, default=100.0)
    args = parser.parse_args()

    data = np.loadtxt(args.input, delimiter=",")
    if data.ndim == 1:
        data = data[None, :]
    resampled = resample(data, args.input_fps, args.output_fps)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output, resampled, delimiter=",", fmt="%.6f")
    print(
        f"[ok] {args.input} ({data.shape[0]} frames @ {args.input_fps:g} fps) -> "
        f"{args.output} ({resampled.shape[0]} frames @ {args.output_fps:g} fps)"
    )


if __name__ == "__main__":
    main()
