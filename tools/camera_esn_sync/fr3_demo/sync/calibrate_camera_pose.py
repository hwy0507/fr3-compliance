"""Calibrate camera-to-robot extrinsics from session frames using cv2.solvePnP.

Supports either interactive mouse clicking (--interactive) or a JSON list of
[{"frame": <camera_frame_idx>, "u": <px>, "v": <px>}, ...] (--points).
"""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np
from render_fusion import load_ee_trajectory, rows

# Default approximate D435i 640x480 color intrinsics (fx=fy~615, cx=320, cy=240).
# For best accuracy, pass --intrinsics with actual device intrinsics JSON.
DEFAULT_K = np.array([[615.0, 0.0, 320.0], [0.0, 615.0, 240.0], [0.0, 0.0, 1.0]], dtype=float)
DEFAULT_DIST = np.zeros(5, dtype=float)


def _load_intrinsics(path):
    if path is None:
        return DEFAULT_K.copy(), DEFAULT_DIST.copy()
    data = json.loads(Path(path).read_text())
    return np.asarray(data['camera_matrix'], dtype=float), np.asarray(data.get('dist_coeffs', [0, 0, 0, 0, 0]), dtype=float)


def _pick_points_interactive(raw_yuyv, frame_indices):
    picked = []
    for fi in frame_indices:
        bgr = cv2.cvtColor(raw_yuyv[fi], cv2.COLOR_YUV2BGR_YUY2)
        vis = bgr.copy()
        cv2.putText(vis, f'Frame {fi}: Click EE tip, press SPACE/ENTER (ESC skip)', (12, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        click = []

        def on_mouse(event, x, y, _flags, _param):
            if event == cv2.EVENT_LBUTTONDOWN:
                click.clear()
                click.extend([x, y])
                tmp = vis.copy()
                cv2.circle(tmp, (x, y), 6, (0, 0, 255), -1)
                cv2.imshow('Calibrate EE', tmp)

        cv2.imshow('Calibrate EE', vis)
        cv2.setMouseCallback('Calibrate EE', on_mouse)
        while True:
            key = cv2.waitKey(30) & 0xFF
            if key in (13, 32) and click:
                picked.append({'frame': int(fi), 'u': float(click[0]), 'v': float(click[1])})
                break
            if key == 27:
                break
    cv2.destroyAllWindows()
    return picked


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--session', type=Path, required=True)
    ap.add_argument('--intrinsics', type=Path, default=None, help='Optional JSON with camera_matrix and dist_coeffs')
    ap.add_argument('--points', type=Path, default=None, help='JSON list of {"frame": int, "u": float, "v": float}')
    ap.add_argument('--interactive', action='store_true', help='Click end-effector positions on sampled frames')
    ap.add_argument('--frames', type=str, default='', help='Comma-separated camera frame indices for --interactive')
    ap.add_argument('--output', type=Path, default=None, help='Defaults to SESSION/camera_calib.json')
    args = ap.parse_args()

    root = args.session.resolve()
    camera = root / 'camera'
    cycle = next(root.glob('cycle_*'))
    policy = cycle / 'execute_diagnostics/policy_sync.csv'
    pr = rows(policy)
    pairs = rows(root / 'alignment/frame_policy_pairs.csv')
    cr = rows(camera / 'camera_frames.csv')
    n = len(cr)
    ee_xyz, ee_source = load_ee_trajectory(pr, policy.parent)
    raw = np.memmap(camera / 'camera.yuyv', mode='r', dtype=np.uint8, shape=(n, 480, 640, 2))

    if args.points:
        picked = json.loads(args.points.read_text())
    elif args.interactive:
        if args.frames:
            indices = [int(x) for x in args.frames.split(',') if x.strip()]
        else:
            valid_idx = [i for i, p in enumerate(pairs) if int(p['valid']) == 1]
            step = max(1, len(valid_idx) // 8)
            indices = valid_idx[::step][:8]
        picked = _pick_points_interactive(raw, indices)
    else:
        raise SystemExit('Specify --points <json> or --interactive')

    obj_pts, img_pts = [], []
    for item in picked:
        fi = int(item['frame'])
        pair = pairs[fi]
        if int(pair['valid']) != 1:
            raise ValueError(f'Frame {fi} has no valid policy alignment')
        pi = int(pair['nearest_policy_sample'])
        obj_pts.append(ee_xyz[pi])
        img_pts.append([float(item['u']), float(item['v'])])

    if len(obj_pts) < 4:
        raise ValueError(f'Need at least 4 valid points for solvePnP, got {len(obj_pts)}')

    obj_arr = np.asarray(obj_pts, dtype=float)
    img_arr = np.asarray(img_pts, dtype=float)
    K, dist = _load_intrinsics(args.intrinsics)
    ok, rvec, tvec = cv2.solvePnP(obj_arr, img_arr, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError('cv2.solvePnP failed')
    proj, _ = cv2.projectPoints(obj_arr, rvec, tvec, K, dist)
    err = np.linalg.norm(proj.reshape(-1, 2) - img_arr, axis=1)

    calib = {
        'image_size': [640, 480],
        'camera_matrix': K.tolist(),
        'dist_coeffs': dist.tolist(),
        'rvec': rvec.ravel().tolist(),
        'tvec': tvec.ravel().tolist(),
        'ee_trajectory_source': ee_source,
        'num_points': len(obj_pts),
        'mean_reproj_error_px': float(err.mean()),
        'max_reproj_error_px': float(err.max()),
    }
    out_path = args.output or (root / 'camera_calib.json')
    out_path.write_text(json.dumps(calib, indent=2) + '\n')
    print(json.dumps(calib, indent=2))


if __name__ == '__main__':
    main()
