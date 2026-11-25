"""Offline unit tests for end-effector trajectory extraction and projection."""
import csv
from pathlib import Path
import sys
import tempfile
import types
import unittest
import numpy as np

# Allow importing render_fusion without requiring imageio_ffmpeg / cv2 at test time.
for mod_name in ('cv2', 'imageio_ffmpeg'):
    if mod_name not in sys.modules:
        sys.modules[mod_name] = types.ModuleType(mod_name)

from render_fusion import draw_dashed_polyline, fr3_fk_xyz, load_ee_trajectory, load_reference_trajectory, project_3d_to_camera, project_xy_box


class RenderTrajectoryTests(unittest.TestCase):
    def test_fr3_fk_zero_pose_finite_and_symmetric(self):
        xyz = fr3_fk_xyz(np.zeros(7))
        self.assertEqual(xyz.shape, (3,))
        self.assertTrue(np.all(np.isfinite(xyz)))
        # At q=0, y should be near 0 and z should be positive above base.
        self.assertAlmostEqual(float(xyz[1]), 0.0, places=5)
        self.assertGreater(float(xyz[2]), 0.5)

    def test_load_ee_trajectory_prefers_callback_states(self):
        with tempfile.TemporaryDirectory() as tmp:
            diag = Path(tmp)
            policy_rows = [
                {'robot_time_s': '1.00', **{f'obs{i}': '0.0' for i in range(7)}},
                {'robot_time_s': '1.04', **{f'obs{i}': '0.1' for i in range(7)}},
            ]
            with (diag / 'callback_states.csv').open('w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=['robot_time_s', 'O_T_EE_12', 'O_T_EE_13', 'O_T_EE_14'])
                w.writeheader()
                w.writerow({'robot_time_s': '1.000', 'O_T_EE_12': '0.30', 'O_T_EE_13': '-0.10', 'O_T_EE_14': '0.45'})
                w.writerow({'robot_time_s': '1.041', 'O_T_EE_12': '0.35', 'O_T_EE_13': '-0.05', 'O_T_EE_14': '0.48'})
            xyz, source = load_ee_trajectory(policy_rows, diag)
            self.assertEqual(source, 'callback_states_O_T_EE')
            np.testing.assert_allclose(xyz[0], [0.30, -0.10, 0.45])
            np.testing.assert_allclose(xyz[1], [0.35, -0.05, 0.48])

    def test_load_ee_trajectory_falls_back_to_fk(self):
        with tempfile.TemporaryDirectory() as tmp:
            diag = Path(tmp)
            policy_rows = [
                {'robot_time_s': '1.00', **{f'obs{i}': '0.0' for i in range(7)}},
                {'robot_time_s': '1.04', 'obs0': '0.3', **{f'obs{i}': '0.0' for i in range(1, 7)}},
            ]
            xyz, source = load_ee_trajectory(policy_rows, diag)
            self.assertEqual(source, 'policy_q_fk')
            self.assertEqual(xyz.shape, (2, 3))
            self.assertFalse(np.allclose(xyz[0], xyz[1]))

    def test_project_xy_box_stays_within_bounds(self):
        box = (984, 713, 1242, 868)
        xyz = np.array([[0.30, -0.10, 0.40], [0.45, 0.05, 0.42], [0.32, 0.02, 0.41]])
        extra = np.array([[0.25, -0.15, 0.40], [0.50, 0.10, 0.42]])
        pts, bounds, extra_pts = project_xy_box(xyz, box, extra_xyz_m=extra)
        self.assertEqual(len(pts), 3)
        self.assertEqual(len(extra_pts), 2)
        self.assertEqual(len(bounds), 4)
        for px, py in pts + extra_pts:
            self.assertGreaterEqual(px, box[0])
            self.assertLessEqual(px, box[2])
            self.assertGreaterEqual(py, box[1])
            self.assertLessEqual(py, box[3])

    def test_load_reference_trajectory_from_raw_commands(self):
        with tempfile.TemporaryDirectory() as tmp:
            diag = Path(tmp)
            with (diag / 'raw_commands.csv').open('w', newline='') as f:
                fields = ['index', 'elapsed_s'] + [f'nominal_reference_{i}' for i in range(7)]
                w = csv.DictWriter(f, fieldnames=fields)
                w.writeheader()
                for k in range(5):
                    row = {'index': k, 'elapsed_s': k * 0.001, **{f'nominal_reference_{i}': '0.0' for i in range(7)}}
                    row['nominal_reference_0'] = str(0.1 * k)
                    w.writerow(row)
            ref_xyz, src = load_reference_trajectory(diag)
            self.assertEqual(src, 'nominal_reference_fk')
            self.assertEqual(ref_xyz.shape, (5, 3))
            self.assertFalse(np.allclose(ref_xyz[0], ref_xyz[-1]))

    def test_draw_dashed_polyline_smoke(self):
        from PIL import Image, ImageDraw
        img = Image.new('RGB', (100, 100), 'white')
        d = ImageDraw.Draw(img)
        draw_dashed_polyline(d, [(10, 10), (90, 10), (90, 90)], fill='#8fa1b5', width=2)
        # Ensure pixels along the path were drawn
        arr = np.asarray(img)
        self.assertTrue(np.any(arr < 255))

    def test_project_3d_to_camera_pinhole_and_behind_mask(self):
        calib = {
            'image_size': [640, 480],
            'camera_matrix': [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]],
            'dist_coeffs': [0.0, 0.0, 0.0, 0.0, 0.0],
            'rvec': [0.0, 0.0, 0.0],
            'tvec': [0.0, 0.0, 1.0],
        }
        pts = np.array([
            [0.0, 0.0, 0.0],    # z_cam = 1.0 -> (320, 240)
            [0.1, -0.1, 0.0],   # z_cam = 1.0 -> (380, 180)
            [0.0, 0.0, -2.0],   # z_cam = -1.0 -> behind camera
        ])
        uv, ok = project_3d_to_camera(pts, calib)
        self.assertEqual(ok.tolist(), [True, True, False])
        np.testing.assert_allclose(uv[0], [320.0, 240.0], atol=1e-6)
        np.testing.assert_allclose(uv[1], [380.0, 180.0], atol=1e-6)


if __name__ == '__main__':
    unittest.main()
