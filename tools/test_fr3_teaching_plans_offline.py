"""Synthetic plans only. Optional C++ validation uses ONLY --offline, never an IP."""
import importlib.abc
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from scipy.interpolate import make_interp_spline

sys.path.insert(0, str(Path(__file__).resolve().parent))
import export_fr3_taught_plans_20261114 as exporter
from prepare_fr3_teaching_candidates_20261114 import interpolation_knots


class NoHardware(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname.split('.')[0] in ('pylibfranka', 'pyrealsense2'):
            raise AssertionError('hardware import forbidden')


class PlanTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        guard = NoHardware();sys.meta_path.insert(0, guard)
        self.addCleanup(sys.meta_path.remove, guard)
        net = patch.object(socket, 'socket', side_effect=AssertionError('network forbidden'))
        net.start();self.addCleanup(net.stop)
        self.q0 = np.array([0., -.5, 0., -1.5, 0., 1.5, 0.])
        self.q1 = self.q0 + np.array([.04, 0., .02, 0., 0., 0., 0.])

    def export(self, hold=None):
        candidate = self.root/'candidate';candidate.mkdir()
        (candidate/'ANALYSIS.json').write_text(json.dumps({'held_j1_rad': hold}))
        t = np.linspace(0., 5., 31);u=t/5
        q = self.q0 + (u**3*(10-15*u+6*u*u))[:, None]*(self.q1-self.q0)
        if hold is not None:q[:, 0] = hold
        zeros=np.zeros(7)
        spline=make_interp_spline(t,q,k=5,bc_type=([(1,zeros),(2,zeros)],[(1,zeros),(2,zeros)]))
        np.savez(candidate/'CANDIDATES_NOT_FOR_EXECUTION.npz',spline_degree=5,
                 spline_knots=spline.t,spline_coefficients=spline.c,duration_s=5.,
                 return_q=np.array([q[-1],q[0]]),return_time_s=np.array([0.,12.]))
        cfg=dict(m_total=0.,F_T_EE=np.eye(4).ravel(order='F').tolist(),
                 F_x_Ctotal=[0.]*3,I_total=[0.]*9)
        with patch.object(exporter,'review_session',return_value=({'robot_config':cfg,'duration_robot_s':5.},None,None)):
            exporter.export(candidate,self.root/'fake-session',self.root/'plans')
        return self.root/'plans'

    def validate_cpp(self, plan, output):
        binary=os.environ.get('FR3_OFFLINE_EXECUTOR')
        if not binary:self.skipTest('set FR3_OFFLINE_EXECUTOR for compiled parser validation')
        return subprocess.run([binary,'--offline',str(plan),str(output)],capture_output=True,text=True,timeout=10)

    def test_all_seven_joints_preserved_and_cpp_parses(self):
        plans=self.export()
        text=(plans/'replay.plan').read_text()
        self.assertTrue(text.startswith('FR3_TAUGHT_ALL_JOINTS_V2'))
        # The J1 polynomial is not silently replaced by a constant.
        lines=text.splitlines();j1=np.array([float(x) for x in lines[6].split()])
        self.assertGreater(np.max(np.abs(j1[:5])),0)
        for phase in ('return','replay'):
            run=self.validate_cpp(plans/(phase+'.plan'),self.root/(phase+'_check'))
            self.assertEqual(run.returncode,0,run.stdout+run.stderr)
            diag=(self.root/(phase+'_check')/'diagnostic_states.txt').read_text()
            self.assertIn('first_callback=not_observed',diag)

    def test_fixed_j1_legacy_remains_explicit(self):
        plans=self.export(-.1)
        self.assertTrue((plans/'replay.plan').read_text().startswith('FR3_TAUGHT_FIXED_J1_V1'))
        run=self.validate_cpp(plans/'replay.plan',self.root/'checked')
        self.assertEqual(run.returncode,0,run.stdout+run.stderr)

    def test_relabel_moving_j1_as_held_rejected(self):
        plans=self.export();p=plans/'replay.plan'
        p.write_text(p.read_text().replace('FR3_TAUGHT_ALL_JOINTS_V2','FR3_TAUGHT_FIXED_J1_V1'))
        run=self.validate_cpp(p,self.root/'rejected')
        self.assertNotEqual(run.returncode,0)
        self.assertIn('J1 must not move',run.stderr)

    def test_nonintegral_duration_has_no_short_terminal_interval(self):
        for duration in (59.822, 59.709, 5., 5.001):
            knots=interpolation_knots(duration)
            steps=np.diff(knots)
            self.assertEqual(knots[0],0.)
            self.assertEqual(knots[-1],duration)
            self.assertLessEqual(steps.max(),.02000000001)
            self.assertGreater(steps.min(),.019)
            self.assertLess(np.ptp(steps),1e-12)
        for duration in (0.,-1.,float('nan')):
            with self.assertRaises(ValueError):interpolation_knots(duration)


if __name__=='__main__':unittest.main()
