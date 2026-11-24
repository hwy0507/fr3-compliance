import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import start_esn as demo


class DemoTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.p = json.loads((demo.HERE / 'profile.example.json').read_text())
        self.p['expected_home'] = str(self.root)
        binary = self.root / 'synthetic_controller_never_execute'
        binary.write_bytes(b'NOT EXECUTABLE; OFFLINE TEST ONLY')
        self.p['binary'] = str(binary)
        self.p['binary_sha256'] = demo.digest(binary)
        poses = self.root / 'franka-poses'
        poses.mkdir()
        for name in ('home', 'pos2'):
            file = poses / (name + '.json')
            file.write_text(json.dumps({'q': [0] * 7, 'offline_test_only': True}))
            self.p['poses'][name] = {'path': str(file), 'sha256': demo.digest(file)}
        self.profile = self.root / 'profile.json'
        self.profile.write_text(json.dumps(self.p))

    def tearDown(self):
        self.tmp.cleanup()

    def checked(self):
        with patch.object(Path, 'home', return_value=self.root):
            return demo.checked_profile(self.profile)

    def test_fixed_command_no_high_no_preview_no_retry(self):
        cmd = demo.command(self.checked())
        self.assertEqual(cmd[-5:], ['172.16.0.2', 'home', 'pos2', '0.105', 'normal'])

    def test_tampered_controller_refused(self):
        Path(self.p['binary']).write_bytes(b'CHANGED')
        with self.assertRaisesRegex(RuntimeError, 'Controller changed'):
            self.checked()

    def test_tampered_pose_refused(self):
        Path(self.p['poses']['home']['path']).write_text('{"q":[1]}')
        with self.assertRaisesRegex(RuntimeError, 'Recorded pose changed'):
            self.checked()

    def test_high_profile_refused(self):
        self.p['collision_profile'] = 'high'
        self.profile.write_text(json.dumps(self.p))
        with self.assertRaisesRegex(RuntimeError, 'profile mismatch'):
            self.checked()

    def test_different_home_refused(self):
        with patch.object(Path, 'home', return_value=self.root / 'other'):
            with self.assertRaisesRegex(RuntimeError, 'Different HOME'):
                demo.checked_profile(self.profile)

    def test_debug_overrides_removed(self):
        overrides = {key: '1' for key in demo.DEBUG_KEYS}
        with patch.dict(os.environ, overrides):
            env = demo.child_environment(self.root)
        for key in demo.DEBUG_KEYS:
            if key != 'FR3_ESN_DIAG_DIR':
                self.assertNotIn(key, env)
        self.assertEqual(env['FR3_ESN_DIAG_DIR'], str(self.root))

    def test_no_esn_explicit_bypass(self):
        with patch.dict(os.environ, {'ESN_OFF': '0', 'DQ_TEST': '1'}):
            env = demo.child_environment(self.root, esn_enabled=False)
        self.assertEqual(env['ESN_OFF'], '1')
        self.assertNotIn('DQ_TEST', env)
        self.assertNotIn('ESN_OFF', demo.child_environment(self.root, esn_enabled=True))

    def test_old_installation_controller_detected(self):
        identity = {'pid': 87654321, 'start_ticks': '100',
                    'argv': ['/different/old/install/esn_yield_roundtrip_diag_20261124']}
        with patch.object(Path, 'iterdir', return_value=[Path('/proc/87654321')]), \
             patch.object(demo, 'process_identity', return_value=identity):
            self.assertEqual(demo.active_controls('/new/package'), [identity])

    def test_reused_pid_refused(self):
        active = {'phase': 'running', 'child': {'pid': 123, 'start_ticks': 'old'}}
        identity = {'pid': 123, 'start_ticks': 'new', 'argv': [self.p['binary']]}
        with self.assertRaisesRegex(RuntimeError, 'identity changed'):
            demo.verify_stop_target(active, identity, self.p['binary'])

    def test_different_process_command_refused(self):
        active = {'phase': 'running', 'child': {'pid': 123, 'start_ticks': 'same'}}
        identity = {'pid': 123, 'start_ticks': 'same', 'argv': ['/bin/sleep', '10']}
        with self.assertRaisesRegex(RuntimeError, 'command does not match'):
            demo.verify_stop_target(active, identity, self.p['binary'])

    def test_known_good_stop_identity(self):
        active = {'phase': 'running', 'child': {'pid': 123, 'start_ticks': 'same'}}
        identity = {'pid': 123, 'start_ticks': 'same', 'argv': [self.p['binary']]}
        demo.verify_stop_target(active, identity, self.p['binary'])

    def test_local_check_never_launches(self):
        with patch.object(demo, 'checked_profile', return_value=self.checked()), \
             patch.object(demo, 'active_controls', return_value=[]), \
             patch.object(demo.subprocess, 'Popen', side_effect=AssertionError('NO HARDWARE')):
            self.assertEqual(demo.main(['--check']), 0)

    def test_existing_controller_blocks_start(self):
        with patch.object(demo, 'HERE', self.root), \
             patch.object(demo, 'active_controls', return_value=[{'pid': 123}]), \
             patch.object(demo.subprocess, 'Popen', side_effect=AssertionError('NO HARDWARE')):
            with self.assertRaisesRegex(RuntimeError, 'Existing controller'):
                demo.start(self.p)

    def test_concurrent_launch_blocked(self):
        import fcntl
        with (self.root / '.start.lock').open('a') as held:
            fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with patch.object(demo, 'HERE', self.root), \
                 patch.object(demo.subprocess, 'Popen', side_effect=AssertionError('NO HARDWARE')):
                with self.assertRaisesRegex(RuntimeError, 'already starting/running'):
                    demo.start(self.p)

    def test_no_flag_does_not_launch(self):
        with patch.object(demo.subprocess, 'Popen', side_effect=AssertionError('NO HARDWARE')):
            self.assertEqual(demo.main([]), 0)

    def test_cancel_starting_creates_stop_marker_without_signal(self):
        run = self.root / 'synthetic_run'
        run.mkdir()
        (self.root / 'active.json').write_text(json.dumps({'phase': 'starting', 'run': str(run)}))
        with patch.object(demo, 'HERE', self.root), \
             patch.object(demo.os, 'pidfd_open', side_effect=AssertionError('NO SIGNAL'), create=True):
            self.assertEqual(demo.stop(), 0)
        self.assertTrue((run / 'STOP').exists())


if __name__ == '__main__':
    unittest.main()
