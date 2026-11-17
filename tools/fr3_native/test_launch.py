"""Synthetic source/authorization regressions; no hardware or subprocess launch."""
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import launch


class LaunchTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.config = {'schema': 'FR3_NATIVE_LAUNCH_V1', 'proposal': 'FR3_NATIVE_CARTESIAN_HOLD_V1',
                       'review': dict.fromkeys(launch.FLAGS, True), 'robot_ip': '192.0.2.1',
                       'mode': 'hold_only', 'contact_authorized': False,
                       'issued_unix': 1000, 'expires_unix': 1300, 'operator_authorization': 'SYNTHETIC ONLY',
                       'cpu': min(os.sched_getaffinity(0)), 'previous_stiffness': [1000]*3+[100]*3, 'sources': {}}
        for role in ('stiffness', 'collision'):
            record = {'kind': 'vendor_current_readback', 'scope': 'current_fci', 'robot_ip': '192.0.2.1',
                      'status': 'verified_current', 'boot_context_confirmed': True,
                      'no_intervening_changes_confirmed': True, 'verified_unix': 1000,
                      'provenance': 'SYNTHETIC ONLY: never a live source', 'contact_envelope_reviewed': True,
                      'values': self.config['previous_stiffness'] if role == 'stiffness' else
                      {key: [2 if key.startswith('lower') else 4]*n for key, n in launch.THRESHOLDS.items()}}
            self.write_source(role, record)

    def write_source(self, role, record):
        raw = json.dumps(record).encode(); path = self.base / f'{role}.json'; path.write_bytes(raw)
        self.config['sources'][role] = {'path': path.name, 'sha256': hashlib.sha256(raw).hexdigest()}

    def source(self, role):
        return launch.load_json((self.base / f'{role}.json').read_bytes())

    def validate(self):
        return launch.validate(self.config, self.base, now=1100)

    def test_synthetic_complete_review_passes_local_validation(self):
        self.assertEqual(self.validate()['mode'], 'hold_only')

    def test_each_review_flag_is_required_and_boolean(self):
        for flag in launch.FLAGS:
            for value in (False, 1, 'true'):
                with self.subTest(flag=flag, value=value):
                    self.config['review'][flag] = value
                    with self.assertRaises(ValueError): self.validate()
                    self.config['review'][flag] = True

    def test_unknown_original_refused(self):
        self.config['previous_stiffness'] = None
        with self.assertRaises(ValueError): self.validate()

    def test_source_hash_change_refused(self):
        (self.base / 'collision.json').write_text('{}')
        with self.assertRaises(ValueError): self.validate()

    def test_historical_or_defaults_not_current_sources(self):
        for kind in ('historical_task', 'example_default', 'unknown'):
            record = self.source('stiffness'); record['kind'] = kind; self.write_source('stiffness', record)
            with self.assertRaises(ValueError): self.validate()

    def test_other_robot_scope_boot_or_freshness_refused(self):
        for key, value in [('robot_ip', '192.0.2.2'), ('scope', 'desk_task'),
                           ('boot_context_confirmed', False), ('no_intervening_changes_confirmed', False),
                           ('verified_unix', 0), ('verified_unix', 1200), ('provenance', '')]:
            record = self.source('collision'); original = copy.deepcopy(record)
            record[key] = value; self.write_source('collision', record)
            with self.assertRaises(ValueError): self.validate()
            self.write_source('collision', original)

    def test_all_eight_collision_arrays_and_order_required(self):
        record = self.source('collision'); record['values'].pop('upper_force_nominal')
        self.write_source('collision', record)
        with self.assertRaises(ValueError): self.validate()

    def test_stiffness_must_match_source(self):
        self.config['previous_stiffness'][0] = 900
        with self.assertRaises(ValueError): self.validate()

    def test_expiry_and_lifetime_refused(self):
        for issued, expires in [(1200, 1300), (1000, 1099), (1000, 1105), (1000, 2000)]:
            self.config.update(issued_unix=issued, expires_unix=expires)
            with self.assertRaises(ValueError): self.validate()

    def test_light_push_requires_separate_authority(self):
        self.config['mode'] = 'light_push'
        with self.assertRaises(ValueError): self.validate()
        self.config['contact_authorized'] = True
        with self.assertRaises(ValueError): self.validate()
        self.config['contact_point_and_force_measurement'] = 'Synthetic end-effector contact and force scale'
        self.assertEqual(self.validate()['mode'], 'light_push')

    def test_duplicate_keys_nan_and_boolean_numbers_refused(self):
        for text in ('{"a":1,"a":2}', '{"value":NaN}'):
            with self.assertRaises(ValueError): launch.load_json(text)
        with self.assertRaises(ValueError): launch.numbers([True]*6, 6)

    def test_shipped_template_blocks_before_subprocess(self):
        template = Path(__file__).with_name('REVIEW_TEMPLATE.json')
        with patch('sys.argv', ['launch', 'execute', str(template), str(self.base/'run'), '--binary', '/does/not/exist']), \
             patch.object(launch.os, 'execv') as child:
            with self.assertRaises(ValueError): launch.main()
            child.assert_not_called()
        self.assertFalse((self.base/'run').exists())

    def test_noninteractive_execute_cannot_start_or_create_run(self):
        path = self.base/'review.json'; path.write_text(json.dumps(self.config))
        with patch('sys.argv', ['launch', 'execute', str(path), str(self.base/'run'), '--binary', '/missing']), \
             patch.object(launch.time, 'time', return_value=1100), \
             patch.object(launch.sys.stdin, 'isatty', return_value=False), \
             patch.object(launch.os, 'execv') as entry:
            with self.assertRaises(ValueError): launch.main()
            entry.assert_not_called()
        self.assertFalse((self.base/'run').exists())

    def test_terminal_handoff_uses_exec_not_a_killable_child(self):
        path = self.base/'review.json'; path.write_text(json.dumps(self.config))
        binary = self.base/'fake_binary'; binary.write_text('SYNTHETIC NONEXECUTABLE')
        run = self.base/'new_run'
        with patch('sys.argv', ['launch', 'execute', str(path), str(run), '--binary', str(binary)]), \
             patch.object(launch.time, 'time', return_value=1100), \
             patch.object(launch.sys.stdin, 'isatty', return_value=True), \
             patch.object(launch.sys.stdout, 'isatty', return_value=True), \
             patch.object(launch.os, 'execv') as entry:
            launch.main(); entry.assert_called_once()
            self.assertEqual(entry.call_args.args[0], str(binary))
            self.assertEqual(entry.call_args.args[1][1], '--execute-reviewed')
            with self.assertRaises(FileExistsError): launch.main()
        self.assertEqual((run/'review_snapshot.json').read_text(), path.read_text())
        self.assertFalse((run/'LIVE_STARTED').exists())

    def test_stop_latch_is_idempotent_and_local(self):
        (self.base/'review_snapshot.json').write_text(json.dumps(self.config))
        with patch('sys.argv', ['launch', 'stop', str(self.base)]), patch.object(launch.os, 'execv') as child:
            launch.main(); launch.main(); child.assert_not_called()
        self.assertTrue((self.base/'STOP').exists())

    def test_retain_profile_does_not_need_or_invent_original_values(self):
        self.config.update(schema='FR3_NATIVE_RETAIN_LAUNCH_V1', end_policy='retain_trial',
            collision_policy='preserve_unchanged', previous_stiffness=None, numeric_collision_values_verified=False,
            review=dict.fromkeys(launch.RETAIN_FLAGS, True))
        sample={'robot_mode':'RobotMode.Idle','current_errors':'[]',
                **{k:[0]*n for k,n in [('joint_contact',7),('cartesian_contact',6),('joint_collision',7),('cartesian_collision',6)]}}
        raw=json.dumps({'complete':True,'samples':[sample]}).encode(); path=self.base/'readonly.json';path.write_bytes(raw)
        self.config['readonly_preflight']={'path':path.name,'sha256':hashlib.sha256(raw).hexdigest()}
        checked=self.validate(); self.assertTrue(checked['retain']);self.assertIsNone(checked['previous'])
        self.assertTrue(launch.manifest(checked,'a'*32).startswith('FR3_NATIVE_RETAIN_V1'))
        self.config['review']['retain_trial_authorized']=False
        with self.assertRaises(ValueError):self.validate()
        self.config['review']['retain_trial_authorized']=True
        self.config['numeric_collision_values_verified']=True
        with self.assertRaises(ValueError):self.validate()
        self.config.update(numeric_collision_values_verified=False, mode='free_guiding', contact_authorized=True,
                           contact_point_and_force_measurement='Synthetic gentle horizontal guiding; raw guards unchanged')
        with self.assertRaises(ValueError):self.validate()
        self.config['free_guiding_authorized']=True
        checked=self.validate();self.assertEqual(checked['mode'],'free_guiding')
        self.assertIn('\n2\n',launch.manifest(checked,'a'*32))
        self.config['minimum_command_success_rate']=.95
        with self.assertRaises(ValueError):self.validate()
        self.config['minimum_success_95_authorized']=True
        self.assertTrue(launch.manifest(self.validate(),'a'*32).startswith('FR3_NATIVE_RETAIN_95_V1'))
        self.config['minimum_command_success_rate']=.94
        with self.assertRaises(ValueError):self.validate()
        self.config['minimum_command_success_rate']=.90
        with self.assertRaises(ValueError):self.validate()
        self.config['minimum_success_90_authorized']=True
        self.assertTrue(launch.manifest(self.validate(),'a'*32).startswith('FR3_NATIVE_RETAIN_90_V1'))
        self.config['maximum_callback_period_s']=.005
        self.config['native_gap_5ms_reviewed']=True
        self.config['operator_ended']=True
        with self.assertRaises(ValueError):self.validate()
        self.config['operator_ended_authorized']=True
        self.assertTrue(launch.manifest(self.validate(),'a'*32).startswith('FR3_NATIVE_OPERATOR_V1'))
        self.config['relative_moment_limit_nm']=.5
        with self.assertRaises(ValueError):self.validate()
        self.config['guiding_moment_05_reviewed']=True
        self.assertTrue(launch.manifest(self.validate(),'a'*32).startswith('FR3_NATIVE_OPERATOR_MOMENT05_V1'))
        self.config['relative_moment_limit_nm']=.5001
        with self.assertRaises(ValueError):self.validate()
        self.config['relative_moment_limit_nm']=1.
        with self.assertRaises(ValueError):self.validate()  # 0.5 review cannot authorize 1Nm
        self.config['guiding_moment_10_reviewed']=True
        self.assertTrue(launch.manifest(self.validate(),'a'*32).startswith('FR3_NATIVE_OPERATOR_MOMENT10_V1'))
        for value in (1.0001,2.,True):
            self.config['relative_moment_limit_nm']=value
            with self.assertRaises(ValueError):self.validate()
        self.config['relative_moment_limit_nm']=1.
        self.config['mode']='hold_only'
        with self.assertRaises(ValueError):self.validate()


if __name__ == '__main__':
    unittest.main()
