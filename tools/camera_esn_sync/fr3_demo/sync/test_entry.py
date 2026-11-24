"""Launcher lifecycle tests with fake subprocesses; no camera or robot access."""
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
import start_synced as entry

class FakeCamera:
    def __init__(self,args,ready=True):
        self.args=args;self.code=None;self.terminated=False
        if ready:
            folder=Path(args[-2]);folder.mkdir();(folder/'READY').write_text('123\n')
        else:self.code=1
    def poll(self):return self.code
    def terminate(self):self.terminated=True;self.code=0
    def wait(self,timeout=None):return self.code
    def kill(self):self.code=-9

class EntryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.p={'cpu':'21','review_root':'unused','binary':'fake_never_executed'}
        self.patches=[patch.object(entry,'ARTIFACT',self.root),patch.object(entry,'HERE',self.root),
            patch.object(entry.demo,'active_controls',return_value=[]),patch.object(entry.os,'sched_getaffinity',return_value={0,1,21})]
        for p in self.patches:p.start()
    def tearDown(self):
        for p in reversed(self.patches):p.stop()
        self.tmp.cleanup()
    def test_camera_failure_blocks_motion(self):
        with patch.object(entry.subprocess,'Popen',side_effect=lambda args,**kw:FakeCamera(args,False)),patch.object(entry.demo,'start') as start:
            with self.assertRaisesRegex(RuntimeError,'NOT been launched'):entry.start(self.p)
            start.assert_not_called()
    def test_success_stops_camera_and_excludes_control_cpu(self):
        cameras=[]
        def launch(args,**kw):
            c=FakeCamera(args);cameras.append(c);return c
        with patch.object(entry.subprocess,'Popen',side_effect=launch),patch.object(entry.demo,'start',return_value=0) as start:
            self.assertEqual(entry.start(self.p),0)
            start.assert_called_once();self.assertTrue(cameras[0].terminated)
            self.assertEqual(cameras[0].args[2],'0,1')
    def test_robot_start_exception_still_releases_camera(self):
        cameras=[]
        def launch(args,**kw):
            c=FakeCamera(args);cameras.append(c);return c
        with patch.object(entry.subprocess,'Popen',side_effect=launch),patch.object(entry.demo,'start',side_effect=RuntimeError('mock failure')):
            with self.assertRaisesRegex(RuntimeError,'mock failure'):entry.start(self.p)
            self.assertTrue(cameras[0].terminated)
    def test_existing_control_blocks_camera_and_motion(self):
        with patch.object(entry.demo,'active_controls',return_value=['existing']),patch.object(entry.subprocess,'Popen') as spawn,patch.object(entry.demo,'start') as start:
            with self.assertRaisesRegex(RuntimeError,'Existing controller'):entry.start(self.p)
            spawn.assert_not_called();start.assert_not_called()
    def test_no_esn_mode_is_forwarded_explicitly(self):
        with patch.object(entry.subprocess,'Popen',side_effect=lambda args,**kw:FakeCamera(args)),patch.object(entry.demo,'start',return_value=0) as start:
            entry.start(self.p,esn_enabled=False)
            self.assertIs(start.call_args.kwargs['esn_enabled'],False)

if __name__=='__main__':unittest.main()
