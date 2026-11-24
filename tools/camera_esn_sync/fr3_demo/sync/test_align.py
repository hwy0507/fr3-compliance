import csv
import json
from pathlib import Path
import tempfile
import unittest
from align import match

def write_csv(path,rows):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)

class AlignmentTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.c=self.root/'camera';self.p=self.root/'policy';self.out=self.root/'result'
        self.c.mkdir();self.p.mkdir()
        meta={'clock':'CLOCK_MONOTONIC','boot_id':'SYNTHETIC_TEST_ONLY'}
        (self.c/'clock.json').write_text(json.dumps(meta));(self.p/'policy_clock.json').write_text(json.dumps(meta))
        start=100000000000000
        self.cr=[{'frame':i,'sequence':i,'capture_monotonic_ns':start+round(i*1e9/30), 'flags':0x12000,'timestamp_source':'SOE'} for i in range(31)]
        self.pr=[{'sample':i,'callback_monotonic_ns':start+i*40000000,'state_ready_monotonic_ns':start+i*40000000+100000} for i in range(26)]
        self.save()
    def save(self,overflow=False):
        write_csv(self.c/'camera_frames.csv',self.cr);write_csv(self.p/'policy_sync.csv',self.pr)
        (self.p/'policy_sync_status.json').write_text(json.dumps({'rows':len(self.pr),'overflow':overflow}))
    def tearDown(self):self.tmp.cleanup()
    def test_30fps_to_25hz(self):
        r=match(self.c,self.p,self.out)
        self.assertEqual(r['valid_matches'],31);self.assertLessEqual(r['max_timestamp_pair_delta_ms'],20.000001)
        with (self.out/'frame_policy_pairs.csv').open() as f:
            rows=list(csv.DictReader(f))
        self.assertEqual(rows[0]['latest_state_ready_by_capture'],'')
        self.assertEqual(rows[1]['latest_state_ready_by_capture'],'0')
    def test_boot_mismatch(self):
        (self.p/'policy_clock.json').write_text(json.dumps({'clock':'CLOCK_MONOTONIC','boot_id':'different'}))
        with self.assertRaisesRegex(ValueError,'boot'):match(self.c,self.p,self.out)
    def test_wrong_camera_clock(self):
        self.cr[5]['flags']=0;self.save()
        with self.assertRaisesRegex(ValueError,'not monotonic'):match(self.c,self.p,self.out)
    def test_timestamp_regression(self):
        self.pr[5]['callback_monotonic_ns']=self.pr[4]['callback_monotonic_ns'];self.save()
        with self.assertRaisesRegex(ValueError,'nonmonotonic'):match(self.c,self.p,self.out)
    def test_no_overlap(self):
        for row in self.cr:row['capture_monotonic_ns']+=2000000000
        self.save()
        with self.assertRaisesRegex(ValueError,'overlapping'):match(self.c,self.p,self.out)
    def test_detect_dropped_camera_frames(self):
        del self.cr[8];self.save();self.assertEqual(match(self.c,self.p,self.out)['camera_sequence_gaps'],1)
    def test_missing_policy_interval(self):
        del self.pr[5:10];self.save();r=match(self.c,self.p,self.out)
        self.assertGreater(r['unmatched_or_invalid_frames'],0);self.assertEqual(r['max_policy_gap_ms'],240)
    def test_overflow_rejected(self):
        self.save(True)
        with self.assertRaisesRegex(ValueError,'overflow'):match(self.c,self.p,self.out)

if __name__=='__main__':unittest.main()
