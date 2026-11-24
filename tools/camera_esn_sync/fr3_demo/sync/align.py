"""Offline timestamp matching; refuses different boots/clocks and malformed logs."""
import argparse
from bisect import bisect_right, bisect_left
import csv
import json
from pathlib import Path

def read_rows(path):
    with Path(path).open() as f:return list(csv.DictReader(f))

def strictly_increasing(values):
    return bool(values) and all(b>a for a,b in zip(values,values[1:]))

def match(camera, policy, output):
    camera,policy,output=map(Path,(camera,policy,output))
    cc=json.loads((camera/'clock.json').read_text());pc=json.loads((policy/'policy_clock.json').read_text())
    if cc['clock']!='CLOCK_MONOTONIC' or pc['clock']!=cc['clock'] or pc['boot_id']!=cc['boot_id']:
        raise ValueError('Clock domain or host boot differs; no matching performed')
    status=json.loads((policy/'policy_sync_status.json').read_text())
    if status['overflow']:raise ValueError('Policy diagnostic buffer overflow')
    c=read_rows(camera/'camera_frames.csv');p=read_rows(policy/'policy_sync.csv')
    ct=[int(x['capture_monotonic_ns']) for x in c]
    pt=[int(x['callback_monotonic_ns']) for x in p]
    ready=[int(x['state_ready_monotonic_ns']) for x in p]
    seq=[int(x['sequence']) for x in c]
    if not all(map(strictly_increasing,(ct,pt,ready,seq))):raise ValueError('Empty or nonmonotonic timestamps/sequence')
    if status['rows']!=len(p):raise ValueError('Policy row count mismatch')
    if any(b<a for a,b in zip(pt,ready)):raise ValueError('Policy ready timestamp precedes callback')
    if any((int(x['flags'])&0xe000)!=0x2000 for x in c):raise ValueError('Camera timestamps are not monotonic')
    if min(ct[-1],pt[-1])<max(ct[0],pt[0]):raise ValueError('No overlapping camera/policy time interval')
    pairs=[];deltas=[]
    for frame,stamp in zip(c,ct):
        insertion=bisect_left(pt,stamp)
        candidates=[i for i in (insertion-1,insertion) if 0<=i<len(pt)]
        nearest=min(candidates,key=lambda i:abs(pt[i]-stamp))
        delta=(pt[nearest]-stamp)/1e6
        inside=pt[0]<=stamp<=pt[-1]
        gap_ok=not(0<insertion<len(pt) and pt[insertion]-pt[insertion-1]>80000000)
        valid=inside and gap_ok and abs(delta)<=25
        causal=bisect_right(ready,stamp)-1
        pairs.append({'frame':frame['frame'],'capture_monotonic_ns':stamp,
            'nearest_policy_sample':p[nearest]['sample'],'policy_minus_camera_ms':delta,
            'valid':int(valid),'latest_state_ready_by_capture':p[causal]['sample'] if causal>=0 else '',
            'causal_state_age_ms':(stamp-ready[causal])/1e6 if causal>=0 else ''})
        if valid:deltas.append(abs(delta))
    if not deltas:raise ValueError('No valid frame matches within 25 ms')
    output.mkdir(parents=True,exist_ok=False)
    with (output/'frame_policy_pairs.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=pairs[0].keys());w.writeheader();w.writerows(pairs)
    result={'camera_frames':len(c),'policy_samples':len(p),'valid_matches':len(deltas),
      'unmatched_or_invalid_frames':len(c)-len(deltas),'max_timestamp_pair_delta_ms':max(deltas),
      'mean_timestamp_pair_delta_ms':sum(deltas)/len(deltas),
      'camera_sequence_gaps':sum(b-a-1 for a,b in zip(seq,seq[1:])),
      'max_policy_gap_ms':max((b-a)/1e6 for a,b in zip(pt,pt[1:])) if len(pt)>1 else 0,
      'camera_timestamp_sources':sorted(set(x['timestamp_source'] for x in c)),
      'synthetic_policy_data':pc.get('synthetic_test_data',False),
      'measured_physical_exposure_to_robot_sensor_error':False,
      'note':'Pair deltas are sampling alignment, not a calibrated physical latency bound; callback time is host receipt, not sensor acquisition.'}
    (output/'alignment.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('--camera',required=True);a.add_argument('--policy',required=True);a.add_argument('--output',required=True)
    args=a.parse_args();print(json.dumps(match(args.camera,args.policy,args.output),indent=2))
