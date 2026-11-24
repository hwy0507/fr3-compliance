#!/usr/bin/python3
"""Separate opt-in entry: --check is local; --start performs one REAL ESN cycle.

The ordinary demo entry/profile are not changed. No automatic start or retry.
"""
import argparse
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
ARTIFACT=ROOT/'artifacts/fr3_camera_sync_20261124'
DEVICE='/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435i_Intel_R__RealSense_TM__Depth_Camera_435i_263143064395-video-index0'
spec=importlib.util.spec_from_file_location('demo_launcher',HERE.parent/'start_esn.py')
demo=importlib.util.module_from_spec(spec);spec.loader.exec_module(demo)

def profile():
    build=json.loads((ARTIFACT/'build.json').read_text())
    p=demo.checked_profile(ARTIFACT/'candidate_profile.json')
    if demo.digest(HERE/'policy_sync.hpp')!=build['header_sha256']:
        raise RuntimeError('Sync header changed since candidate compilation')
    if p['binary_sha256']!=build['candidate_binary_sha256']:
        raise RuntimeError('Candidate binary mismatch')
    return p

def start(p, esn_enabled=True):
    with (HERE/'.sync.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if demo.active_controls(p['review_root']):
            raise RuntimeError('Existing controller; no new launch')
        parent=ARTIFACT/'sessions';parent.mkdir(exist_ok=True)
        run=Path(tempfile.mkdtemp(prefix='session_',dir=parent))
        camera_dir=run/'camera'
        # Separate low-priority camera process, kept off the control core.
        cpus=sorted(os.sched_getaffinity(0)-{int(p['cpu'])})[:2]
        if not cpus:raise RuntimeError('No non-control CPU available for camera')
        command=['taskset','-c',','.join(map(str,cpus)),'nice','-n','5',
                 str(ARTIFACT/'capture_v4l2'),DEVICE,str(camera_dir),'115']
        child=None;old={};code=125
        try:
            def stop_pending(signum,frame):raise KeyboardInterrupt('Sync start/recording interrupted')
            old={s:signal.signal(s,stop_pending) for s in (signal.SIGINT,signal.SIGTERM)}
            with (run/'camera.log').open('x') as log:
                child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+8
                while not (camera_dir/'READY').exists():
                    if child.poll() is not None:raise RuntimeError('Camera failed; robot has NOT been launched. See '+str(run/'camera.log'))
                    if time.monotonic()>deadline:raise RuntimeError('Camera readiness timeout; robot NOT launched')
                    time.sleep(.05)
                if child.poll() is not None:raise RuntimeError('Camera exited before controller launch')
                print('Camera timestamps ready. Starting authorized cycle; hands off until robot READY.',flush=True)
                p=dict(p);p['run_root']=str(run)
                demo.save(run/'session.json',{'camera':str(camera_dir),'camera_command':command,
                  'candidate_binary':p['binary'],'esn_enabled':bool(esn_enabled),'live_dashboard_implemented':False})
                # Same reviewed guards, motion settings, signal handler and stop identity checks.
                code=demo.start(p, esn_enabled=esn_enabled)
        finally:
            if child is not None and child.poll() is None:
                child.terminate()
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:child.kill();child.wait()
            for s,handler in old.items():signal.signal(s,handler)
        policies=list(run.glob('cycle_*/execute_diagnostics/policy_sync.csv'))
        if len(policies)==1 and (camera_dir/'complete.json').exists():
            from align import match
            try:
                result=match(camera_dir,policies[0].parent,run/'alignment')
                print(json.dumps(result,indent=2),flush=True)
            except Exception as e:
                demo.save(run/'alignment_failed.json',{'error':str(e)})
                print('Synchronization invalid: '+str(e),flush=True)
        else:
            print('Recording incomplete; do not claim synchronized results.',flush=True)
        print('Sync session: '+str(run),flush=True)
        return code

def main():
    a=argparse.ArgumentParser(description=__doc__)
    g=a.add_mutually_exclusive_group();g.add_argument('--check',action='store_true');g.add_argument('--start',action='store_true');g.add_argument('--stop',action='store_true')
    a.add_argument('--no-esn',action='store_true',help='Explicit baseline: bypass ESN slowdown/yield; inference is logged only')
    args=a.parse_args()
    if args.stop:return demo.stop()
    if not(args.check or args.start):a.print_help();return 0
    p=profile()
    if args.check:
        print(json.dumps({'candidate_files_verified':True,'robot_connected':False,
         'camera_opened':False,'default_demo_profile_unchanged':True,
         'esn_enabled':not args.no_esn,
         'live_dashboard_implemented':False,'command':demo.command(p)},indent=2));return 0
    return start(p, esn_enabled=not args.no_esn)

if __name__=='__main__':
    try:raise SystemExit(main())
    except KeyboardInterrupt:raise SystemExit(130)
