#!/usr/bin/python3
"""Fixed demo entry. Only --start launches hardware; --check stays entirely local."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

HERE = Path(__file__).resolve().parent
DEBUG_KEYS = ('ESN_OFF', 'DQ_TEST', 'DQ_DEBUG', 'VEL_LIM', 'ACC_LIM',
              'Y_MAX', 'R_MAX', 'F_DEAD', 'M_DEAD', 'FR3_ESN_DIAG_DIR')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked_profile(path=HERE / 'profile.json'):
    p = json.loads(Path(path).read_text())
    required = {'schema': 'FR3_DEMO_FIXED_ESN_V1', 'robot_ip': '172.16.0.2',
                'nominal_speed_rad_s': '0.105', 'collision_profile': 'normal',
                'cpu': '21', 'esn_enabled': True, 'continuous_cycle': True,
                'automatic_approach': False, 'automatic_recovery_or_retry': False,
                'joint_stiffness_during_motion_Nm_rad': [300] * 7}
    for key, value in required.items():
        if p.get(key) != value:
            raise RuntimeError('Fixed demo profile mismatch: ' + key)
    if str(Path.home()) != p['expected_home']:
        raise RuntimeError('Different HOME would select different robot poses')
    if digest(p['binary']) != p['binary_sha256']:
        raise RuntimeError('Controller changed; review it before starting')
    for name in ('home', 'pos2'):
        pose = p['poses'][name]
        expected = Path(p['expected_home']) / 'franka-poses' / (name + '.json')
        if Path(pose['path']) != expected or digest(expected) != pose['sha256']:
            raise RuntimeError('Recorded pose changed: ' + name)
    return p


def command(p):
    # The controller itself performs fresh state/limit checks and calibration.
    return ['taskset', '-c', p['cpu'], 'stdbuf', '-oL', '-eL', p['binary'],
            p['robot_ip'], 'home', 'pos2', p['nominal_speed_rad_s'], 'normal']


def child_environment(diag, esn_enabled=True):
    env = os.environ.copy()
    for key in DEBUG_KEYS:
        env.pop(key, None)
    env['FR3_ESN_DIAG_DIR'] = str(diag)
    if not esn_enabled:
        env['ESN_OFF'] = '1'
    return env


def process_identity(pid):
    root = Path('/proc') / str(pid)
    raw = (root / 'stat').read_text()
    fields = raw[raw.rfind(')') + 2:].split()
    args = [x.decode(errors='replace') for x in
            (root / 'cmdline').read_bytes().split(b'\0') if x]
    return {'pid': int(pid), 'start_ticks': fields[19], 'argv': args}


def active_controls(review_root):
    found = []
    wrappers = {'run_once.py', 'run_cycle.py', 'run_esn_once.py',
                'run_esn_contact_once.py', 'run_esn_contact_once_02.py',
                'run_esn_contact_once_03.py', 'run_esn_contact_once_04.py'}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            identity = process_identity(entry.name)
            args = identity['argv']
            if any((Path(a).name.startswith('esn_yield') or
                    Path(a).name in wrappers or
                    Path(a).name in {'taught_path', 'waypoints'}) for a in args):
                found.append(identity)
        except (OSError, ValueError, IndexError):
            continue
    return found


def save(path, value):
    path = Path(path)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    tmp.replace(path)


def verify_stop_target(active, identity, binary):
    if active.get('phase') != 'running':
        raise RuntimeError('No demo marked running')
    if identity['pid'] != active['child']['pid'] or identity['start_ticks'] != active['child']['start_ticks']:
        raise RuntimeError('Process identity changed; no signal sent')
    if binary not in identity['argv']:
        raise RuntimeError('Controller command does not match; no signal sent')


def stop():
    path = HERE / 'active.json'
    if not path.exists():
        print('No demo started through this entry.')
        return 0
    active = json.loads(path.read_text())
    if active.get('phase') == 'starting':
        (Path(active['run']) / 'STOP').touch(exist_ok=True)
        print('Start cancellation requested.')
        return 0
    if active.get('phase') != 'running':
        print('Demo already ended.')
        return 0
    pid = active['child']['pid']
    # pidfd prevents a reused PID receiving a stop signal after identity checking.
    try:
        fd = os.pidfd_open(pid)
    except ProcessLookupError:
        print('Controller has already exited.')
        return 0
    try:
        verify_stop_target(active, process_identity(pid), active['binary'])
        signal.pidfd_send_signal(fd, signal.SIGTERM)
    finally:
        os.close(fd)
    print('Stop requested; controller will abort without recovery or retry.')
    return 0


def start(p, esn_enabled=True):
    p = dict(p)
    p['esn_enabled'] = bool(esn_enabled)
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        raise RuntimeError('Use /usr/bin/python3 for this entry; verified process-stop interfaces required')
    with (HERE / '.start.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Demo already starting/running; no second launch')
        conflicts = active_controls(p['review_root'])
        if conflicts:
            raise RuntimeError('Existing controller found: ' + str(conflicts))
        root = Path(p['run_root'])
        root.mkdir(parents=True, exist_ok=True)
        run = Path(tempfile.mkdtemp(prefix='cycle_', dir=root))
        diag = run / 'execute_diagnostics'
        diag.mkdir()
        cmd = command(p)
        save(run / 'execute_command.json', {'argv': cmd, 'profile': p,
                                           'binary_sha256': p['binary_sha256'],
                                           'single_control_session': True,
                                           'separate_preview_process': False})
        policy_csv = Path('/tmp/esn_shadow_obs.csv')
        if policy_csv.is_file():
            shutil.copyfile(policy_csv, run / 'previous_csv_backup.csv')
        child = None
        stop_requested = False
        ready = False
        ready_seconds = None
        code = 125
        started = time.monotonic()
        epoch = time.time()
        state = {'phase': 'starting', 'run': str(run), 'binary': p['binary']}
        save(HERE / 'active.json', state)

        def request_stop(signum, frame):
            nonlocal stop_requested
            stop_requested = True
            if child is not None and child.poll() is None:
                child.terminate()

        old_handlers = {sig: signal.signal(sig, request_stop)
                        for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            print('Starting one ' + ('ESN' if esn_enabled else 'BASELINE (ESN OFF)') + ' cycle at 0.105 rad/s. Hands off until READY.', flush=True)
            print('Run: ' + str(run), flush=True)
            with (run / 'execute.log').open('x') as log:
                if stop_requested or (run / 'STOP').exists():
                    raise RuntimeError('Start cancelled before controller launch')
                child = subprocess.Popen(cmd, env=child_environment(diag, esn_enabled),
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                         text=True, bufsize=1)
                if stop_requested or (run / 'STOP').exists():
                    child.terminate()
                identity = process_identity(child.pid)
                state.update(phase='running', child=identity)
                save(HERE / 'active.json', state)
                save(run / 'execute_process.json', state)
                for line in child.stdout:
                    if (run / 'STOP').exists() and child.poll() is None:
                        child.terminate()
                    log.write(line)
                    log.flush()
                    print(line, end='', flush=True)
                    if not ready and re.match(r'^\s*\d+\.\d+\s*\|', line):
                        ready = True
                        ready_seconds = time.monotonic() - started
                        print('READY: 基线结束，控制已运行，可以轻推末端。' if esn_enabled else 'READY: ESN 已旁路，纯轨迹往返已运行；请勿施加外力。', flush=True)
                code = child.wait()
        finally:
            if child is not None and child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
            for sig, handler in old_handlers.items():
                signal.signal(sig, handler)
            result = {'return_code': code, 'wall_seconds': time.monotonic() - started,
                      'ready_observed': ready, 'ready_seconds_after_local_invocation': ready_seconds,
                      'automatic_recovery_or_retry': False,
                      'final_robot_state_read': False}
            save(run / 'execute_result.json', result)
            state.update(phase='finished', result=result)
            save(HERE / 'active.json', state)
            if policy_csv.is_file() and policy_csv.stat().st_mtime >= epoch:
                shutil.copyfile(policy_csv, run / 'esn_observations.csv')
        print('Demo ended. Exit code: ' + str(code), flush=True)
        return code


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--start', action='store_true', help='Start one real robot ESN cycle')
    group.add_argument('--check', action='store_true', help='Local files/processes only; no robot connection')
    group.add_argument('--stop', action='store_true', help='Signal only this launcher\'s verified controller')
    args = parser.parse_args(argv)
    if not any((args.start, args.check, args.stop)):
        parser.print_help()
        return 0
    try:
        if args.stop:
            return stop()
        p = checked_profile()
        if args.check:
            conflicts = active_controls(p['review_root'])
            stop_supported = hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal')
            print(json.dumps({'local_files_verified': True, 'robot_connected': False,
                              'active_known_controllers': conflicts,
                              'verified_stop_interface_available': stop_supported,
                              'command': command(p)},
                             ensure_ascii=False, indent=2))
            return 1 if conflicts or not stop_supported else 0
        return start(p)
    except Exception as error:
        print('REFUSED/FAILED: ' + str(error), file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
