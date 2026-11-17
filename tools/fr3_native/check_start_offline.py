"""Compile the real entry in both builds; execute only blocked/local/synthetic paths."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', required=True, type=Path)
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    if args.evidence.exists(): raise FileExistsError(args.evidence)
    args.evidence.parent.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    if args.sanitize: env['ASAN_OPTIONS'] = env.get('ASAN_OPTIONS', '')+':detect_leaks=0'
    commands = []
    sources = sorted(HERE.glob('*.hpp'))+sorted(HERE.glob('*.cpp'))+sorted(HERE.glob('*.py'))+[HERE/'REVIEW_TEMPLATE.json', HERE.parent/'fr3_wrench_trace.hpp']
    before = {str(x.relative_to(ROOT)): hashlib.sha256(x.read_bytes()).hexdigest() for x in sources}

    def run(argv, expected=0):
        r = subprocess.run([str(x) for x in argv], cwd=ROOT, env=env, capture_output=True, text=True, timeout=120)
        commands.append({'argv': [str(x) for x in argv], 'returncode': r.returncode,
                         'stdout': r.stdout, 'stderr': r.stderr})
        if r.returncode != expected: raise RuntimeError(r.stdout+r.stderr)
        return r

    flags = ['g++', '-std=c++17', '-O1' if args.sanitize else '-O2', '-Wall', '-Wextra', '-Werror',
             '-isystem', '/usr/include/eigen3', '-pthread']
    if args.sanitize: flags += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
    try:
        with tempfile.TemporaryDirectory(prefix='fr3-start-check-') as temporary:
            temp = Path(temporary); gate = temp/'synthetic_gate'
            run(flags+[HERE/'start_offline.cpp', '-o', gate, '-lfranka'])
            run([gate])
            guide = temp/'synthetic_guiding'
            run(flags+[HERE/'guiding_offline.cpp', '-o', guide, '-lfranka'])
            run([guide])
            trace = temp/'synthetic_trace'
            run(flags+[HERE/'trace_offline.cpp','-o',trace,'-lfranka'])
            run([trace])
            run(['python', '-m', 'unittest', 'discover', '-s', HERE, '-p', 'test_launch.py', '-v'])
            run(['python', HERE/'launch.py', 'check', HERE/'REVIEW_TEMPLATE.json'], expected=2)
            # Real SDK entry is always compiled and linked, never started with a
            # valid live manifest. The enabled build only sees malformed input.
            for build in (0, 1):
                binary = temp/f'controlled_start_{build}'
                run(flags+[f'-DFR3_NATIVE_EXECUTION_BUILD={build}', HERE/'controlled_start.cpp',
                           '-o', binary, '-lfranka'])
                bad = temp/f'bad_manifest_{build}.txt'; bad.write_text('INVALID\n')
                r = run([binary, '--execute-reviewed', bad, temp/'not_created'], expected=1)
                assert not (temp/'not_created').exists()
                if build == 0: assert 'interlock OFF' in r.stderr
                else: assert 'schema/mode' in r.stderr
            # Local-only manifest parser happy path; it never creates Robot.
            valid = temp/'local_manifest.txt'; now = int(time.time())
            valid.write_text('FR3_NATIVE_START_V1\n192.0.2.1\n0\n2\n0\n'+str(now)+'\n'+str(now+300)+'\n'+
                             'a'*32+'\n1 1 1 1 1 1 1 1 1\n1000 1000 1000 100 100 100\n'+'b'*64+'\n'+'c'*64+'\n')
            run([binary, '--validate-manifest', valid])
        after = {str(x.relative_to(ROOT)): hashlib.sha256(x.read_bytes()).hexdigest() for x in sources}
        if before != after: raise RuntimeError('sources changed during checks; evidence not reproducible')
        evidence = {'scope': 'Synthetic/local/blocked paths only; no Robot instance or real parameter setter',
                    'real_entry_both_builds_compiled_linked': True, 'execution_build_default': False,
                    'valid_live_entry_executed': False, 'hardware_connected': False,
                    'robot_parameters_written': False, 'sanitizers': args.sanitize, 'leak_sanitizer_enabled': False,
                    'commands': commands, 'source_sha256': after}
        with args.evidence.open('x') as f: json.dump(evidence, f, indent=2); f.write('\n')
        print('PASS controlled-start local/blocked/operator checks; no robot or parameter writes')
    except BaseException:
        with args.evidence.with_suffix('.failure.json').open('x') as f: json.dump(commands, f, indent=2)
        raise


if __name__ == '__main__': main()
