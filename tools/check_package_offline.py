"""Check the standalone import using only local, blocked or synthetic paths.

Does not run hardware tools, valid control manifests, sudo or network settings.
The historical VMC Python-parity runner and ESN bundle verifier have external
reference inputs; those are documented separately, not silently skipped here.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    missing = [name for name in ('numpy', 'scipy') if importlib.util.find_spec(name) is None]
    if missing:
        parser.error('Missing offline dependencies: '+', '.join(missing)+
                     '; select a Python environment with NumPy and SciPy installed')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    commands = []
    files = sorted(p for p in (ROOT/'tools').rglob('*')
                   if p.suffix in ('.py', '.cpp', '.hpp', '.inc', '.json'))
    before = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    env = os.environ.copy()
    env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')

    def run(argv):
        r = subprocess.run([str(x) for x in argv], cwd=ROOT, env=env,
                           capture_output=True, text=True, timeout=180)
        commands.append({'argv': [str(x) for x in argv], 'returncode': r.returncode,
                         'stdout': r.stdout, 'stderr': r.stderr})
        if r.returncode:
            raise RuntimeError(r.stdout+r.stderr)
        return r

    flags = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror',
             '-isystem', '/usr/include/eigen3', '-pthread']
    try:
        for p in files:
            if p.suffix not in ('.cpp', '.hpp'):
                continue
            for name in re.findall(r'^\s*#include\s+"([^\"]+)"', p.read_text(), re.M):
                if not (p.parent/name).is_file():
                    raise ValueError(f'Missing local include: {p.relative_to(ROOT)} -> {name}')
        run([sys.executable, ROOT/'tools/fr3_native/check_offline.py', '--evidence', out/'native_core.json'])
        run([sys.executable, ROOT/'tools/fr3_native/check_start_offline.py', '--evidence', out/'native_start.json'])
        run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tools/fr3_comm', '-p', 'test_*.py', '-v'])
        for source, binary in [('fr3_taught_path.cpp', 'taught_path'),
                               ('test_fr3_taught_path_offline.cpp', 'taught_math'),
                               ('test_fr3_wrench_trace_offline.cpp', 'wrench_trace'),
                               ('fr3_comm/read_only_probe.cpp', 'read_only_probe'),
                               ('fr3_contact/offline_test.cpp', 'vmc_core')]:
            test_flags = ['-DEIGEN_RUNTIME_NO_MALLOC'] if source == 'fr3_contact/offline_test.cpp' else []
            run(flags+test_flags+[ROOT/'tools'/source, '-o', out/binary, '-lfranka'])
        # Synthetic-only executables; the real probe/taught entry is never given an IP.
        run([out/'taught_math'])
        run([out/'wrench_trace', out/'synthetic_trace'])
        run([out/'vmc_core'])
        env['FR3_OFFLINE_EXECUTOR'] = str(out/'taught_path')
        run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tools',
             '-p', 'test_fr3_teaching_plans_offline.py', '-v'])
        after = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
        if before != after:
            raise RuntimeError('Source changed during checks')
        record = {'scope': 'Standalone import, local include closure, synthetic/blocked tests only',
                  'passed': True, 'hardware_connected': False, 'motion_started': False,
                  'real_robot_parameters_written': False, 'network_settings_changed': False,
                  'sanitizers_this_run': False, 'source_sha256': after, 'commands': commands,
                  'not_repeated': ['ESN original-bundle parity (external bundle required)',
                                   'VMC original-Python parity (upstream research sources required)'],
                  'limitations': ['not physical contact, real-time, load or stopping validation']}
        with (out/'SUMMARY.json').open('x') as f:
            json.dump(record, f, indent=2); f.write('\n')
        print('PASS standalone offline checks; no robot connection or hardware changes')
    except BaseException:
        with (out/'FAILURE.json').open('x') as f:
            json.dump(commands, f, indent=2)
        raise


if __name__ == '__main__':
    main()
