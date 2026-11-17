"""Offline-only conversion/regression of saved static_model.json + state.npz.

No hardware imports, Robot instance, parameter setter, or control loop. The C++
fixture tests wrench guards in an otherwise synthetic valid state, NOT a full
hardware preflight. Raw input files stay in the caller's local record directory.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import numpy as np

HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(x):
    return {k: np.asarray(f(x, axis=0)).tolist()
            for k, f in [('mean', np.mean), ('std', np.std), ('min', np.min), ('max', np.max)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record-dir', required=True, type=Path)
    parser.add_argument('--evidence', required=True, type=Path)
    parser.add_argument('--sanitize', action='store_true')
    args = parser.parse_args()
    if args.evidence.exists():
        raise FileExistsError(args.evidence)
    def forbidden(*a, **k):
        raise AssertionError('network forbidden in recorded offline regression')
    socket.socket = forbidden
    paths = [args.record_dir / x for x in ('static_model.json', 'state.npz', 'READONLY_ASSESSMENT.json')]
    before = {p.name: sha(p) for p in paths}
    record = json.loads(paths[0].read_text())
    assessment = json.loads(paths[2].read_text())
    for p in paths[:2]:
        assert assessment['source_sha256'][p.name] == before[p.name], 'input provenance mismatch'
    assert record['complete'] and not record['hardware_commands_sent'] and not record['parameters_written']
    with np.load(paths[1], allow_pickle=False) as npz:
        assert bool(npz['complete']) and not bool(npz['interrupted'])
        config = json.loads(str(npz['metadata_json']))['config']
    assert config == assessment['config'], 'companion configuration mismatch'
    samples = record['samples']
    assert samples
    def arr(key, width):
        a = np.asarray([s[key] for s in samples], dtype=float)
        assert a.shape == (len(samples), width) and np.isfinite(a).all(), key
        return a
    t_flat = arr('O_T_EE', 16)
    t = t_flat.reshape(-1, 4, 4).transpose(0, 2, 1)
    k = np.asarray(config['EE_T_K']).reshape(4, 4, order='F')
    assert np.isfinite(k).all()
    for key in ('F_T_EE', 'm_total', 'F_x_Ctotal', 'I_total'):
        assert all(np.array_equal(s[key], config[key]) for s in samples), key
    # EE_T_K was saved in the companion state metadata, not each model sample.
    # Its constancy between captures is an explicit provenance limitation.
    assert all(s['robot_mode'] == 'RobotMode.Idle' and s['current_errors'] == '[]'
               and s['last_motion_errors'] == '[]' for s in samples)
    assert all(np.all(arr(key, width) == 0) for key, width in
               [('joint_contact', 7), ('joint_collision', 7),
                ('cartesian_contact', 6), ('cartesian_collision', 6)])
    times = np.asarray([s['robot_time_s'] for s in samples])
    assert np.isfinite(times).all() and np.all(np.diff(times) > 0)
    o, kw = arr('O_F_ext_hat_K', 6), arr('K_F_ext_hat_K', 6)
    r, p = t[:, :3, :3], t[:, :3, 3]
    rk = r @ k[:3, :3]
    f = np.einsum('nij,nj->ni', rk, kw[:, :3])
    m = np.einsum('nij,nj->ni', rk, kw[:, 3:])
    arm = np.einsum('nij,j->ni', r, k[:3, 3])
    expected_raw = -np.c_[f, m]
    expected_ee = -np.c_[f, m + np.cross(arm, f)]
    origin_predicted = np.c_[f, m + np.cross(p + arm, f)]
    # Independent O-origin route; this relationship is empirical for this record.
    from_origin = -np.c_[o[:, :3], o[:, 3:] - np.cross(p, o[:, :3])]
    np.testing.assert_allclose(o, origin_predicted, rtol=0, atol=1e-6)
    np.testing.assert_allclose(expected_ee, from_origin, rtol=0, atol=1e-6)
    rows = np.c_[t_flat, np.tile(k.reshape(16, order='F'), (len(samples), 1)), o, kw]
    env = os.environ.copy()
    if args.sanitize:
        env['ASAN_OPTIONS'] = env.get('ASAN_OPTIONS', '') + ':detect_leaks=0'
    def checked(argv):
        proc = subprocess.run(argv, capture_output=True, text=True, env=env)
        if proc.returncode:
            for i in range(1, 100):
                failure = args.evidence.with_suffix(f'.failure{i}.txt')
                if not failure.exists():
                    with failure.open('x') as f_out:
                        f_out.write(f'{argv}\n{proc.stdout}\n{proc.stderr}')
                    break
            raise RuntimeError(f'offline command failed: {proc.stderr}')
        return proc
    with tempfile.TemporaryDirectory(prefix='fr3-recorded-wrench-') as tmp:
        exe, fixture = Path(tmp)/'offline_test', Path(tmp)/'wrench.txt'
        flags = ['g++', '-std=c++17', '-O1' if args.sanitize else '-O2', '-Wall', '-Wextra',
                 '-Werror', '-DEIGEN_RUNTIME_NO_MALLOC', '-isystem', '/usr/include/eigen3']
        if args.sanitize:
            flags += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer']
        checked(flags + [str(HERE/'offline_test.cpp'), '-o', str(exe), '-lfranka'])
        np.savetxt(fixture, rows, fmt='%.17g')
        actual = np.loadtxt(checked([str(exe), '--wrench-fixture', str(fixture)]).stdout.splitlines())
    assert actual.shape == (len(samples), 15)
    expected = np.c_[expected_ee, expected_raw]
    np.testing.assert_allclose(actual[:, :12], expected, rtol=1e-12, atol=1e-12)
    assert np.all(actual[:, 12] == 1), 'recorded frame/config conversion rejected'
    # This recording still violates the unmodified 1 Nm stop. Never turn it into a ready gate.
    assert np.all(actual[:, 13] == 0) and np.all(actual[:, 14] == 8), 'expected Fault::Wrench'
    q, dq, tau = arr('q', 7), arr('dq', 7), arr('tau_J', 7)
    ext, gravity = arr('tau_ext_hat_filtered', 7), arr('gravity', 7)
    coriolis = arr('coriolis', 7)
    jac = arr('zero_jacobian', 42).reshape(-1, 7, 6).transpose(0, 2, 1)
    measured_residual = tau-gravity-coriolis
    # Diagnostic only: internal friction/observer filtering are not modeled by tau-g-c.
    projected = np.einsum('nji,nj->ni', jac, expected_ee)
    n = len(samples)
    result = {
        'hardware_connected': False, 'motion_started': False, 'parameters_written': False,
        'hardware_qualified': False, 'sanitizers': args.sanitize, 'leak_sanitizer_enabled': False,
        'samples': n, 'duration_robot_s': float(times[-1]-times[0]), 'input_sha256': before,
        'source_sha256': {p.name: sha(p) for p in sorted(HERE.glob('*.hpp')) +
                          [HERE/'offline_test.cpp', HERE/'profile_generated.inc', Path(__file__).resolve()]},
        'conversion_max_abs_error': float(np.max(np.abs(actual[:, :12]-expected))),
        'O_force_rotation_error_max_N': float(np.linalg.norm(o[:, :3]-f, axis=1).max()),
        'O_moment_origin_transform_error_max_Nm': float(np.linalg.norm(o[:, 3:]-origin_predicted[:, 3:], axis=1).max()),
        'old_conversion_moment_error_norm_Nm': stats(np.linalg.norm(-o[:, 3:]-expected_ee[:, 3:], axis=1)),
        'force_norm_N': stats(np.linalg.norm(expected_ee[:, :3], axis=1)),
        'corrected_EE_moment_norm_Nm': stats(np.linalg.norm(expected_ee[:, 3:], axis=1)),
        'K_moment_norm_Nm': stats(np.linalg.norm(expected_raw[:, 3:], axis=1)),
        'conversion_configuration_passed': int(np.sum(actual[:, 12])),
        'isolated_wrench_guard_rejected': int(np.sum(actual[:, 13] == 0)),
        'gate_scope': 'Recorded wrenches in synthetic otherwise-valid state; NOT full hardware preflight',
        'force_startup_1N_exceeded': int(np.sum(np.linalg.norm(expected_raw[:, :3], axis=1) > 1)),
        'moment_startup_015Nm_exceeded': int(np.sum(np.linalg.norm(expected_raw[:, 3:], axis=1) > .15)),
        'tau_ext_Nm': stats(ext), 'tau_minus_gravity_minus_coriolis_Nm': stats(measured_residual),
        'tau_model_residual_minus_reported_external_Nm': stats(measured_residual-ext),
        'J_transpose_corrected_environment_wrench_minus_tau_ext_Nm': stats(projected-ext),
        'J_transpose_corrected_environment_wrench_plus_tau_ext_Nm': stats(projected+ext),
        'force_first_to_last_half_mean_change_N': float(np.linalg.norm(f[n//2:].mean(0)-f[:n//2].mean(0))),
        'moment_first_to_last_half_mean_change_Nm': float(np.linalg.norm(m[n//2:].mean(0)-m[:n//2].mean(0))),
        'q_span_max_rad': float(np.ptp(q, axis=0).max()), 'dq_max_rad_s': float(np.abs(dq).max()),
        'rotation_orthogonality_error_max': float(np.linalg.norm(r.transpose(0, 2, 1)@r-np.eye(3), axis=(1, 2)).max()),
        'collision_threshold_values_verified': False,
        'limitations': ['single stationary pose cannot identify mass/COM/inertia and torque bias separately',
                       'EE_T_K sourced from adjacent state metadata, not stored per model sample',
                       'O moment convention observation limited to recorded firmware/configuration',
                       'wrench algebra consistency does not validate physical sign, load, friction, or safety',
                       'no bias cancellation, robot setter, threshold relaxation, or live test performed']}
    assert before == {p.name: sha(p) for p in paths}, 'input files changed'
    with args.evidence.open('x') as output:
        json.dump(result, output, indent=2, allow_nan=False); output.write('\n')
    print(f'PASS {n} saved samples; conversion max error {result["conversion_max_abs_error"]:.3g}; '
          f'{result["isolated_wrench_guard_rejected"]} correctly rejected by unchanged wrench guard. NOT ready.')


if __name__ == '__main__':
    main()
