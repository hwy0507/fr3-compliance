"""Fail-closed native-hold launcher. check/stop are local-only; never auto-start.

An execute request requires fresh explicit authorization, verified current source
records and a second, exact confirmation at the C++ program's physical terminal.
This validates supplied attestations; it cannot manufacture or authenticate vendor
readback evidence. The shipped review template intentionally cannot execute.
"""
import argparse
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import secrets
import sys
import time

FLAGS = ('motion_authorized', 'stiffness_write_authorized', 'restore_authorized',
         'previous_stiffness_verified', 'static_baseline_verified', 'tool_and_frames_verified',
         'existing_collision_protection_verified', 'realtime_and_stopping_verified', 'onsite_ready')
RETAIN_FLAGS = ('motion_authorized', 'stiffness_write_authorized', 'static_baseline_conditions_reviewed', 'onsite_ready',
                'retain_trial_authorized', 'collision_settings_preserved', 'collision_values_unverified',
                'tool_configuration_and_static_reviewed', 'host_realtime_and_stop_reviewed')
KINDS = {'vendor_current_readback', 'acknowledged_applied_record'}
THRESHOLDS = {f'{side}_{kind}_{phase}': n for side in ('lower', 'upper')
              for kind, n in [('torque', 7), ('force', 6)]
              for phase in ('acceleration', 'nominal')}


def unique_pairs(pairs):
    result = {}
    for k, v in pairs:
        if k in result:
            raise ValueError(f'duplicate JSON key: {k}')
        result[k] = v
    return result


def load_json(raw):
    def invalid(value):
        raise ValueError(f'non-finite JSON value: {value}')
    return json.loads(raw, object_pairs_hook=unique_pairs, parse_constant=invalid)


def numbers(value, n):
    if not isinstance(value, list) or len(value) != n:
        raise ValueError(f'need {n} numeric values')
    if any(type(x) not in (int, float) or not math.isfinite(x) or x <= 0 for x in value):
        raise ValueError('nonpositive, nonfinite or nonnumeric value')
    return value


def validate(config, base, now=None):
    now = time.time() if now is None else now
    retain = config.get('schema') == 'FR3_NATIVE_RETAIN_LAUNCH_V1'
    if config.get('schema') not in ('FR3_NATIVE_LAUNCH_V1', 'FR3_NATIVE_RETAIN_LAUNCH_V1'):
        raise ValueError('unsupported review schema')
    if config.get('proposal') != 'FR3_NATIVE_CARTESIAN_HOLD_V1':
        raise ValueError('unknown native proposal')
    minimum = config.get('minimum_command_success_rate', .99)
    if type(minimum) not in (int,float) or minimum not in (.99,.95,.90):
        raise ValueError('only 99 default or explicitly authorized 95/90 percent is supported')
    if minimum == .95 and (not retain or config.get('minimum_success_95_authorized') is not True):
        raise ValueError('95 percent needs separate retain-profile authorization')
    if minimum == .90 and (not retain or config.get('minimum_success_90_authorized') is not True):
        raise ValueError('90 percent needs separate retain-profile authorization')
    period=config.get('maximum_callback_period_s',.002)
    if period not in (.002,.005) or (period>.002 and
        (not retain or minimum!=.90 or config.get('native_gap_5ms_reviewed') is not True)):
        raise ValueError('larger receive gap needs reviewed low-speed native retain profile')
    duration=config.get('session_timeout_s',15)
    if duration not in (15,60) or (duration>15 and
        (not retain or period!=.005 or minimum!=.90 or config.get('extended_session_reviewed') is not True)):
        raise ValueError('extended coordination window requires its explicit native review')
    operator_ended=config.get('operator_ended',False)
    if type(operator_ended) is not bool or (operator_ended and
        (not retain or minimum!=.90 or period!=.005 or config.get('mode')!='free_guiding' or
         config.get('operator_ended_authorized') is not True)):
        raise ValueError('operator-ended mode requires supervised free-guiding authorization')
    moment=config.get('relative_moment_limit_nm',.25)
    moment_review={.5:'guiding_moment_05_reviewed',1.:'guiding_moment_10_reviewed'}
    if type(moment) not in (int,float) or moment not in (.25,.5,1.) or (moment>.25 and
        (not operator_ended or config.get(moment_review[moment]) is not True)):
        raise ValueError('0.5/1Nm candidates require their own reviewed operator-ended guiding profile')
    review = config.get('review', {})
    flags = RETAIN_FLAGS if retain else FLAGS
    if set(review) != set(flags) or any(review[k] is not True for k in flags):
        raise ValueError('review/authorization incomplete; no robot connection allowed')
    ip = str(ipaddress.IPv4Address(config['robot_ip']))
    mode = config['mode']
    if mode not in ('hold_only', 'light_push', 'free_guiding'):
        raise ValueError('unknown trial mode')
    if mode != 'hold_only' and config.get('contact_authorized') is not True:
        raise ValueError('light-push contact not explicitly authorized')
    if mode != 'hold_only' and (not isinstance(config.get('contact_point_and_force_measurement'), str)
                                 or not config['contact_point_and_force_measurement'].strip()):
        raise ValueError('contact point and force-scale basis missing')
    if mode == 'free_guiding' and (not retain or config.get('free_guiding_authorized') is not True):
        raise ValueError('free guiding requires separate retain-policy authorization')
    issued, expires = config['issued_unix'], config['expires_unix']
    if type(issued) is not int or type(expires) is not int or not issued <= now < expires:
        raise ValueError('review not yet valid or expired')
    if expires-issued > 600 or expires-now < 30:
        raise ValueError('review lifetime too long / too little time remaining')
    if not isinstance(config.get('operator_authorization'), str) or not config['operator_authorization'].strip():
        raise ValueError('missing explicit operator authorization record')
    cpu = config['cpu']
    if type(cpu) is not int or cpu not in os.sched_getaffinity(0):
        raise ValueError('invalid/unavailable control CPU')
    if retain:
        if config.get('end_policy') != 'retain_trial' or config.get('collision_policy') != 'preserve_unchanged':
            raise ValueError('explicit retain stiffness / preserve collision policies required')
        if config.get('previous_stiffness') is not None or config.get('numeric_collision_values_verified') is not False:
            raise ValueError('retain profile must not mislabel unknown original parameters as verified')
        source = config['readonly_preflight']; path = (base/source['path']).resolve()
        if path.stat().st_size > 16000000: raise ValueError('preflight record too large')
        raw = path.read_bytes(); digest = hashlib.sha256(raw).hexdigest()
        if source['sha256'] != digest: raise ValueError('readonly preflight SHA mismatch')
        record = load_json(raw)
        if record.get('complete') is not True or not record.get('samples'):
            raise ValueError('readonly preflight incomplete')
        for state in record['samples']:
            if state['robot_mode'] != 'RobotMode.Idle' or state['current_errors'] != '[]':
                raise ValueError('readonly preflight not Idle/error-free')
            for key in ('joint_contact','cartesian_contact','joint_collision','cartesian_collision'):
                if any(type(x) not in (int,float) or not math.isfinite(x) or x != 0 for x in state[key]):
                    raise ValueError('readonly preflight contact/collision/invalid flags')
        return {'robot_ip': ip, 'mode': mode, 'cpu': cpu, 'previous': None, 'issued': issued, 'expires': expires,
                'retain': True, 'minimum_success': minimum, 'maximum_period': period, 'session_timeout': duration, 'operator_ended':operator_ended,
                'relative_moment_limit':moment, 'evidence': {'readonly_preflight': {'sha256': digest},
                 'authorization_sha256': hashlib.sha256(config['operator_authorization'].encode()).hexdigest(),
                 'original_stiffness_verified': False, 'numeric_collision_values_verified': False}}
    previous = numbers(config['previous_stiffness'], 6)
    if any(x < (10 if i < 3 else 1) or x > (3000 if i < 3 else 300)
           for i, x in enumerate(previous)):
        raise ValueError('original stiffness outside supported range')
    evidence = {}
    for role in ('stiffness', 'collision'):
        source = config['sources'][role]
        path = (base / source['path']).resolve()
        if path.stat().st_size > 131072:
            raise ValueError('source record too large')
        raw = path.read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if source['sha256'] != digest:
            raise ValueError(f'{role} evidence SHA256 mismatch')
        record = load_json(raw)
        if record.get('kind') not in KINDS or record.get('scope') != 'current_fci' or record.get('robot_ip') != ip:
            raise ValueError(f'{role} is not an accepted current-FCI source')
        if record.get('status') != 'verified_current' or record.get('boot_context_confirmed') is not True or \
                record.get('no_intervening_changes_confirmed') is not True:
            raise ValueError(f'{role} current boot/configuration not confirmed')
        verified = record.get('verified_unix')
        if type(verified) is not int or not issued-300 <= verified <= now:
            raise ValueError(f'{role} evidence confirmation not fresh')
        if not isinstance(record.get('provenance'), str) or not record['provenance'].strip():
            raise ValueError(f'{role} provenance missing')
        if role == 'stiffness':
            if numbers(record['values'], 6) != previous:
                raise ValueError('previous stiffness differs from source')
        else:
            values = record['values']
            if set(values) != set(THRESHOLDS):
                raise ValueError('need all eight FCI threshold arrays, not Desk task scalars')
            for key, n in THRESHOLDS.items():
                numbers(values[key], n)
            for kind in ('torque', 'force'):
                for phase in ('acceleration', 'nominal'):
                    if any(a > b for a, b in zip(values[f'lower_{kind}_{phase}'], values[f'upper_{kind}_{phase}'])):
                        raise ValueError('lower contact threshold exceeds upper collision threshold')
            if record.get('contact_envelope_reviewed') is not True:
                raise ValueError('effective protection/contact envelope not reviewed')
        evidence[role] = {'sha256': digest, 'record': record}
    return {'robot_ip': ip, 'mode': mode, 'cpu': cpu, 'previous': previous,
            'issued': issued, 'expires': expires, 'evidence': evidence}


def manifest(checked, session):
    retain = checked.get('retain', False)
    retain_schema = { .90:'FR3_NATIVE_RETAIN_90_V1',.95:'FR3_NATIVE_RETAIN_95_V1' }.get(checked.get('minimum_success'),'FR3_NATIVE_RETAIN_V1')
    if checked.get('maximum_period')==.005:retain_schema='FR3_NATIVE_RETAIN_90_GAP5_V1'
    if checked.get('session_timeout')==60:retain_schema='FR3_NATIVE_RETAIN_90_GAP5_LONG_V1'
    if checked.get('operator_ended'):retain_schema='FR3_NATIVE_OPERATOR_V1'
    if checked.get('relative_moment_limit')==.5:retain_schema='FR3_NATIVE_OPERATOR_MOMENT05_V1'
    if checked.get('relative_moment_limit')==1.:retain_schema='FR3_NATIVE_OPERATOR_MOMENT10_V1'
    start = [retain_schema if retain else 'FR3_NATIVE_START_V1', checked['robot_ip'],
                      str({'hold_only': 0, 'light_push': 1, 'free_guiding': 2}[checked['mode']]), str(checked['cpu']),
                      '0' if checked['mode'] == 'hold_only' else '1',
                      str(checked['issued']), str(checked['expires']), session,
                      ' '.join(['1'] * len(FLAGS))]
    if retain:
        return '\n'.join(start+[checked['evidence']['readonly_preflight']['sha256'],
                                checked['evidence']['authorization_sha256']])+'\n'
    return '\n'.join(start+[
                      ' '.join(format(x, '.17g') for x in checked['previous']),
                      checked['evidence']['stiffness']['sha256'],
                      checked['evidence']['collision']['sha256']]) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    check = sub.add_parser('check'); check.add_argument('review', type=Path)
    execute = sub.add_parser('execute'); execute.add_argument('review', type=Path)
    execute.add_argument('new_run', type=Path); execute.add_argument('--binary', type=Path, required=True)
    stop = sub.add_parser('stop'); stop.add_argument('run', type=Path)
    args = parser.parse_args()
    if args.action == 'stop':
        # This sends no robot API command. The supervisor observes this latch.
        run = args.run.resolve()
        marker = load_json((run / 'review_snapshot.json').read_bytes())
        if marker.get('schema') not in ('FR3_NATIVE_LAUNCH_V1', 'FR3_NATIVE_RETAIN_LAUNCH_V1'):
            raise ValueError('not a native trial run directory')
        try:
            with (run / 'STOP').open('x') as out:
                out.write('operator requested stop\n'); out.flush(); os.fsync(out.fileno())
        except FileExistsError:
            pass
        print('STOP requested; wait for controller exit. This is not an emergency-stop acknowledgement.')
        return 0
    raw = args.review.read_bytes(); config = load_json(raw)
    checked = validate(config, args.review.resolve().parent)
    if args.action == 'check':
        print('REVIEW_VALID: local files only; no Robot, parameter write or control')
        return 0
    if not sys.stdin.isatty() or not sys.stdout.isatty():
        raise ValueError('execute requires an attended interactive terminal')
    binary = args.binary.resolve(strict=True)
    args.new_run.mkdir(mode=0o700)  # exclusive; never reuse a previous run
    with (args.new_run / 'review_snapshot.json').open('xb') as out:
        out.write(raw)
    session = secrets.token_hex(16)
    with (args.new_run / 'manifest.txt').open('x') as out:
        out.write(manifest(checked, session))
    with (args.new_run / 'source_evidence.json').open('x') as out:
        json.dump(checked['evidence'], out, indent=2)
    with (args.new_run / 'binary_sha256.txt').open('x') as out:
        out.write(hashlib.sha256(binary.read_bytes()).hexdigest()+'\n')
    # C++ checks its manifest, single-use marker, expiry and terminal confirmation
    # before constructing Robot. No shell or implicit execution on check.
    # Replace the launcher. A Python subprocess parent may kill its child on
    # KeyboardInterrupt before the C++ restoration transaction can unwind.
    os.execv(str(binary), [str(binary), '--execute-reviewed',
                          str((args.new_run / 'manifest.txt').resolve()), str(args.new_run.resolve())])


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, KeyError, TypeError, OSError) as error:
        print(f'BLOCKED before launch: {error}', file=sys.stderr)
        raise SystemExit(2)
