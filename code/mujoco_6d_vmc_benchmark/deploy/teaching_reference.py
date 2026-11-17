"""Offline reference loading and matching; no hardware imports or setters."""
import hashlib
import json
from pathlib import Path

import numpy as np

from deploy.contracts import finite_vector, validate_robot_config
from deploy.recording import load_recording


def validate_reference(reference):
    if reference.get('pose_record_accepted') is not True:
        raise ValueError('reference not accepted as stationary')
    finite_vector(reference['q_rad'], 7, 'reference q')
    validate_robot_config(reference['tool_configuration'])
    return reference


def load_start_reference(path):
    path = Path(path)
    data = path.read_bytes()
    value = json.loads(data)
    if value.get('schema') == 'FR3_EXPERIMENT_REFERENCE_V1':
        raw = path.parent / 'state.npz'
        if hashlib.sha256(raw.read_bytes()).hexdigest() != value['recording_sha256']:
            raise ValueError('zero reference recording hash mismatch')
        arm = load_recording(raw, max_gap_s=.05)
        if arm['analysis_only'] or len(arm['q']) < 10:
            raise ValueError('zero reference recording incomplete')
        for item in arm['sample_metadata']:
            status = json.loads(item)
            if (status['robot_mode'] != 'RobotMode.Idle' or status['current_errors'] != '[]'
                    or status['last_motion_errors'] != '[]'):
                raise ValueError('zero reference is not error-free Idle')
        if (np.max(np.ptp(arm['q'], axis=0)) > .001
                or np.max(np.abs(arm['dq'])) > .01):
            raise ValueError('zero reference not stationary')
        q = finite_vector(value['q_reference_rad'], 7, 'zero q')
        if not np.allclose(q, arm['q'].mean(axis=0), atol=1e-10, rtol=0):
            raise ValueError('zero reference differs from recorded mean')
        if value['config'] != arm['metadata']['config']:
            raise ValueError('zero reference configuration mismatch')
        value = dict(pose_record_accepted=True, q_rad=q.tolist(),
                     tool_configuration=value['config'], name=value['name'])
    validate_reference(value)
    return dict(pose_record_accepted=True, q_rad=value['q_rad'],
                tool_configuration=value['tool_configuration'],
                name=value.get('name', path.stem), reference_sha256=hashlib.sha256(data).hexdigest(),
                source=str(path.resolve()), start_tolerance_rad=.001)


def check_reference_packet(packet, reference):
    validate_reference(reference)
    if packet['config'] != reference['tool_configuration']:
        raise ValueError('teaching tool configuration differs from saved zero')
    q = finite_vector(packet['q'], 7, 'teaching initial q')
    if np.max(np.abs(q - np.asarray(reference['q_rad']))) > .001:
        raise ValueError('teaching start differs from saved zero; no automatic return')
    if np.max(np.abs(finite_vector(packet['dq'], 7, 'teaching initial dq'))) > .01:
        raise ValueError('teaching start is moving; no automatic stop')
