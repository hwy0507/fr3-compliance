"""OFFLINE ONLY: preserve recorded timing; screen a direct joint-space return.

Exports candidate curves, never an executable plan. No hardware binding, network,
physics stepping, mode changes or control. Environment clearance is NOT known.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'code/mujoco_6d_vmc_benchmark'))
from deploy.review_teaching import review_session
from deploy.recording import decode_pose
from deploy.teaching_reference import load_start_reference


class Kinematics:
    def __init__(self, urdf, frame):
        root = ET.parse(urdf).getroot()
        self.chain = []
        self.lower, self.upper = [], []
        for i in range(1, 9):
            j = root.find(f"joint[@name='joint{i}']")
            if j is None or j.find('parent').get('link') != f'link{i-1}' or j.find('child').get('link') != f'link{i}':
                raise ValueError('unexpected factory joint chain')
            origin = j.find('origin')
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
            transform[:3, 3] = np.fromstring(origin.get('xyz'), sep=' ')
            axis = None
            if i < 8:
                if j.get('type') != 'revolute':
                    raise ValueError('expected revolute joint')
                axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                if not np.isclose(np.linalg.norm(axis), 1):
                    raise ValueError('non-unit joint axis')
                self.lower.append(float(j.find('limit').get('lower')))
                self.upper.append(float(j.find('limit').get('upper')))
            elif j.get('type') != 'fixed':
                raise ValueError('expected fixed flange joint')
            self.chain.append((transform, axis))
        self.frame = decode_pose(np.array(frame))

    def fk(self, q):
        pose = np.eye(4)
        for i, (origin, axis) in enumerate(self.chain):
            pose = pose @ origin
            if axis is not None:
                motion = np.eye(4)
                motion[:3, :3] = Rotation.from_rotvec(axis*q[i]).as_matrix()
                pose = pose @ motion
        return pose @ self.frame


def interpolation_knots(duration, maximum_step=.02):
    """Cover the original time axis with equal intervals, including both ends.

    Appending duration to arange can leave a 1--2 ms terminal interval next to
    20 ms intervals. Endpoint sensor noise then creates an artificial jerk peak.
    This changes knot spacing, not the recorded duration or safety thresholds.
    """
    if not np.isfinite(duration) or duration <= 0 or not np.isfinite(maximum_step) or maximum_step <= 0:
        raise ValueError('invalid interpolation duration/step')
    return np.linspace(0., duration, max(1, int(np.ceil(duration/maximum_step)))+1)


def prepare(session, start_path, urdf, output, hold_j1_rad=None):
    review, arm, _ = review_session(session)
    start = load_start_reference(start_path)
    if (review.get('start_reference') is not None and
            review['start_reference']['reference_sha256'] != start['reference_sha256']):
        raise ValueError('session reference differs from candidate reference')
    if start.get('pose_record_accepted') is not True:
        raise ValueError('saved start pose not accepted as stationary reference')
    if review['robot_config'] != start['tool_configuration']:
        raise ValueError('tool configuration mismatch')
    if any('not stationary' in x for x in review['data_review_flags']):
        raise ValueError('recording endpoints not stationary')
    t = arm['robot_time_s'] - arm['robot_time_s'][0]
    raw = arm['q']
    if t[-1] < 3 or np.max(np.abs(raw[0] - start['q_rad'])) > .001:
        raise ValueError('recording too short or start mismatch')
    # Fixed 101 ms zero-phase smoothing, 20 ms knots, C4 quintic spline;
    # preserve the original time axis, with zero velocity/acceleration at ends.
    grid = np.unique(np.r_[np.arange(0., t[-1], .001), t[-1]])
    uniform = np.column_stack([np.interp(grid, t, raw[:, j]) for j in range(7)])
    smooth = savgol_filter(uniform, 101, 3, axis=0)
    knots = interpolation_knots(float(t[-1]))
    points = np.column_stack([np.interp(knots, grid, smooth[:, j]) for j in range(7)])
    points[0], points[-1] = start['q_rad'], raw[-1]
    if hold_j1_rad is not None:
        if not np.isfinite(hold_j1_rad):
            raise ValueError('nonfinite J1 hold reference')
        points[:, 0] = hold_j1_rad
    zeros = np.zeros(7)
    spline = make_interp_spline(knots, points, k=5,
                               bc_type=([(1, zeros), (2, zeros)], [(1, zeros), (2, zeros)]))
    q = spline(grid)
    error = np.max(np.abs(spline(t)-raw), axis=0)
    derivatives = {n: np.max(np.abs(spline(grid, nu=n)), axis=0) for n in (1, 2, 3)}
    # Screening budgets only. Actual position-dependent limits and limiter replay
    # still mandatory; these thresholds never authorize motion.
    flags = list(review['data_review_flags'])
    if (error if hold_j1_rad is None else error[1:]).max() > .002:
        flags.append('smoothed path differs by more than 0.002 rad')
    if derivatives[2].max() > 9.5 or derivatives[3].max() > 4500:
        flags.append('original timing exceeds sampled acceleration/jerk screening budget')
    if np.any(derivatives[1] > np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])*.9):
        flags.append('original timing exceeds nominal velocity screening budget')
    for n in (1, 2):
        if np.max(np.abs(spline([0., t[-1]], nu=n))) > 1e-7:
            raise ValueError('spline endpoint derivative mismatch')
    model = Kinematics(urdf, review['robot_config']['F_T_EE'])
    if np.any(q < np.array(model.lower)) or np.any(q > np.array(model.upper)):
        flags.append('candidate exceeds factory URDF position limits')
    indices = np.unique(np.r_[np.arange(0, len(raw), 100), len(raw)-1]).astype(int)
    measured = np.array([decode_pose(arm['O_T_EE'][i]) for i in indices])
    fk_raw = np.array([model.fk(raw[i]) for i in indices])
    fk_fit = np.array([model.fk(spline(t[i])) for i in indices])
    position_error = np.linalg.norm(fk_raw[:, :3, 3]-measured[:, :3, 3], axis=1)
    angle_error = Rotation.from_matrix(np.einsum('nji,njk->nik', fk_raw[:, :3, :3], measured[:, :3, :3])).magnitude()
    if position_error.max() > .005 or angle_error.max() > .02:
        flags.append('factory FK versus measured EE mismatch')
    # Geometric straight line in seven joint coordinates. It is not a Cartesian
    # straight line and is not a proven shortest collision-free path.
    end, goal = raw[-1].copy(), np.asarray(start['q_rad']).copy()
    if hold_j1_rad is not None:
        end[0] = goal[0] = hold_j1_rad
    duration = max(12., 1.875*np.max(np.abs(goal-end))/.12)
    u = np.linspace(0, 1, 501)
    blend = u**3*(10+u*(-15+6*u))
    return_q = end[None, :] + blend[:, None]*(goal-end)
    return_fk = np.array([model.fk(x) for x in return_q])
    report = dict(schema='fr3_teaching_offline_candidates_v1', hardware_replay_ready=False,
        replay_time_scale=1., replay_duration_s=float(t[-1]),
        held_j1_rad=hold_j1_rad,
        j1_modification_requires_user_choice=hold_j1_rad is not None,
        path_fit_max_abs_error_deg=np.rad2deg(error).tolist(),
        sampled_peak_velocity_rad_s=derivatives[1].tolist(),
        sampled_peak_acceleration_rad_s2=derivatives[2].tolist(),
        sampled_peak_jerk_rad_s3=derivatives[3].tolist(),
        fk_raw_vs_measured_max_position_error_m=float(position_error.max()),
        fk_raw_vs_measured_max_angle_error_rad=float(angle_error.max()),
        sampled_smoothing_max_tcp_shift_m=float(np.linalg.norm(fk_fit[:, :3, 3]-fk_raw[:, :3, 3],axis=1).max()),
        return_candidate=dict(kind='straight joint-space segment, quintic time law',
            modeled_start='recorded endpoint, NOT a fresh robot read',duration_s=float(duration),
            delta_q_deg=np.rad2deg(goal-end).tolist(),
            tcp_start_m=return_fk[0,:3,3].tolist(),tcp_end_m=return_fk[-1,:3,3].tolist(),
            sampled_tcp_xyz_min_m=return_fk[:,:3,3].min(axis=0).tolist(),
            sampled_tcp_xyz_max_m=return_fk[:,:3,3].max(axis=0).tolist(),
            sampled_tcp_path_length_m=float(np.linalg.norm(np.diff(return_fk[:,:3,3],axis=0),axis=1).sum())),
        flags=flags, collision_check_performed=False,
        limitations=['No actual joint-dependent velocity-limit/limiter audit yet',
            'Derivative checks use 1ms sampling, not analytic interval certification',
            'No computer, table, camera, cable or full-arm swept-volume clearance model',
            'Return requires a fresh state check and review of the current scene',
            'Original timing is a candidate, not an approved motion program'],
        sources={str(Path(p).resolve()):hashlib.sha256(Path(p).read_bytes()).hexdigest()
                 for p in [Path(session)/'arm.npz',start_path,urdf,Path(__file__)]})
    out = Path(output); out.mkdir(parents=True, exist_ok=False)
    with (out/'ANALYSIS.json').open('x') as f: json.dump(report,f,indent=2,allow_nan=False)
    with (out/'CANDIDATES_NOT_FOR_EXECUTION.npz').open('xb') as f:
        np.savez_compressed(f, spline_knots=spline.t,spline_coefficients=spline.c,spline_degree=5,
                            duration_s=t[-1],return_time_s=u*duration,return_q=return_q,
                            hardware_replay_ready=False)
    with (out/'RETURN_TCP_SCREEN.csv').open('x') as f:
        np.savetxt(f,np.column_stack([u*duration,return_fk[:,:3,3]]),delimiter=',',
                   header='t_s,x_m,y_m,z_m',comments='')
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('session','start','urdf','output'):p.add_argument('--'+arg,type=Path,required=True)
    p.add_argument('--hold-j1-rad',type=float,default=None)
    a=p.parse_args()
    print(json.dumps(prepare(a.session,a.start,a.urdf,a.output,a.hold_j1_rad),indent=2))


if __name__=='__main__':
    main()
