"""Offline conversion of all-joint or explicitly fixed-J1 candidates to plans.

Export is not execution approval. No hardware imports, connections or commands.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.interpolate import PPoly
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'code/mujoco_6d_vmc_benchmark'))
from deploy.review_teaching import review_session


def export(candidate,session,output):
    candidate,output=Path(candidate),Path(output)
    report=json.loads((candidate/'ANALYSIS.json').read_text())
    hold=report['held_j1_rad']
    if hold is not None and not np.isfinite(hold):raise ValueError('nonfinite fixed-J1 reference')
    review,_,_=review_session(session);config=review['robot_config']
    with np.load(candidate/'CANDIDATES_NOT_FOR_EXECUTION.npz',allow_pickle=False) as a:
        degree=int(a['spline_degree']);knots=a['spline_knots'];coef=a['spline_coefficients']
        duration=float(a['duration_s']);rq=a['return_q'];rt=a['return_time_s']
    if degree!=5 or abs(duration-review['duration_robot_s'])>1e-8:
        raise ValueError('candidate time/degree mismatch')
    splines=[PPoly.from_spline((knots,coef[:,j],degree)) for j in range(7)]
    intervals=[i for i in range(len(splines[0].x)-1) if splines[0].x[i+1]>splines[0].x[i]]
    output.mkdir(parents=True,exist_ok=False)
    def save(phase,length,pieces):
        p=output/(phase+'.plan')
        with p.open('x') as f:
            schema='FR3_TAUGHT_FIXED_J1_V1' if hold is not None else 'FR3_TAUGHT_ALL_JOINTS_V2'
            j1_header=hold if hold is not None else float(pieces[0][2][0][-1])
            f.write(f'{schema} {phase} {len(pieces)}\n')
            f.write(' '.join(format(x,'.17g') for x in [length,config['m_total'],j1_header])+'\n')
            for key in ['F_T_EE','F_x_Ctotal','I_total']:
                f.write(' '.join(format(x,'.17g') for x in config[key])+'\n')
            for begin,end,coefficients in pieces:
                coefficients=np.array(coefficients,copy=True)
                if hold is not None:coefficients[0]=[0,0,0,0,0,hold]
                if coefficients.shape!=(7,6) or not np.isfinite(coefficients).all():raise ValueError('invalid polynomial')
                f.write(f'{begin:.17g} {end:.17g}\n')
                for row in coefficients:f.write(' '.join(format(x,'.17g') for x in row)+'\n')
        return p
    replay=save('replay',duration,[(splines[0].x[i],splines[0].x[i+1],
                 np.array([sp.c[:,i] for sp in splines])) for i in intervals])
    total=float(rt[-1]);delta=rq[-1]-rq[0]
    c=np.column_stack([6*delta/total**5,-15*delta/total**4,10*delta/total**3,np.zeros(7),np.zeros(7),rq[0]])
    returning=save('return',total,[(0.,total,c)])
    manifest=dict(hardware_execution_approved=False,scene_clearance_verified=False,
                  fixed_j1_rad=hold,files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [returning,replay]},
                  candidate_analysis_sha256=hashlib.sha256((candidate/'ANALYSIS.json').read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    with (output/'MANIFEST.json').open('x') as f:json.dump(manifest,f,indent=2)
    print('Two plans exported for review; no hardware connection or execution.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('candidate','session','output'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();export(a.candidate,a.session,a.output)
