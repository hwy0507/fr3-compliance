"""Export only the explicitly selected, hash-pinned ESN. No hardware/network imports."""
import argparse, hashlib, json
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
CHECKPOINT = '453543784cdba345046eb18227c0dec7202a1c4a24bf624445e6d3c518363ecd'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def export(bundle):
    selection=json.loads((bundle/'SELECTION.json').read_text())
    if selection['default_seed']!=45 or selection['seeds'][-1]['checkpoint_sha256']!=CHECKPOINT:
        raise ValueError('Selection drift; review a new model explicitly')
    path=bundle/selection['checkpoint_relative']
    if sha(path)!=CHECKPOINT: raise ValueError('Checkpoint hash mismatch')
    with np.load(path,allow_pickle=False) as z:
        flags={'absolute_inputs':True,'context_inputs':True,'position_context':True,
               'axis_separable':False,'reset_at_each_observation':False,
               'readout_activation':'clipped_linear','output_gain':1.0,
               'contract':'proprio48_office_vmc_memory_esn_v1'}
        shapes={'scale':(6,), 'w':(64,64),'win':(64,156),'leak':(64,),
                'state_context_readout_w':(64,12),'weights':(360,7)}
        if set(z.files)!=set(flags)|set(shapes): raise ValueError('Unsupported model schema')
        for k,v in flags.items():
            if z[k].shape!=() or z[k].item()!=v: raise ValueError(k)
        for k,s in shapes.items():
            if z[k].shape!=s or not np.isfinite(z[k]).all(): raise ValueError(k)
        if np.any(z['scale']<=0) or np.any(z['leak']<=0) or np.any(z['leak']>1):raise ValueError('Invalid scales/leaks')
        if np.any(z['weights'][-12:]!=0):raise ValueError('Context-only action forbidden')
        lines=['// Generated from selected seed45; no robot parameters.',f'inline constexpr char kCheckpointSha[]="{CHECKPOINT}";']
        for k in shapes:
            values=z[k].ravel(order='C')
            lines.append(f'inline constexpr double k_{k}[{len(values)}]={{')
            lines.extend(','.join(format(float(x),'.17g') for x in values[i:i+8])+',' for i in range(0,len(values),8))
            lines.append('};')
    return '\n'.join(lines)+'\n'
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bundle',type=Path);p.add_argument('--check',action='store_true');a=p.parse_args()
    text=export(a.bundle);out=HERE/'model_generated.inc'
    if a.check:
        if out.read_text()!=text:raise ValueError('Generated header differs')
    else:out.write_text(text)
    print('Selected seed45 weights verified; hardware untouched.')
if __name__=='__main__':main()
