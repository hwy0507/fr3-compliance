"""Verify the frozen bundle and C++ port. No Robot instance, control, or network."""
import argparse, ast, hashlib, importlib.util, json, os, socket, subprocess, tarfile, time
from pathlib import Path
import numpy as np
from export_model import export, sha
HERE=Path(__file__).resolve().parent
ARCHIVE_SHA='f7c248362c28863420f1be9a3cc2c9c363ac29f72d9cbfd5ea2211aae279eecf'
def classes(path,names,scope):
    tree=ast.parse(path.read_text());nodes=[n for n in tree.body if isinstance(n,(ast.ClassDef,ast.FunctionDef)) and n.name in names]
    if len(nodes)!=len(names):raise ValueError('Missing reference classes')
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(path),'exec'),scope)
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bundle',type=Path);p.add_argument('--archive',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--sanitize',action='store_true');a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    def forbidden(*args,**kwargs):raise AssertionError('Network forbidden in offline test')
    socket.socket=forbidden
    manifest=json.loads((a.bundle/'MANIFEST.json').read_text())
    for row in manifest:
        path=a.bundle/row['path']
        if not path.resolve().is_relative_to(a.bundle.resolve()):raise ValueError('Manifest path escape')
        if path.stat().st_size!=row['bytes'] or sha(path)!=row['sha256']:raise ValueError(f'Manifest mismatch {path}')
    archive_members=0
    if a.archive:
        if sha(a.archive)!=ARCHIVE_SHA:raise ValueError('Final archive SHA mismatch')
        with tarfile.open(a.archive,'r:gz') as t:
            seen=set()
            for member in t:
                if not member.isfile():raise ValueError('Only regular files supported')
                rel=Path(member.name)
                if rel.is_absolute() or '..' in rel.parts or member.name in seen:raise ValueError('Unsafe archive member')
                seen.add(member.name);local=a.bundle.parent/rel
                with t.extractfile(member) as f: digest=hashlib.file_digest(f,'sha256').hexdigest() if hasattr(hashlib,'file_digest') else hashlib.sha256(f.read()).hexdigest()
                if not local.is_file() or local.stat().st_size!=member.size or sha(local)!=digest:raise ValueError(f'Archive differs from extracted file: {member.name}')
                archive_members+=1
    if (HERE/'model_generated.inc').read_text()!=export(a.bundle):raise ValueError('Header drift')
    scope={'np':np};actor_path=a.bundle/'runtime/input/office_vmc_memory_esn_20260927.py'
    classes(actor_path,{'OfficeVMCMemoryESN'},scope)
    actor=scope['OfficeVMCMemoryESN'](a.bundle/json.loads((a.bundle/'SELECTION.json').read_text())['checkpoint_relative'])
    ref=json.loads((a.bundle/'ACTOR_REFERENCE.json').read_text());reference=np.array(next(x for x in ref['results'] if x['seed']==45)['actions'])
    recorded=np.array(ref['observations']);actual=np.array([actor.act(o) for o in recorded]);remote_error=float(np.max(np.abs(actual-reference)))
    if remote_error>1e-10:raise ValueError('Packaged reference mismatch')
    # Extract the exact original filter/config AST, without importing simulator modules.
    from dataclasses import dataclass,field
    fs={'np':np,'dataclass':dataclass,'field':field,'ARM_DOF':7}
    source=a.bundle/'runtime/frozen_source/scripts'
    classes(source/'wbc_velocity_residual_core.py',{'VelocityResidualSafetyConfig','FilteredVelocityResidualAction','VelocityResidualActionFilter'},fs)
    filt=fs['VelocityResidualActionFilter'](fs['VelocityResidualSafetyConfig'](maximum_linear_yield_mps=.32))
    rng=np.random.default_rng(20261117);rows=[];actor.reset()
    observations=list(recorded)
    for i in range(4096):
        o=rng.normal(size=48)*.02
        if i<512:o[39:45]=0
        elif i<1536:o[39:45]=[-5,1,0,.1,0,0]
        elif i<2560:o[39:45]=0
        else:o[39:45]=rng.normal(size=6)*[30,30,30,3,3,3]
        o[45:48]=[1,0,0];observations.append(o)
    for i,o in enumerate(observations):
        reset=int(i in (0,64,2048))
        if reset:actor.reset();filt.reset()
        action=actor.act(o);filtered=filt.filter(action,.04)
        rows.append(np.r_[reset,o,action,actor.state,filtered.wbc_scale,filtered.cartesian_yield_twist])
    with (a.output/'actor.txt').open('w') as f:f.write(str(len(rows))+'\n');np.savetxt(f,rows,fmt='%.17g')
    oscope={'np':np};classes(source/'haptic_vmc_teacher_20260916.py',{'JointLoadObserver','equivalent_tool_wrench'},oscope)
    observer=oscope['JointLoadObserver'](np.zeros(7),np.full(7,.1));orows=[]
    for i in range(2048):
        reset=int(i in (0,1024));dt=.00005 if i<1024 else .001
        if reset:observer=oscope['JointLoadObserver'](np.zeros(7),np.full(7,.1))
        velocity=.001*np.sin(np.arange(7)+i*.01);aM=rng.normal(size=(7,7));mass=aM@aM.T*.01+np.eye(7)
        bias=np.arange(7)*.5;passive=-.1*velocity;motor=bias+np.sin(i*.03+np.arange(7))*.2
        load=observer.update(velocity,mass,bias,passive,motor,dt)
        j=rng.normal(size=(6,7));j[-1]*=1e-4 if i%4==0 else 1 # SVD rank cutoff cases
        wrench=oscope['equivalent_tool_wrench'](j,load)
        orows.append(np.r_[reset,dt,velocity,mass.ravel(),bias,passive,motor,load,j.ravel(),wrench])
    with (a.output/'observer.txt').open('w') as f:f.write(str(len(orows))+'\n');np.savetxt(f,orows,fmt='%.17g')
    binary=a.output/'offline_test';args=['g++','-std=c++17','-O1','-g0','-I/usr/include/eigen3',str(HERE/'offline_test.cpp'),'-lfranka','-o',str(binary)]
    if a.sanitize:args[3:3]=['-fsanitize=address,undefined','-fno-omit-frame-pointer']
    env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    if a.sanitize:env['ASAN_OPTIONS']='detect_leaks=0'
    for label,cmd in [('compile',args),('test',[str(binary),str(a.output/'actor.txt'),str(a.output/'observer.txt'),str(a.output/'numeric.json')])]:
        r=subprocess.run(cmd,capture_output=True,text=True,env=env)
        (a.output/(label+'.log')).write_text(r.stdout+r.stderr)
        if r.returncode:raise RuntimeError(f'{label} failed; see {a.output/(label+".log")}')
    result=json.loads((a.output/'numeric.json').read_text())
    result.update(bundle_files_verified=len(manifest),archive_members_verified=archive_members,archive_sha256=ARCHIVE_SHA if a.archive else None,
      remote_reference_max_abs=remote_error,sanitize=a.sanitize,leak_sanitizer=False,
      sources={str(x.relative_to(HERE.parent)):sha(x) for d in (HERE,HERE.parent/'fr3_contact') for x in d.iterdir() if x.is_file()},
      scope='offline numeric parity and fault injection; no robot connection; no hardware timing qualification')
    (a.output/'SUMMARY.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
