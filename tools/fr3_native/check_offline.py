"""Compile/link native hold integration; execute only synthetic offline checks."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sanitize',action='store_true')
    p.add_argument('--evidence',required=True,type=Path)
    args=p.parse_args()
    if args.evidence.exists():raise FileExistsError(args.evidence)
    args.evidence.parent.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy()
    if args.sanitize:env['ASAN_OPTIONS']=env.get('ASAN_OPTIONS','')+':detect_leaks=0'
    flags=['g++','-std=c++17','-O1' if args.sanitize else '-O2','-Wall','-Wextra','-Werror',
           '-DEIGEN_RUNTIME_NO_MALLOC','-isystem','/usr/include/eigen3']
    if args.sanitize:flags+=['-fsanitize=address,undefined','-fno-omit-frame-pointer']
    with tempfile.TemporaryDirectory(prefix='fr3-native-offline-') as temporary:
        exe=Path(temporary)/'native_offline'
        def run(argv,expect=0):
            r=subprocess.run(argv,capture_output=True,text=True,env=env)
            if r.returncode!=expect:
                for i in range(1,100):
                    failure=args.evidence.with_suffix(f'.failure{i}.txt')
                    if not failure.exists():
                        with failure.open('x') as f:f.write(f'{argv}\n{r.stdout}\n{r.stderr}')
                        break
                raise RuntimeError(r.stdout+r.stderr)
            return r
        run(flags+[str(HERE/'offline.cpp'),'-o',str(exe),'-lfranka'])
        result=run([str(exe),'--offline'])
        proposal=run([str(exe),'--proposal'])
        rejects=[]
        for argv in [[str(exe),'172.16.0.2'],[str(exe),'--execute-reviewed'],[str(exe),'--execute-reviewed','172.16.0.2']]:
            r=run(argv,expect=1)
            assert 'no IP, execution or setter option' in r.stderr
            rejects.append(argv[1:])
    sources=(sorted(HERE.glob('*.hpp'))+sorted(HERE.glob('*.cpp'))+[Path(__file__).resolve(),
        ROOT/'tools/fr3_wrench_trace.hpp']+sorted((ROOT/'tools/fr3_contact').glob('*.hpp'))+
        [ROOT/'tools/fr3_contact/profile_generated.inc'])
    data={'scope':'Synthetic native-impedance guard and fake stiffness transaction checks only',
          'hardware_connected':False,'motion_started':False,'real_parameter_setter_called':False,
          'real_native_session_linked_but_never_invoked':True,'live_cli_present':False,
          'sanitizers':args.sanitize,'leak_sanitizer_enabled':False,'compiler_flags':flags,
          'tests_stdout':result.stdout,'proposal_stdout':proposal.stdout,'rejected_cli_arguments':rejects,
          'source_sha256':{str(x.relative_to(ROOT)):hashlib.sha256(x.read_bytes()).hexdigest() for x in sources},
          'limitations':['no physical native controller, force accuracy, real-time or stopping validation',
                         'proposal is an engineering starting point, not vendor-certified contact limits',
                         'baseline, original stiffness, tool, protection, site and live authorization remain required']}
    with args.evidence.open('x') as f:json.dump(data,f,indent=2);f.write('\n')
    print(result.stdout.strip());print('OFFLINE ONLY; native session not authorized or executed.')


if __name__=='__main__':main()
