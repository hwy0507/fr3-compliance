"""Compile isolated timestamp-instrumented candidate. Never execute a robot binary."""
import difflib
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
SUITE=ROOT/'source'
OUT=ROOT/'artifacts/fr3_camera_sync_20261124'
BASE=SUITE/'franka-safe/esn_yield_roundtrip_diag_20261124.cpp'
EXPECTED='2fda89ec3febab757771df6d7a4c6455d6e93a324bb9e6dc7a28a98a1451d443'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    assert sha(BASE)==EXPECTED, 'Base controller changed; review before patching'
    OUT.mkdir(parents=True,exist_ok=True)
    original=BASE.read_text();source=original
    edits=[('#include "fr3_wrench_trace.hpp"', '#include "fr3_wrench_trace.hpp"\n#include "policy_sync.hpp"'),
      ('  Diagnostics diagnostic(diagnostic_env);', '  Diagnostics diagnostic(diagnostic_env);\n  esn_sync::Recorder policy_sync(diagnostic_env);'),
      ('          const auto callback_begin=std::chrono::steady_clock::now();', '          const auto callback_begin=std::chrono::steady_clock::now();\n          const int64_t sync_entry_ns=esn_sync::monotonic_ns();'),
      ('            const fr3_esn128::V7 action = policy.step(obs, kPolicyDt);', '            const fr3_esn128::V7 action = policy.step(obs, kPolicyDt);\n            policy_sync.capture(sync_entry_ns,rs.time.toSec(),t,obs,policy.state,action);')]
    for before,after in edits:
        assert source.count(before)==1
        source=source.replace(before,after)
    # Includes resolve through the original source directory; original source/binary stay untouched.
    path=OUT/'esn_yield_roundtrip_sync_candidate.cpp';path.write_text(source)
    (OUT/'instrumentation.diff').write_text(''.join(difflib.unified_diff(original.splitlines(True),source.splitlines(True),fromfile=str(BASE),tofile=str(path))))
    binary=OUT/'esn_yield_roundtrip_sync_candidate'
    cmd=['g++','-O3','-DNDEBUG','-std=c++17','-I/usr/include/eigen3',
         '-I'+str(SUITE/'franka-safe'),
         '-I'+str(HERE),str(path),'-lfranka','-o',str(binary)]
    subprocess.run(cmd,check=True)
    current=json.loads((ROOT/'fr3_demo/profile.example.json').read_text())
    profile=dict(current);profile.update(binary=str(binary),binary_sha256=sha(binary),
        review_root=str(ROOT),run_root=str(OUT/'demo_runs'))
    (ROOT/'fr3_demo/profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    subprocess.run(['g++','-O2','-std=c++17',str(HERE/'capture_v4l2.cpp'),'-o',str(OUT/'capture_v4l2')],check=True)
    (OUT/'candidate_profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    result={'base_source_sha256':EXPECTED,'candidate_source_sha256':sha(path),
      'candidate_binary_sha256':sha(binary),'header_sha256':sha(HERE/'policy_sync.hpp'),
      'source_bundle':str(SUITE),
      'edits':len(edits),'robot_program_executed':False,'package_local_profile_generated':True,'compile_command':cmd}
    (OUT/'build.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
