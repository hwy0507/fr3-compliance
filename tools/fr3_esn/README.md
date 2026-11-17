# Selected ESN64/16 → FR3 adapter

This is an **offline-checked integration candidate**, not a qualified live robot application.
No motion CLI is supplied. Offline checks cannot instantiate a Robot. The only live integration
function is `run_reviewed_esn_hold` in `franka_session.hpp`; every review flag defaults to false,
observer calibration defaults to NaN, and validation occurs before even a read operation.
Do not change these flags merely to get past a failed preflight.

Read [the Chinese deployment handoff](../../docs/FR3_ESN_SIM2REAL_PREPARATION_20261117.md).

## Reproduce offline checks

From the repository root (requires C++17, Eigen3, libfranka 0.21.3 headers/library, Python + NumPy):

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /home/airpig/miniforge3/envs/sii/bin/python \
  tools/fr3_esn/check_offline.py \
  ../artifacts/fr3_esn_w64_n16_deploy_20261117 \
  --archive ../artifacts/FR3_ESN64_16_部署准备包_20261117.tar.gz \
  --output /tmp/fr3_esn_offline_unique_run
```

Use a new output directory for every attempt. Add `--sanitize` for ASan/UBSan (LSan disabled
because this environment's tracing sandbox does not support it). This command verifies all
bundle files and the final archive, extracts only selected reference class ASTs, generates
synthetic/recorded-observation fixtures, compiles the C++ offline tester and compares results.
It never invokes a simulation rollout, training, robot SDK connection or control session.

Files:

- `export_model.py`: strict schema + SHA-pinned seed45 export; other architectures/options rejected.
- `esn_actor.hpp` / `model_generated.inc`: immutable weights, persistent 64-dimensional state;
  reset once per session; exact 25Hz inference and original action filter.
- `observer.hpp`: original residual observer equations and truncated-SVD wrench mapping;
  hardware sampling rate and model calibration remain a transfer risk.
- `esn_core.hpp`: canonical48 construction, conservative resolved-rate WBC, independent SDK
  protection inputs, bounded torque servo, latched faults, bounded reference increments.
- `franka_session.hpp`: libfranka conversion and reviewed hold integration; full raw state,
  model terms, policy state/action and faults recorded in preallocated memory.
- `offline_test.cpp` / `check_offline.py`: numerical parity, reset/cadence, no Eigen heap
  allocations in the tested math, contact/release, fault injection and default-denied review checks.

The hand/camera/payload settings are not generated or written by this module. No parameter
setter, recovery, Desk Run, gripper command or automatic zero-return is included. Preserve
SDK protections and rate limiting. Save `Trace` and any SDK exception after leaving the realtime
callback, including failed sessions. Do not publish raw robot records by default.
