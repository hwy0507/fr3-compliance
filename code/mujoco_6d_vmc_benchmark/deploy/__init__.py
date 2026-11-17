"""Real-FR3 deployment package for the SII compliance project.

MuJoCo is used purely as a kinematics/dynamics computation backend fed with
measured robot state, so every model-dependent quantity (FK, Jacobian, mass
matrix, bias/passive torque) is computed with the *same* model and math as in
training.  Only measured q / dq / tau_J cross the sim-to-real boundary.
"""
import sys
from pathlib import Path

_SCRIPTS = Path(__file__).resolve().parent.parent / "scripts"
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))
