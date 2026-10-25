"""Run the retained offline production-C tests from this research snapshot."""
import subprocess
import sys
from verify_baseline import BASELINE, verify

if __name__ == "__main__":
    verify()
    for script in ("run_estop_status_tests.py", "run_wheel_rpdo_tests.py"):
        subprocess.run([sys.executable, str(BASELINE / "tools" / script)], cwd=BASELINE, check=True)
    verify()
    print("PASS: offline baseline regressions. No hardware acceptance or research result is implied.")
