"""Execute production PD15 state/scheduling code with fake GPIO and TX enqueue.

No target, serial port or programmer is accessed. UART delivery is not simulated.
"""
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output" / "host_tests"


def main():
    compiler = os.environ.get("AMR_HOST_CC") or shutil.which("clang") or shutil.which("gcc")
    if not compiler:
        raise SystemExit("A host C11 compiler is required (AMR_HOST_CC / clang / gcc)")
    OUT.mkdir(parents=True, exist_ok=True)
    exe = OUT / ("estop_status_tests.exe" if os.name == "nt" else "estop_status_tests")
    target = []
    if os.name == "nt" and os.environ.get("AMR_HOST_SYSROOT"):
        target = ["--target=x86_64-w64-windows-gnu", "--sysroot=" + os.environ["AMR_HOST_SYSROOT"]]
    subprocess.run([compiler, *target, "-std=c11", "-O0", "-g", "-Wall", "-Wextra", "-Werror",
                    "-I" + str(ROOT / "tests/host"), "-I" + str(ROOT / "Core/Inc"),
                    str(ROOT / "Core/Src/estop_status.c"),
                    str(ROOT / "tests/host/test_estop_status.c"), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
