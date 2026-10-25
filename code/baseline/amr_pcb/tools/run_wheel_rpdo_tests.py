"""Compile and execute production C on the host with an injected CAN/HAL bus.
The fake bus cannot prove physical timing, motor application or compatibility.
"""
from pathlib import Path
import os
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'output' / 'host_tests'
OUT.mkdir(parents=True, exist_ok=True)
compiler = os.environ.get('AMR_HOST_CC') or shutil.which('clang') or shutil.which('gcc')
if not compiler:
    raise SystemExit('NOT RUN: host C compiler missing')
compiler_args = []
if sys.platform.startswith('win') and os.environ.get('AMR_HOST_SYSROOT'):
    compiler_args = ['--target=x86_64-w64-windows-gnu',
                     '--sysroot=' + os.environ['AMR_HOST_SYSROOT']]
# Generate inert trace sinks from the actual prototypes (no motion/CAN mocks).
header = (ROOT / 'Core/Inc/can_trace.h').read_text()
stubs = ['#include "can_trace.h"\n']
for proto in re.findall(r'void CanTrace_\w+\([^;]+\);', header):
    args = re.findall(r'(\w+)\s*[,)]', proto)
    stubs.append(proto[:-1] + ' { ' + ''.join(f'(void){a};' for a in args) + ' }\n')
stubs.append('CanTraceRuntimeStats_t CanTrace_GetRuntimeStats(CanTraceRuntimeChannel_t c) { (void)c; CanTraceRuntimeStats_t s={0}; return s; }\n')
(OUT / 'trace_sinks.c').write_text(''.join(stubs))
sources = ['canopen', 'wheel_pdo_config', 'wheel_rpdo_tx', 'motion_control',
           'drive_target', 'drive_core', 'drive_control', 'drive_safety',
           'drive_diag', 'drive_fault_reset', 'drive', 'odom', 'lift', 'drive_lift_stop_diag']
cmd = [compiler, *compiler_args, '-std=c11', '-O0', '-g', '-Wall', '-Wextra', '-Werror',
       '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
       '-DAMR_HOST_TEST', '-DFIRMWARE_BUILD_PROFILE_ID=1',
       '-DDEBUG_SDO=0', '-DDEBUG_CMD_VEL=0', '-DDEBUG_STATUSWORD=0',
       '-Itests/host', '-ICore/Inc', '-ICore/Src',
       *[f'Core/Src/{s}.c' for s in sources], 'tests/host/fake_bus.c',
       str(OUT / 'trace_sinks.c'), 'tests/host/test_wheel_rpdo.c', 'tests/host/test_drive_rpdo.c',
       'tests/host/test_drive_retarget.c',
       'tests/host/test_lift_base_stop.c',
       'tests/host/test_pdo_startup_recovery.c',
       '-o', str(OUT / 'wheel_rpdo_tests.exe')]
if sys.platform.startswith('linux'):
    cmd.append('-lm')  # Link after objects: motion_control uses libm functions.
subprocess.run(cmd, cwd=ROOT, check=True)
subprocess.run([str(OUT / 'wheel_rpdo_tests.exe')], cwd=ROOT, check=True)

# Exercise startup-only recovery using the production state machines and a
# wire-level fake-device hook. This separate main leaves retained suites intact.
boot_cmd = [arg for arg in cmd if arg != 'tests/host/test_wheel_rpdo.c']
boot_cmd[boot_cmd.index('-o') + 1] = str(OUT / 'drive_boot_recovery_tests.exe')
boot_cmd += ['tests/host/test_drive_boot_recovery.c']
subprocess.run(boot_cmd, cwd=ROOT, check=True)
subprocess.run([str(OUT / 'drive_boot_recovery_tests.exe')], cwd=ROOT, check=True)
strict_boot_cmd = [*boot_cmd, '-DDRIVE_BASIC_BRINGUP_PROFILE=0']
strict_boot_cmd[strict_boot_cmd.index('-o') + 1] = str(OUT / 'drive_boot_recovery_strict_tests.exe')
subprocess.run(strict_boot_cmd, cwd=ROOT, check=True)
subprocess.run([str(OUT / 'drive_boot_recovery_strict_tests.exe')], cwd=ROOT, check=True)

# Parse the actual production-C UART output, not just handwritten fixtures.
# Both configured and failed snapshots must round-trip with no identity data.
sys.path.insert(0, str(ROOT / 'jetson_tools'))
from amr_rpdo_diagnostics import parse_rpdo, require_rpdo_ready
for filename, should_pass in (('rpdo_no_identity_snapshot.log', True),
                              ('rpdo_abort_snapshot.log', False),
                              ('pdo_recovery_snapshot.log', True),
                              ('pdo_startup_blocked_snapshot.log', False)):
    wire = (OUT / filename).read_text().splitlines()
    assert all(len(line) + 2 <= 192 for line in wire)
    rows = parse_rpdo(wire, '12345678')
    assert rows is not None, filename
    for row in rows[3:5]:
        assert row['identity'] == 'NOT_QUERIED'
        assert all(row[field] == 'UNKNOWN' for field in ('vendor', 'product', 'revision', 'serial'))
    try:
        require_rpdo_ready(rows)
    except RuntimeError as error:
        assert not should_pass, error
        details = (('Node 1', 'WRITE', '1601', '01', '06040041',
                    '60FF0020', '00000000', 'OBJECT_NOT_MAPPABLE')
                   if filename == 'rpdo_abort_snapshot.log' else
                   ('WAIT_SAFE', 'DRIVE_FAULT_ACTIVE', 'tracking', 'inhibit'))
        for detail in details:
            assert detail in str(error), error
    else:
        assert should_pass, 'Failed config was accepted by Jetson gate'
    if filename == 'pdo_recovery_snapshot.log':
        assert rows[8]['startup'] == 'COMPLETE' and rows[8]['blocked'] == 'NONE'
        assert rows[8]['tracking'] == rows[8]['inhibit'] == rows[8]['hw_dis'] == '1'
        assert rows[8]['drive'] == 'DISABLED'
print('PASS: production-C diagnostics -> Jetson parser/gate, unknown identity and full failure context')

# BASIC is the only profile whose historical latches may be ignored here.
# Compile the same gates with BASIC disabled and exercise both strict gates.
strict_cmd = [arg for arg in cmd if arg != 'tests/host/test_wheel_rpdo.c']
strict_cmd[strict_cmd.index('-o') + 1] = str(OUT / 'pdo_strict_gate_tests.exe')
strict_cmd += ['-DDRIVE_BASIC_BRINGUP_PROFILE=0', '-DAMR_HOST_TEST_PDO_GATE_MAIN']
subprocess.run(strict_cmd, cwd=ROOT, check=True)
subprocess.run([str(OUT / 'pdo_strict_gate_tests.exe')], cwd=ROOT, check=True)

# Lighting exercises the real UART parser, Drive/Lift fault predicates and
# GPIO writer on the same fake bus; unrelated peripheral queries are stubbed.
light_cmd = [arg for arg in cmd if arg != 'tests/host/test_wheel_rpdo.c']
light_cmd[light_cmd.index('-o') + 1] = str(OUT / 'status_light_tests.exe')
light_cmd += ['Core/Src/jetson_protocol.c', 'Core/Src/firmware_identity.c',
              'Core/Src/status_light.c', 'Core/Src/safety_io.c',
              'tests/host/test_status_light.c']
subprocess.run(light_cmd, cwd=ROOT, check=True)
subprocess.run([str(OUT / 'status_light_tests.exe')], cwd=ROOT, check=True)
