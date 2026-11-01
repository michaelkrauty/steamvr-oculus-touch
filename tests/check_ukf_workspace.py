#!/usr/bin/env python3
"""Build a scoped native regression and compare allocating/owned-workspace paths.

Only synthetic fixtures are used. The temporary executable is removed on exit.
Optional --output-dir retains raw per-step evidence and a machine-readable summary.
No timing, peak-memory or tracking-latency claim is made.
"""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--checks', choices=['0', '1'], default='1')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    src = root / 'third_party/openhmd/src'
    summary = []
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='ukf-workspace-', dir=args.output_dir) as temp:
        binary = str(Path(temp) / 'probe')
        command = shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=gnu11', '-O2', '-DOHMD_NONBLOCK_LOG', '-fno-strict-aliasing',
            '-fno-strict-overflow', f'-DMATRIX_CHECKS={args.checks}',
            f'-I{src}', f'-I{src / "drv_oculus_rift"}',
            f'-I{src.parent / "include"}', str(root / 'tests/test_ukf_workspace.c'),
        ] + [str(src / name) for name in [
            'matrices.c', 'ukf.c', 'unscented.c', 'omath.c',
            'drv_oculus_rift/rift-kalman-6dof.c',
        ]] + [
            '-Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc,--wrap=free,'
            '--wrap=ukf_base_update,--wrap=matrix2d_multiplyXYXt_workspace',
            '-lm', '-o', binary,
        ]
        print(shlex.join(command), flush=True)
        subprocess.run(command, check=True)
        for slots in [0, 1, 2, 3, 5]:
            for mixed in [0, 1]:
                records = {}
                counts = {}
                for variant, reference in [('reference', '1'), ('production', '0')]:
                    text = subprocess.check_output(
                        [binary, str(slots), str(mixed), reference], text=True)
                    lines = text.splitlines()
                    assert lines[-1] == 'PASS'
                    if args.output_dir:
                        (args.output_dir / f'{variant}-{slots}-{mixed}.txt').write_text(text)
                    records[variant] = [line for line in lines if line.startswith(
                        ('STATE ', 'CROSS ', 'FINAL ', 'FINAL_CROSS ', 'CONTROL ', 'LIFECYCLE '))]
                    totals = defaultdict(lambda: defaultdict(int))
                    for line in lines:
                        if not line.startswith('COUNT '):
                            continue
                        _, name, *fields = line.split()
                        totals[name]['operations'] += 1
                        for field in fields:
                            key, value = field.split('=')
                            totals[name][key] += int(value)
                    counts[variant] = {key: dict(value) for key, value in totals.items()}
                assert records['reference'] == records['production'], (slots, mixed)
                summary.append(dict(slots=slots, mixed=bool(mixed),
                                    exact_records_equal=len(records['production']), counts=counts))
                print(f'PASS slots={slots} mixed={mixed}: whole-operation allocation counts, '
                      'state/covariance/Pxz step hashes and full final values, balanced teardown',
                      flush=True)
    if args.output_dir:
        (args.output_dir / 'comparison.json').write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    main()
