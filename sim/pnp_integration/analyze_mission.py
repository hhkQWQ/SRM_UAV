#!/usr/bin/env python3
"""Summarize actual visual closed-loop flight from the simulation CSV."""
import argparse
import csv
import json
import math
from pathlib import Path
from collections import Counter


def quantile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))]


def number(row, key):
    try:
        value = float(row[key])
        return value if math.isfinite(value) else None
    except (KeyError, ValueError, TypeError):
        return None


def summarize(path, final_seconds=10.):
    with Path(path).open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    timed = [r for r in rows if number(r, 'sim_time_s') is not None]
    if not timed:
        return {'file': str(path), 'passed': False, 'reason': 'NO_SAMPLES'}
    active = [r for r in timed if r.get('handover_state') == 'ACTIVE']
    last_time = number(timed[-1], 'sim_time_s')
    final = [r for r in timed if number(r, 'sim_time_s') >= last_time - final_seconds]
    horizontal = [number(r, 'target_horizontal_error_m') for r in final]
    vertical = [number(r, 'target_height_error_m') for r in final]
    horizontal = [x for x in horizontal if x is not None]
    vertical = [x for x in vertical if x is not None]
    offboard = sum(r.get('offboard') in ('True', '1') for r in final) / len(final)
    measured = sum(r.get('track_state') == 'MEASURED' for r in final) / len(final)
    active_rate = sum(r.get('handover_state') == 'ACTIVE' for r in final) / len(final)
    coverage = number(final[-1], 'sim_time_s') - number(final[0], 'sim_time_s')
    absolute_vertical = [abs(x) for x in vertical]
    h95, z95 = quantile(horizontal, .95), quantile(absolute_vertical, .95)
    # Pass is based on Gazebo truth, not on the controller declaring HOLD.
    passed = bool(active and coverage >= .9 * final_seconds and h95 is not None
                  and z95 is not None and h95 <= .10 and z95 <= .10
                  and len(horizontal) == len(final) and len(vertical) == len(final)
                  and offboard >= .95 and active_rate >= .95 and measured >= .90)
    return {
        'file': str(Path(path).resolve()), 'passed': passed,
        'criterion': 'final window >=90% coverage; truth XY and |height-1m| P95 <=0.10m; active visual Offboard >=95%; measured >=90%; all truth samples available',
        'rows': len(timed), 'active_rows': len(active),
        'first_active_sim_time_s': number(active[0], 'sim_time_s') if active else None,
        'final_window_s': [number(final[0], 'sim_time_s'), last_time],
        'final_sample_count': len(final), 'final_truth_sample_count': len(horizontal),
        'final_offboard_rate': offboard, 'final_active_rate': active_rate, 'final_measured_rate': measured,
        'final_horizontal_error_m': {'mean': sum(horizontal) / len(horizontal) if horizontal else None,
                                     'p95': h95, 'max': max(horizontal) if horizontal else None},
        'final_height_error_m': {'bias': sum(vertical) / len(vertical) if vertical else None,
                                 'abs_p95': z95, 'abs_max': max(absolute_vertical) if vertical else None},
        'mission_states': dict(Counter(r.get('mission_state') for r in timed)),
        'handover_states': dict(Counter(r.get('handover_state') for r in timed)),
        'control_states': dict(Counter(r.get('control_state') for r in timed)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='?')
    parser.add_argument('--latest', action='store_true')
    parser.add_argument('--log-dir', default=str(Path(__file__).resolve().parents[1] / 'logs'))
    parser.add_argument('--final-seconds', type=float, default=10.)
    parser.add_argument('--output')
    options = parser.parse_args()
    if not math.isfinite(options.final_seconds) or options.final_seconds <= 0:
        parser.error('--final-seconds must be positive')
    if options.latest:
        candidates = list(Path(options.log_dir).glob('pnp_sim_*.csv'))
        if not candidates:
            parser.error('No simulation CSV logs found')
        path = max(candidates, key=lambda p: p.stat().st_mtime)
    elif options.csv:
        path = Path(options.csv)
    else:
        parser.error('Supply CSV or --latest')
    report = json.dumps(summarize(path, options.final_seconds), indent=2, ensure_ascii=False)
    if options.output:
        Path(options.output).write_text(report + '\n')
    print(report)


if __name__ == '__main__':
    main()
