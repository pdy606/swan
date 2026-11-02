#!/usr/bin/env python3
import csv
import math
import sys
from collections import defaultdict


def rmse(values):
    if not values:
        return float('nan')
    return math.sqrt(sum(v * v for v in values) / len(values))


def mean(values):
    return sum(values) / len(values) if values else float('nan')


def main():
    if len(sys.argv) != 2:
        print('Usage: python3 tools/evaluate_distance.py distance_validation.csv')
        raise SystemExit(2)

    rows = []
    with open(sys.argv[1], newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                true_x = float(row['true_x'])
                true_z = float(row['true_z'])
                pred_x = float(row['pred_x'])
                pred_z = float(row['pred_z'])
            except (ValueError, KeyError):
                continue
            row['true_x'] = true_x
            row['true_z'] = true_z
            row['pred_x'] = pred_x
            row['pred_z'] = pred_z
            rows.append(row)

    if not rows:
        print('No valid rows.')
        return

    groups = defaultdict(list)
    groups['ALL'] = rows
    for row in rows:
        groups[row.get('method', 'UNKNOWN')].append(row)

    for name, group in groups.items():
        err_x = [r['pred_x'] - r['true_x'] for r in group]
        err_z = [r['pred_z'] - r['true_z'] for r in group]
        abs_x = [abs(v) for v in err_x]
        abs_z = [abs(v) for v in err_z]
        radial = [math.hypot(ex, ez) for ex, ez in zip(err_x, err_z)]

        print(f'[{name}] n={len(group)}')
        print(f'  X MAE   : {mean(abs_x):.3f} m')
        print(f'  Z MAE   : {mean(abs_z):.3f} m')
        print(f'  X RMSE  : {rmse(err_x):.3f} m')
        print(f'  Z RMSE  : {rmse(err_z):.3f} m')
        print(f'  2D error: {mean(radial):.3f} m mean')


if __name__ == '__main__':
    main()
