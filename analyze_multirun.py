#!/usr/bin/env python3
"""
analyze_multirun.py — Analisis distribusi RMSE multi-run
=========================================================
Usage:
  python3 analyze_multirun.py pose_eval_langkahB1_run*.csv
  python3 analyze_multirun.py --baseline pose_eval_langkah0.csv pose_eval_fase2_step1_v2.csv \
                               --test    pose_eval_langkahB1_run*.csv
"""
import csv, sys, math, argparse, os

def load_rmse(path):
    """Baca CSV, return dict {source: [errors...]}"""
    rows = []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)
    if not rows:
        return {}

    result = {}
    for key, col in [('EKF', 'err_ekf_pos_m'),
                     ('AMCL', 'err_amcl_pos_m'),
                     ('Cox', 'err_cox_pos_m')]:
        if col not in rows[0]:
            continue
        vals = []
        for row in rows:
            try:
                v = float(row[col])
                if not math.isnan(v):
                    vals.append(v)
            except (ValueError, KeyError):
                pass
        if vals:
            result[key] = vals
    return result

def rmse(errs):
    return math.sqrt(sum(e**2 for e in errs) / len(errs))

def stats(rmse_list):
    """Input: list of per-run RMSE values. Output: mean, std, min, max."""
    n = len(rmse_list)
    if n == 0:
        return None
    m = sum(rmse_list) / n
    s = math.sqrt(sum((x - m)**2 for x in rmse_list) / n) if n > 1 else 0.0
    return {'mean': m, 'std': s, 'min': min(rmse_list), 'max': max(rmse_list), 'n': n}

def analyze_group(files, label):
    """Compute per-run RMSE and distribution stats for a group of files."""
    per_run = {}  # {source: [rmse_run1, rmse_run2, ...]}
    print(f"\n{'='*60}")
    print(f"  {label}  ({len(files)} run(s))")
    print(f"{'='*60}")
    for path in sorted(files):
        name = os.path.basename(path)
        errs = load_rmse(path)
        if not errs:
            print(f"  {name}: tidak bisa dibaca atau kosong")
            continue
        row_parts = [f"  {name:<40}"]
        for src in ['EKF', 'AMCL', 'Cox']:
            if src not in errs:
                row_parts.append(f"  {src}=---")
                continue
            r = rmse(errs[src])
            row_parts.append(f"  {src}={r:.4f}m")
            per_run.setdefault(src, []).append(r)
        print(''.join(row_parts))

    if per_run:
        print(f"\n  {'DISTRIBUSI':<8}  {'N':>3}  {'mean':>8}  {'std':>8}  {'min':>8}  {'max':>8}  {'spread':>8}")
        print("  " + "─" * 60)
        for src in ['EKF', 'AMCL', 'Cox']:
            if src not in per_run:
                continue
            s = stats(per_run[src])
            spread = s['max'] - s['min']
            print(f"  {src:<8}  {s['n']:>3}  "
                  f"{s['mean']:>8.4f}  {s['std']:>8.4f}  "
                  f"{s['min']:>8.4f}  {s['max']:>8.4f}  {spread:>8.4f}")
    return per_run

def compare(base_stats, test_stats):
    print(f"\n{'='*60}")
    print("  PERBANDINGAN BASELINE vs TEST")
    print(f"{'='*60}")
    print(f"  {'Src':<6}  {'Base mean':>10}  {'Base spread':>12}  "
          f"{'Test mean':>10}  {'Test spread':>12}  {'Δmean':>8}")
    print("  " + "─" * 66)
    for src in ['EKF', 'AMCL', 'Cox']:
        if src not in base_stats or src not in test_stats:
            continue
        b = stats(base_stats[src])
        t = stats(test_stats[src])
        b_spread = b['max'] - b['min']
        t_spread = t['max'] - t['min']
        dmean = t['mean'] - b['mean']
        sign = '+' if dmean >= 0 else ''
        print(f"  {src:<6}  {b['mean']:>10.4f}  {b_spread:>12.4f}  "
              f"{t['mean']:>10.4f}  {t_spread:>12.4f}  {sign}{dmean:>7.4f}")
    print()
    print("  Interpretasi:")
    print("    Spawn fix BERHASIL: Test spread << Base spread (konvergensi deterministik)")
    print("    Spawn fix PARSIAL : Test mean turun tapi spread masih lebar (ada non-det lain)")
    print("    Spawn fix GAGAL   : Test mean >= Base mean")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('files', nargs='*', help='CSV files (B-1 runs jika tanpa --baseline/--test)')
    parser.add_argument('--baseline', nargs='+', metavar='CSV', help='Baseline run CSVs')
    parser.add_argument('--test',     nargs='+', metavar='CSV', help='Test (B-1) run CSVs')
    args = parser.parse_args()

    if args.baseline and args.test:
        base = analyze_group(args.baseline, 'BASELINE (sebelum spawn fix)')
        test = analyze_group(args.test,     'TEST B-1  (spawn fix coupled)')
        compare(base, test)
    elif args.baseline:
        analyze_group(args.baseline, 'BASELINE')
    elif args.test:
        analyze_group(args.test, 'TEST B-1 (spawn fix coupled)')
    elif args.files:
        analyze_group(args.files, 'RUN ANALYSIS')
    else:
        parser.print_help()

if __name__ == '__main__':
    main()
