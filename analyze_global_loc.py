#!/usr/bin/env python3
"""
analyze_global_loc.py — Analisis struktur ambiguitas AMCL dari global localization test
========================================================================================
Usage:
  python3 analyze_global_loc.py global_loc_results.csv
  python3 analyze_global_loc.py global_loc_results.csv --gt-x -0.363 --gt-y 0.0
"""
import csv, math, sys, argparse
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from collections import defaultdict


STR_FIELDS = {'verdict', 'pose_changed'}

def load(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append({k: v if k in STR_FIELDS else float(v)
                         for k, v in row.items()})
    return rows


def cluster_modes(rows, radius=0.5):
    """Kelompokkan titik konvergensi ke dalam mode (cluster sederhana dengan radius)."""
    modes = []
    for r in rows:
        cx, cy = r['amcl_x'], r['amcl_y']
        found = False
        for mode in modes:
            mx, my = mode['cx'], mode['cy']
            if math.sqrt((cx-mx)**2 + (cy-my)**2) < radius:
                mode['points'].append((cx, cy, r['trial']))
                # Update centroid
                mode['cx'] = sum(p[0] for p in mode['points']) / len(mode['points'])
                mode['cy'] = sum(p[1] for p in mode['points']) / len(mode['points'])
                found = True
                break
        if not found:
            modes.append({'cx': cx, 'cy': cy, 'points': [(cx, cy, r['trial'])]})
    return modes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', help='CSV dari global_loc_test.py')
    parser.add_argument('--gt-x', type=float, default=-0.363)
    parser.add_argument('--gt-y', type=float, default=0.0)
    args = parser.parse_args()

    rows = load(args.csv)
    gt_x, gt_y = args.gt_x, args.gt_y
    n = len(rows)

    print(f"\n{'='*60}")
    print(f"ANALISIS STRUKTUR AMBIGUITAS AMCL")
    print(f"CSV: {args.csv}  |  N={n}  |  GT=({gt_x},{gt_y})")
    print(f"{'='*60}")

    # Per-trial table
    print(f"\n{'Trial':>6}  {'AMCL_x':>8}  {'AMCL_y':>8}  {'err_m':>6}  {'verdict'}")
    print("─"*50)
    for r in rows:
        print(f"{r['trial']:>6.0f}  {r['amcl_x']:>8.3f}  {r['amcl_y']:>8.3f}  "
              f"{r['err_m']:>6.3f}  {r['verdict']}")

    # Mode clustering
    modes = cluster_modes(rows, radius=0.5)
    modes_sorted = sorted(modes, key=lambda m: -len(m['points']))

    print(f"\n{'='*60}")
    print(f"MODE KONVERGENSI (cluster radius=0.5m)")
    print(f"{'='*60}")
    print(f"{'Mode':>5}  {'N':>3}  {'cx':>7}  {'cy':>7}  {'err_GT':>8}  {'trials'}")
    print("─"*60)

    true_mode = None
    for i, mode in enumerate(modes_sorted):
        err_gt = math.sqrt((mode['cx']-gt_x)**2 + (mode['cy']-gt_y)**2)
        trials = [str(int(p[2])) for p in mode['points']]
        label = 'BENAR' if err_gt < 0.3 else f'FALSE_{i}'
        if err_gt < 0.3:
            true_mode = mode
        pct = len(mode['points'])/n*100
        print(f"  {label:<10}  {len(mode['points']):>3} ({pct:.0f}%)  "
              f"{mode['cx']:>7.3f}  {mode['cy']:>7.3f}  {err_gt:>8.3f}m  runs={','.join(trials)}")

    # Verdict
    n_true  = sum(1 for r in rows if r['err_m'] < 0.3)
    n_false = n - n_true
    p_true  = n_true / n * 100
    n_modes = len(modes)

    print(f"\n{'='*60}")
    print(f"VERDICT AMBIGUITAS")
    print(f"{'='*60}")
    print(f"  Konvergen BENAR : {n_true}/{n} ({p_true:.0f}%)")
    print(f"  False minimum  : {n_false}/{n} ({100-p_true:.0f}%)")
    print(f"  Jumlah MODE    : {n_modes}")
    print()
    if n_modes == 1 and n_true == n:
        print("  → TIDAK AMBIGU: satu mode, selalu benar. Masalah lain.")
    elif n_modes <= 2:
        print("  → AMBIGUITAS TUNGGAL: satu false minimum dominan. Satu fitur unik mungkin cukup.")
    elif n_modes <= 4:
        print("  → AMBIGUITAS MODERAT: 3-4 mode. Perlu 2-3 fitur unik.")
    else:
        print("  → AMBIGUITAS PARAH: banyak mode tersebar. Feature layer KRITIS, bukan opsional.")

    # Cek apakah ada FALSE mode dalam 0.22m dari GT (bahaya tighten cov_xx)
    # CATATAN: hanya mode FALSE (err_GT > 0.3m) yang dihitung — true mode TIDAK dihitung
    false_modes_within_0_22 = [m for m in modes
                                if math.sqrt((m['cx']-gt_x)**2+(m['cy']-gt_y)**2) < 0.22
                                and math.sqrt((m['cx']-gt_x)**2+(m['cy']-gt_y)**2) > 0.30]
    print()
    if false_modes_within_0_22:
        print(f"  !! BAHAYA TIGHTEN: {len(false_modes_within_0_22)} false mode dalam radius 0.22m dari GT.")
        print(f"     Tighten cov_xx ke 0.05 (σ=0.22m) TIDAK AMAN — partikel masih menjangkau false mode.")
    else:
        print(f"  Tidak ada false mode dalam 0.22m dari GT.")
        print(f"  (Tighten cov_xx ke σ=0.22m AMAN dari false mode. Masih perlu uji kidnap recovery.)")

    # v2: cek apakah ada trial NOT_CONVERGED atau NO_REINIT
    no_reinit = sum(1 for r in rows if r.get('verdict','') == 'NO_REINIT')
    not_conv  = sum(1 for r in rows if r.get('verdict','') == 'NOT_CONVERGED')
    if no_reinit > 0:
        print(f"\n  !! {no_reinit} trial dengan verdict NO_REINIT — pose tidak berubah setelah reinit.")
        print(f"     Kemungkinan: AMCL tidak re-initialize. Hasil test mungkin TIDAK VALID.")
        print(f"     Solusi: periksa apakah service /reinitialize_global_localization aktif di AMCL lifecycle.")
    if not_conv > 0:
        print(f"\n  !! {not_conv} trial TIMEOUT (belum konvergen). Coba wait_s lebih lama.")

    # Plot
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.set_title(f'Struktur Ambiguitas AMCL — Global Localization dari ({gt_x},{gt_y})\n'
                 f'N={n} trials  |  {n_true} benar ({p_true:.0f}%)  |  {n_modes} mode', fontsize=10)

    # Lapangan
    field = plt.Rectangle((-4.5,-3.0), 9.0, 6.0, fill=False, ec='black', lw=2)
    ax.add_patch(field)
    ax.axhline(0, color='gray', lw=0.5, ls=':')
    ax.axvline(0, color='gray', lw=0.5, ls=':')

    # Goal zones
    for xside in [(-4.5,-3.9), (3.9,4.5)]:
        ax.axvspan(xside[0], xside[1], alpha=0.07, color='yellow')

    # True GT position
    ax.plot(gt_x, gt_y, 'g*', ms=18, zorder=10, label=f'GT spawn ({gt_x},{gt_y})')
    ax.add_patch(plt.Circle((gt_x, gt_y), 0.22, fill=False, ec='green', lw=1.5,
                             ls='--', label='σ_x=0.22m radius (tighten cov_xx)'))
    ax.add_patch(plt.Circle((gt_x, gt_y), 0.707, fill=False, ec='orange', lw=1.0,
                             ls=':', label='σ_x=0.71m radius (current)'))

    # Scatter tiap trial
    colors = plt.cm.tab10(np.linspace(0, 1, max(len(modes), 1)))
    for i, mode in enumerate(modes_sorted):
        err_gt = math.sqrt((mode['cx']-gt_x)**2 + (mode['cy']-gt_y)**2)
        label = 'BENAR' if err_gt < 0.3 else f'FALSE_MIN_{i}'
        col = 'green' if err_gt < 0.3 else colors[i % len(colors)]
        xs = [p[0] for p in mode['points']]
        ys = [p[1] for p in mode['points']]
        ax.scatter(xs, ys, s=80, color=col, zorder=8, label=f'{label} N={len(xs)} ({err_gt:.2f}m)')
        ax.plot(mode['cx'], mode['cy'], 'x', color=col, ms=15, mew=3, zorder=9)

    ax.set_xlim(-5.0, 5.0); ax.set_ylim(-3.5, 3.5)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.2)

    out = args.csv.replace('.csv', '_peta_ambiguitas.png')
    plt.tight_layout()
    plt.savefig(out, dpi=130)
    print(f"\nPlot tersimpan: {out}")
    plt.close()


if __name__ == '__main__':
    main()
