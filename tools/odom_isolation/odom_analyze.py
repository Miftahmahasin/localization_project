#!/usr/bin/env python3
"""
tools/odom_isolation/odom_analyze.py
--------------------------------------
Fase 0 — Analisis CSV dari odom_logger.py.

Jalankan (tidak butuh ROS):
  python3 tools/odom_isolation/odom_analyze.py exp1.csv exp2_slow.csv exp2_fast.csv exp3_yaw.csv

Output:
  fase0_plots/plot1_timeseries.png
  fase0_plots/plot2_scatter.png
  fase0_plots/plot3_k_vs_speed.png
  fase0_plots/plot4_yaw.png
  FASE0_HASIL.md
"""
import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# ── helpers ──────────────────────────────────────────────────────────────────

def load_csv(path: str) -> list[dict]:
    with open(path) as f:
        return list(csv.DictReader(f))


def wrap_angle(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


def cumdisp(xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """Cumulative displacement from successive position differences."""
    dx = np.diff(xs)
    dy = np.diff(ys)
    step = np.sqrt(dx**2 + dy**2)
    return np.concatenate([[0.0], np.cumsum(step)])


def linreg_no_intercept(X: np.ndarray, Y: np.ndarray):
    """y = k*x, no intercept. Returns (k, R2)."""
    mask = np.isfinite(X) & np.isfinite(Y)
    X, Y = X[mask], Y[mask]
    if len(X) < 3:
        return float('nan'), float('nan')
    k = float(np.dot(X, Y) / np.dot(X, X))
    residuals = Y - k * X
    ss_res = float(np.dot(residuals, residuals))
    ss_tot = float(np.var(Y) * len(Y))
    R2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0
    return k, R2


def yaw_drift_rate(ts: np.ndarray, gt_yaws: np.ndarray, odom_yaws: np.ndarray):
    """Returns drift rate in deg/s (linear fit of yaw_error vs time)."""
    errors = np.array([math.degrees(wrap_angle(o - g))
                       for o, g in zip(odom_yaws, gt_yaws)])
    if len(ts) < 3:
        return float('nan')
    # linear fit error vs time
    p = np.polyfit(ts, errors, 1)
    return float(p[0])  # slope = deg/s


# ── per-experiment analysis ───────────────────────────────────────────────────

def analyse_file(path: str) -> dict:
    rows = load_csv(path)
    if not rows:
        return None

    ts        = np.array([float(r['t_s'])        for r in rows])
    odom_x    = np.array([float(r['odom_x'])     for r in rows])
    odom_y    = np.array([float(r['odom_y'])     for r in rows])
    odom_yaw  = np.array([float(r['odom_yaw_rad']) for r in rows])
    gt_x      = np.array([float(r['gt_x'])       for r in rows])
    gt_y      = np.array([float(r['gt_y'])       for r in rows])
    gt_yaw    = np.array([float(r['gt_yaw_rad']) for r in rows])
    label     = rows[0]['exp_label']
    speed_cmd = float(rows[0]['speed_cmd'])

    # displacement (cumulative, body-frame not needed — just world Δ)
    odom_disp = cumdisp(odom_x, odom_y)
    gt_disp   = cumdisp(gt_x,   gt_y)

    k_disp, R2_disp = linreg_no_intercept(odom_disp, gt_disp)

    # per-axis (signed: project odom delta onto GT direction)
    # Simple: use raw diff sums for x and y separately
    odom_dx_cumsum = np.concatenate([[0.0], np.cumsum(np.diff(odom_x))])
    gt_dx_cumsum   = np.concatenate([[0.0], np.cumsum(np.diff(gt_x))])
    odom_dy_cumsum = np.concatenate([[0.0], np.cumsum(np.diff(odom_y))])
    gt_dy_cumsum   = np.concatenate([[0.0], np.cumsum(np.diff(gt_y))])

    k_x, R2_x = linreg_no_intercept(odom_dx_cumsum, gt_dx_cumsum)
    k_y, R2_y = linreg_no_intercept(odom_dy_cumsum, gt_dy_cumsum)

    drift = yaw_drift_rate(ts, gt_yaw, odom_yaw)

    return {
        'path':      path,
        'label':     label,
        'speed_cmd': speed_cmd,
        'n_rows':    len(rows),
        'duration':  float(ts[-1] - ts[0]),
        'ts':        ts,
        'odom_x': odom_x, 'odom_y': odom_y, 'odom_yaw': odom_yaw,
        'gt_x':   gt_x,   'gt_y':   gt_y,   'gt_yaw':   gt_yaw,
        'odom_disp': odom_disp, 'gt_disp': gt_disp,
        'odom_dx': odom_dx_cumsum, 'gt_dx': gt_dx_cumsum,
        'odom_dy': odom_dy_cumsum, 'gt_dy': gt_dy_cumsum,
        'k_disp': k_disp, 'R2_disp': R2_disp,
        'k_x': k_x, 'R2_x': R2_x,
        'k_y': k_y, 'R2_y': R2_y,
        'yaw_drift_deg_per_s': drift,
    }


# ── plots ─────────────────────────────────────────────────────────────────────

def plot_timeseries(results: list, outdir: str):
    n = len(results)
    fig, axes = plt.subplots(n, 2, figsize=(14, 4 * n), squeeze=False)
    fig.suptitle('Plot 1 — Time Series: odom vs GT', fontsize=13)

    for i, r in enumerate(results):
        ts = r['ts'] - r['ts'][0]

        ax = axes[i][0]
        ax.plot(ts, r['gt_x'],   'b-', lw=1.5, label='gt_x')
        ax.plot(ts, r['odom_x'], 'r--', lw=1.5, label='odom_x')
        ax.set_title(f"{r['label']}  (speed={r['speed_cmd']:.2f})")
        ax.set_xlabel('t (s)'); ax.set_ylabel('x (m)')
        ax.legend(); ax.grid(True, alpha=0.4)

        ax2 = axes[i][1]
        ax2.plot(ts, r['gt_y'],   'b-', lw=1.5, label='gt_y')
        ax2.plot(ts, r['odom_y'], 'r--', lw=1.5, label='odom_y')
        ax2.set_title(f"{r['label']}  y-axis")
        ax2.set_xlabel('t (s)'); ax2.set_ylabel('y (m)')
        ax2.legend(); ax2.grid(True, alpha=0.4)

    plt.tight_layout()
    path = os.path.join(outdir, 'plot1_timeseries.png')
    plt.savefig(path, dpi=120)
    plt.close()
    return path


def plot_scatter(results: list, outdir: str):
    n = len(results)
    fig, axes = plt.subplots(1, n, figsize=(6 * n, 5), squeeze=False)
    fig.suptitle('Plot 2 — Displacement Scatter: odom vs GT (k = GT/odom)', fontsize=13)

    for i, r in enumerate(results):
        ax = axes[0][i]
        odisp = r['odom_disp']
        gdisp = r['gt_disp']
        ax.scatter(odisp, gdisp, s=4, alpha=0.4, label='data')

        if not math.isnan(r['k_disp']):
            xlim = max(odisp.max(), 0.01)
            xs = np.array([0, xlim])
            ax.plot(xs, r['k_disp'] * xs, 'r-', lw=2,
                    label=f"k={r['k_disp']:.3f}, R²={r['R2_disp']:.3f}")

        ax.plot([0, max(odisp.max(), gdisp.max())],
                [0, max(odisp.max(), gdisp.max())],
                'k:', lw=1, label='k=1 (ideal)')
        ax.set_title(r['label'])
        ax.set_xlabel('odom displacement (m)')
        ax.set_ylabel('GT displacement (m)')
        ax.legend(fontsize=8); ax.grid(True, alpha=0.4)

    plt.tight_layout()
    path = os.path.join(outdir, 'plot2_scatter.png')
    plt.savefig(path, dpi=120)
    plt.close()
    return path


def plot_k_vs_speed(results: list, outdir: str):
    speeds = [r['speed_cmd'] for r in results]
    ks_x   = [r['k_x']   for r in results]
    ks_y   = [r['k_y']   for r in results]
    ks_d   = [r['k_disp'] for r in results]

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(speeds, ks_d, 'ko-', lw=2, ms=8, label='k_disp (forward)')
    ax.plot(speeds, ks_x, 'b^--', lw=1.5, ms=7, label='k_x')
    ax.plot(speeds, ks_y, 'rs--', lw=1.5, ms=7, label='k_y')
    ax.axhline(1.0, color='gray', ls=':', lw=1, label='k=1 (ideal)')
    ax.set_xlabel('speed_cmd')
    ax.set_ylabel('k = GT / odom')
    ax.set_title('Plot 3 — k vs Kecepatan')
    ax.legend(); ax.grid(True, alpha=0.4)

    plt.tight_layout()
    path = os.path.join(outdir, 'plot3_k_vs_speed.png')
    plt.savefig(path, dpi=120)
    plt.close()
    return path


def plot_yaw(results: list, outdir: str):
    # Gunakan eksperimen berlabel 'yaw' kalau ada, otherwise semua
    yaw_results = [r for r in results if 'yaw' in r['label'].lower()] or results

    n = len(yaw_results)
    fig, axes = plt.subplots(n, 1, figsize=(10, 4 * n), squeeze=False)
    fig.suptitle('Plot 4 — Yaw: odom vs GT', fontsize=13)

    for i, r in enumerate(yaw_results):
        ts = r['ts'] - r['ts'][0]
        odom_yaw_deg = np.degrees(r['odom_yaw'])
        gt_yaw_deg   = np.degrees(r['gt_yaw'])
        ax = axes[i][0]
        ax.plot(ts, gt_yaw_deg,   'b-', lw=1.5, label='gt_yaw')
        ax.plot(ts, odom_yaw_deg, 'r--', lw=1.5, label='odom_yaw')
        drift = r['yaw_drift_deg_per_s']
        ax.set_title(f"{r['label']}  yaw_drift={drift:+.3f} deg/s")
        ax.set_xlabel('t (s)'); ax.set_ylabel('yaw (deg)')
        ax.legend(); ax.grid(True, alpha=0.4)

    plt.tight_layout()
    path = os.path.join(outdir, 'plot4_yaw.png')
    plt.savefig(path, dpi=120)
    plt.close()
    return path


# ── verdict ───────────────────────────────────────────────────────────────────

def compute_verdict(results: list) -> tuple[str, str]:
    ks   = [r['k_disp'] for r in results if not math.isnan(r['k_disp'])]
    R2s  = [r['R2_disp'] for r in results if not math.isnan(r['R2_disp'])]

    if not ks:
        return 'UNKNOWN', 'Tidak cukup data untuk verdict.'

    k_mean  = float(np.mean(ks))
    k_std   = float(np.std(ks))
    R2_mean = float(np.mean(R2s))
    R2_min  = float(np.min(R2s))

    k_variation = k_std / abs(k_mean) if k_mean != 0 else float('inf')

    if R2_mean > 0.95 and k_variation < 0.10:
        verdict = f'SYSTEMATIC (scale, k={k_mean:.3f})'
        reason  = (f'R²={R2_mean:.3f} tinggi dan k hampir konstan '
                   f'(std/mean={k_variation:.2%}). Error sangat bisa diprediksi '
                   f'→ correctable dengan faktor {1/k_mean:.3f}.')
    elif R2_min < 0.80:
        verdict = 'RANDOM (slip)'
        reason  = (f'R²_min={R2_min:.3f} rendah. Hubungan odom–GT tidak linear '
                   f'→ odometri tidak andal untuk prediction.')
    elif k_variation >= 0.15:
        verdict = f'MIXED (scale + velocity-dependent, k_mean={k_mean:.3f})'
        reason  = (f'R² cukup ({R2_mean:.3f}) tapi k bervariasi lebar '
                   f'(std/mean={k_variation:.2%}) antar kecepatan '
                   f'→ scale error bergantung kecepatan (slip mulai muncul).')
    else:
        verdict = f'SYSTEMATIC (scale, k={k_mean:.3f}) [borderline]'
        reason  = (f'R²={R2_mean:.3f}, k_variation={k_variation:.2%}. '
                   f'Kemungkinan systematic tapi verifikasi dengan eksperimen lebih lama.')

    return verdict, reason


# ── report ────────────────────────────────────────────────────────────────────

def generate_report(results: list, plots: dict, out_path: str):
    verdict, reason = compute_verdict(results)

    lines = [
        '# FASE0_HASIL.md — Hasil Isolasi Odometri',
        '',
        f'**Tanggal**: {__import__("datetime").date.today()}',
        '',
        '---',
        '',
        '## 1. Topik yang Dipakai',
        '',
        '| Peran | Topik | Alasan |',
        '|-------|-------|--------|',
        '| Raw odom | `/odom` | Output `legged_odometry_kf_node` sebelum EKF — belum terkontaminasi AMCL |',
        '| Ground truth | `/ground_truth/odom` | Posisi absolut dari Webots `getPosition()` via `op3_extern_controller.cpp` |',
        '',
        '---',
        '',
        '## 2. Hasil per Eksperimen',
        '',
        '| Label | Speed | Durasi | n_rows | k_disp | R²_disp | k_x | R²_x | k_y | R²_y | Yaw drift (deg/s) |',
        '|-------|-------|--------|--------|--------|---------|-----|------|-----|------|-------------------|',
    ]

    for r in results:
        def f(v): return f'{v:.4f}' if not math.isnan(v) else 'N/A'
        lines.append(
            f"| {r['label']} | {r['speed_cmd']:.2f} | {r['duration']:.1f}s | {r['n_rows']} "
            f"| {f(r['k_disp'])} | {f(r['R2_disp'])} "
            f"| {f(r['k_x'])} | {f(r['R2_x'])} "
            f"| {f(r['k_y'])} | {f(r['R2_y'])} "
            f"| {f(r['yaw_drift_deg_per_s'])} |"
        )

    lines += [
        '',
        '---',
        '',
        '## 3. Plot',
        '',
        f'![Time series](fase0_plots/plot1_timeseries.png)',
        f'![Scatter](fase0_plots/plot2_scatter.png)',
        f'![k vs speed](fase0_plots/plot3_k_vs_speed.png)',
        f'![Yaw](fase0_plots/plot4_yaw.png)',
        '',
        '---',
        '',
        '## 4. Interpretasi',
        '',
        '### k per sumbu',
        '',
        '| Sumbu | k (GT/odom) | Interpretasi |',
        '|-------|-------------|--------------|',
    ]

    for r in results:
        def f(v): return f'{v:.3f}' if not math.isnan(v) else 'N/A'
        lines.append(
            f"| {r['label']} x | {f(r['k_x'])} | "
            f"{'undershoot' if not math.isnan(r['k_x']) and r['k_x'] > 1 else 'overshoot/ok'} |"
        )

    lines += [
        '',
        '### Yaw Drift',
        '',
    ]
    for r in results:
        drift = r['yaw_drift_deg_per_s']
        lines.append(
            f"- **{r['label']}**: {drift:+.3f} deg/s "
            f"({'significant' if abs(drift) > 0.5 else 'acceptable'})"
            if not math.isnan(drift) else f"- **{r['label']}**: N/A"
        )

    lines += [
        '',
        '---',
        '',
        '## 5. VERDICT',
        '',
        f'```',
        f'{verdict}',
        f'```',
        '',
        f'**Penjelasan**: {reason}',
        '',
        '---',
        '',
        '## 6. Keputusan Selanjutnya (Gate G0)',
        '',
        '- Jika **SYSTEMATIC**: lanjut ke **Branch A** (kalibrasi scale factor `1/k`)',
        '- Jika **RANDOM**: lanjut ke **Branch B** (visual odometry)',
        '- Jika **MIXED**: diskusi dengan user sebelum melanjutkan',
        '',
        '> Dokumen ini dihasilkan otomatis oleh `tools/odom_isolation/odom_analyze.py`',
    ]

    with open(out_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    return verdict, reason


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Fase 0: Analisis CSV odom vs GT')
    parser.add_argument('csv_files', nargs='+', help='Satu atau lebih file CSV dari odom_logger.py')
    parser.add_argument('--outdir', default='fase0_plots', help='Direktori output plot (default: fase0_plots)')
    parser.add_argument('--report', default='FASE0_HASIL.md', help='Path laporan markdown (default: FASE0_HASIL.md)')
    args = parser.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    results = []
    for path in args.csv_files:
        if not os.path.exists(path):
            print(f'[WARN] File tidak ditemukan: {path}', file=sys.stderr)
            continue
        r = analyse_file(path)
        if r is None:
            print(f'[WARN] File kosong: {path}', file=sys.stderr)
            continue
        results.append(r)
        print(f'[OK] {path}: label={r["label"]} n={r["n_rows"]} '
              f'k_disp={r["k_disp"]:.3f} R2={r["R2_disp"]:.3f}')

    if not results:
        print('Tidak ada data valid. Keluar.', file=sys.stderr)
        sys.exit(1)

    plots = {}
    plots['timeseries'] = plot_timeseries(results, args.outdir)
    plots['scatter']    = plot_scatter(results, args.outdir)
    plots['k_vs_speed'] = plot_k_vs_speed(results, args.outdir)
    plots['yaw']        = plot_yaw(results, args.outdir)
    print(f'Plot disimpan di {args.outdir}/')

    verdict, reason = generate_report(results, plots, args.report)
    print(f'\nLaporan: {args.report}')
    print(f'\n{"="*60}')
    print(f'VERDICT: {verdict}')
    print(f'Alasan : {reason}')
    print(f'{"="*60}')
    print('\nBerhenti di sini. Laporkan verdict ke user untuk keputusan G0.')


if __name__ == '__main__':
    main()
