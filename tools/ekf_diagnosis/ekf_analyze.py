#!/usr/bin/env python3
"""
tools/ekf_diagnosis/ekf_analyze.py
------------------------------------
Fase 1A — Diagnosa supresi EKF.

Input : CSV dari localization_evaluator.py (v2, dengan kolom odom_x/odom_y)
Output: fase1a_plots/ + FASE1A_HASIL.md

Usage:
  python3 tools/ekf_diagnosis/ekf_analyze.py pose_eval_<timestamp>.csv

Kolom CSV yang diharapkan:
  t_s, gt_x, gt_y, gt_yaw_deg,
  odom_x, odom_y, odom_yaw_deg,
  ekf_x, ekf_y, ekf_yaw_deg,
  amcl_x, amcl_y, amcl_yaw_deg,
  err_ekf_pos_m, err_ekf_yaw_deg,
  err_amcl_pos_m, err_amcl_yaw_deg
"""
import sys
import os
import math
import csv

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUTDIR = 'fase1a_plots'
REPORT = 'FASE1A_HASIL.md'


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────

def load_csv(path: str) -> dict:
    rows = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    if not rows:
        raise ValueError(f"CSV kosong: {path}")

    def col(key):
        vals = []
        for r in rows:
            v = r.get(key, 'nan')
            try:
                vals.append(float(v))
            except ValueError:
                vals.append(float('nan'))
        return np.array(vals)

    return {
        't':        col('t_s'),
        'gt_x':     col('gt_x'),
        'gt_y':     col('gt_y'),
        'odom_x':   col('odom_x'),
        'odom_y':   col('odom_y'),
        'ekf_x':    col('ekf_x'),
        'ekf_y':    col('ekf_y'),
        'amcl_x':   col('amcl_x'),
        'amcl_y':   col('amcl_y'),
        'err_ekf':  col('err_ekf_pos_m'),
        'err_amcl': col('err_amcl_pos_m'),
    }


def displacement_from_start(xs: np.ndarray) -> np.ndarray:
    """Return x - x[0], masking NaN."""
    mask = ~np.isnan(xs)
    if not np.any(mask):
        return np.full_like(xs, np.nan)
    x0 = xs[mask][0]
    out = xs - x0
    out[~mask] = np.nan
    return out


def corr(a: np.ndarray, b: np.ndarray) -> float:
    """Pearson r setelah drop NaN."""
    mask = ~(np.isnan(a) | np.isnan(b))
    if mask.sum() < 5:
        return float('nan')
    return float(np.corrcoef(a[mask], b[mask])[0, 1])


def rmse(errs: np.ndarray) -> float:
    valid = errs[~np.isnan(errs)]
    if len(valid) == 0:
        return float('nan')
    return float(np.sqrt(np.mean(valid**2)))


def x_capture_pct(src_disp: np.ndarray, gt_disp: np.ndarray) -> float:
    """
    x-capture % = (total src displacement / total GT displacement) * 100
    hanya saat GT bergerak positif.
    """
    mask = (~np.isnan(src_disp)) & (~np.isnan(gt_disp)) & (gt_disp > 0.05)
    if mask.sum() < 5:
        return float('nan')
    # Gunakan last valid point
    src_end = src_disp[mask][-1]
    gt_end  = gt_disp[mask][-1]
    if abs(gt_end) < 0.01:
        return float('nan')
    return 100.0 * src_end / gt_end


def amcl_range(amcl_x: np.ndarray) -> float:
    """Peak-to-peak AMCL x selama run."""
    valid = amcl_x[~np.isnan(amcl_x)]
    if len(valid) < 2:
        return float('nan')
    return float(valid.max() - valid.min())


# ──────────────────────────────────────────────────────────────────────────────
# Plots
# ──────────────────────────────────────────────────────────────────────────────

def plot_x_vs_time(d: dict, out: str):
    """Plot 1 — x(t) untuk keempat sumber."""
    t = d['t']
    gt_d    = displacement_from_start(d['gt_x'])
    odom_d  = displacement_from_start(d['odom_x'])
    ekf_d   = displacement_from_start(d['ekf_x'])
    amcl_d  = displacement_from_start(d['amcl_x'])

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    ax1 = axes[0]
    ax1.plot(t, gt_d,   'k-',  lw=2,   label='GT (ground truth)')
    ax1.plot(t, odom_d, 'g--', lw=1.5, label='/odom (raw FK)')
    ax1.plot(t, ekf_d,  'b-',  lw=1.5, label='EKF /odometry/filtered')
    ax1.plot(t, amcl_d, 'r:',  lw=1.5, label='AMCL /amcl_pose')
    ax1.set_ylabel('Δx displacement [m]')
    ax1.set_title('Fase 1A — x displacement vs waktu\n'
                  'Hipotesis: EKF mengikuti AMCL (beku), bukan odom (bergerak)')
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(0, color='gray', lw=0.5)

    ax2 = axes[1]
    # Gap: berapa meter EKF tertinggal dari odom dan GT
    ekf_lag_vs_gt   = gt_d   - ekf_d
    ekf_lag_vs_odom = odom_d - ekf_d
    ax2.plot(t, ekf_lag_vs_gt,   'b-',  lw=1.5, label='GT − EKF (EKF lag vs GT)')
    ax2.plot(t, ekf_lag_vs_odom, 'g--', lw=1.5, label='odom − EKF (EKF lag vs odom)')
    ax2.set_xlabel('t [s]')
    ax2.set_ylabel('lag [m]')
    ax2.set_title('EKF lag: positif = EKF tertinggal di belakang')
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(0, color='gray', lw=0.5)

    plt.tight_layout()
    plt.savefig(out, dpi=120)
    plt.close()
    print(f"  Plot disimpan: {out}")


def plot_ekf_tracking(d: dict, out: str):
    """Plot 2 — Korelasi EKF vs AMCL dan EKF vs odom (displacement)."""
    gt_d    = displacement_from_start(d['gt_x'])
    odom_d  = displacement_from_start(d['odom_x'])
    ekf_d   = displacement_from_start(d['ekf_x'])
    amcl_d  = displacement_from_start(d['amcl_x'])

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # EKF vs AMCL
    ax = axes[0]
    mask = ~(np.isnan(ekf_d) | np.isnan(amcl_d))
    ax.scatter(amcl_d[mask], ekf_d[mask], s=2, alpha=0.3, color='red')
    lim = max(np.nanmax(np.abs(ekf_d)), np.nanmax(np.abs(amcl_d))) + 0.1
    ax.plot([-lim, lim], [-lim, lim], 'k--', lw=1, label='y=x (perfect track)')
    r = corr(amcl_d, ekf_d)
    ax.set_xlabel('AMCL Δx [m]')
    ax.set_ylabel('EKF Δx [m]')
    ax.set_title(f'EKF tracking AMCL\nr={r:.4f}')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # EKF vs odom
    ax = axes[1]
    mask = ~(np.isnan(ekf_d) | np.isnan(odom_d))
    ax.scatter(odom_d[mask], ekf_d[mask], s=2, alpha=0.3, color='green')
    lim = max(np.nanmax(np.abs(ekf_d)), np.nanmax(np.abs(odom_d))) + 0.1
    ax.plot([-lim, lim], [-lim, lim], 'k--', lw=1, label='y=x (perfect track)')
    r = corr(odom_d, ekf_d)
    ax.set_xlabel('Odom Δx [m]')
    ax.set_ylabel('EKF Δx [m]')
    ax.set_title(f'EKF tracking odom\nr={r:.4f}')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.suptitle('Fase 1A — EKF mengikuti AMCL atau odom?\n'
                 'r tinggi = lebih mirip sumber tersebut', fontsize=11)
    plt.tight_layout()
    plt.savefig(out, dpi=120)
    plt.close()
    print(f"  Plot disimpan: {out}")


def plot_amcl_y_range(d: dict, out: str):
    """Plot 3 — AMCL y vs waktu (deteksi constraint y)."""
    t = d['t']
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, d['gt_y'],   'k-',  lw=1.5, label='GT y')
    ax.plot(t, d['amcl_y'], 'r--', lw=1.5, label='AMCL y')
    ax.plot(t, d['ekf_y'],  'b:',  lw=1.5, label='EKF y')
    ax.set_xlabel('t [s]')
    ax.set_ylabel('y [m]')
    ax.set_title('AMCL y range (harus bervariasi jika ada observasi lateral)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out, dpi=120)
    plt.close()
    print(f"  Plot disimpan: {out}")


def plot_rmse_time(d: dict, out: str):
    """Plot 4 — RMSE kumulatif EKF dan AMCL vs waktu."""
    t       = d['t']
    err_ekf = d['err_ekf']
    err_amcl = d['err_amcl']

    # Rolling RMSE (window 20 samples = 4s at 5Hz)
    def rolling_rmse(arr, w=20):
        out = np.full_like(arr, np.nan)
        for i in range(w, len(arr)):
            chunk = arr[i-w:i]
            valid = chunk[~np.isnan(chunk)]
            if len(valid) > 0:
                out[i] = np.sqrt(np.mean(valid**2))
        return out

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, rolling_rmse(err_ekf),  'b-', lw=1.5, label='EKF RMSE (rolling 4s)')
    ax.plot(t, rolling_rmse(err_amcl), 'r-', lw=1.5, label='AMCL RMSE (rolling 4s)')
    ax.axhline(0.15, color='orange', lw=1, ls='--', label='threshold 0.15m')
    ax.set_xlabel('t [s]')
    ax.set_ylabel('RMSE [m]')
    ax.set_title('Rolling RMSE vs waktu')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out, dpi=120)
    plt.close()
    print(f"  Plot disimpan: {out}")


# ──────────────────────────────────────────────────────────────────────────────
# Diagnosis
# ──────────────────────────────────────────────────────────────────────────────

def diagnose(d: dict) -> dict:
    gt_d    = displacement_from_start(d['gt_x'])
    odom_d  = displacement_from_start(d['odom_x'])
    ekf_d   = displacement_from_start(d['ekf_x'])
    amcl_d  = displacement_from_start(d['amcl_x'])

    r_ekf_amcl = corr(ekf_d, amcl_d)
    r_ekf_odom = corr(ekf_d, odom_d)
    r_odom_gt  = corr(odom_d, gt_d)

    xcap_ekf  = x_capture_pct(ekf_d,  gt_d)
    xcap_odom = x_capture_pct(odom_d, gt_d)

    rmse_ekf  = rmse(d['err_ekf'])
    rmse_amcl = rmse(d['err_amcl'])
    amcl_x_range = amcl_range(d['amcl_x'])

    # Suppression score: berapa besar lag EKF vs odom pada akhir run
    mask = ~(np.isnan(ekf_d) | np.isnan(odom_d) | np.isnan(gt_d))
    if mask.sum() > 5:
        ekf_final  = ekf_d[mask][-1]
        odom_final = odom_d[mask][-1]
        gt_final   = gt_d[mask][-1]
        ekf_lag_m  = gt_final - ekf_final
        odom_lag_m = gt_final - odom_final
    else:
        ekf_lag_m  = float('nan')
        odom_lag_m = float('nan')

    # Verdict: EKF suppressed jika r_ekf_amcl >> r_ekf_odom
    if math.isnan(r_ekf_amcl) or math.isnan(r_ekf_odom):
        verdict = "DATA TIDAK CUKUP"
        verdict_detail = "Tidak cukup data untuk menentukan."
    elif r_ekf_amcl > 0.85 and r_ekf_odom < 0.6:
        verdict = "EKF SUPPRESSED — mengikuti AMCL"
        verdict_detail = (
            f"EKF sangat berkorelasi dengan AMCL (r={r_ekf_amcl:.3f}) "
            f"dan lemah dengan odom (r={r_ekf_odom:.3f}). "
            f"Hipotesis TERKONFIRMASI: EKF diabaikan odom yang bagus demi AMCL yang beku."
        )
    elif r_ekf_odom > 0.85 and r_ekf_amcl < 0.7:
        verdict = "EKF MENGIKUTI ODOM"
        verdict_detail = (
            f"EKF berkorelasi kuat dengan odom (r={r_ekf_odom:.3f}). "
            f"EKF tidak disuppress — odom sudah dipercaya."
        )
    elif r_ekf_amcl > 0.7 and r_ekf_odom > 0.7:
        verdict = "EKF MIXED — mengikuti keduanya"
        verdict_detail = (
            f"EKF berkorelasi dengan AMCL (r={r_ekf_amcl:.3f}) "
            f"dan odom (r={r_ekf_odom:.3f}). "
            f"Fusion beroperasi normal — keduanya aktif."
        )
    else:
        verdict = "TIDAK JELAS"
        verdict_detail = f"r_amcl={r_ekf_amcl:.3f}, r_odom={r_ekf_odom:.3f}. Butuh lebih banyak data."

    return {
        'r_ekf_amcl':   r_ekf_amcl,
        'r_ekf_odom':   r_ekf_odom,
        'r_odom_gt':    r_odom_gt,
        'xcap_ekf':     xcap_ekf,
        'xcap_odom':    xcap_odom,
        'rmse_ekf':     rmse_ekf,
        'rmse_amcl':    rmse_amcl,
        'amcl_x_range': amcl_x_range,
        'ekf_lag_m':    ekf_lag_m,
        'odom_lag_m':   odom_lag_m,
        'verdict':      verdict,
        'verdict_detail': verdict_detail,
        'n_rows':       int(len(d['t'])),
        'duration_s':   float(d['t'][-1] - d['t'][0]) if len(d['t']) > 1 else 0.0,
    }


# ──────────────────────────────────────────────────────────────────────────────
# Report
# ──────────────────────────────────────────────────────────────────────────────

def write_report(res: dict, csv_path: str):
    lines = [
        "# FASE1A_HASIL.md — Diagnosa Supresi EKF",
        "",
        f"**Tanggal**: {__import__('datetime').date.today()}",
        f"**Input CSV**: `{csv_path}`",
        "",
        "---",
        "",
        "## 1. Ringkasan Data",
        "",
        f"| Metrik | Nilai |",
        f"|--------|-------|",
        f"| Jumlah baris | {res['n_rows']} |",
        f"| Durasi | {res['duration_s']:.1f}s |",
        "",
        "---",
        "",
        "## 2. Korelasi Tracking EKF",
        "",
        "| Pasangan | Pearson r | Interpretasi |",
        "|----------|-----------|--------------|",
        f"| EKF vs AMCL | {res['r_ekf_amcl']:.4f} | {'tinggi → EKF ikut AMCL' if res['r_ekf_amcl'] > 0.8 else 'rendah'} |",
        f"| EKF vs odom | {res['r_ekf_odom']:.4f} | {'tinggi → EKF ikut odom' if res['r_ekf_odom'] > 0.8 else 'rendah → odom disuppress'} |",
        f"| odom vs GT  | {res['r_odom_gt']:.4f}  | kualitas odom mentah |",
        "",
        "---",
        "",
        "## 3. X-Capture % dan RMSE",
        "",
        "| Source | X-capture % | RMSE [m] | Keterangan |",
        "|--------|-------------|----------|-----------|",
        f"| EKF  | {res['xcap_ekf']:.1f}% | {res['rmse_ekf']:.4f}m | output fusion |",
        f"| AMCL | — | {res['rmse_amcl']:.4f}m | particle filter saja |",
        f"| odom | {res['xcap_odom']:.1f}% | — | raw FK, tanpa fusion |",
        "",
        f"**EKF lag vs GT** di akhir run: {res['ekf_lag_m']:.3f}m",
        f"**odom lag vs GT** di akhir run: {res['odom_lag_m']:.3f}m",
        "",
        "---",
        "",
        "## 4. AMCL y Range",
        "",
        f"AMCL x range (peak-to-peak): **{res['amcl_x_range']:.3f}m**",
        "",
        "Interpretasi: nilai < 0.1m berarti AMCL beku (tidak mendapat koreksi dari scan).",
        "",
        "---",
        "",
        "## 5. VERDICT",
        "",
        f"```",
        f"{res['verdict']}",
        f"```",
        "",
        f"**Detail**: {res['verdict_detail']}",
        "",
        "---",
        "",
        "## 6. Plot",
        "",
        "![x vs waktu](fase1a_plots/plot1_x_time.png)",
        "![EKF tracking](fase1a_plots/plot2_ekf_tracking.png)",
        "![AMCL y range](fase1a_plots/plot3_amcl_y.png)",
        "![RMSE waktu](fase1a_plots/plot4_rmse.png)",
        "",
        "---",
        "",
        "## 7. Keputusan Selanjutnya (Gate G1)",
        "",
        "- Jika **EKF SUPPRESSED**: lanjut ke Fase 1B — turunkan kepercayaan ke AMCL",
        "  atau naikkan kepercayaan ke odom.",
        "- Jika **EKF MIXED/ODOM**: sistem sudah berjalan benar, perbaiki sumber lain.",
        "",
        "> Dihasilkan otomatis oleh `tools/ekf_diagnosis/ekf_analyze.py`",
    ]
    with open(REPORT, 'w') as f:
        f.write('\n'.join(lines))
    print(f"  Laporan: {REPORT}")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 ekf_analyze.py <pose_eval_*.csv>")
        sys.exit(1)

    csv_path = sys.argv[1]
    if not os.path.exists(csv_path):
        print(f"File tidak ditemukan: {csv_path}")
        sys.exit(1)

    print(f"[Fase 1A] Analisis: {csv_path}")
    d = load_csv(csv_path)

    # Validasi kolom odom
    if np.all(np.isnan(d['odom_x'])):
        print("[WARN] Kolom odom_x semua NaN — pastikan localization_evaluator.py v2 dipakai")
        print("       (versi lama tidak punya kolom odom_x/odom_y)")

    os.makedirs(OUTDIR, exist_ok=True)

    print("Membuat plot...")
    plot_x_vs_time(d,      f'{OUTDIR}/plot1_x_time.png')
    plot_ekf_tracking(d,   f'{OUTDIR}/plot2_ekf_tracking.png')
    plot_amcl_y_range(d,   f'{OUTDIR}/plot3_amcl_y.png')
    plot_rmse_time(d,      f'{OUTDIR}/plot4_rmse.png')

    res = diagnose(d)
    write_report(res, csv_path)

    print()
    print("=" * 60)
    print(f"  r(EKF,AMCL) = {res['r_ekf_amcl']:.4f}")
    print(f"  r(EKF,odom) = {res['r_ekf_odom']:.4f}")
    print(f"  x-capture EKF  = {res['xcap_ekf']:.1f}%")
    print(f"  x-capture odom = {res['xcap_odom']:.1f}%")
    print(f"  RMSE EKF       = {res['rmse_ekf']:.4f}m")
    print(f"  AMCL x range   = {res['amcl_x_range']:.3f}m")
    print()
    print(f"VERDICT: {res['verdict']}")
    print(f"  {res['verdict_detail']}")
    print("=" * 60)


if __name__ == '__main__':
    main()
